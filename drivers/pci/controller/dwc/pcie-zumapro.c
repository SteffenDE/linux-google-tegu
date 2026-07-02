// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe root-complex driver for the Google Zumapro (Tensor G4) SoC.
 *
 * Drives the GEN3A_1 DesignWare PCIe host used by the on-board WiFi.  The
 * controller-domain (ELBI) reset and link bring-up sequence is ported from the
 * downstream pcie-exynos-rc.c establish_link()/assert_phy_reset() path; the PMA
 * and PCS programming lives in the separate phy-zumapro-pcie.c generic PHY,
 * which this driver brackets with the ELBI PMA-reset assert/release.
 */

#include <linux/arm-smccc.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pcie-zumapro.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#include "pcie-designware.h"

#define to_zumapro_pcie(x)	dev_get_drvdata((x)->dev)

static const struct of_device_id zumapro_pcie_of_match[];

/* ELBI (app/controller) registers, relative to pci->elbi_base. */
#define PCIE_APP_LTSSM_ENABLE		0x0054
#define   LTSSM_ENABLE			0x1
#define PCIE_APP_XFER_PENDING		0x0074
#define   APP_XFER_PENDING		0x1
#define PCIE_DEVICE_TYPE		0x0080
#define   DEVICE_TYPE_RC		0x4
#define PCIE_ELBI_RDLH_LINKUP		0x02c8
#define   LTSSM_STATE_MASK		0x3f
/*
 * The link is established across the whole range from recovery
 * (S_RCVRY_LOCK) through L1 idle (S_L1_IDLE), matching downstream
 * exynos_pcie_rc_link_up().  Requiring exactly L0 makes dw_pcie_link_up()
 * report "down" whenever the link dips into L0s/L1/recovery, which gates off
 * endpoint config access (reads return 0xffffffff) and was a source of
 * cold-boot mis-enumeration.
 */
#define   LTSSM_STATE_RCVRY_LOCK	0x0d
#define   LTSSM_STATE_L0		0x11
#define   LTSSM_STATE_L1_IDLE		0x14
#define PCIE_LINKDOWN_RST_CTRL_SEL	0x03a0
#define   LINKDOWN_RST_MANUAL		BIT(1)
#define PCIE_SOFT_RESET			0x03a4
#define   SOFT_RESET_ALL		0xf
#define   SOFT_RESET_PWR_PULSE		0xd	/* SOFT_RESET_ALL with PWR bit low */
#define PCIE_QCH_SEL			0x03a8
#define   CLOCK_GATING_AXI_MASK		(0xf << 0)
#define   CLOCK_GATING_APB_MASK		(0xf << 4)
#define   CLOCK_GATING_PMU_MASK		(0xf << 8)
#define PCIE_APP_REQ_EXIT_L1_MODE	0x03bc
#define   APP_REQ_EXIT_L1_MODE		BIT(0)
#define   L1_REQ_NAK_CTRL_MASTER	BIT(4)
#define PCIE_SLV_PEND_SEL_NAK		0x03d8
#define PCIE_MSTR_PEND_SEL_NAK		0x0474
#define   PEND_SEL_NAK			0x1
#define PCIE_DBI_L1_EXIT_DISABLE	0x1078
#define   DBI_L1_EXIT_DISABLE		0x1

/* PMA reset trio in ELBI space (no symbolic name downstream). */
#define PCIE_PMA_RST_0			0x1400
#define PCIE_PMA_RST_1			0x1404
#define PCIE_PMA_RST_2			0x1408

/*
 * PMU PCIE_PHY_CONTROL: bit 10 is the PCIe wakeup/IDLE-IP control.  Like the
 * isolation bit the PHY driver owns, this register is firewalled to the secure
 * world on Zumapro, so it is programmed through the EL3 private-register SMC
 * (downstream rmw_priv_reg()); a non-secure write async-aborts (SError).
 */
#define PCIE_PMU_WAKE_CTRL		BIT(10)
#define EXYNOS_SMC_CMD_PRIV_REG		0x82000504
#define EXYNOS_PRIV_REG_OPT_RMW		2

/* PERST settle time after deassert (downstream perst-delay-us default). */
#define PCIE_PERST_DELAY_US		20000

/* Per-attempt link-up poll budget and cold-boot retraining retry count. */
#define PCIE_LINK_WAIT_US		50000
#define PCIE_LINK_WAIT_STEP_US		10
#define PCIE_LINK_TRAIN_RETRIES		10

struct zumapro_pcie {
	struct dw_pcie		pci;
	struct clk_bulk_data	*clks;
	int			num_clks;
	struct phy		*phy;
	struct gpio_desc	*perst;
	phys_addr_t		pmu_phys;
	/* Optional CP (modem) rail-sequencing lines; see cp_power_on(). */
	struct gpio_desc	*cp_pda_active;
	struct gpio_desc	*cp_wakeup;
	struct gpio_desc	*cp_pm_wrst;
	struct gpio_desc	*cp_pwr;
	struct gpio_desc	*cp_nreset;
	struct gpio_desc	*cp_wrst;
};

/*
 * Bring-up scaffolding: power the Exynos Modem 5300 endpoint so its mask-ROM
 * comes up PCIe-visible.  The sequence is the downstream
 * modem_ctrl_s5100.c power_on_cp()/gpio_power_offon_cp() (non-WRESET_WA)
 * path: assert pda_active, run the power-off half so a warm-rebooted CP
 * reaches a clean cold state, then the rail power-on order, then the 200 ms
 * ROM settle downstream applies before link-up (register_pcie()).  The ROM
 * boots over PCIe and parks waiting for a boot-image doorbell, so nothing
 * beyond link training and enumeration happens until a modem driver exists.
 * This moves into that modem-control driver once the cpif port lands.
 */
static void zumapro_pcie_cp_power_on(struct zumapro_pcie *zp)
{
	dev_info(zp->pci.dev, "powering on the CP (modem) endpoint\n");

	gpiod_direction_output(zp->cp_pda_active, 1);

	/* power-off half: reach a clean cold state */
	gpiod_direction_output(zp->cp_wakeup, 1);
	msleep(10);
	gpiod_set_value_cansleep(zp->cp_wakeup, 0);
	gpiod_direction_output(zp->cp_nreset, 0);
	gpiod_direction_output(zp->cp_wrst, 0);
	gpiod_direction_output(zp->cp_pwr, 0);
	msleep(30);
	gpiod_direction_output(zp->cp_pm_wrst, 0);
	msleep(50);

	/* power-on half */
	gpiod_set_value_cansleep(zp->cp_pm_wrst, 1);
	msleep(10);
	gpiod_set_value_cansleep(zp->cp_pwr, 1);
	msleep(10);
	gpiod_set_value_cansleep(zp->cp_nreset, 1);
	msleep(10);
	gpiod_set_value_cansleep(zp->cp_wrst, 1);

	/* ROM settle before link training */
	msleep(200);
}

static int zumapro_pcie_cp_get_gpios(struct zumapro_pcie *zp)
{
	struct device *dev = zp->pci.dev;

	/*
	 * All lines are requested as-is: the power-on sequence sets each
	 * direction and level in the downstream order, so requesting them
	 * output-low here would drop a possibly-running CP's rails in
	 * request order instead.
	 */
	zp->cp_pwr = devm_gpiod_get_optional(dev, "google,cp-pwr", GPIOD_ASIS);
	if (IS_ERR(zp->cp_pwr))
		return PTR_ERR(zp->cp_pwr);
	if (!zp->cp_pwr)
		return 0;

	zp->cp_pda_active = devm_gpiod_get(dev, "google,cp-pda-active",
					   GPIOD_ASIS);
	zp->cp_wakeup = devm_gpiod_get(dev, "google,cp-wakeup", GPIOD_ASIS);
	zp->cp_pm_wrst = devm_gpiod_get(dev, "google,cp-pm-wrst", GPIOD_ASIS);
	zp->cp_nreset = devm_gpiod_get(dev, "google,cp-nreset", GPIOD_ASIS);
	zp->cp_wrst = devm_gpiod_get(dev, "google,cp-wrst", GPIOD_ASIS);
	if (IS_ERR(zp->cp_pda_active) || IS_ERR(zp->cp_wakeup) ||
	    IS_ERR(zp->cp_pm_wrst) || IS_ERR(zp->cp_nreset) ||
	    IS_ERR(zp->cp_wrst))
		return dev_err_probe(dev, -EINVAL,
				     "incomplete CP power-on GPIO set\n");

	return 0;
}

/*
 * Controller reset and PMA reset pulse before PHY bring-up.  The PMA reset is
 * pulsed and left *released* (=1): downstream runs the whole phy_config
 * (external-PLL select/lock and the PMA PLL/CDR/OC locks) with the PMA out of
 * reset, and on HW a working link reads PMA_RST = 1 while 0 = held.  Holding
 * the PMA asserted across phy_power_on blocks the external PLL from locking.
 */
static void zumapro_pcie_assert_phy_reset(struct zumapro_pcie *zp)
{
	void __iomem *elbi = zp->pci.elbi_base;

	writel(DEVICE_TYPE_RC, elbi + PCIE_DEVICE_TYPE);

	writel(SOFT_RESET_ALL, elbi + PCIE_SOFT_RESET);
	writel(SOFT_RESET_PWR_PULSE, elbi + PCIE_SOFT_RESET);
	udelay(10);
	writel(SOFT_RESET_ALL, elbi + PCIE_SOFT_RESET);
	udelay(10);

	writel(0, elbi + PCIE_PMA_RST_1);
	writel(0, elbi + PCIE_PMA_RST_2);
	writel(0, elbi + PCIE_PMA_RST_0);
	udelay(10);
	writel(1, elbi + PCIE_PMA_RST_1);
	writel(1, elbi + PCIE_PMA_RST_2);
	writel(1, elbi + PCIE_PMA_RST_0);

	writel(1, elbi + PCIE_SLV_PEND_SEL_NAK);
}

/* ELBI app-layer configuration done after the PHY is locked. */
static void zumapro_pcie_config_elbi(struct zumapro_pcie *zp)
{
	void __iomem *elbi = zp->pci.elbi_base;
	u32 val;

	writel(DBI_L1_EXIT_DISABLE, elbi + PCIE_DBI_L1_EXIT_DISABLE);

	val = readl(elbi + PCIE_APP_REQ_EXIT_L1_MODE);
	val |= APP_REQ_EXIT_L1_MODE | L1_REQ_NAK_CTRL_MASTER;
	writel(val, elbi + PCIE_APP_REQ_EXIT_L1_MODE);

	writel(LINKDOWN_RST_MANUAL, elbi + PCIE_LINKDOWN_RST_CTRL_SEL);
	writel(APP_XFER_PENDING, elbi + PCIE_APP_XFER_PENDING);

	/* Disable Q-channel clock gating (zumapro ip-ver >= 0x889500 path). */
	val = readl(elbi + PCIE_QCH_SEL);
	val &= ~(CLOCK_GATING_PMU_MASK | CLOCK_GATING_APB_MASK |
		 CLOCK_GATING_AXI_MASK);
	writel(val, elbi + PCIE_QCH_SEL);

	writel(PEND_SEL_NAK, elbi + PCIE_MSTR_PEND_SEL_NAK);
}

static int zumapro_pcie_wait_link_up(struct dw_pcie *pci)
{
	u32 state;

	return readl_poll_timeout(pci->elbi_base + PCIE_ELBI_RDLH_LINKUP, state,
				  (state & LTSSM_STATE_MASK) >= LTSSM_STATE_RCVRY_LOCK &&
				  (state & LTSSM_STATE_MASK) <= LTSSM_STATE_L1_IDLE,
				  PCIE_LINK_WAIT_STEP_US, PCIE_LINK_WAIT_US);
}

/*
 * Enable LTSSM and wait for the link.  Cold-boot link training on this SoC is
 * marginal and often needs several attempts (downstream retries up to 10
 * times), so on failure reset the endpoint over PERST# and retrain.  Only the
 * endpoint reset and the app-layer LTSSM enable are toggled here; the
 * controller core reset is left alone so the dw_pcie_setup_rc() RC config that
 * already ran survives across retries.
 */
static int zumapro_pcie_start_link(struct dw_pcie *pci)
{
	struct zumapro_pcie *zp = to_zumapro_pcie(pci);
	int try, ret;

	for (try = 0; try < PCIE_LINK_TRAIN_RETRIES; try++) {
		writel(LTSSM_ENABLE, pci->elbi_base + PCIE_APP_LTSSM_ENABLE);

		ret = zumapro_pcie_wait_link_up(pci);
		if (!ret) {
			/*
			 * config_elbi() asserted app_xfer_pending to hold the
			 * link in L0 through bring-up; downstream clears it
			 * once the link is established (steady state 0).
			 * Left asserted it vetoes every ASPM L1 entry, so the
			 * link never reaches the L1.x substates and CLKREQ#
			 * stays asserted -- the WLAN module then idles ~120mW
			 * above its floor even with the firmware in deep
			 * sleep.
			 */
			writel(0, pci->elbi_base + PCIE_APP_XFER_PENDING);
			return 0;
		}

		dev_info(pci->dev,
			 "link training attempt %d timed out, retraining\n",
			 try + 1);

		writel(0, pci->elbi_base + PCIE_APP_LTSSM_ENABLE);
		gpiod_set_value_cansleep(zp->perst, 1);
		usleep_range(1000, 2000);
		gpiod_set_value_cansleep(zp->perst, 0);
		usleep_range(PCIE_PERST_DELAY_US, PCIE_PERST_DELAY_US + 2000);
	}

	dev_err(pci->dev, "link failed to come up after %d attempts\n",
		PCIE_LINK_TRAIN_RETRIES);
	/* Downstream also releases the bring-up hold on the failure path. */
	writel(0, pci->elbi_base + PCIE_APP_XFER_PENDING);
	return -ETIMEDOUT;
}

static bool zumapro_pcie_link_up(struct dw_pcie *pci)
{
	u32 state = readl(pci->elbi_base + PCIE_ELBI_RDLH_LINKUP) &
		    LTSSM_STATE_MASK;

	return state >= LTSSM_STATE_RCVRY_LOCK && state <= LTSSM_STATE_L1_IDLE;
}

static const struct dw_pcie_ops zumapro_dw_pcie_ops = {
	.link_up	= zumapro_pcie_link_up,
	.start_link	= zumapro_pcie_start_link,
};

/* Masked read-modify-write of a secure PMU register through the EL3 SMC. */
static int zumapro_pcie_pmu_rmw(phys_addr_t reg, u32 mask, u32 val)
{
	struct arm_smccc_res res;

	arm_smccc_smc(EXYNOS_SMC_CMD_PRIV_REG, reg, EXYNOS_PRIV_REG_OPT_RMW,
		      mask, val, 0, 0, 0, &res);

	return res.a0 ? -EIO : 0;
}

static int zumapro_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct zumapro_pcie *zp = to_zumapro_pcie(pci);
	int ret;

	/* Power the modem endpoint before the link comes up (stub). */
	if (zp->cp_pwr)
		zumapro_pcie_cp_power_on(zp);

	/* Release the PHY from PMU isolation. */
	ret = phy_init(zp->phy);
	if (ret)
		return ret;

	/* Controller reset; PMA left released for PHY bring-up. */
	zumapro_pcie_assert_phy_reset(zp);

	ret = phy_power_on(zp->phy);
	if (ret)
		goto err_phy_exit;

	ret = phy_calibrate(zp->phy);
	if (ret)
		goto err_phy_off;

	zumapro_pcie_config_elbi(zp);

	/*
	 * Set the PMU PCIe wakeup control bit.  Downstream pokes this bit-10 of
	 * PCIE_PHY_CONTROL after the PHY reset; whether it is required for
	 * link-up on bare hardware is still TRACE NEEDED.
	 */
	ret = zumapro_pcie_pmu_rmw(zp->pmu_phys, PCIE_PMU_WAKE_CTRL,
				   PCIE_PMU_WAKE_CTRL);
	if (ret)
		dev_warn(pci->dev, "failed to set PMU PCIe wake control\n");

	/* Release PERST# and let the endpoint settle before link training. */
	gpiod_set_value_cansleep(zp->perst, 0);
	usleep_range(PCIE_PERST_DELAY_US, PCIE_PERST_DELAY_US + 2000);

	return 0;

err_phy_off:
	phy_power_off(zp->phy);
err_phy_exit:
	phy_exit(zp->phy);
	return ret;
}

/*
 * Enable ASPM (including L1.1/L1.2) on one device.  The substate timing, LTR
 * and L1SS enable bits are all programmed by the generic ASPM core; the host
 * driver's only job is to request the states once the device is enumerated.
 * Mirrors pcie-qcom.c qcom_pcie_enable_aspm().  The on-board BCM4383 needs the
 * link to reach L1.2 before its firmware will enter in-band deep sleep.
 */
static int zumapro_pcie_enable_aspm(struct pci_dev *pdev, void *userdata)
{
	int l1ss = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_L1SS);
	u16 lnkctl = 0, devctl2 = 0;
	u32 l1ss_ctl1 = 0;

	/* Substates may only be enabled from D0 (matches pcie-qcom.c). */
	pci_set_power_state_locked(pdev, PCI_D0);
	pci_enable_link_state_locked(pdev, PCIE_LINK_STATE_ALL);

	/* Report what the core actually managed to turn on, per device. */
	pcie_capability_read_word(pdev, PCI_EXP_LNKCTL, &lnkctl);
	pcie_capability_read_word(pdev, PCI_EXP_DEVCTL2, &devctl2);
	if (l1ss)
		pci_read_config_dword(pdev, l1ss + PCI_L1SS_CTL1, &l1ss_ctl1);
	pci_dbg(pdev, "ASPM lnkctl=%#06x (L1=%d CCC=%d CLKREQ_EN=%d) LTR=%d L1SS_CTL1=%#x\n",
		lnkctl,
		!!(lnkctl & PCI_EXP_LNKCTL_ASPM_L1),
		!!(lnkctl & PCI_EXP_LNKCTL_CCC),
		!!(lnkctl & PCI_EXP_LNKCTL_CLKREQ_EN),
		!!(devctl2 & PCI_EXP_DEVCTL2_LTR_EN), l1ss_ctl1);

	return 0;
}

/*
 * The endpoint reports the latencies from its LTR extended capability in the
 * LTR messages it sends, and L1.2 entry requires the reported latency to be
 * at or above the link's LTR_L1_2_THRESHOLD.  Nothing on a DT platform
 * programs that capability (on ACPI systems the BIOS does), so it stays 0,
 * the device reports "no latency tolerance", and the link never descends
 * below L1.1 -- the WLAN module then idles well above its L1.2 power floor.
 * Downstream bcmdhd writes 3 ms snoop/no-snoop before enabling L1SS
 * (exynos_pcie_rc_set_l1ss(), WIFI_L1SS_LTR_LATENCY); mirror that.
 */
#define ZUMAPRO_PCIE_LTR_3MS	0x1003	/* scale 4 (~1.05 ms/unit), value 3 */

static void zumapro_pcie_set_ep_ltr_latency(struct pci_dev *ep)
{
	int ltr = pci_find_ext_capability(ep, PCI_EXT_CAP_ID_LTR);

	if (!ltr) {
		pci_info(ep, "no LTR capability, L1.2 stays ineligible\n");
		return;
	}

	pci_write_config_word(ep, ltr + PCI_LTR_MAX_SNOOP_LAT,
			      ZUMAPRO_PCIE_LTR_3MS);
	pci_write_config_word(ep, ltr + PCI_LTR_MAX_NOSNOOP_LAT,
			      ZUMAPRO_PCIE_LTR_3MS);
	pci_dbg(ep, "LTR max snoop/no-snoop latency set to 3 ms\n");
}

/*
 * The ASPM core enables the L1 PM Substates by writing PCI_L1SS_CTL1 through
 * normal config accesses, but the DWC root port's config space is read-only
 * unless DBI read-only-write is enabled, and dw_pcie_own_conf_map_bus() does
 * not toggle it.  So the core sets the endpoint's enable bits while the root
 * port keeps its synthesis default (substates disabled), and the link never
 * descends below L1.0.  Re-apply the endpoint's enable bits to the root port
 * through the DBI to complete the link's L1SS configuration.
 */
static void zumapro_pcie_fixup_rc_l1ss(struct dw_pcie *pci, struct pci_dev *ep)
{
	u16 rc_l1ss = dw_pcie_find_ext_capability(pci, PCI_EXT_CAP_ID_L1SS);
	int ep_l1ss = pci_find_ext_capability(ep, PCI_EXT_CAP_ID_L1SS);
	u32 ep_ctl1 = 0, val;

	if (!rc_l1ss || !ep_l1ss)
		return;

	pci_read_config_dword(ep, ep_l1ss + PCI_L1SS_CTL1, &ep_ctl1);

	dw_pcie_dbi_ro_wr_en(pci);
	val = dw_pcie_read_dbi(pci, rc_l1ss + PCI_L1SS_CTL1, 0x4);
	val &= ~PCI_L1SS_CTL1_L1SS_MASK;
	val |= ep_ctl1 & PCI_L1SS_CTL1_L1SS_MASK;
	dw_pcie_write_dbi(pci, rc_l1ss + PCI_L1SS_CTL1, 0x4, val);
	dw_pcie_dbi_ro_wr_dis(pci);

	dev_dbg(pci->dev, "L1SS: root-port CTL1 fixed up to %#x\n",
		dw_pcie_read_dbi(pci, rc_l1ss + PCI_L1SS_CTL1, 0x4));
}

/*
 * Runs after pci_host_probe() has enumerated the endpoint.  The DWC core only
 * leaves the RC's L1 PM Substate capability advertised because probe() sets
 * pci->l1ss_support (see dw_pcie_hide_unsupported_l1ss()); with the capability
 * visible on both ends, walking the hierarchy and enabling the link states
 * lets the ASPM core do the actual L1SS/LTR register programming.
 */
static void zumapro_pcie_host_post_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct zumapro_pcie *zp = to_zumapro_pcie(pci);
	struct pci_dev *rp, *ep = NULL;

	/*
	 * Skip the ASPM/L1SS walk on the modem channel: the endpoint here is
	 * the CP mask ROM parked mid-boot-protocol, and downstream keeps
	 * L1SS off for the whole CP boot (s51xx_pcie_restore_state()) and
	 * only enables L1.2 from complete_normal_boot() once the CP is
	 * ONLINE.  Power management of the modem link is the modem driver's
	 * business, not enumeration's.
	 */
	if (zp->cp_pwr)
		return;

	rp = pci_get_slot(pp->bridge->bus, PCI_DEVFN(0, 0));
	if (rp && rp->subordinate)
		ep = pci_get_slot(rp->subordinate, PCI_DEVFN(0, 0));

	/* Latency must be in place before LTR_EN/L1.2 get enabled below. */
	if (ep)
		zumapro_pcie_set_ep_ltr_latency(ep);

	pci_walk_bus(pp->bridge->bus, zumapro_pcie_enable_aspm, NULL);

	if (ep)
		zumapro_pcie_fixup_rc_l1ss(pci, ep);
	pci_dev_put(ep);
	pci_dev_put(rp);
}

static const struct dw_pcie_host_ops zumapro_pcie_host_ops = {
	.init		= zumapro_pcie_host_init,
	.post_init	= zumapro_pcie_host_post_init,
};

/*
 * Modem (s5300) boot-assist hooks; see include/linux/pcie-zumapro.h.  Callers
 * resolve the RC platform device from a DT phandle, so guard against being
 * handed a device another driver owns.  Single-caller bring-up scaffolding:
 * no locking against concurrent host operations.
 */
static struct zumapro_pcie *zumapro_pcie_from_dev(struct device *rc_dev)
{
	if (!rc_dev->driver ||
	    rc_dev->driver->of_match_table != zumapro_pcie_of_match)
		return NULL;

	return dev_get_drvdata(rc_dev);
}

/*
 * Point the iMSI-RX termination address at the modem's MSI carveout.  The
 * default target (dw_pcie_msi_host_init() picks cfg0_base) is fine for pure
 * MSI senders, but the s5300 mask ROM treats its MSI capability address as
 * the base of a 4K status block and DMA-writes boot_stage and the boot-image
 * descriptor response at offsets above it -- those writes pass the iMSI-RX
 * address filter and hit the bus, so they must land in the dedicated
 * carveout, not in the config window.  Mirrors downstream
 * exynos_pcie_set_msi_ctrl_addr().  Must run before the endpoint's MSI
 * vectors are allocated so the capability is composed with this address.
 */
int zumapro_pcie_set_msi_target(struct device *rc_dev, phys_addr_t target)
{
	struct zumapro_pcie *zp = zumapro_pcie_from_dev(rc_dev);

	if (!zp)
		return -ENODEV;

	zp->pci.pp.msi_data = target;
	dw_pcie_writel_dbi(&zp->pci, PCIE_MSI_ADDR_LO, lower_32_bits(target));
	dw_pcie_writel_dbi(&zp->pci, PCIE_MSI_ADDR_HI, upper_32_bits(target));

	dev_info(rc_dev, "MSI target moved to %pap\n", &target);

	return 0;
}
EXPORT_SYMBOL_GPL(zumapro_pcie_set_msi_target);

/*
 * Link bounce for the modem boot handshake: after the first-stage download
 * the CP bootloader expects the link to drop and retrain (downstream
 * start_normal_boot() runs poweroff/poweron between the boot_stage poll and
 * the link-ack doorbell).  Only PERST# and the app-layer LTSSM enable are
 * toggled -- the same minimal cycle the start_link() retry loop already uses
 * on this hardware -- so the RC core config from dw_pcie_setup_rc() survives.
 * If the bounce proves insufficient on hardware, the escalation path is the
 * downstream full poweroff/poweron (PHY power cycle included).
 */
int zumapro_pcie_modem_link_down(struct device *rc_dev)
{
	struct zumapro_pcie *zp = zumapro_pcie_from_dev(rc_dev);

	if (!zp)
		return -ENODEV;

	/*
	 * Downstream s5100_poweroff_pcie() deasserts AP2CP_WAKEUP before
	 * dropping the link; the CP samples it to know the AP intends the
	 * link to be down.  NULL-safe no-op when no CP lines exist (CH1).
	 */
	gpiod_set_value_cansleep(zp->cp_wakeup, 0);
	writel(0, zp->pci.elbi_base + PCIE_APP_LTSSM_ENABLE);
	gpiod_set_value_cansleep(zp->perst, 1);
	usleep_range(1000, 2000);

	return 0;
}
EXPORT_SYMBOL_GPL(zumapro_pcie_modem_link_down);

int zumapro_pcie_modem_link_up(struct device *rc_dev)
{
	struct zumapro_pcie *zp = zumapro_pcie_from_dev(rc_dev);

	if (!zp)
		return -ENODEV;

	/*
	 * Downstream s5100_poweron_pcie() asserts AP2CP_WAKEUP before every
	 * (re)train; the CP bootloader may gate its link participation on it.
	 * Left asserted afterwards -- the downstream steady-state while the
	 * link is up.
	 */
	gpiod_set_value_cansleep(zp->cp_wakeup, 1);
	gpiod_set_value_cansleep(zp->perst, 0);
	usleep_range(PCIE_PERST_DELAY_US, PCIE_PERST_DELAY_US + 2000);

	return zumapro_pcie_start_link(&zp->pci);
}
EXPORT_SYMBOL_GPL(zumapro_pcie_modem_link_up);

static int zumapro_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct zumapro_pcie *zp;
	struct of_phandle_args pmu_args;
	struct resource pmu_res;
	int ret;

	zp = devm_kzalloc(dev, sizeof(*zp), GFP_KERNEL);
	if (!zp)
		return -ENOMEM;

	zp->pci.dev = dev;
	zp->pci.ops = &zumapro_dw_pcie_ops;
	zp->pci.pp.ops = &zumapro_pcie_host_ops;

	/*
	 * Keep the L1 PM Substate capability advertised so the ASPM core can
	 * manage L1.1/L1.2; otherwise the DWC core hides it (it requires
	 * CLKREQ#, which the board's PCIe pinmux provides).
	 */
	zp->pci.l1ss_support = true;

	zp->phy = devm_of_phy_get(dev, np, NULL);
	if (IS_ERR(zp->phy))
		return dev_err_probe(dev, PTR_ERR(zp->phy), "failed to get PHY\n");

	zp->num_clks = devm_clk_bulk_get_all_enabled(dev, &zp->clks);
	if (zp->num_clks < 0)
		return dev_err_probe(dev, zp->num_clks, "failed to get clocks\n");

	/* PERST# held asserted until host_init releases it. */
	zp->perst = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(zp->perst))
		return dev_err_probe(dev, PTR_ERR(zp->perst),
				     "failed to get reset GPIO\n");

	ret = zumapro_pcie_cp_get_gpios(zp);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to get CP power-on GPIOs\n");

	ret = of_parse_phandle_with_fixed_args(np, "samsung,pmu-syscon",
					       1, 0, &pmu_args);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get PMU syscon\n");
	ret = of_address_to_resource(pmu_args.np, 0, &pmu_res);
	of_node_put(pmu_args.np);
	if (ret)
		return dev_err_probe(dev, ret, "failed to resolve PMU base\n");
	zp->pmu_phys = pmu_res.start + pmu_args.args[0];

	platform_set_drvdata(pdev, zp);

	ret = dw_pcie_host_init(&zp->pci.pp);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize host\n");

	return 0;
}

static void zumapro_pcie_remove(struct platform_device *pdev)
{
	struct zumapro_pcie *zp = platform_get_drvdata(pdev);

	dw_pcie_host_deinit(&zp->pci.pp);
	phy_power_off(zp->phy);
	phy_exit(zp->phy);
}

static const struct of_device_id zumapro_pcie_of_match[] = {
	{ .compatible = "google,zumapro-pcie" },
	{ },
};
MODULE_DEVICE_TABLE(of, zumapro_pcie_of_match);

static struct platform_driver zumapro_pcie_driver = {
	.probe	= zumapro_pcie_probe,
	.remove	= zumapro_pcie_remove,
	.driver = {
		.name			= "zumapro-pcie",
		.of_match_table		= zumapro_pcie_of_match,
		.suppress_bind_attrs	= true,
	},
};
module_platform_driver(zumapro_pcie_driver);

MODULE_DESCRIPTION("Google Zumapro PCIe root complex driver");
MODULE_LICENSE("GPL");
