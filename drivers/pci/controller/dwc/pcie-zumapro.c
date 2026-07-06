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
#include <linux/bitfield.h>
#include <linux/bitmap.h>
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
#include <linux/phy/phy-zumapro-pcie.h>
#include <linux/platform_device.h>

#include "pcie-designware.h"

#define to_zumapro_pcie(x)	dev_get_drvdata((x)->dev)

static const struct of_device_id zumapro_pcie_of_match[];

/* ELBI (app/controller) registers, relative to pci->elbi_base. */
#define PCIE_IRQ0			0x0000
#define   IRQ_RADM_PM_TO_ACK		BIT(29)
#define PCIE_APP_LTSSM_ENABLE		0x0054
#define   LTSSM_ENABLE			0x1
#define PCIE_APP_REQ_EXIT_L1		0x006c
#define PCIE_APP_XFER_PENDING		0x0074
#define   APP_XFER_PENDING		0x1
#define PCIE_DEVICE_TYPE		0x0080
#define   DEVICE_TYPE_RC		0x4
#define PCIE_XMIT_PME_TURNOFF		0x0118
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
#define   LTSSM_STATE_L2_IDLE		0x15
#define PCIE_LINKDOWN_RST_CTRL_SEL	0x03a0
#define   LINKDOWN_RST_MANUAL		BIT(1)
#define PCIE_SOFT_RESET			0x03a4
#define   SOFT_RESET_ALL		0xf
#define   SOFT_RESET_PWR_PULSE		0xd	/* SOFT_RESET_ALL with PWR bit low */
#define   SOFT_PWR_RESET		BIT(1)	/* active-low: 1 released, 0 asserted */
#define   SOFT_NON_STICKY_RESET		BIT(3)	/* active-low, as above */
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
#define PCIE_L1_EXIT_WAIT_US		3000
#define PCIE_L1_EXIT_WAIT_STEP_US	10
/* PME_Turn_Off / L2 entry poll budget: downstream MAX_L2_TIMEOUT (2000) x 10 us. */
#define PCIE_L2_ENTER_WAIT_US		20000
#define PCIE_L2_ENTER_WAIT_STEP_US	10

struct zumapro_pcie {
	struct dw_pcie		pci;
	struct clk_bulk_data	*clks;
	int			num_clks;
	struct phy		*phy;
	struct gpio_desc	*perst;
	phys_addr_t		pmu_phys;
	/* Optional CP (modem) rail-sequencing lines; see cp_power_on(). */
	struct gpio_desc	*cp_pda_active;
	struct gpio_desc	*cp_dump_noti;
	struct gpio_desc	*cp_wakeup;
	struct gpio_desc	*cp_pm_wrst;
	struct gpio_desc	*cp_pwr;
	struct gpio_desc	*cp_nreset;
	struct gpio_desc	*cp_wrst;
	/* PHY was powered down by modem_link_down; link_up skips its off. */
	bool			cp_phy_off;
	/* Experiment: light bounce (LTSSM-only, no PERST/PHY reset). */
	bool			cp_light;
	/*
	 * iMSI-RX enable/mask/target snapshot taken in modem_link_down before
	 * SOFT_PWR_RESET wipes the controller core, restored in modem_link_up
	 * after dw_pcie_setup_rc().  The generic DWC MSI domain only writes the
	 * per-message ENABLE bits from the irq-chip unmask at request time, so a
	 * mid-boot bounce that resets the core would otherwise drop every modem
	 * MSI (the CP fires its post-link-ack notify on message 4).
	 */
	u32			saved_msi_enable;
	u32			saved_msi_mask;
	u32			saved_msi_addr_lo;
	u32			saved_msi_addr_hi;
	bool			msi_saved;
};

static struct zumapro_pcie *zumapro_pcie_from_dev(struct device *rc_dev);

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
/*
 * Downstream modem_ctrl_s5100.c power_on_cp() -> gpio_power_offon_cp()
 * (non-WRESET_WA): assert pda_active, run the power-off half then the power-on
 * order, then the 200 ms ROM settle before link-up.  The cp_pwr line is a soft
 * control -- the modem PMIC self-refreshes CP DRAM across it -- so this cycle
 * does NOT wipe the resident MAIN image (downstream warm-boots with only the
 * 92 KB PBL every time, which proves MAIN survives).
 */
static void zumapro_pcie_cp_power_on(struct zumapro_pcie *zp)
{
	dev_info(zp->pci.dev, "powering on the CP (modem) endpoint\n");

	gpiod_direction_output(zp->cp_pda_active, 1);
	/*
	 * Downstream power_on_cp() drives DUMP_NOTI low before the rails; a
	 * floating line risks CP MAIN booting into dump mode.  Optional so
	 * old DTBs keep working.
	 */
	if (zp->cp_dump_noti)
		gpiod_direction_output(zp->cp_dump_noti, 0);

	gpiod_direction_output(zp->cp_wakeup, 1);
	msleep(10);
	gpiod_set_value_cansleep(zp->cp_wakeup, 0);
	/*
	 * WARM reset (downstream gpio_power_wreset_cp, confirmed on the live
	 * device): hold cp_pwr/cp_nreset HIGH and pulse ONLY pm_wrst + cp_wrst.
	 * The full mainline-vs-downstream log diff isolated this cold
	 * cp_pwr/cp_nreset cycle as the SOLE remaining AP-side difference vs a
	 * working boot; cp_pwr is a soft power control (DRAM self-refreshes
	 * across it) so the cold drop plausibly power-cycles the CP PCIe
	 * controller and clears the sticky state that lets the doorbell-generate
	 * decode survive the later PERST bounce -- which the warm path keeps.
	 *
	 * Differs from the reverted "wrst-only" attempt (which pulsed cp_wrst
	 * alone and regressed the PBL download to 0x1ff): downstream pulses BOTH
	 * pm_wrst and cp_wrst, 50ms/10ms.  Caveat: if the bootloader left cp_pwr
	 * LOW (cold CP), forcing it high here is a fresh power-on, not a warm
	 * reset -- but on a warm boot after downstream it is already high.
	 * Risk: if this regresses the download to 0x1ff, the CP needs a graceful
	 * halt (cp2ap_cp_wrst handshake) first; revert to the cold cycle then.
	 */
	dev_info(zp->pci.dev, "TEGU_CP_TRACE gpio PWR=1 NRESET=1 (warm, held high)\n");
	gpiod_direction_output(zp->cp_pwr, 1);
	gpiod_direction_output(zp->cp_nreset, 1);
	dev_info(zp->pci.dev, "TEGU_CP_TRACE gpio CP_WRST=0\n");
	gpiod_direction_output(zp->cp_wrst, 0);
	dev_info(zp->pci.dev, "TEGU_CP_TRACE gpio PM_WRST=0\n");
	gpiod_direction_output(zp->cp_pm_wrst, 0);
	msleep(50);
	dev_info(zp->pci.dev, "TEGU_CP_TRACE gpio PM_WRST=1\n");
	gpiod_set_value_cansleep(zp->cp_pm_wrst, 1);
	msleep(10);
	dev_info(zp->pci.dev, "TEGU_CP_TRACE gpio CP_WRST=1\n");
	gpiod_set_value_cansleep(zp->cp_wrst, 1);

	/* ROM settle before link training */
	msleep(200);

	/*
	 * Downstream raises AP2CP_WAKEUP before every link-up, including
	 * this first ROM-phase one (s5100_poweron_pcie()); the CP only
	 * keeps its end of the link fully awake while it is high.
	 */
	gpiod_set_value_cansleep(zp->cp_wakeup, 1);
	msleep(5);
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
	zp->cp_dump_noti = devm_gpiod_get_optional(dev, "google,cp-dump-noti",
						   GPIOD_ASIS);
	if (IS_ERR(zp->cp_dump_noti))
		return PTR_ERR(zp->cp_dump_noti);
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

/*
 * Post-PLL-lock controller reset profile, replayed verbatim from downstream
 * establish_link() (pcie-exynos-rc.c: a SOFT_PWR_RESET pulse then a
 * SOFT_NON_STICKY_RESET pulse, both run *after* phy_config/PLL-lock and
 * *before* the app-layer ELBI config).  The from-scratch bring-up omitted the
 * NON_STICKY pulse entirely and only pulsed SOFT_PWR_RESET for udelay(10)
 * inside assert_phy_reset (pre-PLL).  Restoring the exact downstream
 * reset/clock profile matters because the modem's PERST-domain doorbell-generate
 * decode must survive the bounce, and its survival depends on precisely this
 * profile -- see research/modem-issues.md ("2026-07-06 prior-art deep-dive") and
 * research/prior-art/findings/A-rc-bounce.md.  Bits are active-low (1 released,
 * 0 asserted); downstream RMWs single bits and holds the assert for mdelay(1).
 */
static void zumapro_pcie_controller_reset_pulse(struct zumapro_pcie *zp)
{
	void __iomem *elbi = zp->pci.elbi_base;
	u32 val;

	/* SOFT_PWR_RESET pulse (assert held mdelay(1)). */
	val = readl(elbi + PCIE_SOFT_RESET);
	val &= ~SOFT_PWR_RESET;
	writel(val, elbi + PCIE_SOFT_RESET);
	mdelay(1);
	val |= SOFT_PWR_RESET;
	writel(val, elbi + PCIE_SOFT_RESET);

	/* Downstream re-asserts DEVICE_TYPE=RC here, after the power reset. */
	writel(DEVICE_TYPE_RC, elbi + PCIE_DEVICE_TYPE);

	/* SOFT_NON_STICKY_RESET pulse (release, settle, assert mdelay(1), release). */
	val = readl(elbi + PCIE_SOFT_RESET);
	val |= SOFT_NON_STICKY_RESET;
	writel(val, elbi + PCIE_SOFT_RESET);
	usleep_range(10, 12);
	val &= ~SOFT_NON_STICKY_RESET;
	writel(val, elbi + PCIE_SOFT_RESET);
	mdelay(1);
	val |= SOFT_NON_STICKY_RESET;
	writel(val, elbi + PCIE_SOFT_RESET);
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
		pr_err("BRK SL: ltssm1 try %d\n", try + 1);
		writel(LTSSM_ENABLE, pci->elbi_base + PCIE_APP_LTSSM_ENABLE);
		pr_err("BRK SL: polling\n");

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

		/*
		 * The endpoint may have stopped driving CLKREQ# by now (a
		 * crashed CP bootloader did on hardware, wedging the whole
		 * interconnect on the next ELBI write); park the sub-block
		 * clock on the OSC for the dead-link window and only return
		 * to HW mode for the retrain itself.
		 */
		zumapro_pcie_phy_safe_clk(zp->phy, true);
		pr_err("BRK SL: ltssm0\n");
		writel(0, pci->elbi_base + PCIE_APP_LTSSM_ENABLE);
		pr_err("BRK SL: perst cycle\n");
		gpiod_set_value_cansleep(zp->perst, 1);
		usleep_range(1000, 2000);
		gpiod_set_value_cansleep(zp->perst, 0);
		usleep_range(PCIE_PERST_DELAY_US, PCIE_PERST_DELAY_US + 2000);
		zumapro_pcie_phy_safe_clk(zp->phy, false);
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

/* Downstream CP L1SS programming values. */
#define ZUMAPRO_PCIE_CP_TCOMMON_32US		(0x20 << 8)
#define ZUMAPRO_PCIE_CP_TPOWERON_200US		0xa1
#define ZUMAPRO_PCIE_ACK_F_ASPM_CONTROL		0x70c
#define ZUMAPRO_PCIE_L1_ENTRANCE_LATENCY	GENMASK(29, 27)
#define ZUMAPRO_PCIE_L1_ENTRANCE_LATENCY_64US	FIELD_PREP(GENMASK(29, 27), 7)

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

static int zumapro_pcie_modem_request_l1_exit(struct zumapro_pcie *zp)
{
	struct dw_pcie *pci = &zp->pci;
	void __iomem *elbi = pci->elbi_base;
	u32 mode, state;
	int ret;

	dev_info(pci->dev,
		 "modem L1 exit pre: rdlh %#010x req %#010x mode %#010x xfer %#010x\n",
		 readl(elbi + PCIE_ELBI_RDLH_LINKUP),
		 readl(elbi + PCIE_APP_REQ_EXIT_L1),
		 readl(elbi + PCIE_APP_REQ_EXIT_L1_MODE),
		 readl(elbi + PCIE_APP_XFER_PENDING));

	writel(1, elbi + PCIE_APP_REQ_EXIT_L1);

	mode = readl(elbi + PCIE_APP_REQ_EXIT_L1_MODE);
	mode &= ~APP_REQ_EXIT_L1_MODE;
	writel(mode, elbi + PCIE_APP_REQ_EXIT_L1_MODE);

	ret = readl_poll_timeout(elbi + PCIE_ELBI_RDLH_LINKUP, state,
				 (state & LTSSM_STATE_MASK) == LTSSM_STATE_L0,
				 PCIE_L1_EXIT_WAIT_STEP_US,
				 PCIE_L1_EXIT_WAIT_US);
	if (ret)
		dev_warn(pci->dev,
			 "modem L1 exit did not reach L0: rdlh %#010x state %#x\n",
			 state, state & LTSSM_STATE_MASK);

	mode = readl(elbi + PCIE_APP_REQ_EXIT_L1_MODE);
	mode |= APP_REQ_EXIT_L1_MODE | L1_REQ_NAK_CTRL_MASTER;
	writel(mode, elbi + PCIE_APP_REQ_EXIT_L1_MODE);
	writel(0, elbi + PCIE_APP_REQ_EXIT_L1);

	dev_info(pci->dev,
		 "modem L1 exit post: rdlh %#010x req %#010x mode %#010x xfer %#010x\n",
		 readl(elbi + PCIE_ELBI_RDLH_LINKUP),
		 readl(elbi + PCIE_APP_REQ_EXIT_L1),
		 readl(elbi + PCIE_APP_REQ_EXIT_L1_MODE),
		 readl(elbi + PCIE_APP_XFER_PENDING));

	return ret;
}

int zumapro_pcie_modem_enable_l1ss(struct device *rc_dev, struct pci_dev *ep)
{
	struct zumapro_pcie *zp = zumapro_pcie_from_dev(rc_dev);
	struct dw_pcie *pci;
	u16 rc_exp, rc_lnkctl, rc_devctl2;
	u16 ep_lnkctl = 0, ep_devctl2 = 0;
	u16 rc_l1ss;
	int ep_l1ss;
	u32 rc_lnkcap = 0, ep_lnkcap = 0;
	u32 rc_ctl1, rc_ctl2, ep_ctl1 = 0, ep_ctl2 = 0, ep_ack = 0;
	u32 app_req, app_mode, app_xfer, rdlh;
	u32 val;
	int ret;

	if (!zp || !ep)
		return -ENODEV;

	pci = &zp->pci;
	rc_exp = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	rc_l1ss = dw_pcie_find_ext_capability(pci, PCI_EXT_CAP_ID_L1SS);
	ep_l1ss = pci_find_ext_capability(ep, PCI_EXT_CAP_ID_L1SS);
	if (!rc_exp || !rc_l1ss || !ep_l1ss) {
		dev_warn(pci->dev,
			 "modem L1SS skipped: rc_exp %#x rc_l1ss %#x ep_l1ss %#x\n",
			 rc_exp, rc_l1ss, ep_l1ss);
		return -ENODEV;
	}

	ret = pci_set_power_state(ep, PCI_D0);
	if (ret)
		dev_warn(&ep->dev, "failed to force D0 before L1SS: %d\n", ret);

	rc_lnkcap = dw_pcie_readl_dbi(pci, rc_exp + PCI_EXP_LNKCAP);
	ret = pcie_capability_read_dword(ep, PCI_EXP_LNKCAP, &ep_lnkcap);
	if (ret)
		dev_warn(&ep->dev, "failed to read endpoint Link Capabilities: %d\n",
			 ret);

	ret = zumapro_pcie_modem_request_l1_exit(zp);
	if (ret)
		dev_warn(pci->dev,
			 "continuing modem L1SS setup after L1 exit failure: %d\n",
			 ret);

	/*
	 * Mirror downstream exynos_pcie_rc_set_l1ss() for EP_SAMSUNG_MODEM:
	 * RC L1SS/TPOWERON/LTR, then EP LTR/TPOWERON/entrance latency/L1SS,
	 * then ASPM L1+CCC on the RC and ASPM L1+CCC+CLKREQ on the EP.
	 */
	dw_pcie_dbi_ro_wr_en(pci);
	val = dw_pcie_readl_dbi(pci, rc_l1ss + PCI_L1SS_CTL1);
	val |= ZUMAPRO_PCIE_CP_TCOMMON_32US | PCI_L1SS_CTL1_L1SS_MASK;
	dw_pcie_writel_dbi(pci, rc_l1ss + PCI_L1SS_CTL1, val);
	dw_pcie_writel_dbi(pci, rc_l1ss + PCI_L1SS_CTL2,
			   ZUMAPRO_PCIE_CP_TPOWERON_200US);

	rc_devctl2 = dw_pcie_readw_dbi(pci, rc_exp + PCI_EXP_DEVCTL2);
	rc_devctl2 |= PCI_EXP_DEVCTL2_LTR_EN;
	dw_pcie_writew_dbi(pci, rc_exp + PCI_EXP_DEVCTL2, rc_devctl2);
	dw_pcie_dbi_ro_wr_dis(pci);

	ret = pcie_capability_set_word(ep, PCI_EXP_DEVCTL2,
				       PCI_EXP_DEVCTL2_LTR_EN);
	if (ret)
		dev_warn(&ep->dev, "failed to enable endpoint LTR: %d\n", ret);

	pci_write_config_dword(ep, ep_l1ss + PCI_L1SS_CTL2,
			       ZUMAPRO_PCIE_CP_TPOWERON_200US);

	ret = pci_read_config_dword(ep, ZUMAPRO_PCIE_ACK_F_ASPM_CONTROL, &val);
	if (ret) {
		dev_warn(&ep->dev, "failed to read endpoint ASPM control: %d\n",
			 ret);
	} else {
		val &= ~ZUMAPRO_PCIE_L1_ENTRANCE_LATENCY;
		val |= ZUMAPRO_PCIE_L1_ENTRANCE_LATENCY_64US;
		pci_write_config_dword(ep, ZUMAPRO_PCIE_ACK_F_ASPM_CONTROL, val);
	}

	pci_read_config_dword(ep, ep_l1ss + PCI_L1SS_CTL1, &val);
	val |= PCI_L1SS_CTL1_L1SS_MASK;
	pci_write_config_dword(ep, ep_l1ss + PCI_L1SS_CTL1, val);

	dw_pcie_dbi_ro_wr_en(pci);
	rc_lnkctl = dw_pcie_readw_dbi(pci, rc_exp + PCI_EXP_LNKCTL);
	rc_lnkctl &= ~PCI_EXP_LNKCTL_ASPMC;
	rc_lnkctl |= PCI_EXP_LNKCTL_CCC | PCI_EXP_LNKCTL_ASPM_L1;
	dw_pcie_writew_dbi(pci, rc_exp + PCI_EXP_LNKCTL, rc_lnkctl);
	dw_pcie_dbi_ro_wr_dis(pci);

	ret = pcie_capability_clear_and_set_word(ep, PCI_EXP_LNKCTL,
						 PCI_EXP_LNKCTL_ASPMC,
						 PCI_EXP_LNKCTL_CCC |
						 PCI_EXP_LNKCTL_CLKREQ_EN |
						 PCI_EXP_LNKCTL_ASPM_L1);
	if (ret)
		dev_warn(&ep->dev, "failed to enable endpoint ASPM L1: %d\n",
			 ret);

	rc_lnkctl = dw_pcie_readw_dbi(pci, rc_exp + PCI_EXP_LNKCTL);
	rc_devctl2 = dw_pcie_readw_dbi(pci, rc_exp + PCI_EXP_DEVCTL2);
	rc_ctl1 = dw_pcie_readl_dbi(pci, rc_l1ss + PCI_L1SS_CTL1);
	rc_ctl2 = dw_pcie_readl_dbi(pci, rc_l1ss + PCI_L1SS_CTL2);
	pcie_capability_read_word(ep, PCI_EXP_LNKCTL, &ep_lnkctl);
	pcie_capability_read_word(ep, PCI_EXP_DEVCTL2, &ep_devctl2);
	pci_read_config_dword(ep, ep_l1ss + PCI_L1SS_CTL1, &ep_ctl1);
	pci_read_config_dword(ep, ep_l1ss + PCI_L1SS_CTL2, &ep_ctl2);
	pci_read_config_dword(ep, ZUMAPRO_PCIE_ACK_F_ASPM_CONTROL, &ep_ack);
	app_req = readl(pci->elbi_base + PCIE_APP_REQ_EXIT_L1);
	app_mode = readl(pci->elbi_base + PCIE_APP_REQ_EXIT_L1_MODE);
	app_xfer = readl(pci->elbi_base + PCIE_APP_XFER_PENDING);
	rdlh = readl(pci->elbi_base + PCIE_ELBI_RDLH_LINKUP);
	dev_info(pci->dev,
		 "modem L1SS enabled: RC lnkcap %#010x(clkpm=%d) lnkctl %#06x devctl2 %#06x l1ss %#010x/%#010x; EP lnkcap %#010x(clkpm=%d) lnkctl %#06x devctl2 %#06x l1ss %#010x/%#010x ack %#010x; app req %#010x mode %#010x xfer %#010x rdlh %#010x\n",
		 rc_lnkcap, !!(rc_lnkcap & PCI_EXP_LNKCAP_CLKPM), rc_lnkctl,
		 rc_devctl2, rc_ctl1, rc_ctl2, ep_lnkcap,
		 !!(ep_lnkcap & PCI_EXP_LNKCAP_CLKPM), ep_lnkctl, ep_devctl2,
		 ep_ctl1, ep_ctl2, ep_ack, app_req, app_mode, app_xfer, rdlh);

	return 0;
}
EXPORT_SYMBOL_GPL(zumapro_pcie_modem_enable_l1ss);

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
	 * the CP mask ROM parked mid-boot-protocol.  Power management of the
	 * modem link is the modem driver's business, not enumeration's -- it
	 * enables CP L1SS itself right after the post-bounce restore
	 * (s51xx_pcie_restore_state(boot_on=false), before the link-ack
	 * doorbell; complete_normal_boot()'s later enable is a redundant
	 * second call).
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
 * Reserve the first @count MSI vectors on this root complex so the next
 * endpoint allocation lands above them.  The s5300 CP signals the AP on MSI
 * message-data base 4 (m1n1 trace of a working downstream boot: EP data 4, RC
 * iMSI-RX ENABLE 0xf1 / MASK 0xffffff0c, INIT_START and every MAIN-phase
 * interrupt on bit 4).  The generic DWC MSI domain otherwise hands out the
 * lowest free region (base 0); reserving vectors 0-3 makes the modem's
 * four-vector request land at base 4 with MME=2 -- the width the mask ROM
 * tolerates (requesting eight to reach bit 4 sets MME=3 and aborts the PBL
 * download at boot_stage 0x1ff / err 0x1000).  Must run before the endpoint's
 * MSI vectors are allocated.
 */
int zumapro_pcie_reserve_msi_base(struct device *rc_dev, unsigned int count)
{
	struct zumapro_pcie *zp = zumapro_pcie_from_dev(rc_dev);

	if (!zp)
		return -ENODEV;
	if (!count || count > zp->pci.pp.num_vectors)
		return -EINVAL;

	bitmap_set(zp->pci.pp.msi_irq_in_use, 0, count);
	dev_info(rc_dev, "reserved MSI vectors 0-%u so the modem lands at base %u\n",
		 count - 1, count);

	return 0;
}
EXPORT_SYMBOL_GPL(zumapro_pcie_reserve_msi_base);

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
/*
 * Experiment toggle: light bounce keeps the CP's 0x14E00000 PCIe controller
 * (and the ROM-programmed inbound doorbell decode BL1 polls) alive by
 * dropping and re-establishing the link with LTSSM alone -- no PERST, no PHY
 * power-cycle.  The heavy bounce resets that controller and the doorbell dies
 * post-bounce (hw + PBL disassembly: BL1 never rebuilds the decode).
 */
int zumapro_pcie_modem_set_light(struct device *rc_dev, bool light)
{
	struct zumapro_pcie *zp = zumapro_pcie_from_dev(rc_dev);

	if (!zp)
		return -ENODEV;
	zp->cp_light = light;
	dev_info(rc_dev, "modem bounce mode: %s\n", light ? "light" : "heavy");

	return 0;
}
EXPORT_SYMBOL_GPL(zumapro_pcie_modem_set_light);

/*
 * Diagnostic: dump the RC iMSI-RX enable/mask/status so the modem boot driver
 * can tell, after ringing the link-ack, whether the CP actually fired its
 * notify MSI (message 4) but the RC dropped it (STATUS bit set, ENABLE/MASK
 * wrong) versus the CP never firing at all (STATUS stays clear -- the doorbell
 * genuinely did not reach BL1).
 */
int zumapro_pcie_modem_msi_status(struct device *rc_dev)
{
	struct zumapro_pcie *zp = zumapro_pcie_from_dev(rc_dev);

	if (!zp)
		return -ENODEV;

	dev_info(rc_dev, "iMSI-RX now: en %#x mask %#x status %#x addr %#x:%#x\n",
		 dw_pcie_readl_dbi(&zp->pci, PCIE_MSI_INTR0_ENABLE),
		 dw_pcie_readl_dbi(&zp->pci, PCIE_MSI_INTR0_MASK),
		 dw_pcie_readl_dbi(&zp->pci, PCIE_MSI_INTR0_STATUS),
		 dw_pcie_readl_dbi(&zp->pci, PCIE_MSI_ADDR_HI),
		 dw_pcie_readl_dbi(&zp->pci, PCIE_MSI_ADDR_LO));

	return 0;
}
EXPORT_SYMBOL_GPL(zumapro_pcie_modem_msi_status);

int zumapro_pcie_modem_link_down(struct device *rc_dev)
{
	struct zumapro_pcie *zp = zumapro_pcie_from_dev(rc_dev);
	void __iomem *elbi;
	u32 val, mode;
	int ret;

	if (!zp)
		return -ENODEV;
	elbi = zp->pci.elbi_base;

	/*
	 * Keep the CP's PCIe refclk alive across the heavy bounce: the PMA lanes
	 * are still power-cycled (retrain works, unlike the cp_light path that
	 * cannot re-detect), but phy_power_off/on leave the external PLL and PHY
	 * block clocks up so the discrete CP never loses its reference clock --
	 * the suspected doorbell-decode killer (downstream phy_all_pwrdn keeps
	 * them up too; see research/prior-art/findings/A-rc-bounce.md #3).
	 * Cleared once the link is back up in modem_link_up().
	 */
	zumapro_pcie_phy_keep_refclk(zp->phy, true);

	/*
	 * Downstream s5100_poweroff_pcie() deasserts AP2CP_WAKEUP before
	 * dropping the link (mif_gpio_set_value(..., 0, 5): drop then settle
	 * 5 ms); the CP samples it to know the AP intends the link to be down.
	 * NULL-safe no-op when no CP lines exist (CH1).
	 */
	gpiod_set_value_cansleep(zp->cp_wakeup, 0);
	usleep_range(5000, 6000);

	/*
	 * Light down (v3): park the sub-block clock, drop LTSSM, assert PERST --
	 * but NO SOFT_PWR_RESET and NO PHY power-off, so the PHY PLL / refclk
	 * the CP's PCIe controller runs on stays locked across the bounce.  An
	 * LTSSM-off-only drop (no PERST) did not make the CP raise CP2AP_WAKEUP
	 * (hw), so BL1's handoff needs the PERST reset; keeping the PHY alive
	 * isolates whether the heavy path's PHY power cycle is what loses the
	 * doorbell decode.  link_up re-links x2/Gen3 without touching the PHY.
	 */
	if (zp->cp_light) {
		pr_err("BRK LD: light down (perst only, phy kept alive)\n");
		zumapro_pcie_phy_safe_clk(zp->phy, true);
		writel(0, elbi + PCIE_APP_LTSSM_ENABLE);
		gpiod_set_value_cansleep(zp->perst, 1);
		zp->cp_phy_off = false;
		usleep_range(1000, 2000);
		return 0;
	}

	/*
	 * PME_Turn_Off handshake while the link is still up, mirroring
	 * exynos_pcie_rc_send_pme_turn_off(): pulse the turn-off, take the
	 * PM_TO_ACK, then WAIT FOR THE LINK TO ENTER L2_IDLE before tearing
	 * down.  The golden trace reaches RDLH 0x15 (S_L2_IDLE) at t=265.99
	 * before the relink -- an orderly L2/L3 power-down.  The PBL
	 * disassembly (research/modem-pbl-re.md) proved BL1 never rebuilds its
	 * inbound doorbell decode after a link event, so that decode (in the
	 * PERST-reset 0x14E00000 controller) must survive the bounce; the
	 * working theory is it survives an orderly L2 entry but not a surprise
	 * PERST-at-L1.  An earlier build that polled for L2 saw only L0, but
	 * that was before D3hot + wakeup-low dropped the link to L1 pre-PME;
	 * from L1 the PME should now complete to L2.  ELBI accesses are safe
	 * here -- the link is up and the EP is still driving CLKREQ#.
	 */
	val = readl(elbi + PCIE_ELBI_RDLH_LINKUP) & LTSSM_STATE_MASK;
	if (val < LTSSM_STATE_RCVRY_LOCK || val > LTSSM_STATE_L1_IDLE) {
		dev_info(zp->pci.dev,
			 "link not up (ltssm %#x), skipping PME_Turn_Off\n", val);
	} else {
		/*
		 * PCIE_IRQ0 is write-1-to-clear (establish_link reads it and
		 * writes the value straight back); clear any stale PM_TO_ACK
		 * latch first so the poll below observes this handshake, and
		 * log the raw state so a pre-latched bit 29 (which would make
		 * the ack poll pass vacuously) is visible in the boot log.
		 */
		val = readl(elbi + PCIE_IRQ0);
		writel(val, elbi + PCIE_IRQ0);
		dev_info(zp->pci.dev,
			 "pre-PME ltssm %#x irq0 %#010x (%#010x after clear)\n",
			 readl(elbi + PCIE_ELBI_RDLH_LINKUP) & LTSSM_STATE_MASK,
			 val, readl(elbi + PCIE_IRQ0));

		writel(1, elbi + PCIE_APP_REQ_EXIT_L1);
		mode = readl(elbi + PCIE_APP_REQ_EXIT_L1_MODE);
		mode &= ~APP_REQ_EXIT_L1_MODE;
		mode |= L1_REQ_NAK_CTRL_MASTER;
		writel(mode, elbi + PCIE_APP_REQ_EXIT_L1_MODE);

		writel(1, elbi + PCIE_XMIT_PME_TURNOFF);
		ret = readl_poll_timeout(elbi + PCIE_IRQ0, val,
					 val & IRQ_RADM_PM_TO_ACK,
					 PCIE_L2_ENTER_WAIT_STEP_US,
					 PCIE_L2_ENTER_WAIT_US);
		if (ret)
			dev_warn(zp->pci.dev,
				 "no PM_TO_ACK from endpoint (irq0 %#x)\n", val);
		else
			dev_info(zp->pci.dev, "PM_TO_ACK (irq0 %#010x)\n", val);
		udelay(10);
		writel(0, elbi + PCIE_XMIT_PME_TURNOFF);

		/*
		 * Let the EP send Enter_L23_Ready and the link settle into
		 * L2_IDLE before PERST -- the orderly power-down the golden
		 * trace shows and the CP controller's inbound decode likely
		 * depends on.  Proceed on timeout like downstream, but log the
		 * achieved state: reaching 0x15 here is the whole point of this
		 * experiment.
		 */
		ret = readl_poll_timeout(elbi + PCIE_ELBI_RDLH_LINKUP, val,
					 (val & LTSSM_STATE_MASK) ==
					 LTSSM_STATE_L2_IDLE,
					 PCIE_L2_ENTER_WAIT_STEP_US,
					 PCIE_L2_ENTER_WAIT_US);
		if (ret)
			dev_warn(zp->pci.dev,
				 "link did not reach L2_IDLE before PERST (ltssm %#x)\n",
				 val & LTSSM_STATE_MASK);
		else
			dev_info(zp->pci.dev, "link reached L2_IDLE, orderly down\n");
	}

	/*
	 * Teardown in the order the downstream trace executes: assert PERST,
	 * switch the sub-block onto the OSC ("level clk switching for
	 * stability" -- with the link down the CP may stop driving CLKREQ#,
	 * and an ELBI access on the gated clock stalls the interconnect, seen
	 * on hardware when a cold CP's bootloader died during the bounce),
	 * disable the LTSSM, pulse SOFT_PWR_RESET, then power the PHY fully
	 * down (downstream phy_all_pwrdn).  The PHY-off matters to the CP:
	 * downstream's lanes are electrically dead for the whole
	 * CP2AP_WAKEUP wait, ours used to stay terminated until link_up.
	 */
	/*
	 * Snapshot the iMSI-RX enable/mask/target while the controller core is
	 * still live -- SOFT_PWR_RESET below zeroes them, and dw_pcie_setup_rc()
	 * on the way up does not re-arm the per-message ENABLE bits (the DWC MSI
	 * irq-chip only writes them from unmask, which already ran at probe).
	 * Without restoring these the CP's post-link-ack MSI (message 4) is
	 * silently dropped by the RC, indistinguishable from a dead doorbell.
	 */
	zp->saved_msi_enable = dw_pcie_readl_dbi(&zp->pci, PCIE_MSI_INTR0_ENABLE);
	zp->saved_msi_mask = dw_pcie_readl_dbi(&zp->pci, PCIE_MSI_INTR0_MASK);
	zp->saved_msi_addr_lo = dw_pcie_readl_dbi(&zp->pci, PCIE_MSI_ADDR_LO);
	zp->saved_msi_addr_hi = dw_pcie_readl_dbi(&zp->pci, PCIE_MSI_ADDR_HI);
	zp->msi_saved = true;
	dev_info(zp->pci.dev,
		 "iMSI-RX snapshot: en %#x mask %#x addr %#x:%#x\n",
		 zp->saved_msi_enable, zp->saved_msi_mask,
		 zp->saved_msi_addr_hi, zp->saved_msi_addr_lo);

	pr_err("BRK LD: perst assert\n");
	gpiod_set_value_cansleep(zp->perst, 1);
	zumapro_pcie_phy_safe_clk(zp->phy, true);
	pr_err("BRK LD: ltssm0\n");
	writel(0, elbi + PCIE_APP_LTSSM_ENABLE);
	writel(SOFT_RESET_PWR_PULSE, elbi + PCIE_SOFT_RESET);
	udelay(20);
	writel(SOFT_RESET_ALL, elbi + PCIE_SOFT_RESET);
	pr_err("BRK LD: phy off\n");
	phy_power_off(zp->phy);
	zp->cp_phy_off = true;
	usleep_range(1000, 2000);
	pr_err("BRK LD: done\n");

	return 0;
}
EXPORT_SYMBOL_GPL(zumapro_pcie_modem_link_down);

int zumapro_pcie_modem_link_up(struct device *rc_dev)
{
	struct zumapro_pcie *zp = zumapro_pcie_from_dev(rc_dev);
	void __iomem *elbi;
	int try, ret;

	if (!zp)
		return -ENODEV;
	elbi = zp->pci.elbi_base;

	/*
	 * Downstream s5100_poweron_pcie() asserts AP2CP_WAKEUP before every
	 * (re)train; the CP bootloader may gate its link participation on it.
	 * Left asserted afterwards -- the downstream steady-state while the
	 * link is up.
	 */
	gpiod_set_value_cansleep(zp->cp_wakeup, 1);

	/*
	 * Light re-link (v3): keep the PHY powered the whole time -- NO
	 * phy_power_off/on, NO PMA reset -- so the refclk the CP's PCIe
	 * controller runs on never drops.  The heavy path retrains fine but
	 * kills BL1's inbound doorbell decode; the two things it does that this
	 * skips are the PHY power cycle and SOFT_PWR_RESET.  A bare PERST with
	 * no controller re-config (v2) stalled training at S_PRE_DETECT_QUIET
	 * (rdlh 0x..05) because the post-PBL EP re-links at x2/Gen3 while the RC
	 * stayed at the mask-ROM x1 -- so give it the same width/speed, ELBI
	 * app-config and RC re-setup the heavy path does, minus the PHY
	 * teardown.  The cold-boot start_link retry (PERST cycle, no phy-off)
	 * proves a bare PERST retrains; if it trains here AND the doorbell then
	 * reaches BL1, the PHY power cycle was the decode killer -- drop it.
	 */
	if (zp->cp_light) {
		u16 lnksta;
		u8 cap;

		pr_err("BRK LU: light relink (perst cycle, phy kept alive)\n");
		zp->pci.num_lanes = 2;
		zp->pci.max_link_speed = 3;
		gpiod_set_value_cansleep(zp->perst, 0);
		usleep_range(PCIE_PERST_DELAY_US, PCIE_PERST_DELAY_US + 2000);
		zumapro_pcie_phy_safe_clk(zp->phy, false);
		zumapro_pcie_config_elbi(zp);
		dw_pcie_setup_rc(&zp->pci.pp);
		dw_pcie_writel_dbi(&zp->pci, 0x890, 0x12000);
		writel(LTSSM_ENABLE, elbi + PCIE_APP_LTSSM_ENABLE);
		usleep_range(5000, 6000);
		ret = zumapro_pcie_wait_link_up(&zp->pci);
		if (!ret) {
			usleep_range(2800, 3000);
			cap = dw_pcie_find_capability(&zp->pci, PCI_CAP_ID_EXP);
			lnksta = dw_pcie_readw_dbi(&zp->pci, cap + PCI_EXP_LNKSTA);
			dev_info(zp->pci.dev,
				 "light re-link up (Gen%u x%u), phy never power-cycled\n",
				 FIELD_GET(PCI_EXP_LNKSTA_CLS, lnksta),
				 FIELD_GET(PCI_EXP_LNKSTA_NLW, lnksta));
			writel(0, elbi + PCIE_APP_XFER_PENDING);
			return 0;
		}
		dev_warn(zp->pci.dev,
			 "light re-link failed (rdlh %#x), no heavy fallback in light mode\n",
			 readl(elbi + PCIE_ELBI_RDLH_LINKUP));
		zumapro_pcie_phy_safe_clk(zp->phy, true);
		return -ETIMEDOUT;
	}

	/*
	 * Downstream re-links with the full poweron sequence and its
	 * establish_link retry label re-runs PHY re-config, controller
	 * soft reset, ELBI re-config and RC re-setup before EVERY training
	 * attempt.  Retraining the half-dead core with a bare PERST cycle
	 * wedged the interconnect on the second attempt's LTSSM-state read
	 * (hw-observed), so mirror the heavyweight loop here.
	 *
	 * The post-PBL bootloader is a 2-lane endpoint (the mask ROM is x1)
	 * and downstream re-links this phase at x2 (s5100_poweron_pcie()
	 * width argument).  With the PHY terminating both lanes but the MAC
	 * in x1 mode, the EP waits for training sets on lane 1 forever --
	 * hw-observed as the LTSSM parking in Polling.Configuration (0x04).
	 * Drive both lanes for the re-link; the mask-ROM link keeps the
	 * board's x1 setting.
	 *
	 * Downstream also re-links this phase at GEN3 (poweron speed
	 * argument, LNKCTL2 TLS on the RC; the PHY config is speed-agnostic)
	 * and PERST-retries until the negotiated speed reaches the target,
	 * so the CP firmware only ever sees a Gen3 x2 link here -- it may
	 * gate the IPC start on it.  dw_pcie_setup_rc() arms the directed
	 * speed change, so training completes at Gen1 and upshifts.
	 */
	zp->pci.num_lanes = 2;
	zp->pci.max_link_speed = 3;
	for (try = 0; try < PCIE_LINK_TRAIN_RETRIES; try++) {
		gpiod_set_value_cansleep(zp->perst, 1);
		usleep_range(1000, 2000);

		pr_err("BRK LU: reinit %d\n", try + 1);
		/*
		 * modem_link_down already parked the sub-block clock and
		 * powered the PHY off before the CP2AP_WAKEUP wait; skip both
		 * then (safe_clk's debug reads touch PMA/PCS registers, which
		 * need the PHY clocks that phy_power_off disabled).
		 */
		if (!zp->cp_phy_off) {
			zumapro_pcie_phy_safe_clk(zp->phy, true);
			phy_power_off(zp->phy);
		}
		zp->cp_phy_off = false;
		zumapro_pcie_assert_phy_reset(zp);
		ret = phy_power_on(zp->phy);
		if (ret)
			return ret;
		phy_calibrate(zp->phy);
		zumapro_pcie_controller_reset_pulse(zp);
		zumapro_pcie_config_elbi(zp);
		zumapro_pcie_pmu_rmw(zp->pmu_phys, PCIE_PMU_WAKE_CTRL,
				     PCIE_PMU_WAKE_CTRL);

		pr_err("BRK LU: perst release\n");
		gpiod_set_value_cansleep(zp->perst, 0);
		usleep_range(PCIE_PERST_DELAY_US, PCIE_PERST_DELAY_US + 2000);

		pr_err("BRK LU: setup_rc\n");
		dw_pcie_setup_rc(&zp->pci.pp);

		/*
		 * Re-arm the iMSI-RX enable/mask/target snapshotted before the
		 * core reset.  dw_pcie_setup_rc() re-inits the MSI target from
		 * pp.msi_data but leaves ENABLE zeroed, so without this the RC
		 * drops the CP's post-link-ack MSI (message 4) and the boot
		 * stalls looking exactly like a doorbell that never landed.
		 */
		if (zp->msi_saved) {
			dev_info(zp->pci.dev,
				 "iMSI-RX post-setup_rc: en %#x mask %#x -> restoring en %#x mask %#x\n",
				 dw_pcie_readl_dbi(&zp->pci, PCIE_MSI_INTR0_ENABLE),
				 dw_pcie_readl_dbi(&zp->pci, PCIE_MSI_INTR0_MASK),
				 zp->saved_msi_enable, zp->saved_msi_mask);
			dw_pcie_writel_dbi(&zp->pci, PCIE_MSI_ADDR_LO,
					   zp->saved_msi_addr_lo);
			dw_pcie_writel_dbi(&zp->pci, PCIE_MSI_ADDR_HI,
					   zp->saved_msi_addr_hi);
			dw_pcie_writel_dbi(&zp->pci, PCIE_MSI_INTR0_ENABLE,
					   zp->saved_msi_enable);
			dw_pcie_writel_dbi(&zp->pci, PCIE_MSI_INTR0_MASK,
					   zp->saved_msi_mask);
		}

		/*
		 * Downstream writes GEN3_RELATED (0x890) = 0x12000 ("EQ Off")
		 * before every PERST release; Gen3 training on this PHY runs
		 * without equalization phases 2/3.
		 */
		dw_pcie_writel_dbi(&zp->pci, 0x890, 0x12000);

		pr_err("BRK LU: ltssm try %d\n", try + 1);
		writel(LTSSM_ENABLE, elbi + PCIE_APP_LTSSM_ENABLE);
		usleep_range(1000, 1500);
		pr_err("BRK LU: rdlh after 1ms %#x\n",
		       readl(elbi + PCIE_ELBI_RDLH_LINKUP));
		ret = zumapro_pcie_wait_link_up(&zp->pci);
		if (!ret) {
			u8 cap = dw_pcie_find_capability(&zp->pci,
							 PCI_CAP_ID_EXP);
			u16 lnksta;

			/* Downstream settles 3ms for the Gen1->Gen3 directed
			 * speed change before judging the result.
			 */
			usleep_range(2800, 3000);
			lnksta = dw_pcie_readw_dbi(&zp->pci,
						   cap + PCI_EXP_LNKSTA);
			dev_info(zp->pci.dev,
				 "bounce retrain succeeded on attempt %d (Gen%u x%u)\n",
				 try + 1,
				 FIELD_GET(PCI_EXP_LNKSTA_CLS, lnksta),
				 FIELD_GET(PCI_EXP_LNKSTA_NLW, lnksta));
			/*
			 * Below-target link: PERST-retry like downstream
			 * (up to the loop limit, then proceed with what
			 * trained -- downstream accepts after 10 tries too).
			 */
			if (FIELD_GET(PCI_EXP_LNKSTA_CLS, lnksta) < 3 &&
			    try < PCIE_LINK_TRAIN_RETRIES - 1) {
				dev_info(zp->pci.dev,
					 "link below target speed, retraining\n");
				continue;
			}
			writel(0, elbi + PCIE_APP_XFER_PENDING);
			zumapro_pcie_phy_keep_refclk(zp->phy, false);
			return 0;
		}
		dev_info(zp->pci.dev,
			 "bounce retrain attempt %d timed out (rdlh %#x)\n",
			 try + 1, readl(elbi + PCIE_ELBI_RDLH_LINKUP));
	}

	/* Dead link: park the sub-block clock so the RC stays touchable. */
	zumapro_pcie_phy_keep_refclk(zp->phy, false);
	zumapro_pcie_phy_safe_clk(zp->phy, true);
	return -ETIMEDOUT;
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
