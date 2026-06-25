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
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#include "pcie-designware.h"

#define to_zumapro_pcie(x)	dev_get_drvdata((x)->dev)

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
};

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
		if (!ret)
			return 0;

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

static const struct dw_pcie_host_ops zumapro_pcie_host_ops = {
	.init	= zumapro_pcie_host_init,
};

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
