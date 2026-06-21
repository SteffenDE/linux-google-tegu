// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Zumapro (Tensor G4) PCIe PHY driver.
 *
 * Drives the GEN3A_1 PCIe PHY (PMA + PCS) used by the on-board WiFi root
 * complex.  The register sequences are ported verbatim from the downstream
 * pcie-exynos-zuma-rc-cal.c "ch_num == 1" / EP_BCM_WIFI path; only the plain
 * MMIO pokes are kept (no cal-if/ACPM/SMC), and the controller-domain ELBI
 * reset bracketing is left to the root-complex driver, which calls
 * phy_power_on() between asserting and releasing the PMA reset and then
 * phy_calibrate() to wait for the PLL/CDR locks.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/* PMU PCIE_PHY_CONTROL: bit 0 == bypass isolation (1 = PHY active). */
#define PCIE_PHY_CONTROL_BYPASS		BIT(0)

/* PMA */
#define PMA_PHY_INPUT_CLK		0x032c	/* 0x3 = ungate, 0x0 = gate */
#define PMA_PLL_LOCK			0x0a80	/* bit 0 */
#define PMA_CDR_LOCK			0x15c0	/* bit 4 */
#define PMA_OC_DONE			0x140c	/* bit 7 */

/* udbg / external PLL */
#define PLL_EXT_CTRL			0xc700	/* bit0 init, bit1 resetb override */
#define PLL_EXT_LOCK			0xc734	/* bit 2 */
#define PLL_PWR_GATING			0xc800	/* power-gating / state monitor */

/* HSI2 SoC block: PHY input-clock mux mode + select */
#define SOC_PHY_CLK_MODE		0x4000
#define SOC_PHY_CLK_SEL			0x4004

#define PCIE_PHY_LOCK_US		100000
#define PCIE_PHY_LOCK_STEP_US		10

#define PCIE_PHY_NUM_CLKS		3

struct zumapro_pcie_phy {
	struct device		*dev;
	void __iomem		*pma;
	void __iomem		*pcs;
	void __iomem		*pll;
	void __iomem		*soc;
	struct clk_bulk_data	clks[PCIE_PHY_NUM_CLKS];
	struct regmap		*pmu;
	unsigned int		pmu_offset;
};

struct zumapro_pcie_phy_reg {
	u32 off;
	u32 val;
};

/* PMA common + lane/CDR common config (Phases C, D). Order matters: a few
 * offsets are written twice and the later value wins, matching downstream.
 */
static const struct zumapro_pcie_phy_reg pma_common_seq[] = {
	{ 0x0000, 0x88 }, { 0x001c, 0x66 }, { 0x01f4, 0x00 }, { 0x0514, 0x59 },
	{ 0x051c, 0x11 }, { 0x062c, 0x0e }, { 0x0644, 0x22 }, { 0x0688, 0x03 },
	{ 0x06d4, 0x28 }, { 0x0788, 0x64 }, { 0x078c, 0x64 }, { 0x0790, 0x50 },
	{ 0x0794, 0x50 }, { 0x0944, 0x05 }, { 0x0948, 0x05 }, { 0x094c, 0x05 },
	{ 0x0950, 0x05 }, { 0x0590, 0x02 }, { 0x07f8, 0xb0 }, { 0x0730, 0x08 },
	{ 0x0344, 0xc0 }, { 0x0040, 0x04 }, { 0x0204, 0x03 }, { 0x02d4, 0x1f },
	{ 0x0358, 0x10 }, { 0x0018, 0x01 },
	/* lane/CDR common */
	{ 0x0514, 0x5b }, { 0x0608, 0x0c }, { 0x060c, 0x0f }, { 0x0610, 0x0f },
	{ 0x0614, 0x0f }, { 0x0618, 0x0f }, { 0x0510, 0x80 }, { 0x0688, 0x03 },
	{ 0x0644, 0x23 }, { 0x0624, 0x11 }, { 0x0630, 0x0f }, { 0x06d0, 0x53 },
};

/* Per-lane block (Phase E). tegu is single-lane, so applied once at lane 0. */
static const struct zumapro_pcie_phy_reg pma_lane_seq[] = {
	{ 0x1140, 0x04 }, { 0x1144, 0x04 }, { 0x1148, 0x04 }, { 0x114c, 0x02 },
	{ 0x1150, 0x00 }, { 0x1154, 0x00 }, { 0x1158, 0x00 }, { 0x115c, 0x00 },
	{ 0x12cc, 0x1c }, { 0x12dc, 0x6c }, { 0x130c, 0x29 }, { 0x13b4, 0x2f },
	{ 0x1a64, 0x05 }, { 0x1a68, 0x05 }, { 0x1a84, 0x05 }, { 0x1a88, 0x05 },
	{ 0x1a98, 0x00 }, { 0x1a9c, 0x00 }, { 0x1aa8, 0x07 }, { 0x1ab8, 0x00 },
	{ 0x1abc, 0x00 }, { 0x1af8, 0x90 }, { 0x1b34, 0x03 }, { 0x1bb0, 0x03 },
	{ 0x1bb4, 0x03 }, { 0x1bc0, 0x06 }, { 0x1bc4, 0x06 }, { 0x1be8, 0x01 },
	{ 0x1bf8, 0x04 }, { 0x1c98, 0x00 }, { 0x1ca4, 0x81 },
};

/* BCM4383-specific PMA tweaks (Phase F): L1SS exit link-down workaround. */
static const struct zumapro_pcie_phy_reg pma_bcm_seq[] = {
	{ 0x1b20, 0x02 }, { 0x1340, 0x23 }, { 0x002c, 0x3c },
};

/* PCS config (Phase H), with the BCM4383 0x17c value (Phase I adds pcs+0x110). */
static const struct zumapro_pcie_phy_reg pcs_seq[] = {
	{ 0x190, 0x100b0604 }, { 0x154, 0x000700d5 }, { 0x100, 0x16400000 },
	{ 0x104, 0x08600000 }, { 0x114, 0x18500000 }, { 0x124, 0x60700000 },
	{ 0x174, 0x00000007 }, { 0x178, 0x00000100 }, { 0x17c, 0x00000700 },
};

static void phy_write_seq(void __iomem *base,
			  const struct zumapro_pcie_phy_reg *seq, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		writel(seq[i].val, base + seq[i].off);
}

/* Bring the PMA out of the all-power-down state (downstream phy_all_pwrdn_clear). */
static void zumapro_pcie_phy_pwrdn_clear(struct zumapro_pcie_phy *phy)
{
	writel(readl(phy->pll + PLL_EXT_CTRL) | BIT(0), phy->pll + PLL_EXT_CTRL);
	udelay(100);
	writel(0x00, phy->pma + 0x000c);
	writel(0x55, phy->pma + 0x0928);
	writel(0x02, phy->pma + 0x0580);
	writel(0x00, phy->pma + 0x0208);
	writel(0x00, phy->pma + 0x1044);
	writel(0x00, phy->pma + 0x1048);
	writel(0x00, phy->pma + 0x104c);
	writel(0x00, phy->pma + 0x1050);
	writel(0x00, phy->pma + 0x185c);
	udelay(10);
}

/* Select the external PLL and wait for it to lock (Phase A). */
static int zumapro_pcie_phy_ext_pll(struct zumapro_pcie_phy *phy)
{
	u32 val;
	int ret;

	writel(0x3, phy->pma + PMA_PHY_INPUT_CLK);
	writel(readl(phy->pll + PLL_EXT_CTRL) | BIT(0), phy->pll + PLL_EXT_CTRL);
	writel(readl(phy->pll + PLL_EXT_CTRL) & ~BIT(1), phy->pll + PLL_EXT_CTRL);

	ret = readl_poll_timeout(phy->pll + PLL_EXT_LOCK, val, val & BIT(2),
				 PCIE_PHY_LOCK_STEP_US, PCIE_PHY_LOCK_US);
	if (ret) {
		dev_err(phy->dev, "external PLL failed to lock\n");
		return ret;
	}

	writel(readl(phy->soc + SOC_PHY_CLK_SEL) & ~BIT(2),
	       phy->soc + SOC_PHY_CLK_SEL);
	writel(readl(phy->soc + SOC_PHY_CLK_SEL) | (0x3 << 4),
	       phy->soc + SOC_PHY_CLK_SEL);

	return 0;
}

static int zumapro_pcie_phy_init(struct phy *p)
{
	struct zumapro_pcie_phy *phy = phy_get_drvdata(p);

	/* Release the PHY from PMU isolation (bypass = 1). */
	return regmap_update_bits(phy->pmu, phy->pmu_offset,
				  PCIE_PHY_CONTROL_BYPASS, PCIE_PHY_CONTROL_BYPASS);
}

static int zumapro_pcie_phy_power_on(struct phy *p)
{
	struct zumapro_pcie_phy *phy = phy_get_drvdata(p);
	int ret;

	ret = clk_bulk_prepare_enable(PCIE_PHY_NUM_CLKS, phy->clks);
	if (ret)
		return ret;

	/* Put the SoC PHY clock mux into SW mode before the PLL select. */
	writel(0x15, phy->soc + SOC_PHY_CLK_MODE);

	zumapro_pcie_phy_pwrdn_clear(phy);

	ret = zumapro_pcie_phy_ext_pll(phy);
	if (ret)
		goto err_clk;

	phy_write_seq(phy->pma, pma_common_seq, ARRAY_SIZE(pma_common_seq));
	phy_write_seq(phy->pma, pma_lane_seq, ARRAY_SIZE(pma_lane_seq));
	phy_write_seq(phy->pma, pma_bcm_seq, ARRAY_SIZE(pma_bcm_seq));
	writel(readl(phy->pma + 0x1350) | BIT(0), phy->pma + 0x1350);

	phy_write_seq(phy->pcs, pcs_seq, ARRAY_SIZE(pcs_seq));
	writel(0x01600202, phy->pcs + 0x110);

	return 0;

err_clk:
	clk_bulk_disable_unprepare(PCIE_PHY_NUM_CLKS, phy->clks);
	return ret;
}

/*
 * Called by the RC driver after it has released the PMA reset: wait for the
 * PLL, CDR and offset-calibration locks, then re-gate the PHY input clock.
 */
static int zumapro_pcie_phy_calibrate(struct phy *p)
{
	struct zumapro_pcie_phy *phy = phy_get_drvdata(p);
	u32 val;
	int ret;

	ret = readl_poll_timeout(phy->pma + PMA_PLL_LOCK, val, val & BIT(0),
				 PCIE_PHY_LOCK_STEP_US, PCIE_PHY_LOCK_US);
	if (ret) {
		dev_err(phy->dev, "PMA PLL failed to lock\n");
		return ret;
	}

	ret = readl_poll_timeout(phy->pma + PMA_CDR_LOCK, val, val & BIT(4),
				 PCIE_PHY_LOCK_STEP_US, PCIE_PHY_LOCK_US);
	if (ret) {
		dev_err(phy->dev, "PMA CDR failed to lock\n");
		return ret;
	}

	ret = readl_poll_timeout(phy->pma + PMA_OC_DONE, val, val & BIT(7),
				 PCIE_PHY_LOCK_STEP_US, PCIE_PHY_LOCK_US);
	if (ret) {
		dev_err(phy->dev, "PMA offset calibration did not complete\n");
		return ret;
	}

	writel(0x0, phy->pma + PMA_PHY_INPUT_CLK);

	/* External-PLL power-gating / state-monitor setting for GEN3A_1. */
	writel(0x421, phy->pll + PLL_PWR_GATING);

	return 0;
}

static int zumapro_pcie_phy_power_off(struct phy *p)
{
	struct zumapro_pcie_phy *phy = phy_get_drvdata(p);

	writel(readl(phy->pma + 0x0204) & ~(0x3 << 2), phy->pma + 0x0204);
	writel(0x2a, phy->pma + 0x1044);
	writel(0xaa, phy->pma + 0x1048);
	writel(0xa8, phy->pma + 0x104c);
	writel(0x80, phy->pma + 0x1050);
	writel(0x0a, phy->pma + 0x185c);
	udelay(1);
	writel(0xff, phy->pma + 0x0208);
	udelay(1);
	writel(0x0a, phy->pma + 0x0580);
	writel(0xaa, phy->pma + 0x0928);
	writel(0x0a, phy->pma + 0x000c);
	udelay(50);
	writel(readl(phy->pll + PLL_EXT_CTRL) & ~BIT(0), phy->pll + PLL_EXT_CTRL);
	udelay(10);

	clk_bulk_disable_unprepare(PCIE_PHY_NUM_CLKS, phy->clks);

	return 0;
}

static int zumapro_pcie_phy_exit(struct phy *p)
{
	struct zumapro_pcie_phy *phy = phy_get_drvdata(p);

	/* Put the PHY back into PMU isolation. */
	return regmap_update_bits(phy->pmu, phy->pmu_offset,
				  PCIE_PHY_CONTROL_BYPASS, 0);
}

static const struct phy_ops zumapro_pcie_phy_ops = {
	.init		= zumapro_pcie_phy_init,
	.exit		= zumapro_pcie_phy_exit,
	.power_on	= zumapro_pcie_phy_power_on,
	.power_off	= zumapro_pcie_phy_power_off,
	.calibrate	= zumapro_pcie_phy_calibrate,
	.owner		= THIS_MODULE,
};

static int zumapro_pcie_phy_probe(struct platform_device *pdev)
{
	static const char * const clk_names[PCIE_PHY_NUM_CLKS] = {
		"ref", "apb", "udbg",
	};
	struct device *dev = &pdev->dev;
	struct zumapro_pcie_phy *phy;
	struct phy_provider *provider;
	struct phy *generic_phy;
	unsigned int args[1];
	int i, ret;

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	phy->dev = dev;

	phy->pma = devm_platform_ioremap_resource_byname(pdev, "pma");
	if (IS_ERR(phy->pma))
		return PTR_ERR(phy->pma);
	phy->pcs = devm_platform_ioremap_resource_byname(pdev, "pcs");
	if (IS_ERR(phy->pcs))
		return PTR_ERR(phy->pcs);
	phy->pll = devm_platform_ioremap_resource_byname(pdev, "pll");
	if (IS_ERR(phy->pll))
		return PTR_ERR(phy->pll);
	phy->soc = devm_platform_ioremap_resource_byname(pdev, "soc");
	if (IS_ERR(phy->soc))
		return PTR_ERR(phy->soc);

	for (i = 0; i < PCIE_PHY_NUM_CLKS; i++)
		phy->clks[i].id = clk_names[i];
	ret = devm_clk_bulk_get(dev, PCIE_PHY_NUM_CLKS, phy->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");

	phy->pmu = syscon_regmap_lookup_by_phandle_args(dev->of_node,
							"samsung,pmu-syscon",
							1, args);
	if (IS_ERR(phy->pmu))
		return dev_err_probe(dev, PTR_ERR(phy->pmu),
				     "failed to get PMU syscon\n");
	phy->pmu_offset = args[0];

	generic_phy = devm_phy_create(dev, NULL, &zumapro_pcie_phy_ops);
	if (IS_ERR(generic_phy))
		return dev_err_probe(dev, PTR_ERR(generic_phy),
				     "failed to create PHY\n");

	phy_set_drvdata(generic_phy, phy);
	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id zumapro_pcie_phy_of_match[] = {
	{ .compatible = "google,zumapro-pcie-phy" },
	{ },
};
MODULE_DEVICE_TABLE(of, zumapro_pcie_phy_of_match);

static struct platform_driver zumapro_pcie_phy_driver = {
	.probe	= zumapro_pcie_phy_probe,
	.driver = {
		.name			= "zumapro-pcie-phy",
		.of_match_table		= zumapro_pcie_phy_of_match,
		.suppress_bind_attrs	= true,
	},
};
module_platform_driver(zumapro_pcie_phy_driver);

MODULE_DESCRIPTION("Google Zumapro PCIe PHY driver");
MODULE_LICENSE("GPL");
