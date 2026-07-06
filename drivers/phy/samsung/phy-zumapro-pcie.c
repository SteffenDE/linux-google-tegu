// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Zumapro (Tensor G4) PCIe PHY driver.
 *
 * Drives the PCIe PHYs (PMA + PCS) of both root complexes: the single-lane
 * GEN3A_1 instance on HSI2 (WiFi, pcie@13120000) and the two-lane PCIE_GEN3_0
 * / "SF_PCIEPHY_X2" instance on HSI1 (modem, pcie@12100000).  The register
 * sequences are ported verbatim from the downstream pcie-exynos-zuma-rc-cal.c
 * "ch_num == 1" (x1) and else (x2) paths; only the plain MMIO pokes are kept
 * (no cal-if/ACPM/SMC), and the controller-domain ELBI reset bracketing is
 * left to the root-complex driver, which calls phy_power_on() between
 * asserting and releasing the PMA reset and then phy_calibrate() to wait for
 * the PLL/CDR locks.
 */

#include <linux/arm-smccc.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-zumapro-pcie.h>
#include <linux/platform_device.h>

/*
 * PMU PCIE_PHY_CONTROL bit 0 bypasses PHY isolation (1 = PHY active).  On
 * Zumapro this register is firewalled to the secure world: a non-secure write
 * async-aborts (SError), so it must be programmed through the EL3
 * private-register SMC, matching the downstream rmw_priv_reg() path (and the
 * secure PMU writes the pmdomain-samsung driver already uses on this SoC).
 */
#define PCIE_PHY_CONTROL_BYPASS		BIT(0)
#define EXYNOS_SMC_CMD_PRIV_REG		0x82000504
#define EXYNOS_PRIV_REG_OPT_RMW		2

/* PMA */
#define PMA_PHY_INPUT_CLK		0x032c	/* 0x3 = ungate, 0x0 = gate */
#define PMA_PLL_LOCK			0x0a80	/* bit 0 */
#define PMA_CDR_LOCK			0x15c0	/* bit 4 */
#define PMA_OC_DONE			0x140c	/* bit 7 */
#define PMA_PLL_ENGAGED			0x0260	/* bit 2 */
#define PMA_PLL_LOCK_FORCE		0x0600	/* field [5:4] */
#define PMA_LANE_STRIDE			0x1000

/* udbg / external PLL */
#define PLL_EXT_CTRL			0xc700	/* x1: bit0 init; both: bit1 resetb */
#define PLL_EXT_CTRL_X2			0xc710	/* x2: bit1 init, bits[9:8] L2 gating */
#define PLL_EXT_LOCK			0xc734	/* x1: bit 2 */
#define PLL_EXT_LOCK_X2			0xc704	/* x2: bit 3 */
#define PLL_PWR_GATING			0xc800	/* power-gating / state monitor */

/* SoC block: PHY input-clock mux mode + select */
#define SOC_PHY_CLK_MODE		0x4000
#define SOC_PHY_CLK_SEL			0x4004

#define PCIE_PHY_LOCK_US		100000
#define PCIE_PHY_LOCK_STEP_US		10

#define PCIE_PHY_NUM_CLKS		3

enum zumapro_pcie_phy_variant {
	ZUMAPRO_PCIE_PHY_GEN3X1,	/* HSI2 GEN3A_1: WiFi, 1 lane */
	ZUMAPRO_PCIE_PHY_GEN3X2,	/* HSI1 PCIE_GEN3_0: modem, 2 lanes */
};

struct zumapro_pcie_phy_data {
	enum zumapro_pcie_phy_variant variant;
	unsigned int num_lanes;
};

struct zumapro_pcie_phy {
	struct device		*dev;
	const struct zumapro_pcie_phy_data *data;
	void __iomem		*pma;
	void __iomem		*pcs;
	void __iomem		*pll;
	void __iomem		*soc;
	struct clk_bulk_data	clks[PCIE_PHY_NUM_CLKS];
	phys_addr_t		pmu_phys;
	/*
	 * When set, power_off/on keep the external PLL and PHY block clocks
	 * running (the PMA lanes are still fully cycled).  A discrete endpoint
	 * that runs off this PHY's refclk -- the modem CP -- then keeps its PCIe
	 * reference clock across a link bounce.  See zumapro_pcie_phy_keep_refclk.
	 */
	bool			keep_refclk;
};

struct zumapro_pcie_phy_reg {
	u32 off;
	u32 val;
};

/* PMA common + lane/CDR common config (Phases C, D). Order matters: a few
 * offsets are written twice and the later value wins, matching downstream.
 */
static const struct zumapro_pcie_phy_reg pma_common_seq_x1[] = {
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

/*
 * The x2 instance's common block (downstream "else" branch) is the x1 list
 * plus an early 0x0510 = 0x81 (later overwritten to 0x80 in the lane-common
 * part, as downstream does).
 */
static const struct zumapro_pcie_phy_reg pma_common_seq_x2[] = {
	{ 0x0000, 0x88 }, { 0x001c, 0x66 }, { 0x01f4, 0x00 }, { 0x0510, 0x81 },
	{ 0x0514, 0x59 }, { 0x051c, 0x11 }, { 0x062c, 0x0e }, { 0x0644, 0x22 },
	{ 0x0688, 0x03 }, { 0x06d4, 0x28 }, { 0x0788, 0x64 }, { 0x078c, 0x64 },
	{ 0x0790, 0x50 }, { 0x0794, 0x50 }, { 0x0944, 0x05 }, { 0x0948, 0x05 },
	{ 0x094c, 0x05 }, { 0x0950, 0x05 }, { 0x0590, 0x02 }, { 0x07f8, 0xb0 },
	{ 0x0730, 0x08 }, { 0x0344, 0xc0 }, { 0x0040, 0x04 }, { 0x0204, 0x03 },
	{ 0x02d4, 0x1f }, { 0x0358, 0x10 }, { 0x0018, 0x01 },
	/* lane/CDR common */
	{ 0x0514, 0x5b }, { 0x0608, 0x0c }, { 0x060c, 0x0f }, { 0x0610, 0x0f },
	{ 0x0614, 0x0f }, { 0x0618, 0x0f }, { 0x0510, 0x80 }, { 0x0688, 0x03 },
	{ 0x0644, 0x23 }, { 0x0624, 0x11 }, { 0x0630, 0x0f }, { 0x06d0, 0x53 },
};

/* Per-lane block (Phase E), applied once per lane at +0x1000 stride. */
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
static const struct zumapro_pcie_phy_reg pcs_seq_x1[] = {
	{ 0x190, 0x100b0604 }, { 0x154, 0x000700d5 }, { 0x100, 0x16400000 },
	{ 0x104, 0x08600000 }, { 0x114, 0x18500000 }, { 0x124, 0x60700000 },
	{ 0x174, 0x00000007 }, { 0x178, 0x00000100 }, { 0x17c, 0x00000700 },
};

/* PCS config for the x2/modem instance: only 0x17c differs. */
static const struct zumapro_pcie_phy_reg pcs_seq_x2[] = {
	{ 0x190, 0x100b0604 }, { 0x154, 0x000700d5 }, { 0x100, 0x16400000 },
	{ 0x104, 0x08600000 }, { 0x114, 0x18500000 }, { 0x124, 0x60700000 },
	{ 0x174, 0x00000007 }, { 0x178, 0x00000100 }, { 0x17c, 0x00000010 },
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
	unsigned int lane;
	u32 val;

	if (phy->data->variant == ZUMAPRO_PCIE_PHY_GEN3X2) {
		/* Ungate the external PLL (x2 wiring uses 0xc700 bit 1). */
		writel(readl(phy->pll + PLL_EXT_CTRL) & ~BIT(1),
		       phy->pll + PLL_EXT_CTRL);
		udelay(100);

		/* Restore the forced-off PLL lock signal (see power_off). */
		val = readl(phy->pma + PMA_PLL_LOCK_FORCE);
		writel(val & ~(0x3 << 4), phy->pma + PMA_PLL_LOCK_FORCE);
	} else {
		writel(readl(phy->pll + PLL_EXT_CTRL) | BIT(0),
		       phy->pll + PLL_EXT_CTRL);
		udelay(100);
	}

	writel(0x00, phy->pma + 0x000c);
	writel(0x55, phy->pma + 0x0928);
	writel(0x02, phy->pma + 0x0580);
	writel(0x00, phy->pma + 0x0208);
	if (phy->data->variant == ZUMAPRO_PCIE_PHY_GEN3X2)
		writel(0x00, phy->pma + 0x0228);
	for (lane = 0; lane < phy->data->num_lanes; lane++) {
		void __iomem *base = phy->pma + lane * PMA_LANE_STRIDE;

		writel(0x00, base + 0x1044);
		writel(0x00, base + 0x1048);
		writel(0x00, base + 0x104c);
		writel(0x00, base + 0x1050);
		writel(0x00, base + 0x185c);
	}
	udelay(10);
}

/* Select the external PLL and wait for it to lock (Phase A). */
static int zumapro_pcie_phy_ext_pll(struct zumapro_pcie_phy *phy)
{
	u32 lock_off, lock_bit, val;
	int ret;

	writel(0x3, phy->pma + PMA_PHY_INPUT_CLK);

	if (phy->data->variant == ZUMAPRO_PCIE_PHY_GEN3X2) {
		/* External PLL initialization */
		writel(readl(phy->pll + PLL_EXT_CTRL_X2) & ~BIT(1),
		       phy->pll + PLL_EXT_CTRL_X2);
		lock_off = PLL_EXT_LOCK_X2;
		lock_bit = BIT(3);
	} else {
		writel(readl(phy->pll + PLL_EXT_CTRL) | BIT(0),
		       phy->pll + PLL_EXT_CTRL);
		lock_off = PLL_EXT_LOCK;
		lock_bit = BIT(2);
	}
	/* Override external PLL RESETB (common to both instances). */
	writel(readl(phy->pll + PLL_EXT_CTRL) & ~BIT(1),
	       phy->pll + PLL_EXT_CTRL);

	ret = readl_poll_timeout(phy->pll + lock_off, val, val & lock_bit,
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

/*
 * Switch the PCIe sub-block clock between HW mode (following the PHY/link,
 * value 0x15 as programmed at power-on) and SW mode on the always-on OSC.
 * Downstream flips to OSC before every intentional link drop ("level clk
 * switching for stability", exynos_pcie_rc_poweroff()): with the link dead
 * the endpoint may stop driving CLKREQ#, and an ELBI/DBI access on the
 * gated clock stalls the whole interconnect.
 */
void zumapro_pcie_phy_safe_clk(struct phy *p, bool safe)
{
	struct zumapro_pcie_phy *phy = phy_get_drvdata(p);
	u32 val;

	pr_err("BRK safe_clk(%d): rd soc+0x4000\n", safe);
	val = readl(phy->soc + SOC_PHY_CLK_MODE);
	pr_err("BRK safe_clk: got %#x, wr\n", val);
	if (safe)
		writel(val & ~0x3, phy->soc + SOC_PHY_CLK_MODE);
	else
		writel(0x15, phy->soc + SOC_PHY_CLK_MODE);
	pr_err("BRK safe_clk: done; pcs150 %#x engaged %#x extlock %#x\n",
	       readl(phy->pcs + 0x150), readl(phy->pma + PMA_PLL_ENGAGED),
	       readl(phy->pll + PLL_EXT_LOCK_X2));
}
EXPORT_SYMBOL_GPL(zumapro_pcie_phy_safe_clk);

/*
 * Keep the external PLL and PHY block clocks running across a power_off/on
 * cycle so a discrete endpoint that runs off this PHY's refclk (the modem CP)
 * does not lose its PCIe reference clock during a link bounce.  The PMA lanes
 * are still fully power-cycled, so RX re-detection works on retrain -- this
 * mirrors downstream phy_all_pwrdn, which powers the lanes down but leaves the
 * external PLL/refclk up (tegu's power_off additionally gates them).  The RC
 * driver arms this for the modem bounce window only.
 */
void zumapro_pcie_phy_keep_refclk(struct phy *p, bool keep)
{
	struct zumapro_pcie_phy *phy = phy_get_drvdata(p);

	phy->keep_refclk = keep;
}
EXPORT_SYMBOL_GPL(zumapro_pcie_phy_keep_refclk);

/* Release (active) or re-assert PHY isolation via the secure PMU RMW. */
static int zumapro_pcie_phy_set_isolation(struct zumapro_pcie_phy *phy, bool active)
{
	struct arm_smccc_res res;

	arm_smccc_smc(EXYNOS_SMC_CMD_PRIV_REG, phy->pmu_phys,
		      EXYNOS_PRIV_REG_OPT_RMW, PCIE_PHY_CONTROL_BYPASS,
		      active ? PCIE_PHY_CONTROL_BYPASS : 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(phy->dev, "secure PMU isolation write failed: %ld\n",
			res.a0);
		return -EIO;
	}

	return 0;
}

static int zumapro_pcie_phy_init(struct phy *p)
{
	struct zumapro_pcie_phy *phy = phy_get_drvdata(p);

	/* Release the PHY from PMU isolation (bypass = 1). */
	return zumapro_pcie_phy_set_isolation(phy, true);
}

static int zumapro_pcie_phy_power_on(struct phy *p)
{
	struct zumapro_pcie_phy *phy = phy_get_drvdata(p);
	const struct zumapro_pcie_phy_reg *common, *pcs;
	size_t n_common, n_pcs;
	unsigned int lane;
	int ret;

	/* In keep_refclk mode the block clocks were never gated (see power_off). */
	if (!phy->keep_refclk) {
		ret = clk_bulk_prepare_enable(PCIE_PHY_NUM_CLKS, phy->clks);
		if (ret)
			return ret;
	}

	/* Put the SoC PHY clock mux into SW mode before the PLL select. */
	writel(0x15, phy->soc + SOC_PHY_CLK_MODE);

	zumapro_pcie_phy_pwrdn_clear(phy);

	ret = zumapro_pcie_phy_ext_pll(phy);
	if (ret)
		goto err_clk;

	if (phy->data->variant == ZUMAPRO_PCIE_PHY_GEN3X2) {
		common = pma_common_seq_x2;
		n_common = ARRAY_SIZE(pma_common_seq_x2);
		pcs = pcs_seq_x2;
		n_pcs = ARRAY_SIZE(pcs_seq_x2);
	} else {
		common = pma_common_seq_x1;
		n_common = ARRAY_SIZE(pma_common_seq_x1);
		pcs = pcs_seq_x1;
		n_pcs = ARRAY_SIZE(pcs_seq_x1);
	}

	phy_write_seq(phy->pma, common, n_common);
	for (lane = 0; lane < phy->data->num_lanes; lane++)
		phy_write_seq(phy->pma + lane * PMA_LANE_STRIDE,
			      pma_lane_seq, ARRAY_SIZE(pma_lane_seq));

	/* WiFi-endpoint-specific PMA tweaks; the modem instance has none. */
	if (phy->data->variant == ZUMAPRO_PCIE_PHY_GEN3X1) {
		phy_write_seq(phy->pma, pma_bcm_seq, ARRAY_SIZE(pma_bcm_seq));
		writel(readl(phy->pma + 0x1350) | BIT(0), phy->pma + 0x1350);
	}

	phy_write_seq(phy->pcs, pcs, n_pcs);
	if (phy->data->variant == ZUMAPRO_PCIE_PHY_GEN3X1)
		writel(0x01600202, phy->pcs + 0x110);

	return 0;

err_clk:
	if (!phy->keep_refclk)
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

	/*
	 * Second-lane CDR/OC checks.  Downstream's lane loop leaves its
	 * pointer at lane 1 so its post-config polls read the +0x1000-shifted
	 * offsets; poll the same lane-1 registers here but only warn on
	 * timeout, so a first-boot GEN1 x1 link (which only uses lane 0) is
	 * not blocked on lane-1 calibration.
	 */
	if (phy->data->num_lanes > 1) {
		void __iomem *lane1 = phy->pma + PMA_LANE_STRIDE;

		ret = readl_poll_timeout(lane1 + PMA_CDR_LOCK, val,
					 val & BIT(4), PCIE_PHY_LOCK_STEP_US,
					 PCIE_PHY_LOCK_US);
		if (ret)
			dev_warn(phy->dev, "lane1 CDR failed to lock\n");
		ret = readl_poll_timeout(lane1 + PMA_OC_DONE, val,
					 val & BIT(7), PCIE_PHY_LOCK_STEP_US,
					 PCIE_PHY_LOCK_US);
		if (ret)
			dev_warn(phy->dev, "lane1 offset calibration did not complete\n");
	}

	if (phy->data->variant == ZUMAPRO_PCIE_PHY_GEN3X2) {
		/* Gate the external PLL clock when the link enters L2. */
		writel(readl(phy->pll + PLL_EXT_CTRL_X2) | (0x3 << 8),
		       phy->pll + PLL_EXT_CTRL_X2);
		/*
		 * Power-gating / state-monitor: downstream establish_link
		 * clears bits [6:5] for this channel instead of the x1
		 * instance's 0x421 value.
		 */
		writel(readl(phy->pll + PLL_PWR_GATING) & ~(0x3 << 5),
		       phy->pll + PLL_PWR_GATING);
		/*
		 * "PLL & BIAS always on": downstream pins this at the end of
		 * every CH0 poweron so the PLLs only gate under explicit
		 * control (pwrdn writes 0x300d9/0x300de).  Left at its
		 * default, the PCS auto-gates the PLL on L2/link-down; with a
		 * dead endpoint never re-asserting CLKREQ#, the gated pclk
		 * wedged the interconnect on the next ELBI read (hw-observed
		 * in the modem boot link bounce).
		 */
		writel(0x300d5, phy->pcs + 0x150);
	}

	writel(0x0, phy->pma + PMA_PHY_INPUT_CLK);

	/* External-PLL power-gating / state-monitor setting for GEN3A_1. */
	if (phy->data->variant == ZUMAPRO_PCIE_PHY_GEN3X1)
		writel(0x421, phy->pll + PLL_PWR_GATING);

	return 0;
}

static int zumapro_pcie_phy_power_off(struct phy *p)
{
	struct zumapro_pcie_phy *phy = phy_get_drvdata(p);
	unsigned int lane;
	u32 val;
	int cnt;

	writel(readl(phy->pma + 0x0204) & ~(0x3 << 2), phy->pma + 0x0204);

	if (phy->data->variant == ZUMAPRO_PCIE_PHY_GEN3X2) {
		/* PLL off, bias on */
		writel(0x300d9, phy->pcs + 0x150);
		udelay(5);
	}

	for (lane = 0; lane < phy->data->num_lanes; lane++) {
		void __iomem *base = phy->pma + lane * PMA_LANE_STRIDE;

		writel(0x2a, base + 0x1044);
		writel(0xaa, base + 0x1048);
		writel(0xa8, base + 0x104c);
		writel(0x80, base + 0x1050);
		writel(0x0a, base + 0x185c);
		udelay(1);
	}

	writel(0xff, phy->pma + 0x0208);
	if (phy->data->variant == ZUMAPRO_PCIE_PHY_GEN3X2)
		writel(0xff, phy->pma + 0x0228);
	udelay(1);
	writel(0x0a, phy->pma + 0x0580);
	writel(0xaa, phy->pma + 0x0928);
	writel(0x0a, phy->pma + 0x000c);

	if (phy->data->variant == ZUMAPRO_PCIE_PHY_GEN3X2) {
		/*
		 * Force the PLL lock signal off (field [5:4] = 2; downstream
		 * computes the same value through a dead masked-clear).
		 */
		val = readl(phy->pma + PMA_PLL_LOCK_FORCE);
		val = (val & ~(0x3 << 4)) | (0x2 << 4);
		writel(val, phy->pma + PMA_PLL_LOCK_FORCE);

		/* PLL off, bias off */
		writel(0x300de, phy->pcs + 0x150);
		udelay(5);

		/* Wait for the PMA PLL to disengage before ext-PLL gating. */
		for (cnt = 0; cnt < 100; cnt++) {
			if (!(readl(phy->pma + PMA_PLL_ENGAGED) & BIT(2)))
				break;
			udelay(1);
		}
		if (cnt == 100) {
			dev_warn(phy->dev,
				 "PMA PLL did not disengage, leaving external PLL on\n");
			goto out_clk;
		}

		/*
		 * Gate the external PLL (x2 wiring uses 0xc700 bit 1) -- skipped
		 * in keep_refclk mode so the modem CP's refclk survives the
		 * bounce.  The PMA lanes above are still powered down regardless.
		 */
		if (!phy->keep_refclk) {
			writel(readl(phy->pll + PLL_EXT_CTRL) | BIT(1),
			       phy->pll + PLL_EXT_CTRL);
			udelay(10);
		}
	} else {
		udelay(50);
		writel(readl(phy->pll + PLL_EXT_CTRL) & ~BIT(0),
		       phy->pll + PLL_EXT_CTRL);
		udelay(10);
	}

out_clk:
	if (!phy->keep_refclk)
		clk_bulk_disable_unprepare(PCIE_PHY_NUM_CLKS, phy->clks);

	return 0;
}

static int zumapro_pcie_phy_exit(struct phy *p)
{
	struct zumapro_pcie_phy *phy = phy_get_drvdata(p);

	/* Put the PHY back into PMU isolation. */
	return zumapro_pcie_phy_set_isolation(phy, false);
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
	struct of_phandle_args pmu_args;
	struct resource pmu_res;
	int i, ret;

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	phy->dev = dev;
	phy->data = of_device_get_match_data(dev);
	if (!phy->data)
		return -EINVAL;

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

	ret = of_parse_phandle_with_fixed_args(dev->of_node, "samsung,pmu-syscon",
					       1, 0, &pmu_args);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get PMU syscon\n");
	ret = of_address_to_resource(pmu_args.np, 0, &pmu_res);
	of_node_put(pmu_args.np);
	if (ret)
		return dev_err_probe(dev, ret, "failed to resolve PMU base\n");
	phy->pmu_phys = pmu_res.start + pmu_args.args[0];

	generic_phy = devm_phy_create(dev, NULL, &zumapro_pcie_phy_ops);
	if (IS_ERR(generic_phy))
		return dev_err_probe(dev, PTR_ERR(generic_phy),
				     "failed to create PHY\n");

	phy_set_drvdata(generic_phy, phy);
	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static const struct zumapro_pcie_phy_data zumapro_pcie_phy_x1_data = {
	.variant	= ZUMAPRO_PCIE_PHY_GEN3X1,
	.num_lanes	= 1,
};

static const struct zumapro_pcie_phy_data zumapro_pcie_phy_x2_data = {
	.variant	= ZUMAPRO_PCIE_PHY_GEN3X2,
	.num_lanes	= 2,
};

static const struct of_device_id zumapro_pcie_phy_of_match[] = {
	{
		.compatible = "google,zumapro-pcie-phy",
		.data = &zumapro_pcie_phy_x1_data,
	}, {
		.compatible = "google,zumapro-pcie-x2-phy",
		.data = &zumapro_pcie_phy_x2_data,
	},
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
