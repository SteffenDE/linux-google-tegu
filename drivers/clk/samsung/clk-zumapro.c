// SPDX-License-Identifier: GPL-2.0-only
/*
 * Common Clock Framework support for Google Tensor G4 "Zumapro".
 *
 * This driver is intentionally scoped to the peripheral clocks used during
 * early bring-up. Register offsets come from the downstream Zuma
 * cmucal-sfr.c/cmucal-node.c tables.
 */

#include <linux/clk-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/google,zumapro.h>

#include "clk.h"
#include "clk-exynos-arm64.h"

/* NOTE: Must be equal to the last clock ID in each CMU increased by one. */
#define CLKS_NR_TOP		(CLK_DOUT_CMU_HSI1_PCIE + 1)
#define CLKS_NR_PERIC0		(CLK_GOUT_PERIC0_USI6_USI_CLK + 1)
#define CLKS_NR_PERIC1		(CLK_GOUT_PERIC1_USI9_USI_CLK + 1)
#define CLKS_NR_HSI2		(CLK_GOUT_HSI2_GPIO_HSI2_QCH + 1)
#define CLKS_NR_HSI1		(CLK_GOUT_HSI1_PCIE_GEN3_0_PIPE_PAL_APB_PCLK + 1)
#define CLKS_NR_HSI0		(CLK_GOUT_HSI0_USI4_USI_PCLK + 1)
#define CLKS_NR_DPUB		(CLK_GOUT_DPUB_DSIM0_OSCCLK + 1)
#define CLKS_NR_DPUF0		(CLK_GOUT_DPUF0_SRAMC_ACLK + 1)
#define CLKS_NR_DPUF1		(CLK_GOUT_DPUF1_SRAMC_ACLK + 1)
#define CLKS_NR_MISC		(CLK_GOUT_MISC_MCT_PCLK + 1)

/*
 * Gate "debug" register window offset. In automatic clock mode the framework
 * reads each gate's auto-clock-gating status from CLK_CON_GAT + this offset.
 * Same value as gs101/gs201 (GS101_GATE_DBG_OFFSET); the Tensor CMU IP is
 * shared across the family.
 */
#define ZUMAPRO_GATE_DBG_OFFSET			0x4000

/*
 * BUS_COMPONENT_DRCG_EN offset in a leaf CMU's SYSREG. Writing all-ones enables
 * dynamic root clock gating of the block's NOC/bus components so the fabric can
 * report idle and the domain can be quiesced for ACPM power-down. Same value as
 * gs101 (GS101_DRCG_EN_OFFSET) and the downstream Zuma cmucal save/restore set.
 */
#define ZUMAPRO_DRCG_EN_OFFSET			0x104

/* ---- CMU_TOP ------------------------------------------------------------ */

/* Register offsets for CMU_TOP (0x26040000) */
#define CLK_CON_CMU_TOP_CONTROLLER_OPTION	0x0800
#define CLK_CON_MUX_MUX_CLKCMU_HSI1_NOC		0x10a4
#define CLK_CON_MUX_MUX_CLKCMU_HSI1_PCIE	0x10a8
#define CLK_CON_MUX_MUX_CLKCMU_HSI2_MMC_CARD	0x10ac
#define CLK_CON_MUX_MUX_CLKCMU_HSI2_NOC		0x10b0
#define CLK_CON_MUX_MUX_CLKCMU_HSI2_PCIE	0x10b4
#define CLK_CON_MUX_MUX_CLKCMU_HSI2_UFS_EMBD	0x10b8
#define CLK_CON_MUX_MUX_CLKCMU_PERIC0_IP	0x10ec
#define CLK_CON_MUX_MUX_CLKCMU_PERIC0_NOC	0x10f0
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_IP	0x10f4
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_NOC	0x10f8
#define CLK_CON_DIV_CLKCMU_HSI1_NOC		0x189c
#define CLK_CON_DIV_CLKCMU_HSI1_PCIE		0x18a0
#define CLK_CON_DIV_CLKCMU_HSI2_MMC_CARD	0x18a4
#define CLK_CON_DIV_CLKCMU_HSI2_NOC		0x18a8
#define CLK_CON_DIV_CLKCMU_HSI2_PCIE		0x18ac
#define CLK_CON_DIV_CLKCMU_HSI2_UFS_EMBD	0x18b0
#define CLK_CON_DIV_CLKCMU_PERIC0_IP		0x18e4
#define CLK_CON_DIV_CLKCMU_PERIC0_NOC		0x18e8
#define CLK_CON_DIV_CLKCMU_PERIC1_IP		0x18ec
#define CLK_CON_DIV_CLKCMU_PERIC1_NOC		0x18f0
#define CLK_CON_GAT_GATE_CLKCMU_HSI1_NOC	0x20cc
#define CLK_CON_GAT_GATE_CLKCMU_HSI1_PCIE	0x20d0
#define CLK_CON_GAT_GATE_CLKCMU_HSI2_MMC_CARD	0x20d4
#define CLK_CON_GAT_GATE_CLKCMU_HSI2_NOC	0x20d8
#define CLK_CON_GAT_GATE_CLKCMU_HSI2_PCIE	0x20dc
#define CLK_CON_GAT_GATE_CLKCMU_HSI2_UFS_EMBD	0x20e0
#define CLK_CON_GAT_GATE_CLKCMU_PERIC0_IP	0x2110
#define CLK_CON_GAT_GATE_CLKCMU_PERIC0_NOC	0x2114
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_IP	0x2118
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_NOC	0x211c
#define CLK_CON_MUX_MUX_CLKCMU_HSI0_NOC		0x1094
#define CLK_CON_MUX_MUX_CLKCMU_HSI0_PERI	0x1098
#define CLK_CON_DIV_CLKCMU_HSI0_NOC		0x188c
#define CLK_CON_DIV_CLKCMU_HSI0_PERI		0x1890
#define CLK_CON_GAT_GATE_CLKCMU_HSI0_NOC	0x20bc
#define CLK_CON_GAT_GATE_CLKCMU_HSI0_PERI	0x20c0
#define CLK_CON_MUX_MUX_CLKCMU_MISC_NOC		0x10d0
#define CLK_CON_DIV_CLKCMU_MISC_NOC		0x18c4
#define CLK_CON_GAT_GATE_CLKCMU_MISC_NOC	0x20f4
#define CLK_CON_MUX_MUX_CLKCMU_DPUB_DSIM	0x1050
#define CLK_CON_MUX_MUX_CLKCMU_DPUB_NOC		0x1054
#define CLK_CON_MUX_MUX_CLKCMU_DPUF0_NOC	0x1058
#define CLK_CON_MUX_MUX_CLKCMU_DPUF1_NOC	0x105c
#define CLK_CON_DIV_CLKCMU_DPUB_DSIM		0x1848
#define CLK_CON_DIV_CLKCMU_DPUB_NOC		0x184c
#define CLK_CON_DIV_CLKCMU_DPUF0_NOC		0x1850
#define CLK_CON_DIV_CLKCMU_DPUF1_NOC		0x1854
#define CLK_CON_GAT_GATE_CLKCMU_DPUB_DSIM	0x2078
#define CLK_CON_GAT_GATE_CLKCMU_DPUB_NOC	0x207c
#define CLK_CON_GAT_GATE_CLKCMU_DPUF0_NOC	0x2080
#define CLK_CON_GAT_GATE_CLKCMU_DPUF1_NOC	0x2084

static const unsigned long top_clk_regs[] __initconst = {
	CLK_CON_CMU_TOP_CONTROLLER_OPTION,
	CLK_CON_MUX_MUX_CLKCMU_DPUB_DSIM,
	CLK_CON_MUX_MUX_CLKCMU_DPUB_NOC,
	CLK_CON_MUX_MUX_CLKCMU_DPUF0_NOC,
	CLK_CON_MUX_MUX_CLKCMU_DPUF1_NOC,
	CLK_CON_MUX_MUX_CLKCMU_HSI1_NOC,
	CLK_CON_MUX_MUX_CLKCMU_HSI1_PCIE,
	CLK_CON_MUX_MUX_CLKCMU_HSI2_MMC_CARD,
	CLK_CON_MUX_MUX_CLKCMU_HSI2_NOC,
	CLK_CON_MUX_MUX_CLKCMU_HSI2_PCIE,
	CLK_CON_MUX_MUX_CLKCMU_HSI2_UFS_EMBD,
	CLK_CON_MUX_MUX_CLKCMU_PERIC0_IP,
	CLK_CON_MUX_MUX_CLKCMU_PERIC0_NOC,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_IP,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_NOC,
	CLK_CON_DIV_CLKCMU_HSI1_NOC,
	CLK_CON_DIV_CLKCMU_HSI1_PCIE,
	CLK_CON_DIV_CLKCMU_HSI2_MMC_CARD,
	CLK_CON_DIV_CLKCMU_HSI2_NOC,
	CLK_CON_DIV_CLKCMU_HSI2_PCIE,
	CLK_CON_DIV_CLKCMU_HSI2_UFS_EMBD,
	CLK_CON_DIV_CLKCMU_PERIC0_IP,
	CLK_CON_DIV_CLKCMU_PERIC0_NOC,
	CLK_CON_DIV_CLKCMU_PERIC1_IP,
	CLK_CON_DIV_CLKCMU_PERIC1_NOC,
	CLK_CON_DIV_CLKCMU_DPUB_DSIM,
	CLK_CON_DIV_CLKCMU_DPUB_NOC,
	CLK_CON_DIV_CLKCMU_DPUF0_NOC,
	CLK_CON_DIV_CLKCMU_DPUF1_NOC,
	CLK_CON_GAT_GATE_CLKCMU_DPUB_DSIM,
	CLK_CON_GAT_GATE_CLKCMU_DPUB_NOC,
	CLK_CON_GAT_GATE_CLKCMU_DPUF0_NOC,
	CLK_CON_GAT_GATE_CLKCMU_DPUF1_NOC,
	CLK_CON_GAT_GATE_CLKCMU_HSI1_NOC,
	CLK_CON_GAT_GATE_CLKCMU_HSI1_PCIE,
	CLK_CON_GAT_GATE_CLKCMU_HSI2_MMC_CARD,
	CLK_CON_GAT_GATE_CLKCMU_HSI2_NOC,
	CLK_CON_GAT_GATE_CLKCMU_HSI2_PCIE,
	CLK_CON_GAT_GATE_CLKCMU_HSI2_UFS_EMBD,
	CLK_CON_GAT_GATE_CLKCMU_PERIC0_IP,
	CLK_CON_GAT_GATE_CLKCMU_PERIC0_NOC,
	CLK_CON_GAT_GATE_CLKCMU_PERIC1_IP,
	CLK_CON_GAT_GATE_CLKCMU_PERIC1_NOC,
	CLK_CON_MUX_MUX_CLKCMU_HSI0_NOC,
	CLK_CON_DIV_CLKCMU_HSI0_NOC,
	CLK_CON_GAT_GATE_CLKCMU_HSI0_NOC,
	CLK_CON_MUX_MUX_CLKCMU_HSI0_PERI,
	CLK_CON_DIV_CLKCMU_HSI0_PERI,
	CLK_CON_GAT_GATE_CLKCMU_HSI0_PERI,
	CLK_CON_MUX_MUX_CLKCMU_MISC_NOC,
	CLK_CON_DIV_CLKCMU_MISC_NOC,
	CLK_CON_GAT_GATE_CLKCMU_MISC_NOC,
};

/*
 * These are fixed factors of shared PLLs in the downstream cmucal data. Keep
 * them fixed for now so this driver observes the bootloader-programmed clock
 * tree without programming PLLs.
 */
static const struct samsung_fixed_rate_clock top_fixed_clks[] __initconst = {
	FRATE(CLK_FOUT_SHARED0_D3, "fout_shared0_d3", NULL, 0, 711000000),
	FRATE(CLK_FOUT_SHARED0_D4, "fout_shared0_d4", NULL, 0, 533249984),
	FRATE(CLK_FOUT_SHARED1_D3, "fout_shared1_d3", NULL, 0, 622000000),
	FRATE(CLK_FOUT_SHARED1_D4, "fout_shared1_d4", NULL, 0, 466500000),
	FRATE(CLK_FOUT_SHARED2_D1, "fout_shared2_d1", NULL, 0, 800000000),
	FRATE(CLK_FOUT_SHARED2_D2, "fout_shared2_d2", NULL, 0, 400000000),
	FRATE(CLK_FOUT_SHARED3_D1, "fout_shared3_d1", NULL, 0, 667000000),
	FRATE(CLK_FOUT_SHARED3_D2, "fout_shared3_d2", NULL, 0, 333500000),
	FRATE(CLK_FOUT_SPARE_PLL, "fout_spare_pll", NULL, 0, 2400000000),
};

PNAME(mout_cmu_pericx_p) = {
	"fout_shared0_d4", "fout_shared2_d2",
	"fout_shared3_d2", "fout_spare_pll",
};
PNAME(mout_cmu_hsi1_noc_p) = {
	"fout_shared0_d4", "fout_shared1_d4",
	"fout_shared2_d2", "fout_shared3_d2",
	"fout_spare_pll", "oscclk", "oscclk", "oscclk",
};
PNAME(mout_cmu_hsi1_pcie_p) = { "oscclk", "fout_shared2_d2" };
PNAME(mout_cmu_hsi2_mmc_card_p) = {
	"fout_shared2_d1", "fout_shared3_d1",
	"fout_shared0_d4", "fout_spare_pll",
};
PNAME(mout_cmu_hsi2_noc_p) = {
	"fout_shared0_d4", "fout_shared1_d4",
	"fout_shared2_d2", "fout_shared3_d2",
	"fout_spare_pll", "oscclk", "oscclk", "oscclk",
};
PNAME(mout_cmu_hsi2_pcie_p) = { "oscclk", "fout_shared2_d2" };
PNAME(mout_cmu_hsi2_ufs_embd_p) = {
	"oscclk", "fout_shared0_d4", "fout_shared2_d2", "fout_spare_pll",
};
PNAME(mout_cmu_hsi0_noc_p) = {
	"fout_shared0_d4", "fout_shared1_d4",
	"fout_shared2_d2", "fout_shared3_d2",
	"fout_spare_pll", "oscclk", "oscclk", "oscclk",
};
PNAME(mout_cmu_hsi0_peri_p) = {
	"fout_shared0_d4", "fout_shared2_d2",
	"fout_shared3_d2", "fout_spare_pll",
};
PNAME(mout_cmu_dpu_noc_p) = {
	"fout_shared0_d3", "fout_shared3_d1",
	"fout_shared1_d3", "fout_shared0_d4",
	"fout_shared1_d4", "fout_shared2_d2",
	"fout_spare_pll", "oscclk",
};
PNAME(mout_cmu_dpub_dsim_p) = { "fout_shared0_d4", "fout_shared2_d2" };
PNAME(mout_cmu_misc_noc_p) = {
	"fout_shared0_d4", "fout_shared2_d2",
	"fout_shared3_d2", "fout_spare_pll",
};

static const struct samsung_mux_clock top_mux_clks[] __initconst = {
	MUX(CLK_MOUT_CMU_DPUB_DSIM, "mout_cmu_dpub_dsim",
	    mout_cmu_dpub_dsim_p, CLK_CON_MUX_MUX_CLKCMU_DPUB_DSIM, 0, 1),
	MUX(CLK_MOUT_CMU_DPUB_NOC, "mout_cmu_dpub_noc",
	    mout_cmu_dpu_noc_p, CLK_CON_MUX_MUX_CLKCMU_DPUB_NOC, 0, 3),
	MUX(CLK_MOUT_CMU_DPUF0_NOC, "mout_cmu_dpuf0_noc",
	    mout_cmu_dpu_noc_p, CLK_CON_MUX_MUX_CLKCMU_DPUF0_NOC, 0, 3),
	MUX(CLK_MOUT_CMU_DPUF1_NOC, "mout_cmu_dpuf1_noc",
	    mout_cmu_dpu_noc_p, CLK_CON_MUX_MUX_CLKCMU_DPUF1_NOC, 0, 3),
	MUX(CLK_MOUT_CMU_HSI1_NOC, "mout_cmu_hsi1_noc",
	    mout_cmu_hsi1_noc_p, CLK_CON_MUX_MUX_CLKCMU_HSI1_NOC, 0, 3),
	MUX(CLK_MOUT_CMU_HSI1_PCIE, "mout_cmu_hsi1_pcie",
	    mout_cmu_hsi1_pcie_p, CLK_CON_MUX_MUX_CLKCMU_HSI1_PCIE, 0, 1),
	MUX(CLK_MOUT_CMU_HSI2_MMC_CARD, "mout_cmu_hsi2_mmc_card",
	    mout_cmu_hsi2_mmc_card_p, CLK_CON_MUX_MUX_CLKCMU_HSI2_MMC_CARD,
	    0, 2),
	MUX(CLK_MOUT_CMU_HSI2_NOC, "mout_cmu_hsi2_noc",
	    mout_cmu_hsi2_noc_p, CLK_CON_MUX_MUX_CLKCMU_HSI2_NOC, 0, 3),
	MUX(CLK_MOUT_CMU_HSI2_PCIE, "mout_cmu_hsi2_pcie",
	    mout_cmu_hsi2_pcie_p, CLK_CON_MUX_MUX_CLKCMU_HSI2_PCIE, 0, 1),
	MUX(CLK_MOUT_CMU_HSI2_UFS_EMBD, "mout_cmu_hsi2_ufs_embd",
	    mout_cmu_hsi2_ufs_embd_p, CLK_CON_MUX_MUX_CLKCMU_HSI2_UFS_EMBD,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC0_NOC, "mout_cmu_peric0_noc",
	    mout_cmu_pericx_p, CLK_CON_MUX_MUX_CLKCMU_PERIC0_NOC, 0, 2),
	MUX(CLK_MOUT_CMU_PERIC0_IP, "mout_cmu_peric0_ip",
	    mout_cmu_pericx_p, CLK_CON_MUX_MUX_CLKCMU_PERIC0_IP, 0, 2),
	MUX(CLK_MOUT_CMU_PERIC1_NOC, "mout_cmu_peric1_noc",
	    mout_cmu_pericx_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_NOC, 0, 2),
	MUX(CLK_MOUT_CMU_PERIC1_IP, "mout_cmu_peric1_ip",
	    mout_cmu_pericx_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_IP, 0, 2),
	MUX(CLK_MOUT_CMU_HSI0_NOC, "mout_cmu_hsi0_noc",
	    mout_cmu_hsi0_noc_p, CLK_CON_MUX_MUX_CLKCMU_HSI0_NOC, 0, 3),
	MUX(CLK_MOUT_CMU_HSI0_PERI, "mout_cmu_hsi0_peri",
	    mout_cmu_hsi0_peri_p, CLK_CON_MUX_MUX_CLKCMU_HSI0_PERI, 0, 2),
	MUX(CLK_MOUT_CMU_MISC_NOC, "mout_cmu_misc_noc",
	    mout_cmu_misc_noc_p, CLK_CON_MUX_MUX_CLKCMU_MISC_NOC, 0, 2),
};

static const struct samsung_gate_clock top_gate_clks[] __initconst = {
	/*
	 * CMU_TOP runs in automatic (HWACG) clock mode, like gs101 and the
	 * downstream Zuma kernel: clk_enable()/clk_disable() are nops and the
	 * hardware Q-Channel gates each feed when its consumer block is idle and
	 * re-ungates on access. So no gate here needs CLK_IS_CRITICAL or
	 * CLK_IGNORE_UNUSED; the bring-up pins those flags papered over (console
	 * UART feed collapse, simplefb blanking, transient probe refcounts) cannot
	 * happen because CCF never touches the gate bits. Critically, this also
	 * lets the ACPM quiesce the peripheral fabric during suspend instead of
	 * spinning on a never-idle domain bit (the APM watchdog seen otherwise).
	 */
	GATE(CLK_GOUT_CMU_DPUB_DSIM, "gout_cmu_dpub_dsim",
	     "mout_cmu_dpub_dsim", CLK_CON_GAT_GATE_CLKCMU_DPUB_DSIM,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_DPUB_NOC, "gout_cmu_dpub_noc",
	     "mout_cmu_dpub_noc", CLK_CON_GAT_GATE_CLKCMU_DPUB_NOC,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_DPUF0_NOC, "gout_cmu_dpuf0_noc",
	     "mout_cmu_dpuf0_noc", CLK_CON_GAT_GATE_CLKCMU_DPUF0_NOC,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_DPUF1_NOC, "gout_cmu_dpuf1_noc",
	     "mout_cmu_dpuf1_noc", CLK_CON_GAT_GATE_CLKCMU_DPUF1_NOC,
	     21, 0, 0),
	/* HSI1 fabric + PCIe reference (modem PCIe CH0 root complex). */
	GATE(CLK_GOUT_CMU_HSI1_NOC, "gout_cmu_hsi1_noc",
	     "mout_cmu_hsi1_noc", CLK_CON_GAT_GATE_CLKCMU_HSI1_NOC,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_HSI1_PCIE, "gout_cmu_hsi1_pcie",
	     "mout_cmu_hsi1_pcie", CLK_CON_GAT_GATE_CLKCMU_HSI1_PCIE,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_HSI2_MMC_CARD, "gout_cmu_hsi2_mmc_card",
	     "mout_cmu_hsi2_mmc_card", CLK_CON_GAT_GATE_CLKCMU_HSI2_MMC_CARD,
	     21, 0, 0),
	/* HSI2 storage fabric (UFS is the boot path). */
	GATE(CLK_GOUT_CMU_HSI2_NOC, "gout_cmu_hsi2_noc",
	     "mout_cmu_hsi2_noc", CLK_CON_GAT_GATE_CLKCMU_HSI2_NOC,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_HSI2_PCIE, "gout_cmu_hsi2_pcie",
	     "mout_cmu_hsi2_pcie", CLK_CON_GAT_GATE_CLKCMU_HSI2_PCIE,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_HSI2_UFS_EMBD, "gout_cmu_hsi2_ufs_embd",
	     "mout_cmu_hsi2_ufs_embd", CLK_CON_GAT_GATE_CLKCMU_HSI2_UFS_EMBD,
	     21, 0, 0),
	/* PERIC0 fabric (debug console UART + every PERIC0 USI). */
	GATE(CLK_GOUT_CMU_PERIC0_NOC, "gout_cmu_peric0_noc",
	     "mout_cmu_peric0_noc", CLK_CON_GAT_GATE_CLKCMU_PERIC0_NOC,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC0_IP, "gout_cmu_peric0_ip",
	     "mout_cmu_peric0_ip", CLK_CON_GAT_GATE_CLKCMU_PERIC0_IP,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC1_NOC, "gout_cmu_peric1_noc",
	     "mout_cmu_peric1_noc", CLK_CON_GAT_GATE_CLKCMU_PERIC1_NOC,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC1_IP, "gout_cmu_peric1_ip",
	     "mout_cmu_peric1_ip", CLK_CON_GAT_GATE_CLKCMU_PERIC1_IP,
	     21, 0, 0),
	/* HSI0 USB fabric (live at fastboot handoff). */
	GATE(CLK_GOUT_CMU_HSI0_NOC, "gout_cmu_hsi0_noc",
	     "mout_cmu_hsi0_noc", CLK_CON_GAT_GATE_CLKCMU_HSI0_NOC,
	     21, 0, 0),
	/* BLK_HSI0 USI feed (touchscreen SPI runs off USI2). */
	GATE(CLK_GOUT_CMU_HSI0_PERI, "gout_cmu_hsi0_peri",
	     "mout_cmu_hsi0_peri", CLK_CON_GAT_GATE_CLKCMU_HSI0_PERI,
	     21, 0, 0),
	/* BLK_MISC fabric (clocks the MCT and the unmodelled MISC IPs). */
	GATE(CLK_GOUT_CMU_MISC_NOC, "gout_cmu_misc_noc",
	     "mout_cmu_misc_noc", CLK_CON_GAT_GATE_CLKCMU_MISC_NOC,
	     21, 0, 0),
};

static const struct samsung_div_clock top_div_clks[] __initconst = {
	DIV(CLK_DOUT_CMU_DPUB_DSIM, "dout_cmu_dpub_dsim",
	    "gout_cmu_dpub_dsim", CLK_CON_DIV_CLKCMU_DPUB_DSIM, 0, 4),
	DIV(CLK_DOUT_CMU_DPUB_NOC, "dout_cmu_dpub_noc",
	    "gout_cmu_dpub_noc", CLK_CON_DIV_CLKCMU_DPUB_NOC, 0, 4),
	DIV(CLK_DOUT_CMU_DPUF0_NOC, "dout_cmu_dpuf0_noc",
	    "gout_cmu_dpuf0_noc", CLK_CON_DIV_CLKCMU_DPUF0_NOC, 0, 4),
	DIV(CLK_DOUT_CMU_DPUF1_NOC, "dout_cmu_dpuf1_noc",
	    "gout_cmu_dpuf1_noc", CLK_CON_DIV_CLKCMU_DPUF1_NOC, 0, 4),
	DIV(CLK_DOUT_CMU_HSI1_NOC, "dout_cmu_hsi1_noc",
	    "gout_cmu_hsi1_noc", CLK_CON_DIV_CLKCMU_HSI1_NOC, 0, 4),
	DIV(CLK_DOUT_CMU_HSI1_PCIE, "dout_cmu_hsi1_pcie",
	    "gout_cmu_hsi1_pcie", CLK_CON_DIV_CLKCMU_HSI1_PCIE, 0, 3),
	DIV(CLK_DOUT_CMU_HSI2_MMC_CARD, "dout_cmu_hsi2_mmc_card",
	    "gout_cmu_hsi2_mmc_card", CLK_CON_DIV_CLKCMU_HSI2_MMC_CARD,
	    0, 9),
	DIV(CLK_DOUT_CMU_HSI2_NOC, "dout_cmu_hsi2_noc",
	    "gout_cmu_hsi2_noc", CLK_CON_DIV_CLKCMU_HSI2_NOC, 0, 4),
	DIV(CLK_DOUT_CMU_HSI2_PCIE, "dout_cmu_hsi2_pcie",
	    "gout_cmu_hsi2_pcie", CLK_CON_DIV_CLKCMU_HSI2_PCIE, 0, 3),
	DIV(CLK_DOUT_CMU_HSI2_UFS_EMBD, "dout_cmu_hsi2_ufs_embd",
	    "gout_cmu_hsi2_ufs_embd", CLK_CON_DIV_CLKCMU_HSI2_UFS_EMBD,
	    0, 4),
	DIV(CLK_DOUT_CMU_PERIC0_NOC, "dout_cmu_peric0_noc",
	    "gout_cmu_peric0_noc", CLK_CON_DIV_CLKCMU_PERIC0_NOC, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC0_IP, "dout_cmu_peric0_ip",
	    "gout_cmu_peric0_ip", CLK_CON_DIV_CLKCMU_PERIC0_IP, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_NOC, "dout_cmu_peric1_noc",
	    "gout_cmu_peric1_noc", CLK_CON_DIV_CLKCMU_PERIC1_NOC, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_IP, "dout_cmu_peric1_ip",
	    "gout_cmu_peric1_ip", CLK_CON_DIV_CLKCMU_PERIC1_IP, 0, 4),
	DIV(CLK_DOUT_CMU_HSI0_NOC, "dout_cmu_hsi0_noc",
	    "gout_cmu_hsi0_noc", CLK_CON_DIV_CLKCMU_HSI0_NOC, 0, 4),
	DIV(CLK_DOUT_CMU_HSI0_PERI, "dout_cmu_hsi0_peri",
	    "gout_cmu_hsi0_peri", CLK_CON_DIV_CLKCMU_HSI0_PERI, 0, 4),
	DIV(CLK_DOUT_CMU_MISC_NOC, "dout_cmu_misc_noc",
	    "gout_cmu_misc_noc", CLK_CON_DIV_CLKCMU_MISC_NOC, 0, 4),
};

static const struct samsung_cmu_info top_cmu_info __initconst = {
	.fixed_clks	= top_fixed_clks,
	.nr_fixed_clks	= ARRAY_SIZE(top_fixed_clks),
	.mux_clks	= top_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(top_mux_clks),
	.div_clks	= top_div_clks,
	.nr_div_clks	= ARRAY_SIZE(top_div_clks),
	.gate_clks	= top_gate_clks,
	.nr_gate_clks	= ARRAY_SIZE(top_gate_clks),
	.nr_clk_ids	= CLKS_NR_TOP,
	.clk_regs	= top_clk_regs,
	.nr_clk_regs	= ARRAY_SIZE(top_clk_regs),
	.auto_clock_gate = true,
	.gate_dbg_offset = ZUMAPRO_GATE_DBG_OFFSET,
	.option_offset	= CLK_CON_CMU_TOP_CONTROLLER_OPTION,
};

static void __init zumapro_cmu_top_init(struct device_node *np)
{
	exynos_arm64_register_cmu(NULL, np, &top_cmu_info);
}

CLK_OF_DECLARE(zumapro_cmu_top, "google,zumapro-cmu-top",
	       zumapro_cmu_top_init);

/* ---- CMU_MISC ----------------------------------------------------------- */

/* Register offsets for CMU_MISC (0x10010000) */
#define CLK_CON_CMU_MISC_CONTROLLER_OPTION			0x0800
#define PLL_CON0_MUX_CLKCMU_MISC_NOC_USER			0x0600
#define CLK_CON_DIV_DIV_CLK_MISC_NOCP				0x1808
#define CLK_CON_GAT_GOUT_BLK_MISC_UID_MCT_IPCLKPORT_PCLK	0x20c8

static const unsigned long misc_clk_regs[] __initconst = {
	CLK_CON_CMU_MISC_CONTROLLER_OPTION,
	PLL_CON0_MUX_CLKCMU_MISC_NOC_USER,
	CLK_CON_DIV_DIV_CLK_MISC_NOCP,
	CLK_CON_GAT_GOUT_BLK_MISC_UID_MCT_IPCLKPORT_PCLK,
};

PNAME(mout_misc_noc_user_p) = { "oscclk", "dout_cmu_misc_noc" };

static const struct samsung_mux_clock misc_mux_clks[] __initconst = {
	MUX(CLK_MOUT_MISC_NOC_USER, "mout_misc_noc_user",
	    mout_misc_noc_user_p, PLL_CON0_MUX_CLKCMU_MISC_NOC_USER, 4, 1),
};

static const struct samsung_div_clock misc_div_clks[] __initconst = {
	DIV(CLK_DOUT_MISC_NOCP, "dout_misc_nocp", "mout_misc_noc_user",
	    CLK_CON_DIV_DIV_CLK_MISC_NOCP, 0, 3),
};

static const struct samsung_gate_clock misc_gate_clks[] __initconst = {
	/*
	 * The MCT register-access pclk.  The MCT v3 driver enables this from
	 * its TIMER_OF_DECLARE init, well before clk_disable_unused(), so a
	 * plain refcounted gate keeps it on for the life of the timer.
	 */
	GATE(CLK_GOUT_MISC_MCT_PCLK, "gout_misc_mct_pclk", "dout_misc_nocp",
	     CLK_CON_GAT_GOUT_BLK_MISC_UID_MCT_IPCLKPORT_PCLK, 21, 0, 0),
};

static const struct samsung_cmu_info misc_cmu_info __initconst = {
	.mux_clks	= misc_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(misc_mux_clks),
	.div_clks	= misc_div_clks,
	.nr_div_clks	= ARRAY_SIZE(misc_div_clks),
	.gate_clks	= misc_gate_clks,
	.nr_gate_clks	= ARRAY_SIZE(misc_gate_clks),
	.nr_clk_ids	= CLKS_NR_MISC,
	.clk_regs	= misc_clk_regs,
	.nr_clk_regs	= ARRAY_SIZE(misc_clk_regs),
	.clk_name	= "bus",
	.auto_clock_gate = true,
	.gate_dbg_offset = ZUMAPRO_GATE_DBG_OFFSET,
	.option_offset	= CLK_CON_CMU_MISC_CONTROLLER_OPTION,
};

static void __init zumapro_cmu_misc_init(struct device_node *np)
{
	exynos_arm64_register_cmu(NULL, np, &misc_cmu_info);
}

/* Register CMU_MISC early, as it is needed for the MCT timer. */
CLK_OF_DECLARE(zumapro_cmu_misc, "google,zumapro-cmu-misc",
	       zumapro_cmu_misc_init);

/* ---- CMU_PERIC0 --------------------------------------------------------- */

/* Register offsets for CMU_PERIC0 (0x10800000) */
#define CLK_CON_CMU_PERIC0_CONTROLLER_OPTION			0x0800
#define PLL_CON0_MUX_CLKCMU_PERIC0_NOC_USER			0x0600
#define PLL_CON0_MUX_CLKCMU_PERIC0_USI0_UART_USER		0x0620
#define PLL_CON0_MUX_CLKCMU_PERIC0_USI6_USI_USER		0x06a0
#define CLK_CON_DIV_DIV_CLK_PERIC0_USI0_UART			0x1808
#define CLK_CON_DIV_DIV_CLK_PERIC0_USI6_USI			0x1824
#define CLK_CON_GAT_CLK_BLK_PERIC0_UID_PERIC0_CMU_PERIC0_IPCLKPORT_PCLK \
								0x2048
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_RSTNSYNC_CLK_PERIC0_USI0_UART_IPCLKPORT_CLK \
								0x20c8
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_RSTNSYNC_CLK_PERIC0_USI6_USI_IPCLKPORT_CLK \
								0x20e4
#define CLK_CON_GAT_CLK_BLK_PERIC0_UID_USI0_UART_IPCLKPORT_IPCLK \
								0x2060
#define CLK_CON_GAT_CLK_BLK_PERIC0_UID_USI0_UART_IPCLKPORT_PCLK \
								0x2064
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_GPIO_PERIC0_IPCLKPORT_PCLK \
								0x20b8
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_SYSREG_PERIC0_IPCLKPORT_PCLK \
								0x20f0
#define CLK_CON_GAT_CLK_BLK_PERIC0_UID_USI6_USI_IPCLKPORT_PCLK \
								0x209c

static const unsigned long peric0_clk_regs[] __initconst = {
	CLK_CON_CMU_PERIC0_CONTROLLER_OPTION,
	PLL_CON0_MUX_CLKCMU_PERIC0_NOC_USER,
	PLL_CON0_MUX_CLKCMU_PERIC0_USI0_UART_USER,
	PLL_CON0_MUX_CLKCMU_PERIC0_USI6_USI_USER,
	CLK_CON_DIV_DIV_CLK_PERIC0_USI0_UART,
	CLK_CON_DIV_DIV_CLK_PERIC0_USI6_USI,
	CLK_CON_GAT_CLK_BLK_PERIC0_UID_PERIC0_CMU_PERIC0_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_PERIC0_UID_RSTNSYNC_CLK_PERIC0_USI0_UART_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_PERIC0_UID_RSTNSYNC_CLK_PERIC0_USI6_USI_IPCLKPORT_CLK,
	CLK_CON_GAT_CLK_BLK_PERIC0_UID_USI0_UART_IPCLKPORT_IPCLK,
	CLK_CON_GAT_CLK_BLK_PERIC0_UID_USI0_UART_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_PERIC0_UID_GPIO_PERIC0_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_PERIC0_UID_SYSREG_PERIC0_IPCLKPORT_PCLK,
	CLK_CON_GAT_CLK_BLK_PERIC0_UID_USI6_USI_IPCLKPORT_PCLK,
};

PNAME(mout_peric0_noc_user_p) = { "oscclk", "dout_cmu_peric0_noc" };
PNAME(mout_peric0_usi0_uart_user_p) = { "oscclk", "dout_cmu_peric0_ip" };
PNAME(mout_peric0_usi6_usi_user_p) = { "oscclk", "dout_cmu_peric0_ip" };

static const struct samsung_mux_clock peric0_mux_clks[] __initconst = {
	MUX(CLK_MOUT_PERIC0_NOC_USER, "mout_peric0_noc_user",
	    mout_peric0_noc_user_p, PLL_CON0_MUX_CLKCMU_PERIC0_NOC_USER,
	    4, 1),
	MUX(CLK_MOUT_PERIC0_USI0_UART_USER, "mout_peric0_usi0_uart_user",
	    mout_peric0_usi0_uart_user_p,
	    PLL_CON0_MUX_CLKCMU_PERIC0_USI0_UART_USER, 4, 1),
	MUX(CLK_MOUT_PERIC0_USI6_USI_USER, "mout_peric0_usi6_usi_user",
	    mout_peric0_usi6_usi_user_p,
	    PLL_CON0_MUX_CLKCMU_PERIC0_USI6_USI_USER, 4, 1),
};

static const struct samsung_div_clock peric0_div_clks[] __initconst = {
	DIV(CLK_DOUT_PERIC0_USI0_UART, "dout_peric0_usi0_uart",
	    "mout_peric0_usi0_uart_user",
	    CLK_CON_DIV_DIV_CLK_PERIC0_USI0_UART, 0, 4),
	DIV_F(CLK_DOUT_PERIC0_USI6_USI, "dout_peric0_usi6_usi",
	      "mout_peric0_usi6_usi_user",
	      CLK_CON_DIV_DIV_CLK_PERIC0_USI6_USI, 0, 4,
	      CLK_SET_RATE_PARENT, 0),
};

static const struct samsung_gate_clock peric0_gate_clks[] __initconst = {
	GATE(CLK_GOUT_PERIC0_GPIO_PERIC0_PCLK,
	     "gout_peric0_gpio_peric0_pclk", "mout_peric0_noc_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_GPIO_PERIC0_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC0_SYSREG_PERIC0_PCLK,
	     "gout_peric0_sysreg_peric0_pclk", "mout_peric0_noc_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_SYSREG_PERIC0_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC0_CMU_PERIC0_PCLK,
	     "gout_peric0_cmu_peric0_pclk", "mout_peric0_noc_user",
	     CLK_CON_GAT_CLK_BLK_PERIC0_UID_PERIC0_CMU_PERIC0_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC0_USI0_UART_IPCLK,
	     "gout_peric0_usi0_uart_ipclk", "dout_peric0_usi0_uart",
	     CLK_CON_GAT_CLK_BLK_PERIC0_UID_USI0_UART_IPCLKPORT_IPCLK,
	     21, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERIC0_USI0_UART_PCLK,
	     "gout_peric0_usi0_uart_pclk", "mout_peric0_noc_user",
	     CLK_CON_GAT_CLK_BLK_PERIC0_UID_USI0_UART_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC0_USI0_UART_CLK,
	     "gout_peric0_usi0_uart_clk", "dout_peric0_usi0_uart",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_RSTNSYNC_CLK_PERIC0_USI0_UART_IPCLKPORT_CLK,
	     21, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERIC0_USI6_USI_PCLK,
	     "gout_peric0_usi6_usi_pclk", "mout_peric0_noc_user",
	     CLK_CON_GAT_CLK_BLK_PERIC0_UID_USI6_USI_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC0_USI6_USI_CLK,
	     "gout_peric0_usi6_usi_clk", "dout_peric0_usi6_usi",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_RSTNSYNC_CLK_PERIC0_USI6_USI_IPCLKPORT_CLK,
	     21, CLK_SET_RATE_PARENT, 0),
};

/*
 * SYSREG registers saved/restored across power-down for the peripheral leaf
 * CMUs. Only BUS_COMPONENT_DRCG_EN; the PERIC SYSREGs have no MEMCLK gate
 * (matches gs101's dcrg_sysreg[]).
 */
static const unsigned long zumapro_dcrg_sysreg[] __initconst = {
	ZUMAPRO_DRCG_EN_OFFSET,
};

static const struct samsung_cmu_info peric0_cmu_info __initconst = {
	.mux_clks	= peric0_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(peric0_mux_clks),
	.div_clks	= peric0_div_clks,
	.nr_div_clks	= ARRAY_SIZE(peric0_div_clks),
	.gate_clks	= peric0_gate_clks,
	.nr_gate_clks	= ARRAY_SIZE(peric0_gate_clks),
	.nr_clk_ids	= CLKS_NR_PERIC0,
	.clk_regs	= peric0_clk_regs,
	.nr_clk_regs	= ARRAY_SIZE(peric0_clk_regs),
	.sysreg_clk_regs = zumapro_dcrg_sysreg,
	.nr_sysreg_clk_regs = ARRAY_SIZE(zumapro_dcrg_sysreg),
	.clk_name	= "bus",
	.auto_clock_gate = true,
	.gate_dbg_offset = ZUMAPRO_GATE_DBG_OFFSET,
	.option_offset	= CLK_CON_CMU_PERIC0_CONTROLLER_OPTION,
	.drcg_offset	= ZUMAPRO_DRCG_EN_OFFSET,
};

/* ---- CMU_PERIC1 --------------------------------------------------------- */

/* Register offsets for CMU_PERIC1 (0x10c00000) */
#define CLK_CON_CMU_PERIC1_CONTROLLER_OPTION			0x0800
#define PLL_CON0_MUX_CLKCMU_PERIC1_NOC_USER			0x0610
#define PLL_CON0_MUX_CLKCMU_PERIC1_USI10_USI_USER		0x0630
#define PLL_CON0_MUX_CLKCMU_PERIC1_USI9_USI_USER		0x0690
#define CLK_CON_DIV_DIV_CLK_PERIC1_USI10_USI			0x180c
#define CLK_CON_DIV_DIV_CLK_PERIC1_USI9_USI			0x1824
#define CLK_CON_GAT_CLK_BLK_PERIC1_UID_PERIC1_CMU_PERIC1_IPCLKPORT_PCLK \
								0x2010
#define CLK_CON_GAT_CLK_BLK_PERIC1_UID_USI10_USI_IPCLKPORT_IPCLK \
								0x2040
#define CLK_CON_GAT_CLK_BLK_PERIC1_UID_USI10_USI_IPCLKPORT_PCLK \
								0x2044
#define CLK_CON_GAT_CLK_BLK_PERIC1_UID_USI9_USI_IPCLKPORT_IPCLK \
								0x2070
#define CLK_CON_GAT_CLK_BLK_PERIC1_UID_USI9_USI_IPCLKPORT_PCLK \
								0x2074
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_GPIO_PERIC1_IPCLKPORT_PCLK \
								0x2080
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_RSTNSYNC_CLK_PERIC1_USI10_USI_IPCLKPORT_CLK \
								0x2090
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_RSTNSYNC_CLK_PERIC1_USI9_USI_IPCLKPORT_CLK \
								0x20a0
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SYSREG_PERIC1_IPCLKPORT_PCLK \
								0x20a4

static const unsigned long peric1_clk_regs[] __initconst = {
	CLK_CON_CMU_PERIC1_CONTROLLER_OPTION,
	PLL_CON0_MUX_CLKCMU_PERIC1_NOC_USER,
	PLL_CON0_MUX_CLKCMU_PERIC1_USI10_USI_USER,
	PLL_CON0_MUX_CLKCMU_PERIC1_USI9_USI_USER,
	CLK_CON_DIV_DIV_CLK_PERIC1_USI10_USI,
	CLK_CON_DIV_DIV_CLK_PERIC1_USI9_USI,
	CLK_CON_GAT_CLK_BLK_PERIC1_UID_PERIC1_CMU_PERIC1_IPCLKPORT_PCLK,
	CLK_CON_GAT_CLK_BLK_PERIC1_UID_USI10_USI_IPCLKPORT_IPCLK,
	CLK_CON_GAT_CLK_BLK_PERIC1_UID_USI10_USI_IPCLKPORT_PCLK,
	CLK_CON_GAT_CLK_BLK_PERIC1_UID_USI9_USI_IPCLKPORT_IPCLK,
	CLK_CON_GAT_CLK_BLK_PERIC1_UID_USI9_USI_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_PERIC1_UID_GPIO_PERIC1_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_PERIC1_UID_RSTNSYNC_CLK_PERIC1_USI10_USI_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_PERIC1_UID_RSTNSYNC_CLK_PERIC1_USI9_USI_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SYSREG_PERIC1_IPCLKPORT_PCLK,
};

PNAME(mout_peric1_noc_user_p) = { "oscclk", "dout_cmu_peric1_noc" };
PNAME(mout_peric1_usi10_usi_user_p) = { "oscclk", "dout_cmu_peric1_ip" };
PNAME(mout_peric1_usi9_usi_user_p) = { "oscclk", "dout_cmu_peric1_ip" };

static const struct samsung_mux_clock peric1_mux_clks[] __initconst = {
	MUX(CLK_MOUT_PERIC1_NOC_USER, "mout_peric1_noc_user",
	    mout_peric1_noc_user_p, PLL_CON0_MUX_CLKCMU_PERIC1_NOC_USER,
	    4, 1),
	MUX(CLK_MOUT_PERIC1_USI10_USI_USER, "mout_peric1_usi10_usi_user",
	    mout_peric1_usi10_usi_user_p,
	    PLL_CON0_MUX_CLKCMU_PERIC1_USI10_USI_USER, 4, 1),
	MUX(CLK_MOUT_PERIC1_USI9_USI_USER, "mout_peric1_usi9_usi_user",
	    mout_peric1_usi9_usi_user_p,
	    PLL_CON0_MUX_CLKCMU_PERIC1_USI9_USI_USER, 4, 1),
};

static const struct samsung_div_clock peric1_div_clks[] __initconst = {
	DIV_F(CLK_DOUT_PERIC1_USI10_USI, "dout_peric1_usi10_usi",
	      "mout_peric1_usi10_usi_user",
	      CLK_CON_DIV_DIV_CLK_PERIC1_USI10_USI, 0, 4,
	      CLK_SET_RATE_PARENT, 0),
	DIV_F(CLK_DOUT_PERIC1_USI9_USI, "dout_peric1_usi9_usi",
	      "mout_peric1_usi9_usi_user",
	      CLK_CON_DIV_DIV_CLK_PERIC1_USI9_USI, 0, 4,
	      CLK_SET_RATE_PARENT, 0),
};

static const struct samsung_gate_clock peric1_gate_clks[] __initconst = {
	GATE(CLK_GOUT_PERIC1_GPIO_PERIC1_PCLK,
	     "gout_peric1_gpio_peric1_pclk", "mout_peric1_noc_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_GPIO_PERIC1_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_SYSREG_PERIC1_PCLK,
	     "gout_peric1_sysreg_peric1_pclk", "mout_peric1_noc_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SYSREG_PERIC1_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_CMU_PERIC1_PCLK,
	     "gout_peric1_cmu_peric1_pclk", "mout_peric1_noc_user",
	     CLK_CON_GAT_CLK_BLK_PERIC1_UID_PERIC1_CMU_PERIC1_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI10_USI_PCLK,
	     "gout_peric1_usi10_usi_pclk", "mout_peric1_noc_user",
	     CLK_CON_GAT_CLK_BLK_PERIC1_UID_USI10_USI_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI10_USI_IPCLK,
	     "gout_peric1_usi10_usi_ipclk", "dout_peric1_usi10_usi",
	     CLK_CON_GAT_CLK_BLK_PERIC1_UID_USI10_USI_IPCLKPORT_IPCLK,
	     21, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERIC1_USI10_USI_CLK,
	     "gout_peric1_usi10_usi_clk", "dout_peric1_usi10_usi",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_RSTNSYNC_CLK_PERIC1_USI10_USI_IPCLKPORT_CLK,
	     21, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERIC1_USI9_USI_PCLK,
	     "gout_peric1_usi9_usi_pclk", "mout_peric1_noc_user",
	     CLK_CON_GAT_CLK_BLK_PERIC1_UID_USI9_USI_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI9_USI_IPCLK,
	     "gout_peric1_usi9_usi_ipclk", "dout_peric1_usi9_usi",
	     CLK_CON_GAT_CLK_BLK_PERIC1_UID_USI9_USI_IPCLKPORT_IPCLK,
	     21, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERIC1_USI9_USI_CLK,
	     "gout_peric1_usi9_usi_clk", "dout_peric1_usi9_usi",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_RSTNSYNC_CLK_PERIC1_USI9_USI_IPCLKPORT_CLK,
	     21, CLK_SET_RATE_PARENT, 0),
};

static const struct samsung_cmu_info peric1_cmu_info __initconst = {
	.mux_clks	= peric1_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(peric1_mux_clks),
	.div_clks	= peric1_div_clks,
	.nr_div_clks	= ARRAY_SIZE(peric1_div_clks),
	.gate_clks	= peric1_gate_clks,
	.nr_gate_clks	= ARRAY_SIZE(peric1_gate_clks),
	.nr_clk_ids	= CLKS_NR_PERIC1,
	.clk_regs	= peric1_clk_regs,
	.nr_clk_regs	= ARRAY_SIZE(peric1_clk_regs),
	.sysreg_clk_regs = zumapro_dcrg_sysreg,
	.nr_sysreg_clk_regs = ARRAY_SIZE(zumapro_dcrg_sysreg),
	.clk_name	= "bus",
	.auto_clock_gate = true,
	.gate_dbg_offset = ZUMAPRO_GATE_DBG_OFFSET,
	.option_offset	= CLK_CON_CMU_PERIC1_CONTROLLER_OPTION,
	.drcg_offset	= ZUMAPRO_DRCG_EN_OFFSET,
};

/* ---- CMU_HSI1 ----------------------------------------------------------- */

/*
 * Register offsets for CMU_HSI1 (0x12000000).
 *
 * BLK_HSI1 hosts the modem's PCIe CH0 root complex (pcie@12100000, Gen3 x2,
 * downstream IP name PCIE_GEN3_0 / PAMIR_G3X2).  Offsets are from the
 * downstream Zuma cmucal-sfr.c tables like the CMU_HSI2 slice below; the block
 * sits in the pd-hsi1 power domain (PMU HSI1_CONFIGURATION 0x15462b00).
 */
#define CLK_CON_CMU_HSI1_CONTROLLER_OPTION		0x0800
#define PLL_CON0_MUX_CLKCMU_HSI1_NOC_USER		0x0600
#define PLL_CON0_MUX_CLKCMU_HSI1_PCIE_USER		0x0610
#define CLK_CON_GAT_CLK_BLK_HSI1_UID_HSI1_CMU_HSI1_IPCLKPORT_PCLK \
								0x2004
#define CLK_CON_GAT_GOUT_BLK_HSI1_UID_GPIO_HSI1_IPCLKPORT_PCLK	0x205c
#define QCH_CON_GPIO_HSI1_QCH				0x303c
#define CLK_CON_GAT_GOUT_BLK_HSI1_UID_SYSREG_HSI1_IPCLKPORT_PCLK \
								0x20d4
#define CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_PHY_REFCLK_IN	0x208c
#define CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_DBI_ACLK	0x2068
#define CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_MSTR_ACLK	0x206c
#define CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_SLV_ACLK	0x2070
#define CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_DRIVER_APB_CLK	0x2074
#define CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_UDBG_APB_PCLK	0x2090
#define CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_PIPE_PAL_APB_PCLK	0x2094

static const unsigned long hsi1_clk_regs[] __initconst = {
	CLK_CON_CMU_HSI1_CONTROLLER_OPTION,
	PLL_CON0_MUX_CLKCMU_HSI1_NOC_USER,
	PLL_CON0_MUX_CLKCMU_HSI1_PCIE_USER,
	CLK_CON_GAT_CLK_BLK_HSI1_UID_HSI1_CMU_HSI1_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_HSI1_UID_GPIO_HSI1_IPCLKPORT_PCLK,
	QCH_CON_GPIO_HSI1_QCH,
	CLK_CON_GAT_GOUT_BLK_HSI1_UID_SYSREG_HSI1_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_PHY_REFCLK_IN,
	CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_DBI_ACLK,
	CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_MSTR_ACLK,
	CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_SLV_ACLK,
	CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_DRIVER_APB_CLK,
	CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_UDBG_APB_PCLK,
	CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_PIPE_PAL_APB_PCLK,
};

PNAME(mout_hsi1_noc_user_p) = { "oscclk", "dout_cmu_hsi1_noc" };
PNAME(mout_hsi1_pcie_user_p) = { "oscclk", "dout_cmu_hsi1_pcie" };

static const struct samsung_mux_clock hsi1_mux_clks[] __initconst = {
	MUX(CLK_MOUT_HSI1_NOC_USER, "mout_hsi1_noc_user",
	    mout_hsi1_noc_user_p, PLL_CON0_MUX_CLKCMU_HSI1_NOC_USER,
	    4, 1),
	MUX(CLK_MOUT_HSI1_PCIE_USER, "mout_hsi1_pcie_user",
	    mout_hsi1_pcie_user_p, PLL_CON0_MUX_CLKCMU_HSI1_PCIE_USER,
	    4, 1),
};

static const struct samsung_gate_clock hsi1_gate_clks[] __initconst = {
	GATE(CLK_GOUT_HSI1_CMU_HSI1_PCLK,
	     "gout_hsi1_cmu_hsi1_pclk", "mout_hsi1_noc_user",
	     CLK_CON_GAT_CLK_BLK_HSI1_UID_HSI1_CMU_HSI1_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI1_GPIO_HSI1_QCH,
	     "gout_hsi1_gpio_hsi1_qch", "mout_hsi1_noc_user",
	     QCH_CON_GPIO_HSI1_QCH, 0, 0, 0),
	GATE(CLK_GOUT_HSI1_GPIO_HSI1_PCLK,
	     "gout_hsi1_gpio_hsi1_pclk", "gout_hsi1_gpio_hsi1_qch",
	     CLK_CON_GAT_GOUT_BLK_HSI1_UID_GPIO_HSI1_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI1_SYSREG_HSI1_PCLK,
	     "gout_hsi1_sysreg_hsi1_pclk", "mout_hsi1_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI1_UID_SYSREG_HSI1_IPCLKPORT_PCLK,
	     21, 0, 0),
	/*
	 * PCIe GEN3_0 (modem).  Only the PHY reference runs off the dedicated
	 * pcie_user mux; the controller AXI/APB clocks share the HSI1 bus mux,
	 * matching the CMU_HSI2 GEN3A_1 layout.
	 */
	GATE(CLK_GOUT_HSI1_PCIE_GEN3_0_PHY_REFCLK_IN,
	     "gout_hsi1_pcie_gen3_0_phy_refclk_in", "mout_hsi1_pcie_user",
	     CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_PHY_REFCLK_IN,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI1_PCIE_GEN3_0_DBI_ACLK,
	     "gout_hsi1_pcie_gen3_0_dbi_aclk", "mout_hsi1_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_DBI_ACLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI1_PCIE_GEN3_0_MSTR_ACLK,
	     "gout_hsi1_pcie_gen3_0_mstr_aclk", "mout_hsi1_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_MSTR_ACLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI1_PCIE_GEN3_0_SLV_ACLK,
	     "gout_hsi1_pcie_gen3_0_slv_aclk", "mout_hsi1_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_SLV_ACLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI1_PCIE_GEN3_0_DRIVER_APB_CLK,
	     "gout_hsi1_pcie_gen3_0_driver_apb_clk", "mout_hsi1_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_DRIVER_APB_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI1_PCIE_GEN3_0_UDBG_APB_PCLK,
	     "gout_hsi1_pcie_gen3_0_udbg_apb_pclk", "mout_hsi1_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_UDBG_APB_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI1_PCIE_GEN3_0_PIPE_PAL_APB_PCLK,
	     "gout_hsi1_pcie_gen3_0_pipe_pal_apb_pclk", "mout_hsi1_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI1_UID_PCIE_GEN3_0_PIPE_PAL_APB_PCLK,
	     21, 0, 0),
};

static const struct samsung_cmu_info hsi1_cmu_info __initconst = {
	.mux_clks	= hsi1_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(hsi1_mux_clks),
	.gate_clks	= hsi1_gate_clks,
	.nr_gate_clks	= ARRAY_SIZE(hsi1_gate_clks),
	.nr_clk_ids	= CLKS_NR_HSI1,
	.clk_regs	= hsi1_clk_regs,
	.nr_clk_regs	= ARRAY_SIZE(hsi1_clk_regs),
	.sysreg_clk_regs = zumapro_dcrg_sysreg,
	.nr_sysreg_clk_regs = ARRAY_SIZE(zumapro_dcrg_sysreg),
	.clk_name	= "bus",
	.auto_clock_gate = true,
	.gate_dbg_offset = ZUMAPRO_GATE_DBG_OFFSET,
	.option_offset	= CLK_CON_CMU_HSI1_CONTROLLER_OPTION,
	.drcg_offset	= ZUMAPRO_DRCG_EN_OFFSET,
};

/* ---- CMU_HSI2 ----------------------------------------------------------- */

/* Register offsets for CMU_HSI2 (0x13000000) */
#define CLK_CON_CMU_HSI2_CONTROLLER_OPTION		0x0800
#define PLL_CON0_MUX_CLKCMU_HSI2_MMC_CARD_USER		0x0600
#define PLL_CON0_MUX_CLKCMU_HSI2_NOC_USER		0x0610
#define PLL_CON0_MUX_CLKCMU_HSI2_PCIE_USER		0x0620
#define PLL_CON0_MUX_CLKCMU_HSI2_UFS_EMBD_USER		0x0630
#define CLK_CON_GAT_CLK_BLK_HSI2_UID_GPIO_HSI2UFS_IPCLKPORT_PCLK \
								0x2008
#define QCH_CON_GPIO_HSI2UFS_QCH			0x302c
#define CLK_CON_GAT_GOUT_BLK_HSI2_UID_GPIO_HSI2_IPCLKPORT_PCLK	0x2060
#define QCH_CON_GPIO_HSI2_QCH				0x3030
#define CLK_CON_GAT_GOUT_BLK_HSI2_UID_HSI2_CMU_HSI2_IPCLKPORT_PCLK \
								0x2064
#define CLK_CON_GAT_GOUT_BLK_HSI2_UID_SYSREG_HSI2_IPCLKPORT_PCLK \
								0x20e8
#define CLK_CON_GAT_GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_ACLK \
								0x210c
#define CLK_CON_GAT_GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_CLK_UNIPRO \
								0x2110
#define CLK_CON_GAT_GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_FMP_CLK \
								0x2114
#define CLK_CON_GAT_GOUT_BLK_HSI2_UID_QE_UFS_EMBD_HSI2_IPCLKPORT_ACLK \
								0x20cc
#define CLK_CON_GAT_GOUT_BLK_HSI2_UID_QE_UFS_EMBD_HSI2_IPCLKPORT_PCLK \
								0x20d0

/* PCIe GEN3A_1: WiFi root complex on pcie@13120000 (CH1, Gen3 x1) */
#define CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_PHY_REFCLK_IN	0x201c
#define CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_DBI_ACLK	0x2088
#define CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_MSTR_ACLK	0x208c
#define CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_SLV_ACLK	0x2090
#define CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_DRIVER_APB_CLK	0x2094
#define CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_UDBG_APB_PCLK	0x2098
#define CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_PIPE_PAL_APB_PCLK	0x209c

static const unsigned long hsi2_clk_regs[] __initconst = {
	CLK_CON_CMU_HSI2_CONTROLLER_OPTION,
	PLL_CON0_MUX_CLKCMU_HSI2_MMC_CARD_USER,
	PLL_CON0_MUX_CLKCMU_HSI2_NOC_USER,
	PLL_CON0_MUX_CLKCMU_HSI2_PCIE_USER,
	PLL_CON0_MUX_CLKCMU_HSI2_UFS_EMBD_USER,
	CLK_CON_GAT_CLK_BLK_HSI2_UID_GPIO_HSI2UFS_IPCLKPORT_PCLK,
	QCH_CON_GPIO_HSI2UFS_QCH,
	CLK_CON_GAT_GOUT_BLK_HSI2_UID_GPIO_HSI2_IPCLKPORT_PCLK,
	QCH_CON_GPIO_HSI2_QCH,
	CLK_CON_GAT_GOUT_BLK_HSI2_UID_HSI2_CMU_HSI2_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_HSI2_UID_SYSREG_HSI2_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_ACLK,
	CLK_CON_GAT_GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_CLK_UNIPRO,
	CLK_CON_GAT_GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_FMP_CLK,
	CLK_CON_GAT_GOUT_BLK_HSI2_UID_QE_UFS_EMBD_HSI2_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_HSI2_UID_QE_UFS_EMBD_HSI2_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_PHY_REFCLK_IN,
	CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_DBI_ACLK,
	CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_MSTR_ACLK,
	CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_SLV_ACLK,
	CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_DRIVER_APB_CLK,
	CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_UDBG_APB_PCLK,
	CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_PIPE_PAL_APB_PCLK,
};

PNAME(mout_hsi2_mmc_card_user_p) = { "oscclk", "dout_cmu_hsi2_mmc_card" };
PNAME(mout_hsi2_noc_user_p) = { "oscclk", "dout_cmu_hsi2_noc" };
PNAME(mout_hsi2_pcie_user_p) = { "oscclk", "dout_cmu_hsi2_pcie" };
PNAME(mout_hsi2_ufs_embd_user_p) = { "oscclk", "dout_cmu_hsi2_ufs_embd" };

static const struct samsung_mux_clock hsi2_mux_clks[] __initconst = {
	MUX(CLK_MOUT_HSI2_MMC_CARD_USER, "mout_hsi2_mmc_card_user",
	    mout_hsi2_mmc_card_user_p,
	    PLL_CON0_MUX_CLKCMU_HSI2_MMC_CARD_USER, 4, 1),
	MUX(CLK_MOUT_HSI2_NOC_USER, "mout_hsi2_noc_user",
	    mout_hsi2_noc_user_p, PLL_CON0_MUX_CLKCMU_HSI2_NOC_USER,
	    4, 1),
	MUX(CLK_MOUT_HSI2_PCIE_USER, "mout_hsi2_pcie_user",
	    mout_hsi2_pcie_user_p, PLL_CON0_MUX_CLKCMU_HSI2_PCIE_USER,
	    4, 1),
	MUX(CLK_MOUT_HSI2_UFS_EMBD_USER, "mout_hsi2_ufs_embd_user",
	    mout_hsi2_ufs_embd_user_p,
	    PLL_CON0_MUX_CLKCMU_HSI2_UFS_EMBD_USER, 4, 1),
};

static const struct samsung_gate_clock hsi2_gate_clks[] __initconst = {
	GATE(CLK_GOUT_HSI2_GPIO_HSI2UFS_QCH,
	     "gout_hsi2_gpio_hsi2ufs_qch", "mout_hsi2_noc_user",
	     QCH_CON_GPIO_HSI2UFS_QCH, 0, 0, 0),
	GATE(CLK_GOUT_HSI2_GPIO_HSI2UFS_PCLK,
	     "gout_hsi2_gpio_hsi2ufs_pclk", "gout_hsi2_gpio_hsi2ufs_qch",
	     CLK_CON_GAT_CLK_BLK_HSI2_UID_GPIO_HSI2UFS_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI2_GPIO_HSI2_QCH,
	     "gout_hsi2_gpio_hsi2_qch", "mout_hsi2_noc_user",
	     QCH_CON_GPIO_HSI2_QCH, 0, 0, 0),
	GATE(CLK_GOUT_HSI2_GPIO_HSI2_PCLK,
	     "gout_hsi2_gpio_hsi2_pclk", "gout_hsi2_gpio_hsi2_qch",
	     CLK_CON_GAT_GOUT_BLK_HSI2_UID_GPIO_HSI2_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI2_CMU_HSI2_PCLK,
	     "gout_hsi2_cmu_hsi2_pclk", "mout_hsi2_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI2_UID_HSI2_CMU_HSI2_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI2_SYSREG_HSI2_PCLK,
	     "gout_hsi2_sysreg_hsi2_pclk", "mout_hsi2_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI2_UID_SYSREG_HSI2_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI2_UFS_EMBD_I_ACLK,
	     "gout_hsi2_ufs_embd_i_aclk", "mout_hsi2_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_ACLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI2_UFS_EMBD_I_CLK_UNIPRO,
	     "gout_hsi2_ufs_embd_i_clk_unipro", "mout_hsi2_ufs_embd_user",
	     CLK_CON_GAT_GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_CLK_UNIPRO,
	     21, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_HSI2_UFS_EMBD_I_FMP_CLK,
	     "gout_hsi2_ufs_embd_i_fmp_clk", "mout_hsi2_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_FMP_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI2_QE_UFS_EMBD_HSI2_ACLK,
	     "gout_hsi2_qe_ufs_embd_hsi2_aclk", "mout_hsi2_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI2_UID_QE_UFS_EMBD_HSI2_IPCLKPORT_ACLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI2_QE_UFS_EMBD_HSI2_PCLK,
	     "gout_hsi2_qe_ufs_embd_hsi2_pclk", "mout_hsi2_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI2_UID_QE_UFS_EMBD_HSI2_IPCLKPORT_PCLK,
	     21, 0, 0),
	/*
	 * PCIe GEN3A_1 (WiFi).  Only the PHY reference runs off the dedicated
	 * pcie_user mux; the controller AXI/APB clocks share the HSI2 bus mux,
	 * matching the gs101 layout.
	 */
	GATE(CLK_GOUT_HSI2_PCIE_GEN3A_1_PHY_REFCLK_IN,
	     "gout_hsi2_pcie_gen3a_1_phy_refclk_in", "mout_hsi2_pcie_user",
	     CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_PHY_REFCLK_IN,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI2_PCIE_GEN3A_1_DBI_ACLK,
	     "gout_hsi2_pcie_gen3a_1_dbi_aclk", "mout_hsi2_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_DBI_ACLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI2_PCIE_GEN3A_1_MSTR_ACLK,
	     "gout_hsi2_pcie_gen3a_1_mstr_aclk", "mout_hsi2_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_MSTR_ACLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI2_PCIE_GEN3A_1_SLV_ACLK,
	     "gout_hsi2_pcie_gen3a_1_slv_aclk", "mout_hsi2_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_SLV_ACLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI2_PCIE_GEN3A_1_DRIVER_APB_CLK,
	     "gout_hsi2_pcie_gen3a_1_driver_apb_clk", "mout_hsi2_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_DRIVER_APB_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI2_PCIE_GEN3A_1_UDBG_APB_PCLK,
	     "gout_hsi2_pcie_gen3a_1_udbg_apb_pclk", "mout_hsi2_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_UDBG_APB_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI2_PCIE_GEN3A_1_PIPE_PAL_APB_PCLK,
	     "gout_hsi2_pcie_gen3a_1_pipe_pal_apb_pclk", "mout_hsi2_noc_user",
	     CLK_CON_GAT_GOUT_BLK_HSI2_UID_PCIE_GEN3A_1_PIPE_PAL_APB_PCLK,
	     21, 0, 0),
};

static const struct samsung_cmu_info hsi2_cmu_info __initconst = {
	.mux_clks	= hsi2_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(hsi2_mux_clks),
	.gate_clks	= hsi2_gate_clks,
	.nr_gate_clks	= ARRAY_SIZE(hsi2_gate_clks),
	.nr_clk_ids	= CLKS_NR_HSI2,
	.clk_regs	= hsi2_clk_regs,
	.nr_clk_regs	= ARRAY_SIZE(hsi2_clk_regs),
	.sysreg_clk_regs = zumapro_dcrg_sysreg,
	.nr_sysreg_clk_regs = ARRAY_SIZE(zumapro_dcrg_sysreg),
	.clk_name	= "bus",
	.auto_clock_gate = true,
	.gate_dbg_offset = ZUMAPRO_GATE_DBG_OFFSET,
	.option_offset	= CLK_CON_CMU_HSI2_CONTROLLER_OPTION,
	.drcg_offset	= ZUMAPRO_DRCG_EN_OFFSET,
};

/* ---- CMU_HSI0 ----------------------------------------------------------- */

/*
 * Register offsets for CMU_HSI0 (0x11000000).
 *
 * USB is live at handoff and this driver only observes the tree.  Per the
 * hardware trace (docs/archive/usb/needs-hardware-2026-05.md H1/H3), the USB reference is the
 * internal PLL_USB / DIV_CLK_HSI0_USB path (19.2 MHz), not the CMU_TOP USB32DRD
 * USER path; model only the NOC USER mux, USB/eUSB dividers, the USB32DRD
 * link Q-channel gate, and the clocks needed by early HSI0 consumers.
 *
 * The USI2 chain (touchscreen SPI) comes from the downstream Zuma CMUCAL like
 * the rest of this file.  TRACE NEEDED: the USI2/PERI offsets have not been
 * hardware-validated yet (the USB offsets from the same table have been).
 */
#define CLK_CON_CMU_HSI0_CONTROLLER_OPTION	0x0800
#define PLL_CON0_MUX_CLKCMU_HSI0_NOC_USER	0x0620
#define PLL_CON0_MUX_CLKCMU_HSI0_PERI_USER	0x0680
#define CLK_CON_MUX_MUX_CLK_HSI0_USI2		0x101c
#define CLK_CON_MUX_MUX_CLK_HSI0_USI4		0x1024
#define CLK_CON_DIV_DIV_CLK_HSI0_USB		0x1804
#define CLK_CON_DIV_DIV_CLK_HSI0_EUSB		0x180c
#define CLK_CON_DIV_DIV_CLK_HSI0_USI2		0x181c
#define CLK_CON_DIV_DIV_CLK_HSI0_USI4		0x1824
#define CLK_CON_GAT_CLK_BLK_HSI0_UID_USI2_HSI0_IPCLKPORT_IPCLK \
							0x20fc
#define CLK_CON_GAT_CLK_BLK_HSI0_UID_USI2_HSI0_IPCLKPORT_PCLK \
							0x2100
#define CLK_CON_GAT_CLK_BLK_HSI0_UID_USI4_HSI0_IPCLKPORT_IPCLK \
							0x210c
#define CLK_CON_GAT_CLK_BLK_HSI0_UID_USI4_HSI0_IPCLKPORT_PCLK \
							0x2110
#define QCH_CON_USB32DRD_QCH_LINK		0x30c0

static const unsigned long hsi0_clk_regs[] __initconst = {
	CLK_CON_CMU_HSI0_CONTROLLER_OPTION,
	PLL_CON0_MUX_CLKCMU_HSI0_NOC_USER,
	PLL_CON0_MUX_CLKCMU_HSI0_PERI_USER,
	CLK_CON_MUX_MUX_CLK_HSI0_USI2,
	CLK_CON_MUX_MUX_CLK_HSI0_USI4,
	CLK_CON_DIV_DIV_CLK_HSI0_USB,
	CLK_CON_DIV_DIV_CLK_HSI0_EUSB,
	CLK_CON_DIV_DIV_CLK_HSI0_USI2,
	CLK_CON_DIV_DIV_CLK_HSI0_USI4,
	CLK_CON_GAT_CLK_BLK_HSI0_UID_USI2_HSI0_IPCLKPORT_IPCLK,
	CLK_CON_GAT_CLK_BLK_HSI0_UID_USI2_HSI0_IPCLKPORT_PCLK,
	CLK_CON_GAT_CLK_BLK_HSI0_UID_USI4_HSI0_IPCLKPORT_IPCLK,
	CLK_CON_GAT_CLK_BLK_HSI0_UID_USI4_HSI0_IPCLKPORT_PCLK,
	QCH_CON_USB32DRD_QCH_LINK,
};

/*
 * PLL_USB is a CMU_HSI0-internal PLL programmed by the bootloader; this driver
 * never touches it.  The trace shows DIV_CLK_HSI0_USB reads /32 and its output
 * (the USB reference) is 19.2 MHz, so PLL_USB runs at 614.4 MHz.  Model it as a
 * fixed rate, like the shared PLLs in CMU_TOP.
 */
static const struct samsung_fixed_rate_clock hsi0_fixed_clks[] __initconst = {
	FRATE(CLK_FOUT_USB, "fout_usb", NULL, 0, 614400000),
};

PNAME(mout_hsi0_noc_user_p) = { "oscclk", "dout_cmu_hsi0_noc" };
PNAME(mout_hsi0_peri_user_p) = { "oscclk", "dout_cmu_hsi0_peri" };
PNAME(mout_hsi0_usi2_p) = { "mout_hsi0_peri_user", "oscclk" };
PNAME(mout_hsi0_usi4_p) = { "mout_hsi0_peri_user", "oscclk" };

static const struct samsung_mux_clock hsi0_mux_clks[] __initconst = {
	MUX(CLK_MOUT_HSI0_NOC_USER, "mout_hsi0_noc_user",
	    mout_hsi0_noc_user_p, PLL_CON0_MUX_CLKCMU_HSI0_NOC_USER, 4, 1),
	MUX(CLK_MOUT_HSI0_PERI_USER, "mout_hsi0_peri_user",
	    mout_hsi0_peri_user_p, PLL_CON0_MUX_CLKCMU_HSI0_PERI_USER, 4, 1),
	MUX(CLK_MOUT_HSI0_USI2, "mout_hsi0_usi2",
	    mout_hsi0_usi2_p, CLK_CON_MUX_MUX_CLK_HSI0_USI2, 0, 1),
	MUX(CLK_MOUT_HSI0_USI4, "mout_hsi0_usi4",
	    mout_hsi0_usi4_p, CLK_CON_MUX_MUX_CLK_HSI0_USI4, 0, 1),
};

static const struct samsung_div_clock hsi0_div_clks[] __initconst = {
	/* USB reference (eUSB2 PHY + DWC3 core "ref"): 614.4 MHz / 32 = 19.2 MHz */
	DIV(CLK_DOUT_HSI0_USB, "dout_hsi0_usb", "fout_usb",
	    CLK_CON_DIV_DIV_CLK_HSI0_USB, 0, 6),
	DIV(CLK_DOUT_HSI0_EUSB, "dout_hsi0_eusb", "mout_hsi0_noc_user",
	    CLK_CON_DIV_DIV_CLK_HSI0_EUSB, 0, 2),
	DIV(CLK_DOUT_HSI0_USI2, "dout_hsi0_usi2", "mout_hsi0_usi2",
	    CLK_CON_DIV_DIV_CLK_HSI0_USI2, 0, 4),
	DIV(CLK_DOUT_HSI0_USI4, "dout_hsi0_usi4", "mout_hsi0_usi4",
	    CLK_CON_DIV_DIV_CLK_HSI0_USI4, 0, 4),
};

static const struct samsung_gate_clock hsi0_gate_clks[] __initconst = {
	GATE(CLK_GOUT_HSI0_USB32DRD_LINK, "gout_hsi0_usb32drd_link",
	     "dout_hsi0_usb", QCH_CON_USB32DRD_QCH_LINK, 0, 0, 0),
	GATE(CLK_GOUT_HSI0_USI2_USI_CLK, "gout_hsi0_usi2_usi_clk",
	     "dout_hsi0_usi2",
	     CLK_CON_GAT_CLK_BLK_HSI0_UID_USI2_HSI0_IPCLKPORT_IPCLK,
	     21, CLK_SET_RATE_PARENT, 0),
	/*
	 * Downstream parents the USI2 pclk on MUX_CLK_HSI0_NOC; this driver
	 * models the BLK_HSI0 fabric as the single NOC USER mux, like the
	 * HSI2 pclk gates.
	 */
	GATE(CLK_GOUT_HSI0_USI2_USI_PCLK, "gout_hsi0_usi2_usi_pclk",
	     "mout_hsi0_noc_user",
	     CLK_CON_GAT_CLK_BLK_HSI0_UID_USI2_HSI0_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_HSI0_USI4_USI_CLK, "gout_hsi0_usi4_usi_clk",
	     "dout_hsi0_usi4",
	     CLK_CON_GAT_CLK_BLK_HSI0_UID_USI4_HSI0_IPCLKPORT_IPCLK,
	     21, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_HSI0_USI4_USI_PCLK, "gout_hsi0_usi4_usi_pclk",
	     "mout_hsi0_noc_user",
	     CLK_CON_GAT_CLK_BLK_HSI0_UID_USI4_HSI0_IPCLKPORT_PCLK,
	     21, 0, 0),
};

static const struct samsung_cmu_info hsi0_cmu_info __initconst = {
	.fixed_clks	= hsi0_fixed_clks,
	.nr_fixed_clks	= ARRAY_SIZE(hsi0_fixed_clks),
	.mux_clks	= hsi0_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(hsi0_mux_clks),
	.div_clks	= hsi0_div_clks,
	.nr_div_clks	= ARRAY_SIZE(hsi0_div_clks),
	.gate_clks	= hsi0_gate_clks,
	.nr_gate_clks	= ARRAY_SIZE(hsi0_gate_clks),
	.nr_clk_ids	= CLKS_NR_HSI0,
	.clk_regs	= hsi0_clk_regs,
	.nr_clk_regs	= ARRAY_SIZE(hsi0_clk_regs),
	.clk_name	= "noc",
	.auto_clock_gate = true,
	.gate_dbg_offset = ZUMAPRO_GATE_DBG_OFFSET,
	.option_offset	= CLK_CON_CMU_HSI0_CONTROLLER_OPTION,
};

/* ---- CMU_DPUB ----------------------------------------------------------- */

/*
 * Register offsets for CMU_DPUB (0x19400000).
 *
 * Source: downstream Zuma CMUCAL and the dumped Tegu DT power-domain cmu_id.
 * TRACE NEEDED: validate the bootloader-selected parent indices/rates on Tegu
 * before allowing the real DRM stack to reparent or change these rates.
 */
#define PLL_CON0_MUX_CLKCMU_DPUB_DSIM_USER	0x0600
#define PLL_CON0_MUX_CLKCMU_DPUB_NOC_USER	0x0610
#define CLK_CON_DIV_DIV_CLK_DPUB_NOCP		0x1800
#define CLK_CON_GAT_CLK_BLK_DPUB_UID_DPUB_CMU_DPUB_IPCLKPORT_PCLK \
							0x2004
#define CLK_CON_GAT_CLK_BLK_DPUB_UID_DPUB_IPCLKPORT_ALVCLK_DSIM0 \
							0x2008
#define CLK_CON_GAT_CLK_BLK_DPUB_UID_DPUB_IPCLKPORT_OSCCLK_DSIM0 \
							0x2010
#define CLK_CON_GAT_GOUT_BLK_DPUB_UID_AD_APB_DECON_MAIN_IPCLKPORT_PCLKM \
							0x2028
#define CLK_CON_GAT_GOUT_BLK_DPUB_UID_DPUB_IPCLKPORT_ACLK_DECON \
							0x202c
#define CLK_CON_GAT_GOUT_BLK_DPUB_UID_SYSREG_DPUB_IPCLKPORT_PCLK \
							0x2044
/*
 * CONTROLLER_OPTION turns on global automatic (HWACG/Q-Channel) clock gating
 * for the whole CMU; the QCH_CON nodes are the per-block Q-Channels the option
 * drives. Saved/restored so auto mode and the QCH state survive a pd_dpub power
 * cycle. Offsets from downstream Zuma cmucal-sfr.c (CMU_DPUB base 0x19400000).
 */
#define DPUB_CMU_DPUB_CONTROLLER_OPTION		0x0800
#define QCH_CON_DPUB_CMU_DPUB_QCH		0x3060
#define QCH_CON_DPUB_QCH			0x3064
#define QCH_CON_DPUB_QCH_ALV_DSIM0		0x3068
#define QCH_CON_DPUB_QCH_ALV_DSIM1		0x306c
#define QCH_CON_DPUB_QCH_OSC_DSIM0		0x3070
#define QCH_CON_DPUB_QCH_OSC_DSIM1		0x3074
#define QCH_CON_D_TZPC_DPUB_QCH			0x3078
#define QCH_CON_GPC_DPUB_QCH			0x307c
#define QCH_CON_SLH_AXI_MI_P_DPUB_QCH		0x30ec
#define QCH_CON_SYSREG_DPUB_QCH			0x30f0

static const unsigned long dpub_clk_regs[] __initconst = {
	DPUB_CMU_DPUB_CONTROLLER_OPTION,
	PLL_CON0_MUX_CLKCMU_DPUB_DSIM_USER,
	PLL_CON0_MUX_CLKCMU_DPUB_NOC_USER,
	CLK_CON_DIV_DIV_CLK_DPUB_NOCP,
	CLK_CON_GAT_CLK_BLK_DPUB_UID_DPUB_CMU_DPUB_IPCLKPORT_PCLK,
	CLK_CON_GAT_CLK_BLK_DPUB_UID_DPUB_IPCLKPORT_ALVCLK_DSIM0,
	CLK_CON_GAT_CLK_BLK_DPUB_UID_DPUB_IPCLKPORT_OSCCLK_DSIM0,
	CLK_CON_GAT_GOUT_BLK_DPUB_UID_AD_APB_DECON_MAIN_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_DPUB_UID_DPUB_IPCLKPORT_ACLK_DECON,
	CLK_CON_GAT_GOUT_BLK_DPUB_UID_SYSREG_DPUB_IPCLKPORT_PCLK,
	QCH_CON_DPUB_CMU_DPUB_QCH,
	QCH_CON_DPUB_QCH,
	QCH_CON_DPUB_QCH_ALV_DSIM0,
	QCH_CON_DPUB_QCH_ALV_DSIM1,
	QCH_CON_DPUB_QCH_OSC_DSIM0,
	QCH_CON_DPUB_QCH_OSC_DSIM1,
	QCH_CON_D_TZPC_DPUB_QCH,
	QCH_CON_GPC_DPUB_QCH,
	QCH_CON_SLH_AXI_MI_P_DPUB_QCH,
	QCH_CON_SYSREG_DPUB_QCH,
};

PNAME(mout_dpub_dsim_user_p) = { "oscclk", "dout_cmu_dpub_dsim" };
PNAME(mout_dpub_noc_user_p) = { "oscclk", "dout_cmu_dpub_noc" };

static const struct samsung_mux_clock dpub_mux_clks[] __initconst = {
	MUX(CLK_MOUT_DPUB_DSIM_USER, "mout_dpub_dsim_user",
	    mout_dpub_dsim_user_p, PLL_CON0_MUX_CLKCMU_DPUB_DSIM_USER, 4, 1),
	MUX(CLK_MOUT_DPUB_NOC_USER, "mout_dpub_noc_user",
	    mout_dpub_noc_user_p, PLL_CON0_MUX_CLKCMU_DPUB_NOC_USER, 4, 1),
};

static const struct samsung_div_clock dpub_div_clks[] __initconst = {
	DIV(CLK_DOUT_DPUB_NOCP, "dout_dpub_nocp", "mout_dpub_noc_user",
	    CLK_CON_DIV_DIV_CLK_DPUB_NOCP, 0, 3),
};

static const struct samsung_gate_clock dpub_gate_clks[] __initconst = {
	/*
	 * CMU_DPUB runs in automatic clock mode (see dpub_cmu_info), so the
	 * hardware Q-Channel gates each of these when its consumer is idle and
	 * re-ungates on access. None of them needs CLK_IGNORE_UNUSED or
	 * CLK_IS_CRITICAL: clk_disable is a no-op in auto mode, and the QCH is
	 * what previously had to be worked around by pinning (see below). This
	 * matches gs101, which leaves the equivalent DPU gates with flags 0.
	 */
	GATE(CLK_GOUT_DPUB_CMU_DPUB_PCLK, "gout_dpub_cmu_dpub_pclk",
	     "dout_dpub_nocp",
	     CLK_CON_GAT_CLK_BLK_DPUB_UID_DPUB_CMU_DPUB_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_DPUB_DECON_PCLK, "gout_dpub_decon_pclk",
	     "mout_dpub_noc_user",
	     CLK_CON_GAT_GOUT_BLK_DPUB_UID_AD_APB_DECON_MAIN_IPCLKPORT_PCLKM,
	     21, 0, 0),
	GATE(CLK_GOUT_DPUB_DECON_ACLK, "gout_dpub_decon_aclk",
	     "mout_dpub_noc_user",
	     CLK_CON_GAT_GOUT_BLK_DPUB_UID_DPUB_IPCLKPORT_ACLK_DECON,
	     21, 0, 0),
	/*
	 * Previously CLK_IS_CRITICAL: the MIPI D-PHY's first cold bring-up access
	 * is a write to SYSREG_DPUB via zumapro_dphy_sysreg_update(), and the
	 * DSIM0 ALV/OSC clocks keep the dphy register window alive. When the CMU
	 * was in manual mode the QCH gating of these was not modeled, so a
	 * post-teardown access could stall the NoC and freeze the SoC; pinning
	 * them was the blunt-force stand-in for the QCH IGNORE_FORCE_PM bit.
	 * Automatic mode now models the Q-Channel (auto-ungate on access), so the
	 * pins are removed -- and a permanently-on clock here is exactly what
	 * blocks pd_dpub from quiescing for deep suspend.
	 */
	GATE(CLK_GOUT_DPUB_SYSREG_PCLK, "gout_dpub_sysreg_pclk",
	     "dout_dpub_nocp",
	     CLK_CON_GAT_GOUT_BLK_DPUB_UID_SYSREG_DPUB_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_DPUB_DSIM0_ALVCLK, "gout_dpub_dsim0_alvclk",
	     "mout_dpub_dsim_user",
	     CLK_CON_GAT_CLK_BLK_DPUB_UID_DPUB_IPCLKPORT_ALVCLK_DSIM0,
	     21, 0, 0),
	GATE(CLK_GOUT_DPUB_DSIM0_OSCCLK, "gout_dpub_dsim0_oscclk",
	     "oscclk",
	     CLK_CON_GAT_CLK_BLK_DPUB_UID_DPUB_IPCLKPORT_OSCCLK_DSIM0,
	     21, 0, 0),
};

static const struct samsung_cmu_info dpub_cmu_info __initconst = {
	.mux_clks	= dpub_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(dpub_mux_clks),
	.div_clks	= dpub_div_clks,
	.nr_div_clks	= ARRAY_SIZE(dpub_div_clks),
	.gate_clks	= dpub_gate_clks,
	.nr_gate_clks	= ARRAY_SIZE(dpub_gate_clks),
	.nr_clk_ids	= CLKS_NR_DPUB,
	.clk_regs	= dpub_clk_regs,
	.nr_clk_regs	= ARRAY_SIZE(dpub_clk_regs),
	.clk_name	= "bus",
	.auto_clock_gate = true,
	.gate_dbg_offset = ZUMAPRO_GATE_DBG_OFFSET,
	.option_offset	= DPUB_CMU_DPUB_CONTROLLER_OPTION,
};

/* ---- CMU_DPUF0 ---------------------------------------------------------- */

/*
 * Register offsets for CMU_DPUF0 (0x19800000).
 *
 * Source: downstream Zuma CMUCAL and dumped.dts pd-dpuf0 cmu_id.
 * TRACE NEEDED: the gate subset follows downstream naming, but the first real
 * DRM test should confirm which DPUF0 clocks toggle around layer fetch.
 */
#define PLL_CON0_MUX_CLKCMU_DPUF0_NOC_USER	0x0600
#define CLK_CON_DIV_DIV_CLK_DPUF0_NOCP		0x1800
#define CLK_CON_GAT_CLK_BLK_DPUF0_UID_DPUF0_CMU_DPUF0_IPCLKPORT_PCLK \
							0x2004
#define CLK_CON_GAT_CLK_BLK_DPUF0_UID_DPUF0_IPCLKPORT_ACLK_SRAMC \
							0x2008
#define CLK_CON_GAT_GOUT_BLK_DPUF0_UID_SYSMMU_S0_PMMU1_DPUF0_IPCLKPORT_CLK \
							0x2060
#define CLK_CON_GAT_GOUT_BLK_DPUF0_UID_AD_APB_DPU_DMA_IPCLKPORT_PCLKM \
							0x206c
#define CLK_CON_GAT_GOUT_BLK_DPUF0_UID_DPUF0_IPCLKPORT_ACLK \
							0x2070
#define CLK_CON_GAT_GOUT_BLK_DPUF0_UID_SYSMMU_S0_DPUF0_IPCLKPORT_CLK \
							0x20a4
#define CLK_CON_GAT_GOUT_BLK_DPUF0_UID_SYSMMU_S0_PMMU0_DPUF0_IPCLKPORT_CLK \
							0x20a8
#define CLK_CON_GAT_GOUT_BLK_DPUF0_UID_SYSREG_DPUF0_IPCLKPORT_PCLK \
							0x20ac

static const unsigned long dpuf0_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_DPUF0_NOC_USER,
	CLK_CON_DIV_DIV_CLK_DPUF0_NOCP,
	CLK_CON_GAT_CLK_BLK_DPUF0_UID_DPUF0_CMU_DPUF0_IPCLKPORT_PCLK,
	CLK_CON_GAT_CLK_BLK_DPUF0_UID_DPUF0_IPCLKPORT_ACLK_SRAMC,
	CLK_CON_GAT_GOUT_BLK_DPUF0_UID_SYSMMU_S0_PMMU1_DPUF0_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_DPUF0_UID_AD_APB_DPU_DMA_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_DPUF0_UID_DPUF0_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPUF0_UID_SYSMMU_S0_DPUF0_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_DPUF0_UID_SYSMMU_S0_PMMU0_DPUF0_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_DPUF0_UID_SYSREG_DPUF0_IPCLKPORT_PCLK,
};

PNAME(mout_dpuf0_noc_user_p) = { "oscclk", "dout_cmu_dpuf0_noc" };

static const struct samsung_mux_clock dpuf0_mux_clks[] __initconst = {
	MUX(CLK_MOUT_DPUF0_NOC_USER, "mout_dpuf0_noc_user",
	    mout_dpuf0_noc_user_p, PLL_CON0_MUX_CLKCMU_DPUF0_NOC_USER, 4, 1),
};

static const struct samsung_div_clock dpuf0_div_clks[] __initconst = {
	DIV(CLK_DOUT_DPUF0_NOCP, "dout_dpuf0_nocp", "mout_dpuf0_noc_user",
	    CLK_CON_DIV_DIV_CLK_DPUF0_NOCP, 0, 3),
};

static const struct samsung_gate_clock dpuf0_gate_clks[] __initconst = {
	GATE(CLK_GOUT_DPUF0_CMU_DPUF0_PCLK, "gout_dpuf0_cmu_dpuf0_pclk",
	     "dout_dpuf0_nocp",
	     CLK_CON_GAT_CLK_BLK_DPUF0_UID_DPUF0_CMU_DPUF0_IPCLKPORT_PCLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPUF0_DPUF_ACLK, "gout_dpuf0_dpuf_aclk",
	     "mout_dpuf0_noc_user",
	     CLK_CON_GAT_GOUT_BLK_DPUF0_UID_DPUF0_IPCLKPORT_ACLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPUF0_DPUF_DMA_PCLK, "gout_dpuf0_dpuf_dma_pclk",
	     "mout_dpuf0_noc_user",
	     CLK_CON_GAT_GOUT_BLK_DPUF0_UID_AD_APB_DPU_DMA_IPCLKPORT_PCLKM,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPUF0_SYSREG_PCLK, "gout_dpuf0_sysreg_pclk",
	     "dout_dpuf0_nocp",
	     CLK_CON_GAT_GOUT_BLK_DPUF0_UID_SYSREG_DPUF0_IPCLKPORT_PCLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPUF0_SYSMMU_S0_CLK, "gout_dpuf0_sysmmu_s0_clk",
	     "mout_dpuf0_noc_user",
	     CLK_CON_GAT_GOUT_BLK_DPUF0_UID_SYSMMU_S0_DPUF0_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPUF0_SYSMMU_PMMU0_CLK, "gout_dpuf0_sysmmu_pmmu0_clk",
	     "mout_dpuf0_noc_user",
	     CLK_CON_GAT_GOUT_BLK_DPUF0_UID_SYSMMU_S0_PMMU0_DPUF0_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPUF0_SYSMMU_PMMU1_CLK, "gout_dpuf0_sysmmu_pmmu1_clk",
	     "mout_dpuf0_noc_user",
	     CLK_CON_GAT_GOUT_BLK_DPUF0_UID_SYSMMU_S0_PMMU1_DPUF0_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPUF0_SRAMC_ACLK, "gout_dpuf0_sramc_aclk",
	     "mout_dpuf0_noc_user",
	     CLK_CON_GAT_CLK_BLK_DPUF0_UID_DPUF0_IPCLKPORT_ACLK_SRAMC,
	     21, CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info dpuf0_cmu_info __initconst = {
	.mux_clks	= dpuf0_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(dpuf0_mux_clks),
	.div_clks	= dpuf0_div_clks,
	.nr_div_clks	= ARRAY_SIZE(dpuf0_div_clks),
	.gate_clks	= dpuf0_gate_clks,
	.nr_gate_clks	= ARRAY_SIZE(dpuf0_gate_clks),
	.nr_clk_ids	= CLKS_NR_DPUF0,
	.clk_regs	= dpuf0_clk_regs,
	.nr_clk_regs	= ARRAY_SIZE(dpuf0_clk_regs),
	.clk_name	= "bus",
};

/* ---- CMU_DPUF1 ---------------------------------------------------------- */

/*
 * Register offsets for CMU_DPUF1 (0x19c00000).
 *
 * Source: downstream Zuma CMUCAL and dumped.dts pd-dpuf1 cmu_id.
 * TRACE NEEDED: same as DPUF0; validate the DPUF1 fetch path once a real
 * composition requires the second fetch domain.
 */
#define PLL_CON0_MUX_CLKCMU_DPUF1_NOC_USER	0x0600
#define CLK_CON_DIV_DIV_CLK_DPUF1_NOCP		0x1800
#define CLK_CON_GAT_CLK_BLK_DPUF1_UID_DPUF1_CMU_DPUF1_IPCLKPORT_PCLK \
							0x2004
#define CLK_CON_GAT_CLK_BLK_DPUF1_UID_DPUF1_IPCLKPORT_ACLK_SRAMC \
							0x2008
#define CLK_CON_GAT_GOUT_BLK_DPUF1_UID_SYSMMU_S0_DPUF1_IPCLKPORT_CLK \
							0x2058
#define CLK_CON_GAT_CLK_BLK_DPUF1_UID_SYSMMU_S0_PMMU1_DPUF1_IPCLKPORT_CLK \
							0x205c
#define CLK_CON_GAT_GOUT_BLK_DPUF1_UID_AD_APB_DPU_DMA_IPCLKPORT_PCLKM \
							0x2064
#define CLK_CON_GAT_GOUT_BLK_DPUF1_UID_DPUF1_IPCLKPORT_ACLK \
							0x2068
#define CLK_CON_GAT_GOUT_BLK_DPUF1_UID_SYSMMU_S0_PMMU0_DPUF1_IPCLKPORT_CLK \
							0x2098
#define CLK_CON_GAT_GOUT_BLK_DPUF1_UID_SYSREG_DPUF1_IPCLKPORT_PCLK \
							0x209c

static const unsigned long dpuf1_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_DPUF1_NOC_USER,
	CLK_CON_DIV_DIV_CLK_DPUF1_NOCP,
	CLK_CON_GAT_CLK_BLK_DPUF1_UID_DPUF1_CMU_DPUF1_IPCLKPORT_PCLK,
	CLK_CON_GAT_CLK_BLK_DPUF1_UID_DPUF1_IPCLKPORT_ACLK_SRAMC,
	CLK_CON_GAT_GOUT_BLK_DPUF1_UID_SYSMMU_S0_DPUF1_IPCLKPORT_CLK,
	CLK_CON_GAT_CLK_BLK_DPUF1_UID_SYSMMU_S0_PMMU1_DPUF1_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_DPUF1_UID_AD_APB_DPU_DMA_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_DPUF1_UID_DPUF1_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPUF1_UID_SYSMMU_S0_PMMU0_DPUF1_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_DPUF1_UID_SYSREG_DPUF1_IPCLKPORT_PCLK,
};

PNAME(mout_dpuf1_noc_user_p) = { "oscclk", "dout_cmu_dpuf1_noc" };

static const struct samsung_mux_clock dpuf1_mux_clks[] __initconst = {
	MUX(CLK_MOUT_DPUF1_NOC_USER, "mout_dpuf1_noc_user",
	    mout_dpuf1_noc_user_p, PLL_CON0_MUX_CLKCMU_DPUF1_NOC_USER, 4, 1),
};

static const struct samsung_div_clock dpuf1_div_clks[] __initconst = {
	DIV(CLK_DOUT_DPUF1_NOCP, "dout_dpuf1_nocp", "mout_dpuf1_noc_user",
	    CLK_CON_DIV_DIV_CLK_DPUF1_NOCP, 0, 3),
};

static const struct samsung_gate_clock dpuf1_gate_clks[] __initconst = {
	GATE(CLK_GOUT_DPUF1_CMU_DPUF1_PCLK, "gout_dpuf1_cmu_dpuf1_pclk",
	     "dout_dpuf1_nocp",
	     CLK_CON_GAT_CLK_BLK_DPUF1_UID_DPUF1_CMU_DPUF1_IPCLKPORT_PCLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPUF1_DPUF_ACLK, "gout_dpuf1_dpuf_aclk",
	     "mout_dpuf1_noc_user",
	     CLK_CON_GAT_GOUT_BLK_DPUF1_UID_DPUF1_IPCLKPORT_ACLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPUF1_DPUF_DMA_PCLK, "gout_dpuf1_dpuf_dma_pclk",
	     "mout_dpuf1_noc_user",
	     CLK_CON_GAT_GOUT_BLK_DPUF1_UID_AD_APB_DPU_DMA_IPCLKPORT_PCLKM,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPUF1_SYSREG_PCLK, "gout_dpuf1_sysreg_pclk",
	     "dout_dpuf1_nocp",
	     CLK_CON_GAT_GOUT_BLK_DPUF1_UID_SYSREG_DPUF1_IPCLKPORT_PCLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPUF1_SYSMMU_S0_CLK, "gout_dpuf1_sysmmu_s0_clk",
	     "mout_dpuf1_noc_user",
	     CLK_CON_GAT_GOUT_BLK_DPUF1_UID_SYSMMU_S0_DPUF1_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPUF1_SYSMMU_PMMU0_CLK, "gout_dpuf1_sysmmu_pmmu0_clk",
	     "mout_dpuf1_noc_user",
	     CLK_CON_GAT_GOUT_BLK_DPUF1_UID_SYSMMU_S0_PMMU0_DPUF1_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPUF1_SYSMMU_PMMU1_CLK, "gout_dpuf1_sysmmu_pmmu1_clk",
	     "mout_dpuf1_noc_user",
	     CLK_CON_GAT_CLK_BLK_DPUF1_UID_SYSMMU_S0_PMMU1_DPUF1_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPUF1_SRAMC_ACLK, "gout_dpuf1_sramc_aclk",
	     "mout_dpuf1_noc_user",
	     CLK_CON_GAT_CLK_BLK_DPUF1_UID_DPUF1_IPCLKPORT_ACLK_SRAMC,
	     21, CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info dpuf1_cmu_info __initconst = {
	.mux_clks	= dpuf1_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(dpuf1_mux_clks),
	.div_clks	= dpuf1_div_clks,
	.nr_div_clks	= ARRAY_SIZE(dpuf1_div_clks),
	.gate_clks	= dpuf1_gate_clks,
	.nr_gate_clks	= ARRAY_SIZE(dpuf1_gate_clks),
	.nr_clk_ids	= CLKS_NR_DPUF1,
	.clk_regs	= dpuf1_clk_regs,
	.nr_clk_regs	= ARRAY_SIZE(dpuf1_clk_regs),
	.clk_name	= "bus",
};

/* ---- platform_driver ---------------------------------------------------- */

static int __init zumapro_cmu_probe(struct platform_device *pdev)
{
	const struct samsung_cmu_info *info;
	struct device *dev = &pdev->dev;

	info = of_device_get_match_data(dev);
	exynos_arm64_register_cmu(dev, dev->of_node, info);

	return 0;
}

static const struct of_device_id zumapro_cmu_of_match[] = {
	{
		.compatible = "google,zumapro-cmu-peric0",
		.data = &peric0_cmu_info,
	}, {
		.compatible = "google,zumapro-cmu-peric1",
		.data = &peric1_cmu_info,
	}, {
		.compatible = "google,zumapro-cmu-hsi1",
		.data = &hsi1_cmu_info,
	}, {
		.compatible = "google,zumapro-cmu-hsi2",
		.data = &hsi2_cmu_info,
	}, {
		.compatible = "google,zumapro-cmu-hsi0",
		.data = &hsi0_cmu_info,
	}, {
		.compatible = "google,zumapro-cmu-dpub",
		.data = &dpub_cmu_info,
	}, {
		.compatible = "google,zumapro-cmu-dpuf0",
		.data = &dpuf0_cmu_info,
	}, {
		.compatible = "google,zumapro-cmu-dpuf1",
		.data = &dpuf1_cmu_info,
	}, {
	},
};

static struct platform_driver zumapro_cmu_driver __refdata = {
	.driver	= {
		.name = "zumapro-cmu",
		.of_match_table = zumapro_cmu_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = zumapro_cmu_probe,
};

static int __init zumapro_cmu_init(void)
{
	return platform_driver_register(&zumapro_cmu_driver);
}
core_initcall(zumapro_cmu_init);
