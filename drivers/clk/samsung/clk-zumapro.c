// SPDX-License-Identifier: GPL-2.0-only
/*
 * Common Clock Framework support for Google Tensor G4 "Zumapro".
 *
 * This initial driver is intentionally scoped to the already-running UART
 * path: CMU_TOP parents for PERIC0 and the CMU_PERIC0 UART clocks. Register
 * offsets come from the downstream Zuma cmucal-sfr.c/cmucal-node.c tables.
 */

#include <linux/clk-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/google,zumapro.h>

#include "clk.h"
#include "clk-exynos-arm64.h"

/* NOTE: Must be equal to the last clock ID in each CMU increased by one. */
#define CLKS_NR_TOP		(CLK_DOUT_CMU_PERIC0_IP + 1)
#define CLKS_NR_PERIC0		(CLK_GOUT_PERIC0_USI0_UART_CLK + 1)

/* ---- CMU_TOP ------------------------------------------------------------ */

/* Register offsets for CMU_TOP (0x26040000) */
#define CLK_CON_MUX_MUX_CLKCMU_PERIC0_IP	0x10ec
#define CLK_CON_MUX_MUX_CLKCMU_PERIC0_NOC	0x10f0
#define CLK_CON_DIV_CLKCMU_PERIC0_IP		0x18e4
#define CLK_CON_DIV_CLKCMU_PERIC0_NOC		0x18e8
#define CLK_CON_GAT_GATE_CLKCMU_PERIC0_IP	0x2110
#define CLK_CON_GAT_GATE_CLKCMU_PERIC0_NOC	0x2114

static const unsigned long top_clk_regs[] __initconst = {
	CLK_CON_MUX_MUX_CLKCMU_PERIC0_IP,
	CLK_CON_MUX_MUX_CLKCMU_PERIC0_NOC,
	CLK_CON_DIV_CLKCMU_PERIC0_IP,
	CLK_CON_DIV_CLKCMU_PERIC0_NOC,
	CLK_CON_GAT_GATE_CLKCMU_PERIC0_IP,
	CLK_CON_GAT_GATE_CLKCMU_PERIC0_NOC,
};

/*
 * These are fixed factors of shared PLLs in the downstream cmucal data. Keep
 * them fixed for now so this driver observes the bootloader-programmed clock
 * tree without programming PLLs.
 */
static const struct samsung_fixed_rate_clock top_fixed_clks[] __initconst = {
	FRATE(CLK_FOUT_SHARED0_D4, "fout_shared0_d4", NULL, 0, 533249984),
	FRATE(CLK_FOUT_SHARED2_D2, "fout_shared2_d2", NULL, 0, 400000000),
	FRATE(CLK_FOUT_SHARED3_D2, "fout_shared3_d2", NULL, 0, 333500000),
	FRATE(CLK_FOUT_SPARE_PLL, "fout_spare_pll", NULL, 0, 2400000000),
};

PNAME(mout_cmu_peric0_p) = {
	"fout_shared0_d4", "fout_shared2_d2",
	"fout_shared3_d2", "fout_spare_pll",
};

static const struct samsung_mux_clock top_mux_clks[] __initconst = {
	MUX(CLK_MOUT_CMU_PERIC0_NOC, "mout_cmu_peric0_noc",
	    mout_cmu_peric0_p, CLK_CON_MUX_MUX_CLKCMU_PERIC0_NOC, 0, 2),
	MUX(CLK_MOUT_CMU_PERIC0_IP, "mout_cmu_peric0_ip",
	    mout_cmu_peric0_p, CLK_CON_MUX_MUX_CLKCMU_PERIC0_IP, 0, 2),
};

static const struct samsung_gate_clock top_gate_clks[] __initconst = {
	/*
	 * Keep both CMU_TOP PERIC0 feeds running continuously. They sit at
	 * the root of every PERIC0 USI's clock chain, and the debug console
	 * UART hangs off the same chain: clk_uart_baud0 reaches
	 * gout_cmu_peric0_ip via mout_peric0_usi0_uart_user, and the UART
	 * pclk reaches gout_cmu_peric0_noc via mout_peric0_noc_user.
	 *
	 * exynos_usi_enable() finishes by calling clk_bulk_disable_unprepare()
	 * on the USI's clocks, so each USI's probe transiently bumps these
	 * feeds' refcount up and then immediately drops it again. Until
	 * samsung_tty probes and pins the chain via CCF, the console is on
	 * earlycon and is not holding it. Without CLK_IS_CRITICAL the
	 * transient takes the refcount to zero, CCF gates the feed, the
	 * UART loses its baud/pclk source, and earlycon's TX-poll spins
	 * forever waiting on a FIFO that can no longer drain.
	 *
	 * Observed on tegu via the m1n1 MMIO trace while bringing up the
	 * USI6 I2C controller: the clk-disable read landed on the IP feed
	 * (CMU_TOP + 0x2110) and no further traced MMIO ever followed.
	 */
	GATE(CLK_GOUT_CMU_PERIC0_NOC, "gout_cmu_peric0_noc",
	     "mout_cmu_peric0_noc", CLK_CON_GAT_GATE_CLKCMU_PERIC0_NOC,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_PERIC0_IP, "gout_cmu_peric0_ip",
	     "mout_cmu_peric0_ip", CLK_CON_GAT_GATE_CLKCMU_PERIC0_IP,
	     21, CLK_IS_CRITICAL, 0),
};

static const struct samsung_div_clock top_div_clks[] __initconst = {
	DIV(CLK_DOUT_CMU_PERIC0_NOC, "dout_cmu_peric0_noc",
	    "gout_cmu_peric0_noc", CLK_CON_DIV_CLKCMU_PERIC0_NOC, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC0_IP, "dout_cmu_peric0_ip",
	    "gout_cmu_peric0_ip", CLK_CON_DIV_CLKCMU_PERIC0_IP, 0, 4),
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
};

static void __init zumapro_cmu_top_init(struct device_node *np)
{
	exynos_arm64_register_cmu(NULL, np, &top_cmu_info);
}

CLK_OF_DECLARE(zumapro_cmu_top, "google,zumapro-cmu-top",
	       zumapro_cmu_top_init);

/* ---- CMU_PERIC0 --------------------------------------------------------- */

/* Register offsets for CMU_PERIC0 (0x10800000) */
#define PLL_CON0_MUX_CLKCMU_PERIC0_NOC_USER			0x0600
#define PLL_CON0_MUX_CLKCMU_PERIC0_USI0_UART_USER		0x0620
#define CLK_CON_DIV_DIV_CLK_PERIC0_USI0_UART			0x1808
#define CLK_CON_GAT_CLK_BLK_PERIC0_UID_PERIC0_CMU_PERIC0_IPCLKPORT_PCLK \
								0x2048
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_RSTNSYNC_CLK_PERIC0_USI0_UART_IPCLKPORT_CLK \
								0x20c8
#define CLK_CON_GAT_CLK_BLK_PERIC0_UID_USI0_UART_IPCLKPORT_IPCLK \
								0x2060
#define CLK_CON_GAT_CLK_BLK_PERIC0_UID_USI0_UART_IPCLKPORT_PCLK \
								0x2064
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_GPIO_PERIC0_IPCLKPORT_PCLK \
								0x20b8
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_SYSREG_PERIC0_IPCLKPORT_PCLK \
								0x20f0

static const unsigned long peric0_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_PERIC0_NOC_USER,
	PLL_CON0_MUX_CLKCMU_PERIC0_USI0_UART_USER,
	CLK_CON_DIV_DIV_CLK_PERIC0_USI0_UART,
	CLK_CON_GAT_CLK_BLK_PERIC0_UID_PERIC0_CMU_PERIC0_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_PERIC0_UID_RSTNSYNC_CLK_PERIC0_USI0_UART_IPCLKPORT_CLK,
	CLK_CON_GAT_CLK_BLK_PERIC0_UID_USI0_UART_IPCLKPORT_IPCLK,
	CLK_CON_GAT_CLK_BLK_PERIC0_UID_USI0_UART_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_PERIC0_UID_GPIO_PERIC0_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_PERIC0_UID_SYSREG_PERIC0_IPCLKPORT_PCLK,
};

PNAME(mout_peric0_noc_user_p) = { "oscclk", "dout_cmu_peric0_noc" };
PNAME(mout_peric0_usi0_uart_user_p) = { "oscclk", "dout_cmu_peric0_ip" };

static const struct samsung_mux_clock peric0_mux_clks[] __initconst = {
	MUX(CLK_MOUT_PERIC0_NOC_USER, "mout_peric0_noc_user",
	    mout_peric0_noc_user_p, PLL_CON0_MUX_CLKCMU_PERIC0_NOC_USER,
	    4, 1),
	MUX(CLK_MOUT_PERIC0_USI0_UART_USER, "mout_peric0_usi0_uart_user",
	    mout_peric0_usi0_uart_user_p,
	    PLL_CON0_MUX_CLKCMU_PERIC0_USI0_UART_USER, 4, 1),
};

static const struct samsung_div_clock peric0_div_clks[] __initconst = {
	DIV(CLK_DOUT_PERIC0_USI0_UART, "dout_peric0_usi0_uart",
	    "mout_peric0_usi0_uart_user",
	    CLK_CON_DIV_DIV_CLK_PERIC0_USI0_UART, 0, 4),
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
