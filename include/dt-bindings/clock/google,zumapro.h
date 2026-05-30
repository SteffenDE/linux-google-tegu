/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Device Tree binding constants for Google Zumapro clock controllers.
 */

#ifndef _DT_BINDINGS_CLOCK_GOOGLE_ZUMAPRO_H
#define _DT_BINDINGS_CLOCK_GOOGLE_ZUMAPRO_H

/* CMU_TOP fixed PLL-derived parents */
#define CLK_FOUT_SHARED0_D4			1
#define CLK_FOUT_SHARED2_D2			2
#define CLK_FOUT_SHARED3_D2			3
#define CLK_FOUT_SPARE_PLL			4
#define CLK_FOUT_SHARED1_D4			17
#define CLK_FOUT_SHARED2_D1			18
#define CLK_FOUT_SHARED3_D1			19

/* CMU_TOP muxes */
#define CLK_MOUT_CMU_PERIC0_NOC		5
#define CLK_MOUT_CMU_PERIC0_IP			6
#define CLK_MOUT_CMU_PERIC1_NOC		11
#define CLK_MOUT_CMU_PERIC1_IP			12
#define CLK_MOUT_CMU_HSI2_MMC_CARD		20
#define CLK_MOUT_CMU_HSI2_NOC			21
#define CLK_MOUT_CMU_HSI2_PCIE			22
#define CLK_MOUT_CMU_HSI2_UFS_EMBD		23

/* CMU_TOP gates */
#define CLK_GOUT_CMU_PERIC0_NOC		7
#define CLK_GOUT_CMU_PERIC0_IP			8
#define CLK_GOUT_CMU_PERIC1_NOC		13
#define CLK_GOUT_CMU_PERIC1_IP			14
#define CLK_GOUT_CMU_HSI2_MMC_CARD		24
#define CLK_GOUT_CMU_HSI2_NOC			25
#define CLK_GOUT_CMU_HSI2_PCIE			26
#define CLK_GOUT_CMU_HSI2_UFS_EMBD		27

/* CMU_TOP dividers */
#define CLK_DOUT_CMU_PERIC0_NOC		9
#define CLK_DOUT_CMU_PERIC0_IP			10
#define CLK_DOUT_CMU_PERIC1_NOC		15
#define CLK_DOUT_CMU_PERIC1_IP			16
#define CLK_DOUT_CMU_HSI2_MMC_CARD		28
#define CLK_DOUT_CMU_HSI2_NOC			29
#define CLK_DOUT_CMU_HSI2_PCIE			30
#define CLK_DOUT_CMU_HSI2_UFS_EMBD		31

/* CMU_TOP HSI0 (USB) feeds.  Only the NOC feed is used: the USB reference is
 * derived from CMU_HSI0's internal PLL_USB, not a CMU_TOP USB32DRD feed (see
 * the CMU_HSI0 block below and the hardware trace in research/NEEDS-HARDWARE.md).
 */
#define CLK_MOUT_CMU_HSI0_NOC			32
#define CLK_GOUT_CMU_HSI0_NOC			34
#define CLK_DOUT_CMU_HSI0_NOC			36

/* CMU_PERIC0 muxes */
#define CLK_MOUT_PERIC0_NOC_USER		1
#define CLK_MOUT_PERIC0_USI0_UART_USER		2

/* CMU_PERIC0 dividers */
#define CLK_DOUT_PERIC0_USI0_UART		3

/* CMU_PERIC0 gates */
#define CLK_GOUT_PERIC0_GPIO_PERIC0_PCLK	4
#define CLK_GOUT_PERIC0_SYSREG_PERIC0_PCLK	5
#define CLK_GOUT_PERIC0_CMU_PERIC0_PCLK		6
#define CLK_GOUT_PERIC0_USI0_UART_IPCLK		7
#define CLK_GOUT_PERIC0_USI0_UART_PCLK		8
#define CLK_GOUT_PERIC0_USI0_UART_CLK		9

/* CMU_PERIC0 USI6 clocks */
#define CLK_MOUT_PERIC0_USI6_USI_USER		10
#define CLK_DOUT_PERIC0_USI6_USI		11
#define CLK_GOUT_PERIC0_USI6_USI_PCLK		12
#define CLK_GOUT_PERIC0_USI6_USI_CLK		13

/* CMU_PERIC1 muxes */
#define CLK_MOUT_PERIC1_NOC_USER		1
#define CLK_MOUT_PERIC1_USI10_USI_USER		2

/* CMU_PERIC1 dividers */
#define CLK_DOUT_PERIC1_USI10_USI		3

/* CMU_PERIC1 gates */
#define CLK_GOUT_PERIC1_GPIO_PERIC1_PCLK	4
#define CLK_GOUT_PERIC1_SYSREG_PERIC1_PCLK	5
#define CLK_GOUT_PERIC1_CMU_PERIC1_PCLK		6
#define CLK_GOUT_PERIC1_USI10_USI_PCLK		7
#define CLK_GOUT_PERIC1_USI10_USI_IPCLK		8
#define CLK_GOUT_PERIC1_USI10_USI_CLK		9

/* CMU_HSI2 muxes */
#define CLK_MOUT_HSI2_MMC_CARD_USER		1
#define CLK_MOUT_HSI2_NOC_USER			2
#define CLK_MOUT_HSI2_PCIE_USER			3
#define CLK_MOUT_HSI2_UFS_EMBD_USER		4

/* CMU_HSI2 gates */
#define CLK_GOUT_HSI2_GPIO_HSI2UFS_PCLK		5
#define CLK_GOUT_HSI2_CMU_HSI2_PCLK		6
#define CLK_GOUT_HSI2_SYSREG_HSI2_PCLK		7
#define CLK_GOUT_HSI2_UFS_EMBD_I_ACLK		8
#define CLK_GOUT_HSI2_UFS_EMBD_I_CLK_UNIPRO	9
#define CLK_GOUT_HSI2_UFS_EMBD_I_FMP_CLK	10
#define CLK_GOUT_HSI2_QE_UFS_EMBD_HSI2_ACLK	11
#define CLK_GOUT_HSI2_QE_UFS_EMBD_HSI2_PCLK	12

/*
 * CMU_HSI0 (USB) clocks.
 *
 * USB is already running at handoff (the device boots over fastboot), so this
 * driver observes the bootloader-programmed tree rather than reprogramming it.
 *
 * A hardware trace (research/NEEDS-HARDWARE.md H1/H3) showed the USB reference
 * does NOT come from the CMU_TOP USB32DRD USER path (that reads 26 MHz and is
 * unused); the internal MUX_CLK_HSI0_USB32DRD selects DIV_CLK_HSI0_USB off the
 * CMU_HSI0-internal PLL_USB (614.4 MHz / 32 = 19.2 MHz).  The eUSB2 PHY "ref"
 * and the DWC3 core "ref" both run at this 19.2 MHz; modelling the wrong rate
 * would make phy-snps-eusb2 program the wrong PLL config and break USB HS.
 *
 * So expose: the NOC USER mux (fabric / link_aclk / phy), DIV_CLK_HSI0_USB (the
 * 19.2 MHz reference), and DIV_CLK_HSI0_EUSB (eUSB control, NOC/2).
 */
#define CLK_MOUT_HSI0_NOC_USER			1
#define CLK_DOUT_HSI0_USB			2
#define CLK_DOUT_HSI0_EUSB			3
#define CLK_FOUT_USB				4

#endif /* _DT_BINDINGS_CLOCK_GOOGLE_ZUMAPRO_H */
