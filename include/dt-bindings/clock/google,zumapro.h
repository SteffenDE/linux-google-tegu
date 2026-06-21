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
#define CLK_FOUT_SHARED0_D3			37
#define CLK_FOUT_SHARED1_D3			38

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

/*
 * CMU_TOP HSI0 PERI feed.  Unlike USB, the BLK_HSI0 USI serial engines (the
 * touchscreen SPI hangs off USI2) are clocked from this CMU_TOP feed via the
 * CMU_HSI0 PERI USER mux.
 */
#define CLK_MOUT_CMU_HSI0_PERI			51
#define CLK_GOUT_CMU_HSI0_PERI			52
#define CLK_DOUT_CMU_HSI0_PERI			53

/*
 * CMU_TOP MISC feed.  Only the NOC (bus) feed is exposed: it is the input the
 * CMU_MISC slice's USER mux selects to clock the Multi-Core Timer (MCT).
 */
#define CLK_MOUT_CMU_MISC_NOC			54
#define CLK_GOUT_CMU_MISC_NOC			55
#define CLK_DOUT_CMU_MISC_NOC			56

/*
 * CMU_TOP display feeds.  These keep the bootloader-owned display clock tree
 * visible to CCF while the proper DECON/DSIM/DPUF drivers are still being
 * brought up.
 */
#define CLK_MOUT_CMU_DPUB_DSIM			39
#define CLK_MOUT_CMU_DPUB_NOC			40
#define CLK_MOUT_CMU_DPUF0_NOC			41
#define CLK_MOUT_CMU_DPUF1_NOC			42
#define CLK_GOUT_CMU_DPUB_DSIM			43
#define CLK_GOUT_CMU_DPUB_NOC			44
#define CLK_GOUT_CMU_DPUF0_NOC			45
#define CLK_GOUT_CMU_DPUF1_NOC			46
#define CLK_DOUT_CMU_DPUB_DSIM			47
#define CLK_DOUT_CMU_DPUB_NOC			48
#define CLK_DOUT_CMU_DPUF0_NOC			49
#define CLK_DOUT_CMU_DPUF1_NOC			50

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
#define CLK_GOUT_HSI2_GPIO_HSI2UFS_QCH		13

/* CMU_HSI2 PCIe GEN3A_1 gates (WiFi root complex) */
#define CLK_GOUT_HSI2_PCIE_GEN3A_1_PHY_REFCLK_IN	14
#define CLK_GOUT_HSI2_PCIE_GEN3A_1_DBI_ACLK		15
#define CLK_GOUT_HSI2_PCIE_GEN3A_1_MSTR_ACLK		16
#define CLK_GOUT_HSI2_PCIE_GEN3A_1_SLV_ACLK		17
#define CLK_GOUT_HSI2_PCIE_GEN3A_1_DRIVER_APB_CLK	18
#define CLK_GOUT_HSI2_PCIE_GEN3A_1_UDBG_APB_PCLK	19
#define CLK_GOUT_HSI2_PCIE_GEN3A_1_PIPE_PAL_APB_PCLK	20
#define CLK_GOUT_HSI2_GPIO_HSI2_PCLK			21
#define CLK_GOUT_HSI2_GPIO_HSI2_QCH			22

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
 * So expose: the NOC USER mux (fabric / bus / phy), DIV_CLK_HSI0_USB (the
 * 19.2 MHz reference), DIV_CLK_HSI0_EUSB (eUSB control, NOC/2), and the
 * USB32DRD link Q-channel gate consumed by the DWC3 wrapper.
 */
#define CLK_MOUT_HSI0_NOC_USER			1
#define CLK_DOUT_HSI0_USB			2
#define CLK_DOUT_HSI0_EUSB			3
#define CLK_FOUT_USB				4

/*
 * CMU_HSI0 USI2 clocks (touchscreen SPI).  The ipclk chain is
 * PERI USER mux -> USI2 mux -> USI2 div -> ipclk gate; the pclk gate sits on
 * the NOC user mux like the other BLK_HSI0 bus clocks.
 */
#define CLK_MOUT_HSI0_PERI_USER			5
#define CLK_MOUT_HSI0_USI2			6
#define CLK_DOUT_HSI0_USI2			7
#define CLK_GOUT_HSI0_USI2_USI_CLK		8
#define CLK_GOUT_HSI0_USI2_USI_PCLK		9

/* USB32DRD link Q-channel gate. */
#define CLK_GOUT_HSI0_USB32DRD_LINK		10

/* CMU_DPUB display backbone clocks */
#define CLK_MOUT_DPUB_DSIM_USER		1
#define CLK_MOUT_DPUB_NOC_USER			2
#define CLK_DOUT_DPUB_NOCP			3
#define CLK_GOUT_DPUB_CMU_DPUB_PCLK		4
#define CLK_GOUT_DPUB_DECON_PCLK		5
#define CLK_GOUT_DPUB_DECON_ACLK		6
#define CLK_GOUT_DPUB_SYSREG_PCLK		7
#define CLK_GOUT_DPUB_DSIM0_ALVCLK		8
#define CLK_GOUT_DPUB_DSIM0_OSCCLK		9

/* CMU_DPUF0 display fetch clocks */
#define CLK_MOUT_DPUF0_NOC_USER		1
#define CLK_DOUT_DPUF0_NOCP			2
#define CLK_GOUT_DPUF0_CMU_DPUF0_PCLK		3
#define CLK_GOUT_DPUF0_DPUF_ACLK		4
#define CLK_GOUT_DPUF0_DPUF_DMA_PCLK		5
#define CLK_GOUT_DPUF0_SYSREG_PCLK		6
#define CLK_GOUT_DPUF0_SYSMMU_S0_CLK		7
#define CLK_GOUT_DPUF0_SYSMMU_PMMU0_CLK		8
#define CLK_GOUT_DPUF0_SYSMMU_PMMU1_CLK		9
#define CLK_GOUT_DPUF0_SRAMC_ACLK		10

/* CMU_DPUF1 display fetch clocks */
#define CLK_MOUT_DPUF1_NOC_USER		1
#define CLK_DOUT_DPUF1_NOCP			2
#define CLK_GOUT_DPUF1_CMU_DPUF1_PCLK		3
#define CLK_GOUT_DPUF1_DPUF_ACLK		4
#define CLK_GOUT_DPUF1_DPUF_DMA_PCLK		5
#define CLK_GOUT_DPUF1_SYSREG_PCLK		6
#define CLK_GOUT_DPUF1_SYSMMU_S0_CLK		7
#define CLK_GOUT_DPUF1_SYSMMU_PMMU0_CLK		8
#define CLK_GOUT_DPUF1_SYSMMU_PMMU1_CLK		9
#define CLK_GOUT_DPUF1_SRAMC_ACLK		10

/*
 * CMU_MISC clocks.  Only the path that clocks the Multi-Core Timer (MCT) is
 * modelled: the NOC USER mux (oscclk / CMU_TOP misc feed), the peripheral
 * divider, and the MCT pclk gate the timer driver enables.
 */
#define CLK_MOUT_MISC_NOC_USER			1
#define CLK_DOUT_MISC_NOCP			2
#define CLK_GOUT_MISC_MCT_PCLK			3

#endif /* _DT_BINDINGS_CLOCK_GOOGLE_ZUMAPRO_H */
