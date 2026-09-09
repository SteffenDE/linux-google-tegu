// SPDX-License-Identifier: GPL-2.0
//
// Power sequences of the Google Zumapro (Tensor G4) power domains.
//
// Copyright (c) 2026 Steffen Deusch
//
// Transcribed from the vendor kernel's pmucal tables for this SoC: for every
// domain a CONFIGURATION write followed by a STATUS poll, the CMU controller
// option written before the domain goes down, and for the GPU pair the PLL,
// mux and Q-channel handling around the transition.  The register names in
// the comments are the vendor's.

#include "exynos-pm-domains.h"

/* The domain's block in the PMU */
#define PD_CONFIGURATION		0x0
#define PD_STATUS			0x4
#define PD_OUT				0x20
#define PD_LOCAL_PWR_CFG		BIT(0)

/* Every CMU */
#define CMU_CONTROLLER_OPTION		0x800
/*
 * Cleared before every power-down in the vendor tables; the clock driver
 * carries the same bit as the memory power-gating enable.
 */
#define CMU_OPT_EN_MEM_PWR_GATING	BIT(24)

/* PLL_CONx and PLL_CON0_MUX_*_USER */
#define PLL_CON0_MUX_SEL		BIT(4)
#define PLL_CON_ENABLE			BIT(31)
#define PLL_CON_LOCKED			BIT(29)

/* QCH_CON_* */
#define QCH_CON_ENABLE			BIT(0)
#define QCH_CON_CLOCK_REQ		BIT(1)
#define QCH_CON_IGNORE_FORCE_PM_EN	BIT(2)
#define QCH_CON_MODE_MASK		GENMASK(2, 0)

/*
 * What every domain does: CONFIGURATION up and wait for STATUS to follow,
 * and the controller option of the domain's CMU down before CONFIGURATION.
 */
static const struct exynos_pd_step zumapro_pd_on[] = {
	PD_WRITE(PMU, PD_CONFIGURATION, PD_LOCAL_PWR_CFG, PD_LOCAL_PWR_CFG),
	PD_WAIT(PMU, PD_STATUS, PD_LOCAL_PWR_CFG, PD_LOCAL_PWR_CFG),
};

static const struct exynos_pd_step zumapro_pd_off[] = {
	PD_WRITE(CMU, CMU_CONTROLLER_OPTION, CMU_OPT_EN_MEM_PWR_GATING, 0),
	PD_WRITE(PMU, PD_CONFIGURATION, PD_LOCAL_PWR_CFG, 0),
	PD_WAIT(PMU, PD_STATUS, PD_LOCAL_PWR_CFG, 0),
};

/*
 * A CMU with a clock provider saves and restores its registers from its
 * runtime PM callbacks or its genpd notifier, so the domains below carry no
 * save list here.  The provider's list is the vendor's for MFC; for DPUB,
 * DPUF0, DPUF1 and HSI1 it is shorter than the vendor's, which does not
 * matter while those domains never power off.
 */

/*
 * G3D, the GPU's top level.  Nothing else drives CMU_G3D, so the domain
 * carries the vendor's save list for it: the two PLLs, the dividers and
 * muxes, the Q-channels, the controller options, the clock-divider step
 * and short-stop settings, and the SYSREG_G3D clock gating words.  The
 * restore also switches the G3DCORE muxes to their PLL side and waits for a
 * PLL that is enabled to lock, in the vendor's order.
 */
static const struct exynos_pd_step zumapro_g3d_save[] = {
	PD_SAVE(CMU, 0x0000),	/* PLL_LOCKTIME_PLL_G3D */
	PD_SAVE(CMU, 0x0008),	/* PLL_LOCKTIME_PLL_G3D_L2 */
	PD_SAVE(CMU, 0x0080),	/* PLL_LOCKTIME_REG_PLL_G3D */
	PD_SAVE(CMU, 0x0084),	/* PLL_LOCKTIME_REG_PLL_G3D_L2 */
	PD_SAVE(CMU, 0x0108),	/* PLL_CON2_PLL_G3D */
	PD_SAVE(CMU, 0x0110),	/* PLL_CON4_PLL_G3D */
	PD_SAVE(CMU, 0x0114),	/* PLL_CON5_PLL_G3D */
	PD_SAVE(CMU, 0x011c),	/* PLL_CON7_PLL_G3D */
	PD_SAVE(CMU, 0x0120),	/* PLL_CON8_PLL_G3D */
	PD_SAVE(CMU, 0x0188),	/* PLL_CON2_PLL_G3D_L2 */
	PD_SAVE(CMU, 0x0190),	/* PLL_CON4_PLL_G3D_L2 */
	PD_SAVE(CMU, 0x0194),	/* PLL_CON5_PLL_G3D_L2 */
	PD_SAVE(CMU, 0x019c),	/* PLL_CON7_PLL_G3D_L2 */
	PD_SAVE(CMU, 0x01a0),	/* PLL_CON8_PLL_G3D_L2 */
	PD_SAVE(CMU, 0x0118),	/* PLL_CON6_PLL_G3D */
	PD_SAVE(CMU, 0x0198),	/* PLL_CON6_PLL_G3D_L2 */
	PD_SAVE(CMU, 0x1800),	/* CLK_CON_DIV_CLK_G3D_ADD_CH_CLK */
	PD_SAVE(CMU, 0x1804),	/* CLK_CON_DIV_DIV_CLK_G3D_L2_GLB */
	PD_SAVE(CMU, 0x1808),	/* CLK_CON_DIV_DIV_CLK_G3D_NOCP */
	PD_SAVE(CMU, 0x180c),	/* CLK_CON_DIV_DIV_CLK_G3D_NOCP_LH */
	PD_SAVE(CMU, 0x1810),	/* CLK_CON_DIV_DIV_CLK_G3D_STACKS */
	PD_SAVE(CMU, 0x1814),	/* CLK_CON_DIV_DIV_CLK_G3D_TOP */
	PD_SAVE(CMU, 0x1000),	/* CLK_CON_MUX_MUX_CLK_G3D_L2_GLB */
	PD_SAVE(CMU, 0x1004),	/* CLK_CON_MUX_MUX_CLK_G3D_STACKS */
	PD_SAVE(CMU, 0x1008),	/* CLK_CON_MUX_MUX_CLK_G3D_TOP */
	PD_SAVE(CMU, 0x0600),	/* PLL_CON0_MUX_CLKCMU_G3D_GLB_USER */
	PD_SAVE(CMU, 0x0610),	/* PLL_CON0_MUX_CLKCMU_G3D_NOCD_USER */
	PD_SAVE(CMU, 0x0620),	/* PLL_CON0_MUX_CLKCMU_G3D_SWITCH_USER */
	/* PLL_CON0_MUX_CLK_G3DCORE_{COREGROUP,STACKS,TOP,TRACE}_USER */
	PD_WRITE(CMU, 0x0630, PLL_CON0_MUX_SEL, PLL_CON0_MUX_SEL),
	PD_WRITE(CMU, 0x0640, PLL_CON0_MUX_SEL, PLL_CON0_MUX_SEL),
	PD_WRITE(CMU, 0x0650, PLL_CON0_MUX_SEL, PLL_CON0_MUX_SEL),
	PD_WRITE(CMU, 0x0660, PLL_CON0_MUX_SEL, PLL_CON0_MUX_SEL),
	/* PLL_CON6_PLL_G3D, PLL_CON6_PLL_G3D_L2: wait for lock if enabled */
	PD_SKIP_IF(CMU, 0x0118, PLL_CON_ENABLE, 0),
	PD_WAIT(CMU, 0x0118, PLL_CON_LOCKED, PLL_CON_LOCKED),
	PD_SKIP_IF(CMU, 0x0198, PLL_CON_ENABLE, 0),
	PD_WAIT(CMU, 0x0198, PLL_CON_LOCKED, PLL_CON_LOCKED),
	PD_SAVE(CMU, 0x010c),	/* PLL_CON3_PLL_G3D */
	PD_SAVE(CMU, 0x018c),	/* PLL_CON3_PLL_G3D_L2 */
	/* PLL_CON3_PLL_G3D, PLL_CON3_PLL_G3D_L2: wait for lock if enabled */
	PD_SKIP_IF(CMU, 0x010c, PLL_CON_ENABLE, 0),
	PD_WAIT(CMU, 0x010c, PLL_CON_LOCKED, PLL_CON_LOCKED),
	PD_SKIP_IF(CMU, 0x018c, PLL_CON_ENABLE, 0),
	PD_WAIT(CMU, 0x018c, PLL_CON_LOCKED, PLL_CON_LOCKED),
	PD_SAVE(CMU, 0x0100),	/* PLL_CON0_PLL_G3D */
	PD_SAVE(CMU, 0x0104),	/* PLL_CON1_PLL_G3D */
	PD_SAVE(CMU, 0x0180),	/* PLL_CON0_PLL_G3D_L2 */
	PD_SAVE(CMU, 0x0184),	/* PLL_CON1_PLL_G3D_L2 */
	PD_SAVE(CMU, 0x3000),	/* DMYQCH_CON_ADD_G3D_QCH */
	PD_SAVE(CMU, 0x3004),	/* DMYQCH_CON_ADM_DAP_G_GPU_QCH */
	PD_SAVE(CMU, 0x3034),	/* QCH_CON_ADD_APBIF_G3D_QCH */
	PD_SAVE(CMU, 0x3038),	/* QCH_CON_D_TZPC_G3D_QCH */
	PD_SAVE(CMU, 0x303c),	/* QCH_CON_G3D_CMU_G3D_QCH */
	PD_SAVE(CMU, 0x3040),	/* QCH_CON_GPC_G3D_QCH */
	PD_SAVE(CMU, 0x3044),	/* QCH_CON_GPU_QCH */
	PD_SAVE(CMU, 0x3048),	/* QCH_CON_LH_ACEL_SI_D0_G3D_QCH */
	PD_SAVE(CMU, 0x304c),	/* QCH_CON_LH_ACEL_SI_D1_G3D_QCH */
	PD_SAVE(CMU, 0x3050),	/* QCH_CON_LH_ACEL_SI_D2_G3D_QCH */
	PD_SAVE(CMU, 0x3054),	/* QCH_CON_LH_ACEL_SI_D3_G3D_QCH */
	PD_SAVE(CMU, 0x3058),	/* QCH_CON_LH_ATB_SI_LT_G3D_CPUCL0_QCH */
	PD_SAVE(CMU, 0x305c),	/* QCH_CON_LH_AXI_MI_IP_G3D_QCH */
	PD_SAVE(CMU, 0x3060),	/* QCH_CON_LH_AXI_MI_P_G3D_CU_QCH */
	PD_SAVE(CMU, 0x3064),	/* QCH_CON_LH_AXI_SI_IP_G3D_QCH */
	PD_SAVE(CMU, 0x3068),	/* QCH_CON_LH_AXI_SI_P_G3D_CU_QCH */
	PD_SAVE(CMU, 0x306c),	/* QCH_CON_PPCFW_G3D0_QCH */
	PD_SAVE(CMU, 0x3070),	/* QCH_CON_PPCFW_G3D1_QCH */
	PD_SAVE(CMU, 0x3074),	/* QCH_CON_PPMU_G3D_D0_QCH */
	PD_SAVE(CMU, 0x3078),	/* QCH_CON_PPMU_G3D_D1_QCH */
	PD_SAVE(CMU, 0x307c),	/* QCH_CON_PPMU_G3D_D2_QCH */
	PD_SAVE(CMU, 0x3080),	/* QCH_CON_PPMU_G3D_D3_QCH */
	PD_SAVE(CMU, 0x3084),	/* QCH_CON_RSTNSYNC_CLK_G3D_DD_QCH */
	PD_SAVE(CMU, 0x3088),	/* QCH_CON_SLH_AXI_MI_P_G3D_QCH */
	PD_SAVE(CMU, 0x308c),	/* QCH_CON_SLH_AXI_SI_D_G3DMMU_QCH */
	PD_SAVE(CMU, 0x3090),	/* QCH_CON_SSMT_G3D0_QCH */
	PD_SAVE(CMU, 0x3094),	/* QCH_CON_SSMT_G3D1_QCH */
	PD_SAVE(CMU, 0x3098),	/* QCH_CON_SSMT_G3D2_QCH */
	PD_SAVE(CMU, 0x309c),	/* QCH_CON_SSMT_G3D3_QCH */
	PD_SAVE(CMU, 0x30a0),	/* QCH_CON_SYSMMU_S0_G3D_QCH_S0 */
	PD_SAVE(CMU, 0x30a4),	/* QCH_CON_SYSMMU_S0_PMMU0_G3D_QCH_S0 */
	PD_SAVE(CMU, 0x30a8),	/* QCH_CON_SYSMMU_S0_PMMU1_G3D_QCH_S0 */
	PD_SAVE(CMU, 0x30ac),	/* QCH_CON_SYSMMU_S0_PMMU2_G3D_QCH_S0 */
	PD_SAVE(CMU, 0x30b0),	/* QCH_CON_SYSMMU_S0_PMMU3_G3D_QCH_S0 */
	PD_SAVE(CMU, 0x30b4),	/* QCH_CON_SYSREG_G3D_QCH */
	PD_SAVE(CMU, 0x30b8),	/* QCH_CON_UASC_G3D_QCH */
	PD_SAVE(CMU, 0x0800),	/* G3D_CMU_G3D_CONTROLLER_OPTION */
	PD_SAVE(CMU, 0x0804),	/* G3D_EMBEDDED_CMU_G3D_CONTROLLER_OPTION */
	PD_SAVE(CMU, 0x0840),	/* G3D_CLKDIVSTEP_CON_HEAVY */
	PD_SAVE(CMU, 0x0844),	/* G3D_CLKDIVSTEP_CON_LIGHT */
	PD_SAVE(CMU, 0x0834),	/* G3D_CLKDIVSTEP_OCP_FLT */
	PD_SAVE(CMU, 0x0838),	/* G3D_CLKDIVSTEP_VDROOP_FLT */
	PD_SAVE(CMU, 0x083c),	/* G3D_CLKDIVSTEP_SMPL_FLT */
	PD_SAVE(CMU, 0x0830),	/* G3D_CLKDIVSTEP */
	PD_SAVE(CMU, 0x0820),	/* G3D_SHORTSTOP */
	PD_SAVE(SYSREG, 0x0104),	/* BUS_COMPONENT_DRCG_EN */
	PD_SAVE(SYSREG, 0x0108),	/* MEMCLK */
};

/*
 * EMBEDDED_G3D, the shader cores, a child of G3D.  Coming up, both PLLs are
 * put back into their running configuration and the G3DCORE muxes switched
 * to them, EMBEDDED_G3D_OUT bit 1 is set through the PMU's set-bit alias
 * and given 280 us before the domain is configured on.  Going down, the GPU
 * Q-channel is told to ignore forced power management while the domain
 * goes off, then returned to a plain clock request, the muxes are switched
 * away from the PLLs and the PLLs parked.
 */
static const struct exynos_pd_step zumapro_g3d_cores_on[] = {
	PD_WRITE(CMU, 0x0104, U32_MAX, 0x38000003),	/* PLL_CON1_PLL_G3D */
	PD_WRITE(CMU, 0x0100, PLL_CON0_MUX_SEL, PLL_CON0_MUX_SEL),	/* PLL_CON0_PLL_G3D */
	PD_WRITE(CMU, 0x0184, U32_MAX, 0x38000003),	/* PLL_CON1_PLL_G3D_L2 */
	PD_WRITE(CMU, 0x0180, PLL_CON0_MUX_SEL, PLL_CON0_MUX_SEL),	/* PLL_CON0_PLL_G3D_L2 */
	/* PLL_CON0_MUX_CLK_G3DCORE_{COREGROUP,STACKS,TOP,TRACE}_USER */
	PD_WRITE(CMU, 0x0630, PLL_CON0_MUX_SEL, PLL_CON0_MUX_SEL),
	PD_WRITE(CMU, 0x0640, PLL_CON0_MUX_SEL, PLL_CON0_MUX_SEL),
	PD_WRITE(CMU, 0x0650, PLL_CON0_MUX_SEL, PLL_CON0_MUX_SEL),
	PD_WRITE(CMU, 0x0660, PLL_CON0_MUX_SEL, PLL_CON0_MUX_SEL),
	PD_SET_BITS(PD_OUT, BIT(1)),			/* EMBEDDED_G3D_OUT */
	PD_DELAY_US(280),
	PD_WRITE(PMU, PD_CONFIGURATION, PD_LOCAL_PWR_CFG, PD_LOCAL_PWR_CFG),
	PD_WAIT(PMU, PD_STATUS, PD_LOCAL_PWR_CFG, PD_LOCAL_PWR_CFG),
};

static const struct exynos_pd_step zumapro_g3d_cores_off[] = {
	PD_WRITE(CMU, 0x3044, QCH_CON_MODE_MASK,	/* QCH_CON_GPU_QCH */
		 QCH_CON_CLOCK_REQ | QCH_CON_IGNORE_FORCE_PM_EN),
	PD_WRITE(PMU, PD_CONFIGURATION, PD_LOCAL_PWR_CFG, 0),
	PD_WAIT(PMU, PD_STATUS, PD_LOCAL_PWR_CFG, 0),
	PD_WRITE(CMU, 0x3044, QCH_CON_MODE_MASK, QCH_CON_CLOCK_REQ),
	/* PLL_CON0_MUX_CLK_G3DCORE_{COREGROUP,STACKS,TOP,TRACE}_USER */
	PD_WRITE(CMU, 0x0630, PLL_CON0_MUX_SEL, 0),
	PD_WRITE(CMU, 0x0640, PLL_CON0_MUX_SEL, 0),
	PD_WRITE(CMU, 0x0650, PLL_CON0_MUX_SEL, 0),
	PD_WRITE(CMU, 0x0660, PLL_CON0_MUX_SEL, 0),
	PD_WRITE(CMU, 0x0100, PLL_CON0_MUX_SEL, 0),	/* PLL_CON0_PLL_G3D */
	PD_WRITE(CMU, 0x0104, U32_MAX, 0x18000001),	/* PLL_CON1_PLL_G3D */
	PD_WRITE(CMU, 0x0180, PLL_CON0_MUX_SEL, 0),	/* PLL_CON0_PLL_G3D_L2 */
	PD_WRITE(CMU, 0x0184, U32_MAX, 0x18000001),	/* PLL_CON1_PLL_G3D_L2 */
};

const struct exynos_pd_sequences zumapro_pd_sequences[] = {
	/* G3D */
	EXYNOS_PD_SEQUENCES(0x15461a80, zumapro_pd_on, zumapro_g3d_save,
			    zumapro_pd_off),
	/* EMBEDDED_G3D */
	EXYNOS_PD_SEQUENCES_NO_SAVE(0x15461c80, zumapro_g3d_cores_on,
				    zumapro_g3d_cores_off),
	/* DPUB */
	EXYNOS_PD_SEQUENCES_NO_SAVE(0x15461d00, zumapro_pd_on, zumapro_pd_off),
	/* DPUF0 */
	EXYNOS_PD_SEQUENCES_NO_SAVE(0x15461d80, zumapro_pd_on, zumapro_pd_off),
	/* DPUF1 */
	EXYNOS_PD_SEQUENCES_NO_SAVE(0x15461e00, zumapro_pd_on, zumapro_pd_off),
	/* MFC */
	EXYNOS_PD_SEQUENCES_NO_SAVE(0x15462080, zumapro_pd_on, zumapro_pd_off),
	/* HSI1 */
	EXYNOS_PD_SEQUENCES_NO_SAVE(0x15462b00, zumapro_pd_on, zumapro_pd_off),
	{ /* sentinel */ },
};
