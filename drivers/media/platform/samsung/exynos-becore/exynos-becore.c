// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Zumapro camera back-end core bring-up driver
 *
 * The register sequences here are the common processor lifecycle observed on
 * the Pixel 9a vendor stack.  The debugfs diagnostic accepts a structurally
 * fixed, relocatable RGBP/YUVP program for offline bring-up; it is deliberately
 * not a camera ABI.  Separate GTNR-startup and MCSC recipes can be normalized
 * into dormant command lists while those downstream stages are brought up.
 * Powering a block down is safe only after every owned processor has accepted
 * a software reset.
 */

#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/log2.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/refcount.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <media/exynos-becore.h>
#include <media/media-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#include "exynos-becore-recipe.h"
#include "exynos-becore-gtnr-recipe.h"
#include "exynos-becore-mcsc-recipe.h"

#define BECORE_GLOBAL_ENABLE		0x0000
#define BECORE_GLOBAL_ENABLE_CLEAR	0x0008
#define BECORE_SW_RESET			0x0010
#define BECORE_SET_CTRL			0x0030
#define BECORE_COMMAND_Q_STOP_ON_FRAME	0x003c
#define BECORE_FRO_GLOBAL_ENABLE		0x006c

#define BECORE_CMDQ_QUE_CMD_L		0x0400
#define BECORE_CMDQ_QUE_CMD_M		0x0404
#define BECORE_CMDQ_QUE_CMD_H		0x0408
#define BECORE_CMDQ_QUE_CMD_START	0x040c
#define BECORE_CMDQ_ADD_TO_QUEUE_0	0x0484
#define BECORE_CMDQ_INT_STATUS		0x04f8
#define BECORE_CMDQ_INT_CLEAR		0x04fc
#define BECORE_CMDQ_INT_ENABLE		0x04f4
#define BECORE_CMDQ_ENABLE		0x0500

#define BECORE_INT0_ENABLE		0x0804
#define BECORE_INT0_STATUS		0x0808
#define BECORE_INT0_CLEAR		0x080c
#define BECORE_INT1_ENABLE		0x0814
#define BECORE_INT1_STATUS		0x0818
#define BECORE_INT1_CLEAR		0x081c

#define BECORE_C_LOADER_ENABLE		0x1000
#define BECORE_C_LOADER_MODE		0x1004
#define BECORE_STAT_RDMACL_EN		0x1600
#define BECORE_RGBP_C_LOADER_ENABLE	0x4000

#define BECORE_RESET_TIMEOUT_US		1000

#define BECORE_INT_FRAME_END		BIT(1)
#define BECORE_INT_CMDQ_HOLD		BIT(2)
#define BECORE_INT_EXPECTED		(BECORE_INT_FRAME_END | BECORE_INT_CMDQ_HOLD)
#define BECORE_YUVP_STAGE_BLOCKS	(BIT(BECORE_RGBP) | BIT(BECORE_YUVP))

#define BECORE_CMDQ_HEADER_BYTES		16
#define BECORE_CMDQ_PAYLOAD_BYTES	64
#define BECORE_CMDQ_MODE		0x9000

/* Fixed neutral LTM policy for the proven 4000x3000 processing profile. */
#define BECORE_LTM_GRID_ROW_BYTES	0x800
#define BECORE_LTM_GRID_ROWS		48
#define BECORE_LTM_GRID_CELL_BYTES	0x100
#define BECORE_LTM_GRID_WIDTH_CELLS	4
#define BECORE_LTM_GRID_HEIGHT_CELLS	24
#define BECORE_LTM_UNITY_Q14		BIT(14)
#define BECORE_GRID_SIZE			(BECORE_LTM_GRID_ROW_BYTES * \
					 BECORE_LTM_GRID_ROWS)
#define BECORE_RUN_TIMEOUT_MS		1000

#define BECORE_RGBP_PHYS_BASE		0x1c440000
#define BECORE_YUVP_PHYS_BASE		0x1c840000
#define BECORE_GTNR_PHYS_BASE		0x1cc40000
#define BECORE_MCSC_PHYS_BASE		0x1d040000

#define BECORE_RGBP_INPUT_IMAGE_REG	(BECORE_RGBP_PHYS_BASE + 0x1c50)
#define BECORE_RGBP_INPUT_HEADER_REG	(BECORE_RGBP_PHYS_BASE + 0x1d10)
#define BECORE_RGBP_INPUT_ENABLE_REG	(BECORE_RGBP_PHYS_BASE + 0x1c00)
#define BECORE_RGBP_INPUT_COMP_REG	(BECORE_RGBP_PHYS_BASE + 0x1c04)
#define BECORE_RGBP_INPUT_FORMAT_REG	(BECORE_RGBP_PHYS_BASE + 0x1c10)
/*
 * RGBP's chain geometry and the crop that narrows it, named from Samsung
 * RGBP v1.20. The Bayer input is the sensor's full 4208 x 3120 and RGBP hands
 * 4160 x 3120 to YUVP, so DMSCCROP removes 48 columns; the captured start of
 * (24, 0) is exactly centred. SC downstream of the crop then runs at unity,
 * which is what makes its x8/8 filter coefficients correct and lets them stay
 * fixed.
 */
#define BECORE_RGBP_CHAIN_SRC_SIZE_REG	(BECORE_RGBP_PHYS_BASE + 0x0200)
#define BECORE_RGBP_CHAIN_DST_SIZE_REG	(BECORE_RGBP_PHYS_BASE + 0x0204)
#define BECORE_RGBP_CROP_START_REG	(BECORE_RGBP_PHYS_BASE + 0x0234)
#define BECORE_RGBP_CROP_SIZE_REG	(BECORE_RGBP_PHYS_BASE + 0x0238)
#define BECORE_RGBP_SC_DST_SIZE_REG	(BECORE_RGBP_PHYS_BASE + 0x441c)
#define BECORE_RGBP_SC_H_RATIO_REG	(BECORE_RGBP_PHYS_BASE + 0x4420)
#define BECORE_RGBP_SC_V_RATIO_REG	(BECORE_RGBP_PHYS_BASE + 0x4424)
/*
 * Ten registers that say which RGBP blocks run, named from Samsung RGBP v1.20.
 * None needs a value from the capture: an enable the program leaves clear is
 * zero and an asserted bypass is one.
 *
 * OTF_CROP_CTRL is the one that reads backwards. Its only field is
 * RGB_DMSCCROP_BYPASS, whose vendor default is 1, and the capture clears it --
 * so zero here is what keeps the 48-column demosaic crop above *running*.
 * Do not fold it in with the asserted bypasses.
 *
 * UPSC_CTRL0 is a control word rather than a bare enable: bit 0 enables, bit 8
 * bypasses and two more disable clock gates. Zero leaves the upscaler neither
 * enabled nor bypassed, which is what the capture does.
 *
 * DECOMP's frame size is not a tuning either. It is the Bayer input's, packed
 * with the height in the high half: RGBP v1.20 gives DECOMP width bits [13:0]
 * and height bits [29:16], the other way round from CHAIN_SRC_IMG_SIZE.
 */
#define BECORE_RGBP_CINFIFO_FRAME_IN_REG (BECORE_RGBP_PHYS_BASE + 0x0084)
#define BECORE_RGBP_SATFLAG_ENABLE_REG	(BECORE_RGBP_PHYS_BASE + 0x0218)
#define BECORE_RGBP_DMSCCROP_BYPASS_REG	(BECORE_RGBP_PHYS_BASE + 0x0230)
#define BECORE_RGBP_WDMADECOMP_EN_REG	(BECORE_RGBP_PHYS_BASE + 0x2000)
#define BECORE_RGBP_WDMAY_EN_REG	(BECORE_RGBP_PHYS_BASE + 0x2400)
#define BECORE_RGBP_WDMAUV_EN_REG	(BECORE_RGBP_PHYS_BASE + 0x2600)
#define BECORE_RGBP_DECOMP_BYPASS_REG	(BECORE_RGBP_PHYS_BASE + 0x3e00)
#define BECORE_RGBP_DECOMP_SIZE_REG	(BECORE_RGBP_PHYS_BASE + 0x3e08)
#define BECORE_RGBP_GAMMALR_BYPASS_REG	(BECORE_RGBP_PHYS_BASE + 0x4600)
#define BECORE_RGBP_UPSC_CTRL0_REG	(BECORE_RGBP_PHYS_BASE + 0x4800)
#define BECORE_RGBP_GAMMAHR_BYPASS_REG	(BECORE_RGBP_PHYS_BASE + 0x4a00)
/*
 * RGBP's global tone map, named from Samsung RGBP v1.20. The block runs, but
 * the curve it runs is an exact identity: at every one of its 64 knots the
 * captured output is the input times 32. That factor is the block's own field
 * widths -- v1.20 gives its input knots 13 bits and its output values 18, and
 * 18 - 13 is 5 -- so times 32 is this block's unity, and there is no tuning
 * here to recover. What is left is the knot grid, a description of where the
 * curve is sampled, and the luma weights, which are BT.601's.
 *
 * IN_POINTS packs two knots per register with the even one in the low half,
 * which the vendor's own field descriptors state; OUT_POINTS needs a register
 * each because 18 bits do not fit beside anything.
 */
/*
 * Three RGBP blocks and one YUVP register whose values are named constants
 * rather than anyone's tuning.
 *
 * RGB_RGBTOYUV is BT.601 at Q13, full range rather than studio, and it is
 * column-major by input channel -- (Y, U, V) of R, then of G, then of B. Read
 * row-major, six of the nine coefficients look wrong. The coefficients are
 * kept as exact rationals of Kr = 299/1000 and Kb = 114/1000 so that no float
 * reaches a register, and Q13 is deliberately not twice the front end's Q12
 * copy: the two are independent roundings of the same reals.
 *
 * YUV444TO422 is [1, 2, 1] / 4 scaled by 32, the classic chroma decimation
 * filter, written unconditionally with no tuning import anywhere.
 *
 * BYR_DNS's binning is Q10 unity -- this path does not bin -- and its radial
 * centre is the sensor's full array halved and negated. Samsung's own
 * rgbp_hw_s_dns_size() writes -(full_width >> 1 & ~1) plus the crop as an
 * offset; the two agree exactly while the crop is centred, which mainline's
 * is, and becore_rgbp_input is the full array. A crop that was *not* centred
 * would need the offset term as well, so this is a statement about the current
 * geometry rather than a general derivation. The noise curve above it is real
 * tuning and stays.
 *
 * SHARPENHANCER's LPF_NORM is log2 of its three low-pass kernels' sums packed
 * at bits 0, 8 and 16 -- 16, 512 and 4096, which is what the captured taps
 * really sum to. Note what is and is not checked: the taps themselves stay in
 * the recipe and are not read here, so the power-of-two test below is a
 * property of these three constants and not of the kernels. Deriving the sums
 * from the tap registers would make it a real check, and would want them out
 * of the recipe first.
 */
/*
 * Two blocks whose captured words are literal constants rather than a scene.
 *
 * MCSC's DJAG runs at a neutral profile, and the thirteen words stated here
 * have three separate provenances -- worth keeping apart, because they are not
 * equally strong:
 *
 *  - Seven are Samsung's init_djag_cfgs in is-hw-djag-v2.c *and* the POR reset
 *    value is-sfr-mcsc-v10_1.h gives for the same field: the three LFSR seeds,
 *    the two dither-ramp words, the saturation/dither thresholds and the
 *    coring threshold. Two independent sources agreeing is the strongest case
 *    in this file.
 *  - Three are POR only, because init_djag_cfgs has no such field: both
 *    pre-scaler phase offsets and the round mode. Samsung's own code writes
 *    them as hardcoded literals rather than from a setfile.
 *  - Three are neither, and are stated from what the block is being asked to
 *    do. CTRL is 0x403 because is_scaler_set_djag_enable() sets exactly bits
 *    0, 1 and 10 from one enable and every other bit is POR-clear: DJAG, its
 *    pre-scaler and EZ post are on. RECOM_CTRL and RECOM_WEIGHT are zero
 *    because the detail-restoration sub-block is off -- and it is off by two
 *    independent signals, its control clear *and* its weight zero where POR is
 *    0x400. Two signals rather than one is what makes that a fact rather than
 *    a hypothesis; a lone enable bit is exactly what misled us about YUVP's
 *    tone mapping.
 *
 * What stays in the recipe is not all "differs from the profile". The
 * shooting-detection thresholds, the cross-filter weights, CP_ARBI's mode, the
 * dither white/black guard band and RECOM's biquad shift do differ, and they
 * are keyed on a scaling ratio we hold one row of. But six of RECOM's radial
 * registers are already at POR and are left alone only because a disabled
 * sub-block's radial configuration is not worth a claim.
 *
 * RGBP's DMSC is a real demosaic and most of it is tuning. Fourteen registers
 * are literals GetDefaultDmsc writes *after* the tuning path has run, so they
 * do not vary with the scene. Three cautions:
 *
 *  - Unlike DJAG these are the vendor's compiled-in defaults and not a
 *    hardware reset state, since RGBP v1.20 publishes no POR values. They are
 *    a bring-up default under ADR 0009, not a derivation.
 *  - GetDefaultDmsc writes 23 such literals, not fourteen. The rest are left
 *    in the recipe; this is a conservative subset, not the whole set.
 *  - Every one is a read-modify-write. For thirteen the preserved bits are
 *    reserved and the literal determines every defined field. BASE_CONFIG is
 *    the exception: it preserves bit 0 and bit 16, and bit 16 is a real field,
 *    SKIP_BYR2RGB_EN. Its captured 0x36a therefore also asserts that the
 *    tuning path left those two clear.
 *
 * EDGE_DESAT_RED_PRESERVE_GAIN looks like it belongs to that group and does
 * not: TranslateDmsc computes it as clamp(f * 1023, 0, 0x3ff) into a 10-bit
 * field, so it is tuning that happens to be 0x100 here. It stays in the
 * recipe.
 *
 * Watch the name: GetDefaultDmsc(DmscRgbpOutput&) is a different function from
 * the front end's, which takes a DmscOutput& and programs ISPFE.
 */
/*
 * RGBP's BYR_DNS and YUVP's YUVNR carry the same object: an eight-knot
 * piecewise-linear curve of noise standard deviation against pixel level, one
 * curve for luma and one for chroma. The knots are genuine tuning and stay in
 * the recipe. What follows from them is the eight slopes, the shift they are
 * taken at, and -- because the chroma curve is measured on the same domain as
 * the luma one -- the chroma domain itself.
 *
 * A slope is (dY << 11) / dX, and the eighth field repeats the seventh because
 * there are eight fields for seven segments. The trap is the rounding: **DNS
 * rounds to nearest and YUVNR truncates**. Both were verified on both channels
 * of both blocks. Using one rule for the other block still reproduces every
 * exact division and misses by one everywhere else, so a spot check on a
 * couple of knots passes and the curve is quietly wrong in between.
 *
 * These are the first values that are not a function of the geometry or of a
 * constant, but of the block's own neighbouring registers, which is why
 * becore_recipe_fixed_value() exists.
 */
#define BECORE_RGBP_DNS_X_G_REG		(BECORE_RGBP_DNS_BASE + 0x110)
#define BECORE_RGBP_DNS_Y_G_REG		(BECORE_RGBP_DNS_BASE + 0x120)
#define BECORE_RGBP_DNS_SLOPE_G_REG	(BECORE_RGBP_DNS_BASE + 0x130)
#define BECORE_RGBP_DNS_SHIFT_G_REG	(BECORE_RGBP_DNS_BASE + 0x140)
#define BECORE_RGBP_DNS_X_RB_REG	(BECORE_RGBP_DNS_BASE + 0x144)
#define BECORE_RGBP_DNS_Y_RB_REG	(BECORE_RGBP_DNS_BASE + 0x154)
#define BECORE_RGBP_DNS_SLOPE_RB_REG	(BECORE_RGBP_DNS_BASE + 0x164)
#define BECORE_RGBP_DNS_SHIFT_RB_REG	(BECORE_RGBP_DNS_BASE + 0x174)
#define BECORE_YUVP_NR_BASE		(BECORE_YUVP_PHYS_BASE + 0x3000)
#define BECORE_YUVP_NR_X_Y_REG		(BECORE_YUVP_NR_BASE + 0x220)
#define BECORE_YUVP_NR_Y_Y_REG		(BECORE_YUVP_NR_BASE + 0x230)
#define BECORE_YUVP_NR_SLOPE_Y_REG	(BECORE_YUVP_NR_BASE + 0x240)
#define BECORE_YUVP_NR_SHIFT_Y_REG	(BECORE_YUVP_NR_BASE + 0x260)
#define BECORE_YUVP_NR_X_UV_REG		(BECORE_YUVP_NR_BASE + 0x274)
#define BECORE_YUVP_NR_Y_UV_REG		(BECORE_YUVP_NR_BASE + 0x284)
#define BECORE_YUVP_NR_SLOPE_UV_REG	(BECORE_YUVP_NR_BASE + 0x2a4)
#define BECORE_YUVP_NR_SHIFT_UV_REG	(BECORE_YUVP_NR_BASE + 0x2b4)
#define BECORE_NOISE_KNOTS		8
#define BECORE_NOISE_TABLE_REGS		(BECORE_NOISE_KNOTS / 2)
#define BECORE_NOISE_TABLE_LAST		((BECORE_NOISE_TABLE_REGS - 1) * 4)
#define BECORE_NOISE_SLOPE_SHIFT	11
#define BECORE_NOISE_SLOPE_MASK		GENMASK(12, 0)
#define BECORE_NOISE_SHIFT_NIBBLES	8
/*
 * A scaler that starts on a pixel. RGBP's SC, MCSC's POLY_SC0 and its POST_PC0
 * chroma converter each put two 20-bit init phase offsets at the same place in
 * their register map, and all six words are the field table's POR zero: no
 * sub-pixel origin. Nothing here is scene-dependent, and a sub-pixel origin
 * would need a reason none of these blocks has at this geometry -- though note
 * that is a single-geometry observation, since POLY_SC0 and POST_PC0 run at
 * unity here and DJAG does the scaling.
 *
 * The two MCSC blocks put a round-mode bit after the offsets, also at POR, and
 * those are stated too. RGBP's scaler does not: its round mode is bit 0 of
 * YUV_SC_CTRL1 two registers *earlier*, its POR is zero, and the capture sets
 * it -- so RGBP rounds against the reset value and that word stays in the
 * recipe. RGBP has no register at +0x4410 at all.
 *
 * The rest of what MCSC still replayed above its scalers is the shape of the
 * job rather than a value: it reads memory, not an OTF stream, so CINFIFO and
 * both IP_USE gates are clear and INPUT_TYPE is memory; it drives one output,
 * so the four other WDMA channels and the HF statistics RDMA are off; and the
 * raster it reads is YUVP's output, which the driver already describes.
 */
#define BECORE_RGBP_SC_PHASE_FIRST	(BECORE_RGBP_PHYS_BASE + 0x4408)
#define BECORE_RGBP_SC_PHASE_LAST	(BECORE_RGBP_PHYS_BASE + 0x440c)
#define BECORE_MCSC_SC0_PHASE_FIRST	(BECORE_MCSC_PHYS_BASE + 0x5018)
#define BECORE_MCSC_PC0_PHASE_FIRST	(BECORE_MCSC_PHYS_BASE + 0x6014)
/* Two offsets everywhere; the MCSC blocks add a round mode after them. */
#define BECORE_SCALER_PHASE_LAST	0x08
#define BECORE_MCSC_OTF_GATE_FIRST	(BECORE_MCSC_PHYS_BASE + 0x0080)
#define BECORE_MCSC_OTF_GATE_LAST	(BECORE_MCSC_PHYS_BASE + 0x0084)
#define BECORE_MCSC_INPUT_TYPE_REG	(BECORE_MCSC_PHYS_BASE + 0x0200)
#define BECORE_MCSC_IN_WIDTH_REG	(BECORE_MCSC_PHYS_BASE + 0x0210)
#define BECORE_MCSC_IN_HEIGHT_REG	(BECORE_MCSC_PHYS_BASE + 0x0214)
#define BECORE_MCSC_CINFIFO_FIRST	(BECORE_MCSC_PHYS_BASE + 0x1000)
#define BECORE_MCSC_CINFIFO_LAST	(BECORE_MCSC_PHYS_BASE + 0x1004)
#define BECORE_MCSC_STAT_RDMA_FIRST	(BECORE_MCSC_PHYS_BASE + 0x1a00)
#define BECORE_MCSC_STAT_RDMA_LAST	(BECORE_MCSC_PHYS_BASE + 0x1a04)
#define BECORE_MCSC_WDMA_W1_FIRST	(BECORE_MCSC_PHYS_BASE + 0x2200)
#define BECORE_MCSC_WDMA_W2_FIRST	(BECORE_MCSC_PHYS_BASE + 0x2400)
#define BECORE_MCSC_WDMA_W3_FIRST	(BECORE_MCSC_PHYS_BASE + 0x2600)
#define BECORE_MCSC_WDMA_W4_FIRST	(BECORE_MCSC_PHYS_BASE + 0x2800)
/* Each unused channel is quiesced by its enable and its compression control. */
#define BECORE_MCSC_WDMA_OFF_LAST	0x04
#define BECORE_MCSC_DJAG_BASE		(BECORE_MCSC_PHYS_BASE + 0x4000)
#define BECORE_MCSC_DJAG_CTRL_REG	(BECORE_MCSC_DJAG_BASE + 0x000)
#define BECORE_MCSC_DJAG_PS_FIRST	(BECORE_MCSC_DJAG_BASE + 0x01c)
#define BECORE_MCSC_DJAG_PS_LAST	(BECORE_MCSC_DJAG_BASE + 0x024)
#define BECORE_MCSC_DJAG_TUNE_FIRST	(BECORE_MCSC_DJAG_BASE + 0x050)
#define BECORE_MCSC_DJAG_TUNE_LAST	(BECORE_MCSC_DJAG_BASE + 0x068)
#define BECORE_MCSC_DJAG_RECOM_CTRL_REG	(BECORE_MCSC_DJAG_BASE + 0x080)
#define BECORE_MCSC_DJAG_RECOM_WEIGHT_REG (BECORE_MCSC_DJAG_BASE + 0x088)
#define BECORE_DJAG_DITHER_FIELD_BITS	6
#define BECORE_DJAG_SAT_CTRL		5
#define BECORE_DJAG_DITHER_THRES	5
#define BECORE_DJAG_DITHER_THRES_SHIFT	14
#define BECORE_DJAG_CP_HF_THRES		40
#define BECORE_RGBP_DMSC_BASE		(BECORE_RGBP_PHYS_BASE + 0x3000)
#define BECORE_RGBP_DNS_BASE		(BECORE_RGBP_PHYS_BASE + 0x3000)
#define BECORE_RGBP_DNS_BINNING_REG	(BECORE_RGBP_DNS_BASE + 0x1a4)
#define BECORE_RGBP_DNS_CENTRE_REG	(BECORE_RGBP_DNS_BASE + 0x1c0)
#define BECORE_RGBP_DNS_BINNING_UNITY	1024	/* Q10 */
#define BECORE_RGBP_DNS_CENTRE_MASK	GENMASK(14, 0)
#define BECORE_RGBP_CSC_BASE		(BECORE_RGBP_PHYS_BASE + 0x3b00)
/* YUVP carries the same twenty words, bit for bit, 0x100 lower. */
#define BECORE_YUVP_CSC_BASE		(BECORE_YUVP_PHYS_BASE + 0x3a00)
#define BECORE_YUVP_CSC_FIRST		(BECORE_YUVP_CSC_BASE + 0x00)
#define BECORE_YUVP_CSC_LAST		(BECORE_YUVP_CSC_BASE + 0x4c)
#define BECORE_RGBP_CSC_FIRST		(BECORE_RGBP_CSC_BASE + 0x00)
#define BECORE_RGBP_CSC_LAST		(BECORE_RGBP_CSC_BASE + 0x4c)
#define BECORE_RGBP_CSC_Q		13
#define BECORE_RGBP_CSC_FIELD_MASK	GENMASK(13, 0)
#define BECORE_RGBP_CSC_MAX		0xfff	/* full range, not studio */
#define BECORE_RGBP_CSC_CHROMA_OFFSET	0x800
#define BECORE_RGBP_CHROMA_LPF_BASE	(BECORE_RGBP_PHYS_BASE + 0x3c00)
#define BECORE_RGBP_CHROMA_LPF_CTRL_REG	(BECORE_RGBP_CHROMA_LPF_BASE + 0x00)
#define BECORE_RGBP_CHROMA_LPF_FIRST	(BECORE_RGBP_CHROMA_LPF_BASE + 0x08)
#define BECORE_RGBP_CHROMA_LPF_LAST	(BECORE_RGBP_CHROMA_LPF_BASE + 0x0c)
#define BECORE_YUVP_LPF_NORM_REG	(BECORE_YUVP_PHYS_BASE + 0x5150)
/*
 * RGBP's forward gamma is not a tuning curve: at all 65 of its knots the
 * captured output is round(sqrt(x) * 4096) on a 0..4096 input, an exact
 * gamma-2.0 encode. It is there to move linear light into a gamma domain for
 * the blocks downstream -- YUVP's DEGAMMARGB undoes it with an approximate
 * square, and YUVP's own forward gamma is where the creative tone curve lives.
 * So this block needs no values from anywhere.
 *
 * The grid it is sampled on is the same 65-point grid the vendor's tone curve
 * uses -- 8 steps of 8, 12 of 16, 8 of 32, 16 of 64 and 20 of 128 at Q12,
 * tiling 0..4096 exactly and finest near black, where a square root moves
 * fastest. It is a sampling choice, not a rendering one, and it is not the
 * hardware's reset grid, which is coarser below 64.
 *
 * Two shapes to know. The table has a genuine four-register hole between
 * logical points 23 and 24 -- 0x37ec jumps to 0x3800 -- so it is written out
 * as two runs rather than walked by stride. And the 65th knot of each table
 * would need 1 << 12 exactly, one past its 12-bit field, so the vendor stores
 * its distance from the 64th in the register after the table, with the sign in
 * a _DELTA_SIGN that a rising curve never needs.
 */
#define BECORE_RGBP_GAMMA_BASE		(BECORE_RGBP_PHYS_BASE + 0x3600)
#define BECORE_RGBP_GAMMA_CTRL_FIRST	(BECORE_RGBP_GAMMA_BASE + 0x000)
#define BECORE_RGBP_GAMMA_CTRL_LAST	(BECORE_RGBP_GAMMA_BASE + 0x004)
#define BECORE_RGBP_GAMMA_TBL_FIRST	(BECORE_RGBP_GAMMA_BASE + 0x00c)
#define BECORE_RGBP_GAMMA_TBL_LAST	(BECORE_RGBP_GAMMA_BASE + 0x08c)
#define BECORE_RGBP_GAMMA_X_LOW_FIRST	(BECORE_RGBP_GAMMA_BASE + 0x1c0)
#define BECORE_RGBP_GAMMA_X_LOW_LAST	(BECORE_RGBP_GAMMA_BASE + 0x1ec)
#define BECORE_RGBP_GAMMA_X_HIGH_FIRST	(BECORE_RGBP_GAMMA_BASE + 0x200)
#define BECORE_RGBP_GAMMA_X_HIGH_LAST	(BECORE_RGBP_GAMMA_BASE + 0x254)
#define BECORE_RGBP_GAMMA_KNOTS		65
#define BECORE_RGBP_GAMMA_SEGMENTS	(BECORE_RGBP_GAMMA_KNOTS - 1)
#define BECORE_RGBP_GAMMA_Q		12
/* The grid is stored at Q12; the block is told to meet its wider input. */
#define BECORE_RGBP_GAMMA_X_LSHIFT	5
#define BECORE_RGBP_GTM_BASE		(BECORE_RGBP_PHYS_BASE + 0x3900)
#define BECORE_RGBP_GTM_LAST		(BECORE_RGBP_PHYS_BASE + 0x3a94)
#define BECORE_RGBP_GTM_BYPASS		0x000
#define BECORE_RGBP_GTM_GAIN_MODE_EN	0x004
#define BECORE_RGBP_GTM_IN_POINTS	0x008
#define BECORE_RGBP_GTM_OUT_POINTS	0x088
#define BECORE_RGBP_GTM_Y_WEIGHT_0	0x188
#define BECORE_RGBP_GTM_Y_WEIGHT_1	0x18c
#define BECORE_RGBP_GTM_V_BLEND_RATIO	0x190
#define BECORE_RGBP_GTM_INPUT_RSHIFT	0x194
#define BECORE_RGBP_GTM_KNOTS		64
#define BECORE_RGBP_GTM_OUT_SHIFT	5
/* BT.601 luma, Q8: the three sum to 256. */
#define BECORE_RGBP_GTM_Y_WEIGHT_R	77
#define BECORE_RGBP_GTM_Y_WEIGHT_G	150
#define BECORE_RGBP_GTM_Y_WEIGHT_B	29
#define BECORE_RGBP_INPUT_WIDTH_REG	(BECORE_RGBP_PHYS_BASE + 0x1c20)
#define BECORE_RGBP_INPUT_HEIGHT_REG	(BECORE_RGBP_PHYS_BASE + 0x1c24)
#define BECORE_RGBP_INPUT_STRIDE_REG	(BECORE_RGBP_PHYS_BASE + 0x1c28)
#define BECORE_RGBP_INPUT_HEADER_STRIDE_REG \
	(BECORE_RGBP_PHYS_BASE + 0x1c34)
#define BECORE_RGBP_INPUT_BUSINFO_REG	(BECORE_RGBP_PHYS_BASE + 0x1c4c)
/*
 * Two gates YUVP leaves in a state that does not depend on the scene, named
 * from Samsung YUVP v1.20: DTP asserts its own BYPASS -- it is a test-pattern
 * generator, and nothing wants one -- and COUTFIFO0 is left disabled.
 *
 * The third candidate was 0x6000..0x65fc, which Samsung's tables call DRCDIST
 * and whose first register reads as RGB_DRCDIST_BYPASS = 1 -- 359 table words,
 * 21% of the captured program, apparently switched off. They are not. Zeroing
 * them produces a completely black frame, and so does zeroing them while
 * keeping the block's grid geometry, step multipliers and CONFIG words, which
 * was the obvious explanation [HW 2026-08-21].
 *
 * It is not a dynamic-range block with a bypass. It is **local tone mapping**,
 * and the captured words say so on their own: 0x6014..0x601c hold 1225, 2404
 * and 467, the BT.601 luma weights in Q12; 0x6020..0x611c hold 128 strictly
 * monotone samples rising to 32737 of 32768; and 0x64e0..0x65fc hold 144 cells
 * of 0x0100, unity in Q8. The block takes RGB, forms a luma, looks up a tone
 * curve and applies a gain grid. Zeroing a tone curve is why the frame went
 * black. Vendor code agrees: it has a YuvpLtmBlock whose CMDQ configuration
 * takes the image size -- which is why four of these words track the chain
 * geometry between cameras -- and which carries a hardware-backed array of
 * 49152 shorts, exactly the 96 KiB LTM grid this driver already generates as
 * an identity.
 *
 * So the block runs. Its two curves and its CONFIG words are live per-frame
 * tuning and stay in the recipe; the gate, the luma weights, the grid geometry
 * and the vendor's own identity fills are stated below.
 *
 * Lyric's embedded register descriptors name the whole range rgb_diablo_ltm_*,
 * and that is what fixes where each stated run ends: the gain LUT does not
 * stop at 0x65fc -- that is only where the captured program's header ended --
 * it runs to 0x66d4, which is where 122 more unity entries put it.
 *
 *   0x6000  ltm_enable                    the block runs
 *   0x6014  lumacalc_rgby_coeff_r/g/b     BT.601 luma at Q12
 *   0x6120  slcgrid_*                     9 words, from the image size
 *   0x62dc  crecon_satctrl_lut            258 entries, all zero
 *   0x64e0  crecon_luma_lut               130 entries of unity Q8
 *   0x65e4  crecon_gain_lut               122 entries of unity Q8
 *
 * GetDefaultLtm fills the luma and gain LUTs with 0x0100 and clears the
 * saturation LUT, so the two identity fills are the vendor's own and not an
 * artefact of the scene this was captured from -- and neither differs between
 * the rear and front cameras, where the four grid reciprocals do.
 */
#define BECORE_YUVP_COUTFIFO0_EN_REG	(BECORE_YUVP_PHYS_BASE + 0x1200)
#define BECORE_YUVP_DTP_BYPASS_REG	(BECORE_YUVP_PHYS_BASE + 0x3000)
#define BECORE_YUVP_LTM_BASE		(BECORE_YUVP_PHYS_BASE + 0x6000)
#define BECORE_YUVP_LTM_ENABLE_REG	(BECORE_YUVP_LTM_BASE + 0x000)
#define BECORE_YUVP_LTM_LUMA_FIRST	(BECORE_YUVP_LTM_BASE + 0x014)
#define BECORE_YUVP_LTM_LUMA_LAST	(BECORE_YUVP_LTM_BASE + 0x01c)
#define BECORE_YUVP_LTM_GRID_FIRST	(BECORE_YUVP_LTM_BASE + 0x120)
#define BECORE_YUVP_LTM_GRID_LAST	(BECORE_YUVP_LTM_BASE + 0x140)
#define BECORE_YUVP_LTM_SATCTRL_FIRST	(BECORE_YUVP_LTM_BASE + 0x2dc)
#define BECORE_YUVP_LTM_SATCTRL_LAST	(BECORE_YUVP_LTM_BASE + 0x4dc)
#define BECORE_YUVP_LTM_UNITY_FIRST	(BECORE_YUVP_LTM_BASE + 0x4e0)
#define BECORE_YUVP_LTM_UNITY_LAST	(BECORE_YUVP_LTM_BASE + 0x6d4)
/*
 * The grid is a fixed 32 x 24 x 8 bilateral grid: 32 * 24 * 8 cells of eight
 * shorts is 49152, exactly the array YuvpLtmBlock::ConfigureWith carries. On a
 * 4:3 frame its cells are square, which is why the horizontal and vertical
 * reciprocals below come out equal and why this looked unsolvable.
 */
#define BECORE_LTM_LUMA_Q12_R		1225
#define BECORE_LTM_LUMA_Q12_G		2404
#define BECORE_LTM_LUMA_Q12_B		467
#define BECORE_LTM_SLCGRID_COLUMNS	32
#define BECORE_LTM_SLCGRID_ROWS		24
#define BECORE_LTM_SLCGRID_DEPTH	8
#define BECORE_LTM_SLCGRID_CELL_SHORTS	8
#define BECORE_LTM_UNITY_Q8_PAIR	0x01000100
/*
 * The register block and the 96 KiB buffer are two halves of one thing, and
 * nothing else in this driver says so: state it where both are in scope, so a
 * future edit to either has to answer for the other.
 */
static_assert(BECORE_LTM_SLCGRID_COLUMNS * BECORE_LTM_SLCGRID_ROWS *
	      BECORE_LTM_SLCGRID_DEPTH * BECORE_LTM_SLCGRID_CELL_SHORTS *
	      sizeof(__le16) == BECORE_GRID_SIZE);
#define BECORE_YUVP_GRID_REG		(BECORE_YUVP_PHYS_BASE + 0x1c50)
#define BECORE_YUVP_OUTPUT_PLANE1_REG	(BECORE_YUVP_PHYS_BASE + 0x2450)
#define BECORE_YUVP_OUTPUT_PLANE2_REG	(BECORE_YUVP_PHYS_BASE + 0x2490)
#define BECORE_YUVP_OUTPUT_ENABLE_REG	(BECORE_YUVP_PHYS_BASE + 0x2400)
#define BECORE_YUVP_OUTPUT_MODE_REG	(BECORE_YUVP_PHYS_BASE + 0x2404)
#define BECORE_YUVP_OUTPUT_FORMAT_REG	(BECORE_YUVP_PHYS_BASE + 0x2410)
#define BECORE_YUVP_OUTPUT_LOSSY_REG	(BECORE_YUVP_PHYS_BASE + 0x2418)
#define BECORE_YUVP_OUTPUT_WIDTH_REG	(BECORE_YUVP_PHYS_BASE + 0x2420)
#define BECORE_YUVP_OUTPUT_HEIGHT_REG	(BECORE_YUVP_PHYS_BASE + 0x2424)
#define BECORE_YUVP_OUTPUT_STRIDE1_REG	(BECORE_YUVP_PHYS_BASE + 0x2428)
#define BECORE_YUVP_OUTPUT_STRIDE2_REG	(BECORE_YUVP_PHYS_BASE + 0x242c)
#define BECORE_YUVP_OUTPUT_VOTF_REG	(BECORE_YUVP_PHYS_BASE + 0x243c)
#define BECORE_YUVP_OUTPUT_BUSINFO_REG	(BECORE_YUVP_PHYS_BASE + 0x244c)

#define BECORE_GTNR_INPUT_PLANE1_REG	(BECORE_GTNR_PHYS_BASE + 0x1e50)
#define BECORE_GTNR_INPUT_PLANE2_REG	(BECORE_GTNR_PHYS_BASE + 0x1e90)
#define BECORE_GTNR_INPUT_VOTF_REG	(BECORE_GTNR_PHYS_BASE + 0x1e3c)
#define BECORE_GTNR_INPUT_FORMAT_REG	(BECORE_GTNR_PHYS_BASE + 0x1e10)
#define BECORE_GTNR_INPUT_LOSSY_REG	(BECORE_GTNR_PHYS_BASE + 0x1e18)
#define BECORE_GTNR_INPUT_COMP_REG	(BECORE_GTNR_PHYS_BASE + 0x1e04)
#define BECORE_GTNR_INPUT_WIDTH_REG	(BECORE_GTNR_PHYS_BASE + 0x1e20)
#define BECORE_GTNR_INPUT_HEIGHT_REG	(BECORE_GTNR_PHYS_BASE + 0x1e24)
#define BECORE_GTNR_INPUT_STRIDE1_REG	(BECORE_GTNR_PHYS_BASE + 0x1e28)
#define BECORE_GTNR_INPUT_STRIDE2_REG	(BECORE_GTNR_PHYS_BASE + 0x1e2c)
#define BECORE_GTNR_INPUT_BUSINFO_REG	(BECORE_GTNR_PHYS_BASE + 0x1e4c)
#define BECORE_GTNR_INPUT_MAX_MO_REG	(BECORE_GTNR_PHYS_BASE + 0x1e40)
#define BECORE_GTNR_INPUT_MAX_BL_REG	(BECORE_GTNR_PHYS_BASE + 0x1e48)
#define BECORE_GTNR_INPUT_ENABLE_REG	(BECORE_GTNR_PHYS_BASE + 0x1e00)
#define BECORE_GTNR_OUTPUT_PLANE1_REG	(BECORE_GTNR_PHYS_BASE + 0x3050)
#define BECORE_GTNR_OUTPUT_PLANE2_REG	(BECORE_GTNR_PHYS_BASE + 0x3090)
#define BECORE_GTNR_OUTPUT_FORMAT_REG	(BECORE_GTNR_PHYS_BASE + 0x3010)
#define BECORE_GTNR_OUTPUT_LOSSY_REG	(BECORE_GTNR_PHYS_BASE + 0x3018)
#define BECORE_GTNR_OUTPUT_COMP_REG	(BECORE_GTNR_PHYS_BASE + 0x3004)
#define BECORE_GTNR_OUTPUT_WIDTH_REG	(BECORE_GTNR_PHYS_BASE + 0x3020)
#define BECORE_GTNR_OUTPUT_HEIGHT_REG	(BECORE_GTNR_PHYS_BASE + 0x3024)
#define BECORE_GTNR_OUTPUT_STRIDE1_REG	(BECORE_GTNR_PHYS_BASE + 0x3028)
#define BECORE_GTNR_OUTPUT_STRIDE2_REG	(BECORE_GTNR_PHYS_BASE + 0x302c)
#define BECORE_GTNR_OUTPUT_BUSINFO_REG	(BECORE_GTNR_PHYS_BASE + 0x304c)
#define BECORE_GTNR_OUTPUT_MAX_MO_REG	(BECORE_GTNR_PHYS_BASE + 0x3040)
#define BECORE_GTNR_OUTPUT_MAX_BL_REG	(BECORE_GTNR_PHYS_BASE + 0x3048)
#define BECORE_GTNR_OUTPUT_ENABLE_REG	(BECORE_GTNR_PHYS_BASE + 0x3000)

#define BECORE_MCSC_INPUT_PLANE1_REG	(BECORE_MCSC_PHYS_BASE + 0x1850)
#define BECORE_MCSC_INPUT_PLANE2_REG	(BECORE_MCSC_PHYS_BASE + 0x1890)
#define BECORE_MCSC_INPUT_VOTF_REG	(BECORE_MCSC_PHYS_BASE + 0x183c)
#define BECORE_MCSC_INPUT_FORMAT_REG	(BECORE_MCSC_PHYS_BASE + 0x1810)
#define BECORE_MCSC_INPUT_LOSSY_REG	(BECORE_MCSC_PHYS_BASE + 0x1818)
#define BECORE_MCSC_INPUT_COMP_REG	(BECORE_MCSC_PHYS_BASE + 0x1804)
#define BECORE_MCSC_INPUT_WIDTH_REG	(BECORE_MCSC_PHYS_BASE + 0x1820)
#define BECORE_MCSC_INPUT_HEIGHT_REG	(BECORE_MCSC_PHYS_BASE + 0x1824)
#define BECORE_MCSC_INPUT_STRIDE1_REG	(BECORE_MCSC_PHYS_BASE + 0x1828)
#define BECORE_MCSC_INPUT_STRIDE2_REG	(BECORE_MCSC_PHYS_BASE + 0x182c)
#define BECORE_MCSC_INPUT_BUSINFO_REG	(BECORE_MCSC_PHYS_BASE + 0x184c)
#define BECORE_MCSC_INPUT_MAX_BL_REG	(BECORE_MCSC_PHYS_BASE + 0x1848)
#define BECORE_MCSC_INPUT_ENABLE_REG	(BECORE_MCSC_PHYS_BASE + 0x1800)
#define BECORE_MCSC_OUTPUT_PLANE1_REG	(BECORE_MCSC_PHYS_BASE + 0x2050)
#define BECORE_MCSC_OUTPUT_PLANE2_REG	(BECORE_MCSC_PHYS_BASE + 0x2090)
#define BECORE_MCSC_OUTPUT_FORMAT_REG	(BECORE_MCSC_PHYS_BASE + 0x2010)
#define BECORE_MCSC_OUTPUT_COMP_REG	(BECORE_MCSC_PHYS_BASE + 0x2004)
#define BECORE_MCSC_OUTPUT_WIDTH_REG	(BECORE_MCSC_PHYS_BASE + 0x2020)
#define BECORE_MCSC_OUTPUT_HEIGHT_REG	(BECORE_MCSC_PHYS_BASE + 0x2024)
#define BECORE_MCSC_OUTPUT_STRIDE1_REG	(BECORE_MCSC_PHYS_BASE + 0x2028)
#define BECORE_MCSC_OUTPUT_STRIDE2_REG	(BECORE_MCSC_PHYS_BASE + 0x202c)
#define BECORE_MCSC_OUTPUT_BUSINFO_REG	(BECORE_MCSC_PHYS_BASE + 0x204c)
#define BECORE_MCSC_OUTPUT_MAX_BL_REG	(BECORE_MCSC_PHYS_BASE + 0x2048)
#define BECORE_MCSC_OUTPUT_ENABLE_REG	(BECORE_MCSC_PHYS_BASE + 0x2000)
#define BECORE_MCSC_OUTPUT_DITHER_REG	(BECORE_MCSC_PHYS_BASE + 0x2f00)
/*
 * DJAG's pre-scaler geometry -- Samsung MCSC v10.1 names these
 * YUV_DJAG_IMG_SIZE, YUV_DJAG_PS_SRC_POS/SRC_SIZE/DST_SIZE and
 * YUV_DJAG_PS_H/V_RATIO. This is the block that crops and scales; POLY_SC0
 * downstream of it runs at unity on this recipe, which is why its x8/8 filter
 * coefficients are correct and unaffected by the ratio here.
 *
 * Each of the first four registers packs two 16-bit halves with the width in
 * the high one, and the two ratios are the crop as a 20-bit fixed-point
 * fraction of the destination -- Samsung's own GET_ZOOM_RATIO(in, out), which
 * is ((in) << MCSC_PRECISION) / (out) with MCSC_PRECISION 20. For the recorded
 * 3536 x 2652 crop into 4000 x 3000 that truncates to exactly the 0x000e24dd
 * the vendor program carries, and so does the vertical.
 */
#define BECORE_MCSC_DJAG_IMG_SIZE_REG	(BECORE_MCSC_PHYS_BASE + 0x4004)
#define BECORE_MCSC_DJAG_PS_SRC_POS_REG	(BECORE_MCSC_PHYS_BASE + 0x4008)
#define BECORE_MCSC_DJAG_PS_SRC_SIZE_REG (BECORE_MCSC_PHYS_BASE + 0x400c)
#define BECORE_MCSC_DJAG_PS_DST_SIZE_REG	(BECORE_MCSC_PHYS_BASE + 0x4010)
#define BECORE_MCSC_DJAG_PS_H_RATIO_REG	(BECORE_MCSC_PHYS_BASE + 0x4014)
#define BECORE_MCSC_DJAG_PS_V_RATIO_REG	(BECORE_MCSC_PHYS_BASE + 0x4018)
#define BECORE_RATIO_SHIFT		20
/*
 * MCSC's chain below DJAG, named from Samsung MCSC v10.1. DJAG has already
 * produced the output raster, so the POLY_SC0 scaler and the POST_PC0 chroma
 * converter each map that raster onto itself: neither crops and neither
 * scales. Carried as constants these did not move when the output geometry
 * did, and 4000 x 3000 happened to be right; derived, they cannot disagree
 * with the surface MCSC writes.
 */
#define BECORE_MCSC_SC0_SRC_POS_REG	(BECORE_MCSC_PHYS_BASE + 0x5004)
#define BECORE_MCSC_SC0_SRC_SIZE_REG	(BECORE_MCSC_PHYS_BASE + 0x5008)
#define BECORE_MCSC_SC0_DST_SIZE_REG	(BECORE_MCSC_PHYS_BASE + 0x500c)
#define BECORE_MCSC_SC0_H_RATIO_REG	(BECORE_MCSC_PHYS_BASE + 0x5010)
#define BECORE_MCSC_SC0_V_RATIO_REG	(BECORE_MCSC_PHYS_BASE + 0x5014)
#define BECORE_MCSC_PC0_IMG_SIZE_REG	(BECORE_MCSC_PHYS_BASE + 0x6004)
#define BECORE_MCSC_PC0_DST_SIZE_REG	(BECORE_MCSC_PHYS_BASE + 0x6008)
#define BECORE_MCSC_PC0_H_RATIO_REG	(BECORE_MCSC_PHYS_BASE + 0x600c)
#define BECORE_MCSC_PC0_V_RATIO_REG	(BECORE_MCSC_PHYS_BASE + 0x6010)
/*
 * The two poly-phase scalers' filter coefficients. RGBP's SC and MCSC's
 * POLY_SC0 carry bit-identical tables, at 0x4500/0x4548 and 0x5024/0x506c
 * respectively: nine phases of four vertical and eight horizontal taps, two
 * taps to a register with the lower-numbered one in the low half.
 */
#define BECORE_RGBP_SC_V_COEFF_FIRST	(BECORE_RGBP_PHYS_BASE + 0x4500)
#define BECORE_RGBP_SC_V_COEFF_LAST	(BECORE_RGBP_PHYS_BASE + 0x4544)
#define BECORE_RGBP_SC_H_COEFF_FIRST	(BECORE_RGBP_PHYS_BASE + 0x4548)
#define BECORE_RGBP_SC_H_COEFF_LAST	(BECORE_RGBP_PHYS_BASE + 0x45d4)
#define BECORE_MCSC_SC0_V_COEFF_FIRST	(BECORE_MCSC_PHYS_BASE + 0x5024)
#define BECORE_MCSC_SC0_V_COEFF_LAST	(BECORE_MCSC_PHYS_BASE + 0x5068)
#define BECORE_MCSC_SC0_H_COEFF_FIRST	(BECORE_MCSC_PHYS_BASE + 0x506c)
#define BECORE_MCSC_SC0_H_COEFF_LAST	(BECORE_MCSC_PHYS_BASE + 0x50f8)
#define BECORE_SC_PHASES		9
#define BECORE_SC_V_TAPS		4
#define BECORE_SC_H_TAPS		8
#define BECORE_SC_COEFF_SUM		512
#define BECORE_SC_COEFF_MASK		GENMASK(10, 0)
#define BECORE_SC_RATIO_X8_8		(1U << BECORE_RATIO_SHIFT)

enum becore_block_id {
	BECORE_RGBP,
	BECORE_MCFP,
	BECORE_YUVP,
	BECORE_MCSC,
	BECORE_NUM_BLOCKS,
};

struct becore_regval {
	u32 offset;
	u32 value;
};

struct becore_rgbp_input_profile {
	u32 width;
	u32 height;
	u32 data_format;
	u32 comp_control;
	u32 sbwc_block_width;
	u32 bytes_per_pixel;
	u32 header_stride;
	u32 businfo;
};

/*
 * The active dimensions and DMA fields are from the live ultrawide program.
 * As in Pablo's common DMA API, the payload and header geometry are derived
 * from the image profile.  Lyric additionally writes the 256-pixel-aligned
 * SBWC storage width after enabling the RDMA.
 */
static const struct becore_rgbp_input_profile becore_rgbp_input = {
	.width = 4208,
	.height = 3120,
	.data_format = 0x18,
	.comp_control = 0x9,
	.sbwc_block_width = 256,
	.bytes_per_pixel = 2,
	.header_stride = 0x40,
	.businfo = 0,
};

enum becore_rgbp_input_word {
	BECORE_RGBP_CROP_SIZE,
	BECORE_RGBP_CROP_START,
	BECORE_RGBP_SC_DST_SIZE,
	BECORE_RGBP_SC_H_RATIO,
	BECORE_RGBP_SC_V_RATIO,
	BECORE_RGBP_CHAIN_SRC_SIZE,
	BECORE_RGBP_CHAIN_DST_SIZE,
	BECORE_RGBP_INPUT_FORMAT,
	BECORE_RGBP_INPUT_COMP,
	BECORE_RGBP_INPUT_ACTIVE_WIDTH,
	BECORE_RGBP_INPUT_HEIGHT,
	BECORE_RGBP_INPUT_STRIDE,
	BECORE_RGBP_INPUT_HEADER_STRIDE,
	BECORE_RGBP_INPUT_BUSINFO,
	BECORE_RGBP_INPUT_ENABLE,
	BECORE_RGBP_INPUT_STORAGE_WIDTH,
	BECORE_RGBP_INPUT_WORD_COUNT,
};

/*
 * A value the driver generates from what the block it programs is *for*.
 *
 * Unlike a typed word, which is matched to its register by its position in the
 * program, one of these is resolved by register address alone -- so an entry
 * cannot drift onto a neighbouring word, and a range can cover a whole block
 * without naming each of its registers.
 */
enum becore_generated_kind {
	BECORE_GEN_OFF,		/* an enable the program leaves clear */
	BECORE_GEN_BYPASS,	/* an asserted bypass bit */
	BECORE_GEN_RUNNING,	/* a bypass the program clears: the block runs */
	BECORE_GEN_DECOMP_SIZE,	/* a frame size, from the Bayer input */
	BECORE_GEN_CSC,		/* RGB to YUV: BT.601, full range, Q13 */
	BECORE_GEN_CHROMA_LPF,	/* 4:4:4 to 4:2:2, a fixed binomial filter */
	BECORE_GEN_DJAG,	/* MCSC DJAG at Samsung's neutral profile */
	BECORE_GEN_DMSC,	/* what GetDefaultDmsc writes after the tuning */
	BECORE_GEN_DNS_GEOMETRY,	/* binning and radial centre, from the array */
	BECORE_GEN_GAMMA,	/* RGBP's forward gamma, a square-root encode */
	BECORE_GEN_LPF_NORM,	/* log2 of the sharpener's three kernel sums */
	BECORE_GEN_NOISE_SLOPE,	/* a noise curve's slopes, from its own knots */
	BECORE_GEN_NOISE_SHIFT,	/* the shift those slopes are taken at */
	BECORE_GEN_NOISE_DOMAIN,	/* a chroma domain repeating the luma one */
	BECORE_GEN_SCALER_PHASE,	/* a scaler starting on a pixel, rounding */
	BECORE_GEN_MCSC_INPUT_SIZE,	/* the raster MCSC reads, from YUVP */
	BECORE_GEN_GTM,		/* RGBP's tone map, an identity */
	BECORE_GEN_LTM,		/* YUVP's tone mapping: gate, luma, grid, identity */
	BECORE_GEN_CHAIN_SIZE,	/* a raster size, from the output profile */
	BECORE_GEN_CHAIN_ORIGIN,	/* a chain stage that does not crop */
	BECORE_GEN_CHAIN_RATIO,	/* a chain stage that does not scale */
	BECORE_GEN_SC_V_COEFF,	/* poly-phase vertical taps, from the ratio */
	BECORE_GEN_SC_H_COEFF,	/* poly-phase horizontal taps, from the ratio */
};

struct becore_generated_range {
	u32 first;		/* physical register, inclusive */
	u32 last;		/* inclusive; equal to first for one register */
	u32 kind;
};

/*
 * How many words each block hands to becore_generated_value(). Held here
 * rather than in the generated table so that a recipe which quietly stopped
 * carrying one of them fails validation instead of programming the capture.
 */
#define BECORE_RGBP_GENERATED_WORDS	291
#define BECORE_YUVP_GENERATED_WORDS	305
#define BECORE_MCSC_GENERATED_WORDS	99

static const struct becore_generated_range becore_rgbp_generated[] = {
	{ BECORE_RGBP_CINFIFO_FRAME_IN_REG, BECORE_RGBP_CINFIFO_FRAME_IN_REG,
	  BECORE_GEN_OFF },
	{ BECORE_RGBP_SATFLAG_ENABLE_REG, BECORE_RGBP_SATFLAG_ENABLE_REG,
	  BECORE_GEN_OFF },
	{ BECORE_RGBP_DMSCCROP_BYPASS_REG, BECORE_RGBP_DMSCCROP_BYPASS_REG,
	  BECORE_GEN_RUNNING },
	{ BECORE_RGBP_WDMADECOMP_EN_REG, BECORE_RGBP_WDMADECOMP_EN_REG,
	  BECORE_GEN_OFF },
	{ BECORE_RGBP_WDMAY_EN_REG, BECORE_RGBP_WDMAY_EN_REG,
	  BECORE_GEN_OFF },
	{ BECORE_RGBP_WDMAUV_EN_REG, BECORE_RGBP_WDMAUV_EN_REG,
	  BECORE_GEN_OFF },
	{ BECORE_RGBP_DNS_SLOPE_G_REG,
	  BECORE_RGBP_DNS_SLOPE_G_REG + BECORE_NOISE_TABLE_LAST,
	  BECORE_GEN_NOISE_SLOPE },
	{ BECORE_RGBP_DNS_SHIFT_G_REG, BECORE_RGBP_DNS_SHIFT_G_REG,
	  BECORE_GEN_NOISE_SHIFT },
	{ BECORE_RGBP_DNS_X_RB_REG,
	  BECORE_RGBP_DNS_X_RB_REG + BECORE_NOISE_TABLE_LAST,
	  BECORE_GEN_NOISE_DOMAIN },
	{ BECORE_RGBP_DNS_SLOPE_RB_REG,
	  BECORE_RGBP_DNS_SLOPE_RB_REG + BECORE_NOISE_TABLE_LAST,
	  BECORE_GEN_NOISE_SLOPE },
	{ BECORE_RGBP_DNS_SHIFT_RB_REG, BECORE_RGBP_DNS_SHIFT_RB_REG,
	  BECORE_GEN_NOISE_SHIFT },
	{ BECORE_RGBP_DMSC_BASE + 0x20c, BECORE_RGBP_DMSC_BASE + 0x20c,
	  BECORE_GEN_DMSC },			/* BASE_CONFIG */
	{ BECORE_RGBP_DMSC_BASE + 0x238, BECORE_RGBP_DMSC_BASE + 0x238,
	  BECORE_GEN_DMSC },			/* EXTRACT_COLORS_CONFIG */
	{ BECORE_RGBP_DMSC_BASE + 0x24c, BECORE_RGBP_DMSC_BASE + 0x250,
	  BECORE_GEN_DMSC },			/* GREEN_HUE, GREEN_SAT */
	{ BECORE_RGBP_DMSC_BASE + 0x258, BECORE_RGBP_DMSC_BASE + 0x258,
	  BECORE_GEN_DMSC },			/* POST_PROCESS_CONFIG */
	{ BECORE_RGBP_DMSC_BASE + 0x264, BECORE_RGBP_DMSC_BASE + 0x264,
	  BECORE_GEN_DMSC },			/* DIR_DETECT_SELECTION */
	{ BECORE_RGBP_DMSC_BASE + 0x26c, BECORE_RGBP_DMSC_BASE + 0x274,
	  BECORE_GEN_DMSC },			/* ADD_COLORS_GREEN..SHARPENING */
	{ BECORE_RGBP_DMSC_BASE + 0x288, BECORE_RGBP_DMSC_BASE + 0x288,
	  BECORE_GEN_DMSC },			/* NEAR_EDGE_DESAT_EN */
	{ BECORE_RGBP_DMSC_BASE + 0x2a0, BECORE_RGBP_DMSC_BASE + 0x2a0,
	  BECORE_GEN_DMSC },			/* RED_PRESERVE_EN; its GAIN
						 * at 0x2a4 is tuning
						 */
	{ BECORE_RGBP_DMSC_BASE + 0x2a8, BECORE_RGBP_DMSC_BASE + 0x2ac,
	  BECORE_GEN_DMSC },			/* RED_PRESERVE_THRES, _LIMIT */
	{ BECORE_RGBP_DMSC_BASE + 0x2b4, BECORE_RGBP_DMSC_BASE + 0x2b4,
	  BECORE_GEN_DMSC },			/* ADD_YBLUR */
	{ BECORE_RGBP_DNS_BINNING_REG, BECORE_RGBP_DNS_BINNING_REG,
	  BECORE_GEN_DNS_GEOMETRY },
	{ BECORE_RGBP_DNS_CENTRE_REG, BECORE_RGBP_DNS_CENTRE_REG,
	  BECORE_GEN_DNS_GEOMETRY },
	{ BECORE_RGBP_GAMMA_CTRL_FIRST, BECORE_RGBP_GAMMA_CTRL_LAST,
	  BECORE_GEN_GAMMA },
	{ BECORE_RGBP_GAMMA_TBL_FIRST, BECORE_RGBP_GAMMA_TBL_LAST,
	  BECORE_GEN_GAMMA },
	{ BECORE_RGBP_GAMMA_X_LOW_FIRST, BECORE_RGBP_GAMMA_X_LOW_LAST,
	  BECORE_GEN_GAMMA },
	{ BECORE_RGBP_GAMMA_X_HIGH_FIRST, BECORE_RGBP_GAMMA_X_HIGH_LAST,
	  BECORE_GEN_GAMMA },
	{ BECORE_RGBP_GTM_BASE, BECORE_RGBP_GTM_LAST, BECORE_GEN_GTM },
	{ BECORE_RGBP_CSC_FIRST, BECORE_RGBP_CSC_LAST, BECORE_GEN_CSC },
	{ BECORE_RGBP_CHROMA_LPF_CTRL_REG, BECORE_RGBP_CHROMA_LPF_CTRL_REG,
	  BECORE_GEN_CHROMA_LPF },
	{ BECORE_RGBP_CHROMA_LPF_FIRST, BECORE_RGBP_CHROMA_LPF_LAST,
	  BECORE_GEN_CHROMA_LPF },
	{ BECORE_RGBP_DECOMP_BYPASS_REG, BECORE_RGBP_DECOMP_BYPASS_REG,
	  BECORE_GEN_BYPASS },
	{ BECORE_RGBP_DECOMP_SIZE_REG, BECORE_RGBP_DECOMP_SIZE_REG,
	  BECORE_GEN_DECOMP_SIZE },
	{ BECORE_RGBP_GAMMALR_BYPASS_REG, BECORE_RGBP_GAMMALR_BYPASS_REG,
	  BECORE_GEN_BYPASS },
	{ BECORE_RGBP_UPSC_CTRL0_REG, BECORE_RGBP_UPSC_CTRL0_REG,
	  BECORE_GEN_OFF },
	{ BECORE_RGBP_GAMMAHR_BYPASS_REG, BECORE_RGBP_GAMMAHR_BYPASS_REG,
	  BECORE_GEN_BYPASS },
	{ BECORE_RGBP_SC_PHASE_FIRST, BECORE_RGBP_SC_PHASE_LAST,
	  BECORE_GEN_SCALER_PHASE },
	{ BECORE_RGBP_SC_V_COEFF_FIRST, BECORE_RGBP_SC_V_COEFF_LAST,
	  BECORE_GEN_SC_V_COEFF },
	{ BECORE_RGBP_SC_H_COEFF_FIRST, BECORE_RGBP_SC_H_COEFF_LAST,
	  BECORE_GEN_SC_H_COEFF },
};

static const struct becore_generated_range becore_yuvp_generated[] = {
	{ BECORE_YUVP_COUTFIFO0_EN_REG, BECORE_YUVP_COUTFIFO0_EN_REG,
	  BECORE_GEN_OFF },
	{ BECORE_YUVP_DTP_BYPASS_REG, BECORE_YUVP_DTP_BYPASS_REG,
	  BECORE_GEN_BYPASS },
	{ BECORE_YUVP_CSC_FIRST, BECORE_YUVP_CSC_LAST, BECORE_GEN_CSC },
	{ BECORE_YUVP_NR_SLOPE_Y_REG,
	  BECORE_YUVP_NR_SLOPE_Y_REG + BECORE_NOISE_TABLE_LAST,
	  BECORE_GEN_NOISE_SLOPE },
	{ BECORE_YUVP_NR_SHIFT_Y_REG, BECORE_YUVP_NR_SHIFT_Y_REG,
	  BECORE_GEN_NOISE_SHIFT },
	{ BECORE_YUVP_NR_X_UV_REG,
	  BECORE_YUVP_NR_X_UV_REG + BECORE_NOISE_TABLE_LAST,
	  BECORE_GEN_NOISE_DOMAIN },
	{ BECORE_YUVP_NR_SLOPE_UV_REG,
	  BECORE_YUVP_NR_SLOPE_UV_REG + BECORE_NOISE_TABLE_LAST,
	  BECORE_GEN_NOISE_SLOPE },
	{ BECORE_YUVP_NR_SHIFT_UV_REG, BECORE_YUVP_NR_SHIFT_UV_REG,
	  BECORE_GEN_NOISE_SHIFT },
	{ BECORE_YUVP_LPF_NORM_REG, BECORE_YUVP_LPF_NORM_REG,
	  BECORE_GEN_LPF_NORM },
	{ BECORE_YUVP_LTM_ENABLE_REG, BECORE_YUVP_LTM_ENABLE_REG,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_LUMA_FIRST, BECORE_YUVP_LTM_LUMA_LAST,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_GRID_FIRST, BECORE_YUVP_LTM_GRID_LAST,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_SATCTRL_FIRST, BECORE_YUVP_LTM_SATCTRL_LAST,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_UNITY_FIRST, BECORE_YUVP_LTM_UNITY_LAST,
	  BECORE_GEN_LTM },
};

static const struct becore_generated_range becore_mcsc_generated[] = {
	{ BECORE_MCSC_OTF_GATE_FIRST, BECORE_MCSC_OTF_GATE_LAST,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_INPUT_TYPE_REG, BECORE_MCSC_INPUT_TYPE_REG,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_IN_WIDTH_REG, BECORE_MCSC_IN_HEIGHT_REG,
	  BECORE_GEN_MCSC_INPUT_SIZE },
	{ BECORE_MCSC_CINFIFO_FIRST, BECORE_MCSC_CINFIFO_LAST,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_STAT_RDMA_FIRST, BECORE_MCSC_STAT_RDMA_LAST,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_WDMA_W1_FIRST,
	  BECORE_MCSC_WDMA_W1_FIRST + BECORE_MCSC_WDMA_OFF_LAST,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_WDMA_W2_FIRST,
	  BECORE_MCSC_WDMA_W2_FIRST + BECORE_MCSC_WDMA_OFF_LAST,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_WDMA_W3_FIRST,
	  BECORE_MCSC_WDMA_W3_FIRST + BECORE_MCSC_WDMA_OFF_LAST,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_WDMA_W4_FIRST,
	  BECORE_MCSC_WDMA_W4_FIRST + BECORE_MCSC_WDMA_OFF_LAST,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_SC0_PHASE_FIRST,
	  BECORE_MCSC_SC0_PHASE_FIRST + BECORE_SCALER_PHASE_LAST,
	  BECORE_GEN_SCALER_PHASE },
	{ BECORE_MCSC_PC0_PHASE_FIRST,
	  BECORE_MCSC_PC0_PHASE_FIRST + BECORE_SCALER_PHASE_LAST,
	  BECORE_GEN_SCALER_PHASE },
	{ BECORE_MCSC_DJAG_CTRL_REG, BECORE_MCSC_DJAG_CTRL_REG,
	  BECORE_GEN_DJAG },
	{ BECORE_MCSC_DJAG_PS_FIRST, BECORE_MCSC_DJAG_PS_LAST,
	  BECORE_GEN_DJAG },
	{ BECORE_MCSC_DJAG_TUNE_FIRST, BECORE_MCSC_DJAG_TUNE_LAST,
	  BECORE_GEN_DJAG },
	{ BECORE_MCSC_DJAG_RECOM_CTRL_REG, BECORE_MCSC_DJAG_RECOM_CTRL_REG,
	  BECORE_GEN_DJAG },
	{ BECORE_MCSC_DJAG_RECOM_WEIGHT_REG, BECORE_MCSC_DJAG_RECOM_WEIGHT_REG,
	  BECORE_GEN_DJAG },
	{ BECORE_MCSC_SC0_SRC_POS_REG, BECORE_MCSC_SC0_SRC_POS_REG,
	  BECORE_GEN_CHAIN_ORIGIN },
	{ BECORE_MCSC_SC0_SRC_SIZE_REG, BECORE_MCSC_SC0_DST_SIZE_REG,
	  BECORE_GEN_CHAIN_SIZE },
	{ BECORE_MCSC_SC0_H_RATIO_REG, BECORE_MCSC_SC0_V_RATIO_REG,
	  BECORE_GEN_CHAIN_RATIO },
	{ BECORE_MCSC_PC0_IMG_SIZE_REG, BECORE_MCSC_PC0_DST_SIZE_REG,
	  BECORE_GEN_CHAIN_SIZE },
	{ BECORE_MCSC_PC0_H_RATIO_REG, BECORE_MCSC_PC0_V_RATIO_REG,
	  BECORE_GEN_CHAIN_RATIO },
	{ BECORE_MCSC_SC0_V_COEFF_FIRST, BECORE_MCSC_SC0_V_COEFF_LAST,
	  BECORE_GEN_SC_V_COEFF },
	{ BECORE_MCSC_SC0_H_COEFF_FIRST, BECORE_MCSC_SC0_H_COEFF_LAST,
	  BECORE_GEN_SC_H_COEFF },
};

static const u32 becore_rgbp_input_regs[] = {
	[BECORE_RGBP_CROP_SIZE] = BECORE_RGBP_CROP_SIZE_REG,
	[BECORE_RGBP_CROP_START] = BECORE_RGBP_CROP_START_REG,
	[BECORE_RGBP_SC_DST_SIZE] = BECORE_RGBP_SC_DST_SIZE_REG,
	[BECORE_RGBP_SC_H_RATIO] = BECORE_RGBP_SC_H_RATIO_REG,
	[BECORE_RGBP_SC_V_RATIO] = BECORE_RGBP_SC_V_RATIO_REG,
	[BECORE_RGBP_CHAIN_SRC_SIZE] = BECORE_RGBP_CHAIN_SRC_SIZE_REG,
	[BECORE_RGBP_CHAIN_DST_SIZE] = BECORE_RGBP_CHAIN_DST_SIZE_REG,
	[BECORE_RGBP_INPUT_FORMAT] = BECORE_RGBP_INPUT_FORMAT_REG,
	[BECORE_RGBP_INPUT_COMP] = BECORE_RGBP_INPUT_COMP_REG,
	[BECORE_RGBP_INPUT_ACTIVE_WIDTH] = BECORE_RGBP_INPUT_WIDTH_REG,
	[BECORE_RGBP_INPUT_HEIGHT] = BECORE_RGBP_INPUT_HEIGHT_REG,
	[BECORE_RGBP_INPUT_STRIDE] = BECORE_RGBP_INPUT_STRIDE_REG,
	[BECORE_RGBP_INPUT_HEADER_STRIDE] = BECORE_RGBP_INPUT_HEADER_STRIDE_REG,
	[BECORE_RGBP_INPUT_BUSINFO] = BECORE_RGBP_INPUT_BUSINFO_REG,
	[BECORE_RGBP_INPUT_ENABLE] = BECORE_RGBP_INPUT_ENABLE_REG,
	[BECORE_RGBP_INPUT_STORAGE_WIDTH] = BECORE_RGBP_INPUT_WIDTH_REG,
};

struct becore_yuvp_output_profile {
	u32 width;
	u32 height;
	u32 data_format;
	u32 mode;
	u32 lossy_byte32num;
	u32 votf_enable;
	u32 bytes_per_pixel;
	u32 block_height;
	u32 luma_height_align;
	u32 plane_gap;
	u32 businfo;
};

enum becore_yuvp_output_profile_id {
	BECORE_YUVP_OUTPUT_SBWCL,
	BECORE_YUVP_OUTPUT_P010,
	BECORE_YUVP_OUTPUT_PROFILE_COUNT,
};

/* Pixel uses a combined, two-plane WDMA where Pablo v1.1 uses split blocks. */
static const struct becore_yuvp_output_profile becore_yuvp_outputs[] = {
	[BECORE_YUVP_OUTPUT_SBWCL] = {
		.width = 4160,
		.height = 3120,
		.data_format = 0x2000,
		.mode = 0xa,
		.lossy_byte32num = 2,
		.votf_enable = 3,
		.bytes_per_pixel = 2,
		.block_height = 4,
		.luma_height_align = 16,
		.plane_gap = 0x40,
		.businfo = 0,
	},
	[BECORE_YUVP_OUTPUT_P010] = {
		.width = 4160,
		.height = 3120,
		.data_format = 0x2000,
		.mode = 0,
		.lossy_byte32num = 0,
		.votf_enable = 0,
		.bytes_per_pixel = 2,
		.businfo = 0,
	},
};

enum becore_yuvp_output_word {
	BECORE_YUVP_OUTPUT_VOTF,
	BECORE_YUVP_OUTPUT_FORMAT,
	BECORE_YUVP_OUTPUT_LOSSY,
	BECORE_YUVP_OUTPUT_MODE,
	BECORE_YUVP_OUTPUT_WIDTH,
	BECORE_YUVP_OUTPUT_HEIGHT,
	BECORE_YUVP_OUTPUT_STRIDE1,
	BECORE_YUVP_OUTPUT_STRIDE2,
	BECORE_YUVP_OUTPUT_BUSINFO,
	BECORE_YUVP_OUTPUT_ENABLE,
	BECORE_YUVP_OUTPUT_WORD_COUNT,
};

static const u32 becore_yuvp_output_regs[] = {
	[BECORE_YUVP_OUTPUT_VOTF] = BECORE_YUVP_OUTPUT_VOTF_REG,
	[BECORE_YUVP_OUTPUT_FORMAT] = BECORE_YUVP_OUTPUT_FORMAT_REG,
	[BECORE_YUVP_OUTPUT_LOSSY] = BECORE_YUVP_OUTPUT_LOSSY_REG,
	[BECORE_YUVP_OUTPUT_MODE] = BECORE_YUVP_OUTPUT_MODE_REG,
	[BECORE_YUVP_OUTPUT_WIDTH] = BECORE_YUVP_OUTPUT_WIDTH_REG,
	[BECORE_YUVP_OUTPUT_HEIGHT] = BECORE_YUVP_OUTPUT_HEIGHT_REG,
	[BECORE_YUVP_OUTPUT_STRIDE1] = BECORE_YUVP_OUTPUT_STRIDE1_REG,
	[BECORE_YUVP_OUTPUT_STRIDE2] = BECORE_YUVP_OUTPUT_STRIDE2_REG,
	[BECORE_YUVP_OUTPUT_BUSINFO] = BECORE_YUVP_OUTPUT_BUSINFO_REG,
	[BECORE_YUVP_OUTPUT_ENABLE] = BECORE_YUVP_OUTPUT_ENABLE_REG,
};

struct becore_gtnr_dma_profile {
	u32 width;
	u32 height;
	u32 data_format;
	u32 comp_control;
	u32 lossy_byte32num;
	u32 votf_enable;
	u32 stride;
	u32 businfo;
	u32 max_mo;
	u32 max_bl;
	u32 enable;
};

/*
 * The first physical-ultrawide GTNR frame has no temporal inputs or map DMAs.
 * It reads YUVP's 4160x3120 lossy-SBWC surface and writes a separate surface
 * with the same bounded layout.  Keep this startup shape dormant until GTNR's
 * power, reset, interrupt, and cross-block completion lifecycle is established.
 */
static const struct becore_gtnr_dma_profile becore_gtnr_input = {
	.width = 4160,
	.height = 3120,
	.data_format = 0x2000,
	.comp_control = 0xa,
	.lossy_byte32num = 2,
	.votf_enable = 1,
	.stride = 0x2080,
	.businfo = 1,
	.max_mo = 0x100,
	.max_bl = 0x10,
	.enable = 1,
};

static const struct becore_gtnr_dma_profile becore_gtnr_output = {
	.width = 4160,
	.height = 3120,
	.data_format = 0x2000,
	.comp_control = 0xa,
	.lossy_byte32num = 2,
	.stride = 0x2080,
	.businfo = 0,
	.max_mo = 0x100,
	.max_bl = 0x10,
	.enable = 1,
};

enum becore_gtnr_dma_word {
	BECORE_GTNR_INPUT_VOTF,
	BECORE_GTNR_INPUT_FORMAT,
	BECORE_GTNR_INPUT_LOSSY,
	BECORE_GTNR_INPUT_COMP,
	BECORE_GTNR_INPUT_WIDTH,
	BECORE_GTNR_INPUT_HEIGHT,
	BECORE_GTNR_INPUT_STRIDE1,
	BECORE_GTNR_INPUT_STRIDE2,
	BECORE_GTNR_INPUT_BUSINFO,
	BECORE_GTNR_INPUT_MAX_MO,
	BECORE_GTNR_INPUT_MAX_BL,
	BECORE_GTNR_INPUT_ENABLE,
	BECORE_GTNR_OUTPUT_FORMAT,
	BECORE_GTNR_OUTPUT_LOSSY,
	BECORE_GTNR_OUTPUT_COMP,
	BECORE_GTNR_OUTPUT_WIDTH,
	BECORE_GTNR_OUTPUT_HEIGHT,
	BECORE_GTNR_OUTPUT_STRIDE1,
	BECORE_GTNR_OUTPUT_STRIDE2,
	BECORE_GTNR_OUTPUT_BUSINFO,
	BECORE_GTNR_OUTPUT_MAX_MO,
	BECORE_GTNR_OUTPUT_MAX_BL,
	BECORE_GTNR_OUTPUT_ENABLE,
	BECORE_GTNR_DMA_WORD_COUNT,
};

static const u32 becore_gtnr_dma_regs[] = {
	[BECORE_GTNR_INPUT_VOTF] = BECORE_GTNR_INPUT_VOTF_REG,
	[BECORE_GTNR_INPUT_FORMAT] = BECORE_GTNR_INPUT_FORMAT_REG,
	[BECORE_GTNR_INPUT_LOSSY] = BECORE_GTNR_INPUT_LOSSY_REG,
	[BECORE_GTNR_INPUT_COMP] = BECORE_GTNR_INPUT_COMP_REG,
	[BECORE_GTNR_INPUT_WIDTH] = BECORE_GTNR_INPUT_WIDTH_REG,
	[BECORE_GTNR_INPUT_HEIGHT] = BECORE_GTNR_INPUT_HEIGHT_REG,
	[BECORE_GTNR_INPUT_STRIDE1] = BECORE_GTNR_INPUT_STRIDE1_REG,
	[BECORE_GTNR_INPUT_STRIDE2] = BECORE_GTNR_INPUT_STRIDE2_REG,
	[BECORE_GTNR_INPUT_BUSINFO] = BECORE_GTNR_INPUT_BUSINFO_REG,
	[BECORE_GTNR_INPUT_MAX_MO] = BECORE_GTNR_INPUT_MAX_MO_REG,
	[BECORE_GTNR_INPUT_MAX_BL] = BECORE_GTNR_INPUT_MAX_BL_REG,
	[BECORE_GTNR_INPUT_ENABLE] = BECORE_GTNR_INPUT_ENABLE_REG,
	[BECORE_GTNR_OUTPUT_FORMAT] = BECORE_GTNR_OUTPUT_FORMAT_REG,
	[BECORE_GTNR_OUTPUT_LOSSY] = BECORE_GTNR_OUTPUT_LOSSY_REG,
	[BECORE_GTNR_OUTPUT_COMP] = BECORE_GTNR_OUTPUT_COMP_REG,
	[BECORE_GTNR_OUTPUT_WIDTH] = BECORE_GTNR_OUTPUT_WIDTH_REG,
	[BECORE_GTNR_OUTPUT_HEIGHT] = BECORE_GTNR_OUTPUT_HEIGHT_REG,
	[BECORE_GTNR_OUTPUT_STRIDE1] = BECORE_GTNR_OUTPUT_STRIDE1_REG,
	[BECORE_GTNR_OUTPUT_STRIDE2] = BECORE_GTNR_OUTPUT_STRIDE2_REG,
	[BECORE_GTNR_OUTPUT_BUSINFO] = BECORE_GTNR_OUTPUT_BUSINFO_REG,
	[BECORE_GTNR_OUTPUT_MAX_MO] = BECORE_GTNR_OUTPUT_MAX_MO_REG,
	[BECORE_GTNR_OUTPUT_MAX_BL] = BECORE_GTNR_OUTPUT_MAX_BL_REG,
	[BECORE_GTNR_OUTPUT_ENABLE] = BECORE_GTNR_OUTPUT_ENABLE_REG,
};

struct becore_mcsc_dma_profile {
	u32 width;
	u32 height;
	u32 data_format;
	u32 comp_control;
	u32 lossy_byte32num;
	u32 votf_enable;
	u32 stride;
	u32 businfo;
	u32 max_bl;
	u32 enable;
	u32 dither;
};

/*
 * The physical-ultrawide 4000x3000 request carries YUVP's 4160x3120 lossy
 * SBWC intermediate into MCSC and writes output zero as linear NV21.  The
 * captured program used VOTF; a sequential run replaces only that transport
 * control so MCSC reads the completed driver-owned surface from memory.
 */
static const struct becore_mcsc_dma_profile becore_mcsc_input = {
	.width = 4160,
	.height = 3120,
	.data_format = 0x2000,
	.comp_control = 0xa,
	.lossy_byte32num = 2,
	.votf_enable = 0x00400001,
	.stride = 0x2080,
	.businfo = 2,
	.max_bl = 0x10,
	.enable = 1,
};

static const struct becore_mcsc_dma_profile becore_mcsc_output = {
	.width = 4000,
	.height = 3000,
	.data_format = 0x800,
	.comp_control = 0,
	.stride = 0xfc0,
	.businfo = 0,
	.max_bl = 4,
	.enable = 1,
	.dither = 0x10,
};

/*
 * How much of DJAG's input the output is taken from. The captured
 * program crops 3536 x 2652 out of the 4160 x 3120 YUVP surface and scales it
 * up to fill 4000 x 3000 -- a margin the vendor reserves for electronic
 * stabilisation, which this driver does not implement but must reproduce
 * exactly while the rest of the program is the captured one. The window is
 * centred, so only its size is a parameter.
 */
struct becore_mcsc_djag_profile {
	u32 crop_width;
	u32 crop_height;
};

static const struct becore_mcsc_djag_profile becore_mcsc_djag = {
	.crop_width = 4160,
	.crop_height = 3120,
};

enum becore_mcsc_input_transport {
	BECORE_MCSC_INPUT_CAPTURED_VOTF,
	BECORE_MCSC_INPUT_MEMORY,
};

enum becore_mcsc_dma_word {
	BECORE_MCSC_INPUT_VOTF,
	BECORE_MCSC_INPUT_FORMAT,
	BECORE_MCSC_INPUT_LOSSY,
	BECORE_MCSC_INPUT_COMP,
	BECORE_MCSC_INPUT_WIDTH,
	BECORE_MCSC_INPUT_HEIGHT,
	BECORE_MCSC_INPUT_STRIDE1,
	BECORE_MCSC_INPUT_STRIDE2,
	BECORE_MCSC_INPUT_BUSINFO,
	BECORE_MCSC_INPUT_MAX_BL,
	BECORE_MCSC_INPUT_ENABLE,
	BECORE_MCSC_DJAG_IMG_SIZE,
	BECORE_MCSC_DJAG_PS_SRC_POS,
	BECORE_MCSC_DJAG_PS_SRC_SIZE,
	BECORE_MCSC_DJAG_PS_DST_SIZE,
	BECORE_MCSC_DJAG_PS_H_RATIO,
	BECORE_MCSC_DJAG_PS_V_RATIO,
	BECORE_MCSC_OUTPUT_FORMAT,
	BECORE_MCSC_OUTPUT_COMP,
	BECORE_MCSC_OUTPUT_WIDTH,
	BECORE_MCSC_OUTPUT_HEIGHT,
	BECORE_MCSC_OUTPUT_STRIDE1,
	BECORE_MCSC_OUTPUT_STRIDE2,
	BECORE_MCSC_OUTPUT_BUSINFO,
	BECORE_MCSC_OUTPUT_MAX_BL,
	BECORE_MCSC_OUTPUT_ENABLE,
	BECORE_MCSC_OUTPUT_DITHER,
	BECORE_MCSC_DMA_WORD_COUNT,
};

static const u32 becore_mcsc_dma_regs[] = {
	[BECORE_MCSC_INPUT_VOTF] = BECORE_MCSC_INPUT_VOTF_REG,
	[BECORE_MCSC_INPUT_FORMAT] = BECORE_MCSC_INPUT_FORMAT_REG,
	[BECORE_MCSC_INPUT_LOSSY] = BECORE_MCSC_INPUT_LOSSY_REG,
	[BECORE_MCSC_INPUT_COMP] = BECORE_MCSC_INPUT_COMP_REG,
	[BECORE_MCSC_INPUT_WIDTH] = BECORE_MCSC_INPUT_WIDTH_REG,
	[BECORE_MCSC_INPUT_HEIGHT] = BECORE_MCSC_INPUT_HEIGHT_REG,
	[BECORE_MCSC_INPUT_STRIDE1] = BECORE_MCSC_INPUT_STRIDE1_REG,
	[BECORE_MCSC_INPUT_STRIDE2] = BECORE_MCSC_INPUT_STRIDE2_REG,
	[BECORE_MCSC_INPUT_BUSINFO] = BECORE_MCSC_INPUT_BUSINFO_REG,
	[BECORE_MCSC_INPUT_MAX_BL] = BECORE_MCSC_INPUT_MAX_BL_REG,
	[BECORE_MCSC_INPUT_ENABLE] = BECORE_MCSC_INPUT_ENABLE_REG,
	[BECORE_MCSC_OUTPUT_FORMAT] = BECORE_MCSC_OUTPUT_FORMAT_REG,
	[BECORE_MCSC_OUTPUT_COMP] = BECORE_MCSC_OUTPUT_COMP_REG,
	[BECORE_MCSC_OUTPUT_WIDTH] = BECORE_MCSC_OUTPUT_WIDTH_REG,
	[BECORE_MCSC_OUTPUT_HEIGHT] = BECORE_MCSC_OUTPUT_HEIGHT_REG,
	[BECORE_MCSC_OUTPUT_STRIDE1] = BECORE_MCSC_OUTPUT_STRIDE1_REG,
	[BECORE_MCSC_OUTPUT_STRIDE2] = BECORE_MCSC_OUTPUT_STRIDE2_REG,
	[BECORE_MCSC_OUTPUT_BUSINFO] = BECORE_MCSC_OUTPUT_BUSINFO_REG,
	[BECORE_MCSC_OUTPUT_MAX_BL] = BECORE_MCSC_OUTPUT_MAX_BL_REG,
	[BECORE_MCSC_OUTPUT_ENABLE] = BECORE_MCSC_OUTPUT_ENABLE_REG,
	[BECORE_MCSC_OUTPUT_DITHER] = BECORE_MCSC_OUTPUT_DITHER_REG,
	[BECORE_MCSC_DJAG_IMG_SIZE] = BECORE_MCSC_DJAG_IMG_SIZE_REG,
	[BECORE_MCSC_DJAG_PS_SRC_POS] = BECORE_MCSC_DJAG_PS_SRC_POS_REG,
	[BECORE_MCSC_DJAG_PS_SRC_SIZE] = BECORE_MCSC_DJAG_PS_SRC_SIZE_REG,
	[BECORE_MCSC_DJAG_PS_DST_SIZE] = BECORE_MCSC_DJAG_PS_DST_SIZE_REG,
	[BECORE_MCSC_DJAG_PS_H_RATIO] = BECORE_MCSC_DJAG_PS_H_RATIO_REG,
	[BECORE_MCSC_DJAG_PS_V_RATIO] = BECORE_MCSC_DJAG_PS_V_RATIO_REG,
};

struct becore_device;

#define BECORE_INPUT_SLOT_COUNT	3

struct becore_dma_buffer {
	void *cpu;
	dma_addr_t dma;
	size_t size;
	size_t staged_bytes;
	struct sg_table *sgt;
};

struct becore_ltm_gain_offset_group {
	__le16 gain[4];
	__le16 offset[4];
};

#define BECORE_LTM_GRID_GROUPS_PER_CELL \
	(BECORE_LTM_GRID_CELL_BYTES / \
	 sizeof(struct becore_ltm_gain_offset_group))

struct becore_ltm_grid_cell {
	struct becore_ltm_gain_offset_group
		groups[BECORE_LTM_GRID_GROUPS_PER_CELL];
};

enum becore_input_slot_state {
	BECORE_INPUT_FREE,
	BECORE_INPUT_PRODUCER,
	BECORE_INPUT_READY,
	BECORE_INPUT_BACKEND,
	BECORE_INPUT_QUARANTINED,
};

struct becore_input_slot {
	struct becore_dma_buffer buffer;
	enum becore_input_slot_state state;
	u64 producer_cookie;
	u64 ready_sequence;
};

struct becore_cmdq_program {
	void *cpu;
	dma_addr_t dma;
	size_t size;
	u32 header_count;
};

struct becore_block {
	struct becore_device *becore;
	const char *name;
	void __iomem *base;
	u32 int0_mask_prepare;
	u32 int0_mask;
	u32 int1_mask;
	u32 cmdq_int_mask;
	u32 last_int0;
	u32 last_int1;
	u32 last_cmdq_int;
	atomic64_t int0_count;
	atomic64_t int1_count;
};

struct becore_irq {
	struct becore_block *block;
	int irq;
	bool int1;
};

struct becore_video_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

static inline struct becore_video_buffer *
to_becore_video_buffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct becore_video_buffer, vb);
}

struct becore_device {
	struct device *dev;
	struct becore_block blocks[BECORE_NUM_BLOCKS];
	struct becore_irq irqs[BECORE_NUM_BLOCKS * 2];
	void __iomem *ssmt[14];
	void __iomem *sysreg_rgbp;
	void __iomem *sysreg_mcsc;
	struct dev_pm_domain_list *pm_domains;
	struct dentry *debugfs;
	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *red_balance;
	struct v4l2_ctrl *blue_balance;
	struct video_device vdev;
	struct media_pad vdev_pad;
	struct vb2_queue queue;
	/* Serializes V4L2 ioctls and vb2 queue setup/teardown. */
	struct mutex video_lock;
	/* Protects the pending processed-output buffer list. */
	spinlock_t queue_lock;
	struct list_head queued_outputs;
	struct work_struct video_work;
	/* Serializes staging, input ownership, execution, and output inspection. */
	struct mutex lock;
	/* Protects IRQ-driven command-hold/frame completion state. */
	spinlock_t run_lock;
	struct completion run_completion;
	struct becore_input_slot inputs[BECORE_INPUT_SLOT_COUNT];
	struct becore_input_slot *run_input;
	struct becore_dma_buffer grid;
	struct becore_dma_buffer output;
	struct becore_dma_buffer gtnr_output;
	struct becore_dma_buffer mcsc_output;
	struct exynos_becore_input *input_producer;
	u64 producer_sequence;
	u64 input_sequence;
	struct becore_cmdq_program program[BECORE_NUM_BLOCKS];
	struct becore_cmdq_program gtnr_program;
	struct becore_cmdq_program mcsc_program;
	u8 *recipe;
	u8 *gtnr_recipe;
	u8 *mcsc_recipe;
	size_t recipe_staged_bytes;
	size_t gtnr_recipe_staged_bytes;
	size_t mcsc_recipe_staged_bytes;
	u32 recipe_generation;
	u32 gtnr_recipe_generation;
	u32 gtnr_encoded_generation;
	u32 mcsc_recipe_generation;
	u32 mcsc_encoded_generation;
	u32 grid_generation;
	u32 run_generation;
	u32 completed_generation;
	u32 video_sequence;
	u32 cmdq_hold_mask;
	u32 frame_done_mask;
	u32 expected_mask;
	u32 output_changed_bytes;
	u32 output_first_changed;
	u32 mcsc_output_changed_bytes;
	u32 mcsc_output_first_changed;
	u32 output_profile;
	u32 active_output_profile;
	u32 mcsc_completed_generation;
	u32 mcsc_completed_output_size;
	enum becore_mcsc_input_transport mcsc_encoded_transport;
	dma_addr_t active_output_dma;
	size_t active_output_size;
	size_t active_capture_size;
	size_t completed_output_size;
	int last_run_result;
	bool running;
	bool start_issued;
	bool abort_run;
	bool irq_error;
	bool irqs_enabled;
	bool reset_failed;
	bool active_output_packed;
	bool active_mcsc;
	bool output_quarantined;
	bool video_streaming;
	bool producer_streaming;
};

struct exynos_becore_input {
	struct becore_device *becore;
	struct device *producer;
	const struct exynos_becore_input_producer_ops *ops;
	void *producer_data;
	refcount_t callback_users;
	wait_queue_head_t callback_wait;
	bool disconnected;
	struct sg_table sgts[BECORE_INPUT_SLOT_COUNT];
	dma_addr_t dmas[BECORE_INPUT_SLOT_COUNT];
};

static void becore_video_return_all(struct becore_device *becore,
				    enum vb2_buffer_state state);
static void becore_video_controls_ungrab(struct becore_device *becore);

static const char * const becore_pm_domain_names[] = {
	"yuvp",
	"rgbp",
	"gdc",
	"mcsc",
};

static const char * const becore_irq_names[] = {
	"rgbp-int0", "rgbp-int1",
	"mcfp-int0", "mcfp-int1",
	"yuvp-int0", "yuvp-int1",
	"mcsc-int0", "mcsc-int1",
};

static const char * const becore_ssmt_names[] = {
	"ssmt-rgbp",
	"ssmt-yuvp",
	"ssmt-mcfp0",
	"ssmt-mcfp1",
	"ssmt-mcfp2",
	"ssmt-mcfp3",
	"ssmt-mcfp4",
	"ssmt-mcsc0",
	"ssmt-mcsc1",
	"ssmt-mcsc2",
	"ssmt-mcsc3",
	"ssmt-mcsc4",
	"ssmt-mcsc5",
	"ssmt-mcsc6",
};

static const struct becore_regval becore_rgbp_init[] = {
	{ BECORE_SET_CTRL, 0x1 },
	{ BECORE_GLOBAL_ENABLE_CLEAR, 0x1 },
	{ BECORE_FRO_GLOBAL_ENABLE, 0x3fffff },
	{ BECORE_COMMAND_Q_STOP_ON_FRAME, 0x40 },
	{ BECORE_CMDQ_ENABLE, 0x1 },
	{ BECORE_STAT_RDMACL_EN, 0x1 },
	{ BECORE_GLOBAL_ENABLE, 0x1 },
	{ BECORE_RGBP_C_LOADER_ENABLE, 0x1 },
	{ BECORE_SET_CTRL, 0x0 },
};

static const struct becore_regval becore_mcfp_init[] = {
	{ BECORE_SET_CTRL, 0x1 },
	{ BECORE_GLOBAL_ENABLE_CLEAR, 0x1 },
	{ BECORE_FRO_GLOBAL_ENABLE, 0xffff },
	{ BECORE_COMMAND_Q_STOP_ON_FRAME, 0x40 },
	{ BECORE_CMDQ_ENABLE, 0x1 },
	{ BECORE_STAT_RDMACL_EN, 0x1 },
	{ BECORE_C_LOADER_ENABLE, 0x1 },
	{ BECORE_C_LOADER_MODE, 0x3 },
	{ BECORE_GLOBAL_ENABLE, 0x1 },
	{ BECORE_SET_CTRL, 0x0 },
};

static const struct becore_regval becore_yuvp_init[] = {
	{ BECORE_SET_CTRL, 0x1 },
	{ BECORE_GLOBAL_ENABLE_CLEAR, 0x1 },
	{ BECORE_FRO_GLOBAL_ENABLE, 0xedff },
	{ BECORE_COMMAND_Q_STOP_ON_FRAME, 0x40 },
	{ BECORE_CMDQ_ENABLE, 0x1 },
	{ BECORE_C_LOADER_ENABLE, 0x1 },
	{ BECORE_C_LOADER_MODE, 0x1b },
	{ BECORE_STAT_RDMACL_EN, 0x1 },
	{ BECORE_GLOBAL_ENABLE, 0x1 },
	{ BECORE_SET_CTRL, 0x0 },
};

/* MCSC keeps SET_CTRL asserted while its command queue is active. */
static const struct becore_regval becore_mcsc_init[] = {
	{ BECORE_SET_CTRL, 0x1 },
	{ BECORE_GLOBAL_ENABLE_CLEAR, 0x1 },
	{ BECORE_FRO_GLOBAL_ENABLE, 0x3fffff },
	{ BECORE_CMDQ_ENABLE, 0x1 },
	{ BECORE_STAT_RDMACL_EN, 0x1 },
	{ BECORE_GLOBAL_ENABLE, 0x1 },
};

static void becore_write_table(struct becore_block *block,
			       const struct becore_regval *table,
			       size_t count)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		writel_relaxed(table[i].value, block->base + table[i].offset);
}

static void becore_issue_reset(struct becore_block *block)
{
	if (block != &block->becore->blocks[BECORE_RGBP])
		writel_relaxed(0, block->base + BECORE_C_LOADER_ENABLE);

	writel_relaxed(1, block->base + BECORE_SW_RESET);
	writel_relaxed(0, block->base + BECORE_SET_CTRL);
}

static int becore_wait_reset(struct becore_block *block)
{
	u32 value;
	int ret;

	ret = readl_poll_timeout(block->base + BECORE_SW_RESET, value, !value,
				  1, BECORE_RESET_TIMEOUT_US);
	if (ret)
		dev_err(block->becore->dev, "%s reset timed out (0x%08x)\n",
			block->name, value);

	return ret;
}

static int becore_reset_all(struct becore_device *becore)
{
	int first_error = 0;
	unsigned int i;
	int ret;

	/* The OTF-connected chain has to receive reset as one hardware phase. */
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		becore_issue_reset(&becore->blocks[i]);

	/* Wait for every block even if an earlier one failed. */
	for (i = 0; i < BECORE_NUM_BLOCKS; i++) {
		ret = becore_wait_reset(&becore->blocks[i]);
		if (ret && !first_error)
			first_error = ret;
	}

	return first_error;
}

static size_t becore_cmdq_program_size(u32 header_count)
{
	return ALIGN((size_t)header_count * BECORE_CMDQ_HEADER_BYTES,
		     BECORE_CMDQ_PAYLOAD_BYTES) +
	       (size_t)header_count * BECORE_CMDQ_PAYLOAD_BYTES;
}

static const u8 *becore_recipe_records(const struct becore_device *becore,
				       enum becore_block_id id)
{
	const u8 *records = becore->recipe + BECORE_RECIPE_HEADER_BYTES;

	if (id == BECORE_YUVP)
		records += BECORE_RGBP_HEADER_COUNT * BECORE_RECIPE_RECORD_BYTES;

	return records;
}

static u32 becore_rgbp_input_storage_width(void)
{
	return ALIGN(becore_rgbp_input.width,
		     becore_rgbp_input.sbwc_block_width);
}

static u32 becore_rgbp_input_stride(void)
{
	return becore_rgbp_input_storage_width() *
	       becore_rgbp_input.bytes_per_pixel;
}

static size_t becore_rgbp_input_image_offset(void)
{
	return (size_t)becore_rgbp_input.header_stride *
	       becore_rgbp_input.height;
}

static size_t becore_rgbp_input_size(void)
{
	size_t image_bytes = (size_t)becore_rgbp_input_stride() *
			     becore_rgbp_input.height;

	return ALIGN(becore_rgbp_input_image_offset() + image_bytes, SZ_4K);
}

/*
 * Two conventions the Exynos ISP blocks share, so they live here rather than
 * with any one of them.
 *
 * Geometry registers pack two 16-bit halves into one word with the width in
 * the high one. Scaling ratios are a 20-bit fixed-point fraction of the
 * destination, truncated -- Samsung spells this GET_ZOOM_RATIO(in, out),
 * ((in) << MCSC_PRECISION) / (out) with MCSC_PRECISION 20. The captured
 * programs are the check on the rounding: DJAG's 3536 << 20 over 4000 is
 * 926941.18 and the vendor writes 926941.
 */
static u32 becore_pack_size(u32 high, u32 low)
{
	return (high << 16) | low;
}

static u32 becore_zoom_ratio(u32 in, u32 out)
{
	if (!out)
		return 0;

	return (u32)div_u64((u64)in << BECORE_RATIO_SHIFT, out);
}

/*
 * What RGBP hands downstream. It is the geometry YUVP then carries, so it is
 * taken from the YUVP profile rather than repeated here -- if the two ever
 * disagreed the chain would be describing two different images. Both YUVP
 * output profiles share it deliberately: this is the chain's geometry, not a
 * property of how the surface is encoded, so the diagnostic P010 selector must
 * not move it.
 */
static u32 becore_rgbp_out_width(void)
{
	return becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL].width;
}

static u32 becore_rgbp_out_height(void)
{
	return becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL].height;
}

/*
 * The crop is centred in the Bayer input, so its origin is derived. An odd
 * margin would put the window on the wrong Bayer phase and swap colours, so
 * refuse it rather than round.
 */
static int becore_rgbp_crop_origin(u32 *x, u32 *y)
{
	u32 w = becore_rgbp_out_width();
	u32 h = becore_rgbp_out_height();

	if (w > becore_rgbp_input.width || h > becore_rgbp_input.height)
		return -ERANGE;
	*x = (becore_rgbp_input.width - w) / 2;
	*y = (becore_rgbp_input.height - h) / 2;
	if ((*x | *y) & 1)
		return -ERANGE;

	return 0;
}

static int becore_rgbp_input_value(u32 index, u32 reg, u32 *value)
{
	if (index >= BECORE_RGBP_INPUT_WORD_COUNT ||
	    reg != becore_rgbp_input_regs[index])
		return -EINVAL;
	if (!value)
		return 0;

	switch (index) {
	case BECORE_RGBP_CHAIN_SRC_SIZE:
		*value = becore_pack_size(becore_rgbp_input.width,
					       becore_rgbp_input.height);
		break;
	case BECORE_RGBP_CHAIN_DST_SIZE:
	case BECORE_RGBP_CROP_SIZE:
	case BECORE_RGBP_SC_DST_SIZE:
		*value = becore_pack_size(becore_rgbp_out_width(),
					       becore_rgbp_out_height());
		break;
	case BECORE_RGBP_CROP_START: {
		u32 x, y;
		int ret = becore_rgbp_crop_origin(&x, &y);

		if (ret)
			return ret;
		*value = becore_pack_size(x, y);
		break;
	}
	case BECORE_RGBP_SC_H_RATIO:
		*value = becore_zoom_ratio(becore_rgbp_out_width(),
						becore_rgbp_out_width());
		break;
	case BECORE_RGBP_SC_V_RATIO:
		*value = becore_zoom_ratio(becore_rgbp_out_height(),
						becore_rgbp_out_height());
		break;
	case BECORE_RGBP_INPUT_FORMAT:
		*value = becore_rgbp_input.data_format;
		break;
	case BECORE_RGBP_INPUT_COMP:
		*value = becore_rgbp_input.comp_control;
		break;
	case BECORE_RGBP_INPUT_ACTIVE_WIDTH:
		*value = becore_rgbp_input.width;
		break;
	case BECORE_RGBP_INPUT_HEIGHT:
		*value = becore_rgbp_input.height;
		break;
	case BECORE_RGBP_INPUT_STRIDE:
		*value = becore_rgbp_input_stride();
		break;
	case BECORE_RGBP_INPUT_HEADER_STRIDE:
		*value = becore_rgbp_input.header_stride;
		break;
	case BECORE_RGBP_INPUT_BUSINFO:
		*value = becore_rgbp_input.businfo;
		break;
	case BECORE_RGBP_INPUT_ENABLE:
		*value = 1;
		break;
	case BECORE_RGBP_INPUT_STORAGE_WIDTH:
		*value = becore_rgbp_input_storage_width();
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct becore_yuvp_output_profile *
becore_yuvp_output_profile(const struct becore_device *becore)
{
	return &becore_yuvp_outputs[becore->active_output_profile];
}

static u32
becore_yuvp_output_stride(const struct becore_yuvp_output_profile *profile)
{
	return profile->width * profile->bytes_per_pixel;
}

static size_t
becore_yuvp_output_plane2_offset(const struct becore_yuvp_output_profile *profile)
{
	u32 stride = becore_yuvp_output_stride(profile);
	u32 luma_height;
	u32 luma_rows;

	if (!profile->mode)
		return ALIGN((size_t)stride * profile->height, SZ_4K);

	luma_height = ALIGN(profile->height, profile->luma_height_align);
	luma_rows = DIV_ROUND_UP(luma_height, profile->block_height);

	return (size_t)stride * luma_rows + profile->plane_gap;
}

static size_t
becore_yuvp_output_size(const struct becore_yuvp_output_profile *profile)
{
	u32 stride = becore_yuvp_output_stride(profile);
	u32 chroma_height = DIV_ROUND_UP(profile->height, 2);
	u32 chroma_rows;
	size_t chroma_bytes;

	if (profile->mode)
		chroma_rows = DIV_ROUND_UP(chroma_height, profile->block_height);
	else
		chroma_rows = chroma_height;
	chroma_bytes = (size_t)stride * chroma_rows;

	return ALIGN(becore_yuvp_output_plane2_offset(profile) + chroma_bytes,
		     SZ_4K);
}

static size_t
becore_yuvp_packed_plane2_offset(const struct becore_yuvp_output_profile *profile)
{
	return (size_t)becore_yuvp_output_stride(profile) * profile->height;
}

static size_t
becore_yuvp_packed_output_size(const struct becore_yuvp_output_profile *profile)
{
	u32 chroma_height = DIV_ROUND_UP(profile->height, 2);

	return becore_yuvp_packed_plane2_offset(profile) +
	       (size_t)becore_yuvp_output_stride(profile) * chroma_height;
}

static size_t becore_active_output_plane2_offset(const struct becore_device *becore)
{
	const struct becore_yuvp_output_profile *profile =
		becore_yuvp_output_profile(becore);

	if (becore->active_output_packed)
		return becore_yuvp_packed_plane2_offset(profile);

	return becore_yuvp_output_plane2_offset(profile);
}

static size_t becore_active_output_size(const struct becore_device *becore)
{
	const struct becore_yuvp_output_profile *profile =
		becore_yuvp_output_profile(becore);

	if (becore->active_output_packed)
		return becore_yuvp_packed_output_size(profile);

	return becore_yuvp_output_size(profile);
}

static size_t becore_yuvp_output_allocation_size(void)
{
	const struct becore_yuvp_output_profile *sbwcl =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];
	const struct becore_yuvp_output_profile *p010 =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_P010];

	return max(becore_yuvp_output_size(sbwcl),
		   becore_yuvp_output_size(p010));
}

static int becore_yuvp_output_value(struct becore_device *becore, u32 index,
				    u32 reg, u32 *value)
{
	const struct becore_yuvp_output_profile *profile =
		becore_yuvp_output_profile(becore);

	if (index >= BECORE_YUVP_OUTPUT_WORD_COUNT ||
	    reg != becore_yuvp_output_regs[index])
		return -EINVAL;
	if (!value)
		return 0;

	switch (index) {
	case BECORE_YUVP_OUTPUT_VOTF:
		*value = profile->votf_enable;
		break;
	case BECORE_YUVP_OUTPUT_FORMAT:
		*value = profile->data_format;
		break;
	case BECORE_YUVP_OUTPUT_LOSSY:
		*value = profile->lossy_byte32num;
		break;
	case BECORE_YUVP_OUTPUT_MODE:
		*value = profile->mode;
		break;
	case BECORE_YUVP_OUTPUT_WIDTH:
		*value = profile->width;
		break;
	case BECORE_YUVP_OUTPUT_HEIGHT:
		*value = profile->height;
		break;
	case BECORE_YUVP_OUTPUT_STRIDE1:
	case BECORE_YUVP_OUTPUT_STRIDE2:
		*value = becore_yuvp_output_stride(profile);
		break;
	case BECORE_YUVP_OUTPUT_BUSINFO:
		*value = profile->businfo;
		break;
	case BECORE_YUVP_OUTPUT_ENABLE:
		*value = 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int becore_gtnr_dma_value(u32 index, u32 reg, u32 *value)
{
	if (index >= BECORE_GTNR_DMA_WORD_COUNT ||
	    reg != becore_gtnr_dma_regs[index])
		return -EINVAL;
	if (!value)
		return 0;

	switch (index) {
	case BECORE_GTNR_INPUT_VOTF:
		*value = becore_gtnr_input.votf_enable;
		break;
	case BECORE_GTNR_INPUT_FORMAT:
		*value = becore_gtnr_input.data_format;
		break;
	case BECORE_GTNR_INPUT_LOSSY:
		*value = becore_gtnr_input.lossy_byte32num;
		break;
	case BECORE_GTNR_INPUT_COMP:
		*value = becore_gtnr_input.comp_control;
		break;
	case BECORE_GTNR_INPUT_WIDTH:
		*value = becore_gtnr_input.width;
		break;
	case BECORE_GTNR_INPUT_HEIGHT:
		*value = becore_gtnr_input.height;
		break;
	case BECORE_GTNR_INPUT_STRIDE1:
	case BECORE_GTNR_INPUT_STRIDE2:
		*value = becore_gtnr_input.stride;
		break;
	case BECORE_GTNR_INPUT_BUSINFO:
		*value = becore_gtnr_input.businfo;
		break;
	case BECORE_GTNR_INPUT_MAX_MO:
		*value = becore_gtnr_input.max_mo;
		break;
	case BECORE_GTNR_INPUT_MAX_BL:
		*value = becore_gtnr_input.max_bl;
		break;
	case BECORE_GTNR_INPUT_ENABLE:
		*value = becore_gtnr_input.enable;
		break;
	case BECORE_GTNR_OUTPUT_FORMAT:
		*value = becore_gtnr_output.data_format;
		break;
	case BECORE_GTNR_OUTPUT_LOSSY:
		*value = becore_gtnr_output.lossy_byte32num;
		break;
	case BECORE_GTNR_OUTPUT_COMP:
		*value = becore_gtnr_output.comp_control;
		break;
	case BECORE_GTNR_OUTPUT_WIDTH:
		*value = becore_gtnr_output.width;
		break;
	case BECORE_GTNR_OUTPUT_HEIGHT:
		*value = becore_gtnr_output.height;
		break;
	case BECORE_GTNR_OUTPUT_STRIDE1:
	case BECORE_GTNR_OUTPUT_STRIDE2:
		*value = becore_gtnr_output.stride;
		break;
	case BECORE_GTNR_OUTPUT_BUSINFO:
		*value = becore_gtnr_output.businfo;
		break;
	case BECORE_GTNR_OUTPUT_MAX_MO:
		*value = becore_gtnr_output.max_mo;
		break;
	case BECORE_GTNR_OUTPUT_MAX_BL:
		*value = becore_gtnr_output.max_bl;
		break;
	case BECORE_GTNR_OUTPUT_ENABLE:
		*value = becore_gtnr_output.enable;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static size_t becore_mcsc_output_plane2_offset(void)
{
	return (size_t)becore_mcsc_output.stride * becore_mcsc_output.height;
}

static size_t becore_mcsc_output_active_size(void)
{
	size_t chroma = (size_t)becore_mcsc_output.stride *
			DIV_ROUND_UP(becore_mcsc_output.height, 2);

	return becore_mcsc_output_plane2_offset() + chroma;
}

static size_t becore_mcsc_output_size(void)
{
	return ALIGN(becore_mcsc_output_active_size(), SZ_4K);
}

/*
 * The crop is centred in the scaler's input, so its origin is derived rather
 * than carried. An odd margin would land the window off a chroma boundary on a
 * 4:2:0 output, so refuse it instead of silently rounding.
 */
static int becore_mcsc_djag_origin(u32 *x, u32 *y)
{
	if (becore_mcsc_djag.crop_width > becore_mcsc_input.width ||
	    becore_mcsc_djag.crop_height > becore_mcsc_input.height)
		return -ERANGE;
	*x = (becore_mcsc_input.width - becore_mcsc_djag.crop_width) / 2;
	*y = (becore_mcsc_input.height - becore_mcsc_djag.crop_height) / 2;
	if ((*x | *y) & 1)
		return -ERANGE;

	return 0;
}

static int
becore_mcsc_dma_value(u32 index, u32 reg,
		      enum becore_mcsc_input_transport transport, u32 *value)
{
	if (index >= BECORE_MCSC_DMA_WORD_COUNT ||
	    reg != becore_mcsc_dma_regs[index])
		return -EINVAL;
	if (!value)
		return 0;

	switch (index) {
	case BECORE_MCSC_INPUT_VOTF:
		*value = transport == BECORE_MCSC_INPUT_MEMORY ? 0 :
			 becore_mcsc_input.votf_enable;
		break;
	case BECORE_MCSC_INPUT_FORMAT:
		*value = becore_mcsc_input.data_format;
		break;
	case BECORE_MCSC_INPUT_LOSSY:
		*value = becore_mcsc_input.lossy_byte32num;
		break;
	case BECORE_MCSC_INPUT_COMP:
		*value = becore_mcsc_input.comp_control;
		break;
	case BECORE_MCSC_INPUT_WIDTH:
		*value = becore_mcsc_input.width;
		break;
	case BECORE_MCSC_INPUT_HEIGHT:
		*value = becore_mcsc_input.height;
		break;
	case BECORE_MCSC_INPUT_STRIDE1:
	case BECORE_MCSC_INPUT_STRIDE2:
		*value = becore_mcsc_input.stride;
		break;
	case BECORE_MCSC_INPUT_BUSINFO:
		*value = becore_mcsc_input.businfo;
		break;
	case BECORE_MCSC_INPUT_MAX_BL:
		*value = becore_mcsc_input.max_bl;
		break;
	case BECORE_MCSC_INPUT_ENABLE:
		*value = becore_mcsc_input.enable;
		break;
	case BECORE_MCSC_OUTPUT_FORMAT:
		*value = becore_mcsc_output.data_format;
		break;
	case BECORE_MCSC_OUTPUT_COMP:
		*value = becore_mcsc_output.comp_control;
		break;
	case BECORE_MCSC_OUTPUT_WIDTH:
		*value = becore_mcsc_output.width;
		break;
	case BECORE_MCSC_OUTPUT_HEIGHT:
		*value = becore_mcsc_output.height;
		break;
	case BECORE_MCSC_OUTPUT_STRIDE1:
	case BECORE_MCSC_OUTPUT_STRIDE2:
		*value = becore_mcsc_output.stride;
		break;
	case BECORE_MCSC_OUTPUT_BUSINFO:
		*value = becore_mcsc_output.businfo;
		break;
	case BECORE_MCSC_OUTPUT_MAX_BL:
		*value = becore_mcsc_output.max_bl;
		break;
	case BECORE_MCSC_OUTPUT_ENABLE:
		*value = becore_mcsc_output.enable;
		break;
	case BECORE_MCSC_OUTPUT_DITHER:
		*value = becore_mcsc_output.dither;
		break;
	case BECORE_MCSC_DJAG_IMG_SIZE:
		*value = becore_pack_size(becore_mcsc_input.width,
						 becore_mcsc_input.height);
		break;
	case BECORE_MCSC_DJAG_PS_SRC_POS: {
		u32 x, y;
		int ret = becore_mcsc_djag_origin(&x, &y);

		if (ret)
			return ret;
		*value = becore_pack_size(x, y);
		break;
	}
	case BECORE_MCSC_DJAG_PS_SRC_SIZE:
		*value = becore_pack_size(becore_mcsc_djag.crop_width,
						 becore_mcsc_djag.crop_height);
		break;
	case BECORE_MCSC_DJAG_PS_DST_SIZE:
		*value = becore_pack_size(becore_mcsc_output.width,
						 becore_mcsc_output.height);
		break;
	case BECORE_MCSC_DJAG_PS_H_RATIO:
		*value = becore_zoom_ratio(becore_mcsc_djag.crop_width,
						  becore_mcsc_output.width);
		break;
	case BECORE_MCSC_DJAG_PS_V_RATIO:
		*value = becore_zoom_ratio(becore_mcsc_djag.crop_height,
						  becore_mcsc_output.height);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static size_t becore_gtnr_surface_plane2_offset(void)
{
	const struct becore_yuvp_output_profile *profile =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];

	return becore_yuvp_output_plane2_offset(profile);
}

static size_t becore_gtnr_surface_size(void)
{
	const struct becore_yuvp_output_profile *profile =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];

	return becore_yuvp_output_size(profile);
}

static dma_addr_t becore_gtnr_address_dma(struct becore_device *becore, u32 reg)
{
	switch (reg) {
	case BECORE_GTNR_INPUT_PLANE1_REG:
		return becore->output.dma;
	case BECORE_GTNR_INPUT_PLANE2_REG:
		return becore->output.dma + becore_gtnr_surface_plane2_offset();
	case BECORE_GTNR_OUTPUT_PLANE1_REG:
		return becore->gtnr_output.dma;
	case BECORE_GTNR_OUTPUT_PLANE2_REG:
		return becore->gtnr_output.dma +
		       becore_gtnr_surface_plane2_offset();
	default:
		return DMA_MAPPING_ERROR;
	}
}

static dma_addr_t becore_mcsc_address_dma(struct becore_device *becore, u32 reg)
{
	const struct becore_yuvp_output_profile *input =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];

	switch (reg) {
	case BECORE_MCSC_INPUT_PLANE1_REG:
		return becore->output.dma;
	case BECORE_MCSC_INPUT_PLANE2_REG:
		return becore->output.dma + becore_yuvp_output_plane2_offset(input);
	case BECORE_MCSC_OUTPUT_PLANE1_REG:
		return becore->mcsc_output.dma;
	case BECORE_MCSC_OUTPUT_PLANE2_REG:
		return becore->mcsc_output.dma +
		       becore_mcsc_output_plane2_offset();
	default:
		return DMA_MAPPING_ERROR;
	}
}

static u32 becore_typed_word_count(enum becore_block_id id)
{
	if (id == BECORE_RGBP)
		return BECORE_RGBP_INPUT_WORD_COUNT;
	if (id == BECORE_YUVP)
		return BECORE_YUVP_OUTPUT_WORD_COUNT;

	return 0;
}

static int becore_typed_value(struct becore_device *becore,
			      enum becore_block_id id, u32 index, u32 reg,
			      u32 *value)
{
	if (id == BECORE_RGBP)
		return becore_rgbp_input_value(index, reg, value);
	if (id == BECORE_YUVP)
		return becore_yuvp_output_value(becore, index, reg, value);

	return -EINVAL;
}

/*
 * The register a value word programs. Pair-mode headers carry it beside the
 * value; sequential-mode ones step from the header's target.
 */
static int becore_shape_register(const struct becore_cmdq_shape *shape,
				 u32 word, u32 *reg)
{
	if (word >= 16)
		return -EINVAL;
	if (shape->mode == 0x00090000) {
		if (!(word & 1))
			return -EINVAL;
		*reg = shape->pair_registers[word / 2];
		return 0;
	}
	if (shape->mode == 0x00080000) {
		*reg = shape->target + word * 4;
		return 0;
	}

	return -EINVAL;
}

/*
 * The knot grid: 16 knots every 16 codes across the first 256, then 8 every
 * 32, 24 every 64 and 16 every 128. Each knot is the left edge of its segment
 * and the segments tile the 12-bit input exactly, so the last knot is 3968
 * rather than 4095. Finer where the eye is, which is the ordinary reason a
 * tone curve is sampled unevenly; the grid decides where the curve is
 * measured, not what it does.
 *
 * tools/camera-becore-recipe.py carries the same arithmetic and checks it
 * against the vendor capture, which is the only thing that checks it -- the
 * recipe holds zero for these words, so nothing here can be validated against
 * it. Change one copy and change the other.
 */
static const struct {
	u8 knots;
	u16 step;
} becore_rgbp_gtm_grid[] = {
	{ 16, 16 }, { 8, 32 }, { 24, 64 }, { 16, 128 },
};

static_assert(16 + 8 + 24 + 16 == BECORE_RGBP_GTM_KNOTS);
static_assert(16 * 16 + 8 * 32 + 24 * 64 + 16 * 128 == 1 << 12);

static int becore_rgbp_gtm_knot(u32 index, u32 *knot)
{
	u32 x = 0;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_rgbp_gtm_grid); i++) {
		if (index < becore_rgbp_gtm_grid[i].knots) {
			*knot = x + index * becore_rgbp_gtm_grid[i].step;
			return 0;
		}
		x += becore_rgbp_gtm_grid[i].knots *
		     becore_rgbp_gtm_grid[i].step;
		index -= becore_rgbp_gtm_grid[i].knots;
	}

	return -EINVAL;
}

/*
 * YUVP local tone mapping, by offset from BECORE_YUVP_LTM_BASE. What the block
 * is for, rather than what one scene wanted from it: it forms a guide luma
 * from RGB, looks that up in a tone curve and applies a spatial gain grid. The
 * curve and the CONFIG words are the tuning and stay in the recipe; the gate,
 * the luma weights, the grid the frame is divided into and the vendor's own
 * unity fills are all stateable.
 */
static int becore_yuvp_ltm_value(u32 offset, u32 *value)
{
	u32 cell;

	if (offset & 3)
		return -EINVAL;
	/*
	 * The grid's cells are square on a 4:3 frame, which is why the
	 * horizontal and vertical reciprocals below come out equal. A geometry
	 * where they are not would need two of them and would mean the frame
	 * is no longer 4:3, so refuse the whole block rather than program half
	 * a grid -- and refuse it here rather than only when a reciprocal is
	 * asked for, so this matches the generator that checks these values.
	 */
	cell = becore_rgbp_out_width() / BECORE_LTM_SLCGRID_COLUMNS;
	if (!cell || cell != becore_rgbp_out_height() / BECORE_LTM_SLCGRID_ROWS)
		return -ERANGE;

	switch (offset) {
	case 0x000:				/* LTM_ENABLE: the block runs */
		*value = 1;
		return 0;
	case 0x014:				/* LUMACALC_RGBY_COEFF_R */
		*value = BECORE_LTM_LUMA_Q12_R;
		return 0;
	case 0x018:				/* ..._COEFF_G */
		*value = BECORE_LTM_LUMA_Q12_G;
		return 0;
	case 0x01c:				/* ..._COEFF_B */
		*value = BECORE_LTM_LUMA_Q12_B;
		return 0;
	case 0x120:				/* SLCGRID_START_X_POS */
	case 0x124:				/* SLCGRID_START_Y_POS */
		*value = 0;
		return 0;
	case 0x128:				/* SLCGRID_GRID_DEPTH */
		*value = BECORE_LTM_SLCGRID_DEPTH;
		return 0;
	case 0x12c:				/* SLCGRID_GRID_WIDTH */
		*value = BECORE_LTM_SLCGRID_COLUMNS;
		return 0;
	case 0x130:				/* SLCGRID_GRID_HEIGHT */
		*value = BECORE_LTM_SLCGRID_ROWS;
		return 0;
	}

	if (offset >= 0x2dc && offset <= 0x4dc) {
		/* CRECON_SATCTRL_LUT: saturation control contributes nothing. */
		*value = 0;
		return 0;
	}

	if (offset >= 0x4e0 && offset <= 0x6d4) {
		/* CRECON_LUMA_LUT and _GAIN_LUT: unity, as GetDefaultLtm fills them. */
		*value = BECORE_LTM_UNITY_Q8_PAIR;
		return 0;
	}

	/* The remaining four are the reciprocals it steps the grid with. */
	switch (offset) {
	case 0x134:				/* SLCGRID_GRID_X_SCALE */
	case 0x13c:				/* SLCGRID_GRID_Y_SCALE */
		*value = 65536 / cell;
		return 0;
	case 0x138:				/* SLCGRID_GRID_X_SCALE_HALF */
	case 0x140:				/* SLCGRID_GRID_Y_SCALE_HALF */
		*value = 32768 / cell;
		return 0;
	}

	return -EINVAL;
}

/*
 * Read one fixed word back out of the recipe by register address.
 *
 * Every other generated value is a function of the hardware description or of
 * a constant, and resolves from the register alone. A noise curve's slopes are
 * a function of its knots, which are tuning and stay in the recipe -- so
 * resolving one means finding a sibling register's value.
 *
 * What makes that safe is not the order this runs in: it reads the
 * compiled-in table, so its answer does not depend on validation having run.
 * It is that the two ends agree by construction. Validation refuses any staged
 * recipe whose fixed words differ from these, and encode copies the staged
 * payload and then overwrites only address, typed and generated words -- so a
 * knot that is programmed is always the knot a slope was derived from.
 *
 * A command list's last write to a register is the one the hardware keeps, and
 * this recipe really does write one YUVP register twice with two values, so
 * take the last match rather than the first. A register carried only by a
 * repeated-target header, or one that is not a fixed word at all, is invisible
 * here and returns -EINVAL, which fails the encode rather than guessing.
 */
static int becore_recipe_fixed_value(enum becore_block_id id, u32 reg,
				     u32 *value)
{
	const struct becore_cmdq_shape *shape;
	bool found = false;
	u32 count;
	u32 i;
	u32 word;

	switch (id) {
	case BECORE_RGBP:
		shape = becore_rgbp_shape;
		count = BECORE_RGBP_HEADER_COUNT;
		break;
	case BECORE_YUVP:
		shape = becore_yuvp_shape;
		count = BECORE_YUVP_HEADER_COUNT;
		break;
	case BECORE_MCSC:
		shape = becore_mcsc_shape;
		count = BECORE_MCSC_HEADER_COUNT;
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		for (word = 0; word < shape[i].valid_words; word++) {
			u32 candidate;

			if (!(shape[i].fixed_mask & BIT(word)))
				continue;
			if (becore_shape_register(&shape[i], word, &candidate) ||
			    candidate != reg)
				continue;
			*value = shape[i].fixed_values[word];
			found = true;
		}
	}

	return found ? 0 : -EINVAL;
}

/*
 * Each entry names the knots its slopes come from, so a chroma curve whose own
 * domain registers this table generates points at the luma copy it repeats
 * rather than at itself -- those would read back as zero.
 */
struct becore_noise_curve {
	enum becore_block_id block;
	u32 x_first;
	u32 y_first;
	u32 slope_first;
	u32 shift_reg;
	u32 domain_first;	/* 0 when the curve owns its own domain */
	bool round;		/* to nearest; false truncates */
};

static const struct becore_noise_curve becore_noise_curves[] = {
	{ BECORE_RGBP, BECORE_RGBP_DNS_X_G_REG, BECORE_RGBP_DNS_Y_G_REG,
	  BECORE_RGBP_DNS_SLOPE_G_REG, BECORE_RGBP_DNS_SHIFT_G_REG, 0, true },
	{ BECORE_RGBP, BECORE_RGBP_DNS_X_G_REG, BECORE_RGBP_DNS_Y_RB_REG,
	  BECORE_RGBP_DNS_SLOPE_RB_REG, BECORE_RGBP_DNS_SHIFT_RB_REG,
	  BECORE_RGBP_DNS_X_RB_REG, true },
	{ BECORE_YUVP, BECORE_YUVP_NR_X_Y_REG, BECORE_YUVP_NR_Y_Y_REG,
	  BECORE_YUVP_NR_SLOPE_Y_REG, BECORE_YUVP_NR_SHIFT_Y_REG, 0, false },
	{ BECORE_YUVP, BECORE_YUVP_NR_X_Y_REG, BECORE_YUVP_NR_Y_UV_REG,
	  BECORE_YUVP_NR_SLOPE_UV_REG, BECORE_YUVP_NR_SHIFT_UV_REG,
	  BECORE_YUVP_NR_X_UV_REG, false },
};

static int becore_noise_curve_for(enum becore_block_id id, u32 reg, u32 kind,
				  size_t *found)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_noise_curves); i++) {
		const struct becore_noise_curve *curve = &becore_noise_curves[i];
		u32 first;
		u32 regs;

		if (curve->block != id)
			continue;
		if (kind == BECORE_GEN_NOISE_SLOPE) {
			first = curve->slope_first;
			regs = BECORE_NOISE_TABLE_REGS;
		} else if (kind == BECORE_GEN_NOISE_DOMAIN) {
			first = curve->domain_first;
			regs = BECORE_NOISE_TABLE_REGS;
		} else {
			first = curve->shift_reg;
			regs = 1;
		}
		if (first && reg >= first && reg < first + regs * 4) {
			*found = i;
			return 0;
		}
	}

	return -EINVAL;
}

/*
 * The knots resolved once, rather than scanned for on every frame.
 *
 * The recipe's fixed words are compile-time constants, so a curve's knots are
 * too, but finding one means walking every header of its block. That walk is
 * cheap once and expensive per frame -- the encode path runs it for all 396
 * headers of a submission -- so it happens at probe and the result is what the
 * per-frame arithmetic reads.
 */
struct becore_noise_knots {
	s32 x[BECORE_NOISE_KNOTS];
	s32 y[BECORE_NOISE_KNOTS];
};

static struct becore_noise_knots
	becore_noise_knots[ARRAY_SIZE(becore_noise_curves)];
static bool becore_noise_knots_ready;

/* Knot `index` of an eight-knot table: two 16-bit knots per register. */
static int becore_noise_read_knot(enum becore_block_id id, u32 first,
				  u32 index, s32 *knot)
{
	u32 word;
	int ret;

	if (index >= BECORE_NOISE_KNOTS)
		return -EINVAL;
	ret = becore_recipe_fixed_value(id, first + (index / 2) * 4, &word);
	if (ret)
		return ret;
	*knot = (index & 1) ? (word >> 16) & 0xffff : word & 0xffff;

	return 0;
}

/*
 * Resolve every curve's knots, and check each domain rises while we are here.
 * A curve that cannot be resolved is a wiring mistake in the table above --
 * most likely a chroma curve pointed at its own generated domain instead of
 * the luma copy it repeats -- and it is worth failing probe over rather than
 * discovering at the first STREAMON.
 */
static int becore_noise_knots_resolve(struct device *dev)
{
	size_t i;
	u32 index;
	int ret;

	for (i = 0; i < ARRAY_SIZE(becore_noise_curves); i++) {
		const struct becore_noise_curve *curve = &becore_noise_curves[i];

		for (index = 0; index < BECORE_NOISE_KNOTS; index++) {
			ret = becore_noise_read_knot(curve->block,
						     curve->x_first, index,
						     &becore_noise_knots[i].x[index]);
			if (ret)
				return dev_err_probe(dev, ret,
						     "noise curve %zu has no knot domain\n",
						     i);
			ret = becore_noise_read_knot(curve->block,
						     curve->y_first, index,
						     &becore_noise_knots[i].y[index]);
			if (ret)
				return dev_err_probe(dev, ret,
						     "noise curve %zu has no knot range\n",
						     i);
		}
		for (index = 1; index < BECORE_NOISE_KNOTS; index++) {
			if (becore_noise_knots[i].x[index] <=
			    becore_noise_knots[i].x[index - 1])
				return dev_err_probe(dev, -ERANGE,
						     "noise curve %zu's domain does not rise\n",
						     i);
		}
	}
	becore_noise_knots_ready = true;

	return 0;
}

/*
 * One segment's slope. There are eight slope fields for seven segments, so the
 * eighth repeats the seventh: past the last knot the curve does not turn.
 */
static int becore_noise_slope(size_t curve_index, u32 index, u32 *slope)
{
	const struct becore_noise_curve *curve =
		&becore_noise_curves[curve_index];
	const struct becore_noise_knots *knots =
		&becore_noise_knots[curve_index];
	s32 magnitude;
	s32 quotient;
	s32 delta;
	s32 dx;

	if (index >= BECORE_NOISE_KNOTS - 1)
		index = BECORE_NOISE_KNOTS - 2;
	dx = knots->x[index + 1] - knots->x[index];
	if (dx <= 0)
		return -ERANGE;
	delta = knots->y[index + 1] - knots->y[index];
	magnitude = (delta < 0 ? -delta : delta) << BECORE_NOISE_SLOPE_SHIFT;
	if (curve->round)
		quotient = (2 * magnitude + dx) / (2 * dx);
	else
		quotient = magnitude / dx;
	if (delta < 0)
		quotient = -quotient;
	if (quotient > (s32)(BECORE_NOISE_SLOPE_MASK >> 1) ||
	    quotient < -(s32)(BECORE_NOISE_SLOPE_MASK >> 1) - 1)
		return -ERANGE;
	*slope = (u32)quotient & BECORE_NOISE_SLOPE_MASK;

	return 0;
}

static int becore_noise_value(enum becore_block_id id, u32 reg, u32 kind,
			      u32 *value)
{
	size_t curve_index;
	u32 packed = 0;
	u32 index;
	u32 high;
	u32 low;
	u32 i;
	int ret;

	if (!becore_noise_knots_ready)
		return -EINVAL;
	ret = becore_noise_curve_for(id, reg, kind, &curve_index);
	if (ret)
		return ret;
	if (kind == BECORE_GEN_NOISE_SHIFT) {
		for (i = 0; i < BECORE_NOISE_SHIFT_NIBBLES; i++)
			packed |= (u32)BECORE_NOISE_SLOPE_SHIFT << (4 * i);
		*value = packed;
		return 0;
	}
	if (kind == BECORE_GEN_NOISE_DOMAIN) {
		const struct becore_noise_knots *knots =
			&becore_noise_knots[curve_index];

		index = (reg - becore_noise_curves[curve_index].domain_first) /
			4 * 2;
		if (index + 1 >= BECORE_NOISE_KNOTS)
			return -EINVAL;
		*value = (((u32)knots->x[index + 1] & 0xffff) << 16) |
			 ((u32)knots->x[index] & 0xffff);
		return 0;
	}

	index = (reg - becore_noise_curves[curve_index].slope_first) / 4 * 2;
	ret = becore_noise_slope(curve_index, index, &low);
	if (ret)
		return ret;
	ret = becore_noise_slope(curve_index, index + 1, &high);
	if (ret)
		return ret;
	*value = (high << 16) | low;

	return 0;
}

/* Samsung's init_djag_cfgs, which is also this field's POR reset value. */
static const u16 becore_djag_lfsr_seeds[] = { 44257, 4671, 47792 };
static const u8 becore_djag_dither_ramp[] = { 0, 0, 1, 2, 3, 4, 6, 7, 8 };

/*
 * The literals GetDefaultDmsc(DmscRgbpOutput&) writes once the tuning path has
 * run, so they are the same whatever the scene was.
 */
static const struct becore_regval becore_dmsc_defaults[] = {
	{ 0x20c, 0x36a },		/* BASE_CONFIG */
	{ 0x238, 0x07f },		/* EXTRACT_COLORS_CONFIG */
	{ 0x24c, 0x00d500b4 },		/* GREEN_HUE: min 0xb4, max 0xd5 */
	{ 0x250, 0x0dac0046 },		/* GREEN_SAT: min 0x46, max 0xdac */
	{ 0x258, 0xfff },		/* POST_PROCESS_CONFIG */
	{ 0x264, 0x508 },		/* DIR_DETECT_SELECTION */
	{ 0x26c, 0x000600a0 },		/* ADD_COLORS_GREEN */
	{ 0x270, 0x120 },		/* FALSE_COLORS */
	{ 0x274, 0x003 },		/* SHARPENING_CONFIG */
	{ 0x288, 0x001 },		/* NEAR_EDGE_DESAT_EN */
	{ 0x2a0, 0x001 },		/* EDGE_DESAT_RED_PRESERVE_EN */
	{ 0x2a8, 0x000 },		/* EDGE_DESAT_RED_PRESERVE_THRES */
	{ 0x2ac, 0x800 },		/* EDGE_DESAT_RED_PRESERVE_LIMIT */
	{ 0x2b4, 0x011 },		/* ADD_YBLUR */
};

static int becore_mcsc_djag_value(u32 offset, u32 *value)
{
	u32 packed = 0;
	u32 first;
	u32 count;
	u32 i;

	switch (offset) {
	case 0x000:		/* CTRL: DJAG, its pre-scaler and EZ post on */
		*value = BIT(0) | BIT(1) | BIT(10);
		return 0;
	case 0x01c:		/* PS_H_INIT_PHASE_OFFSET */
	case 0x020:		/* PS_V_INIT_PHASE_OFFSET */
	case 0x080:		/* RECOM_CTRL: detail restoration is off */
	case 0x088:		/* RECOM_WEIGHT: and off by its weight too */
		*value = 0;
		return 0;
	case 0x024:		/* PS_ROUND_MODE */
		*value = 1;
		return 0;
	case 0x050:		/* LFSR_SEED_0 */
	case 0x054:		/* LFSR_SEED_1 */
	case 0x058:		/* LFSR_SEED_2 */
		*value = becore_djag_lfsr_seeds[(offset - 0x050) / 4];
		return 0;
	case 0x05c:		/* DITHER_VALUE_04: five 6-bit steps */
	case 0x060:		/* DITHER_VALUE_58: the remaining four */
		first = offset == 0x05c ? 0 : 5;
		count = offset == 0x05c ? 5 : 4;
		for (i = 0; i < count; i++)
			packed |= (u32)becore_djag_dither_ramp[first + i] <<
				  (BECORE_DJAG_DITHER_FIELD_BITS * i);
		*value = packed;
		return 0;
	case 0x064:		/* DITHER_THRES, which also carries SAT_CTRL */
		*value = BECORE_DJAG_SAT_CTRL |
			 BECORE_DJAG_DITHER_THRES <<
			 BECORE_DJAG_DITHER_THRES_SHIFT;
		return 0;
	case 0x068:		/* CP_HF_THRES */
		*value = BECORE_DJAG_CP_HF_THRES;
		return 0;
	}

	return -EINVAL;
}

static int becore_rgbp_dmsc_value(u32 offset, u32 *value)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_dmsc_defaults); i++) {
		if (becore_dmsc_defaults[i].offset != offset)
			continue;
		*value = becore_dmsc_defaults[i].value;
		return 0;
	}

	return -EINVAL;
}

/* Where each block's scaler puts its two init phase offsets and round mode. */
static const u32 becore_scaler_phase_first[] = {
	BECORE_RGBP_SC_PHASE_FIRST,
	BECORE_MCSC_SC0_PHASE_FIRST,
	BECORE_MCSC_PC0_PHASE_FIRST,
};

static int becore_scaler_phase_value(u32 reg, u32 *value)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_scaler_phase_first); i++) {
		u32 first = becore_scaler_phase_first[i];

		if (reg < first || reg > first + BECORE_SCALER_PHASE_LAST)
			continue;
		switch (reg - first) {
		case 0x00:	/* H_INIT_PHASE_OFFSET: no sub-pixel origin */
		case 0x04:	/* V_INIT_PHASE_OFFSET */
			*value = 0;
			return 0;
		case 0x08:	/* ROUND_MODE, on the MCSC blocks only */
			*value = 1;
			return 0;
		}
		return -EINVAL;
	}

	return -EINVAL;
}

/*
 * BT.601 as exact rationals, column-major by input channel. Kr is 299/1000 and
 * Kb 114/1000; the chroma rows are those over 2 * (1 - Kb) and 2 * (1 - Kr),
 * whose denominators are 1772 and 1402.
 */
static const struct becore_csc_coefficient {
	s32 numerator;
	s32 denominator;
} becore_csc_matrix[3][3] = {
	{ { 299, 1000 }, { -299, 1772 }, { 1, 2 } },
	{ { 587, 1000 }, { -587, 1772 }, { -587, 1402 } },
	{ { 114, 1000 }, { 1, 2 }, { -114, 1402 } },
};

/* [1, 2, 1] / 4 scaled by 32, one byte per tap. */
static const u8 becore_chroma_lpf_taps[] = { 0, 32, 64, 32, 0 };

/* The sharpener's three low-pass kernels sum to these; all powers of two. */
static const u32 becore_sharpenhancer_lpf_sums[] = { 16, 512, 4096 };

static u32 becore_csc_coefficient(const struct becore_csc_coefficient *coef)
{
	s32 magnitude = coef->numerator < 0 ? -coef->numerator : coef->numerator;

	magnitude = (magnitude * (1 << BECORE_RGBP_CSC_Q) * 2 +
		     coef->denominator) / (2 * coef->denominator);
	if (coef->numerator < 0)
		magnitude = -magnitude;

	return magnitude & BECORE_RGBP_CSC_FIELD_MASK;
}

static int becore_rgbp_csc_value(u32 offset, u32 *value)
{
	if (offset & 3)
		return -EINVAL;
	if (offset >= 0x04 && offset < 0x28) {
		u32 index = (offset - 0x04) / 4;

		*value = becore_csc_coefficient(&becore_csc_matrix[index / 3]
								 [index % 3]);
		return 0;
	}

	switch (offset) {
	case 0x00:		/* BYPASS: the block runs */
	case 0x28:		/* YMIN */
	case 0x30:		/* UMIN */
	case 0x38:		/* VMIN */
	case 0x40:		/* LS: no post-matrix shift */
	case 0x44:		/* YOS: full range puts luma's offset at zero */
		*value = 0;
		return 0;
	case 0x2c:		/* YMAX */
	case 0x34:		/* UMAX */
	case 0x3c:		/* VMAX */
		*value = BECORE_RGBP_CSC_MAX;
		return 0;
	case 0x48:		/* UOS */
	case 0x4c:		/* VOS */
		*value = BECORE_RGBP_CSC_CHROMA_OFFSET;
		return 0;
	}

	return -EINVAL;
}

static int becore_rgbp_chroma_lpf_value(u32 offset, u32 *value)
{
	u32 packed = 0;
	size_t i;

	switch (offset) {
	case 0x00:		/* ISP_BYPASS: the block runs */
		*value = 0;
		return 0;
	case 0x08:		/* COEFFS: the first four taps, one byte each */
		for (i = 0; i < 4; i++)
			packed |= (u32)becore_chroma_lpf_taps[i] << (8 * i);
		*value = packed;
		return 0;
	case 0x0c:		/* COEFFS_1: the fifth */
		*value = becore_chroma_lpf_taps[4];
		return 0;
	}

	return -EINVAL;
}

static int becore_rgbp_dns_geometry_value(u32 offset, u32 *value)
{
	s32 x;
	s32 y;

	switch (offset) {
	case 0x1a4:		/* BINNING: Q10, x in [0:13], y in [16:29] */
		*value = (BECORE_RGBP_DNS_BINNING_UNITY << 16) |
			 BECORE_RGBP_DNS_BINNING_UNITY;
		return 0;
	case 0x1c0:		/* RADIAL_CENTER: 15-bit signed, x low, y high */
		x = -(s32)((becore_rgbp_input.width >> 1) & ~1u);
		y = -(s32)((becore_rgbp_input.height >> 1) & ~1u);
		*value = ((y & BECORE_RGBP_DNS_CENTRE_MASK) << 16) |
			 (x & BECORE_RGBP_DNS_CENTRE_MASK);
		return 0;
	}

	return -EINVAL;
}

/* log2 of each kernel's sum, packed at bits 0, 8 and 16. */
static int becore_yuvp_lpf_norm_value(u32 *value)
{
	u32 packed = 0;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_sharpenhancer_lpf_sums); i++) {
		u32 total = becore_sharpenhancer_lpf_sums[i];

		if (!is_power_of_2(total))
			return -EINVAL;
		packed |= (u32)(ilog2(total)) << (8 * i);
	}
	*value = packed;

	return 0;
}

/* The x grid: finest where a square root moves fastest, tiling Q12 exactly. */
static int becore_rgbp_gamma_knot(u32 index, u32 *x)
{
	static const struct {
		u8 count;
		u16 step;
	} segments[] = {
		{ 8, 8 }, { 12, 16 }, { 8, 32 }, { 16, 64 }, { 20, 128 },
	};
	u32 value = 0;
	size_t i;

	/*
	 * The grid tiles 0..1 << Q exactly, and index 64 relies on falling out
	 * of the loop below with nothing left. A miscount here would be silent
	 * -- the recipe carries zero for these words -- so make it loud.
	 */
	static_assert(8 + 12 + 8 + 16 + 20 == BECORE_RGBP_GAMMA_SEGMENTS);
	static_assert(8 * 8 + 12 * 16 + 8 * 32 + 16 * 64 + 20 * 128 ==
		      1 << BECORE_RGBP_GAMMA_Q);

	for (i = 0; i < ARRAY_SIZE(segments); i++) {
		if (index < segments[i].count) {
			*x = value + index * segments[i].step;
			return 0;
		}
		value += segments[i].count * segments[i].step;
		index -= segments[i].count;
	}
	if (index)
		return -EINVAL;
	*x = value;		/* the 65th knot closes the grid at 1 << Q */

	return 0;
}

/* round(sqrt(x / 4096) * 4096), which is exactly round(sqrt(x << 12)). */
static u32 becore_rgbp_gamma_encode(u32 x)
{
	u32 n = x << BECORE_RGBP_GAMMA_Q;
	u32 root = int_sqrt(n);

	return n - root * root > root ? root + 1 : root;
}

static int becore_rgbp_gamma_point(u32 index, bool encode, u32 *value)
{
	u32 x;
	int ret;

	if (index >= BECORE_RGBP_GAMMA_KNOTS)
		return -EINVAL;
	ret = becore_rgbp_gamma_knot(index, &x);
	if (ret)
		return ret;
	*value = encode ? becore_rgbp_gamma_encode(x) : x;

	return 0;
}

/* Two points per register, the lower-numbered one in the low half. */
static int becore_rgbp_gamma_pair(u32 index, bool encode, u32 *value)
{
	u32 low;
	u32 high;
	int ret;

	ret = becore_rgbp_gamma_point(index, encode, &low);
	if (ret)
		return ret;
	ret = becore_rgbp_gamma_point(index + 1, encode, &high);
	if (ret)
		return ret;
	*value = (high << 16) | low;

	return 0;
}

/*
 * The 65th point, as its distance from the 64th: it does not fit the field.
 *
 * The vendor stores a magnitude and puts the direction in a separate
 * _DELTA_SIGN register. Neither of these curves falls, so that register stays
 * unwritten -- but take the magnitude rather than the difference anyway, so a
 * curve that did fall would encode a small number here instead of wrapping.
 */
static int becore_rgbp_gamma_delta(bool encode, u32 *value)
{
	u32 last;
	u32 prev;
	int ret;

	ret = becore_rgbp_gamma_point(BECORE_RGBP_GAMMA_KNOTS - 1, encode, &last);
	if (ret)
		return ret;
	ret = becore_rgbp_gamma_point(BECORE_RGBP_GAMMA_KNOTS - 2, encode, &prev);
	if (ret)
		return ret;
	*value = last > prev ? last - prev : prev - last;

	return 0;
}

static int becore_rgbp_gamma_value(u32 offset, u32 *value)
{
	if (offset & 3)
		return -EINVAL;

	switch (offset) {
	case 0x000:		/* BYPASS: the block runs */
	case 0x004:		/* PEDESTAL_EN */
		*value = 0;
		return 0;
	case 0x08c:		/* R_GAMMA_TBL last-knot delta */
		return becore_rgbp_gamma_delta(true, value);
	case 0x250:		/* X_PNTS_TBL last-knot delta */
		return becore_rgbp_gamma_delta(false, value);
	case 0x254:		/* X_PNTS_LSHIFT */
		*value = BECORE_RGBP_GAMMA_X_LSHIFT;
		return 0;
	}

	if (offset >= 0x00c && offset < 0x08c)
		return becore_rgbp_gamma_pair((offset - 0x00c) / 4 * 2, true,
					      value);
	if (offset >= 0x1c0 && offset <= 0x1ec)
		return becore_rgbp_gamma_pair((offset - 0x1c0) / 4 * 2, false,
					      value);
	/*
	 * The four-register hole between points 23 and 24 is 0x1f0..0x1fc. No
	 * range covers it and the capture writes nothing there, so it is never
	 * asked for; if it were, the -EINVAL below would reject it.
	 */
	if (offset >= 0x200 && offset < 0x250)
		return becore_rgbp_gamma_pair((offset - 0x200) / 4 * 2 + 24,
					      false, value);

	return -EINVAL;
}

/* The identity itself: out[i] == in[i] << 5 at every knot. */
static int becore_rgbp_gtm_value(u32 offset, u32 *value)
{
	u32 knot;
	u32 low;
	int ret;

	if (offset & 3)
		return -EINVAL;

	switch (offset) {
	case BECORE_RGBP_GTM_BYPASS:		/* the block runs */
	case BECORE_RGBP_GTM_GAIN_MODE_EN:
	case BECORE_RGBP_GTM_V_BLEND_RATIO:
	case BECORE_RGBP_GTM_INPUT_RSHIFT:
		*value = 0;
		return 0;
	case BECORE_RGBP_GTM_Y_WEIGHT_0:
		*value = (BECORE_RGBP_GTM_Y_WEIGHT_G << 16) |
			 BECORE_RGBP_GTM_Y_WEIGHT_R;
		return 0;
	case BECORE_RGBP_GTM_Y_WEIGHT_1:
		*value = BECORE_RGBP_GTM_Y_WEIGHT_B;
		return 0;
	}

	if (offset >= BECORE_RGBP_GTM_IN_POINTS &&
	    offset < BECORE_RGBP_GTM_OUT_POINTS) {
		u32 index = (offset - BECORE_RGBP_GTM_IN_POINTS) / 4 * 2;

		ret = becore_rgbp_gtm_knot(index, &low);
		if (ret)
			return ret;
		ret = becore_rgbp_gtm_knot(index + 1, &knot);
		if (ret)
			return ret;
		*value = (knot << 16) | low;
		return 0;
	}

	if (offset >= BECORE_RGBP_GTM_OUT_POINTS &&
	    offset < BECORE_RGBP_GTM_Y_WEIGHT_0) {
		ret = becore_rgbp_gtm_knot((offset -
					    BECORE_RGBP_GTM_OUT_POINTS) / 4,
					   &knot);
		if (ret)
			return ret;
		*value = knot << BECORE_RGBP_GTM_OUT_SHIFT;
		return 0;
	}

	return -EINVAL;
}

/*
 * Below DJAG the chain neither crops nor scales, so its stages map the output
 * raster onto itself. Going through the ratio helper rather than writing a
 * literal unity keeps this in the form every other ratio is written in, so a
 * stage that starts scaling shows up as a changed value here -- and takes its
 * filter coefficients with it.
 */
static u32 becore_mcsc_chain_ratio(void)
{
	return becore_zoom_ratio(becore_mcsc_output.width,
				 becore_mcsc_output.width);
}

/*
 * Samsung publishes its poly-phase coefficients because they are a function of
 * the scaling ratio rather than of the scene: get_scaler_coef_ver2() picks one
 * of seven sets by comparing the ratio against x8/8, x7/8 and so on down to
 * x2/8. Both scalers here run at unity, which selects x8/8, and x8/8 is the
 * only set carried -- any other ratio makes the encode fail rather than
 * quietly programming the wrong filter for it.
 *
 * These are Samsung's numbers verbatim, indexed [tap][phase], from
 * is-hw-api-rgbp-v1_20.c. Zuma's fields hold a quarter of that precision, so
 * a phase sums to 512 rather than 2048 and the last tap takes up the
 * remainder. That is not cosmetic: Samsung's own x8/8 horizontal phase 7 sums
 * to 2052, and the captured program carries the renormalised value.
 */
static const s16 becore_sc_h_coeff_x8_8[BECORE_SC_H_TAPS][BECORE_SC_PHASES] = {
	{    0,   -8,  -16,  -20,  -24,  -24,  -24,  -24,  -20 },
	{    0,   32,   56,   80,   92,  100,  104,  100,   92 },
	{    0, -100, -184, -248, -292, -320, -332, -328, -312 },
	{ 2048, 2036, 1996, 1928, 1832, 1716, 1580, 1428, 1264 },
	{    0,  120,  256,  404,  568,  740,  912, 1092, 1264 },
	{    0,  -36,  -76, -120, -164, -212, -252, -284, -312 },
	{    0,    8,   20,   32,   48,   60,   76,   84,   92 },
	{    0,   -4,   -4,   -8,  -12,  -12,  -16,  -16,  -20 },
};

static const s16 becore_sc_v_coeff_x8_8[BECORE_SC_V_TAPS][BECORE_SC_PHASES] = {
	{    0,  -60, -100, -124, -132, -132, -124, -108,  -92 },
	{ 2048, 2032, 1980, 1892, 1772, 1632, 1468, 1296, 1116 },
	{    0,   80,  180,  300,  440,  592,  760,  936, 1116 },
	{    0,   -4,  -12,  -20,  -32,  -44,  -56,  -76,  -92 },
};

static s32 becore_sc_coeff(const s16 (*table)[BECORE_SC_PHASES], u32 taps,
			   u32 tap, u32 phase)
{
	s32 sum = 0;
	u32 i;

	if (tap + 1 < taps)
		return table[tap][phase] / 4;
	for (i = 0; i + 1 < taps; i++)
		sum += table[i][phase] / 4;

	return BECORE_SC_COEFF_SUM - sum;
}

/* Two taps of one phase, the lower-numbered one in the low half. */
static int becore_sc_coeff_value(const s16 (*table)[BECORE_SC_PHASES],
				 u32 taps, u32 ratio, u32 index, u32 *value)
{
	u32 pairs = taps / 2;
	u32 phase = index / pairs;
	u32 pair = index % pairs;

	if (ratio != BECORE_SC_RATIO_X8_8 || phase >= BECORE_SC_PHASES)
		return -EINVAL;
	*value = (((u32)becore_sc_coeff(table, taps, pair * 2 + 1, phase) &
		   BECORE_SC_COEFF_MASK) << 16) |
		 ((u32)becore_sc_coeff(table, taps, pair * 2, phase) &
		  BECORE_SC_COEFF_MASK);

	return 0;
}

/*
 * The ratio the scaler these coefficients belong to is programmed with. Taken
 * from the same place the driver takes the value it writes into the ratio
 * register, so the filter and the ratio cannot describe different scalings.
 */
static int becore_sc_ratio(enum becore_block_id id, bool vertical, u32 *ratio)
{
	u32 index;

	if (id == BECORE_RGBP) {
		index = vertical ? BECORE_RGBP_SC_V_RATIO :
				   BECORE_RGBP_SC_H_RATIO;
		return becore_rgbp_input_value(index,
					       becore_rgbp_input_regs[index],
					       ratio);
	}
	if (id != BECORE_MCSC)
		return -EINVAL;
	*ratio = becore_mcsc_chain_ratio();

	return 0;
}

static u32 becore_generated_word_count(enum becore_block_id id)
{
	if (id == BECORE_RGBP)
		return BECORE_RGBP_GENERATED_WORDS;
	if (id == BECORE_YUVP)
		return BECORE_YUVP_GENERATED_WORDS;
	if (id == BECORE_MCSC)
		return BECORE_MCSC_GENERATED_WORDS;

	return 0;
}

/*
 * The by-register class is only unambiguous while the ranges are. A first
 * match wins below, so an overlap would silently hand a register the wrong
 * intent -- and these ranges are wide enough that an overlap is an easy edit
 * to make. Check the tables once, at probe, rather than trusting the reader.
 */
static int becore_generated_tables_validate(struct device *dev)
{
	static const struct becore_generated_range *tables[] = {
		becore_rgbp_generated,
		becore_yuvp_generated,
		becore_mcsc_generated,
	};
	static const size_t counts[] = {
		ARRAY_SIZE(becore_rgbp_generated),
		ARRAY_SIZE(becore_yuvp_generated),
		ARRAY_SIZE(becore_mcsc_generated),
	};
	size_t block;
	size_t i;
	size_t j;

	for (block = 0; block < ARRAY_SIZE(tables); block++) {
		for (i = 0; i < counts[block]; i++) {
			const struct becore_generated_range *a =
				&tables[block][i];

			if (a->first > a->last)
				return dev_err_probe(dev, -EINVAL,
						     "generated range %zu:%zu is inverted\n",
						     block, i);
			for (j = 0; j < i; j++) {
				const struct becore_generated_range *b =
					&tables[block][j];

				if (a->first <= b->last && b->first <= a->last)
					return dev_err_probe(dev, -EINVAL,
							     "generated ranges %zu:%zu and %zu:%zu overlap\n",
							     block, j, block, i);
			}
		}
	}

	return 0;
}

static int becore_generated_value(enum becore_block_id id, u32 reg, u32 *value)
{
	const struct becore_generated_range *table;
	size_t count;
	size_t i;

	if (id == BECORE_RGBP) {
		table = becore_rgbp_generated;
		count = ARRAY_SIZE(becore_rgbp_generated);
	} else if (id == BECORE_YUVP) {
		table = becore_yuvp_generated;
		count = ARRAY_SIZE(becore_yuvp_generated);
	} else if (id == BECORE_MCSC) {
		table = becore_mcsc_generated;
		count = ARRAY_SIZE(becore_mcsc_generated);
	} else {
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		u32 result;

		if (reg < table[i].first || reg > table[i].last)
			continue;
		switch (table[i].kind) {
		case BECORE_GEN_OFF:
			result = 0;
			break;
		case BECORE_GEN_BYPASS:
			result = 1;
			break;
		case BECORE_GEN_RUNNING:
			result = 0;
			break;
		case BECORE_GEN_DECOMP_SIZE:
			result = becore_pack_size(becore_rgbp_input.height,
						  becore_rgbp_input.width);
			break;
		case BECORE_GEN_NOISE_SLOPE:
		case BECORE_GEN_NOISE_SHIFT:
		case BECORE_GEN_NOISE_DOMAIN:
			if (becore_noise_value(id, reg, table[i].kind, &result))
				return -EINVAL;
			break;
		case BECORE_GEN_DJAG:
			if (becore_mcsc_djag_value(reg - BECORE_MCSC_DJAG_BASE,
						   &result))
				return -EINVAL;
			break;
		case BECORE_GEN_DMSC:
			if (becore_rgbp_dmsc_value(reg - BECORE_RGBP_DMSC_BASE,
						   &result))
				return -EINVAL;
			break;
		case BECORE_GEN_CSC:
			if (becore_rgbp_csc_value(reg - (id == BECORE_RGBP ?
							 BECORE_RGBP_CSC_BASE :
							 BECORE_YUVP_CSC_BASE),
						  &result))
				return -EINVAL;
			break;
		case BECORE_GEN_SCALER_PHASE:
			if (becore_scaler_phase_value(reg, &result))
				return -EINVAL;
			break;
		case BECORE_GEN_MCSC_INPUT_SIZE:
			if (reg == BECORE_MCSC_IN_WIDTH_REG)
				result = becore_rgbp_out_width();
			else if (reg == BECORE_MCSC_IN_HEIGHT_REG)
				result = becore_rgbp_out_height();
			else
				return -EINVAL;
			break;
		case BECORE_GEN_CHROMA_LPF:
			if (becore_rgbp_chroma_lpf_value(reg -
						BECORE_RGBP_CHROMA_LPF_BASE,
						&result))
				return -EINVAL;
			break;
		case BECORE_GEN_DNS_GEOMETRY:
			if (becore_rgbp_dns_geometry_value(reg -
						BECORE_RGBP_DNS_BASE, &result))
				return -EINVAL;
			break;
		case BECORE_GEN_LPF_NORM:
			if (becore_yuvp_lpf_norm_value(&result))
				return -EINVAL;
			break;
		case BECORE_GEN_GAMMA:
			if (becore_rgbp_gamma_value(reg -
						    BECORE_RGBP_GAMMA_BASE,
						    &result))
				return -EINVAL;
			break;
		case BECORE_GEN_GTM:
			if (becore_rgbp_gtm_value(reg - BECORE_RGBP_GTM_BASE,
						  &result))
				return -EINVAL;
			break;
		case BECORE_GEN_LTM:
			if (becore_yuvp_ltm_value(reg - BECORE_YUVP_LTM_BASE,
						  &result))
				return -EINVAL;
			break;
		case BECORE_GEN_SC_V_COEFF:
		case BECORE_GEN_SC_H_COEFF: {
			bool vertical = table[i].kind == BECORE_GEN_SC_V_COEFF;
			u32 ratio;

			if (becore_sc_ratio(id, vertical, &ratio))
				return -EINVAL;
			if (becore_sc_coeff_value(vertical ?
						  becore_sc_v_coeff_x8_8 :
						  becore_sc_h_coeff_x8_8,
						  vertical ? BECORE_SC_V_TAPS :
							     BECORE_SC_H_TAPS,
						  ratio,
						  (reg - table[i].first) / 4,
						  &result))
				return -EINVAL;
			break;
		}
		case BECORE_GEN_CHAIN_ORIGIN:
			result = becore_pack_size(0, 0);
			break;
		case BECORE_GEN_CHAIN_SIZE:
			result = becore_pack_size(becore_mcsc_output.width,
						  becore_mcsc_output.height);
			break;
		case BECORE_GEN_CHAIN_RATIO:
			result = becore_mcsc_chain_ratio();
			break;
		default:
			return -EINVAL;
		}
		if (value)
			*value = result;

		return 0;
	}

	return -EINVAL;
}

static int becore_recipe_header_validate(const struct becore_device *becore)
{
	const u8 *header = becore->recipe;

	if (becore->recipe_staged_bytes != BECORE_RECIPE_BYTES)
		return -EINVAL;
	if (get_unaligned_le32(header) != BECORE_RECIPE_MAGIC ||
	    get_unaligned_le32(header + 4) != BECORE_RECIPE_VERSION ||
	    get_unaligned_le32(header + 8) != BECORE_RECIPE_HEADER_BYTES ||
	    get_unaligned_le32(header + 12) != BECORE_RECIPE_RECORD_BYTES ||
	    get_unaligned_le32(header + 16) != BECORE_RGBP_HEADER_COUNT ||
	    get_unaligned_le32(header + 20) != BECORE_YUVP_HEADER_COUNT ||
	    get_unaligned_le32(header + 24) != BECORE_RECIPE_BYTES ||
	    get_unaligned_le32(header + 28))
		return -EINVAL;

	return 0;
}

static dma_addr_t becore_address_dma(struct becore_device *becore, u32 reg)
{
	struct becore_dma_buffer *input = &becore->run_input->buffer;

	switch (reg) {
	case BECORE_RGBP_INPUT_IMAGE_REG:
		return input->dma + becore_rgbp_input_image_offset();
	case BECORE_RGBP_INPUT_HEADER_REG:
		return input->dma;
	case BECORE_YUVP_GRID_REG:
		return becore->grid.dma;
	case BECORE_YUVP_OUTPUT_PLANE1_REG:
		return becore->active_output_dma;
	case BECORE_YUVP_OUTPUT_PLANE2_REG:
		return becore->active_output_dma +
		       becore_active_output_plane2_offset(becore);
	default:
		return DMA_MAPPING_ERROR;
	}
}

static bool becore_address_reg_valid(u32 reg)
{
	switch (reg) {
	case BECORE_RGBP_INPUT_IMAGE_REG:
	case BECORE_RGBP_INPUT_HEADER_REG:
	case BECORE_YUVP_GRID_REG:
	case BECORE_YUVP_OUTPUT_PLANE1_REG:
	case BECORE_YUVP_OUTPUT_PLANE2_REG:
		return true;
	default:
		return false;
	}
}

static int becore_recipe_block_validate(struct becore_device *becore,
					enum becore_block_id id,
					const struct becore_cmdq_shape *shape,
					u32 header_count, bool validate_dma)
{
	const u8 *record = becore_recipe_records(becore, id);
	u32 address_count = 0;
	u32 typed_count = 0;
	u32 generated_count = 0;
	u32 i;

	for (i = 0; i < header_count; i++, record += BECORE_RECIPE_RECORD_BYTES) {
		const u8 *words = record + 12;
		u16 used_mask;
		u16 value_mask;
		u32 word;

		if (!shape[i].valid_words || shape[i].valid_words > 16)
			return -EINVAL;
		used_mask = shape[i].valid_words == 16 ? U16_MAX :
			    GENMASK(shape[i].valid_words - 1, 0);
		value_mask = shape[i].mode == 0x00090000 ?
			     used_mask & 0xaaaa : used_mask;
		if (get_unaligned_le32(record) != shape[i].mode ||
		    get_unaligned_le32(record + 4) != shape[i].target ||
		    get_unaligned_le32(record + 8) != shape[i].type_map ||
		    (shape[i].address_mask | shape[i].typed_mask |
		     shape[i].generated_mask | shape[i].fixed_mask) !=
		     value_mask ||
		    (shape[i].address_mask & ~used_mask) ||
		    (shape[i].typed_mask & ~used_mask) ||
		    (shape[i].generated_mask & ~used_mask) ||
		    (shape[i].fixed_mask & ~used_mask) ||
		    (shape[i].address_mask & shape[i].typed_mask) ||
		    (shape[i].address_mask & shape[i].generated_mask) ||
		    (shape[i].address_mask & shape[i].fixed_mask) ||
		    (shape[i].typed_mask & shape[i].generated_mask) ||
		    (shape[i].typed_mask & shape[i].fixed_mask) ||
		    (shape[i].generated_mask & shape[i].fixed_mask))
			return -EINVAL;

		for (word = 0; word < 16; word++) {
			u32 value = get_unaligned_le32(words + word * 4);

			if (word >= shape[i].valid_words) {
				if (value)
					return -EINVAL;
				continue;
			}

			if (shape[i].mode == 0x00090000 && !(word & 1)) {
				if (value != shape[i].pair_registers[word / 2])
					return -EINVAL;
				continue;
			}

			if (shape[i].fixed_mask & BIT(word)) {
				if (value != shape[i].fixed_values[word])
					return -EINVAL;
			}

			if (shape[i].typed_mask & BIT(word)) {
				u32 reg;

				if (shape[i].mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape[i].pair_registers[word / 2];
				if (becore_typed_value(becore, id, typed_count, reg,
						       NULL))
					return -EINVAL;
				typed_count++;
			}

			if (shape[i].generated_mask & BIT(word)) {
				u32 reg;

				if (becore_shape_register(&shape[i], word, &reg) ||
				    becore_generated_value(id, reg, NULL))
					return -EINVAL;
				generated_count++;
			}

			if (shape[i].address_mask & BIT(word)) {
				u32 reg;

				/* All five supported DMA fields are pair-mode values. */
				if (shape[i].mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape[i].pair_registers[word / 2];
				if (!becore_address_reg_valid(reg) ||
				    (validate_dma &&
				     becore_address_dma(becore, reg) ==
				     DMA_MAPPING_ERROR))
					return -EINVAL;
				address_count++;
			}
		}
	}

	if ((id == BECORE_RGBP && address_count != 2) ||
	    (id == BECORE_YUVP && address_count != 3))
		return -EINVAL;
	if (typed_count != becore_typed_word_count(id) ||
	    generated_count != becore_generated_word_count(id))
		return -EINVAL;

	return 0;
}

static int becore_recipe_records_validate(struct becore_device *becore,
					  bool validate_dma)
{
	int ret;

	ret = becore_recipe_header_validate(becore);
	if (ret)
		return ret;

	ret = becore_recipe_block_validate(becore, BECORE_RGBP,
					   becore_rgbp_shape,
					   BECORE_RGBP_HEADER_COUNT,
					   validate_dma);
	if (ret)
		return ret;

	return becore_recipe_block_validate(becore, BECORE_YUVP,
					    becore_yuvp_shape,
					    BECORE_YUVP_HEADER_COUNT,
					    validate_dma);
}

static int becore_recipe_validate(struct becore_device *becore)
{
	struct becore_dma_buffer *input = &becore->run_input->buffer;

	if (input->size != becore_rgbp_input_size() ||
	    input->staged_bytes != input->size ||
	    becore->grid.staged_bytes != BECORE_GRID_SIZE)
		return -EINVAL;

	return becore_recipe_records_validate(becore, true);
}

static int becore_encode_block(struct becore_device *becore,
			       enum becore_block_id id,
			       const struct becore_cmdq_shape *shape,
			       u32 header_count)
{
	struct becore_cmdq_program *program = &becore->program[id];
	const u8 *record = becore_recipe_records(becore, id);
	size_t payload_offset = ALIGN((size_t)header_count *
				      BECORE_CMDQ_HEADER_BYTES,
				      BECORE_CMDQ_PAYLOAD_BYTES);
	u32 i;
	u32 typed_count = 0;
	u32 generated_count = 0;

	if (!program->cpu || program->header_count != header_count ||
	    program->size != becore_cmdq_program_size(header_count) ||
	    upper_32_bits(program->dma) ||
	    upper_32_bits(program->dma + program->size - 1))
		return -EINVAL;

	memset(program->cpu, 0, program->size);
	for (i = 0; i < header_count;
	     i++, record += BECORE_RECIPE_RECORD_BYTES) {
		u8 *header = (u8 *)program->cpu + i * BECORE_CMDQ_HEADER_BYTES;
		u8 *payload = (u8 *)program->cpu + payload_offset +
			      i * BECORE_CMDQ_PAYLOAD_BYTES;
		dma_addr_t payload_dma = program->dma + payload_offset +
					 i * BECORE_CMDQ_PAYLOAD_BYTES;
		u32 word;

		put_unaligned_le32(shape[i].mode, header);
		put_unaligned_le32(lower_32_bits(payload_dma), header + 4);
		put_unaligned_le32(shape[i].target, header + 8);
		put_unaligned_le32(shape[i].type_map, header + 12);
		memcpy(payload, record + 12, BECORE_CMDQ_PAYLOAD_BYTES);

		for (word = 0; word < shape[i].valid_words; word++) {
			dma_addr_t dma;
			u32 reg;
			u32 value;

			if (shape[i].typed_mask & BIT(word)) {
				reg = shape[i].pair_registers[word / 2];
				if (becore_typed_value(becore, id, typed_count, reg,
						       &value))
					return -EINVAL;
				put_unaligned_le32(value, payload + word * 4);
				typed_count++;
				continue;
			}

			if (shape[i].generated_mask & BIT(word)) {
				if (becore_shape_register(&shape[i], word, &reg) ||
				    becore_generated_value(id, reg, &value))
					return -EINVAL;
				put_unaligned_le32(value, payload + word * 4);
				generated_count++;
				continue;
			}

			if (!(shape[i].address_mask & BIT(word)))
				continue;
			reg = shape[i].pair_registers[word / 2];
			dma = becore_address_dma(becore, reg);
			if (dma == DMA_MAPPING_ERROR || upper_32_bits(dma))
				return -EINVAL;
			put_unaligned_le32(lower_32_bits(dma), payload + word * 4);
		}
	}
	if (typed_count != becore_typed_word_count(id) ||
	    generated_count != becore_generated_word_count(id))
		return -EINVAL;

	return 0;
}

static int becore_encode_programs(struct becore_device *becore)
{
	int ret;

	ret = becore_encode_block(becore, BECORE_RGBP, becore_rgbp_shape,
				  BECORE_RGBP_HEADER_COUNT);
	if (ret)
		return ret;

	return becore_encode_block(becore, BECORE_YUVP, becore_yuvp_shape,
				   BECORE_YUVP_HEADER_COUNT);
}

static int becore_gtnr_recipe_validate(struct becore_device *becore)
{
	const u8 *header = becore->gtnr_recipe;
	const u8 *record = header + BECORE_GTNR_RECIPE_HEADER_BYTES;
	const struct becore_yuvp_output_profile *input =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];
	u32 address_count = 0;
	u32 typed_count = 0;
	u32 i;

	if (becore->gtnr_recipe_staged_bytes != BECORE_GTNR_RECIPE_BYTES ||
	    get_unaligned_le32(header) != BECORE_GTNR_RECIPE_MAGIC ||
	    get_unaligned_le32(header + 4) != BECORE_GTNR_RECIPE_VERSION ||
	    get_unaligned_le32(header + 8) != BECORE_GTNR_RECIPE_HEADER_BYTES ||
	    get_unaligned_le32(header + 12) != BECORE_GTNR_RECIPE_RECORD_BYTES ||
	    get_unaligned_le32(header + 16) != BECORE_GTNR_HEADER_COUNT ||
	    get_unaligned_le32(header + 20) != BECORE_GTNR_RECIPE_BYTES ||
	    get_unaligned_le32(header + 24) || get_unaligned_le32(header + 28))
		return -EINVAL;
	if (becore->output.size < becore_gtnr_surface_size() ||
	    becore->gtnr_output.size != becore_gtnr_surface_size() ||
	    becore_gtnr_input.width != input->width ||
	    becore_gtnr_input.height != input->height ||
	    becore_gtnr_input.stride != becore_yuvp_output_stride(input) ||
	    becore_gtnr_output.width != input->width ||
	    becore_gtnr_output.height != input->height ||
	    becore_gtnr_output.stride != becore_yuvp_output_stride(input))
		return -EINVAL;

	for (i = 0; i < BECORE_GTNR_HEADER_COUNT;
	     i++, record += BECORE_GTNR_RECIPE_RECORD_BYTES) {
		const struct becore_cmdq_shape *shape = &becore_gtnr_shape[i];
		const u8 *words = record + 12;
		u16 used_mask;
		u32 word;

		if (!shape->valid_words || shape->valid_words > 16)
			return -EINVAL;
		used_mask = shape->valid_words == 16 ? U16_MAX :
			    GENMASK(shape->valid_words - 1, 0);
		if (get_unaligned_le32(record) != shape->mode ||
		    get_unaligned_le32(record + 4) != shape->target ||
		    get_unaligned_le32(record + 8) != shape->type_map ||
		    shape->generated_mask ||
		    (shape->address_mask & ~used_mask) ||
		    (shape->typed_mask & ~used_mask) ||
		    (shape->fixed_mask & ~used_mask) ||
		    (shape->address_mask & shape->typed_mask) ||
		    (shape->address_mask & shape->fixed_mask) ||
		    (shape->typed_mask & shape->fixed_mask))
			return -EINVAL;

		for (word = 0; word < 16; word++) {
			u32 value = get_unaligned_le32(words + word * 4);

			if (word >= shape->valid_words) {
				if (value)
					return -EINVAL;
				continue;
			}
			if (shape->mode == 0x00090000 && !(word & 1)) {
				if (value != shape->pair_registers[word / 2])
					return -EINVAL;
				continue;
			}
			if ((shape->fixed_mask & BIT(word)) &&
			    value != shape->fixed_values[word])
				return -EINVAL;

			if (shape->typed_mask & BIT(word)) {
				u32 reg;

				if (shape->mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape->pair_registers[word / 2];
				if (becore_gtnr_dma_value(typed_count, reg, NULL))
					return -EINVAL;
				typed_count++;
			}
			if (shape->address_mask & BIT(word)) {
				u32 reg;

				if (shape->mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape->pair_registers[word / 2];
				if (becore_gtnr_address_dma(becore, reg) ==
				    DMA_MAPPING_ERROR)
					return -EINVAL;
				address_count++;
			}
		}
	}

	if (address_count != 4 || typed_count != BECORE_GTNR_DMA_WORD_COUNT)
		return -EINVAL;

	return 0;
}

static int becore_encode_gtnr(struct becore_device *becore)
{
	struct becore_cmdq_program *program = &becore->gtnr_program;
	const u8 *record = becore->gtnr_recipe +
			   BECORE_GTNR_RECIPE_HEADER_BYTES;
	size_t payload_offset = ALIGN((size_t)BECORE_GTNR_HEADER_COUNT *
				      BECORE_CMDQ_HEADER_BYTES,
				      BECORE_CMDQ_PAYLOAD_BYTES);
	u32 typed_count = 0;
	u32 i;

	if (!program->cpu || program->header_count != BECORE_GTNR_HEADER_COUNT ||
	    program->size != becore_cmdq_program_size(BECORE_GTNR_HEADER_COUNT) ||
	    upper_32_bits(program->dma) ||
	    upper_32_bits(program->dma + program->size - 1))
		return -EINVAL;

	memset(program->cpu, 0, program->size);
	for (i = 0; i < BECORE_GTNR_HEADER_COUNT;
	     i++, record += BECORE_GTNR_RECIPE_RECORD_BYTES) {
		const struct becore_cmdq_shape *shape = &becore_gtnr_shape[i];
		u8 *header = (u8 *)program->cpu + i * BECORE_CMDQ_HEADER_BYTES;
		u8 *payload = (u8 *)program->cpu + payload_offset +
			      i * BECORE_CMDQ_PAYLOAD_BYTES;
		dma_addr_t payload_dma = program->dma + payload_offset +
					 i * BECORE_CMDQ_PAYLOAD_BYTES;
		u32 word;

		put_unaligned_le32(shape->mode, header);
		put_unaligned_le32(lower_32_bits(payload_dma), header + 4);
		put_unaligned_le32(shape->target, header + 8);
		put_unaligned_le32(shape->type_map, header + 12);
		memcpy(payload, record + 12, BECORE_CMDQ_PAYLOAD_BYTES);

		for (word = 0; word < shape->valid_words; word++) {
			dma_addr_t dma;
			u32 reg;
			u32 value;

			if (shape->typed_mask & BIT(word)) {
				reg = shape->pair_registers[word / 2];
				if (becore_gtnr_dma_value(typed_count, reg, &value))
					return -EINVAL;
				put_unaligned_le32(value, payload + word * 4);
				typed_count++;
				continue;
			}
			if (!(shape->address_mask & BIT(word)))
				continue;
			reg = shape->pair_registers[word / 2];
			dma = becore_gtnr_address_dma(becore, reg);
			if (dma == DMA_MAPPING_ERROR || upper_32_bits(dma))
				return -EINVAL;
			put_unaligned_le32(lower_32_bits(dma), payload + word * 4);
		}
	}

	if (typed_count != BECORE_GTNR_DMA_WORD_COUNT)
		return -EINVAL;

	return 0;
}

static int becore_recipe_records_generate(u8 *record,
					  const struct becore_cmdq_shape *shapes,
					  u32 header_count)
{
	u32 i;

	static_assert(BECORE_RECIPE_RECORD_BYTES ==
		      BECORE_MCSC_RECIPE_RECORD_BYTES);
	for (i = 0; i < header_count;
	     i++, record += BECORE_RECIPE_RECORD_BYTES) {
		const struct becore_cmdq_shape *shape = &shapes[i];
		u16 used_mask;
		u16 value_mask;
		u32 word;

		if (!shape->valid_words || shape->valid_words > 16)
			return -EINVAL;
		used_mask = shape->valid_words == 16 ? U16_MAX :
			    GENMASK(shape->valid_words - 1, 0);
		if (shape->mode == 0x00090000) {
			if (shape->valid_words & 1)
				return -EINVAL;
			value_mask = used_mask & 0xaaaa;
		} else if (shape->mode == 0x00080000) {
			if (shape->address_mask || shape->typed_mask)
				return -EINVAL;
			value_mask = used_mask;
		} else if (shape->mode == 0x000b0000) {
			/*
			 * Every word of a repeated-target header names the
			 * same register, so a by-register class cannot say
			 * which value belongs where.
			 */
			if (shape->address_mask || shape->typed_mask ||
			    shape->generated_mask)
				return -EINVAL;
			value_mask = used_mask;
		} else {
			return -EINVAL;
		}
		if ((shape->address_mask | shape->typed_mask |
		     shape->generated_mask | shape->fixed_mask) != value_mask ||
		    (shape->address_mask & shape->typed_mask) ||
		    (shape->address_mask & shape->generated_mask) ||
		    (shape->address_mask & shape->fixed_mask) ||
		    (shape->typed_mask & shape->generated_mask) ||
		    (shape->typed_mask & shape->fixed_mask) ||
		    (shape->generated_mask & shape->fixed_mask))
			return -EINVAL;

		put_unaligned_le32(shape->mode, record);
		put_unaligned_le32(shape->target, record + 4);
		put_unaligned_le32(shape->type_map, record + 8);
		for (word = 0; word < shape->valid_words; word++) {
			u32 value = 0;

			if (shape->mode == 0x00090000 && !(word & 1))
				value = shape->pair_registers[word / 2];
			else if (shape->fixed_mask & BIT(word))
				value = shape->fixed_values[word];
			put_unaligned_le32(value, record + 12 + word * 4);
		}
	}

	return 0;
}

static int becore_recipe_generate(struct becore_device *becore)
{
	u8 *header = becore->recipe;
	u8 *record = header + BECORE_RECIPE_HEADER_BYTES;
	int ret;

	memset(header, 0, BECORE_RECIPE_BYTES);
	put_unaligned_le32(BECORE_RECIPE_MAGIC, header);
	put_unaligned_le32(BECORE_RECIPE_VERSION, header + 4);
	put_unaligned_le32(BECORE_RECIPE_HEADER_BYTES, header + 8);
	put_unaligned_le32(BECORE_RECIPE_RECORD_BYTES, header + 12);
	put_unaligned_le32(BECORE_RGBP_HEADER_COUNT, header + 16);
	put_unaligned_le32(BECORE_YUVP_HEADER_COUNT, header + 20);
	put_unaligned_le32(BECORE_RECIPE_BYTES, header + 24);

	ret = becore_recipe_records_generate(record, becore_rgbp_shape,
					     BECORE_RGBP_HEADER_COUNT);
	if (ret)
		return ret;
	record += BECORE_RGBP_HEADER_COUNT * BECORE_RECIPE_RECORD_BYTES;
	ret = becore_recipe_records_generate(record, becore_yuvp_shape,
					     BECORE_YUVP_HEADER_COUNT);
	if (ret)
		return ret;

	becore->recipe_staged_bytes = BECORE_RECIPE_BYTES;
	becore->recipe_generation = 1;

	return 0;
}

static int becore_mcsc_recipe_generate(struct becore_device *becore)
{
	u8 *header = becore->mcsc_recipe;
	int ret;

	memset(header, 0, BECORE_MCSC_RECIPE_BYTES);
	put_unaligned_le32(BECORE_MCSC_RECIPE_MAGIC, header);
	put_unaligned_le32(BECORE_MCSC_RECIPE_VERSION, header + 4);
	put_unaligned_le32(BECORE_MCSC_RECIPE_HEADER_BYTES, header + 8);
	put_unaligned_le32(BECORE_MCSC_RECIPE_RECORD_BYTES, header + 12);
	put_unaligned_le32(BECORE_MCSC_HEADER_COUNT, header + 16);
	put_unaligned_le32(BECORE_MCSC_RECIPE_BYTES, header + 20);

	ret = becore_recipe_records_generate(header +
			BECORE_MCSC_RECIPE_HEADER_BYTES,
			becore_mcsc_shape, BECORE_MCSC_HEADER_COUNT);
	if (ret)
		return ret;

	becore->mcsc_recipe_staged_bytes = BECORE_MCSC_RECIPE_BYTES;
	becore->mcsc_recipe_generation = 1;

	return 0;
}

static int becore_mcsc_recipe_validate(struct becore_device *becore)
{
	const u8 *header = becore->mcsc_recipe;
	const u8 *record = header + BECORE_MCSC_RECIPE_HEADER_BYTES;
	const struct becore_yuvp_output_profile *input =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];
	u32 address_count = 0;
	u32 typed_count = 0;
	u32 generated_count = 0;
	u32 i;

	if (becore->mcsc_recipe_staged_bytes != BECORE_MCSC_RECIPE_BYTES ||
	    get_unaligned_le32(header) != BECORE_MCSC_RECIPE_MAGIC ||
	    get_unaligned_le32(header + 4) != BECORE_MCSC_RECIPE_VERSION ||
	    get_unaligned_le32(header + 8) != BECORE_MCSC_RECIPE_HEADER_BYTES ||
	    get_unaligned_le32(header + 12) != BECORE_MCSC_RECIPE_RECORD_BYTES ||
	    get_unaligned_le32(header + 16) != BECORE_MCSC_HEADER_COUNT ||
	    get_unaligned_le32(header + 20) != BECORE_MCSC_RECIPE_BYTES ||
	    get_unaligned_le32(header + 24) || get_unaligned_le32(header + 28))
		return -EINVAL;
	if (becore->output.size < becore_yuvp_output_size(input) ||
	    becore->mcsc_output.size != becore_mcsc_output_size())
		return -EINVAL;

	for (i = 0; i < BECORE_MCSC_HEADER_COUNT;
	     i++, record += BECORE_MCSC_RECIPE_RECORD_BYTES) {
		const struct becore_cmdq_shape *shape = &becore_mcsc_shape[i];
		const u8 *words = record + 12;
		u16 used_mask;
		u32 word;

		if (!shape->valid_words || shape->valid_words > 16)
			return -EINVAL;
		used_mask = shape->valid_words == 16 ? U16_MAX :
			    GENMASK(shape->valid_words - 1, 0);
		if (get_unaligned_le32(record) != shape->mode ||
		    get_unaligned_le32(record + 4) != shape->target ||
		    get_unaligned_le32(record + 8) != shape->type_map ||
		    (shape->address_mask & ~used_mask) ||
		    (shape->typed_mask & ~used_mask) ||
		    (shape->generated_mask & ~used_mask) ||
		    (shape->fixed_mask & ~used_mask) ||
		    (shape->address_mask & shape->typed_mask) ||
		    (shape->address_mask & shape->generated_mask) ||
		    (shape->address_mask & shape->fixed_mask) ||
		    (shape->typed_mask & shape->generated_mask) ||
		    (shape->typed_mask & shape->fixed_mask) ||
		    (shape->generated_mask & shape->fixed_mask))
			return -EINVAL;

		for (word = 0; word < 16; word++) {
			u32 value = get_unaligned_le32(words + word * 4);

			if (word >= shape->valid_words) {
				if (value)
					return -EINVAL;
				continue;
			}
			if (shape->mode == 0x00090000 && !(word & 1)) {
				if (value != shape->pair_registers[word / 2])
					return -EINVAL;
				continue;
			}
			if ((shape->fixed_mask & BIT(word)) &&
			    value != shape->fixed_values[word])
				return -EINVAL;

			if (shape->typed_mask & BIT(word)) {
				u32 reg;

				if (shape->mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape->pair_registers[word / 2];
				if (becore_mcsc_dma_value(typed_count, reg,
							  BECORE_MCSC_INPUT_CAPTURED_VOTF,
							  NULL))
					return -EINVAL;
				typed_count++;
			}

			if (shape->generated_mask & BIT(word)) {
				u32 reg;

				if (becore_shape_register(shape, word, &reg) ||
				    becore_generated_value(BECORE_MCSC, reg,
							   NULL))
					return -EINVAL;
				generated_count++;
			}
			if (shape->address_mask & BIT(word)) {
				u32 reg;

				if (shape->mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape->pair_registers[word / 2];
				if (becore_mcsc_address_dma(becore, reg) ==
				    DMA_MAPPING_ERROR)
					return -EINVAL;
				address_count++;
			}
		}
	}

	if (address_count != 4 || typed_count != BECORE_MCSC_DMA_WORD_COUNT ||
	    generated_count != BECORE_MCSC_GENERATED_WORDS)
		return -EINVAL;

	return 0;
}

static int
becore_encode_mcsc(struct becore_device *becore,
		   enum becore_mcsc_input_transport transport)
{
	struct becore_cmdq_program *program = &becore->mcsc_program;
	const u8 *record = becore->mcsc_recipe +
			   BECORE_MCSC_RECIPE_HEADER_BYTES;
	size_t payload_offset = ALIGN((size_t)BECORE_MCSC_HEADER_COUNT *
				      BECORE_CMDQ_HEADER_BYTES,
				      BECORE_CMDQ_PAYLOAD_BYTES);
	u32 typed_count = 0;
	u32 generated_count = 0;
	u32 i;

	if (!program->cpu || program->header_count != BECORE_MCSC_HEADER_COUNT ||
	    program->size != becore_cmdq_program_size(BECORE_MCSC_HEADER_COUNT) ||
	    upper_32_bits(program->dma) ||
	    upper_32_bits(program->dma + program->size - 1))
		return -EINVAL;

	memset(program->cpu, 0, program->size);
	for (i = 0; i < BECORE_MCSC_HEADER_COUNT;
	     i++, record += BECORE_MCSC_RECIPE_RECORD_BYTES) {
		const struct becore_cmdq_shape *shape = &becore_mcsc_shape[i];
		u8 *header = (u8 *)program->cpu + i * BECORE_CMDQ_HEADER_BYTES;
		u8 *payload = (u8 *)program->cpu + payload_offset +
			      i * BECORE_CMDQ_PAYLOAD_BYTES;
		dma_addr_t payload_dma = program->dma + payload_offset +
					 i * BECORE_CMDQ_PAYLOAD_BYTES;
		u32 word;

		put_unaligned_le32(shape->mode, header);
		put_unaligned_le32(lower_32_bits(payload_dma), header + 4);
		put_unaligned_le32(shape->target, header + 8);
		put_unaligned_le32(shape->type_map, header + 12);
		memcpy(payload, record + 12, BECORE_CMDQ_PAYLOAD_BYTES);

		for (word = 0; word < shape->valid_words; word++) {
			dma_addr_t dma;
			u32 reg;
			u32 value;

			if (shape->typed_mask & BIT(word)) {
				reg = shape->pair_registers[word / 2];
				if (becore_mcsc_dma_value(typed_count, reg, transport,
							  &value))
					return -EINVAL;
				put_unaligned_le32(value, payload + word * 4);
				typed_count++;
				continue;
			}

			if (shape->generated_mask & BIT(word)) {
				if (becore_shape_register(shape, word, &reg) ||
				    becore_generated_value(BECORE_MCSC, reg,
							   &value))
					return -EINVAL;
				put_unaligned_le32(value, payload + word * 4);
				generated_count++;
				continue;
			}

			if (!(shape->address_mask & BIT(word)))
				continue;
			reg = shape->pair_registers[word / 2];
			dma = becore_mcsc_address_dma(becore, reg);
			if (dma == DMA_MAPPING_ERROR || upper_32_bits(dma))
				return -EINVAL;
			put_unaligned_le32(lower_32_bits(dma), payload + word * 4);
		}
	}

	if (typed_count != BECORE_MCSC_DMA_WORD_COUNT ||
	    generated_count != BECORE_MCSC_GENERATED_WORDS)
		return -EINVAL;

	return 0;
}

static void becore_prepare_irqs(struct becore_block *block)
{
	writel_relaxed(block->int0_mask_prepare,
		       block->base + BECORE_INT0_ENABLE);
	if (block->int1_mask)
		writel_relaxed(block->int1_mask,
			       block->base + BECORE_INT1_ENABLE);
	writel_relaxed(block->cmdq_int_mask,
		       block->base + BECORE_CMDQ_INT_ENABLE);
}

static void becore_quiesce_irqs(struct becore_block *block)
{
	writel_relaxed(BIT(2), block->base + BECORE_INT0_ENABLE);
	if (block->int1_mask)
		writel_relaxed(0, block->base + BECORE_INT1_ENABLE);
	writel_relaxed(0, block->base + BECORE_CMDQ_INT_ENABLE);
}

static void becore_enable_linux_irqs(struct becore_device *becore)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(becore->irqs); i++)
		enable_irq(becore->irqs[i].irq);
	becore->irqs_enabled = true;
}

static void becore_disable_linux_irqs(struct becore_device *becore)
{
	unsigned int i;

	if (!becore->irqs_enabled)
		return;

	/* disable_irq() also waits for a handler already running on another CPU. */
	for (i = 0; i < ARRAY_SIZE(becore->irqs); i++)
		disable_irq(becore->irqs[i].irq);
	becore->irqs_enabled = false;
}

static int becore_runtime_resume(struct device *dev)
{
	struct becore_device *becore = dev_get_drvdata(dev);
	unsigned int i;
	int ret;

	ret = becore_reset_all(becore);
	if (ret) {
		/* Report success so runtime PM retains all supplier references. */
		becore->reset_failed = true;
		return 0;
	}
	becore->reset_failed = false;

	becore_write_table(&becore->blocks[BECORE_RGBP], becore_rgbp_init,
			   ARRAY_SIZE(becore_rgbp_init));
	becore_write_table(&becore->blocks[BECORE_MCFP], becore_mcfp_init,
			   ARRAY_SIZE(becore_mcfp_init));
	becore_write_table(&becore->blocks[BECORE_YUVP], becore_yuvp_init,
			   ARRAY_SIZE(becore_yuvp_init));
	becore_write_table(&becore->blocks[BECORE_MCSC], becore_mcsc_init,
			   ARRAY_SIZE(becore_mcsc_init));

	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		becore_prepare_irqs(&becore->blocks[i]);
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		writel_relaxed(becore->blocks[i].int0_mask,
			       becore->blocks[i].base + BECORE_INT0_ENABLE);

	becore_enable_linux_irqs(becore);

	return 0;
}

static int becore_runtime_suspend(struct device *dev)
{
	struct becore_device *becore = dev_get_drvdata(dev);
	unsigned int i;
	int ret;

	becore_disable_linux_irqs(becore);
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		becore_quiesce_irqs(&becore->blocks[i]);
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		writel_relaxed(0, becore->blocks[i].base + BECORE_INT0_ENABLE);

	/* A failed reset must veto the following genpd power-down. */
	ret = becore_reset_all(becore);
	becore->reset_failed = !!ret;

	return ret;
}

static void becore_process_frame_irq(struct becore_block *block, u32 status,
				     bool error)
{
	struct becore_device *becore = block->becore;
	enum becore_block_id id = block - becore->blocks;
	unsigned long flags;
	unsigned int i;
	bool complete_run = false;

	if (id == BECORE_MCFP)
		return;

	spin_lock_irqsave(&becore->run_lock, flags);
	if (!becore->running)
		goto unlock;
	if (error) {
		becore->irq_error = true;
		becore->abort_run = true;
		complete_run = true;
		goto unlock;
	}
	if (!(becore->expected_mask & BIT(id)))
		goto unlock;

	if (status & BECORE_INT_CMDQ_HOLD) {
		becore->cmdq_hold_mask |= BIT(id);
		if ((becore->cmdq_hold_mask & becore->expected_mask) ==
		    becore->expected_mask &&
		    !becore->start_issued && !becore->abort_run) {
			becore->start_issued = true;
			/* Release the downstream end of the chain first. */
			for (i = BECORE_NUM_BLOCKS; i-- > 0;)
				if (becore->expected_mask & BIT(i))
					writel(1, becore->blocks[i].base +
					       BECORE_CMDQ_ADD_TO_QUEUE_0);
		}
	}
	if (status & BECORE_INT_FRAME_END) {
		becore->frame_done_mask |= BIT(id);
		if ((becore->frame_done_mask & becore->expected_mask) ==
		    becore->expected_mask)
			complete_run = true;
	}

unlock:
	spin_unlock_irqrestore(&becore->run_lock, flags);

	if (complete_run)
		complete(&becore->run_completion);
}

static irqreturn_t becore_irq_handler(int irq, void *data)
{
	struct becore_irq *irq_data = data;
	struct becore_block *block = irq_data->block;
	u32 cmdq_status;
	u32 status;

	if (irq_data->int1) {
		status = readl_relaxed(block->base + BECORE_INT1_STATUS);
		if (!status)
			return IRQ_NONE;

		writel_relaxed(status, block->base + BECORE_INT1_CLEAR);
		WRITE_ONCE(block->last_int1, status);
		atomic64_inc(&block->int1_count);
		becore_process_frame_irq(block, status, true);
		return IRQ_HANDLED;
	}

	status = readl_relaxed(block->base + BECORE_INT0_STATUS);
	cmdq_status = readl_relaxed(block->base + BECORE_CMDQ_INT_STATUS);
	if (!status && !cmdq_status)
		return IRQ_NONE;

	if (status)
		writel_relaxed(status, block->base + BECORE_INT0_CLEAR);
	if (cmdq_status)
		writel_relaxed(cmdq_status, block->base + BECORE_CMDQ_INT_CLEAR);

	WRITE_ONCE(block->last_int0, status);
	WRITE_ONCE(block->last_cmdq_int, cmdq_status);
	atomic64_inc(&block->int0_count);
	becore_process_frame_irq(block, status,
				 !!cmdq_status || !!(status & ~BECORE_INT_EXPECTED));

	return IRQ_HANDLED;
}

static int becore_map_resources(struct platform_device *pdev,
				struct becore_device *becore)
{
	static const char * const block_names[] = {
		"rgbp", "mcfp", "yuvp", "mcsc",
	};
	struct device *dev = &pdev->dev;
	unsigned int i;

	for (i = 0; i < BECORE_NUM_BLOCKS; i++) {
		becore->blocks[i].base =
			devm_platform_ioremap_resource_byname(pdev, block_names[i]);
		if (IS_ERR(becore->blocks[i].base))
			return dev_err_probe(dev, PTR_ERR(becore->blocks[i].base),
					     "cannot map %s\n", block_names[i]);
	}

	for (i = 0; i < ARRAY_SIZE(becore->ssmt); i++) {
		becore->ssmt[i] =
			devm_platform_ioremap_resource_byname(pdev,
						       becore_ssmt_names[i]);
		if (IS_ERR(becore->ssmt[i]))
			return dev_err_probe(dev, PTR_ERR(becore->ssmt[i]),
					     "cannot map %s\n", becore_ssmt_names[i]);
	}

	becore->sysreg_rgbp =
		devm_platform_ioremap_resource_byname(pdev, "sysreg-rgbp");
	if (IS_ERR(becore->sysreg_rgbp))
		return dev_err_probe(dev, PTR_ERR(becore->sysreg_rgbp),
				     "cannot map sysreg-rgbp\n");

	becore->sysreg_mcsc =
		devm_platform_ioremap_resource_byname(pdev, "sysreg-mcsc");
	if (IS_ERR(becore->sysreg_mcsc))
		return dev_err_probe(dev, PTR_ERR(becore->sysreg_mcsc),
				     "cannot map sysreg-mcsc\n");

	return 0;
}

static int becore_request_irqs(struct platform_device *pdev,
				struct becore_device *becore)
{
	struct device *dev = &pdev->dev;
	unsigned int i;
	int irq;
	int ret;

	for (i = 0; i < ARRAY_SIZE(becore->irqs); i++) {
		irq = platform_get_irq_byname(pdev, becore_irq_names[i]);
		if (irq < 0)
			return irq;

		becore->irqs[i].block = &becore->blocks[i / 2];
		becore->irqs[i].irq = irq;
		becore->irqs[i].int1 = i & 1;
		ret = devm_request_irq(dev, irq, becore_irq_handler,
				       IRQF_NO_AUTOEN,
				       becore_irq_names[i], &becore->irqs[i]);
		if (ret)
			return dev_err_probe(dev, ret, "cannot request %s\n",
					     becore_irq_names[i]);
	}

	return 0;
}

static int becore_alloc_dma_buffer(struct becore_device *becore,
				   struct becore_dma_buffer *buffer,
				   size_t size, const char *name)
{
	buffer->cpu = dmam_alloc_coherent(becore->dev, size, &buffer->dma,
					  GFP_KERNEL);
	if (!buffer->cpu)
		return dev_err_probe(becore->dev, -ENOMEM,
				     "cannot allocate %s buffer\n", name);
	buffer->size = size;
	if (upper_32_bits(buffer->dma) ||
	    upper_32_bits(buffer->dma + buffer->size - 1))
		return dev_err_probe(becore->dev, -ERANGE,
				     "%s buffer is outside 32-bit DMA\n", name);

	return 0;
}

static int becore_ltm_grid_generate(struct becore_device *becore)
{
	struct becore_dma_buffer *grid = &becore->grid;
	u32 row, column, group, channel;

	static_assert(sizeof(struct becore_ltm_grid_cell) ==
		      BECORE_LTM_GRID_CELL_BYTES);
	if (!grid->cpu || grid->size != BECORE_GRID_SIZE ||
	    BECORE_LTM_GRID_WIDTH_CELLS * BECORE_LTM_GRID_CELL_BYTES >
		BECORE_LTM_GRID_ROW_BYTES ||
	    BECORE_LTM_GRID_HEIGHT_CELLS > BECORE_LTM_GRID_ROWS)
		return -EINVAL;

	/*
	 * Lyric's LTM translator stores four Q14 gains followed by four signed
	 * offsets in each 16-byte group.  Unity gains and zero offsets provide a
	 * neutral policy surface; inactive cells and physical-row padding stay 0.
	 */
	memset(grid->cpu, 0, grid->size);
	for (row = 0; row < BECORE_LTM_GRID_HEIGHT_CELLS; row++) {
		u8 *row_base = (u8 *)grid->cpu +
			       row * BECORE_LTM_GRID_ROW_BYTES;

		for (column = 0; column < BECORE_LTM_GRID_WIDTH_CELLS;
		     column++) {
			struct becore_ltm_grid_cell *cell =
				(void *)(row_base +
					 column * BECORE_LTM_GRID_CELL_BYTES);

			for (group = 0; group < ARRAY_SIZE(cell->groups); group++)
				for (channel = 0;
				     channel < ARRAY_SIZE(cell->groups[group].gain);
				     channel++)
					cell->groups[group].gain[channel] =
						cpu_to_le16(BECORE_LTM_UNITY_Q14);
		}
	}

	grid->staged_bytes = grid->size;
	becore->grid_generation = 1;

	return 0;
}

static void becore_free_shared_input(void *data)
{
	struct becore_device *becore = data;
	unsigned int i;

	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_dma_buffer *input = &becore->inputs[i].buffer;

		if (input->cpu)
			dma_vunmap_noncontiguous(becore->dev, input->cpu);
		if (input->sgt)
			dma_free_noncontiguous(becore->dev, input->size,
					       input->sgt, DMA_BIDIRECTIONAL);
		input->cpu = NULL;
		input->sgt = NULL;
	}
}

static int becore_alloc_shared_input(struct becore_device *becore)
{
	unsigned int i;
	int ret;

	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_dma_buffer *input = &becore->inputs[i].buffer;

		input->size = becore_rgbp_input_size();
		input->sgt = dma_alloc_noncontiguous(becore->dev, input->size,
						     DMA_BIDIRECTIONAL,
						     GFP_KERNEL, 0);
		if (!input->sgt) {
			ret = -ENOMEM;
			goto err_free;
		}

		input->dma = sg_dma_address(input->sgt->sgl);
		input->cpu = dma_vmap_noncontiguous(becore->dev, input->size,
						    input->sgt);
		if (!input->cpu) {
			ret = -ENOMEM;
			goto err_free;
		}
		if (upper_32_bits(input->dma) ||
		    upper_32_bits(input->dma + input->size - 1)) {
			ret = -ERANGE;
			goto err_free;
		}
	}

	ret = devm_add_action_or_reset(becore->dev,
				       becore_free_shared_input, becore);
	return ret;

err_free:
	becore_free_shared_input(becore);
	return dev_err_probe(becore->dev, ret,
			     "cannot allocate Bayer input slot %u\n", i);
}

static int becore_alloc_cmdq_buffer(struct becore_device *becore,
				    struct becore_cmdq_program *program,
				    u32 header_count, const char *name)
{
	program->header_count = header_count;
	program->size = becore_cmdq_program_size(header_count);
	program->cpu = dmam_alloc_coherent(becore->dev, program->size,
					   &program->dma, GFP_KERNEL);
	if (!program->cpu)
		return dev_err_probe(becore->dev, -ENOMEM,
				     "cannot allocate %s CMDQ program\n",
				     name);
	if (upper_32_bits(program->dma) ||
	    upper_32_bits(program->dma + program->size - 1))
		return dev_err_probe(becore->dev, -ERANGE,
				     "%s CMDQ program is outside 32-bit DMA\n",
				     name);

	return 0;
}

static int becore_alloc_cmdq_program(struct becore_device *becore,
				     enum becore_block_id id,
				     u32 header_count)
{
	return becore_alloc_cmdq_buffer(becore, &becore->program[id],
					header_count, becore->blocks[id].name);
}

static int becore_alloc_diagnostic(struct becore_device *becore)
{
	size_t output_size = becore_yuvp_output_allocation_size();
	int ret;

	ret = becore_generated_tables_validate(becore->dev);
	if (ret)
		return ret;
	ret = becore_noise_knots_resolve(becore->dev);
	if (ret)
		return ret;
	becore->recipe = devm_kzalloc(becore->dev, BECORE_RECIPE_BYTES,
				      GFP_KERNEL);
	if (!becore->recipe)
		return -ENOMEM;
	ret = becore_recipe_generate(becore);
	if (ret)
		return dev_err_probe(becore->dev, ret,
				     "invalid built-in RGBP/YUVP recipe\n");
	becore->gtnr_recipe = devm_kzalloc(becore->dev,
					   BECORE_GTNR_RECIPE_BYTES, GFP_KERNEL);
	if (!becore->gtnr_recipe)
		return -ENOMEM;
	becore->mcsc_recipe = devm_kzalloc(becore->dev,
					   BECORE_MCSC_RECIPE_BYTES, GFP_KERNEL);
	if (!becore->mcsc_recipe)
		return -ENOMEM;
	ret = becore_mcsc_recipe_generate(becore);
	if (ret)
		return dev_err_probe(becore->dev, ret,
				     "invalid built-in MCSC recipe\n");

	ret = becore_alloc_shared_input(becore);
	if (ret)
		return ret;
	ret = becore_alloc_dma_buffer(becore, &becore->grid,
				      BECORE_GRID_SIZE, "YUVP grid");
	if (ret)
		return ret;
	ret = becore_ltm_grid_generate(becore);
	if (ret)
		return dev_err_probe(becore->dev, ret,
				     "cannot generate neutral YUVP grid\n");
	ret = becore_alloc_dma_buffer(becore, &becore->output,
				      output_size, "YUVP output");
	if (ret)
		return ret;
	ret = becore_alloc_dma_buffer(becore, &becore->gtnr_output,
				      becore_gtnr_surface_size(), "GTNR output");
	if (ret)
		return ret;
	ret = becore_alloc_dma_buffer(becore, &becore->mcsc_output,
				      becore_mcsc_output_size(), "MCSC output");
	if (ret)
		return ret;
	ret = becore_alloc_cmdq_program(becore, BECORE_RGBP,
					BECORE_RGBP_HEADER_COUNT);
	if (ret)
		return ret;

	ret = becore_alloc_cmdq_program(becore, BECORE_YUVP,
					BECORE_YUVP_HEADER_COUNT);
	if (ret)
		return ret;
	ret = becore_alloc_cmdq_buffer(becore, &becore->gtnr_program,
				       BECORE_GTNR_HEADER_COUNT, "GTNR startup");
	if (ret)
		return ret;

	return becore_alloc_cmdq_buffer(becore, &becore->mcsc_program,
					BECORE_MCSC_HEADER_COUNT, "MCSC");
}

static int becore_clone_sgtable(struct sg_table *dst,
				struct sg_table *src)
{
	struct scatterlist *src_sg;
	struct scatterlist *dst_sg;
	unsigned int i;
	int ret;

	ret = sg_alloc_table(dst, src->orig_nents, GFP_KERNEL);
	if (ret)
		return ret;

	dst_sg = dst->sgl;
	for_each_sg(src->sgl, src_sg, src->orig_nents, i) {
		sg_set_page(dst_sg, sg_page(src_sg), src_sg->length,
			    src_sg->offset);
		dst_sg = sg_next(dst_sg);
	}

	return 0;
}

/**
 * exynos_becore_input_map() - map the BE-core input into its producer domain
 * @backend: BE-core platform device
 * @producer: device which will write the compressed Bayer object
 * @ops: producer lifetime operations for an ordinary processed stream
 * @producer_data: producer-private callback argument
 *
 * Each producer_acquire() returns one generation-tagged slot and its address
 * in @producer's DMA domain.  BE-core retains distinct mappings of the same
 * pages and never exposes those IOVAs to the producer.
 */
struct exynos_becore_input *
exynos_becore_input_map(struct device *backend, struct device *producer,
			const struct exynos_becore_input_producer_ops *ops,
			void *producer_data)
{
	struct becore_device *becore;
	struct exynos_becore_input *input;
	unsigned int i;
	int ret;

	if (!backend || !producer || !ops || !ops->start_streaming ||
	    !ops->stop_streaming)
		return ERR_PTR(-EINVAL);
	becore = dev_get_drvdata(backend);
	if (!becore || !becore->inputs[0].buffer.sgt)
		return ERR_PTR(-EPROBE_DEFER);

	input = kzalloc_obj(*input, GFP_KERNEL);
	if (!input)
		return ERR_PTR(-ENOMEM);
	input->becore = becore;
	input->producer = get_device(producer);
	input->ops = ops;
	input->producer_data = producer_data;
	refcount_set(&input->callback_users, 1);
	init_waitqueue_head(&input->callback_wait);

	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_dma_buffer *slot = &becore->inputs[i].buffer;

		ret = becore_clone_sgtable(&input->sgts[i], slot->sgt);
		if (ret)
			goto err_mappings;
		ret = dma_map_sgtable(producer, &input->sgts[i],
				      DMA_FROM_DEVICE, 0);
		if (ret) {
			sg_free_table(&input->sgts[i]);
			goto err_mappings;
		}
		input->dmas[i] = sg_dma_address(input->sgts[i].sgl);
		if (input->sgts[i].nents != 1 ||
		    sg_dma_len(input->sgts[i].sgl) < slot->size ||
		    upper_32_bits(input->dmas[i]) ||
		    upper_32_bits(input->dmas[i] + slot->size - 1)) {
			ret = -ERANGE;
			dma_unmap_sgtable(producer, &input->sgts[i],
					  DMA_FROM_DEVICE, 0);
			sg_free_table(&input->sgts[i]);
			goto err_mappings;
		}
	}
	mutex_lock(&becore->lock);
	if (becore->reset_failed) {
		ret = -EIO;
		mutex_unlock(&becore->lock);
		goto err_mappings;
	}
	if (becore->input_producer) {
		ret = -EBUSY;
		mutex_unlock(&becore->lock);
		goto err_mappings;
	}
	becore->input_producer = input;
	mutex_unlock(&becore->lock);

	return input;

err_mappings:
	while (i--) {
		dma_unmap_sgtable(producer, &input->sgts[i],
				  DMA_FROM_DEVICE, 0);
		sg_free_table(&input->sgts[i]);
	}
	put_device(input->producer);
	kfree(input);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(exynos_becore_input_map);

/**
 * exynos_becore_input_disconnect() - sever and join the producer callbacks
 * @input: attachment returned by exynos_becore_input_map()
 *
 * No callback can begin after this returns.  An ordinary BE-core stream is
 * cancelled and errored, but the caller must still quiesce its DMA before
 * exynos_becore_input_unmap() releases or retains the shared mappings.
 */
void exynos_becore_input_disconnect(struct exynos_becore_input *input)
{
	struct becore_device *becore;
	unsigned long flags;
	bool cancel = false;
	bool streaming;

	if (!input)
		return;
	becore = input->becore;

	mutex_lock(&becore->lock);
	if (input->disconnected) {
		mutex_unlock(&becore->lock);
		wait_event(input->callback_wait,
			   refcount_read(&input->callback_users) == 1);
		return;
	}
	if (WARN_ON_ONCE(becore->input_producer != input)) {
		mutex_unlock(&becore->lock);
		return;
	}

	input->disconnected = true;
	becore->input_producer = NULL;
	streaming = becore->video_streaming;
	becore->video_streaming = false;
	becore->producer_streaming = false;
	spin_lock_irqsave(&becore->run_lock, flags);
	if (becore->running) {
		becore->abort_run = true;
		cancel = true;
	}
	spin_unlock_irqrestore(&becore->run_lock, flags);
	if (cancel)
		complete(&becore->run_completion);
	mutex_unlock(&becore->lock);

	cancel_work_sync(&becore->video_work);
	if (streaming) {
		becore_video_controls_ungrab(becore);
		vb2_queue_error(&becore->queue);
		becore_video_return_all(becore, VB2_BUF_STATE_ERROR);
	}
	wait_event(input->callback_wait,
		   refcount_read(&input->callback_users) == 1);
}
EXPORT_SYMBOL_GPL(exynos_becore_input_disconnect);

static struct exynos_becore_input *
becore_input_callback_get(struct becore_device *becore)
{
	struct exynos_becore_input *input = becore->input_producer;

	lockdep_assert_held(&becore->lock);
	if (!input || input->disconnected)
		return NULL;
	refcount_inc(&input->callback_users);

	return input;
}

static void becore_input_callback_put(struct exynos_becore_input *input)
{
	refcount_dec(&input->callback_users);
	wake_up_all(&input->callback_wait);
}

void exynos_becore_input_unmap(struct exynos_becore_input *input)
{
	struct becore_device *becore;
	unsigned int i;

	if (!input)
		return;
	becore = input->becore;
	exynos_becore_input_disconnect(input);

	mutex_lock(&becore->lock);
	if (WARN_ON_ONCE(!input->disconnected || becore->input_producer)) {
		mutex_unlock(&becore->lock);
		return;
	}
	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		if (becore->inputs[i].state == BECORE_INPUT_BACKEND ||
		    becore->inputs[i].state == BECORE_INPUT_QUARANTINED) {
			dev_crit(becore->dev,
				 "retaining producer mappings for active/quarantined input slot %u\n",
				 i);
			mutex_unlock(&becore->lock);
			return;
		}
	}
	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_input_slot *slot = &becore->inputs[i];

		if (slot->state == BECORE_INPUT_PRODUCER)
			dma_sync_sgtable_for_cpu(input->producer,
						 &input->sgts[i], DMA_FROM_DEVICE);
		if (slot->state != BECORE_INPUT_BACKEND) {
			slot->state = BECORE_INPUT_FREE;
			slot->buffer.staged_bytes = 0;
			slot->producer_cookie = 0;
			slot->ready_sequence = 0;
		}
	}
	mutex_unlock(&becore->lock);

	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		dma_unmap_sgtable(input->producer, &input->sgts[i],
				  DMA_FROM_DEVICE, 0);
		sg_free_table(&input->sgts[i]);
	}
	put_device(input->producer);
	WARN_ON_ONCE(!refcount_dec_and_test(&input->callback_users));
	kfree(input);
}
EXPORT_SYMBOL_GPL(exynos_becore_input_unmap);

size_t exynos_becore_input_size(struct exynos_becore_input *input)
{
	return input->becore->inputs[0].buffer.size;
}
EXPORT_SYMBOL_GPL(exynos_becore_input_size);

static struct becore_input_slot *
becore_input_ticket(struct exynos_becore_input *input,
		    const struct exynos_becore_input_buffer *buffer)
{
	struct becore_input_slot *slot;

	if (!buffer || buffer->slot >= BECORE_INPUT_SLOT_COUNT)
		return NULL;
	slot = &input->becore->inputs[buffer->slot];
	if (slot->state != BECORE_INPUT_PRODUCER ||
	    slot->producer_cookie != buffer->cookie ||
	    input->dmas[buffer->slot] != buffer->dma ||
	    slot->buffer.size != buffer->size)
		return NULL;

	return slot;
}

int exynos_becore_input_producer_acquire(struct exynos_becore_input *input,
					 struct exynos_becore_input_buffer *buffer)
{
	struct becore_device *becore = input->becore;
	struct becore_input_slot *slot = NULL;
	u64 cookie;
	unsigned int i;
	int ret = 0;

	if (!buffer)
		return -EINVAL;

	mutex_lock(&becore->lock);
	if (becore->input_producer != input) {
		ret = -EINVAL;
		goto unlock;
	}
	if (becore->reset_failed) {
		ret = -EIO;
		goto unlock;
	}
	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++)
		if (becore->inputs[i].state == BECORE_INPUT_FREE) {
			slot = &becore->inputs[i];
			break;
		}
	if (!slot) {
		ret = -EBUSY;
		goto unlock;
	}

	/* Discard any cached CPU copy before the front end overwrites it. */
	dma_sync_sgtable_for_device(becore->dev, slot->buffer.sgt,
				    DMA_BIDIRECTIONAL);
	dma_sync_sgtable_for_device(input->producer, &input->sgts[i],
				    DMA_FROM_DEVICE);
	slot->buffer.staged_bytes = 0;
	slot->ready_sequence = 0;
	cookie = ++becore->producer_sequence;
	if (!cookie)
		cookie = ++becore->producer_sequence;
	slot->producer_cookie = cookie;
	slot->state = BECORE_INPUT_PRODUCER;
	*buffer = (struct exynos_becore_input_buffer) {
		.dma = input->dmas[i],
		.size = slot->buffer.size,
		.cookie = cookie,
		.slot = i,
	};

unlock:
	mutex_unlock(&becore->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_becore_input_producer_acquire);

int exynos_becore_input_producer_complete(struct exynos_becore_input *input,
					  const struct exynos_becore_input_buffer *buffer)
{
	struct becore_device *becore = input->becore;
	struct becore_input_slot *slot;
	int ret = 0;

	mutex_lock(&becore->lock);
	if (becore->input_producer != input) {
		ret = -EINVAL;
		goto unlock;
	}
	slot = becore_input_ticket(input, buffer);
	if (!slot) {
		ret = -EINVAL;
		goto unlock;
	}

	/* The caller has quiesced the producer at a completed-frame boundary. */
	dma_sync_sgtable_for_cpu(input->producer, &input->sgts[buffer->slot],
				 DMA_FROM_DEVICE);
	dma_sync_sgtable_for_device(becore->dev, slot->buffer.sgt,
				    DMA_TO_DEVICE);
	slot->buffer.staged_bytes = slot->buffer.size;
	slot->producer_cookie = 0;
	slot->ready_sequence = ++becore->input_sequence;
	slot->state = BECORE_INPUT_READY;
	if (becore->video_streaming && becore->producer_streaming)
		schedule_work(&becore->video_work);

unlock:
	mutex_unlock(&becore->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_becore_input_producer_complete);

void exynos_becore_input_producer_abort(struct exynos_becore_input *input,
					const struct exynos_becore_input_buffer *buffer)
{
	struct becore_device *becore = input->becore;
	struct becore_input_slot *slot;

	mutex_lock(&becore->lock);
	if (becore->input_producer == input) {
		slot = becore_input_ticket(input, buffer);
		if (!slot)
			goto unlock;
		dma_sync_sgtable_for_cpu(input->producer,
					 &input->sgts[buffer->slot],
					 DMA_FROM_DEVICE);
		slot->buffer.staged_bytes = 0;
		slot->producer_cookie = 0;
		slot->ready_sequence = 0;
		slot->state = BECORE_INPUT_FREE;
	}

unlock:
	mutex_unlock(&becore->lock);
}
EXPORT_SYMBOL_GPL(exynos_becore_input_producer_abort);

static ssize_t becore_stage_write(struct becore_device *becore,
				  const char __user *buf, size_t count,
				  loff_t *ppos, void *staged, size_t capacity,
				  size_t *staged_bytes, u32 *generation)
{
	ssize_t ret = count;

	if (!count)
		return 0;

	mutex_lock(&becore->lock);
	if (becore->reset_failed) {
		ret = -EIO;
		goto unlock;
	}
	if (becore->video_streaming) {
		ret = -EBUSY;
		goto unlock;
	}
	if (becore->running) {
		ret = -EBUSY;
		goto unlock;
	}
	if (staged == becore->inputs[0].buffer.cpu &&
	    becore->inputs[0].state != BECORE_INPUT_FREE) {
		ret = -EBUSY;
		goto unlock;
	}
	if (*ppos < 0 || *ppos > capacity) {
		ret = -EINVAL;
		goto unlock;
	}
	if (count > capacity - *ppos) {
		ret = -EFBIG;
		goto unlock;
	}
	if (*ppos == 0)
		*staged_bytes = 0;
	if (*ppos != *staged_bytes) {
		ret = -ESPIPE;
		goto unlock;
	}
	if (copy_from_user((u8 *)staged + *staged_bytes, buf, count)) {
		/* A real partial replacement is never considered a valid stage. */
		*staged_bytes = 0;
		ret = -EFAULT;
		goto unlock;
	}

	*staged_bytes += count;
	*ppos += count;
	if (generation && *staged_bytes == capacity)
		(*generation)++;

unlock:
	mutex_unlock(&becore->lock);
	return ret;
}

static ssize_t becore_recipe_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else
		ret = simple_read_from_buffer(buf, count, ppos, becore->recipe,
					      becore->recipe_staged_bytes);
	mutex_unlock(&becore->lock);

	return ret;
}

static ssize_t becore_recipe_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;

	return becore_stage_write(becore, buf, count, ppos, becore->recipe,
				  BECORE_RECIPE_BYTES,
				  &becore->recipe_staged_bytes,
				  &becore->recipe_generation);
}

static const struct file_operations becore_recipe_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_recipe_read,
	.write = becore_recipe_write,
	.llseek = default_llseek,
};

static ssize_t becore_gtnr_recipe_read(struct file *file, char __user *buf,
				       size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->gtnr_recipe,
					      becore->gtnr_recipe_staged_bytes);
	mutex_unlock(&becore->lock);

	return ret;
}

static ssize_t becore_gtnr_recipe_write(struct file *file,
					const char __user *buf, size_t count,
					loff_t *ppos)
{
	struct becore_device *becore = file->private_data;

	return becore_stage_write(becore, buf, count, ppos,
				  becore->gtnr_recipe, BECORE_GTNR_RECIPE_BYTES,
				  &becore->gtnr_recipe_staged_bytes,
				  &becore->gtnr_recipe_generation);
}

static const struct file_operations becore_gtnr_recipe_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_gtnr_recipe_read,
	.write = becore_gtnr_recipe_write,
	.llseek = default_llseek,
};

static ssize_t becore_mcsc_recipe_read(struct file *file, char __user *buf,
				       size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->mcsc_recipe,
					      becore->mcsc_recipe_staged_bytes);
	mutex_unlock(&becore->lock);

	return ret;
}

static ssize_t becore_mcsc_recipe_write(struct file *file,
					const char __user *buf, size_t count,
					loff_t *ppos)
{
	struct becore_device *becore = file->private_data;

	return becore_stage_write(becore, buf, count, ppos,
				  becore->mcsc_recipe, BECORE_MCSC_RECIPE_BYTES,
				  &becore->mcsc_recipe_staged_bytes,
				  &becore->mcsc_recipe_generation);
}

static const struct file_operations becore_mcsc_recipe_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_mcsc_recipe_read,
	.write = becore_mcsc_recipe_write,
	.llseek = default_llseek,
};

static ssize_t becore_input_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	struct becore_dma_buffer *input = &becore->inputs[0].buffer;

	return becore_stage_write(becore, buf, count, ppos,
				  input->cpu, input->size,
				  &input->staged_bytes, NULL);
}

static const struct file_operations becore_input_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = becore_input_write,
	.llseek = default_llseek,
};

static ssize_t becore_grid_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;

	return becore_stage_write(becore, buf, count, ppos,
				  becore->grid.cpu, becore->grid.size,
				  &becore->grid.staged_bytes,
				  &becore->grid_generation);
}

static const struct file_operations becore_grid_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = becore_grid_write,
	.llseek = default_llseek,
};

static ssize_t becore_output_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else if (!becore->completed_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->output.cpu,
					      becore->completed_output_size);
	mutex_unlock(&becore->lock);

	return ret;
}

static const struct file_operations becore_output_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_output_read,
	.llseek = default_llseek,
};

static ssize_t becore_mcsc_output_read(struct file *file, char __user *buf,
				       size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed || becore->output_quarantined)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else if (!becore->mcsc_completed_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->mcsc_output.cpu,
					      becore->mcsc_completed_output_size);
	mutex_unlock(&becore->lock);

	return ret;
}

static const struct file_operations becore_mcsc_output_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_mcsc_output_read,
	.llseek = default_llseek,
};

static ssize_t becore_encoded_read(struct file *file, char __user *buf,
				   size_t count, loff_t *ppos,
				   enum becore_block_id id)
{
	struct becore_device *becore = file->private_data;
	struct becore_cmdq_program *program = &becore->program[id];
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else if (!becore->completed_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos, program->cpu,
					      program->size);
	mutex_unlock(&becore->lock);

	return ret;
}

static ssize_t becore_rgbp_encoded_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	return becore_encoded_read(file, buf, count, ppos, BECORE_RGBP);
}

static const struct file_operations becore_rgbp_encoded_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_rgbp_encoded_read,
	.llseek = default_llseek,
};

static ssize_t becore_yuvp_encoded_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	return becore_encoded_read(file, buf, count, ppos, BECORE_YUVP);
}

static const struct file_operations becore_yuvp_encoded_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_yuvp_encoded_read,
	.llseek = default_llseek,
};

static ssize_t becore_gtnr_encoded_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else if (becore->gtnr_recipe_staged_bytes != BECORE_GTNR_RECIPE_BYTES ||
		 becore->gtnr_encoded_generation != becore->gtnr_recipe_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->gtnr_program.cpu,
					      becore->gtnr_program.size);
	mutex_unlock(&becore->lock);

	return ret;
}

static const struct file_operations becore_gtnr_encoded_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_gtnr_encoded_read,
	.llseek = default_llseek,
};

static int becore_gtnr_encode_set(void *data, u64 value)
{
	struct becore_device *becore = data;
	int ret;

	if (value != 1)
		return -EINVAL;

	mutex_lock(&becore->lock);
	if (becore->reset_failed) {
		ret = -EIO;
	} else if (becore->running || becore->video_streaming) {
		ret = -EBUSY;
	} else {
		ret = becore_gtnr_recipe_validate(becore);
		if (!ret)
			ret = becore_encode_gtnr(becore);
		if (!ret)
			becore->gtnr_encoded_generation =
				becore->gtnr_recipe_generation;
	}
	mutex_unlock(&becore->lock);

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(becore_gtnr_encode_fops, NULL,
			 becore_gtnr_encode_set, "%llu\n");

static ssize_t becore_mcsc_encoded_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else if (becore->mcsc_recipe_staged_bytes != BECORE_MCSC_RECIPE_BYTES ||
		 becore->mcsc_encoded_generation != becore->mcsc_recipe_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->mcsc_program.cpu,
					      becore->mcsc_program.size);
	mutex_unlock(&becore->lock);

	return ret;
}

static const struct file_operations becore_mcsc_encoded_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_mcsc_encoded_read,
	.llseek = default_llseek,
};

static int becore_mcsc_encode_set(void *data, u64 value)
{
	struct becore_device *becore = data;
	int ret;

	if (value != 1)
		return -EINVAL;

	mutex_lock(&becore->lock);
	if (becore->reset_failed) {
		ret = -EIO;
	} else if (becore->running || becore->video_streaming) {
		ret = -EBUSY;
	} else {
		ret = becore_mcsc_recipe_validate(becore);
		if (!ret)
			ret = becore_encode_mcsc(becore,
						 BECORE_MCSC_INPUT_CAPTURED_VOTF);
		if (!ret)
			becore->mcsc_encoded_generation =
				becore->mcsc_recipe_generation;
		if (!ret)
			becore->mcsc_encoded_transport =
				BECORE_MCSC_INPUT_CAPTURED_VOTF;
	}
	mutex_unlock(&becore->lock);

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(becore_mcsc_encode_fops, NULL,
			 becore_mcsc_encode_set, "%llu\n");

static void becore_clear_pending_irqs(struct becore_block *block)
{
	writel_relaxed(U32_MAX, block->base + BECORE_INT0_CLEAR);
	writel_relaxed(U32_MAX, block->base + BECORE_INT1_CLEAR);
	writel_relaxed(U32_MAX, block->base + BECORE_CMDQ_INT_CLEAR);
}

static void becore_publish_program(struct becore_device *becore,
				   enum becore_block_id id)
{
	struct becore_cmdq_program *program = id == BECORE_MCSC ?
		&becore->mcsc_program : &becore->program[id];
	struct becore_block *block = &becore->blocks[id];

	writel_relaxed(lower_32_bits(program->dma),
		       block->base + BECORE_CMDQ_QUE_CMD_L);
	writel_relaxed(BECORE_CMDQ_MODE | program->header_count,
		       block->base + BECORE_CMDQ_QUE_CMD_M);
	writel_relaxed(0xff, block->base + BECORE_CMDQ_QUE_CMD_H);
	writel_relaxed(1, block->base + BECORE_CMDQ_QUE_CMD_START);
}

static void becore_measure_buffer(const struct becore_dma_buffer *buffer,
				  u32 *changed_bytes, u32 *first_changed)
{
	const u8 *output = buffer->cpu;
	u32 changed = 0;
	u32 first = U32_MAX;
	u32 i;

	for (i = 0; i < buffer->size; i++) {
		if (output[i] == 0xa5)
			continue;
		if (first == U32_MAX)
			first = i;
		changed++;
	}
	*changed_bytes = changed;
	*first_changed = first;
}

static int becore_run_stage(struct becore_device *becore, u32 blocks)
{
	unsigned long flags;
	unsigned long waited;
	unsigned int i;
	int ret;

	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		if (blocks & BIT(i))
			becore_clear_pending_irqs(&becore->blocks[i]);
	reinit_completion(&becore->run_completion);

	spin_lock_irqsave(&becore->run_lock, flags);
	becore->expected_mask = blocks;
	becore->start_issued = false;
	if (becore->abort_run) {
		spin_unlock_irqrestore(&becore->run_lock, flags);
		return -ECANCELED;
	}
	spin_unlock_irqrestore(&becore->run_lock, flags);

	/* Publish every DMA ownership transition before starting the next stage. */
	dma_wmb();
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		if (blocks & BIT(i))
			becore_publish_program(becore, i);
	mutex_unlock(&becore->lock);

	waited = wait_for_completion_timeout(&becore->run_completion,
					     msecs_to_jiffies(BECORE_RUN_TIMEOUT_MS));

	mutex_lock(&becore->lock);
	spin_lock_irqsave(&becore->run_lock, flags);
	if (!waited)
		ret = -ETIMEDOUT;
	else if (becore->irq_error)
		ret = -EIO;
	else if (becore->abort_run)
		ret = -ECANCELED;
	else if ((becore->frame_done_mask & blocks) != blocks)
		ret = -EIO;
	else
		ret = 0;
	spin_unlock_irqrestore(&becore->run_lock, flags);

	return ret;
}

static struct becore_input_slot *
becore_next_input(struct becore_device *becore, bool allow_staged)
{
	struct becore_input_slot *next = NULL;
	unsigned int i;

	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_input_slot *slot = &becore->inputs[i];

		if (slot->state != BECORE_INPUT_READY)
			continue;
		if (!next || slot->ready_sequence < next->ready_sequence)
			next = slot;
	}
	if (next)
		return next;

	/* Slot zero remains the upload-and-repeat diagnostic oracle. */
	if (allow_staged && becore->inputs[0].state == BECORE_INPUT_FREE)
		return &becore->inputs[0];

	return NULL;
}

static int becore_run_frame(struct becore_device *becore, u32 output_profile,
			    bool ready_only, void *capture_output,
			    bool packed_output, bool run_mcsc)
{
	unsigned long flags;
	bool diagnostic_output = !capture_output;
	bool input_claimed = false;
	int pm_ret;
	int ret;
	u32 i;

	mutex_lock(&becore->lock);
	if (diagnostic_output && becore->video_streaming) {
		ret = -EBUSY;
		goto unlock;
	}
	if (!diagnostic_output && !becore->video_streaming) {
		ret = -ECANCELED;
		goto unlock;
	}
	if (becore->running) {
		ret = -EBUSY;
		goto unlock;
	}
	if (becore->reset_failed) {
		ret = -EIO;
		goto record_error;
	}
	if (output_profile >= BECORE_YUVP_OUTPUT_PROFILE_COUNT) {
		ret = -EINVAL;
		goto record_error;
	}
	if (packed_output && output_profile != BECORE_YUVP_OUTPUT_P010) {
		ret = -EINVAL;
		goto record_error;
	}
	if (run_mcsc && (output_profile != BECORE_YUVP_OUTPUT_SBWCL ||
			 packed_output)) {
		ret = -EINVAL;
		goto record_error;
	}
	becore->run_input = becore_next_input(becore, !ready_only);
	if (!becore->run_input) {
		if (ready_only) {
			ret = -ENODATA;
			goto unlock;
		}
		ret = -EBUSY;
		goto record_error;
	}
	becore->active_output_profile = output_profile;
	becore->active_output_packed = packed_output;
	becore->active_output_dma = becore->output.dma;
	becore->active_output_size = becore_active_output_size(becore);
	becore->active_capture_size = run_mcsc ?
		becore_mcsc_output_active_size() : becore->active_output_size;
	if (!becore->active_output_dma ||
	    upper_32_bits(becore->active_output_dma) ||
	    upper_32_bits(becore->active_output_dma +
			  becore->active_output_size - 1) ||
	    becore->active_output_size > becore->output.size) {
		ret = -EINVAL;
		goto record_error;
	}
	ret = becore_recipe_validate(becore);
	if (ret)
		goto record_error;
	if (run_mcsc) {
		ret = becore_mcsc_recipe_validate(becore);
		if (ret)
			goto record_error;
	}
	becore->run_input->state = BECORE_INPUT_BACKEND;
	input_claimed = true;

	ret = pm_runtime_resume_and_get(becore->dev);
	if (ret)
		goto record_error;
	if (becore->reset_failed) {
		/* The runtime callback deliberately retains every supplier. */
		ret = -EIO;
		goto record_error;
	}

	ret = becore_encode_programs(becore);
	if (ret)
		goto put_power;
	if (run_mcsc) {
		ret = becore_encode_mcsc(becore, BECORE_MCSC_INPUT_MEMORY);
		if (ret)
			goto put_power;
		becore->mcsc_encoded_generation = becore->mcsc_recipe_generation;
		becore->mcsc_encoded_transport = BECORE_MCSC_INPUT_MEMORY;
	}

	if (diagnostic_output) {
		memset(becore->output.cpu, 0xa5, becore->output.size);
		if (run_mcsc)
			memset(becore->mcsc_output.cpu, 0xa5,
			       becore->mcsc_output.size);
	}
	becore->output_changed_bytes = 0;
	becore->output_first_changed = U32_MAX;
	becore->mcsc_output_changed_bytes = 0;
	becore->mcsc_output_first_changed = U32_MAX;
	becore->mcsc_completed_generation = 0;
	becore->mcsc_completed_output_size = 0;
	for (i = 0; i < BECORE_NUM_BLOCKS; i++) {
		WRITE_ONCE(becore->blocks[i].last_int0, 0);
		WRITE_ONCE(becore->blocks[i].last_int1, 0);
		WRITE_ONCE(becore->blocks[i].last_cmdq_int, 0);
		atomic64_set(&becore->blocks[i].int0_count, 0);
		atomic64_set(&becore->blocks[i].int1_count, 0);
	}
	spin_lock_irqsave(&becore->run_lock, flags);
	becore->cmdq_hold_mask = 0;
	becore->frame_done_mask = 0;
	becore->expected_mask = 0;
	becore->abort_run = false;
	becore->irq_error = false;
	becore->running = true;
	becore->run_generation++;
	spin_unlock_irqrestore(&becore->run_lock, flags);
	becore->active_mcsc = run_mcsc;

	/* Move staged or producer-written Bayer pages into RGBP's DMA domain. */
	dma_sync_sgtable_for_device(becore->dev, becore->run_input->buffer.sgt,
				    DMA_TO_DEVICE);
	ret = becore_run_stage(becore, BECORE_YUVP_STAGE_BLOCKS);
	if (!ret && run_mcsc)
		ret = becore_run_stage(becore, BIT(BECORE_MCSC));

	spin_lock_irqsave(&becore->run_lock, flags);
	becore->running = false;
	spin_unlock_irqrestore(&becore->run_lock, flags);

	/* runtime_suspend synchronizes IRQs and resets all four processors. */
	pm_ret = pm_runtime_put_sync(becore->dev);
	if (pm_ret < 0) {
		/* Match probe: never leave a failed-reset device at usage zero. */
		pm_runtime_get_noresume(becore->dev);
		/*
		 * Neither DMA mapping may be returned after an unproven stop.
		 * The output is driver-owned, so userspace's vb2 buffer was never
		 * exposed to the processors and remains safe to return with ERROR.
		 */
		becore->run_input->state = BECORE_INPUT_QUARANTINED;
		becore->output_quarantined = true;
		becore->completed_generation = 0;
		becore->completed_output_size = 0;
		becore->mcsc_completed_generation = 0;
		becore->mcsc_completed_output_size = 0;
		if (!ret)
			ret = pm_ret;
	} else {
		dma_sync_sgtable_for_cpu(becore->dev,
					 becore->run_input->buffer.sgt,
					 DMA_TO_DEVICE);
		becore->run_input->state = BECORE_INPUT_FREE;
		becore->run_input->ready_sequence = 0;
	}
	becore->run_input = NULL;
	input_claimed = false;
	if (pm_ret >= 0)
		dma_rmb();
	if (capture_output && !ret && pm_ret >= 0)
		memcpy(capture_output,
		       run_mcsc ? becore->mcsc_output.cpu : becore->output.cpu,
		       becore->active_capture_size);
	if (diagnostic_output && pm_ret >= 0) {
		becore_measure_buffer(&becore->output,
				      &becore->output_changed_bytes,
				      &becore->output_first_changed);
		becore->completed_generation = becore->run_generation;
		becore->completed_output_size = becore->active_output_size;
		if (run_mcsc) {
			becore_measure_buffer(&becore->mcsc_output,
					      &becore->mcsc_output_changed_bytes,
					      &becore->mcsc_output_first_changed);
			if (!ret) {
				becore->mcsc_completed_generation =
					becore->run_generation;
				becore->mcsc_completed_output_size =
					becore_mcsc_output_size();
			}
		}
	}
	becore->last_run_result = ret;
	mutex_unlock(&becore->lock);

	return ret;

put_power:
	pm_ret = pm_runtime_put_sync(becore->dev);
	if (pm_ret < 0) {
		pm_runtime_get_noresume(becore->dev);
		if (!ret)
			ret = pm_ret;
	}
record_error:
	if (input_claimed) {
		becore->run_input->state = BECORE_INPUT_FREE;
		becore->run_input->ready_sequence = 0;
	}
	becore->run_input = NULL;
	becore->last_run_result = ret;
unlock:
	mutex_unlock(&becore->lock);
	return ret;
}

static int becore_run_set(void *data, u64 value)
{
	struct becore_device *becore = data;

	if (value != 1)
		return -EINVAL;
	if (READ_ONCE(becore->video_streaming))
		return -EBUSY;

	return becore_run_frame(becore, READ_ONCE(becore->output_profile),
				false, NULL, false, false);
}

static int becore_run_get(void *data, u64 *value)
{
	struct becore_device *becore = data;

	mutex_lock(&becore->lock);
	*value = becore->running;
	mutex_unlock(&becore->lock);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(becore_run_fops, becore_run_get, becore_run_set,
			 "%llu\n");

static int becore_mcsc_run_set(void *data, u64 value)
{
	struct becore_device *becore = data;

	if (value != 1)
		return -EINVAL;
	if (READ_ONCE(becore->video_streaming))
		return -EBUSY;

	return becore_run_frame(becore, BECORE_YUVP_OUTPUT_SBWCL,
				false, NULL, false, true);
}
DEFINE_DEBUGFS_ATTRIBUTE(becore_mcsc_run_fops, NULL, becore_mcsc_run_set,
			 "%llu\n");

static int becore_cancel_set(void *data, u64 value)
{
	struct becore_device *becore = data;
	unsigned long flags;
	bool cancelled = false;

	if (value != 1)
		return -EINVAL;

	mutex_lock(&becore->lock);
	if (becore->video_streaming) {
		mutex_unlock(&becore->lock);
		return -EBUSY;
	}
	spin_lock_irqsave(&becore->run_lock, flags);
	if (becore->running) {
		becore->abort_run = true;
		cancelled = true;
	}
	spin_unlock_irqrestore(&becore->run_lock, flags);
	if (cancelled)
		complete(&becore->run_completion);
	mutex_unlock(&becore->lock);

	return cancelled ? 0 : -EALREADY;
}
DEFINE_DEBUGFS_ATTRIBUTE(becore_cancel_fops, NULL, becore_cancel_set, "%llu\n");

/* ---- processed NV21 capture queue -------------------------------------- */

static void becore_video_fill_pix(struct v4l2_pix_format *pix)
{
	pix->width = becore_mcsc_output.width;
	pix->height = becore_mcsc_output.height;
	pix->pixelformat = V4L2_PIX_FMT_NV21;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = becore_mcsc_output.stride;
	pix->sizeimage = becore_mcsc_output_active_size();
	/* The captured recipe uses a full-range BT.601 RGB-to-YUV matrix. */
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->flags = 0;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static void becore_video_return_all(struct becore_device *becore,
				    enum vb2_buffer_state state)
{
	struct becore_video_buffer *buf, *tmp;
	LIST_HEAD(done);

	spin_lock_irq(&becore->queue_lock);
	list_splice_tail_init(&becore->queued_outputs, &done);
	spin_unlock_irq(&becore->queue_lock);

	list_for_each_entry_safe(buf, tmp, &done, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
}

static void becore_video_controls_snapshot(struct becore_device *becore,
					   struct exynos_becore_input_stream_config *config)
{
	v4l2_ctrl_lock(becore->red_balance);
	config->red_balance = becore->red_balance->val;
	config->blue_balance = becore->blue_balance->val;
	__v4l2_ctrl_grab(becore->red_balance, true);
	__v4l2_ctrl_grab(becore->blue_balance, true);
	v4l2_ctrl_unlock(becore->red_balance);
}

static void becore_video_controls_ungrab(struct becore_device *becore)
{
	v4l2_ctrl_lock(becore->red_balance);
	__v4l2_ctrl_grab(becore->red_balance, false);
	__v4l2_ctrl_grab(becore->blue_balance, false);
	v4l2_ctrl_unlock(becore->red_balance);
}

static void becore_video_discard_ready(struct becore_device *becore)
{
	unsigned int i;

	mutex_lock(&becore->lock);
	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_input_slot *slot = &becore->inputs[i];

		if (slot->state != BECORE_INPUT_READY)
			continue;
		slot->state = BECORE_INPUT_FREE;
		slot->buffer.staged_bytes = 0;
		slot->ready_sequence = 0;
	}
	mutex_unlock(&becore->lock);
}

static void becore_video_stop_producer(struct becore_device *becore)
{
	struct exynos_becore_input *input = NULL;

	mutex_lock(&becore->lock);
	if (becore->producer_streaming) {
		becore->producer_streaming = false;
		input = becore_input_callback_get(becore);
	}
	mutex_unlock(&becore->lock);

	if (!input)
		return;
	input->ops->stop_streaming(input->producer_data);
	becore_video_discard_ready(becore);
	becore_input_callback_put(input);
}

static void becore_video_fail(struct becore_device *becore,
			      struct becore_video_buffer *buf)
{
	mutex_lock(&becore->lock);
	becore->video_streaming = false;
	mutex_unlock(&becore->lock);

	becore_video_stop_producer(becore);
	becore_video_controls_ungrab(becore);
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	vb2_queue_error(&becore->queue);
	becore_video_return_all(becore, VB2_BUF_STATE_ERROR);
}

static void becore_video_work(struct work_struct *work)
{
	struct becore_device *becore =
		container_of(work, struct becore_device, video_work);

	for (;;) {
		struct becore_video_buffer *buf;
		void *vaddr;
		int ret;

		mutex_lock(&becore->lock);
		if (!becore->video_streaming) {
			mutex_unlock(&becore->lock);
			return;
		}
		mutex_unlock(&becore->lock);

		spin_lock_irq(&becore->queue_lock);
		if (list_empty(&becore->queued_outputs)) {
			spin_unlock_irq(&becore->queue_lock);
			return;
		}
		buf = list_first_entry(&becore->queued_outputs,
				       struct becore_video_buffer, list);
		list_del(&buf->list);
		spin_unlock_irq(&becore->queue_lock);

		vaddr = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);
		if (WARN_ON_ONCE(!vaddr)) {
			becore_video_fail(becore, buf);
			return;
		}
		ret = becore_run_frame(becore, BECORE_YUVP_OUTPUT_SBWCL,
				       true, vaddr, false, true);
		if (ret == -ENODATA || ret == -EBUSY) {
			spin_lock_irq(&becore->queue_lock);
			list_add(&buf->list, &becore->queued_outputs);
			spin_unlock_irq(&becore->queue_lock);
			return;
		}
		if (ret) {
			bool stopping;

			mutex_lock(&becore->lock);
			stopping = !becore->video_streaming;
			mutex_unlock(&becore->lock);
			if (ret == -ECANCELED && stopping) {
				vb2_buffer_done(&buf->vb.vb2_buf,
						VB2_BUF_STATE_ERROR);
				return;
			}
			becore_video_fail(becore, buf);
			return;
		}

		buf->vb.vb2_buf.timestamp = ktime_get_ns();
		mutex_lock(&becore->lock);
		buf->vb.sequence = becore->video_sequence++;
		mutex_unlock(&becore->lock);
		buf->vb.field = V4L2_FIELD_NONE;
		vb2_set_plane_payload(&buf->vb.vb2_buf, 0,
				      becore->active_capture_size);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	}
}

static int becore_queue_setup(struct vb2_queue *q, unsigned int *nbufs,
			      unsigned int *nplanes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct v4l2_pix_format pix;

	becore_video_fill_pix(&pix);
	if (*nplanes) {
		if (*nplanes != 1 || sizes[0] < pix.sizeimage)
			return -EINVAL;
		return 0;
	}

	*nplanes = 1;
	sizes[0] = pix.sizeimage;

	return 0;
}

static int becore_buf_prepare(struct vb2_buffer *vb)
{
	struct v4l2_pix_format pix;

	becore_video_fill_pix(&pix);
	if (vb2_plane_size(vb, 0) < pix.sizeimage ||
	    !vb2_plane_vaddr(vb, 0))
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, pix.sizeimage);

	return 0;
}

static void becore_buf_queue(struct vb2_buffer *vb)
{
	struct becore_device *becore = vb2_get_drv_priv(vb->vb2_queue);
	struct becore_video_buffer *buf =
		to_becore_video_buffer(to_vb2_v4l2_buffer(vb));

	spin_lock_irq(&becore->queue_lock);
	list_add_tail(&buf->list, &becore->queued_outputs);
	spin_unlock_irq(&becore->queue_lock);

	if (READ_ONCE(becore->video_streaming))
		schedule_work(&becore->video_work);
}

static int becore_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct becore_device *becore = vb2_get_drv_priv(q);
	struct exynos_becore_input_stream_config stream_config;
	struct exynos_becore_input *input;
	int ret;

	mutex_lock(&becore->lock);
	if (becore->video_streaming || becore->running) {
		ret = -EBUSY;
		goto unlock;
	}
	ret = becore_recipe_records_validate(becore, false);
	if (ret)
		goto unlock;
	ret = becore_mcsc_recipe_validate(becore);
	if (ret)
		goto unlock;
	if (becore->grid.staged_bytes != BECORE_GRID_SIZE) {
		ret = -EINVAL;
		goto unlock;
	}
	if (becore->reset_failed) {
		ret = -EIO;
		goto unlock;
	}
	input = becore_input_callback_get(becore);
	if (!input) {
		ret = -ENODEV;
		goto unlock;
	}

	becore->video_sequence = 0;
	/* The shared driver-owned output now becomes the video bounce buffer. */
	becore->completed_generation = 0;
	becore->completed_output_size = 0;
	becore_video_controls_snapshot(becore, &stream_config);
	becore->video_streaming = true;
	mutex_unlock(&becore->lock);

	ret = input->ops->start_streaming(input->producer_data, &stream_config);
	if (ret) {
		becore_input_callback_put(input);
		mutex_lock(&becore->lock);
		becore->video_streaming = false;
		mutex_unlock(&becore->lock);
		cancel_work_sync(&becore->video_work);
		becore_video_controls_ungrab(becore);
		becore_video_return_all(becore, VB2_BUF_STATE_QUEUED);
		return ret;
	}

	mutex_lock(&becore->lock);
	if (becore->input_producer == input && !input->disconnected &&
	    becore->video_streaming) {
		becore->producer_streaming = true;
	} else {
		ret = -ENODEV;
	}
	mutex_unlock(&becore->lock);
	if (ret)
		input->ops->stop_streaming(input->producer_data);
	becore_input_callback_put(input);
	if (ret) {
		cancel_work_sync(&becore->video_work);
		becore_video_controls_ungrab(becore);
		becore_video_return_all(becore, VB2_BUF_STATE_QUEUED);
		return ret;
	}
	schedule_work(&becore->video_work);

	return 0;

unlock:
	mutex_unlock(&becore->lock);
	becore_video_return_all(becore, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void becore_stop_streaming(struct vb2_queue *q)
{
	struct becore_device *becore = vb2_get_drv_priv(q);
	unsigned long flags;
	bool cancel = false;

	mutex_lock(&becore->lock);
	becore->video_streaming = false;
	spin_lock_irqsave(&becore->run_lock, flags);
	if (becore->running) {
		becore->abort_run = true;
		cancel = true;
	}
	spin_unlock_irqrestore(&becore->run_lock, flags);
	if (cancel)
		complete(&becore->run_completion);
	mutex_unlock(&becore->lock);

	cancel_work_sync(&becore->video_work);
	becore_video_stop_producer(becore);
	becore_video_controls_ungrab(becore);
	becore_video_return_all(becore, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops becore_vb2_ops = {
	.queue_setup = becore_queue_setup,
	.buf_prepare = becore_buf_prepare,
	.buf_queue = becore_buf_queue,
	.start_streaming = becore_start_streaming,
	.stop_streaming = becore_stop_streaming,
};

static int becore_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strscpy(cap->driver, "exynos-becore", sizeof(cap->driver));
	strscpy(cap->card, "zumapro BE-core MCSC NV21", sizeof(cap->card));

	return 0;
}

static int becore_enum_fmt(struct file *file, void *priv,
			   struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_NV21;

	return 0;
}

static int becore_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	becore_video_fill_pix(&f->fmt.pix);

	return 0;
}

static int becore_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct becore_device *becore = video_drvdata(file);

	if (vb2_is_busy(&becore->queue))
		return -EBUSY;

	becore_video_fill_pix(&f->fmt.pix);

	return 0;
}

static int becore_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || fsize->pixel_format != V4L2_PIX_FMT_NV21)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = becore_mcsc_output.width;
	fsize->discrete.height = becore_mcsc_output.height;

	return 0;
}

static const struct v4l2_ioctl_ops becore_ioctl_ops = {
	.vidioc_querycap = becore_querycap,
	.vidioc_enum_fmt_vid_cap = becore_enum_fmt,
	.vidioc_g_fmt_vid_cap = becore_g_fmt,
	.vidioc_s_fmt_vid_cap = becore_s_fmt,
	.vidioc_try_fmt_vid_cap = becore_g_fmt,
	.vidioc_enum_framesizes = becore_enum_framesizes,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations becore_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

static const struct video_device becore_video_template = {
	.name = "exynos-becore MCSC NV21 capture",
	.fops = &becore_fops,
	.ioctl_ops = &becore_ioctl_ops,
	.release = video_device_release_empty,
	.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING,
	.vfl_dir = VFL_DIR_RX,
};

static int becore_status_show(struct seq_file *s, void *unused)
{
	static const char * const input_state_names[] = {
		[BECORE_INPUT_FREE] = "free",
		[BECORE_INPUT_PRODUCER] = "producer",
		[BECORE_INPUT_READY] = "ready",
		[BECORE_INPUT_BACKEND] = "backend",
		[BECORE_INPUT_QUARANTINED] = "quarantined",
	};
	struct becore_device *becore = s->private;
	struct list_head *pos;
	unsigned long flags;
	unsigned int queued_outputs = 0;
	u32 cmdq_hold_mask;
	u32 frame_done_mask;
	u32 expected_mask;
	bool start_issued;
	bool irq_error;
	bool running;
	bool video_streaming;
	u32 i;

	mutex_lock(&becore->lock);
	spin_lock_irqsave(&becore->run_lock, flags);
	running = becore->running;
	cmdq_hold_mask = becore->cmdq_hold_mask;
	frame_done_mask = becore->frame_done_mask;
	expected_mask = becore->expected_mask;
	start_issued = becore->start_issued;
	irq_error = becore->irq_error;
	spin_unlock_irqrestore(&becore->run_lock, flags);
	spin_lock_irqsave(&becore->queue_lock, flags);
	list_for_each(pos, &becore->queued_outputs)
		queued_outputs++;
	spin_unlock_irqrestore(&becore->queue_lock, flags);
	video_streaming = becore->video_streaming;
	seq_printf(s, "running          %u\n", running);
	seq_printf(s,
		   "video_queue      streaming %u, producer %u, queued %u, sequence %u\n",
		   video_streaming, becore->producer_streaming, queued_outputs,
		   becore->video_sequence);
	seq_printf(s, "runtime          %s\n",
		   pm_runtime_status_suspended(becore->dev) ? "suspended" : "active");
	seq_printf(s, "recipe           %zu/%u bytes, generation %u\n",
		   becore->recipe_staged_bytes, BECORE_RECIPE_BYTES,
		   becore->recipe_generation);
	seq_printf(s, "gtnr_recipe      %zu/%u bytes, generation %u, encoded %u\n",
		   becore->gtnr_recipe_staged_bytes, BECORE_GTNR_RECIPE_BYTES,
		   becore->gtnr_recipe_generation,
		   becore->gtnr_encoded_generation);
	seq_printf(s, "mcsc_recipe      %zu/%u bytes, generation %u, encoded %u\n",
		   becore->mcsc_recipe_staged_bytes, BECORE_MCSC_RECIPE_BYTES,
		   becore->mcsc_recipe_generation,
		   becore->mcsc_encoded_generation);
	seq_printf(s, "mcsc_transport   %s\n",
		   becore->mcsc_encoded_transport == BECORE_MCSC_INPUT_MEMORY ?
		   "memory" : "captured-votf");
	seq_printf(s, "input            %zu/%zu bytes, iova %pad\n",
		   becore->inputs[0].buffer.staged_bytes,
		   becore->inputs[0].buffer.size, &becore->inputs[0].buffer.dma);
	seq_printf(s, "input_state      %s\n",
		   input_state_names[becore->inputs[0].state]);
	seq_puts(s, "input_slots      ");
	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_input_slot *slot = &becore->inputs[i];

		seq_printf(s, "%s%u:%s",
			   i ? " " : "", i, input_state_names[slot->state]);
		if (slot->state == BECORE_INPUT_PRODUCER)
			seq_printf(s, "#%llu", slot->producer_cookie);
		else if (slot->state == BECORE_INPUT_READY)
			seq_printf(s, "@%llu", slot->ready_sequence);
	}
	seq_putc(s, '\n');
	if (becore->input_producer) {
		seq_printf(s, "input_producer   %s iovas",
			   dev_name(becore->input_producer->producer));
		for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++)
			seq_printf(s, " %u:%pad", i,
				   &becore->input_producer->dmas[i]);
		seq_putc(s, '\n');
	}
	seq_printf(s, "grid             %zu/%zu bytes, generation %u, iova %pad\n",
		   becore->grid.staged_bytes, becore->grid.size,
		   becore->grid_generation,
		   &becore->grid.dma);
	seq_printf(s, "output           %zu active/%zu completed/%zu allocated bytes, iova %pad\n",
		   becore->active_output_size, becore->completed_output_size,
		   becore->output.size,
		   &becore->output.dma);
	seq_printf(s, "capture_size     %zu bytes\n",
		   becore->active_capture_size);
	seq_printf(s, "output_profile   %u requested, %u active\n",
		   READ_ONCE(becore->output_profile),
		   becore->active_output_profile);
	seq_printf(s, "active_path      %s\n",
		   becore->active_mcsc ? "YUVP-memory-to-MCSC" : "YUVP");
	seq_printf(s, "rgbp_cmdq        %zu bytes/%u headers, iova %pad\n",
		   becore->program[BECORE_RGBP].size,
		   becore->program[BECORE_RGBP].header_count,
		   &becore->program[BECORE_RGBP].dma);
	seq_printf(s, "yuvp_cmdq        %zu bytes/%u headers, iova %pad\n",
		   becore->program[BECORE_YUVP].size,
		   becore->program[BECORE_YUVP].header_count,
		   &becore->program[BECORE_YUVP].dma);
	seq_printf(s, "gtnr_output      %zu bytes, iova %pad\n",
		   becore->gtnr_output.size, &becore->gtnr_output.dma);
	seq_printf(s, "gtnr_cmdq        %zu bytes/%u headers, iova %pad\n",
		   becore->gtnr_program.size,
		   becore->gtnr_program.header_count,
		   &becore->gtnr_program.dma);
	seq_printf(s, "mcsc_output      %u completed/%zu allocated bytes, iova %pad\n",
		   becore->mcsc_completed_output_size, becore->mcsc_output.size,
		   &becore->mcsc_output.dma);
	seq_printf(s, "mcsc_cmdq        %zu bytes/%u headers, iova %pad\n",
		   becore->mcsc_program.size,
		   becore->mcsc_program.header_count,
		   &becore->mcsc_program.dma);
	seq_printf(s, "run_generation   %u\n", becore->run_generation);
	seq_printf(s, "completed         %u\n", becore->completed_generation);
	seq_printf(s, "last_result       %d\n", becore->last_run_result);
	seq_printf(s, "cmdq_hold         %#x\n", cmdq_hold_mask);
	seq_printf(s, "frame_done        %#x\n", frame_done_mask);
	seq_printf(s, "expected          %#x\n", expected_mask);
	seq_printf(s, "start_issued      %u\n", start_issued);
	seq_printf(s, "irq_error         %u\n", irq_error);
	seq_printf(s, "reset_failed      %u\n", becore->reset_failed);
	seq_printf(s, "output_quarantined %u\n", becore->output_quarantined);
	seq_printf(s, "output_changed    %u\n", becore->output_changed_bytes);
	seq_printf(s, "output_first      %#x\n", becore->output_first_changed);
	seq_printf(s, "mcsc_completed    %u\n",
		   becore->mcsc_completed_generation);
	seq_printf(s, "mcsc_changed      %u\n",
		   becore->mcsc_output_changed_bytes);
	seq_printf(s, "mcsc_first        %#x\n",
		   becore->mcsc_output_first_changed);
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		seq_printf(s,
			   "%-4s irq0 %lld last %#010x cmdq %#010x; irq1 %lld last %#010x\n",
			   becore->blocks[i].name,
			   atomic64_read(&becore->blocks[i].int0_count),
			   READ_ONCE(becore->blocks[i].last_int0),
			   READ_ONCE(becore->blocks[i].last_cmdq_int),
			   atomic64_read(&becore->blocks[i].int1_count),
			   READ_ONCE(becore->blocks[i].last_int1));
	mutex_unlock(&becore->lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(becore_status);

static void becore_video_unregister(void *data)
{
	struct becore_device *becore = data;

	vb2_video_unregister_device(&becore->vdev);
	media_device_unregister(&becore->mdev);
	media_entity_cleanup(&becore->vdev.entity);
	v4l2_ctrl_handler_free(&becore->ctrl_handler);
	v4l2_device_unregister(&becore->v4l2_dev);
	media_device_cleanup(&becore->mdev);
}

static int becore_video_register(struct becore_device *becore)
{
	struct v4l2_ctrl_handler *handler = &becore->ctrl_handler;
	struct vb2_queue *q = &becore->queue;
	int ret;

	becore->mdev.dev = becore->dev;
	strscpy(becore->mdev.model, "zumapro BE-core",
		sizeof(becore->mdev.model));
	media_device_init(&becore->mdev);
	becore->v4l2_dev.mdev = &becore->mdev;

	ret = v4l2_device_register(becore->dev, &becore->v4l2_dev);
	if (ret)
		goto err_mdev;

	ret = v4l2_ctrl_handler_init(handler, 2);
	if (ret)
		goto err_v4l2;
	becore->red_balance =
		v4l2_ctrl_new_std(handler, NULL, V4L2_CID_RED_BALANCE,
				  EXYNOS_BECORE_WBG_GAIN_MIN_Q12,
				  EXYNOS_BECORE_WBG_GAIN_MAX_Q12, 1,
				  EXYNOS_BECORE_WBG_RED_DEFAULT_Q12);
	becore->blue_balance =
		v4l2_ctrl_new_std(handler, NULL, V4L2_CID_BLUE_BALANCE,
				  EXYNOS_BECORE_WBG_GAIN_MIN_Q12,
				  EXYNOS_BECORE_WBG_GAIN_MAX_Q12, 1,
				  EXYNOS_BECORE_WBG_BLUE_DEFAULT_Q12);
	if (handler->error) {
		ret = handler->error;
		goto err_ctrl;
	}

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP;
	q->dev = becore->dev;
	q->drv_priv = becore;
	q->ops = &becore_vb2_ops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->buf_struct_size = sizeof(struct becore_video_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers = 1;
	q->lock = &becore->video_lock;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_ctrl;

	becore->vdev = becore_video_template;
	becore->vdev.v4l2_dev = &becore->v4l2_dev;
	becore->vdev.ctrl_handler = &becore->ctrl_handler;
	becore->vdev.queue = q;
	becore->vdev.lock = &becore->video_lock;
	becore->vdev.entity.function = MEDIA_ENT_F_IO_V4L;
	video_set_drvdata(&becore->vdev, becore);

	becore->vdev_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&becore->vdev.entity, 1,
				     &becore->vdev_pad);
	if (ret)
		goto err_ctrl;

	ret = video_register_device(&becore->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_entity;

	ret = media_device_register(&becore->mdev);
	if (ret)
		goto err_vdev;

	return devm_add_action_or_reset(becore->dev,
					becore_video_unregister, becore);

err_vdev:
	video_unregister_device(&becore->vdev);
err_entity:
	media_entity_cleanup(&becore->vdev.entity);
err_ctrl:
	v4l2_ctrl_handler_free(&becore->ctrl_handler);
err_v4l2:
	v4l2_device_unregister(&becore->v4l2_dev);
err_mdev:
	media_device_cleanup(&becore->mdev);
	return ret;
}

static void becore_debugfs_remove(void *data)
{
	struct becore_device *becore = data;

	debugfs_remove_recursive(becore->debugfs);
}

static int becore_debugfs_init(struct becore_device *becore)
{
	struct dentry *dir;

	dir = debugfs_create_dir(dev_name(becore->dev), NULL);
	if (IS_ERR(dir))
		return PTR_ERR(dir);
	becore->debugfs = dir;
	debugfs_create_file("program", 0600, dir, becore, &becore_recipe_fops);
	debugfs_create_file("gtnr_program", 0600, dir, becore,
			    &becore_gtnr_recipe_fops);
	debugfs_create_file("mcsc_program", 0600, dir, becore,
			    &becore_mcsc_recipe_fops);
	debugfs_create_file("input", 0200, dir, becore, &becore_input_fops);
	debugfs_create_file("grid", 0200, dir, becore, &becore_grid_fops);
	debugfs_create_u32("output_profile", 0644, dir,
			   &becore->output_profile);
	debugfs_create_file("output", 0400, dir, becore, &becore_output_fops);
	debugfs_create_file("mcsc_output", 0400, dir, becore,
			    &becore_mcsc_output_fops);
	debugfs_create_file("rgbp_cmdq", 0400, dir, becore,
			    &becore_rgbp_encoded_fops);
	debugfs_create_file("yuvp_cmdq", 0400, dir, becore,
			    &becore_yuvp_encoded_fops);
	debugfs_create_file("gtnr_cmdq", 0400, dir, becore,
			    &becore_gtnr_encoded_fops);
	debugfs_create_file("gtnr_encode", 0200, dir, becore,
			    &becore_gtnr_encode_fops);
	debugfs_create_file("mcsc_cmdq", 0400, dir, becore,
			    &becore_mcsc_encoded_fops);
	debugfs_create_file("mcsc_encode", 0200, dir, becore,
			    &becore_mcsc_encode_fops);
	debugfs_create_file("run", 0600, dir, becore, &becore_run_fops);
	debugfs_create_file("mcsc_run", 0200, dir, becore,
			    &becore_mcsc_run_fops);
	debugfs_create_file("cancel", 0200, dir, becore, &becore_cancel_fops);
	debugfs_create_file("status", 0400, dir, becore, &becore_status_fops);

	return devm_add_action_or_reset(becore->dev, becore_debugfs_remove,
					becore);
}

static int becore_probe(struct platform_device *pdev)
{
	static const struct dev_pm_domain_attach_data pm_domain_data = {
		.pd_names = becore_pm_domain_names,
		.num_pd_names = ARRAY_SIZE(becore_pm_domain_names),
	};
	struct device *dev = &pdev->dev;
	struct becore_device *becore;
	int ret;

	becore = devm_kzalloc(dev, sizeof(*becore), GFP_KERNEL);
	if (!becore)
		return -ENOMEM;

	becore->dev = dev;
	mutex_init(&becore->lock);
	mutex_init(&becore->video_lock);
	spin_lock_init(&becore->run_lock);
	spin_lock_init(&becore->queue_lock);
	init_completion(&becore->run_completion);
	INIT_LIST_HEAD(&becore->queued_outputs);
	INIT_WORK(&becore->video_work, becore_video_work);
	becore->output_first_changed = U32_MAX;
	becore->mcsc_output_first_changed = U32_MAX;
	becore->active_output_profile = BECORE_YUVP_OUTPUT_SBWCL;
	becore->active_output_size = becore_active_output_size(becore);
	becore->active_capture_size = becore_mcsc_output_active_size();
	becore->blocks[BECORE_RGBP] = (struct becore_block) {
		.becore = becore,
		.name = "RGBP",
		.int0_mask_prepare = 0x18e1fc02,
		.int0_mask = 0x18e1fc06,
		.int1_mask = 0x7fff,
		.cmdq_int_mask = 0x7,
	};
	becore->blocks[BECORE_MCFP] = (struct becore_block) {
		.becore = becore,
		.name = "MCFP",
		.int0_mask_prepare = 0x3ffffc02,
		.int0_mask = 0x3ffffc06,
		.cmdq_int_mask = 0x7,
	};
	becore->blocks[BECORE_YUVP] = (struct becore_block) {
		.becore = becore,
		.name = "YUVP",
		.int0_mask_prepare = 0x3fe1fc02,
		.int0_mask = 0x3fe1fc06,
		.int1_mask = 0x1ffffff,
		.cmdq_int_mask = 0xff,
	};
	becore->blocks[BECORE_MCSC] = (struct becore_block) {
		.becore = becore,
		.name = "MCSC",
		.int0_mask_prepare = 0x3ffffc02,
		.int0_mask = 0x3ffffc06,
		.int1_mask = 0x501,
		.cmdq_int_mask = 0x1,
	};
	platform_set_drvdata(pdev, becore);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "no 32-bit DMA\n");

	ret = becore_map_resources(pdev, becore);
	if (ret)
		return ret;

	ret = devm_pm_domain_attach_list(dev, &pm_domain_data,
					 &becore->pm_domains);
	if (ret < 0)
		return dev_err_probe(dev, ret, "cannot attach power domains\n");
	if (ret != (int)ARRAY_SIZE(becore_pm_domain_names))
		return dev_err_probe(dev, -ENODEV,
				     "attached %d of %zu power domains\n", ret,
				     ARRAY_SIZE(becore_pm_domain_names));

	ret = becore_request_irqs(pdev, becore);
	if (ret)
		return ret;
	ret = becore_alloc_diagnostic(becore);
	if (ret)
		return ret;
	becore->active_output_dma = becore->output.dma;

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot power camera back end\n");
	if (becore->reset_failed) {
		dev_crit(dev, "processor reset failed; retaining power\n");
		return 0;
	}

	ret = pm_runtime_put_sync(dev);
	if (ret < 0) {
		/* Keep all suppliers active when the reset vetoes power-down. */
		pm_runtime_get_noresume(dev);
		dev_crit(dev, "cannot quiesce camera back end; retaining power\n");
		return 0;
	}

	ret = becore_video_register(becore);
	if (ret)
		return ret;

	return becore_debugfs_init(becore);
}

static const struct dev_pm_ops becore_pm_ops = {
	SET_RUNTIME_PM_OPS(becore_runtime_suspend, becore_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static const struct of_device_id becore_of_match[] = {
	{ .compatible = "google,zumapro-becore" },
	{ }
};
MODULE_DEVICE_TABLE(of, becore_of_match);

static struct platform_driver becore_driver = {
	.probe = becore_probe,
	.driver = {
		.name = "exynos-becore",
		.of_match_table = becore_of_match,
		.pm = pm_ptr(&becore_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(becore_driver);

MODULE_DESCRIPTION("Google Zumapro camera back-end core");
MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_LICENSE("GPL");
