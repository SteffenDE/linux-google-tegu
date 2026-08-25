// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Zumapro camera back-end core -- the words the driver states itself
 *
 * A captured program carries zero for every register the vendor's own encoder
 * computed, so the driver computes them again: 348 words for RGBP, 1365 for
 * YUVP and 116 for MCSC.  The three tables here divide each block's register
 * space into ranges and give every range a kind, and becore_generated_value()
 * dispatches on that kind to the function that derives the word -- from the
 * rasters, from a profile read off the silicon, or from a curve this driver
 * evaluates.
 *
 * Nothing here reads a parameters buffer.  A generated word is what a stream
 * with no buffer runs on; where a register can carry tuning as well,
 * exynos-becore-params.c substitutes over this later in the same record.
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/dev_printk.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/types.h>

#include "exynos-becore-regs.h"
#include "exynos-becore-common.h"
#include "exynos-becore-recipe.h"
#include "exynos-becore-gtnr-recipe.h"
#include "exynos-becore-mcsc-recipe.h"
#include "exynos-becore-sharpen.h"
#include "exynos-becore-yuvnr.h"
#include "exynos-becore-byrdns.h"
#include "exynos-becore-dmsc.h"
/*
 * How the block is told to read the affine grid's slope and bias fields.
 * TranslateLtm takes the largest slope and the largest bias over the grid it
 * is about to write, raises both to at least 1.0, and then:
 *
 *   frac_bit    = ((int)slope) >> 14 ? 1 : (clz32((int)slope) - 1) & 0xf
 *   bias_adjust = (int)bias < 16 ? ~min(clz32((int)bias), 31) & 3 : 3
 *
 * becore_ltm_grid_generate() writes unity gain and zero bias in every cell, so
 * for this driver's own grid they are 14 and 0.
 *
 * That is a derivation and not a replay, and the captures show why it has to
 * be: 401 of the 426 hold 14, and the other 25 are frames whose grid ran past
 * unity gain. A driver that copied one program's word would be wrong the
 * moment its grid stopped being neutral -- which is the thing to remember if
 * a parameters block ever carries a grid: these two are constants only for as
 * long as the grid beside them is, and nothing couples them automatically.
 */
#define BECORE_LTM_TRANS_SLOPE_FRAC_BIT	14
#define BECORE_LTM_TRANS_BIAS_BIT_ADJUST 0

/*
 * The block's second curve: the tone adjustment the local map is applied
 * through, and the one register file here whose words the driver carries
 * rather than computes.
 *
 * It is the shipped apcamera.SCurve -- midpoint 0.1864, slope_midpoint 1.158,
 * relative_highlight_compression 1.0, relative_shadow_crushing 0.95 -- sampled
 * at x = i / 128 and quantised the way TranslateLtmToneAdjust does, as
 * clamp(round(y * 16384), 0, 16384).
 *
 * The evaluator those four numbers drive is three control points and a
 * spline. The midpoint is a fixed point, so the curve passes through (m, m)
 * at the tuning's slope, and both ends are pinned:
 *
 *	(0,	  0,	    (1 / slope_midpoint) * (1 - relative_shadow_crushing))
 *	(midpoint, midpoint, slope_midpoint)
 *	(1,	  1,	    1 + (white - 1) * relative_highlight_compression)
 *
 * where white is the slope that a logarithmic compression of the upper span,
 * ln(1 + k * x) / ln(1 + k), leaves at its end: that span's own chord slope
 * times k / ((k + 1) * ln(k + 1)), the chord being 1 here only because both
 * of its ends sit on y = x. Between two control points the interpolant is
 * Stineman's, which the vendor's own file name says -- with L the chord and
 * T0, T1 the endpoint tangents, A = T0 - L, B = T1 - L, and
 * y = L + A * B / (A + B).
 *
 * It stays a table for two reasons. The arithmetic is float32 throughout,
 * including a logf and a degree-5 polynomial for k whose error the shipped
 * curve carries; and there is no per-frame input to it -- the tuning is
 * byte-identical on all three cameras, and the 129 words are bit-identical in
 * all 426 captured programs and at every one of the eighteen captured
 * readouts, so this is one fixed vendor profile rather than a per-lens
 * calibration or a per-frame policy. tools/camera-ltm-scurve.py is the
 * evaluator, and its --check regenerates exactly this table.
 */
static const u16 becore_ltm_toneadj[BECORE_LTM_TONEADJ_ENTRIES] = {
	    0,    31,    98,   188,   291,   405,   526,   652,
	  782,   916,  1052,  1190,  1330,  1471,  1613,  1757,
	 1901,  2046,  2192,  2338,  2484,  2631,  2779,  2927,
	 3075,  3223,  3370,  3517,  3664,  3810,  3956,  4102,
	 4247,  4391,  4536,  4679,  4823,  4966,  5108,  5251,
	 5392,  5534,  5675,  5815,  5955,  6095,  6235,  6374,
	 6512,  6650,  6788,  6925,  7062,  7199,  7335,  7471,
	 7606,  7741,  7876,  8010,  8144,  8278,  8411,  8543,
	 8676,  8808,  8939,  9070,  9201,  9331,  9462,  9591,
	 9720,  9849,  9978, 10106, 10234, 10361, 10488, 10615,
	10741, 10867, 10993, 11118, 11243, 11367, 11491, 11615,
	11738, 11861, 11984, 12106, 12228, 12350, 12471, 12592,
	12712, 12832, 12952, 13072, 13191, 13309, 13428, 13546,
	13663, 13781, 13898, 14014, 14131, 14246, 14362, 14477,
	14592, 14707, 14821, 14935, 15048, 15161, 15274, 15387,
	15499, 15611, 15722, 15833, 15944, 16055, 16165, 16275,
	16384,
};

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
 * Three RGBP blocks and seven YUVP registers whose values are named constants
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
 * is, and the input profile's raster is the full array. A crop that was *not*
 * centred would need the offset term as well, so this is a statement about the
 * current geometry rather than a general derivation. The noise curve above it
 * is real tuning and stays.
 *
 * SHARPENHANCER's LPF_NORM is log2 of its three low-pass kernels' sums packed
 * at bits 0, 8 and 16 -- 16, 512 and 4096. Those sums used to be constants
 * here, and this paragraph used to say so; the taps left the recipe when the
 * block's non-tuning tail was stated, so becore_yuvp_lpf_norm_value() now adds
 * up becore_sharpen_kernels[] and refuses a kernel that does not sum to a
 * power of two. The test is a property of the kernels rather than of three
 * numbers standing beside them.
 *
 * SHARPENHANCER's ten noise seeds are not tuning either, and they are the one
 * thing in the block that a capture cannot be replayed for: the vendor redraws
 * them every frame, so the recipe was carrying one frame's dice. They are the
 * same kind of object as MCSC's DJAG LFSR seeds, which this driver already
 * writes as constants.
 *
 * The value is the vendor's own. lyric::TranslateYuvSharpEnhancer writes these
 * seeds as immediates -- 11111 times 1 to 5, with the second generator's five
 * starting one step along -- and that is what reaches the hardware whenever
 * nothing has redrawn them yet: five of the 426 captured programs carry
 * exactly this tuple. So this is Lyric's compiled-in seed and not a constant
 * of our choosing.
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
 * curve for luma and one for chroma. The knots are genuine tuning, and the
 * demosaicer's stay in the recipe; the noise reducer's are a parameters block
 * away, so the driver generates those from the block or, with none in force,
 * from the vendor's own default curve. What follows from the knots either way
 * is the eight slopes, the shift they are taken at, and -- because the chroma
 * curve is measured on the same domain as the luma one -- the chroma domain
 * itself.
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
	BECORE_GEN_YUV2RGB,	/* YUV to RGB: the BT.601 inverse at Q12 */
	BECORE_GEN_CSC420,	/* RGB to YUV 4:2:0: BT.601 at Q14, 10-bit out */
	BECORE_GEN_DITHER420,	/* the dither a 10-bit output needs */
	BECORE_GEN_CHROMA_LPF,	/* 4:4:4 to 4:2:2, a fixed binomial filter */
	BECORE_GEN_DJAG,	/* MCSC DJAG at Samsung's neutral profile */
	BECORE_GEN_DMSC,	/* what GetDefaultDmsc writes after the tuning */
	BECORE_GEN_DNS_GEOMETRY,	/* binning and radial centre, from the array */
	BECORE_GEN_DNS_BIQUAD,	/* the biquad filter's resolution octave */
	BECORE_GEN_BYR_DNS,	/* the Bayer denoiser bypassed, its tuning at zero */
	BECORE_GEN_BYR_DMSC,	/* the demosaic bypassed, its tuning at zero */
	BECORE_GEN_GAMMA,	/* RGBP's forward gamma, a square-root encode */
	BECORE_GEN_YUVP_GAMMA,	/* YUVP's tone-curve gates and its x grid */
	BECORE_GEN_YUVP_DEGAMMA,	/* the inverse of RGBP's encode */
	BECORE_GEN_SHARPEN_DEFAULT,	/* what GetDefaultYuvSharpEnhancer writes */
	BECORE_GEN_NR_DEFAULT,	/* YUVNR's fixed output: GetDefaultYuvNr's */
	BECORE_GEN_YUVNR,	/* the noise reducer bypassed, on its default */
	BECORE_GEN_NR_LUMA_GRID,	/* its luma-gain curve's knot grid */
	BECORE_GEN_NR_GEOMETRY,	/* its radial pair, from the crop and chain */
	BECORE_GEN_GRID_DMA,	/* the LTM grid RDMA, from our own buffer */
	BECORE_GEN_YUVP_CHAIN_SIZE,	/* the raster YUVP is handed */
	BECORE_GEN_SHARPEN,	/* the sharpener bypassed, and its tuning at zero */
	BECORE_GEN_UNWRITTEN,	/* a register of a block we program that no
				 * vendor code writes at all: its reset value
				 */
	BECORE_GEN_SHARPEN_GEOMETRY,	/* the sharpener's crop and its ratio */
	BECORE_GEN_BAYER_PHASE,	/* the mosaic the producer negotiated */
	BECORE_GEN_LPF,		/* the sharpener's three low-pass kernels */
	BECORE_GEN_LPF_NORM,	/* log2 of the sharpener's three kernel sums */
	BECORE_GEN_NOISE_SEED,	/* the sharpener noise generator's ten seeds */
	BECORE_GEN_SCENE_INPUT,	/* an input nothing in mainline produces */
	BECORE_GEN_NOISE_SLOPE,	/* a noise curve's slopes, from its own knots */
	BECORE_GEN_NOISE_SHIFT,	/* the shift those slopes are taken at */
	BECORE_GEN_NOISE_DOMAIN,	/* a chroma domain repeating the luma one */
	BECORE_GEN_SCALER_PHASE,	/* a scaler starting on a pixel, rounding */
	BECORE_GEN_MCSC_INPUT_SIZE,	/* the raster MCSC reads, from YUVP */
	BECORE_GEN_GTM,		/* RGBP's tone map, an identity */
	BECORE_GEN_LTM,		/* YUVP's tone mapping, all of it bar the guide
				 * curve: gates, luma, grid, the fields that
				 * describe it, identity fills, tone-adjust
				 */
	BECORE_GEN_CLUT_1DLUT,	/* the colour LUT's per-channel input identity */
	BECORE_GEN_CLUT,	/* the colour LUT's gate and its YUV-to-RGB matrix */
	BECORE_GEN_INVCCM33,	/* the inverse colour matrix, an exact identity */
	BECORE_GEN_CCM,		/* the colour matrix's gate, identity and offsets */
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
	{ BECORE_RGBP_DTP_MODE_REG, BECORE_RGBP_DTP_MODE_REG,
	  BECORE_GEN_OFF },
	{ BECORE_RGBP_DMSC_OP_MODE_REG, BECORE_RGBP_DMSC_OP_MODE_REG,
	  BECORE_GEN_OFF },
	BECORE_BYR_DNS_TUNING_RANGES
	BECORE_DMSC_TUNING_RANGES
	{ BECORE_RGBP_SC_CTRL0_REG, BECORE_RGBP_SC_CTRL0_REG,
	  BECORE_GEN_OFF },
	{ BECORE_RGBP_DNS_PHASE_REG, BECORE_RGBP_DNS_PHASE_REG,
	  BECORE_GEN_BAYER_PHASE },
	{ BECORE_RGBP_DMSC_PHASE_REG, BECORE_RGBP_DMSC_PHASE_REG,
	  BECORE_GEN_BAYER_PHASE },
	{ BECORE_RGBP_DNS_X_G_REG,
	  BECORE_RGBP_DNS_X_G_REG + BECORE_NOISE_TABLE_LAST,
	  BECORE_GEN_NOISE_DOMAIN },
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
	/*
	 * The whole of what GetDefaultDmsc deposits, which is 23 registers
	 * rather than the fourteen this used to state.  The other nine are the
	 * same kind of literal read out of the same function; the conservative
	 * subset was a bring-up state, not a distinction the vendor makes.
	 */
	{ BECORE_RGBP_DMSC_BASE + 0x20c, BECORE_RGBP_DMSC_BASE + 0x218,
	  BECORE_GEN_DMSC },			/* BASE_CONFIG, PRE_DEMOSAICING */
	{ BECORE_RGBP_DMSC_BASE + 0x220, BECORE_RGBP_DMSC_BASE + 0x224,
	  BECORE_GEN_DMSC },			/* IGNORE_RB, _RB_1 */
	{ BECORE_RGBP_DMSC_BASE + 0x22c, BECORE_RGBP_DMSC_BASE + 0x230,
	  BECORE_GEN_DMSC },			/* DIR_DETECT_HV_LINEAR,
						 * CONTRAST_MINIMUM
						 */
	{ BECORE_RGBP_DMSC_BASE + 0x238, BECORE_RGBP_DMSC_BASE + 0x238,
	  BECORE_GEN_DMSC },			/* EXTRACT_COLORS_CONFIG */
	{ BECORE_RGBP_DMSC_BASE + 0x24c, BECORE_RGBP_DMSC_BASE + 0x258,
	  BECORE_GEN_DMSC },			/* GREEN_HUE, GREEN_SAT,
						 * GREEN_SELECTIVITY,
						 * POST_PROCESS_CONFIG
						 */
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
	{ BECORE_RGBP_DMSC_BASE + 0x2a8, BECORE_RGBP_DMSC_BASE + 0x2b0,
	  BECORE_GEN_DMSC },			/* RED_PRESERVE_THRES, _LIMIT,
						 * DESAT_LUMA_GAIN
						 */
	{ BECORE_RGBP_DMSC_BASE + 0x2b4, BECORE_RGBP_DMSC_BASE + 0x2b4,
	  BECORE_GEN_DMSC },			/* ADD_YBLUR */
	{ BECORE_RGBP_DNS_BINNING_REG, BECORE_RGBP_DNS_BINNING_REG,
	  BECORE_GEN_DNS_GEOMETRY },
	{ BECORE_RGBP_DNS_CENTRE_REG, BECORE_RGBP_DNS_CENTRE_REG,
	  BECORE_GEN_DNS_GEOMETRY },
	{ BECORE_RGBP_DNS_BIQUAD_REG, BECORE_RGBP_DNS_BIQUAD_REG,
	  BECORE_GEN_DNS_BIQUAD },
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
	{ BECORE_YUVP_CHAIN_IMG_SIZE_REG, BECORE_YUVP_CHAIN_IMG_SIZE_REG,
	  BECORE_GEN_YUVP_CHAIN_SIZE },
	{ BECORE_YUVP_GRID_DMA_EN_REG, BECORE_YUVP_GRID_DMA_EN_REG,
	  BECORE_GEN_GRID_DMA },
	{ BECORE_YUVP_GRID_DMA_FORMAT_REG, BECORE_YUVP_GRID_DMA_FORMAT_REG,
	  BECORE_GEN_GRID_DMA },
	{ BECORE_YUVP_GRID_DMA_WIDTH_REG, BECORE_YUVP_GRID_DMA_STRIDE_REG,
	  BECORE_GEN_GRID_DMA },
	{ BECORE_YUVP_GRID_DMA_BUSINFO_REG, BECORE_YUVP_GRID_DMA_BUSINFO_REG,
	  BECORE_GEN_GRID_DMA },
	{ BECORE_YUVP_COUTFIFO_GSE_EN_REG, BECORE_YUVP_COUTFIFO_GSE_EN_REG,
	  BECORE_GEN_OFF },
	{ BECORE_YUVP_COUTFIFO_MCSC_EN_REG, BECORE_YUVP_COUTFIFO_MCSC_EN_REG,
	  BECORE_GEN_OFF },
	{ BECORE_YUVP_DTP_BYPASS_REG, BECORE_YUVP_DTP_BYPASS_REG,
	  BECORE_GEN_BYPASS },
	{ BECORE_YUVP_YUV2RGB_BYPASS_REG, BECORE_YUVP_YUV2RGB_BYPASS_REG,
	  BECORE_GEN_YUV2RGB },
	{ BECORE_YUVP_YUV2RGB_FIRST, BECORE_YUVP_YUV2RGB_LAST,
	  BECORE_GEN_YUV2RGB },
	{ BECORE_YUVP_CSC_FIRST, BECORE_YUVP_CSC_LAST, BECORE_GEN_CSC },
	{ BECORE_YUVP_DITHER420_FIRST, BECORE_YUVP_DITHER420_LAST,
	  BECORE_GEN_DITHER420 },
	{ BECORE_YUVP_YUV2RGB_ZUMA_BYPASS_REG,
	  BECORE_YUVP_YUV2RGB_ZUMA_BYPASS_REG, BECORE_GEN_YUV2RGB },
	{ BECORE_YUVP_YUV2RGB_ZUMA_FIRST, BECORE_YUVP_YUV2RGB_ZUMA_LAST,
	  BECORE_GEN_YUV2RGB },
	{ BECORE_YUVP_CSC420_FIRST, BECORE_YUVP_CSC420_LAST,
	  BECORE_GEN_CSC420 },
	{ BECORE_YUVP_CSC420_VER_SAMPLING_REG,
	  BECORE_YUVP_CSC420_VER_SAMPLING_REG, BECORE_GEN_CSC420 },
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
	{ BECORE_YUVP_NR_LUMA_GRID_FIRST, BECORE_YUVP_NR_LUMA_GRID_LAST,
	  BECORE_GEN_NR_LUMA_GRID },
	{ BECORE_YUVP_PHYS_BASE + 0x3208, BECORE_YUVP_PHYS_BASE + 0x3208,
	  BECORE_GEN_NR_DEFAULT },	/* low_power_en */
	{ BECORE_YUVP_PHYS_BASE + 0x32d0, BECORE_YUVP_PHYS_BASE + 0x32d0,
	  BECORE_GEN_NR_DEFAULT },	/* radthrs_radial_config */
	{ BECORE_YUVP_PHYS_BASE + 0x3314, BECORE_YUVP_PHYS_BASE + 0x3314,
	  BECORE_GEN_NR_DEFAULT },	/* center_weight */
	{ BECORE_YUVP_PHYS_BASE + 0x3320, BECORE_YUVP_PHYS_BASE + 0x3320,
	  BECORE_GEN_NR_DEFAULT },	/* radial */
	{ BECORE_YUVP_PHYS_BASE + 0x334c, BECORE_YUVP_PHYS_BASE + 0x334c,
	  BECORE_GEN_NR_DEFAULT },	/* radial_thresh_limit */
	{ BECORE_YUVP_PHYS_BASE + 0x3398, BECORE_YUVP_PHYS_BASE + 0x3398,
	  BECORE_GEN_NR_DEFAULT },	/* contents_aware */
	{ BECORE_YUVP_PHYS_BASE + 0x33a4, BECORE_YUVP_PHYS_BASE + 0x33a4,
	  BECORE_GEN_NR_DEFAULT },	/* confmap_seg_0 */
	{ BECORE_YUVP_PHYS_BASE + 0x33ac, BECORE_YUVP_PHYS_BASE + 0x33ac,
	  BECORE_GEN_NR_DEFAULT },	/* confmap_seg_1 */
	{ BECORE_YUVP_PHYS_BASE + 0x33b4, BECORE_YUVP_PHYS_BASE + 0x33b4,
	  BECORE_GEN_NR_DEFAULT },	/* confmap_seg_2 */
	{ BECORE_YUVP_PHYS_BASE + 0x33bc, BECORE_YUVP_PHYS_BASE + 0x33bc,
	  BECORE_GEN_NR_DEFAULT },	/* confmap_seg_3 */
	{ BECORE_YUVP_PHYS_BASE + 0x33c4, BECORE_YUVP_PHYS_BASE + 0x33c4,
	  BECORE_GEN_NR_DEFAULT },	/* confmap_seg_4 */
	{ BECORE_YUVP_PHYS_BASE + 0x33cc, BECORE_YUVP_PHYS_BASE + 0x33cc,
	  BECORE_GEN_NR_DEFAULT },	/* confmap_seg_5 */
	{ BECORE_YUVP_PHYS_BASE + 0x33d4, BECORE_YUVP_PHYS_BASE + 0x33d4,
	  BECORE_GEN_NR_DEFAULT },	/* confmap_seg_6 */
	{ BECORE_YUVP_PHYS_BASE + 0x344c, BECORE_YUVP_PHYS_BASE + 0x3458,
	  BECORE_GEN_NR_DEFAULT },	/* wavelet_edge_map..nlm_map_suppression_uv */
	{ BECORE_YUVP_PHYS_BASE + 0x3500, BECORE_YUVP_PHYS_BASE + 0x3500,
	  BECORE_GEN_NR_DEFAULT },	/* tuning_param1 */
	{ BECORE_YUVP_PHYS_BASE + 0x3508, BECORE_YUVP_PHYS_BASE + 0x3508,
	  BECORE_GEN_NR_DEFAULT },	/* tuning_param3 */
	{ BECORE_YUVP_PHYS_BASE + 0x35dc, BECORE_YUVP_PHYS_BASE + 0x35dc,
	  BECORE_GEN_NR_DEFAULT },	/* wide_edge */
	{ BECORE_YUVP_PHYS_BASE + 0x3610, BECORE_YUVP_PHYS_BASE + 0x3610,
	  BECORE_GEN_NR_DEFAULT },	/* h_nr_en */
	{ BECORE_YUVP_PHYS_BASE + 0x3784, BECORE_YUVP_PHYS_BASE + 0x3788,
	  BECORE_GEN_NR_DEFAULT },	/* filterweights_param3..filterweights_param4 */
	/*
	 * The block's radial geometry, which no parameters block could carry
	 * because it is the frame's shape rather than its tuning.
	 */
	{ BECORE_YUVP_PHYS_BASE + BECORE_YUVNR_BINNING,
	  BECORE_YUVP_PHYS_BASE + BECORE_YUVNR_BINNING,
	  BECORE_GEN_NR_GEOMETRY },
	{ BECORE_YUVP_PHYS_BASE + BECORE_YUVNR_RADIAL_CENTER,
	  BECORE_YUVP_PHYS_BASE + BECORE_YUVNR_RADIAL_CENTER,
	  BECORE_GEN_NR_GEOMETRY },
	BECORE_YUVNR_TUNING_RANGES
	/*
	 * The temporal gain curve, which the generated list above cannot carry
	 * because neither its knots nor its slopes are field deposits.  The
	 * knots are `SetTnrLut`'s six, built out of `mcfp_gain_lut_x`'s five
	 * and a stated 64; the slopes describe `mcfp_gain_lut_y`, and that
	 * *is* in the list -- so leaving either replayed would leave the
	 * recipe describing a curve the driver no longer writes.
	 */
	{ BECORE_YUVP_PHYS_BASE + BECORE_YUVNR_TNR_KNOT_FIRST,
	  BECORE_YUVP_PHYS_BASE + BECORE_YUVNR_TNR_KNOT_FIRST +
	  (BECORE_YUVNR_TNR_KNOTS / 2 - 1) * 4, BECORE_GEN_YUVNR },
	{ BECORE_YUVP_PHYS_BASE + BECORE_YUVNR_TNR_SLOPE_FIRST,
	  BECORE_YUVP_PHYS_BASE + BECORE_YUVNR_TNR_SLOPE_FIRST +
	  (BECORE_YUVNR_TNR_KNOTS - 2) * 4, BECORE_GEN_YUVNR },
	{ BECORE_YUVP_SHARPEN_BYPASS_REG, BECORE_YUVP_SHARPEN_BYPASS_REG,
	  BECORE_GEN_SHARPEN },
	{ BECORE_YUVP_SHARPEN_SENSOR_REG, BECORE_YUVP_SHARPEN_SENSOR_REG,
	  BECORE_GEN_SHARPEN_GEOMETRY },
	{ BECORE_YUVP_SHARPEN_STEP_REG, BECORE_YUVP_SHARPEN_STEP_REG,
	  BECORE_GEN_SHARPEN_GEOMETRY },
	{ BECORE_YUVP_SHARPEN_CONT_CONFIG4_REG,
	  BECORE_YUVP_SHARPEN_CONT_CONFIG4_REG, BECORE_GEN_UNWRITTEN },
	{ BECORE_YUVP_SHARPEN_SKIN_B_GAIN_REG,
	  BECORE_YUVP_SHARPEN_SKIN_B_GAIN_REG, BECORE_GEN_UNWRITTEN },
	{ BECORE_YUVP_SHARPEN_APPLY_DESAT3_REG,
	  BECORE_YUVP_SHARPEN_APPLY_DESAT3_REG, BECORE_GEN_UNWRITTEN },
	BECORE_SHARPEN_TUNING_RANGES
	{ BECORE_YUVP_PHYS_BASE + 0x5008, BECORE_YUVP_PHYS_BASE + 0x5008,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5010, BECORE_YUVP_PHYS_BASE + 0x5010,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5018, BECORE_YUVP_PHYS_BASE + 0x5018,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5154, BECORE_YUVP_PHYS_BASE + 0x5158,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5160, BECORE_YUVP_PHYS_BASE + 0x5160,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5168, BECORE_YUVP_PHYS_BASE + 0x5168,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5170, BECORE_YUVP_PHYS_BASE + 0x5170,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x517c, BECORE_YUVP_PHYS_BASE + 0x5184,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x518c, BECORE_YUVP_PHYS_BASE + 0x518c,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5198, BECORE_YUVP_PHYS_BASE + 0x5198,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x51a0, BECORE_YUVP_PHYS_BASE + 0x51a8,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5200, BECORE_YUVP_PHYS_BASE + 0x5204,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x523c, BECORE_YUVP_PHYS_BASE + 0x5248,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5254, BECORE_YUVP_PHYS_BASE + 0x5254,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5268, BECORE_YUVP_PHYS_BASE + 0x5278,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5290, BECORE_YUVP_PHYS_BASE + 0x52b0,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x52d4, BECORE_YUVP_PHYS_BASE + 0x52e0,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5304, BECORE_YUVP_PHYS_BASE + 0x5314,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5328, BECORE_YUVP_PHYS_BASE + 0x5330,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5710, BECORE_YUVP_PHYS_BASE + 0x571c,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5804, BECORE_YUVP_PHYS_BASE + 0x5820,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5830, BECORE_YUVP_PHYS_BASE + 0x5830,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5840, BECORE_YUVP_PHYS_BASE + 0x5840,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5850, BECORE_YUVP_PHYS_BASE + 0x5850,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5860, BECORE_YUVP_PHYS_BASE + 0x5860,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5870, BECORE_YUVP_PHYS_BASE + 0x5884,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x5904, BECORE_YUVP_PHYS_BASE + 0x5908,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x59ac, BECORE_YUVP_PHYS_BASE + 0x59c8,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_PHYS_BASE + 0x59f0, BECORE_YUVP_PHYS_BASE + 0x59f4,
	  BECORE_GEN_SHARPEN_DEFAULT },
	{ BECORE_YUVP_LPF_FIRST, BECORE_YUVP_LPF_LAST,
	  BECORE_GEN_LPF },
	{ BECORE_YUVP_LPF_NORM_REG, BECORE_YUVP_LPF_NORM_REG,
	  BECORE_GEN_LPF_NORM },
	{ BECORE_YUVP_CONFMAP_FIRST, BECORE_YUVP_CONFMAP_LAST,
	  BECORE_GEN_SCENE_INPUT },
	{ BECORE_YUVP_FACE_REGION_FIRST, BECORE_YUVP_FACE_REGION_LAST,
	  BECORE_GEN_SCENE_INPUT },
	{ BECORE_YUVP_ROI_REGION_FIRST, BECORE_YUVP_ROI_REGION_LAST,
	  BECORE_GEN_SCENE_INPUT },
	{ BECORE_YUVP_NOISE_SEED_FIRST, BECORE_YUVP_NOISE_SEED_LAST,
	  BECORE_GEN_NOISE_SEED },
	{ BECORE_YUVP_LTM_ENABLE_REG, BECORE_YUVP_LTM_ENABLE_REG,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_CONFIG_FIRST, BECORE_YUVP_LTM_CONFIG_LAST,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_LUMA_FIRST, BECORE_YUVP_LTM_LUMA_LAST,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_GMAP_FIRST, BECORE_YUVP_LTM_GMAP_LAST,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_GRID_FIRST, BECORE_YUVP_LTM_GRID_LAST,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_TRANS_FIRST, BECORE_YUVP_LTM_TRANS_LAST,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_TONEADJ_FIRST, BECORE_YUVP_LTM_TONEADJ_LAST,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_LUMA_THRES_REG, BECORE_YUVP_LTM_LUMA_THRES_REG,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_SATCTRL_FIRST, BECORE_YUVP_LTM_SATCTRL_LAST,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_UNITY_FIRST, BECORE_YUVP_LTM_UNITY_LAST,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_DEGAMMA_GATE_FIRST, BECORE_YUVP_DEGAMMA_GATE_LAST,
	  BECORE_GEN_YUVP_DEGAMMA },
	{ BECORE_YUVP_DEGAMMA_TBL_FIRST, BECORE_YUVP_DEGAMMA_TBL_LAST,
	  BECORE_GEN_YUVP_DEGAMMA },
	{ BECORE_YUVP_DEGAMMA_X_LOW_FIRST, BECORE_YUVP_DEGAMMA_X_LOW_LAST,
	  BECORE_GEN_YUVP_DEGAMMA },
	{ BECORE_YUVP_DEGAMMA_X_HIGH_FIRST, BECORE_YUVP_DEGAMMA_X_HIGH_LAST,
	  BECORE_GEN_YUVP_DEGAMMA },
	{ BECORE_YUVP_GAMMA_GATE_FIRST, BECORE_YUVP_GAMMA_GATE_LAST,
	  BECORE_GEN_YUVP_GAMMA },
	{ BECORE_YUVP_GAMMA_R_FIRST, BECORE_YUVP_GAMMA_R_DELTA_REG,
	  BECORE_GEN_YUVP_GAMMA },
	{ BECORE_YUVP_GAMMA_G_LOW_FIRST, BECORE_YUVP_GAMMA_G_LOW_LAST,
	  BECORE_GEN_YUVP_GAMMA },
	{ BECORE_YUVP_GAMMA_G_HIGH_FIRST, BECORE_YUVP_GAMMA_G_DELTA_REG,
	  BECORE_GEN_YUVP_GAMMA },
	{ BECORE_YUVP_GAMMA_B_FIRST, BECORE_YUVP_GAMMA_B_DELTA_REG,
	  BECORE_GEN_YUVP_GAMMA },
	{ BECORE_YUVP_GAMMA_X_LOW_FIRST, BECORE_YUVP_GAMMA_X_LOW_LAST,
	  BECORE_GEN_YUVP_GAMMA },
	{ BECORE_YUVP_GAMMA_X_HIGH_FIRST, BECORE_YUVP_GAMMA_X_HIGH_LAST,
	  BECORE_GEN_YUVP_GAMMA },
	{ BECORE_YUVP_INVCCM33_FIRST, BECORE_YUVP_INVCCM33_LAST,
	  BECORE_GEN_INVCCM33 },
	{ BECORE_YUVP_CCM_CONFIG_REG, BECORE_YUVP_CCM_OFFSET_LAST,
	  BECORE_GEN_CCM },
	{ BECORE_YUVP_CLUT_BYPASS_REG, BECORE_YUVP_CLUT_EN_CONFIG_REG,
	  BECORE_GEN_CLUT },
	{ BECORE_YUVP_CLUT_MATRIX_FIRST, BECORE_YUVP_CLUT_MATRIX_LAST,
	  BECORE_GEN_CLUT },
	{ BECORE_YUVP_CLUT_1DLUT_Y_FIRST,
	  BECORE_YUVP_CLUT_1DLUT_Y_FIRST + BECORE_CLUT_1DLUT_LAST,
	  BECORE_GEN_CLUT_1DLUT },
	{ BECORE_YUVP_CLUT_1DLUT_U_FIRST,
	  BECORE_YUVP_CLUT_1DLUT_U_FIRST + BECORE_CLUT_1DLUT_LAST,
	  BECORE_GEN_CLUT_1DLUT },
	{ BECORE_YUVP_CLUT_1DLUT_V_FIRST,
	  BECORE_YUVP_CLUT_1DLUT_V_FIRST + BECORE_CLUT_1DLUT_LAST,
	  BECORE_GEN_CLUT_1DLUT },
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
	{ BECORE_MCSC_RDMA_R0_FIRST,
	  BECORE_MCSC_RDMA_R0_FIRST + BECORE_MCSC_WDMA_OFF_LAST,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_WDMA_W0_FIRST,
	  BECORE_MCSC_WDMA_W0_FIRST + BECORE_MCSC_WDMA_OFF_LAST,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_SECU_SEQID_REG, BECORE_MCSC_SECU_SEQID_REG,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_HWFC_START_REG, BECORE_MCSC_HWFC_START_REG,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_PC0_CTRL_REG, BECORE_MCSC_PC0_CTRL_REG,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_PC0_COEFF_CTRL_REG, BECORE_MCSC_PC0_COEFF_CTRL_REG,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_PC1_CTRL_REG, BECORE_MCSC_PC4_CTRL_REG,
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
	{ BECORE_MCSC_DJAG_RADIAL_CENTRE_REG,
	  BECORE_MCSC_DJAG_RADIAL_CENTRE_REG, BECORE_GEN_DJAG },
	{ BECORE_MCSC_DJAG_RADIAL_FIRST, BECORE_MCSC_DJAG_RADIAL_LAST,
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
 * from RGB, looks that up in a tone curve and applies a spatial gain grid.
 * The guide curve comes from a parameters block with the ramp below under it;
 * everything else in the block is answered here -- the gate and the enables,
 * the luma weights, the grid the frame is divided into, the fields that
 * describe that grid to the block, the vendor's own unity fills, and its
 * tone-adjust curve.
 */
/*
 * How far the block steps through the grid per raster pixel, at Q16.
 *
 * The reciprocal of a cell size is *not* this: the two agree only where the
 * cells divide the raster exactly, and 32 columns over 2608 gives 804 where
 * 65536 / (2608 / 32) gives 809. The grid is 32 x 24 x 8 at every readout the
 * vendor was captured at, 16:9 ones included, so its cells are square only on
 * a 4:3 frame and the horizontal and vertical steps are independent. The
 * half-step is the floor of half the step.
 */
static int becore_ltm_grid_scale(u32 cells, u32 extent, u32 *scale)
{
	u32 step;

	if (!extent)
		return -EINVAL;
	step = (u32)DIV_ROUND_CLOSEST_ULL((u64)cells << 16, extent);
	if (step > U16_MAX)
		return -ERANGE;
	*scale = step;

	return 0;
}

/* Sample n of the guide curve's identity: round(n * ONE / (POINTS - 1)). */
static u32 becore_ltm_curve_identity(u32 index)
{
	return DIV_ROUND_CLOSEST(index * EXYNOS_BECORE_LTM_CURVE_ONE,
				 EXYNOS_BECORE_LTM_CURVE_POINTS - 1);
}

static int becore_yuvp_ltm_value(const struct becore_raster *chain, u32 offset,
				 u32 *value)
{
	u32 scale;
	int ret;

	if (offset & 3)
		return -EINVAL;

	switch (offset) {
	case 0x000:				/* LTM_ENABLE: the block runs */
		*value = 1;
		return 0;
	case 0x004:				/* ENABLE_SLICE_AFFINE_GRID */
	case 0x008:				/* ENABLE_SATURATION_CTRL */
		*value = 1;
		return 0;
	case 0x00c:				/* LUMACALC_LIGHTNESS_WEIGHT */
		*value = 0;
		return 0;
	case 0x010:				/* LUMACALC_GRAY_WEIGHT */
		*value = BECORE_LTM_LUMACALC_GRAY_Q8;
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
	case 0x1a4:				/* TRANS_SLOPE_FRAC_BIT */
		*value = BECORE_LTM_TRANS_SLOPE_FRAC_BIT;
		return 0;
	case 0x1a8:				/* TRANS_BIAS_BIT_ADJUST */
		*value = BECORE_LTM_TRANS_BIAS_BIT_ADJUST;
		return 0;
	case 0x1d0:				/* TONEADJ_USE_OUTPUT_LUMA */
		*value = 1;
		return 0;
	case 0x2d8:				/* CRECON_INPUT_LUMA_THRES */
		*value = 0;
		return 0;
	}

	if (offset >= 0x1ac && offset <= 0x1cc) {
		/*
		 * TRANS_SCALE_LUT_00..08: seventeen entries two to a register,
		 * which the translator fills with
		 * clamp(round(min_output_scale_lut[i] * 128), 0, 65535) over a
		 * shipped tuning whose LUT is seventeen zeros on all three
		 * cameras. The ninth register's high half is padding, as the
		 * tone-adjust curve's last one is.
		 */
		*value = 0;
		return 0;
	}

	if (offset >= BECORE_YUVP_LTM_GMAP_FIRST - BECORE_YUVP_LTM_BASE &&
	    offset <= BECORE_YUVP_LTM_GMAP_LAST - BECORE_YUVP_LTM_BASE) {
		/*
		 * The identity ramp, two samples to a register with the
		 * lower-numbered one in the low half.
		 *
		 * Rounded to nearest rather than truncated, which is worth
		 * saying because it is *not* about the endpoint: 127 * 32767
		 * divides by 127 exactly, so truncation reaches unity too.
		 * It is about the 63 samples from index 64 up, where the two
		 * differ by one -- and the identity has to be bit-exact
		 * against what the parameters path packs, or "send the ramp
		 * and the frame does not move" stops being a check on this
		 * default.
		 */
		u32 index = (offset - (BECORE_YUVP_LTM_GMAP_FIRST -
				       BECORE_YUVP_LTM_BASE)) / 4 *
			    BECORE_LTM_CURVE_PER_REG;

		*value = becore_ltm_curve_identity(index) |
			 becore_ltm_curve_identity(index + 1) << 16;
		return 0;
	}

	if (offset >= BECORE_YUVP_LTM_TONEADJ_FIRST - BECORE_YUVP_LTM_BASE &&
	    offset <= BECORE_YUVP_LTM_TONEADJ_LAST - BECORE_YUVP_LTM_BASE) {
		/*
		 * The tone-adjust curve, two Q14 samples to a register with
		 * the lower-numbered one in the low half. The odd sample count
		 * leaves the last high half zero.
		 */
		u32 index = (offset - (BECORE_YUVP_LTM_TONEADJ_FIRST -
				       BECORE_YUVP_LTM_BASE)) / 4 *
			    BECORE_LTM_TONEADJ_PER_REG;

		*value = becore_ltm_toneadj[index];
		if (index + 1 < BECORE_LTM_TONEADJ_ENTRIES)
			*value |= (u32)becore_ltm_toneadj[index + 1] << 16;
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

	/* The remaining four are the steps it walks the grid with. */
	switch (offset) {
	case 0x134:				/* SLCGRID_GRID_X_SCALE */
	case 0x138:				/* ..._X_SCALE_HALF */
		ret = becore_ltm_grid_scale(BECORE_LTM_SLCGRID_COLUMNS,
					    chain->width, &scale);
		if (ret)
			return ret;
		*value = offset == 0x134 ? scale : scale >> 1;
		return 0;
	case 0x13c:				/* SLCGRID_GRID_Y_SCALE */
	case 0x140:				/* ..._Y_SCALE_HALF */
		ret = becore_ltm_grid_scale(BECORE_LTM_SLCGRID_ROWS,
					    chain->height, &scale);
		if (ret)
			return ret;
		*value = offset == 0x13c ? scale : scale >> 1;
		return 0;
	}

	return -EINVAL;
}

/*
 * DIABLO_CCM's gate, its nine coefficients and its three offsets, by offset
 * from the block's base.
 *
 * Say which registers rather than answering for the whole block: a mis-stated
 * range in the table above would otherwise be answered rather than refused,
 * and the generator that has to agree with this refuses it.
 */
static int becore_yuvp_ccm_value(u32 offset, u32 *value)
{
	u32 index;

	if (offset & 3)
		return -EINVAL;

	if (offset >= BECORE_YUVP_CCM_MATRIX_FIRST - BECORE_YUVP_CCM_BASE &&
	    offset <= BECORE_YUVP_CCM_MATRIX_LAST - BECORE_YUVP_CCM_BASE) {
		index = (offset - (BECORE_YUVP_CCM_MATRIX_FIRST -
				   BECORE_YUVP_CCM_BASE)) / 4;
		/* Row-major, so the diagonal is at 0, 4 and 8. */
		*value = index % 4 ? 0 : EXYNOS_BECORE_CCM_ONE;
		return 0;
	}

	if (offset != 0x000 && (offset < 0x028 || offset > 0x030))
		return -EINVAL;

	*value = 0;

	return 0;
}

/* One coefficient of the inverse colour matrix, by offset from the first. */
static int becore_yuvp_invccm33_value(u32 offset, u32 *value)
{
	u32 index = offset / 4;

	if (offset & 3 || index >= BECORE_INVCCM33_COEFFICIENTS)
		return -EINVAL;

	*value = index % 4 ? 0 : 1 << BECORE_INVCCM33_Q;

	return 0;
}

/*
 * One register of a colour-LUT input curve, by offset from the curve's first.
 *
 * The identity is what the block does when nothing is asking it to reshape a
 * channel, and it is also what the vendor writes: en_config leaves all three
 * of these curves disabled, so their contents are a default rather than a
 * tuning choice. Packing is Google's own, from clut_packing.h.
 */
static int becore_yuvp_clut_1dlut_value(u32 offset, u32 *value)
{
	u32 packed = 0;
	u32 index;
	u32 entry;
	u32 field;

	if (offset & 3 || offset > BECORE_CLUT_1DLUT_LAST)
		return -EINVAL;

	if (offset == BECORE_CLUT_1DLUT_LAST) {
		/*
		 * The curve's last point is 1.0, one past a 10-bit field, so
		 * the register after entry 63 holds its distance from unity
		 * instead -- the same trick the gamma tables use for their
		 * sixty-fifth knot.
		 */
		entry = (BECORE_CLUT_1DLUT_ENTRIES - 1) * BECORE_CLUT_1DLUT_STEP;
		*value = entry | ((BECORE_CLUT_ONE - entry)
				  << BECORE_CLUT_FIELD_BITS);
		return 0;
	}

	index = offset / 4 * BECORE_CLUT_1DLUT_PER_REG;
	for (field = 0; field < BECORE_CLUT_1DLUT_PER_REG; field++) {
		entry = (index + field) * BECORE_CLUT_1DLUT_STEP;
		packed |= entry << (BECORE_CLUT_FIELD_BITS * field);
	}
	*value = packed;

	return 0;
}

/*
 * Read one fixed word back out of the recipe by register address.
 *
 * Every other generated value is a function of the hardware description or of
 * a constant, and resolves from the register alone. The demosaicer's noise
 * slopes are a function of its knots, which are tuning and stay in the recipe
 * -- so resolving one means finding a sibling register's value.
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

const struct becore_noise_curve becore_noise_curves[BECORE_NOISE_CURVES] = {
	{ BECORE_RGBP, BECORE_RGBP_DNS_X_G_REG, BECORE_RGBP_DNS_Y_G_REG,
	  BECORE_RGBP_DNS_SLOPE_G_REG, BECORE_RGBP_DNS_SHIFT_G_REG,
	  BECORE_RGBP_DNS_X_G_REG, true, true, BECORE_NOISE_FROM_BYR_DNS,
	  offsetof(struct exynos_becore_params_byr_dns, std_lut_y_g) },
	{ BECORE_RGBP, BECORE_RGBP_DNS_X_G_REG, BECORE_RGBP_DNS_Y_RB_REG,
	  BECORE_RGBP_DNS_SLOPE_RB_REG, BECORE_RGBP_DNS_SHIFT_RB_REG,
	  BECORE_RGBP_DNS_X_RB_REG, true, true, BECORE_NOISE_FROM_BYR_DNS,
	  offsetof(struct exynos_becore_params_byr_dns, std_lut_y_rb) },
	{ BECORE_YUVP, BECORE_YUVP_NR_X_Y_REG, BECORE_YUVP_NR_Y_Y_REG,
	  BECORE_YUVP_NR_SLOPE_Y_REG, BECORE_YUVP_NR_SHIFT_Y_REG, 0, false,
	  true, BECORE_NOISE_FROM_YUVNR,
	  offsetof(struct exynos_becore_params_yuvnr, std_lut_y) },
	{ BECORE_YUVP, BECORE_YUVP_NR_X_Y_REG, BECORE_YUVP_NR_Y_UV_REG,
	  BECORE_YUVP_NR_SLOPE_UV_REG, BECORE_YUVP_NR_SHIFT_UV_REG,
	  BECORE_YUVP_NR_X_UV_REG, false, true, BECORE_NOISE_FROM_YUVNR,
	  offsetof(struct exynos_becore_params_yuvnr, std_lut_uv) },
};

/*
 * The Bayer denoiser's knot domain, which is not tuning: `ByrDnsDynamicParam`
 * has no x array at all and `lyric::TranslateByrDns` writes these eight
 * twelve-bit literals behind a guard that both y arrays are exactly eight
 * long.  Both of its curves share the one copy -- the rb registers repeat it
 * -- so a parameters block carries ranges and nothing else.
 */
static const s32 becore_byr_dns_domain[BECORE_NOISE_KNOTS] = {
	0, 150, 300, 500, 1000, 1800, 2500, 4095,
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
 * Every knot of a curve a parameters block carries, clamped as it is sent.
 *
 * The two blocks differ in where the *domain* comes from and only there: the
 * noise reducer's is tuning and travels in its block, the Bayer denoiser's is
 * the vendor's own compiled-in literals and belongs to the driver.
 */
static void
becore_noise_knots_from_params(const void *block,
			       const struct becore_noise_curve *curve,
			       struct becore_noise_knots *knots)
{
	const __s32 *range = (const __s32 *)((const u8 *)block +
					     curve->params_range);
	const __s32 *domain;
	u32 index;

	switch (curve->source) {
	case BECORE_NOISE_FROM_YUVNR:
		domain = ((const struct exynos_becore_params_yuvnr *)block)
				 ->std_lut_x;
		break;
	case BECORE_NOISE_FROM_BYR_DNS:
		domain = becore_byr_dns_domain;
		break;
	default:
		/*
		 * A curve whose knots no block carries has no business here,
		 * and a source added without a case would otherwise take the
		 * Bayer denoiser's literals in silence.
		 */
		WARN_ONCE(1, "noise curve source %u carries no domain\n",
			  curve->source);
		domain = becore_byr_dns_domain;
		break;
	}

	for (index = 0; index < BECORE_NOISE_KNOTS; index++) {
		knots->x[index] = clamp(domain[index], 0,
					BECORE_NOISE_KNOT_MAX);
		knots->y[index] = clamp(range[index], 0, BECORE_NOISE_KNOT_MAX);
	}
}

/* A curve whose knots the recipe still carries: the demosaicer's two. */
static int becore_noise_knots_from_recipe(struct device *dev, size_t curve_index,
					  struct becore_noise_knots *knots)
{
	const struct becore_noise_curve *curve =
		&becore_noise_curves[curve_index];
	u32 index;
	int ret;

	for (index = 0; index < BECORE_NOISE_KNOTS; index++) {
		ret = becore_noise_read_knot(curve->block, curve->x_first,
					     index, &knots->x[index]);
		if (ret)
			return dev_err_probe(dev, ret,
					     "noise curve %zu has no knot domain\n",
					     curve_index);
		ret = becore_noise_read_knot(curve->block, curve->y_first,
					     index, &knots->y[index]);
		if (ret)
			return dev_err_probe(dev, ret,
					     "noise curve %zu has no knot range\n",
					     curve_index);
	}

	return 0;
}

/*
 * Resolve every curve's knots, and check each domain rises while we are here.
 * A curve that cannot be resolved is a wiring mistake in the table above --
 * most likely a chroma curve pointed at its own generated domain instead of
 * the luma copy it repeats -- and it is worth failing probe over rather than
 * discovering at the first STREAMON.
 *
 * No curve comes from the recipe any more: the driver generates all four
 * blocks' knot registers, so the recipe no longer carries them, and what a
 * stream with no parameters buffer writes there is the block's own default --
 * becore_yuvnr_off's stated curve, and for the Bayer denoiser a parameter set
 * of all zeros over the domain the driver states. Resolving them out of the
 * same block the field table encodes is what keeps the slopes describing the
 * knots the hardware was given -- the same reason becore_noise_knots_for()
 * takes a buffer's knots over these.
 *
 * becore_noise_knots_from_recipe() is kept for a curve that has neither, which
 * is the state every one of these was in until its translator was read.
 */
int becore_noise_knots_resolve(struct device *dev)
{
	static const struct exynos_becore_params_byr_dns byr_dns_off = {};
	size_t i;
	u32 index;
	int ret;

	for (i = 0; i < ARRAY_SIZE(becore_noise_curves); i++) {
		const struct becore_noise_curve *curve = &becore_noise_curves[i];

		if (curve->source == BECORE_NOISE_FROM_YUVNR) {
			becore_noise_knots_from_params(&becore_yuvnr_off, curve,
						       &becore_noise_knots[i]);
		} else if (curve->source == BECORE_NOISE_FROM_BYR_DNS) {
			becore_noise_knots_from_params(&byr_dns_off, curve,
						       &becore_noise_knots[i]);
		} else {
			ret = becore_noise_knots_from_recipe(dev, i,
							     &becore_noise_knots[i]);
			if (ret)
				return ret;
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
 * Where a curve's knots come from.
 *
 * Every one of these curves is also carried by a parameters block, and when
 * one is in force the knots have to come from *it* rather than from the
 * defaults resolved at probe. Otherwise a buffer would move the knot registers
 * and leave behind the slopes and shifts that describe them, which is a
 * piecewise-linear curve whose segments contradict its own knots. The clamp is
 * the one the generated field table applies when it deposits these same values
 * into those knot registers, and it is here for the same reason: the slope has
 * to describe the knots the hardware was given.
 */
static_assert(BECORE_NOISE_KNOTS == EXYNOS_BECORE_YUVNR_STD_LUT_POINTS);
static_assert(BECORE_NOISE_KNOTS == EXYNOS_BECORE_BYR_DNS_STD_LUT_POINTS);

static const struct becore_noise_knots *
becore_noise_knots_for(const struct becore_device *becore, size_t curve_index,
		       struct becore_noise_knots *scratch)
{
	const struct becore_noise_curve *curve =
		&becore_noise_curves[curve_index];

	if (curve->source == BECORE_NOISE_FROM_YUVNR &&
	    becore->params.yuvnr_valid)
		becore_noise_knots_from_params(&becore->params.yuvnr, curve,
					       scratch);
	else if (curve->source == BECORE_NOISE_FROM_BYR_DNS &&
		 becore->params.byr_dns_valid)
		becore_noise_knots_from_params(&becore->params.byr_dns, curve,
					       scratch);
	else
		return &becore_noise_knots[curve_index];

	return scratch;
}

/*
 * One segment's slope, and the number of fractional bits it is expressed at.
 * There are eight slope fields for seven segments, so the eighth repeats the
 * seventh: past the last knot the curve does not turn.
 *
 * `TranslateYuvNrCommon` starts at eleven fractional bits and gives one up at
 * a time until the quotient fits the register's signed 13-bit field. Two
 * details of that are the vendor's rather than the obvious thing, and both had
 * to be read off the decompile: it truncates the quotient to `short` *before*
 * testing the fit, and it accepts a magnitude of exactly 4095 rather than the
 * 4096 the field would hold.
 *
 * No shipped tuning reaches either -- the widest quotient over the 6,174
 * segments of this phone's own trees is 2,348, so the loop runs once and the
 * truncation changes nothing. A parameters block can reach both, because its
 * knots are userspace's: a full-range rise over a one-step gap is a quotient
 * of eight million, which the truncation turns into a small negative slope
 * that then fits. Reproducing that is the point -- it is what the hardware
 * would have been given.
 *
 * `TranslateByrDns` does the same thing from the same eleven bits against the
 * same 4095, with one difference the census made worth reproducing: it *rounds*
 * the quotient where the noise reducer truncates it. Assuming one block's rule
 * for the other reproduces every exact division and misses by one everywhere
 * else, which is precisely the kind of error no offline check would have
 * found while both blocks replayed their slopes.
 */
static int becore_noise_segment(const struct becore_noise_curve *curve,
				const struct becore_noise_knots *knots,
				u32 index, u32 *slope, u32 *shift)
{
	const s32 limit = (s32)(BECORE_NOISE_SLOPE_MASK >> 1);
	u32 bits = BECORE_NOISE_SLOPE_SHIFT;
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

	for (;;) {
		magnitude = (delta < 0 ? -delta : delta) << bits;
		if (curve->round)
			quotient = (2 * magnitude + dx) / (2 * dx);
		else
			quotient = magnitude / dx;
		if (delta < 0)
			quotient = -quotient;

		if (!curve->search) {
			if (quotient > limit || quotient < -limit - 1)
				return -ERANGE;
			break;
		}

		quotient = (s16)quotient;
		if ((quotient < 0 ? -quotient : quotient) <= limit)
			break;
		if (!bits)
			return -ERANGE;
		bits--;
	}

	*slope = (u32)quotient & BECORE_NOISE_SLOPE_MASK;
	*shift = bits;

	return 0;
}

static int becore_noise_value(const struct becore_device *becore,
			      enum becore_block_id id, u32 reg, u32 kind,
			      u32 *value)
{
	const struct becore_noise_curve *curve;
	const struct becore_noise_knots *knots;
	struct becore_noise_knots scratch;
	size_t curve_index;
	u32 packed = 0;
	u32 shift;
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
	curve = &becore_noise_curves[curve_index];
	knots = becore_noise_knots_for(becore, curve_index, &scratch);

	if (kind == BECORE_GEN_NOISE_SHIFT) {
		for (i = 0; i < BECORE_NOISE_SHIFT_NIBBLES; i++) {
			ret = becore_noise_segment(curve, knots, i, &low,
						   &shift);
			if (ret)
				return ret;
			packed |= shift << (4 * i);
		}
		*value = packed;
		return 0;
	}
	if (kind == BECORE_GEN_NOISE_DOMAIN) {
		index = (reg - curve->domain_first) / 4 * 2;
		if (index + 1 >= BECORE_NOISE_KNOTS)
			return -EINVAL;
		*value = (((u32)knots->x[index + 1] & 0xffff) << 16) |
			 ((u32)knots->x[index] & 0xffff);
		return 0;
	}

	index = (reg - curve->slope_first) / 4 * 2;
	ret = becore_noise_segment(curve, knots, index, &low, &shift);
	if (ret)
		return ret;
	ret = becore_noise_segment(curve, knots, index + 1, &high, &shift);
	if (ret)
		return ret;
	*value = (high << 16) | low;

	return 0;
}

/* Samsung's init_djag_cfgs, which is also this field's POR reset value. */
static const u16 becore_djag_lfsr_seeds[] = { 44257, 4671, 47792 };
static const u8 becore_djag_dither_ramp[] = { 0, 0, 1, 2, 3, 4, 6, 7, 8 };

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
	/*
	 * Zero at every ratio and not only at unity: seven captured programs
	 * run this pre-scaler as a stretcher and all seven write zero, where
	 * POLY_SC0's rule would put half the ratio.  See
	 * becore_scaler_init_phase().
	 */
	case 0x01c:		/* PS_H_INIT_PHASE_OFFSET */
	case 0x020:		/* PS_V_INIT_PHASE_OFFSET */
	case 0x080:		/* RECOM_CTRL: detail restoration is off */
	case 0x088:		/* RECOM_WEIGHT: and off by its weight too */
	case 0x0a4:		/* RECOM_RADIAL_CENTER */
	case 0x0ac:		/* RECOM_RADIAL_BIQUAD_FACTOR_A */
	case 0x0b0:		/* RECOM_RADIAL_BIQUAD_FACTOR_B */
	case 0x0b4:		/* RECOM_RADIAL_GAIN_ENABLE: and its radial gain
				 * with it, all four
				 */
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

/* Where each block's scaler puts its two init phase offsets and round mode. */
static const u32 becore_scaler_phase_first[] = {
	BECORE_RGBP_SC_PHASE_FIRST,
	BECORE_MCSC_SC0_PHASE_FIRST,
	BECORE_MCSC_PC0_PHASE_FIRST,
};

/*
 * The ratios are the caller's because they are the block's rather than the
 * register's: one pair covers whichever of that block's scalers this register
 * belongs to. Taking them from becore_sc_ratio() is what stops the origin and
 * the poly-phase filter describing different scalings, which is the reason the
 * coefficients take it from there too.
 *
 * All three of these follow the rule, RGBP's included, and no capture could
 * have said so: all twelve captured RGBP programs shrink, so every one writes
 * the same zero either reading predicts. Lyric settles it, and
 * lyric::YuvscDriver::ConfigureYuvscBlock() is where -- it divides the crop by
 * the destination, converts at Q20, and selects between `ratio >> 1` and zero
 * on the sign of the comparison, with the same `ubfx #1, #20` and `csel` that
 * lyric::McscScalerChain::ConfigureCropAndScaleBlockInternal() uses for
 * POLY_SC0. Samsung's is_scaler_set_post_scaler_coef() is the third.
 */
static int becore_scaler_phase_value(u32 reg, u32 h_ratio, u32 v_ratio,
				     u32 *value)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_scaler_phase_first); i++) {
		u32 first = becore_scaler_phase_first[i];

		if (reg < first || reg > first + BECORE_SCALER_PHASE_LAST)
			continue;
		switch (reg - first) {
		case 0x00:	/* H_INIT_PHASE_OFFSET */
		case 0x04:	/* V_INIT_PHASE_OFFSET */
			*value = becore_scaler_init_phase((reg - first) ?
							  v_ratio : h_ratio);
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
 * BT.601 as rationals, column-major by input channel: (Y, U, V) of R, then of
 * G, then of B. The luma row is Kr, Kg, Kb exactly. The two chroma rows are
 * the four-decimal values the vendor's own defaults carry rather than
 * Kr / (2 * (1 - Kb)) and its relatives in full, and that is not cosmetic: at
 * Q13 the two forms agree in all nine coefficients, so the block this table
 * was written for cannot tell them apart, but the Q14 copy in YUVP can, and
 * the exact form misses two of its nine. The constants are Lyric's own, from
 * SetDefaultTuning(RgbYuvConversionTuning *) in liblyric_iq.so, whose float
 * array reads 0.299, 0.587, 0.114, -0.1687, -0.3313, 0.5, 0.5, -0.4187,
 * -0.0813.
 */
static const struct becore_csc_coefficient {
	s32 numerator;
	s32 denominator;
} becore_csc_matrix[3][3] = {
	{ { 299, 1000 }, { -1687, 10000 }, { 1, 2 } },
	{ { 587, 1000 }, { -3313, 10000 }, { -4187, 10000 } },
	{ { 114, 1000 }, { 1, 2 }, { -813, 10000 } },
};

/*
 * BT.601 the other way, as exact rationals and row-major: R = Y + 2(1 - Kr) V,
 * G = Y - Kb/(1 - Kr - Kb) * 2(1 - Kb) U - Kr/(1 - Kr - Kb) * 2(1 - Kr) V, and
 * B = Y + 2(1 - Kb) U. The colour LUT converts to RGB before it looks a colour
 * up, which is why a YUV block carries this at all.
 *
 * This one stays exact where the forward table above had to become the
 * vendor's decimals, and the difference is measured rather than assumed: the
 * two forms are indistinguishable in all nine coefficients up to Q15 and first
 * diverge at Q16, in G's U coefficient. The three blocks that read this table
 * are at Q10 and Q12, so there is four binary places of headroom -- adding
 * another copy is only a hazard if the hardware ever carries this matrix at
 * Q16, which nothing here does.
 */
static const struct becore_csc_coefficient
becore_clut_yuv2rgb[3][3] = {
	{ { 1, 1 }, { 0, 1 },			{ 1402, 1000 } },
	{ { 1, 1 }, { -114 * 1772, 587 * 1000 }, { -299 * 1402, 587 * 1000 } },
	{ { 1, 1 }, { 1772, 1000 },		{ 0, 1 } },
};

static_assert(ARRAY_SIZE(becore_clut_yuv2rgb) *
	      ARRAY_SIZE(becore_clut_yuv2rgb[0]) ==
	      BECORE_CLUT_MATRIX_COEFFICIENTS);

/* [1, 2, 1] / 4 scaled by 32, one byte per tap. */
static const u8 becore_chroma_lpf_taps[] = { 0, 32, 64, 32, 0 };

/*
 * The sharpener's three low-pass kernels, and LPF_NORM above them.
 *
 * Each is square, symmetric and stored as one quadrant, row-major with the
 * *corner* first and the centre last, two 16-bit taps to a register with the
 * lower-numbered one in the low half; an odd quadrant leaves the last high
 * half unused. So a span x span quadrant describes a (2 * span - 1)-tap
 * kernel, and a tap's weight in the whole kernel is 4 unless it sits on the
 * centre row or column.
 *
 * Not tuning: `TranslateYuvSharpEnhancer` never writes any of these twenty
 * registers, so they come from `GetDefaultYuvSharpEnhancer` and are the same
 * in all 426 captured programs. A three-scale unsharp-mask band splitter is
 * what the block *is*.
 *
 * Stating them turns LPF_NORM into a real check. It is the log2 of each
 * kernel's sum, and until now those three sums were themselves constants, so
 * the power-of-two test was a property of the constants rather than of the
 * kernels. Now the sums are computed from the taps: 16, 512 and 4096, which
 * is what these quadrants really add up to, and a kernel that stopped
 * summing to a power of two would fail the encode instead of being encoded
 * wrong.
 */
/*
 * The rest of SHARPENHANCER that is not tuning: 84 registers
 * `TranslateYuvSharpEnhancer` never writes, so they come from
 * `GetDefaultYuvSharpEnhancer` and are identical in all 426 captured programs,
 * on three cameras and at eighteen readouts.
 *
 * Be clear what this is and is not. It is not a derivation and it is not a
 * hardware reset state: it is the vendor's compiled-in default, the same
 * standing RGBP's `DMSC` tail has, and it is a bring-up default under
 * ADR 0009 rather than something the driver worked out. What it buys is that
 * the kernel stops carrying one captured *program* and starts carrying the
 * block's own defaults, which is what makes the 145 registers beside them --
 * the ones the translator does write, from the tuning tree -- the only thing
 * left that a parameters block has to own.
 *
 * Two families in here are recognisable and worth recording rather than
 * leaving as magic:
 *
 *  - `yuv2rgb_coeff_0..4` at 0x5804 are the BT.601 full-range inverse at Q10
 *    in 13-bit signed fields, nine coefficients over ten halves with the last
 *    unused, column-major by *input* channel: (1024, 1024, 1024) for Y, then
 *    (0, -352, 1815) for U and (1436, -731, 0) for V. That is
 *    round(2(1-Kb) * 1024) = 1815 and round(2(1-Kr) * 1024) = 1436 on the
 *    nose. The two offsets beside them are not decoded.
 *  - `noise_gain_lut_luma*` at 0x5710 is a monotone breakpoint ramp, two
 *    16-bit entries to a register: 0, 16, 32, 64, 96, 128, 192, 256. The
 *    three `*_power_luma_*` groups are the same eight numbers packed at bits
 *    0 and 8 instead, in fields wider than a byte because the last entry is
 *    256: 0 | 16 << 8, 32 | 64 << 8, 96 | 128 << 8, 192 | 256 << 8. Both
 *    could be generated from the ramp; neither is today.
 */
/*
 * `YUVNR`'s fixed half: 23 words the block's tuning never reaches.
 *
 * The block has two output structures, and three things together say which
 * words belong to which. `YuvpYuvNrBlock::InitContext` hands its virtual a
 * second pointer at `&context + 0x278`. The map from context word to register
 * that `ConfigureWith` carries climbs 0x3200..0x37d0 over the 117 words below
 * that offset and then *starts over* at 0x3208 for 23 more -- two ascending
 * runs is what two structures look like from the register side. And
 * `YuvNrFixedOutput` is exactly 0x5c bytes, twenty-three u32, which is the
 * size of the second run and not of the first.
 *
 * `GetDefaultYuvNr(YuvNrFixedOutput&, YuvNrOutput&)` fills the fixed one from
 * compiled-in literals and the other by constructing a default
 * `apcamera.YuvNrTuning` and running `TranslateYuvNrCommon` over it. So these
 * 23 are the same kind of thing as the sharpener's 83 -- the block's own
 * defaults, not one frame's tuning -- and they are the first of `YUVNR` the
 * driver can state without reading the encode.
 *
 * Twelve of the literals are visible in the decompile and match the capture
 * digit for digit: `radial` is `|= 0x10000000`, `radial_thresh_limit` is
 * `& 0xe000e000 | 0x16661666`, a 128-bit store writes `wavelet_edge_map`,
 * `wavelet_wide_filter` and both `nlm_map_suppression` words at once as
 * 0x01140641, 0x28000080, 7 and 7, and a `dup` writes 0x321 into both
 * `filterweights_param3` and `_param4`.
 *
 * The other eleven are *masked* rather than set, so their zero is the context
 * arriving zeroed. All 426 captured programs agree, on three cameras and
 * eighteen sessions, but the decompile alone does not prove it -- and it is
 * where a non-zero incoming context would show up first, because
 * `contents_aware`'s mask keeps bits 8..31 and the `confmap_seg_*` masks keep
 * bits 6, 7, 22 and 23.
 *
 * Eight of them say the same thing the sharpener's confidence map said:
 * `contents_aware` is clear and `confmap_seg_0..6` are zero, so content-aware
 * segmentation is off. That is by construction here rather than by
 * coincidence -- mainline has no segmentation producer to fill it.
 */
struct becore_yuvnr_default {
	u32 offset;			/* from BECORE_YUVP_PHYS_BASE */
	u32 value;
};

#define BECORE_YUVNR_DEFAULTS		23

static const struct becore_yuvnr_default becore_yuvnr_defaults[] = {
	{ 0x3208, 0x00000000 },	/* low_power_en */
	{ 0x32d0, 0x00000001 },	/* radthrs_radial_config */
	{ 0x3314, 0x00000707 },	/* center_weight */
	{ 0x3320, 0x10000000 },	/* radial */
	{ 0x334c, 0x16661666 },	/* radial_thresh_limit */
	{ 0x3398, 0x00000000 },	/* contents_aware */
	{ 0x33a4, 0x00000000 },	/* confmap_seg_0 */
	{ 0x33ac, 0x00000000 },	/* confmap_seg_1 */
	{ 0x33b4, 0x00000000 },	/* confmap_seg_2 */
	{ 0x33bc, 0x00000000 },	/* confmap_seg_3 */
	{ 0x33c4, 0x00000000 },	/* confmap_seg_4 */
	{ 0x33cc, 0x00000000 },	/* confmap_seg_5 */
	{ 0x33d4, 0x00000000 },	/* confmap_seg_6 */
	{ 0x344c, 0x01140641 },	/* wavelet_edge_map */
	{ 0x3450, 0x28000080 },	/* wavelet_wide_filter */
	{ 0x3454, 0x00000007 },	/* nlm_map_suppression_y */
	{ 0x3458, 0x00000007 },	/* nlm_map_suppression_uv */
	{ 0x3500, 0x00000000 },	/* tuning_param1 */
	{ 0x3508, 0x00000000 },	/* tuning_param3 */
	{ 0x35dc, 0x00000001 },	/* wide_edge */
	{ 0x3610, 0x0000001f },	/* h_nr_en */
	{ 0x3784, 0x00000321 },	/* filterweights_param3 */
	{ 0x3788, 0x00000321 },	/* filterweights_param4 */
};

static_assert(ARRAY_SIZE(becore_yuvnr_defaults) == BECORE_YUVNR_DEFAULTS);

struct becore_sharpen_default {
	u32 offset;			/* from BECORE_YUVP_PHYS_BASE */
	u32 value;
};

#define BECORE_SHARPEN_DEFAULTS		83

static const struct becore_sharpen_default becore_sharpen_defaults[] = {
	{ 0x5008, 0x00000000 },	/* mono_mode_en */
	{ 0x5010, 0x00000000 },	/* start_crop */
	{ 0x5018, 0x00000000 },	/* strip */
	{ 0x5154, 0x00000000 },	/* wluma_pedestal_value */
	{ 0x5158, 0x00002000 },	/* wluma_pedestal_gain */
	{ 0x5160, 0x03340320 },	/* texture_index_1 */
	{ 0x5168, 0x18184848 },	/* dirconf */
	{ 0x5170, 0x02261e0d },	/* local_contrast */
	{ 0x517c, 0x00010001 },	/* gf_sharp_epsilon */
	{ 0x5180, 0x00000100 },	/* gf_sharp_luma_th */
	{ 0x5184, 0x04000002 },	/* gf_sharp_luma_gain */
	{ 0x518c, 0x00000000 },	/* fsharp_en */
	{ 0x5198, 0x00000000 },	/* wide_sharp_lc_control_1 */
	{ 0x51a0, 0x00000000 },	/* wide_sharp_lc_1 */
	{ 0x51a4, 0x00000000 },	/* wide_sharp_lc_2 */
	{ 0x51a8, 0x00000000 },	/* wide_sharp_lc_3 */
	{ 0x5200, 0x00000000 },	/* contents_aware_en */
	{ 0x5204, 0x003ab7fe },	/* config_0 */
	{ 0x523c, 0x00000003 },	/* limit_parameters_1 */
	{ 0x5240, 0x00000000 },	/* limit_parameters_2 */
	{ 0x5244, 0x00000003 },	/* limit_parameters_3 */
	{ 0x5248, 0x00000000 },	/* limit_parameters_4 */
	{ 0x5254, 0x000006fc },	/* sat */
	{ 0x5268, 0x00000000 },	/* gf_sharp_thresh_anchor_gain */
	{ 0x526c, 0x00001000 },	/* gf_sharp_power_luma_0_to_1 */
	{ 0x5270, 0x00004020 },	/* gf_sharp_power_luma_2_to_3 */
	{ 0x5274, 0x00008060 },	/* gf_sharp_power_luma_4_to_5 */
	{ 0x5278, 0x010000c0 },	/* gf_sharp_power_luma_6_to_7 */
	{ 0x5290, 0x00000000 },	/* gf_sharp_limit_interp */
	{ 0x5294, 0x00000000 },	/* medi_edge_thresh_anchor_gain */
	{ 0x5298, 0x00000000 },	/* narr_edge_thresh_anchor_gain */
	{ 0x529c, 0x00000000 },	/* medi_txtr_thresh_anchor_gain */
	{ 0x52a0, 0x00000000 },	/* narr_txtr_thresh_anchor_gain */
	{ 0x52a4, 0x00001000 },	/* edge_power_luma_0_to_1 */
	{ 0x52a8, 0x00004020 },	/* edge_power_luma_2_to_3 */
	{ 0x52ac, 0x00008060 },	/* edge_power_luma_4_to_5 */
	{ 0x52b0, 0x010000c0 },	/* edge_power_luma_6_to_7 */
	{ 0x52d4, 0x00001000 },	/* txtr_power_luma_0_to_1 */
	{ 0x52d8, 0x00004020 },	/* txtr_power_luma_2_to_3 */
	{ 0x52dc, 0x00008060 },	/* txtr_power_luma_4_to_5 */
	{ 0x52e0, 0x010000c0 },	/* txtr_power_luma_6_to_7 */
	{ 0x5304, 0x00000000 },	/* narr_edge_uv_limit */
	{ 0x5308, 0x00000000 },	/* narr_txtr_uv_limit */
	{ 0x530c, 0x3c1c3c37 },	/* edge_uv_limit */
	{ 0x5310, 0x39143914 },	/* txtr_uv_limit */
	{ 0x5314, 0x03030000 },	/* limit_uv_parameters */
	{ 0x5328, 0x02000200 },	/* invs_halo_texture_weight_gain */
	{ 0x532c, 0x00000000 },	/* invs_halo_texture_weight_min */
	{ 0x5330, 0x01000100 },	/* invs_halo_texture_weight_max */
	{ 0x5710, 0x00100000 },	/* noise_gain_lut_luma */
	{ 0x5714, 0x00400020 },	/* noise_gain_lut_luma_1 */
	{ 0x5718, 0x00800060 },	/* noise_gain_lut_luma_2 */
	{ 0x571c, 0x010000c0 },	/* noise_gain_lut_luma_3 */
	{ 0x5804, 0x04000400 },	/* yuv2rgb_coeff_0 */
	{ 0x5808, 0x00000400 },	/* yuv2rgb_coeff_1 */
	{ 0x580c, 0x07171ea0 },	/* yuv2rgb_coeff_2 */
	{ 0x5810, 0x1d25059c },	/* yuv2rgb_coeff_3 */
	{ 0x5814, 0x00000000 },	/* yuv2rgb_coeff_4 */
	{ 0x5818, 0x03800000 },	/* yuv2rgb_offset_0 */
	{ 0x581c, 0x00000380 },	/* yuv2rgb_offset_1 */
	{ 0x5820, 0x00000000 },	/* contd_yuv2rgb_lshift */
	{ 0x5830, 0x00152614 },	/* shift_0 */
	{ 0x5840, 0x00152614 },	/* shift_1 */
	{ 0x5850, 0x00060705 },	/* shift_2 */
	{ 0x5860, 0x00060705 },	/* shift_3 */
	{ 0x5870, 0x00242612 },	/* shift_4 */
	{ 0x5874, 0x00000000 },	/* seg_0 */
	{ 0x5878, 0x00000000 },	/* seg_1 */
	{ 0x587c, 0x00000000 },	/* seg_2 */
	{ 0x5880, 0x00000000 },	/* seg_3 */
	{ 0x5884, 0x00000000 },	/* seg_4 */
	{ 0x5904, 0x00000000 },	/* multiplier */
	{ 0x5908, 0x00000000 },	/* clamp */
	{ 0x59ac, 0x00000000 },	/* face_margin_left */
	{ 0x59b0, 0x00000000 },	/* face_margin_left_1 */
	{ 0x59b4, 0x00000000 },	/* face_margin_right */
	{ 0x59b8, 0x00000000 },	/* face_margin_right_1 */
	{ 0x59bc, 0x00000000 },	/* face_margin_up */
	{ 0x59c0, 0x00000000 },	/* face_margin_up_1 */
	{ 0x59c4, 0x00000000 },	/* face_margin_down */
	{ 0x59c8, 0x00000000 },	/* face_margin_down_1 */
	{ 0x59f0, 0x00000000 },	/* radial_gain_params */
	{ 0x59f4, 0x00000000 },	/* weight_val_params */
};

/*
 * Say how many, because an empty or short table fails *closed* but only on a
 * device: the lookup refuses every register, the encode returns -EINVAL and a
 * debugfs `run` reports nothing but "Invalid argument". This exact table
 * reached hardware empty once, written through a shell heredoc that ate its
 * own contents, and the generator's mirror was correct the whole time -- so
 * nothing offline could see it.
 */
static_assert(ARRAY_SIZE(becore_sharpen_defaults) == BECORE_SHARPEN_DEFAULTS);

struct becore_sharpen_kernel {
	u32 first;			/* register, from BECORE_YUVP_PHYS_BASE */
	u32 span;			/* the quadrant is span x span */
	u32 taps;			/* ARRAY_SIZE of the quadrant below */
	const u8 *quadrant;
};

static const u8 becore_sharpen_lpf3[] = {
	1, 2,
	2, 4,
};

static const u8 becore_sharpen_lpf5[] = {
	 3, 10, 14,
	10, 32, 44,
	14, 44, 60,
};

static const u8 becore_sharpen_lpf9[] = {
	 3,  8,  14,  20,  23,
	 8, 18,  34,  49,  55,
	14, 34,  63,  91, 104,
	20, 49,  91, 133, 151,
	23, 55, 104, 151, 168,
};

/*
 * Offsets from BECORE_YUVP_PHYS_BASE, which is what the caller passes. Each
 * row carries its quadrant's length as well as its span, so that a row naming
 * the wrong array is caught rather than read past: becore_yuvp_lpf_value()
 * refuses a row whose span does not square to its length.
 */
static const struct becore_sharpen_kernel becore_sharpen_kernels[] = {
	{ 0x5100, 2, ARRAY_SIZE(becore_sharpen_lpf3), becore_sharpen_lpf3 },
	{ 0x5108, 3, ARRAY_SIZE(becore_sharpen_lpf5), becore_sharpen_lpf5 },
	{ 0x511c, 5, ARRAY_SIZE(becore_sharpen_lpf9), becore_sharpen_lpf9 },
};

/*
 * The extents are the whole claim, so state them twice: the range in the table
 * above says which registers are the driver's, and 2 + 5 + 13 says how many
 * registers three quadrants of 4, 9 and 25 taps really need.
 */
static_assert(BECORE_YUVP_LPF_FIRST - BECORE_YUVP_PHYS_BASE == 0x5100);
static_assert((BECORE_YUVP_LPF_LAST - BECORE_YUVP_LPF_FIRST) / 4 + 1 ==
	      DIV_ROUND_UP(ARRAY_SIZE(becore_sharpen_lpf3),
			   BECORE_SHARPEN_TAPS_PER_REG) +
	      DIV_ROUND_UP(ARRAY_SIZE(becore_sharpen_lpf5),
			   BECORE_SHARPEN_TAPS_PER_REG) +
	      DIV_ROUND_UP(ARRAY_SIZE(becore_sharpen_lpf9),
			   BECORE_SHARPEN_TAPS_PER_REG));

static_assert(ARRAY_SIZE(becore_sharpen_lpf3) == 2 * 2);
static_assert(ARRAY_SIZE(becore_sharpen_lpf5) == 3 * 3);
static_assert(ARRAY_SIZE(becore_sharpen_lpf9) == 5 * 5);

/*
 * round(coefficient << q), away from zero, as a signed field of its own.
 *
 * The rounded numerator does not fit 32 bits for every table here -- the
 * inverse matrix below carries 419198 -- so it is formed at 64 bits rather
 * than left to a bound each new coefficient would have to be checked against.
 * Denominators stay small enough to divide by.
 */
static u32 becore_csc_coefficient(const struct becore_csc_coefficient *coef,
				  u32 q, u32 mask)
{
	s64 magnitude = coef->numerator < 0 ? -coef->numerator : coef->numerator;

	magnitude = div_s64(magnitude * (1 << q) * 2 + coef->denominator,
			    2 * coef->denominator);
	if (coef->numerator < 0)
		magnitude = -magnitude;

	return (u32)magnitude & mask;
}

static int becore_rgbp_csc_value(u32 offset, u32 *value)
{
	if (offset & 3)
		return -EINVAL;
	if (offset >= 0x04 && offset < 0x28) {
		u32 index = (offset - 0x04) / 4;

		*value = becore_csc_coefficient(&becore_csc_matrix[index / 3]
								 [index % 3],
						BECORE_RGBP_CSC_Q,
						BECORE_RGBP_CSC_FIELD_MASK);
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

/*
 * YUVP's two YUV-to-RGB matrices, by offset from either block's base. The
 * two copies are bit-identical, so one intent serves both.
 */
static int becore_yuvp_yuv2rgb_value(u32 offset, u32 *value)
{
	static const struct becore_csc_coefficient minus_half = { -1, 2 };
	u32 index;

	if (offset & 3)
		return -EINVAL;
	if (offset >= 0x08 && offset < 0x2c) {
		index = (offset - 0x08) / 4;
		/*
		 * The register order is input-slow and output-fast, as in the
		 * forward block. becore_clut_yuv2rgb is stored the other way
		 * from becore_csc_matrix -- [out][in] rather than [in][out] --
		 * so the two subscripts swap here and not in the forward path.
		 */
		*value = becore_csc_coefficient(&becore_clut_yuv2rgb[index % 3]
								    [index / 3],
						BECORE_YUVP_YUV2RGB_Q,
						BECORE_YUVP_YUV2RGB_FIELD_MASK);
		return 0;
	}

	switch (offset) {
	case 0x00:		/* BYPASS: the block runs */
	case 0x30:		/* OFFSET_0_0: luma carries no pedestal */
		*value = 0;
		return 0;
	case 0x2c:		/* LSHIFT */
		*value = BECORE_YUVP_YUV2RGB_LSHIFT;
		return 0;
	case 0x34:		/* OFFSET_0_1 */
	case 0x38:		/* OFFSET_0_2: half scale, subtracted */
		*value = becore_csc_coefficient(&minus_half,
						BECORE_YUVP_YUV2RGB_Q,
						BECORE_YUVP_YUV2RGB_OFFSET_MASK);
		return 0;
	}

	return -EINVAL;
}

/* The 4:2:0 converter, by offset from BECORE_YUVP_CSC420_BASE. */
static int becore_yuvp_csc420_value(u32 offset, u32 *value)
{
	static const struct becore_csc_coefficient half = { 1, 2 };
	u32 index;

	if (offset & 3)
		return -EINVAL;
	if (offset >= 0x04 && offset < 0x28) {
		index = (offset - 0x04) / 4;
		*value = becore_csc_coefficient(&becore_csc_matrix[index / 3]
								  [index % 3],
						BECORE_YUVP_CSC420_Q,
						BECORE_YUVP_CSC420_FIELD_MASK);
		return 0;
	}

	switch (offset) {
	case 0x00:		/* CTRL: not bypassed, and the output is 10-bit */
		*value = BECORE_YUVP_CSC420_CTRL_OUT10;
		return 0;
	case 0x28:		/* YMIN */
	case 0x30:		/* UMIN */
	case 0x38:		/* VMIN */
	case 0x44:		/* YOS: full range puts luma's offset at zero */
		*value = 0;
		return 0;
	case 0x2c:		/* YMAX */
	case 0x34:		/* UMAX */
	case 0x3c:		/* VMAX */
		*value = BECORE_YUVP_CSC420_MAX;
		return 0;
	case 0x40:		/* LS */
		*value = BECORE_YUVP_CSC420_LS;
		return 0;
	case 0x48:		/* UOS */
	case 0x4c:		/* VOS: half scale at this block's own Q */
		*value = becore_csc_coefficient(&half,
						BECORE_YUVP_CSC420_Q,
						BECORE_YUVP_CSC420_LIMIT_MASK);
		return 0;
	case 0x50:		/* UV_COEFF */
		*value = BECORE_YUVP_CSC420_UV_COEFF;
		return 0;
	case 0x54:		/* UV_ORDER: U first */
		*value = 0;
		return 0;
	case 0xa0:		/* VER_SAMPLING_POSITION */
		*value = BECORE_YUVP_CSC420_VER_SAMPLING;
		return 0;
	}

	return -EINVAL;
}

/*
 * The dither the 10-bit output needs, and the only thing that turns it on.
 * Both words come out of the 4:2:0 converter's own translator, which enables
 * the dither exactly when the output is narrower than the input -- here a
 * 14-bit accumulate feeding a 10-bit frame.
 */
static int becore_yuvp_dither420_value(u32 offset, u32 *value)
{
	switch (offset) {
	case 0x00:		/* BYPASS: the block runs */
		*value = 0;
		return 0;
	case 0x04:		/* CONTROL */
		*value = 1;
		return 0;
	}

	return -EINVAL;
}

/*
 * The colour LUT's gate and the matrix in front of its lattice, by offset from
 * BECORE_YUVP_CLUT_BASE.
 *
 * The lattice itself is not here and cannot be: 4,913 (R, G, B) nodes of a
 * (U, V) pair is per-camera tuning, and it belongs in a parameters buffer
 * rather than in this driver. Which is also why the gate says *bypass*: the
 * entries are the chroma the block outputs rather than an offset to it, so
 * there is no neutral fill that leaves the picture alone, and the only
 * identity this block has is not running. becore_params_value() clears the
 * bypass when a lattice arrives.
 *
 * The rest of it is stateable and is what the block is for: convert YUV to RGB
 * with the ordinary BT.601 inverse, and take the input curves as written.
 */
static int becore_yuvp_clut_value(u32 offset, u32 *value)
{
	u32 index;

	if (offset & 3)
		return -EINVAL;

	switch (offset) {
	case 0x000:		/* BYPASS: nothing until a lattice arrives */
		*value = 1;
		return 0;
	case 0x004:		/* EN_CONFIG: the matrix, and no input curve */
		*value = BECORE_CLUT_EN_MATRIX;
		return 0;
	}

	if (offset < 0x03c || offset > 0x04c)
		return -EINVAL;

	/*
	 * Two coefficients per register, the lower-numbered one in the low
	 * half. The ninth is the last, so the upper half of the fifth register
	 * has nothing to carry.
	 */
	index = (offset - 0x03c) / 4 * 2;
	*value = becore_csc_coefficient(&becore_clut_yuv2rgb[index / 3]
							    [index % 3],
					BECORE_CLUT_MATRIX_Q,
					BECORE_CLUT_MATRIX_FIELD_MASK);
	index++;
	if (index < BECORE_CLUT_MATRIX_COEFFICIENTS)
		*value |= becore_csc_coefficient(&becore_clut_yuv2rgb[index / 3]
								     [index % 3],
						 BECORE_CLUT_MATRIX_Q,
						 BECORE_CLUT_MATRIX_FIELD_MASK)
			  << 16;

	return 0;
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

static int
becore_rgbp_dns_geometry_value(const struct becore_raster *array,
			       u32 offset, u32 *value)
{
	s32 x;
	s32 y;

	switch (offset) {
	case 0x1a4:		/* BINNING: Q10, x in [0:13], y in [16:29] */
		*value = (BECORE_RGBP_DNS_BINNING_UNITY << 16) |
			 BECORE_RGBP_DNS_BINNING_UNITY;
		return 0;
	/*
	 * RADIAL_CENTER: 15-bit signed, x low, y high.  Plain `-(w / 2)`, and
	 * the `& ~1` this used to carry was Samsung's rather than Lyric's:
	 * `rgbp_hw_s_dns_size()` in the downstream driver masks the halved
	 * width to an even number and `lyric::TranslateByrDns` does not
	 * (`neg w8, w8, lsr #1` at `liblyric_iq.so` +0x14f73c, the same
	 * instruction `SetRaidalConfig` uses for `YUVNR`'s copy).  Lyric is
	 * what programs this hardware, and no captured array shows the
	 * difference: every one of them halves to an even number already.
	 */
	case 0x1c0:
		x = -(s32)(array->width >> 1);
		y = -(s32)(array->height >> 1);
		*value = ((y & BECORE_RGBP_DNS_CENTRE_MASK) << 16) |
			 (x & BECORE_RGBP_DNS_CENTRE_MASK);
		return 0;
	}

	return -EINVAL;
}

/*
 * A per-scene input, empty because mainline produces no scene.
 *
 * All 138 of these are zero in every one of the 426 captured programs, but
 * that is not the reason they are zero here: this driver has no face detector
 * and no segmentation producer, so an empty rectangle and an all-zero
 * confidence map are what the block reads by construction rather than because
 * the capture happened to have no faces in it. The ranges in the table above
 * are the statement of which registers those are; this only refuses one that
 * is not on a word boundary, since a mis-stated range would otherwise be
 * answered rather than refused.
 */
static int becore_yuvp_scene_input_value(u32 reg, u32 *value)
{
	if (reg & 3)
		return -EINVAL;

	*value = 0;

	return 0;
}

/*
 * Seed n of generator g is 11111 * ((g + n) mod 5 + 1), truncated to 16 bits,
 * and the pair sharing a register is (2w, 2w + 1) of that generator's five.
 */
static int becore_yuvp_noise_seed_value(u32 offset, u32 *value)
{
	u32 generator, word, half, packed = 0;

	if (offset > BECORE_YUVP_NOISE_SEED_LAST - BECORE_YUVP_NOISE_SEED_FIRST ||
	    offset % sizeof(u32))
		return -EINVAL;

	generator = offset / sizeof(u32) / 3;
	word = offset / sizeof(u32) % 3;

	for (half = 0; half < 2; half++) {
		u32 index = word * 2 + half;
		u32 step;

		if (index >= BECORE_YUVP_NOISE_SEEDS)
			break;	/* the third register's high half is reserved */

		step = (generator + index) % BECORE_YUVP_NOISE_SEEDS + 1;
		packed |= (BECORE_YUVP_NOISE_SEED_STEP * step & 0xffff) <<
			  (16 * half);
	}
	*value = packed;

	return 0;
}

/*
 * What a quadrant adds up to over the whole kernel: a tap counts four times
 * unless it is on the centre row or column, which the quadrant stores last.
 */
static u32 becore_sharpen_kernel_sum(const struct becore_sharpen_kernel *kernel)
{
	u32 total = 0;
	u32 row, column;

	if (kernel->span * kernel->span != kernel->taps)
		return 0;	/* refused by becore_yuvp_lpf_value() as well */

	for (row = 0; row < kernel->span; row++)
		for (column = 0; column < kernel->span; column++)
			total += (u32)kernel->quadrant[row * kernel->span +
						       column] *
				 (row == kernel->span - 1 ? 1 : 2) *
				 (column == kernel->span - 1 ? 1 : 2);

	return total;
}

/*
 * The local tone mapper's grid RDMA, by offset from YUVP's base.
 *
 * The vendor's numbers here are this driver's own allocation: 0x800 bytes a
 * row over 48 rows is the 96 KiB a 32 x 24 x 8 bilateral grid needs, and the
 * stride is the row. Deriving them from the allocation rather than replaying
 * them is what makes a DMA programmed to read past its own buffer impossible
 * to write by accident -- the address beside them is already relocated to
 * `becore->grid`, and this is the length that goes with it.
 */
static int becore_yuvp_grid_dma_value(u32 offset, u32 *value)
{
	switch (offset) {
	case 0x1c00:				/* STAT_RDMA_GRID_EN */
		*value = 1;
		return 0;
	case 0x1c10:				/* ..._DATA_FORMAT */
	case 0x1c4c:				/* ..._BUSINFO */
		*value = 0;
		return 0;
	case 0x1c20:				/* ..._WIDTH */
	case 0x1c28:				/* ..._IMG_STRIDE_1P */
		*value = BECORE_LTM_GRID_ROW_BYTES;
		return 0;
	case 0x1c24:				/* ..._HEIGHT */
		*value = BECORE_LTM_GRID_ROWS;
		return 0;
	}

	return -EINVAL;
}

/*
 * One knot of `YUVNR`'s luma-gain x grid, by index, called only with 0..31.
 *
 * The last one is not a knot but the *width of the last interval*, which is
 * the same thing `GAMMARGB`'s `X_PNTS_TBL` does and for the same reason: the
 * final knot is 4096 and the field is twelve bits, so what fits is
 * 4096 - 3840 = 256. The layout differs -- `GAMMARGB` gives the width a
 * register of its own where this packs it in the last pair's high half -- so
 * it is the convention that carries over, not the arrangement.
 */
static u32 becore_nr_luma_grid_knot(u32 knot)
{
	if (knot < BECORE_NR_LUMA_GRID_KNOTS - 1)
		return knot * BECORE_NR_LUMA_GRID_STEP;

	return BECORE_NR_LUMA_GRID_FULL_SCALE -
	       (BECORE_NR_LUMA_GRID_KNOTS - 2) * BECORE_NR_LUMA_GRID_STEP;
}

/*
 * One register of that grid: two knots, the lower-numbered one in the low
 * half.
 *
 * The curve sampled on the grid is a different thing and stays in the recipe:
 * `luma_gain_y` is a repeated float in the block's *dynamic* parameters, and
 * fifteen of its sixteen registers move frame to frame in the captures --
 * the sixteenth holds the curve's first two entries, which happen never to.
 */
static int becore_yuvp_nr_luma_grid(u32 offset, u32 *value)
{
	u32 index;

	if (offset & 3 ||
	    offset < BECORE_YUVP_NR_LUMA_GRID_FIRST - BECORE_YUVP_PHYS_BASE ||
	    offset > BECORE_YUVP_NR_LUMA_GRID_LAST - BECORE_YUVP_PHYS_BASE)
		return -EINVAL;

	index = (offset - (BECORE_YUVP_NR_LUMA_GRID_FIRST -
			   BECORE_YUVP_PHYS_BASE)) / 4 *
		BECORE_NR_LUMA_GRID_PER_REG;
	*value = becore_nr_luma_grid_knot(index) |
		 becore_nr_luma_grid_knot(index + 1) << 16;

	return 0;
}

/*
 * `YUVNR`'s radial geometry: the frame's shape rather than its tuning, so no
 * member of the parameters block could carry it and the driver derives it.
 *
 * Both registers describe the **crop** -- RGBP's DMSCCROP window -- and not
 * the raster YUVP reads. `RADIAL_CENTER` is the crop's centre in two 15-bit
 * fields, and `BINNING`'s two Q10 fields are the crop over the chain per axis,
 * which is what turns a pixel of the raster the block is processing into a
 * position in the crop the fall-off is defined over. The vendor takes the
 * scaler's ratios from the crop the same way, and the eighteen captured
 * readouts settle it: the array and the crop coincide on some of them and the
 * crop and the chain on others, so no single one of the three would do.
 *
 * The top three bits of `BINNING` are a **radius bucket**: `lower_bound` over
 * eight ascending radii compiled into `liblyric_iq.so` at 0x4a410, each about
 * root two times the last, against the crop's diagonal -- and then that index
 * **minus one**, floored at zero. The minus one is not a detail to leave out
 * of this comment: `BYR_DNS`'s lookup a few hundred lines up searches the same
 * kind of table and takes the plain index.
 *
 * Both halves of the lookup had to be read rather than assumed: the radius is
 * *rounded*, and the search counts edges strictly below it. Truncating and
 * counting `edge <= radius` gives the same answer for every geometry in the
 * captures, which is exactly why reading it mattered.
 *
 * All 18 captured readouts across three cameras reproduce both words exactly,
 * over crop-to-chain ratios from 1.0 to 2.0 and four of the eight bucket
 * values.
 */
static const u16 becore_yuvnr_radii[] = {
	650, 1150, 1618, 2296, 3243, 4578, 6474, 9190,
};

/*
 * The square is a u64 and the root comes from int_sqrt64(). becore_raster_
 * validate() now bounds the array to 32768 and the crop is cut out of it, so a
 * u32 square would no longer wrap -- 46,341 is where it would -- but the width
 * this takes is an argument rather than a raster, and a caller that reaches it
 * with something wider should get a bucket rather than a wrapped radius. The
 * correction below is exact
 * round-to-nearest -- `square - root * root > root` is `square >= root^2 +
 * root + 1`, which is `sqrt(square) >= root + 0.5` -- and a tie cannot happen,
 * since `(root + 0.5)^2` is never an integer.
 */
static u32 becore_yuvnr_radius_bucket(u32 width, u32 height)
{
	u64 square = (u64)width * width + (u64)height * height;
	u64 radius = int_sqrt64(square);
	u32 index;

	if (square - radius * radius > radius)
		radius++;
	for (index = 0; index < ARRAY_SIZE(becore_yuvnr_radii); index++)
		if (becore_yuvnr_radii[index] >= radius)
			break;

	return index ? index - 1 : 0;
}

/*
 * The sharpener's crop size and its chain-to-crop ratio. Reported for the
 * whole crop rather than the scaled raster because that is the frame the
 * tuning's spatial terms are expressed over, which is also why the step
 * exists: it is what converts one into the other.
 */
static int
becore_sharpen_geometry_value(const struct becore_raster *array,
			      const struct becore_raster *chain,
			      u32 reg, u32 *value)
{
	struct becore_rect crop;
	u32 chain_w = chain->width;
	u32 chain_h = chain->height;
	int ret;

	ret = becore_rgbp_crop(array, chain, &crop);
	if (ret)
		return ret;
	if (!chain_w || !chain_h)
		return -EINVAL;

	if (reg == BECORE_YUVP_SHARPEN_SENSOR_REG) {
		*value = (crop.height << 16) | crop.width;
		return 0;
	}
	if (reg == BECORE_YUVP_SHARPEN_STEP_REG) {
		u32 horizontal = DIV_ROUND_CLOSEST(BECORE_SHARPEN_STEP_Q *
						   crop.width, chain_w);
		u32 vertical = DIV_ROUND_CLOSEST(BECORE_SHARPEN_STEP_Q *
						 crop.height, chain_h);

		/*
		 * Q8 in sixteen bits, so the crop may be up to 255.996 times
		 * the chain and no more.  Masking a larger one keeps the low
		 * bits, exactly as YUVNR's binning would, and the result is a
		 * spatial step running at a fraction of its real rate with
		 * nothing saying so.
		 *
		 * Nothing can reach it today: binning is the same ratio at a
		 * sixteenth of this bound, so a crop this much larger than the
		 * chain is refused a few registers earlier in the same record
		 * walk.  Which is the reason to state it here rather than
		 * leave it to the order two unrelated words are evaluated in.
		 */
		if (horizontal > BECORE_SHARPEN_STEP_MASK ||
		    vertical > BECORE_SHARPEN_STEP_MASK)
			return -ERANGE;
		*value = ((vertical & BECORE_SHARPEN_STEP_MASK) << 16) |
			 (horizontal & BECORE_SHARPEN_STEP_MASK);
		return 0;
	}

	return -EINVAL;
}

int
becore_yuvnr_geometry_value(const struct becore_raster *array,
			    const struct becore_raster *chain,
			    u32 offset, u32 *value)
{
	struct becore_rect crop;
	int ret;
	s32 x;
	s32 y;

	ret = becore_rgbp_crop(array, chain, &crop);
	if (ret)
		return ret;

	switch (offset) {
	case BECORE_YUVNR_BINNING:
		x = DIV_ROUND_CLOSEST(BECORE_YUVNR_BINNING_Q * crop.width,
				      chain->width);
		y = DIV_ROUND_CLOSEST(BECORE_YUVNR_BINNING_Q * crop.height,
				      chain->height);
		/*
		 * Q10 in fourteen bits, so the crop may be up to 15.999 times
		 * the chain and no more.  Masking a larger one keeps the low
		 * bits: a crop 16.25 times the chain reads back as 0.25, a
		 * fall-off running the wrong way over the frame.
		 */
		if (x > BECORE_YUVNR_BINNING_MASK ||
		    y > BECORE_YUVNR_BINNING_MASK)
			return -ERANGE;
		*value = ((u32)x & BECORE_YUVNR_BINNING_MASK) |
			 (((u32)y & BECORE_YUVNR_BINNING_MASK) <<
			  BECORE_YUVNR_BINNING_Y_SHIFT) |
			 (becore_yuvnr_radius_bucket(crop.width, crop.height) <<
			  BECORE_YUVNR_BUCKET_SHIFT);
		return 0;
	case BECORE_YUVNR_RADIAL_CENTER:
		x = -(s32)(crop.width >> 1);
		y = -(s32)(crop.height >> 1);
		*value = (((u32)y & BECORE_YUVNR_CENTRE_MASK) << 16) |
			 ((u32)x & BECORE_YUVNR_CENTRE_MASK);
		return 0;
	}

	return -EINVAL;
}

/* One of YUVNR's fixed-output words, by offset from YUVP's base. */
static int becore_yuvp_nr_default(u32 offset, u32 *value)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_yuvnr_defaults); i++) {
		if (becore_yuvnr_defaults[i].offset != offset)
			continue;
		*value = becore_yuvnr_defaults[i].value;
		return 0;
	}

	return -EINVAL;
}

/* One of the sharpener's compiled-in defaults, by offset from YUVP's base. */
static int becore_yuvp_sharpen_default(u32 offset, u32 *value)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_sharpen_defaults); i++) {
		if (becore_sharpen_defaults[i].offset != offset)
			continue;
		*value = becore_sharpen_defaults[i].value;
		return 0;
	}

	return -EINVAL;
}

/* One register of a kernel's quadrant: two taps, the lower one in the low half. */
static int becore_yuvp_lpf_value(u32 offset, u32 *value)
{
	size_t i;

	if (offset & 3)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(becore_sharpen_kernels); i++) {
		const struct becore_sharpen_kernel *kernel =
			&becore_sharpen_kernels[i];
		u32 taps = kernel->taps;
		u32 registers = DIV_ROUND_UP(taps, BECORE_SHARPEN_TAPS_PER_REG);
		u32 index;

		/* A row naming another kernel's quadrant, caught before it is read. */
		if (kernel->span * kernel->span != taps)
			return -EINVAL;

		if (offset < kernel->first ||
		    offset >= kernel->first + registers * sizeof(u32))
			continue;

		index = (offset - kernel->first) / sizeof(u32) *
			BECORE_SHARPEN_TAPS_PER_REG;
		*value = kernel->quadrant[index];
		/* An odd quadrant leaves the last high half unused. */
		if (index + 1 < taps)
			*value |= (u32)kernel->quadrant[index + 1] << 16;
		return 0;
	}

	return -EINVAL;
}

/* log2 of each kernel's sum, packed at bits 0, 8 and 16. */
static int becore_yuvp_lpf_norm_value(u32 *value)
{
	u32 packed = 0;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_sharpen_kernels); i++) {
		u32 total = becore_sharpen_kernel_sum(&becore_sharpen_kernels[i]);

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

/*
 * YUVP's tone-curve grid: RGBP's own x grid, four times finer because this
 * block's values are Q14 where RGBP's are Q12.  One grid serves all three
 * channels, and the driver states it whether or not a curve has arrived.
 */
static int becore_yuvp_gamma_x(u32 index, u32 *x)
{
	int ret = becore_rgbp_gamma_knot(index, x);

	if (ret)
		return ret;
	*x <<= BECORE_YUVP_GAMMA_Q - BECORE_RGBP_GAMMA_Q;

	return 0;
}

/* Two points per register, the lower-numbered one in the low half. */
static int becore_yuvp_gamma_x_pair(u32 index, u32 *value)
{
	u32 low;
	u32 high;
	int ret;

	ret = becore_yuvp_gamma_x(index, &low);
	if (ret)
		return ret;
	ret = becore_yuvp_gamma_x(index + 1, &high);
	if (ret)
		return ret;
	*value = (high << 16) | low;

	return 0;
}

/* The 65th point as its distance from the 64th, as a magnitude. */
static int becore_yuvp_gamma_x_delta(u32 *value)
{
	u32 last;
	u32 prev;
	int ret;

	ret = becore_yuvp_gamma_x(BECORE_RGBP_GAMMA_KNOTS - 1, &last);
	if (ret)
		return ret;
	ret = becore_yuvp_gamma_x(BECORE_RGBP_GAMMA_KNOTS - 2, &prev);
	if (ret)
		return ret;
	*value = last > prev ? last - prev : prev - last;

	return 0;
}

/*
 * Where each channel's tone-curve table is, and which knot its first register
 * carries.
 *
 * Three things make this a table rather than arithmetic. The green table has a
 * four-register reserved hole in the middle of it, so it takes two entries and
 * the second one starts at knot 40. The last knot of every table is stored in
 * a register of its own, as its distance from the knot before it, because a
 * value of 1 << Q does not fit the field. And the three tables are not evenly
 * spaced, so nothing derives one from another.
 */
struct becore_yuvp_gamma_range {
	u32 first;		/* physical register, inclusive */
	u32 last;		/* inclusive; equal to first for one register */
	u8 channel;
	u8 knot;		/* the knot this range's first register holds */
	bool delta;		/* the last knot, as a distance */
};

#define BECORE_YUVP_GAMMA_LAST_KNOT	(EXYNOS_BECORE_GAMMA_POINTS - 1)

static const struct becore_yuvp_gamma_range becore_yuvp_gamma_tables[] = {
	{ BECORE_YUVP_GAMMA_R_FIRST, BECORE_YUVP_GAMMA_R_LAST, 0, 0, false },
	{ BECORE_YUVP_GAMMA_R_DELTA_REG, BECORE_YUVP_GAMMA_R_DELTA_REG, 0,
	  BECORE_YUVP_GAMMA_LAST_KNOT, true },
	{ BECORE_YUVP_GAMMA_G_LOW_FIRST, BECORE_YUVP_GAMMA_G_LOW_LAST, 1, 0,
	  false },
	{ BECORE_YUVP_GAMMA_G_HIGH_FIRST, BECORE_YUVP_GAMMA_G_HIGH_LAST, 1,
	  BECORE_YUVP_GAMMA_G_SPLIT_KNOT, false },
	{ BECORE_YUVP_GAMMA_G_DELTA_REG, BECORE_YUVP_GAMMA_G_DELTA_REG, 1,
	  BECORE_YUVP_GAMMA_LAST_KNOT, true },
	{ BECORE_YUVP_GAMMA_B_FIRST, BECORE_YUVP_GAMMA_B_LAST, 2, 0, false },
	{ BECORE_YUVP_GAMMA_B_DELTA_REG, BECORE_YUVP_GAMMA_B_DELTA_REG, 2,
	  BECORE_YUVP_GAMMA_LAST_KNOT, true },
};

/* Which table a register belongs to, and which knot it carries. */
static const struct becore_yuvp_gamma_range *
becore_yuvp_gamma_lookup(u32 reg, u32 *knot)
{
	size_t i;

	if (reg & 3)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(becore_yuvp_gamma_tables); i++) {
		const struct becore_yuvp_gamma_range *range =
			&becore_yuvp_gamma_tables[i];

		if (reg < range->first || reg > range->last)
			continue;
		*knot = range->delta ? range->knot :
			range->knot + (reg - range->first) / 4 *
			BECORE_GAMMA_KNOTS_PER_REG;
		return range;
	}

	return NULL;
}

/*
 * One register of the identity curve: out[i] == in[i] at every knot, which
 * for this block is the same table as the grid.
 *
 * This is what the block does when userspace has sent nothing, and it is a
 * claim about the block rather than a convenient fill.  Unlike the colour LUT,
 * whose entries *are* its output and whose neutral table therefore produces
 * grey, a gamma table maps an input to an output: every knot sits on y = x and
 * the interpolation between two of them is a straight line through both, so
 * the stage passes its input through as an asserted bypass would.  What it
 * does not do is look good -- what arrives here is very nearly linear light,
 * and the curve that makes it a picture is the one an IPA sends.
 */
static int becore_yuvp_gamma_identity(u32 reg, u32 *value)
{
	const struct becore_yuvp_gamma_range *range;
	u32 knot;

	range = becore_yuvp_gamma_lookup(reg, &knot);
	if (!range)
		return -EINVAL;
	if (range->delta)
		return becore_yuvp_gamma_x_delta(value);

	return becore_yuvp_gamma_x_pair(knot, value);
}

/*
 * One knot of DEGAMMARGB, which undoes RGBP's square-root encode.
 *
 * Normalised, the encode is a square root, so its inverse is a square. Both
 * blocks carry it in their own fixed point -- the encode at Q12, this at Q14
 * -- so from knot 15 up (0-based, as everywhere here) the inverse is
 * `DIV_ROUND_CLOSEST(x * x, 1 << 14)`, and that reproduces every captured
 * value there.
 *
 * Below knot 15 the curve is a straight line, and the line is not arbitrary:
 * it is the inverse of the *encode's own first segment*. That segment runs
 * from the origin to (x[1], becore_rgbp_gamma_encode(x[1])) -- 8 to 181,
 * since the encode is `round(sqrt(x << 12))` -- so the inverse over it has
 * slope x[1] / 181, and that is the toe a pure square cannot have: x squared
 * has zero slope at black and would quantise the deepest shadows away, while
 * the forward curve is straight across its own first segment, so its exact
 * inverse is straight there too.
 *
 * The two pieces do not meet. At knot 15 the toe gives 31.12 where the square
 * gives 30.25, and every captured program takes the square, so the join is by
 * knot index rather than at the point where the two cross -- which is at
 * x = 724.15, between knots 15 and 16. It shows up as an enlarged step rather
 * than a jump: the table reads 28, 30, 36 where a continued toe would have
 * given 28, 31, 34. Why the vendor's generator breaks there is not
 * established. What is established is that these 65 values are bit-identical
 * on three cameras and at all eighteen captured readouts.
 */
static int becore_yuvp_degamma_point(u32 index, u32 *value)
{
	u32 first;
	u32 slope;
	u32 x;
	int ret;

	ret = becore_yuvp_gamma_x(index, &x);
	if (ret)
		return ret;
	if (index >= BECORE_YUVP_DEGAMMA_TOE_KNOTS) {
		*value = DIV_ROUND_CLOSEST(x * x, 1 << BECORE_YUVP_GAMMA_Q);
		return 0;
	}
	ret = becore_rgbp_gamma_knot(1, &first);
	if (ret)
		return ret;
	/*
	 * The divisor is the encode's first knot rather than a literal, so
	 * that this stays the inverse of whatever the encode actually is. It
	 * is 181 while the grid's first step is 8; refuse a zero rather than
	 * divide by one, because a grid that started with a zero-length
	 * segment would have no inverse over it at all.
	 */
	slope = becore_rgbp_gamma_encode(first);
	if (!slope)
		return -EINVAL;
	*value = DIV_ROUND_CLOSEST(x * first, slope);

	return 0;
}

/* Two knots per register, the lower-numbered one in the low half. */
static int becore_yuvp_degamma_pair(u32 index, u32 *value)
{
	u32 low;
	u32 high;
	int ret;

	ret = becore_yuvp_degamma_point(index, &low);
	if (ret)
		return ret;
	ret = becore_yuvp_degamma_point(index + 1, &high);
	if (ret)
		return ret;
	*value = (high << 16) | low;

	return 0;
}

/* The 65th knot as its distance from the 64th, as a magnitude. */
static int becore_yuvp_degamma_delta(u32 *value)
{
	u32 last;
	u32 prev;
	int ret;

	ret = becore_yuvp_degamma_point(BECORE_RGBP_GAMMA_KNOTS - 1, &last);
	if (ret)
		return ret;
	ret = becore_yuvp_degamma_point(BECORE_RGBP_GAMMA_KNOTS - 2, &prev);
	if (ret)
		return ret;
	*value = last > prev ? last - prev : prev - last;

	return 0;
}

static int becore_yuvp_degamma_value(u32 reg, u32 *value)
{
	u32 offset = reg - BECORE_YUVP_DEGAMMA_BASE;

	if (offset & 3)
		return -EINVAL;

	switch (offset) {
	case 0x000:		/* BYPASS: the block runs */
	case 0x004:		/* PEDESTAL_EN */
		*value = 0;
		return 0;
	case 0x08c:		/* the curve's last-knot delta */
		return becore_yuvp_degamma_delta(value);
	case 0x250:		/* X_PNTS_TBL last-knot delta */
		return becore_yuvp_gamma_x_delta(value);
	}

	if (offset >= 0x00c && offset < 0x08c)
		return becore_yuvp_degamma_pair((offset - 0x00c) / 4 * 2,
						value);
	if (offset >= 0x1c0 && offset <= 0x1ec)
		return becore_yuvp_gamma_x_pair((offset - 0x1c0) / 4 * 2,
						value);
	if (offset >= 0x200 && offset < 0x250)
		return becore_yuvp_gamma_x_pair((offset - 0x200) / 4 * 2 + 24,
						value);

	return -EINVAL;
}

/*
 * The block's gates, its grid, and -- when no parameters block has carried a
 * curve -- the identity in its three tables.  A curve that has arrived is
 * substituted over this by becore_params_apply(), which runs after every
 * generated word is written.
 */
static int becore_yuvp_gamma_value(u32 reg, u32 *value)
{
	u32 offset = reg - BECORE_YUVP_GAMMA_BASE;

	if (offset & 3)
		return -EINVAL;

	switch (offset) {
	case 0x000:		/* BYPASS: the block runs */
	case 0x004:		/* PEDESTAL_EN */
		*value = 0;
		return 0;
	case 0x250:		/* X_PNTS_TBL last-knot delta */
		return becore_yuvp_gamma_x_delta(value);
	}

	if (offset >= 0x1c0 && offset <= 0x1ec)
		return becore_yuvp_gamma_x_pair((offset - 0x1c0) / 4 * 2,
						value);
	/*
	 * The four-register hole between points 23 and 24 is 0x1f0..0x1fc; no
	 * range covers it, so it is never asked for.
	 */
	if (offset >= 0x200 && offset < 0x250)
		return becore_yuvp_gamma_x_pair((offset - 0x200) / 4 * 2 + 24,
						value);
	if (reg >= BECORE_YUVP_GAMMA_R_FIRST &&
	    reg <= BECORE_YUVP_GAMMA_B_DELTA_REG)
		return becore_yuvp_gamma_identity(reg, value);

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
 * DJAG is the stage that crops and scales on this path, so what it hands down
 * is already the output raster: there is exactly one raster below it, and
 * POLY_SC0 and POST_PC0 map that raster onto itself. Their ratio is therefore
 * unity by construction rather than by measurement, and this cannot be
 * written as a ratio of two independent extents because there is no second
 * extent to name.
 *
 * Which means the poly-phase filter set below is pinned to x8/8 here whatever
 * the geometry, since the ratio selects it. That is right for this
 * configuration and would be wrong for one where DJAG is switched off and
 * POLY_SC0 does the scaling, which is what the vendor's own 640x480 request
 * does -- there it runs in the x5/8 band. Making that configuration
 * expressible means giving the chain a source raster of its own, and until
 * then the honest thing is to say so rather than to route unity through a
 * helper that looks like a derivation.
 *
 * The one thing that is per axis is which extent the register describes, so
 * take it from the matching one: a vertical register computed from a width
 * would be a real defect the moment the two differ.
 */
static int becore_mcsc_chain_ratio(const struct becore_raster *output,
				   bool vertical, u32 *ratio)
{
	u32 extent = vertical ? output->height : output->width;

	return becore_zoom_ratio(extent, extent, ratio);
}

/*
 * Samsung publishes its poly-phase coefficients because they are a function of
 * the scaling ratio rather than of the scene: get_scaler_coef_ver2() picks one
 * of seven sets by comparing the ratio against x8/8, x7/8 and so on down to
 * x2/8. All seven are here because a scaler that downscales needs whichever
 * one its ratio selects -- the rear ultrawide and the rear camera run at
 * unity, the front camera at 4096/4000 and its 640x480 mode at exactly 2x.
 *
 * These are Samsung's numbers verbatim from is-hw-api-mcscaler-v9_1.c, indexed
 * [set][tap][phase]. The MCSC table rather than the RGBP one, although the two
 * publish the same coefficients: RGBP's are at 2048 and Zuma's fields hold
 * 512, and quartering them reproduces two transcription errors that the
 * hardware contradicts -- horizontal x7/8 tap 0 phase 1, where Samsung's row
 * breaks its own monotone run, and horizontal x5/8 tap 0 phase 0, where it has
 * the wrong sign. Each error also corrupts whichever tap absorbs the
 * renormalisation, so each costs two words. MCSC publishes them already at
 * 512, so nothing is scaled or renormalised here and there is no residual.
 *
 * Checked against every capture: 54 coefficient words in each of eighteen
 * captured RGBP programs, over three cameras and seven sensor readouts,
 * exercising x8/8, x7/8, x5/8 and x4/8, all matching bit for bit.
 */
static const s16 becore_sc_v_coeff[BECORE_SC_SETS][BECORE_SC_V_TAPS][BECORE_SC_PHASES] = {
	[BECORE_SC_SET_X8_8] = {
		{     0,   -15,   -25,   -31,   -33,   -33,   -31,   -27,   -23 },
		{   512,   508,   495,   473,   443,   408,   367,   324,   279 },
		{     0,    20,    45,    75,   110,   148,   190,   234,   279 },
		{     0,    -1,    -3,    -5,    -8,   -11,   -14,   -19,   -23 },
	},
	[BECORE_SC_SET_X7_8] = {
		{    32,    17,     3,    -7,   -14,   -18,   -20,   -20,   -19 },
		{   448,   446,   437,   421,   399,   373,   343,   310,   275 },
		{    32,    55,    79,   107,   138,   170,   204,   240,   275 },
		{     0,    -6,    -7,    -9,   -11,   -13,   -15,   -18,   -19 },
	},
	[BECORE_SC_SET_X6_8] = {
		{    61,    46,    31,    19,     9,     2,    -3,    -7,    -9 },
		{   390,   390,   383,   371,   356,   337,   315,   291,   265 },
		{    61,    83,   106,   130,   156,   183,   210,   238,   265 },
		{     0,    -7,    -8,    -8,    -9,   -10,   -10,   -10,    -9 },
	},
	[BECORE_SC_SET_X5_8] = {
		{    85,    71,    56,    43,    32,    23,    16,     9,     5 },
		{   341,   341,   336,   328,   317,   304,   288,   271,   251 },
		{    86,   105,   124,   145,   166,   187,   209,   231,   251 },
		{     0,    -5,    -4,    -4,    -3,    -2,    -1,     1,     5 },
	},
	[BECORE_SC_SET_X4_8] = {
		{   104,    89,    76,    63,    52,    42,    33,    26,    20 },
		{   304,   302,   298,   293,   285,   275,   264,   251,   236 },
		{   104,   120,   136,   153,   170,   188,   205,   221,   236 },
		{     0,     1,     2,     3,     5,     7,    10,    14,    20 },
	},
	[BECORE_SC_SET_X3_8] = {
		{   118,   103,    90,    78,    67,    57,    48,    40,    33 },
		{   276,   273,   270,   266,   260,   253,   244,   234,   223 },
		{   118,   129,   143,   157,   171,   185,   199,   211,   223 },
		{     0,     7,     9,    11,    14,    17,    21,    27,    33 },
	},
	[BECORE_SC_SET_X2_8] = {
		{   127,   111,   100,    88,    78,    68,    59,    50,    43 },
		{   258,   252,   250,   247,   242,   237,   230,   222,   213 },
		{   127,   135,   147,   159,   171,   182,   193,   204,   213 },
		{     0,    14,    15,    18,    21,    25,    30,    36,    43 },
	},
};

static const s16 becore_sc_h_coeff[BECORE_SC_SETS][BECORE_SC_H_TAPS][BECORE_SC_PHASES] = {
	[BECORE_SC_SET_X8_8] = {
		{     0,    -2,    -4,    -5,    -6,    -6,    -6,    -6,    -5 },
		{     0,     8,    14,    20,    23,    25,    26,    25,    23 },
		{     0,   -25,   -46,   -62,   -73,   -80,   -83,   -82,   -78 },
		{   512,   509,   499,   482,   458,   429,   395,   357,   316 },
		{     0,    30,    64,   101,   142,   185,   228,   273,   316 },
		{     0,    -9,   -19,   -30,   -41,   -53,   -63,   -71,   -78 },
		{     0,     2,     5,     8,    12,    15,    19,    21,    23 },
		{     0,    -1,    -1,    -2,    -3,    -3,    -4,    -5,    -5 },
	},
	[BECORE_SC_SET_X7_8] = {
		{    12,     9,     7,     5,     3,     2,     1,     0,    -1 },
		{   -32,   -24,   -16,    -9,    -3,     2,     7,    10,    13 },
		{    56,    29,     6,   -14,   -30,   -43,   -53,   -60,   -65 },
		{   444,   445,   438,   426,   410,   390,   365,   338,   309 },
		{    52,    82,   112,   144,   177,   211,   244,   277,   309 },
		{   -32,   -39,   -46,   -52,   -58,   -63,   -66,   -66,   -65 },
		{    12,    13,    14,    15,    16,    16,    16,    15,    13 },
		{     0,    -3,    -3,    -3,    -3,    -3,    -2,    -2,    -1 },
	},
	[BECORE_SC_SET_X6_8] = {
		{     8,     9,     8,     8,     8,     7,     7,     5,     5 },
		{   -44,   -40,   -36,   -32,   -27,   -22,   -18,   -13,    -9 },
		{   100,    77,    57,    38,    20,     5,    -9,   -20,   -30 },
		{   384,   382,   377,   369,   358,   344,   329,   310,   290 },
		{   100,   123,   147,   171,   196,   221,   245,   268,   290 },
		{   -44,   -47,   -49,   -49,   -48,   -47,   -43,   -37,   -30 },
		{     8,     8,     7,     5,     3,     1,    -2,    -5,    -9 },
		{     0,     0,     1,     2,     2,     3,     3,     4,     5 },
	},
	[BECORE_SC_SET_X5_8] = {
		{    -3,    -3,    -1,     0,     1,     2,     2,     3,     3 },
		{   -31,   -32,   -33,   -32,   -31,   -30,   -28,   -25,   -23 },
		{   130,   113,    97,    81,    66,    52,    38,    26,    15 },
		{   320,   319,   315,   311,   304,   296,   286,   274,   261 },
		{   130,   147,   165,   182,   199,   216,   232,   247,   261 },
		{   -31,   -29,   -26,   -22,   -17,   -11,    -3,     5,    15 },
		{    -3,    -6,    -8,   -11,   -13,   -16,   -18,   -21,   -23 },
		{     0,     3,     3,     3,     3,     3,     3,     3,     3 },
	},
	[BECORE_SC_SET_X4_8] = {
		{   -11,   -10,    -9,    -8,    -7,    -6,    -5,    -5,    -4 },
		{     0,    -4,    -7,   -10,   -12,   -14,   -15,   -16,   -17 },
		{   140,   129,   117,   106,    95,    85,    74,    64,    55 },
		{   255,   254,   253,   250,   246,   241,   236,   229,   222 },
		{   140,   151,   163,   174,   185,   195,   204,   214,   222 },
		{     0,     5,    10,    16,    22,    29,    37,    46,    55 },
		{   -12,   -13,   -14,   -15,   -16,   -16,   -17,   -17,   -17 },
		{     0,     0,    -1,    -1,    -1,    -2,    -2,    -3,    -4 },
	},
	[BECORE_SC_SET_X3_8] = {
		{    -5,    -5,    -5,    -5,    -5,    -5,    -5,    -5,    -5 },
		{    31,    27,    23,    19,    16,    12,    10,     7,     5 },
		{   133,   126,   119,   112,   105,    98,    91,    84,    78 },
		{   195,   195,   194,   193,   191,   189,   185,   182,   178 },
		{   133,   139,   146,   152,   158,   163,   169,   174,   178 },
		{    31,    37,    41,    47,    53,    59,    65,    71,    78 },
		{    -6,    -4,    -3,    -2,    -2,     0,     1,     3,     5 },
		{     0,    -3,    -3,    -4,    -4,    -4,    -4,    -4,    -5 },
	},
	[BECORE_SC_SET_X2_8] = {
		{    10,     9,     7,     6,     5,     4,     4,     3,     2 },
		{    52,    48,    45,    41,    38,    35,    31,    29,    26 },
		{   118,   114,   110,   106,   102,    98,    94,    89,    85 },
		{   152,   152,   151,   150,   149,   148,   146,   145,   143 },
		{   118,   122,   125,   129,   132,   135,   138,   140,   143 },
		{    52,    56,    60,    64,    68,    72,    77,    81,    85 },
		{    10,    11,    13,    15,    17,    19,    21,    23,    26 },
		{     0,     0,     1,     1,     1,     1,     1,     2,     2 },
	},
};

/*
 * The set get_scaler_coef_ver2() selects for a ratio. Its bands are
 * is-hw-api-mcscaler-v3.h's RATIO_X*_8: at or below the first is x8/8, and
 * anything past the last saturates at x2/8 rather than failing, because the
 * filter is a best fit and not a description of the scaling.
 */
static const u32 becore_sc_ratio_bands[BECORE_SC_SETS] = {
	1048576, 1198373, 1398101, 1677722, 2097152, 2796203, 4194304,
};

static u32 becore_sc_coeff_set(u32 ratio)
{
	u32 set;

	for (set = 0; set + 1 < BECORE_SC_SETS; set++)
		if (ratio <= becore_sc_ratio_bands[set])
			break;

	return set;
}

/* Two taps of one phase, the lower-numbered one in the low half. */
static int becore_sc_coeff_value(bool vertical, u32 ratio, u32 index,
				 u32 *value)
{
	u32 set = becore_sc_coeff_set(ratio);
	u32 taps = vertical ? BECORE_SC_V_TAPS : BECORE_SC_H_TAPS;
	u32 pairs = taps / 2;
	u32 phase = index / pairs;
	u32 pair = index % pairs;
	s16 low, high;

	if (phase >= BECORE_SC_PHASES)
		return -EINVAL;
	if (vertical) {
		low = becore_sc_v_coeff[set][pair * 2][phase];
		high = becore_sc_v_coeff[set][pair * 2 + 1][phase];
	} else {
		low = becore_sc_h_coeff[set][pair * 2][phase];
		high = becore_sc_h_coeff[set][pair * 2 + 1][phase];
	}
	*value = (((u32)high & BECORE_SC_COEFF_MASK) << 16) |
		 ((u32)low & BECORE_SC_COEFF_MASK);

	return 0;
}

/*
 * The ratio the scaler these coefficients belong to is programmed with. Taken
 * from the same place the driver takes the value it writes into the ratio
 * register, so the filter and the ratio cannot describe different scalings.
 */
static int becore_sc_ratio(const struct becore_device *becore,
			   enum becore_block_id id, bool vertical, u32 *ratio)
{
	u32 index;

	if (id == BECORE_RGBP) {
		index = vertical ? BECORE_RGBP_SC_V_RATIO :
				   BECORE_RGBP_SC_H_RATIO;
		return becore_rgbp_input_value(becore_rgbp_input_profile(becore),
					       &becore->array, &becore->chain,
					       index,
					       becore_rgbp_input_regs[index],
					       ratio);
	}
	if (id != BECORE_MCSC)
		return -EINVAL;
	return becore_mcsc_chain_ratio(&becore->scaled, vertical, ratio);
}

u32 becore_generated_word_count(enum becore_block_id id)
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
int becore_generated_tables_validate(struct device *dev)
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

int becore_generated_value(const struct becore_device *becore,
			   enum becore_block_id id, u32 reg, u32 *value)
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
			result = becore_pack_size(becore->array.height,
						  becore->array.width);
			break;
		case BECORE_GEN_NOISE_SLOPE:
		case BECORE_GEN_NOISE_SHIFT:
		case BECORE_GEN_NOISE_DOMAIN:
			if (becore_noise_value(becore, id, reg, table[i].kind,
					       &result))
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
		case BECORE_GEN_YUV2RGB:
			if (becore_yuvp_yuv2rgb_value(
				    reg - (reg >= BECORE_YUVP_YUV2RGB_ZUMA_BASE ?
					   BECORE_YUVP_YUV2RGB_ZUMA_BASE :
					   BECORE_YUVP_YUV2RGB_BASE),
				    &result))
				return -EINVAL;
			break;
		case BECORE_GEN_CSC420:
			if (becore_yuvp_csc420_value(reg -
						     BECORE_YUVP_CSC420_BASE,
						     &result))
				return -EINVAL;
			break;
		case BECORE_GEN_DITHER420:
			if (becore_yuvp_dither420_value(
				    reg - BECORE_YUVP_DITHER420_BASE, &result))
				return -EINVAL;
			break;
		case BECORE_GEN_SCALER_PHASE: {
			u32 h_ratio;
			u32 v_ratio;
			int err = becore_sc_ratio(becore, id, false, &h_ratio);

			if (!err)
				err = becore_sc_ratio(becore, id, true,
						      &v_ratio);
			if (!err)
				err = becore_scaler_phase_value(reg, h_ratio,
								v_ratio,
								&result);
			if (err)
				return err;
			break;
		}
		case BECORE_GEN_MCSC_INPUT_SIZE:
			if (reg == BECORE_MCSC_IN_WIDTH_REG)
				result = becore->chain.width;
			else if (reg == BECORE_MCSC_IN_HEIGHT_REG)
				result = becore->chain.height;
			else
				return -EINVAL;
			break;
		case BECORE_GEN_CHROMA_LPF:
			if (becore_rgbp_chroma_lpf_value(reg -
						BECORE_RGBP_CHROMA_LPF_BASE,
						&result))
				return -EINVAL;
			break;
		case BECORE_GEN_DNS_GEOMETRY: {
			u32 offset = reg - BECORE_RGBP_DNS_BASE;

			if (becore_rgbp_dns_geometry_value(&becore->array, offset,
							   &result))
				return -EINVAL;
			break;
		}
		case BECORE_GEN_DNS_BIQUAD:
			if (becore_byr_dns_biquad_value(&becore->array, NULL, &result))
				return -EINVAL;
			break;
		/*
		 * The block with nothing sent to it, as the sharpener is: its
		 * tuning the encode of a parameter set of all zeros, which the
		 * inverted `enable` turns into the denoiser bypassed.  A
		 * parameters block replaces both through the same table, so
		 * the bypass and the values cannot disagree.
		 */
		case BECORE_GEN_BYR_DNS:
			if (becore_byrdns_value(becore, NULL, reg, &result))
				return -EINVAL;
			break;
		/*
		 * And the demosaic, over the one parameter set in this driver
		 * that is not all zeros -- see becore_dmsc_neutral, and the
		 * measurement that settled it.
		 */
		case BECORE_GEN_BYR_DMSC:
			if (becore_dmsc_value(&becore_dmsc_neutral, reg,
					      &result))
				return -EINVAL;
			break;
		case BECORE_GEN_SHARPEN_DEFAULT:
			if (becore_yuvp_sharpen_default(
				    reg - BECORE_YUVP_PHYS_BASE, &result))
				return -EINVAL;
			break;
		case BECORE_GEN_NR_DEFAULT:
			if (becore_yuvp_nr_default(reg - BECORE_YUVP_PHYS_BASE,
						   &result))
				return -EINVAL;
			break;
		case BECORE_GEN_NR_GEOMETRY:
			if (becore_yuvnr_geometry_value(&becore->array, &becore->chain,
							reg - BECORE_YUVP_PHYS_BASE,
							&result))
				return -EINVAL;
			break;
		case BECORE_GEN_NR_LUMA_GRID:
			if (becore_yuvp_nr_luma_grid(reg -
						     BECORE_YUVP_PHYS_BASE,
						     &result))
				return -EINVAL;
			break;
		case BECORE_GEN_GRID_DMA:
			if (becore_yuvp_grid_dma_value(reg -
						       BECORE_YUVP_PHYS_BASE,
						       &result))
				return -EINVAL;
			break;
		case BECORE_GEN_YUVP_CHAIN_SIZE:
			result = becore_pack_size(becore->chain.width,
						  becore->chain.height);
			break;
		/*
		 * The block with nothing sent to it: bypassed, and its tuning
		 * the encode of a parameter set of all zeros. A parameters
		 * block replaces both -- the same flag clears the bypass and
		 * brings the values, so the two cannot disagree.
		 */
		case BECORE_GEN_SHARPEN:
			if (reg == BECORE_YUVP_SHARPEN_BYPASS_REG) {
				result = 1;
				break;
			}
			if (becore_sharpen_value(NULL, reg, &result))
				return -EINVAL;
			break;
		case BECORE_GEN_UNWRITTEN:
			result = 0;
			break;
		case BECORE_GEN_SHARPEN_GEOMETRY:
			if (becore_sharpen_geometry_value(&becore->array, &becore->chain,
							  reg, &result))
				return -EINVAL;
			break;
		case BECORE_GEN_BAYER_PHASE: {
			int phase = becore_bayer_phase(becore->input_code);

			if (phase < 0)
				return -EINVAL;
			result = phase;
			break;
		}
		case BECORE_GEN_YUVNR:
			/*
			 * The same three the per-frame path uses, in the same
			 * order: the field table first, then the two
			 * derivations whose registers the table does not hold.
			 * So the default cannot diverge from what a buffer
			 * holding the same values would produce.
			 *
			 * The noise reducer's default is not the sharpener's
			 * all-zero parameter set, and cannot be: a domain of
			 * eight zeros has no slopes, and
			 * becore_params_check_yuvnr() refuses those same eight
			 * from userspace. becore_yuvnr_off carries the
			 * vendor's own curve there and zero everywhere else.
			 */
			if (becore_yuvnr_value(&becore_yuvnr_off, reg,
					       &result) &&
			    becore_yuvnr_tnr_lut_x(&becore_yuvnr_off, reg,
						   &result) &&
			    becore_yuvnr_tnr_slope(&becore_yuvnr_off, reg,
						   &result))
				return -EINVAL;
			break;
		case BECORE_GEN_LPF:
			if (becore_yuvp_lpf_value(reg - BECORE_YUVP_PHYS_BASE,
						  &result))
				return -EINVAL;
			break;
		case BECORE_GEN_LPF_NORM:
			if (becore_yuvp_lpf_norm_value(&result))
				return -EINVAL;
			break;
		case BECORE_GEN_SCENE_INPUT:
			if (becore_yuvp_scene_input_value(reg, &result))
				return -EINVAL;
			break;
		case BECORE_GEN_NOISE_SEED:
			if (becore_yuvp_noise_seed_value(
				    reg - BECORE_YUVP_NOISE_SEED_FIRST, &result))
				return -EINVAL;
			break;
		case BECORE_GEN_GAMMA:
			if (becore_rgbp_gamma_value(reg -
						    BECORE_RGBP_GAMMA_BASE,
						    &result))
				return -EINVAL;
			break;
		case BECORE_GEN_YUVP_GAMMA:
			if (becore_yuvp_gamma_value(reg, &result))
				return -EINVAL;
			break;
		case BECORE_GEN_YUVP_DEGAMMA:
			if (becore_yuvp_degamma_value(reg, &result))
				return -EINVAL;
			break;
		case BECORE_GEN_GTM:
			if (becore_rgbp_gtm_value(reg - BECORE_RGBP_GTM_BASE,
						  &result))
				return -EINVAL;
			break;
		case BECORE_GEN_LTM:
			if (becore_yuvp_ltm_value(&becore->chain,
						  reg - BECORE_YUVP_LTM_BASE,
						  &result))
				return -EINVAL;
			break;
		case BECORE_GEN_CLUT_1DLUT:
			if (becore_yuvp_clut_1dlut_value(reg - table[i].first,
							 &result))
				return -EINVAL;
			break;
		case BECORE_GEN_INVCCM33:
			if (becore_yuvp_invccm33_value(reg -
						BECORE_YUVP_INVCCM33_FIRST,
						&result))
				return -EINVAL;
			break;
		case BECORE_GEN_CCM:
			if (becore_yuvp_ccm_value(reg - BECORE_YUVP_CCM_BASE,
						  &result))
				return -EINVAL;
			break;
		case BECORE_GEN_CLUT:
			if (becore_yuvp_clut_value(reg -
						   BECORE_YUVP_CLUT_BASE,
						   &result))
				return -EINVAL;
			break;
		case BECORE_GEN_SC_V_COEFF:
		case BECORE_GEN_SC_H_COEFF: {
			bool vertical = table[i].kind == BECORE_GEN_SC_V_COEFF;
			u32 ratio;

			if (becore_sc_ratio(becore, id, vertical, &ratio))
				return -EINVAL;
			if (becore_sc_coeff_value(vertical, ratio,
						  (reg - table[i].first) / 4,
						  &result))
				return -EINVAL;
			break;
		}
		case BECORE_GEN_CHAIN_ORIGIN:
			result = becore_pack_size(0, 0);
			break;
		case BECORE_GEN_CHAIN_SIZE:
			result = becore_pack_size(becore->scaled.width,
						  becore->scaled.height);
			break;
		case BECORE_GEN_CHAIN_RATIO: {
			bool vertical = reg == BECORE_MCSC_SC0_V_RATIO_REG ||
					reg == BECORE_MCSC_PC0_V_RATIO_REG;
			int err = becore_mcsc_chain_ratio(&becore->scaled,
							  vertical, &result);

			if (err)
				return err;
			break;
		}
		default:
			return -EINVAL;
		}
		if (value)
			*value = result;

		return 0;
	}

	return -EINVAL;
}

/*
 * One register of one channel's tone curve, packed from the samples userspace
 * sent, or -ENOENT if this register is not part of a table.
 *
 * The curve was checked non-decreasing at buf_prepare, which is what makes the
 * last knot's magnitude enough: the direction lives in a _DELTA_SIGN register
 * that stays unwritten, so a falling curve would encode as a rising one.
 */
int becore_yuvp_gamma_curve_value(const struct becore_params_state *params,
				  u32 reg, u32 *value)
{
	const struct becore_yuvp_gamma_range *range;
	const u16 *curve;
	u32 knot;

	range = becore_yuvp_gamma_lookup(reg, &knot);
	if (!range)
		return -ENOENT;
	curve = params->gamma[range->channel];
	/*
	 * A magnitude, as the driver's other two delta encoders take one: the
	 * curve was checked non-decreasing, so this is the difference, and
	 * taking it this way means a curve that somehow fell would encode a
	 * small number rather than wrap.
	 */
	if (range->delta)
		*value = curve[knot] > curve[knot - 1] ?
			 curve[knot] - curve[knot - 1] :
			 curve[knot - 1] - curve[knot];
	else
		*value = curve[knot] | ((u32)curve[knot + 1] << 16);

	return 0;
}
