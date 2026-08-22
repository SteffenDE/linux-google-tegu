/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Google zumapro camera back-end core (RGBP/YUVP/MCSC) ISP parameters
 *
 * Copyright (C) 2026 Steffen Deusch <steffen@deusch.me>
 */

#ifndef __UAPI_EXYNOS_BECORE_CONFIG_H
#define __UAPI_EXYNOS_BECORE_CONFIG_H

#include <linux/media/v4l2-isp.h>
#include <linux/types.h>

/**
 * enum exynos_becore_params_block_type - Parameters block type
 *
 * @EXYNOS_BECORE_PARAM_BLOCK_CCM: The colour-correction matrix applied in YUVP,
 *	:c:type:`exynos_becore_params_ccm`
 * @EXYNOS_BECORE_PARAM_BLOCK_LTM_CURVE: The local tone mapper's guide curve,
 *	:c:type:`exynos_becore_params_ltm_curve`
 * @EXYNOS_BECORE_PARAM_BLOCK_CLUT: The colour LUT's chroma lattice,
 *	:c:type:`exynos_becore_params_clut`
 * @EXYNOS_BECORE_PARAM_BLOCK_GAMMA: The output tone curve, one per channel,
 *	:c:type:`exynos_becore_params_gamma`
 * @EXYNOS_BECORE_PARAM_BLOCK_SENTINEL: Not a block type; the number of them
 *
 * None of these is anything the kernel could know: the matrix is white
 * balance's own output, the guide curve is the exposure estimate's, the
 * lattice comes from a tuning tree indexed by the illuminant estimate, and the
 * tone curve is the grade a calibration ships. What none of them carries is a
 * register address or a program: userspace supplies values in the units the
 * block is specified in, and the driver encodes them.
 */
enum exynos_becore_params_block_type {
	EXYNOS_BECORE_PARAM_BLOCK_CCM = 0,
	EXYNOS_BECORE_PARAM_BLOCK_LTM_CURVE,
	EXYNOS_BECORE_PARAM_BLOCK_CLUT,
	EXYNOS_BECORE_PARAM_BLOCK_GAMMA,
	EXYNOS_BECORE_PARAM_BLOCK_SENTINEL,
};

#define EXYNOS_BECORE_CCM_COEFFICIENTS		9
#define EXYNOS_BECORE_CCM_OFFSETS		3

/*
 * Q11: 2048 is 1.0. The hardware field is the full width of the type -- every
 * captured program writes a plain 16-bit two's complement value -- so the
 * range is the range of __s16 and the row sum below is the real constraint.
 */
#define EXYNOS_BECORE_CCM_ONE			2048

/**
 * struct exynos_becore_params_ccm - Colour-correction matrix
 *
 * @header: The parameters block header
 * @matrix: Nine Q11 coefficients, row-major: the first three produce the
 *	first output channel from the three input channels, and so on
 * @offsets: Three Q11 offsets added after the matrix, one per output channel
 *
 * Each row must sum to exactly %EXYNOS_BECORE_CCM_ONE. That is what makes the
 * matrix preserve neutrals, it is what the vendor's own encoder enforces by
 * renormalising its third coefficient, and the driver rejects a block that
 * does not satisfy it -- a matrix that tints grey is far more likely to be an
 * arithmetic mistake in userspace than an intention. It is also the only
 * constraint on the coefficients' magnitude, which is deliberate: nothing
 * observed says the field is narrower than 16 bits, and inventing a limit
 * would refuse a matrix the hardware would have accepted.
 */
struct exynos_becore_params_ccm {
	struct v4l2_isp_params_block_header header;
	__s16 matrix[EXYNOS_BECORE_CCM_COEFFICIENTS];
	__s16 offsets[EXYNOS_BECORE_CCM_OFFSETS];
} __attribute__((aligned(8)));

#define EXYNOS_BECORE_LTM_CURVE_POINTS		128

/* Q15 over the unit interval; the hardware field is 16 bits. */
#define EXYNOS_BECORE_LTM_CURVE_ONE		32767

/**
 * struct exynos_becore_params_ltm_curve - Local tone mapper's guide curve
 *
 * @header: The parameters block header
 * @curve: 128 evenly spaced Q15 samples of the tone curve, ascending
 *
 * The curve must be non-decreasing and no sample may exceed
 * %EXYNOS_BECORE_LTM_CURVE_ONE. A tone curve that goes backwards inverts
 * contrast over that interval, which no tone mapper wants and which is what a
 * sign or ordering error in userspace looks like.
 *
 * Disabling this block (%V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) returns the curve
 * to whatever the driver's own default is rather than switching tone mapping
 * off: the block has no bypass that leaves a usable picture behind it.
 */
struct exynos_becore_params_ltm_curve {
	struct v4l2_isp_params_block_header header;
	__u16 curve[EXYNOS_BECORE_LTM_CURVE_POINTS];
} __attribute__((aligned(8)));

/* 17 nodes per axis, indexed by RGB, each holding one (U, V) chroma pair. */
#define EXYNOS_BECORE_CLUT_AXIS_NODES		17
#define EXYNOS_BECORE_CLUT_NODES \
	(EXYNOS_BECORE_CLUT_AXIS_NODES * EXYNOS_BECORE_CLUT_AXIS_NODES * \
	 EXYNOS_BECORE_CLUT_AXIS_NODES)

/*
 * A sample is clamp(round(f * 1024) + 512) of a signed chroma float, so 512 is
 * no chroma at all, and the hardware field is ten bits wide.
 */
#define EXYNOS_BECORE_CLUT_NEUTRAL		512
#define EXYNOS_BECORE_CLUT_MAX			1023

/**
 * struct exynos_becore_params_clut - The colour LUT's chroma lattice
 *
 * @header: The parameters block header
 * @lut_u: One blue-difference sample per node
 * @lut_v: One red-difference sample per node
 *
 * The block converts YUV to RGB, indexes a 17 x 17 x 17 lattice with that RGB
 * and takes a (U, V) pair out of it; luma is not an output. Nodes walk red
 * slowest and blue fastest, so node (r, g, b) is at index
 * ``(r * 17 + g) * 17 + b``.
 *
 * A sample is the chroma the block outputs rather than an offset to the chroma
 * arriving, which is why there is no identity lattice: an all-neutral one
 * outputs grey rather than leaving the picture alone. Sending no block, or
 * sending this one with %V4L2_ISP_PARAMS_FL_BLOCK_DISABLE, bypasses the stage
 * instead -- which is the driver's default, and is what passes the upstream
 * chroma through untouched.
 *
 * No sample may exceed %EXYNOS_BECORE_CLUT_MAX, and both ends of the grey
 * axis -- node (0, 0, 0) and node (16, 16, 16) -- must be
 * %EXYNOS_BECORE_CLUT_NEUTRAL in both arrays. Black and white have no hue,
 * every lattice the vendor ships is neutral at both, and a lattice that is not
 * is what a stream written one sample out of step looks like.
 */
struct exynos_becore_params_clut {
	struct v4l2_isp_params_block_header header;
	__u16 lut_u[EXYNOS_BECORE_CLUT_NODES];
	__u16 lut_v[EXYNOS_BECORE_CLUT_NODES];
} __attribute__((aligned(8)));

#define EXYNOS_BECORE_GAMMA_POINTS		65
#define EXYNOS_BECORE_GAMMA_CHANNELS		3

/*
 * Q14 over the unit interval. The hardware field is fourteen bits, so only the
 * last knot may reach unity: it is the one the block stores as a distance from
 * its neighbour rather than as a value, precisely because it does not fit.
 */
#define EXYNOS_BECORE_GAMMA_ONE			16384
#define EXYNOS_BECORE_GAMMA_MAX			(EXYNOS_BECORE_GAMMA_ONE - 1)

/**
 * struct exynos_becore_params_gamma - The output tone curve
 *
 * @header: The parameters block header
 * @curve: 65 Q14 samples per channel, red, green and blue in that order,
 *	each non-decreasing
 *
 * This is the grade: the curve that takes the scene's linear light to the
 * output's tone. It is the last thing in the chain that shapes luma, and the
 * hardware holds one table per channel, so a white balance or a creative
 * split-tone can be carried here as well as a plain gamma.
 *
 * **The 65 samples are not evenly spaced.** The block samples a curve on a
 * fixed non-uniform grid, finest where a curve moves fastest: from zero, eight
 * steps of 32, then twelve of 64, eight of 128, sixteen of 256 and twenty of
 * 512, which reaches %EXYNOS_BECORE_GAMMA_ONE exactly at the sixty-fifth knot.
 * Sample the curve at those inputs. The grid is the hardware's rather than the
 * curve's and the driver states it; it does not resample what arrives here.
 *
 * Every sample must be at most %EXYNOS_BECORE_GAMMA_MAX except the last, which
 * may reach %EXYNOS_BECORE_GAMMA_ONE, and no channel may decrease. A tone
 * curve that goes backwards inverts contrast over that interval, and the
 * hardware could not encode it in any case: the last knot's direction lives in
 * a sign register the driver does not write. The last knot also reaches the
 * hardware as its distance from the one before it rather than as a value, and
 * that distance has to fit the same field, so a curve may not do almost all of
 * its rise in its final segment.
 *
 * Disabling this block (%V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) returns the curve to
 * the driver's own default rather than switching the stage off: the block has
 * no bypass this driver ever asserts, and the default is the identity, so the
 * stage keeps running and passes its input through. That is a picture in
 * very nearly linear light, not a pleasing one.
 */
struct exynos_becore_params_gamma {
	struct v4l2_isp_params_block_header header;
	__u16 curve[EXYNOS_BECORE_GAMMA_CHANNELS][EXYNOS_BECORE_GAMMA_POINTS];
} __attribute__((aligned(8)));

/**
 * define EXYNOS_BECORE_PARAMS_MAX_SIZE - Maximum parameters data size
 *
 * One of each block type. A buffer larger than this is refused, and one
 * smaller is fine: a frame only carries what changed.
 */
#define EXYNOS_BECORE_PARAMS_MAX_SIZE \
	(sizeof(struct exynos_becore_params_ccm) + \
	 sizeof(struct exynos_becore_params_ltm_curve) + \
	 sizeof(struct exynos_becore_params_clut) + \
	 sizeof(struct exynos_becore_params_gamma))

#endif /* __UAPI_EXYNOS_BECORE_CONFIG_H */
