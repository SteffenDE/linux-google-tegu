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
 * @EXYNOS_BECORE_PARAM_BLOCK_SENTINEL: Not a block type; the number of them
 *
 * Both of these are per-frame results of an algorithm rather than a per-module
 * calibration: the matrix comes from white balance and the curve from the
 * exposure estimate, and both change frame to frame in a moving scene. What
 * they do *not* carry is a register address or a program: userspace supplies
 * values in the units the block is specified in, and the driver encodes them.
 */
enum exynos_becore_params_block_type {
	EXYNOS_BECORE_PARAM_BLOCK_CCM = 0,
	EXYNOS_BECORE_PARAM_BLOCK_LTM_CURVE,
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

/**
 * define EXYNOS_BECORE_PARAMS_MAX_SIZE - Maximum parameters data size
 *
 * One of each block type. A buffer larger than this is refused, and one
 * smaller is fine: a frame only carries what changed.
 */
#define EXYNOS_BECORE_PARAMS_MAX_SIZE \
	(sizeof(struct exynos_becore_params_ccm) + \
	 sizeof(struct exynos_becore_params_ltm_curve))

#endif /* __UAPI_EXYNOS_BECORE_CONFIG_H */
