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
 * @EXYNOS_BECORE_PARAM_BLOCK_SHARPEN: The sharpener's tuning,
 *	:c:type:`exynos_becore_params_sharpen`
 * @EXYNOS_BECORE_PARAM_BLOCK_YUVNR: The noise reducer's tuning,
 *	:c:type:`exynos_becore_params_yuvnr`
 * @EXYNOS_BECORE_PARAM_BLOCK_SENTINEL: Not a block type; the number of them
 *
 * None of these is anything the kernel could know: the matrix is white
 * balance's own output, the guide curve is the exposure estimate's, the
 * lattice comes from a tuning tree indexed by the illuminant estimate, the
 * tone curve is the grade a calibration ships, and the sharpener's and noise
 * reducer's tuning come from trees indexed by analog gain and exposure ratio.
 * What none of them carries is a register address or a program: userspace
 * supplies values in the units the block is specified in, and the driver
 * encodes them.
 */
enum exynos_becore_params_block_type {
	EXYNOS_BECORE_PARAM_BLOCK_CCM = 0,
	EXYNOS_BECORE_PARAM_BLOCK_LTM_CURVE,
	EXYNOS_BECORE_PARAM_BLOCK_CLUT,
	EXYNOS_BECORE_PARAM_BLOCK_GAMMA,
	EXYNOS_BECORE_PARAM_BLOCK_SHARPEN,
	EXYNOS_BECORE_PARAM_BLOCK_YUVNR,
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

/*
 * The sharpener carries 24 enables and 217 tuning values, and every one of
 * them is a `float` in the vendor's own tuning proto reaching the hardware as
 * ``round(f * scale)`` at one of sixteen fixed points.  Userspace does that
 * multiplication and this block carries the result: an integer at the
 * hardware's own scale, which is what the comment on each member names.  The
 * kernel has no FPU, so a float here would have to be decoded by hand; and the
 * scale *is* the unit, so a value without one would be meaningless.
 *
 * Every value is __s32 whatever its width, including the enables.  The
 * hardware fields are 1 to 20 bits wide and 38 of them are two's complement,
 * and a struct that mirrored those widths would carry six integer types, put
 * padding between them and make one wrong type a silent misread.  One type
 * costs a kilobyte and makes the whole block a flat sequence of values.
 *
 * **A value past the field it reaches saturates**, which is what the vendor's
 * own encoder does and is why no range appears below.  It is not a courtesy:
 * the phone's shipped tuning holds 2.0 for @noise_gain_lut at ordinary gains
 * and the hardware field is eight bits, so a calibration that fits nowhere
 * else is the normal case rather than a mistake.  Refusing a buffer for it
 * would mean every caller carried its own copy of the field widths, and where
 * those live is what this interface is for.
 */

/* Eight-entry gain curves, indexed by the luma the pixel sits at. */
#define EXYNOS_BECORE_SHARPEN_GAIN_LUT		8

/* Five independent hue windows, each with its own low and high edge. */
#define EXYNOS_BECORE_SHARPEN_DESAT_HUES	5

/**
 * struct exynos_becore_sharpen_setting - A threshold or limit, four ways
 * @negative_low: the low end of the range, for a negative-going signal
 * @negative_high: its high end, for a negative-going signal
 * @positive_low: the low end of the range, for a positive-going signal
 * @positive_high: its high end, for a positive-going signal
 *
 * The block treats an overshoot and an undershoot separately -- a halo above
 * an edge and a halo below it are not equally visible -- and interpolates
 * between the low and high ends over the detail's own strength.
 */
struct exynos_becore_sharpen_setting {
	__s32 negative_low;
	__s32 negative_high;
	__s32 positive_low;
	__s32 positive_high;
};

/**
 * struct exynos_becore_sharpen_extreme - The two ends of a range
 * @min: the low end
 * @max: the high end
 */
struct exynos_becore_sharpen_extreme {
	__s32 min;
	__s32 max;
};

/**
 * struct exynos_becore_sharpen_sign - One value per direction
 * @negative: for a negative-going signal
 * @positive: for a positive-going signal
 */
struct exynos_becore_sharpen_sign {
	__s32 negative;
	__s32 positive;
};

/**
 * struct exynos_becore_sharpen_brightness - One value per end of the tone range
 * @dark: applied in the shadows
 * @bright: applied in the highlights
 */
struct exynos_becore_sharpen_brightness {
	__s32 dark;
	__s32 bright;
};

/**
 * struct exynos_becore_sharpen_signal - One value per spatial band
 * @narrow: the finest band, from the 3x3 low-pass
 * @medium: the middle band, from the 5x5
 * @wide: the coarsest band, from the 9x9
 *
 * The block splits luma into three bands with three low-pass kernels and
 * treats each separately; those kernels are the driver's, not userspace's.
 */
struct exynos_becore_sharpen_signal {
	__s32 narrow;
	__s32 medium;
	__s32 wide;
};

/**
 * struct exynos_becore_sharpen_area - One value per kind of neighbourhood
 * @flat: where the detector found no structure
 * @edge: where it found an edge
 * @texture: where it found texture
 */
struct exynos_becore_sharpen_area {
	__s32 flat;
	__s32 edge;
	__s32 texture;
};

/**
 * struct exynos_becore_sharpen_content - One value per detected content class
 * @skin: skin
 * @skinb: the second skin class, which the vendor tunes separately
 * @sky: sky
 * @grass: foliage
 * @extra: a fifth class the tuning is free to define
 *
 * The classes are hue, saturation and value windows in the block's own HSV
 * conversion, so "sky" means the pixels this block's windows select and not
 * anything a segmentation network produced.
 */
struct exynos_becore_sharpen_content {
	__s32 skin;
	__s32 skinb;
	__s32 sky;
	__s32 grass;
	__s32 extra;
};

/**
 * struct exynos_becore_sharpen_hfarea - A high-frequency threshold pair
 * @texture: the amount of texture a pixel needs before the class applies
 * @edge: the amount of edge it needs
 */
struct exynos_becore_sharpen_hfarea {
	__s32 texture;
	__s32 edge;
};

/**
 * struct exynos_becore_sharpen_strength - What a content class does
 * @sharpening: how much the class scales sharpening
 * @noise: how much it scales the injected noise
 */
struct exynos_becore_sharpen_strength {
	__s32 sharpening;
	__s32 noise;
};

/**
 * struct exynos_becore_params_sharpen - The sharpener's tuning
 *
 * @header: The parameters block header
 * @enable_low_power: run the block in its reduced-power mode
 * @enable_texture_index: compute the texture index
 * @enable_edge_index: compute the edge index
 * @enable_gf_sharp: run the guided-filter sharpener
 * @enable_invs_halo: run halo suppression
 * @enable_wide_sharp: run the wide-band sharpener
 * @enable_wide_sharp_lc: run its local-contrast stage
 * @enable_radial_correction: fall the sharpening off towards the corners
 * @enable_noise: inject synthetic noise
 * @enable_noise_radial_correction: fall that noise off towards the corners
 * @enable_edge_gain: apply the flat/edge gain pair
 * @enable_texture_gain: apply the flat/texture gain pair
 * @enable_desat_gray: desaturate near-grey pixels
 * @enable_desat_edge: desaturate the hue windows below
 * @enable_content_detector: run the HSV content classifier at all
 * @enable_content_skin: let the skin class act
 * @enable_content_skinb: let the second skin class act
 * @enable_content_sky: let the sky class act
 * @enable_content_grass: let the foliage class act
 * @enable_content_extra: let the fifth class act
 * @enable_face_beautification: run the face path
 * @enable_face_beautification_sharp: let it change sharpening
 * @enable_face_beautification_noise: let it change noise
 * @enable_face_beautification_brightness: let it change brightness
 * @edge_index_dir_multiplier: how much a direction agrees before it counts
 * @medium_edge_thresholds: where the medium band calls a signal an edge
 * @narrow_edge_thresholds: the same for the narrow band
 * @medium_texture_thresholds: where the medium band calls a signal texture
 * @narrow_texture_thresholds: the same for the narrow band
 * @texture_gf_thresholds: the same for the guided filter's texture
 * @medium_edge_limits: how far the medium band may move an edge pixel
 * @narrow_edge_limits: the same for the narrow band
 * @medium_texture_limits: how far it may move a texture pixel
 * @narrow_texture_limits: the same for the narrow band
 * @texture_gf_limits: the same for the guided filter
 * @edge_limits: the overall limit on an edge pixel
 * @texture_limits: the overall limit on a texture pixel
 * @medium_edge_sharp_power_gain_lut: medium-band edge gain against luma
 * @narrow_edge_sharp_power_gain_lut: narrow-band edge gain against luma
 * @medium_texture_sharp_power_gain_lut: medium-band texture gain against luma
 * @narrow_texture_sharp_power_gain_lut: narrow-band texture gain against luma
 * @texture_gf_sharp_power_gain_lut: guided-filter gain against luma
 * @wide_sharp_lc_simple_power: the wide band's local-contrast strength
 * @wide_edge_sharp_powers: the wide band's edge gain, dark and bright
 * @wide_texture_sharp_powers: the wide band's texture gain, dark and bright
 * @edge_gain_for_flat: edge-index gain where the neighbourhood is flat
 * @texture_gain_for_flat: texture-index gain where it is flat
 * @edge_gain_for_edge: edge-index gain where the neighbourhood is an edge
 * @texture_gain_for_texture: texture-index gain where it is texture
 * @invs_halo_grade_offset: shifts where halo suppression starts to act
 * @invs_halo_texture_weight_offsets: shifts it per direction of the signal
 * @radial_sharp_power: how far sharpening falls off towards the corners
 * @radial_sharp_powers: the floor that fall-off stops at; @max is not read
 * @noise_gains: the injected noise's amplitude per band
 * @noise_shifts: the shift its generator is sampled at, per band
 * @noise_narrow_gains: the narrow band's noise gain per neighbourhood
 * @noise_medium_gains: the medium band's
 * @noise_wide_gains: the wide band's
 * @noise_gain_lut: noise gain against luma
 * @noise_power: the overall noise amplitude
 * @noise_radial_power: how far the noise falls off towards the corners
 * @noise_radial_powers: the floor and ceiling that fall-off works between
 * @content_hsv_hue_min: each class's hue window, low edge
 * @content_hsv_hue_max: each class's hue window, high edge
 * @content_hsv_sat_min: each class's saturation window, low edge
 * @content_hsv_sat_max: each class's saturation window, high edge
 * @content_hsv_val_min: each class's value window, low edge
 * @content_hsv_val_max: each class's value window, high edge
 * @content_skin_thresholds: how much detail a pixel needs to count as skin
 * @content_skinb_thresholds: the same for the second skin class
 * @content_sky_thresholds: the same for sky
 * @content_grass_thresholds: the same for foliage
 * @content_extra_thresholds: the same for the fifth class
 * @content_skin: what skin does to sharpening and noise
 * @content_skinb: what the second skin class does
 * @content_sky: what sky does
 * @content_grass: what foliage does
 * @content_extra: what the fifth class does
 * @desat_gray_sat_min: how unsaturated a pixel must be to count as grey
 * @desat_gray_val_min: how bright it must be
 * @desat_gray_threshold: where grey desaturation starts
 * @desat_gray_min: the saturation it leaves behind
 * @desat_gray_slope: how sharply it acts
 * @desat_gray_s_smooth_shift: how far the saturation edge is feathered
 * @desat_gray_s_level_shift: how far its level is quantised
 * @desat_gray_v_smooth_shift: the same for value
 * @desat_gray_v_level_shift: the same for value
 * @desat_edge_hue_mins: five hue windows, low edges
 * @desat_edge_hue_maxs: five hue windows, high edges
 * @desat_edge_sats: the saturation window those hues act in
 * @desat_edge_vals: the value window they act in
 * @desat_edge_threshold: where hue desaturation starts
 * @desat_edge_min: the saturation it leaves behind
 * @desat_edge_slope: how sharply it acts
 * @desat_edge_h_smooth_shift: how far the hue edge is feathered
 * @desat_edge_h_level_shift: how far its level is quantised
 * @desat_edge_s_smooth_shift: the same for saturation
 * @desat_edge_s_level_shift: the same for saturation
 * @desat_edge_v_smooth_shift: the same for value
 * @desat_edge_v_level_shift: the same for value
 * @face_sharp_level: what the face path does to sharpening
 * @face_noise_level: what it does to noise
 * @face_brightness_level: what it does to brightness
 * @face_brightness_gain: the gain that brightness is applied with
 * @radial_face_margin_gain: the reciprocal margin the radial fall-off uses
 *
 * `SHARPENHANCER` is the block that decides what the picture's detail looks
 * like: it splits luma into three spatial bands, calls each neighbourhood
 * flat, edge or texture, scales each band by that and by the pixel's luma,
 * limits how far any pixel may move, and injects synthetic noise so that the
 * result does not look plastic.  On top of that it classifies pixels by hue,
 * saturation and value into five content classes and lets each one change what
 * the rest of the block does.
 *
 * None of that is anything the kernel could know.  The vendor's own values are
 * a two-dimensional tuning tree indexed by **analog gain** and **exposure
 * ratio** -- so this block is per-frame in exactly the way white balance is,
 * and an IPA computes it from the sensor's own settings rather than from the
 * scene.  What the kernel supplies underneath is the block's bypass: send
 * nothing, or send this block with %V4L2_ISP_PARAMS_FL_BLOCK_DISABLE, and the
 * stage is switched off and passes its input through untouched.  That bypass
 * is measured rather than asserted -- forcing it changes the picture, softly
 * and only in the detail -- which is what makes it a defensible default and
 * not a value borrowed from one capture.
 *
 * Three things the block reads are **not** here, because mainline has no
 * producer for them: the 98-word confidence map, the five face rectangles and
 * the five region-of-interest rectangles.  The driver states those as the
 * empty they are.  So @face_sharp_level and its neighbours are carried, but
 * nothing selects a face for them to act on yet.
 */
struct exynos_becore_params_sharpen {
	struct v4l2_isp_params_block_header header;

	/* What runs at all. 0 or 1. */
	__s32 enable_low_power;
	__s32 enable_texture_index;
	__s32 enable_edge_index;
	__s32 enable_gf_sharp;
	__s32 enable_invs_halo;
	__s32 enable_wide_sharp;
	__s32 enable_wide_sharp_lc;
	__s32 enable_radial_correction;
	__s32 enable_noise;
	__s32 enable_noise_radial_correction;
	__s32 enable_edge_gain;
	__s32 enable_texture_gain;
	__s32 enable_desat_gray;
	__s32 enable_desat_edge;
	__s32 enable_content_detector;
	__s32 enable_content_skin;
	__s32 enable_content_skinb;
	__s32 enable_content_sky;
	__s32 enable_content_grass;
	__s32 enable_content_extra;
	__s32 enable_face_beautification;
	__s32 enable_face_beautification_sharp;
	__s32 enable_face_beautification_noise;
	__s32 enable_face_beautification_brightness;

	/* How much the directions have to agree. round(f * 128). */
	__s32 edge_index_dir_multiplier;

	/* Where a signal starts to count as detail. round(f * 1023). */
	struct exynos_becore_sharpen_setting medium_edge_thresholds;
	struct exynos_becore_sharpen_setting narrow_edge_thresholds;
	struct exynos_becore_sharpen_setting medium_texture_thresholds;
	struct exynos_becore_sharpen_setting narrow_texture_thresholds;
	struct exynos_becore_sharpen_setting texture_gf_thresholds;

	/* How far a pixel may be moved. round(f * 255). */
	struct exynos_becore_sharpen_setting medium_edge_limits;
	struct exynos_becore_sharpen_setting narrow_edge_limits;
	struct exynos_becore_sharpen_setting medium_texture_limits;
	struct exynos_becore_sharpen_setting narrow_texture_limits;
	struct exynos_becore_sharpen_setting texture_gf_limits;
	struct exynos_becore_sharpen_setting edge_limits;
	struct exynos_becore_sharpen_setting texture_limits;

	/* Gain against the pixel's own luma. round(f * 128). */
	__s32 medium_edge_sharp_power_gain_lut[EXYNOS_BECORE_SHARPEN_GAIN_LUT];
	__s32 narrow_edge_sharp_power_gain_lut[EXYNOS_BECORE_SHARPEN_GAIN_LUT];
	__s32 medium_texture_sharp_power_gain_lut[EXYNOS_BECORE_SHARPEN_GAIN_LUT];
	__s32 narrow_texture_sharp_power_gain_lut[EXYNOS_BECORE_SHARPEN_GAIN_LUT];
	__s32 texture_gf_sharp_power_gain_lut[EXYNOS_BECORE_SHARPEN_GAIN_LUT];

	/* The wide band's local contrast. round(f * 63). */
	__s32 wide_sharp_lc_simple_power;

	/* The wide band's gains, per end of the tone range. round(f * 255). */
	struct exynos_becore_sharpen_brightness wide_edge_sharp_powers;
	struct exynos_becore_sharpen_brightness wide_texture_sharp_powers;

	/* What a flat neighbourhood scales the two indices by. round(f * 255). */
	__s32 edge_gain_for_flat;
	__s32 texture_gain_for_flat;

	/* What a structured one scales them by. round(f * 511). */
	__s32 edge_gain_for_edge;
	__s32 texture_gain_for_texture;

	/* Where halo suppression starts to act. round(f * 1023). */
	__s32 invs_halo_grade_offset;
	struct exynos_becore_sharpen_sign invs_halo_texture_weight_offsets;

	/* How far sharpening falls off towards the corners. round(f * 1023). */
	__s32 radial_sharp_power;

	/* The floor that fall-off stops at. round(f * 2048). */
	struct exynos_becore_sharpen_extreme radial_sharp_powers;

	/* The injected noise's amplitude, per band. round(f * 16). */
	struct exynos_becore_sharpen_signal noise_gains;

	/* The shift its generator is sampled at, per band. round(f * 11). */
	struct exynos_becore_sharpen_signal noise_shifts;

	/* What each band's noise is scaled by. round(f * 128). */
	struct exynos_becore_sharpen_area noise_narrow_gains;
	struct exynos_becore_sharpen_area noise_medium_gains;
	struct exynos_becore_sharpen_area noise_wide_gains;
	__s32 noise_gain_lut[EXYNOS_BECORE_SHARPEN_GAIN_LUT];

	/* The overall noise amplitude. round(f * 8). */
	__s32 noise_power;

	/* How far the noise falls off towards the corners. round(f * 511). */
	__s32 noise_radial_power;

	/* The floor and ceiling that fall-off works between. round(f * 2048). */
	struct exynos_becore_sharpen_extreme noise_radial_powers;

	/* Each content class's hue window. round(f * 768). */
	struct exynos_becore_sharpen_content content_hsv_hue_min;
	struct exynos_becore_sharpen_content content_hsv_hue_max;

	/* Its saturation window. round(f * 1023). */
	struct exynos_becore_sharpen_content content_hsv_sat_min;
	struct exynos_becore_sharpen_content content_hsv_sat_max;

	/* Its value window. round(f * 2045). */
	struct exynos_becore_sharpen_content content_hsv_val_min;
	struct exynos_becore_sharpen_content content_hsv_val_max;

	/* How much detail a pixel needs before a class applies. round(f * 63). */
	struct exynos_becore_sharpen_hfarea content_skin_thresholds;
	struct exynos_becore_sharpen_hfarea content_skinb_thresholds;
	struct exynos_becore_sharpen_hfarea content_sky_thresholds;
	struct exynos_becore_sharpen_hfarea content_grass_thresholds;
	struct exynos_becore_sharpen_hfarea content_extra_thresholds;

	/* What the class then does. round(f * 32). */
	struct exynos_becore_sharpen_strength content_skin;
	struct exynos_becore_sharpen_strength content_skinb;
	struct exynos_becore_sharpen_strength content_sky;

	/* The same, at a coarser step. round(f * 8). */
	struct exynos_becore_sharpen_strength content_grass;
	struct exynos_becore_sharpen_strength content_extra;

	/* How unsaturated a pixel must be to count as grey. round(f * 1023). */
	__s32 desat_gray_sat_min;

	/* How bright it must be. round(f * 2045). */
	__s32 desat_gray_val_min;

	/* Where grey desaturation acts, and what it leaves. round(f * 1023). */
	__s32 desat_gray_threshold;
	__s32 desat_gray_min;

	/* How sharply it acts. round(f * 10). */
	__s32 desat_gray_slope;

	/* How far its edges are feathered and quantised. round(f * 8). */
	__s32 desat_gray_s_smooth_shift;
	__s32 desat_gray_s_level_shift;
	__s32 desat_gray_v_smooth_shift;
	__s32 desat_gray_v_level_shift;

	/* Five hue windows to desaturate. round(f * 768). */
	__s32 desat_edge_hue_mins[EXYNOS_BECORE_SHARPEN_DESAT_HUES];
	__s32 desat_edge_hue_maxs[EXYNOS_BECORE_SHARPEN_DESAT_HUES];

	/* The saturation window they act in. round(f * 1023). */
	struct exynos_becore_sharpen_extreme desat_edge_sats;

	/* The value window they act in. round(f * 2045). */
	struct exynos_becore_sharpen_extreme desat_edge_vals;

	/* Where hue desaturation acts, and what it leaves. round(f * 1023). */
	__s32 desat_edge_threshold;
	__s32 desat_edge_min;

	/* How sharply it acts. round(f * 10). */
	__s32 desat_edge_slope;

	/* How far its edges are feathered and quantised. round(f * 8). */
	__s32 desat_edge_h_smooth_shift;
	__s32 desat_edge_h_level_shift;
	__s32 desat_edge_s_smooth_shift;
	__s32 desat_edge_s_level_shift;
	__s32 desat_edge_v_smooth_shift;
	__s32 desat_edge_v_level_shift;

	/* What the face path does where a face is. round(f * 32). */
	__s32 face_sharp_level;
	__s32 face_noise_level;
	__s32 face_brightness_level;

	/* The gain brightness is applied with. round(f * 1024). */
	__s32 face_brightness_gain;

	/* The reciprocal margin the radial fall-off uses. round(f * 1048575). */
	__s32 radial_face_margin_gain;
} __attribute__((aligned(8)));

/*
 * The noise reducer carries 19 enables and 174 tuning values, and every one of
 * them is a `float` in the vendor's own tuning proto reaching the hardware as
 * ``round(f * scale)`` at one of nine fixed points.  Userspace does that
 * multiplication and this block carries the result, exactly as the sharpener's
 * does: an integer at the hardware's own scale, which is what the comment
 * above each member names.  Every value is __s32 whatever its width, so the
 * block is a flat sequence with no padding in it.
 *
 * **A value past the field it reaches saturates**, which is what the vendor's
 * own translators do.
 *
 * Three things about this block are not the sharpener's, and each of them is
 * why a member below exists or does not.
 *
 * **The block's slope tables are not here, because they are not tuning.**  The
 * hardware wants a slope and a shift per segment of each noise curve, and the
 * vendor computes both from the knots; the driver does the same arithmetic
 * from @std_lut_x and the range beside it, and from @mcfp_gain_lut_x and
 * @mcfp_gain_lut_y for the temporal curve.  Sending a slope would mean sending
 * something that has to agree with the knots beside it, and nothing would
 * check that it did.  The same goes for the luma curve's last interval -- see
 * @luma_gain_y -- and for the six register knots @mcfp_gain_lut_x's five
 * become.
 *
 * The radial gain's *geometry* is absent for a different reason: it is the
 * frame's size rather than tuning, so no member here could carry it.  Those
 * two registers are still the ones the driver replays.
 *
 * **Two of the pyramid's arrays are at a different fixed point per level**,
 * because each level halves the signal range, so they are three members each
 * rather than an array of three.  Naming the levels is what lets the scale
 * stay a property of the member.
 *
 * **The two-coefficient vertical IIR mode is not exposed.**  Turning it on
 * makes six registers read a different set of tuning values -- the ``_up``
 * variants of the vertical and horizontal sigmas, a second filter length and
 * two row limits.  Every one of the 34 shipped tuning files that has this
 * block has the mode off, and no captured program uses it, so the alternative
 * has nothing to check an implementation against.  The block therefore
 * describes the mode the hardware is actually run in, and @lfnr_enable's own
 * gating -- which is a gate and not a mode -- is reproduced exactly.
 */

/* Eight-knot noise curves against luma, shared by the Y and UV axes. */
#define EXYNOS_BECORE_YUVNR_STD_LUT_POINTS	8

/* Three-tap low-pass kernels, and the three widths the long filter takes. */
#define EXYNOS_BECORE_YUVNR_TAPS		3

/* The guided filter's epsilon, one per vertical level. */
#define EXYNOS_BECORE_YUVNR_GF_EPS_POINTS	3

/* The luma-gain curve, sampled on a 32-point grid of luma. */
#define EXYNOS_BECORE_YUVNR_LUMA_GAIN_POINTS	32

/* The temporal filter's gain curve. */
#define EXYNOS_BECORE_YUVNR_MCFP_LUT_POINTS	6

/*
 * Its domain is one knot shorter than its range: the hardware wants six knots
 * and the first of them is a constant the block does not carry.
 */
#define EXYNOS_BECORE_YUVNR_MCFP_LUT_X_POINTS	5

/* Nine directional gains, one per tap of the vertical or horizontal filter. */
#define EXYNOS_BECORE_YUVNR_GAIN_POINTS		9

/**
 * struct exynos_becore_params_yuvnr - The noise reducer's tuning
 *
 * @header: The parameters block header
 * @enable: run the block at all; zero bypasses it
 * @hfnr_enable: run the high-frequency noise reducer
 * @lfnr_enable: run the low-frequency noise reducer. Zero also forces
 *	@luma_gain_enable, @add_hf_y_enable and @add_hf_uv_enable off, which is
 *	what the vendor's translators do and what the driver reproduces
 * @luma_gain_enable: apply @luma_gain_y against luma
 * @wavelet_nlm_relation_enable: let the wavelet stage see the non-local means
 *	stage's decision
 * @slope_new_enable: run the gradation detector
 * @mcfp_weight_map_enable: take the temporal filter's weight map as an input
 * @vert_long_th_radial_luma_enable: fall @vert_long_th off towards the corners
 * @fallback_hpf_y_th_radial_luma_enable: the same for @fallback_hpf_y_th
 * @fallback_long_y_th_radial_luma_enable: the same for @fallback_long_y_th
 * @noise_addback_y_enable: add luma noise back after denoising
 * @noise_addback_uv_enable: add chroma noise back
 * @noise_addback_th_radial_luma_enable: fall that addition off towards the
 *	corners
 * @add_hf_y_enable: add high-frequency luma detail back
 * @add_hf_uv_enable: add high-frequency chroma detail back
 * @return_noise_y_enable: run the luma noise-return stage
 * @return_noise_uv_enable: run the chroma one
 * @return_noise_adaptive_coring: let its coring follow the local signal
 * @return_noise_adaptive_power: let its amplitude follow the local signal
 * @std_lut_x: the luma each knot of the noise curves sits at, ascending and
 *	strictly increasing -- the driver divides by the gaps between them
 * @std_lut_y: the luma noise standard deviation at each knot
 * @std_lut_uv: the chroma noise standard deviation at each knot
 * @std_factor_y: overall scale on the luma curve
 * @std_factor_uv: overall scale on the chroma curve
 * @y_support: the luma filter's support radius
 * @uv_support: the chroma filter's
 * @slope_est_support: the gradation detector's
 * @lpf_for_y_denoise: which low-pass the luma denoiser runs on
 * @pattern_y: which of the four fixed dither patterns to use
 * @vert_filter_length: how many rows the vertical filter spans
 * @lpf_y: the luma low-pass kernel's three taps
 * @lpf_uv: the chroma low-pass kernel's three taps
 * @fallback_long_y_filter_width: the fallback long filter's three widths
 * @min_snr_sq_mean_ratio_y: how much of the luma mean square counts as signal
 * @min_snr_sq_mean_ratio_uv: the same for chroma
 * @min_snr_bin_width_y: the luma histogram's bin width
 * @min_snr_bin_width_uv: the chroma histogram's
 * @min_snr_pix_number_low_y: how few luma pixels a bin may hold and still count
 * @min_snr_pix_number_low_uv: the same for chroma
 * @min_snr_pix_number_high_y: how many it takes to count fully
 * @min_snr_pix_number_high_uv: the same for chroma
 * @radial_gain_a_y: the luma radial gain's first coefficient
 * @radial_gain_b_y: its second
 * @radial_gain_a_uv: the chroma radial gain's first coefficient
 * @radial_gain_b_uv: its second
 * @wavelet_th_vh_gain: gain on the wavelet threshold along the axes
 * @wavelet_th_di_gain: gain on it along the diagonals
 * @slope_new_thr_power: how sharply the gradation detector's threshold acts
 * @slope_new_diff_weight_coring_th: below which a gradient is ignored
 * @slope_new_diff_weight_gain: the gain applied above it
 * @slope_new_diff_weight_clipping_th: where that gain stops growing
 * @slope_new_weight_smooth_kernel: which smoothing kernel its weight uses
 * @mcfp_weight_mean: which mean the temporal filter's weight is taken over
 * @mcfp_weight_mean_in: any positive value selects the input-side mean; the
 *	hardware field is one bit and takes only whether this is positive
 * @mcfp_center_weight: how much the centre tap counts
 * @mcfp_y_hfnr_min_ratio: the floor on luma high-frequency noise reduction
 * @mcfp_uv_hfnr_min_ratio: the floor on the chroma one
 * @mcfp_gain_lut_x: where the temporal filter's gain curve is sampled, on the
 *	block's own seven-bit scale, and **truncated rather than rounded** --
 *	see the comment above the member. The hardware wants six knots and
 *	these are the last five: the first is a constant the driver states.
 *	Each is capped at **123 plus its own index**, except the last, which is
 *	capped at 127 instead, and then raised to its predecessor plus one if it
 *	does not exceed it -- which is what the vendor's own translator does, so
 *	a curve sent flat or falling arrives rising by one count a knot rather
 *	than being refused
 * @mcfp_gain_lut_y: the temporal filter's gain at each knot
 * @luma_gain_y: the noise gain against luma, on a 32-point grid. The hardware
 *	takes 31 of the 32 and then the **width of the last interval**, so the
 *	driver derives that from the last two entries -- which is why nothing
 *	reads @luma_gain_y[31] directly
 * @v_sigma_y: the vertical filter's luma sigma
 * @v_sigma_uv: its chroma sigma
 * @vertical_gf_eps: the vertical guided filter's epsilon per level
 * @v_gain: the vertical filter's nine tap gains
 * @wide_gain_y: the wide vertical filter's luma gain
 * @wide_gain_uv: its chroma gain
 * @vert_long_th_0: the long vertical filter's first threshold
 * @vert_long_th_1: its second
 * @vert_long_th_2: its third
 * @vert_long_gain: the long vertical filter's three gains
 * @h_sigma_y_level0: the horizontal filter's luma sigma at the finest level
 * @h_sigma_uv_level0: its chroma sigma there
 * @h_sigma_y_level1: the luma sigma one level coarser
 * @h_sigma_uv_level1: the chroma sigma there
 * @h_sigma_y_level2: the luma sigma at the coarsest level
 * @h_sigma_uv_level2: the chroma sigma there
 * @h_gain: the horizontal filter's nine tap gains
 * @h_coring_l1_0: the first level's first coring threshold
 * @h_coring_l1_1: its second
 * @h_coring_l1_2: its third
 * @h_coring_l2_0: the second level's first coring threshold
 * @h_coring_l2_1: its second
 * @h_coring_l2_2: its third
 * @h_coring_gain_uv_l1: how much of the first level's coring chroma takes
 * @h_coring_gain_uv_l2: the same for the second level
 * @fallback_hpf_y_th: where the high-pass fallback starts to act
 * @fallback_long_y_th: where the long fallback starts to act
 * @fallback_hpf_y_gain: how much the high-pass fallback then applies
 * @fallback_long_y_gain: how much the long fallback applies
 * @noise_addback_y_th: below which no luma noise is added back
 * @noise_addback_y_clip: the ceiling on how much is
 * @noise_addback_y_gain: the gain it is added at
 * @noise_addback_uv_th: below which no chroma noise is added back
 * @noise_addback_uv_clip: the ceiling on how much is
 * @noise_addback_uv_gain: the gain it is added at
 * @hf_y_coring_th: below which high-frequency luma detail is not added back
 * @hf_uv_coring_th: the same for chroma
 * @add_hf_uv_gain: the gain chroma detail is added back at
 * @return_noise_coring_threshold_y: below which no luma noise is returned
 * @return_noise_coring_threshold_uv: the same for chroma
 * @return_noise_power_y: the luma noise-return amplitude
 * @return_noise_power_uv: the chroma one
 * @return_noise_power_shifter_y: how far that amplitude is shifted down
 * @return_noise_power_shifter_uv: the same for chroma
 * @return_noise_limit_y: the ceiling on returned luma noise
 * @return_noise_limit_uv: the ceiling on returned chroma noise
 * @reserved: must be zero
 *
 * The values come from a tuning tree the vendor indexes by scaler ratio,
 * analog gain and exposure ratio, so this is a per-frame block rather than a
 * calibration: what it carries is what a tuning tree evaluates to for the
 * frame that is about to be processed.
 */
struct exynos_becore_params_yuvnr {
	struct v4l2_isp_params_block_header header;

	/* 0 or 1. */
	__s32 enable;
	__s32 hfnr_enable;
	__s32 lfnr_enable;
	__s32 luma_gain_enable;
	__s32 wavelet_nlm_relation_enable;
	__s32 slope_new_enable;
	__s32 mcfp_weight_map_enable;
	__s32 vert_long_th_radial_luma_enable;
	__s32 fallback_hpf_y_th_radial_luma_enable;
	__s32 fallback_long_y_th_radial_luma_enable;
	__s32 noise_addback_y_enable;
	__s32 noise_addback_uv_enable;
	__s32 noise_addback_th_radial_luma_enable;
	__s32 add_hf_y_enable;
	__s32 add_hf_uv_enable;
	__s32 return_noise_y_enable;
	__s32 return_noise_uv_enable;
	__s32 return_noise_adaptive_coring;
	__s32 return_noise_adaptive_power;

	/* Luma, on the block's own 12-bit scale. round(f * 1). */
	__s32 std_lut_x[EXYNOS_BECORE_YUVNR_STD_LUT_POINTS];

	/* Noise standard deviation over the unit interval. round(f * 4096). */
	__s32 std_lut_y[EXYNOS_BECORE_YUVNR_STD_LUT_POINTS];
	__s32 std_lut_uv[EXYNOS_BECORE_YUVNR_STD_LUT_POINTS];

	/* Scale on the curve above. round(f * 16). */
	__s32 std_factor_y;
	__s32 std_factor_uv;

	/* Counts and selectors, whose unit is the count. round(f * 1). */
	__s32 y_support;
	__s32 uv_support;
	__s32 slope_est_support;
	__s32 lpf_for_y_denoise;
	__s32 pattern_y;
	__s32 vert_filter_length;
	__s32 lpf_y[EXYNOS_BECORE_YUVNR_TAPS];
	__s32 lpf_uv[EXYNOS_BECORE_YUVNR_TAPS];
	__s32 fallback_long_y_filter_width[EXYNOS_BECORE_YUVNR_TAPS];

	/* A fraction of the mean square. round(f * 256). */
	__s32 min_snr_sq_mean_ratio_y;
	__s32 min_snr_sq_mean_ratio_uv;

	/* Histogram bins and pixel counts. round(f * 1). */
	__s32 min_snr_bin_width_y;
	__s32 min_snr_bin_width_uv;
	__s32 min_snr_pix_number_low_y;
	__s32 min_snr_pix_number_low_uv;
	__s32 min_snr_pix_number_high_y;
	__s32 min_snr_pix_number_high_uv;

	/* The radial gain's two coefficients. round(f * 1024). */
	__s32 radial_gain_a_y;
	__s32 radial_gain_b_y;
	__s32 radial_gain_a_uv;
	__s32 radial_gain_b_uv;

	/* Gains on the wavelet threshold. round(f * 64). */
	__s32 wavelet_th_vh_gain;
	__s32 wavelet_th_di_gain;

	/* The gradation detector's thresholds and gain. round(f * 256). */
	__s32 slope_new_thr_power;
	__s32 slope_new_diff_weight_coring_th;
	__s32 slope_new_diff_weight_gain;
	__s32 slope_new_diff_weight_clipping_th;

	/* Which smoothing kernel it uses. round(f * 1). */
	__s32 slope_new_weight_smooth_kernel;

	/* The temporal filter's selectors and centre weight. round(f * 1). */
	__s32 mcfp_weight_mean;
	__s32 mcfp_weight_mean_in;
	__s32 mcfp_center_weight;

	/* Floors on high-frequency noise reduction. round(f * 256). */
	__s32 mcfp_y_hfnr_min_ratio;
	__s32 mcfp_uv_hfnr_min_ratio;

	/*
	 * Where that curve's knots sit, on the block's own scale. round(f * 1).
	 * The hardware *truncates* this one -- and @std_lut_x above -- rather
	 * than rounding it, so a tuning value of 103.6 reaches the register as
	 * 103. Both are integers in every shipped tuning, which is why the
	 * distinction shows up nowhere until userspace sends a value that is
	 * not.
	 */
	__s32 mcfp_gain_lut_x[EXYNOS_BECORE_YUVNR_MCFP_LUT_X_POINTS];

	/* The temporal filter's gain curve. round(f * 4096). */
	__s32 mcfp_gain_lut_y[EXYNOS_BECORE_YUVNR_MCFP_LUT_POINTS];

	/* Noise gain against luma, as a plain gain. round(f * 1). */
	__s32 luma_gain_y[EXYNOS_BECORE_YUVNR_LUMA_GAIN_POINTS];

	/* The vertical filter's sigmas, epsilons and gains. round(f * 4096). */
	__s32 v_sigma_y;
	__s32 v_sigma_uv;
	__s32 vertical_gf_eps[EXYNOS_BECORE_YUVNR_GF_EPS_POINTS];
	__s32 v_gain[EXYNOS_BECORE_YUVNR_GAIN_POINTS];
	__s32 wide_gain_y;
	__s32 wide_gain_uv;

	/* The long vertical filter's first threshold. round(f * 4096). */
	__s32 vert_long_th_0;

	/*
	 * Its other two, which the hardware carries one bit narrower and so one
	 * fixed point coarser. round(f * 2048).
	 */
	__s32 vert_long_th_1;
	__s32 vert_long_th_2;

	/* Its three gains. round(f * 16). */
	__s32 vert_long_gain[EXYNOS_BECORE_YUVNR_TAPS];

	/*
	 * The horizontal filter's sigmas. Each level of the pyramid halves
	 * the signal range, so each level is at its own fixed point rather
	 * than a common one -- which is why these are six members and not two
	 * arrays of three.
	 */
	/* round(f * 4096). */
	__s32 h_sigma_y_level0;
	__s32 h_sigma_uv_level0;

	/* round(f * 8192). */
	__s32 h_sigma_y_level1;
	__s32 h_sigma_uv_level1;

	/* round(f * 16384). */
	__s32 h_sigma_y_level2;
	__s32 h_sigma_uv_level2;

	/* Its nine tap gains. round(f * 4096). */
	__s32 h_gain[EXYNOS_BECORE_YUVNR_GAIN_POINTS];

	/* The first coring level, whose first entry is one point finer than the
	 * other two. round(f * 8192).
	 */
	__s32 h_coring_l1_0;

	/* round(f * 4096). */
	__s32 h_coring_l1_1;
	__s32 h_coring_l1_2;

	/* The second coring level, one point coarser throughout. round(f * 16384). */
	__s32 h_coring_l2_0;

	/* round(f * 8192). */
	__s32 h_coring_l2_1;
	__s32 h_coring_l2_2;

	/* How much of that coring chroma takes. round(f * 8). */
	__s32 h_coring_gain_uv_l1;
	__s32 h_coring_gain_uv_l2;

	/* Where the two fallbacks start to act. round(f * 4096). */
	__s32 fallback_hpf_y_th;
	__s32 fallback_long_y_th;

	/* How much they then apply. round(f * 16). */
	__s32 fallback_hpf_y_gain;
	__s32 fallback_long_y_gain;

	/* The luma addback's threshold and ceiling. round(f * 4096). */
	__s32 noise_addback_y_th;
	__s32 noise_addback_y_clip;

	/* The gain it is added at. round(f * 256). */
	__s32 noise_addback_y_gain;

	/* The chroma addback's threshold and ceiling. round(f * 2048). */
	__s32 noise_addback_uv_th;
	__s32 noise_addback_uv_clip;

	/* The gain it is added at. round(f * 256). */
	__s32 noise_addback_uv_gain;

	/* High-frequency luma coring. round(f * 4096). */
	__s32 hf_y_coring_th;

	/* High-frequency chroma coring. round(f * 2048). */
	__s32 hf_uv_coring_th;

	/* The gain chroma detail is added back at. round(f * 256). */
	__s32 add_hf_uv_gain;

	/* Where the noise-return stage starts to act. round(f * 4096). */
	__s32 return_noise_coring_threshold_y;
	__s32 return_noise_coring_threshold_uv;

	/* Its amplitude. round(f * 1024). */
	__s32 return_noise_power_y;
	__s32 return_noise_power_uv;

	/* How far that amplitude is shifted down. round(f * 1). */
	__s32 return_noise_power_shifter_y;
	__s32 return_noise_power_shifter_uv;

	/* The ceiling on what it returns. round(f * 4096). */
	__s32 return_noise_limit_y;
	__s32 return_noise_limit_uv;

	/*
	 * Must be zero.  The block carries an odd number of values and every
	 * block is eight-byte aligned, so one word of it is padding either
	 * way -- naming it is what keeps the block a flat sequence userspace
	 * can write without knowing what the compiler did, and what lets a
	 * value take it later without changing the block's size.
	 */
	__s32 reserved;
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
	 sizeof(struct exynos_becore_params_gamma) + \
	 sizeof(struct exynos_becore_params_sharpen) + \
	 sizeof(struct exynos_becore_params_yuvnr))

#endif /* __UAPI_EXYNOS_BECORE_CONFIG_H */
