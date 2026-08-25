/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Google zumapro camera front end (ISPFE/LMP) statistics
 *
 * Copyright (C) 2026 Steffen Deusch <steffen@deusch.me>
 */

#ifndef __UAPI_EXYNOS_ISPFE_CONFIG_H
#define __UAPI_EXYNOS_ISPFE_CONFIG_H

#include <linux/media/v4l2-isp.h>
#include <linux/types.h>

/**
 * enum exynos_ispfe_stats_version - ISPFE statistics buffer versioning
 *
 * @EXYNOS_ISPFE_STATS_VERSION_V1: First version of the layout below
 *
 * The driver writes this into every buffer it completes. A consumer that does
 * not recognise the value must not interpret the rest of the buffer.
 */
enum exynos_ispfe_stats_version {
	EXYNOS_ISPFE_STATS_VERSION_V1 = 1,
};

/**
 * DOC: ISPFE statistics measurement flags
 *
 * Which of the grids in :c:type:`exynos_ispfe_stats_buffer` the hardware
 * actually wrote for this frame. A grid whose flag is clear holds no result
 * and its contents are undefined -- a buffer completed while nothing is
 * capturing has no flags set at all.
 *
 * %EXYNOS_ISPFE_STATS_AWB:
 *	The auto white balance grid, :c:type:`exynos_ispfe_stats_awb`
 * %EXYNOS_ISPFE_STATS_AE:
 *	The post-shading auto exposure grid, :c:type:`exynos_ispfe_stats_ae`
 */
#define EXYNOS_ISPFE_STATS_AWB			(1U << 0)
#define EXYNOS_ISPFE_STATS_AE			(1U << 1)

/**
 * enum exynos_ispfe_params_block_type - Parameters block type
 *
 * @EXYNOS_ISPFE_PARAM_BLOCK_WHITE_BALANCE: LMP's white balance gains,
 *	:c:type:`exynos_ispfe_params_white_balance`
 * @EXYNOS_ISPFE_PARAM_BLOCK_METERING: What the two statistics grids count,
 *	:c:type:`exynos_ispfe_params_metering`
 * @EXYNOS_ISPFE_PARAM_BLOCK_SENTINEL: Not a block type; the number of them
 *
 * The front end applies white balance before it meters, so the gains are both
 * what balances the picture and what the exposure and white balance grids are
 * measured through. Nothing about them is knowable to the kernel: they are an
 * estimate of the illuminant, which is what an AWB algorithm exists to make.
 *
 * A block carries values in the units the block is specified in, never a
 * register, an address or a command.
 */
enum exynos_ispfe_params_block_type {
	EXYNOS_ISPFE_PARAM_BLOCK_WHITE_BALANCE = 0,
	EXYNOS_ISPFE_PARAM_BLOCK_METERING,
	EXYNOS_ISPFE_PARAM_BLOCK_SENTINEL,
};

/* The four gains, in the order the statistics grids sum their channels. */
#define EXYNOS_ISPFE_WB_RED			0
#define EXYNOS_ISPFE_WB_GREEN_RED		1
#define EXYNOS_ISPFE_WB_GREEN_BLUE		2
#define EXYNOS_ISPFE_WB_BLUE			3
#define EXYNOS_ISPFE_WB_GAINS			4

/* Unsigned Q12: 4096 is 1.0, and is what the greens are captured at. */
#define EXYNOS_ISPFE_WB_GAIN_ONE		4096

/*
 * The largest gain this interface accepts, and it is a **policy** rather than
 * a hardware limit -- state it as the latter and the next person will believe
 * the hardware stops here. The white balance field itself is twenty bits, so
 * the hardware would take a gain of 255.99998, and the vendor's own translator
 * clamps nothing.
 *
 * What this bound is, is the largest gain that can be described to *defect
 * pixel correction* as well: it takes the red and blue gains in a ten-bit
 * field, so 1023 * 32 + 15 is the last one that does not saturate there. The
 * vendor is content to saturate it and go on; this refuses instead, because a
 * white balance that silently stops correcting defects above a threshold
 * nobody stated is worse than one that says no. The green gains are not in
 * that path and are held to the same bound anyway, because a white balance
 * whose greens leave the range its red and blue are held to is a mistake
 * rather than a mode.
 *
 * The vendor derives its two quantisations independently from one float --
 * round(gain * 4096) for white balance and round(gain * 128) for defect pixel
 * correction -- where this derives the second from the first. The two can
 * differ by one at a rounding boundary, which is exactly where this bound
 * sits.
 */
#define EXYNOS_ISPFE_WB_GAIN_MAX		32751

/**
 * struct exynos_ispfe_params_white_balance - LMP's white balance gains
 *
 * @header: The parameters block header
 * @gains: Four unsigned Q12 gains, indexed by %EXYNOS_ISPFE_WB_RED and its
 *	siblings
 *
 * Each gain is at least one -- a gain of zero is a channel switched off rather
 * than balanced, which no white balance wants -- and at most
 * %EXYNOS_ISPFE_WB_GAIN_MAX.
 *
 * Disabling this block (%V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) returns the gains to
 * the driver's default rather than switching white balance off. The stage does
 * have an enable bit and this interface does not expose it: unity gains are
 * the same picture, and a block whose disable means two different things --
 * "put the default back" or "stop correcting" -- would need userspace to say
 * which.
 */
struct exynos_ispfe_params_white_balance {
	struct v4l2_isp_params_block_header header;
	__u32 gains[EXYNOS_ISPFE_WB_GAINS];
} __attribute__((aligned(8)));

/*
 * Thresholds are compared against the sample as **signed** sixteen-bit, which
 * the counts say rather than any document: the exposure grid ships with its
 * dark threshold at 0x8000 and reports nothing dark, where an unsigned reading
 * of 32768 would put every sample in the frame below it. The white balance
 * grid ships with zero and reports four fifths of a dim room dark, which is
 * the floor of the same domain.
 *
 * So %EXYNOS_ISPFE_METERING_EXCLUDE_NONE is the dark threshold that excludes
 * nothing and %EXYNOS_ISPFE_METERING_SAMPLE_MAX the saturation threshold that
 * does, and they are the two ends of one signed range.
 */
#define EXYNOS_ISPFE_METERING_SAMPLE_MAX	32767
#define EXYNOS_ISPFE_METERING_EXCLUDE_NONE	(-32768)

/*
 * The luma weights are unsigned Q8 in a nine-bit field, so a weight of two is
 * the most that can be described. The window they gate is a full unsigned
 * sixteen bits, because it compares a weighted sum of four samples rather than
 * one -- the two are different widths and the vendor's own accessors read them
 * that way.
 */
#define EXYNOS_ISPFE_METERING_LUMA_ONE		256
#define EXYNOS_ISPFE_METERING_LUMA_COEFF_MAX	511
#define EXYNOS_ISPFE_METERING_LUMA_MAX		65535

/**
 * struct exynos_ispfe_params_metering - What the two statistics grids count
 *
 * @header: The parameters block header
 * @awb_saturation_threshold: A white balance sample above this is counted
 *	saturated instead of summed
 * @awb_dark_threshold: A white balance sample below this -- or at it; the
 *	measurement does not separate the two -- is counted dark instead of
 *	summed. The vendor sets zero here and four fifths of a dim room's
 *	samples still come back dark, which says the floor of the domain is at
 *	or just above zero either way
 * @awb_luma_coeff: Unsigned Q8 weights on R, Gr, Gb and B -- indexed by
 *	%EXYNOS_ISPFE_WB_RED and its siblings -- forming the luma that
 *	@awb_luma_threshold_low and @awb_luma_threshold_high gate on. Each is at
 *	most %EXYNOS_ISPFE_METERING_LUMA_COEFF_MAX, which is the field's own
 *	width; a set that describes an ordinary luma sums to
 *	%EXYNOS_ISPFE_METERING_LUMA_ONE, and the driver does not require that,
 *	because a deliberately weighted metering is a use rather than a mistake
 * @awb_luma_threshold_low: The low end of the luma window
 * @awb_luma_threshold_high: The high end of it
 * @awb_diff_coring_threshold: A coring threshold on neighbour differences
 * @ae_saturation_threshold: The exposure grid's own saturation threshold
 * @ae_dark_threshold: The exposure grid's own dark threshold
 * @reserved: Must be zero
 *
 * The two grids exclude samples rather than clamping them -- which is what
 * makes a grey-world estimate over the white balance grid ignore both ends --
 * so the four sample thresholds decide what the numbers in
 * :c:type:`exynos_ispfe_stats_buffer` are a mean *of*. An algorithm that
 * cannot set them is metering through someone else's choice.
 *
 * **Only the four sample thresholds are known to reach that buffer.** Moving
 * either grid's saturation or dark threshold moves its counts and sums exactly
 * as the names say, measured. Moving @awb_luma_coeff, the window it feeds, or
 * @awb_diff_coring_threshold changes nothing this interface exposes -- with the
 * window placed over the scene's luma, far below it, and with coring at its
 * maximum, the white balance grid's counts and means do not move. They are real
 * fields at read offsets and the driver programs them, so they most likely gate
 * the per-colour quantities :c:type:`exynos_ispfe_stats_awb_region` still
 * carries as reserved; until something decodes those, a consumer has no way to
 * observe what they did, and this interface does not pretend otherwise.
 *
 * A dark threshold above its own saturation threshold would exclude every
 * sample in the frame, and is refused rather than metered.
 *
 * The two grids take their own thresholds and are not two views of one
 * setting: the exposure grid is metered after lens shading -- the hardware's
 * own name for its tap says so -- so the two see the same scene at different
 * levels.
 *
 * The vendor's own values are 32256 and 0 for the white balance grid and
 * %EXYNOS_ISPFE_METERING_SAMPLE_MAX and %EXYNOS_ISPFE_METERING_EXCLUDE_NONE
 * for the exposure grid, so as shipped the exposure grid counts every sample
 * and the white balance grid excludes what sits at either end of its range.
 * They are what a block sent with ``V4L2_ISP_PARAMS_FL_BLOCK_DISABLE`` returns
 * to.
 *
 * What is **not** here is the region geometry. The grids' 64 x 48 regions and
 * the cell size that tiles them over the sensor are derived from the format,
 * not chosen: they are the hardware's description of itself, and a buffer that
 * could disagree with the driver about them would give the two of them two
 * sources for one fact.
 */
struct exynos_ispfe_params_metering {
	struct v4l2_isp_params_block_header header;
	__s16 awb_saturation_threshold;
	__s16 awb_dark_threshold;
	__u16 awb_luma_coeff[EXYNOS_ISPFE_WB_GAINS];
	__u16 awb_luma_threshold_low;
	__u16 awb_luma_threshold_high;
	__u16 awb_diff_coring_threshold;
	__s16 ae_saturation_threshold;
	__s16 ae_dark_threshold;
	__u16 reserved;
} __attribute__((aligned(8)));

/**
 * define EXYNOS_ISPFE_PARAMS_MAX_SIZE - Maximum parameters data size
 *
 * The largest a buffer's block list can be, which is every block type once.
 */
#define EXYNOS_ISPFE_PARAMS_MAX_SIZE \
	(sizeof(struct exynos_ispfe_params_white_balance) + \
	 sizeof(struct exynos_ispfe_params_metering))

/*
 * Both grids are the same shape: LMP meters 64 x 48 rectangular regions over
 * the picture, and the cell size rather than the region count is what changes
 * with the sensor mode.
 */
#define EXYNOS_ISPFE_STATS_COLUMNS		64
#define EXYNOS_ISPFE_STATS_ROWS			48
#define EXYNOS_ISPFE_STATS_REGIONS \
	(EXYNOS_ISPFE_STATS_COLUMNS * EXYNOS_ISPFE_STATS_ROWS)

/**
 * struct exynos_ispfe_stats_grid_header - Metadata every LMP grid begins with
 *
 * @reserved0: Undefined
 * @frame_id: The hardware's own frame counter for the frame this grid was
 *	metered from
 * @reserved1: Undefined
 * @columns: Regions per row the hardware metered
 * @rows: Rows of regions the hardware metered
 * @reserved2: Undefined
 *
 * The reserved words are not zero. They are the parts of the hardware's own
 * 64-byte metadata area whose meaning has not been established, and a consumer
 * must ignore them rather than assume they will stay as they are.
 *
 * @columns and @rows are what the hardware reports rather than what it was
 * asked for, so they are the check that a grid holds a result at all. The
 * driver only sets the corresponding measurement flag when they are the
 * %EXYNOS_ISPFE_STATS_COLUMNS x %EXYNOS_ISPFE_STATS_ROWS this hardware meters.
 *
 * @frame_id is the hardware's own numbering rather than this interface's;
 * use @exynos_ispfe_stats_buffer.frame_sequence to say which frame a buffer
 * describes.
 */
struct exynos_ispfe_stats_grid_header {
	__u32 reserved0[5];
	__u32 frame_id;
	__u32 reserved1[4];
	__u16 columns;
	__u16 rows;
	__u32 reserved2[5];
};

/**
 * struct exynos_ispfe_stats_awb_region - One region of the white balance grid
 *
 * @sum: Sums of the accepted samples, in R, Gr, Gb and B order
 * @reserved0: Two further retained per-colour quantities, meaning unknown
 * @usable_count: Samples that were neither dark nor saturated, shared by the
 *	four colour sums
 * @dark_count: Samples below the configured dark threshold
 * @saturated_count: Samples above the configured saturation threshold
 * @reserved1: Undefined
 * @reserved2: Three further unsigned quantities, meaning unknown, then padding
 *
 * The mean of a channel is @sum divided by @usable_count: dark and saturated
 * samples are excluded from the sums rather than clamped into them, which is
 * what makes a grey-world estimate over this grid ignore both. The three
 * counts add up to the number of samples the region's cell contains, and that
 * identity over the whole grid is the cheapest check that a buffer holds one
 * complete frame rather than a torn one.
 */
struct exynos_ispfe_stats_awb_region {
	__s32 sum[4];
	__s32 reserved0[8];
	__u16 usable_count;
	__u16 dark_count;
	__u16 saturated_count;
	__u16 reserved1;
	__u32 reserved2[10];
};

/**
 * struct exynos_ispfe_stats_ae_region - One region of the exposure grid
 *
 * @sum: Sums of the accepted samples, in R, Gr, Gb and B order
 * @usable_count: Per colour, samples that were neither dark nor saturated
 * @y_diff_sum: Per colour, the sum of vertical neighbour differences
 * @dark_count: Per colour, samples below the configured dark threshold
 * @x_diff_sum: Per colour, the sum of horizontal neighbour differences
 * @saturated_count: Per colour, samples above the configured saturation
 *	threshold
 * @reserved: Undefined
 *
 * Unlike the white balance grid this one counts per colour rather than once
 * per region. The difference sums are a sharpness measure over the region and
 * are signed; everything else is a count or a sum of samples.
 *
 * These statistics are metered after lens shading correction, so they describe
 * the corrected picture rather than the sensor's raw response.
 */
struct exynos_ispfe_stats_ae_region {
	__u32 sum[4];
	__u16 usable_count[4];
	__s32 y_diff_sum[4];
	__u16 dark_count[4];
	__s32 x_diff_sum[4];
	__u16 saturated_count[4];
	__u32 reserved[6];
};

/**
 * struct exynos_ispfe_stats_awb - The auto white balance grid
 *
 * @header: What the hardware recorded about this grid
 * @regions: The metered regions
 *
 * A region is at ``regions[row * EXYNOS_ISPFE_STATS_COLUMNS + column]``. The
 * row stride is the array's full width even when @header reports fewer
 * columns, because it is the hardware's own stride rather than a packing this
 * interface chose.
 */
struct exynos_ispfe_stats_awb {
	struct exynos_ispfe_stats_grid_header header;
	struct exynos_ispfe_stats_awb_region regions[EXYNOS_ISPFE_STATS_REGIONS];
};

/**
 * struct exynos_ispfe_stats_ae - The post-shading auto exposure grid
 *
 * @header: What the hardware recorded about this grid
 * @regions: The metered regions, indexed as for
 *	:c:type:`exynos_ispfe_stats_awb`
 */
struct exynos_ispfe_stats_ae {
	struct exynos_ispfe_stats_grid_header header;
	struct exynos_ispfe_stats_ae_region regions[EXYNOS_ISPFE_STATS_REGIONS];
};

/**
 * struct exynos_ispfe_stats_buffer - ISPFE per-frame statistics
 *
 * @version: One of :c:type:`exynos_ispfe_stats_version`
 * @stats_type: A bitmask of the %EXYNOS_ISPFE_STATS_ flags saying which grids
 *	below the hardware wrote for this frame
 * @frame_sequence: The front end's number for the frame this buffer was
 *	metered from, or zero if it was metered from no frame at all
 * @reserved: Undefined; zero
 * @awb: The white balance grid, valid when %EXYNOS_ISPFE_STATS_AWB is set
 * @ae: The exposure grid, valid when %EXYNOS_ISPFE_STATS_AE is set
 *
 * One buffer is one frame's statistics. Which frame is said twice, and neither
 * is the buffer's ``sequence``: the buffer's timestamp is that frame's end,
 * bit for bit the timestamp the same frame's image buffer carries, and
 * @frame_sequence is the number that image buffer carries. The timestamp is
 * the one that survives a capture being stopped and started, because the
 * frame counter restarts with it.
 *
 * The buffer's own ``sequence`` counts this node's buffers instead, one per
 * buffer with no gaps, because a buffer that carries no frame still has to be
 * numbered and a number that goes backwards is the one thing a queue may not
 * produce. Frames that produced no statistics show up as a jump in
 * @frame_sequence, which is where a gap belongs.
 *
 * A buffer with no measurement flags set carries no result. Which of the two
 * reasons applies is in @frame_sequence: zero means nothing was capturing when
 * the buffer was queued, and non-zero means that frame really happened and the
 * hardware wrote no grid for it. Check @stats_type before reading either grid
 * either way.
 *
 * Nothing here is a register or an address: these are the values the hardware
 * measured, in the units it measured them in.
 */
struct exynos_ispfe_stats_buffer {
	__u32 version;
	__u32 stats_type;
	__u32 frame_sequence;
	__u32 reserved;
	struct exynos_ispfe_stats_awb awb;
	struct exynos_ispfe_stats_ae ae;
};

#endif /* __UAPI_EXYNOS_ISPFE_CONFIG_H */
