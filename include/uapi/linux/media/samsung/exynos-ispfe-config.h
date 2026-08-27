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
 * **A version's layout only ever grows at the end.** A member added to it does
 * not bump the version, because everything before it keeps its offset and its
 * meaning -- so take the buffer's extent from the format's ``buffersize`` and
 * ``bytesused`` rather than from the size of a struct compiled against a newer
 * header, and take its *content* from @exynos_ispfe_stats_buffer.stats_type.
 * A member whose flag is clear was not written and may not be there at all.
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
 *
 * %EXYNOS_ISPFE_STATS_HISTOGRAM:
 *	The per-pixel RGBY histogram is present, in @histogram.
 * %EXYNOS_ISPFE_STATS_FLICKER:
 *	The per-row sums are present, in @flicker.
 * %EXYNOS_ISPFE_STATS_LSC:
 *	The lens shading grid, :c:type:`exynos_ispfe_stats_lsc`
 * %EXYNOS_ISPFE_STATS_MOTION:
 *	The motion metering map, :c:type:`exynos_ispfe_stats_motion`
 */
#define EXYNOS_ISPFE_STATS_AWB			(1U << 0)
#define EXYNOS_ISPFE_STATS_AE			(1U << 1)
#define EXYNOS_ISPFE_STATS_HISTOGRAM		(1U << 2)
#define EXYNOS_ISPFE_STATS_FLICKER		(1U << 3)
#define EXYNOS_ISPFE_STATS_LSC			(1U << 4)
#define EXYNOS_ISPFE_STATS_MOTION		(1U << 5)

/**
 * enum exynos_ispfe_params_block_type - Parameters block type
 *
 * @EXYNOS_ISPFE_PARAM_BLOCK_WHITE_BALANCE: LMP's white balance gains,
 *	:c:type:`exynos_ispfe_params_white_balance`
 * @EXYNOS_ISPFE_PARAM_BLOCK_METERING: What the two statistics grids count,
 *	:c:type:`exynos_ispfe_params_metering`
 * @EXYNOS_ISPFE_PARAM_BLOCK_LENS_SHADING: The lens shading gain grid,
 *	:c:type:`exynos_ispfe_params_lens_shading`
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
	EXYNOS_ISPFE_PARAM_BLOCK_LENS_SHADING,
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
 * the per-colour quantities :c:type:`exynos_ispfe_stats_rggb_region` still
 * carries as reserved; until something decodes those, a consumer has no way to
 * observe what they did, and this interface does not pretend otherwise.
 *
 * A dark threshold above its own saturation threshold would exclude every
 * sample in the frame, and is refused rather than metered.
 *
 * The two grids take their own thresholds and are not two views of one
 * setting. What they are *not* is two points in the chain: both are metered
 * after lens shading correction, and with the unity table the driver ships
 * they return bit-identical sums wherever neither excludes a sample
 * [HW 2026-08-27]. The grid metered before that stage is
 * :c:type:`exynos_ispfe_stats_lsc`, and nothing here configures it.
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

/*
 * The shading grid: 33 columns by 25 rows of four gains, one per Bayer colour
 * in the same R, Gr, Gb, B order everything else here uses, covering the whole
 * picture with the outermost samples on its edges.
 *
 * The hardware reads them in a tiled layout, four rows of gains to a record,
 * with the twenty-fifth row alone in a record whose other three rows are
 * padding. That layout is the driver's business: what a block carries is the
 * grid.
 */
#define EXYNOS_ISPFE_LSC_COLUMNS		33
#define EXYNOS_ISPFE_LSC_ROWS			25

/* Unsigned Q12, the same scale as the white balance gains: 4096 is 1.0. */
#define EXYNOS_ISPFE_LSC_GAIN_ONE		4096

/*
 * The field is sixteen bits, so this is what can be described rather than a
 * policy: a gain of 15.999756. The vendor's own tables run from exactly
 * %EXYNOS_ISPFE_LSC_GAIN_ONE at their lowest point, near the centre, to about
 * 5.2 at a corner, and none of the 54 captured ones has a single sample below
 * unity.
 */
#define EXYNOS_ISPFE_LSC_GAIN_MAX		65535

/**
 * struct exynos_ispfe_params_lens_shading - The lens shading gain grid
 *
 * @header: The parameters block header
 * @gains: %EXYNOS_ISPFE_LSC_ROWS rows of %EXYNOS_ISPFE_LSC_COLUMNS columns of
 *	four unsigned Q12 gains, indexed by %EXYNOS_ISPFE_WB_RED and its
 *	siblings
 *
 * The gain applied to a pixel is this grid interpolated at its position, so
 * the grid describes the whole picture rather than a corner of it and its
 * outermost samples sit on the edges. Each gain is at least one -- zero is a
 * channel switched off rather than corrected -- and at most
 * %EXYNOS_ISPFE_LSC_GAIN_MAX.
 *
 * This is the only place in the graph where shading can be corrected: the back
 * end has no such stage. What it corrects is everything downstream of it,
 * which is the processed image and **two of the three metering grids** --
 * :c:type:`exynos_ispfe_stats_ae` and :c:type:`exynos_ispfe_stats_awb` are
 * metered after this stage, and moving the grid moves both.
 *
 * What it does **not** reach is the raw output, which leaves the receiver
 * before this stage runs, and :c:type:`exynos_ispfe_stats_lsc`, which is
 * metered above it. So a grid sent while the raw path is streaming changes the
 * exposure and white balance grids, and changes neither the raw buffers nor
 * the lens shading grid -- which is what makes that grid an estimate of the
 * falloff rather than a view of the correction.
 *
 * A grid is per unit and per readout rather than per scene. Measured over 54
 * captured vendor programs, a table is the same table across the frames of one
 * capture to within 1.12% at its worst grid point, and differs by up to 30%
 * between two sensor readouts of one camera and 42% between two cameras. So a
 * consumer that has one calibration for this unit and this mode has what this
 * block wants, and does not need to recompute it per frame.
 *
 * Disabling this block (%V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) returns the grid to
 * the driver's default, which is unity everywhere -- so for this block, and
 * unlike the two above, disabling really does switch correction off. The
 * kernel ships no calibration: a grid belongs to one unit at one readout, and
 * two readouts of one camera differ by up to 30% at a grid point, so there is
 * no table it could carry that would be right for the next stream.
 *
 * The cost is visible and is meant to be. With no grid sent, a corner of this
 * lens reads about a sixth of its centre, and a picture that looks
 * uncalibrated is a better answer than one silently corrected by some other
 * unit's numbers.
 */
struct exynos_ispfe_params_lens_shading {
	struct v4l2_isp_params_block_header header;
	__u16 gains[EXYNOS_ISPFE_LSC_ROWS][EXYNOS_ISPFE_LSC_COLUMNS]
		   [EXYNOS_ISPFE_WB_GAINS];
} __attribute__((aligned(8)));

/**
 * define EXYNOS_ISPFE_PARAMS_MAX_SIZE - Maximum parameters data size
 *
 * The largest a buffer's block list can be, which is every block type once.
 */
#define EXYNOS_ISPFE_PARAMS_MAX_SIZE \
	(sizeof(struct exynos_ispfe_params_white_balance) + \
	 sizeof(struct exynos_ispfe_params_metering) + \
	 sizeof(struct exynos_ispfe_params_lens_shading))

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
 * struct exynos_ispfe_stats_rggb_region - One region of an RGGB metering grid
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
 *
 * **Two grids carry this record**, :c:type:`exynos_ispfe_stats_awb` and
 * :c:type:`exynos_ispfe_stats_lsc`, and they are two instances of one hardware
 * writer rather than a resemblance: Lyric reads both through a single
 * `lyric::LmpAwbLscStatsOutput`, whose ``GetRggbRegionStatsV2`` is where the
 * field names below come from, and its allocator builds one for
 * ``outputs.awb_stats`` and one for ``outputs.lsc_stats`` from the same call.
 * The two blocks' register layouts are the same too -- what differs between
 * them is the thresholds and the luma weights each is programmed with, not
 * what it writes.
 */
struct exynos_ispfe_stats_rggb_region {
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
 * **@sum is signed, and a dark region really does go below zero.** Samples
 * reach this grid after black level subtraction, so a region darker than the
 * pedestal sums negative -- 847 of a frame's 12,288 region sums at one line of
 * exposure, none at 1500, on one scene. Read unsigned, each of those becomes
 * about 4.29 billion and a mean over the grid comes out two to three orders of
 * magnitude high: 456,378 units against the 11.8 the same buffer holds. The
 * white balance grid's @sum was signed from the start and this one was not,
 * which is the whole of the difference.
 *
 * These statistics are metered after lens shading correction, so they describe
 * the corrected picture rather than the sensor's raw response -- measured, not
 * taken from the tap's name; see :c:type:`exynos_ispfe_stats_lsc`, which is
 * the grid on the other side of that stage.
 */
struct exynos_ispfe_stats_ae_region {
	__s32 sum[4];
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
	struct exynos_ispfe_stats_rggb_region regions[EXYNOS_ISPFE_STATS_REGIONS];
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
 * struct exynos_ispfe_stats_lsc - The lens shading grid
 *
 * @header: What the hardware recorded about this grid
 * @regions: The metered regions, indexed as for
 *	:c:type:`exynos_ispfe_stats_awb`
 *
 * The third of the front end's rectangular metering grids, and the second to
 * carry :c:type:`exynos_ispfe_stats_rggb_region` -- see that type for why the
 * record is shared rather than merely alike. Its name is the hardware's own:
 * the recipe's tap is ``lmp/lsc_stats`` and Lyric's allocator calls its buffer
 * ``outputs.lsc_stats``.
 *
 * **This is the one grid metered before lens shading correction**, and the
 * other two are metered after it [HW 2026-08-27]. Which is what the tap is
 * for: an estimate of the falloff has to see the falloff. It was measured by
 * sending :c:type:`exynos_ispfe_params_lens_shading` a table with twice the
 * gain at the corners as at the centre and fitting all three grids against a
 * raw frame of the same scene -- the raw output leaves the receiver before any
 * of this runs, so a grid upstream of the correction stays an affine function
 * of it with constant coefficients and one downstream does not. This grid held
 * at an R-squared of 0.99997 with a radial residual correlation of +0.09; the
 * exposure grid fell to 0.948 at +0.72, and the two grids' per-colour ratio
 * ran from 1.04 at the centre of the field to 1.80 at its corners.
 *
 * **With the table the driver ships, that costs one part in 4096 and nothing
 * else** [HW 2026-08-27]. Its default shading grid is unity, so the stage it
 * straddles is a unit gain: the exposure and white balance grids come back
 * *bit-identical* to each other in every region where neither excludes a
 * sample, and this one reads about one Q12 step above them, which is the
 * rounding of that multiply. So a consumer must not expect a difference here
 * until something sends a real calibration -- and must not assume there is
 * none once something does.
 *
 * The one thing the vendor's own allocator does say is that this grid's
 * geometry is not negotiable where the white balance grid's is: it builds this
 * one at a hardcoded 64 x 48 and that one from the request's fields.
 *
 * The thresholds and luma weights it is programmed with are its own and are
 * not the white balance grid's -- the vendor ships -30000 and 31500 here
 * against 0 and 32256 there -- so what its counts exclude is a different set
 * of samples. :c:type:`exynos_ispfe_params_metering` does not reach this block
 * yet.
 */
struct exynos_ispfe_stats_lsc {
	struct exynos_ispfe_stats_grid_header header;
	struct exynos_ispfe_stats_rggb_region regions[EXYNOS_ISPFE_STATS_REGIONS];
};

/*
 * The histogram's four planes, in the order it carries them, and how far apart
 * they sit.
 *
 * %EXYNOS_ISPFE_HISTOGRAM_BINS is the **buffer's** stride and the block's
 * maximum, not how many bins are in use. The block takes a bin count as a
 * power of two and the recipes program 256, while the planes stay 512 apart
 * regardless -- so the upper half of every plane is never written and reads
 * back as the zeros the driver left. A consumer that walks all 512 gets the
 * right answer for a distribution; one that divides by the array length does
 * not.
 *
 * That is a configuration and not a limit: Lyric's own default is 512, and the
 * count travels with a `block_size` word the hardware requires to equal
 * `16 << log2(bins)` -- four planes of four bytes a bin. Nothing here can raise
 * it, because the count lives in the front end's replayed program rather than
 * in this interface, and no captured program has ever used 512.
 */
#define EXYNOS_ISPFE_HISTOGRAM_RED		0
#define EXYNOS_ISPFE_HISTOGRAM_GREEN		1
#define EXYNOS_ISPFE_HISTOGRAM_BLUE		2
#define EXYNOS_ISPFE_HISTOGRAM_LUMA		3
#define EXYNOS_ISPFE_HISTOGRAM_PLANES		4
#define EXYNOS_ISPFE_HISTOGRAM_BINS		512
/* The bits of @exynos_ispfe_stats_histogram.bins_log2 that carry the count. */
#define EXYNOS_ISPFE_HISTOGRAM_BINS_MASK	0x1f

/**
 * struct exynos_ispfe_stats_histogram - The per-pixel RGBY histogram
 *
 * @reserved0: The hardware's own metadata area, undecoded
 * @bins_log2: How many bins the hardware was configured for, as a power of two
 *	in its **low five bits**; mask before shifting
 * @reserved1: The rest of that metadata area, also undecoded
 * @bins: Per plane, how many samples fell in each bin
 * @total: Per plane, how many samples the histogram counted at all
 *
 * Unlike the grids this has no spatial resolution: it is one distribution
 * over a region of the frame, where they are means over 64 x 48 regions. The
 * two are complementary rather than ordered -- a quantile is not derivable from
 * a mean, and a weighted metering is not derivable from a distribution -- and
 * an exposure algorithm that wants constraint modes needs this one.
 *
 * The block writes **three** of these, one per region of interest, and this
 * interface carries the first. The other two are allocated and written and
 * nothing reads them; what the three regions are is not decoded.
 *
 * **Only the first ``1 << (@bins_log2 & 0x1f)`` bins of each plane are
 * written**, and the recipes program 256 where the stride is 512. Take the
 * count from @bins_log2 rather than from the array's length: a consumer that
 * divides by the length gets an answer wrong by exactly the ratio, and
 * `libipa`'s constraint arithmetic divides by the bin count.
 *
 * The bins past that count read as zero because the allocation was zeroed, not
 * because anything clears them per frame -- so on a torn buffer they can carry
 * an older frame's counts.
 *
 * It is the hardware's own report rather than a copy of the configuration --
 * `lyric::LmpRgbyHistogramStatsOutput::ValidateMetadata` checks its own
 * expectation against this same word -- so it says what the frame was actually
 * binned with.
 *
 * **@bins_log2 is what says a buffer holds a result.** The driver clears it
 * before arming and the hardware writes it, so it is zero for a frame nothing
 * wrote. @total counts samples rather than values and would be the more
 * natural test, and it cannot be used for one: it sits three pages into a
 * non-contiguous allocation, too far for the driver to clear -- see the note on
 * @reserved0 -- so it is **not** reset between frames and a torn buffer carries
 * the previous occupant's totals rather than zeros.
 *
 * The first 64 bytes are the hardware's, in the same place the grids keep
 * their own metadata area, and nothing has decoded them. They are not assumed
 * to have the grids' layout.
 */
struct exynos_ispfe_stats_histogram {
	__u32 reserved0[10];
	__u32 bins_log2;
	__u32 reserved1[5];
	__u32 bins[EXYNOS_ISPFE_HISTOGRAM_PLANES][EXYNOS_ISPFE_HISTOGRAM_BINS];
	__u32 total[EXYNOS_ISPFE_HISTOGRAM_PLANES];
};

/*
 * The most rows the flicker block will sum. It is the hardware's own bound --
 * `lyric::LmpFlickerStatsOutput::Make` refuses a larger count and refuses a
 * buffer smaller than the 0x46c0 bytes this many rows need -- rather than a
 * picture height, and it is comfortably above the 3120 rows this sensor's full
 * readout has. Take the rows in use from @exynos_ispfe_stats_flicker.rows.
 */
#define EXYNOS_ISPFE_FLICKER_ROWS		4512

/**
 * struct exynos_ispfe_stats_flicker - One sum per row of the picture
 *
 * @reserved0: The hardware's own metadata area, undecoded
 * @rows: How many entries of @row_sum the hardware wrote
 * @reserved1: The rest of that metadata area, also undecoded
 * @row_sum: Per entry, the sum of its samples
 *
 * **An entry is a pair of picture rows, not one.** The block reports 1560
 * entries for the 4208x3120 readout, and summing a raw frame's rows in pairs
 * reproduces them to an R-squared of 0.999973 where the first 1560 rows alone
 * give 0.77 [HW 2026-08-26]. So an entry spans two line times, which is what
 * converts a period in entries to one in seconds, and @rows is half the
 * picture height rather than the whole of it.
 *
 * The samples are the ones the metering grids see -- after black level
 * subtraction and with the gains applied -- rather than the sensor's. The same
 * fit puts the slope at 34.06 and the intercept at 64.1 counts a sample, which
 * is the black level, and the mean per sample agrees with the white balance
 * grid's own mean over the same frame to 1%.
 *
 * Where the grids and the histograms resolve the picture in space and in
 * intensity, this resolves it in **time**: a rolling shutter reads one row
 * after another, so a light that is modulated at twice the mains frequency
 * writes its own waveform down the frame, and a sum along each row is the
 * cheapest thing that recovers it. That is the whole of what this measurement
 * is for -- the sums say nothing about the picture that the exposure grid does
 * not say better, and everything about the *illuminant*.
 *
 * There is nothing to configure and so nothing here describes a configuration.
 * The block's register printer has exactly three fields -- an enable, a frame
 * id and this buffer's address -- so unlike the grids there is no region of
 * interest, no channel selection and no weighting: a row is a row of the
 * picture the front end is processing, and the sum is over all of it.
 *
 * **@rows is what says a buffer holds a result.** The driver clears it before
 * the frame is armed and the hardware writes it, so it is zero for a frame
 * nothing wrote; it is also the hardware's own report rather than a copy of a
 * configuration, which is how the vendor's own reader validates the buffer it
 * allocated. Entries past @rows are not written and hold whatever they last
 * did.
 *
 * The sums are **signed because the vendor reads them signed** --
 * `ZumaAeInputParser::ExtractFlickerData` copies them into a ``vector<int>`` --
 * and not because they go negative. **They do not**: the block floors at zero.
 * On a black frame at one line of exposure and unit gain, all 1560 entries are
 * exactly zero while the exposure grid metered from the same frames sums to
 * about -12 million per colour, which is what a signed grid does below the
 * black level pedestal [HW 2026-08-26]. So the two are not alike, and a
 * consumer should not carry the exposure grid's expectations here.
 *
 * Nothing reachable distinguishes the two readings in any case: a full row
 * pair sums to a few tens of millions in the metering domain, so the sign bit
 * is never in use.
 *
 * The first 64 bytes are the hardware's, in the same place the grids keep
 * their own metadata area, and only @rows is decoded. They are not assumed to
 * have the grids' layout.
 */
struct exynos_ispfe_stats_flicker {
	__u32 reserved0[9];
	__u32 rows;
	__u32 reserved1[6];
	__s32 row_sum[EXYNOS_ISPFE_FLICKER_ROWS];
};

/*
 * The largest map the motion metering block will write, which is the
 * hardware's own bound rather than a picture size:
 * `lyric::LmpMotionMeteringStatsOutput::Make` refuses more columns or rows
 * than these, and refuses a buffer smaller than the 0x18040 bytes they need.
 * The recipes program 128 x 96. Take the map in use from
 * @exynos_ispfe_stats_motion.columns and @exynos_ispfe_stats_motion.rows.
 */
#define EXYNOS_ISPFE_MOTION_COLUMNS		256
#define EXYNOS_ISPFE_MOTION_ROWS		192
#define EXYNOS_ISPFE_MOTION_CELLS \
	(EXYNOS_ISPFE_MOTION_COLUMNS * EXYNOS_ISPFE_MOTION_ROWS)

/**
 * struct exynos_ispfe_stats_motion - A luma map of the whole frame
 *
 * @reserved0: The hardware's own metadata area, undecoded
 * @stride: The distance in **bytes** from one row of @luma to the next
 * @columns: Cells per row the hardware wrote
 * @rows: Rows of cells it wrote
 * @reserved1: The rest of that metadata area, also undecoded
 * @luma: The cells, row-major, at ``luma[row * (@stride / 2) + column]``,
 *	each **centred** on its point of the metering grid
 *
 * This is the only one of the front end's measurements that is a *picture*.
 * The grids reduce the frame to 64 x 48 per-colour sums and the histogram to
 * one distribution; this reduces it to a small greyscale image -- 128 x 96
 * as the recipes program it -- with no thresholds, no exclusions and one value
 * per cell. So it is the cheapest thing here to compare against a raw frame,
 * and the cheapest to compare against *itself* on the previous frame, which is
 * what the vendor's name for it describes: a scene that moved changes this map
 * where a scene that only changed brightness does not.
 *
 * **Index with @stride and not with %EXYNOS_ISPFE_MOTION_COLUMNS.** Unlike the
 * grids, whose row stride is the hardware's fixed 64 regions however many
 * it reports, this block packs its rows to the map it was configured for:
 * `LmpMotionMeteringStatsOutput` expects ``ALIGN(columns * 2, 32)`` bytes and
 * checks the hardware's own report against it, so at 128 columns the rows are
 * 256 bytes apart rather than 512. A consumer that walks the array by its
 * declared width reads the second half of every row as the first half of the
 * next one, and gets a plausible picture rather than an error.
 *
 * **@stride, @columns and @rows together are what says a buffer holds a
 * result.** The driver clears them before the frame is armed and the hardware
 * writes them, so a frame nothing wrote reads zero -- and the driver publishes
 * them only when every index the formula above can produce lands inside @luma,
 * because a consumer indexes with numbers it is handed.
 *
 * **A cell is centred on its grid point rather than starting there**
 * [HW 2026-08-27], which is the one thing about this result that is not
 * obvious and the one that fails quietly. Cell *k* of a row is filtered about
 * the pixel ``roi_start + cell_size * k``, so the first cell begins half a
 * cell *before* the region of interest starts. Reducing a raw frame into cells
 * that begin at the grid point instead fits this map at an R-squared of 0.905
 * where centring them gives 0.996 -- a recognisable picture with everything
 * half a cell out, rather than an error.
 *
 * What a cell holds is a **weighted luma of its neighbourhood, in the same
 * domain the grids meter in** [HW 2026-08-27]. Against a raw frame of the same
 * scene, applying the block's own filter -- the 25 signed coefficients in its
 * payload, which are half of a symmetric 31-tap triangle summing to 1 << 10 --
 * separably down both axes reaches an R-squared of 0.998556, against 0.995952
 * for a plain mean of the same window; and the fit's intercept puts the black
 * level at 62.4 counts a sample, where the grids and the per-row sums put it
 * at 64.1. Letting the four colour weights float reaches 0.999954 and does not
 * measure them: one scene's Bayer planes are nearly proportional, so what is
 * determined is the weighted sum and not the split.
 *
 * It is metered **after** lens shading correction, with the two grids that
 * are and not with :c:type:`exynos_ispfe_stats_lsc` [HW 2026-08-27]: under a
 * shading table with twice the corner gain, the same fit falls from 0.9986 to
 * 0.9533 exactly as the exposure grid's does.
 *
 * The first 64 bytes are the hardware's, in the same place the grids keep
 * their own metadata area, and only the three fields above are decoded. They
 * are not assumed to have the grids' layout, though the frame counter is in
 * the same word: `GetMetadata` reads it at byte 0x14.
 */
struct exynos_ispfe_stats_motion {
	__u32 reserved0[9];
	__u32 stride;
	__u16 columns;
	__u16 rows;
	__u32 reserved1[5];
	__u16 luma[EXYNOS_ISPFE_MOTION_CELLS];
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
 * @histogram: The RGBY histogram, valid when %EXYNOS_ISPFE_STATS_HISTOGRAM is
 *	set
 * @flicker: The per-row sums, valid when %EXYNOS_ISPFE_STATS_FLICKER is set
 * @lsc: The lens shading grid, valid when %EXYNOS_ISPFE_STATS_LSC is set
 * @motion: The motion metering map, valid when %EXYNOS_ISPFE_STATS_MOTION is
 *	set
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
 * hardware wrote no grid for it. Check @stats_type before reading any of them
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
	struct exynos_ispfe_stats_histogram histogram;
	struct exynos_ispfe_stats_flicker flicker;
	struct exynos_ispfe_stats_lsc lsc;
	struct exynos_ispfe_stats_motion motion;
};

#endif /* __UAPI_EXYNOS_ISPFE_CONFIG_H */
