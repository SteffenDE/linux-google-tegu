/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Google zumapro camera front end (ISPFE/LMP) statistics
 *
 * Copyright (C) 2026 Steffen Deusch <steffen@deusch.me>
 */

#ifndef __UAPI_EXYNOS_ISPFE_CONFIG_H
#define __UAPI_EXYNOS_ISPFE_CONFIG_H

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
