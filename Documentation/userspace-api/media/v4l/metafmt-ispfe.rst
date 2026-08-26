.. SPDX-License-Identifier: GPL-2.0

.. _v4l2-meta-fmt-ispfe-stats:
.. _v4l2-meta-fmt-ispfe-params:

***************************************************************************
V4L2_META_FMT_ISPFE_STATS ('IFES'), V4L2_META_FMT_ISPFE_PARAMS ('IFEP')
***************************************************************************

3A Statistics
=============

The zumapro camera front end meters every frame it receives and writes the
results to its statistics metadata capture video node, using the
:c:type:`v4l2_meta_format` interface. One buffer holds one frame's results,
described by :c:type:`exynos_ispfe_stats_buffer`: the auto white balance grid
and the post-shading auto exposure grid, both 64 x 48 regions over the picture,
a per-pixel RGBY histogram of a region of it, and one sum per row of it.

The four are complementary rather than alternatives, because each resolves the
frame along a different axis and none is derivable from another. A grid gives a
mean per region and so can be weighted spatially; the histogram gives a
distribution and so can answer a quantile; the row sums resolve the frame in
time, because a rolling shutter reads one row after another and a light
modulated at twice the mains frequency therefore writes its waveform down the
picture.

Which frame a buffer describes is said by its timestamp, which is bit for bit
the timestamp the same frame's image buffer carries, and by ``frame_sequence``,
which is the number that image buffer carries -- whichever node that frame left
through. The buffer's own ``sequence`` counts this node's buffers instead, one
per buffer, because a buffer that carries no frame still has to be numbered and
the front end's counter restarts with every raw capture.

A buffer is only filled while something is capturing. A frame that arrived
with no statistics buffer queued produces none, which shows up as a jump in
``frame_sequence`` rather than as a stale result; and a buffer queued while
nothing is capturing at all is returned immediately with ``stats_type`` zero,
so a consumer is never left waiting on a frame nobody is going to run. Check
``stats_type`` before reading any of them: a grid whose flag is clear was not
written and holds no result. ``frame_sequence`` says which of the two reasons
applies -- zero for a buffer that describes no frame, non-zero for a frame that
happened and produced no grid.

.. code-block:: c

	struct exynos_ispfe_stats_buffer *stats =
		(struct exynos_ispfe_stats_buffer *)buffer;

	if (stats->version != EXYNOS_ISPFE_STATS_VERSION_V1)
		return;

	if (stats->stats_type & EXYNOS_ISPFE_STATS_AWB) {
		/* The sums are signed: samples reach the grids after black
		 * level subtraction, so a region darker than the pedestal
		 * sums below zero. Reading them unsigned turns each such
		 * region into about 4.29 billion.
		 */
		__s64 sum[4] = {};
		__u64 usable = 0;

		for (unsigned int i = 0; i < EXYNOS_ISPFE_STATS_REGIONS; i++) {
			const struct exynos_ispfe_stats_awb_region *region =
				&stats->awb.regions[i];

			for (unsigned int c = 0; c < 4; c++)
				sum[c] += region->sum[c];
			usable += region->usable_count;
		}

		/* A grey-world estimate: dark and saturated samples are
		 * already excluded from both the sums and the count.
		 */
		if (usable)
			estimate_illuminant(sum, usable);
	}

Configuration Parameters
========================

The configuration parameters are passed to the zumapro camera front end's
metadata output video node, using the :c:type:`v4l2_meta_format` interface.
They use the v4l2-isp parameters system: groups of parameters are defined as
distinct structs, or "blocks", which userspace appends to the data member of
:c:type:`v4l2_isp_params_buffer`. Each block-specific struct embeds
:c:type:`v4l2_isp_params_block_header` as its first member, and userspace must
populate the type member with a value from
:c:type:`exynos_ispfe_params_block_type`.

Three block types exist, and each carries what only an algorithm or a
calibration can decide.

:c:type:`exynos_ispfe_params_white_balance` carries the gains the front end
applies before it meters, which are therefore both what balances the picture
and what the two grids above are measured through. Nothing about them is
knowable to the kernel -- they are an estimate of the illuminant, which is what
an AWB algorithm exists to make -- so until a buffer carries one the driver
applies the gains its captured program was taken with.

:c:type:`exynos_ispfe_params_metering` carries what the two grids *count*: the
sample thresholds above and below which a sample is recorded as saturated or
dark instead of being summed, and the luma window the white balance grid gates
on. The grids exclude samples rather than clamping them, so these decide what
the sums and counts in :c:type:`exynos_ispfe_stats_buffer` are a mean of, and
an algorithm that cannot set them is metering through someone else's choice.
What the block does *not* carry is the region geometry: the 64 x 48 regions and
the cell size that tiles them over the sensor are derived from the format
rather than chosen.

:c:type:`exynos_ispfe_params_lens_shading` carries the gain grid that corrects
the lens's falloff: 33 x 25 points over the whole picture, four gains each, in
the same R, Gr, Gb, B order as everything else here. It is the only stage in
the graph that can correct shading. What it corrects is the processed image and
the exposure statistics, which are metered after it; what it does not reach is
the raw output, which leaves the receiver before it runs. A grid is a
calibration of one lens at one sensor readout rather
than a per-frame decision, so a consumer that has one for this unit and this
mode has what the block wants; until a buffer carries one the driver applies
the grid its captured program was taken with.

A buffer is taken by the next program encode and the front end encodes one
program per frame, so a value reaches the frame after next: the same latency
the sensor's own controls have, and for the same reason.

The driver rejects a buffer at :c:func:`VIDIOC_QBUF` if a gain is outside
``1 .. EXYNOS_ISPFE_WB_GAIN_MAX``, if a metering dark threshold is above its
own saturation threshold, if the luma window is inverted, or if any shading
gain is zero -- so a mistake is reported against the buffer that carried it. A block sent with
``V4L2_ISP_PARAMS_FL_BLOCK_DISABLE`` returns that part of the configuration to
the driver's default rather than switching the stage off; it carries no values,
and none are checked.

.. code-block:: c

	struct v4l2_isp_params_buffer *params =
		(struct v4l2_isp_params_buffer *)buffer;

	params->version = V4L2_ISP_PARAMS_VERSION_V1;
	params->data_size = 0;

	void *data = (void *)params->data;

	struct exynos_ispfe_params_white_balance *wb =
		(struct exynos_ispfe_params_white_balance *)data;

	wb->header.type = EXYNOS_ISPFE_PARAM_BLOCK_WHITE_BALANCE;
	wb->header.flags |= V4L2_ISP_PARAMS_FL_BLOCK_ENABLE;
	wb->header.size = sizeof(struct exynos_ispfe_params_white_balance);

	/* Unsigned Q12: EXYNOS_ISPFE_WB_GAIN_ONE is a gain of 1.0. */
	wb->gains[EXYNOS_ISPFE_WB_RED] = red;
	wb->gains[EXYNOS_ISPFE_WB_GREEN_RED] = EXYNOS_ISPFE_WB_GAIN_ONE;
	wb->gains[EXYNOS_ISPFE_WB_GREEN_BLUE] = EXYNOS_ISPFE_WB_GAIN_ONE;
	wb->gains[EXYNOS_ISPFE_WB_BLUE] = blue;

	data += sizeof(struct exynos_ispfe_params_white_balance);
	params->data_size += sizeof(struct exynos_ispfe_params_white_balance);

	struct exynos_ispfe_params_metering *met =
		(struct exynos_ispfe_params_metering *)data;

	met->header.type = EXYNOS_ISPFE_PARAM_BLOCK_METERING;
	met->header.flags |= V4L2_ISP_PARAMS_FL_BLOCK_ENABLE;
	met->header.size = sizeof(struct exynos_ispfe_params_metering);

	/* Count everything the sensor can produce, on both grids. */
	met->awb_saturation_threshold = EXYNOS_ISPFE_METERING_SAMPLE_MAX;
	met->awb_dark_threshold = EXYNOS_ISPFE_METERING_EXCLUDE_NONE;
	met->ae_saturation_threshold = EXYNOS_ISPFE_METERING_SAMPLE_MAX;
	met->ae_dark_threshold = EXYNOS_ISPFE_METERING_EXCLUDE_NONE;

	/* An unweighted luma, gated over its whole range. */
	for (unsigned int i = 0; i < EXYNOS_ISPFE_WB_GAINS; i++)
		met->awb_luma_coeff[i] = EXYNOS_ISPFE_METERING_LUMA_ONE / 4;
	met->awb_luma_threshold_low = 0;
	met->awb_luma_threshold_high = EXYNOS_ISPFE_METERING_LUMA_MAX;

	data += sizeof(struct exynos_ispfe_params_metering);
	params->data_size += sizeof(struct exynos_ispfe_params_metering);

	struct exynos_ispfe_params_lens_shading *lsc =
		(struct exynos_ispfe_params_lens_shading *)data;

	lsc->header.type = EXYNOS_ISPFE_PARAM_BLOCK_LENS_SHADING;
	lsc->header.flags |= V4L2_ISP_PARAMS_FL_BLOCK_ENABLE;
	lsc->header.size = sizeof(struct exynos_ispfe_params_lens_shading);

	/* Unsigned Q12 again: a grid of unity gains corrects nothing. */
	for (unsigned int row = 0; row < EXYNOS_ISPFE_LSC_ROWS; row++)
		for (unsigned int col = 0; col < EXYNOS_ISPFE_LSC_COLUMNS; col++)
			for (unsigned int c = 0; c < EXYNOS_ISPFE_WB_GAINS; c++)
				lsc->gains[row][col][c] = calibration[row][col][c];

	data += sizeof(struct exynos_ispfe_params_lens_shading);
	params->data_size += sizeof(struct exynos_ispfe_params_lens_shading);

The ``bytesused`` field of the queued buffer must be the size of
:c:type:`v4l2_isp_params_buffer` plus ``data_size``.

zumapro ISPFE uAPI data types
=============================

.. kernel-doc:: include/uapi/linux/media/samsung/exynos-ispfe-config.h
