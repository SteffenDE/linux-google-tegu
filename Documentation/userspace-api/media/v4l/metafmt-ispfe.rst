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
:c:type:`v4l2_meta_format` interface. One buffer holds one frame's results:
the auto white balance grid and the post-shading auto exposure grid, both
64 x 48 regions over the picture, described by
:c:type:`exynos_ispfe_stats_buffer`.

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
``stats_type`` before reading either grid: a grid whose flag is clear was not
written and holds no result. ``frame_sequence`` says which of the two reasons
applies -- zero for a buffer that describes no frame, non-zero for a frame that
happened and produced no grid.

.. code-block:: c

	struct exynos_ispfe_stats_buffer *stats =
		(struct exynos_ispfe_stats_buffer *)buffer;

	if (stats->version != EXYNOS_ISPFE_STATS_VERSION_V1)
		return;

	if (stats->stats_type & EXYNOS_ISPFE_STATS_AWB) {
		__u64 sum[4] = {}, usable = 0;

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

One block type exists so far: the white balance gains the front end applies
before it meters, which are therefore both what balances the picture and what
the two grids above are measured through. Nothing about them is knowable to the
kernel -- they are an estimate of the illuminant, which is what an AWB
algorithm exists to make -- so until a buffer carries one the driver applies
the gains its captured program was taken with.

A buffer is taken by the next program encode and the front end encodes one
program per frame, so a gain reaches the frame after next: the same latency the
sensor's own controls have, and for the same reason.

The driver rejects a buffer at :c:func:`VIDIOC_QBUF` if a gain is outside
``1 .. EXYNOS_ISPFE_WB_GAIN_MAX``, so a mistake is reported against the buffer
that carried it. A block sent with ``V4L2_ISP_PARAMS_FL_BLOCK_DISABLE`` returns
the gains to the driver's default rather than switching white balance off; it
carries no values, and none are checked.

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

The ``bytesused`` field of the queued buffer must be the size of
:c:type:`v4l2_isp_params_buffer` plus ``data_size``.

zumapro ISPFE uAPI data types
=============================

.. kernel-doc:: include/uapi/linux/media/samsung/exynos-ispfe-config.h
