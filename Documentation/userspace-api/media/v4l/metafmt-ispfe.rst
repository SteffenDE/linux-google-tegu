.. SPDX-License-Identifier: GPL-2.0

.. _v4l2-meta-fmt-ispfe-stats:

*******************************************
V4L2_META_FMT_ISPFE_STATS ('IFES')
*******************************************

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

zumapro ISPFE uAPI data types
=============================

.. kernel-doc:: include/uapi/linux/media/samsung/exynos-ispfe-config.h
