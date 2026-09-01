.. SPDX-License-Identifier: GPL-2.0

.. _v4l2-meta-fmt-becore-params:

*******************************************
V4L2_META_FMT_BECORE_PARAMS ('BECP')
*******************************************

Configuration Parameters
========================

The configuration parameters are passed to the zumapro camera back end's
metadata output video node, using the :c:type:`v4l2_meta_format` interface.
They use the v4l2-isp parameters system: groups of parameters are defined as
distinct structs, or "blocks", which userspace appends to the data member of
:c:type:`v4l2_isp_params_buffer`. Each block-specific struct embeds
:c:type:`v4l2_isp_params_block_header` as its first member, and userspace must
populate the type member with a value from
:c:type:`exynos_becore_params_block_type`.

A buffer carries only what changed. A block that is not present leaves that
part of the configuration as it was, so a steady scene needs no buffer at all,
and a block sent with ``V4L2_ISP_PARAMS_FL_BLOCK_DISABLE`` returns that part to
the driver's own default. For most blocks that default is the value the driver
would have programmed anyway; for the colour LUT and the sharpener it is
bypass, because for the first a lattice is the whole of what that stage does
and there is no neutral one, and for the second there is no set of gains that
is honestly neutral for a stage whose job is to decide what detail looks like.

The driver rejects a buffer at :c:func:`VIDIOC_QBUF` if a block is
inconsistent, so a mistake is reported against the buffer that carried it:
every row of the colour matrix must sum to
``EXYNOS_BECORE_CCM_ONE``, which is what makes the matrix preserve neutrals;
neither of the tone mapper's two curves may decrease, nor exceed the unity
each is scaled against -- they are Q15 and Q14 in fields of the same width,
so a curve sent in the wrong one fits and is twice as steep; and the colour
LUT's samples must fit ``EXYNOS_BECORE_CLUT_MAX`` with both ends of its grey
axis neutral.

The tone mapper's bilateral grid is checked only for range, and that is a
statement about what the driver knows rather than about how much it trusts
userspace. A curve's samples sit at inputs the driver states, so it can say that
one goes backwards; the grid's levels sit wherever the guide curve puts them,
and that curve comes from userspace too, so the same numbers are a sensible tone
map under one curve and an inverted one under another. The two limits it does
enforce have different reasons: ``EXYNOS_BECORE_LTM_GRID_SLOPE_MAX`` is where
Q20 runs out of a 32-bit integer, while the hardware's exponent still has room
above it, and ``EXYNOS_BECORE_LTM_GRID_BIAS_MAX`` is the last rung of the
exponent ladder the bias has, past which no scaling makes it fit.

One thing about the block is worth knowing before using it: the quantisation
step is chosen from the largest magnitude anywhere in the grid, because the
hardware carries one exponent for the whole of it. A single cell asking for a
1000x gain therefore costs every other cell most of its precision, and unity
would be written as 32 parts in 32768. Bounding what an algorithm asks for is
what keeps the rest of the grid smooth.

The sharpener is the exception and is deliberately not checked that way. Its
values are fixed-point numbers in the hardware's own fields, and a value past
the field it reaches **saturates** rather than being refused -- which is what
the vendor's own encoder does, and what the phone's shipped calibration needs:
one of its noise gains is 2.0 at ordinary gains where the hardware field is
eight bits at Q7. Refusing a buffer for that would oblige every caller to
carry its own copy of the field widths.

.. code-block:: c

	struct v4l2_isp_params_buffer *params =
		(struct v4l2_isp_params_buffer *)buffer;

	params->version = V4L2_ISP_PARAMS_VERSION_V1;
	params->data_size = 0;

	void *data = (void *)params->data;

	struct exynos_becore_params_ccm *ccm =
		(struct exynos_becore_params_ccm *)data;

	ccm->header.type = EXYNOS_BECORE_PARAM_BLOCK_CCM;
	ccm->header.flags |= V4L2_ISP_PARAMS_FL_BLOCK_ENABLE;
	ccm->header.size = sizeof(struct exynos_becore_params_ccm);

	/* An identity matrix: each row sums to EXYNOS_BECORE_CCM_ONE. */
	ccm->matrix[0] = EXYNOS_BECORE_CCM_ONE;
	ccm->matrix[4] = EXYNOS_BECORE_CCM_ONE;
	ccm->matrix[8] = EXYNOS_BECORE_CCM_ONE;

	data += sizeof(struct exynos_becore_params_ccm);
	params->data_size += sizeof(struct exynos_becore_params_ccm);

The ``bytesused`` field of the queued buffer must be the size of
:c:type:`v4l2_isp_params_buffer` plus ``data_size``.

The colour LUT is the one block whose hardware stage does not run at all until
a buffer carries it, and it is by far the largest, so a pipeline with no
lattice to send should leave it out rather than fill one in. Sending one looks
like this::

	struct exynos_becore_params_clut *clut =
		(struct exynos_becore_params_clut *)data;

	clut->header.type = EXYNOS_BECORE_PARAM_BLOCK_CLUT;
	clut->header.flags |= V4L2_ISP_PARAMS_FL_BLOCK_ENABLE;
	clut->header.size = sizeof(struct exynos_becore_params_clut);

	for (unsigned int r = 0; r < EXYNOS_BECORE_CLUT_AXIS_NODES; r++)
		for (unsigned int g = 0; g < EXYNOS_BECORE_CLUT_AXIS_NODES; g++)
			for (unsigned int b = 0; b < EXYNOS_BECORE_CLUT_AXIS_NODES; b++) {
				unsigned int node =
					(r * EXYNOS_BECORE_CLUT_AXIS_NODES + g) *
					EXYNOS_BECORE_CLUT_AXIS_NODES + b;

				clut->lut_u[node] = chroma_u(r, g, b);
				clut->lut_v[node] = chroma_v(r, g, b);
			}

	data += sizeof(struct exynos_becore_params_clut);
	params->data_size += sizeof(struct exynos_becore_params_clut);

zumapro BE-core uAPI data types
===============================

.. kernel-doc:: include/uapi/linux/media/samsung/exynos-becore-config.h
