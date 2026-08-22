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
the driver's own default rather than switching the hardware stage off.

The driver rejects a buffer at :c:func:`VIDIOC_QBUF` if a block is
inconsistent, so a mistake is reported against the buffer that carried it:
every row of the colour matrix must sum to
``EXYNOS_BECORE_CCM_ONE``, which is what makes the matrix preserve neutrals,
and the tone curve must not decrease.

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

zumapro BE-core uAPI data types
===============================

.. kernel-doc:: include/uapi/linux/media/samsung/exynos-becore-config.h
