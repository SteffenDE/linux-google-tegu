// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Zumapro camera back-end core -- the media entities
 *
 * Two entities and what hangs off them: the subdev whose sink pad the producer
 * links to, and the capture node the processed picture comes out of.  The pad
 * is where the array raster and the mosaic arrive from a real producer, and
 * STREAMON is where they are latched -- both of them together, because the
 * mosaic's one reader is the demosaic's phase and a stream that fails to start
 * has to put back what it wrote.
 *
 * The queue is the other half.  A buffer is checked at buf_prepare, handed to
 * the run loop by a work item, and returned by it; a stream that stops with
 * buffers in flight returns them itself, which is why the two paths meet in
 * becore_video_return_all().
 */

#include <linux/array_size.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#include "exynos-becore-regs.h"
#include "exynos-becore-common.h"
/*
 * The four Bayer orders, numbered the way Samsung's OTF_INPUT_ORDER_BAYER_*
 * enum numbers them, which is what BYR_DNS and BYR_DMSC take: the index into
 * this table is the register value. The three cameras' captured programs write
 * 0, 1 and 2, and the one sensor with a driver settles which is which -- the
 * ultrawide's IMX712 reads out RGGB with both flip bits clear, and its
 * captured phase is 1.
 *
 * The fourth is unused by any of the three and is here because the field is
 * two bits wide and a table with a hole in it is worse than one without.
 */
static const u32 becore_input_codes[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,	/* the main camera's */
	MEDIA_BUS_FMT_SRGGB10_1X10,	/* the ultrawide's */
	MEDIA_BUS_FMT_SBGGR10_1X10,	/* the front's */
	MEDIA_BUS_FMT_SGBRG10_1X10,
};

#define BECORE_INPUT_DEFAULT_CODE	MEDIA_BUS_FMT_SRGGB10_1X10

/* The CFA phase a media-bus code means, or -EINVAL for one we cannot place. */
int becore_bayer_phase(u32 code)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_input_codes); i++)
		if (becore_input_codes[i] == code)
			return i;

	return -EINVAL;
}

/*
 * And back: the media-bus code a phase means, or 0 for a phase there is no
 * code for. Zero is not a media-bus code, so a caller that ignores the range
 * check cannot mistake the answer for one.
 */
u32 becore_bayer_code(u32 phase)
{
	if (phase >= ARRAY_SIZE(becore_input_codes))
		return 0;

	return becore_input_codes[phase];
}

/* ---- the input subdevice ------------------------------------------------ */

/*
 * A sink pad on the producer's graph, and the reason the back end needs one:
 * everything it knows about the frame arriving from the front end is a
 * compiled-in profile, including the Bayer phase, which differs between the
 * three cameras. The front end already negotiates the mosaic on its own pads.
 * The fact exists in the system and the back end could not see it.
 *
 * The pad carries no format of its own. A sink whose producer is a fixed
 * hardware path has nothing to negotiate: what arrives is what the front end
 * sends, so get_fmt reports the remote pad's format and set_fmt is get_fmt.
 * That is deliberate -- offering a settable format here would let userspace
 * tell the driver something the hardware contradicts, with no way to arbitrate.
 */
/* The format the producer says it is sending, or -EPIPE if nothing is. */
static int becore_input_format(struct becore_device *becore,
			       struct v4l2_mbus_framefmt *format)
{
	struct v4l2_subdev_format remote = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct media_pad *pad;
	struct v4l2_subdev *sd;
	int ret;

	pad = media_pad_remote_pad_first(&becore->sink_pad);
	if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
		return -EPIPE;
	sd = media_entity_to_v4l2_subdev(pad->entity);
	remote.pad = pad->index;
	ret = v4l2_subdev_call_state_active(sd, pad, get_fmt, &remote);
	if (ret)
		return ret;

	*format = remote.format;

	return 0;
}

/*
 * Take the producer's mosaic and the raster it sends them on, or keep what the
 * driver already has if there is no producer to ask -- which is the offline
 * loop, where the staged frame is the ultrawide's and so is the default.
 *
 * The two are not kept the same way, and the difference is which of them means
 * anything without a producer. input_code is per-stream state whose only
 * source is the remote pad, so with nothing there it returns to the
 * compiled-in default; the array raster is device state that has to describe
 * the slots whatever is or is not attached, so it is left as it stands.
 *
 * A code the table cannot place is refused rather than guessed. The phase
 * decides which of the four quads the demosaic reads as red, so a wrong one is
 * not a subtle error, and a refusal at STREAMON is a far better failure than a
 * picture with its colours swapped.
 *
 * A raster is refused on three counts, and only the first is about the number
 * itself. It has to be one the register fields can carry; it has to lay a
 * frame out in exactly the slot the producer was handed; and there has to be
 * a crop of it that reaches the chain, which is the one derivation between the
 * two rasters that can fail on its own. Everything else the raster feeds is
 * range-checked where it is encoded.
 *
 * The middle one is an equality and not a bound, which looks stricter than it
 * needs to be and is not. A slot the producer filled stages its whole
 * allocation, and becore_recipe_validate() then requires the staged length to
 * equal what the raster and the profile say the frame is -- so a raster that
 * merely *fits* is accepted here and refuses every frame afterwards, which is
 * a far worse failure than a refusal at STREAMON.
 */
static int becore_latch_input_format(struct becore_device *becore)
{
	struct v4l2_mbus_framefmt format;
	struct becore_raster array;
	struct becore_rect crop;
	int ret;

	lockdep_assert_held(&becore->lock);

	ret = becore_input_format(becore, &format);
	if (ret == -EPIPE) {
		becore->input_code = BECORE_INPUT_DEFAULT_CODE;
		return 0;
	}
	if (ret)
		return ret;
	if (becore_bayer_phase(format.code) < 0) {
		dev_err(becore->dev, "producer sends mosaic 0x%04x, which this driver cannot place\n",
			format.code);
		return -EINVAL;
	}

	array.width = format.width;
	array.height = format.height;
	ret = becore_raster_validate(becore->dev, "array", &array,
				     BECORE_ARRAY_EXTENT_MAX);
	if (ret)
		return ret;
	ret = becore_input_profiles_validate(becore->dev, &array);
	if (ret)
		return ret;
	if (becore_input_allocation_size(&array) !=
	    becore->inputs[0].buffer.size) {
		dev_err(becore->dev,
			"producer sends %ux%u, which lays out in %zu bytes a slot where the slots are %zu\n",
			array.width, array.height,
			becore_input_allocation_size(&array),
			becore->inputs[0].buffer.size);
		return -ENOSPC;
	}
	ret = becore_rgbp_crop(&array, &becore->chain, &crop);
	if (ret) {
		dev_err(becore->dev,
			"no crop of %ux%u reaches the %ux%u chain\n",
			array.width, array.height,
			becore->chain.width, becore->chain.height);
		return ret;
	}

	becore->input_code = format.code;
	becore->array = array;
	/*
	 * And what a frame the producer hands over will be stamped with, which
	 * is why this is unwound with the other two when a stream does not
	 * start.  The check above is a length -- it is the only thing this
	 * driver can hold a pad's raster to -- and a length does not
	 * distinguish two array widths inside one 256-column bucket.  So the
	 * pad *can* report a raster the producer then refuses to send, and
	 * every path where that happens is a path that fails below.  Keeping
	 * what was reported would leave the stamp naming a raster the producer
	 * has just said it does not write.
	 */
	becore->producer_array = array;

	return 0;
}

static int becore_sd_init_state(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state)
{
	struct becore_device *becore = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *sink =
		v4l2_subdev_state_get_format(state, 0);

	sink->code = BECORE_INPUT_DEFAULT_CODE;
	sink->width = becore->array.width;
	sink->height = becore->array.height;
	sink->field = V4L2_FIELD_NONE;
	sink->colorspace = V4L2_COLORSPACE_RAW;
	sink->ycbcr_enc = V4L2_YCBCR_ENC_601;
	sink->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	sink->xfer_func = V4L2_XFER_FUNC_NONE;

	return 0;
}

static int becore_sd_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(becore_input_codes))
		return -EINVAL;
	code->code = becore_input_codes[code->index];

	return 0;
}

/*
 * The producer decides, so a set is a get, and what is reported is what link
 * validation last saw the producer send.
 *
 * **This must not ask the remote.** `v4l2_subdev_link_validate()` locks both
 * subdevs' states and then calls the sink's `get_fmt`, so a `get_fmt` that
 * fetches the source's format takes a lock its own caller is holding. That is
 * a self-deadlock, and it hangs `STREAMON` on the *producer's* video node --
 * where the back end is only a pad on the graph and nothing about it is being
 * used. The stored format is kept in step from `link_validate` below instead.
 */
static int becore_sd_get_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *format)
{
	if (format->pad)
		return -EINVAL;
	format->format = *v4l2_subdev_state_get_format(state, 0);

	return 0;
}

/*
 * Take the producer's format rather than compare against it.
 *
 * A sink whose producer is a fixed hardware path has nothing to negotiate:
 * what arrives is what the front end sends, so the default validation -- which
 * refuses a link whose two ends disagree -- would refuse every pipeline until
 * userspace had told the back end what the hardware was already doing.
 *
 * Nothing is refused here, and that is deliberate too. The back end cannot
 * place every mosaic or work at every raster, but the producer's raw path does
 * not go through the back end at all, so neither is a reason to stop a raw
 * capture. becore_latch_input_format() refuses both at the back end's own
 * STREAMON, which is where it matters.
 */
static int becore_sd_link_validate(struct v4l2_subdev *sd,
				   struct media_link *link,
				   struct v4l2_subdev_format *source_fmt,
				   struct v4l2_subdev_format *sink_fmt)
{
	struct v4l2_subdev_state *state = v4l2_subdev_get_locked_active_state(sd);

	if (state)
		*v4l2_subdev_state_get_format(state, 0) = source_fmt->format;

	return 0;
}

static const struct v4l2_subdev_pad_ops becore_subdev_pad_ops = {
	.enum_mbus_code = becore_sd_enum_mbus_code,
	.get_fmt = becore_sd_get_fmt,
	.set_fmt = becore_sd_get_fmt,
	.link_validate = becore_sd_link_validate,
};

static const struct v4l2_subdev_ops becore_subdev_ops = {
	.pad = &becore_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops becore_subdev_internal_ops = {
	.init_state = becore_sd_init_state,
};

static const struct media_entity_operations becore_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* ---- processed NV21 capture queue -------------------------------------- */

static void becore_video_fill_pix(const struct becore_raster *output,
				  struct v4l2_pix_format *pix)
{
	pix->width = output->width;
	pix->height = output->height;
	pix->pixelformat = V4L2_PIX_FMT_NV21;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = becore_mcsc_output_stride(output);
	pix->sizeimage = becore_mcsc_output_active_size(output);
	/* The captured recipe uses a full-range BT.601 RGB-to-YUV matrix. */
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->flags = 0;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

void becore_video_return_all(struct becore_device *becore,
			     enum vb2_buffer_state state)
{
	struct becore_video_buffer *buf, *tmp;
	LIST_HEAD(done);

	spin_lock_irq(&becore->queue_lock);
	list_splice_tail_init(&becore->queued_outputs, &done);
	spin_unlock_irq(&becore->queue_lock);

	list_for_each_entry_safe(buf, tmp, &done, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
}

/*
 * The balance a stream starts at, kept where a slot can be seeded from it
 * without taking the control handler's lock under the device's.  Both controls
 * are grabbed for the length of a stream, so this cannot move under a frame.
 */
static int becore_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct becore_device *becore =
		container_of(ctrl->handler, struct becore_device, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_RED_BALANCE:
		WRITE_ONCE(becore->stream_gains.red, ctrl->val);
		return 0;
	case V4L2_CID_BLUE_BALANCE:
		WRITE_ONCE(becore->stream_gains.blue, ctrl->val);
		return 0;
	}

	return -EINVAL;
}

static const struct v4l2_ctrl_ops becore_ctrl_ops = {
	.s_ctrl = becore_s_ctrl,
};

static void becore_video_controls_snapshot(struct becore_device *becore,
					   struct exynos_becore_input_stream_config *config)
{
	v4l2_ctrl_lock(becore->red_balance);
	config->red_balance = becore->red_balance->val;
	config->blue_balance = becore->blue_balance->val;
	__v4l2_ctrl_grab(becore->red_balance, true);
	__v4l2_ctrl_grab(becore->blue_balance, true);
	v4l2_ctrl_unlock(becore->red_balance);
}

void becore_video_controls_ungrab(struct becore_device *becore)
{
	v4l2_ctrl_lock(becore->red_balance);
	__v4l2_ctrl_grab(becore->red_balance, false);
	__v4l2_ctrl_grab(becore->blue_balance, false);
	v4l2_ctrl_unlock(becore->red_balance);
}

static void becore_video_discard_ready(struct becore_device *becore)
{
	unsigned int i;

	mutex_lock(&becore->lock);
	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_input_slot *slot = &becore->inputs[i];

		if (slot->state != BECORE_INPUT_READY)
			continue;
		slot->state = BECORE_INPUT_FREE;
		slot->buffer.staged_bytes = 0;
		slot->ready_sequence = 0;
	}
	mutex_unlock(&becore->lock);
}

void becore_params_drain_idle(struct becore_device *becore)
{
	if (!READ_ONCE(becore->video_streaming))
		schedule_work(&becore->params_work);
}

static void becore_video_stop_producer(struct becore_device *becore)
{
	struct exynos_becore_input *input = NULL;

	mutex_lock(&becore->lock);
	if (becore->producer_streaming) {
		becore->producer_streaming = false;
		input = becore_input_callback_get(becore);
	}
	mutex_unlock(&becore->lock);

	if (!input)
		return;
	input->ops->stop_streaming(input->producer_data);
	becore_video_discard_ready(becore);
	becore_input_callback_put(input);
}

static void becore_video_fail(struct becore_device *becore,
			      struct becore_video_buffer *buf)
{
	mutex_lock(&becore->lock);
	becore->video_streaming = false;
	becore_stream_power_put(becore);
	mutex_unlock(&becore->lock);

	becore_video_stop_producer(becore);
	becore_video_controls_ungrab(becore);
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	vb2_queue_error(&becore->queue);
	becore_video_return_all(becore, VB2_BUF_STATE_ERROR);
}

void becore_video_work(struct work_struct *work)
{
	struct becore_device *becore =
		container_of(work, struct becore_device, video_work);

	for (;;) {
		struct becore_video_buffer *buf;
		int ret;

		mutex_lock(&becore->lock);
		if (!becore->video_streaming) {
			mutex_unlock(&becore->lock);
			return;
		}
		mutex_unlock(&becore->lock);

		spin_lock_irq(&becore->queue_lock);
		if (list_empty(&becore->queued_outputs)) {
			spin_unlock_irq(&becore->queue_lock);
			return;
		}
		buf = list_first_entry(&becore->queued_outputs,
				       struct becore_video_buffer, list);
		list_del(&buf->list);
		spin_unlock_irq(&becore->queue_lock);

		ret = becore_run_frame(becore, BECORE_RGBP_INPUT_SBWC,
				       BECORE_YUVP_OUTPUT_SBWCL,
				       true, &buf->vb.vb2_buf, false, true);
		if (ret == -ENODATA || ret == -EBUSY) {
			spin_lock_irq(&becore->queue_lock);
			list_add(&buf->list, &becore->queued_outputs);
			spin_unlock_irq(&becore->queue_lock);
			return;
		}
		if (ret) {
			bool stopping;

			mutex_lock(&becore->lock);
			stopping = !becore->video_streaming;
			mutex_unlock(&becore->lock);
			if (ret == -ECANCELED && stopping) {
				vb2_buffer_done(&buf->vb.vb2_buf,
						VB2_BUF_STATE_ERROR);
				return;
			}
			becore_video_fail(becore, buf);
			return;
		}

		buf->vb.vb2_buf.timestamp = ktime_get_ns();
		mutex_lock(&becore->lock);
		buf->vb.sequence = becore->video_sequence++;
		mutex_unlock(&becore->lock);
		buf->vb.field = V4L2_FIELD_NONE;
		vb2_set_plane_payload(&buf->vb.vb2_buf, 0,
				      becore->active_capture_size);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	}
}

static int becore_queue_setup(struct vb2_queue *q, unsigned int *nbufs,
			      unsigned int *nplanes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct becore_device *becore = vb2_get_drv_priv(q);
	struct v4l2_pix_format pix;

	becore_video_fill_pix(&becore->scaled, &pix);
	if (*nplanes) {
		if (*nplanes != 1 || sizes[0] < pix.sizeimage)
			return -EINVAL;
		return 0;
	}

	*nplanes = 1;
	sizes[0] = pix.sizeimage;

	return 0;
}

static int becore_buf_prepare(struct vb2_buffer *vb)
{
	struct becore_device *becore = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_pix_format pix;
	dma_addr_t dma;

	becore_video_fill_pix(&becore->scaled, &pix);
	if (vb2_plane_size(vb, 0) < pix.sizeimage)
		return -EINVAL;

	/*
	 * An imported buffer's address is not the driver's to choose, so every
	 * property MCSC needs of it is checked here rather than at the point of
	 * use: a run that refuses a buffer takes the whole stream down with it,
	 * where QBUF refusing one costs the caller only that buffer.
	 */
	dma = vb2_dma_contig_plane_dma_addr(vb, 0);
	if (!dma || upper_32_bits(dma) ||
	    upper_32_bits(dma + becore_mcsc_output_active_size(&becore->scaled) - 1)) {
		dev_err_ratelimited(becore->dev,
				    "buffer at %pad is outside 32-bit DMA\n",
				    &dma);
		return -EINVAL;
	}
	/*
	 * The raw node refuses a misaligned import because the front end drops
	 * the low bits rather than faulting, and MCSC's own stride is a
	 * multiple of this, so hold a capture to the same rule.  Every buffer
	 * either node allocates is page-aligned and cannot reach this.
	 */
	if (!IS_ALIGNED(dma, BECORE_CAPTURE_ALIGN)) {
		dev_err_ratelimited(becore->dev,
				    "buffer at %pad is not %u-byte aligned\n",
				    &dma, BECORE_CAPTURE_ALIGN);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, pix.sizeimage);

	return 0;
}

static void becore_buf_queue(struct vb2_buffer *vb)
{
	struct becore_device *becore = vb2_get_drv_priv(vb->vb2_queue);
	struct becore_video_buffer *buf =
		to_becore_video_buffer(to_vb2_v4l2_buffer(vb));

	spin_lock_irq(&becore->queue_lock);
	list_add_tail(&buf->list, &becore->queued_outputs);
	spin_unlock_irq(&becore->queue_lock);

	if (READ_ONCE(becore->video_streaming))
		schedule_work(&becore->video_work);
}

static int becore_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct becore_device *becore = vb2_get_drv_priv(q);
	struct exynos_becore_input_stream_config stream_config;
	struct exynos_becore_input *input;
	struct becore_raster was_array;
	struct becore_raster was_producer_array;
	u32 was_code;
	int ret;

	mutex_lock(&becore->lock);
	was_array = becore->array;
	was_producer_array = becore->producer_array;
	was_code = becore->input_code;
	if (becore->video_streaming || becore->running) {
		ret = -EBUSY;
		goto unlock;
	}
	/*
	 * A debug override must not be able to leak into an ordinary capture,
	 * and a forgotten one is easy to leave behind, so the ordinary path
	 * refuses to start rather than quietly running a swept program.
	 */
	if (becore->override_count) {
		ret = -EPERM;
		goto unlock;
	}
	/*
	 * Latch the producer's format first, because everything below is
	 * checked against it: the record walk evaluates every word the array
	 * raster derives, and doing that before the latch would prove it of
	 * the previous stream's raster.
	 */
	ret = becore_latch_input_format(becore);
	if (ret)
		goto unlock;
	ret = becore_recipe_records_validate(becore, false);
	if (ret)
		goto unlock;
	ret = becore_mcsc_recipe_validate(becore);
	if (ret)
		goto unlock;
	if (becore->grid.staged_bytes != BECORE_GRID_SIZE) {
		ret = -EINVAL;
		goto unlock;
	}
	if (becore->reset_failed) {
		ret = -EIO;
		goto unlock;
	}
	input = becore_input_callback_get(becore);
	if (!input) {
		ret = -ENODEV;
		goto unlock;
	}

	ret = becore_stream_power_get(becore);
	if (ret) {
		becore_input_callback_put(input);
		goto unlock;
	}

	becore->video_sequence = 0;
	/* The shared driver-owned output now becomes the video bounce buffer. */
	becore->completed_generation = 0;
	becore->completed_output_size = 0;
	becore_video_controls_snapshot(becore, &stream_config);
	becore->video_streaming = true;
	mutex_unlock(&becore->lock);

	ret = input->ops->start_streaming(input->producer_data, &stream_config);
	if (ret) {
		becore_input_callback_put(input);
		mutex_lock(&becore->lock);
		becore->video_streaming = false;
		/*
		 * The producer refused, so nothing it said about the frame it
		 * sends was ever acted on -- and today refusing is what it does
		 * for every raster but one, because the program that writes a
		 * slot is a captured PDMA recipe selected by exactly matching
		 * its source pad. Leaving what was latched behind would let the
		 * offline loop, which has no producer to disagree with, encode
		 * a geometry no frame was ever taken at, in a phase no sensor
		 * read out.
		 */
		becore->array = was_array;
		becore->producer_array = was_producer_array;
		becore->input_code = was_code;
		becore_stream_power_put(becore);
		mutex_unlock(&becore->lock);
		cancel_work_sync(&becore->video_work);
		becore_video_controls_ungrab(becore);
		becore_video_return_all(becore, VB2_BUF_STATE_QUEUED);
		becore_params_drain_idle(becore);
		return ret;
	}

	mutex_lock(&becore->lock);
	if (becore->input_producer == input && !input->disconnected &&
	    becore->video_streaming) {
		becore->producer_streaming = true;
	} else {
		/* The producer went away under the dropped lock; same rule. */
		becore->array = was_array;
		becore->producer_array = was_producer_array;
		becore->input_code = was_code;
		ret = -ENODEV;
	}
	mutex_unlock(&becore->lock);
	if (ret)
		input->ops->stop_streaming(input->producer_data);
	becore_input_callback_put(input);
	if (ret) {
		mutex_lock(&becore->lock);
		becore_stream_power_put(becore);
		mutex_unlock(&becore->lock);
		cancel_work_sync(&becore->video_work);
		becore_video_controls_ungrab(becore);
		becore_video_return_all(becore, VB2_BUF_STATE_QUEUED);
		/*
		 * vb2 does not call stop_streaming after a failed start, so
		 * anything queued while video_streaming was briefly true would
		 * otherwise wait for a frame that is not coming.
		 */
		becore_params_drain_idle(becore);
		return ret;
	}
	schedule_work(&becore->video_work);

	return 0;

unlock:
	/*
	 * Nothing was proven at what was latched, so none of it persists. The
	 * mosaic matters here as much as the raster: its one reader is the
	 * demosaic's phase, the offline loop never re-latches, and a phase left
	 * behind by a stream that never started is a picture with its colours
	 * swapped and nothing saying so.
	 */
	becore->array = was_array;
	becore->producer_array = was_producer_array;
	becore->input_code = was_code;
	mutex_unlock(&becore->lock);
	becore_video_return_all(becore, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void becore_stop_streaming(struct vb2_queue *q)
{
	struct becore_device *becore = vb2_get_drv_priv(q);
	unsigned long flags;
	bool cancel = false;

	mutex_lock(&becore->lock);
	becore->video_streaming = false;
	spin_lock_irqsave(&becore->run_lock, flags);
	if (becore->running) {
		becore->abort_run = true;
		cancel = true;
	}
	spin_unlock_irqrestore(&becore->run_lock, flags);
	if (cancel)
		complete(&becore->run_completion);
	mutex_unlock(&becore->lock);

	cancel_work_sync(&becore->video_work);
	becore_video_stop_producer(becore);
	becore_video_controls_ungrab(becore);
	becore_video_return_all(becore, VB2_BUF_STATE_ERROR);
	mutex_lock(&becore->lock);
	becore_stream_power_put(becore);
	mutex_unlock(&becore->lock);
	/* Nothing is going to run a frame for a queued parameters buffer now. */
	becore_params_drain_idle(becore);
}

static const struct vb2_ops becore_vb2_ops = {
	.queue_setup = becore_queue_setup,
	.buf_prepare = becore_buf_prepare,
	.buf_queue = becore_buf_queue,
	.start_streaming = becore_start_streaming,
	.stop_streaming = becore_stop_streaming,
};

static int becore_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strscpy(cap->driver, "exynos-becore", sizeof(cap->driver));
	strscpy(cap->card, "zumapro BE-core MCSC NV21", sizeof(cap->card));

	return 0;
}

static int becore_enum_fmt(struct file *file, void *priv,
			   struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_NV21;

	return 0;
}

static int becore_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct becore_device *becore = video_drvdata(file);

	becore_video_fill_pix(&becore->scaled, &f->fmt.pix);

	return 0;
}

/*
 * What the scaler can be asked for, and the chain it would read to deliver it.
 *
 * **The chain follows the request's aspect ratio.** It is the only thing that
 * decides the output's: becore_rgbp_crop() takes the largest centred window of
 * the array with the chain's aspect, and MCSC scales the chain to the output,
 * so a chain of a different shape from the output is two scalers working at
 * two ratios -- an anamorphic squeeze, not a crop. Deriving it here is what
 * makes every size this function reports back an honest one, and it is why the
 * bounds below do not depend on which size was set last.
 *
 * A request whose aspect the chain already has costs nothing: 4:3 lands on
 * 4160x3120, which is where the chain has always been.
 *
 * Two extents rather than a list: MCSC's DJAG takes any ratio its Q20 register
 * can hold, so the sizes between the bounds are as real as the bounds. The
 * ceiling is the chain -- **not** because the register refuses to go above it,
 * but because nothing has established that this scaler upscales: every one of
 * the sixteen captured output rasters is a downscale of its chain, so offering
 * more would be advertising something nobody has run.
 *
 * The floor is the ratio's own: past a 4096x downscale the Q20 value does not
 * fit, which for a 4160-wide chain is a raster of two.
 * %BECORE_SCALED_EXTENT_MIN is the larger of that and what an image with
 * subsampled chroma needs, and it does not move with the chain because the
 * chain cannot get near it.
 *
 * Both extents are even because the chroma plane is subsampled in both
 * directions; the width needs no more alignment than that, since the stride
 * pads to 64 on its own.
 *
 * The aspect is taken from the size the *caller* asked for rather than from
 * the clamped one, so a request larger than the pipeline can deliver still
 * says which shape it wanted: 8000x4500 comes back as the largest 16:9 there
 * is rather than as the largest 4:3.
 */
/* One extent, into what a chain of this size can carry. */
static u32 becore_video_clamp(u32 want, u32 chain)
{
	return clamp_t(u32, ALIGN_DOWN(want, 2), BECORE_SCALED_EXTENT_MIN,
		       ALIGN_DOWN(chain, 2));
}

static void becore_video_negotiate(const struct becore_device *becore,
				   struct becore_raster *scaled,
				   struct becore_raster *chain)
{
	struct becore_raster derived;
	struct becore_raster candidate;

	/*
	 * The fallback is the compiled-in box, and deliberately not the chain
	 * in force.  Keeping what is there would buy nothing -- `S_FMT`
	 * replaces the chain either way -- and what is there is not this
	 * node's to trust: the debugfs geometry file sets one for the offline
	 * loop and is bounded only by what the *hardware* accepts.  An 8x8
	 * chain set there would invert the clamp below, and a clamp with its
	 * bounds the wrong way round returns the ceiling, so this node would
	 * answer 8x8 while its size enumeration promises thirty-two.  The box
	 * is the one chain that always works.
	 */
	chain->width = BECORE_CHAIN_WIDTH;
	chain->height = BECORE_CHAIN_HEIGHT;

	/*
	 * And a derived chain is taken only if the *pair* it makes is one
	 * %VIDIOC_S_FMT would accept.  Deriving it is arithmetic; whether it
	 * works with an output is three ratio checks, and running the same
	 * function the set runs is what stops `TRY_FMT` reporting a size
	 * `S_FMT` then rejects -- by construction rather than by this
	 * function's author having worked out which of the three binds.
	 *
	 * The candidate is clamped before it is checked, because the clamped
	 * one is what would be used: DJAG's ratio is the chain over the
	 * *output*, so checking the unclamped request would be checking a
	 * geometry nobody was going to run.
	 */
	if (!becore_chain_for_output(scaled, &derived)) {
		candidate.width = becore_video_clamp(scaled->width,
						     derived.width);
		candidate.height = becore_video_clamp(scaled->height,
						      derived.height);
		if (!becore_geometry_ratios(NULL, &becore->array, &derived,
					    &candidate)) {
			*chain = derived;
			*scaled = candidate;
			return;
		}
	}

	scaled->width = becore_video_clamp(scaled->width, chain->width);
	scaled->height = becore_video_clamp(scaled->height, chain->height);
}

/*
 * The whole geometry, said in the units the selection API says it in.
 *
 * Three rectangles, and what each one means here:
 *
 * %V4L2_SEL_TGT_CROP_BOUNDS and %V4L2_SEL_TGT_CROP_DEFAULT are both the array
 * -- "what the driver writer considers the complete picture", which for a
 * Bayer array with no optical-black region is all of it.  They are equal on
 * purpose: the API's own way of asking "is this device cropping?" is to
 * compare the active rectangle against the default, and a default that moved
 * with the format would answer no however much was being cropped.
 *
 * %V4L2_SEL_TGT_CROP is what becore_rgbp_crop() derives from the chain in
 * force, so it moves with the requested aspect ratio: a 4:3 output leaves the
 * array's full height, and a 16:9 one comes back 4208x2368 at (0, 376), which
 * is the vendor's own 16:9 readout.  That difference from the default *is* the
 * policy this driver picks on the consumer's behalf, and saying it here is the
 * whole point of implementing this ioctl.
 *
 * The compose targets are the buffer's, all four the whole of it, because
 * nothing here letterboxes and nothing writes outside the picture.  They are
 * worth reporting rather than refusing: the API's way of asking "is this
 * device scaling, and by how much?" is to compare the crop against the
 * compose, and without them an application has to guess from the format.
 *
 * %V4L2_SEL_TGT_COMPOSE_PADDED is among them, and it is the one target here
 * that states something about the *hardware* rather than about a choice this
 * driver made: it is "the active area and all padding pixels that are inserted
 * or modified by hardware", and the scaler's output stride is the width
 * rounded up to 64, so most formats have padding columns beyond the picture.
 * Reporting it equal to the compose rectangle asserts the write-DMA never
 * writes them, which is measured rather than assumed -- poisoning every buffer
 * before queueing it and reading back what survives leaves the padding
 * untouched at four geometries with 8, 20, 32 and 48 padding columns, over
 * 2.1 million padding bytes, while the picture columns carry a live scene.
 *
 * **Read-only.**  Nothing here can set a rectangle, and the specification's
 * "if cropping (composing) is not supported then the active rectangle is not
 * mutable and it is always equal to the bounds rectangle" does not fit that.
 * It fits the compose side, where the two *are* equal; the crop side deviates,
 * because this hardware genuinely crops and it is just not the application's
 * choice yet.  Reporting the bounds instead
 * would be a lie about where the picture comes from, and a no-op
 * %VIDIOC_S_SELECTION that quietly ignored a rectangle would be a worse one --
 * a consumer asking for digital zoom would get none and no error.  Making it
 * settable is the change that separates the aspect ratio from the field of
 * view, and libcamera's own scaler-crop control maps onto exactly this
 * rectangle.
 *
 * Implementing this also makes %VIDIOC_CROPCAP and %VIDIOC_G_CROP valid on
 * this node, which the V4L2 core answers out of the targets above.
 */
static int becore_g_selection(struct file *file, void *priv,
			      struct v4l2_selection *sel)
{
	struct becore_device *becore = video_drvdata(file);
	struct becore_rect crop;
	int ret = 0;

	if (sel->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/*
	 * @video_lock, which the core holds here, would be enough on its own --
	 * both writers of these three take it.  @lock is taken because it is
	 * what every reader outside this node uses for them, so that a future
	 * writer that does not go through the video device stays covered.
	 */
	mutex_lock(&becore->lock);
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = becore->array.width;
		sel->r.height = becore->array.height;
		break;
	case V4L2_SEL_TGT_CROP:
		/*
		 * Cannot fail for a pair this driver installed: every path
		 * that sets one runs becore_rgbp_crop() first, and the
		 * compiled-in pair probe starts from yields a valid crop too.
		 * So the error is defensive, and %EINVAL rather than what
		 * becore_rgbp_crop() returns, because %ERANGE is a set-side
		 * answer about @flags.
		 */
		if (becore_rgbp_crop(&becore->array, &becore->chain, &crop)) {
			ret = -EINVAL;
			break;
		}
		sel->r.left = crop.x;
		sel->r.top = crop.y;
		sel->r.width = crop.width;
		sel->r.height = crop.height;
		break;
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = becore->scaled.width;
		sel->r.height = becore->scaled.height;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&becore->lock);

	return ret;
}

static int becore_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct becore_device *becore = video_drvdata(file);
	struct becore_raster scaled = {
		.width = f->fmt.pix.width,
		.height = f->fmt.pix.height,
	};
	struct becore_raster chain;

	becore_video_negotiate(becore, &scaled, &chain);
	becore_video_fill_pix(&scaled, &f->fmt.pix);

	return 0;
}

/*
 * The scaled raster is what a consumer chooses, and the chain comes with it.
 *
 * The array stays where it is -- it is the producer's, not this node's. The
 * chain is derived, because it is the only thing that decides the output's
 * aspect ratio and a consumer that asks for 16:9 is asking for 16:9 rather
 * than for a squeezed 4:3. What that changes with it is the field of view:
 * becore_rgbp_crop() narrows the array to the chain's shape, so a 16:9 request
 * is a 16:9 *crop* of the sensor and not a letterbox of the whole of it. That
 * is a choice this driver makes on the consumer's behalf and states here;
 * nothing in this interface lets the two be separated yet.
 *
 * Reallocating the surfaces is the part that can fail, and
 * becore_geometry_apply() is what puts the previous rasters back if it does. A
 * request the clamp already satisfied can still be refused there -- the DJAG
 * and binning ratio checks are exact where the clamp is a bound -- so the
 * return value matters and the format is only reported back once it has been
 * applied.
 */
static int becore_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct becore_device *becore = video_drvdata(file);
	struct becore_raster scaled = {
		.width = f->fmt.pix.width,
		.height = f->fmt.pix.height,
	};
	struct becore_raster chain;
	int ret;

	if (vb2_is_busy(&becore->queue))
		return -EBUSY;

	becore_video_negotiate(becore, &scaled, &chain);

	/*
	 * @video_lock is already held: it is the video device's own, so the
	 * queue cannot start streaming underneath this.  What it does not cover
	 * is the offline loop, which runs frames of its own through the same
	 * surfaces -- so the same guards the debugfs geometry file uses apply
	 * here, and for the same reason.
	 */
	mutex_lock(&becore->lock);
	if (becore->reset_failed || becore->output_quarantined)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else if (scaled.width == becore->scaled.width &&
		 scaled.height == becore->scaled.height &&
		 chain.width == becore->chain.width &&
		 chain.height == becore->chain.height)
		ret = 0;
	else
		ret = becore_geometry_apply(becore, &becore->array,
					    &chain, &scaled);
	mutex_unlock(&becore->lock);
	if (ret)
		return ret;

	becore_video_fill_pix(&becore->scaled, &f->fmt.pix);

	return 0;
}

static int becore_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || fsize->pixel_format != V4L2_PIX_FMT_NV21)
		return -EINVAL;

	/*
	 * The box the chain is fitted inside rather than the chain in force,
	 * because the chain now follows whatever aspect ratio is asked for --
	 * so what is reachable does not depend on which size was set last, and
	 * an enumeration that quoted the current chain would shrink after a
	 * 16:9 request and mislead the next caller.
	 *
	 * Not every pair inside the range is reachable at once: the height at
	 * the full width is the 4:3 one. That is what VIDIOC_TRY_FMT is for,
	 * and it is exact where this is a bound.
	 */
	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = BECORE_SCALED_EXTENT_MIN;
	fsize->stepwise.max_width = ALIGN_DOWN(BECORE_CHAIN_WIDTH, 2);
	fsize->stepwise.step_width = 2;
	fsize->stepwise.min_height = BECORE_SCALED_EXTENT_MIN;
	fsize->stepwise.max_height = ALIGN_DOWN(BECORE_CHAIN_HEIGHT, 2);
	fsize->stepwise.step_height = 2;

	return 0;
}

static const struct v4l2_ioctl_ops becore_ioctl_ops = {
	.vidioc_querycap = becore_querycap,
	.vidioc_enum_fmt_vid_cap = becore_enum_fmt,
	.vidioc_g_fmt_vid_cap = becore_g_fmt,
	.vidioc_s_fmt_vid_cap = becore_s_fmt,
	.vidioc_try_fmt_vid_cap = becore_try_fmt,
	.vidioc_enum_framesizes = becore_enum_framesizes,
	.vidioc_g_selection = becore_g_selection,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

const struct v4l2_file_operations becore_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

static const struct video_device becore_video_template = {
	.name = "exynos-becore MCSC NV21 capture",
	.fops = &becore_fops,
	.ioctl_ops = &becore_ioctl_ops,
	.release = video_device_release_empty,
	.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING,
	.vfl_dir = VFL_DIR_RX,
};

static void becore_video_cleanup(void *data)
{
	struct becore_device *becore = data;

	/*
	 * Both drivers are built in and suppress unbind, so this runs only on
	 * a probe-failure unwind, before any producer has put these entities on
	 * a graph and so before either node has streamed.  The two cancels are
	 * for the offline loop, which is the one thing that can queue work with
	 * no node registered at all.
	 */
	cancel_work_sync(&becore->params_work);
	cancel_work_sync(&becore->video_work);
	media_entity_cleanup(&becore->params_vdev.entity);
	media_entity_cleanup(&becore->vdev.entity);
	v4l2_subdev_cleanup(&becore->sd);
	media_entity_cleanup(&becore->sd.entity);
	v4l2_ctrl_handler_free(&becore->ctrl_handler);
}

/*
 * Everything a video node needs except the v4l2_device it hangs from, which
 * this device does not own -- see exynos_becore_input_register_graph().
 */
int becore_video_init(struct becore_device *becore)
{
	struct v4l2_ctrl_handler *handler = &becore->ctrl_handler;
	struct vb2_queue *q = &becore->queue;
	int ret;

	ret = v4l2_ctrl_handler_init(handler, 2);
	if (ret)
		return ret;
	becore->red_balance =
		v4l2_ctrl_new_std(handler, &becore_ctrl_ops, V4L2_CID_RED_BALANCE,
				  EXYNOS_BECORE_WBG_GAIN_MIN_Q12,
				  EXYNOS_BECORE_WBG_GAIN_MAX_Q12, 1,
				  EXYNOS_BECORE_WBG_RED_DEFAULT_Q12);
	becore->blue_balance =
		v4l2_ctrl_new_std(handler, &becore_ctrl_ops, V4L2_CID_BLUE_BALANCE,
				  EXYNOS_BECORE_WBG_GAIN_MIN_Q12,
				  EXYNOS_BECORE_WBG_GAIN_MAX_Q12, 1,
				  EXYNOS_BECORE_WBG_BLUE_DEFAULT_Q12);
	/*
	 * The control op has not run yet, so seed these with the same defaults
	 * the controls were created at.  The offline loop encodes without ever
	 * setting one.  The greens are here and no control writes them: nothing
	 * this side of the interface can estimate an illuminant, and the two
	 * gains a V4L2 control names are the two the vendor's own AWB moves.
	 */
	becore->stream_gains = (struct exynos_becore_input_gains) {
		.red = EXYNOS_BECORE_WBG_RED_DEFAULT_Q12,
		.green_red = EXYNOS_BECORE_WBG_UNITY_Q12,
		.green_blue = EXYNOS_BECORE_WBG_UNITY_Q12,
		.blue = EXYNOS_BECORE_WBG_BLUE_DEFAULT_Q12,
	};
	if (handler->error) {
		ret = handler->error;
		goto err_ctrl;
	}

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->dev = becore->dev;
	q->drv_priv = becore;
	q->ops = &becore_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct becore_video_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers = 1;
	q->lock = &becore->video_lock;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_ctrl;

	/* The producer's v4l2_device is filled in when it registers. */
	becore->vdev = becore_video_template;
	becore->vdev.ctrl_handler = &becore->ctrl_handler;
	becore->vdev.queue = q;
	becore->vdev.lock = &becore->video_lock;
	becore->vdev.entity.function = MEDIA_ENT_F_IO_V4L;
	video_set_drvdata(&becore->vdev, becore);

	/*
	 * The input subdevice is initialised here and registered later, by the
	 * producer, on the producer's graph: this device has nothing to link it
	 * to on its own.
	 */
	v4l2_subdev_init(&becore->sd, &becore_subdev_ops);
	becore->sd.internal_ops = &becore_subdev_internal_ops;
	becore->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	becore->sd.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	becore->sd.entity.ops = &becore_subdev_entity_ops;
	becore->sd.owner = THIS_MODULE;
	becore->sd.dev = becore->dev;
	strscpy(becore->sd.name, "exynos-becore input", sizeof(becore->sd.name));
	v4l2_set_subdevdata(&becore->sd, becore);
	becore->input_code = BECORE_INPUT_DEFAULT_CODE;

	becore->sink_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&becore->sd.entity, 1, &becore->sink_pad);
	if (ret)
		goto err_ctrl;

	ret = v4l2_subdev_init_finalize(&becore->sd);
	if (ret)
		goto err_sd_entity;

	becore->vdev_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&becore->vdev.entity, 1,
				     &becore->vdev_pad);
	if (ret)
		goto err_sd;

	ret = becore_params_init(becore);
	if (ret)
		goto err_entity;

	ret = devm_add_action_or_reset(becore->dev,
				       becore_video_cleanup, becore);
	if (ret)
		return ret;
	becore->video_ready = true;

	return 0;

err_entity:
	media_entity_cleanup(&becore->vdev.entity);
err_sd:
	v4l2_subdev_cleanup(&becore->sd);
err_sd_entity:
	media_entity_cleanup(&becore->sd.entity);
err_ctrl:
	v4l2_ctrl_handler_free(&becore->ctrl_handler);
	return ret;
}
