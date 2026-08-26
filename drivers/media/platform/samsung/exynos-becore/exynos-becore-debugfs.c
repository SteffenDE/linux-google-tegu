// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Zumapro camera back-end core -- the bring-up surface
 *
 * Everything here is debugfs, and debugfs is where this driver keeps the
 * things that are deliberately not an ABI: upload a validated program, stage a
 * compressed frame and a statistics grid, run one offline frame into a
 * driver-owned allocation, and read back the register lists that were encoded
 * for it.  Userspace gets none of this through V4L2, and nothing here is meant
 * to survive into a camera stack.
 *
 * It is also the strongest oracle the project has.  The command list a run
 * encodes is what the *driver* wrote rather than what the sensor saw, so the
 * offline loop reproduces byte for byte in a dark room, and every claim about
 * this back end's register arithmetic has been settled by comparing those
 * bytes against the vendor's own captured programs.
 *
 * Three of them change how a program is encoded rather than supplying one or
 * reading one back.  becore_geometry_apply() takes the three rasters apart and
 * puts them back, which is what turned a failure at a second geometry from
 * three unknowns into one echo each; bayer_phase states the mosaic the staged
 * frame was read out on, which is the one thing about it the loop cannot work
 * out for itself; the override file substitutes a register's value into every
 * record, which is how a field's meaning gets swept when no capture
 * distinguishes the candidates.
 */

#include <linux/array_size.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/minmax.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "exynos-becore-regs.h"
#include "exynos-becore-common.h"
#include "exynos-becore-recipe.h"
#include "exynos-becore-gtnr-recipe.h"
#include "exynos-becore-mcsc-recipe.h"
static const char * const becore_timing_names[BECORE_TIMING_PHASE_COUNT] = {
	[BECORE_TIMING_RESUME] = "resume",
	[BECORE_TIMING_ENCODE] = "encode",
	[BECORE_TIMING_ARM] = "arm",
	[BECORE_TIMING_STAGE1] = "stage1",
	[BECORE_TIMING_STAGE2] = "stage2",
	[BECORE_TIMING_CRC] = "crc",
	[BECORE_TIMING_SUSPEND] = "suspend",
	[BECORE_TIMING_FRAME] = "frame",
};

/*
 * @slot is the input slot @staged belongs to, or NULL for the objects that are
 * not slots.  It is passed rather than recovered from @staged because a stage
 * has to be recorded as having dirtied the slot, and recognising the slot by
 * comparing pointers would silently record the wrong one -- or none -- the day
 * a second slot gains a staging file.
 */
static ssize_t becore_stage_write(struct becore_device *becore,
				  const char __user *buf, size_t count,
				  loff_t *ppos, void *staged, size_t capacity,
				  size_t *staged_bytes, u32 *generation,
				  struct becore_input_slot *slot)
{
	ssize_t ret = count;

	if (!count)
		return 0;

	mutex_lock(&becore->lock);
	if (becore->reset_failed) {
		ret = -EIO;
		goto unlock;
	}
	if (becore->video_streaming) {
		ret = -EBUSY;
		goto unlock;
	}
	if (becore->running) {
		ret = -EBUSY;
		goto unlock;
	}
	if (slot && slot->state != BECORE_INPUT_FREE) {
		ret = -EBUSY;
		goto unlock;
	}
	if (*ppos < 0 || *ppos > capacity) {
		ret = -EINVAL;
		goto unlock;
	}
	if (count > capacity - *ppos) {
		ret = -EFBIG;
		goto unlock;
	}
	if (*ppos == 0) {
		*staged_bytes = 0;
		/*
		 * A stage that starts at zero is what says which raster the
		 * frame is at: whoever writes it writes it for the geometry
		 * selected now.  Taken here rather than at the end so that a
		 * geometry change part-way through a multi-write stage leaves
		 * the raster the earlier bytes were written for, and the run
		 * refuses rather than encoding for a raster half the frame is
		 * not at.
		 */
		if (slot) {
			slot->raster = becore->array;
			/*
			 * And the balance, for the same reason and at the same
			 * moment: a staged frame has no producer to state the
			 * gains it was taken through, so what stands for them
			 * is what the controls hold when the stage begins.
			 */
			slot->gains = becore_stream_gains(becore);
		}
	}
	if (*ppos != *staged_bytes) {
		ret = -ESPIPE;
		goto unlock;
	}
	/*
	 * Before the copy, not after: copy_from_user() can fault having already
	 * written part of the buffer, and that path returns without reaching
	 * anything below.  Lines left dirty with the slot recorded clean would
	 * write back over what the front end DMAs into it next.
	 */
	if (slot)
		slot->cpu_dirty = true;
	if (copy_from_user((u8 *)staged + *staged_bytes, buf, count)) {
		/* A real partial replacement is never considered a valid stage. */
		*staged_bytes = 0;
		ret = -EFAULT;
		goto unlock;
	}

	*staged_bytes += count;
	*ppos += count;
	if (generation && *staged_bytes == capacity)
		(*generation)++;

unlock:
	mutex_unlock(&becore->lock);
	return ret;
}

static ssize_t becore_recipe_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else
		ret = simple_read_from_buffer(buf, count, ppos, becore->recipe,
					      becore->recipe_staged_bytes);
	mutex_unlock(&becore->lock);

	return ret;
}

static ssize_t becore_recipe_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;

	return becore_stage_write(becore, buf, count, ppos, becore->recipe,
				  BECORE_RECIPE_BYTES,
				  &becore->recipe_staged_bytes,
				  &becore->recipe_generation, NULL);
}

static const struct file_operations becore_recipe_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_recipe_read,
	.write = becore_recipe_write,
	.llseek = default_llseek,
};

static ssize_t becore_gtnr_recipe_read(struct file *file, char __user *buf,
				       size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->gtnr_recipe,
					      becore->gtnr_recipe_staged_bytes);
	mutex_unlock(&becore->lock);

	return ret;
}

static ssize_t becore_gtnr_recipe_write(struct file *file,
					const char __user *buf, size_t count,
					loff_t *ppos)
{
	struct becore_device *becore = file->private_data;

	return becore_stage_write(becore, buf, count, ppos,
				  becore->gtnr_recipe, BECORE_GTNR_RECIPE_BYTES,
				  &becore->gtnr_recipe_staged_bytes,
				  &becore->gtnr_recipe_generation, NULL);
}

static const struct file_operations becore_gtnr_recipe_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_gtnr_recipe_read,
	.write = becore_gtnr_recipe_write,
	.llseek = default_llseek,
};

static ssize_t becore_mcsc_recipe_read(struct file *file, char __user *buf,
				       size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->mcsc_recipe,
					      becore->mcsc_recipe_staged_bytes);
	mutex_unlock(&becore->lock);

	return ret;
}

static ssize_t becore_mcsc_recipe_write(struct file *file,
					const char __user *buf, size_t count,
					loff_t *ppos)
{
	struct becore_device *becore = file->private_data;

	return becore_stage_write(becore, buf, count, ppos,
				  becore->mcsc_recipe, BECORE_MCSC_RECIPE_BYTES,
				  &becore->mcsc_recipe_staged_bytes,
				  &becore->mcsc_recipe_generation, NULL);
}

static const struct file_operations becore_mcsc_recipe_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_mcsc_recipe_read,
	.write = becore_mcsc_recipe_write,
	.llseek = default_llseek,
};

static ssize_t becore_input_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	struct becore_dma_buffer *input = &becore->inputs[0].buffer;

	return becore_stage_write(becore, buf, count, ppos,
				  input->cpu, input->size,
				  &input->staged_bytes, NULL,
				  &becore->inputs[0]);
}

static const struct file_operations becore_input_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = becore_input_write,
	.llseek = default_llseek,
};

static ssize_t becore_grid_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;

	return becore_stage_write(becore, buf, count, ppos,
				  becore->grid.cpu, becore->grid.size,
				  &becore->grid.staged_bytes,
				  &becore->grid_generation, NULL);
}

static const struct file_operations becore_grid_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = becore_grid_write,
	.llseek = default_llseek,
};

static ssize_t becore_output_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else if (!becore->completed_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->output.cpu,
					      becore->completed_output_size);
	mutex_unlock(&becore->lock);

	return ret;
}

static const struct file_operations becore_output_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_output_read,
	.llseek = default_llseek,
};

static ssize_t becore_mcsc_output_read(struct file *file, char __user *buf,
				       size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed || becore->output_quarantined)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else if (!becore->mcsc_completed_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->mcsc_output.cpu,
					      becore->mcsc_completed_output_size);
	mutex_unlock(&becore->lock);

	return ret;
}

static const struct file_operations becore_mcsc_output_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_mcsc_output_read,
	.llseek = default_llseek,
};

static ssize_t becore_encoded_read(struct file *file, char __user *buf,
				   size_t count, loff_t *ppos,
				   enum becore_block_id id)
{
	struct becore_device *becore = file->private_data;
	struct becore_cmdq_program *program = &becore->program[id];
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else if (!becore->completed_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos, program->cpu,
					      becore_cmdq_encoded_size(program));
	mutex_unlock(&becore->lock);

	return ret;
}

static ssize_t becore_rgbp_encoded_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	return becore_encoded_read(file, buf, count, ppos, BECORE_RGBP);
}

static const struct file_operations becore_rgbp_encoded_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_rgbp_encoded_read,
	.llseek = default_llseek,
};

static ssize_t becore_yuvp_encoded_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	return becore_encoded_read(file, buf, count, ppos, BECORE_YUVP);
}

static const struct file_operations becore_yuvp_encoded_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_yuvp_encoded_read,
	.llseek = default_llseek,
};

static ssize_t becore_gtnr_encoded_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else if (becore->gtnr_recipe_staged_bytes != BECORE_GTNR_RECIPE_BYTES ||
		 becore->gtnr_encoded_generation != becore->gtnr_recipe_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->gtnr_program.cpu,
					      becore_cmdq_encoded_size(&becore->gtnr_program));
	mutex_unlock(&becore->lock);

	return ret;
}

static const struct file_operations becore_gtnr_encoded_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_gtnr_encoded_read,
	.llseek = default_llseek,
};

static int becore_gtnr_encode_set(void *data, u64 value)
{
	struct becore_device *becore = data;
	int ret;

	if (value != 1)
		return -EINVAL;

	mutex_lock(&becore->lock);
	if (becore->reset_failed) {
		ret = -EIO;
	} else if (becore->running || becore->video_streaming) {
		ret = -EBUSY;
	} else {
		ret = becore_gtnr_recipe_validate(becore);
		if (!ret)
			ret = becore_encode_gtnr(becore);
		if (!ret)
			becore->gtnr_encoded_generation =
				becore->gtnr_recipe_generation;
	}
	mutex_unlock(&becore->lock);

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(becore_gtnr_encode_fops, NULL,
			 becore_gtnr_encode_set, "%llu\n");

static ssize_t becore_mcsc_encoded_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else if (becore->mcsc_recipe_staged_bytes != BECORE_MCSC_RECIPE_BYTES ||
		 becore->mcsc_encoded_generation != becore->mcsc_recipe_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->mcsc_program.cpu,
					      becore_cmdq_encoded_size(&becore->mcsc_program));
	mutex_unlock(&becore->lock);

	return ret;
}

static const struct file_operations becore_mcsc_encoded_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_mcsc_encoded_read,
	.llseek = default_llseek,
};

static int becore_mcsc_encode_set(void *data, u64 value)
{
	struct becore_device *becore = data;
	int ret;

	if (value != 1)
		return -EINVAL;

	mutex_lock(&becore->lock);
	if (becore->reset_failed) {
		ret = -EIO;
	} else if (becore->running || becore->video_streaming) {
		ret = -EBUSY;
	} else {
		becore->mcsc_dest_dma = becore->mcsc_output.dma;
		ret = becore_mcsc_recipe_validate(becore);
		if (!ret)
			ret = becore_encode_mcsc(becore,
						 BECORE_MCSC_INPUT_CAPTURED_VOTF);
		if (!ret)
			becore->mcsc_encoded_generation =
				becore->mcsc_recipe_generation;
		if (!ret)
			becore->mcsc_encoded_transport =
				BECORE_MCSC_INPUT_CAPTURED_VOTF;
	}
	mutex_unlock(&becore->lock);

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(becore_mcsc_encode_fops, NULL,
			 becore_mcsc_encode_set, "%llu\n");

static int becore_run_set(void *data, u64 value)
{
	struct becore_device *becore = data;

	if (value != 1)
		return -EINVAL;
	if (READ_ONCE(becore->video_streaming))
		return -EBUSY;

	return becore_run_frame(becore, READ_ONCE(becore->input_profile),
				READ_ONCE(becore->output_profile),
				false, NULL, false, false);
}

static int becore_run_get(void *data, u64 *value)
{
	struct becore_device *becore = data;

	mutex_lock(&becore->lock);
	*value = becore->running;
	mutex_unlock(&becore->lock);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(becore_run_fops, becore_run_get, becore_run_set,
			 "%llu\n");

static int becore_mcsc_run_set(void *data, u64 value)
{
	struct becore_device *becore = data;

	if (value != 1)
		return -EINVAL;
	if (READ_ONCE(becore->video_streaming))
		return -EBUSY;

	return becore_run_frame(becore, READ_ONCE(becore->input_profile),
				BECORE_YUVP_OUTPUT_SBWCL,
				false, NULL, false, true);
}
DEFINE_DEBUGFS_ATTRIBUTE(becore_mcsc_run_fops, NULL, becore_mcsc_run_set,
			 "%llu\n");

static int becore_cancel_set(void *data, u64 value)
{
	struct becore_device *becore = data;
	unsigned long flags;
	bool cancelled = false;

	if (value != 1)
		return -EINVAL;

	mutex_lock(&becore->lock);
	if (becore->video_streaming) {
		mutex_unlock(&becore->lock);
		return -EBUSY;
	}
	spin_lock_irqsave(&becore->run_lock, flags);
	if (becore->running) {
		becore->abort_run = true;
		cancelled = true;
	}
	spin_unlock_irqrestore(&becore->run_lock, flags);
	if (cancelled)
		complete(&becore->run_completion);
	mutex_unlock(&becore->lock);

	return cancelled ? 0 : -EALREADY;
}
DEFINE_DEBUGFS_ATTRIBUTE(becore_cancel_fops, NULL, becore_cancel_set, "%llu\n");

/* One CMDQ program: what the last encode filled, and what it was sized for. */
static void becore_status_program(struct seq_file *s, const char *name,
				  const struct becore_cmdq_program *program)
{
	seq_printf(s, "%-16s %zu encoded/%zu allocated bytes, %u/%u headers, iova %pad\n",
		   name, becore_cmdq_encoded_size(program), program->size,
		   program->header_count, program->capacity, &program->dma);
}

/*
 * Samsung's `votf_debug_state` is a six-value space, not the four its comment
 * lists: 5 and 7 are real and are the two that matter.  A producer sitting in
 * 5 means its packet went out and was never acknowledged -- Pablo treats that
 * as unrecoverable and forces a ramdump for it -- and 7 is the same thing for
 * a reset.
 */
static const char *becore_c2serv_conn_name(u32 state)
{
	static const char * const names[] = {
		[0] = "idle",
		[1] = "consumer waiting",
		[2] = "producer waiting",
		[3] = "connected",
		[5] = "waiting for token ack",
		[7] = "waiting for reset ack",
	};

	return state < ARRAY_SIZE(names) && names[state] ? names[state] :
							   "undocumented";
}

static int becore_status_show(struct seq_file *s, void *unused)
{
	static const char * const input_state_names[] = {
		[BECORE_INPUT_FREE] = "free",
		[BECORE_INPUT_PRODUCER] = "producer",
		[BECORE_INPUT_READY] = "ready",
		[BECORE_INPUT_BACKEND] = "backend",
		[BECORE_INPUT_QUARANTINED] = "quarantined",
	};
	struct becore_device *becore = s->private;
	struct exynos_becore_input_gains stream_gains;
	struct list_head *pos;
	unsigned long flags;
	unsigned int queued_outputs = 0;
	u32 cmdq_hold_mask;
	u32 frame_done_mask;
	u32 expected_mask;
	bool start_issued;
	bool irq_error;
	bool running;
	bool video_streaming;
	u32 which;
	u32 i;

	mutex_lock(&becore->lock);
	spin_lock_irqsave(&becore->run_lock, flags);
	running = becore->running;
	cmdq_hold_mask = becore->cmdq_hold_mask;
	frame_done_mask = becore->frame_done_mask;
	expected_mask = becore->expected_mask;
	start_issued = becore->start_issued;
	irq_error = becore->irq_error;
	spin_unlock_irqrestore(&becore->run_lock, flags);
	spin_lock_irqsave(&becore->queue_lock, flags);
	list_for_each(pos, &becore->queued_outputs)
		queued_outputs++;
	spin_unlock_irqrestore(&becore->queue_lock, flags);
	video_streaming = becore->video_streaming;
	seq_printf(s, "running          %u\n", running);
	seq_printf(s,
		   "video_queue      streaming %u, producer %u, queued %u, sequence %u\n",
		   video_streaming, becore->producer_streaming, queued_outputs,
		   becore->video_sequence);
	seq_printf(s, "runtime          %s\n",
		   pm_runtime_status_suspended(becore->dev) ? "suspended" : "active");
	seq_printf(s, "recipe           %zu/%u bytes, generation %u\n",
		   becore->recipe_staged_bytes, BECORE_RECIPE_BYTES,
		   becore->recipe_generation);
	seq_printf(s, "gtnr_recipe      %zu/%u bytes, generation %u, encoded %u\n",
		   becore->gtnr_recipe_staged_bytes, BECORE_GTNR_RECIPE_BYTES,
		   becore->gtnr_recipe_generation,
		   becore->gtnr_encoded_generation);
	seq_printf(s, "mcsc_recipe      %zu/%u bytes, generation %u, encoded %u\n",
		   becore->mcsc_recipe_staged_bytes, BECORE_MCSC_RECIPE_BYTES,
		   becore->mcsc_recipe_generation,
		   becore->mcsc_encoded_generation);
	seq_printf(s, "mcsc_transport   %s\n",
		   becore->mcsc_encoded_transport == BECORE_MCSC_INPUT_MEMORY ?
		   "memory" : "captured-votf");
	/*
	 * The raster with the length, because a run refuses on either and the
	 * length alone cannot say which: a frame staged before a geometry set
	 * is exactly as long as one staged after it whenever the two array
	 * widths share a 256-column bucket.
	 */
	seq_printf(s, "input            %zu/%zu bytes",
		   becore->inputs[0].buffer.staged_bytes,
		   becore->inputs[0].buffer.size);
	/* The raster describes a frame, so it says nothing without one. */
	if (becore->inputs[0].buffer.staged_bytes)
		seq_printf(s, " at %ux%u", becore->inputs[0].raster.width,
			   becore->inputs[0].raster.height);
	seq_printf(s, ", iova %pad\n", &becore->inputs[0].buffer.dma);
	seq_printf(s, "input_state      %s\n",
		   input_state_names[becore->inputs[0].state]);
	seq_puts(s, "input_slots      ");
	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_input_slot *slot = &becore->inputs[i];

		seq_printf(s, "%s%u:%s",
			   i ? " " : "", i, input_state_names[slot->state]);
		if (slot->state == BECORE_INPUT_PRODUCER)
			seq_printf(s, "#%llu", slot->producer_cookie);
		else if (slot->state == BECORE_INPUT_READY)
			seq_printf(s, "@%llu:%ux%u", slot->ready_sequence,
				   slot->raster.width, slot->raster.height);
	}
	seq_putc(s, '\n');
	/*
	 * The balance every slot holds, and not only the ones holding a frame:
	 * nothing clears a slot's gains when a run gives it back, so a free
	 * slot still says what the last frame through it was encoded against.
	 * That is the whole of what makes the per-frame path checkable from a
	 * shell -- run a capture with a moving AWB and these follow it.
	 */
	stream_gains = becore_stream_gains(becore);
	seq_printf(s, "input_balance    stream %u/%u/%u/%u",
		   stream_gains.red, stream_gains.green_red,
		   stream_gains.green_blue, stream_gains.blue);
	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		const struct exynos_becore_input_gains *gains =
			&becore->inputs[i].gains;

		seq_puts(s, i ? " " : ", slots ");
		/*
		 * A slot that has never been handed out carries the zeros it
		 * was allocated with, and a gain of zero means a channel
		 * switched off -- so say "no frame yet" rather than print four
		 * of them into a field labelled Q12 gains.
		 */
		if (!gains->red && !gains->green_red && !gains->green_blue &&
		    !gains->blue)
			seq_puts(s, "--");
		else
			seq_printf(s, "%u/%u/%u/%u", gains->red,
				   gains->green_red, gains->green_blue,
				   gains->blue);
	}
	seq_puts(s, " Q12\n");
	if (becore->input_producer) {
		seq_printf(s, "input_producer   %s iovas",
			   dev_name(becore->input_producer->producer));
		for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++)
			seq_printf(s, " %u:%pad", i,
				   &becore->input_producer->dmas[i]);
		seq_putc(s, '\n');
	}
	seq_printf(s, "grid             %zu/%zu bytes, generation %u, iova %pad\n",
		   becore->grid.staged_bytes, becore->grid.size,
		   becore->grid_generation,
		   &becore->grid.dma);
	seq_printf(s, "output           %zu active/%zu completed/%zu allocated bytes, iova %pad\n",
		   becore->active_output_size, becore->completed_output_size,
		   becore->output.size,
		   &becore->output.dma);
	seq_printf(s, "capture_size     %zu bytes\n",
		   becore->active_capture_size);
	seq_printf(s, "overrides        %u\n", becore->override_count);
	seq_printf(s,
		   "params           ccm %u, ltm curve %u, ltm tone adjust %u, colour LUT %u, gamma %u, sharpener %u, noise reducer %u\n",
		   becore->params.ccm_valid, becore->params.ltm_curve_valid,
		   becore->params.ltm_tone_adjust_valid,
		   becore->params.clut_valid, becore->params.gamma_valid,
		   becore->params.sharpen_valid, becore->params.yuvnr_valid);
	seq_printf(s, "input_profile    %u requested, %u active, %zu bytes\n",
		   READ_ONCE(becore->input_profile),
		   becore->active_input_profile,
		   becore_rgbp_input_size(becore_rgbp_input_profile(becore),
					  &becore->array));
	seq_printf(s, "output_profile   %u requested, %u active\n",
		   READ_ONCE(becore->output_profile),
		   becore->active_output_profile);
	seq_printf(s, "active_path      %s\n",
		   !becore->active_mcsc ? "YUVP" :
		   becore->active_votf ? "YUVP-votf-to-MCSC" :
					 "YUVP-memory-to-MCSC");
	seq_printf(s, "votf             %u requested, %u active\n",
		   READ_ONCE(becore->votf), becore->active_votf);
	becore_status_program(s, "rgbp_cmdq", &becore->program[BECORE_RGBP]);
	becore_status_program(s, "yuvp_cmdq", &becore->program[BECORE_YUVP]);
	seq_printf(s, "gtnr_output      %zu bytes, iova %pad\n",
		   becore->gtnr_output.size, &becore->gtnr_output.dma);
	becore_status_program(s, "gtnr_cmdq", &becore->gtnr_program);
	seq_printf(s, "mcsc_output      %u completed/%zu allocated bytes, iova %pad\n",
		   becore->mcsc_completed_output_size, becore->mcsc_output.size,
		   &becore->mcsc_output.dma);
	becore_status_program(s, "mcsc_cmdq", &becore->mcsc_program);
	for (i = 0; i < BECORE_NUM_C2SERV; i++) {
		const struct becore_c2serv_state *c2serv =
			&becore->c2serv_state[i];

		seq_printf(s,
			   "votf_%-11s %s, ring_clk %#x, ring %#x, local_ip %#010x, named %#06x, reset_enables %#010x\n",
			   becore_c2serv[i].name,
			   c2serv->ready ? "ready" : "down",
			   c2serv->ring_clk_en, c2serv->ring_enable,
			   c2serv->local_ip, becore_c2serv[i].local_ip,
			   c2serv->reset_enables);

	}

	for (which = 0; which < 2; which++) {
		const struct becore_c2serv_link_state *link = which ?
			&becore->c2serv_done : &becore->c2serv_armed;

		if (!link->sampled)
			continue;

		seq_printf(s, "votf_link        %s\n",
			   which ? "when the run finished" : "as armed");
		for (i = 0; i < BECORE_C2SERV_LINK_PLANES; i++)
			seq_printf(s,
				   "  tws%u            %s, dout %#x, rcv %#x, enable %u, limit %u, dest %#07x, token %u, busy %u, fullness %u\n",
				   i, becore_c2serv_conn_name(link->tws[i].conn),
				   link->tws[i].conn_raw, link->tws[i].rcv_valid,
				   link->tws[i].enable, link->tws[i].limit,
				   link->tws[i].dest, link->tws[i].lines_in_token,
				   link->tws[i].busy, link->tws[i].fullness);
		for (i = 0; i < BECORE_C2SERV_LINK_PLANES; i++)
			seq_printf(s,
				   "  trs%u            %s, dout %#x, rcv %#x, enable %u, limit %u, first %u, token %u, lines %u, busy %u, lost %u\n",
				   i, becore_c2serv_conn_name(link->trs[i].conn),
				   link->trs[i].conn_raw, link->trs[i].rcv_valid,
				   link->trs[i].enable, link->trs[i].limit,
				   link->trs[i].lines_in_first_token,
				   link->trs[i].lines_in_token,
				   link->trs[i].lines_count,
				   link->trs[i].busy, link->trs[i].lost_connection);
	}
	seq_printf(s, "intcam           %lu Hz (saved %lu, raised %u)\n",
		   clk_get_rate(becore->intcam_clk),
		   becore->saved_intcam_rate, becore->intcam_rate_active);
	seq_printf(s, "timing_runs      %u\n", becore->timing_runs);
	for (i = 0; i < BECORE_TIMING_PHASE_COUNT; i++)
		seq_printf(s, "timing_%-9s %llu us last, %llu us total\n",
			   becore_timing_names[i],
			   becore->timing[i].last_ns / NSEC_PER_USEC,
			   becore->timing[i].total_ns / NSEC_PER_USEC);
	seq_printf(s, "run_generation   %u\n", becore->run_generation);
	seq_printf(s, "completed         %u\n", becore->completed_generation);
	seq_printf(s, "last_result       %d\n", becore->last_run_result);
	seq_printf(s, "cmdq_hold         %#x\n", cmdq_hold_mask);
	seq_printf(s, "frame_done        %#x\n", frame_done_mask);
	seq_printf(s, "expected          %#x\n", expected_mask);
	seq_printf(s, "start_issued      %u\n", start_issued);
	seq_printf(s, "irq_error         %u\n", irq_error);
	seq_printf(s, "reset_failed      %u\n", becore->reset_failed);
	seq_printf(s, "output_quarantined %u\n", becore->output_quarantined);
	seq_printf(s, "output_changed    %u\n", becore->output_changed_bytes);
	seq_printf(s, "output_first      %#x\n", becore->output_first_changed);
	seq_printf(s, "mcsc_completed    %u\n",
		   becore->mcsc_completed_generation);
	seq_printf(s, "mcsc_changed      %u\n",
		   becore->mcsc_output_changed_bytes);
	seq_printf(s, "mcsc_first        %#x\n",
		   becore->mcsc_output_first_changed);
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		seq_printf(s,
			   "%-4s irq0 %lld last %#010x cmdq %#010x; irq1 %lld last %#010x\n",
			   becore->blocks[i].name,
			   atomic64_read(&becore->blocks[i].int0_count),
			   READ_ONCE(becore->blocks[i].last_int0),
			   READ_ONCE(becore->blocks[i].last_cmdq_int),
			   atomic64_read(&becore->blocks[i].int1_count),
			   READ_ONCE(becore->blocks[i].last_int1));
	mutex_unlock(&becore->lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(becore_status);

static int becore_override_show(struct seq_file *s, void *unused)
{
	struct becore_device *becore = s->private;
	u32 i;

	mutex_lock(&becore->lock);
	seq_printf(s, "# %u of %u overrides\n", becore->override_count,
		   BECORE_OVERRIDE_MAX);
	for (i = 0; i < becore->override_count; i++)
		seq_printf(s, "%#010x %#010x\n", becore->overrides[i].reg,
			   becore->overrides[i].value);
	mutex_unlock(&becore->lock);

	return 0;
}

static int becore_override_open(struct inode *inode, struct file *file)
{
	return single_open(file, becore_override_show, inode->i_private);
}

/*
 * The list is whatever has been written to one descriptor so far, re-parsed in
 * full every time and installed only if all of it is good.  A sweep therefore
 * never runs against a partly applied set, and neither does a shell:
 * `printf '%s\n' '...'` reaches this node as *two* writes on busybox, the line
 * and then its newline, and treating the second as a fresh list would leave
 * the first one installed under a syntax error or drop it entirely.
 *
 * "One descriptor" is enforced rather than assumed: the text belongs to
 * whichever file wrote it at offset zero, and a second writer is refused until
 * that one is done.  Two shells redirecting into this node at once would
 * otherwise splice one list out of both halves, and it would parse.
 *
 * Each line is a register and the value to encode for it, both as ordinary
 * numbers; a line that is blank or starts with '#' is skipped, so the read
 * back can be piped straight back in.  Writing nothing but whitespace clears
 * the list.
 *
 * A write that leaves the accumulated text unparseable installs nothing and
 * does not advance the descriptor, so a writer that splits mid-token -- which
 * no shell does, but `dd bs=8` would -- has to start again at offset zero.
 */
static int becore_override_parse(struct becore_device *becore)
{
	struct becore_override parsed[BECORE_OVERRIDE_MAX] = {};
	u32 parsed_count = 0;
	char *text;
	char *cursor;
	char *line;
	int ret = 0;

	text = kmemdup_nul(becore->override_text, becore->override_text_len,
			   GFP_KERNEL);
	if (!text)
		return -ENOMEM;

	cursor = text;
	while ((line = strsep(&cursor, "\n"))) {
		char *value_text;
		u32 reg;
		u32 value;
		u32 i;

		line = strim(line);
		if (!*line || *line == '#')
			continue;
		value_text = line;
		strsep(&value_text, " \t");
		if (!value_text || kstrtou32(line, 0, &reg) ||
		    kstrtou32(strim(value_text), 0, &value)) {
			ret = -EINVAL;
			goto out;
		}
		if (parsed_count == BECORE_OVERRIDE_MAX) {
			ret = -E2BIG;
			goto out;
		}
		/* Two values for one register would make the list ordered. */
		for (i = 0; i < parsed_count; i++) {
			if (parsed[i].reg == reg) {
				ret = -EEXIST;
				goto out;
			}
		}
		ret = becore_override_check(reg);
		if (ret)
			goto out;
		parsed[parsed_count].reg = reg;
		parsed[parsed_count].value = value;
		parsed_count++;
	}

	memcpy(becore->overrides, parsed, sizeof(parsed));
	becore->override_count = parsed_count;

out:
	kfree(text);
	return ret;
}

static ssize_t becore_override_write(struct file *file, const char __user *buf,
				     size_t count, loff_t *ppos)
{
	struct becore_device *becore =
		((struct seq_file *)file->private_data)->private;
	ssize_t ret;

	size_t base;

	if (*ppos < 0)
		return -EINVAL;
	if (!count)
		return 0;

	mutex_lock(&becore->lock);
	if (becore->video_streaming || becore->running) {
		ret = -EBUSY;
		goto unlock;
	}
	if (*ppos == 0) {
		base = 0;
	} else if (*ppos != becore->override_text_len ||
		   becore->override_writer != file) {
		ret = -ESPIPE;
		goto unlock;
	} else {
		base = becore->override_text_len;
	}
	/* Nothing is discarded until the write is known to fit. */
	if (count > BECORE_OVERRIDE_TEXT_MAX - base) {
		ret = -EFBIG;
		goto unlock;
	}
	if (copy_from_user(becore->override_text + base, buf, count)) {
		becore->override_text_len = base;
		ret = -EFAULT;
		goto reset;
	}
	becore->override_text_len = base + count;
	becore->override_writer = file;
	ret = becore_override_parse(becore);
	if (ret)
		goto reset;
	*ppos += count;
	ret = count;
	goto unlock;

reset:
	/* A half-written list is not a list; leave nothing behind to inherit. */
	becore->override_text_len = 0;
	becore->override_writer = NULL;
	becore->override_count = 0;
unlock:
	mutex_unlock(&becore->lock);

	return ret;
}

static int becore_override_release(struct inode *inode, struct file *file)
{
	struct becore_device *becore =
		((struct seq_file *)file->private_data)->private;

	mutex_lock(&becore->lock);
	if (becore->override_writer == file) {
		becore->override_writer = NULL;
		becore->override_text_len = 0;
	}
	mutex_unlock(&becore->lock);

	return single_release(inode, file);
}

static const struct file_operations becore_override_fops = {
	.owner = THIS_MODULE,
	.open = becore_override_open,
	.read = seq_read,
	.write = becore_override_write,
	.llseek = seq_lseek,
	.release = becore_override_release,
};

static int becore_geometry_show(struct seq_file *s, void *unused)
{
	struct becore_device *becore = s->private;

	mutex_lock(&becore->lock);
	seq_printf(s, "array  %u %u\n", becore->array.width,
		   becore->array.height);
	seq_printf(s, "chain  %u %u\n", becore->chain.width,
		   becore->chain.height);
	seq_printf(s, "scaled %u %u\n", becore->scaled.width,
		   becore->scaled.height);
	mutex_unlock(&becore->lock);

	return 0;
}

static int becore_geometry_open(struct inode *inode, struct file *file)
{
	return single_open(file, becore_geometry_show, inode->i_private);
}

static ssize_t becore_geometry_write(struct file *file, const char __user *buf,
				     size_t count, loff_t *ppos)
{
	struct becore_device *becore =
		((struct seq_file *)file->private_data)->private;
	struct becore_raster array;
	struct becore_raster chain;
	struct becore_raster scaled;
	char text[64];
	char *line;
	char tail;
	int ret;

	if (count >= sizeof(text))
		return -EFBIG;
	if (copy_from_user(text, buf, count))
		return -EFAULT;
	text[count] = '\0';
	line = strim(text);
	/*
	 * busybox splits `printf '%s\n'` into the line and then its newline,
	 * so a write with nothing in it is the tail of one already accepted.
	 */
	if (!*line)
		return count;
	/* The %c matches only if something follows the six, and refuses it. */
	if (sscanf(line, "%u %u %u %u %u %u %c", &array.width, &array.height,
		   &chain.width, &chain.height, &scaled.width, &scaled.height,
		   &tail) != 6)
		return -EINVAL;

	mutex_lock(&becore->video_lock);
	mutex_lock(&becore->lock);
	if (becore->reset_failed || becore->output_quarantined)
		ret = -EIO;
	else if (becore->running || becore->video_streaming ||
		 vb2_is_busy(&becore->queue))
		ret = -EBUSY;
	else
		ret = becore_geometry_apply(becore, &array, &chain, &scaled);
	mutex_unlock(&becore->lock);
	mutex_unlock(&becore->video_lock);

	return ret ? ret : count;
}

static const struct file_operations becore_geometry_fops = {
	.owner = THIS_MODULE,
	.open = becore_geometry_open,
	.read = seq_read,
	.write = becore_geometry_write,
	.llseek = seq_lseek,
	.release = single_release,
};

/*
 * The mosaic's phase: which quad of the Bayer pattern is red.  Two registers
 * take it, BYR_DNS's and BYR_DMSC's, and it decides what the demosaic reads as
 * colour -- so a wrong one is not a subtle error.
 *
 * It belongs to the producer whenever a stream starts:
 * becore_latch_input_format() takes it off the sink pad at STREAMON and puts
 * it back if the stream does not start.  Where it differs from the array
 * raster, which is latched by the same function, is what it does with no
 * producer on the pad -- the raster stays as it is, because it has to describe
 * the input slots whether anything is attached or not, and the mosaic returns
 * to the compiled-in default, because a mosaic without a frame describes
 * nothing.  So a phase written here holds until something latches, and a
 * stream that ran leaves the producer's behind rather than this one.
 *
 * What this file is for is the offline loop, which has no producer to ask and
 * until now had only that default -- the ultrawide's.  The captured corpus is
 * three cameras reading out on three different phases, so replaying another
 * camera's frame reproduced its whole program except this one word.
 *
 * The phase is written and the media-bus code is printed beside it, because
 * the code is what the pad reports and the phase is what the register takes,
 * and a file that showed only one of them would leave the other to be worked
 * out by hand at exactly the moment the two are being compared.
 *
 * What this cannot check is whether the phase is the frame's.  A raster can be
 * held against the staged frame's length; nothing in a Bayer frame says which
 * quad it started on, so the only statement of that is the provenance of the
 * file that was staged.  Whoever stages a frame states its phase, and the
 * driver takes their word for it.
 */
static int becore_bayer_phase_show(struct seq_file *s, void *unused)
{
	struct becore_device *becore = s->private;

	mutex_lock(&becore->lock);
	seq_printf(s, "%d 0x%04x\n", becore_bayer_phase(becore->input_code),
		   becore->input_code);
	mutex_unlock(&becore->lock);

	return 0;
}

static int becore_bayer_phase_open(struct inode *inode, struct file *file)
{
	return single_open(file, becore_bayer_phase_show, inode->i_private);
}

static ssize_t becore_bayer_phase_write(struct file *file,
					const char __user *buf, size_t count,
					loff_t *ppos)
{
	struct becore_device *becore =
		((struct seq_file *)file->private_data)->private;
	char text[32];
	char *line;
	u32 phase;
	u32 code;
	int ret = 0;

	if (count >= sizeof(text))
		return -EFBIG;
	if (copy_from_user(text, buf, count))
		return -EFAULT;
	text[count] = '\0';
	line = strim(text);
	/* As the geometry file: busybox writes the trailing newline on its own. */
	if (!*line)
		return count;
	/*
	 * kstrtou32() rather than the sscanf() the geometry file parses with,
	 * and the difference matters here: sscanf("%u") drops the overflow and
	 * hands back the wrapped value, so 4294967296 would arrive as a valid
	 * phase 0 rather than as a refusal.  A wrapped raster is refused by the
	 * bounds it then has to pass; a wrapped phase is one of the four.  Its
	 * error code goes back as it stands, so a value too large to be a phase
	 * is -ERANGE and a value that is not a number is -EINVAL.
	 */
	ret = kstrtou32(line, 10, &phase);
	if (ret)
		return ret;
	code = becore_bayer_code(phase);
	if (!code) {
		dev_err(becore->dev,
			"%u is not one of the four Bayer phases\n", phase);
		return -EINVAL;
	}

	mutex_lock(&becore->lock);
	if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else
		becore->input_code = code;
	mutex_unlock(&becore->lock);

	return ret ? ret : count;
}

static const struct file_operations becore_bayer_phase_fops = {
	.owner = THIS_MODULE,
	.open = becore_bayer_phase_open,
	.read = seq_read,
	.write = becore_bayer_phase_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int becore_stream_crc_show(struct seq_file *s, void *unused)
{
	struct becore_device *becore = s->private;
	unsigned int id;

	mutex_lock(&becore->lock);
	seq_printf(s, "seed             0x%02x requested, 0x%02x armed\n",
		   READ_ONCE(becore->stream_crc_seed) &
		   BECORE_STREAM_CRC_SEED_MASK,
		   becore->stream_crc_armed_seed);
	seq_printf(s, "generation       %u of %u\n",
		   becore->stream_crc_generation, becore->run_generation);
	for (id = 0; id < BECORE_NUM_BLOCKS; id++) {
		struct becore_block *block = &becore->blocks[id];
		const struct becore_stream_crc *table;
		size_t count;
		size_t i;

		table = becore_stream_crc_table(id, &count);
		for (i = 0; i < count; i++)
			seq_printf(s,
				   "%-4s %-30s +%#06x armed %#010x result %#010x crc 0x%02x\n",
				   block->name, table[i].name, table[i].offset,
				   block->stream_crc_armed[i],
				   block->stream_crc_result[i],
				   (block->stream_crc_result[i] &
				    BECORE_STREAM_CRC_RESULT_MASK) >>
				   BECORE_STREAM_CRC_RESULT_SHIFT);
	}
	mutex_unlock(&becore->lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(becore_stream_crc);

static void becore_debugfs_remove(void *data)
{
	struct becore_device *becore = data;

	debugfs_remove_recursive(becore->debugfs);
}

int becore_debugfs_init(struct becore_device *becore)
{
	struct dentry *dir;

	dir = debugfs_create_dir(dev_name(becore->dev), NULL);
	if (IS_ERR(dir))
		return PTR_ERR(dir);
	becore->debugfs = dir;
	debugfs_create_file("program", 0600, dir, becore, &becore_recipe_fops);
	debugfs_create_file("gtnr_program", 0600, dir, becore,
			    &becore_gtnr_recipe_fops);
	debugfs_create_file("mcsc_program", 0600, dir, becore,
			    &becore_mcsc_recipe_fops);
	debugfs_create_file("input", 0200, dir, becore, &becore_input_fops);
	debugfs_create_file("grid", 0200, dir, becore, &becore_grid_fops);
	debugfs_create_u32("stream_crc_seed", 0644, dir,
			   &becore->stream_crc_seed);
	debugfs_create_file("stream_crc", 0400, dir, becore,
			    &becore_stream_crc_fops);
	debugfs_create_file("override", 0600, dir, becore,
			    &becore_override_fops);
	debugfs_create_file("geometry", 0600, dir, becore,
			    &becore_geometry_fops);
	debugfs_create_file("bayer_phase", 0600, dir, becore,
			    &becore_bayer_phase_fops);
	debugfs_create_u32("input_profile", 0644, dir,
			   &becore->input_profile);
	debugfs_create_u32("votf", 0644, dir, &becore->votf);
	debugfs_create_u32("votf_tws_limit", 0644, dir,
			   &becore->votf_tws_limit);
	debugfs_create_u32("votf_trs_limit", 0644, dir,
			   &becore->votf_trs_limit);
	debugfs_create_u32("votf_tws_token", 0644, dir,
			   &becore->votf_tws_token[0]);
	debugfs_create_u32("votf_tws_token_uv", 0644, dir,
			   &becore->votf_tws_token[1]);
	debugfs_create_u32("votf_trs_token", 0644, dir,
			   &becore->votf_trs_token[0]);
	debugfs_create_u32("votf_trs_token_uv", 0644, dir,
			   &becore->votf_trs_token[1]);
	debugfs_create_u32("output_profile", 0644, dir,
			   &becore->output_profile);
	debugfs_create_file("output", 0400, dir, becore, &becore_output_fops);
	debugfs_create_file("mcsc_output", 0400, dir, becore,
			    &becore_mcsc_output_fops);
	debugfs_create_file("rgbp_cmdq", 0400, dir, becore,
			    &becore_rgbp_encoded_fops);
	debugfs_create_file("yuvp_cmdq", 0400, dir, becore,
			    &becore_yuvp_encoded_fops);
	debugfs_create_file("gtnr_cmdq", 0400, dir, becore,
			    &becore_gtnr_encoded_fops);
	debugfs_create_file("gtnr_encode", 0200, dir, becore,
			    &becore_gtnr_encode_fops);
	debugfs_create_file("mcsc_cmdq", 0400, dir, becore,
			    &becore_mcsc_encoded_fops);
	debugfs_create_file("mcsc_encode", 0200, dir, becore,
			    &becore_mcsc_encode_fops);
	debugfs_create_file("run", 0600, dir, becore, &becore_run_fops);
	debugfs_create_file("mcsc_run", 0200, dir, becore,
			    &becore_mcsc_run_fops);
	debugfs_create_file("cancel", 0200, dir, becore, &becore_cancel_fops);
	debugfs_create_file("status", 0400, dir, becore, &becore_status_fops);

	return devm_add_action_or_reset(becore->dev, becore_debugfs_remove,
					becore);
}
