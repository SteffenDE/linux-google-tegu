// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Zumapro camera back-end core bring-up driver
 *
 * The register sequences here are the common processor lifecycle observed on
 * the Pixel 9a vendor stack.  The debugfs diagnostic accepts a structurally
 * fixed, relocatable RGBP/YUVP program for offline bring-up; it is deliberately
 * not a camera ABI.  Powering a block down is safe only after all three
 * processors have accepted a software reset.
 */

#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>

#include <media/exynos-becore.h>

#include "exynos-becore-recipe.h"

#define BECORE_GLOBAL_ENABLE		0x0000
#define BECORE_GLOBAL_ENABLE_CLEAR	0x0008
#define BECORE_SW_RESET			0x0010
#define BECORE_SET_CTRL			0x0030
#define BECORE_COMMAND_Q_STOP_ON_FRAME	0x003c
#define BECORE_FRO_GLOBAL_ENABLE		0x006c

#define BECORE_CMDQ_QUE_CMD_L		0x0400
#define BECORE_CMDQ_QUE_CMD_M		0x0404
#define BECORE_CMDQ_QUE_CMD_H		0x0408
#define BECORE_CMDQ_QUE_CMD_START	0x040c
#define BECORE_CMDQ_ADD_TO_QUEUE_0	0x0484
#define BECORE_CMDQ_INT_STATUS		0x04f8
#define BECORE_CMDQ_INT_CLEAR		0x04fc
#define BECORE_CMDQ_INT_ENABLE		0x04f4
#define BECORE_CMDQ_ENABLE		0x0500

#define BECORE_INT0_ENABLE		0x0804
#define BECORE_INT0_STATUS		0x0808
#define BECORE_INT0_CLEAR		0x080c
#define BECORE_INT1_ENABLE		0x0814
#define BECORE_INT1_STATUS		0x0818
#define BECORE_INT1_CLEAR		0x081c

#define BECORE_C_LOADER_ENABLE		0x1000
#define BECORE_C_LOADER_MODE		0x1004
#define BECORE_STAT_RDMACL_EN		0x1600
#define BECORE_RGBP_C_LOADER_ENABLE	0x4000

#define BECORE_RESET_TIMEOUT_US		1000

#define BECORE_INT_FRAME_END		BIT(1)
#define BECORE_INT_CMDQ_HOLD		BIT(2)
#define BECORE_INT_EXPECTED		(BECORE_INT_FRAME_END | BECORE_INT_CMDQ_HOLD)
#define BECORE_ACTIVE_BLOCKS		(BIT(BECORE_RGBP) | BIT(BECORE_YUVP))

#define BECORE_CMDQ_HEADER_BYTES		16
#define BECORE_CMDQ_PAYLOAD_BYTES	64
#define BECORE_CMDQ_MODE		0x9000

#define BECORE_INPUT_SIZE		0x1a17000
#define BECORE_INPUT_IMAGE_OFFSET	0x30c00
#define BECORE_GRID_SIZE			0x18000
#define BECORE_OUTPUT_SIZE		0x949000
#define BECORE_OUTPUT_PLANE2_OFFSET	0x630640
#define BECORE_RUN_TIMEOUT_MS		1000

#define BECORE_RGBP_PHYS_BASE		0x1c440000
#define BECORE_YUVP_PHYS_BASE		0x1c840000

#define BECORE_RGBP_INPUT_IMAGE_REG	(BECORE_RGBP_PHYS_BASE + 0x1c50)
#define BECORE_RGBP_INPUT_HEADER_REG	(BECORE_RGBP_PHYS_BASE + 0x1d10)
#define BECORE_YUVP_GRID_REG		(BECORE_YUVP_PHYS_BASE + 0x1c50)
#define BECORE_YUVP_OUTPUT_PLANE1_REG	(BECORE_YUVP_PHYS_BASE + 0x2450)
#define BECORE_YUVP_OUTPUT_PLANE2_REG	(BECORE_YUVP_PHYS_BASE + 0x2490)

enum becore_block_id {
	BECORE_RGBP,
	BECORE_MCFP,
	BECORE_YUVP,
	BECORE_NUM_BLOCKS,
};

struct becore_regval {
	u32 offset;
	u32 value;
};

struct becore_device;

struct becore_dma_buffer {
	void *cpu;
	dma_addr_t dma;
	size_t size;
	size_t staged_bytes;
	struct sg_table *sgt;
};

struct becore_cmdq_program {
	void *cpu;
	dma_addr_t dma;
	size_t size;
	u32 header_count;
};

struct becore_block {
	struct becore_device *becore;
	const char *name;
	void __iomem *base;
	u32 int0_mask_prepare;
	u32 int0_mask;
	u32 int1_mask;
	u32 cmdq_int_mask;
	u32 last_int0;
	u32 last_int1;
	u32 last_cmdq_int;
	atomic64_t int0_count;
	atomic64_t int1_count;
};

struct becore_irq {
	struct becore_block *block;
	int irq;
	bool int1;
};

struct becore_device {
	struct device *dev;
	struct becore_block blocks[BECORE_NUM_BLOCKS];
	struct becore_irq irqs[BECORE_NUM_BLOCKS * 2];
	void __iomem *ssmt[7];
	void __iomem *sysreg_rgbp;
	struct dev_pm_domain_list *pm_domains;
	struct dentry *debugfs;
	/* Serializes staging, one-shot execution, and output inspection. */
	struct mutex lock;
	/* Protects IRQ-driven command-hold/frame completion state. */
	spinlock_t run_lock;
	struct completion run_completion;
	struct becore_dma_buffer input;
	struct becore_dma_buffer grid;
	struct becore_dma_buffer output;
	struct exynos_becore_input *input_producer;
	struct becore_cmdq_program program[BECORE_NUM_BLOCKS];
	u8 *recipe;
	size_t recipe_staged_bytes;
	u32 recipe_generation;
	u32 run_generation;
	u32 completed_generation;
	u32 cmdq_hold_mask;
	u32 frame_done_mask;
	u32 output_changed_bytes;
	u32 output_first_changed;
	int last_run_result;
	bool running;
	bool start_issued;
	bool abort_run;
	bool irq_error;
	bool irqs_enabled;
	bool reset_failed;
};

struct exynos_becore_input {
	struct becore_device *becore;
	struct device *producer;
	struct sg_table sgt;
	dma_addr_t dma;
	bool producing;
};

static const char * const becore_pm_domain_names[] = {
	"yuvp",
	"rgbp",
	"gdc",
};

static const char * const becore_irq_names[] = {
	"rgbp-int0", "rgbp-int1",
	"mcfp-int0", "mcfp-int1",
	"yuvp-int0", "yuvp-int1",
};

static const char * const becore_ssmt_names[] = {
	"ssmt-rgbp",
	"ssmt-yuvp",
	"ssmt-mcfp0",
	"ssmt-mcfp1",
	"ssmt-mcfp2",
	"ssmt-mcfp3",
	"ssmt-mcfp4",
};

static const struct becore_regval becore_rgbp_init[] = {
	{ BECORE_SET_CTRL, 0x1 },
	{ BECORE_GLOBAL_ENABLE_CLEAR, 0x1 },
	{ BECORE_FRO_GLOBAL_ENABLE, 0x3fffff },
	{ BECORE_COMMAND_Q_STOP_ON_FRAME, 0x40 },
	{ BECORE_CMDQ_ENABLE, 0x1 },
	{ BECORE_STAT_RDMACL_EN, 0x1 },
	{ BECORE_GLOBAL_ENABLE, 0x1 },
	{ BECORE_RGBP_C_LOADER_ENABLE, 0x1 },
	{ BECORE_SET_CTRL, 0x0 },
};

static const struct becore_regval becore_mcfp_init[] = {
	{ BECORE_SET_CTRL, 0x1 },
	{ BECORE_GLOBAL_ENABLE_CLEAR, 0x1 },
	{ BECORE_FRO_GLOBAL_ENABLE, 0xffff },
	{ BECORE_COMMAND_Q_STOP_ON_FRAME, 0x40 },
	{ BECORE_CMDQ_ENABLE, 0x1 },
	{ BECORE_STAT_RDMACL_EN, 0x1 },
	{ BECORE_C_LOADER_ENABLE, 0x1 },
	{ BECORE_C_LOADER_MODE, 0x3 },
	{ BECORE_GLOBAL_ENABLE, 0x1 },
	{ BECORE_SET_CTRL, 0x0 },
};

static const struct becore_regval becore_yuvp_init[] = {
	{ BECORE_SET_CTRL, 0x1 },
	{ BECORE_GLOBAL_ENABLE_CLEAR, 0x1 },
	{ BECORE_FRO_GLOBAL_ENABLE, 0xedff },
	{ BECORE_COMMAND_Q_STOP_ON_FRAME, 0x40 },
	{ BECORE_CMDQ_ENABLE, 0x1 },
	{ BECORE_C_LOADER_ENABLE, 0x1 },
	{ BECORE_C_LOADER_MODE, 0x1b },
	{ BECORE_STAT_RDMACL_EN, 0x1 },
	{ BECORE_GLOBAL_ENABLE, 0x1 },
	{ BECORE_SET_CTRL, 0x0 },
};

static void becore_write_table(struct becore_block *block,
			       const struct becore_regval *table,
			       size_t count)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		writel_relaxed(table[i].value, block->base + table[i].offset);
}

static void becore_issue_reset(struct becore_block *block)
{
	if (block != &block->becore->blocks[BECORE_RGBP])
		writel_relaxed(0, block->base + BECORE_C_LOADER_ENABLE);

	writel_relaxed(1, block->base + BECORE_SW_RESET);
	writel_relaxed(0, block->base + BECORE_SET_CTRL);
}

static int becore_wait_reset(struct becore_block *block)
{
	u32 value;
	int ret;

	ret = readl_poll_timeout(block->base + BECORE_SW_RESET, value, !value,
				  1, BECORE_RESET_TIMEOUT_US);
	if (ret)
		dev_err(block->becore->dev, "%s reset timed out (0x%08x)\n",
			block->name, value);

	return ret;
}

static int becore_reset_all(struct becore_device *becore)
{
	int first_error = 0;
	unsigned int i;
	int ret;

	/* The OTF-connected chain has to receive reset as one hardware phase. */
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		becore_issue_reset(&becore->blocks[i]);

	/* Wait for every block even if an earlier one failed. */
	for (i = 0; i < BECORE_NUM_BLOCKS; i++) {
		ret = becore_wait_reset(&becore->blocks[i]);
		if (ret && !first_error)
			first_error = ret;
	}

	return first_error;
}

static size_t becore_cmdq_program_size(u32 header_count)
{
	return ALIGN((size_t)header_count * BECORE_CMDQ_HEADER_BYTES,
		     BECORE_CMDQ_PAYLOAD_BYTES) +
	       (size_t)header_count * BECORE_CMDQ_PAYLOAD_BYTES;
}

static const u8 *becore_recipe_records(const struct becore_device *becore,
				       enum becore_block_id id)
{
	const u8 *records = becore->recipe + BECORE_RECIPE_HEADER_BYTES;

	if (id == BECORE_YUVP)
		records += BECORE_RGBP_HEADER_COUNT * BECORE_RECIPE_RECORD_BYTES;

	return records;
}

static int becore_recipe_header_validate(const struct becore_device *becore)
{
	const u8 *header = becore->recipe;

	if (becore->recipe_staged_bytes != BECORE_RECIPE_BYTES)
		return -EINVAL;
	if (get_unaligned_le32(header) != BECORE_RECIPE_MAGIC ||
	    get_unaligned_le32(header + 4) != BECORE_RECIPE_VERSION ||
	    get_unaligned_le32(header + 8) != BECORE_RECIPE_HEADER_BYTES ||
	    get_unaligned_le32(header + 12) != BECORE_RECIPE_RECORD_BYTES ||
	    get_unaligned_le32(header + 16) != BECORE_RGBP_HEADER_COUNT ||
	    get_unaligned_le32(header + 20) != BECORE_YUVP_HEADER_COUNT ||
	    get_unaligned_le32(header + 24) != BECORE_RECIPE_BYTES ||
	    get_unaligned_le32(header + 28))
		return -EINVAL;

	return 0;
}

static dma_addr_t becore_address_dma(struct becore_device *becore, u32 reg)
{
	switch (reg) {
	case BECORE_RGBP_INPUT_IMAGE_REG:
		return becore->input.dma + BECORE_INPUT_IMAGE_OFFSET;
	case BECORE_RGBP_INPUT_HEADER_REG:
		return becore->input.dma;
	case BECORE_YUVP_GRID_REG:
		return becore->grid.dma;
	case BECORE_YUVP_OUTPUT_PLANE1_REG:
		return becore->output.dma;
	case BECORE_YUVP_OUTPUT_PLANE2_REG:
		return becore->output.dma + BECORE_OUTPUT_PLANE2_OFFSET;
	default:
		return DMA_MAPPING_ERROR;
	}
}

static int becore_recipe_block_validate(struct becore_device *becore,
					enum becore_block_id id,
					const struct becore_cmdq_shape *shape,
					u32 header_count)
{
	const u8 *record = becore_recipe_records(becore, id);
	u32 address_count = 0;
	u32 i;

	for (i = 0; i < header_count; i++, record += BECORE_RECIPE_RECORD_BYTES) {
		const u8 *words = record + 12;
		u16 used_mask;
		u32 word;

		if (!shape[i].valid_words || shape[i].valid_words > 16)
			return -EINVAL;
		used_mask = shape[i].valid_words == 16 ? U16_MAX :
			    GENMASK(shape[i].valid_words - 1, 0);
		if (get_unaligned_le32(record) != shape[i].mode ||
		    get_unaligned_le32(record + 4) != shape[i].target ||
		    get_unaligned_le32(record + 8) != shape[i].type_map ||
		    (shape[i].address_mask & ~used_mask) ||
		    (shape[i].fixed_mask & ~used_mask) ||
		    (shape[i].address_mask & shape[i].fixed_mask))
			return -EINVAL;

		for (word = 0; word < 16; word++) {
			u32 value = get_unaligned_le32(words + word * 4);

			if (word >= shape[i].valid_words) {
				if (value)
					return -EINVAL;
				continue;
			}

			if (shape[i].mode == 0x00090000 && !(word & 1)) {
				if (value != shape[i].pair_registers[word / 2])
					return -EINVAL;
				continue;
			}

			if (shape[i].fixed_mask & BIT(word)) {
				if (value != shape[i].fixed_values[word])
					return -EINVAL;
			}

			if (shape[i].address_mask & BIT(word)) {
				u32 reg;

				/* All five supported DMA fields are pair-mode values. */
				if (shape[i].mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape[i].pair_registers[word / 2];
				if (becore_address_dma(becore, reg) ==
				    DMA_MAPPING_ERROR)
					return -EINVAL;
				address_count++;
			}
		}
	}

	if ((id == BECORE_RGBP && address_count != 2) ||
	    (id == BECORE_YUVP && address_count != 3))
		return -EINVAL;

	return 0;
}

static int becore_recipe_validate(struct becore_device *becore)
{
	int ret;

	ret = becore_recipe_header_validate(becore);
	if (ret)
		return ret;
	if (becore->input.staged_bytes != BECORE_INPUT_SIZE ||
	    becore->grid.staged_bytes != BECORE_GRID_SIZE)
		return -EINVAL;

	ret = becore_recipe_block_validate(becore, BECORE_RGBP,
					   becore_rgbp_shape,
					   BECORE_RGBP_HEADER_COUNT);
	if (ret)
		return ret;

	return becore_recipe_block_validate(becore, BECORE_YUVP,
					    becore_yuvp_shape,
					    BECORE_YUVP_HEADER_COUNT);
}

static int becore_encode_block(struct becore_device *becore,
			       enum becore_block_id id,
			       const struct becore_cmdq_shape *shape,
			       u32 header_count)
{
	struct becore_cmdq_program *program = &becore->program[id];
	const u8 *record = becore_recipe_records(becore, id);
	size_t payload_offset = ALIGN((size_t)header_count *
				      BECORE_CMDQ_HEADER_BYTES,
				      BECORE_CMDQ_PAYLOAD_BYTES);
	u32 i;

	if (!program->cpu || program->header_count != header_count ||
	    program->size != becore_cmdq_program_size(header_count) ||
	    upper_32_bits(program->dma) ||
	    upper_32_bits(program->dma + program->size - 1))
		return -EINVAL;

	memset(program->cpu, 0, program->size);
	for (i = 0; i < header_count;
	     i++, record += BECORE_RECIPE_RECORD_BYTES) {
		u8 *header = (u8 *)program->cpu + i * BECORE_CMDQ_HEADER_BYTES;
		u8 *payload = (u8 *)program->cpu + payload_offset +
			      i * BECORE_CMDQ_PAYLOAD_BYTES;
		dma_addr_t payload_dma = program->dma + payload_offset +
					 i * BECORE_CMDQ_PAYLOAD_BYTES;
		u32 word;

		put_unaligned_le32(shape[i].mode, header);
		put_unaligned_le32(lower_32_bits(payload_dma), header + 4);
		put_unaligned_le32(shape[i].target, header + 8);
		put_unaligned_le32(shape[i].type_map, header + 12);
		memcpy(payload, record + 12, BECORE_CMDQ_PAYLOAD_BYTES);

		for (word = 0; word < shape[i].valid_words; word++) {
			dma_addr_t dma;
			u32 reg;

			if (!(shape[i].address_mask & BIT(word)))
				continue;
			reg = shape[i].pair_registers[word / 2];
			dma = becore_address_dma(becore, reg);
			if (dma == DMA_MAPPING_ERROR || upper_32_bits(dma))
				return -EINVAL;
			put_unaligned_le32(lower_32_bits(dma), payload + word * 4);
		}
	}

	return 0;
}

static int becore_encode_programs(struct becore_device *becore)
{
	int ret;

	ret = becore_encode_block(becore, BECORE_RGBP, becore_rgbp_shape,
				  BECORE_RGBP_HEADER_COUNT);
	if (ret)
		return ret;

	return becore_encode_block(becore, BECORE_YUVP, becore_yuvp_shape,
				   BECORE_YUVP_HEADER_COUNT);
}

static void becore_prepare_irqs(struct becore_block *block)
{
	writel_relaxed(block->int0_mask_prepare,
		       block->base + BECORE_INT0_ENABLE);
	if (block->int1_mask)
		writel_relaxed(block->int1_mask,
			       block->base + BECORE_INT1_ENABLE);
	writel_relaxed(block->cmdq_int_mask,
		       block->base + BECORE_CMDQ_INT_ENABLE);
}

static void becore_quiesce_irqs(struct becore_block *block)
{
	writel_relaxed(BIT(2), block->base + BECORE_INT0_ENABLE);
	if (block->int1_mask)
		writel_relaxed(0, block->base + BECORE_INT1_ENABLE);
	writel_relaxed(0, block->base + BECORE_CMDQ_INT_ENABLE);
}

static void becore_enable_linux_irqs(struct becore_device *becore)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(becore->irqs); i++)
		enable_irq(becore->irqs[i].irq);
	becore->irqs_enabled = true;
}

static void becore_disable_linux_irqs(struct becore_device *becore)
{
	unsigned int i;

	if (!becore->irqs_enabled)
		return;

	/* disable_irq() also waits for a handler already running on another CPU. */
	for (i = 0; i < ARRAY_SIZE(becore->irqs); i++)
		disable_irq(becore->irqs[i].irq);
	becore->irqs_enabled = false;
}

static int becore_runtime_resume(struct device *dev)
{
	struct becore_device *becore = dev_get_drvdata(dev);
	unsigned int i;
	int ret;

	ret = becore_reset_all(becore);
	if (ret) {
		/* Report success so runtime PM retains all supplier references. */
		becore->reset_failed = true;
		return 0;
	}
	becore->reset_failed = false;

	becore_write_table(&becore->blocks[BECORE_RGBP], becore_rgbp_init,
			   ARRAY_SIZE(becore_rgbp_init));
	becore_write_table(&becore->blocks[BECORE_MCFP], becore_mcfp_init,
			   ARRAY_SIZE(becore_mcfp_init));
	becore_write_table(&becore->blocks[BECORE_YUVP], becore_yuvp_init,
			   ARRAY_SIZE(becore_yuvp_init));

	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		becore_prepare_irqs(&becore->blocks[i]);
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		writel_relaxed(becore->blocks[i].int0_mask,
			       becore->blocks[i].base + BECORE_INT0_ENABLE);

	becore_enable_linux_irqs(becore);

	return 0;
}

static int becore_runtime_suspend(struct device *dev)
{
	struct becore_device *becore = dev_get_drvdata(dev);
	unsigned int i;
	int ret;

	becore_disable_linux_irqs(becore);
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		becore_quiesce_irqs(&becore->blocks[i]);
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		writel_relaxed(0, becore->blocks[i].base + BECORE_INT0_ENABLE);

	/* A failed reset must veto the following genpd power-down. */
	ret = becore_reset_all(becore);
	becore->reset_failed = !!ret;

	return ret;
}

static void becore_process_frame_irq(struct becore_block *block, u32 status,
				     bool error)
{
	struct becore_device *becore = block->becore;
	enum becore_block_id id = block - becore->blocks;
	unsigned long flags;
	bool complete_run = false;

	if (id != BECORE_RGBP && id != BECORE_YUVP)
		return;

	spin_lock_irqsave(&becore->run_lock, flags);
	if (!becore->running)
		goto unlock;
	if (error) {
		becore->irq_error = true;
		becore->abort_run = true;
		complete_run = true;
		goto unlock;
	}

	if (status & BECORE_INT_CMDQ_HOLD) {
		becore->cmdq_hold_mask |= BIT(id);
		if (becore->cmdq_hold_mask == BECORE_ACTIVE_BLOCKS &&
		    !becore->start_issued && !becore->abort_run) {
			becore->start_issued = true;
			/* Release the downstream end of the chain first. */
			writel(1, becore->blocks[BECORE_YUVP].base +
			       BECORE_CMDQ_ADD_TO_QUEUE_0);
			writel(1, becore->blocks[BECORE_RGBP].base +
			       BECORE_CMDQ_ADD_TO_QUEUE_0);
		}
	}
	if (status & BECORE_INT_FRAME_END) {
		becore->frame_done_mask |= BIT(id);
		if (becore->frame_done_mask == BECORE_ACTIVE_BLOCKS)
			complete_run = true;
	}

unlock:
	spin_unlock_irqrestore(&becore->run_lock, flags);

	if (complete_run)
		complete(&becore->run_completion);
}

static irqreturn_t becore_irq_handler(int irq, void *data)
{
	struct becore_irq *irq_data = data;
	struct becore_block *block = irq_data->block;
	u32 cmdq_status;
	u32 status;

	if (irq_data->int1) {
		status = readl_relaxed(block->base + BECORE_INT1_STATUS);
		if (!status)
			return IRQ_NONE;

		writel_relaxed(status, block->base + BECORE_INT1_CLEAR);
		WRITE_ONCE(block->last_int1, status);
		atomic64_inc(&block->int1_count);
		becore_process_frame_irq(block, status, true);
		return IRQ_HANDLED;
	}

	status = readl_relaxed(block->base + BECORE_INT0_STATUS);
	cmdq_status = readl_relaxed(block->base + BECORE_CMDQ_INT_STATUS);
	if (!status && !cmdq_status)
		return IRQ_NONE;

	if (status)
		writel_relaxed(status, block->base + BECORE_INT0_CLEAR);
	if (cmdq_status)
		writel_relaxed(cmdq_status, block->base + BECORE_CMDQ_INT_CLEAR);

	WRITE_ONCE(block->last_int0, status);
	WRITE_ONCE(block->last_cmdq_int, cmdq_status);
	atomic64_inc(&block->int0_count);
	becore_process_frame_irq(block, status,
				 !!cmdq_status || !!(status & ~BECORE_INT_EXPECTED));

	return IRQ_HANDLED;
}

static int becore_map_resources(struct platform_device *pdev,
				struct becore_device *becore)
{
	static const char * const block_names[] = { "rgbp", "mcfp", "yuvp" };
	struct device *dev = &pdev->dev;
	unsigned int i;

	for (i = 0; i < BECORE_NUM_BLOCKS; i++) {
		becore->blocks[i].base =
			devm_platform_ioremap_resource_byname(pdev, block_names[i]);
		if (IS_ERR(becore->blocks[i].base))
			return dev_err_probe(dev, PTR_ERR(becore->blocks[i].base),
					     "cannot map %s\n", block_names[i]);
	}

	for (i = 0; i < ARRAY_SIZE(becore->ssmt); i++) {
		becore->ssmt[i] =
			devm_platform_ioremap_resource_byname(pdev,
						       becore_ssmt_names[i]);
		if (IS_ERR(becore->ssmt[i]))
			return dev_err_probe(dev, PTR_ERR(becore->ssmt[i]),
					     "cannot map %s\n", becore_ssmt_names[i]);
	}

	becore->sysreg_rgbp =
		devm_platform_ioremap_resource_byname(pdev, "sysreg-rgbp");
	if (IS_ERR(becore->sysreg_rgbp))
		return dev_err_probe(dev, PTR_ERR(becore->sysreg_rgbp),
				     "cannot map sysreg-rgbp\n");

	return 0;
}

static int becore_request_irqs(struct platform_device *pdev,
				struct becore_device *becore)
{
	struct device *dev = &pdev->dev;
	unsigned int i;
	int irq;
	int ret;

	for (i = 0; i < ARRAY_SIZE(becore->irqs); i++) {
		irq = platform_get_irq_byname(pdev, becore_irq_names[i]);
		if (irq < 0)
			return irq;

		becore->irqs[i].block = &becore->blocks[i / 2];
		becore->irqs[i].irq = irq;
		becore->irqs[i].int1 = i & 1;
		ret = devm_request_irq(dev, irq, becore_irq_handler,
				       IRQF_NO_AUTOEN,
				       becore_irq_names[i], &becore->irqs[i]);
		if (ret)
			return dev_err_probe(dev, ret, "cannot request %s\n",
					     becore_irq_names[i]);
	}

	return 0;
}

static int becore_alloc_dma_buffer(struct becore_device *becore,
				   struct becore_dma_buffer *buffer,
				   size_t size, const char *name)
{
	buffer->cpu = dmam_alloc_coherent(becore->dev, size, &buffer->dma,
					  GFP_KERNEL);
	if (!buffer->cpu)
		return dev_err_probe(becore->dev, -ENOMEM,
				     "cannot allocate %s buffer\n", name);
	buffer->size = size;
	if (upper_32_bits(buffer->dma) ||
	    upper_32_bits(buffer->dma + buffer->size - 1))
		return dev_err_probe(becore->dev, -ERANGE,
				     "%s buffer is outside 32-bit DMA\n", name);

	return 0;
}

static void becore_free_shared_input(void *data)
{
	struct becore_device *becore = data;
	struct becore_dma_buffer *input = &becore->input;

	if (input->cpu)
		dma_vunmap_noncontiguous(becore->dev, input->cpu);
	if (input->sgt)
		dma_free_noncontiguous(becore->dev, input->size, input->sgt,
				       DMA_BIDIRECTIONAL);
	input->cpu = NULL;
	input->sgt = NULL;
}

static int becore_alloc_shared_input(struct becore_device *becore)
{
	struct becore_dma_buffer *input = &becore->input;
	int ret;

	input->size = BECORE_INPUT_SIZE;
	input->sgt = dma_alloc_noncontiguous(becore->dev, input->size,
					     DMA_BIDIRECTIONAL, GFP_KERNEL, 0);
	if (!input->sgt)
		return dev_err_probe(becore->dev, -ENOMEM,
				     "cannot allocate Bayer input pages\n");

	input->dma = sg_dma_address(input->sgt->sgl);
	input->cpu = dma_vmap_noncontiguous(becore->dev, input->size,
					    input->sgt);
	if (!input->cpu) {
		becore_free_shared_input(becore);
		return dev_err_probe(becore->dev, -ENOMEM,
				     "cannot map Bayer input pages\n");
	}
	if (upper_32_bits(input->dma) ||
	    upper_32_bits(input->dma + input->size - 1)) {
		becore_free_shared_input(becore);
		return dev_err_probe(becore->dev, -ERANGE,
				     "Bayer input is outside 32-bit DMA\n");
	}

	ret = devm_add_action_or_reset(becore->dev,
				       becore_free_shared_input, becore);
	return ret;
}

static int becore_alloc_cmdq_program(struct becore_device *becore,
				     enum becore_block_id id,
				     u32 header_count)
{
	struct becore_cmdq_program *program = &becore->program[id];

	program->header_count = header_count;
	program->size = becore_cmdq_program_size(header_count);
	program->cpu = dmam_alloc_coherent(becore->dev, program->size,
					   &program->dma, GFP_KERNEL);
	if (!program->cpu)
		return dev_err_probe(becore->dev, -ENOMEM,
				     "cannot allocate %s CMDQ program\n",
				     becore->blocks[id].name);
	if (upper_32_bits(program->dma) ||
	    upper_32_bits(program->dma + program->size - 1))
		return dev_err_probe(becore->dev, -ERANGE,
				     "%s CMDQ program is outside 32-bit DMA\n",
				     becore->blocks[id].name);

	return 0;
}

static int becore_alloc_diagnostic(struct becore_device *becore)
{
	int ret;

	becore->recipe = devm_kzalloc(becore->dev, BECORE_RECIPE_BYTES,
				      GFP_KERNEL);
	if (!becore->recipe)
		return -ENOMEM;

	ret = becore_alloc_shared_input(becore);
	if (ret)
		return ret;
	ret = becore_alloc_dma_buffer(becore, &becore->grid,
				      BECORE_GRID_SIZE, "YUVP grid");
	if (ret)
		return ret;
	ret = becore_alloc_dma_buffer(becore, &becore->output,
				      BECORE_OUTPUT_SIZE, "YUVP output");
	if (ret)
		return ret;
	ret = becore_alloc_cmdq_program(becore, BECORE_RGBP,
					BECORE_RGBP_HEADER_COUNT);
	if (ret)
		return ret;

	return becore_alloc_cmdq_program(becore, BECORE_YUVP,
					 BECORE_YUVP_HEADER_COUNT);
}

static int becore_clone_sgtable(struct sg_table *dst,
				struct sg_table *src)
{
	struct scatterlist *src_sg;
	struct scatterlist *dst_sg;
	unsigned int i;
	int ret;

	ret = sg_alloc_table(dst, src->orig_nents, GFP_KERNEL);
	if (ret)
		return ret;

	dst_sg = dst->sgl;
	for_each_sg(src->sgl, src_sg, src->orig_nents, i) {
		sg_set_page(dst_sg, sg_page(src_sg), src_sg->length,
			    src_sg->offset);
		dst_sg = sg_next(dst_sg);
	}

	return 0;
}

/**
 * exynos_becore_input_map() - map the BE-core input into its producer domain
 * @backend: BE-core platform device
 * @producer: device which will write the compressed Bayer object
 *
 * The returned DMA address belongs to @producer.  BE-core retains its own
 * mapping of the same pages and never exposes that IOVA to the producer.
 */
struct exynos_becore_input *
exynos_becore_input_map(struct device *backend, struct device *producer)
{
	struct becore_device *becore;
	struct exynos_becore_input *input;
	int ret;

	if (!backend || !producer)
		return ERR_PTR(-EINVAL);
	becore = dev_get_drvdata(backend);
	if (!becore || !becore->input.sgt)
		return ERR_PTR(-EPROBE_DEFER);

	input = kzalloc_obj(*input, GFP_KERNEL);
	if (!input)
		return ERR_PTR(-ENOMEM);
	input->becore = becore;
	input->producer = get_device(producer);

	ret = becore_clone_sgtable(&input->sgt, becore->input.sgt);
	if (ret)
		goto err_put;
	ret = dma_map_sgtable(producer, &input->sgt, DMA_FROM_DEVICE, 0);
	if (ret)
		goto err_sg;
	input->dma = sg_dma_address(input->sgt.sgl);
	if (input->sgt.nents != 1 ||
	    sg_dma_len(input->sgt.sgl) < becore->input.size ||
	    upper_32_bits(input->dma) ||
	    upper_32_bits(input->dma + becore->input.size - 1)) {
		ret = -ERANGE;
		goto err_unmap;
	}

	mutex_lock(&becore->lock);
	if (becore->input_producer) {
		ret = -EBUSY;
		mutex_unlock(&becore->lock);
		goto err_unmap;
	}
	becore->input_producer = input;
	mutex_unlock(&becore->lock);

	return input;

err_unmap:
	dma_unmap_sgtable(producer, &input->sgt, DMA_FROM_DEVICE, 0);
err_sg:
	sg_free_table(&input->sgt);
err_put:
	put_device(input->producer);
	kfree(input);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(exynos_becore_input_map);

void exynos_becore_input_unmap(struct exynos_becore_input *input)
{
	struct becore_device *becore;

	if (!input)
		return;
	becore = input->becore;

	mutex_lock(&becore->lock);
	if (WARN_ON_ONCE(becore->input_producer != input)) {
		mutex_unlock(&becore->lock);
		return;
	}
	if (input->producing) {
		dma_sync_sgtable_for_cpu(input->producer, &input->sgt,
					 DMA_FROM_DEVICE);
		input->producing = false;
		becore->input.staged_bytes = 0;
	}
	becore->input_producer = NULL;
	mutex_unlock(&becore->lock);

	dma_unmap_sgtable(input->producer, &input->sgt, DMA_FROM_DEVICE, 0);
	sg_free_table(&input->sgt);
	put_device(input->producer);
	kfree(input);
}
EXPORT_SYMBOL_GPL(exynos_becore_input_unmap);

dma_addr_t exynos_becore_input_dma(struct exynos_becore_input *input)
{
	return input->dma;
}
EXPORT_SYMBOL_GPL(exynos_becore_input_dma);

size_t exynos_becore_input_size(struct exynos_becore_input *input)
{
	return input->becore->input.size;
}
EXPORT_SYMBOL_GPL(exynos_becore_input_size);

int exynos_becore_input_producer_begin(struct exynos_becore_input *input)
{
	struct becore_device *becore = input->becore;
	int ret = 0;

	mutex_lock(&becore->lock);
	if (becore->input_producer != input || input->producing) {
		ret = -EINVAL;
		goto unlock;
	}
	if (becore->running) {
		ret = -EBUSY;
		goto unlock;
	}

	/* Discard any cached CPU copy before the front end overwrites it. */
	dma_sync_sgtable_for_device(becore->dev, becore->input.sgt,
				    DMA_BIDIRECTIONAL);
	dma_sync_sgtable_for_device(input->producer, &input->sgt,
				    DMA_FROM_DEVICE);
	becore->input.staged_bytes = 0;
	input->producing = true;

unlock:
	mutex_unlock(&becore->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_becore_input_producer_begin);

int exynos_becore_input_producer_complete(struct exynos_becore_input *input)
{
	struct becore_device *becore = input->becore;
	int ret = 0;

	mutex_lock(&becore->lock);
	if (becore->input_producer != input || !input->producing) {
		ret = -EINVAL;
		goto unlock;
	}

	/* The caller has quiesced the producer at a completed-frame boundary. */
	dma_sync_sgtable_for_cpu(input->producer, &input->sgt,
				 DMA_FROM_DEVICE);
	dma_sync_sgtable_for_device(becore->dev, becore->input.sgt,
				    DMA_TO_DEVICE);
	input->producing = false;
	becore->input.staged_bytes = becore->input.size;

unlock:
	mutex_unlock(&becore->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_becore_input_producer_complete);

static ssize_t becore_stage_write(struct becore_device *becore,
				  const char __user *buf, size_t count,
				  loff_t *ppos, void *staged, size_t capacity,
				  size_t *staged_bytes, u32 *generation)
{
	ssize_t ret = count;

	if (!count)
		return 0;

	mutex_lock(&becore->lock);
	if (becore->running) {
		ret = -EBUSY;
		goto unlock;
	}
	if (staged == becore->input.cpu && becore->input_producer &&
	    becore->input_producer->producing) {
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
	if (*ppos == 0)
		*staged_bytes = 0;
	if (*ppos != *staged_bytes) {
		ret = -ESPIPE;
		goto unlock;
	}
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
	if (becore->running)
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
				  &becore->recipe_generation);
}

static const struct file_operations becore_recipe_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_recipe_read,
	.write = becore_recipe_write,
	.llseek = default_llseek,
};

static ssize_t becore_input_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;

	return becore_stage_write(becore, buf, count, ppos,
				  becore->input.cpu, becore->input.size,
				  &becore->input.staged_bytes, NULL);
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
				  &becore->grid.staged_bytes, NULL);
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
	if (becore->running)
		ret = -EBUSY;
	else if (!becore->completed_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->output.cpu,
					      becore->output.size);
	mutex_unlock(&becore->lock);

	return ret;
}

static const struct file_operations becore_output_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_output_read,
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
	if (becore->running)
		ret = -EBUSY;
	else if (!becore->completed_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos, program->cpu,
					      program->size);
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

static void becore_clear_pending_irqs(struct becore_block *block)
{
	writel_relaxed(U32_MAX, block->base + BECORE_INT0_CLEAR);
	writel_relaxed(U32_MAX, block->base + BECORE_INT1_CLEAR);
	writel_relaxed(U32_MAX, block->base + BECORE_CMDQ_INT_CLEAR);
}

static void becore_publish_program(struct becore_device *becore,
				   enum becore_block_id id)
{
	struct becore_cmdq_program *program = &becore->program[id];
	struct becore_block *block = &becore->blocks[id];

	writel_relaxed(lower_32_bits(program->dma),
		       block->base + BECORE_CMDQ_QUE_CMD_L);
	writel_relaxed(BECORE_CMDQ_MODE | program->header_count,
		       block->base + BECORE_CMDQ_QUE_CMD_M);
	writel_relaxed(0xff, block->base + BECORE_CMDQ_QUE_CMD_H);
	writel_relaxed(1, block->base + BECORE_CMDQ_QUE_CMD_START);
}

static void becore_measure_output(struct becore_device *becore)
{
	const u8 *output = becore->output.cpu;
	u32 changed = 0;
	u32 first = U32_MAX;
	u32 i;

	for (i = 0; i < becore->output.size; i++) {
		if (output[i] == 0xa5)
			continue;
		if (first == U32_MAX)
			first = i;
		changed++;
	}
	becore->output_changed_bytes = changed;
	becore->output_first_changed = first;
}

static int becore_run_set(void *data, u64 value)
{
	struct becore_device *becore = data;
	unsigned long flags;
	unsigned long waited;
	int pm_ret;
	int ret;
	u32 i;

	if (value != 1)
		return -EINVAL;

	mutex_lock(&becore->lock);
	if (becore->running) {
		ret = -EBUSY;
		goto unlock;
	}
	if (becore->reset_failed) {
		ret = -EIO;
		goto record_error;
	}
	ret = becore_recipe_validate(becore);
	if (ret)
		goto record_error;

	ret = pm_runtime_resume_and_get(becore->dev);
	if (ret)
		goto record_error;
	if (becore->reset_failed) {
		/* The runtime callback deliberately retains every supplier. */
		ret = -EIO;
		goto record_error;
	}

	ret = becore_encode_programs(becore);
	if (ret)
		goto put_power;

	memset(becore->output.cpu, 0xa5, becore->output.size);
	becore->output_changed_bytes = 0;
	becore->output_first_changed = U32_MAX;
	for (i = 0; i < BECORE_NUM_BLOCKS; i++) {
		WRITE_ONCE(becore->blocks[i].last_int0, 0);
		WRITE_ONCE(becore->blocks[i].last_int1, 0);
		WRITE_ONCE(becore->blocks[i].last_cmdq_int, 0);
		atomic64_set(&becore->blocks[i].int0_count, 0);
		atomic64_set(&becore->blocks[i].int1_count, 0);
	}
	becore_clear_pending_irqs(&becore->blocks[BECORE_RGBP]);
	becore_clear_pending_irqs(&becore->blocks[BECORE_YUVP]);
	reinit_completion(&becore->run_completion);

	spin_lock_irqsave(&becore->run_lock, flags);
	becore->cmdq_hold_mask = 0;
	becore->frame_done_mask = 0;
	becore->start_issued = false;
	becore->abort_run = false;
	becore->irq_error = false;
	becore->running = true;
	becore->run_generation++;
	spin_unlock_irqrestore(&becore->run_lock, flags);

	/* Move staged or producer-written Bayer pages into RGBP's DMA domain. */
	dma_sync_sgtable_for_device(becore->dev, becore->input.sgt,
				    DMA_TO_DEVICE);
	/* Make the command programs and all DMA ownership changes visible. */
	dma_wmb();
	becore_publish_program(becore, BECORE_RGBP);
	becore_publish_program(becore, BECORE_YUVP);
	mutex_unlock(&becore->lock);

	waited = wait_for_completion_timeout(&becore->run_completion,
					     msecs_to_jiffies(BECORE_RUN_TIMEOUT_MS));

	mutex_lock(&becore->lock);
	spin_lock_irqsave(&becore->run_lock, flags);
	becore->running = false;
	if (!waited)
		ret = -ETIMEDOUT;
	else if (becore->irq_error)
		ret = -EIO;
	else if (becore->abort_run)
		ret = -ECANCELED;
	else if (becore->frame_done_mask != BECORE_ACTIVE_BLOCKS)
		ret = -EIO;
	else
		ret = 0;
	spin_unlock_irqrestore(&becore->run_lock, flags);

	/* runtime_suspend synchronizes IRQs and resets all three processors. */
	pm_ret = pm_runtime_put_sync(becore->dev);
	if (pm_ret < 0) {
		/* Match probe: never leave a failed-reset device at usage zero. */
		pm_runtime_get_noresume(becore->dev);
		if (!ret)
			ret = pm_ret;
	}
	dma_sync_sgtable_for_cpu(becore->dev, becore->input.sgt,
				 DMA_TO_DEVICE);
	dma_rmb();
	becore_measure_output(becore);
	becore->completed_generation = becore->run_generation;
	becore->last_run_result = ret;
	mutex_unlock(&becore->lock);

	return ret;

put_power:
	pm_ret = pm_runtime_put_sync(becore->dev);
	if (pm_ret < 0) {
		pm_runtime_get_noresume(becore->dev);
		if (!ret)
			ret = pm_ret;
	}
record_error:
	becore->last_run_result = ret;
unlock:
	mutex_unlock(&becore->lock);
	return ret;
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

static int becore_cancel_set(void *data, u64 value)
{
	struct becore_device *becore = data;
	unsigned long flags;
	bool cancelled = false;

	if (value != 1)
		return -EINVAL;

	mutex_lock(&becore->lock);
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

static int becore_status_show(struct seq_file *s, void *unused)
{
	struct becore_device *becore = s->private;
	unsigned long flags;
	u32 cmdq_hold_mask;
	u32 frame_done_mask;
	bool start_issued;
	bool irq_error;
	bool running;
	u32 i;

	mutex_lock(&becore->lock);
	spin_lock_irqsave(&becore->run_lock, flags);
	running = becore->running;
	cmdq_hold_mask = becore->cmdq_hold_mask;
	frame_done_mask = becore->frame_done_mask;
	start_issued = becore->start_issued;
	irq_error = becore->irq_error;
	spin_unlock_irqrestore(&becore->run_lock, flags);
	seq_printf(s, "running          %u\n", running);
	seq_printf(s, "runtime          %s\n",
		   pm_runtime_status_suspended(becore->dev) ? "suspended" : "active");
	seq_printf(s, "recipe           %zu/%u bytes, generation %u\n",
		   becore->recipe_staged_bytes, BECORE_RECIPE_BYTES,
		   becore->recipe_generation);
	seq_printf(s, "input            %zu/%zu bytes, iova %pad\n",
		   becore->input.staged_bytes, becore->input.size,
		   &becore->input.dma);
	if (becore->input_producer)
		seq_printf(s, "input_producer   %s iova %pad, active %u\n",
			   dev_name(becore->input_producer->producer),
			   &becore->input_producer->dma,
			   becore->input_producer->producing);
	seq_printf(s, "grid             %zu/%zu bytes, iova %pad\n",
		   becore->grid.staged_bytes, becore->grid.size,
		   &becore->grid.dma);
	seq_printf(s, "output           %zu bytes, iova %pad\n",
		   becore->output.size, &becore->output.dma);
	seq_printf(s, "rgbp_cmdq        %zu bytes/%u headers, iova %pad\n",
		   becore->program[BECORE_RGBP].size,
		   becore->program[BECORE_RGBP].header_count,
		   &becore->program[BECORE_RGBP].dma);
	seq_printf(s, "yuvp_cmdq        %zu bytes/%u headers, iova %pad\n",
		   becore->program[BECORE_YUVP].size,
		   becore->program[BECORE_YUVP].header_count,
		   &becore->program[BECORE_YUVP].dma);
	seq_printf(s, "run_generation   %u\n", becore->run_generation);
	seq_printf(s, "completed         %u\n", becore->completed_generation);
	seq_printf(s, "last_result       %d\n", becore->last_run_result);
	seq_printf(s, "cmdq_hold         %#x\n", cmdq_hold_mask);
	seq_printf(s, "frame_done        %#x\n", frame_done_mask);
	seq_printf(s, "start_issued      %u\n", start_issued);
	seq_printf(s, "irq_error         %u\n", irq_error);
	seq_printf(s, "reset_failed      %u\n", becore->reset_failed);
	seq_printf(s, "output_changed    %u\n", becore->output_changed_bytes);
	seq_printf(s, "output_first      %#x\n", becore->output_first_changed);
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

static void becore_debugfs_remove(void *data)
{
	struct becore_device *becore = data;

	debugfs_remove_recursive(becore->debugfs);
}

static int becore_debugfs_init(struct becore_device *becore)
{
	struct dentry *dir;

	dir = debugfs_create_dir(dev_name(becore->dev), NULL);
	if (IS_ERR(dir))
		return PTR_ERR(dir);
	becore->debugfs = dir;
	debugfs_create_file("program", 0600, dir, becore, &becore_recipe_fops);
	debugfs_create_file("input", 0200, dir, becore, &becore_input_fops);
	debugfs_create_file("grid", 0200, dir, becore, &becore_grid_fops);
	debugfs_create_file("output", 0400, dir, becore, &becore_output_fops);
	debugfs_create_file("rgbp_cmdq", 0400, dir, becore,
			    &becore_rgbp_encoded_fops);
	debugfs_create_file("yuvp_cmdq", 0400, dir, becore,
			    &becore_yuvp_encoded_fops);
	debugfs_create_file("run", 0600, dir, becore, &becore_run_fops);
	debugfs_create_file("cancel", 0200, dir, becore, &becore_cancel_fops);
	debugfs_create_file("status", 0400, dir, becore, &becore_status_fops);

	return devm_add_action_or_reset(becore->dev, becore_debugfs_remove,
					becore);
}

static int becore_probe(struct platform_device *pdev)
{
	static const struct dev_pm_domain_attach_data pm_domain_data = {
		.pd_names = becore_pm_domain_names,
		.num_pd_names = ARRAY_SIZE(becore_pm_domain_names),
	};
	struct device *dev = &pdev->dev;
	struct becore_device *becore;
	int ret;

	becore = devm_kzalloc(dev, sizeof(*becore), GFP_KERNEL);
	if (!becore)
		return -ENOMEM;

	becore->dev = dev;
	mutex_init(&becore->lock);
	spin_lock_init(&becore->run_lock);
	init_completion(&becore->run_completion);
	becore->output_first_changed = U32_MAX;
	becore->blocks[BECORE_RGBP] = (struct becore_block) {
		.becore = becore,
		.name = "RGBP",
		.int0_mask_prepare = 0x18e1fc02,
		.int0_mask = 0x18e1fc06,
		.int1_mask = 0x7fff,
		.cmdq_int_mask = 0x7,
	};
	becore->blocks[BECORE_MCFP] = (struct becore_block) {
		.becore = becore,
		.name = "MCFP",
		.int0_mask_prepare = 0x3ffffc02,
		.int0_mask = 0x3ffffc06,
		.cmdq_int_mask = 0x7,
	};
	becore->blocks[BECORE_YUVP] = (struct becore_block) {
		.becore = becore,
		.name = "YUVP",
		.int0_mask_prepare = 0x3fe1fc02,
		.int0_mask = 0x3fe1fc06,
		.int1_mask = 0x1ffffff,
		.cmdq_int_mask = 0xff,
	};
	platform_set_drvdata(pdev, becore);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "no 32-bit DMA\n");

	ret = becore_map_resources(pdev, becore);
	if (ret)
		return ret;

	ret = devm_pm_domain_attach_list(dev, &pm_domain_data,
					 &becore->pm_domains);
	if (ret < 0)
		return dev_err_probe(dev, ret, "cannot attach power domains\n");
	if (ret != (int)ARRAY_SIZE(becore_pm_domain_names))
		return dev_err_probe(dev, -ENODEV,
				     "attached %d of %zu power domains\n", ret,
				     ARRAY_SIZE(becore_pm_domain_names));

	ret = becore_request_irqs(pdev, becore);
	if (ret)
		return ret;
	ret = becore_alloc_diagnostic(becore);
	if (ret)
		return ret;

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot power camera back end\n");
	if (becore->reset_failed) {
		dev_crit(dev, "processor reset failed; retaining power\n");
		return 0;
	}

	ret = pm_runtime_put_sync(dev);
	if (ret < 0) {
		/* Keep all suppliers active when the reset vetoes power-down. */
		pm_runtime_get_noresume(dev);
		dev_crit(dev, "cannot quiesce camera back end; retaining power\n");
		return 0;
	}

	return becore_debugfs_init(becore);
}

static const struct dev_pm_ops becore_pm_ops = {
	SET_RUNTIME_PM_OPS(becore_runtime_suspend, becore_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static const struct of_device_id becore_of_match[] = {
	{ .compatible = "google,zumapro-becore" },
	{ }
};
MODULE_DEVICE_TABLE(of, becore_of_match);

static struct platform_driver becore_driver = {
	.probe = becore_probe,
	.driver = {
		.name = "exynos-becore",
		.of_match_table = becore_of_match,
		.pm = pm_ptr(&becore_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(becore_driver);

MODULE_DESCRIPTION("Google Zumapro camera back-end core");
MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_LICENSE("GPL");
