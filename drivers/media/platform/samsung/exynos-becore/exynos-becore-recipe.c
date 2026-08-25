// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Zumapro camera back-end core -- the captured program
 *
 * The offline path runs a program the vendor's stack once ran: a fixed list of
 * CMDQ headers with their payloads, uploaded through debugfs as one compact
 * record per header.  Nothing in it is trusted.  Every hardware pointer is
 * absent by construction and regenerated here; every word is either fixed by
 * the captured shape, a typed DMA field the surfaces describe, a generated
 * word the driver states, or a value a parameters buffer carries -- and the
 * validation below refuses a record whose counts do not match the shape it
 * claims, before anything is written to a register.
 *
 * The four sources are applied in that order into each record's payload, so
 * that parameters win over generated words and a debugfs override wins over
 * both.  becore_encode_block() is where the order lives, and it is the reason
 * a block whose value is not substituted comes out as whatever the source
 * before it left there rather than as nothing at all.
 */

#include <linux/align.h>
#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/dev_printk.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include "exynos-becore-regs.h"
#include "exynos-becore-common.h"
#include "exynos-becore-recipe.h"
#include "exynos-becore-gtnr-recipe.h"
#include "exynos-becore-mcsc-recipe.h"
/*
 * The seven DMA channels GTNR's *first-frame* program does not use, and the
 * compression control that travels with three of them. A merge with no
 * previous frame reads no previous frame (PRE0, PRE1), no second current
 * plane (CUR1), no motion or noise map (MMNP), no pyramid level (L0, L14) and
 * no time-of-flight input; it writes no temporal output (TNROUT) and no
 * pyramid back.
 *
 * Read that scope literally, because the census is emphatic about it: of the
 * 581 captured GTNR programs only ten -- the first-frame ones -- clear all
 * fifteen. The other 571 set every enable to 1 and every compression control
 * to the same lossy-SBWC 0xa the two channels in use get. So this is not a
 * property of the block or of the registers; it is what a merge with nothing
 * to merge against does, and the day the normal temporal program lands these
 * fifteen become its most ordinary values. It is scoped to becore_gtnr_shape,
 * the thirteen-header startup recipe, and a second recipe must bring its own
 * answer rather than reach for this one.
 *
 * These fifteen were the whole of what the startup recipe still replayed, so
 * with them stated it carries no captured value at all -- only the command
 * shape, the four image addresses and the typed words that describe the two
 * surfaces the merge does use. They cannot be checked the way the other three
 * blocks' can: GTNR has no MMIO, power or interrupt here and its program is
 * encoded and never submitted, so the recipe generator's own comparison
 * against the capture is the whole of the evidence.
 */
#define BECORE_GTNR_GENERATED_WORDS	15

static const u32 becore_gtnr_startup_off_regs[] = {
	BECORE_GTNR_PHYS_BASE + 0x1800,	/* RDMA_MMNP_EN */
	BECORE_GTNR_PHYS_BASE + 0x1a00,	/* RDMA_PRE0_EN */
	BECORE_GTNR_PHYS_BASE + 0x1a04,	/* RDMA_PRE0_COMP_CONTROL */
	BECORE_GTNR_PHYS_BASE + 0x1c00,	/* RDMA_PRE1_EN */
	BECORE_GTNR_PHYS_BASE + 0x1c04,	/* RDMA_PRE1_COMP_CONTROL */
	BECORE_GTNR_PHYS_BASE + 0x2000,	/* RDMA_CUR1_EN */
	BECORE_GTNR_PHYS_BASE + 0x2004,	/* RDMA_CUR1_COMP_CONTROL */
	BECORE_GTNR_PHYS_BASE + 0x2200,	/* RDMA_L0_EN */
	BECORE_GTNR_PHYS_BASE + 0x2400,	/* RDMA_L14_EN */
	BECORE_GTNR_PHYS_BASE + 0x2600,	/* RDMA_TOF_EN */
	BECORE_GTNR_PHYS_BASE + 0x2800,	/* WDMA_TNROUT_EN */
	BECORE_GTNR_PHYS_BASE + 0x2804,	/* WDMA_TNROUT_COMP_CONTROL */
	BECORE_GTNR_PHYS_BASE + 0x2a00,	/* WDMA_L0_EN */
	BECORE_GTNR_PHYS_BASE + 0x2c00,	/* WDMA_L14_EN */
	BECORE_GTNR_PHYS_BASE + 0x2e00,	/* WDMA_TOF_EN */
};

static_assert(ARRAY_SIZE(becore_gtnr_startup_off_regs) ==
	      BECORE_GTNR_GENERATED_WORDS);

/* Zero, if the startup program leaves this channel off; -EINVAL if not. */
static int becore_gtnr_startup_off_value(u32 reg, u32 *value)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_gtnr_startup_off_regs); i++) {
		if (becore_gtnr_startup_off_regs[i] != reg)
			continue;
		if (value)
			*value = 0;
		return 0;
	}

	return -EINVAL;
}

size_t becore_cmdq_program_size(u32 header_count)
{
	return ALIGN((size_t)header_count * BECORE_CMDQ_HEADER_BYTES,
		     BECORE_CMDQ_PAYLOAD_BYTES) +
	       (size_t)header_count * BECORE_CMDQ_PAYLOAD_BYTES;
}

/* The bytes the last encode filled, which is not the allocation. */
size_t becore_cmdq_encoded_size(const struct becore_cmdq_program *program)
{
	return becore_cmdq_program_size(program->header_count);
}

static const u8 *becore_recipe_records(const struct becore_device *becore,
				       enum becore_block_id id)
{
	const u8 *records = becore->recipe + BECORE_RECIPE_HEADER_BYTES;

	if (id == BECORE_YUVP)
		records += BECORE_RGBP_HEADER_COUNT * BECORE_RECIPE_RECORD_BYTES;

	return records;
}

/*
 * The register a value word programs. Pair-mode headers carry it beside the
 * value; sequential-mode ones step from the header's target.
 */
int becore_shape_register(const struct becore_cmdq_shape *shape,
			  u32 word, u32 *reg)
{
	if (word >= 16)
		return -EINVAL;
	if (shape->mode == 0x00090000) {
		if (!(word & 1))
			return -EINVAL;
		*reg = shape->pair_registers[word / 2];
		return 0;
	}
	if (shape->mode == 0x00080000) {
		*reg = shape->target + word * 4;
		return 0;
	}

	return -EINVAL;
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
	const struct becore_rgbp_input_profile *profile =
		becore_rgbp_input_profile(becore);
	struct becore_dma_buffer *input = &becore->run_input->buffer;

	switch (reg) {
	case BECORE_RGBP_INPUT_IMAGE_REG:
		return input->dma +
		       becore_rgbp_input_image_offset(profile, &becore->array);
	/*
	 * A profile with no header plane leaves the image at offset zero, so
	 * these two name the same page.  That is the right answer rather than
	 * a collision: with compression off the RDMA has no header to fetch,
	 * and the register still has to hold a mapped address.
	 */
	case BECORE_RGBP_INPUT_HEADER_REG:
		return input->dma;
	case BECORE_YUVP_GRID_REG:
		return becore->grid.dma;
	case BECORE_YUVP_OUTPUT_PLANE1_REG:
		return becore->active_output_dma;
	case BECORE_YUVP_OUTPUT_PLANE2_REG:
		return becore->active_output_dma +
		       becore_active_output_plane2_offset(becore);
	default:
		return DMA_MAPPING_ERROR;
	}
}

static bool becore_address_reg_valid(u32 reg)
{
	switch (reg) {
	case BECORE_RGBP_INPUT_IMAGE_REG:
	case BECORE_RGBP_INPUT_HEADER_REG:
	case BECORE_YUVP_GRID_REG:
	case BECORE_YUVP_OUTPUT_PLANE1_REG:
	case BECORE_YUVP_OUTPUT_PLANE2_REG:
		return true;
	default:
		return false;
	}
}

static int becore_recipe_block_validate(struct becore_device *becore,
					enum becore_block_id id,
					const struct becore_cmdq_shape *shape,
					u32 header_count, bool validate_dma)
{
	const u8 *record = becore_recipe_records(becore, id);
	u32 address_count = 0;
	u32 typed_count = 0;
	u32 generated_count = 0;
	u32 i;

	for (i = 0; i < header_count; i++, record += BECORE_RECIPE_RECORD_BYTES) {
		const u8 *words = record + 12;
		u16 used_mask;
		u16 value_mask;
		u32 word;

		if (!shape[i].valid_words || shape[i].valid_words > 16)
			return -EINVAL;
		used_mask = shape[i].valid_words == 16 ? U16_MAX :
			    GENMASK(shape[i].valid_words - 1, 0);
		value_mask = shape[i].mode == 0x00090000 ?
			     used_mask & 0xaaaa : used_mask;
		if (get_unaligned_le32(record) != shape[i].mode ||
		    get_unaligned_le32(record + 4) != shape[i].target ||
		    get_unaligned_le32(record + 8) != shape[i].type_map ||
		    (shape[i].address_mask | shape[i].typed_mask |
		     shape[i].generated_mask | shape[i].fixed_mask) !=
		     value_mask ||
		    (shape[i].address_mask & ~used_mask) ||
		    (shape[i].typed_mask & ~used_mask) ||
		    (shape[i].generated_mask & ~used_mask) ||
		    (shape[i].fixed_mask & ~used_mask) ||
		    (shape[i].address_mask & shape[i].typed_mask) ||
		    (shape[i].address_mask & shape[i].generated_mask) ||
		    (shape[i].address_mask & shape[i].fixed_mask) ||
		    (shape[i].typed_mask & shape[i].generated_mask) ||
		    (shape[i].typed_mask & shape[i].fixed_mask) ||
		    (shape[i].generated_mask & shape[i].fixed_mask))
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

			if (shape[i].typed_mask & BIT(word)) {
				u32 reg;

				if (shape[i].mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape[i].pair_registers[word / 2];
				if (becore_typed_value(becore, id, typed_count, reg,
						       NULL))
					return -EINVAL;
				typed_count++;
			}

			if (shape[i].generated_mask & BIT(word)) {
				u32 reg;

				if (becore_shape_register(&shape[i], word, &reg) ||
				    becore_generated_value(becore, id, reg, NULL))
					return -EINVAL;
				generated_count++;
			}

			if (shape[i].address_mask & BIT(word)) {
				u32 reg;

				/* All five supported DMA fields are pair-mode values. */
				if (shape[i].mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape[i].pair_registers[word / 2];
				if (!becore_address_reg_valid(reg) ||
				    (validate_dma &&
				     becore_address_dma(becore, reg) ==
				     DMA_MAPPING_ERROR))
					return -EINVAL;
				address_count++;
			}
		}
	}

	if ((id == BECORE_RGBP && address_count != 2) ||
	    (id == BECORE_YUVP && address_count != 3))
		return -EINVAL;
	if (typed_count != becore_typed_word_count(id) ||
	    generated_count != becore_generated_word_count(id))
		return -EINVAL;

	return 0;
}

int becore_recipe_records_validate(struct becore_device *becore,
				   bool validate_dma)
{
	int ret;

	ret = becore_recipe_header_validate(becore);
	if (ret)
		return ret;

	ret = becore_recipe_block_validate(becore, BECORE_RGBP,
					   becore_rgbp_shape,
					   BECORE_RGBP_HEADER_COUNT,
					   validate_dma);
	if (ret)
		return ret;

	return becore_recipe_block_validate(becore, BECORE_YUVP,
					    becore_yuvp_shape,
					    BECORE_YUVP_HEADER_COUNT,
					    validate_dma);
}

int becore_recipe_validate(struct becore_device *becore)
{
	struct becore_dma_buffer *input = &becore->run_input->buffer;
	size_t wanted =
		becore_rgbp_input_size(becore_rgbp_input_profile(becore),
				       &becore->array);

	/*
	 * The staged length is the check that the frame and the profile agree.
	 * A slot the producer filled always holds a whole allocation, so it
	 * only ever matches the live profile -- which is the second half of
	 * refusing a producer frame under the linear profile.
	 *
	 * And the raster the frame was written at, which the length cannot
	 * stand in for: becore_rgbp_input_size() reaches the width only
	 * through ALIGN(width, 256), so every array width in one 256-column
	 * bucket at one height lays out in the same number of bytes.  Without
	 * this, a frame written at 4208x3120 runs at 4352x3120 -- the length
	 * matches to the byte -- and 144 columns of compressed padding are
	 * read as picture, in bounds, with nothing saying why the result is
	 * wrong.  Which is reachable in exactly one direction: the array is a
	 * debugfs write for the offline loop, and the frames a producer writes
	 * are at a raster of its own.
	 */
	if (becore->run_input->raster.width != becore->array.width ||
	    becore->run_input->raster.height != becore->array.height) {
		/*
		 * Named rather than left as a bare -EINVAL, because the frame
		 * can have reached that slot from a different surface than the
		 * one being used: a producer's frame outranks the staged one
		 * in becore_next_input(), so an offline run at a second raster
		 * refuses while one is waiting, and nothing else says so.
		 * Running once at the raster named here consumes it.
		 */
		dev_err_ratelimited(becore->dev,
				    "the frame in this slot is %ux%u and the geometry is %ux%u\n",
				    becore->run_input->raster.width,
				    becore->run_input->raster.height,
				    becore->array.width, becore->array.height);
		return -EINVAL;
	}
	if (wanted > input->size || input->staged_bytes != wanted ||
	    becore->grid.staged_bytes != BECORE_GRID_SIZE)
		return -EINVAL;

	return becore_recipe_records_validate(becore, true);
}

/*
 * A debug override replaces the value of a word the program already writes.
 *
 * That restriction is the safety property, not a limitation to work around: an
 * override can change what a register is set to but can never add a write to a
 * register the encoder was not already going to touch, so no unmapped offset
 * and no unrelated block can be reached through it.  On top of that it refuses
 * every address and typed-DMA word outright, because those name driver-owned
 * memory and a wrong one points the hardware at pages it does not own.
 *
 * Only the three blocks the offline loop measures are covered.  GTNR's startup
 * program is not part of that chain and its registers are simply not found.
 */
int becore_override_check(u32 reg)
{
	static const struct {
		const struct becore_cmdq_shape *shape;
		u32 count;
	} programs[] = {
		{ becore_rgbp_shape, BECORE_RGBP_HEADER_COUNT },
		{ becore_yuvp_shape, BECORE_YUVP_HEADER_COUNT },
		{ becore_mcsc_shape, BECORE_MCSC_HEADER_COUNT },
	};
	bool found = false;
	size_t block;
	u32 i;
	u32 word;

	for (block = 0; block < ARRAY_SIZE(programs); block++) {
		for (i = 0; i < programs[block].count; i++) {
			const struct becore_cmdq_shape *shape =
				&programs[block].shape[i];

			/*
			 * Every word of a repeated-target header names the
			 * same register -- it is a streamed LUT port -- so one
			 * override would rewrite the whole payload rather than
			 * one field.  Answer that before asking which word,
			 * because becore_shape_register() cannot say.
			 *
			 * No shape carries such a header today: the colour
			 * LUT's lattice was the only one, and the driver emits
			 * that burst rather than replaying it, which puts it
			 * out of an override's reach entirely.  This stands
			 * for the next recipe that brings one back.
			 */
			if (shape->mode == 0x000b0000) {
				if (shape->target == reg)
					return -EPERM;
				continue;
			}
			for (word = 0; word < shape->valid_words; word++) {
				u32 candidate;

				if (shape->mode == 0x00090000 && !(word & 1))
					continue;
				if (becore_shape_register(shape, word,
							  &candidate) ||
				    candidate != reg)
					continue;
				if ((shape->address_mask | shape->typed_mask) &
				    BIT(word))
					return -EPERM;
				found = true;
			}
		}
	}

	return found ? 0 : -ENOENT;
}

/*
 * Substitute after the record's fixed words have been copied and the typed,
 * generated and address words written, so that parameters and then an override
 * win over every source -- and skip the two kinds becore_override_check()
 * already refuses, rather than relying on that refusal alone.
 */
static void becore_override_apply(const struct becore_device *becore,
				  const struct becore_cmdq_shape *shape,
				  u8 *payload)
{
	u32 word;
	u32 i;

	if (!becore->override_count)
		return;

	for (word = 0; word < shape->valid_words; word++) {
		u32 reg;

		if (shape->mode == 0x00090000 && !(word & 1))
			continue;
		if ((shape->address_mask | shape->typed_mask) & BIT(word))
			continue;
		if (becore_shape_register(shape, word, &reg))
			continue;
		for (i = 0; i < becore->override_count; i++) {
			if (becore->overrides[i].reg != reg)
				continue;
			put_unaligned_le32(becore->overrides[i].value,
					   payload + word * 4);
			break;
		}
	}
}

/*
 * The same substitution, from the parameters buffer rather than debugfs.
 *
 * There is deliberately no "is anything installed" shortcut in front of the
 * loop.  One used to name each block's validity flag, and adding a block
 * without adding it to that list made the new block reach the encoder and
 * stop there: the colour LUT's burst was emitted, its bypass gate was not
 * cleared, and the picture came out identical to a bypassed one with a
 * complete lattice sitting in a stage that never ran.  becore_params_value()
 * already answers -ENOENT for every register when nothing is installed, so
 * the loop is its own shortcut and cannot fall out of step with the blocks.
 */
static void becore_params_apply(const struct becore_device *becore,
				const struct becore_cmdq_shape *shape,
				u8 *payload)
{
	u32 word;

	for (word = 0; word < shape->valid_words; word++) {
		u32 value;
		u32 reg;

		if (shape->mode == 0x00090000 && !(word & 1))
			continue;
		if ((shape->address_mask | shape->typed_mask) & BIT(word))
			continue;
		if (becore_shape_register(shape, word, &reg) ||
		    becore_params_value(becore, reg, &value))
			continue;
		put_unaligned_le32(value, payload + word * 4);
	}
}

/*
 * Where the next header and its payload go.
 *
 * A program used to be exactly the recipe's headers in the recipe's order, and
 * an index served for both. It is not any more -- the colour LUT's burst is
 * emitted between two recipe headers and only when there is a lattice to send
 * -- so the output position has to be counted rather than derived, and the
 * payload area's start depends on how many headers the whole program will
 * have.
 */
struct becore_cmdq_emitter {
	struct becore_cmdq_program *program;
	size_t payload_offset;
	u32 count;
	u32 total;
};

static void becore_cmdq_emitter_init(struct becore_cmdq_emitter *emitter,
				     struct becore_cmdq_program *program,
				     u32 total)
{
	emitter->program = program;
	emitter->payload_offset = ALIGN((size_t)total *
					BECORE_CMDQ_HEADER_BYTES,
					BECORE_CMDQ_PAYLOAD_BYTES);
	emitter->count = 0;
	emitter->total = total;
}

/*
 * A header's type map: two bits per value word, 01 each, and nothing above
 * them.  Built by counting rather than by shifting a constant down, so that
 * neither an empty header nor a full one is a shift the width of the type.
 */
static u32 becore_cmdq_type_map(u32 words)
{
	u32 type_map = 0;
	u32 word;

	for (word = 0; word < words && word < BECORE_CMDQ_PAYLOAD_WORDS; word++)
		type_map |= 1u << (2 * word);

	return type_map;
}

/* One header, and the payload it points at -- zeroed, and the caller's to fill. */
static u8 *becore_cmdq_emit(struct becore_cmdq_emitter *emitter, u32 mode,
			    u32 target, u32 type_map)
{
	size_t offset = emitter->payload_offset +
			(size_t)emitter->count * BECORE_CMDQ_PAYLOAD_BYTES;
	u8 *header;

	if (emitter->count >= emitter->total)
		return NULL;
	header = (u8 *)emitter->program->cpu +
		 (size_t)emitter->count * BECORE_CMDQ_HEADER_BYTES;
	put_unaligned_le32(mode, header);
	put_unaligned_le32(lower_32_bits(emitter->program->dma + offset),
			   header + 4);
	put_unaligned_le32(target, header + 8);
	put_unaligned_le32(type_map, header + 12);
	emitter->count++;

	return (u8 *)emitter->program->cpu + offset;
}

/*
 * One word of the colour LUT's lattice: three 10-bit fields, low field first,
 * taken from a flat stream of one (U, V) pair per node.
 *
 * The samples are masked rather than trusted. buf_prepare refuses a lattice
 * with a sample wider than the field, but the consequence of one getting
 * through here would be corrupting a *neighbouring* node's chroma rather than
 * its own, which is a far harder thing to see in a picture.
 */
static u32 becore_clut_word(const struct becore_params_state *params, u32 index)
{
	u32 packed = 0;
	u32 field;

	for (field = 0; field < BECORE_CLUT_FIELDS_PER_WORD; field++) {
		u32 value = index * BECORE_CLUT_FIELDS_PER_WORD + field;
		u32 sample = EXYNOS_BECORE_CLUT_NEUTRAL;

		if (value < BECORE_CLUT_LATTICE_VALUES)
			sample = value & 1 ? params->clut_v[value / 2] :
					     params->clut_u[value / 2];
		packed |= (sample & BECORE_CLUT_FIELD_MAX) <<
			  (BECORE_CLUT_FIELD_BITS * field);
	}

	return packed;
}

/*
 * DIABLO_CLUT's lattice, as a burst the driver emits rather than words the
 * recipe carries.
 *
 * It cannot come from the recipe: every value in this driver is resolved by the
 * register it programs, and 205 headers all name one FIFO port, so that one
 * address would need 3,276 answers. The index register beside it is written
 * twice with two different values, which has the same problem. Both retire
 * together here.
 *
 * Where the burst goes matters and is not guessed: the vendor fills the lattice
 * at this exact point in YUVP's program, and whether the order relative to the
 * rest of the block matters is not something a register readback could show
 * going wrong. So it is emitted in place rather than appended.
 */
static int becore_encode_clut(const struct becore_device *becore,
			      struct becore_cmdq_emitter *emitter)
{
	const struct becore_params_state *params = &becore->params;
	u32 written = 0;
	u8 *payload;

	payload = becore_cmdq_emit(emitter, 0x00090000, 0,
				   becore_cmdq_type_map(2));
	if (!payload)
		return -EINVAL;
	put_unaligned_le32(BECORE_YUVP_CLUT_INDEX_REG, payload);
	put_unaligned_le32(BECORE_CLUT_FIFO_OPEN, payload + 4);

	while (written < BECORE_CLUT_LATTICE_WORDS) {
		u32 words = min(BECORE_CLUT_LATTICE_WORDS - written,
				(u32)BECORE_CMDQ_PAYLOAD_WORDS);
		u32 word;

		payload = becore_cmdq_emit(emitter, 0x000b0000,
					   BECORE_YUVP_CLUT_FIFO_REG,
					   becore_cmdq_type_map(words));
		if (!payload)
			return -EINVAL;
		for (word = 0; word < words; word++, written++)
			put_unaligned_le32(becore_clut_word(params, written),
					   payload + word * 4);
	}

	payload = becore_cmdq_emit(emitter, 0x00090000, 0,
				   becore_cmdq_type_map(2));
	if (!payload)
		return -EINVAL;
	put_unaligned_le32(BECORE_YUVP_CLUT_INDEX_REG, payload);
	put_unaligned_le32(0, payload + 4);

	return 0;
}

static int becore_encode_block(struct becore_device *becore,
			       enum becore_block_id id,
			       const struct becore_cmdq_shape *shape,
			       u32 header_count)
{
	struct becore_cmdq_program *program = &becore->program[id];
	const u8 *record = becore_recipe_records(becore, id);
	struct becore_cmdq_emitter emitter;
	bool clut = id == BECORE_YUVP && becore->params.clut_valid;
	u32 total = header_count +
		    (clut ? BECORE_YUVP_CLUT_BURST_HEADERS : 0);
	u32 i;
	u32 typed_count = 0;
	u32 generated_count = 0;
	int ret;

	/* An encode produces a whole program or none of one. */
	program->header_count = 0;
	if (!program->cpu || total > program->capacity ||
	    program->size != becore_cmdq_program_size(program->capacity) ||
	    upper_32_bits(program->dma) ||
	    upper_32_bits(program->dma + program->size - 1))
		return -EINVAL;

	memset(program->cpu, 0, program->size);
	becore_cmdq_emitter_init(&emitter, program, total);
	for (i = 0; i < header_count;
	     i++, record += BECORE_RECIPE_RECORD_BYTES) {
		u8 *payload;
		u32 word;

		if (clut && i == BECORE_YUVP_CLUT_HEADER) {
			ret = becore_encode_clut(becore, &emitter);
			if (ret)
				return ret;
		}

		payload = becore_cmdq_emit(&emitter, shape[i].mode,
					   shape[i].target, shape[i].type_map);
		if (!payload)
			return -EINVAL;
		memcpy(payload, record + 12, BECORE_CMDQ_PAYLOAD_BYTES);

		for (word = 0; word < shape[i].valid_words; word++) {
			dma_addr_t dma;
			u32 reg;
			u32 value;

			if (shape[i].typed_mask & BIT(word)) {
				reg = shape[i].pair_registers[word / 2];
				if (becore_typed_value(becore, id, typed_count, reg,
						       &value))
					return -EINVAL;
				put_unaligned_le32(value, payload + word * 4);
				typed_count++;
				continue;
			}

			if (shape[i].generated_mask & BIT(word)) {
				if (becore_shape_register(&shape[i], word, &reg) ||
				    becore_generated_value(becore, id, reg, &value))
					return -EINVAL;
				put_unaligned_le32(value, payload + word * 4);
				generated_count++;
				continue;
			}

			if (!(shape[i].address_mask & BIT(word)))
				continue;
			reg = shape[i].pair_registers[word / 2];
			dma = becore_address_dma(becore, reg);
			if (dma == DMA_MAPPING_ERROR || upper_32_bits(dma))
				return -EINVAL;
			put_unaligned_le32(lower_32_bits(dma), payload + word * 4);
		}
		becore_params_apply(becore, &shape[i], payload);
		becore_override_apply(becore, &shape[i], payload);
	}
	if (emitter.count != total ||
	    typed_count != becore_typed_word_count(id) ||
	    generated_count != becore_generated_word_count(id))
		return -EINVAL;
	program->header_count = total;

	return 0;
}

int becore_encode_programs(struct becore_device *becore)
{
	int ret;

	ret = becore_encode_block(becore, BECORE_RGBP, becore_rgbp_shape,
				  BECORE_RGBP_HEADER_COUNT);
	if (ret)
		return ret;

	return becore_encode_block(becore, BECORE_YUVP, becore_yuvp_shape,
				   BECORE_YUVP_HEADER_COUNT);
}

int becore_gtnr_recipe_validate(struct becore_device *becore)
{
	const u8 *header = becore->gtnr_recipe;
	const u8 *record = header + BECORE_GTNR_RECIPE_HEADER_BYTES;
	u32 address_count = 0;
	u32 generated_count = 0;
	u32 typed_count = 0;
	u32 i;

	if (becore->gtnr_recipe_staged_bytes != BECORE_GTNR_RECIPE_BYTES ||
	    get_unaligned_le32(header) != BECORE_GTNR_RECIPE_MAGIC ||
	    get_unaligned_le32(header + 4) != BECORE_GTNR_RECIPE_VERSION ||
	    get_unaligned_le32(header + 8) != BECORE_GTNR_RECIPE_HEADER_BYTES ||
	    get_unaligned_le32(header + 12) != BECORE_GTNR_RECIPE_RECORD_BYTES ||
	    get_unaligned_le32(header + 16) != BECORE_GTNR_HEADER_COUNT ||
	    get_unaligned_le32(header + 20) != BECORE_GTNR_RECIPE_BYTES ||
	    get_unaligned_le32(header + 24) || get_unaligned_le32(header + 28))
		return -EINVAL;
	if (becore->output.size < becore_gtnr_surface_size(&becore->chain) ||
	    becore->gtnr_output.size != becore_gtnr_surface_size(&becore->chain))
		return -EINVAL;

	for (i = 0; i < BECORE_GTNR_HEADER_COUNT;
	     i++, record += BECORE_GTNR_RECIPE_RECORD_BYTES) {
		const struct becore_cmdq_shape *shape = &becore_gtnr_shape[i];
		const u8 *words = record + 12;
		u16 used_mask;
		u32 word;

		if (!shape->valid_words || shape->valid_words > 16)
			return -EINVAL;
		used_mask = shape->valid_words == 16 ? U16_MAX :
			    GENMASK(shape->valid_words - 1, 0);
		if (get_unaligned_le32(record) != shape->mode ||
		    get_unaligned_le32(record + 4) != shape->target ||
		    get_unaligned_le32(record + 8) != shape->type_map ||
		    (shape->address_mask & ~used_mask) ||
		    (shape->typed_mask & ~used_mask) ||
		    (shape->generated_mask & ~used_mask) ||
		    (shape->fixed_mask & ~used_mask) ||
		    (shape->address_mask & shape->typed_mask) ||
		    (shape->address_mask & shape->generated_mask) ||
		    (shape->address_mask & shape->fixed_mask) ||
		    (shape->typed_mask & shape->generated_mask) ||
		    (shape->typed_mask & shape->fixed_mask) ||
		    (shape->generated_mask & shape->fixed_mask))
			return -EINVAL;

		for (word = 0; word < 16; word++) {
			u32 value = get_unaligned_le32(words + word * 4);

			if (word >= shape->valid_words) {
				if (value)
					return -EINVAL;
				continue;
			}
			if (shape->mode == 0x00090000 && !(word & 1)) {
				if (value != shape->pair_registers[word / 2])
					return -EINVAL;
				continue;
			}
			if ((shape->fixed_mask & BIT(word)) &&
			    value != shape->fixed_values[word])
				return -EINVAL;

			if (shape->typed_mask & BIT(word)) {
				u32 reg;

				if (shape->mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape->pair_registers[word / 2];
				if (becore_gtnr_dma_value(&becore->chain, typed_count, reg, NULL))
					return -EINVAL;
				typed_count++;
			}
			if (shape->generated_mask & BIT(word)) {
				u32 reg;

				if (shape->mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape->pair_registers[word / 2];
				if (becore_gtnr_startup_off_value(reg, NULL))
					return -EINVAL;
				generated_count++;
			}
			if (shape->address_mask & BIT(word)) {
				u32 reg;

				if (shape->mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape->pair_registers[word / 2];
				if (becore_gtnr_address_dma(becore, reg) ==
				    DMA_MAPPING_ERROR)
					return -EINVAL;
				address_count++;
			}
		}
	}

	if (address_count != 4 || typed_count != BECORE_GTNR_DMA_WORD_COUNT ||
	    generated_count != BECORE_GTNR_GENERATED_WORDS)
		return -EINVAL;

	return 0;
}

int becore_encode_gtnr(struct becore_device *becore)
{
	struct becore_cmdq_program *program = &becore->gtnr_program;
	const u8 *record = becore->gtnr_recipe +
			   BECORE_GTNR_RECIPE_HEADER_BYTES;
	size_t payload_offset = ALIGN((size_t)BECORE_GTNR_HEADER_COUNT *
				      BECORE_CMDQ_HEADER_BYTES,
				      BECORE_CMDQ_PAYLOAD_BYTES);
	u32 typed_count = 0;
	u32 i;

	/* An encode produces a whole program or none of one. */
	program->header_count = 0;
	if (!program->cpu || program->capacity != BECORE_GTNR_HEADER_COUNT ||
	    program->size != becore_cmdq_program_size(program->capacity) ||
	    upper_32_bits(program->dma) ||
	    upper_32_bits(program->dma + program->size - 1))
		return -EINVAL;

	memset(program->cpu, 0, program->size);
	for (i = 0; i < BECORE_GTNR_HEADER_COUNT;
	     i++, record += BECORE_GTNR_RECIPE_RECORD_BYTES) {
		const struct becore_cmdq_shape *shape = &becore_gtnr_shape[i];
		u8 *header = (u8 *)program->cpu + i * BECORE_CMDQ_HEADER_BYTES;
		u8 *payload = (u8 *)program->cpu + payload_offset +
			      i * BECORE_CMDQ_PAYLOAD_BYTES;
		dma_addr_t payload_dma = program->dma + payload_offset +
					 i * BECORE_CMDQ_PAYLOAD_BYTES;
		u32 word;

		put_unaligned_le32(shape->mode, header);
		put_unaligned_le32(lower_32_bits(payload_dma), header + 4);
		put_unaligned_le32(shape->target, header + 8);
		put_unaligned_le32(shape->type_map, header + 12);
		memcpy(payload, record + 12, BECORE_CMDQ_PAYLOAD_BYTES);

		for (word = 0; word < shape->valid_words; word++) {
			dma_addr_t dma;
			u32 reg;
			u32 value;

			if (shape->typed_mask & BIT(word)) {
				reg = shape->pair_registers[word / 2];
				if (becore_gtnr_dma_value(&becore->chain, typed_count, reg, &value))
					return -EINVAL;
				put_unaligned_le32(value, payload + word * 4);
				typed_count++;
				continue;
			}
			if (shape->generated_mask & BIT(word)) {
				reg = shape->pair_registers[word / 2];
				if (becore_gtnr_startup_off_value(reg, &value))
					return -EINVAL;
				put_unaligned_le32(value, payload + word * 4);
				continue;
			}
			if (!(shape->address_mask & BIT(word)))
				continue;
			reg = shape->pair_registers[word / 2];
			dma = becore_gtnr_address_dma(becore, reg);
			if (dma == DMA_MAPPING_ERROR || upper_32_bits(dma))
				return -EINVAL;
			put_unaligned_le32(lower_32_bits(dma), payload + word * 4);
		}
	}

	if (typed_count != BECORE_GTNR_DMA_WORD_COUNT)
		return -EINVAL;
	program->header_count = BECORE_GTNR_HEADER_COUNT;

	return 0;
}

static int becore_recipe_records_generate(u8 *record,
					  const struct becore_cmdq_shape *shapes,
					  u32 header_count)
{
	u32 i;

	static_assert(BECORE_RECIPE_RECORD_BYTES ==
		      BECORE_MCSC_RECIPE_RECORD_BYTES);
	for (i = 0; i < header_count;
	     i++, record += BECORE_RECIPE_RECORD_BYTES) {
		const struct becore_cmdq_shape *shape = &shapes[i];
		u16 used_mask;
		u16 value_mask;
		u32 word;

		if (!shape->valid_words || shape->valid_words > 16)
			return -EINVAL;
		used_mask = shape->valid_words == 16 ? U16_MAX :
			    GENMASK(shape->valid_words - 1, 0);
		if (shape->mode == 0x00090000) {
			if (shape->valid_words & 1)
				return -EINVAL;
			value_mask = used_mask & 0xaaaa;
		} else if (shape->mode == 0x00080000) {
			if (shape->address_mask || shape->typed_mask)
				return -EINVAL;
			value_mask = used_mask;
		} else if (shape->mode == 0x000b0000) {
			/*
			 * Every word of a repeated-target header names the
			 * same register, so a by-register class cannot say
			 * which value belongs where.
			 */
			if (shape->address_mask || shape->typed_mask ||
			    shape->generated_mask)
				return -EINVAL;
			value_mask = used_mask;
		} else {
			return -EINVAL;
		}
		if ((shape->address_mask | shape->typed_mask |
		     shape->generated_mask | shape->fixed_mask) != value_mask ||
		    (shape->address_mask & shape->typed_mask) ||
		    (shape->address_mask & shape->generated_mask) ||
		    (shape->address_mask & shape->fixed_mask) ||
		    (shape->typed_mask & shape->generated_mask) ||
		    (shape->typed_mask & shape->fixed_mask) ||
		    (shape->generated_mask & shape->fixed_mask))
			return -EINVAL;

		put_unaligned_le32(shape->mode, record);
		put_unaligned_le32(shape->target, record + 4);
		put_unaligned_le32(shape->type_map, record + 8);
		for (word = 0; word < shape->valid_words; word++) {
			u32 value = 0;

			if (shape->mode == 0x00090000 && !(word & 1))
				value = shape->pair_registers[word / 2];
			else if (shape->fixed_mask & BIT(word))
				value = shape->fixed_values[word];
			put_unaligned_le32(value, record + 12 + word * 4);
		}
	}

	return 0;
}

int becore_recipe_generate(struct becore_device *becore)
{
	u8 *header = becore->recipe;
	u8 *record = header + BECORE_RECIPE_HEADER_BYTES;
	int ret;

	memset(header, 0, BECORE_RECIPE_BYTES);
	put_unaligned_le32(BECORE_RECIPE_MAGIC, header);
	put_unaligned_le32(BECORE_RECIPE_VERSION, header + 4);
	put_unaligned_le32(BECORE_RECIPE_HEADER_BYTES, header + 8);
	put_unaligned_le32(BECORE_RECIPE_RECORD_BYTES, header + 12);
	put_unaligned_le32(BECORE_RGBP_HEADER_COUNT, header + 16);
	put_unaligned_le32(BECORE_YUVP_HEADER_COUNT, header + 20);
	put_unaligned_le32(BECORE_RECIPE_BYTES, header + 24);

	ret = becore_recipe_records_generate(record, becore_rgbp_shape,
					     BECORE_RGBP_HEADER_COUNT);
	if (ret)
		return ret;
	record += BECORE_RGBP_HEADER_COUNT * BECORE_RECIPE_RECORD_BYTES;
	ret = becore_recipe_records_generate(record, becore_yuvp_shape,
					     BECORE_YUVP_HEADER_COUNT);
	if (ret)
		return ret;

	becore->recipe_staged_bytes = BECORE_RECIPE_BYTES;
	becore->recipe_generation = 1;

	return 0;
}

int becore_mcsc_recipe_generate(struct becore_device *becore)
{
	u8 *header = becore->mcsc_recipe;
	int ret;

	memset(header, 0, BECORE_MCSC_RECIPE_BYTES);
	put_unaligned_le32(BECORE_MCSC_RECIPE_MAGIC, header);
	put_unaligned_le32(BECORE_MCSC_RECIPE_VERSION, header + 4);
	put_unaligned_le32(BECORE_MCSC_RECIPE_HEADER_BYTES, header + 8);
	put_unaligned_le32(BECORE_MCSC_RECIPE_RECORD_BYTES, header + 12);
	put_unaligned_le32(BECORE_MCSC_HEADER_COUNT, header + 16);
	put_unaligned_le32(BECORE_MCSC_RECIPE_BYTES, header + 20);

	ret = becore_recipe_records_generate(header +
			BECORE_MCSC_RECIPE_HEADER_BYTES,
			becore_mcsc_shape, BECORE_MCSC_HEADER_COUNT);
	if (ret)
		return ret;

	becore->mcsc_recipe_staged_bytes = BECORE_MCSC_RECIPE_BYTES;
	becore->mcsc_recipe_generation = 1;

	return 0;
}

int becore_mcsc_recipe_validate(struct becore_device *becore)
{
	const u8 *header = becore->mcsc_recipe;
	const u8 *record = header + BECORE_MCSC_RECIPE_HEADER_BYTES;
	const struct becore_yuvp_output_profile *input =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];
	u32 address_count = 0;
	u32 typed_count = 0;
	u32 generated_count = 0;
	u32 i;

	if (becore->mcsc_recipe_staged_bytes != BECORE_MCSC_RECIPE_BYTES ||
	    get_unaligned_le32(header) != BECORE_MCSC_RECIPE_MAGIC ||
	    get_unaligned_le32(header + 4) != BECORE_MCSC_RECIPE_VERSION ||
	    get_unaligned_le32(header + 8) != BECORE_MCSC_RECIPE_HEADER_BYTES ||
	    get_unaligned_le32(header + 12) != BECORE_MCSC_RECIPE_RECORD_BYTES ||
	    get_unaligned_le32(header + 16) != BECORE_MCSC_HEADER_COUNT ||
	    get_unaligned_le32(header + 20) != BECORE_MCSC_RECIPE_BYTES ||
	    get_unaligned_le32(header + 24) || get_unaligned_le32(header + 28))
		return -EINVAL;
	if (becore->output.size < becore_yuvp_output_size(input, &becore->chain) ||
	    becore->mcsc_output.size != becore_mcsc_output_size(&becore->scaled))
		return -EINVAL;

	for (i = 0; i < BECORE_MCSC_HEADER_COUNT;
	     i++, record += BECORE_MCSC_RECIPE_RECORD_BYTES) {
		const struct becore_cmdq_shape *shape = &becore_mcsc_shape[i];
		const u8 *words = record + 12;
		u16 used_mask;
		u32 word;

		if (!shape->valid_words || shape->valid_words > 16)
			return -EINVAL;
		used_mask = shape->valid_words == 16 ? U16_MAX :
			    GENMASK(shape->valid_words - 1, 0);
		if (get_unaligned_le32(record) != shape->mode ||
		    get_unaligned_le32(record + 4) != shape->target ||
		    get_unaligned_le32(record + 8) != shape->type_map ||
		    (shape->address_mask & ~used_mask) ||
		    (shape->typed_mask & ~used_mask) ||
		    (shape->generated_mask & ~used_mask) ||
		    (shape->fixed_mask & ~used_mask) ||
		    (shape->address_mask & shape->typed_mask) ||
		    (shape->address_mask & shape->generated_mask) ||
		    (shape->address_mask & shape->fixed_mask) ||
		    (shape->typed_mask & shape->generated_mask) ||
		    (shape->typed_mask & shape->fixed_mask) ||
		    (shape->generated_mask & shape->fixed_mask))
			return -EINVAL;

		for (word = 0; word < 16; word++) {
			u32 value = get_unaligned_le32(words + word * 4);

			if (word >= shape->valid_words) {
				if (value)
					return -EINVAL;
				continue;
			}
			if (shape->mode == 0x00090000 && !(word & 1)) {
				if (value != shape->pair_registers[word / 2])
					return -EINVAL;
				continue;
			}
			if ((shape->fixed_mask & BIT(word)) &&
			    value != shape->fixed_values[word])
				return -EINVAL;

			if (shape->typed_mask & BIT(word)) {
				u32 reg;

				if (shape->mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape->pair_registers[word / 2];
				if (becore_mcsc_dma_value(&becore->chain,
							  &becore->scaled,
							  typed_count, reg,
							  BECORE_MCSC_INPUT_CAPTURED_VOTF,
							  NULL, NULL))
					return -EINVAL;
				typed_count++;
			}

			if (shape->generated_mask & BIT(word)) {
				u32 reg;

				if (becore_shape_register(shape, word, &reg) ||
				    becore_generated_value(becore, BECORE_MCSC, reg,
							   NULL))
					return -EINVAL;
				generated_count++;
			}
			if (shape->address_mask & BIT(word)) {
				u32 reg;

				if (shape->mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape->pair_registers[word / 2];
				if (becore_mcsc_address_dma(becore, reg) ==
				    DMA_MAPPING_ERROR)
					return -EINVAL;
				address_count++;
			}
		}
	}

	if (address_count != 4 || typed_count != BECORE_MCSC_DMA_WORD_COUNT ||
	    generated_count != BECORE_MCSC_GENERATED_WORDS)
		return -EINVAL;

	return 0;
}

int
becore_encode_mcsc(struct becore_device *becore,
		   enum becore_mcsc_input_transport transport)
{
	struct becore_cmdq_program *program = &becore->mcsc_program;
	const u8 *record = becore->mcsc_recipe +
			   BECORE_MCSC_RECIPE_HEADER_BYTES;
	size_t payload_offset = ALIGN((size_t)BECORE_MCSC_HEADER_COUNT *
				      BECORE_CMDQ_HEADER_BYTES,
				      BECORE_CMDQ_PAYLOAD_BYTES);
	u32 typed_count = 0;
	u32 generated_count = 0;
	u32 i;

	/* An encode produces a whole program or none of one. */
	program->header_count = 0;
	if (!program->cpu || program->capacity != BECORE_MCSC_HEADER_COUNT ||
	    program->size != becore_cmdq_program_size(program->capacity) ||
	    upper_32_bits(program->dma) ||
	    upper_32_bits(program->dma + program->size - 1))
		return -EINVAL;

	memset(program->cpu, 0, program->size);
	for (i = 0; i < BECORE_MCSC_HEADER_COUNT;
	     i++, record += BECORE_MCSC_RECIPE_RECORD_BYTES) {
		const struct becore_cmdq_shape *shape = &becore_mcsc_shape[i];
		u8 *header = (u8 *)program->cpu + i * BECORE_CMDQ_HEADER_BYTES;
		u8 *payload = (u8 *)program->cpu + payload_offset +
			      i * BECORE_CMDQ_PAYLOAD_BYTES;
		dma_addr_t payload_dma = program->dma + payload_offset +
					 i * BECORE_CMDQ_PAYLOAD_BYTES;
		u32 word;

		put_unaligned_le32(shape->mode, header);
		put_unaligned_le32(lower_32_bits(payload_dma), header + 4);
		put_unaligned_le32(shape->target, header + 8);
		put_unaligned_le32(shape->type_map, header + 12);
		memcpy(payload, record + 12, BECORE_CMDQ_PAYLOAD_BYTES);

		for (word = 0; word < shape->valid_words; word++) {
			dma_addr_t dma;
			u32 reg;
			u32 value;

			if (shape->typed_mask & BIT(word)) {
				reg = shape->pair_registers[word / 2];
				if (becore_mcsc_dma_value(&becore->chain,
							  &becore->scaled,
							  typed_count, reg,
							  transport,
							  becore->votf_trs_token,
							  &value))
					return -EINVAL;
				put_unaligned_le32(value, payload + word * 4);
				typed_count++;
				continue;
			}

			if (shape->generated_mask & BIT(word)) {
				if (becore_shape_register(shape, word, &reg) ||
				    becore_generated_value(becore, BECORE_MCSC, reg,
							   &value))
					return -EINVAL;
				put_unaligned_le32(value, payload + word * 4);
				generated_count++;
				continue;
			}

			if (!(shape->address_mask & BIT(word)))
				continue;
			reg = shape->pair_registers[word / 2];
			dma = becore_mcsc_address_dma(becore, reg);
			if (dma == DMA_MAPPING_ERROR || upper_32_bits(dma))
				return -EINVAL;
			put_unaligned_le32(lower_32_bits(dma), payload + word * 4);
		}
		becore_params_apply(becore, shape, payload);
		becore_override_apply(becore, shape, payload);
	}

	if (typed_count != BECORE_MCSC_DMA_WORD_COUNT ||
	    generated_count != BECORE_MCSC_GENERATED_WORDS)
		return -EINVAL;
	program->header_count = BECORE_MCSC_HEADER_COUNT;

	return 0;
}
