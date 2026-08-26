/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * zumapro ISPFE PDMA command encoding.
 *
 * PDMA does not carry pixels.  It fetches a command program out of DRAM at each
 * frame boundary and applies it to the front end, and that program -- not any
 * CPU register write -- is what tells the receiver where to put the frame.  The
 * command grammar below was recovered from the vendor camera stack's own
 * generator, whose four builder primitives serialise to fixed shapes:
 *
 *   block select    0x400b, block_base
 *   grouped write   (command_bytes * 0x1000 - 0x3ffe), then (register, value) pairs
 *   inline burst    (payload_bytes * 0x1000 + 0x4001), register, payload
 *   indirect burst  0x10009, address_lo, address_hi, bytes, register
 *
 * The two length-bearing headers do not count the same thing: a grouped write
 * encodes its whole command including the header word, an inline burst encodes
 * only the payload after the header and target register.
 *
 * The driver builds the program itself so that every address in it is one of
 * its own allocations.  A recipe therefore never contains an address: it names
 * a buffer, and the encoder writes whatever that buffer was mapped at.
 */
#ifndef __EXYNOS_ISPFE_PDMA_H__
#define __EXYNOS_ISPFE_PDMA_H__

#include <linux/types.h>

enum ispfe_pdma_op {
	ISPFE_PDMA_SELECT_BLOCK,
	ISPFE_PDMA_GROUPED_WRITE,
	ISPFE_PDMA_INLINE_BURST,
	ISPFE_PDMA_INDIRECT_BURST,
};

/*
 * The serialised form of each.  A block select and an indirect burst are fixed
 * size; the other two encode their own length in the opcode word, which is why
 * a program can only be walked forwards and why a stale tail cannot be told
 * from a command by looking for opcode-shaped words.
 */
#define PDMA_CMD_SELECT_BLOCK		0x0000400b
#define PDMA_CMD_SELECT_BLOCK_SIZE	8
#define PDMA_CMD_INDIRECT_BURST		0x00010009
#define PDMA_CMD_INDIRECT_BURST_SIZE	20
#define PDMA_CMD_GROUPED_WRITE_HEAD	4
#define PDMA_CMD_GROUPED_WRITE(bytes)	\
	(((((u32)(bytes)) + PDMA_CMD_GROUPED_WRITE_HEAD) << 12) - 0x3ffe)
#define PDMA_CMD_INLINE_BURST_HEAD	8
#define PDMA_CMD_INLINE_BURST(bytes)	((((u32)(bytes)) << 12) + 0x4001)

/*
 * Which allocation an address in the program refers to: the capture
 * destination, one of the blocks an indirect burst streams into the front end,
 * or one of the working areas the front end writes back to.
 */
#define ISPFE_BUF_KIND_BAYER		0
#define ISPFE_BUF_KIND_INPUT		1
#define ISPFE_BUF_KIND_OUTPUT		2
#define ISPFE_BUF_KIND_BACKEND		3

#define ISPFE_BUF_KIND_SHIFT		6
#define ISPFE_BUF_INDEX_MASK		0x3f
#define ISPFE_BUF(kind, index)	\
	(((kind) << ISPFE_BUF_KIND_SHIFT) | (index))
#define ISPFE_BUF_TO_KIND(buffer)	((buffer) >> ISPFE_BUF_KIND_SHIFT)
#define ISPFE_BUF_TO_INDEX(buffer)	((buffer) & ISPFE_BUF_INDEX_MASK)

#define ISPFE_BUF_BAYER			ISPFE_BUF(ISPFE_BUF_KIND_BAYER, 0)
#define ISPFE_BUF_INPUT(n)		ISPFE_BUF(ISPFE_BUF_KIND_INPUT, (n))
#define ISPFE_BUF_OUTPUT(n)		ISPFE_BUF(ISPFE_BUF_KIND_OUTPUT, (n))
#define ISPFE_BUF_BACKEND_HEADER	ISPFE_BUF(ISPFE_BUF_KIND_BACKEND, 0)
#define ISPFE_BUF_BACKEND_IMAGE		ISPFE_BUF(ISPFE_BUF_KIND_BACKEND, 1)
#define ISPFE_BUF_TNR_PYRAMID		ISPFE_BUF(ISPFE_BUF_KIND_BACKEND, 2)

/*
 * One command.  @reg is the target register, or the block base for a block
 * select; @payload is the register/value pairs of a grouped write or the raw
 * bytes of an inline burst, and @len their size.  An indirect burst has no
 * inline payload: it streams @len bytes from @buffer instead.
 *
 * A NULL @payload on an inline burst is @len bytes of zero.  The front end
 * will not run a program that skips a stage in the line-memory chain, so a
 * stage whose output this driver does not read still has to be written -- but
 * writing it one lens's captured tuning is worse than useless, and storing
 * that tuning is what ADR 0009 forbids.  Such a command carries no bytes at
 * all and the encoder blanks the span instead.  It is not valid on a grouped
 * write, whose payload is register numbers as much as values.
 */
struct ispfe_pdma_cmd {
	u8 op;
	u8 buffer;
	u16 len;
	u32 reg;
	const u8 *payload;
};

/*
 * One 36-bit device address inside a command's payload.  @lo and @hi are byte
 * offsets into that payload, because the two halves are adjacent in an inline
 * burst but a register apart in a grouped write.  Relocations are ordered by
 * @cmd so the encoder can apply them as it emits.
 */
struct ispfe_pdma_reloc {
	u16 cmd;
	u16 lo;
	u16 hi;
	u8 buffer;
};

/* Some block-local DMA base registers carry only the low 32 address bits. */
#define ISPFE_PDMA_RELOC_NO_HIGH	0xffff

/*
 * A block an indirect burst streams into the front end.  It has to be in DRAM
 * the device can reach, so the driver copies it into its own program area at
 * @area_offset.
 */
struct ispfe_pdma_input {
	u32 area_offset;
	const u8 *data;
	size_t size;
};

#endif /* __EXYNOS_ISPFE_PDMA_H__ */
