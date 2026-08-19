// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google zumapro (Tensor G4) ISPFE camera receive complex.
 *
 * BLK_ISPFE is twelve CSIS links with nine combo D/C-PHYs behind them, the
 * ISPFE front end, and a PDMA that carries per-frame descriptors.  This is the
 * bring-up stage: it powers the block, restores the CMU and stage-2
 * protection state the power domain does not, and can put one CSIS link, its
 * PHY, a frame controller context and PDMA into a raw Bayer capture aimed at
 * its own DMA buffer.  Driven from debugfs; there is no V4L2 yet.
 *
 * Copyright 2026 Steffen Deusch
 */

#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>

#include "exynos-ispfe-pdma-program.h"
#include "exynos-ispfe-pdma-seeds.h"

/*
 * CSIS is licensed Samsung IP that mainline already drives as
 * drivers/media/platform/nxp/imx-mipi-csis.c, and zumapro's is a later version
 * of it: the register at +0x0 of every link reads a version, i.MX7D reads
 * 0x03030505 there and this reads 0x08010000.  Links are 0x10000 apart.
 *
 * The i.MX driver's field names decode the version, CMN_CTRL and the
 * per-channel config and resolution registers unchanged.  Two things moved.
 * The clock control the i.MX driver has at +0x08 is at +0x0c here -- the
 * capture writes +0x0c and never touches +0x08.  And from +0x18 upwards the
 * i.MX version has D-PHY status and control where this one has two more
 * interrupt (mask, source) pairs, which is what the twelve link banks' four
 * named interrupts each -- int0, int1, frame start, frame end -- are counted
 * against.
 */
#define CSIS_LINK_STRIDE		0x10000
#define CSIS_NUM_LINKS			12
#define CSIS_VERSION			0x0000
#define CSIS_VERSION_EXPECTED		0x08010000

#define CSIS_CMN_CTRL			0x0004
#define CSIS_CMN_CTRL_CSI_EN		BIT(0)
#define CSIS_CMN_CTRL_SW_RESET		BIT(1)
#define CSIS_CMN_CTRL_LANE_NUMBER(n)	((n) << 8)
/*
 * Bits 10 to 14 are set for every sensor in every capture, with bit 10 being
 * the i.MX driver's INTERLEAVE_MODE_DT and the rest above anything it defines.
 * Bit 21 is set only for the main camera, which is the three-lane C-PHY one,
 * so it is very likely the PHY-type selector -- but that is an inference from
 * which sensor sets it, not from a PHY register, so it stays named for what it
 * is known to correlate with.
 */
#define CSIS_CMN_CTRL_COMMON		0x5c00
#define CSIS_CMN_CTRL_CPHY_SENSOR	BIT(21)
/* The value the twelve-link reset loop writes: the top two bits and a reset. */
#define CSIS_CMN_CTRL_RESET_ALL		0x5002

#define CSIS_CLK_CTRL			0x000c
#define CSIS_CLK_CTRL_VAL		0x0000000f

/*
 * Four (mask, source) pairs in the order the device tree names their
 * interrupts.  The frame-start and frame-end sources carry one bit per
 * configured virtual channel: 0x7 for the main sensor's three, 0x3 for an
 * IMX712's two.
 */
#define CSIS_INT0_MSK			0x0010
#define CSIS_INT0_SRC			0x0014
#define CSIS_INT1_MSK			0x0018
#define CSIS_INT1_SRC			0x001c
#define CSIS_FS_MSK			0x0020
#define CSIS_FS_SRC			0x0024
#define CSIS_FE_MSK			0x0028
#define CSIS_FE_SRC			0x002c
#define CSIS_INT0_MSK_VAL		0x01fff1ff
#define CSIS_INT1_MSK_VAL		0x0000007e

#define CSIS_ISP_CONFIG_CH(n)		(0x0040 + (n) * 0x10)
#define CSIS_ISPCFG_DATAFORMAT(fmt)	((fmt) << 2)
/*
 * Pixel mode reads 3 where imx-mipi-csis knows only single, dual and quad --
 * a later-version extension -- and every sensor uses it.  Carried as a whole
 * word rather than picked apart, together with a bit at 15 that only the
 * embedded-data channel sets.
 */
#define CSIS_ISPCFG_COMMON		0x3000
#define CSIS_ISPCFG_EMBEDDED		0x8000
#define CSIS_DT_RAW10			0x2b
#define CSIS_DT_EMBEDDED8		0x12

#define CSIS_ISP_RESOL_CH(n)		(0x0044 + (n) * 0x10)
#define CSIS_ISP_RESOL(w, h)		(((h) << 16) | (w))

/*
 * Written once per link before the channels, meaning unknown, and not a
 * constant: both IMX712s get 0x7fff7fff and the main camera gets 0x80020002.
 * Like the master PHY block's +0x10, this needs splitting before a C-PHY
 * source can be described.
 */
#define CSIS_LINE_INTERVAL		0x0600
#define CSIS_LINE_INTERVAL_VAL		0x7fff7fff
#define CSIS_DMA_CLK_CTRL		0x0690

/*
 * The nine D/C-PHYs live inside the csis-link-phy window rather than behind a
 * phy driver, so the receiver programs them directly, as the vendor stack
 * does.  Bases are the reg of downstream's dcphy_m0s4s4s4s4s4_csi0@* nodes;
 * each is a 0x500 region with a common block at +0x000 and four lane blocks at
 * +0x100 upwards.  The block at +0xC1000 is described by nothing in the device
 * tree and is programmed by every sensor before its own PHY -- the "m0" of
 * m0s4s4s4s4s4, a master in front of the four-lane slaves.
 */
#define PHY_MASTER			0xc1000
#define PHY_NUM_INSTANCES		9
#define PHY_LANE(n)			(0x100 * ((n) + 1))
#define PHY_MAX_LANES			4

/*
 * isp-fe-sysreg +0x500 takes one bit per PHY instance, and it is a reset
 * release rather than a select: the main camera sets bit 0, the ultrawide adds
 * bit 1, and the front camera sets bits 5 and 6 because it drives two adjacent
 * PHY slots -- one common block each, but only the first one's lane blocks.
 */
#define SYSREG_CSIS_PHY_RESET		0x0500

/*
 * And a second gate the sysreg reset says nothing about: the nine PHYs share
 * one PMU isolation bit, and an isolated block reads back zero and swallows
 * writes.  Every dcphy node downstream carries "isolation = <0x3ebc>" against
 * the PMU syscon, and its driver bypasses isolation with bit 0 before
 * releasing the reset.  This is the same shape as the UFS PHY's isolation at
 * 0x3ec0, which is already described that way in this device tree.
 */
#define PHY_PMU_ISO_BYPASS		BIT(0)

/*
 * The names in this block follow downstream's device tree, not the shape this
 * driver first guessed at.  research/dumped.dts carries 202
 * "isp_fe-event-info@N" nodes, each naming an irq-src-reg, an irq-reset-reg
 * (the same address -- write-1-to-clear), an irq-mask-reg and an
 * irq-overflow-reg, decoded into
 * research/data/camera-ispfe-register-roles.txt.  Read against it:
 *
 *   - 0x20400 + n*0x400 is a CSIS-core *logical channel*, not a frame
 *     controller context.  Its +0x04/+0x24/+0x38/+0x4c are the abort_done,
 *     img-proc, img-err and img-mute sources, and +0x10/+0x30/+0x44/+0x58 are
 *     their masks -- three, six, fourteen and two events, which is exactly why
 *     the values are 0x7, 0x12/0x13/0x1b, 0x3fff and 0x3.  None of them is an
 *     enable, a mode, a limit or a depth, which is what they were called here
 *     for three rounds of debugging.
 *   - those four leaves aggregate into +0x2008c on the "csis-core" line
 *     (SPI 479), not the "fc" line.
 *   - the *real* frame controller is FC_CTX(n) further down, and its source is
 *     the only one on SPI 480.
 *
 * The per-context bits in the stop and start words run bayer context c at
 * BIT(c) and phase-detect context c at BIT(7 + c), so a main-camera session --
 * one of each -- shows up as 0x81 and an ultrawide one on the second context
 * as 0x2.
 */
/*
 * The init pulse is not instantaneous, and the vendor never treats it as if it
 * were: it writes FC_INIT/FC_INIT_MSK and then polls +0x2003c until it reads
 * zero -- 0x1 immediately, 0x0 about 2.4 ms later on a cold block, 1.0-1.7 ms
 * on a reconfiguration, 19 times across one session.  Nothing else in the
 * front end is touched until it clears, and START is always at least 10 ms
 * later.  This driver used to continue microseconds afterwards, issuing the
 * LMP_CTRL transitions, the PDMA wrap resets, the allocator init and the START
 * pulse into a block that was still initialising -- which register contents
 * survive, so every readback still matched, and pulses do not.
 * research/data/camera-session-2026-08-17/isp-fe-stream-configure.txt:2.
 */
#define FC_INIT_BUSY			0x2003c
#define FC_INIT_TIMEOUT_US		5000
#define FC_INIT_SETTLE_US		2000
#define FC_INIT			0x20008
#define FC_INIT_VAL		0x0000000f
#define FC_INIT_MSK			0x20010
#define FC_INIT_MSK_VAL		0x00000001
#define CORE_AGG_SRC			0x2008c
/*
 * Datapath, despite sitting between two source registers: it appears in none
 * of the 202 event-info nodes.  Downstream writes it from its event thread
 * only after the flush that the start pulse raises has been acknowledged.
 */
#define LOCH_ENABLE			0x200a0
#define LOCH_STOP			0x200a4
#define LOCH_START			0x200b0
#define FC_INIT_MASK			0x20094
#define FC_INIT_MASK_VAL		0x3fffffff
#define FC_INIT_GO			0x2011c
#define FC_INIT_GO_VAL			0x00000001
#define CORE_EBUF_SRC			0x20110

#define LOCH_COUNT		5
#define LOCH(c)			(0x20400 + (c) * 0x400)

/* Offsets within one context. */
#define LOCH_ABORT_SRC			0x04
#define LOCH_ABORT_MSK			0x10
#define LOCH_ABORT_MSK_VAL		0x00000007
/*
 * Written 1 in the microsecond before LOCH_START and 0 at teardown -- so it is
 * an arm bit after all.  This carried the opposite claim, "the only accesses
 * in a whole session are four writes of 0", and that reading came out of a
 * pid-filtered capture: the setup burst runs on a `RunnerR:P+:A:N` thread the
 * filter did not include, so every write of 1 was missing and only the
 * teardown zeroes survived.  An unfiltered capture has, for a `barghest` RAW
 * session (research/data/camera-reads-2026-08-18/):
 *
 *	615.372074  W  isp-fe +0x028c00 0x1        line memory enabled
 *	615.372075  W  isp-fe +0x020418 0x1        <- this
 *	615.372076  W  isp-fe +0x0200b0 0x1        LOCH_START
 *
 * and the same pair one microsecond apart for the phase-detect channel at
 * +0x022018 / +0x200b0 = 0x80.
 */
#define LOCH_ARM			0x18
#define LOCH_ARM_VAL			0x00000001
#define LOCH_PROC_SRC			0x24
#define LOCH_CFG2			0x2c
#define LOCH_CFG2_VAL		0x0000001b
#define LOCH_PROC_MSK			0x30
#define LOCH_PROC_MSK_CONFIGURE		0x00000012
/*
 * Not a mode at all: the downstream device tree declares +0x30 as
 * "irq-mask-reg" for the logical channel's six img-proc events
 * (isp_fe-event-info@43, research/dumped.dts:10748, src +0x24, overflow +0x28).
 * So 0x12/0x13/0x1b are event subscriptions.  0x13 is what the *YUV* preview
 * session subscribes; a RAW-to-memory session subscribes bits {0,1,3,4} =
 * 0x1b, which is what this replay wants.  The names in this block are kept
 * only because renaming them is a separate change; see ispfe.md.
 */
#define LOCH_PROC_MSK_RAW		0x0000001b
#define LOCH_PROC_MSK_DRAIN		0x00000009
#define LOCH_ERR_SRC			0x38
#define LOCH_ERR_MSK			0x44
#define LOCH_ERR_MSK_VAL		0x00003fff
#define LOCH_ERR_MSK_DRAIN		0x00000010
#define LOCH_MUTE_SRC			0x4c
#define LOCH_MUTE_MSK			0x58
#define LOCH_MUTE_MSK_VAL		0x00000003
#define LOCH_RESOL			0x74
/*
 * Two words that travel with the sensor mode and are not decoded.  They are
 * the same for a raw and a YUV stream on the same sensor, so they are not
 * about the output, and they differ between the two IMX712s -- which have the
 * same width and 48 lines of height between them -- so they are not a
 * per-sensor constant either.  Carried as measured values for one mode.
 */
#define LOCH_WORD0		0x78
#define LOCH_WORD1		0x7c
#define LOCH_ZERO			0x88
/*
 * Where a context is told which CSIS link and virtual channel feed it.  Three
 * sensors settle the layout: the main camera on link 0 writes 0x00000200, the
 * ultrawide on link 1 writes 0x01000200, and the front camera -- which is on
 * link 6 but on bayer context 0, so nothing here follows from the context
 * number -- writes 0x06000200.  The channel field is the same sensor's
 * phase-detect stream writing 0x1200 against its image stream's 0x200.
 */
#define LOCH_SOURCE			0x8c
#define LOCH_SOURCE_LINK(n)		((n) << 24)
#define LOCH_SOURCE_CHANNEL(n)	((n) << 12)
#define LOCH_SOURCE_COMMON		0x00000200
/* All four written with one, meaning unknown; there are three buffers. */
static const u32 fc_ctx_ones[] = { 0x60, 0x64, 0x6c, 0x70 };

/*
 * The line-memory pool proper: nine instances, five bayer and four
 * phase-detect.  What used to be described here as "an instance appearing in
 * three places" is really three different things -- FC_CTX(n) is the frame
 * controller, FC_CTX_BIND(n) binds one to a logical channel, and LMP_INT(n) is
 * the line memory's own interrupt group, whose +0x08 is the mask and +0x0c is
 * not an interrupt register at all.  Which one a stream gets is an allocation
 * rather than a property of the link, so it stays a debugfs control.
 */
#define LMP_CTRL			0x5000c
#define LMP_CTRL_RESET			0x00000000
#define LMP_CTRL_CONFIGURE		0x00000c10
#define LMP_CTRL_RUN			0x00000c11
#define LMP_SIGNATURE			0x50010
#define LMP_SIGNATURE_VAL		0x00dddead

#define FC_NUM_CTX			5
#define FC_CTX_BIND(n)		(0x27c00 + (n) * 0x400)
#define LMP_INT(n)		(0x50058 + (n) * 0x30)
#define FC_CTX(n)		(0x31000 + (n) * 0x2000)
#define LMP_NUM_PDAF			4
#define LMP_PDAF(m)			(0x50148 + (m) * 0x18)

#define FC_BIND_ENABLE			0x00
#define FC_BIND_LOCH		0x04
#define LMP_INT_SRC			0x00
#define LMP_INT_MSK			0x08
#define LMP_INT_MSK_VAL			0x103ffffe
#define LMP_INT_EOF			BIT(1)
#define LMP_INT_CFG			0x0c
#define LMP_INT_CFG_BAYER_VAL		0x3fffffff
#define LMP_INT_CFG_PDAF_VAL		0x000003ff
#define LMP_INT_DEBUG			0x501ac
#define FC_CTX_SRC			0x300
#define FC_CTX_OVF			0x304
#define FC_CTX_CTRL			0x308
#define FC_CTX_CTRL_VAL		0x000007fc
#define FC_CTX_MSK			0x30c
#define FC_CTX_MSK_ARM		0x000007f8
#define FC_CTX_MSK_RUN		0x000007fc

/*
 * A pair of blocks with a slot per stream, eight bytes each from +0x30, both
 * written with the same values, and a control word at +0x08 that changes with
 * the sensor: 0x6c for the main camera, 0x1a for either IMX712, 0x2 with
 * nothing streaming.  It reads like an allocator for a shared resource and it
 * is not decoded; these are the values measured for one IMX712 alone.
 */
#define LMP_ALLOC_A			0x30c00
#define LMP_ALLOC_B			0x30d00
#define LMP_ALLOC_INIT0			0x00
#define LMP_ALLOC_INIT1			0x04
#define LMP_ALLOC_INIT2			0x0c
#define LMP_ALLOC_CTRL			0x08
#define LMP_ALLOC_CTRL_IMX712		0x0000001a
#define LMP_ALLOC_CTRL_B		0x00000002
/* What the allocator words read with nothing streaming, and what stop writes. */
#define LMP_ALLOC_CTRL_IDLE		0x00000002
#define LMP_ALLOC_SLOT(s)		(0x30 + (s) * 8)
#define LMP_ALLOC_NUM_SLOTS		((LMP_ALLOC_B - LMP_ALLOC_A - 0x30) / 8)
#define LMP_ALLOC_SLOT_VAL0		0x00000040
#define LMP_ALLOC_SLOT_VAL1		0x00000020
#define LMP_ALLOC_GATE			0x30f00
#define LMP_ALLOC_GATE_VAL		0x00000001
#define LMP_ALLOC_MODE			0x30100
#define LMP_ALLOC_MODE_VAL		0x00000053

/*
 * PDMA: five contexts of a small DRAM control ring.  Each ring record points at
 * an indirect program that applies register writes at the frame boundary; the
 * image-context write to +0x205c0 inside that program carries the Bayer IOVA.
 * The ring is 1000 bytes whatever the sensor, resolution or frame rate, and its
 * head advances exactly one 16-byte record per frame.
 */
/*
 * Indexed by logical channel, not by frame-controller context -- the two are
 * not the same number and this driver had assumed they were.  The vendor's
 * ultrawide stream runs loch 1, frame-controller bank 2 and PDMA context 1,
 * and programs it at exactly these offsets in exactly this order:
 *
 *   ctx1 +0x2c = 0x1  +0x40 = 0x1f  +0x04 = base  +0x08 = 0  +0x0c = 0x3e8
 *   ctx1 +0x10 = 0    +0x00 = 0x1   +0x28 = 0x1   ... +0x10 = 0x10 13 ms later
 *
 * research/data/camera-session-2026-08-17, 1130.873169-1130.886196.  Its RAW
 * barghest stream is loch 0, bank 4, PDMA context 0, which fixes the index as
 * the channel's.
 */
#define PDMA_CTX(c)			((c) * 0x1000)
#define PDMA_ENABLE			0x00
#define PDMA_BASE_LO			0x04
#define PDMA_BASE_HI			0x08
#define PDMA_SIZE			0x0c
#define PDMA_SIZE_VAL			0x3e8
#define PDMA_HEAD			0x10
#define PDMA_INT0_SRC			0x20
#define PDMA_INT0_MSK			0x2c
#define PDMA_INT0_ARM			0x28
#define PDMA_INT1_SRC			0x34
#define PDMA_INT1_MSK			0x40
#define PDMA_INT0_MSK_VAL		0x00000001
#define PDMA_INT1_MSK_VAL		0x0000001f

#define PDMA_WRAP_CTX(c)		(0x100 + (c) * 0x100)
#define PDMA_WRAP_INIT0			0x08
#define PDMA_WRAP_INIT0_VAL		0x00000007
#define PDMA_WRAP_INIT1			0x10
#define PDMA_WRAP_INIT1_VAL		0x00000001
#define PDMA_WRAP_SRC			0x80
#define PDMA_WRAP_RESET_DONE		BIT(0)
#define PDMA_WRAP_RESET_TIMEOUT_US	5000
#define PDMA_WRAP_MSK			0x8c
#define PDMA_WRAP_MSK_VAL		0x0000001f

/*
 * One PDMA record. The address names an indirect command program and the final
 * word is its exact byte count. The physical-ultrawide RAW capture's current
 * program is a structurally complete 0x1800-byte stream; the first record's
 * older 0x1824 length again outlived the generation caught by dmabuf-peek.
 */
struct ispfe_pdma_desc {
	__le32 cmd;
	__le32 addr_lo;
	__le32 addr_hi;
	__le32 bytes;
} __packed;

#define PDMA_DESC_CMD			0x0000c003
#define PDMA_DESC_BYTES_FIRST		0x00001800
#define PDMA_DESC_BYTES			0x00001800
#define PDMA_NUM_RECORDS \
	(PDMA_SIZE_VAL / sizeof(struct ispfe_pdma_desc))

/*
 * One captured physical-ultrawide RAW program plus the eight input-only
 * buffers reached by its 0x00010009 indirect records. Each input starts on a
 * page boundary, as it did in the vendor session. The Bayer destination and
 * the thirteen private working buffers are relocated separately below.
 */
#define PDMA_PROGRAM_AREA_SIZE		0x0000e000

struct ispfe_pdma_input {
	u32 captured_iova;
	u32 area_offset;
	const u8 *data;
	size_t size;
};

#define PDMA_INPUT(_iova, _offset, _data) { \
	.captured_iova = (_iova), .area_offset = (_offset), \
	.data = (_data), .size = sizeof(_data), \
}

static const struct ispfe_pdma_input ispfe_pdma_inputs[] = {
	PDMA_INPUT(0x1d55e000, 0x2000, ispfe_pdma_input_1d55e000),
	PDMA_INPUT(0x1d5f2000, 0x3000, ispfe_pdma_input_1d5f2000),
	PDMA_INPUT(0x1d480000, 0x5000, ispfe_pdma_input_1d480000),
	PDMA_INPUT(0x1ca7d000, 0x6000, ispfe_pdma_input_1ca7d000),
	PDMA_INPUT(0x1d85c000, 0x7000, ispfe_pdma_input_1d85c000),
	PDMA_INPUT(0x1d673000, 0x8000, ispfe_pdma_input_1d673000),
	PDMA_INPUT(0x1d84b000, 0x9000, ispfe_pdma_input_1d84b000),
	PDMA_INPUT(0x1d45c000, 0xa000, ispfe_pdma_input_1d45c000),
};

struct ispfe_pdma_output {
	u32 captured_iova;
	size_t size;
	enum dma_data_direction direction;
	const u8 *seed;
	size_t seed_size;
	u8 refs;
};

#define PDMA_OUTPUT(_iova, _size, _refs) { \
	.captured_iova = (_iova), .size = (_size), \
	.direction = DMA_FROM_DEVICE, .refs = (_refs), \
}
#define PDMA_OUTPUT_RW(_iova, _size, _refs, _seed) { \
	.captured_iova = (_iova), .size = (_size), \
	.direction = DMA_BIDIRECTIONAL, .seed = (_seed), \
	.seed_size = sizeof(_seed), .refs = (_refs), \
}

/* The Bayer destination at +0x205c0 is the only userspace-visible output. */
#define PDMA_BAYER_IOVA		0x15d00000
#define PDMA_BAYER_REFS		1

/*
 * Captured allocation sizes and DMA directions for the thirteen remaining
 * destinations. The three bidirectional buffers are seeded with the captured
 * 0x2000-byte prefix; bytes beyond the captured prefix start at zero.
 */
static const struct ispfe_pdma_output ispfe_pdma_outputs[] = {
	PDMA_OUTPUT(0x1d55b000,   4096, 1),
	PDMA_OUTPUT(0x1d678000,  20480, 2),
	PDMA_OUTPUT(0x1c9e0000, 102400, 2),
	PDMA_OUTPUT(0x1d55c000,   4096, 1),
	PDMA_OUTPUT(0x1cc00000, 299008, 1),
	PDMA_OUTPUT(0x1d780000, 299008, 1),
	PDMA_OUTPUT(0x1d5fc000,  12288, 1),
	PDMA_OUTPUT(0x1d674000,  12288, 1),
	PDMA_OUTPUT(0x1d454000,  12288, 1),
	PDMA_OUTPUT(0x1d700000, 299008, 1),
	PDMA_OUTPUT_RW(0x1cb80000, 299008, 1, ispfe_pdma_seed_1cb80000),
	PDMA_OUTPUT_RW(0x1cb00000, 462848, 1, ispfe_pdma_seed_1cb00000),
	PDMA_OUTPUT_RW(0x1c980000, 311296, 1, ispfe_pdma_seed_1c980000),
};

static_assert(sizeof(ispfe_pdma_program_ultrawide) == PDMA_DESC_BYTES_FIRST);

/*
 * The vendor stack's own saved frame is width * 2 * height with no padding, so
 * the receiver writes 10-bit samples into 16-bit containers at the natural
 * stride.  Allocating exactly that is deliberate: if the hardware writes past
 * it the System MMU faults, which says more than a silent overrun would.
 */
#define ISPFE_BYTES_PER_PIXEL		2
#define ISPFE_FRAME_POISON		0xa5

/*
 * The ISPFE front end proper.  +0x50000 reads 0x1003aa in every capture taken
 * of the vendor stack, before anything writes it.
 */
#define ISPFE_VERSION			0x50000

/*
 * CMU_ISPFE.  Powering the domain does not bring this up: downstream's
 * ispfe_on[] is two PMU accesses, and the CMU is programmed separately by
 * pmucal_rae_restore_seq() replaying a save/restore list.  None of it is
 * genpd's -- that driver touches this window only on the way *down*, to clear
 * the reset-disable bit -- so the whole list is ours to replay.
 *
 * Register names are downstream's; the values are what a capture of the vendor
 * stack recorded being written, not guesses.
 */
#define CMU_PLL_CON0_MUX_NOC_USER	0x0600
#define CMU_PLL_CON0_MUX_NOC_USER_VAL	0x00000010
#define CMU_DIV_CLK_ISPFE_DCPHY		0x1800
#define CMU_DIV_CLK_ISPFE_DCPHY_VAL	0x00000003
#define CMU_DIV_CLK_ISPFE_NOCP		0x1804
#define CMU_DIV_CLK_ISPFE_NOCP_VAL	0x00000001

/*
 * QCH_CON_*, one per sub-block: the block itself, one per CSIS link, the three
 * SysMMUs and their PMMUs, SYSREG, and the bus interfaces.  Bit 1 is the
 * Q-Channel request.  CMU_ISPFE runs in automatic clock-gating mode, so a feed
 * whose consumer cannot handshake for itself stays gated until software asks --
 * the same thing CIS_CLK needed for the sensor master clocks.
 *
 * The run is contiguous except for one hole, which is absent from downstream's
 * list too rather than being an omission here.
 */
#define CMU_QCH_CON_FIRST		0x301c
#define CMU_QCH_CON_LAST		0x30d0
#define CMU_QCH_CON_HOLE		0x30c8
#define CMU_QCH_CON_ENABLE		BIT(1)

/*
 * Restored last, and the reason the block can be powered *down* again.  Bits
 * 28 and 29 are ENABLE_AUTOMATIC_CLKGATING and ENABLE_POWER_MANAGEMENT: with
 * them clear the CMU does not take part in the power handshake at all, the PMU
 * status never clears, and the power-off times out.  genpd touches this
 * register only on the way down, to clear the reset-disable in bit 24, so
 * nothing else in mainline ever sets it.
 */
#define CMU_CONTROLLER_OPTION		0x0800
#define CMU_CONTROLLER_OPTION_VAL	0xf01ff03f

/*
 * The stage-2 protection units in front of this block's System MMUs.  They
 * reset with the power domain into a state that blocks all DMA -- that is a
 * security property their own documentation states outright -- and the vendor
 * driver reopens them from its resume path whenever the hypervisor that would
 * otherwise program them is absent, by clearing protection for every VID.
 * Mainline has no such hypervisor, so this driver does the same thing, for the
 * same reason, at the same point in the power-on.
 */
#define S2MPU_PROT_EN_PER_VID_CLR	0x54
#define S2MPU_PROT_EN_ALL_VIDS		0xff
#define S2MPU_PROT_EN_PER_VID_SET	0x50

/* Last two entries of the same list, in the sysreg window rather than the CMU. */
#define SYSREG_BUS_COMPONENT_DRCG_EN	0x0104
#define SYSREG_BUS_COMPONENT_DRCG_EN_VAL 0x03ffffff
#define SYSREG_MEMCLK			0x0108
#define SYSREG_MEMCLK_VAL		0x00000000

enum ispfe_window {
	ISPFE_WIN_CORE,
	ISPFE_WIN_CSIS,
	ISPFE_WIN_SYSREG,
	ISPFE_WIN_PDMA,
	ISPFE_WIN_PDMA_WRAP,
	ISPFE_WIN_CMU,
	ISPFE_WIN_S2MPU0,
	ISPFE_WIN_S2MPU1,
	ISPFE_WIN_S2MPU2,
	ISPFE_NUM_WINDOWS,
};

#define ISPFE_NUM_S2MPU			3

static const char * const ispfe_window_names[ISPFE_NUM_WINDOWS] = {
	[ISPFE_WIN_CORE]      = "isp-fe",
	[ISPFE_WIN_CSIS]      = "csis-link-phy",
	[ISPFE_WIN_SYSREG]    = "isp-fe-sysreg",
	[ISPFE_WIN_PDMA]      = "pdma",
	[ISPFE_WIN_PDMA_WRAP] = "pdma-wrap",
	[ISPFE_WIN_CMU]       = "cmu",
	[ISPFE_WIN_S2MPU0]    = "s2mpu0",
	[ISPFE_WIN_S2MPU1]    = "s2mpu1",
	[ISPFE_WIN_S2MPU2]    = "s2mpu2",
};

/*
 * What one CSI-2 source looks like to the receiver.  Everything here is a
 * board or sensor property that a V4L2 subdev would supply; until there is
 * one, it comes from debugfs so that a wrong guess costs a write rather than a
 * rebuild.  The defaults describe tegu's ultrawide IMX712: link bank 1, the
 * PHY at +0xC1700, four D-PHY lanes, 4208x3120 RAW10 with one line of
 * embedded data on the second virtual channel.
 */
struct ispfe_source {
	u32 link;
	u32 phy;
	u32 lanes;
	u32 width;
	u32 height;
	bool cphy;

	/*
	 * Which of the receiver's own resources the stream is put on.  A frame
	 * controller context and a PDMA context of the same index went
	 * together in every session captured, so they are one control; the
	 * line-memory instance is separate because the vendor stack allocates
	 * it independently, and the slot is separate again.
	 */
	u32 loch;
	u32 fcctx;
	u32 slot;

	/* Not derived from anything: measured for one sensor mode. */
	u32 mode_word0;
	u32 mode_word1;
};

struct ispfe_device {
	struct device *dev;
	struct notifier_block genpd_nb;
	struct dentry *debugfs;
	bool state_reported;
	struct regmap *pmu;
	u32 pmu_iso_offset;
	void __iomem *base[ISPFE_NUM_WINDOWS];

	/* Serialises the debugfs controls against the streaming state. */
	struct mutex lock;
	/*
	 * src is what debugfs writes, and it is writable at any time -- there
	 * is no lock on those files and adding one would not help, because the
	 * point of them is to be changed between attempts.  active is the copy
	 * taken once the values have been validated, and it is what the
	 * interrupt handlers, the teardown and the status file read.  Without
	 * that split, raising "phy" while a stream is up indexes past the end
	 * of a nine-entry table and writes MMIO outside the mapping, from a
	 * hard interrupt.
	 */
	struct ispfe_source src;
	struct ispfe_source active;
	/*
	 * Off by default, and that is a bug rather than a policy: after the
	 * D/C-PHY isolation has been bypassed BLK_ISPFE will not power down
	 * again -- "Power domain ISPFE disable failed", then the APM watchdog --
	 * even though the block is re-isolated before genpd touches the PMU.  No
	 * frame can arrive without it, so it is a control until that conflict is
	 * understood.
	 */
	bool phy_isolation_bypass;
	/*
	 * A deliberate runtime-PM reference for separating PHY isolation from
	 * the domain power-down.  This is a bring-up control: downstream drops
	 * the shared isolation bypass while its ISPFE reference is still held,
	 * whereas GENPD_NOTIFY_PRE_OFF leaves no room to observe or settle the
	 * transition before the PMU starts the domain handshake.
	 */
	bool power_hold;
	/*
	 * Written across the twelve MIPI PHY link wrapper Q-channels from
	 * GENPD_NOTIFY_PRE_OFF; ISPFE_PREDOWN_QCH_OFF leaves them alone, which
	 * is what the vendor does and therefore the default.
	 */
	u32 predown_qch;
	/*
	 * How long to leave the block alone between the device init and the
	 * stream start; a tenth of it again before the run masks.  Sweepable
	 * because the vendor's intervals are what a HAL happened to take, not
	 * a documented requirement.
	 */
	u32 settle_us;
	/*
	 * The line-memory allocator's control word.  0x1a is what one IMX712
	 * alone was measured writing; the ultrawide with a phase-detect stream
	 * beside it wrote 0x88 and the main camera 0x6c, and nothing decodes
	 * the value, so it is a knob rather than a constant.
	 */
	u32 lmp_alloc_ctrl;
	/* Bring-up overrides for the measured opcode/length and head push. */
	u32 pdma_cmd;
	/*
	 * Override the descriptor's program address. Zero selects the relocated
	 * captured program; a deliberate unmapped IOVA remains useful for proving
	 * that PDMA issues the read and reaches the System MMU.
	 */
	u32 pdma_addr;
	u32 pdma_bytes;
	u32 pdma_bytes_first;
	bool pdma_no_kick;
	bool streaming;
	/* Advanced by debugfs and the line-memory EOF IRQ. */
	enum {
		ISPFE_SNAPSHOT_IDLE,
		ISPFE_SNAPSHOT_ARMED,
		ISPFE_SNAPSHOT_REDIRECTING,
		ISPFE_SNAPSHOT_REDIRECTED,
		ISPFE_SNAPSHOT_PUBLISHING,
		ISPFE_SNAPSHOT_READY,
	} snapshot_state;
	int link_irq;
	int core_irq;
	int fc_irq;
	int lmp_irq;
	int pdma_irq;
	/*
	 * request_irq() keeps the name it is given rather than copying it, so
	 * this cannot be a local: /proc/interrupts would print whatever took
	 * over the stack slot.
	 */
	char link_name[8];

	/* The frame, indirect program area and ring that points at the program. */
	void *frame;
	dma_addr_t frame_dma;
	size_t frame_size;
	void *spare_frame;
	dma_addr_t spare_frame_dma;
	void *program;
	dma_addr_t program_dma;
	struct ispfe_pdma_desc *ring;
	dma_addr_t ring_dma;
	struct {
		void *cpu;
		dma_addr_t dma;
	} pdma_output[ARRAY_SIZE(ispfe_pdma_outputs)];
	u32 head;

	/* Everything below is written from an interrupt. */
	atomic_t frame_start;
	atomic_t frame_end;
	atomic_t fc_events;
	atomic_t core_events;
	atomic_t lmp_events;
	atomic_t pdma_events;
	u32 int0_seen;
	u32 int1_seen;
	u32 fc_seen;
	u32 lmp_seen;
	u32 core_seen[4];
	u32 pdma_seen[3];
};

/*
 * reg of downstream's dcphy_m0s4s4s4s4s4_csi0@* nodes, minus the
 * csis-link-phy base at 0x1C040000.  Nine slots for twelve link banks, so a
 * link's PHY is not its own index: the front camera is link 6 on PHY 5.
 */
static const u32 ispfe_phy_base[PHY_NUM_INSTANCES] = {
	0xc1200, 0xc1700, 0xc2200, 0xc2700, 0xc2c00,
	0xc3100, 0xc3600, 0xc3700, 0xc3c00,
};

/*
 * How many lane blocks each instance actually has room for, from the gaps
 * between those bases: most are 0x500 apart and hold a common block plus four
 * lanes, but instances 5 and 6 are only 0x100 apart, so neither has any lane
 * block of its own before the next instance's common block starts.  Writing
 * four lanes into one of those walks into its neighbour.
 */
static u32 ispfe_phy_lanes(u32 phy)
{
	u32 next = phy + 1 < PHY_NUM_INSTANCES ? ispfe_phy_base[phy + 1]
					       : ispfe_phy_base[phy] + 0x500;

	return (next - ispfe_phy_base[phy]) / 0x100 - 1;
}

/*
 * PHY settings, transcribed from what the vendor stack writes for either
 * IMX712 -- both sensors get byte-identical sequences, which is what says this
 * is one code path with the geometry as its only argument.  The main camera
 * differs in exactly one word (the master block's +0x10 is 0x240 rather than
 * 0x200) and in using three lane blocks instead of four, so a C-PHY source
 * will need this table split; nothing here is decoded far enough to derive
 * either from the link rate.
 */
struct ispfe_reg {
	u32 off;
	u32 val;
};

static const struct ispfe_reg ispfe_phy_master[] = {
	{ 0x00, 0x00000010 }, { 0x04, 0x00000110 }, { 0x08, 0x00003223 },
	{ 0x0c, 0x00000000 }, { 0x10, 0x00000200 }, { 0x14, 0x00000000 },
};

static const struct ispfe_reg ispfe_phy_common[] = {
	{ 0x04, 0x00001334 }, { 0x08, 0x00000005 }, { 0x0c, 0x0000f0ae },
	{ 0x10, 0x00000002 }, { 0x14, 0x00008600 }, { 0x18, 0x00004000 },
	{ 0x30, 0x00000103 }, { 0x40, 0x00000001 },
};

static const struct ispfe_reg ispfe_phy_lane[] = {
	{ 0x04, 0x00001334 }, { 0x08, 0x0000000d }, { 0x0c, 0x000020ae },
	{ 0x10, 0x00000302 }, { 0x14, 0x00008600 }, { 0x18, 0x00004000 },
	{ 0x30, 0x00000137 }, { 0x34, 0x00000003 }, { 0x40, 0x00000001 },
	{ 0x50, 0x0000081a },
};

/* Enable is the last write to a PHY block, and zero is how one is reset. */
#define PHY_ENABLE_COMMON		0x3
#define PHY_ENABLE_LANE			0x1

/*
 * A lane-block word the vendor stack writes in one pass only, and it is the
 * teardown: after the lane is disabled and the link reset, before the sysreg
 * reset is asserted.  Recorded in
 * research/data/camera-nofilter-2026-08-17/ispfe-teardown-to-power-down.txt.
 */
#define PHY_LANE_STOP			0x1c
#define PHY_LANE_STOP_VAL		0x00000200

/*
 * Deliberately not devm_ioremap_resource(): two of these windows are nested.
 * "pdma" and "pdma-wrap" fall inside the 1 MB "isp-fe" range, and the "cmu"
 * window overlaps the four bytes the power domain maps for its reset-disable
 * bit, so requesting any of them exclusively fails with -EBUSY the moment the
 * enclosing one has been taken.  The device tree names them separately because
 * the hardware does; mapping them without an exclusive claim is what lets both
 * views coexist.
 */
static void __iomem *ispfe_map(struct platform_device *pdev, const char *name)
{
	struct resource *res;
	void __iomem *base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res)
		return IOMEM_ERR_PTR(-ENODEV);

	base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!base)
		return IOMEM_ERR_PTR(-ENOMEM);

	return base;
}

/*
 * Restore what the power domain does not.  Ordered as downstream restores it:
 * the two dividers first, then the NOC mux user, then the Q-channel requests
 * that let those clocks reach the sub-blocks.  Setting the mux first would put
 * the block on CLKCMU while both dividers were still at their reset ratio of
 * one, briefly overclocking the NoC and the D/C-PHY.
 *
 * This has to run on every power-on, not once at probe.  A power cycle resets
 * CMU_ISPFE, and a domain brought back up without it has no clocks for its
 * Q-channels to hand back -- the next power-down handshake then never
 * completes, and on this SoC that does not fail quietly: the PMU write times
 * out, ACPM stops answering, and the APM watchdog resets the phone.
 *
 * The values are the ones the *bootloader* leaves in CMU_ISPFE.  Nothing in
 * the vendor stack programs this CMU either: its power-domain code saves the
 * live registers on the way down and writes them back on the way up, so the
 * configuration every later power cycle restores is still the bootloader's,
 * and a first power-on after reset restores nothing at all.
 */
static void ispfe_cmu_restore(struct ispfe_device *ispfe)
{
	void __iomem *sysreg = ispfe->base[ISPFE_WIN_SYSREG];
	void __iomem *cmu = ispfe->base[ISPFE_WIN_CMU];
	u32 off;

	writel_relaxed(CMU_DIV_CLK_ISPFE_DCPHY_VAL, cmu + CMU_DIV_CLK_ISPFE_DCPHY);
	writel_relaxed(CMU_DIV_CLK_ISPFE_NOCP_VAL, cmu + CMU_DIV_CLK_ISPFE_NOCP);
	writel_relaxed(CMU_PLL_CON0_MUX_NOC_USER_VAL, cmu + CMU_PLL_CON0_MUX_NOC_USER);

	/*
	 * A flat write, not a read-modify-write: bit 0 is ENABLE, and it means
	 * "skip", so preserving it would leave the request bit ignored.
	 */
	for (off = CMU_QCH_CON_FIRST; off <= CMU_QCH_CON_LAST; off += 4) {
		if (off == CMU_QCH_CON_HOLE)
			continue;
		writel_relaxed(CMU_QCH_CON_ENABLE, cmu + off);
	}

	/*
	 * Then the three entries that close downstream's list, in its order:
	 * the controller option that lets the CMU take part in power
	 * management at all, and the two sysreg gates.
	 */
	writel_relaxed(CMU_CONTROLLER_OPTION_VAL, cmu + CMU_CONTROLLER_OPTION);
	writel_relaxed(SYSREG_BUS_COMPONENT_DRCG_EN_VAL,
		       sysreg + SYSREG_BUS_COMPONENT_DRCG_EN);
	writel_relaxed(SYSREG_MEMCLK_VAL, sysreg + SYSREG_MEMCLK);

	/*
	 * Order the posted writes above against the first access to a block
	 * they are what clocks.  Device-nGnRE only guarantees ordering within a
	 * peripheral, and CMU_ISPFE and the ISPFE front end are not the same
	 * one; a read of the CMU both flushes and provides the barrier.
	 */
	readl(cmu + CMU_CONTROLLER_OPTION);
}

/*
 * The PMU isolation shared by all nine PHYs.  While it is set the whole PHY
 * region reads back zero and takes no writes, which looks exactly like a
 * sensor streaming happily into a receiver that never sees a frame boundary --
 * the link's own registers read back correctly, because they are on the other
 * side of it.
 *
 * Isolation has to be restored before the domain goes down.  Downstream does
 * that from PHY power-off while the ISPFE runtime-PM reference is still held;
 * GENPD_NOTIFY_PRE_OFF is the final backstop here, delivered before genpd
 * touches the PMU.  Both placements have nevertheless failed the subsequent
 * power-down on hardware, so power_hold below can separate and expose the two
 * transitions instead of running straight into the watchdog.
 */
static int ispfe_phy_isolation(struct ispfe_device *ispfe, bool bypass)
{
	unsigned int val;
	int ret;

	/*
	 * Reported once, as found.  Whether the bootloader leaves the PHYs
	 * isolated is the assumption underneath both this and the power-down
	 * that follows it, and reading it costs one register.
	 */
	if (!ispfe->state_reported &&
	    !regmap_read(ispfe->pmu, ispfe->pmu_iso_offset, &val)) {
		dev_info(ispfe->dev, "PHY isolation %#010x\n", val);
		ispfe->state_reported = true;
	}

	ret = regmap_update_bits(ispfe->pmu, ispfe->pmu_iso_offset,
				 PHY_PMU_ISO_BYPASS,
				 bypass ? PHY_PMU_ISO_BYPASS : 0);
	if (ret)
		return ret;

	/*
	 * The secure monitor accepting the update only says the request was
	 * delivered.  Read the hardware state back: whether bit 0 actually
	 * follows both edges is the cheapest open measurement, and a failed
	 * bypass must stop the caller before it programs an isolated PHY.
	 */
	ret = regmap_read(ispfe->pmu, ispfe->pmu_iso_offset, &val);
	if (ret)
		return ret;

	dev_info(ispfe->dev, "PHY isolation %s, readback %#010x\n",
		 bypass ? "bypassed" : "enabled", val);

	if (!!(val & PHY_PMU_ISO_BYPASS) != bypass)
		return -EIO;

	return 0;
}

/*
 * BLK_ISPFE holds more than this device: the three ISPFE SysMMUs sit in it
 * too, and because they are this device's IOMMUs they are also its runtime-PM
 * suppliers -- so they resume, and touch their own registers, *before* our
 * ->runtime_resume would get a chance to bring the CMU up.  Hang the restore
 * off the domain instead, where genpd runs it after the PMU has powered the
 * block and before any device in it is resumed.  That is also where the vendor
 * stack does it: its power-domain enable path is the PMU sequence, the TZPC
 * restore, and then the CMU restore, in that order.
 */
static void ispfe_s2mpu_open(struct ispfe_device *ispfe)
{
	unsigned int i;

	for (i = 0; i < ISPFE_NUM_S2MPU; i++) {
		void __iomem *s2mpu = ispfe->base[ISPFE_WIN_S2MPU0 + i];

		/*
		 * Reported once, because what these read as immediately after a
		 * power cycle is the thing this is assuming.  The GPU bring-up
		 * found the bootloader leaves them open, but that only holds
		 * until something power-cycles the domain, and this driver does.
		 */
		if (!ispfe->state_reported)
			dev_info(ispfe->dev, "S2MPU%u protection %#010x\n", i,
				 readl_relaxed(s2mpu + S2MPU_PROT_EN_PER_VID_SET));

		writel_relaxed(S2MPU_PROT_EN_ALL_VIDS,
			       s2mpu + S2MPU_PROT_EN_PER_VID_CLR);
	}
}

/*
 * The twelve-link reset and the frame controller's init pulse.  This is why a
 * link can be configured without knowing what the last session left in it --
 * and the vendor stack issues the same pair on the way *down* as well, three
 * times over, so it brackets a session rather than opening one.
 */
static void ispfe_links_reset(struct ispfe_device *ispfe)
{
	void __iomem *core = ispfe->base[ISPFE_WIN_CORE];
	void __iomem *csis = ispfe->base[ISPFE_WIN_CSIS];
	ktime_t start = ktime_get();
	unsigned int i;
	u32 busy, first;
	int ret;

	for (i = 0; i < CSIS_NUM_LINKS; i++)
		writel_relaxed(CSIS_CMN_CTRL_RESET_ALL,
			       csis + i * CSIS_LINK_STRIDE + CSIS_CMN_CTRL);

	writel_relaxed(FC_INIT_VAL, core + FC_INIT);
	writel_relaxed(FC_INIT_MSK_VAL, core + FC_INIT_MSK);

	/*
	 * Report the first sample separately from the poll.  The vendor reads
	 * 0x1 here, one microsecond after the same two writes, and 0x0 about a
	 * millisecond later; this driver has been returning from the poll in
	 * 9-22 us, which is either a block that initialises two orders of
	 * magnitude faster or a busy bit that never sets.  One read says which.
	 */
	first = readl(core + FC_INIT_BUSY);

	ret = readl_poll_timeout(core + FC_INIT_BUSY, busy, !busy, 50,
				 FC_INIT_TIMEOUT_US);
	/*
	 * Then wait anyway.  The poll is not load-bearing here and cannot be:
	 * the busy bit reads 0x1 for the vendor one microsecond after these
	 * same two writes and 0x0 a millisecond later, and reads 0x0
	 * immediately for us -- so on this driver the poll returns at once and
	 * the caller runs the LMP_CTRL transitions, the allocator, the PDMA
	 * resets and the START pulse into a block that has had no settling
	 * time at all.  Register contents survive that; pulses do not.  Sleep
	 * the interval the vendor's own poll grants and leave the readback in
	 * the log as the measurement it is.
	 */
	usleep_range(FC_INIT_SETTLE_US, FC_INIT_SETTLE_US + 500);

	if (ret)
		dev_warn(ispfe->dev, "front end still initialising: %#x\n", busy);
	else
		dev_info(ispfe->dev, "front end init: busy %#x, took %lld us\n",
			 first, ktime_us_delta(ktime_get(), start));
}

/*
 * A domain that only had the shared isolation bypassed powers down again; one
 * that has streamed does not, and its ISPFE_STATUS bit never moves in the 5 ms
 * the vendor's own sequencer allows.  The PMU is not at fault -- CONFIGURATION
 * reads back as written -- and neither is the bus: all twelve links answer
 * right up to the refusal.  The block is alive and will not quiesce.
 * Re-resetting the links on the isolated side does not clear it either
 * [all HW 2026-08-18].
 *
 * That leaves the Q-channels.  Twelve of the 45 CMU_ISPFE QCH_CON registers
 * are the MIPI PHY link wrapper's, one per CSIS bank, and the wrapper is
 * exactly what has to hand its clock back for the domain to go down.  Every
 * one of them is restored to 0x2 on power-on: bit 1 is the software clock
 * request, the same "dummy Q-channel" bit CIS_CLK needed in phase 1.  A
 * forced request that the block cannot retract is the shape of this failure,
 * so predown_qch writes a chosen value across those twelve here, after
 * isolation and before genpd touches the PMU.
 *
 * Default is not to write at all, because the vendor does not: it powers down
 * with 0x2 still in place.  This is an experiment, and the sweep is 0x0 (no
 * request), 0x1 (request ignored -- bit 0 means skip) and 0x3.
 */
#define CMU_QCH_CON_PHY_WRAP_FIRST	0x3048
#define CMU_QCH_CON_PHY_WRAP_LAST	0x3074
#define ISPFE_PREDOWN_QCH_OFF		U32_MAX
#define ISPFE_SETTLE_US_DEFAULT		20000

static void ispfe_predown_quiesce(struct ispfe_device *ispfe)
{
	void __iomem *csis = ispfe->base[ISPFE_WIN_CSIS];
	void __iomem *cmu = ispfe->base[ISPFE_WIN_CMU];
	unsigned int i, live = 0;
	u32 off, qch;

	for (i = 0; i < CSIS_NUM_LINKS; i++)
		if (readl(csis + i * CSIS_LINK_STRIDE + CSIS_VERSION) ==
		    CSIS_VERSION_EXPECTED)
			live++;

	qch = READ_ONCE(ispfe->predown_qch);
	if (qch != ISPFE_PREDOWN_QCH_OFF) {
		for (off = CMU_QCH_CON_PHY_WRAP_FIRST;
		     off <= CMU_QCH_CON_PHY_WRAP_LAST; off += 4)
			writel_relaxed(qch, cmu + off);
		readl(cmu + CMU_QCH_CON_PHY_WRAP_FIRST);
	}

	dev_info(ispfe->dev, "pre-off: %u of %u links respond, wrapper qch %#x\n",
		 live, CSIS_NUM_LINKS, qch);
}

static int ispfe_genpd_notify(struct notifier_block *nb, unsigned long action,
			      void *unused)
{
	struct ispfe_device *ispfe = container_of(nb, struct ispfe_device,
						  genpd_nb);
	int ret;

	switch (action) {
	case GENPD_NOTIFY_ON:
		ispfe_cmu_restore(ispfe);
		ispfe_s2mpu_open(ispfe);
		if (ispfe->phy_isolation_bypass) {
			ret = ispfe_phy_isolation(ispfe, true);
			if (ret)
				dev_err(ispfe->dev,
					"cannot bypass PHY isolation: %d\n", ret);
		}
		break;
	case GENPD_NOTIFY_PRE_OFF:
		ret = ispfe_phy_isolation(ispfe, false);
		ispfe_predown_quiesce(ispfe);
		if (ret)
			return notifier_from_errno(ret);
		break;
	}

	return NOTIFY_OK;
}

/*
 * Still needed alongside the notifier: genpd only powers a domain on that is
 * off, and the bootloader hands BLK_ISPFE over powered.  The first resume
 * after boot therefore fires no notification at all -- harmlessly, because
 * that is exactly the state the restore reproduces, but the block is then
 * running on a configuration nothing in this kernel has written.  Writing it
 * once from here makes that case indistinguishable from every later one.
 */
static int ispfe_runtime_resume(struct device *dev)
{
	struct ispfe_device *ispfe = dev_get_drvdata(dev);

	ispfe_cmu_restore(ispfe);
	ispfe_s2mpu_open(ispfe);
	if (ispfe->phy_isolation_bypass)
		return ispfe_phy_isolation(ispfe, true);

	return 0;
}

static const struct dev_pm_ops ispfe_pm_ops = {
	RUNTIME_PM_OPS(NULL, ispfe_runtime_resume, NULL)
};

static void ispfe_genpd_notifier_remove(void *dev)
{
	dev_pm_genpd_remove_notifier(dev);
}

static void __iomem *ispfe_link(struct ispfe_device *ispfe)
{
	return ispfe->base[ISPFE_WIN_CSIS] + ispfe->active.link * CSIS_LINK_STRIDE;
}

static void __iomem *ispfe_phy(struct ispfe_device *ispfe)
{
	return ispfe->base[ISPFE_WIN_CSIS] + ispfe_phy_base[ispfe->active.phy];
}

static void ispfe_write_seq(void __iomem *base, const struct ispfe_reg *seq,
			    size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		writel_relaxed(seq[i].val, base + seq[i].off);
}

/*
 * A link's PHY, in the order the vendor stack brings it up: every block reset
 * to zero, then the shared master, then the common block and its lane blocks,
 * each closed by writing its own enable last.
 */
static void ispfe_phy_start(struct ispfe_device *ispfe)
{
	void __iomem *csis = ispfe->base[ISPFE_WIN_CSIS];
	void __iomem *phy = ispfe_phy(ispfe);
	u32 lane;

	writel_relaxed(0, phy);
	for (lane = 0; lane < ispfe->active.lanes; lane++)
		writel_relaxed(0, phy + PHY_LANE(lane));

	ispfe_write_seq(csis + PHY_MASTER, ispfe_phy_master,
			ARRAY_SIZE(ispfe_phy_master));

	ispfe_write_seq(phy, ispfe_phy_common, ARRAY_SIZE(ispfe_phy_common));
	writel_relaxed(PHY_ENABLE_COMMON, phy);

	for (lane = 0; lane < ispfe->active.lanes; lane++) {
		ispfe_write_seq(phy + PHY_LANE(lane), ispfe_phy_lane,
				ARRAY_SIZE(ispfe_phy_lane));
		writel_relaxed(PHY_ENABLE_LANE, phy + PHY_LANE(lane));
	}
}

/*
 * Released after the link is enabled, not with the rest of the PHY, which is
 * where the vendor stack puts it.  Read-modify-written because another sensor
 * may already hold its own bit.
 */
static void ispfe_phy_reset_set(struct ispfe_device *ispfe, bool released)
{
	void __iomem *reg = ispfe->base[ISPFE_WIN_SYSREG] +
			    SYSREG_CSIS_PHY_RESET;
	u32 val = readl_relaxed(reg);

	if (released)
		val |= BIT(ispfe->active.phy);
	else
		val &= ~BIT(ispfe->active.phy);

	/*
	 * Not relaxed.  This register is in the sysreg window and everything
	 * it is ordered against -- the PHY blocks and the link enable -- is in
	 * the CSIS one, and Device-nGnRE only orders accesses within a single
	 * peripheral.  The reset release has to land after the link is
	 * enabled, which is the order the vendor stack uses.
	 */
	writel(val, reg);
}

/*
 * The PHY and its link come down interleaved, and the order is the vendor
 * stack's, recovered from an unfiltered capture of a real teardown
 * (research/data/camera-nofilter-2026-08-17/ispfe-teardown-to-power-down.txt):
 *
 *   every lane block disabled, the link reset with CSI_EN still set, a second
 *   pass over the lane blocks writing PHY_LANE_STOP, and only then the sysreg
 *   reset -- with the link's interrupt masks cleared afterwards, not before.
 *
 * This driver used to assert the sysreg reset first and write the blocks into
 * a PHY that was already in it.  The common block is deliberately left alone:
 * the vendor never writes it here, and ispfe_phy_start() zeroes it anyway.
 */
static void ispfe_phy_link_stop(struct ispfe_device *ispfe)
{
	void __iomem *link = ispfe_link(ispfe);
	void __iomem *phy = ispfe_phy(ispfe);
	u32 lane, ctrl;

	for (lane = 0; lane < ispfe->active.lanes; lane++)
		writel_relaxed(0, phy + PHY_LANE(lane));

	/* How the vendor stack takes a link down: keep CSI_EN, add a reset. */
	ctrl = readl_relaxed(link + CSIS_CMN_CTRL);
	writel_relaxed(ctrl | CSIS_CMN_CTRL_SW_RESET, link + CSIS_CMN_CTRL);

	for (lane = 0; lane < ispfe->active.lanes; lane++)
		writel_relaxed(PHY_LANE_STOP_VAL,
			       phy + PHY_LANE(lane) + PHY_LANE_STOP);

	ispfe_phy_reset_set(ispfe, false);

	writel_relaxed(0, link + CSIS_INT0_MSK);
	writel_relaxed(0, link + CSIS_INT1_MSK);
	writel_relaxed(0, link + CSIS_FS_MSK);
	writel_relaxed(0, link + CSIS_FE_MSK);
}

/*
 * One CSIS link.  Two virtual channels: the image in RAW10 and the sensor's
 * one line of embedded data, which the receiver has to be told about or the
 * image channel is handed a frame that is one line too tall.
 */
static void ispfe_link_start(struct ispfe_device *ispfe)
{
	void __iomem *link = ispfe_link(ispfe);
	u32 ctrl;

	writel_relaxed(0, link + CSIS_DMA_CLK_CTRL);

	writel_relaxed(CSIS_ISPCFG_COMMON | CSIS_ISPCFG_DATAFORMAT(CSIS_DT_RAW10),
		       link + CSIS_ISP_CONFIG_CH(0));
	writel_relaxed(CSIS_ISP_RESOL(ispfe->active.width, ispfe->active.height),
		       link + CSIS_ISP_RESOL_CH(0));

	writel_relaxed(CSIS_ISPCFG_COMMON | CSIS_ISPCFG_EMBEDDED |
		       CSIS_ISPCFG_DATAFORMAT(CSIS_DT_EMBEDDED8),
		       link + CSIS_ISP_CONFIG_CH(1));
	writel_relaxed(CSIS_ISP_RESOL(ispfe->active.width, 1),
		       link + CSIS_ISP_RESOL_CH(1));

	writel_relaxed(CSIS_LINE_INTERVAL_VAL, link + CSIS_LINE_INTERVAL);
	writel_relaxed(CSIS_CLK_CTRL_VAL, link + CSIS_CLK_CTRL);

	ctrl = CSIS_CMN_CTRL_COMMON | CSIS_CMN_CTRL_CSI_EN |
	       CSIS_CMN_CTRL_LANE_NUMBER(ispfe->active.lanes - 1);
	if (ispfe->active.cphy)
		ctrl |= CSIS_CMN_CTRL_CPHY_SENSOR;
	writel_relaxed(ctrl, link + CSIS_CMN_CTRL);

	ispfe_phy_reset_set(ispfe, true);
}

static void ispfe_link_unmask(struct ispfe_device *ispfe)
{
	void __iomem *link = ispfe_link(ispfe);

	writel_relaxed(CSIS_INT0_MSK_VAL, link + CSIS_INT0_MSK);
	writel_relaxed(CSIS_INT1_MSK_VAL, link + CSIS_INT1_MSK);
	writel_relaxed(~0u, link + CSIS_FS_MSK);
	writel_relaxed(~0u, link + CSIS_FE_MSK);

	/*
	 * Flush before returning: what happens next is userspace starting the
	 * sensor over i2c, and a mutex release does not order posted writes to
	 * a device.
	 */
	readl(link + CSIS_CMN_CTRL);
}

/*
 * A source register is cleared by writing back what was read, which is what
 * the vendor stack does and what a write-1-to-clear register wants.
 */
static u32 ispfe_ack(void __iomem *link, u32 reg)
{
	u32 val = readl_relaxed(link + reg);

	writel_relaxed(val, link + reg);

	return val;
}

static irqreturn_t ispfe_link_isr(int irq, void *data)
{
	struct ispfe_device *ispfe = data;
	void __iomem *link = ispfe_link(ispfe);
	u32 int0, int1, fs, fe;

	int0 = ispfe_ack(link, CSIS_INT0_SRC);
	int1 = ispfe_ack(link, CSIS_INT1_SRC);
	fs = ispfe_ack(link, CSIS_FS_SRC);
	fe = ispfe_ack(link, CSIS_FE_SRC);

	if (fs) {
		atomic_inc(&ispfe->frame_start);

		/*
		 * Advance the descriptor head on frame *start*, not frame end.
		 * The head is a producer credit: the vendor stages one before
		 * the sensor is running at all and batches further ones ahead
		 * of the frames that will consume them.  Advancing on frame end
		 * -- which is what this did -- keeps the producer index behind
		 * the consumer for the whole session, so the front end never
		 * has a descriptor to fetch and never learns where to put a
		 * frame.  Every record names the same buffer, so the value is
		 * the same either way; the timing is the point.
		 */
		ispfe->head += sizeof(*ispfe->ring);
		if (ispfe->head >= PDMA_NUM_RECORDS * sizeof(*ispfe->ring))
			ispfe->head = 0;
		writel_relaxed(ispfe->head,
			       ispfe->base[ISPFE_WIN_PDMA] +
			       PDMA_CTX(ispfe->active.loch) + PDMA_HEAD);
	}
	if (fe)
		atomic_inc(&ispfe->frame_end);

	/*
	 * Sticky rather than counted: what a bring-up wants to know is which
	 * bits have ever been seen, and a counter per bit would say no more.
	 * These are not error registers -- the vendor stack's int1 reads 0x6
	 * on every good frame -- so establish the healthy value before reading
	 * anything into a non-zero one.
	 */
	WRITE_ONCE(ispfe->int0_seen, READ_ONCE(ispfe->int0_seen) | int0);
	WRITE_ONCE(ispfe->int1_seen, READ_ONCE(ispfe->int1_seen) | int1);

	return (int0 | int1 | fs | fe) ? IRQ_HANDLED : IRQ_NONE;
}

/*
 * Everything the front end wants before a stream: the frame controller, the
 * line-memory pool and its allocator, and PDMA's per-context enables.  Ordered
 * as the vendor stack does it, including the signature word at LMP_SIGNATURE,
 * which is not configuration.
 */
static void ispfe_device_init(struct ispfe_device *ispfe)
{
	void __iomem *core = ispfe->base[ISPFE_WIN_CORE];
	unsigned int i;

	ispfe_links_reset(ispfe);

	writel_relaxed(LMP_CTRL_RESET, core + LMP_CTRL);
	/*
	 * Note that the first of these is ISPFE_VERSION, which reads 0x1003aa
	 * before anything writes it and is what ispfe_report() prints.  The
	 * vendor stack writes zero to it here, so a report taken after a
	 * stream has run is not the same measurement as one taken at probe.
	 */
	writel_relaxed(0, core + ISPFE_VERSION);
	writel_relaxed(0, core + 0x50004);
	writel_relaxed(0, core + 0x50008);
	writel_relaxed(LMP_CTRL_CONFIGURE, core + LMP_CTRL);
	writel_relaxed(LMP_SIGNATURE_VAL, core + LMP_SIGNATURE);
	for (i = 0x50014; i <= 0x50054; i += 4)
		writel_relaxed(i == 0x50020 ? 0x00020002 : 0, core + i);

	/* A block of thresholds, written once and never varying. */
	writel_relaxed(0x00000006, core + 0x50294);
	writel_relaxed(0x00000064, core + 0x50298);
	writel_relaxed(0x000001f4, core + 0x5029c);
	writel_relaxed(0x00001388, core + 0x502a0);
	writel_relaxed(0x00000000, core + 0x502a4);
	writel_relaxed(0x0000005a, core + 0x502a8);
	writel_relaxed(0x00000190, core + 0x502ac);
	writel_relaxed(LMP_CTRL_RUN, core + LMP_CTRL);

	for (i = 0; i < FC_NUM_CTX; i++) {
		void __iomem *lmp = core + LMP_INT(i);

		writel_relaxed(0, lmp + LMP_INT_SRC);
		writel_relaxed(0, lmp + 0x04);
		writel_relaxed(0, lmp + LMP_INT_MSK);
		writel_relaxed(LMP_INT_CFG_BAYER_VAL, lmp + LMP_INT_CFG);
		writel_relaxed(0, lmp + 0x10);
	}
	for (i = 0; i < LMP_NUM_PDAF; i++) {
		void __iomem *lmp = core + LMP_PDAF(i);

		writel_relaxed(0, lmp + LMP_INT_SRC);
		writel_relaxed(0, lmp + 0x04);
		writel_relaxed(0, lmp + LMP_INT_MSK);
		writel_relaxed(LMP_INT_CFG_PDAF_VAL, lmp + LMP_INT_CFG);
		writel_relaxed(0, lmp + 0x10);
	}

	writel_relaxed(LMP_ALLOC_GATE_VAL, core + LMP_ALLOC_GATE);
	writel_relaxed(1, core + LMP_ALLOC_A + LMP_ALLOC_INIT0);
	writel_relaxed(1, core + LMP_ALLOC_A + LMP_ALLOC_INIT1);
	writel_relaxed(0, core + LMP_ALLOC_A + LMP_ALLOC_INIT2);
	writel_relaxed(1, core + LMP_ALLOC_B + LMP_ALLOC_INIT0);
	writel_relaxed(1, core + LMP_ALLOC_B + LMP_ALLOC_INIT1);
	writel_relaxed(0, core + LMP_ALLOC_B + LMP_ALLOC_INIT2);
	writel_relaxed(LMP_ALLOC_MODE_VAL, core + LMP_ALLOC_MODE);

	writel_relaxed(FC_INIT_MASK_VAL, core + FC_INIT_MASK);
	writel_relaxed(FC_INIT_GO_VAL, core + FC_INIT_GO);

	for (i = 0; i < LOCH_COUNT; i++) {
		void __iomem *wrap = ispfe->base[ISPFE_WIN_PDMA_WRAP] +
				     PDMA_WRAP_CTX(i);

		writel_relaxed(PDMA_WRAP_INIT0_VAL, wrap + PDMA_WRAP_INIT0);
		writel_relaxed(PDMA_WRAP_INIT1_VAL, wrap + PDMA_WRAP_INIT1);
	}

	/*
	 * And wait for those resets to finish, because ispfe_pdma_start() runs
	 * microseconds later and writes the ring base, its size and the enable
	 * -- straight into a context still being reset.  The reset then lands
	 * on top: with this poll absent, a dump taken mid-stream shows
	 * PDMA_ENABLE, PDMA_BASE_LO/HI and PDMA_SIZE all reading zero while
	 * PDMA_HEAD, written afterwards, holds its value.  A PDMA with no ring
	 * base can never fetch a descriptor, which is why the frame controller
	 * was never handed one and never armed.
	 *
	 * The reset-done bit is PDMA_WRAP_SRC bit 0, which the vendor has
	 * latched well before it starts a stream.
	 */
	for (i = 0; i < LOCH_COUNT; i++) {
		void __iomem *wrap = ispfe->base[ISPFE_WIN_PDMA_WRAP] +
				     PDMA_WRAP_CTX(i);
		u32 done;

		if (readl_poll_timeout(wrap + PDMA_WRAP_SRC, done,
				       done & PDMA_WRAP_RESET_DONE, 20,
				       PDMA_WRAP_RESET_TIMEOUT_US))
			dev_warn(ispfe->dev,
				 "PDMA %u reset did not complete: %#x\n", i,
				 done);
		else
			writel_relaxed(done, wrap + PDMA_WRAP_SRC);
	}
}

/*
 * Fill every record with the same descriptor rather than staging one per
 * frame, so a missed head advance costs a repeated frame rather than a lost
 * one.  That is the one place this deliberately diverges from the vendor
 * stack, and the reason is arithmetic.
 *
 * The ring is 1000 bytes and a record is 16, so a lap is 62 records and 8
 * bytes left over: the vendor's head reaches 0x3e0, whose record keeps its
 * first half there and its second half at offset 0, and the next lap then
 * starts at offset 8.  No static fill can serve that, because offset 0 would
 * have to be both the tail of the wrapped record and the head of the first
 * one.  The vendor can use it only because it rewrites each record before
 * advancing.
 *
 * So this fills the 62 records that fit whole and never lets the head reach
 * the split one.  The last eight bytes are left alone.
 */
static unsigned int
ispfe_pdma_patch_iova(void *program, u32 captured, u32 replacement)
{
	__le32 *words = program;
	unsigned int i, patched = 0;

	for (i = 0; i < PDMA_DESC_BYTES_FIRST / sizeof(*words); i++) {
		if (le32_to_cpu(words[i]) != captured)
			continue;
		words[i] = cpu_to_le32(replacement);
		patched++;
	}

	return patched;
}

static int ispfe_pdma_program_prepare(struct ispfe_device *ispfe)
{
	unsigned int i, patched;

	if (upper_32_bits(ispfe->program_dma + PDMA_PROGRAM_AREA_SIZE - 1) ||
	    upper_32_bits(ispfe->frame_dma + ispfe->frame_size - 1))
		return -ERANGE;

	memset(ispfe->program, 0, PDMA_PROGRAM_AREA_SIZE);
	memcpy(ispfe->program, ispfe_pdma_program_ultrawide,
	       sizeof(ispfe_pdma_program_ultrawide));

	patched = ispfe_pdma_patch_iova(ispfe->program, PDMA_BAYER_IOVA,
					lower_32_bits(ispfe->frame_dma));
	if (patched != PDMA_BAYER_REFS) {
		dev_err(ispfe->dev,
			"program Bayer output %#x has %u references, expected %u\n",
			PDMA_BAYER_IOVA, patched, PDMA_BAYER_REFS);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(ispfe_pdma_inputs); i++) {
		const struct ispfe_pdma_input *input = &ispfe_pdma_inputs[i];
		u32 replacement = lower_32_bits(ispfe->program_dma +
						input->area_offset);

		if (input->area_offset + input->size > PDMA_PROGRAM_AREA_SIZE)
			return -EOVERFLOW;
		memcpy((u8 *)ispfe->program + input->area_offset,
		       input->data, input->size);
		patched = ispfe_pdma_patch_iova(ispfe->program,
						input->captured_iova,
						replacement);
		if (patched != 1) {
			dev_err(ispfe->dev,
				"program input %#x has %u references, expected 1\n",
				input->captured_iova, patched);
			return -EINVAL;
		}
	}

	for (i = 0; i < ARRAY_SIZE(ispfe_pdma_outputs); i++) {
		const struct ispfe_pdma_output *output = &ispfe_pdma_outputs[i];
		dma_addr_t dma = ispfe->pdma_output[i].dma;

		if (upper_32_bits(dma + output->size - 1))
			return -ERANGE;

		patched = ispfe_pdma_patch_iova(ispfe->program,
						output->captured_iova,
						lower_32_bits(dma));
		if (patched != output->refs) {
			dev_err(ispfe->dev,
				"program output %#x has %u references, expected %u\n",
				output->captured_iova, patched, output->refs);
			return -EINVAL;
		}
	}

	return 0;
}

static void ispfe_ring_fill(struct ispfe_device *ispfe)
{
	unsigned int i;

	for (i = 0; i < PDMA_NUM_RECORDS; i++) {
		ispfe->ring[i].cmd = cpu_to_le32(READ_ONCE(ispfe->pdma_cmd));
		u32 addr = READ_ONCE(ispfe->pdma_addr);

		ispfe->ring[i].addr_lo = cpu_to_le32(
			addr ? addr : lower_32_bits(ispfe->program_dma));
		ispfe->ring[i].addr_hi = cpu_to_le32(
			addr ? 0 : upper_32_bits(ispfe->program_dma));
		/* Keep the separate first-record control for parser experiments. */
		ispfe->ring[i].bytes =
			cpu_to_le32(i ? READ_ONCE(ispfe->pdma_bytes)
				      : READ_ONCE(ispfe->pdma_bytes_first));
	}
}

static void ispfe_pdma_start(struct ispfe_device *ispfe)
{
	void __iomem *pdma = ispfe->base[ISPFE_WIN_PDMA] +
			     PDMA_CTX(ispfe->active.loch);
	void __iomem *wrap = ispfe->base[ISPFE_WIN_PDMA_WRAP] +
			     PDMA_WRAP_CTX(ispfe->active.loch);

	ispfe->head = 0;

	writel_relaxed(PDMA_INT0_MSK_VAL, pdma + PDMA_INT0_MSK);
	writel_relaxed(PDMA_INT1_MSK_VAL, pdma + PDMA_INT1_MSK);
	writel_relaxed(PDMA_WRAP_MSK_VAL, wrap + PDMA_WRAP_MSK);

	writel_relaxed(lower_32_bits(ispfe->ring_dma), pdma + PDMA_BASE_LO);
	writel_relaxed(upper_32_bits(ispfe->ring_dma), pdma + PDMA_BASE_HI);
	writel_relaxed(PDMA_SIZE_VAL, pdma + PDMA_SIZE);
	ispfe->head = 0;
	writel_relaxed(0, pdma + PDMA_HEAD);
	dma_wmb();
	writel_relaxed(1, pdma + PDMA_ENABLE);
	writel_relaxed(1, pdma + PDMA_INT0_ARM);
}

/*
 * The push that arms the frame controller, and it has to be an edge the
 * *started* context sees: the vendor writes head 0, enables PDMA, starts the
 * context, writes the run masks, and only then pushes head 0 -> 0x10.  The
 * front end fetches its first descriptor and raises its arm events 100-450 us
 * later -- measured at 255 us, 445 us and 298 us across three independent
 * armings in two captures -- tens of milliseconds before any sensor streams.
 * Staging the credit before PDMA_ENABLE, which is what this driver did, means
 * the started context never sees the transition.
 */
static void ispfe_pdma_kick(struct ispfe_device *ispfe)
{
	void __iomem *pdma = ispfe->base[ISPFE_WIN_PDMA] +
			     PDMA_CTX(ispfe->active.loch);

	if (READ_ONCE(ispfe->pdma_no_kick))
		return;

	ispfe->head = sizeof(*ispfe->ring);
	dma_wmb();
	writel_relaxed(ispfe->head, pdma + PDMA_HEAD);
	readl(pdma + PDMA_HEAD);
}

static void ispfe_pdma_stop(struct ispfe_device *ispfe)
{
	void __iomem *pdma = ispfe->base[ISPFE_WIN_PDMA] +
			     PDMA_CTX(ispfe->active.loch);
	void __iomem *wrap = ispfe->base[ISPFE_WIN_PDMA_WRAP] +
			     PDMA_WRAP_CTX(ispfe->active.loch);

	writel_relaxed(0, pdma + PDMA_ENABLE);
	writel_relaxed(0, pdma + PDMA_INT0_MSK);
	writel_relaxed(0, pdma + PDMA_INT1_MSK);
	writel_relaxed(0, wrap + PDMA_WRAP_MSK);
}

/*
 * The frame controller context, in the order the vendor stack writes it for a
 * raw stream: the four configuration words, the line-memory instance bound to
 * this context, the context started, then everything the mode describes, and
 * last of all the mode word that puts it in raw rather than configure.  That
 * last write is what makes the context run, so it is deliberately after the
 * link and its interrupts are up.
 */
static void ispfe_fc_start(struct ispfe_device *ispfe)
{
	void __iomem *core = ispfe->base[ISPFE_WIN_CORE];
	void __iomem *ctx = core + LOCH(ispfe->active.loch);
	void __iomem *bind = core + FC_CTX_BIND(ispfe->active.fcctx);
	void __iomem *bank = core + FC_CTX(ispfe->active.fcctx);
	void __iomem *alloc = core + LMP_ALLOC_A;
	unsigned int i;

	writel_relaxed(FC_CTX_CTRL_VAL, bank + FC_CTX_CTRL);
	writel_relaxed(FC_CTX_MSK_ARM, bank + FC_CTX_MSK);

	writel_relaxed(LMP_ALLOC_SLOT_VAL0,
		       alloc + LMP_ALLOC_SLOT(ispfe->active.slot));
	writel_relaxed(LMP_ALLOC_SLOT_VAL1,
		       alloc + LMP_ALLOC_SLOT(ispfe->active.slot) + 4);
	writel_relaxed(LMP_ALLOC_SLOT_VAL0,
		       core + LMP_ALLOC_B + LMP_ALLOC_SLOT(ispfe->active.slot));
	writel_relaxed(LMP_ALLOC_SLOT_VAL1,
		       core + LMP_ALLOC_B + LMP_ALLOC_SLOT(ispfe->active.slot) + 4);
	writel_relaxed(READ_ONCE(ispfe->lmp_alloc_ctrl),
		       alloc + LMP_ALLOC_CTRL);
	writel_relaxed(LMP_ALLOC_CTRL_B, core + LMP_ALLOC_B + LMP_ALLOC_CTRL);

	writel_relaxed(LMP_INT_MSK_VAL,
		       core + LMP_INT(ispfe->active.fcctx) + LMP_INT_MSK);

	writel_relaxed(LOCH_ABORT_MSK_VAL, ctx + LOCH_ABORT_MSK);
	writel_relaxed(LOCH_PROC_MSK_CONFIGURE, ctx + LOCH_PROC_MSK);
	writel_relaxed(LOCH_ERR_MSK_VAL, ctx + LOCH_ERR_MSK);
	writel_relaxed(LOCH_MUTE_MSK_VAL, ctx + LOCH_MUTE_MSK);

	/*
	 * The line memory is bound to the context here, before the context is
	 * started -- which is what the comment above has always said and what
	 * the capture shows, and is not what this function used to do: the
	 * bind had drifted to the very end, after the mode words and the
	 * source.  A context started with no line memory behind it is a
	 * candidate for accepting no frames at all.
	 */
	writel_relaxed(ispfe->active.loch, bind + FC_BIND_LOCH);
	writel_relaxed(1, bind + FC_BIND_ENABLE);

	/*
	 * Arm the channel, then start it.  The vendor writes these one
	 * microsecond apart and in this order, immediately after the line
	 * memory it just bound is enabled; see LOCH_ARM.
	 */
	writel_relaxed(LOCH_ARM_VAL, ctx + LOCH_ARM);
	writel_relaxed(BIT(ispfe->active.loch), core + LOCH_START);

	writel_relaxed(ispfe->active.mode_word0, ctx + LOCH_WORD0);
	writel_relaxed(ispfe->active.mode_word1, ctx + LOCH_WORD1);
	for (i = 0; i < ARRAY_SIZE(fc_ctx_ones); i++)
		writel_relaxed(1, ctx + fc_ctx_ones[i]);
	writel_relaxed(LOCH_SOURCE_LINK(ispfe->active.link) |
		       LOCH_SOURCE_CHANNEL(0) | LOCH_SOURCE_COMMON,
		       ctx + LOCH_SOURCE);
	writel_relaxed(0, ctx + LOCH_ZERO);
	writel_relaxed(CSIS_ISP_RESOL(ispfe->active.width, ispfe->active.height),
		       ctx + LOCH_RESOL);
	writel_relaxed(LOCH_CFG2_VAL, ctx + LOCH_CFG2);

	/*
	 * Deliberately NOT writing LOCH_ENABLE here.  It is not a mask -- it
	 * appears in none of the 202 event-info nodes -- and downstream writes
	 * it from its event thread only after the flush that the start pulse
	 * above kicks off has been acknowledged: 3 ms after start in the RAW
	 * session, 11 ms in the ultrawide preview, both still well before the
	 * sensor is told to stream.  ispfe_fc_arm() below does that.
	 */
}

/*
 * The last step of arming, and the one that has to wait: give the channel its
 * enable once the start pulse's flush has been seen and acknowledged.  A
 * timeout here is itself the measurement -- if no core event ever arrives with
 * nothing streaming, the start pulse is not producing the flush downstream
 * gets, which is a different fault from the enable landing too early.
 */
static void ispfe_fc_arm(struct ispfe_device *ispfe)
{
	void __iomem *core = ispfe->base[ISPFE_WIN_CORE];
	int waited;

	for (waited = 0; waited < 50; waited++) {
		if (atomic_read(&ispfe->core_events))
			break;
		usleep_range(200, 300);
	}

	writel_relaxed(BIT(ispfe->active.loch), core + LOCH_ENABLE);
	readl(core + LOCH_ENABLE);

	dev_info(ispfe->dev, "arm: %u core events after %d us, abort_done %#x\n",
		 atomic_read(&ispfe->core_events), waited * 250,
		 READ_ONCE(ispfe->core_seen[0]));
}

static void ispfe_fc_run(struct ispfe_device *ispfe)
{
	void __iomem *core = ispfe->base[ISPFE_WIN_CORE];

	writel_relaxed(FC_CTX_MSK_RUN,
		       core + FC_CTX(ispfe->active.fcctx) + FC_CTX_MSK);
	writel_relaxed(LOCH_PROC_MSK_RAW,
		       core + LOCH(ispfe->active.loch) + LOCH_PROC_MSK);
}

static void ispfe_fc_stop(struct ispfe_device *ispfe)
{
	void __iomem *core = ispfe->base[ISPFE_WIN_CORE];
	void __iomem *ctx = core + LOCH(ispfe->active.loch);

	writel_relaxed(0, core + LOCH_ENABLE);
	writel_relaxed(BIT(ispfe->active.loch), core + LOCH_STOP);
	writel_relaxed(0, ctx + LOCH_ARM);
	writel_relaxed(0, ctx + LOCH_ABORT_MSK);
	writel_relaxed(0, ctx + LOCH_PROC_MSK);
	writel_relaxed(0, ctx + LOCH_ERR_MSK);
	writel_relaxed(0, ctx + LOCH_MUTE_MSK);
	writel_relaxed(0, core + FC_CTX_BIND(ispfe->active.fcctx) +
			  FC_BIND_ENABLE);
	writel_relaxed(0, core + FC_CTX(ispfe->active.fcctx) +
			  FC_CTX_MSK);
	writel_relaxed(0, core + FC_CTX(ispfe->active.fcctx) +
			  FC_CTX_CTRL);

	/*
	 * And the allocator back to what it reads with nothing streaming.
	 * open.md still has "what the control word counts" open; this is not
	 * an answer to that, only the value the vendor writes on the way out.
	 */
	writel_relaxed(LMP_ALLOC_CTRL_IDLE, core + LMP_ALLOC_A + LMP_ALLOC_CTRL);
	writel_relaxed(LMP_ALLOC_CTRL_IDLE, core + LMP_ALLOC_B + LMP_ALLOC_CTRL);
	writel_relaxed(0, core + LMP_ALLOC_GATE);
}

/*
 * The logical channel's own interrupts, on the "csis-core" line (SPI 479),
 * which this driver did not previously claim at all.
 *
 * The downstream device tree is a register map for this: each isp_fe-event-info
 * node names an irq-src-reg, an irq-reset-reg (the same address -- these are
 * write-1-to-clear), an irq-mask-reg and an irq-overflow-reg.  For the loch
 * block at +0x20400 + n*0x400 the leaves are abort_done at +0x04, img proc at
 * +0x24, img err at +0x38 and img mute at +0x4c, aggregated into +0x2008c,
 * with the ebuf control at +0x20110 (research/dumped.dts:10123 onwards).
 *
 * This is why +0x24 was found holding a pending proc event with its overflow
 * bit at +0x28 also set, on a channel that had processed thirty images and had
 * clean error and mute registers: nothing was acknowledging them.
 */
/* abort_done, img proc, img err, img mute -- event-info@36/@43/@50/@57. */
static const u32 ispfe_core_leaves[] = { 0x04, 0x24, 0x38, 0x4c };
static const char * const ispfe_core_leaf_names[] = {
	"abort_done", "proc", "err", "mute"
};

static irqreturn_t ispfe_core_isr(int irq, void *data)
{
	struct ispfe_device *ispfe = data;
	void __iomem *core = ispfe->base[ISPFE_WIN_CORE];
	void __iomem *ctx = core + LOCH(ispfe->active.loch);
	u32 agg, leaf, all = 0;
	unsigned int i;

	agg = ispfe_ack(core, CORE_AGG_SRC);

	for (i = 0; i < ARRAY_SIZE(ispfe_core_leaves); i++) {
		u32 ovf;

		leaf = ispfe_ack(ctx, ispfe_core_leaves[i]);
		all |= leaf;

		/* LWIS clears a non-zero overflow the same way (lwis_interrupt.c). */
		ovf = readl_relaxed(ctx + ispfe_core_leaves[i] + 4);
		if (ovf)
			writel_relaxed(ovf, ctx + ispfe_core_leaves[i] + 4);
		WRITE_ONCE(ispfe->core_seen[i],
			   READ_ONCE(ispfe->core_seen[i]) | leaf);
	}

	ispfe_ack(core, CORE_EBUF_SRC);

	if (!agg && !all)
		return IRQ_NONE;

	atomic_inc(&ispfe->core_events);

	return IRQ_HANDLED;
}

static unsigned int
ispfe_pdma_patch_iova(void *program, u32 captured, u32 replacement);

static void ispfe_snapshot_complete(struct ispfe_device *ispfe)
{
	unsigned int patched;

	/*
	 * A head credit makes PDMA fetch this program immediately. The credit
	 * issued at frame start has therefore already captured the primary IOVA
	 * by this EOF and can apply it to one more frame. Retarget the shared
	 * program now, but do not publish the primary until the following EOF has
	 * drained that in-flight descriptor. Every later credit then sees the
	 * spare IOVA. This is the debug interface's small two-buffer version of
	 * the handoff a vb2 queue will provide.
	 */
	if (cmpxchg(&ispfe->snapshot_state, ISPFE_SNAPSHOT_ARMED,
		    ISPFE_SNAPSHOT_REDIRECTING) == ISPFE_SNAPSHOT_ARMED) {
		patched = ispfe_pdma_patch_iova(ispfe->program,
						lower_32_bits(ispfe->frame_dma),
						lower_32_bits(ispfe->spare_frame_dma));
		if (WARN_ON_ONCE(patched != PDMA_BAYER_REFS)) {
			cmpxchg(&ispfe->snapshot_state,
				ISPFE_SNAPSHOT_REDIRECTING, ISPFE_SNAPSHOT_IDLE);
			return;
		}
		dma_wmb();
		cmpxchg(&ispfe->snapshot_state, ISPFE_SNAPSHOT_REDIRECTING,
			ISPFE_SNAPSHOT_REDIRECTED);
		return;
	}

	if (cmpxchg(&ispfe->snapshot_state, ISPFE_SNAPSHOT_REDIRECTED,
		    ISPFE_SNAPSHOT_PUBLISHING) != ISPFE_SNAPSHOT_REDIRECTED)
		return;

	dma_rmb();
	cmpxchg(&ispfe->snapshot_state, ISPFE_SNAPSHOT_PUBLISHING,
		ISPFE_SNAPSHOT_READY);
}

static irqreturn_t ispfe_fc_isr(int irq, void *data)
{
	struct ispfe_device *ispfe = data;
	void __iomem *core = ispfe->base[ISPFE_WIN_CORE];
	void __iomem *bank = core + FC_CTX(ispfe->active.fcctx);
	u32 src, ovf;

	/*
	 * The frame controller's own source, not the CSIS-core aggregate at
	 * CORE_AGG_SRC: that one is on the csis-core line and belongs to
	 * ispfe_core_isr().  Reading it here made this handler return
	 * IRQ_NONE whenever the aggregate happened to be clear, and ack
	 * another line's registers when it was not.
	 */
	src = readl_relaxed(bank + FC_CTX_SRC);
	if (!src)
		return IRQ_NONE;

	writel_relaxed(src, bank + FC_CTX_SRC);

	ovf = readl_relaxed(bank + FC_CTX_OVF);
	if (ovf)
		writel_relaxed(ovf, bank + FC_CTX_OVF);

	WRITE_ONCE(ispfe->fc_seen, READ_ONCE(ispfe->fc_seen) | src);
	atomic_inc(&ispfe->fc_events);
	return IRQ_HANDLED;
}

/*
 * All five bayer line-memory instances share one GIC line, and device init
 * unmasks every one of them, so acknowledging only the configured instance
 * would leave a level-triggered line asserted for ever.  That is an interrupt
 * storm the kernel's spurious detector cannot see, because the handler keeps
 * returning IRQ_HANDLED for it.  Acknowledge all of them, which is what the
 * vendor stack's handler does.
 */
static irqreturn_t ispfe_lmp_isr(int irq, void *data)
{
	struct ispfe_device *ispfe = data;
	void __iomem *core = ispfe->base[ISPFE_WIN_CORE];
	bool target_done = false;
	unsigned int i;
	u32 seen;

	seen = readl_relaxed(core + LMP_INT_DEBUG);
	writel_relaxed(seen, core + LMP_INT_DEBUG);

	for (i = 0; i < FC_NUM_CTX; i++) {
		void __iomem *lmp = core + LMP_INT(i);
		void __iomem *bank = core + FC_CTX(i);
		u32 src = readl_relaxed(lmp + LMP_INT_SRC);
		u32 bsrc = readl_relaxed(bank + FC_CTX_SRC);

		if (i == ispfe->active.fcctx) {
			WRITE_ONCE(ispfe->lmp_seen,
				   READ_ONCE(ispfe->lmp_seen) | src);
			if (src & LMP_INT_EOF)
				target_done = true;
		}
		seen |= src | bsrc;
		writel_relaxed(src, lmp + LMP_INT_SRC);
		writel_relaxed(bsrc, bank + FC_CTX_SRC);
	}

	if (!seen)
		return IRQ_NONE;

	atomic_inc(&ispfe->lmp_events);

	/*
	 * The debug capture used to tear the receiver down at an arbitrary point
	 * in the next frame. That left one moving zero-filled band in an otherwise
	 * complete image. Once userspace asks for a snapshot, retarget future
	 * frames to the spare buffer after the configured line-memory context
	 * reports EOF. This avoids changing live hardware at EOF: gating LOCH there
	 * made its teardown path zero part of the buffer it had just completed.
	 */
	if (target_done)
		ispfe_snapshot_complete(ispfe);

	return IRQ_HANDLED;
}

/* All five PDMA contexts share one line too, for the same reason. */
static irqreturn_t ispfe_pdma_isr(int irq, void *data)
{
	struct ispfe_device *ispfe = data;
	unsigned int i;
	u32 seen = 0;

	for (i = 0; i < LOCH_COUNT; i++) {
		void __iomem *pdma = ispfe->base[ISPFE_WIN_PDMA] + PDMA_CTX(i);
		void __iomem *wrap = ispfe->base[ISPFE_WIN_PDMA_WRAP] +
				     PDMA_WRAP_CTX(i);
		u32 int0 = readl_relaxed(pdma + PDMA_INT0_SRC);
		u32 int1 = readl_relaxed(pdma + PDMA_INT1_SRC);
		u32 wr = readl_relaxed(wrap + PDMA_WRAP_SRC);

		seen |= int0 | int1 | wr;
		/*
		 * Which bits, not just how many: int1 is five error bits and
		 * the wrap source carries the reset-done the vendor has
		 * latched by stream start.  A descriptor fetch that faults and
		 * one that completes both show up here as "an interrupt".
		 */
		/*
		 * PDMA is indexed by the logical channel, not the frame
		 * controller context -- ispfe_pdma_start() uses PDMA_CTX(loch)
		 * -- so recording against fcctx reported a context this stream
		 * never programmed.  That is why a run could show one PDMA
		 * event with all three sticky words reading zero.
		 */
		if (i == ispfe->active.loch) {
			WRITE_ONCE(ispfe->pdma_seen[0],
				   READ_ONCE(ispfe->pdma_seen[0]) | int0);
			WRITE_ONCE(ispfe->pdma_seen[1],
				   READ_ONCE(ispfe->pdma_seen[1]) | int1);
			WRITE_ONCE(ispfe->pdma_seen[2],
				   READ_ONCE(ispfe->pdma_seen[2]) | wr);
		}
		writel_relaxed(int0, pdma + PDMA_INT0_SRC);
		writel_relaxed(int1, pdma + PDMA_INT1_SRC);
		writel_relaxed(wr, wrap + PDMA_WRAP_SRC);
	}

	if (!seen)
		return IRQ_NONE;

	atomic_inc(&ispfe->pdma_events);

	return IRQ_HANDLED;
}

static void ispfe_buffers_free(struct ispfe_device *ispfe)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ispfe_pdma_outputs); i++) {
		if (!ispfe->pdma_output[i].cpu)
			continue;
		dma_free_coherent(ispfe->dev, ispfe_pdma_outputs[i].size,
				  ispfe->pdma_output[i].cpu,
				  ispfe->pdma_output[i].dma);
		ispfe->pdma_output[i].cpu = NULL;
	}
	if (ispfe->program) {
		dma_free_coherent(ispfe->dev, PDMA_PROGRAM_AREA_SIZE,
				  ispfe->program, ispfe->program_dma);
		ispfe->program = NULL;
	}
	if (ispfe->ring) {
		dma_free_coherent(ispfe->dev, PAGE_SIZE, ispfe->ring,
				  ispfe->ring_dma);
		ispfe->ring = NULL;
	}
	if (ispfe->spare_frame) {
		dma_free_coherent(ispfe->dev, ispfe->frame_size,
				  ispfe->spare_frame, ispfe->spare_frame_dma);
		ispfe->spare_frame = NULL;
	}
	if (ispfe->frame) {
		dma_free_coherent(ispfe->dev, ispfe->frame_size, ispfe->frame,
				  ispfe->frame_dma);
		ispfe->frame = NULL;
		ispfe->frame_size = 0;
	}
}

static void ispfe_pdma_outputs_reset(struct ispfe_device *ispfe)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ispfe_pdma_outputs); i++) {
		const struct ispfe_pdma_output *output = &ispfe_pdma_outputs[i];

		memset(ispfe->pdma_output[i].cpu, 0, output->size);
		if (output->direction == DMA_BIDIRECTIONAL) {
			WARN_ON_ONCE(output->seed_size > output->size);
			memcpy(ispfe->pdma_output[i].cpu, output->seed,
			       min(output->seed_size, output->size));
		}
	}
}

static bool ispfe_buffers_ready(struct ispfe_device *ispfe)
{
	unsigned int i;

	if (!ispfe->frame || !ispfe->spare_frame || !ispfe->ring ||
	    !ispfe->program)
		return false;
	for (i = 0; i < ARRAY_SIZE(ispfe_pdma_outputs); i++)
		if (!ispfe->pdma_output[i].cpu)
			return false;

	return true;
}

/*
 * The frame buffer outlives the stream: a capture is read out after it has
 * been stopped, so freeing it on stop would throw away the only thing the
 * whole exercise produces.  It is reallocated when the geometry changes and
 * released when the driver goes away.
 */
static int ispfe_buffers_alloc(struct ispfe_device *ispfe)
{
	size_t size = array3_size(ispfe->active.width, ispfe->active.height,
				  ISPFE_BYTES_PER_PIXEL);
	unsigned int i;
	int ret;

	if (size == SIZE_MAX)
		return -EOVERFLOW;

	if (ispfe_buffers_ready(ispfe) && ispfe->frame_size == size) {
		memset(ispfe->frame, ISPFE_FRAME_POISON, size);
		memset(ispfe->spare_frame, ISPFE_FRAME_POISON, size);
		ispfe_pdma_outputs_reset(ispfe);
		ret = ispfe_pdma_program_prepare(ispfe);
		if (ret)
			return ret;
		ispfe_ring_fill(ispfe);
		return 0;
	}

	ispfe_buffers_free(ispfe);

	ispfe->frame = dma_alloc_coherent(ispfe->dev, size, &ispfe->frame_dma,
					  GFP_KERNEL);
	if (!ispfe->frame)
		return -ENOMEM;
	ispfe->frame_size = size;
	ispfe->spare_frame = dma_alloc_coherent(ispfe->dev, size,
					       &ispfe->spare_frame_dma,
					       GFP_KERNEL);
	if (!ispfe->spare_frame) {
		ispfe_buffers_free(ispfe);
		return -ENOMEM;
	}

	ispfe->ring = dma_alloc_coherent(ispfe->dev, PAGE_SIZE,
					 &ispfe->ring_dma, GFP_KERNEL);
	if (!ispfe->ring) {
		ispfe_buffers_free(ispfe);
		return -ENOMEM;
	}
	ispfe->program = dma_alloc_coherent(ispfe->dev,
					    PDMA_PROGRAM_AREA_SIZE,
					    &ispfe->program_dma, GFP_KERNEL);
	if (!ispfe->program) {
		ispfe_buffers_free(ispfe);
		return -ENOMEM;
	}
	for (i = 0; i < ARRAY_SIZE(ispfe_pdma_outputs); i++) {
		ispfe->pdma_output[i].cpu = dma_alloc_coherent(
			ispfe->dev, ispfe_pdma_outputs[i].size,
			&ispfe->pdma_output[i].dma, GFP_KERNEL);
		if (!ispfe->pdma_output[i].cpu) {
			ispfe_buffers_free(ispfe);
			return -ENOMEM;
		}
	}

	/*
	 * Poisoned rather than zeroed, so that a frame that never arrived is
	 * distinguishable from one that arrived black.
	 */
	memset(ispfe->frame, ISPFE_FRAME_POISON, size);
	memset(ispfe->spare_frame, ISPFE_FRAME_POISON, size);
	ispfe_pdma_outputs_reset(ispfe);
	ret = ispfe_pdma_program_prepare(ispfe);
	if (ret) {
		ispfe_buffers_free(ispfe);
		return ret;
	}
	ispfe_ring_fill(ispfe);

	return 0;
}

static int ispfe_request_irqs(struct ispfe_device *ispfe)
{
	struct platform_device *pdev = to_platform_device(ispfe->dev);
	int ret;

	ret = request_irq(ispfe->link_irq, ispfe_link_isr, 0, ispfe->link_name,
			  ispfe);
	if (ret)
		return ret;

	ispfe->core_irq = platform_get_irq_byname(pdev, "csis-core");
	ispfe->fc_irq = platform_get_irq_byname(pdev, "fc");
	ispfe->lmp_irq = platform_get_irq_byname(pdev, "lmp-bayer");
	ispfe->pdma_irq = platform_get_irq_byname(pdev, "pdma");
	if (ispfe->core_irq < 0 || ispfe->fc_irq < 0 || ispfe->lmp_irq < 0 ||
	    ispfe->pdma_irq < 0) {
		ret = -ENODEV;
		goto err_link;
	}

	ret = request_irq(ispfe->core_irq, ispfe_core_isr, 0, "ispfe-core",
			  ispfe);
	if (ret)
		goto err_link;

	ret = request_irq(ispfe->fc_irq, ispfe_fc_isr, 0, "ispfe-fc", ispfe);
	if (ret)
		goto err_core;

	ret = request_irq(ispfe->lmp_irq, ispfe_lmp_isr, 0, "ispfe-lmp", ispfe);
	if (ret)
		goto err_fc;

	ret = request_irq(ispfe->pdma_irq, ispfe_pdma_isr, 0, "ispfe-pdma",
			  ispfe);
	if (ret)
		goto err_lmp;

	return 0;

err_lmp:
	free_irq(ispfe->lmp_irq, ispfe);
err_fc:
	free_irq(ispfe->fc_irq, ispfe);
err_core:
	free_irq(ispfe->core_irq, ispfe);
err_link:
	free_irq(ispfe->link_irq, ispfe);
	return ret;
}

static void ispfe_free_irqs(struct ispfe_device *ispfe)
{
	free_irq(ispfe->pdma_irq, ispfe);
	free_irq(ispfe->lmp_irq, ispfe);
	free_irq(ispfe->fc_irq, ispfe);
	free_irq(ispfe->core_irq, ispfe);
	free_irq(ispfe->link_irq, ispfe);
}

/*
 * Bring one raw stream up, in the order the vendor stack does: the block, the
 * line-memory pool, PDMA, the frame controller context, the PHY, the link, the
 * link's interrupts, and only then the mode word that makes the context run.
 *
 * The sensor is not started here.  Nothing in this driver knows how to, and
 * the vendor stack has the same ordering -- the receiver is configured and
 * armed, and the sensor's own stream-on comes last.
 */
static int ispfe_start(struct ispfe_device *ispfe)
{
	struct platform_device *pdev = to_platform_device(ispfe->dev);
	int ret;

	/*
	 * Validate, then snapshot.  Every one of these ends up as an array
	 * index, a shift or an MMIO offset, and they stay writable while a
	 * stream is up -- so checking them and then reading them again later
	 * would leave the check meaning nothing.
	 */
	if (ispfe->src.link >= CSIS_NUM_LINKS ||
	    ispfe->src.phy >= PHY_NUM_INSTANCES ||
	    ispfe->src.lanes < 1 ||
	    ispfe->src.lanes > ispfe_phy_lanes(ispfe->src.phy) ||
	    ispfe->src.loch >= LOCH_COUNT ||
	    ispfe->src.fcctx >= FC_NUM_CTX ||
	    ispfe->src.slot >= LMP_ALLOC_NUM_SLOTS ||
	    ispfe->src.width - 1 >= U16_MAX || ispfe->src.height - 1 >= U16_MAX)
		return -EINVAL;

	ispfe->active = ispfe->src;

	snprintf(ispfe->link_name, sizeof(ispfe->link_name), "csis%u",
		 ispfe->active.link);
	ret = platform_get_irq_byname(pdev, ispfe->link_name);
	if (ret < 0)
		return ret;
	ispfe->link_irq = ret;

	ret = ispfe_buffers_alloc(ispfe);
	if (ret)
		return ret;

	atomic_set(&ispfe->frame_start, 0);
	atomic_set(&ispfe->frame_end, 0);
	atomic_set(&ispfe->fc_events, 0);
	atomic_set(&ispfe->core_events, 0);
	atomic_set(&ispfe->lmp_events, 0);
	atomic_set(&ispfe->pdma_events, 0);
	WRITE_ONCE(ispfe->snapshot_state, ISPFE_SNAPSHOT_IDLE);
	ispfe->int0_seen = 0;
	ispfe->int1_seen = 0;
	ispfe->fc_seen = 0;
	ispfe->lmp_seen = 0;
	memset(ispfe->core_seen, 0, sizeof(ispfe->core_seen));
	memset(ispfe->pdma_seen, 0, sizeof(ispfe->pdma_seen));

	ret = pm_runtime_resume_and_get(ispfe->dev);
	if (ret)
		return ret;

	/*
	 * Requested before anything is unmasked, and after the domain is up:
	 * the handlers touch the block's registers.
	 */
	ret = ispfe_request_irqs(ispfe);
	if (ret)
		goto err_put;

	ispfe_device_init(ispfe);

	/*
	 * The vendor leaves 110 ms between finishing the device init and
	 * touching the frame controller, and 460 us to 2 ms between the START
	 * pulse and the run masks.  This driver had neither: the whole
	 * sequence ran in 1.1 ms with nothing between any two steps, which is
	 * indistinguishable from correct in a register readback and is not
	 * indistinguishable to a block that has to sequence something.  Both
	 * intervals are one knob, because nothing yet separates them.
	 */
	fsleep(READ_ONCE(ispfe->settle_us));
	ispfe_pdma_start(ispfe);
	ispfe_fc_start(ispfe);
	ispfe_phy_start(ispfe);
	ispfe_link_start(ispfe);
	ispfe_link_unmask(ispfe);
	fsleep(READ_ONCE(ispfe->settle_us) / 10);
	ispfe_fc_run(ispfe);
	ispfe_pdma_kick(ispfe);
	ispfe_fc_arm(ispfe);

	ispfe->streaming = true;

	return 0;

err_put:
	pm_runtime_put(ispfe->dev);
	return ret;
}

/*
 * Ordered as the vendor stack tears a session down, which is a good deal more
 * than undoing ispfe_start(): the context and its line memory go first, then
 * the PHY and link together, then PDMA, and last the same twelve-link reset
 * and frame-controller init pulse that opened the session.  Downstream leaves
 * about 2 ms between that last pulse and dropping the power domain, and it
 * powers BLK_ISPFE off at every camera close.
 */
static void ispfe_stop(struct ispfe_device *ispfe)
{
	unsigned int state = READ_ONCE(ispfe->snapshot_state);

	/*
	 * A ready frame deliberately survives stop: debugfs reads it after the
	 * hardware is quiescent. Let an IRQ that already owns publication finish,
	 * but atomically cancel every earlier state before synchronizing the line.
	 */
	while (state != ISPFE_SNAPSHOT_IDLE &&
	       state != ISPFE_SNAPSHOT_PUBLISHING &&
	       state != ISPFE_SNAPSHOT_READY) {
		unsigned int old;

		old = cmpxchg(&ispfe->snapshot_state, state,
			      ISPFE_SNAPSHOT_IDLE);
		if (old == state)
			break;
		state = old;
	}
	synchronize_irq(ispfe->lmp_irq);
	ispfe_fc_stop(ispfe);
	ispfe_phy_link_stop(ispfe);
	ispfe_pdma_stop(ispfe);
	ispfe_links_reset(ispfe);
	ispfe_free_irqs(ispfe);
	ispfe->streaming = false;
	pm_runtime_put(ispfe->dev);
}

static int ispfe_enable_set(void *data, u64 val)
{
	struct ispfe_device *ispfe = data;
	int ret = 0;

	guard(mutex)(&ispfe->lock);

	if (!!val == ispfe->streaming)
		return 0;
	if (val)
		ret = ispfe_start(ispfe);
	else
		ispfe_stop(ispfe);

	return ret;
}

static int ispfe_enable_get(void *data, u64 *val)
{
	struct ispfe_device *ispfe = data;

	guard(mutex)(&ispfe->lock);
	*val = ispfe->streaming;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ispfe_enable_fops, ispfe_enable_get, ispfe_enable_set,
			 "%llu\n");

/*
 * Writing one arms a one-shot handoff. Reading returns one only after the
 * line-memory pipeline reported EOF and future frames were redirected to the
 * spare Bayer buffer.
 */
static int ispfe_snapshot_set(void *data, u64 val)
{
	struct ispfe_device *ispfe = data;

	if (val > 1)
		return -EINVAL;

	guard(mutex)(&ispfe->lock);

	if (!ispfe->streaming)
		return -EPIPE;
	if (READ_ONCE(ispfe->snapshot_state) == ISPFE_SNAPSHOT_READY)
		return val ? 0 : -EBUSY;

	if (val) {
		if (cmpxchg(&ispfe->snapshot_state, ISPFE_SNAPSHOT_IDLE,
			    ISPFE_SNAPSHOT_ARMED) != ISPFE_SNAPSHOT_IDLE)
			return -EBUSY;
	} else {
		if (cmpxchg(&ispfe->snapshot_state, ISPFE_SNAPSHOT_ARMED,
			    ISPFE_SNAPSHOT_IDLE) != ISPFE_SNAPSHOT_ARMED &&
		    READ_ONCE(ispfe->snapshot_state) != ISPFE_SNAPSHOT_IDLE)
			return -EBUSY;
	}

	return 0;
}

static int ispfe_snapshot_get(void *data, u64 *val)
{
	struct ispfe_device *ispfe = data;

	guard(mutex)(&ispfe->lock);
	*val = smp_load_acquire(&ispfe->snapshot_state) ==
		ISPFE_SNAPSHOT_READY;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ispfe_snapshot_fops, ispfe_snapshot_get,
			 ispfe_snapshot_set, "%llu\n");

static int ispfe_phy_isolation_bypass_set(void *data, u64 val)
{
	struct ispfe_device *ispfe = data;
	bool bypass;
	int ret;

	if (val > 1)
		return -EINVAL;
	bypass = val;

	guard(mutex)(&ispfe->lock);

	if (bypass == ispfe->phy_isolation_bypass)
		return 0;
	if (ispfe->streaming)
		return -EBUSY;
	if (bypass && !ispfe->power_hold)
		return -EBUSY;

	/*
	 * The hold makes the domain known-live and separates this edge from its
	 * power-down.  Require it before bypassing, then keep it until a
	 * successful re-isolation has cleared the requested state; this makes
	 * the known-fatal unheld sequence unrepresentable through debugfs.
	 */
	ret = ispfe_phy_isolation(ispfe, bypass);
	if (ret)
		return ret;

	ispfe->phy_isolation_bypass = bypass;

	return 0;
}

static int ispfe_phy_isolation_bypass_get(void *data, u64 *val)
{
	struct ispfe_device *ispfe = data;

	guard(mutex)(&ispfe->lock);
	*val = ispfe->phy_isolation_bypass;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ispfe_phy_isolation_bypass_fops,
			 ispfe_phy_isolation_bypass_get,
			 ispfe_phy_isolation_bypass_set, "%llu\n");

static int ispfe_power_hold_set(void *data, u64 val)
{
	struct ispfe_device *ispfe = data;
	int ret;

	if (val > 1)
		return -EINVAL;

	guard(mutex)(&ispfe->lock);

	if (!!val == ispfe->power_hold)
		return 0;
	if (ispfe->streaming)
		return -EBUSY;

	if (val) {
		ret = pm_runtime_resume_and_get(ispfe->dev);
		if (ret)
			return ret;
		ispfe->power_hold = true;
		return 0;
	}

	/* Never drop the diagnostic hold while the requested state is unsafe. */
	if (ispfe->phy_isolation_bypass)
		return -EBUSY;

	ispfe->power_hold = false;
	ret = pm_runtime_put_sync(ispfe->dev);

	return ret < 0 ? ret : 0;
}

static int ispfe_power_hold_get(void *data, u64 *val)
{
	struct ispfe_device *ispfe = data;

	guard(mutex)(&ispfe->lock);
	*val = ispfe->power_hold;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ispfe_power_hold_fops, ispfe_power_hold_get,
			 ispfe_power_hold_set, "%llu\n");

static int ispfe_predown_qch_set(void *data, u64 val)
{
	struct ispfe_device *ispfe = data;

	if (val != ISPFE_PREDOWN_QCH_OFF && val > U32_MAX)
		return -EINVAL;

	WRITE_ONCE(ispfe->predown_qch, val);

	return 0;
}

static int ispfe_predown_qch_get(void *data, u64 *val)
{
	struct ispfe_device *ispfe = data;

	*val = READ_ONCE(ispfe->predown_qch);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ispfe_predown_qch_fops, ispfe_predown_qch_get,
			 ispfe_predown_qch_set, "%#llx\n");

/*
 * A readback of everything a stream configures, for diffing against the vendor
 * capture.  Three transcription bugs were found by reading that capture and
 * none of them by reading the driver, because a register that is written and
 * never read back cannot be checked -- and a value the hardware rejects or
 * clears looks identical to one it accepted.
 *
 * Gated on streaming: BLK_ISPFE is powered down the rest of the time and a
 * read into an unpowered domain is an SError, not an error return.
 */
static int ispfe_regs_show(struct seq_file *s, void *unused)
{
	struct ispfe_device *ispfe = s->private;
	void __iomem *core, *ctx, *link;
	unsigned int i;

	guard(mutex)(&ispfe->lock);

	if (!ispfe->streaming) {
		seq_puts(s, "not streaming\n");
		return 0;
	}

	core = ispfe->base[ISPFE_WIN_CORE];
	ctx = core + LOCH(ispfe->active.loch);
	link = ispfe_link(ispfe);

	seq_printf(s, "# link %u ctx %u lmp %u slot %u\n", ispfe->active.link,
		   ispfe->active.loch, ispfe->active.fcctx, ispfe->active.slot);

	for (i = 0; i <= 0x2c; i += 4)
		seq_printf(s, "link   +0x%04x %#010x\n", i,
			   readl_relaxed(link + i));
	for (i = 0x40; i <= 0x54; i += 4)
		seq_printf(s, "link   +0x%04x %#010x\n", i,
			   readl_relaxed(link + i));

	for (i = 0; i <= 0x11c; i += 4)
		seq_printf(s, "fc     +0x%05x %#010x\n", 0x20000 + i,
			   readl_relaxed(core + 0x20000 + i));
	for (i = 0; i <= 0x9c; i += 4)
		seq_printf(s, "fcctx  +0x%02x    %#010x\n", i,
			   readl_relaxed(ctx + i));

	for (i = 0; i <= 0x10; i += 4)
		seq_printf(s, "lmpint +0x%05x %#010x\n",
			   LMP_INT(ispfe->active.fcctx) + i,
			   readl_relaxed(core + LMP_INT(ispfe->active.fcctx) + i));
	/*
	 * Every bayer bank and every logical channel, not just the configured
	 * pair: if a frame is being delivered to a context we did not
	 * configure, the source register of that context is where it shows.
	 */
	for (i = 0; i < FC_NUM_CTX; i++)
		seq_printf(s, "bank%u  src %#010x msk %#010x ovf %#010x\n", i,
			   readl_relaxed(core + FC_CTX(i) + 0x300),
			   readl_relaxed(core + FC_CTX(i) + 0x30c),
			   readl_relaxed(core + FC_CTX(i) + 0x304));
	for (i = 0; i < LOCH_COUNT; i++)
		seq_printf(s, "loch%u  proc %#010x ovf %#010x err %#010x bind %#010x/%#010x\n",
			   i,
			   readl_relaxed(core + LOCH(i) + 0x24),
			   readl_relaxed(core + LOCH(i) + 0x28),
			   readl_relaxed(core + LOCH(i) + 0x38),
			   readl_relaxed(core + FC_CTX_BIND(i)),
			   readl_relaxed(core + FC_CTX_BIND(i) + 4));
	seq_printf(s, "lmpbind+0x%05x %#010x\n",
		   FC_CTX_BIND(ispfe->active.fcctx),
		   readl_relaxed(core + FC_CTX_BIND(ispfe->active.fcctx)));
	seq_printf(s, "lmpbind+0x%05x %#010x\n",
		   FC_CTX_BIND(ispfe->active.fcctx) + 4,
		   readl_relaxed(core + FC_CTX_BIND(ispfe->active.fcctx) + 4));

	seq_printf(s, "alloc  +0x%05x %#010x\n", LMP_ALLOC_A + LMP_ALLOC_CTRL,
		   readl_relaxed(core + LMP_ALLOC_A + LMP_ALLOC_CTRL));
	seq_printf(s, "alloc  +0x%05x %#010x\n", LMP_ALLOC_GATE,
		   readl_relaxed(core + LMP_ALLOC_GATE));
	seq_printf(s, "lmpctrl+0x%05x %#010x\n", LMP_CTRL,
		   readl_relaxed(core + LMP_CTRL));

	for (i = 0; i <= 0x40; i += 4)
		seq_printf(s, "pdma   +0x%02x    %#010x\n", i,
			   readl_relaxed(ispfe->base[ISPFE_WIN_PDMA] +
					 PDMA_CTX(ispfe->active.loch) + i));
	for (i = 0; i <= 0x18; i += 4)
		seq_printf(s, "pdmawrp+0x%02x    %#010x\n", i,
			   readl_relaxed(ispfe->base[ISPFE_WIN_PDMA_WRAP] +
					 PDMA_WRAP_CTX(ispfe->active.loch) + i));
	seq_printf(s, "pdmawrp+0x80    %#010x\n",
		   readl_relaxed(ispfe->base[ISPFE_WIN_PDMA_WRAP] +
				 PDMA_WRAP_CTX(ispfe->active.loch) + PDMA_WRAP_SRC));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ispfe_regs);

static int ispfe_status_show(struct seq_file *s, void *unused)
{
	struct ispfe_device *ispfe = s->private;
	unsigned int isolation, i;

	guard(mutex)(&ispfe->lock);

	seq_printf(s, "streaming    %u\n", ispfe->streaming);
	seq_printf(s, "snapshot_state %u\n", READ_ONCE(ispfe->snapshot_state));
	seq_printf(s, "snapshot_armed %u\n",
		   READ_ONCE(ispfe->snapshot_state) == ISPFE_SNAPSHOT_ARMED);
	seq_printf(s, "frame_frozen %u\n",
		   READ_ONCE(ispfe->snapshot_state) == ISPFE_SNAPSHOT_READY);
	seq_printf(s, "snapshot_redirected %u\n",
		   READ_ONCE(ispfe->snapshot_state) >=
		   ISPFE_SNAPSHOT_REDIRECTED);
	seq_printf(s, "power_hold   %u\n", ispfe->power_hold);
	seq_printf(s, "phy_bypass   %u\n", ispfe->phy_isolation_bypass);
	if (!regmap_read(ispfe->pmu, ispfe->pmu_iso_offset, &isolation))
		seq_printf(s, "phy_iso_pmu  %#010x\n", isolation);
	seq_printf(s, "frame_start  %u\n", atomic_read(&ispfe->frame_start));
	seq_printf(s, "frame_end    %u\n", atomic_read(&ispfe->frame_end));
	seq_printf(s, "core_events  %u\n", atomic_read(&ispfe->core_events));
	for (i = 0; i < ARRAY_SIZE(ispfe_core_leaf_names); i++)
		seq_printf(s, "core_%-9s %#010x\n", ispfe_core_leaf_names[i],
			   READ_ONCE(ispfe->core_seen[i]));
	seq_printf(s, "fc_events    %u\n", atomic_read(&ispfe->fc_events));
	seq_printf(s, "lmp_events   %u\n", atomic_read(&ispfe->lmp_events));
	seq_printf(s, "pdma_events  %u\n", atomic_read(&ispfe->pdma_events));
	seq_printf(s, "pdma_int0    %#010x\n", READ_ONCE(ispfe->pdma_seen[0]));
	seq_printf(s, "pdma_int1    %#010x  (isp_fe_ctx%u_pdma_err)\n",
		   READ_ONCE(ispfe->pdma_seen[1]), ispfe->active.loch);
	seq_printf(s, "pdma_wrap    %#010x\n", READ_ONCE(ispfe->pdma_seen[2]));
	seq_printf(s, "int0_seen    %#010x\n", READ_ONCE(ispfe->int0_seen));
	seq_printf(s, "int1_seen    %#010x\n", READ_ONCE(ispfe->int1_seen));
	seq_printf(s, "fc_seen      %#010x\n", READ_ONCE(ispfe->fc_seen));
	seq_printf(s, "lmp_seen     %#010x\n", READ_ONCE(ispfe->lmp_seen));
	seq_printf(s, "frame_iova   %pad\n", &ispfe->frame_dma);
	seq_printf(s, "spare_iova   %pad\n", &ispfe->spare_frame_dma);
	seq_printf(s, "frame_size   %zu\n", ispfe->frame_size);
	seq_printf(s, "program_iova %pad\n", &ispfe->program_dma);
	seq_printf(s, "ring_iova    %pad\n", &ispfe->ring_dma);
	for (i = 0; i < ARRAY_SIZE(ispfe_pdma_outputs); i++)
		seq_printf(s, "aux%02u_iova  %pad  size %zu\n", i,
			   &ispfe->pdma_output[i].dma,
			   ispfe_pdma_outputs[i].size);
	seq_printf(s, "ring_head    %#x\n", ispfe->head);

	if (ispfe->streaming) {
		void __iomem *core = ispfe->base[ISPFE_WIN_CORE];

		seq_printf(s, "cmn_ctrl     %#010x\n",
			   readl_relaxed(ispfe_link(ispfe) + CSIS_CMN_CTRL));
		seq_printf(s, "fc_mode      %#010x\n",
			   readl_relaxed(core + LOCH(ispfe->active.loch) +
					 LOCH_PROC_MSK));
		seq_printf(s, "pdma_head    %#010x\n",
			   readl_relaxed(ispfe->base[ISPFE_WIN_PDMA] +
					 PDMA_CTX(ispfe->active.loch) + PDMA_HEAD));
	}

	/*
	 * How much of the buffer is still poison, sampled rather than counted
	 * in full: the question a bring-up asks of an untouched 25 MB buffer is
	 * only "did anything at all write to it", and where.
	 */
	if (ispfe->frame) {
		const u8 *p = ispfe->frame;
		size_t i, step = max_t(size_t, ispfe->frame_size / 4096, 1);
		size_t touched = 0, samples = 0;
		ssize_t first = -1;

		for (i = 0; i < ispfe->frame_size; i += step, samples++) {
			if (p[i] == ISPFE_FRAME_POISON)
				continue;
			if (first < 0)
				first = i;
			touched++;
		}
		seq_printf(s, "frame_dirty  %zu of %zu samples, first %zd\n",
			   touched, samples, first);
	}

	return 0;
}

/*
 * The captured frame, straight out of the buffer the hardware was pointed at.
 * There is no V4L2 device yet, so this is how a frame leaves the phone; it is
 * raw Bayer at the configured geometry, ten bits in a sixteen-bit container.
 */
static ssize_t ispfe_frame_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct ispfe_device *ispfe = file->private_data;

	guard(mutex)(&ispfe->lock);

	if (!ispfe->frame)
		return -ENODATA;
	if (smp_load_acquire(&ispfe->snapshot_state) !=
	    ISPFE_SNAPSHOT_READY)
		return -EAGAIN;

	return simple_read_from_buffer(buf, count, ppos, ispfe->frame,
				       ispfe->frame_size);
}

static const struct file_operations ispfe_frame_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = ispfe_frame_read,
	.llseek = default_llseek,
};
DEFINE_SHOW_ATTRIBUTE(ispfe_status);

static void ispfe_debugfs_init(struct ispfe_device *ispfe)
{
	struct dentry *d;

	d = debugfs_create_dir(dev_name(ispfe->dev), NULL);
	ispfe->debugfs = d;

	debugfs_create_u32("link", 0644, d, &ispfe->src.link);
	debugfs_create_u32("phy", 0644, d, &ispfe->src.phy);
	debugfs_create_u32("lanes", 0644, d, &ispfe->src.lanes);
	debugfs_create_u32("width", 0644, d, &ispfe->src.width);
	debugfs_create_u32("height", 0644, d, &ispfe->src.height);
	debugfs_create_bool("cphy", 0644, d, &ispfe->src.cphy);
	debugfs_create_file("phy_isolation_bypass", 0644, d, ispfe,
			    &ispfe_phy_isolation_bypass_fops);
	debugfs_create_file("power_hold", 0644, d, ispfe,
			    &ispfe_power_hold_fops);
	debugfs_create_file("predown_qch", 0644, d, ispfe,
			    &ispfe_predown_qch_fops);
	debugfs_create_file("regs", 0444, d, ispfe, &ispfe_regs_fops);
	debugfs_create_u32("loch", 0644, d, &ispfe->src.loch);
	debugfs_create_u32("fcctx", 0644, d, &ispfe->src.fcctx);
	debugfs_create_u32("slot", 0644, d, &ispfe->src.slot);
	debugfs_create_u32("settle_us", 0644, d, &ispfe->settle_us);
	debugfs_create_x32("lmp_alloc_ctrl", 0644, d, &ispfe->lmp_alloc_ctrl);
	debugfs_create_x32("pdma_cmd", 0644, d, &ispfe->pdma_cmd);
	debugfs_create_x32("pdma_addr", 0644, d, &ispfe->pdma_addr);
	debugfs_create_x32("pdma_bytes", 0644, d, &ispfe->pdma_bytes);
	debugfs_create_x32("pdma_bytes_first", 0644, d,
			   &ispfe->pdma_bytes_first);
	debugfs_create_bool("pdma_no_kick", 0644, d, &ispfe->pdma_no_kick);
	debugfs_create_u32("mode_word0", 0644, d, &ispfe->src.mode_word0);
	debugfs_create_u32("mode_word1", 0644, d, &ispfe->src.mode_word1);
	debugfs_create_file("enable", 0644, d, ispfe, &ispfe_enable_fops);
	debugfs_create_file("snapshot", 0644, d, ispfe,
			    &ispfe_snapshot_fops);
	debugfs_create_file("status", 0444, d, ispfe, &ispfe_status_fops);
	debugfs_create_file("frame", 0444, d, ispfe, &ispfe_frame_fops);
}

static void ispfe_report(struct ispfe_device *ispfe)
{
	void __iomem *csis = ispfe->base[ISPFE_WIN_CSIS];
	unsigned int i, live = 0;
	u32 version;

	version = readl(ispfe->base[ISPFE_WIN_CORE] + ISPFE_VERSION);

	/*
	 * Counted against the known version rather than against link 0, so that
	 * twelve identically-wrong reads cannot pass for twelve live links.  A
	 * link reading back as zero or all-ones points at the CMU restore, not
	 * at the link.
	 */
	for (i = 0; i < CSIS_NUM_LINKS; i++)
		if (readl(csis + i * CSIS_LINK_STRIDE + CSIS_VERSION) ==
		    CSIS_VERSION_EXPECTED)
			live++;

	dev_info(ispfe->dev, "ISPFE %#010x, CSIS %#010x, %u of %u links respond\n",
		 version, readl(csis + CSIS_VERSION), live, CSIS_NUM_LINKS);
}

static int ispfe_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ispfe_device *ispfe;
	unsigned int i;
	int ret;

	ispfe = devm_kzalloc(dev, sizeof(*ispfe), GFP_KERNEL);
	if (!ispfe)
		return -ENOMEM;

	ispfe->dev = dev;
	ispfe->predown_qch = ISPFE_PREDOWN_QCH_OFF;
	ispfe->settle_us = ISPFE_SETTLE_US_DEFAULT;
	ispfe->lmp_alloc_ctrl = LMP_ALLOC_CTRL_IMX712;
	ispfe->pdma_cmd = PDMA_DESC_CMD;
	ispfe->pdma_bytes = PDMA_DESC_BYTES;
	ispfe->pdma_bytes_first = PDMA_DESC_BYTES_FIRST;
	platform_set_drvdata(pdev, ispfe);

	ret = devm_mutex_init(dev, &ispfe->lock);
	if (ret)
		return ret;

	/*
	 * Stated rather than inherited.  A platform device defaults to a
	 * 32-bit coherent mask and the System MMU aperture is wider, so
	 * widening this would start putting a non-zero high word in the PDMA
	 * descriptor -- and every ring base the vendor stack was ever seen to
	 * write is 32-bit with that word zero.
	 */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "no 32-bit DMA\n");

	/*
	 * tegu's ultrawide IMX712 as the default source, because it is the
	 * simplest of the three: one recovered mode at full readout and an
	 * ordinary four-lane D-PHY, where the main camera is three-lane C-PHY.
	 */
	ispfe->src = (struct ispfe_source){
		.link = 1, .phy = 1, .lanes = 4, .width = 4208, .height = 3120,
		/*
		 * A standalone physical-output RAW capture allocated this sensor
		 * on logical/PDMA channel 0, frame-controller context 4 and line-
		 * memory slot 2.  The earlier mixed preview's channel 1/context 2
		 * were allocator choices, not sensor properties.  Keep the measured
		 * sensor-mode words while making the captured allocation.
		 */
		.loch = 0, .fcctx = 4, .slot = 2,
		.mode_word0 = 0x000c44a0, .mode_word1 = 0x000014f8,
	};

	for (i = 0; i < ISPFE_NUM_WINDOWS; i++) {
		ispfe->base[i] = ispfe_map(pdev, ispfe_window_names[i]);
		if (IS_ERR(ispfe->base[i]))
			return dev_err_probe(dev, PTR_ERR(ispfe->base[i]),
					     "cannot map %s\n",
					     ispfe_window_names[i]);
	}

	/*
	 * Registered before runtime PM is enabled so that devres tears the two
	 * down in the right order.  devres releases in reverse and the driver
	 * core runs it before detaching the genpd, so the notifier is
	 * unregistered while dev_to_genpd() still resolves -- the other way
	 * round leaves a notifier_block that lives in this devm allocation on
	 * the domain's chain after the allocation is gone.
	 */
	ispfe->genpd_nb.notifier_call = ispfe_genpd_notify;
	ret = dev_pm_genpd_add_notifier(dev, &ispfe->genpd_nb);
	if (ret)
		return dev_err_probe(dev, ret, "cannot watch the power domain\n");

	ret = devm_add_action_or_reset(dev, ispfe_genpd_notifier_remove, dev);
	if (ret)
		return ret;

	/*
	 * The PMU register that isolates the D/C-PHYs, and the offset of the
	 * isolation word within it -- both from the phandle, in the same shape
	 * the UFS PHY already uses for its own isolation next door at 0x3ec0.
	 */
	ispfe->pmu = syscon_regmap_lookup_by_phandle(dev->of_node,
						    "samsung,pmu-syscon");
	if (IS_ERR(ispfe->pmu))
		return dev_err_probe(dev, PTR_ERR(ispfe->pmu),
				     "cannot reach the PMU\n");

	ret = of_property_read_u32_index(dev->of_node, "samsung,pmu-syscon", 1,
					 &ispfe->pmu_iso_offset);
	if (ret)
		return dev_err_probe(dev, ret,
				     "no PHY isolation offset\n");

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	/*
	 * The first resume of this domain is also the first time anything has
	 * asked the secure monitor to restore BLK_ISPFE's TZPC state, so a
	 * failure here is worth reporting rather than retrying quietly.
	 */
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot power BLK_ISPFE\n");

	ispfe_report(ispfe);

	pm_runtime_put(dev);

	ispfe_debugfs_init(ispfe);

	return 0;
}

/*
 * Nothing here is devm: the debugfs files hand out a pointer to this device
 * and can start a stream, so they have to be gone before anything they touch
 * is, and a stream left running holds a runtime-PM reference.
 */
static void ispfe_remove(struct platform_device *pdev)
{
	struct ispfe_device *ispfe = platform_get_drvdata(pdev);
	int ret;

	debugfs_remove_recursive(ispfe->debugfs);

	scoped_guard(mutex, &ispfe->lock) {
		if (ispfe->streaming)
			ispfe_stop(ispfe);
		if (ispfe->power_hold) {
			ret = ispfe_phy_isolation(ispfe, false);
			if (ret) {
				dev_crit(ispfe->dev,
					 "cannot re-isolate PHYs; retaining power hold\n");
				ispfe_buffers_free(ispfe);
				return;
			}
			ispfe->phy_isolation_bypass = false;
			ispfe->power_hold = false;
			pm_runtime_put_sync(ispfe->dev);
		}
		ispfe_buffers_free(ispfe);
	}
}

static const struct of_device_id ispfe_of_match[] = {
	{ .compatible = "google,zumapro-ispfe" },
	{ }
};
MODULE_DEVICE_TABLE(of, ispfe_of_match);

static struct platform_driver ispfe_driver = {
	.probe = ispfe_probe,
	.remove = ispfe_remove,
	.driver = {
		.name = "exynos-ispfe",
		.of_match_table = ispfe_of_match,
		.pm = pm_ptr(&ispfe_pm_ops),
		/*
		 * ->remove cannot veto driver-core's subsequent genpd detach and
		 * power-down if re-isolation fails.  This bring-up driver is built
		 * in, so hide the userspace unbind path rather than make that known
		 * watchdog sequence reachable while a diagnostic hold exists.
		 */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(ispfe_driver);

MODULE_DESCRIPTION("Google zumapro ISPFE camera receive complex");
MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_LICENSE("GPL");
