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
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/interconnect.h>
#include <linux/mfd/syscon.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of_platform.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <media/exynos-becore.h>
#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#include "exynos-ispfe-pdma-program.h"
#include "exynos-ispfe-pdma-program-binned.h"
#include "exynos-ispfe-pdma-program-backend.h"
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
 * LMP_CTRL transitions, the PDMA wrap resets, the FC LMP-IDMA init and the
 * START pulse into a block that was still initialising -- which register
 * contents survive, so every readback still matched, and pulses do not.
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
 * Two frame-controller LMP-IDMA AXI common banks.  Lyric's register
 * descriptors name the first four words and the per-context threshold pairs;
 * the capture writes the same thresholds to both banks.  The threshold entry
 * is selected by the frame-controller context itself, not by an independent
 * allocator slot.
 */
#define FC_LMP_IDMA0			0x30c00
#define FC_LMP_IDMA1			0x30d00
#define FC_LMP_IDMA_MAX_BURST		0x00
#define FC_LMP_IDMA_AXI_REORDER_EN	0x04
#define FC_LMP_IDMA_AXI_MAX_OST		0x08
#define FC_LMP_IDMA_AXCACHE		0x0c
#define FC_LMP_IDMA_MAX_BURST_VAL	0x00000001
#define FC_LMP_IDMA_AXI_REORDER_EN_VAL	0x00000001
#define FC_LMP_IDMA_AXCACHE_VAL		0x00000000
#define FC_LMP_IDMA_AXI_MAX_OST_IMX712	0x0000001a
#define FC_LMP_IDMA1_AXI_MAX_OST_VAL	0x00000002
/* What both max-outstanding words read idle, and what stop writes. */
#define FC_LMP_IDMA_AXI_MAX_OST_IDLE	0x00000002
#define FC_LMP_IDMA_AXI_THRE(ctx)	(0x20 + (ctx) * 8)
#define FC_LMP_IDMA_AXI_THRE_HIGH_VAL	0x00000040
#define FC_LMP_IDMA_AXI_THRE_LOW_VAL	0x00000020
#define FC_CFG_BUS_CTRL			0x30f00
#define FC_CFG_BUS_CTRL_VAL		0x00000001
#define FC_LMCS_COMN			0x30100
#define FC_LMCS_COMN_VAL		0x00000053

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
/*
 * The vendor's ring is 1000 bytes, which is 62 sixteen-byte records and eight
 * bytes left over -- so its head reaches 0x3e0, whose record wraps with half at
 * the end of the ring and half at the start.  A producer that rewrites each
 * record before publishing it can use that; one that fills the ring in advance
 * cannot, because offset 0 would have to be both the tail of the wrapped record
 * and the head of the first.  Round the size down to whole records instead of
 * leaving those eight bytes to be reasoned about: then a lap is exactly
 * PDMA_NUM_RECORDS credits and the head arithmetic closes.
 */
#define PDMA_SIZE_VAL			0x3e0
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
 * word is its exact byte count.
 */
struct ispfe_pdma_desc {
	__le32 cmd;
	__le32 addr_lo;
	__le32 addr_hi;
	__le32 bytes;
} __packed;

#define PDMA_DESC_CMD			0x0000c003
#define PDMA_NUM_RECORDS \
	(PDMA_SIZE_VAL / sizeof(struct ispfe_pdma_desc))

/*
 * PDMA applies one program per frame boundary, so a stream that puts each frame
 * in a different buffer needs one program per frame in flight rather than one
 * shared program to be rewritten underneath the hardware. Slots are page-
 * aligned, as the vendor's rotating program buffers were.
 */
/*
 * How many further credits must exist before the buffer a credit named can be
 * handed back.  One, because a record credited at a frame start governs the
 * frame after it: the frame ending at the next end-of-frame is the one the
 * credit before it placed.
 */
#define ISPFE_CREDIT_LATENCY_DEFAULT	1

#define PDMA_BUF_SLOTS			4
/*
 * One more, aimed at a buffer nobody reads.  The receiver runs continuously, so
 * every frame boundary needs a program whether or not userspace has a buffer
 * ready; without somewhere harmless to put those frames the only choices are to
 * stall the ring or to write over a buffer that has already been handed out.
 */
#define PDMA_DUMP_SLOT			PDMA_BUF_SLOTS
#define PDMA_SLOTS			(PDMA_BUF_SLOTS + 1)
#define PDMA_SLOT_STRIDE		ALIGN(ISPFE_PDMA_RECIPE_BYTES, PAGE_SIZE)
#define PDMA_PROGRAMS_SIZE		(PDMA_SLOTS * PDMA_SLOT_STRIDE)
#define ISPFE_PDMA_MAX_BLOCKS_BYTES	ISPFE_PDMA_BACKEND_BLOCKS_BYTES

enum ispfe_backend_buffer_state {
	ISPFE_BACKEND_BUFFER_IDLE,
	ISPFE_BACKEND_BUFFER_PREPARING,
	ISPFE_BACKEND_BUFFER_READY,
	ISPFE_BACKEND_BUFFER_FLIGHT,
	ISPFE_BACKEND_BUFFER_DONE,
};

struct ispfe_backend_buffer {
	struct exynos_becore_input_buffer ticket;
	struct list_head list;
	enum ispfe_backend_buffer_state state;
	unsigned int program_slot;
	u64 retire_credit;
};

/*
 * The working areas the front end writes back to.  The blocks it *reads* do not
 * change between frames and live in one shared area every slot's program points
 * at; these are the other direction, and are equally shared because only one
 * stream runs at a time.
 */
struct ispfe_pdma_output {
	size_t size;
	enum dma_data_direction direction;
	const u8 *seed;
	size_t seed_size;
};

#define PDMA_OUTPUT(_size) { \
	.size = (_size), .direction = DMA_FROM_DEVICE, \
}
#define PDMA_OUTPUT_RW(_size, _seed) { \
	.size = (_size), .direction = DMA_BIDIRECTIONAL, \
	.seed = (_seed), .seed_size = sizeof(_seed), \
}

/* Large enough for a 2048x1536 NV12 runtime geometry experiment. */
#define ISPFE_LMP_ML0_MAX_SIZE		0x00480000

/* Exact full-mode LMP main-Bayer allocation observed on the ultrawide. */
#define ISPFE_BACKEND_INPUT_SIZE		0x01a17000
#define ISPFE_BACKEND_IMAGE_OFFSET	0x00030c00
/* Separate full-mode TNR-pyramid output allocation from the same request. */
#define ISPFE_TNR_PYRAMID_SIZE		0x00468000

/*
 * The working areas the front end writes back to, in the order the recipe's
 * ISPFE_BUF_OUTPUT() indices name them. Sizes and directions are the vendor
 * session's own allocation classes; the three bidirectional buffers are seeded
 * with its captured 0x2000-byte prefix and the rest of each starts at zero.
 * Outputs 0--9 are completion/statistics buffers. Outputs 10--12 are the LMP's
 * processed-image destinations: planar linear RGB, YUV420 ML output 0 and
 * interleaved RGB888 ML output 2 respectively.
 */
static const struct ispfe_pdma_output ispfe_pdma_outputs[] = {
	PDMA_OUTPUT(4096),
	PDMA_OUTPUT(20480),
	PDMA_OUTPUT(102400),
	PDMA_OUTPUT(4096),
	PDMA_OUTPUT(299008),
	PDMA_OUTPUT(299008),
	PDMA_OUTPUT(12288),
	PDMA_OUTPUT(12288),
	PDMA_OUTPUT(12288),
	PDMA_OUTPUT(299008),
	PDMA_OUTPUT_RW(299008, ispfe_pdma_seed_1cb80000),
	PDMA_OUTPUT_RW(ISPFE_LMP_ML0_MAX_SIZE, ispfe_pdma_seed_1cb00000),
	PDMA_OUTPUT_RW(311296, ispfe_pdma_seed_1c980000),
};

#define ISPFE_PDMA_OUTPUT_RGB		10
#define ISPFE_PDMA_OUTPUT_ML0		11
#define ISPFE_PDMA_OUTPUT_ML2		12

/*
 * ML output 0 is fed by the three-output scaler whose input is 1052x780 in
 * both captured sensor-mode recipes.  Keep the vendor's 640x480 setup as
 * profile 0, and make the first geometry experiment a downscale to 1024x768:
 * it raises the output bandwidth without changing DDS or asking the scaler to
 * invent pixels.  The factors are input / output in unsigned Q21.
 */
#define ISPFE_LMP_ML0_PROFILE_CAPTURED	0
#define ISPFE_LMP_ML0_PROFILE_1024X768	1
#define ISPFE_LMP_ML0_PROFILE_COUNT	2
#define ISPFE_LMP_ML0_SCALER_INPUT	6
#define ISPFE_LMP_ML0_1024_WIDTH		1024
#define ISPFE_LMP_ML0_1024_HEIGHT	768
#define ISPFE_LMP_ML0_1024_STRIDE	1024
#define ISPFE_LMP_ML0_1024_H_FACTOR	0x0020e000
#define ISPFE_LMP_ML0_1024_V_FACTOR	0x00208000
#define ISPFE_LMP_ML0_1024_Y_SIZE	0x000c0000
#define ISPFE_LMP_ML0_1024_PLANE2	0x00180000

/*
 * 16 phases x four signed Q15 taps in each direction, generated by the
 * recovered g3aa_lmp_scale_crop rule in camera-pdma-report.py for
 * 1052x780 -> 1024x768.  Horizontal precedes vertical.
 */
static const u8 ispfe_lmp_ml0_1024_lut[0x100] = {
	0x22, 0x02, 0xef, 0x7b, 0x22, 0x02, 0xcd, 0xff, 0x9f, 0xfd, 0x21, 0x7b,
	0x1c, 0x08, 0x24, 0xff, 0x31, 0xfa, 0x45, 0x78, 0x6c, 0x0f, 0x1e, 0xfe,
	0xb3, 0xf7, 0xa3, 0x73, 0xd6, 0x17, 0xd4, 0xfc, 0x0e, 0xf6, 0x76, 0x6d,
	0x1d, 0x21, 0x5f, 0xfb, 0x28, 0xf5, 0xf9, 0x65, 0x04, 0x2b, 0xdb, 0xf9,
	0xe8, 0xf4, 0x6b, 0x5d, 0x4d, 0x35, 0x60, 0xf8, 0x34, 0xf5, 0x07, 0x54,
	0xba, 0x3f, 0x0b, 0xf7, 0xf3, 0xf5, 0x0d, 0x4a, 0x0d, 0x4a, 0xf3, 0xf5,
	0x0b, 0xf7, 0xba, 0x3f, 0x07, 0x54, 0x34, 0xf5, 0x60, 0xf8, 0x4d, 0x35,
	0x6b, 0x5d, 0xe8, 0xf4, 0xdb, 0xf9, 0x04, 0x2b, 0xf9, 0x65, 0x28, 0xf5,
	0x5f, 0xfb, 0x1d, 0x21, 0x76, 0x6d, 0x0e, 0xf6, 0xd4, 0xfc, 0xd6, 0x17,
	0xa3, 0x73, 0xb3, 0xf7, 0x1e, 0xfe, 0x6c, 0x0f, 0x45, 0x78, 0x31, 0xfa,
	0x24, 0xff, 0x1c, 0x08, 0x21, 0x7b, 0x9f, 0xfd, 0x37, 0x01, 0xa5, 0x7d,
	0x36, 0x01, 0xee, 0xff, 0xd5, 0xfc, 0xb0, 0x7c, 0x11, 0x07, 0x6a, 0xff,
	0x86, 0xf9, 0xa7, 0x79, 0x56, 0x0e, 0x7d, 0xfe, 0x2c, 0xf7, 0xcd, 0x74,
	0xc8, 0x16, 0x3f, 0xfd, 0xac, 0xf5, 0x60, 0x6e, 0x26, 0x20, 0xce, 0xfb,
	0xeb, 0xf4, 0x9f, 0x66, 0x32, 0x2a, 0x44, 0xfa, 0xd1, 0xf4, 0xc9, 0x5d,
	0xab, 0x34, 0xbb, 0xf8, 0x40, 0xf5, 0x20, 0x54, 0x50, 0x3f, 0x50, 0xf7,
	0x1e, 0xf6, 0xe2, 0x49, 0xe2, 0x49, 0x1e, 0xf6, 0x50, 0xf7, 0x50, 0x3f,
	0x20, 0x54, 0x40, 0xf5, 0xbb, 0xf8, 0xab, 0x34, 0xc9, 0x5d, 0xd1, 0xf4,
	0x44, 0xfa, 0x32, 0x2a, 0x9f, 0x66, 0xeb, 0xf4, 0xce, 0xfb, 0x26, 0x20,
	0x60, 0x6e, 0xac, 0xf5, 0x3f, 0xfd, 0xc8, 0x16, 0xcd, 0x74, 0x2c, 0xf7,
	0x7d, 0xfe, 0x56, 0x0e, 0xa7, 0x79, 0x86, 0xf9, 0x6a, 0xff, 0x11, 0x07,
	0xb0, 0x7c, 0xd5, 0xfc,
};

/*
 * The vendor stack's own saved frame is width * 2 * height with no padding, so
 * the receiver writes 10-bit samples into 16-bit containers at the natural
 * stride.  Allocating exactly that is deliberate: if the hardware writes past
 * it the System MMU faults, which says more than a silent overrun would.
 */
#define ISPFE_BYTES_PER_PIXEL		2

/*
 * One recipe per sensor mode, and deliberately not one recipe parameterised on
 * the frame size.  Byte-diffing the vendor's two programs gives 214 differing
 * bytes in 110 runs, of which only about a dozen are the geometry in a form
 * anything could compute: the rest are tile counts, recomputed ramps and
 * line-memory tuning with no visible relation to the size.  The write stride
 * makes the point on its own -- 4224 for a 2104-pixel row, where width * 2 is
 * 4208 -- so a computed recipe would be wrong by sixteen bytes a line with no
 * error bit to say so.
 *
 * Both are generated by camera-pdma-emit.py, which re-encodes what it emits and
 * refuses to write unless the result is byte-for-byte the captured program.
 */
struct ispfe_pdma_program {
	const struct ispfe_pdma_cmd *cmds;
	unsigned int num_cmds;
	const struct ispfe_pdma_reloc *relocs;
	unsigned int num_relocs;
	const struct ispfe_pdma_input *inputs;
	unsigned int num_inputs;
	/* The geometry the capture was taken at; a format must match exactly. */
	u32 width;
	u32 height;
	u32 stride;
	u32 recipe_bytes;
	u32 blocks_bytes;
	bool raw_output;
	bool backend_output;
	bool patch_backend_output;
	bool backend_recipe;
	bool fixed_resources;
	u8 required_loch;
	u8 required_fcctx;
	u32 required_fc_axi_max_ost;
};

static const struct ispfe_pdma_program ispfe_pdma_programs[] = {
	{
		.cmds = ispfe_pdma_recipe,
		.num_cmds = ARRAY_SIZE(ispfe_pdma_recipe),
		.relocs = ispfe_pdma_relocs,
		.num_relocs = ARRAY_SIZE(ispfe_pdma_relocs),
		.inputs = ispfe_pdma_inputs,
		.num_inputs = ARRAY_SIZE(ispfe_pdma_inputs),
		.width = 4208, .height = 3120, .stride = 8416,
		.recipe_bytes = ISPFE_PDMA_RECIPE_BYTES,
		.blocks_bytes = ISPFE_PDMA_BLOCKS_BYTES,
		.raw_output = true,
		.backend_output = true,
		.patch_backend_output = true,
	},
	{
		.cmds = ispfe_pdma_binned_recipe,
		.num_cmds = ARRAY_SIZE(ispfe_pdma_binned_recipe),
		.relocs = ispfe_pdma_binned_relocs,
		.num_relocs = ARRAY_SIZE(ispfe_pdma_binned_relocs),
		.inputs = ispfe_pdma_binned_inputs,
		.num_inputs = ARRAY_SIZE(ispfe_pdma_binned_inputs),
		.width = 2104, .height = 1560, .stride = 4224,
		.recipe_bytes = ISPFE_PDMA_BINNED_RECIPE_BYTES,
		.blocks_bytes = ISPFE_PDMA_BINNED_BLOCKS_BYTES,
		.raw_output = true,
	},
	{
		.cmds = ispfe_pdma_backend_recipe,
		.num_cmds = ARRAY_SIZE(ispfe_pdma_backend_recipe),
		.relocs = ispfe_pdma_backend_relocs,
		.num_relocs = ARRAY_SIZE(ispfe_pdma_backend_relocs),
		.inputs = ispfe_pdma_backend_inputs,
		.num_inputs = ARRAY_SIZE(ispfe_pdma_backend_inputs),
		.width = 4208, .height = 3120, .stride = 8416,
		.recipe_bytes = ISPFE_PDMA_BACKEND_RECIPE_BYTES,
		.blocks_bytes = ISPFE_PDMA_BACKEND_BLOCKS_BYTES,
		.backend_output = true,
		.backend_recipe = true,
		.fixed_resources = true,
		.required_loch = 0,
		.required_fcctx = 3,
		.required_fc_axi_max_ost = FC_LMP_IDMA_AXI_MAX_OST_IMX712,
	},
};

/*
 * The program area and the shared block area are sized once for all recipes.
 * They happen to agree today; if a future capture does not, these say so at
 * build time rather than by overrunning an allocation.
 */
static_assert(ISPFE_PDMA_BINNED_RECIPE_BYTES == ISPFE_PDMA_RECIPE_BYTES);
static_assert(ISPFE_PDMA_BINNED_BLOCKS_BYTES == ISPFE_PDMA_BLOCKS_BYTES);
static_assert(ISPFE_PDMA_BACKEND_RECIPE_BYTES <= PDMA_SLOT_STRIDE);
static_assert(ISPFE_PDMA_BLOCKS_BYTES <= ISPFE_PDMA_MAX_BLOCKS_BYTES);

/* The recipe for a geometry, or NULL if none was captured for it. */
static const struct ispfe_pdma_program *ispfe_program_for(u32 width, u32 height,
						  bool backend_recipe)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ispfe_pdma_programs); i++)
		if (ispfe_pdma_programs[i].width == width &&
		    ispfe_pdma_programs[i].height == height &&
		    ispfe_pdma_programs[i].backend_recipe == backend_recipe)
			return &ispfe_pdma_programs[i];

	return NULL;
}
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
 * Which combo D/C-PHY instance serves each CSIS link bank, and the two
 * frame-controller context words that go with the sensor mode arriving on it.
 *
 * Nine PHYs serve twelve link banks, so a link's PHY is not its own index --
 * the front camera is link 6 on PHY 5 -- and the mapping was measured by
 * watching which PHY instance had its common and lane blocks programmed in
 * the same segment that bound the link.  Only the three links this phone uses
 * have been measured; a device tree naming any other link is refused rather
 * than guessed at.
 *
 * The two words are the *mode's*, not the link's.  They are indexed by link
 * only because each link here carries one sensor and one recovered mode; they
 * are the same for a raw and a YUV stream on the same sensor, and they differ
 * between the two identical IMX712s, so nothing about them is derivable yet.
 * A second mode on one of these links needs them measured again.
 */
struct ispfe_link_cfg {
	bool known;
	u8 phy;
	u32 mode_word0;
	u32 mode_word1;
};

static const struct ispfe_link_cfg ispfe_link_cfg[CSIS_NUM_LINKS] = {
	[0] = { true, 0, 0x013bd6d0, 0x00000180 },	/* barghest, main */
	[1] = { true, 1, 0x000c44a0, 0x000014f8 },	/* leshen-uw */
	[6] = { true, 5, 0x0007ca00, 0x000015f0 },	/* leshen, front */
};

/*
 * What one CSI-2 source looks like to the receiver.  The link bank, the lane
 * count and the PHY type come from the device tree's endpoint, and the
 * geometry from the format negotiated on the subdev's source pad.  What is
 * left is the receiver's own resource allocation, which is the driver's to
 * choose and stays a debugfs control while the choice is still being swept.
 */
struct ispfe_source {
	u32 link;
	u32 phy;
	u32 lanes;
	u32 width;
	u32 height;
	bool cphy;

	/*
	 * Which of the receiver's own resources the stream is put on.  The
	 * logical channel also selects the PDMA context.  The frame-controller
	 * context is independent, but directly selects its LMP-IDMA AXI threshold
	 * pair; there is no third allocator-slot choice.
	 */
	u32 loch;
	u32 fcctx;

	/* Not derived from anything: measured for one sensor mode. */
	u32 mode_word0;
	u32 mode_word1;
};

/*
 * The media graph.  The sensor feeds the receiver's sink pad, the receiver's
 * source pad feeds the capture node, and neither link can be switched: this
 * block has twelve link banks but the driver drives one stream, and which bank
 * that stream is on is the device tree's statement rather than a choice
 * userspace makes.
 */
#define ISPFE_PAD_SINK			0
#define ISPFE_PAD_SOURCE		1
#define ISPFE_NUM_PADS			2

/*
 * The receive path does not convert: what the sensor puts on the link is what
 * lands in memory, ten bits per sample in a sixteen-bit little-endian
 * container at the natural stride.  So the mosaic order is the sensor's and
 * these pairs are a renaming rather than a format list.
 */
struct ispfe_format {
	u32 code;
	u32 pixelformat;
};

static const struct ispfe_format ispfe_formats[] = {
	{ MEDIA_BUS_FMT_SRGGB10_1X10, V4L2_PIX_FMT_SRGGB10 },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, V4L2_PIX_FMT_SGRBG10 },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, V4L2_PIX_FMT_SGBRG10 },
	{ MEDIA_BUS_FMT_SBGGR10_1X10, V4L2_PIX_FMT_SBGGR10 },
};

/*
 * The ultrawide IMX712's full-array mode, which is what every frame captured
 * so far was taken at.  Used as the format the subdev starts in; the bounds
 * are the CSIS resolution register's, which is sixteen bits per axis.
 */
#define ISPFE_DEFAULT_WIDTH		4208U
#define ISPFE_DEFAULT_HEIGHT		3120U
#define ISPFE_DEFAULT_CODE		MEDIA_BUS_FMT_SRGGB10_1X10
#define ISPFE_MIN_WIDTH			32U
#define ISPFE_MIN_HEIGHT		32U
#define ISPFE_MAX_WIDTH			U16_MAX
#define ISPFE_MAX_HEIGHT		U16_MAX

/* Exact ultrawide session requests measured at downstream's public APIs. */
#define ISPFE_MEMORY_BW_KBPS		974745U
#define ISPFE_CAM_SETUP_RATE		711000000UL
#define ISPFE_CAM_ACTIVE_RATE		111000000UL

struct ispfe_device {
	struct device *dev;
	struct clk *cam_clk;
	struct icc_path *memory_path;
	unsigned long saved_cam_rate;
	bool cam_rate_active;
	bool memory_vote_active;
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
	 * Whether the diagnostic has asked for the shared D/C-PHY isolation to
	 * be held open.  Streaming opens and closes it for itself; this is for
	 * observing the two PMU edges separately from a domain power-down, and
	 * while it is set the stream leaves the bypass alone on the way out.
	 */
	bool phy_isolation_bypass;
	/*
	 * Whether *this stream* took the bypass and therefore owes the restore.
	 * Separate from the request above so that acquire and release test the
	 * same fact: the request is the operator's and can in principle change,
	 * where this is set exactly where the bypass is taken.
	 */
	bool phy_bypass_held;
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
	 * The first FC LMP-IDMA bank's AXI max-outstanding word.  0x1a is what
	 * one IMX712 alone was measured writing; the ultrawide with a
	 * phase-detect stream beside it wrote 0x88 and the main camera 0x6c.
	 * The register's purpose is known, but not how the value is packed, so
	 * it remains a knob rather than a constant.
	 */
	u32 fc_axi_max_ost;
	/* Requested and stream-latched processed-output geometry experiment. */
	u32 lmp_ml0_profile;
	u32 active_lmp_ml0_profile;
	/* Select the captured ordinary back-end program for a debugfs run. */
	u32 backend_recipe;
	bool active_backend_recipe;
	/*
	 * Idle-only debugfs staging for runtime PDMA experiments.  The program
	 * remains tied to the selected captured recipe: its command grammar,
	 * targets and grouped-write register words are validated at stream start,
	 * while every DMA address is discarded and regenerated by the driver.
	 */
	u8 *pdma_program_staged;
	u8 *pdma_blocks_staged;
	size_t pdma_program_staged_bytes;
	size_t pdma_blocks_staged_bytes;
	u32 pdma_program_staged_generation;
	u32 pdma_blocks_staged_generation;
	u32 active_pdma_program_generation;
	u32 active_pdma_blocks_generation;
	bool pdma_program_override;
	bool active_pdma_program_override;
	/* Bring-up overrides for the measured opcode/length and head push. */
	u32 pdma_cmd;
	/*
	 * Override the descriptor's program address. Zero selects the relocated
	 * captured program; a deliberate unmapped IOVA remains useful for proving
	 * that PDMA issues the read and reaches the System MMU.
	 */
	u32 pdma_addr;
	/*
	 * The recipe for the geometry the stream latched, chosen in the same
	 * place src is snapshotted so it cannot move under a running stream.
	 */
	const struct ispfe_pdma_program *prog;
	u32 pdma_bytes;
	u32 pdma_bytes_first;
	bool pdma_no_kick;
	bool streaming;
	bool sensor_streaming;
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
	/* PDMA_SLOTS programs, and the block area every one of them shares. */
	void *programs;
	dma_addr_t programs_dma;
	void *blocks;
	dma_addr_t blocks_dma;
	/*
	 * Byte offsets within a program of the frame destination's two address
	 * halves, so a destination can be retargeted without searching the
	 * program for an address.
	 */
	u32 bayer_lo;
	u32 bayer_hi;
	/* Full-mode LMP main-Bayer output shared with the camera back end. */
	struct exynos_becore_input *backend_input;
	struct exynos_becore_input_buffer backend_buffer;
	struct ispfe_backend_buffer backend_buffers[PDMA_BUF_SLOTS];
	struct work_struct backend_fill_work;
	struct list_head backend_ready;
	struct list_head backend_flight;
	struct list_head backend_done;
	unsigned long backend_programs_used;
	unsigned int backend_flight_count;
	u64 backend_credit_count;
	u32 backend_completed;
	u32 backend_dropped;
	int backend_queue_error;
	bool backend_queue_active;
	void *backend_spare;
	dma_addr_t backend_spare_dma;
	size_t backend_input_size;
	void *tnr_pyramid;
	dma_addr_t tnr_pyramid_dma;
	bool backend_producing;
	bool backend_handed_off;
	u32 backend_image_lo;
	u32 backend_image_hi;
	u32 backend_header_lo;
	u32 backend_header_hi;

	/*
	 * Who is driving the hardware.  The debugfs diagnostic and the V4L2
	 * queue arm the same single receive path, so they are mutually
	 * exclusive rather than layered.
	 */
	enum ispfe_owner {
		ISPFE_OWNER_NONE,
		ISPFE_OWNER_DEBUGFS,
		ISPFE_OWNER_BACKEND,
		ISPFE_OWNER_V4L2,
	} owner;

	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_subdev sd;
	struct media_pad pads[ISPFE_NUM_PADS];
	struct v4l2_async_notifier notifier;
	/*
	 * The sensor, and which of its pads the link comes from.  Written by
	 * the notifier under the driver's own lock, and read from the capture
	 * queue, which holds that lock for the whole of start and stop -- so an
	 * unbind cannot take the subdev away from under a stream, it waits for
	 * the stream to finish instead.
	 */
	struct v4l2_subdev *sensor;
	u32 sensor_pad;
	struct video_device vdev;
	struct media_pad vdev_pad;
	struct media_pipeline pipe;
	struct vb2_queue queue;
	struct v4l2_pix_format fmt;
	struct work_struct fill_work;
	u32 sequence;
	/*
	 * How many frame boundaries pass between a ring record being credited
	 * and the frame it governs finishing.  Measured rather than assumed,
	 * and a control because getting it wrong hands userspace a buffer the
	 * receiver has not written yet.
	 */
	u32 credit_latency;

	/* Guards the three buffer lists and the slot bitmap against the IRQs. */
	spinlock_t slock;
	struct list_head pending;
	struct list_head ready;
	struct list_head flight;
	unsigned int flight_count;
	unsigned long slots_used;
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
#define ISPFE_PHY_MAX_LANES		4

static u32 ispfe_phy_lanes(u32 phy)
{
	u32 next = phy + 1 < PHY_NUM_INSTANCES ? ispfe_phy_base[phy + 1]
					       : ispfe_phy_base[phy] + 0x500;
	u32 room = (next - ispfe_phy_base[phy]) / 0x100 - 1;

	/*
	 * The gap to the next instance is a bound on what can be written
	 * without walking into a neighbour, not a lane count: instance 1 has
	 * 0xb00 of room and the part is an m0s4s4s4s4s4, so four is the real
	 * limit and the gap only ever lowers it.
	 */
	return min(room, ISPFE_PHY_MAX_LANES);
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
 * Isolation has to be restored before the domain goes down, which is why the
 * bypass belongs to a stream: ispfe_start() takes it once the domain is up and
 * ispfe_stop() puts it back while the runtime-PM reference is still held, the
 * same place downstream does it from its PHY power-off.  GENPD_NOTIFY_PRE_OFF
 * is the backstop, delivered before genpd touches the PMU.
 *
 * This was long thought to be what stopped BLK_ISPFE powering down after a
 * capture.  It was not: the real cause was a program that activated frame-
 * controller context 4 while teardown cleared context 0, and with the whole
 * allocation matched a streamed domain reaches off cleanly with the bypass
 * taken and restored here [HW 2026-08-19].
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
	 * the caller runs the LMP_CTRL transitions, the FC LMP-IDMA setup, the PDMA
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
 * A streamed domain does power down: it needed the program's own frame-
 * controller context to be the one teardown quiesces, and with that matched the
 * PMU handshake completes and the isolation bypass taken by ispfe_start() is
 * not involved [HW 2026-08-19].  What follows is a knob left from when that was
 * not understood and the refusal was being blamed on the Q-channels, kept
 * because it is the one lever over the wrapper clocks and costs nothing while
 * it is off.
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
	if (ispfe->phy_isolation_bypass || ispfe->pdma_program_override)
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

static dma_addr_t ispfe_slot_dma(struct ispfe_device *ispfe, unsigned int slot)
{
	return ispfe->programs_dma + slot * PDMA_SLOT_STRIDE;
}

/* Point one ring record at a program.  The caller owns the head. */
static void ispfe_ring_record(struct ispfe_device *ispfe, unsigned int index,
			      unsigned int slot)
{
	dma_addr_t dma = ispfe_slot_dma(ispfe, slot);
	u32 bytes = ispfe->active_backend_recipe ?
		    ispfe->prog->recipe_bytes : READ_ONCE(ispfe->pdma_bytes);

	ispfe->ring[index].cmd = cpu_to_le32(READ_ONCE(ispfe->pdma_cmd));
	ispfe->ring[index].addr_lo = cpu_to_le32(lower_32_bits(dma));
	ispfe->ring[index].addr_hi = cpu_to_le32(upper_32_bits(dma));
	ispfe->ring[index].bytes = cpu_to_le32(bytes);
}

struct ispfe_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	unsigned int slot;
};

static struct ispfe_buffer *to_ispfe_buffer(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct ispfe_buffer, vb);
}

/*
 * Hand the ring one program, at a frame start.  PDMA fetches a credited record
 * almost immediately but only applies it at the following frame boundary, so
 * what this credits is where the *next* frame lands, not this one.  A credit
 * per frame start is also the pacing the vendor uses: crediting several at once
 * lets the front end fetch and overwrite them faster than frames consume them.
 */
static void ispfe_queue_credit(struct ispfe_device *ispfe)
{
	struct ispfe_buffer *buf;
	unsigned int slot = PDMA_DUMP_SLOT;

	spin_lock(&ispfe->slock);
	if (!list_empty(&ispfe->ready)) {
		buf = list_first_entry(&ispfe->ready, struct ispfe_buffer, list);
		list_move_tail(&buf->list, &ispfe->flight);
		ispfe->flight_count++;
		slot = buf->slot;
	}
	spin_unlock(&ispfe->slock);

	ispfe_ring_record(ispfe, ispfe->head / sizeof(*ispfe->ring), slot);
	ispfe->head += sizeof(*ispfe->ring);
	if (ispfe->head >= PDMA_NUM_RECORDS * sizeof(*ispfe->ring))
		ispfe->head = 0;
	dma_wmb();
	writel_relaxed(ispfe->head, ispfe->base[ISPFE_WIN_PDMA] +
		       PDMA_CTX(ispfe->active.loch) + PDMA_HEAD);
}

/*
 * Complete the oldest frame in flight, at the line-memory end-of-frame.  The
 * buffer that just finished is not the one credited at the preceding frame
 * start but the one before it, because a credit governs the following frame --
 * so a buffer is only done once enough later credits exist to place it.
 */
static void ispfe_queue_complete(struct ispfe_device *ispfe)
{
	struct ispfe_buffer *buf = NULL;

	spin_lock(&ispfe->slock);
	/*
	 * Clamped: a latency the queue can never reach means no buffer is ever
	 * completed and DQBUF blocks for ever, with nothing said.
	 */
	if (ispfe->flight_count >
	    min_t(u32, READ_ONCE(ispfe->credit_latency), PDMA_BUF_SLOTS - 1)) {
		buf = list_first_entry(&ispfe->flight, struct ispfe_buffer,
				       list);
		list_del(&buf->list);
		ispfe->flight_count--;
		__clear_bit(buf->slot, &ispfe->slots_used);
	}
	spin_unlock(&ispfe->slock);

	if (!buf)
		return;

	/*
	 * The frame counter, not a count of buffers handed back: frames that
	 * landed in the dump slot because nothing was queued then show up as a
	 * gap, which is what a sequence number is for.  The timestamp is this
	 * end-of-frame, so it trails the buffer's own frame by the credit
	 * latency.
	 */
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = ispfe->sequence;
	buf->vb.field = V4L2_FIELD_NONE;
	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, ispfe->fmt.sizeimage);
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

	/* A slot is free now, so a pending buffer can be encoded into it. */
	schedule_work(&ispfe->fill_work);
}

static void ispfe_backend_queue_credit(struct ispfe_device *ispfe)
{
	struct ispfe_backend_buffer *buf = NULL;
	unsigned int slot = PDMA_DUMP_SLOT;

	spin_lock(&ispfe->slock);
	ispfe->backend_credit_count++;
	if (!list_empty(&ispfe->backend_ready)) {
		buf = list_first_entry(&ispfe->backend_ready,
				       struct ispfe_backend_buffer, list);
		list_move_tail(&buf->list, &ispfe->backend_flight);
		buf->state = ISPFE_BACKEND_BUFFER_FLIGHT;
		buf->retire_credit = ispfe->backend_credit_count +
			min_t(u32, READ_ONCE(ispfe->credit_latency),
			      PDMA_BUF_SLOTS - 1);
		ispfe->backend_flight_count++;
		slot = buf->program_slot;
	} else {
		ispfe->backend_dropped++;
	}
	spin_unlock(&ispfe->slock);

	ispfe_ring_record(ispfe, ispfe->head / sizeof(*ispfe->ring), slot);
	ispfe->head += sizeof(*ispfe->ring);
	if (ispfe->head >= PDMA_NUM_RECORDS * sizeof(*ispfe->ring))
		ispfe->head = 0;
	dma_wmb();
	writel_relaxed(ispfe->head, ispfe->base[ISPFE_WIN_PDMA] +
		       PDMA_CTX(ispfe->active.loch) + PDMA_HEAD);

	if (!buf)
		schedule_work(&ispfe->backend_fill_work);
}

static void ispfe_backend_queue_complete(struct ispfe_device *ispfe)
{
	struct ispfe_backend_buffer *buf = NULL;

	spin_lock(&ispfe->slock);
	/* Dump credits age a real buffer too; they are backpressure, not a stall. */
	if (!list_empty(&ispfe->backend_flight)) {
		buf = list_first_entry(&ispfe->backend_flight,
				       struct ispfe_backend_buffer, list);
		if (ispfe->backend_credit_count >= buf->retire_credit) {
			list_move_tail(&buf->list, &ispfe->backend_done);
			buf->state = ISPFE_BACKEND_BUFFER_DONE;
			ispfe->backend_flight_count--;
		} else {
			buf = NULL;
		}
	}
	spin_unlock(&ispfe->slock);

	if (buf)
		schedule_work(&ispfe->backend_fill_work);
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
		 * frame.
		 */
		if (ispfe->owner == ISPFE_OWNER_V4L2) {
			ispfe_queue_credit(ispfe);
		} else if (ispfe->owner == ISPFE_OWNER_BACKEND) {
			ispfe_backend_queue_credit(ispfe);
		} else {
			/* Every record names one buffer; only the timing matters. */
			ispfe->head += sizeof(*ispfe->ring);
			if (ispfe->head >= PDMA_NUM_RECORDS * sizeof(*ispfe->ring))
				ispfe->head = 0;
			writel_relaxed(ispfe->head,
				       ispfe->base[ISPFE_WIN_PDMA] +
				       PDMA_CTX(ispfe->active.loch) + PDMA_HEAD);
		}
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
 * Everything the front end wants before a stream: the frame controller and
 * its LMP-IDMA AXI state, plus PDMA's per-context enables.  Ordered as the
 * vendor stack does it, including the signature word at LMP_SIGNATURE, which
 * is not configuration.
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

	writel_relaxed(FC_CFG_BUS_CTRL_VAL, core + FC_CFG_BUS_CTRL);
	writel_relaxed(FC_LMP_IDMA_MAX_BURST_VAL,
		       core + FC_LMP_IDMA0 + FC_LMP_IDMA_MAX_BURST);
	writel_relaxed(FC_LMP_IDMA_AXI_REORDER_EN_VAL,
		       core + FC_LMP_IDMA0 + FC_LMP_IDMA_AXI_REORDER_EN);
	writel_relaxed(FC_LMP_IDMA_AXCACHE_VAL,
		       core + FC_LMP_IDMA0 + FC_LMP_IDMA_AXCACHE);
	writel_relaxed(FC_LMP_IDMA_MAX_BURST_VAL,
		       core + FC_LMP_IDMA1 + FC_LMP_IDMA_MAX_BURST);
	writel_relaxed(FC_LMP_IDMA_AXI_REORDER_EN_VAL,
		       core + FC_LMP_IDMA1 + FC_LMP_IDMA_AXI_REORDER_EN);
	writel_relaxed(FC_LMP_IDMA_AXCACHE_VAL,
		       core + FC_LMP_IDMA1 + FC_LMP_IDMA_AXCACHE);
	writel_relaxed(FC_LMCS_COMN_VAL, core + FC_LMCS_COMN);

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
 * What the recipe's buffer names resolve to for this driver's allocations.  The
 * frame destination is per program rather than per driver, because each program
 * aims one frame at one buffer.
 */
static dma_addr_t ispfe_pdma_buffer(struct ispfe_device *ispfe, u8 buffer,
				    dma_addr_t bayer, dma_addr_t backend)
{
	unsigned int index = ISPFE_BUF_TO_INDEX(buffer);

	switch (ISPFE_BUF_TO_KIND(buffer)) {
	case ISPFE_BUF_KIND_BAYER:
		return bayer;
	case ISPFE_BUF_KIND_INPUT:
		if (index < ispfe->prog->num_inputs)
			return ispfe->blocks_dma +
			       ispfe->prog->inputs[index].area_offset;
		break;
	case ISPFE_BUF_KIND_OUTPUT:
		if (index < ARRAY_SIZE(ispfe_pdma_outputs))
			return ispfe->pdma_output[index].dma;
		break;
	case ISPFE_BUF_KIND_BACKEND:
		switch (index) {
		case 0:
			return backend;
		case 1:
			return backend + ISPFE_BACKEND_IMAGE_OFFSET;
		case 2:
			return ispfe->tnr_pyramid_dma;
		}
		break;
	}

	return DMA_MAPPING_ERROR;
}

#define ISPFE_LMP_DDS_CONFIG_REG		0x00056154
#define ISPFE_LMP_RGB_SCALER_CONFIG_REG	0x00056208
#define ISPFE_LMP_SCALER_CONFIG_REG	0x00056228
#define ISPFE_LMP_FORMATTER0_CONFIG_REG	0x0005626c
#define ISPFE_LMP_BACKEND_FORMATTER_REG	0x000565cc
#define ISPFE_LMP_BACKEND_WDMA_CONFIG_REG 0x000565d8
#define ISPFE_LMP_BATCH_CONFIG_REG	0x000567d8

#define ISPFE_LMP_CAPTURED_OUTPUT_GATES	0x0000b1f8
#define ISPFE_LMP_BACKEND_OUTPUT_GATE	BIT(10)
#define ISPFE_LMP_TNR_OUTPUT_GATE	BIT(16)

static int ispfe_pdma_apply_backend_output(struct ispfe_device *ispfe,
					   const struct ispfe_pdma_cmd *cmd,
					   dma_addr_t dma, u8 *payload,
					   u32 payload_at,
					   u32 *image_lo, u32 *image_hi,
					   u32 *header_lo, u32 *header_hi)
{
	if (!ispfe->prog->patch_backend_output)
		return 0;

	switch (cmd->reg) {
	case ISPFE_LMP_BACKEND_FORMATTER_REG:
		if (cmd->len != 0x0c)
			return -EINVAL;
		put_unaligned_le32(0x000d0003, payload);
		put_unaligned_le32(0x00001000, payload + 0x04);
		put_unaligned_le32(0x00000003, payload + 0x08);
		break;
	case ISPFE_LMP_BACKEND_WDMA_CONFIG_REG:
		if (cmd->len != 0x3c)
			return -EINVAL;
		memset(payload, 0, cmd->len);
		put_unaligned_le32(0x0c301070, payload);
		put_unaligned_le32(0x00000b51, payload + 0x04);
		put_unaligned_le32(0x00000110, payload + 0x08);
		put_unaligned_le32(0x00000002, payload + 0x14);
		put_unaligned_le32(0x00000001, payload + 0x20);
		break;
	case ISPFE_LMP_BATCH_CONFIG_REG:
		if (cmd->len < 0xc8 ||
		    get_unaligned_le32(payload + 0x20) !=
		    ISPFE_LMP_CAPTURED_OUTPUT_GATES)
			return -EINVAL;
		if (upper_32_bits(dma) ||
		    upper_32_bits(dma + ISPFE_BACKEND_IMAGE_OFFSET) ||
		    upper_32_bits(ispfe->tnr_pyramid_dma))
			return -ERANGE;
		put_unaligned_le32(ISPFE_LMP_CAPTURED_OUTPUT_GATES |
				     ISPFE_LMP_BACKEND_OUTPUT_GATE |
				     ISPFE_LMP_TNR_OUTPUT_GATE,
				     payload + 0x20);
		put_unaligned_le32(lower_32_bits(dma +
						 ISPFE_BACKEND_IMAGE_OFFSET),
				     payload + 0x80);
		put_unaligned_le32(upper_32_bits(dma +
						 ISPFE_BACKEND_IMAGE_OFFSET),
				     payload + 0x84);
		put_unaligned_le32(lower_32_bits(dma), payload + 0x88);
		put_unaligned_le32(upper_32_bits(dma), payload + 0x8c);
		put_unaligned_le32(lower_32_bits(ispfe->tnr_pyramid_dma),
				     payload + 0xc0);
		put_unaligned_le32(upper_32_bits(ispfe->tnr_pyramid_dma),
				     payload + 0xc4);
		*image_lo = payload_at + 0x80;
		*image_hi = payload_at + 0x84;
		*header_lo = payload_at + 0x88;
		*header_hi = payload_at + 0x8c;
		break;
	}

	return 0;
}

/*
 * Apply the small, semantic delta from the captured ML0 profile after copying
 * a recipe payload and before relocating its addresses.  Keeping this outside
 * the generated headers preserves those files as byte-exact captures.
 */
static int ispfe_pdma_apply_ml0_profile(struct ispfe_device *ispfe,
					const struct ispfe_pdma_cmd *cmd,
					u8 *payload)
{
	if (ispfe->active_lmp_ml0_profile == ISPFE_LMP_ML0_PROFILE_CAPTURED)
		return 0;
	if (ispfe->active_lmp_ml0_profile != ISPFE_LMP_ML0_PROFILE_1024X768)
		return -EINVAL;

	switch (cmd->reg) {
	case ISPFE_LMP_SCALER_CONFIG_REG:
		if (cmd->len < 0x34)
			return -EINVAL;
		/* Destination 0, followed by its horizontal and vertical Q21 factors. */
		put_unaligned_le32(ISPFE_LMP_ML0_1024_WIDTH |
				     ISPFE_LMP_ML0_1024_HEIGHT << 16,
				     payload + 0x20);
		put_unaligned_le32(ISPFE_LMP_ML0_1024_H_FACTOR,
				     payload + 0x2c);
		put_unaligned_le32(ISPFE_LMP_ML0_1024_V_FACTOR,
				     payload + 0x30);
		break;
	case ISPFE_LMP_FORMATTER0_CONFIG_REG:
		if (cmd->len < 0x28)
			return -EINVAL;
		put_unaligned_le32(ISPFE_LMP_ML0_1024_STRIDE, payload + 0x20);
		put_unaligned_le32(ISPFE_LMP_ML0_1024_STRIDE, payload + 0x24);
		break;
	case ISPFE_LMP_BATCH_CONFIG_REG:
		if (cmd->len < 0x10)
			return -EINVAL;
		put_unaligned_le32(ISPFE_LMP_ML0_1024_Y_SIZE, payload + 0x08);
		put_unaligned_le32(ISPFE_LMP_ML0_1024_PLANE2, payload + 0x0c);
		break;
	}

	return 0;
}

static int ispfe_pdma_prepare_ml0_lut(struct ispfe_device *ispfe)
{
	const struct ispfe_pdma_input *input;

	if (ispfe->active_lmp_ml0_profile == ISPFE_LMP_ML0_PROFILE_CAPTURED)
		return 0;
	if (ispfe->active_lmp_ml0_profile != ISPFE_LMP_ML0_PROFILE_1024X768 ||
	    ispfe->prog->num_inputs <= ISPFE_LMP_ML0_SCALER_INPUT)
		return -EINVAL;

	input = &ispfe->prog->inputs[ISPFE_LMP_ML0_SCALER_INPUT];
	if (input->size < sizeof(ispfe_lmp_ml0_1024_lut) ||
	    input->area_offset + sizeof(ispfe_lmp_ml0_1024_lut) >
	    ispfe->prog->blocks_bytes)
		return -EINVAL;

	memcpy((u8 *)ispfe->blocks + input->area_offset,
	       ispfe_lmp_ml0_1024_lut, sizeof(ispfe_lmp_ml0_1024_lut));
	return 0;
}

/* Serialised size of the recipe, which the encoder writes in one pass. */
static size_t ispfe_pdma_recipe_bytes(const struct ispfe_pdma_program *prog)
{
	size_t total = 0;
	unsigned int i;

	for (i = 0; i < prog->num_cmds; i++) {
		const struct ispfe_pdma_cmd *cmd = &prog->cmds[i];

		switch (cmd->op) {
		case ISPFE_PDMA_SELECT_BLOCK:
			total += PDMA_CMD_SELECT_BLOCK_SIZE;
			break;
		case ISPFE_PDMA_INDIRECT_BURST:
			total += PDMA_CMD_INDIRECT_BURST_SIZE;
			break;
		case ISPFE_PDMA_GROUPED_WRITE:
			total += PDMA_CMD_GROUPED_WRITE_HEAD + cmd->len;
			break;
		case ISPFE_PDMA_INLINE_BURST:
			total += PDMA_CMD_INLINE_BURST_HEAD + cmd->len;
			break;
		default:
			return 0;
		}
	}

	return total;
}

static bool ispfe_pdma_reloc_byte(const struct ispfe_pdma_program *prog,
				  unsigned int command, size_t byte)
{
	unsigned int i;

	for (i = 0; i < prog->num_relocs; i++) {
		const struct ispfe_pdma_reloc *reloc = &prog->relocs[i];

		if (reloc->cmd != command)
			continue;
		if ((byte >= reloc->lo && byte < reloc->lo + sizeof(u32)) ||
		    (reloc->hi != ISPFE_PDMA_RELOC_NO_HIGH &&
		     byte >= reloc->hi && byte < reloc->hi + sizeof(u32)))
			return true;
	}

	return false;
}

/* The address-free fields needed for the current scaler experiment. */
static bool ispfe_pdma_geometry_byte(const struct ispfe_pdma_cmd *cmd,
				     size_t byte)
{
	switch (cmd->reg) {
	case ISPFE_LMP_RGB_SCALER_CONFIG_REG:
		return (byte >= 0x08 && byte < 0x10) ||
		       (byte >= 0x18 && byte < 0x1c);
	case ISPFE_LMP_SCALER_CONFIG_REG:
		return (byte >= 0x1c && byte < 0x24) ||
		       (byte >= 0x2c && byte < 0x34) ||
		       (byte >= 0x3c && byte < 0x44);
	case ISPFE_LMP_FORMATTER0_CONFIG_REG:
		return byte >= 0x20 && byte < 0x28;
	case ISPFE_LMP_BATCH_CONFIG_REG:
		return byte >= 0x08 && byte < 0x10;
	default:
		return false;
	}
}

#define ISPFE_LMP_SCALER_FACTOR_MAX	GENMASK(23, 0)

static u64 ispfe_pdma_scale_factor(u32 source, u32 destination)
{
	return div_u64((u64)source << 21, destination);
}

static bool ispfe_pdma_scale_factor_matches(const u8 *payload, size_t offset,
					     u32 source, u32 destination)
{
	u64 factor = ispfe_pdma_scale_factor(source, destination);

	return factor <= ISPFE_LMP_SCALER_FACTOR_MAX &&
	       get_unaligned_le32(payload + offset) == factor;
}

static int ispfe_pdma_staged_geometry_validate(const u8 *dds, const u8 *rgb,
						const u8 *scaler,
						const u8 *formatter,
						const u8 *batch)
{
	u32 flags = get_unaligned_le32(dds);
	u32 source = get_unaligned_le32(dds + 8);
	u32 input = get_unaligned_le32(scaler + 0x1c);
	u32 destination = get_unaligned_le32(scaler + 0x20);
	u32 output2 = get_unaligned_le32(scaler + 0x28);
	u32 width = source & U16_MAX, height = source >> 16;
	u32 input_width, input_height, dest_width, dest_height;
	u32 rgb_width, rgb_height, rgb_destination;
	u32 y_stride = get_unaligned_le32(formatter + 0x20);
	u32 uv_stride = get_unaligned_le32(formatter + 0x24);
	u32 uv_offset = get_unaligned_le32(batch + 0x08);
	u32 plane2 = get_unaligned_le32(batch + 0x0c);
	u32 divisor = 1 << (((flags >> 8) & 3) + 1);
	u64 y_size, total_size;

	if (!width || !height || width % divisor || height % divisor)
		return -EINVAL;
	input_width = width / divisor;
	input_height = height / divisor;
	if (input != (input_width | input_height << 16))
		return -EINVAL;

	dest_width = destination & U16_MAX;
	dest_height = destination >> 16;
	if (!dest_width || !dest_height || dest_width & 1 || dest_height & 1 ||
	    dest_width > input_width || dest_height > input_height ||
	    y_stride != uv_stride || y_stride < dest_width || y_stride % 32 ||
	    !ispfe_pdma_scale_factor_matches(scaler, 0x2c, input_width,
					     dest_width) ||
	    !ispfe_pdma_scale_factor_matches(scaler, 0x30, input_height,
					     dest_height))
		return -EINVAL;

	y_size = (u64)y_stride * dest_height;
	total_size = y_size + (u64)uv_stride * dest_height / 2;
	if (uv_offset != y_size || plane2 != y_size * 2 ||
	    total_size > ISPFE_LMP_ML0_MAX_SIZE)
		return -EINVAL;

	/* Output 2 stays enabled, so its factors must follow the new input too. */
	dest_width = output2 & U16_MAX;
	dest_height = output2 >> 16;
	if (!dest_width || !dest_height ||
	    !ispfe_pdma_scale_factor_matches(scaler, 0x3c, input_width,
					     dest_width) ||
	    !ispfe_pdma_scale_factor_matches(scaler, 0x40, input_height,
					     dest_height))
		return -EINVAL;

	/* The enabled linear-RGB branch has one automatic 2x pre-bin stage. */
	rgb_width = input_width;
	rgb_height = input_height;
	if (rgb_width > 1024 || rgb_height > 768) {
		if (rgb_width & 1 || rgb_height & 1)
			return -EINVAL;
		rgb_width /= 2;
		rgb_height /= 2;
	}
	if (get_unaligned_le32(rgb + 0x18) !=
	    (rgb_width | rgb_height << 16))
		return -EINVAL;
	rgb_destination = get_unaligned_le32(rgb + 4);
	dest_width = rgb_destination & U16_MAX;
	dest_height = rgb_destination >> 16;
	if (!dest_width || !dest_height ||
	    !ispfe_pdma_scale_factor_matches(rgb, 0x08, rgb_width,
					     dest_width) ||
	    !ispfe_pdma_scale_factor_matches(rgb, 0x0c, rgb_height,
					     dest_height))
		return -EINVAL;

	return 0;
}

/*
 * A staged program is an editable geometry template, not an executable DMA
 * stream supplied by userspace.  Keep the captured recipe's complete command
 * skeleton, grouped register words, gates and every address-capable field.
 * Only the decoded address-free geometry words above and known relocations may
 * differ.  Indirect and embedded buffer addresses are then regenerated from
 * the driver's relocation table while encoding each slot.
 */
static int ispfe_pdma_staged_validate(struct ispfe_device *ispfe)
{
	const struct ispfe_pdma_program *prog = ispfe->prog;
	const u8 *program = ispfe->pdma_program_staged;
	const u8 *dds = NULL, *rgb = NULL, *scaler = NULL;
	const u8 *formatter = NULL, *batch = NULL;
	size_t at = 0;
	unsigned int i;

	if (ispfe->pdma_program_staged_bytes != ISPFE_PDMA_RECIPE_BYTES ||
	    ispfe->pdma_blocks_staged_bytes != ISPFE_PDMA_BLOCKS_BYTES ||
	    ispfe_pdma_recipe_bytes(prog) != ISPFE_PDMA_RECIPE_BYTES)
		return -EINVAL;

	for (i = 0; i < prog->num_cmds; i++) {
		const struct ispfe_pdma_cmd *cmd = &prog->cmds[i];
		const u8 *payload;
		size_t j;

		switch (cmd->op) {
		case ISPFE_PDMA_SELECT_BLOCK:
			if (get_unaligned_le32(program + at) !=
			    PDMA_CMD_SELECT_BLOCK ||
			    get_unaligned_le32(program + at + 4) != cmd->reg)
				return -EINVAL;
			at += PDMA_CMD_SELECT_BLOCK_SIZE;
			break;
		case ISPFE_PDMA_INDIRECT_BURST:
			if (get_unaligned_le32(program + at) !=
			    PDMA_CMD_INDIRECT_BURST ||
			    get_unaligned_le32(program + at + 12) != cmd->len ||
			    get_unaligned_le32(program + at + 16) != cmd->reg)
				return -EINVAL;
			at += PDMA_CMD_INDIRECT_BURST_SIZE;
			break;
		case ISPFE_PDMA_GROUPED_WRITE:
			if (cmd->len % 8 ||
			    get_unaligned_le32(program + at) !=
			    PDMA_CMD_GROUPED_WRITE(cmd->len))
				return -EINVAL;
			at += PDMA_CMD_GROUPED_WRITE_HEAD;
			for (j = 0; j < cmd->len; j += 8)
				if (get_unaligned_le32(program + at + j) !=
				    get_unaligned_le32(cmd->payload + j))
					return -EINVAL;
			for (j = 0; j < cmd->len; j++)
				if (!ispfe_pdma_reloc_byte(prog, i, j) &&
				    program[at + j] != cmd->payload[j])
					return -EINVAL;
			at += cmd->len;
			break;
		case ISPFE_PDMA_INLINE_BURST:
			if (get_unaligned_le32(program + at) !=
			    PDMA_CMD_INLINE_BURST(cmd->len) ||
			    get_unaligned_le32(program + at + 4) != cmd->reg)
				return -EINVAL;
			at += PDMA_CMD_INLINE_BURST_HEAD;
			payload = program + at;
			for (j = 0; j < cmd->len; j++)
				if (!ispfe_pdma_reloc_byte(prog, i, j) &&
				    !ispfe_pdma_geometry_byte(cmd, j) &&
				    payload[j] != cmd->payload[j])
					return -EINVAL;
			switch (cmd->reg) {
			case ISPFE_LMP_DDS_CONFIG_REG:
				dds = payload;
				/*
				 * Both vendor recipes deliver 1052x780 here.  The
				 * full-mode /2 experiment instead delivered 2104x1560
				 * and left LMP unable to quiesce [HW 2026-08-20].
				 * DDS is therefore part of the captured hardware
				 * envelope, not editable output geometry.
				 */
				if (get_unaligned_le32(payload) !=
				    get_unaligned_le32(cmd->payload))
					return -EINVAL;
				break;
			case ISPFE_LMP_RGB_SCALER_CONFIG_REG:
				rgb = payload;
				break;
			case ISPFE_LMP_SCALER_CONFIG_REG:
				scaler = payload;
				break;
			case ISPFE_LMP_FORMATTER0_CONFIG_REG:
				formatter = payload;
				break;
			case ISPFE_LMP_BATCH_CONFIG_REG:
				batch = payload;
				break;
			}
			at += cmd->len;
			break;
		default:
			return -EINVAL;
		}
	}

	if (at != ISPFE_PDMA_RECIPE_BYTES || !dds || !rgb || !scaler ||
	    !formatter || !batch)
		return -EINVAL;

	return ispfe_pdma_staged_geometry_validate(dds, rgb, scaler, formatter,
						    batch);
}

/*
 * Build the program PDMA fetches at each frame boundary.  The recipe carries
 * opcodes, target registers and payloads; every address in it is named rather
 * than stored, so what reaches the hardware is always this driver's own
 * mapping.  The earlier bring-up step replayed a captured byte template and
 * substituted addresses by searching the whole program for their captured
 * values, which cannot distinguish a destination from a tuning word that
 * happens to equal one.
 *
 * Sizes are checked before anything is written, so the writes below are known
 * to stay inside the program area.
 */
static int ispfe_pdma_encode(struct ispfe_device *ispfe, unsigned int slot,
			     dma_addr_t bayer, dma_addr_t backend)
{
	const struct ispfe_pdma_program *prog = ispfe->prog;
	const struct ispfe_pdma_reloc *reloc = prog->relocs;
	const struct ispfe_pdma_reloc *last = prog->relocs + prog->num_relocs;
	u32 bayer_lo = 0, bayer_hi = 0;
	u32 backend_image_lo = 0, backend_image_hi = 0;
	u32 backend_header_lo = 0, backend_header_hi = 0;
	unsigned int i;
	u8 *program;
	size_t at = 0;
	int ret;

	if (slot >= PDMA_SLOTS)
		return -EINVAL;

	program = ispfe->programs + slot * PDMA_SLOT_STRIDE;

	if (ispfe_pdma_recipe_bytes(prog) != prog->recipe_bytes ||
	    prog->recipe_bytes > PDMA_SLOT_STRIDE) {
		dev_err(ispfe->dev, "PDMA recipe does not serialise to %#x bytes\n",
			prog->recipe_bytes);
		return -EINVAL;
	}

	for (i = 0; i < prog->num_cmds; i++) {
		const struct ispfe_pdma_cmd *cmd = &prog->cmds[i];
		dma_addr_t dma;

		switch (cmd->op) {
		case ISPFE_PDMA_SELECT_BLOCK:
			put_unaligned_le32(PDMA_CMD_SELECT_BLOCK, program + at);
			put_unaligned_le32(cmd->reg, program + at + 4);
			at += PDMA_CMD_SELECT_BLOCK_SIZE;
			continue;
		case ISPFE_PDMA_INDIRECT_BURST:
			dma = ispfe_pdma_buffer(ispfe, cmd->buffer, bayer,
						backend);
			if (dma == DMA_MAPPING_ERROR)
				return -EINVAL;
			/*
			 * This is the one length the device reads for itself,
			 * so bound it against the block rather than trusting
			 * the table to agree with it.
			 */
			if (ISPFE_BUF_TO_KIND(cmd->buffer) != ISPFE_BUF_KIND_INPUT ||
			    cmd->len > prog->inputs[ISPFE_BUF_TO_INDEX(cmd->buffer)].size)
				return -EINVAL;
			put_unaligned_le32(PDMA_CMD_INDIRECT_BURST,
					   program + at);
			put_unaligned_le32(lower_32_bits(dma), program + at + 4);
			put_unaligned_le32(upper_32_bits(dma), program + at + 8);
			put_unaligned_le32(cmd->len, program + at + 12);
			put_unaligned_le32(cmd->reg, program + at + 16);
			at += PDMA_CMD_INDIRECT_BURST_SIZE;
			continue;
		case ISPFE_PDMA_GROUPED_WRITE:
			put_unaligned_le32(PDMA_CMD_GROUPED_WRITE(cmd->len),
					   program + at);
			at += PDMA_CMD_GROUPED_WRITE_HEAD;
			break;
		case ISPFE_PDMA_INLINE_BURST:
			put_unaligned_le32(PDMA_CMD_INLINE_BURST(cmd->len),
					   program + at);
			put_unaligned_le32(cmd->reg, program + at + 4);
			at += PDMA_CMD_INLINE_BURST_HEAD;
			break;
		default:
			return -EINVAL;
		}

		if (ispfe->active_pdma_program_override)
			memcpy(program + at,
			       ispfe->pdma_program_staged + at, cmd->len);
		else {
			memcpy(program + at, cmd->payload, cmd->len);
			ret = ispfe_pdma_apply_ml0_profile(ispfe, cmd,
						   program + at);
			if (ret)
				return ret;
		}
		ret = ispfe_pdma_apply_backend_output(ispfe, cmd, backend,
						      program + at, at,
						      &backend_image_lo,
						      &backend_image_hi,
						      &backend_header_lo,
						      &backend_header_hi);
		if (ret)
			return ret;

		for (; reloc < last && reloc->cmd == i; reloc++) {
			if (reloc->lo + 4 > cmd->len ||
			    (reloc->hi != ISPFE_PDMA_RELOC_NO_HIGH &&
			     reloc->hi + 4 > cmd->len))
				return -EINVAL;
			dma = ispfe_pdma_buffer(ispfe, reloc->buffer, bayer,
						backend);
			if (dma == DMA_MAPPING_ERROR)
				return -EINVAL;
			put_unaligned_le32(lower_32_bits(dma),
					   program + at + reloc->lo);
			if (reloc->hi != ISPFE_PDMA_RELOC_NO_HIGH)
				put_unaligned_le32(upper_32_bits(dma),
						   program + at + reloc->hi);
			if (reloc->buffer == ISPFE_BUF_BAYER) {
				bayer_lo = at + reloc->lo;
				bayer_hi = at + reloc->hi;
			} else if (reloc->buffer == ISPFE_BUF_BACKEND_IMAGE) {
				backend_image_lo = at + reloc->lo;
				backend_image_hi = at + reloc->hi;
			} else if (reloc->buffer == ISPFE_BUF_BACKEND_HEADER) {
				backend_header_lo = at + reloc->lo;
				backend_header_hi = at + reloc->hi;
			}
		}

		at += cmd->len;
	}

	/*
	 * Relocations are emitted in command order, so anything left over names
	 * a command the recipe no longer has.
	 */
	if (reloc != last || (prog->raw_output && !bayer_lo)) {
		dev_err(ispfe->dev, "PDMA recipe relocations do not match it\n");
		return -EINVAL;
	}
	if (prog->backend_output &&
	    (!backend_image_lo || !backend_image_hi ||
	     !backend_header_lo || !backend_header_hi)) {
		dev_err(ispfe->dev, "PDMA recipe has no back-end output addresses\n");
		return -EINVAL;
	}

	/*
	 * The same for every slot, because every slot encodes the same recipe.
	 * Derived locally rather than accumulated in place: once a queue is
	 * encoding buffers this runs concurrently with itself, and a walk that
	 * cleared the device's copy on entry would let one encode fail the
	 * check below on another's zero.
	 */
	ispfe->bayer_lo = bayer_lo;
	ispfe->bayer_hi = bayer_hi;
	ispfe->backend_image_lo = backend_image_lo;
	ispfe->backend_image_hi = backend_image_hi;
	ispfe->backend_header_lo = backend_header_lo;
	ispfe->backend_header_hi = backend_header_hi;

	return 0;
}

static void ispfe_backend_queue_fill(struct ispfe_device *ispfe)
{
	for (;;) {
		struct ispfe_backend_buffer *buf;
		int ret;

		spin_lock_irq(&ispfe->slock);
		if (!ispfe->backend_queue_active ||
		    list_empty(&ispfe->backend_done)) {
			spin_unlock_irq(&ispfe->slock);
			break;
		}
		buf = list_first_entry(&ispfe->backend_done,
				       struct ispfe_backend_buffer, list);
		list_del_init(&buf->list);
		buf->state = ISPFE_BACKEND_BUFFER_PREPARING;
		spin_unlock_irq(&ispfe->slock);

		ret = exynos_becore_input_producer_complete(ispfe->backend_input,
							    &buf->ticket);
		if (ret)
			exynos_becore_input_producer_abort(ispfe->backend_input,
							   &buf->ticket);

		spin_lock_irq(&ispfe->slock);
		__clear_bit(buf->program_slot,
			    &ispfe->backend_programs_used);
		buf->state = ISPFE_BACKEND_BUFFER_IDLE;
		if (ret)
			ispfe->backend_queue_error = ret;
		else
			ispfe->backend_completed++;
		spin_unlock_irq(&ispfe->slock);
	}

	for (;;) {
		struct ispfe_backend_buffer *buf = NULL;
		unsigned int program_slot;
		unsigned int i;
		bool acquired = false;
		int ret;

		spin_lock_irq(&ispfe->slock);
		if (!ispfe->backend_queue_active ||
		    ispfe->backend_queue_error) {
			spin_unlock_irq(&ispfe->slock);
			return;
		}
		program_slot = find_first_zero_bit(&ispfe->backend_programs_used,
						   PDMA_BUF_SLOTS);
		if (program_slot >= PDMA_BUF_SLOTS) {
			spin_unlock_irq(&ispfe->slock);
			return;
		}
		for (i = 0; i < ARRAY_SIZE(ispfe->backend_buffers); i++)
			if (ispfe->backend_buffers[i].state ==
			    ISPFE_BACKEND_BUFFER_IDLE) {
				buf = &ispfe->backend_buffers[i];
				break;
			}
		if (!buf) {
			spin_unlock_irq(&ispfe->slock);
			return;
		}
		__set_bit(program_slot, &ispfe->backend_programs_used);
		buf->program_slot = program_slot;
		buf->state = ISPFE_BACKEND_BUFFER_PREPARING;
		spin_unlock_irq(&ispfe->slock);

		ret = exynos_becore_input_producer_acquire(ispfe->backend_input,
							   &buf->ticket);
		if (!ret) {
			acquired = true;
			ret = ispfe_pdma_encode(ispfe, program_slot,
						ispfe->frame_dma,
						buf->ticket.dma);
		}
		if (ret && acquired)
			exynos_becore_input_producer_abort(ispfe->backend_input,
							   &buf->ticket);

		spin_lock_irq(&ispfe->slock);
		if (ret || !ispfe->backend_queue_active) {
			__clear_bit(program_slot,
				    &ispfe->backend_programs_used);
			buf->state = ISPFE_BACKEND_BUFFER_IDLE;
			if (!ret) {
				spin_unlock_irq(&ispfe->slock);
				exynos_becore_input_producer_abort(ispfe->backend_input,
								   &buf->ticket);
				return;
			}
			if (ret != -EBUSY)
				ispfe->backend_queue_error = ret;
			spin_unlock_irq(&ispfe->slock);
			return;
		}
		buf->state = ISPFE_BACKEND_BUFFER_READY;
		list_add_tail(&buf->list, &ispfe->backend_ready);
		spin_unlock_irq(&ispfe->slock);
	}
}

static void ispfe_backend_fill_work(struct work_struct *work)
{
	struct ispfe_device *ispfe =
		container_of(work, struct ispfe_device, backend_fill_work);

	ispfe_backend_queue_fill(ispfe);
}

/*
 * Stage the shared blocks and encode one program per slot.  Every slot starts
 * out aimed at the primary frame buffer; a queue retargets them per buffer.
 */
static int ispfe_pdma_program_prepare(struct ispfe_device *ispfe)
{
	dma_addr_t backend = ispfe->owner == ISPFE_OWNER_BACKEND ?
			     ispfe->backend_spare_dma :
			     ispfe->backend_buffer.dma;
	unsigned int i;
	int ret;

	if (upper_32_bits(ispfe->blocks_dma + ISPFE_PDMA_MAX_BLOCKS_BYTES - 1) ||
	    upper_32_bits(ispfe->programs_dma + PDMA_PROGRAMS_SIZE - 1) ||
	    upper_32_bits(ispfe->frame_dma + ispfe->frame_size - 1) ||
	    upper_32_bits(ispfe->spare_frame_dma + ispfe->frame_size - 1))
		return -ERANGE;

	for (i = 0; i < ARRAY_SIZE(ispfe_pdma_outputs); i++)
		if (upper_32_bits(ispfe->pdma_output[i].dma +
				  ispfe_pdma_outputs[i].size - 1))
			return -ERANGE;

	if (ispfe->active_pdma_program_override) {
		memset(ispfe->blocks, 0, ISPFE_PDMA_MAX_BLOCKS_BYTES);
		memcpy(ispfe->blocks, ispfe->pdma_blocks_staged,
		       ISPFE_PDMA_BLOCKS_BYTES);
	} else {
		memset(ispfe->blocks, 0, ISPFE_PDMA_MAX_BLOCKS_BYTES);
		for (i = 0; i < ispfe->prog->num_inputs; i++) {
			const struct ispfe_pdma_input *input =
				&ispfe->prog->inputs[i];

			if (input->area_offset + input->size >
			    ispfe->prog->blocks_bytes)
				return -EOVERFLOW;
			memcpy((u8 *)ispfe->blocks + input->area_offset,
			       input->data, input->size);
		}
		ret = ispfe_pdma_prepare_ml0_lut(ispfe);
		if (ret)
			return ret;
	}

	memset(ispfe->programs, 0, PDMA_PROGRAMS_SIZE);
	for (i = 0; i < PDMA_SLOTS; i++) {
		ret = ispfe_pdma_encode(ispfe, i,
					i == PDMA_DUMP_SLOT ?
					ispfe->spare_frame_dma :
					ispfe->frame_dma,
					backend);
		if (ret)
			return ret;
	}

	return 0;
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
static void ispfe_ring_fill(struct ispfe_device *ispfe)
{
	unsigned int slot = ispfe->owner == ISPFE_OWNER_V4L2 ||
			    ispfe->owner == ISPFE_OWNER_BACKEND ?
			    PDMA_DUMP_SLOT : 0;
	u32 addr = READ_ONCE(ispfe->pdma_addr);
	unsigned int i;

	/*
	 * A record the producer has not refreshed still gets consumed, so what
	 * it names matters.  Under the queue that is the dump slot, which puts
	 * an unclaimed frame somewhere nobody reads; the diagnostic keeps its
	 * single program, whose frame is the whole point of the exercise.
	 */
	for (i = 0; i < PDMA_NUM_RECORDS; i++) {
		ispfe_ring_record(ispfe, i, slot);
		/* A deliberate address, for proving PDMA issues the read at all. */
		if (addr) {
			ispfe->ring[i].addr_lo = cpu_to_le32(addr);
			ispfe->ring[i].addr_hi = cpu_to_le32(0);
		}
		/* Keep the separate first-record control for parser experiments. */
		if (!i && !ispfe->active_backend_recipe)
			ispfe->ring[i].bytes =
				cpu_to_le32(READ_ONCE(ispfe->pdma_bytes_first));
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
	void __iomem *idma0 = core + FC_LMP_IDMA0;
	unsigned int i;

	writel_relaxed(FC_CTX_CTRL_VAL, bank + FC_CTX_CTRL);
	writel_relaxed(FC_CTX_MSK_ARM, bank + FC_CTX_MSK);

	writel_relaxed(FC_LMP_IDMA_AXI_THRE_HIGH_VAL,
		       idma0 + FC_LMP_IDMA_AXI_THRE(ispfe->active.fcctx));
	writel_relaxed(FC_LMP_IDMA_AXI_THRE_LOW_VAL,
		       idma0 + FC_LMP_IDMA_AXI_THRE(ispfe->active.fcctx) + 4);
	writel_relaxed(FC_LMP_IDMA_AXI_THRE_HIGH_VAL,
		       core + FC_LMP_IDMA1 +
		       FC_LMP_IDMA_AXI_THRE(ispfe->active.fcctx));
	writel_relaxed(FC_LMP_IDMA_AXI_THRE_LOW_VAL,
		       core + FC_LMP_IDMA1 +
		       FC_LMP_IDMA_AXI_THRE(ispfe->active.fcctx) + 4);
	writel_relaxed(READ_ONCE(ispfe->fc_axi_max_ost),
		       idma0 + FC_LMP_IDMA_AXI_MAX_OST);
	writel_relaxed(FC_LMP_IDMA1_AXI_MAX_OST_VAL,
		       core + FC_LMP_IDMA1 + FC_LMP_IDMA_AXI_MAX_OST);

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
	 * The arm belongs to the CSIS Bayer WDMA, not to the logical channel as
	 * a whole.  A raw program configures that WDMA before committing it;
	 * ordinary LMP/back-end programs leave its config-enable clear.  Arming
	 * the latter raises and latches wdma_config_miss on the first frame.
	 */
	if (ispfe->prog->raw_output)
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
	 * And both FC LMP-IDMA max-outstanding words back to what they read with
	 * nothing streaming.  Their encoding remains unknown; this is only the
	 * value the vendor writes on the way out.
	 */
	writel_relaxed(FC_LMP_IDMA_AXI_MAX_OST_IDLE,
		       core + FC_LMP_IDMA0 + FC_LMP_IDMA_AXI_MAX_OST);
	writel_relaxed(FC_LMP_IDMA_AXI_MAX_OST_IDLE,
		       core + FC_LMP_IDMA1 + FC_LMP_IDMA_AXI_MAX_OST);
	writel_relaxed(0, core + FC_CFG_BUS_CTRL);
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

static void ispfe_backend_retarget(struct ispfe_device *ispfe, u8 *program,
				   dma_addr_t dma)
{
	put_unaligned_le32(lower_32_bits(dma + ISPFE_BACKEND_IMAGE_OFFSET),
			   program + ispfe->backend_image_lo);
	put_unaligned_le32(upper_32_bits(dma + ISPFE_BACKEND_IMAGE_OFFSET),
			   program + ispfe->backend_image_hi);
	put_unaligned_le32(lower_32_bits(dma),
			   program + ispfe->backend_header_lo);
	put_unaligned_le32(upper_32_bits(dma),
			   program + ispfe->backend_header_hi);
}

static void ispfe_snapshot_complete(struct ispfe_device *ispfe)
{
	u8 *program = ispfe->programs;

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
		if (ispfe->prog->raw_output && WARN_ON_ONCE(!ispfe->bayer_lo)) {
			cmpxchg(&ispfe->snapshot_state,
				ISPFE_SNAPSHOT_REDIRECTING, ISPFE_SNAPSHOT_IDLE);
			return;
		}
		if (ispfe->prog->raw_output) {
			put_unaligned_le32(lower_32_bits(ispfe->spare_frame_dma),
					   program + ispfe->bayer_lo);
			put_unaligned_le32(upper_32_bits(ispfe->spare_frame_dma),
					   program + ispfe->bayer_hi);
		}
		if (ispfe->prog->backend_output) {
			if (WARN_ON_ONCE(!ispfe->backend_image_lo ||
					 !ispfe->backend_header_lo)) {
				cmpxchg(&ispfe->snapshot_state,
					ISPFE_SNAPSHOT_REDIRECTING,
					ISPFE_SNAPSHOT_IDLE);
				return;
			}
			ispfe_backend_retarget(ispfe, program,
					       ispfe->backend_spare_dma);
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
	if (target_done) {
		if (ispfe->owner == ISPFE_OWNER_V4L2) {
			ispfe->sequence++;
			ispfe_queue_complete(ispfe);
		} else if (ispfe->owner == ISPFE_OWNER_BACKEND) {
			ispfe->sequence++;
			ispfe_backend_queue_complete(ispfe);
		} else {
			ispfe_snapshot_complete(ispfe);
		}
	}

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

	if (ispfe->tnr_pyramid) {
		dma_free_coherent(ispfe->dev, ISPFE_TNR_PYRAMID_SIZE,
				  ispfe->tnr_pyramid, ispfe->tnr_pyramid_dma);
		ispfe->tnr_pyramid = NULL;
	}
	for (i = 0; i < ARRAY_SIZE(ispfe_pdma_outputs); i++) {
		if (!ispfe->pdma_output[i].cpu)
			continue;
		dma_free_coherent(ispfe->dev, ispfe_pdma_outputs[i].size,
				  ispfe->pdma_output[i].cpu,
				  ispfe->pdma_output[i].dma);
		ispfe->pdma_output[i].cpu = NULL;
	}
	if (ispfe->programs) {
		dma_free_coherent(ispfe->dev, PDMA_PROGRAMS_SIZE,
				  ispfe->programs, ispfe->programs_dma);
		ispfe->programs = NULL;
		ispfe->bayer_lo = 0;
		ispfe->bayer_hi = 0;
		ispfe->backend_image_lo = 0;
		ispfe->backend_image_hi = 0;
		ispfe->backend_header_lo = 0;
		ispfe->backend_header_hi = 0;
	}
	if (ispfe->blocks) {
		dma_free_coherent(ispfe->dev, ISPFE_PDMA_MAX_BLOCKS_BYTES,
				  ispfe->blocks, ispfe->blocks_dma);
		ispfe->blocks = NULL;
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
	    !ispfe->programs || !ispfe->blocks)
		return false;
	if (ispfe->prog->backend_output && !ispfe->tnr_pyramid)
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
	size_t size = size_mul(ispfe->prog->stride, ispfe->active.height);
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
	ispfe->blocks = dma_alloc_coherent(ispfe->dev,
					   ISPFE_PDMA_MAX_BLOCKS_BYTES,
					   &ispfe->blocks_dma, GFP_KERNEL);
	if (!ispfe->blocks) {
		ispfe_buffers_free(ispfe);
		return -ENOMEM;
	}
	ispfe->programs = dma_alloc_coherent(ispfe->dev, PDMA_PROGRAMS_SIZE,
					     &ispfe->programs_dma, GFP_KERNEL);
	if (!ispfe->programs) {
		ispfe_buffers_free(ispfe);
		return -ENOMEM;
	}
	if (ispfe->prog->backend_output) {
		ispfe->tnr_pyramid = dma_alloc_coherent(
			ispfe->dev, ISPFE_TNR_PYRAMID_SIZE,
			&ispfe->tnr_pyramid_dma, GFP_KERNEL);
		if (!ispfe->tnr_pyramid) {
			ispfe_buffers_free(ispfe);
			return -ENOMEM;
		}
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
 * The downstream session has two distinct requirements: ISPFE's measured
 * 974745 KB/s real-time memory vote, and a short CAM-domain pulse while the
 * receiver is configured.  Keep ownership explicit so that a failed unwind
 * remains retryable instead of silently becoming the next session's idle
 * baseline.
 */
static int ispfe_qos_disable(struct ispfe_device *ispfe)
{
	int cam_ret = 0;
	int memory_ret = 0;

	if (ispfe->cam_rate_active) {
		cam_ret = clk_set_rate(ispfe->cam_clk, ispfe->saved_cam_rate);
		if (!cam_ret)
			ispfe->cam_rate_active = false;
	}
	if (ispfe->memory_vote_active) {
		memory_ret = icc_set_bw(ispfe->memory_path, 0, 0);
		if (!memory_ret)
			ispfe->memory_vote_active = false;
	}
	if (!ispfe->cam_rate_active && !ispfe->memory_vote_active)
		ispfe->saved_cam_rate = 0;

	return cam_ret ?: memory_ret;
}

static int ispfe_qos_enable(struct ispfe_device *ispfe)
{
	int cleanup_ret;
	int ret;

	/* Retry a restore left incomplete by the preceding stream. */
	if (ispfe->cam_rate_active || ispfe->memory_vote_active) {
		ret = ispfe_qos_disable(ispfe);
		if (ret)
			return ret;
	}

	ispfe->saved_cam_rate = clk_get_rate(ispfe->cam_clk);
	if (!ispfe->saved_cam_rate)
		return -EIO;

	ret = icc_set_bw(ispfe->memory_path, ISPFE_MEMORY_BW_KBPS,
			 ISPFE_MEMORY_BW_KBPS);
	if (ret)
		goto err_clear;
	ispfe->memory_vote_active = true;

	/* A transport error can arrive after firmware accepted the request. */
	ispfe->cam_rate_active = true;
	ret = clk_set_rate(ispfe->cam_clk,
			   max(ispfe->saved_cam_rate, ISPFE_CAM_SETUP_RATE));
	if (ret) {
		cleanup_ret = ispfe_qos_disable(ispfe);
		if (cleanup_ret)
			dev_err(ispfe->dev,
				"cannot restore QoS after CAM error: %d\n",
				cleanup_ret);
		return ret;
	}

	return 0;

err_clear:
	if (!ispfe->cam_rate_active && !ispfe->memory_vote_active)
		ispfe->saved_cam_rate = 0;
	return ret;
}

static int ispfe_qos_set_active(struct ispfe_device *ispfe)
{
	return clk_set_rate(ispfe->cam_clk,
			    max(ispfe->saved_cam_rate, ISPFE_CAM_ACTIVE_RATE));
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
	struct v4l2_subdev_state *state;
	const struct v4l2_mbus_framefmt *fmt;
	int ret;

	/*
	 * The geometry the receiver is programmed with is the format on the
	 * subdev's source pad, read once here so that a later S_FMT cannot move
	 * it under a running stream.
	 */
	state = v4l2_subdev_lock_and_get_active_state(&ispfe->sd);
	fmt = v4l2_subdev_state_get_format(state, ISPFE_PAD_SOURCE);
	ispfe->src.width = fmt->width;
	ispfe->src.height = fmt->height;
	v4l2_subdev_unlock_state(state);

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
	    ispfe->lmp_ml0_profile >= ISPFE_LMP_ML0_PROFILE_COUNT ||
	    ispfe->backend_recipe > 1 ||
	    ispfe->src.width - 1 >= U16_MAX || ispfe->src.height - 1 >= U16_MAX)
		return -EINVAL;

	ispfe->prog = ispfe_program_for(ispfe->src.width, ispfe->src.height,
					ispfe->backend_recipe);
	if (!ispfe->prog) {
		dev_err(ispfe->dev,
			"no PDMA recipe captured for %ux%u\n",
			ispfe->src.width, ispfe->src.height);
		return -EINVAL;
	}
	if (ispfe->prog->backend_recipe && ispfe->owner == ISPFE_OWNER_V4L2)
		return -EOPNOTSUPP;
	if (ispfe->prog->fixed_resources &&
	    (ispfe->src.loch != ispfe->prog->required_loch ||
	     ispfe->src.fcctx != ispfe->prog->required_fcctx ||
	     ispfe->fc_axi_max_ost != ispfe->prog->required_fc_axi_max_ost)) {
		dev_err(ispfe->dev,
			"PDMA recipe needs loch %u, FC %u, AXI max OST %#x\n",
			ispfe->prog->required_loch, ispfe->prog->required_fcctx,
			ispfe->prog->required_fc_axi_max_ost);
		return -EINVAL;
	}
	if (ispfe->prog->backend_recipe &&
	    ispfe->lmp_ml0_profile != ISPFE_LMP_ML0_PROFILE_CAPTURED)
		return -EINVAL;
	if (ispfe->pdma_program_override) {
		if (ispfe->prog->backend_recipe)
			return -EINVAL;
		/* A staged program already contains its final profile payload. */
		if (ispfe->lmp_ml0_profile != ISPFE_LMP_ML0_PROFILE_CAPTURED)
			return -EINVAL;
		ret = ispfe_pdma_staged_validate(ispfe);
		if (ret) {
			dev_err(ispfe->dev,
				"staged PDMA program does not match the %ux%u recipe\n",
				ispfe->src.width, ispfe->src.height);
			return ret;
		}
	}

	ispfe->active = ispfe->src;
	ispfe->active_lmp_ml0_profile = ispfe->lmp_ml0_profile;
	ispfe->active_backend_recipe = ispfe->prog->backend_recipe;
	ispfe->active_pdma_program_override = ispfe->pdma_program_override;
	ispfe->active_pdma_program_generation =
		ispfe->active_pdma_program_override ?
		ispfe->pdma_program_staged_generation : 0;
	ispfe->active_pdma_blocks_generation =
		ispfe->active_pdma_program_override ?
		ispfe->pdma_blocks_staged_generation : 0;
	if (ispfe->backend_producing)
		return -EBUSY;

	snprintf(ispfe->link_name, sizeof(ispfe->link_name), "csis%u",
		 ispfe->active.link);
	ret = platform_get_irq_byname(pdev, ispfe->link_name);
	if (ret < 0)
		return ret;
	ispfe->link_irq = ret;

	/* Reserve the exact slot whose IOVA the program encoder will publish. */
	if (ispfe->prog->backend_output &&
	    ispfe->owner != ISPFE_OWNER_BACKEND) {
		ret = exynos_becore_input_producer_acquire(ispfe->backend_input,
							   &ispfe->backend_buffer);
		if (ret)
			return ret;
		ispfe->backend_producing = true;
		ispfe->backend_handed_off = false;
	}

	ret = ispfe_buffers_alloc(ispfe);
	if (ret)
		goto err_backend_early;

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

	ret = ispfe_qos_enable(ispfe);
	if (ret) {
		dev_err_probe(ispfe->dev, ret, "cannot establish camera QoS\n");
		goto err_backend_early;
	}

	ret = pm_runtime_resume_and_get(ispfe->dev);
	if (ret)
		goto err_qos;

	/*
	 * The nine D/C-PHYs sit behind one PMU isolation bit, and while it is
	 * set their whole register region reads zero and takes no writes -- so
	 * a stream programmed through it produces no frame start, no frame end
	 * and no error bits either, which reads like a lane or sensor fault and
	 * is neither.  Bypassing it therefore belongs to starting a stream, not
	 * to a diagnostic.  Isolation is restored in ispfe_stop() while this
	 * runtime-PM reference is still held, which is where downstream does it
	 * too, with GENPD_NOTIFY_PRE_OFF as the backstop.
	 */
	if (!ispfe->phy_isolation_bypass) {
		/*
		 * Claimed before the call rather than after it: the request can
		 * fail with the bit already set, because the readback that
		 * verifies it is a second register access.  Restoring is
		 * idempotent, so owning it from here is what makes every exit
		 * put it back.
		 */
		ispfe->phy_bypass_held = true;
		ret = ispfe_phy_isolation(ispfe, true);
		if (ret) {
			dev_err(ispfe->dev,
				"cannot bypass PHY isolation: %d\n", ret);
			goto err_isolate;
		}
	}

	/*
	 * Requested before anything is unmasked, and after the domain is up:
	 * the handlers touch the block's registers.
	 */
	ret = ispfe_request_irqs(ispfe);
	if (ret)
		goto err_isolate;

	ispfe_device_init(ispfe);
	ret = ispfe_qos_set_active(ispfe);
	if (ret) {
		dev_err(ispfe->dev, "cannot set active CAM rate: %d\n", ret);
		goto err_backend;
	}

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

err_backend:
	if (ispfe->backend_producing) {
		exynos_becore_input_producer_abort(ispfe->backend_input,
						   &ispfe->backend_buffer);
		ispfe->backend_producing = false;
		ispfe->backend_handed_off = false;
	}
	ispfe_free_irqs(ispfe);
err_isolate:
	if (ispfe->phy_bypass_held) {
		ispfe->phy_bypass_held = false;
		ispfe_phy_isolation(ispfe, false);
	}
	pm_runtime_put(ispfe->dev);
err_qos:
	if (ispfe_qos_disable(ispfe))
		dev_err(ispfe->dev, "cannot restore camera QoS after start error\n");
err_backend_early:
	if (ispfe->backend_producing) {
		exynos_becore_input_producer_abort(ispfe->backend_input,
						   &ispfe->backend_buffer);
		ispfe->backend_producing = false;
		ispfe->backend_handed_off = false;
	}
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
	int ret;

	/* Downstream gives the teardown MMIO the same short CAM pulse as setup. */
	ret = clk_set_rate(ispfe->cam_clk,
			   max(ispfe->saved_cam_rate, ISPFE_CAM_SETUP_RATE));
	if (ret)
		dev_err(ispfe->dev, "cannot set teardown CAM rate: %d\n", ret);

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
	if (ispfe->backend_producing &&
	    READ_ONCE(ispfe->snapshot_state) != ISPFE_SNAPSHOT_READY) {
		exynos_becore_input_producer_abort(ispfe->backend_input,
						   &ispfe->backend_buffer);
		ispfe->backend_producing = false;
		ispfe->backend_handed_off = false;
	}
	/*
	 * Before the runtime-PM reference goes, so that the edge is taken and
	 * read back with the domain still up.  Not done when the diagnostic
	 * control asked for the bypass, which owns it until it is cleared, and
	 * reported when it fails: the readback is the cheapest measurement this
	 * block offers and a silent disagreement is the one thing it cannot say.
	 */
	if (ispfe->phy_bypass_held) {
		ispfe->phy_bypass_held = false;
		if (ispfe_phy_isolation(ispfe, false))
			dev_err(ispfe->dev, "cannot re-isolate the PHYs\n");
	}
	if (ispfe_qos_disable(ispfe))
		dev_err(ispfe->dev, "cannot restore camera QoS after stop\n");
	ispfe->streaming = false;
	pm_runtime_put(ispfe->dev);
}

static int ispfe_sensor_power(struct ispfe_device *ispfe, bool on);

/*
 * Arms the receiver and nothing else.  It does not power or start the sensor,
 * which is deliberate: the cheapest oracle this block has is the arm flush,
 * which the frame controller raises milliseconds after its START pulse with no
 * sensor, no PHY traffic and no frame involved.  So this stays a receiver-only
 * diagnostic, and anything that needs an actual frame goes through the capture
 * queue, which owns the sensor.
 */
static int ispfe_enable_set(void *data, u64 val)
{
	struct ispfe_device *ispfe = data;
	int ret = 0;

	guard(mutex)(&ispfe->lock);

	if (ispfe->sensor_streaming)
		return -EBUSY;
	if (!!val == ispfe->streaming)
		return 0;
	/* One receive path, so the queue and the diagnostic take turns. */
	if (ispfe->owner == ISPFE_OWNER_V4L2 ||
	    ispfe->owner == ISPFE_OWNER_BACKEND)
		return -EBUSY;
	if (val) {
		ispfe->owner = ISPFE_OWNER_DEBUGFS;
		ret = ispfe_start(ispfe);
		if (ret)
			ispfe->owner = ISPFE_OWNER_NONE;
	} else {
		ispfe_stop(ispfe);
		ispfe->owner = ISPFE_OWNER_NONE;
	}

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

/* Start the real sensor behind the otherwise receiver-only diagnostic. */
static int ispfe_capture_set(void *data, u64 val)
{
	struct ispfe_device *ispfe = data;
	int ret;

	if (val > 1)
		return -EINVAL;

	guard(mutex)(&ispfe->lock);
	if (ispfe->owner == ISPFE_OWNER_V4L2 ||
	    ispfe->owner == ISPFE_OWNER_BACKEND)
		return -EBUSY;
	if (!!val == ispfe->sensor_streaming)
		return 0;

	if (val) {
		if (ispfe->streaming || ispfe->owner != ISPFE_OWNER_NONE)
			return -EBUSY;
		ispfe->owner = ISPFE_OWNER_DEBUGFS;
		ret = ispfe_sensor_power(ispfe, true);
		if (ret)
			goto err_owner;
		ret = ispfe_start(ispfe);
		if (ret)
			goto err_power;
		ret = v4l2_subdev_enable_streams(&ispfe->sd,
						 ISPFE_PAD_SOURCE, BIT_ULL(0));
		if (ret)
			goto err_stop;
		ispfe->sensor_streaming = true;
		return 0;
	}

	ispfe_stop(ispfe);
	ret = v4l2_subdev_disable_streams(&ispfe->sd, ISPFE_PAD_SOURCE,
					  BIT_ULL(0));
	if (ret)
		dev_err(ispfe->dev, "cannot stop the sensor: %d\n", ret);
	ispfe_sensor_power(ispfe, false);
	ispfe->sensor_streaming = false;
	ispfe->owner = ISPFE_OWNER_NONE;

	return 0;

err_stop:
	ispfe_stop(ispfe);
err_power:
	ispfe_sensor_power(ispfe, false);
err_owner:
	ispfe->owner = ISPFE_OWNER_NONE;
	return ret;
}

static int ispfe_capture_get(void *data, u64 *val)
{
	struct ispfe_device *ispfe = data;

	guard(mutex)(&ispfe->lock);
	*val = ispfe->sensor_streaming;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ispfe_capture_fops, ispfe_capture_get,
			 ispfe_capture_set, "%llu\n");

static void ispfe_backend_queue_reset(struct ispfe_device *ispfe)
{
	unsigned int i;

	spin_lock_irq(&ispfe->slock);
	INIT_LIST_HEAD(&ispfe->backend_ready);
	INIT_LIST_HEAD(&ispfe->backend_flight);
	INIT_LIST_HEAD(&ispfe->backend_done);
	ispfe->backend_programs_used = 0;
	ispfe->backend_flight_count = 0;
	ispfe->backend_credit_count = 0;
	ispfe->backend_completed = 0;
	ispfe->backend_dropped = 0;
	ispfe->backend_queue_error = 0;
	for (i = 0; i < ARRAY_SIZE(ispfe->backend_buffers); i++) {
		INIT_LIST_HEAD(&ispfe->backend_buffers[i].list);
		ispfe->backend_buffers[i].state = ISPFE_BACKEND_BUFFER_IDLE;
	}
	spin_unlock_irq(&ispfe->slock);
}

static void ispfe_backend_queue_abort_all(struct ispfe_device *ispfe)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ispfe->backend_buffers); i++) {
		struct ispfe_backend_buffer *buf = &ispfe->backend_buffers[i];

		if (buf->state != ISPFE_BACKEND_BUFFER_IDLE)
			exynos_becore_input_producer_abort(ispfe->backend_input,
							   &buf->ticket);
	}

	spin_lock_irq(&ispfe->slock);
	INIT_LIST_HEAD(&ispfe->backend_ready);
	INIT_LIST_HEAD(&ispfe->backend_flight);
	INIT_LIST_HEAD(&ispfe->backend_done);
	ispfe->backend_programs_used = 0;
	ispfe->backend_flight_count = 0;
	ispfe->backend_credit_count = 0;
	for (i = 0; i < ARRAY_SIZE(ispfe->backend_buffers); i++) {
		INIT_LIST_HEAD(&ispfe->backend_buffers[i].list);
		ispfe->backend_buffers[i].state = ISPFE_BACKEND_BUFFER_IDLE;
	}
	spin_unlock_irq(&ispfe->slock);
}

static int ispfe_backend_queue_set(void *data, u64 val)
{
	struct ispfe_device *ispfe = data;
	bool ready;
	int ret;

	if (val > 1)
		return -EINVAL;

	guard(mutex)(&ispfe->lock);
	if (!!val == ispfe->backend_queue_active)
		return 0;
	if (val) {
		if (ispfe->streaming || ispfe->owner != ISPFE_OWNER_NONE)
			return -EBUSY;
		if (ispfe->backend_recipe != 1)
			return -EINVAL;

		ispfe->owner = ISPFE_OWNER_BACKEND;
		ispfe_backend_queue_reset(ispfe);
		spin_lock_irq(&ispfe->slock);
		ispfe->backend_queue_active = true;
		spin_unlock_irq(&ispfe->slock);

		ret = ispfe_sensor_power(ispfe, true);
		if (ret)
			goto err_queue;
		ret = ispfe_start(ispfe);
		if (ret)
			goto err_power;

		ispfe_backend_queue_fill(ispfe);
		spin_lock_irq(&ispfe->slock);
		ready = !list_empty(&ispfe->backend_ready);
		ret = ispfe->backend_queue_error;
		spin_unlock_irq(&ispfe->slock);
		if (ret || !ready) {
			if (!ret)
				ret = -ENOBUFS;
			goto err_stop;
		}

		ret = v4l2_subdev_enable_streams(&ispfe->sd,
						 ISPFE_PAD_SOURCE, BIT_ULL(0));
		if (ret)
			goto err_stop;
		ispfe->sensor_streaming = true;
		return 0;
	}

	spin_lock_irq(&ispfe->slock);
	ispfe->backend_queue_active = false;
	spin_unlock_irq(&ispfe->slock);
	cancel_work_sync(&ispfe->backend_fill_work);
	ispfe_stop(ispfe);
	cancel_work_sync(&ispfe->backend_fill_work);
	ret = v4l2_subdev_disable_streams(&ispfe->sd, ISPFE_PAD_SOURCE,
					  BIT_ULL(0));
	if (ret)
		dev_err(ispfe->dev, "cannot stop the sensor: %d\n", ret);
	ispfe_sensor_power(ispfe, false);
	ispfe->sensor_streaming = false;
	ispfe_backend_queue_abort_all(ispfe);
	ispfe->owner = ISPFE_OWNER_NONE;

	return 0;

err_stop:
	spin_lock_irq(&ispfe->slock);
	ispfe->backend_queue_active = false;
	spin_unlock_irq(&ispfe->slock);
	cancel_work_sync(&ispfe->backend_fill_work);
	ispfe_stop(ispfe);
	cancel_work_sync(&ispfe->backend_fill_work);
	ispfe_backend_queue_abort_all(ispfe);
err_power:
	ispfe_sensor_power(ispfe, false);
err_queue:
	spin_lock_irq(&ispfe->slock);
	ispfe->backend_queue_active = false;
	spin_unlock_irq(&ispfe->slock);
	ispfe->owner = ISPFE_OWNER_NONE;
	return ret;
}

static int ispfe_backend_queue_get(void *data, u64 *val)
{
	struct ispfe_device *ispfe = data;

	guard(mutex)(&ispfe->lock);
	*val = ispfe->backend_queue_active;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ispfe_backend_queue_fops, ispfe_backend_queue_get,
			 ispfe_backend_queue_set, "%llu\n");

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
	/*
	 * The queue routes end-of-frame to buffer completion, so an armed
	 * snapshot would simply never advance -- which reads as a hang rather
	 * than as the two owners overlapping.
	 */
	if (ispfe->owner == ISPFE_OWNER_V4L2 ||
	    ispfe->owner == ISPFE_OWNER_BACKEND)
		return -EBUSY;
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

/* Transfer one quiesced, complete LMP main-Bayer frame to RGBP ownership. */
static int ispfe_backend_handoff_set(void *data, u64 val)
{
	struct ispfe_device *ispfe = data;
	int ret;

	if (val > 1)
		return -EINVAL;

	guard(mutex)(&ispfe->lock);
	if (ispfe->streaming)
		return -EBUSY;
	if (!ispfe->backend_producing)
		return -EALREADY;

	if (!val) {
		exynos_becore_input_producer_abort(ispfe->backend_input,
						   &ispfe->backend_buffer);
		ispfe->backend_producing = false;
		ispfe->backend_handed_off = false;
		return 0;
	}
	/* Pairs with the EOF-side publication of the completed snapshot. */
	if (smp_load_acquire(&ispfe->snapshot_state) != ISPFE_SNAPSHOT_READY)
		return -ENODATA;

	ret = exynos_becore_input_producer_complete(ispfe->backend_input,
						    &ispfe->backend_buffer);
	if (!ret) {
		ispfe->backend_producing = false;
		ispfe->backend_handed_off = true;
	}

	return ret;
}

static int ispfe_backend_handoff_get(void *data, u64 *val)
{
	struct ispfe_device *ispfe = data;

	guard(mutex)(&ispfe->lock);
	*val = ispfe->backend_handed_off;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ispfe_backend_handoff_fops,
			 ispfe_backend_handoff_get,
			 ispfe_backend_handoff_set, "%llu\n");

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

	seq_printf(s, "# link %u loch %u fcctx %u\n", ispfe->active.link,
		   ispfe->active.loch, ispfe->active.fcctx);

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

	seq_printf(s, "fcaxi  +0x%05x %#010x\n",
		   FC_LMP_IDMA0 + FC_LMP_IDMA_AXI_MAX_OST,
		   readl_relaxed(core + FC_LMP_IDMA0 + FC_LMP_IDMA_AXI_MAX_OST));
	seq_printf(s, "fcaxi  +0x%05x %#010x\n", FC_CFG_BUS_CTRL,
		   readl_relaxed(core + FC_CFG_BUS_CTRL));
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
	struct list_head *pos;
	unsigned int backend_ready = 0, backend_done = 0;
	unsigned int backend_flight, backend_completed, backend_dropped;
	u64 backend_credits;
	unsigned int isolation, i, flight;
	dma_addr_t backend_dma;
	unsigned long backend_programs, slots;
	int backend_error;
	bool backend_active;

	guard(mutex)(&ispfe->lock);
	backend_dma = ispfe->backend_buffer.dma;

	seq_printf(s, "streaming    %u\n", ispfe->streaming);
	seq_printf(s, "sensor_stream %u\n", ispfe->sensor_streaming);
	seq_printf(s, "owner        %s\n",
		   ispfe->owner == ISPFE_OWNER_V4L2 ? "v4l2" :
		   ispfe->owner == ISPFE_OWNER_BACKEND ? "backend" :
		   ispfe->owner == ISPFE_OWNER_DEBUGFS ? "debugfs" : "none");
	scoped_guard(spinlock_irqsave, &ispfe->slock) {
		flight = ispfe->flight_count;
		slots = ispfe->slots_used;
		backend_flight = ispfe->backend_flight_count;
		backend_programs = ispfe->backend_programs_used;
		backend_credits = ispfe->backend_credit_count;
		backend_completed = ispfe->backend_completed;
		backend_dropped = ispfe->backend_dropped;
		backend_error = ispfe->backend_queue_error;
		backend_active = ispfe->backend_queue_active;
		list_for_each(pos, &ispfe->backend_ready)
			backend_ready++;
		list_for_each(pos, &ispfe->backend_done)
			backend_done++;
	}
	seq_printf(s, "queue        %u in flight, slots %#lx, seq %u\n",
		   flight, slots, ispfe->sequence);
	seq_printf(s,
		   "backend_queue active %u, ready %u, flight %u, done %u, programs %#lx, credits %llu, completed %u, dropped %u, error %d\n",
		   backend_active, backend_ready, backend_flight, backend_done,
		   backend_programs, backend_credits, backend_completed,
		   backend_dropped, backend_error);
	seq_printf(s, "snapshot_state %u\n", READ_ONCE(ispfe->snapshot_state));
	seq_printf(s, "snapshot_armed %u\n",
		   READ_ONCE(ispfe->snapshot_state) == ISPFE_SNAPSHOT_ARMED);
	seq_printf(s, "frame_frozen %u\n",
		   READ_ONCE(ispfe->snapshot_state) == ISPFE_SNAPSHOT_READY);
	seq_printf(s, "snapshot_redirected %u\n",
		   READ_ONCE(ispfe->snapshot_state) >=
		   ISPFE_SNAPSHOT_REDIRECTED);
	seq_printf(s, "power_hold   %u\n", ispfe->power_hold);
	seq_printf(s, "camera_qos   CAM %lu Hz (saved %lu, owned %u), memory %u\n",
		   clk_get_rate(ispfe->cam_clk), ispfe->saved_cam_rate,
		   ispfe->cam_rate_active, ispfe->memory_vote_active);
	seq_printf(s, "phy_bypass   %u\n", ispfe->phy_isolation_bypass);
	seq_printf(s, "lmp_ml0_profile %u requested, %u active\n",
		   ispfe->lmp_ml0_profile, ispfe->active_lmp_ml0_profile);
	seq_printf(s, "backend_recipe %u requested, %u active\n",
		   ispfe->backend_recipe, ispfe->active_backend_recipe);
	seq_printf(s, "pdma_override %u requested, %u active\n",
		   ispfe->pdma_program_override,
		   ispfe->active_pdma_program_override);
	seq_printf(s, "pdma_staged  program %zu/%u gen %u, blocks %zu/%u gen %u\n",
		   ispfe->pdma_program_staged_bytes, ISPFE_PDMA_RECIPE_BYTES,
		   ispfe->pdma_program_staged_generation,
		   ispfe->pdma_blocks_staged_bytes, ISPFE_PDMA_BLOCKS_BYTES,
		   ispfe->pdma_blocks_staged_generation);
	seq_printf(s, "pdma_active  program gen %u, blocks gen %u\n",
		   ispfe->active_pdma_program_generation,
		   ispfe->active_pdma_blocks_generation);
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
	seq_printf(s, "program_iova %pad  %u slots of %#x\n",
		   &ispfe->programs_dma, PDMA_SLOTS, PDMA_SLOT_STRIDE);
	seq_printf(s, "blocks_iova  %pad\n", &ispfe->blocks_dma);
	seq_printf(s, "program_size %u\n",
		   ispfe->prog ? ispfe->prog->recipe_bytes :
		   ISPFE_PDMA_RECIPE_BYTES);
	seq_printf(s, "bayer_reloc  %#x/%#x\n", ispfe->bayer_lo,
		   ispfe->bayer_hi);
	seq_printf(s,
		   "backend      input %pad, spare %pad, size %zu, producing %u, handed_off %u\n",
		   &backend_dma,
		   &ispfe->backend_spare_dma, ispfe->backend_input_size,
		   ispfe->backend_producing, ispfe->backend_handed_off);
	seq_printf(s, "backend_reloc image %#x/%#x, header %#x/%#x\n",
		   ispfe->backend_image_lo, ispfe->backend_image_hi,
		   ispfe->backend_header_lo, ispfe->backend_header_hi);
	seq_printf(s, "tnr_iova     %pad  size %u\n",
		   &ispfe->tnr_pyramid_dma, ISPFE_TNR_PYRAMID_SIZE);
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

/*
 * The encoded program, so what PDMA was actually handed can be decoded and
 * checked against the recipe and this driver's own buffer addresses, rather
 * than inferred from whether a frame came out looking right.
 */
static ssize_t ispfe_program_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct ispfe_device *ispfe = file->private_data;

	guard(mutex)(&ispfe->lock);

	if (!ispfe->programs)
		return -ENODATA;

	/* Slot 0, which is the one the diagnostic's ring records point at. */
	return simple_read_from_buffer(buf, count, ppos, ispfe->programs,
				       ispfe->prog->recipe_bytes);
}

static ssize_t ispfe_pdma_stage_write(struct ispfe_device *ispfe,
				      const char __user *buf, size_t count,
				      loff_t *ppos, u8 *staged,
				      size_t capacity, size_t *staged_bytes,
				      u32 *generation)
{
	ssize_t ret = count;

	if (!count)
		return 0;

	guard(mutex)(&ispfe->lock);

	if (ispfe->streaming || ispfe->owner != ISPFE_OWNER_NONE)
		return -EBUSY;
	if (*ppos < 0 || *ppos > capacity)
		return -EINVAL;
	if (count > capacity - *ppos)
		return -EFBIG;
	if (*ppos == 0)
		*staged_bytes = 0;
	if (*ppos != *staged_bytes)
		return -ESPIPE;
	if (copy_from_user(staged + *staged_bytes, buf, count))
		return -EFAULT;

	*staged_bytes += count;
	*ppos += count;
	if (*staged_bytes == capacity)
		(*generation)++;

	return ret;
}

static ssize_t ispfe_program_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct ispfe_device *ispfe = file->private_data;

	return ispfe_pdma_stage_write(ispfe, buf, count, ppos,
				      ispfe->pdma_program_staged,
				      ISPFE_PDMA_RECIPE_BYTES,
				      &ispfe->pdma_program_staged_bytes,
				      &ispfe->pdma_program_staged_generation);
}

static const struct file_operations ispfe_program_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = ispfe_program_read,
	.write = ispfe_program_write,
	.llseek = default_llseek,
};

static ssize_t ispfe_blocks_read(struct file *file, char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct ispfe_device *ispfe = file->private_data;

	guard(mutex)(&ispfe->lock);

	if (ispfe->streaming)
		return -EBUSY;
	if (!ispfe->blocks)
		return -ENODATA;

	return simple_read_from_buffer(buf, count, ppos, ispfe->blocks,
				       ispfe->prog->blocks_bytes);
}

static ssize_t ispfe_blocks_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct ispfe_device *ispfe = file->private_data;

	return ispfe_pdma_stage_write(ispfe, buf, count, ppos,
				      ispfe->pdma_blocks_staged,
				      ISPFE_PDMA_BLOCKS_BYTES,
				      &ispfe->pdma_blocks_staged_bytes,
				      &ispfe->pdma_blocks_staged_generation);
}

static const struct file_operations ispfe_blocks_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = ispfe_blocks_read,
	.write = ispfe_blocks_write,
	.llseek = default_llseek,
};

static int ispfe_program_override_get(void *data, u64 *value)
{
	struct ispfe_device *ispfe = data;

	guard(mutex)(&ispfe->lock);
	*value = ispfe->pdma_program_override;

	return 0;
}

static int ispfe_program_override_set(void *data, u64 value)
{
	struct ispfe_device *ispfe = data;

	if (value > 1)
		return -EINVAL;

	guard(mutex)(&ispfe->lock);
	if (ispfe->streaming || ispfe->owner != ISPFE_OWNER_NONE)
		return -EBUSY;
	/*
	 * An unproven program can leave LMP busy even after the receiver is
	 * stopped.  Keep the domain live for register inspection and an orderly
	 * reboot rather than asking genpd to power down a block that cannot
	 * acknowledge the request.
	 */
	if (value && !ispfe->power_hold)
		return -EBUSY;
	if (value &&
	    (ispfe->pdma_program_staged_bytes != ISPFE_PDMA_RECIPE_BYTES ||
	     ispfe->pdma_blocks_staged_bytes != ISPFE_PDMA_BLOCKS_BYTES))
		return -EINVAL;

	ispfe->pdma_program_override = value;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ispfe_program_override_fops,
			 ispfe_program_override_get,
			 ispfe_program_override_set, "%llu\n");

/*
 * The captured program already enables three memory-backed processed outputs.
 * Keep this diagnostic read-only and require the stream to be stopped: unlike
 * the per-frame Bayer queue, these buffers are shared and hardware overwrites
 * them continuously while streaming.
 */
static ssize_t ispfe_pdma_output_read(struct file *file, char __user *buf,
				      size_t count, loff_t *ppos,
				      unsigned int index)
{
	struct ispfe_device *ispfe = file->private_data;

	guard(mutex)(&ispfe->lock);

	if (index >= ARRAY_SIZE(ispfe_pdma_outputs))
		return -EINVAL;
	if (ispfe->streaming)
		return -EBUSY;
	if (!ispfe->pdma_output[index].cpu)
		return -ENODATA;

	return simple_read_from_buffer(buf, count, ppos,
				       ispfe->pdma_output[index].cpu,
				       ispfe_pdma_outputs[index].size);
}

static ssize_t ispfe_lmp_rgb_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	return ispfe_pdma_output_read(file, buf, count, ppos,
				      ISPFE_PDMA_OUTPUT_RGB);
}

static const struct file_operations ispfe_lmp_rgb_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = ispfe_lmp_rgb_read,
	.llseek = default_llseek,
};

static ssize_t ispfe_lmp_ml0_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	return ispfe_pdma_output_read(file, buf, count, ppos,
				      ISPFE_PDMA_OUTPUT_ML0);
}

static const struct file_operations ispfe_lmp_ml0_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = ispfe_lmp_ml0_read,
	.llseek = default_llseek,
};

static ssize_t ispfe_lmp_ml2_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	return ispfe_pdma_output_read(file, buf, count, ppos,
				      ISPFE_PDMA_OUTPUT_ML2);
}

static const struct file_operations ispfe_lmp_ml2_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = ispfe_lmp_ml2_read,
	.llseek = default_llseek,
};

DEFINE_SHOW_ATTRIBUTE(ispfe_status);

static void ispfe_debugfs_init(struct ispfe_device *ispfe)
{
	struct dentry *d;

	d = debugfs_create_dir(dev_name(ispfe->dev), NULL);
	ispfe->debugfs = d;

	/*
	 * The link, its lane count and its PHY type describe how the board is
	 * wired and now come from the device tree; the geometry comes from the
	 * format negotiated on the subdev.  What is left here is the receiver's
	 * own resource allocation and the values still being swept.
	 */
	debugfs_create_u32("phy", 0644, d, &ispfe->src.phy);
	debugfs_create_file("phy_isolation_bypass", 0644, d, ispfe,
			    &ispfe_phy_isolation_bypass_fops);
	debugfs_create_file("power_hold", 0644, d, ispfe,
			    &ispfe_power_hold_fops);
	debugfs_create_file("predown_qch", 0644, d, ispfe,
			    &ispfe_predown_qch_fops);
	debugfs_create_file("regs", 0444, d, ispfe, &ispfe_regs_fops);
	debugfs_create_u32("loch", 0644, d, &ispfe->src.loch);
	debugfs_create_u32("fcctx", 0644, d, &ispfe->src.fcctx);
	debugfs_create_u32("settle_us", 0644, d, &ispfe->settle_us);
	debugfs_create_x32("fc_axi_max_ost", 0644, d,
			   &ispfe->fc_axi_max_ost);
	debugfs_create_u32("lmp_ml0_profile", 0644, d,
			   &ispfe->lmp_ml0_profile);
	debugfs_create_u32("backend_recipe", 0644, d,
			   &ispfe->backend_recipe);
	debugfs_create_u32("credit_latency", 0644, d, &ispfe->credit_latency);
	debugfs_create_x32("pdma_cmd", 0644, d, &ispfe->pdma_cmd);
	debugfs_create_x32("pdma_addr", 0644, d, &ispfe->pdma_addr);
	debugfs_create_x32("pdma_bytes", 0644, d, &ispfe->pdma_bytes);
	debugfs_create_x32("pdma_bytes_first", 0644, d,
			   &ispfe->pdma_bytes_first);
	debugfs_create_bool("pdma_no_kick", 0644, d, &ispfe->pdma_no_kick);
	debugfs_create_u32("mode_word0", 0644, d, &ispfe->src.mode_word0);
	debugfs_create_u32("mode_word1", 0644, d, &ispfe->src.mode_word1);
	debugfs_create_file("enable", 0644, d, ispfe, &ispfe_enable_fops);
	debugfs_create_file("capture", 0644, d, ispfe, &ispfe_capture_fops);
	debugfs_create_file("snapshot", 0644, d, ispfe,
			    &ispfe_snapshot_fops);
	debugfs_create_file("backend_handoff", 0644, d, ispfe,
			    &ispfe_backend_handoff_fops);
	debugfs_create_file("backend_queue", 0644, d, ispfe,
			    &ispfe_backend_queue_fops);
	debugfs_create_file("status", 0444, d, ispfe, &ispfe_status_fops);
	debugfs_create_file("frame", 0444, d, ispfe, &ispfe_frame_fops);
	debugfs_create_file("program", 0644, d, ispfe, &ispfe_program_fops);
	debugfs_create_file("blocks", 0644, d, ispfe, &ispfe_blocks_fops);
	debugfs_create_file("program_override", 0644, d, ispfe,
			    &ispfe_program_override_fops);
	debugfs_create_file("lmp_rgb", 0444, d, ispfe, &ispfe_lmp_rgb_fops);
	debugfs_create_file("lmp_ml0", 0444, d, ispfe, &ispfe_lmp_ml0_fops);
	debugfs_create_file("lmp_ml2", 0444, d, ispfe, &ispfe_lmp_ml2_fops);
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

/* ---- the V4L2 capture queue -------------------------------------------- */

/* The vendor's own WDMA address writer rejects anything less aligned. */
#define ISPFE_FRAME_ALIGN		32

static const struct ispfe_format *ispfe_format_by_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ispfe_formats); i++)
		if (ispfe_formats[i].code == code)
			return &ispfe_formats[i];

	return NULL;
}

/*
 * The capture node's format is the media-bus format on the receiver's source
 * pad, restated in pixel-format terms: ten bits per sample in a sixteen-bit
 * little-endian container at the natural stride, which is what the vendor's
 * own saved frames measure.  Nothing in the receive path converts, so the two
 * cannot disagree and there is nothing here to negotiate separately.
 */
static void ispfe_fill_pix(const struct v4l2_mbus_framefmt *fmt,
			   struct v4l2_pix_format *pix)
{
	const struct ispfe_format *info = ispfe_format_by_code(fmt->code);
	const struct ispfe_pdma_program *prog;

	pix->width = fmt->width;
	pix->height = fmt->height;
	pix->pixelformat = info ? info->pixelformat : V4L2_PIX_FMT_SRGGB10;
	pix->field = V4L2_FIELD_NONE;
	/*
	 * The recipe carries the write stride, and the binned mode's is padded
	 * -- 4224 for 2104 pixels, not 4208 -- so reporting width * 2 would
	 * under-size every buffer by sixteen bytes a line.
	 */
	prog = ispfe_program_for(pix->width, pix->height, false);
	pix->bytesperline = prog ? prog->stride :
			    pix->width * ISPFE_BYTES_PER_PIXEL;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = V4L2_COLORSPACE_RAW;
	/*
	 * Set rather than left alone: the core only sanitises these when the
	 * application does not claim the extended format, so an app that does
	 * would otherwise get its own values echoed back.
	 */
	pix->flags = 0;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static void ispfe_active_pix(struct ispfe_device *ispfe,
			     struct v4l2_pix_format *pix)
{
	struct v4l2_subdev_state *state;

	state = v4l2_subdev_lock_and_get_active_state(&ispfe->sd);
	ispfe_fill_pix(v4l2_subdev_state_get_format(state, ISPFE_PAD_SOURCE),
		       pix);
	v4l2_subdev_unlock_state(state);
}

static void ispfe_queue_return_all(struct ispfe_device *ispfe,
				   enum vb2_buffer_state state)
{
	struct list_head done;
	struct ispfe_buffer *buf, *tmp;

	INIT_LIST_HEAD(&done);

	scoped_guard(spinlock_irqsave, &ispfe->slock) {
		list_splice_tail_init(&ispfe->flight, &done);
		list_splice_tail_init(&ispfe->ready, &done);
		list_splice_tail_init(&ispfe->pending, &done);
		ispfe->flight_count = 0;
		ispfe->slots_used = 0;
	}

	list_for_each_entry_safe(buf, tmp, &done, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
}

/*
 * Give queued buffers a program each.  Encoding is 6 KiB of work, so it happens
 * here in process context rather than in the frame-start interrupt, which then
 * only has to publish a sixteen-byte record.
 */
static void ispfe_queue_fill(struct ispfe_device *ispfe)
{
	for (;;) {
		struct ispfe_buffer *buf;
		unsigned int slot;
		dma_addr_t dma;
		int ret;

		scoped_guard(spinlock_irqsave, &ispfe->slock) {
			if (list_empty(&ispfe->pending))
				return;
			slot = find_first_zero_bit(&ispfe->slots_used,
						   PDMA_BUF_SLOTS);
			if (slot >= PDMA_BUF_SLOTS)
				return;
			__set_bit(slot, &ispfe->slots_used);
			buf = list_first_entry(&ispfe->pending,
					       struct ispfe_buffer, list);
			list_del(&buf->list);
			buf->slot = slot;
		}

		dma = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
		ret = ispfe_pdma_encode(ispfe, slot, dma,
					ispfe->backend_buffer.dma);
		if (ret) {
			scoped_guard(spinlock_irqsave, &ispfe->slock)
				__clear_bit(slot, &ispfe->slots_used);
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
			continue;
		}

		/*
		 * Only now is the slot safe to credit, which is why the buffer
		 * reaches the ready list after its program is complete.
		 */
		scoped_guard(spinlock_irqsave, &ispfe->slock)
			list_add_tail(&buf->list, &ispfe->ready);
	}
}

static void ispfe_fill_work(struct work_struct *work)
{
	struct ispfe_device *ispfe =
		container_of(work, struct ispfe_device, fill_work);

	ispfe_queue_fill(ispfe);
}

static int ispfe_queue_setup(struct vb2_queue *q, unsigned int *nbufs,
			     unsigned int *nplanes, unsigned int sizes[],
			     struct device *alloc_devs[])
{
	struct ispfe_device *ispfe = vb2_get_drv_priv(q);
	u32 size;

	/*
	 * Latched here rather than read at start: buffers are sized now, and
	 * the receiver has to be told the same numbers when it is armed.
	 */
	ispfe_active_pix(ispfe, &ispfe->fmt);
	size = ispfe->fmt.sizeimage;

	if (*nplanes) {
		if (*nplanes != 1 || sizes[0] < size)
			return -EINVAL;
		return 0;
	}

	*nplanes = 1;
	sizes[0] = size;

	return 0;
}

static int ispfe_buf_prepare(struct vb2_buffer *vb)
{
	struct ispfe_device *ispfe = vb2_get_drv_priv(vb->vb2_queue);
	dma_addr_t dma = vb2_dma_contig_plane_dma_addr(vb, 0);

	if (vb2_plane_size(vb, 0) < ispfe->fmt.sizeimage)
		return -EINVAL;

	/*
	 * An imported buffer's address is not the driver's to choose, and the
	 * front end simply drops the low bits rather than reporting a fault --
	 * which would show up as a sheared image and nothing else.
	 */
	if (!IS_ALIGNED(dma, ISPFE_FRAME_ALIGN)) {
		dev_err_ratelimited(ispfe->dev,
				    "buffer at %pad is not %u-byte aligned\n",
				    &dma, ISPFE_FRAME_ALIGN);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, ispfe->fmt.sizeimage);

	return 0;
}

static void ispfe_buf_queue(struct vb2_buffer *vb)
{
	struct ispfe_device *ispfe = vb2_get_drv_priv(vb->vb2_queue);
	struct ispfe_buffer *buf = to_ispfe_buffer(to_vb2_v4l2_buffer(vb));

	scoped_guard(spinlock_irqsave, &ispfe->slock)
		list_add_tail(&buf->list, &ispfe->pending);

	/*
	 * Not vb2_is_streaming(): with a min_queued_buffers of two, STREAMON
	 * with one buffer queued succeeds and defers the actual start to the
	 * next QBUF -- which enqueues every buffer into the driver *before*
	 * calling start_streaming.  So the queue can be streaming while there
	 * is still no program area to encode into.  Gate on the driver's own
	 * state instead; ispfe_start_streaming() drains what accumulated.
	 */
	if (ispfe->owner == ISPFE_OWNER_V4L2)
		ispfe_queue_fill(ispfe);
}

/*
 * Power the source without starting it.  The receiver is armed against a link
 * that has to be alive already -- lanes in LP-11, master clock running -- but
 * it must not see a frame before it is armed, which is what separates powering
 * the sensor from streaming it.  A sensor with no such distinction simply
 * answers -ENOIOCTLCMD and is powered by its stream instead.
 */
static int ispfe_sensor_power(struct ispfe_device *ispfe, bool on)
{
	int ret;

	if (!ispfe->sensor)
		return on ? -ENODEV : 0;

	if (on) {
		ret = v4l2_subdev_call(ispfe->sensor, video, pre_streamon, 0);
		return ret == -ENOIOCTLCMD ? 0 : ret;
	}

	ret = v4l2_subdev_call(ispfe->sensor, video, post_streamoff);
	if (ret && ret != -ENOIOCTLCMD)
		dev_err(ispfe->dev, "cannot power the sensor down: %d\n", ret);

	return 0;
}

static int ispfe_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct ispfe_device *ispfe = vb2_get_drv_priv(q);
	int ret;

	if (ispfe->owner != ISPFE_OWNER_NONE) {
		ispfe_queue_return_all(ispfe, VB2_BUF_STATE_QUEUED);
		return -EBUSY;
	}

	ret = video_device_pipeline_start(&ispfe->vdev, &ispfe->pipe);
	if (ret)
		goto err_return;

	ispfe->sequence = 0;
	ispfe->owner = ISPFE_OWNER_V4L2;

	ret = ispfe_sensor_power(ispfe, true);
	if (ret)
		goto err_pipeline;

	ret = ispfe_start(ispfe);
	if (ret)
		goto err_power;

	/*
	 * The buffers were sized at REQBUFS from the format on the source pad,
	 * and ispfe_start() has just latched its own copy of the same format.
	 * If the two disagree the receiver would write past the end of every
	 * buffer, so this is a hard stop rather than a resize.
	 */
	if (ispfe->fmt.sizeimage != size_mul(ispfe->prog->stride,
					     ispfe->active.height)) {
		dev_err(ispfe->dev,
			"geometry changed under the queue: %ux%u now\n",
			ispfe->active.width, ispfe->active.height);
		ret = -EINVAL;
		goto err_stop;
	}

	ispfe_queue_fill(ispfe);

	/*
	 * The sensor goes last.  Starting it before the receiver is armed loses
	 * the first frames and can leave the link's error bits set, which reads
	 * like a PHY fault and is not one.
	 */
	ret = v4l2_subdev_enable_streams(&ispfe->sd, ISPFE_PAD_SOURCE,
					 BIT_ULL(0));
	if (ret)
		goto err_stop;
	ispfe->sensor_streaming = true;

	return 0;

err_stop:
	ispfe_stop(ispfe);
err_power:
	ispfe_sensor_power(ispfe, false);
err_pipeline:
	ispfe->owner = ISPFE_OWNER_NONE;
	video_device_pipeline_stop(&ispfe->vdev);
err_return:
	ispfe_queue_return_all(ispfe, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void ispfe_stop_streaming(struct vb2_queue *q)
{
	struct ispfe_device *ispfe = vb2_get_drv_priv(q);
	int ret;

	/*
	 * The receiver goes down first, while the sensor is still clocking it.
	 * Told to stop, the sensor drops the lanes mid-frame, and everything
	 * downstream then has no clock -- the PHY's recovered one is gone -- so
	 * a reset written into a clock domain that is not running does not
	 * take, and the block will not quiesce for the power-down.  This is
	 * also the order the vendor stack tears a session down in.
	 *
	 * ispfe_stop() frees the interrupts, so nothing can be crediting or
	 * completing by the time the lists are emptied.
	 */
	ispfe_stop(ispfe);
	ret = v4l2_subdev_disable_streams(&ispfe->sd, ISPFE_PAD_SOURCE,
					  BIT_ULL(0));
	if (ret)
		dev_err(ispfe->dev, "cannot stop the sensor: %d\n", ret);
	ispfe_sensor_power(ispfe, false);
	ispfe->sensor_streaming = false;
	cancel_work_sync(&ispfe->fill_work);
	ispfe->owner = ISPFE_OWNER_NONE;
	video_device_pipeline_stop(&ispfe->vdev);
	ispfe_queue_return_all(ispfe, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops ispfe_vb2_ops = {
	.queue_setup = ispfe_queue_setup,
	.buf_prepare = ispfe_buf_prepare,
	.buf_queue = ispfe_buf_queue,
	.start_streaming = ispfe_start_streaming,
	.stop_streaming = ispfe_stop_streaming,
};

static int ispfe_querycap(struct file *file, void *priv,
			  struct v4l2_capability *cap)
{
	strscpy(cap->driver, "exynos-ispfe", sizeof(cap->driver));
	strscpy(cap->card, "zumapro ISPFE", sizeof(cap->card));

	return 0;
}

/*
 * One entry, and it is the format the source pad currently carries.  The
 * receive path does not convert, so the node cannot offer a choice: what the
 * sensor puts on the link is what lands in memory, and the way to change it is
 * to set a different format on the pads.
 */
static int ispfe_enum_fmt(struct file *file, void *priv,
			  struct v4l2_fmtdesc *f)
{
	struct ispfe_device *ispfe = video_drvdata(file);
	struct v4l2_subdev_state *state;
	struct v4l2_pix_format pix;
	u32 code;

	if (f->index)
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(&ispfe->sd);
	code = v4l2_subdev_state_get_format(state, ISPFE_PAD_SOURCE)->code;
	ispfe_fill_pix(v4l2_subdev_state_get_format(state, ISPFE_PAD_SOURCE),
		       &pix);
	v4l2_subdev_unlock_state(state);

	/*
	 * The core leaves mbus_code alone for a media-controller node so that
	 * the driver can filter on it, and the answer for a code this pipeline
	 * is not carrying is that there is no such format.
	 */
	if (f->mbus_code && f->mbus_code != code)
		return -EINVAL;

	f->pixelformat = pix.pixelformat;

	return 0;
}

static int ispfe_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct ispfe_device *ispfe = video_drvdata(file);

	ispfe_active_pix(ispfe, &f->fmt.pix);

	return 0;
}

/*
 * The node advertises V4L2_CAP_IO_MC, so the format is the pipeline's rather
 * than the application's: a request is answered with what the pads are set to
 * produce.  Changing it means setting a format on the subdev pads.
 */
static int ispfe_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct ispfe_device *ispfe = video_drvdata(file);

	if (vb2_is_busy(&ispfe->queue))
		return -EBUSY;

	ispfe_active_pix(ispfe, &f->fmt.pix);

	return 0;
}

static int ispfe_enum_framesizes(struct file *file, void *priv,
				 struct v4l2_frmsizeenum *fsize)
{
	struct ispfe_device *ispfe = video_drvdata(file);
	struct v4l2_pix_format pix;

	ispfe_active_pix(ispfe, &pix);
	if (fsize->index || fsize->pixel_format != pix.pixelformat)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = pix.width;
	fsize->discrete.height = pix.height;

	return 0;
}

static const struct v4l2_ioctl_ops ispfe_ioctl_ops = {
	.vidioc_querycap = ispfe_querycap,
	.vidioc_enum_fmt_vid_cap = ispfe_enum_fmt,
	.vidioc_g_fmt_vid_cap = ispfe_g_fmt,
	.vidioc_s_fmt_vid_cap = ispfe_s_fmt,
	.vidioc_try_fmt_vid_cap = ispfe_g_fmt,
	.vidioc_enum_framesizes = ispfe_enum_framesizes,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations ispfe_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

static const struct video_device ispfe_video_template = {
	/*
	 * Distinct from the receiver subdev's entity name.  Both end up in the
	 * same media graph and nothing dedups them, so a duplicate would make
	 * media-ctl and libcamera resolve whichever was registered first.
	 */
	.name = "exynos-ispfe capture",
	.fops = &ispfe_fops,
	.ioctl_ops = &ispfe_ioctl_ops,
	.release = video_device_release_empty,
	.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
		       V4L2_CAP_IO_MC,
	.vfl_dir = VFL_DIR_RX,
};

/*
 * The buffers were sized from the source pad's format at REQBUFS.  If a format
 * has been set on the pads since, the pipeline is inconsistent and starting it
 * would have the receiver write frames of one size into buffers of another.
 */
static int ispfe_vdev_link_validate(struct media_link *link)
{
	struct video_device *vdev =
		media_entity_to_video_device(link->sink->entity);
	struct ispfe_device *ispfe = video_get_drvdata(vdev);
	struct v4l2_pix_format pix;

	ispfe_active_pix(ispfe, &pix);
	if (pix.width != ispfe->fmt.width || pix.height != ispfe->fmt.height ||
	    pix.pixelformat != ispfe->fmt.pixelformat) {
		dev_err(ispfe->dev,
			"the queue holds %ux%u buffers, the pad carries %ux%u\n",
			ispfe->fmt.width, ispfe->fmt.height, pix.width,
			pix.height);
		return -EPIPE;
	}

	return 0;
}

static const struct media_entity_operations ispfe_vdev_entity_ops = {
	.link_validate = ispfe_vdev_link_validate,
};

/* ---- the receiver subdevice -------------------------------------------- */

static struct ispfe_device *sd_to_ispfe(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ispfe_device, sd);
}

static int ispfe_sd_init_state(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *sink =
		v4l2_subdev_state_get_format(state, ISPFE_PAD_SINK);
	struct v4l2_mbus_framefmt *source =
		v4l2_subdev_state_get_format(state, ISPFE_PAD_SOURCE);

	sink->code = ISPFE_DEFAULT_CODE;
	sink->width = ISPFE_DEFAULT_WIDTH;
	sink->height = ISPFE_DEFAULT_HEIGHT;
	sink->field = V4L2_FIELD_NONE;
	sink->colorspace = V4L2_COLORSPACE_RAW;
	sink->ycbcr_enc = V4L2_YCBCR_ENC_601;
	sink->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	sink->xfer_func = V4L2_XFER_FUNC_NONE;
	*source = *sink;

	return 0;
}

static int ispfe_sd_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	/* Nothing converts, so the source offers exactly what the sink took. */
	if (code->pad == ISPFE_PAD_SOURCE) {
		if (code->index)
			return -EINVAL;
		code->code = v4l2_subdev_state_get_format(state,
							  ISPFE_PAD_SINK)->code;
		return 0;
	}

	if (code->index >= ARRAY_SIZE(ispfe_formats))
		return -EINVAL;

	code->code = ispfe_formats[code->index].code;

	return 0;
}

static int ispfe_sd_enum_frame_size(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index)
		return -EINVAL;

	if (fse->pad == ISPFE_PAD_SOURCE) {
		const struct v4l2_mbus_framefmt *sink =
			v4l2_subdev_state_get_format(state, ISPFE_PAD_SINK);

		if (fse->code != sink->code)
			return -EINVAL;

		fse->min_width = sink->width;
		fse->max_width = sink->width;
		fse->min_height = sink->height;
		fse->max_height = sink->height;

		return 0;
	}

	if (!ispfe_format_by_code(fse->code))
		return -EINVAL;

	fse->min_width = ISPFE_MIN_WIDTH;
	fse->max_width = ISPFE_MAX_WIDTH;
	fse->min_height = ISPFE_MIN_HEIGHT;
	fse->max_height = ISPFE_MAX_HEIGHT;

	return 0;
}

/*
 * The sink is what the link carries and the source repeats it: this block
 * receives, and the crop, scale and format converters the line-memory
 * processor has have never been programmed to do anything but pass through.
 * So a format is only ever set on the sink, and the source follows it.
 */
static int ispfe_sd_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state,
			    struct v4l2_subdev_format *format)
{
	struct ispfe_device *ispfe = sd_to_ispfe(sd);
	struct v4l2_mbus_framefmt *sink, *source;

	if (format->pad == ISPFE_PAD_SOURCE)
		return v4l2_subdev_get_fmt(sd, state, format);

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    vb2_is_busy(&ispfe->queue))
		return -EBUSY;

	if (!ispfe_format_by_code(format->format.code))
		format->format.code = ISPFE_DEFAULT_CODE;

	format->format.width = clamp(format->format.width, ISPFE_MIN_WIDTH,
				     ISPFE_MAX_WIDTH);
	format->format.height = clamp(format->format.height, ISPFE_MIN_HEIGHT,
				      ISPFE_MAX_HEIGHT);
	format->format.field = V4L2_FIELD_NONE;
	format->format.colorspace = V4L2_COLORSPACE_RAW;
	format->format.ycbcr_enc = V4L2_YCBCR_ENC_601;
	format->format.quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->format.xfer_func = V4L2_XFER_FUNC_NONE;

	sink = v4l2_subdev_state_get_format(state, ISPFE_PAD_SINK);
	source = v4l2_subdev_state_get_format(state, ISPFE_PAD_SOURCE);
	*sink = format->format;
	*source = *sink;

	return 0;
}

/*
 * The receiver is armed by the capture queue, which is where the buffers are;
 * what the subdev owns is the sensor, so enabling the stream here is enabling
 * the source that feeds the link.
 */
static int ispfe_sd_enable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct ispfe_device *ispfe = sd_to_ispfe(sd);

	if (!ispfe->sensor)
		return -ENODEV;

	return v4l2_subdev_enable_streams(ispfe->sensor, ispfe->sensor_pad,
					  BIT_ULL(0));
}

/*
 * A stop that reports failure leaves this subdev's pad marked as streaming, and
 * the next start is then refused with -EALREADY for as long as the machine is
 * up.  A sensor that has gone away has already stopped, and one that could not
 * be told to stop is not made better by wedging the node, so both are reported
 * and neither is returned.
 */
static int ispfe_sd_disable_streams(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state, u32 pad,
				    u64 streams_mask)
{
	struct ispfe_device *ispfe = sd_to_ispfe(sd);
	int ret;

	if (!ispfe->sensor)
		return 0;

	ret = v4l2_subdev_disable_streams(ispfe->sensor, ispfe->sensor_pad,
					  BIT_ULL(0));
	if (ret)
		dev_err(ispfe->dev, "sensor would not stop: %d\n", ret);

	return 0;
}

static const struct v4l2_subdev_pad_ops ispfe_subdev_pad_ops = {
	.enum_mbus_code = ispfe_sd_enum_mbus_code,
	.enum_frame_size = ispfe_sd_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ispfe_sd_set_fmt,
	.enable_streams = ispfe_sd_enable_streams,
	.disable_streams = ispfe_sd_disable_streams,
};

static const struct v4l2_subdev_video_ops ispfe_subdev_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_ops ispfe_subdev_ops = {
	.video = &ispfe_subdev_video_ops,
	.pad = &ispfe_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops ispfe_subdev_internal_ops = {
	.init_state = ispfe_sd_init_state,
};

static const struct media_entity_operations ispfe_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* ---- binding the sensor ------------------------------------------------ */

static int ispfe_notify_bound(struct v4l2_async_notifier *nf,
			      struct v4l2_subdev *sd,
			      struct v4l2_async_connection *asc)
{
	struct ispfe_device *ispfe =
		container_of(nf, struct ispfe_device, notifier);
	int pad;

	pad = media_entity_get_fwnode_pad(&sd->entity, asc->match.fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (pad < 0) {
		dev_err(ispfe->dev, "%s has no source pad for that endpoint\n",
			sd->name);
		return pad;
	}

	guard(mutex)(&ispfe->lock);
	ispfe->sensor = sd;
	ispfe->sensor_pad = pad;

	return 0;
}

static void ispfe_notify_unbind(struct v4l2_async_notifier *nf,
				struct v4l2_subdev *sd,
				struct v4l2_async_connection *asc)
{
	struct ispfe_device *ispfe =
		container_of(nf, struct ispfe_device, notifier);

	guard(mutex)(&ispfe->lock);
	ispfe->sensor = NULL;
}

/*
 * Both links are immutable.  This block has twelve link banks, but the driver
 * runs one stream and which bank it is on is a statement about how the board is
 * wired rather than something userspace chooses.
 */
static int ispfe_notify_complete(struct v4l2_async_notifier *nf)
{
	struct ispfe_device *ispfe =
		container_of(nf, struct ispfe_device, notifier);
	int ret;

	ret = media_create_pad_link(&ispfe->sensor->entity, ispfe->sensor_pad,
				    &ispfe->sd.entity, ISPFE_PAD_SINK,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		return ret;

	ret = v4l2_device_register_subdev_nodes(&ispfe->v4l2_dev);
	if (ret)
		return ret;

	return media_device_register(&ispfe->mdev);
}

static const struct v4l2_async_notifier_operations ispfe_notifier_ops = {
	.bound = ispfe_notify_bound,
	.unbind = ispfe_notify_unbind,
	.complete = ispfe_notify_complete,
};

/*
 * The device tree says which CSIS link bank the sensor arrives on -- that is
 * the port number -- and what the link looks like.  Everything else about the
 * source is either the driver's own allocation or the negotiated format.
 */
static int ispfe_parse_endpoint(struct ispfe_device *ispfe,
				struct fwnode_handle *ep)
{
	struct v4l2_fwnode_endpoint vep = { .bus_type = V4L2_MBUS_UNKNOWN };
	/*
	 * Zeroed because the port number is only filled in when the port node
	 * has a reg; without one this would carry stack contents into the link
	 * bounds check and pick a CSIS bank at random.
	 */
	struct fwnode_endpoint fwep = {};
	const struct ispfe_link_cfg *cfg;
	int ret;

	ret = fwnode_graph_parse_endpoint(ep, &fwep);
	if (ret)
		return dev_err_probe(ispfe->dev, ret,
				     "cannot read the endpoint's port\n");

	if (fwep.port >= CSIS_NUM_LINKS || !ispfe_link_cfg[fwep.port].known)
		return dev_err_probe(ispfe->dev, -EINVAL,
				     "no PHY is known for CSIS link %u\n",
				     fwep.port);

	ret = v4l2_fwnode_endpoint_parse(ep, &vep);
	if (ret)
		return dev_err_probe(ispfe->dev, ret,
				     "cannot parse the endpoint\n");

	switch (vep.bus_type) {
	case V4L2_MBUS_CSI2_DPHY:
		ispfe->src.cphy = false;
		break;
	case V4L2_MBUS_CSI2_CPHY:
		ispfe->src.cphy = true;
		break;
	default:
		return dev_err_probe(ispfe->dev, -EINVAL,
				     "bus type %u is not CSI-2\n",
				     vep.bus_type);
	}

	cfg = &ispfe_link_cfg[fwep.port];
	ispfe->src.link = fwep.port;
	ispfe->src.phy = cfg->phy;
	ispfe->src.lanes = vep.bus.mipi_csi2.num_data_lanes;
	ispfe->src.mode_word0 = cfg->mode_word0;
	ispfe->src.mode_word1 = cfg->mode_word1;

	if (!ispfe->src.lanes ||
	    ispfe->src.lanes > ispfe_phy_lanes(ispfe->src.phy))
		return dev_err_probe(ispfe->dev, -EINVAL,
				     "%u data lanes, PHY %u has %u\n",
				     ispfe->src.lanes, ispfe->src.phy,
				     ispfe_phy_lanes(ispfe->src.phy));

	return 0;
}

static int ispfe_media_register(struct ispfe_device *ispfe)
{
	struct vb2_queue *q = &ispfe->queue;
	struct v4l2_async_connection *asc;
	struct fwnode_handle *ep;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(ispfe->dev), NULL);
	if (!ep)
		return dev_err_probe(ispfe->dev, -ENXIO,
				     "no sensor endpoint\n");

	ret = ispfe_parse_endpoint(ispfe, ep);
	if (ret)
		goto err_ep;

	ispfe->mdev.dev = ispfe->dev;
	strscpy(ispfe->mdev.model, "zumapro ISPFE", sizeof(ispfe->mdev.model));
	media_device_init(&ispfe->mdev);
	ispfe->v4l2_dev.mdev = &ispfe->mdev;

	ret = v4l2_device_register(ispfe->dev, &ispfe->v4l2_dev);
	if (ret)
		goto err_mdev;

	v4l2_subdev_init(&ispfe->sd, &ispfe_subdev_ops);
	ispfe->sd.internal_ops = &ispfe_subdev_internal_ops;
	ispfe->sd.owner = THIS_MODULE;
	ispfe->sd.dev = ispfe->dev;
	ispfe->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	ispfe->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	ispfe->sd.entity.ops = &ispfe_subdev_entity_ops;
	snprintf(ispfe->sd.name, sizeof(ispfe->sd.name), "exynos-ispfe csis%u",
		 ispfe->src.link);
	v4l2_set_subdevdata(&ispfe->sd, ispfe);

	/*
	 * MUST_CONNECT, so that a pipeline with no sensor is refused before any
	 * of the receiver is programmed rather than after it is fully armed.
	 */
	ispfe->pads[ISPFE_PAD_SINK].flags = MEDIA_PAD_FL_SINK |
					    MEDIA_PAD_FL_MUST_CONNECT;
	ispfe->pads[ISPFE_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&ispfe->sd.entity, ISPFE_NUM_PADS,
				     ispfe->pads);
	if (ret)
		goto err_v4l2;

	ret = v4l2_subdev_init_finalize(&ispfe->sd);
	if (ret)
		goto err_sd_entity;

	ret = v4l2_device_register_subdev(&ispfe->v4l2_dev, &ispfe->sd);
	if (ret)
		goto err_sd;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->dev = ispfe->dev;
	q->drv_priv = ispfe;
	q->ops = &ispfe_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct ispfe_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	/*
	 * A frame lands in the buffer credited a frame earlier, so the queue
	 * has to hold at least two before the first one can be completed.
	 */
	q->min_queued_buffers = 2;
	q->lock = &ispfe->lock;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_unreg_sd;

	ispfe->vdev = ispfe_video_template;
	ispfe->vdev.v4l2_dev = &ispfe->v4l2_dev;
	ispfe->vdev.queue = q;
	ispfe->vdev.lock = &ispfe->lock;
	ispfe->vdev.entity.ops = &ispfe_vdev_entity_ops;
	video_set_drvdata(&ispfe->vdev, ispfe);

	ispfe->vdev_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&ispfe->vdev.entity, 1, &ispfe->vdev_pad);
	if (ret)
		goto err_unreg_sd;

	ret = video_register_device(&ispfe->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_vdev_entity;

	ret = media_create_pad_link(&ispfe->sd.entity, ISPFE_PAD_SOURCE,
				    &ispfe->vdev.entity, 0,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		goto err_vdev;

	v4l2_async_nf_init(&ispfe->notifier, &ispfe->v4l2_dev);
	asc = v4l2_async_nf_add_fwnode_remote(&ispfe->notifier, ep,
					      struct v4l2_async_connection);
	if (IS_ERR(asc)) {
		ret = PTR_ERR(asc);
		goto err_nf;
	}

	ispfe->notifier.ops = &ispfe_notifier_ops;
	ret = v4l2_async_nf_register(&ispfe->notifier);
	if (ret)
		goto err_nf;

	fwnode_handle_put(ep);

	return 0;

err_nf:
	v4l2_async_nf_cleanup(&ispfe->notifier);
err_vdev:
	video_unregister_device(&ispfe->vdev);
err_vdev_entity:
	media_entity_cleanup(&ispfe->vdev.entity);
err_unreg_sd:
	v4l2_device_unregister_subdev(&ispfe->sd);
err_sd:
	v4l2_subdev_cleanup(&ispfe->sd);
err_sd_entity:
	media_entity_cleanup(&ispfe->sd.entity);
err_v4l2:
	v4l2_device_unregister(&ispfe->v4l2_dev);
err_mdev:
	media_device_cleanup(&ispfe->mdev);
err_ep:
	fwnode_handle_put(ep);
	return ret;
}

static void ispfe_media_unregister(struct ispfe_device *ispfe)
{
	/*
	 * The video device goes first, because it is the only thing that can be
	 * streaming and stopping the stream needs the sensor still bound.  It
	 * also releases the queue, unlike a bare video_unregister_device(),
	 * which would leave the fill work running over the program area freed
	 * afterwards.
	 */
	vb2_video_unregister_device(&ispfe->vdev);
	v4l2_async_nf_unregister(&ispfe->notifier);
	v4l2_async_nf_cleanup(&ispfe->notifier);
	media_device_unregister(&ispfe->mdev);
	media_entity_cleanup(&ispfe->vdev.entity);
	v4l2_device_unregister_subdev(&ispfe->sd);
	v4l2_subdev_cleanup(&ispfe->sd);
	media_entity_cleanup(&ispfe->sd.entity);
	v4l2_device_unregister(&ispfe->v4l2_dev);
	media_device_cleanup(&ispfe->mdev);
}

static void ispfe_backend_unmap(void *data)
{
	struct ispfe_device *ispfe = data;

	if (ispfe->backend_producing)
		exynos_becore_input_producer_abort(ispfe->backend_input,
						   &ispfe->backend_buffer);
	exynos_becore_input_unmap(ispfe->backend_input);
	ispfe->backend_input = NULL;
	ispfe->backend_producing = false;
	ispfe->backend_handed_off = false;
}

static int ispfe_backend_init(struct ispfe_device *ispfe)
{
	struct platform_device *backend;
	struct device_node *np;
	int ret;

	np = of_parse_phandle(ispfe->dev->of_node, "google,backend", 0);
	if (!np)
		return dev_err_probe(ispfe->dev, -ENODEV,
				     "no camera back-end phandle\n");
	backend = of_find_device_by_node(np);
	of_node_put(np);
	if (!backend)
		return -EPROBE_DEFER;

	ispfe->backend_input =
		exynos_becore_input_map(&backend->dev, ispfe->dev);
	put_device(&backend->dev);
	if (IS_ERR(ispfe->backend_input)) {
		ret = PTR_ERR(ispfe->backend_input);
		ispfe->backend_input = NULL;
		return dev_err_probe(ispfe->dev, ret,
				     "cannot map camera back-end input\n");
	}

	ispfe->backend_input_size =
		exynos_becore_input_size(ispfe->backend_input);
	if (ispfe->backend_input_size != ISPFE_BACKEND_INPUT_SIZE) {
		ret = -EINVAL;
		goto err_unmap;
	}
	ispfe->backend_spare = dmam_alloc_coherent(ispfe->dev,
						   ispfe->backend_input_size,
						   &ispfe->backend_spare_dma,
						   GFP_KERNEL);
	if (!ispfe->backend_spare) {
		ret = -ENOMEM;
		goto err_unmap;
	}
	if (upper_32_bits(ispfe->backend_spare_dma) ||
	    upper_32_bits(ispfe->backend_spare_dma +
			  ispfe->backend_input_size - 1)) {
		ret = -ERANGE;
		goto err_unmap;
	}

	ret = devm_add_action_or_reset(ispfe->dev, ispfe_backend_unmap, ispfe);
	if (ret)
		return ret;

	return 0;

err_unmap:
	exynos_becore_input_unmap(ispfe->backend_input);
	ispfe->backend_input = NULL;
	return dev_err_probe(ispfe->dev, ret,
			     "invalid camera back-end input allocation\n");
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
	ispfe->pdma_program_staged =
		devm_kmalloc(dev, ISPFE_PDMA_RECIPE_BYTES, GFP_KERNEL);
	ispfe->pdma_blocks_staged =
		devm_kmalloc(dev, ISPFE_PDMA_BLOCKS_BYTES, GFP_KERNEL);
	if (!ispfe->pdma_program_staged || !ispfe->pdma_blocks_staged)
		return -ENOMEM;

	ispfe->dev = dev;
	ispfe->cam_clk = devm_clk_get(dev, "cam");
	if (IS_ERR(ispfe->cam_clk))
		return dev_err_probe(dev, PTR_ERR(ispfe->cam_clk),
				     "cannot get CAM clock\n");
	ispfe->memory_path = devm_of_icc_get(dev, "memory");
	if (IS_ERR(ispfe->memory_path))
		return dev_err_probe(dev, PTR_ERR(ispfe->memory_path),
				     "cannot get memory path\n");
	ispfe->predown_qch = ISPFE_PREDOWN_QCH_OFF;
	ispfe->settle_us = ISPFE_SETTLE_US_DEFAULT;
	ispfe->fc_axi_max_ost = FC_LMP_IDMA_AXI_MAX_OST_IMX712;
	ispfe->pdma_cmd = PDMA_DESC_CMD;
	ispfe->pdma_bytes = ISPFE_PDMA_RECIPE_BYTES;
	ispfe->pdma_bytes_first = ISPFE_PDMA_RECIPE_BYTES;
	platform_set_drvdata(pdev, ispfe);

	ret = devm_mutex_init(dev, &ispfe->lock);
	if (ret)
		return ret;

	spin_lock_init(&ispfe->slock);
	INIT_LIST_HEAD(&ispfe->pending);
	INIT_LIST_HEAD(&ispfe->ready);
	INIT_LIST_HEAD(&ispfe->flight);
	INIT_WORK(&ispfe->fill_work, ispfe_fill_work);
	INIT_LIST_HEAD(&ispfe->backend_ready);
	INIT_LIST_HEAD(&ispfe->backend_flight);
	INIT_LIST_HEAD(&ispfe->backend_done);
	INIT_WORK(&ispfe->backend_fill_work, ispfe_backend_fill_work);
	for (i = 0; i < ARRAY_SIZE(ispfe->backend_buffers); i++)
		INIT_LIST_HEAD(&ispfe->backend_buffers[i].list);
	ispfe->credit_latency = ISPFE_CREDIT_LATENCY_DEFAULT;

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
	ret = ispfe_backend_init(ispfe);
	if (ret)
		return ret;

	/*
	 * The receiver's own resources, which are the driver's to allocate: a
	 * standalone physical-output RAW capture put this sensor on logical
	 * and PDMA channel 0 with frame-controller context 4, and an earlier mixed
	 * preview used channel 1 and context 2.  The FC context directly selects
	 * its LMP-IDMA AXI threshold pair.  The rest of the source description
	 * comes from the device tree's endpoint and from the negotiated format.
	 */
	ispfe->src.loch = 0;
	ispfe->src.fcctx = 4;

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

	ret = ispfe_media_register(ispfe);
	if (ret)
		return ret;

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

	/* A debug queue does not belong to the video device, so stop it first. */
	scoped_guard(mutex, &ispfe->lock) {
		if (ispfe->owner == ISPFE_OWNER_BACKEND) {
			spin_lock_irq(&ispfe->slock);
			ispfe->backend_queue_active = false;
			spin_unlock_irq(&ispfe->slock);
		}
	}
	cancel_work_sync(&ispfe->backend_fill_work);
	scoped_guard(mutex, &ispfe->lock) {
		if (ispfe->owner == ISPFE_OWNER_BACKEND) {
			if (ispfe->sensor_streaming) {
				ispfe_stop(ispfe);
				v4l2_subdev_disable_streams(&ispfe->sd,
							    ISPFE_PAD_SOURCE,
							    BIT_ULL(0));
				ispfe_sensor_power(ispfe, false);
				ispfe->sensor_streaming = false;
			}
			cancel_work_sync(&ispfe->backend_fill_work);
			ispfe_backend_queue_abort_all(ispfe);
			ispfe->owner = ISPFE_OWNER_NONE;
		}
	}
	ispfe_media_unregister(ispfe);
	cancel_work_sync(&ispfe->fill_work);

	scoped_guard(mutex, &ispfe->lock) {
		if (ispfe->sensor_streaming) {
			ispfe_stop(ispfe);
			v4l2_subdev_disable_streams(&ispfe->sd,
						    ISPFE_PAD_SOURCE, BIT_ULL(0));
			ispfe_sensor_power(ispfe, false);
			ispfe->sensor_streaming = false;
			ispfe->owner = ISPFE_OWNER_NONE;
		} else if (ispfe->streaming) {
			ispfe_stop(ispfe);
		}
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
		ret = ispfe_qos_disable(ispfe);
		if (ret)
			dev_warn(ispfe->dev,
				 "cannot restore camera QoS while removing: %d\n",
				 ret);
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
