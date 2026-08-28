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
#include <linux/media/samsung/exynos-ispfe-config.h>
#include <media/v4l2-isp.h>
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
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>
#include <media/exynos-becore.h>
#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#include "exynos-ispfe-pdma-program.h"
#include "exynos-ispfe-pdma-program-binned.h"
#include "exynos-ispfe-pdma-program-backend.h"

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

/*
 * One demultiplexer slot per (virtual channel, data type) the link should
 * accept.  The slot index is not the virtual channel: the field at 16 says
 * which channel feeds the slot, so the main camera's three slots are VC0
 * RAW10, VC1 RAW10 for its phase-detect stream, and VC0 again for the
 * embedded-data line -- and an IMX712's two are both VC0, told apart by data
 * type alone.
 *
 * Pixel mode reads 3 where imx-mipi-csis knows only single, dual and quad --
 * a later-version extension -- and every sensor uses it.  The embedded-data
 * slot additionally asks for 64-bit parallel mode, which is what Samsung's
 * own driver for this IP version sets when a slot carries packets rather than
 * an image.
 */
#define CSIS_ISP_CONFIG_CH(n)		(0x0040 + (n) * 0x10)
#define CSIS_ISPCFG_VIRTUAL_CHANNEL(n)	((n) << 16)
#define CSIS_ISPCFG_PARALLEL_MODE(n)	((n) << 14)
#define CSIS_ISPCFG_PIXEL_MODE(n)	((n) << 12)
#define CSIS_ISPCFG_DATAFORMAT(fmt)	((fmt) << 2)
#define CSIS_ISPCFG_PIXEL_MODE_VAL	3
#define CSIS_ISPCFG_PARALLEL_64BIT	2
#define CSIS_DT_RAW10			0x2b
#define CSIS_DT_EMBEDDED8		0x12

#define CSIS_ISP_RESOL_CH(n)		(0x0044 + (n) * 0x10)
#define CSIS_ISP_RESOL(w, h)		(((h) << 16) | (w))

/*
 * Low-power spacer insertion, from Samsung's register table for this IP
 * version.  The enable is at 31 and the two spacer counts are 15 bits each --
 * so the IMX712s' 0x7fff7fff is the feature off with both counts saturated,
 * and the main camera's 0x80020002 turns it on with two spacers each, which is
 * a C-PHY link asking for a shorter gap than the default.  Both forms are the
 * measured ones; neither is derived from the link rate.
 */
#define CSIS_LRTE_CONFIG		0x0600
#define CSIS_LRTE_CONFIG_OFF		0x7fff7fff
#define CSIS_LRTE_CONFIG_CPHY		0x80020002
/* Debug options, cleared rather than left at whatever the last session set. */
#define CSIS_DBG_OPTION_SUITE		0x0690

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
/*
 * One allocation serves whichever recipe is selected, so it is sized for the
 * largest.  The three stopped agreeing when the back-end recipe stopped
 * carrying the scaler tables its cleared enables no longer ask for.
 */
#define ISPFE_PDMA_MAX_BLOCKS_BYTES					\
	MAX(ISPFE_PDMA_BLOCKS_BYTES, MAX(ISPFE_PDMA_BINNED_BLOCKS_BYTES,	\
					 ISPFE_PDMA_BACKEND_BLOCKS_BYTES))

/*
 * The shading table as the hardware reads it: 33 x 25 x four unsigned Q12
 * gains, four rows of gains to a 32-byte record.
 */
#define ISPFE_LSC_TILE_ROWS		4
#define ISPFE_LSC_TILE_WORDS		(ISPFE_LSC_TILE_ROWS * \
					 EXYNOS_ISPFE_WB_GAINS)
#define ISPFE_LSC_LUT_BYTES		(DIV_ROUND_UP(EXYNOS_ISPFE_LSC_ROWS, \
						      ISPFE_LSC_TILE_ROWS) * \
					 EXYNOS_ISPFE_LSC_COLUMNS * \
					 ISPFE_LSC_TILE_WORDS * 2)

/*
 * One copy per program slot, so that a table sent for one frame does not
 * reach frames whose programs were already encoded.  A slot's program points
 * at its own copy; the shared block area keeps the recipe's table for
 * anything that does not.
 */
#define ISPFE_LSC_SLOT_STRIDE		ALIGN(ISPFE_LSC_LUT_BYTES, PAGE_SIZE)
#define ISPFE_LSC_AREA_BYTES		(PDMA_SLOTS * ISPFE_LSC_SLOT_STRIDE)

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
	/*
	 * An LMP image destination rather than a completion or statistics
	 * area.  The driver does not run these: it states no gate for any of
	 * them and the encoder zeroes their address slots, so they are named
	 * only because the recipes' relocation tables name them, and they have
	 * no size and no allocation.
	 */
	bool tapout;
};

struct ispfe_awb_snapshot_file {
	void *data;
	size_t size;
};

#define PDMA_OUTPUT(_size)	{ .size = (_size) }
#define PDMA_TAPOUT()		{ .tapout = true }

/*
 * The back end's input slot, as it allocates it: one layout of the board's
 * whole array, which is what every producer has to fit inside.
 */
#define ISPFE_BACKEND_INPUT_SIZE		0x01a17000

/*
 * The SBWC header region the back end reads before the image -- one 0x40-byte
 * record per picture row, so the image starts that many bytes into the slot.
 *
 * It follows the *producer's* raster and not the board's array, which is a
 * distinction the ultrawide cannot make: its readout is the array, 4208 x 3120,
 * so both rules give the same number and this was a constant for as long as it
 * was the only camera. A camera that reads out smaller separates them. Two
 * things say which rule is right -- the consumer computes the same offset from
 * the raster its producer latched, and the vendor's own capture of the main
 * camera allocates its header and image 0x40 * 3000 apart.
 */
#define ISPFE_BACKEND_HEADER_STRIDE	0x40
/* Separate full-mode TNR-pyramid output allocation from the same request. */
#define ISPFE_TNR_PYRAMID_SIZE		0x00468000

/*
 * The working areas the front end writes back to, in the order the recipe's
 * ISPFE_BUF_OUTPUT() indices name them.  The sizes are the vendor session's
 * own allocation classes.  Outputs 0--9 are completion/statistics buffers.
 * Outputs 10--12 are the LMP's processed-image destinations --
 * planar linear RGB, YUV420 ML output 0 and interleaved RGB888 ML output 2 --
 * which nothing on this driver's path reads, so they are named without a size:
 * no gate is stated for any of the three and the encoder writes a null address
 * for each.
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
	PDMA_TAPOUT(),
	PDMA_TAPOUT(),
	PDMA_TAPOUT(),
};

#define ISPFE_PDMA_OUTPUT_AWB		4
#define ISPFE_PDMA_OUTPUT_AE		9
/*
 * The same grid a second time, from the tap the shading estimate is computed
 * from.  Lyric allocates this one and the white balance one from a single
 * call, so the two are one writer's output twice over.
 */
#define ISPFE_PDMA_OUTPUT_LSC		5
/*
 * The histogram block writes three regions of interest through one gate, and
 * the recipes relocate all three.  They are consecutive.
 */
#define ISPFE_PDMA_OUTPUT_HISTOGRAM	6
#define ISPFE_PDMA_OUTPUT_HISTOGRAM_ROI1	7
#define ISPFE_PDMA_OUTPUT_HISTOGRAM_ROI2	8
/*
 * The flicker block's destination is named twice by the recipe -- once in its
 * own CSR and once in the batch record -- and the relocation is by buffer
 * name, so both follow this area without anything here saying so.
 */
#define ISPFE_PDMA_OUTPUT_FLICKER	1
/*
 * And the motion map's, named the same way twice for the same reason.
 */
#define ISPFE_PDMA_OUTPUT_MOTION	2

/*
 * Where one frame's statistics land while the metadata node is streaming.
 *
 * The shared allocations above are rewritten every frame, which is why the
 * debug drain needs a two-frame handoff to read one of them without tearing.
 * A per-frame node cannot pay that: it needs somewhere that is quiescent the
 * moment its frame retires.  So a slot that a queue has paired with a buffer
 * writes its statistics into one of these instead, chosen when the slot's
 * program is encoded and released again once the result has been copied out.
 * The dump slot and the debugfs diagnostic keep the shared allocations, so
 * neither changes behaviour.
 *
 * They are private, and userspace never sees one.  The hardware only ever
 * writes memory this driver owns, and a statistics buffer is a copy taken
 * after the frame that filled it was retired -- which is the same boundary
 * the drained snapshot uses, and is why a node can be stopped at any time
 * without proving anything about what the front end is doing.
 */
/*
 * Cacheable, and synced by hand at the two boundaries, because the copy out of
 * here is the most expensive thing this driver does per frame.  These are DMA
 * targets on a device with no `dma-coherent`, so dma_alloc_coherent() can only
 * keep its promise by mapping them uncached -- and reading 576 KiB of uncached
 * memory costs 9.5 ms at 62 MB/s, a quarter of a CPU at 29.44 fps.  The vendor
 * does not pay that either: LWIS picks the cached dma-heap and exposes a
 * ranged dma_buf_begin_cpu_access_partial() for the sync.
 *
 * dma_alloc_noncontiguous() rather than dma_alloc_noncoherent(), because the
 * latter goes through dma_common_alloc_pages() under an IOMMU and would want
 * an order-7 physically contiguous allocation for each of these ten -- a
 * fragmentation failure waiting for uptime.  This asks for scattered pages and
 * one contiguous IOVA, which is all the hardware needs: it is handed a single
 * address, and @sgt->nents is 1.
 */
/* Indices into ispfe_stats_area.grid, in the order the buffer carries them. */
#define ISPFE_STATS_GRID_AWB		0
#define ISPFE_STATS_GRID_AE		1
#define ISPFE_STATS_GRID_HISTOGRAM	2
#define ISPFE_STATS_GRID_FLICKER	3
#define ISPFE_STATS_GRID_LSC		4
#define ISPFE_STATS_GRID_MOTION		5
#define ISPFE_STATS_GRID_HISTOGRAM_ROI1	6
#define ISPFE_STATS_GRID_HISTOGRAM_ROI2	7
#define ISPFE_STATS_GRIDS		8

struct ispfe_stats_area {
	struct list_head list;
	void *grid[ISPFE_STATS_GRIDS];
	struct sg_table *sgt[ISPFE_STATS_GRIDS];
	dma_addr_t dma[ISPFE_STATS_GRIDS];
	u64 timestamp;
	u32 sequence;
	/*
	 * Which streaming session of the metadata node armed this area.  A
	 * frame can retire after that session ended -- the front end is a
	 * different queue and keeps going -- and its statistics belong to the
	 * session that asked for them, not to whoever is streaming when they
	 * land.
	 */
	u32 session;
};

/*
 * One per slot a queue can arm, and one more so that a frame retiring while
 * all four are armed still has somewhere to be copied from.  Running out
 * costs that frame's statistics and nothing else.
 */
#define ISPFE_STATS_AREAS		(PDMA_BUF_SLOTS + 1)

/*
 * A grid is the hardware's 64-byte metadata area followed by 64 x 48 records
 * of 0x60 bytes, which is what the statistics writers bound themselves to.
 * The allocation is larger -- it is the vendor's own allocation class -- and
 * the rest of it is not copied out.
 */
#define ISPFE_STATS_GRID_BYTES		0x48040

/*
 * Each grid a statistics buffer carries: the auxiliary output the hardware
 * writes it to, where it goes in the buffer, and the flag that says it was
 * written.  The two sizes are asserted against each other rather than assumed,
 * because the copy length is the interface's and the destination is the
 * hardware's.
 */
/*
 * A grid holds one complete frame when its header reports the geometry this
 * hardware meters.  The header is the first thing the hardware writes and the
 * driver clears it before arming, so a stale or unwritten area fails this.
 *
 * These run on the *copy* rather than on the area -- see ispfe_stats_publish()
 * -- so what one of them approves is what the caller will read.
 */
static bool ispfe_stats_grid_written(const void *grid)
{
	const struct exynos_ispfe_stats_grid_header *header = grid;

	return header->columns == EXYNOS_ISPFE_STATS_COLUMNS &&
	       header->rows == EXYNOS_ISPFE_STATS_ROWS;
}

/*
 * The histogram has no geometry to check, so what says it was written is the
 * bin count the hardware reports into its metadata area -- nonzero for any
 * frame it wrote, and zero for the clear below.
 *
 * **The sample totals would be the stronger test and are not used.** They sit
 * at the far end of the structure, three pages into a *non-contiguous*
 * allocation, and clearing that far with the ranged
 * `dma_sync_single_for_device()` the arm path uses both syncs physical memory
 * that is not the allocation's and leaves the rest of it dirty over what the
 * hardware wrote. That cost 18% of buffers, intermittently.
 *
 * `dma_sync_sgtable_for_device()` would describe the whole thing correctly, so
 * this is a limit of how the arm path syncs rather than of the DMA API -- the
 * door is closed, not locked. What is traded away by not opening it is
 * completeness: the totals said "the hardware finished", where this says "the
 * hardware started". A torn buffer therefore carries the previous frame's tail
 * rather than zeros, which is why @total is documented as unusable for
 * validity rather than merely unused.
 */
static bool ispfe_stats_histogram_written(const void *grid)
{
	const struct exynos_ispfe_stats_histogram *histogram = grid;

	/*
	 * Masked, because that is what the field means and what a consumer
	 * reads: a word with rubbish in its upper bits and nothing in its low
	 * five would otherwise be published as a one-bin histogram rather than
	 * rejected.
	 */
	return (histogram->bins_log2 & EXYNOS_ISPFE_HISTOGRAM_BINS_MASK) != 0;
}

/*
 * The flicker block reports how many rows it summed, into the same metadata
 * area the grids keep their geometry in, and that is what says it wrote this
 * frame: the driver clears it before arming, so an area the hardware did not
 * reach reads zero.
 *
 * The upper bound is not paranoia about the hardware.  A consumer indexes
 * @row_sum with this count, and a torn or unwritten word here would send it
 * past the end of the array -- so the interface only ever publishes a count
 * the array can hold.  Which is why this has to run on the copy: approving a
 * count in the area and then copying out whatever is there now would be a
 * bound that does not bind.
 */
static bool ispfe_stats_flicker_written(const void *grid)
{
	const struct exynos_ispfe_stats_flicker *flicker = grid;

	return flicker->rows != 0 &&
	       flicker->rows <= EXYNOS_ISPFE_FLICKER_ROWS;
}

/*
 * The motion map is the one result whose row stride is not fixed: the block
 * packs its rows to the map it was configured for, and reports the stride it
 * used.  So three fields say it was written rather than one, and they are
 * checked together -- a consumer indexes @luma with all three, and this is the
 * only place that can bound what it is handed.  As with the flicker count,
 * that makes running on the copy rather than on the DMA area load-bearing.
 */
static bool ispfe_stats_motion_written(const void *grid)
{
	const struct exynos_ispfe_stats_motion *motion = grid;
	size_t stride = motion->stride;

	if (!stride || !motion->columns || !motion->rows)
		return false;
	if (motion->columns > EXYNOS_ISPFE_MOTION_COLUMNS ||
	    motion->rows > EXYNOS_ISPFE_MOTION_ROWS)
		return false;

	/*
	 * A stride wider than the whole array first, so that neither product
	 * below can overflow whatever width size_t has here.  The hardware
	 * wrote this word into memory the CPU does not own, which is the whole
	 * reason this function exists, so a torn 0x02000000 with 128 rows
	 * multiplying to exactly 2^32 is a case rather than a curiosity.
	 */
	if (stride > sizeof(motion->luma))
		return false;

	/*
	 * The two bounds the documented index formula needs: a row's cells fit
	 * inside its own stride, and the last row fits inside the array.
	 * Together they cover every index it can produce, which is the whole
	 * of what a consumer needs from here.
	 */
	if ((size_t)motion->columns * sizeof(motion->luma[0]) > stride)
		return false;

	return (size_t)motion->rows * stride <= sizeof(motion->luma);
}

static const struct ispfe_stats_grid {
	unsigned int output;
	size_t offset;
	size_t size;
	/*
	 * How much of the area the driver clears before the frame is armed,
	 * which has to cover whatever @written reads.
	 */
	size_t clear;
	u32 flag;
	bool (*written)(const void *grid);
} ispfe_stats_grids[ISPFE_STATS_GRIDS] = {
	[ISPFE_STATS_GRID_AWB] = {
		.output = ISPFE_PDMA_OUTPUT_AWB,
		.offset = offsetof(struct exynos_ispfe_stats_buffer, awb),
		.size = sizeof(struct exynos_ispfe_stats_awb),
		.clear = sizeof(struct exynos_ispfe_stats_grid_header),
		.flag = EXYNOS_ISPFE_STATS_AWB,
		.written = ispfe_stats_grid_written,
	},
	[ISPFE_STATS_GRID_AE] = {
		.output = ISPFE_PDMA_OUTPUT_AE,
		.offset = offsetof(struct exynos_ispfe_stats_buffer, ae),
		.size = sizeof(struct exynos_ispfe_stats_ae),
		.clear = sizeof(struct exynos_ispfe_stats_grid_header),
		.flag = EXYNOS_ISPFE_STATS_AE,
		.written = ispfe_stats_grid_written,
	},
	[ISPFE_STATS_GRID_HISTOGRAM] = {
		.output = ISPFE_PDMA_OUTPUT_HISTOGRAM,
		.offset = offsetof(struct exynos_ispfe_stats_buffer, histogram),
		.size = sizeof(struct exynos_ispfe_stats_histogram),
		.flag = EXYNOS_ISPFE_STATS_HISTOGRAM,
		.written = ispfe_stats_histogram_written,
		.clear = sizeof(struct exynos_ispfe_stats_grid_header),
	},
	[ISPFE_STATS_GRID_FLICKER] = {
		.output = ISPFE_PDMA_OUTPUT_FLICKER,
		.offset = offsetof(struct exynos_ispfe_stats_buffer, flicker),
		.size = sizeof(struct exynos_ispfe_stats_flicker),
		.flag = EXYNOS_ISPFE_STATS_FLICKER,
		.written = ispfe_stats_flicker_written,
		/*
		 * Named in this structure's own terms and not the grid
		 * header's: the metadata area is in the same place and is the
		 * same size, and that is all the two have in common -- the
		 * kernel-doc says explicitly that this does not have the
		 * grids' layout, so borrowing their type here would be a claim
		 * the interface refuses to make.
		 */
		.clear = offsetofend(struct exynos_ispfe_stats_flicker,
				     reserved1),
	},
	[ISPFE_STATS_GRID_MOTION] = {
		.output = ISPFE_PDMA_OUTPUT_MOTION,
		.offset = offsetof(struct exynos_ispfe_stats_buffer, motion),
		.size = sizeof(struct exynos_ispfe_stats_motion),
		.flag = EXYNOS_ISPFE_STATS_MOTION,
		.written = ispfe_stats_motion_written,
		/* Named in this structure's own terms, as the flicker one is. */
		.clear = offsetofend(struct exynos_ispfe_stats_motion,
				     reserved1),
	},
	[ISPFE_STATS_GRID_LSC] = {
		.output = ISPFE_PDMA_OUTPUT_LSC,
		.offset = offsetof(struct exynos_ispfe_stats_buffer, lsc),
		.size = sizeof(struct exynos_ispfe_stats_lsc),
		.clear = sizeof(struct exynos_ispfe_stats_grid_header),
		.flag = EXYNOS_ISPFE_STATS_LSC,
		.written = ispfe_stats_grid_written,
	},
	[ISPFE_STATS_GRID_HISTOGRAM_ROI1] = {
		.output = ISPFE_PDMA_OUTPUT_HISTOGRAM_ROI1,
		.offset = offsetof(struct exynos_ispfe_stats_buffer,
				   histogram_roi1),
		.size = sizeof(struct exynos_ispfe_stats_histogram),
		.flag = EXYNOS_ISPFE_STATS_HISTOGRAM_ROI1,
		.written = ispfe_stats_histogram_written,
		.clear = sizeof(struct exynos_ispfe_stats_grid_header),
	},
	[ISPFE_STATS_GRID_HISTOGRAM_ROI2] = {
		.output = ISPFE_PDMA_OUTPUT_HISTOGRAM_ROI2,
		.offset = offsetof(struct exynos_ispfe_stats_buffer,
				   histogram_roi2),
		.size = sizeof(struct exynos_ispfe_stats_histogram),
		.flag = EXYNOS_ISPFE_STATS_HISTOGRAM_ROI2,
		.written = ispfe_stats_histogram_written,
		.clear = sizeof(struct exynos_ispfe_stats_grid_header),
	},
};

static_assert(sizeof(struct exynos_ispfe_stats_awb) == ISPFE_STATS_GRID_BYTES);
static_assert(sizeof(struct exynos_ispfe_stats_ae) == ISPFE_STATS_GRID_BYTES);
static_assert(sizeof(struct exynos_ispfe_stats_lsc) == ISPFE_STATS_GRID_BYTES);
/*
 * The layout closes against the vendor's own bound: `LmpRgbyHistogramStatsOutput`
 * refuses a buffer of 258 32-byte units or fewer, and this is the smallest that
 * holds four 512-bin planes and their totals behind the hardware's metadata.
 */
static_assert(sizeof(struct exynos_ispfe_stats_histogram) == 0x2050);
/*
 * The same bound from the other side: `LmpFlickerStatsOutput::Make` refuses a
 * buffer of fewer than 0x46c0 bytes whatever row count it is asked for, which
 * is exactly the metadata area plus %EXYNOS_ISPFE_FLICKER_ROWS sums.
 */
static_assert(sizeof(struct exynos_ispfe_stats_flicker) == 0x46c0);
/*
 * And the two offsets inside it, because the size alone does not pin them:
 * moving a word from @reserved1 to @reserved0 keeps the structure exactly
 * 0x46c0 bytes and silently publishes a reserved word as the row count.
 */
static_assert(offsetof(struct exynos_ispfe_stats_flicker, rows) == 0x24);
static_assert(offsetof(struct exynos_ispfe_stats_flicker, row_sum) == 0x40);
/*
 * And the motion map, from the same reader:
 * `LmpMotionMeteringStatsOutput::Make` refuses a buffer under 0x18040 bytes,
 * `ValidateMetadata` reads the stride at 0x24 and the two dimensions at 0x28
 * and 0x2a, and `GetRegionLumaSignal` indexes 16-bit cells from 0x40.
 */
static_assert(sizeof(struct exynos_ispfe_stats_motion) == 0x18040);
static_assert(offsetof(struct exynos_ispfe_stats_motion, stride) == 0x24);
static_assert(offsetof(struct exynos_ispfe_stats_motion, columns) == 0x28);
static_assert(offsetof(struct exynos_ispfe_stats_motion, rows) == 0x2a);
static_assert(offsetof(struct exynos_ispfe_stats_motion, luma) == 0x40);

struct ispfe_lmp_wbg_profile {
	u32 red;
	u32 green_red;
	u32 green_blue;
	u32 blue;
};

/*
 * What the two metering grids exclude, which is the only part of their
 * configuration that is not geometry.  Held in the units the hardware compares
 * in -- signed for the sample thresholds, unsigned for the luma window -- so
 * that nothing between here and the payload has to convert.
 */
struct ispfe_lmp_metering_profile {
	s16 awb_saturation;
	s16 awb_dark;
	u16 awb_luma_coeff[4];
	u16 awb_luma_low;
	u16 awb_luma_high;
	u16 awb_diff_coring;
	s16 ae_saturation;
	s16 ae_dark;
};

/*
 * The vendor's, and identical in all five captured stats payloads across two
 * sensor rasters -- which is what says they are tuning rather than geometry.
 * As shipped the exposure grid counts every sample and the white balance grid
 * discards only what sits at either end of its range.
 */
static const struct ispfe_lmp_metering_profile ispfe_lmp_metering_captured = {
	.awb_saturation = 32256,
	.awb_dark = 0,
	.awb_luma_coeff = { 0, 0, 0, 256 },
	.awb_luma_low = 0,
	.awb_luma_high = 65535,
	.awb_diff_coring = 0,
	.ae_saturation = 32767,
	.ae_dark = -32768,
};

/* The two LMP WBG stages use the same captured unsigned-Q12 gains. */
static const struct ispfe_lmp_wbg_profile ispfe_lmp_wbg_backend = {
	.red = 8473,
	.green_red = 4096,
	.green_blue = 4096,
	.blue = 6851,
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
	const struct ispfe_lmp_wbg_profile *lmp_wbg;
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
		.lmp_wbg = &ispfe_lmp_wbg_backend,
		.fixed_resources = true,
		.required_loch = 0,
		.required_fcctx = 3,
		.required_fc_axi_max_ost = FC_LMP_IDMA_AXI_MAX_OST_IMX712,
	},
};

/*
 * The program area is sized once for all recipes; the two raw ones happen to
 * agree, and if a future capture does not these say so at build time rather
 * than by overrunning an allocation.  The block area is the largest of the
 * three by construction, so there is nothing to assert about it.
 */
static_assert(ISPFE_PDMA_BINNED_RECIPE_BYTES == ISPFE_PDMA_RECIPE_BYTES);
static_assert(ISPFE_PDMA_BINNED_BLOCKS_BYTES == ISPFE_PDMA_BLOCKS_BYTES);
static_assert(ISPFE_PDMA_BACKEND_RECIPE_BYTES <= PDMA_SLOT_STRIDE);

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
 * One camera as the board describes it: which link bank its data arrives on --
 * that is the device tree port number -- and what the link and its mode look
 * like.  Fixed at probe and never written afterwards.
 *
 * Kept apart from struct ispfe_source below, which is what a *stream* runs
 * with.  That one carries the receiver's own allocations as well, and is
 * writable from debugfs at any time so a value can be swept between attempts;
 * this is the board's statement and there is nothing to sweep in it.
 *
 * Indexed by bank rather than packed, so an entry's position is the hardware
 * fact it describes and matches ispfe_link_cfg[] above.
 */
struct ispfe_link {
	bool present;
	/* Which of the receiver's sink pads this camera arrives on. */
	u16 pad;
	u32 bank;
	u32 phy;
	u32 lanes;
	bool cphy;
	u32 mode_word0;
	u32 mode_word1;

	/*
	 * The sensor on this link, and which of its pads the link comes from.
	 * Written by the notifier under the driver's own lock, and read from
	 * the capture queue, which holds that lock for the whole of start and
	 * stop -- so an unbind cannot take a subdev away from under a stream,
	 * it waits for the stream to finish instead.
	 */
	struct v4l2_subdev *sensor;
	u32 sensor_pad;
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
/*
 * Sink pads come first, one per camera the device tree describes, in ascending
 * bank order; the source pad follows them.  So the source's index depends on
 * how many cameras there are and cannot be a constant -- it is
 * ispfe->source_pad, and with one camera the layout is the one this driver has
 * always had, sink 0 and source 1.
 */
#define ISPFE_PAD_SINK			0
#define ISPFE_MAX_PADS			(CSIS_NUM_LINKS + 1)

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
	/* Every camera the device tree describes, and how many that is. */
	struct ispfe_link links[CSIS_NUM_LINKS];
	unsigned int num_links;
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
	 * it remains a knob for diagnostic/raw streams.  A fixed BE-core
	 * consumer selects and latches its profile-owned value instead.
	 */
	u32 fc_axi_max_ost;
	u32 active_fc_axi_max_ost;
	struct ispfe_lmp_wbg_profile active_lmp_wbg;
	struct ispfe_lmp_metering_profile active_lmp_metering;
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
	 * PDMA_SLOTS copies of the shading table, one per program slot, and the
	 * live one a slot's copy is taken from.  @shading is a whole tiled
	 * table rather than a grid because tiling it is the expensive half and
	 * a parameters buffer has already done it; it is swapped with the
	 * buffer's own allocation under @slock, which keeps exactly one owner
	 * per allocation and never leaves a live table in a buffer userspace is
	 * about to get back.
	 */
	void *lsc;
	dma_addr_t lsc_dma;
	u8 *shading;
	/*
	 * Unity everywhere, in the hardware's tiled layout: what a stream
	 * starts on, and what a disabled parameters block asks for.  Built once
	 * because it is the same table for every recipe -- it is the *absence*
	 * of a calibration rather than one.
	 */
	u8 *lsc_unity;
	int lsc_input;
	/*
	 * Byte offsets within a program of the frame destination's two address
	 * halves, so a destination can be retargeted without searching the
	 * program for an address.
	 */
	u32 bayer_lo;
	u32 bayer_hi;
	/* AWB statistics are redirected here before their snapshot is published. */
	void *awb_spare;
	dma_addr_t awb_spare_dma;
	u32 awb_lo;
	u32 awb_hi;
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
	bool backend_queue_consumer;
	/*
	 * The consumer holds the front end from its own STREAMON, before it has
	 * anything to stream.  While this is set `owner` is ISPFE_OWNER_BACKEND
	 * with nothing armed, and a backend stream starting and stopping inside
	 * it must not take or give back what it did not claim.  Under `lock`.
	 */
	bool backend_reserved;
	void *backend_spare;
	dma_addr_t backend_spare_dma;
	size_t backend_input_size;
	void *tnr_pyramid;
	dma_addr_t tnr_pyramid_dma;
	bool backend_producing;
	bool backend_handed_off;
	/*
	 * Whether a recipe that was captured without one is asked to carry the
	 * LMP main-Bayer side output for a back-end consumer.  The back-end
	 * recipe carries its own and ignores this; the raw recipe's is a graft,
	 * and a full-mode raw stream carrying it produces no LMP event at all
	 * (see ispfe_pdma_apply_backend_output()).  Off by default, so an
	 * ordinary raw capture programs only the destination it was asked for.
	 */
	bool backend_side_output;
	bool active_backend_side_output;
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
	struct media_pad pads[ISPFE_MAX_PADS];
	u16 source_pad;
	/*
	 * The camera whose sink link is enabled, which is the one a stream will
	 * run.  Written from .link_setup under the media core's graph mutex and
	 * read at stream start; a link cannot be changed while a pad is
	 * streaming, so it does not move under a session.
	 */
	struct ispfe_link *active_link;
	struct v4l2_async_notifier notifier;
	struct video_device vdev;
	struct media_pad vdev_pad;
	struct media_pipeline pipe;
	struct vb2_queue queue;
	struct v4l2_pix_format fmt;
	struct work_struct fill_work;
	u32 sequence;
	/* The statistics metadata node, and the private areas that feed it. */
	struct video_device stats_vdev;
	struct media_pad stats_pad;
	struct media_pad params_pad;
	struct vb2_queue stats_queue;
	/*
	 * The statistics queue's own lock, because that queue can be started
	 * and stopped while the capture queue holds ispfe->lock for the whole
	 * of its own start.
	 */
	struct mutex stats_lock;
	struct work_struct stats_work;
	struct ispfe_stats_area stats_areas[ISPFE_STATS_AREAS];
	struct ispfe_stats_area *stats_slot[PDMA_BUF_SLOTS];
	struct list_head stats_pending;
	struct list_head stats_free;
	struct list_head stats_captured;
	bool stats_streaming;
	u32 stats_session;

	/*
	 * The parameters node, which is one block type deep: LMP's white
	 * balance gains.  A buffer is consumed by the next program encode
	 * rather than by a frame, which is the same thing -- the front end
	 * encodes one program per frame.
	 */
	struct video_device params_vdev;
	struct mutex params_lock;
	struct vb2_queue params_queue;
	struct list_head params_pending;
	bool params_streaming;
	u32 stats_published;
	u32 stats_empty;
	u32 stats_dropped;
	/*
	 * Areas whose frame retired after the session that armed them ended,
	 * which an earlier driver returned to the pool at STREAMOFF while the
	 * front end was still writing into them.  A non-zero count is this
	 * node having been stopped under a running capture, not a fault.
	 *
	 * It is a lower bound on that population rather than the whole of it:
	 * an area that retires *during* the gap still matches the session, so
	 * it goes back through the STREAMOFF splice without being counted.
	 * Cycling the node back to back counts most of them; leaving it down
	 * for longer than the retire depth counts none.
	 */
	u32 stats_stale;
	/*
	 * The last frame number published on this node, which is what a
	 * buffer's `sequence` carries -- V4L2's sequence is the frame number,
	 * and a consumer pairing a frame with the settings it was taken at has
	 * nothing else it can read without mapping the payload.
	 *
	 * Held here rather than taken from ispfe->sequence at publish time
	 * because a buffer that carries no frame still has to be numbered, and
	 * it repeats this instead of inventing one.
	 *
	 * It restarts when the *front end's* stream does, which is a real
	 * discontinuity for a consumer that keeps this node streaming across
	 * two captures: the frame counter is per stream, so a second capture
	 * numbers from zero again.  That is the same discontinuity
	 * frame_sequence in the payload has always had, and the buffer's
	 * timestamp is the pairing key that survives it.
	 */
	u32 stats_sequence;
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
	/*
	 * Every credit, not just the ones that carried a buffer.  A dump credit
	 * still consumes a frame, so it still ages whatever is in flight -- see
	 * ispfe_queue_credit().
	 */
	u64 credit_count;
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

/* A camera's description, as the values a stream starts from. */
static void ispfe_link_to_src(struct ispfe_device *ispfe,
			      const struct ispfe_link *link)
{
	ispfe->src.link = link->bank;
	ispfe->src.phy = link->phy;
	ispfe->src.lanes = link->lanes;
	ispfe->src.cphy = link->cphy;
	ispfe->src.mode_word0 = link->mode_word0;
	ispfe->src.mode_word1 = link->mode_word1;
}

/* The camera on a given sink pad, or NULL if that is not a sink pad of ours. */
static struct ispfe_link *ispfe_link_by_pad(struct ispfe_device *ispfe,
					    unsigned int pad)
{
	unsigned int i;

	for (i = 0; i < CSIS_NUM_LINKS; i++)
		if (ispfe->links[i].present && ispfe->links[i].pad == pad)
			return &ispfe->links[i];

	return NULL;
}

/*
 * The camera the enabled sink link selects.
 *
 * Maintained from .link_setup rather than walked for, because the media core
 * calls that under its own graph mutex and a stream start would otherwise have
 * to take it -- and because a link cannot be changed while a pad is streaming,
 * so what this returns is stable for the whole of a session.
 */
static struct ispfe_link *ispfe_active_link(struct ispfe_device *ispfe)
{
	return READ_ONCE(ispfe->active_link);
}

/*
 * The sink pad the source mirrors: the enabled camera's, or the first if
 * userspace has disabled every link, so that the format ops still answer.
 */
static unsigned int ispfe_active_sink_pad(struct ispfe_device *ispfe)
{
	struct ispfe_link *link = ispfe_active_link(ispfe);

	return link ? link->pad : ISPFE_PAD_SINK;
}

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
 * is one code path with the geometry as its only argument.
 *
 * The main camera differs in exactly one word of them, the master block's
 * +0x10, and in using three lane blocks instead of four.  Neither value is
 * decoded far enough to derive from the link rate, so both are carried as the
 * measured pair and chosen by the link type.
 */
#define PHY_MASTER_MODE			0x10
#define PHY_MASTER_MODE_DPHY		0x00000200
#define PHY_MASTER_MODE_CPHY		0x00000240

struct ispfe_reg {
	u32 off;
	u32 val;
};

static const struct ispfe_reg ispfe_phy_master[] = {
	{ 0x00, 0x00000010 }, { 0x04, 0x00000110 }, { 0x08, 0x00003223 },
	{ 0x0c, 0x00000000 }, { PHY_MASTER_MODE, PHY_MASTER_MODE_DPHY },
	{ 0x14, 0x00000000 },
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
 * The shared master block, with the one word that depends on the link type
 * substituted as it goes out rather than corrected afterwards: this driver's
 * write set is diffed against the vendor's register for register, and a D-PHY
 * value followed by a C-PHY one is a difference where the hardware has none.
 */
static void ispfe_phy_write_master(void __iomem *base, bool cphy)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(ispfe_phy_master); i++) {
		u32 val = ispfe_phy_master[i].val;

		if (ispfe_phy_master[i].off == PHY_MASTER_MODE && cphy)
			val = PHY_MASTER_MODE_CPHY;

		writel_relaxed(val, base + ispfe_phy_master[i].off);
	}
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

	ispfe_phy_write_master(csis + PHY_MASTER, ispfe->active.cphy);

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
 * One CSIS link, and two demultiplexer slots on its first virtual channel: the
 * image in RAW10 and the sensor's one line of embedded data, which the
 * receiver has to be told about or the image slot is handed a frame that is
 * one line too tall.  Nothing consumes the second slot -- it has no logical
 * channel behind it, which is also what the vendor stack does with it.
 */
static void ispfe_link_start(struct ispfe_device *ispfe)
{
	void __iomem *link = ispfe_link(ispfe);
	u32 ctrl;

	writel_relaxed(0, link + CSIS_DBG_OPTION_SUITE);

	writel_relaxed(CSIS_ISPCFG_PIXEL_MODE(CSIS_ISPCFG_PIXEL_MODE_VAL) |
		       CSIS_ISPCFG_DATAFORMAT(CSIS_DT_RAW10),
		       link + CSIS_ISP_CONFIG_CH(0));
	writel_relaxed(CSIS_ISP_RESOL(ispfe->active.width, ispfe->active.height),
		       link + CSIS_ISP_RESOL_CH(0));

	writel_relaxed(CSIS_ISPCFG_PIXEL_MODE(CSIS_ISPCFG_PIXEL_MODE_VAL) |
		       CSIS_ISPCFG_PARALLEL_MODE(CSIS_ISPCFG_PARALLEL_64BIT) |
		       CSIS_ISPCFG_DATAFORMAT(CSIS_DT_EMBEDDED8),
		       link + CSIS_ISP_CONFIG_CH(1));
	writel_relaxed(CSIS_ISP_RESOL(ispfe->active.width, 1),
		       link + CSIS_ISP_RESOL_CH(1));

	writel_relaxed(ispfe->active.cphy ? CSIS_LRTE_CONFIG_CPHY
					 : CSIS_LRTE_CONFIG_OFF,
		       link + CSIS_LRTE_CONFIG);
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
	/* The credit count at which this buffer's frame has been and gone. */
	u64 retire_credit;
};

static struct ispfe_buffer *to_ispfe_buffer(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct ispfe_buffer, vb);
}

/* ---- per-frame statistics areas ----------------------------------------- */

/*
 * Put an area back where a slot can take it again.  A detached node is always
 * list_del_init()ed, so "not on a list" is the same test as "empty", and a
 * reset that has already reclaimed this area cannot be undone by a late put.
 */
static void ispfe_stats_area_put_locked(struct ispfe_device *ispfe,
					struct ispfe_stats_area *area)
{
	if (list_empty(&area->list))
		list_add_tail(&area->list, &ispfe->stats_free);
}

/*
 * Take an area for a slot that is about to be encoded, and clear the metadata
 * header of each grid in it so that a grid the hardware does not write this
 * frame reports no geometry rather than the last frame's.
 *
 * Nothing is taken while the metadata node is not streaming: an ordinary
 * capture then keeps writing its statistics into the shared allocations, which
 * is exactly what it did before this node existed.
 */
static struct ispfe_stats_area *ispfe_stats_take(struct ispfe_device *ispfe,
						 unsigned int slot)
{
	struct ispfe_stats_area *area;
	unsigned int grid;

	if (WARN_ON_ONCE(slot >= PDMA_BUF_SLOTS))
		return NULL;

	scoped_guard(spinlock_irqsave, &ispfe->slock) {
		if (!ispfe->stats_streaming ||
		    WARN_ON_ONCE(ispfe->stats_slot[slot]))
			return NULL;
		area = list_first_entry_or_null(&ispfe->stats_free,
						struct ispfe_stats_area, list);
		if (!area) {
			ispfe->stats_dropped++;
			return NULL;
		}
		list_del_init(&area->list);
		area->session = ispfe->stats_session;
		ispfe->stats_slot[slot] = area;
	}

	/*
	 * The area is cacheable, so the cleared headers have to be written
	 * back before the hardware is pointed at them: they are what says a
	 * grid was *not* written this frame, and a copy left dirty in the
	 * cache would be read back over whatever the hardware did write.
	 * Cleaning also leaves the lines clean rather than dirty, which is
	 * what makes the invalidate in ispfe_stats_publish() safe to discard.
	 *
	 * The header only, and not the allocation, because this runs on the
	 * frame path: the encode behind it has to reach the ready list before
	 * the next frame start credits a program.  Cleaning all 292 KiB to
	 * publish 64 bytes of it is 4,672 cache lines a grid rather than one.
	 * Measured, that made no difference to the frame rate -- it is not
	 * where the margin goes -- so this is the cheap way to be right rather
	 * than a fix for anything.  Nothing else here is ever dirty: the body
	 * is written once, by the zeroing at allocation, and handed to the
	 * device there; the only other CPU access to an area is the read in
	 * ispfe_stats_publish(), and a read allocates clean lines.  So the
	 * header is the whole of what a writeback has to cover.
	 *
	 * A ranged sync of one IOVA is only meaningful while it stays inside
	 * a page, because the pages behind it are not contiguous.  Two
	 * conjuncts, and only the first is asserted below: the offset has to
	 * be zero as well, which it is because the grid's address *is* the
	 * allocation's.
	 *
	 * The DMA API asks for the whole buffer to be handed back with
	 * dma_sync_sgtable_for_device() and this deliberately does not.  What
	 * makes that safe is that the device direction is only ever a clean on
	 * this architecture, so the sole thing skipping it can cost is a lost
	 * dirty line -- and the body is never dirty.
	 */
	BUILD_BUG_ON(sizeof(struct exynos_ispfe_stats_grid_header) > PAGE_SIZE);
	for (grid = 0; grid < ISPFE_STATS_GRIDS; grid++) {
		size_t clear = ispfe_stats_grids[grid].clear;

		/*
		 * The BUILD_BUG_ON above asserts the size of a *type*, which is
		 * not the value handed to the sync below -- so it did not catch
		 * a descriptor whose clear ran past the first page, and would
		 * not catch the next one. This is the quantity that matters.
		 */
		if (WARN_ON_ONCE(clear > PAGE_SIZE))
			clear = PAGE_SIZE;

		memset(area->grid[grid], 0, clear);
		dma_sync_single_for_device(ispfe->dev, area->dma[grid], clear,
					   DMA_TO_DEVICE);
	}

	return area;
}

/* Detach a slot's area without publishing it. */
static void ispfe_stats_untake_locked(struct ispfe_device *ispfe,
				      unsigned int slot)
{
	struct ispfe_stats_area *area;

	if (WARN_ON_ONCE(slot >= PDMA_BUF_SLOTS))
		return;
	area = ispfe->stats_slot[slot];
	if (!area)
		return;
	ispfe->stats_slot[slot] = NULL;
	ispfe_stats_area_put_locked(ispfe, area);
}

static void ispfe_stats_untake(struct ispfe_device *ispfe, unsigned int slot)
{
	guard(spinlock_irqsave)(&ispfe->slock);
	ispfe_stats_untake_locked(ispfe, slot);
}

/* Every armed area at once, for a stream that is not going to finish them. */
static void ispfe_stats_untake_all_locked(struct ispfe_device *ispfe)
{
	unsigned int slot;

	for (slot = 0; slot < PDMA_BUF_SLOTS; slot++)
		ispfe_stats_untake_locked(ispfe, slot);
}

/*
 * A frame has retired, so the area its slot was writing into is quiescent by
 * exactly the argument that lets its image buffer be handed back: the credit
 * that placed the frame has been followed by enough later ones to have
 * finished it, and nothing credits the slot again until it has been encoded
 * afresh.  Detach it for the work item to copy out.
 *
 * Returns whether there is now something for that work item to do.
 */
static bool ispfe_stats_capture_locked(struct ispfe_device *ispfe,
				       unsigned int slot, u64 timestamp,
				       u32 sequence)
{
	struct ispfe_stats_area *area;

	if (WARN_ON_ONCE(slot >= PDMA_BUF_SLOTS))
		return false;
	area = ispfe->stats_slot[slot];
	if (!area)
		return false;
	ispfe->stats_slot[slot] = NULL;
	/*
	 * This is also the moment an area armed by a session that has since
	 * ended becomes reclaimable, and the only one: until the frame retires
	 * the front end may still be writing into it.  Reclaimed here rather
	 * than published, because the buffers waiting now belong to somebody
	 * who asked after that frame had already been metered.
	 */
	if (area->session != ispfe->stats_session) {
		ispfe->stats_stale++;
		ispfe_stats_area_put_locked(ispfe, area);
		return false;
	}
	/*
	 * Ordering, not visibility: the area is cacheable, and what makes the
	 * frame's writes readable through this mapping is the invalidate in
	 * ispfe_stats_publish() rather than any barrier here.
	 */
	dma_rmb();
	area->timestamp = timestamp;
	area->sequence = sequence;
	list_add_tail(&area->list, &ispfe->stats_captured);

	return true;
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
	ispfe->credit_count++;
	if (!list_empty(&ispfe->ready)) {
		buf = list_first_entry(&ispfe->ready, struct ispfe_buffer, list);
		list_move_tail(&buf->list, &ispfe->flight);
		/*
		 * Clamped for the same reason the back end clamps it: a latency
		 * the ring cannot reach would leave the buffer in flight for
		 * ever and DQBUF blocked with nothing said.
		 *
		 * Snapshotted per buffer, so a latency *raised* while frames
		 * are in flight cannot strand what is already there.  Lowering
		 * it reaches only buffers credited afterwards, which can leave
		 * one behind the head due earlier than the head is -- retiring
		 * the head alone then holds it for up to the difference, and it
		 * is stamped that many frames late.  Bounded, self-correcting
		 * and only reachable from debugfs.
		 */
		buf->retire_credit = ispfe->credit_count +
			min_t(u32, READ_ONCE(ispfe->credit_latency),
			      PDMA_BUF_SLOTS - 1);
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
	u64 timestamp = ktime_get_ns();
	bool captured = false;

	spin_lock(&ispfe->slock);
	/* Dump credits age a real buffer too; they are backpressure, not a stall. */
	if (!list_empty(&ispfe->flight)) {
		buf = list_first_entry(&ispfe->flight, struct ispfe_buffer,
				       list);
		if (ispfe->credit_count >= buf->retire_credit) {
			list_del(&buf->list);
			ispfe->flight_count--;
			captured = ispfe_stats_capture_locked(ispfe, buf->slot,
							      timestamp,
							      ispfe->sequence);
			__clear_bit(buf->slot, &ispfe->slots_used);
		} else {
			buf = NULL;
		}
	}
	spin_unlock(&ispfe->slock);

	if (captured)
		queue_work(system_dfl_long_wq, &ispfe->stats_work);

	if (!buf)
		return;

	/*
	 * The frame counter, not a count of buffers handed back: frames that
	 * landed in the dump slot because nothing was queued then show up as a
	 * gap, which is what a sequence number is for.  A statistics buffer for
	 * the same frame carries both, which is what pairs the two.
	 *
	 * Both name this buffer's own frame at the default credit latency,
	 * which is what a consumer pairing a frame with the sensor settings it
	 * was taken at depends on.  The counter advanced at this same
	 * end-of-frame, and the buffer being handed back is the one the credit
	 * before the newest placed -- so the two cancel, and they stop
	 * cancelling by one frame for every credit of latency above the
	 * default.
	 *
	 * That cancellation holds only because a buffer is now aged by every
	 * credit.  It used to be aged only by credits that carried a buffer, so
	 * a queue that ran dry left one in flight across the dump frames while
	 * this counter kept counting them, and it was handed back naming a
	 * frame several later than the one it held -- with its statistics
	 * carrying the same wrong number.  Measured: labels 62, 66, 70 against
	 * grids whose hardware frame ids were 59, 62, 66.
	 */
	buf->vb.vb2_buf.timestamp = timestamp;
	buf->vb.sequence = ispfe->sequence;
	buf->vb.field = V4L2_FIELD_NONE;
	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, ispfe->fmt.sizeimage);
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

	/* A slot is free now, so a pending buffer can be encoded into it. */
	queue_work(system_highpri_wq, &ispfe->fill_work);
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
		queue_work(system_highpri_wq, &ispfe->backend_fill_work);
}

static void ispfe_backend_queue_complete(struct ispfe_device *ispfe)
{
	struct ispfe_backend_buffer *buf = NULL;
	u64 timestamp = ktime_get_ns();
	bool captured = false;

	spin_lock(&ispfe->slock);
	/* Dump credits age a real buffer too; they are backpressure, not a stall. */
	if (!list_empty(&ispfe->backend_flight)) {
		buf = list_first_entry(&ispfe->backend_flight,
				       struct ispfe_backend_buffer, list);
		if (ispfe->backend_credit_count >= buf->retire_credit) {
			list_move_tail(&buf->list, &ispfe->backend_done);
			buf->state = ISPFE_BACKEND_BUFFER_DONE;
			ispfe->backend_flight_count--;
			/*
			 * Which frame this is, for the consumer to carry to
			 * the frame it makes from it.  The same number and the
			 * same timestamp the statistics for this frame get,
			 * taken here because this is the moment both are known.
			 */
			buf->ticket.sequence = ispfe->sequence;
			buf->ticket.timestamp = timestamp;
			/*
			 * The program slot is not released until the fill work
			 * has handed the frame on, so nothing re-encodes it in
			 * between and its statistics stay where they were
			 * written.
			 */
			captured = ispfe_stats_capture_locked(ispfe,
							      buf->program_slot,
							      timestamp,
							      ispfe->sequence);
		} else {
			buf = NULL;
		}
	}
	spin_unlock(&ispfe->slock);

	if (captured)
		queue_work(system_dfl_long_wq, &ispfe->stats_work);
	if (buf)
		queue_work(system_highpri_wq, &ispfe->backend_fill_work);
}

/*
 * Tell userspace a frame has started, and which one.  This is the only moment
 * at which a sensor register write can be placed against a known frame: the
 * IMX712 latches exposure and gain at a frame boundary, so a value written
 * during frame n reaches frame n + 2, and without this event the only anchor
 * userspace has is when a *completed* buffer arrived -- two frame boundaries
 * later, with no way to tell which side of the nearer one a write fell.
 *
 * The number is the frame-start count rather than the end-of-frame counter the
 * buffers carry, because the two are separate interrupts and this one must not
 * read a value the other is writing.  They agree: both are reset together for
 * a session and both count every frame, so frame start n is the frame a buffer
 * numbered n will carry.
 *
 * This is the one thing an interrupt handler here reaches outside the device
 * structure, so the lifetime is worth stating.  The handler exists only
 * between ispfe_start() and ispfe_stop(), and free_irq() waits for one that is
 * running -- so the node it queues into has to outlive the stop, not the
 * probe.  It does for both owners that a user can reach: a V4L2 stream is
 * stopped by vb2_video_unregister_device(), which ispfe_media_unregister()
 * calls before it gets as far as the subdev, and a back-end stream is stopped
 * earlier still, in ->remove before that function is called at all.  The
 * debugfs owner is stopped after it, which would be a window if ->remove were
 * reachable; it is not, because this driver is built in and suppresses bind
 * attributes.  A driver that gains an unbind path has to close that, and
 * v4l2_event_queue() tolerating a NULL devnode is not enough on its own.
 */
static void ispfe_frame_sync(struct ispfe_device *ispfe, u32 sequence)
{
	struct v4l2_event ev = {
		.type = V4L2_EVENT_FRAME_SYNC,
		.u.frame_sync.frame_sequence = sequence,
	};

	v4l2_event_queue(ispfe->sd.devnode, &ev);
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
		ispfe_frame_sync(ispfe, atomic_inc_return(&ispfe->frame_start));

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
 * Where the image starts inside a back-end input slot: past the SBWC header
 * region, which is one record per row of the picture being written.  See
 * ISPFE_BACKEND_HEADER_STRIDE for why this is the stream's height and not the
 * array's.
 */
static u32 ispfe_backend_image_offset(const struct ispfe_device *ispfe)
{
	return ISPFE_BACKEND_HEADER_STRIDE * ispfe->active.height;
}

/*
 * Which grid of a statistics area an auxiliary output feeds, or -1 for an
 * output that is not one of them.
 */
static int ispfe_stats_grid_of_output(unsigned int index)
{
	unsigned int grid;

	for (grid = 0; grid < ISPFE_STATS_GRIDS; grid++)
		if (ispfe_stats_grids[grid].output == index)
			return grid;

	return -1;
}

/*
 * Whether a recipe's destination is one this driver does not run, and so has
 * neither an allocation nor an enable.  Only the LMP image tapouts are gated
 * off; everything else a recipe names is always there.
 */
static bool ispfe_pdma_buffer_gated(const struct ispfe_device *ispfe, u8 buffer)
{
	unsigned int index = ISPFE_BUF_TO_INDEX(buffer);

	return ISPFE_BUF_TO_KIND(buffer) == ISPFE_BUF_KIND_OUTPUT &&
	       index < ARRAY_SIZE(ispfe_pdma_outputs) &&
	       ispfe_pdma_outputs[index].tapout;
}

/*
 * What the recipe's buffer names resolve to for this driver's allocations.  The
 * frame destination is per program rather than per driver, because each program
 * aims one frame at one buffer.  So are the statistics, when a queue has given
 * this slot an area to write them into: @stats is NULL for the dump slot and
 * for the debugfs diagnostic, which keep the shared allocations.
 */
static dma_addr_t ispfe_pdma_buffer(struct ispfe_device *ispfe, u8 buffer,
				    dma_addr_t bayer, dma_addr_t backend,
				    const struct ispfe_stats_area *stats,
				    dma_addr_t lsc)
{
	unsigned int index = ISPFE_BUF_TO_INDEX(buffer);
	int grid;

	switch (ISPFE_BUF_TO_KIND(buffer)) {
	case ISPFE_BUF_KIND_BAYER:
		return bayer;
	case ISPFE_BUF_KIND_INPUT:
		/*
		 * The shading table is the one input a frame can carry its own
		 * copy of; @lsc is this slot's, or zero for the paths that
		 * keep the shared area.
		 */
		if (lsc && (int)index == ispfe->lsc_input)
			return lsc;
		if (index < ispfe->prog->num_inputs)
			return ispfe->blocks_dma +
			       ispfe->prog->inputs[index].area_offset;
		break;
	case ISPFE_BUF_KIND_OUTPUT:
		if (index >= ARRAY_SIZE(ispfe_pdma_outputs))
			break;
		grid = ispfe_stats_grid_of_output(index);
		if (stats && grid >= 0)
			return stats->dma[grid];
		return ispfe->pdma_output[index].dma;
	case ISPFE_BUF_KIND_BACKEND:
		switch (index) {
		case 0:
			return backend;
		case 1:
			return backend + ispfe_backend_image_offset(ispfe);
		case 2:
			return ispfe->tnr_pyramid_dma;
		}
		break;
	}

	return DMA_MAPPING_ERROR;
}

#define ISPFE_LMP_DDS_CONFIG_REG		0x00056154
#define ISPFE_LMP_DPC_CONFIG_REG		0x00054724
#define ISPFE_LMP_WBG_CONFIG_REG		0x00054a38
#define ISPFE_LMP_ALSC_WBG_CONFIG_REG	0x00054a94
#define ISPFE_LMP_RGB_SCALER_CONFIG_REG	0x00056208
#define ISPFE_LMP_SCALER_CONFIG_REG	0x00056228
#define ISPFE_LMP_FORMATTER0_CONFIG_REG	0x0005626c
/*
 * Named on the raw recipes' LMP instance, which is the only one that reaches
 * the three switches below.  The gate word is different -- both recipes carry
 * it -- so it also gets the back-end recipe's address, and is matched through
 * ispfe_lmp_block_is() rather than by either one alone.
 */
#define ISPFE_LMP_BACKEND_FORMATTER_REG	0x000565cc
#define ISPFE_LMP_BACKEND_WDMA_CONFIG_REG 0x000565d8
#define ISPFE_LMP_BATCH_CONFIG_REG	0x000567d8

/*
 * The batch record is sixteen 0xa8-byte entries after a 0x20-byte header of
 * plane offsets, and each entry opens with an enable word, an interrupt word
 * and twenty 64-bit destination addresses.  The driver programs entry 0; the
 * other fifteen are zero, because every captured program has a batch size of
 * one.
 *
 * The enable word is one bit per destination, in the order
 * `LmpBayerPipeline::ConfigureBatchEnabled` writes its per-output boolean
 * flags -- *not* the order of the `bitset` it also maintains, which is the
 * interrupt word and has a different assignment.  A flag is set exactly when
 * its destination has an address, and the entry's address slots follow the
 * same list, with the two output headers taking the gate of the output they
 * belong to and the three histogram ROIs sharing one.
 *
 * That does not divide evenly, so state the remainder rather than leave it to
 * be recounted: eighteen flags, plus a slot each for the two output headers,
 * plus two more for the extra histogram ROIs, is twenty-two -- against twenty
 * slots.  The two flags with no slot of their own are sparse phase detection
 * at bit 0 and the HDR statistics at bit 17, the first and last of the run.
 * Every other bit was checked against both captured words, slot for slot.
 *
 * The corpus is what makes that a reading rather than a guess.  Across the 54
 * captured programs the word takes six values, and every difference between
 * them is a destination whose address slot changed with it: bit 1 tracks the
 * CDAF address, bit 14 the second ML output's and bit 15 the third's.  The two
 * bits the driver already had names for -- the LMP main output it hands the
 * back end, and the TNR pyramid -- land where this list puts them, on 10 and
 * 16.  One bit per destination reproduces all 54 words; shifting the run by
 * one, or giving either header or each histogram ROI its own bit, reproduces
 * none of them.  The eight destinations between the anchored bits fill the
 * eight positions between them exactly, so no unnamed gate can hide inside
 * the run.
 *
 * The run does not end at the TNR pyramid, though: the printer has one more
 * field after it.  The corpus could not have found that, because nothing in
 * it ever sets the bit.
 */
#define ISPFE_LMP_GATE_SPARSE_PD	BIT(0)
#define ISPFE_LMP_GATE_CDAF		BIT(1)
#define ISPFE_LMP_GATE_PRE_LSC_AE	BIT(2)
#define ISPFE_LMP_GATE_AWB_STATS	BIT(3)
#define ISPFE_LMP_GATE_LSC_STATS	BIT(4)
#define ISPFE_LMP_GATE_FLICKER_STATS	BIT(5)
#define ISPFE_LMP_GATE_HISTOGRAM	BIT(6)
#define ISPFE_LMP_GATE_POST_LSC_AE	BIT(7)
#define ISPFE_LMP_GATE_MOTION_METERING	BIT(8)
#define ISPFE_LMP_GATE_BILATERAL_GRID	BIT(9)
#define ISPFE_LMP_BACKEND_OUTPUT_GATE	BIT(10)
#define ISPFE_LMP_GATE_SECONDARY_OUTPUT	BIT(11)
#define ISPFE_LMP_GATE_RGB_OUTPUT	BIT(12)
#define ISPFE_LMP_GATE_ML_OUTPUT0	BIT(13)
#define ISPFE_LMP_GATE_ML_OUTPUT1	BIT(14)
#define ISPFE_LMP_GATE_ML_OUTPUT2	BIT(15)
#define ISPFE_LMP_TNR_OUTPUT_GATE	BIT(16)
/*
 * `BatchPdpOutputEnableValue::Print` has an eighteenth field after the TNR
 * pyramid, `sw_hdr_stats_en`.  Named for completeness only: no captured
 * program sets it, so the corpus cannot place it -- what places it is that it
 * is last in the printer, behind a bit the two recipes anchor.
 */
#define ISPFE_LMP_GATE_HDR_STATS	BIT(17)

/*
 * The two words of batch entry 0 the driver states, and the end of the entry
 * that carries them: it reaches no further into the record than entry 0, and
 * 0xc8 is exactly 0x20 + 0xa8.
 */
#define ISPFE_LMP_OUTPUT_GATES_AT	0x20
#define ISPFE_LMP_BATCH_IRQS_AT		0x24
#define ISPFE_LMP_BATCH_CONFIG_MIN	0xc8

/*
 * What this driver runs: the six statistics taps the front end publishes.  The
 * back-end producer's two are added per stream below.
 */
#define ISPFE_LMP_STATISTICS_GATES	(ISPFE_LMP_GATE_AWB_STATS | \
					 ISPFE_LMP_GATE_LSC_STATS | \
					 ISPFE_LMP_GATE_FLICKER_STATS | \
					 ISPFE_LMP_GATE_HISTOGRAM | \
					 ISPFE_LMP_GATE_POST_LSC_AE | \
					 ISPFE_LMP_GATE_MOTION_METERING)
static_assert(ISPFE_LMP_STATISTICS_GATES == 0x000001f8);

/*
 * And what it does not: the four image destinations the vendor's own programs
 * enable -- a small linear RGB frame and three machine-learning ones -- which
 * nothing here reads.  Named rather than merely omitted, because the same four
 * have to stay absent from this word and from the DRAM write-interface mask
 * further down.  Three of the four are named a third time in the encoder's own
 * output table; the fourth's destination is not an output at all but an
 * address slot inside the batch record, which the encoder blanks whole.
 *
 * There is no relation between the first two to assert -- they are different
 * bit spaces, three interfaces to one destination -- so both are pinned to a
 * literal instead, which makes a change to either a deliberate edit rather
 * than a drift.  The disjointness below is the one mistake that *is*
 * expressible: a tapout bit finding its way into the word the driver builds.
 *
 * Three of the four are what the ultrawide's and the front camera's programs
 * enable; the main camera's enable the second machine-learning output as well,
 * which is why the fourth is here rather than only the three that would make
 * every captured program on two cameras pass.
 */
#define ISPFE_LMP_TAPOUT_GATES		(ISPFE_LMP_GATE_RGB_OUTPUT | \
					 ISPFE_LMP_GATE_ML_OUTPUT0 | \
					 ISPFE_LMP_GATE_ML_OUTPUT1 | \
					 ISPFE_LMP_GATE_ML_OUTPUT2)
static_assert(ISPFE_LMP_TAPOUT_GATES == 0x0000f000);
static_assert(!(ISPFE_LMP_STATISTICS_GATES & ISPFE_LMP_TAPOUT_GATES));

/*
 * The interrupt word's own bits, in the field order
 * `BatchPdpInterruptEnableValue::Print` walks.  Register-update and
 * end-of-frame are the pair the front end's ISR services.  The only other
 * value in the corpus adds bit 4, the CDAF window interrupt, on the one camera
 * whose programs run CDAF at all.
 */
#define ISPFE_LMP_BATCH_IRQ_REG_UPDATE	BIT(0)
#define ISPFE_LMP_BATCH_IRQ_EOF		BIT(1)
#define ISPFE_LMP_DPC_CONFIG_SIZE	0x5c
#define ISPFE_LMP_DPC_GAIN_MAX		GENMASK(9, 0)
#define ISPFE_LMP_WBG_CONFIG_SIZE	0x18
/*
 * The two metering payloads, read rather than guessed.
 * `AwbStatsConfigValue::Print` and `AeStatsConfigValue::Print` fetch every
 * field at an explicit byte offset with an explicit mask, and
 * `LmpAwbStats::Configure` and `LmpAeStats::Configure` deposit them at the
 * same ones -- three vendor artefacts that agree.  The compiled-in payloads
 * agree too: at these offsets the metering ROI comes out *exactly* centred in
 * five of the six of them across two sensor rasters, `pic_width` is each
 * sensor's own width, and the thresholds are byte-identical in all six.
 *
 * Only the thresholds are written here.  The six ROI words are the hardware's
 * description of itself and stay replayed; the driver would have to derive
 * them to state them, and nothing needs that yet.
 */
#define ISPFE_LMP_AWB_STATS_CONFIG_REG	0x000547e8
#define ISPFE_LMP_AE_STATS_CONFIG_REG	0x00054870
/*
 * The back-end recipe and the two raw ones drive **different LMP instances**,
 * and every block in them sits one fixed stride apart: dpc, mm_wbg, lsc_stats,
 * histogram and both stats blocks are all 0x1698 higher in the raw programs
 * than in the back-end one.  Matching only the back end's address would leave
 * the raw recipes replaying their thresholds while the completeness check below
 * refused their programs outright.
 */
#define ISPFE_LMP_INSTANCE_STRIDE	0x1698
/*
 * The batch record on the back-end recipe's instance.  Named here rather than
 * beside the other batch constants because it is built from the stride above,
 * and the assertion below has to be able to evaluate it: it pins the address
 * the back-end recipe actually uses, so a future stride change fails the build
 * instead of quietly matching nothing at STREAMON.
 */
#define ISPFE_LMP_BATCH_CONFIG_REG0	(ISPFE_LMP_BATCH_CONFIG_REG - \
					 ISPFE_LMP_INSTANCE_STRIDE)
static_assert(ISPFE_LMP_BATCH_CONFIG_REG0 == 0x00055140);
/*
 * `lmp/frame_config` on the same instance, and the 53-bit DRAM write-interface
 * mask inside it.  `LmpInput::GetActiveDramInterfaces` derives this word by
 * walking the whole CSR image, so it is the one place a destination's *plane
 * count* reaches the hardware; the batch record's gate word says only whether
 * the destination runs.  A destination whose interfaces stay declared here is
 * one the line-memory processor still keeps buffers for.
 *
 * Three interfaces per image output, in output order after the ten reserved
 * for the Bayer path and the statistics: the linear-RGB tapout at 2..4 (three
 * planes when `sw_rgb_tapout_format` is planar, one otherwise), then output
 * formatters 0, 1 and 2 at 6..8, 9..11 and 12..14.  The captured programs read
 * back exactly that -- the back-end recipe's word is `0x3920003e90dc` and the
 * raw recipes' `0x2120003e90dc`, differing only in bits 43 and 44, which
 * `GetActiveDramInterfaces` declares only when `csr_alignmentformatter_enable`
 * is set, and that byte is `0xf1` in the back-end payload and `0xf0` in the
 * raw ones.
 *
 * The model accounts for every set bit in the low half -- 2, 3, 4 from the
 * tapout being planar, 6 and 7 from formatter 0's two planes, 12 from
 * formatter 2's one -- and for every clear one.  Bit 15 is the white balance
 * statistics: `GetActiveDramInterfaces` sets it from its own condition rather
 * than from a formatter, so it belongs to none of the image destinations, and
 * it stays because this driver reads that tap.
 *
 * The main camera's programs add two more of these destinations' worth: bits
 * 9 and 10 from output formatter 1, whose gate the other two cameras leave
 * clear, and the fourteen at 22..31 and 33..36, which are one literal in
 * `GetActiveDramInterfaces` behind the contrast-detect autofocus statistics'
 * enable.  Neither is a destination this driver runs.
 */
#define ISPFE_LMP_FRAME_CONFIG_REG	0x00055d14
#define ISPFE_LMP_FRAME_CONFIG_REG0	(ISPFE_LMP_FRAME_CONFIG_REG - \
					 ISPFE_LMP_INSTANCE_STRIDE)
static_assert(ISPFE_LMP_FRAME_CONFIG_REG0 == 0x0005467c);
#define ISPFE_LMP_ACTIVE_IFS_AT		0x10
#define ISPFE_LMP_FRAME_CONFIG_MIN	(ISPFE_LMP_ACTIVE_IFS_AT + 8)
/*
 * The two scaler stages that feed those destinations, on the back-end recipe's
 * instance, and the enable bits inside each one's first word.
 *
 * `LmpScaler::Configure` emits a `scaler_lut` for output i only when
 * `sw_input_scale[i]` is set and `sw_yuv_scaler_bypass[i]` is not, and
 * `LmpRgbScaler::Configure` emits `rgb_scaler_lut` only when
 * `sw_input_scale_rgb` is set and `sw_rgb_scaler_bypass` is not.  So clearing
 * an enable is how the vendor's own builder stops shipping a table.
 *
 * The corpus agrees, and it is worth being exact about how far.  Across all 54
 * captured programs the table at 0x00069a0c, 0x00069b0c, 0x00069c0c is present
 * exactly when `sw_input_scale[0]`, `[1]`, `[2]` is set, and the one at
 * 0x00069d0c exactly when `sw_input_scale_rgb` is -- which is what identifies
 * which table belongs to which output, and that is the part this code needs.
 * But only `[1]` and `[2]` are ever *observed* clear: `sw_input_scale[0]` is
 * set in all 54 and `sw_input_scale_rgb` is set in all 54, so for those two
 * only the set-implies-present half is witnessed.  Clearing them is the
 * vendor's mechanism carried past the vendor's evidence, and it holds because
 * it was measured on hardware, not because a capture shows it.
 */
#define ISPFE_LMP_SCALER_CONFIG_REG0	(ISPFE_LMP_SCALER_CONFIG_REG - \
					 ISPFE_LMP_INSTANCE_STRIDE)
#define ISPFE_LMP_RGB_SCALER_CONFIG_REG0 (ISPFE_LMP_RGB_SCALER_CONFIG_REG - \
					 ISPFE_LMP_INSTANCE_STRIDE)
static_assert(ISPFE_LMP_SCALER_CONFIG_REG0 == 0x00054b90);
static_assert(ISPFE_LMP_RGB_SCALER_CONFIG_REG0 == 0x00054b70);
/*
 * The enables are `sw_input_scale[i]` in bits 6:4 of the first word and
 * `sw_input_scale_rgb` in bit 1 of the other's, and the driver writes both
 * words whole rather than masking a captured one, so neither needs a name of
 * its own.  What the corpus holds there, for reading the register: `lmp/scaler`
 * takes only three values across all 54 programs -- 0x10, 0x50 and 0x70, one
 * per enabled output -- and `lmp/rgb_scaler` two, 0x02 and 0x06, differing in
 * `sw_binning_enable`.  See ispfe_pdma_apply_scalers().
 */
#define ISPFE_LMP_SCALER_APPLIED	BIT(0)
#define ISPFE_LMP_RGB_SCALER_APPLIED	BIT(1)
#define ISPFE_LMP_SCALERS_APPLIED	(ISPFE_LMP_SCALER_APPLIED | \
					 ISPFE_LMP_RGB_SCALER_APPLIED)

/*
 * The interfaces of the same four image destinations %ISPFE_LMP_TAPOUT_GATES
 * names -- the RGB tapout and the three output formatters, three interfaces
 * each -- and the fourteen the contrast-detect autofocus statistics keep.
 * Pinned to literals for the reason given there: nothing can assert that these
 * are the same destinations, so the two lists are what make dropping one from
 * either a deliberate edit.
 *
 * The CDAF word is not a run of three: `GetActiveDramInterfaces` ORs this one
 * constant behind that block's enable, bit 32 skipped, so it is transcribed
 * rather than built out of a first interface and a count.
 */
#define ISPFE_LMP_TAPOUT_IFS		(GENMASK_ULL(4, 2) | \
					 GENMASK_ULL(14, 6))
static_assert(ISPFE_LMP_TAPOUT_IFS == 0x0000000000007fdcULL);
#define ISPFE_LMP_CDAF_IFS		0x0000001effc00000ULL
#define ISPFE_LMP_WITHDRAWN_IFS		(ISPFE_LMP_TAPOUT_IFS | \
					 ISPFE_LMP_CDAF_IFS)
/*
 * What is left of the word once those are taken out, which is what every
 * captured program on all three cameras carries: 27 bits belonging to blocks
 * nothing here interprets.  Checking that remainder rather than the whole word
 * is what keeps clearing five destinations' worth a bounded change while still
 * refusing a capture that moved anything else.
 *
 * The alignment formatter's two are the one part of it that varies: the
 * back-end programs set them and the raw ones do not, because
 * `GetActiveDramInterfaces` declares them only when
 * `csr_alignmentformatter_enable` is.
 */
#define ISPFE_LMP_CAPTURED_ACTIVE_IFS	0x00002120003e90dcULL
#define ISPFE_LMP_ALIGNMENT_FORMATTER_IFS	(BIT_ULL(43) | BIT_ULL(44))
#define ISPFE_LMP_FIXED_ACTIVE_IFS	(ISPFE_LMP_CAPTURED_ACTIVE_IFS & \
					 ~ISPFE_LMP_WITHDRAWN_IFS)
/*
 * Both halves are pinned, and not only the one the code compares against: the
 * withdrawn bits of the captured word are cleared before anything looks at
 * them, so a typo in them would fire nothing at all if only the remainder were
 * asserted -- and this constant is named for a value a capture carries.
 */
static_assert(ISPFE_LMP_CAPTURED_ACTIVE_IFS == 0x00002120003e90dcULL);
static_assert(ISPFE_LMP_FIXED_ACTIVE_IFS == 0x00002120003e8000ULL);
static_assert(!(ISPFE_LMP_ALIGNMENT_FORMATTER_IFS &
		ISPFE_LMP_WITHDRAWN_IFS));

/*
 * The LUT target area is one instance apart too, and by a *different* stride:
 * aligning the two recipes' register sets accounts for 55 of the 56 they use
 * with three windows -- FC at 0x2000, the LMP core at the 0x1698 above, and
 * the block the indirect bursts stream into at this one.  Reusing the core's
 * stride here would silently miss the raw programs' shading table.
 */
#define ISPFE_LMP_LUT_INSTANCE_STRIDE	0x3ae8
#define ISPFE_LMP_LSC_LUT_REG		0x0006a02c
/*
 * The linearisation curve LMP reads through that same area, and it is an
 * exact identity: 129 knots of four unsigned Q15 channels in the R, Gr, Gb, B
 * order everything else here uses, each holding `knot * 256`, so 32768 is one.
 * Not 130 knots, which the 1,040 bytes invite -- the last record is zero on
 * all four channels, which no ramp would be, so it is padding.
 *
 * Every one of the vendor's 54 captured programs writes exactly this table on
 * all three cameras and all six sensor readouts, which is what says it is the
 * hardware's pass-through rather than one phone's tuning.
 */
#define ISPFE_LMP_LINEARIZATION_LUT_REG	0x000695fc
#define ISPFE_LMP_LINEARIZATION_KNOTS	129
#define ISPFE_LMP_LINEARIZATION_STEP	256
#define ISPFE_LMP_LINEARIZATION_LUT_BYTES \
	((ISPFE_LMP_LINEARIZATION_KNOTS + 1) * EXYNOS_ISPFE_WB_GAINS * 2)
/* What the generated recipes declare the area to be, so an edit here says so. */
static_assert(ISPFE_LMP_LINEARIZATION_LUT_BYTES == 0x410);
/*
 * The histogram's weight map, which is a *spatial* map over the picture: the
 * block's own register printer names `csr_histogram_weights_cell_width` and
 * `_cell_height` beside it.  Every captured program carries the same 1,024
 * bytes and every one of them is 0x80, which is not tuning either --
 * `lyric::LmpHistogram::Configure` memsets exactly this value when the tuning
 * it is given carries no weight LUT of its own.  So it is the vendor's "no
 * weighting" default, and stating it is a statement rather than a replay.
 *
 * How the map is laid out inside the table -- whether a row is packed to the
 * cell count or to a fixed stride -- is not established, and does not need to
 * be while every weight is equal.  It would need to be before this could
 * become an interface, which is why it is not one.
 */
#define ISPFE_LMP_HISTOGRAM_LUT_REG	0x0006c8b8
#define ISPFE_LMP_HISTOGRAM_WEIGHTS	0x400
#define ISPFE_LMP_HISTOGRAM_WEIGHT_FLAT	0x80
/*
 * How many cells that map is read as, which is a property of the raster and
 * not of the scene.  `lyric::LmpHistogram::Configure` takes each axis of the
 * sensor raster, divides by 128, adds one and clears the low bit, and deposits
 * the two counts in one word of the histogram configuration block -- columns
 * in bits 0..8, rows in bits 16..24.  The bits either side of them are zero in
 * all 54 captured programs, so the word is the two counts and nothing else.
 */
#define ISPFE_LMP_HISTOGRAM_CONFIG_REG	0x0005481c
#define ISPFE_LMP_HISTOGRAM_CONFIG_SIZE	0x54
#define ISPFE_LMP_HISTOGRAM_CELLS	0x30
#define ISPFE_LMP_HISTOGRAM_CELL_PITCH	128
#define ISPFE_LMP_HISTOGRAM_CELL_ROWS_SHIFT	16
#define ISPFE_LMP_HISTOGRAM_CELLS_MAX	511
#define ISPFE_LMP_AWB_STATS_CONFIG_SIZE	0x34
#define ISPFE_LMP_AE_STATS_CONFIG_SIZE	0x28
#define ISPFE_LMP_STATS_SATURATION	0x10
#define ISPFE_LMP_STATS_DARK		0x12
#define ISPFE_LMP_AWB_STATS_LUMA_COEFF	0x14
#define ISPFE_LMP_AWB_STATS_LUMA_LOW	0x1c
#define ISPFE_LMP_AWB_STATS_LUMA_HIGH	0x1e
#define ISPFE_LMP_AWB_STATS_DIFF_CORING	0x20
#define ISPFE_LMP_AWB_STATS_CONFIG	BIT(0)
#define ISPFE_LMP_AE_STATS_CONFIG	BIT(1)
#define ISPFE_LMP_STATS_CONFIGS		(ISPFE_LMP_AWB_STATS_CONFIG | \
					 ISPFE_LMP_AE_STATS_CONFIG)
#define ISPFE_LMP_WBG_CONFIG		BIT(0)
#define ISPFE_LMP_ALSC_WBG_CONFIG	BIT(1)
#define ISPFE_LMP_WBG_CONFIGS		(ISPFE_LMP_WBG_CONFIG | \
					 ISPFE_LMP_ALSC_WBG_CONFIG)

/*
 * Replace the captured gain words with the recipe's semantic profile.  The
 * generated header stays the byte-exact diagnostic oracle, while the built-in
 * program no longer takes these policy values from its opaque payload.
 */
static int ispfe_pdma_apply_wbg(struct ispfe_device *ispfe,
				const struct ispfe_pdma_cmd *cmd,
				u8 *payload, unsigned int *applied)
{
	const struct ispfe_lmp_wbg_profile *wbg = &ispfe->active_lmp_wbg;
	unsigned int config;

	if (!ispfe->prog->lmp_wbg)
		return 0;
	if (cmd->reg == ISPFE_LMP_WBG_CONFIG_REG)
		config = ISPFE_LMP_WBG_CONFIG;
	else if (cmd->reg == ISPFE_LMP_ALSC_WBG_CONFIG_REG)
		config = ISPFE_LMP_ALSC_WBG_CONFIG;
	else
		return 0;
	if (cmd->len != ISPFE_LMP_WBG_CONFIG_SIZE || (*applied & config))
		return -EINVAL;

	put_unaligned_le32(wbg->red, payload + 0x04);
	put_unaligned_le32(wbg->green_red, payload + 0x08);
	put_unaligned_le32(wbg->green_blue, payload + 0x0c);
	put_unaligned_le32(wbg->blue, payload + 0x10);
	*applied |= config;

	return 0;
}

/* Either instance's copy of one LMP block. */
static bool ispfe_lmp_block_is(u32 reg, u32 block)
{
	return reg == block || reg == block + ISPFE_LMP_INSTANCE_STRIDE;
}

/*
 * Replace the captured threshold words in the two metering payloads.  The same
 * shape as the white balance above and for the same reason: the driver already
 * ships these values, and this is only a way for them to come from outside.
 *
 * A sample threshold is signed and reaches the payload as the sixteen bits it
 * is; the luma window is unsigned and wider than the sample, because it gates a
 * weighted sum of four of them.
 *
 * The four sample thresholds are the ones whose effect is observable: moving
 * either grid's saturation or dark threshold moves that grid's counts and sums
 * [HW 2026-08-25].  The luma weights, their window and the coring threshold are
 * written just as faithfully and change nothing the statistics node reports,
 * so they most likely feed the per-colour quantities the AWB region still
 * carries as reserved.
 */
static int ispfe_pdma_apply_stats(struct ispfe_device *ispfe,
				  const struct ispfe_pdma_cmd *cmd,
				  u8 *payload, unsigned int *applied)
{
	const struct ispfe_lmp_metering_profile *m = &ispfe->active_lmp_metering;
	unsigned int i;

	if (ispfe_lmp_block_is(cmd->reg, ISPFE_LMP_AWB_STATS_CONFIG_REG)) {
		if (cmd->len != ISPFE_LMP_AWB_STATS_CONFIG_SIZE ||
		    (*applied & ISPFE_LMP_AWB_STATS_CONFIG))
			return -EINVAL;
		put_unaligned_le16(m->awb_saturation,
				   payload + ISPFE_LMP_STATS_SATURATION);
		put_unaligned_le16(m->awb_dark, payload + ISPFE_LMP_STATS_DARK);
		for (i = 0; i < ARRAY_SIZE(m->awb_luma_coeff); i++)
			put_unaligned_le16(m->awb_luma_coeff[i], payload +
					   ISPFE_LMP_AWB_STATS_LUMA_COEFF + i * 2);
		put_unaligned_le16(m->awb_luma_low,
				   payload + ISPFE_LMP_AWB_STATS_LUMA_LOW);
		put_unaligned_le16(m->awb_luma_high,
				   payload + ISPFE_LMP_AWB_STATS_LUMA_HIGH);
		put_unaligned_le16(m->awb_diff_coring,
				   payload + ISPFE_LMP_AWB_STATS_DIFF_CORING);
		*applied |= ISPFE_LMP_AWB_STATS_CONFIG;
		return 0;
	}

	if (ispfe_lmp_block_is(cmd->reg, ISPFE_LMP_AE_STATS_CONFIG_REG)) {
		if (cmd->len != ISPFE_LMP_AE_STATS_CONFIG_SIZE ||
		    (*applied & ISPFE_LMP_AE_STATS_CONFIG))
			return -EINVAL;
		put_unaligned_le16(m->ae_saturation,
				   payload + ISPFE_LMP_STATS_SATURATION);
		put_unaligned_le16(m->ae_dark, payload + ISPFE_LMP_STATS_DARK);
		*applied |= ISPFE_LMP_AE_STATS_CONFIG;
		return 0;
	}

	return 0;
}

static u32 ispfe_lmp_histogram_cells(u32 raster)
{
	return (raster / ISPFE_LMP_HISTOGRAM_CELL_PITCH + 1) & ~1U;
}

/*
 * The histogram's weight map covers the raster in cells, and how many is
 * derivable from the raster rather than tuning -- which is what the corpus
 * says: this is the one word of the block that tracks the sensor readout, and
 * five distinct values appear across the eighteen captured ones.  Replaying
 * the recipe's own would be right at that recipe's readout and wrong at every
 * other, which is exactly the shape of thing ADR 0009 step 2 asks the driver
 * to derive.
 *
 * A raster whose map would not fit the 1,024 weights the hardware reads is
 * refused rather than metered through weights past the end of the table.  No
 * captured readout comes close: the largest is 32 x 24.
 */
static int ispfe_pdma_apply_histogram(struct ispfe_device *ispfe,
				      const struct ispfe_pdma_cmd *cmd,
				      u8 *payload, bool *applied)
{
	u32 columns, rows;

	if (!ispfe_lmp_block_is(cmd->reg, ISPFE_LMP_HISTOGRAM_CONFIG_REG))
		return 0;
	if (cmd->len != ISPFE_LMP_HISTOGRAM_CONFIG_SIZE || *applied)
		return -EINVAL;

	columns = ispfe_lmp_histogram_cells(ispfe->prog->width);
	rows = ispfe_lmp_histogram_cells(ispfe->prog->height);
	/*
	 * Each count against its own nine-bit field first, and only then the
	 * two against the table.  The order is what makes both tests mean
	 * something: 512 cells one way and two the other is 1,024 weights and
	 * still a count that does not fit, so the product alone would let it
	 * through with bit 9 set into a reserved field -- and a raster large
	 * enough to overflow the product entirely would make that test read
	 * zero and pass.  Bounded per axis first, the product cannot wrap.
	 */
	if (!columns || !rows ||
	    columns > ISPFE_LMP_HISTOGRAM_CELLS_MAX ||
	    rows > ISPFE_LMP_HISTOGRAM_CELLS_MAX ||
	    columns * rows > ISPFE_LMP_HISTOGRAM_WEIGHTS)
		return -EINVAL;

	put_unaligned_le32(columns |
			   rows << ISPFE_LMP_HISTOGRAM_CELL_ROWS_SHIFT,
			   payload + ISPFE_LMP_HISTOGRAM_CELLS);
	*applied = true;

	return 0;
}

/* DPC consumes rounded Q7 R/B gains from the same live AWB state as WBG. */
static int ispfe_pdma_apply_dpc(struct ispfe_device *ispfe,
				const struct ispfe_pdma_cmd *cmd,
				u8 *payload, bool *applied)
{
	const struct ispfe_lmp_wbg_profile *wbg = &ispfe->active_lmp_wbg;
	u64 red, blue;

	if (!ispfe->prog->lmp_wbg || cmd->reg != ISPFE_LMP_DPC_CONFIG_REG)
		return 0;
	if (cmd->len != ISPFE_LMP_DPC_CONFIG_SIZE || *applied)
		return -EINVAL;

	red = DIV_ROUND_CLOSEST_ULL((u64)wbg->red, 1U << 5);
	blue = DIV_ROUND_CLOSEST_ULL((u64)wbg->blue, 1U << 5);
	if (red > ISPFE_LMP_DPC_GAIN_MAX ||
	    blue > ISPFE_LMP_DPC_GAIN_MAX)
		return -ERANGE;

	put_unaligned_le16((u16)red, payload + 0x04);
	put_unaligned_le16((u16)blue, payload + 0x06);
	*applied = true;

	return 0;
}

static int ispfe_pdma_apply_backend_output(struct ispfe_device *ispfe,
					   const struct ispfe_pdma_cmd *cmd,
					   dma_addr_t dma, u8 *payload,
					   u32 payload_at,
					   u32 *image_lo, u32 *image_hi,
					   u32 *header_lo, u32 *header_hi)
{
	if (!ispfe->prog->patch_backend_output ||
	    !ispfe->active_backend_side_output)
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
		/*
		 * The gate and interrupt words are not written here:
		 * ispfe_pdma_apply_batch() owns them for both recipes, so that
		 * two functions cannot disagree about one word.
		 */
		if (cmd->len < ISPFE_LMP_BATCH_CONFIG_MIN)
			return -EINVAL;
		if (upper_32_bits(dma) ||
		    upper_32_bits(dma + ispfe_backend_image_offset(ispfe)) ||
		    upper_32_bits(ispfe->tnr_pyramid_dma))
			return -ERANGE;
		put_unaligned_le32(lower_32_bits(dma +
						 ispfe_backend_image_offset(ispfe)),
				     payload + 0x80);
		put_unaligned_le32(upper_32_bits(dma +
						 ispfe_backend_image_offset(ispfe)),
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
 * State both scaler stages off, which is the whole of what the recipe carries
 * for them.
 *
 * Not the same move as the gate word and the interface mask beside it.  Those
 * stop a destination being *written*; this stops the stage computing anything
 * for it, which is how Lyric itself turns an output off -- and, because the
 * builder emits a scaler's table only behind these bits, it is what lets the
 * recipe stop carrying 768 bytes of scaler and RGB-scaler coefficients.
 *
 * This used to clear the input-scale bits out of a captured word and leave the
 * other sixteen registers of `lmp/scaler` and seven of `lmp/rgb_scaler`
 * replayed.  They were configuring a stage that had already been told it is
 * off, so the recipe stubs both commands to this one register and the driver
 * writes it: the payload arrives as four zero bytes and this is what says so
 * on purpose rather than by omission.  Everything past it keeps its reset
 * value, the same shape the five other stubbed blocks take.
 *
 * The two still have to move together.  A cleared enable whose table is still
 * in the program is a shape no capture holds, and so is the reverse; the recipe
 * drops exactly the tables whose stages are stated off here, and the encoder's
 * own completeness check below is what says both commands were present.
 *
 * Back-end recipe only.  The raw recipes still carry all three tables and both
 * stages in full, and the pairing is per recipe rather than per driver.
 */
static int ispfe_pdma_apply_scalers(struct ispfe_device *ispfe,
				    const struct ispfe_pdma_cmd *cmd,
				    u8 *payload, unsigned int *applied)
{
	u32 flag;

	if (!ispfe->prog->backend_recipe)
		return 0;
	if (cmd->reg == ISPFE_LMP_SCALER_CONFIG_REG0)
		flag = ISPFE_LMP_SCALER_APPLIED;
	else if (cmd->reg == ISPFE_LMP_RGB_SCALER_CONFIG_REG0)
		flag = ISPFE_LMP_RGB_SCALER_APPLIED;
	else
		return 0;
	/*
	 * Exactly the enable register and nothing after it: a recipe that went
	 * back to carrying the rest of either block would be replaying tuning
	 * for a stage this says is off, and the stage's coefficient tables are
	 * dropped on the strength of that.
	 */
	if (cmd->len != sizeof(u32) || (*applied & flag))
		return -EINVAL;

	/*
	 * The encoder has already memset a NULL payload to zero, so this write
	 * changes nothing today; it is here so that the word is *stated* rather
	 * than left to that, and so a staged program cannot arrive with an
	 * enable set.  What the function is really for is the two lines around
	 * it -- the shape assertion above and the completeness bookkeeping
	 * below, which is what pairs these stages with the dropped tables.
	 */
	put_unaligned_le32(0, payload);
	*applied |= flag;

	return 0;
}

/*
 * State which DRAM write interfaces are live, rather than replaying the word a
 * capture came with.
 *
 * A destination with no gate is not written; that does not withdraw the
 * interface the line-memory processor reserved for it.  Lyric never leaves the
 * two disagreeing -- `GetActiveDramInterfaces` rebuilds this word from the same
 * per-block enables that decide the gates -- so a program whose gate word says
 * a destination is off while this word still claims its interfaces is a shape
 * no capture contains.
 *
 * Only bits are cleared, and only ones belonging to the four image
 * destinations %ISPFE_LMP_TAPOUT_GATES names and to the contrast-detect
 * autofocus statistics: the batch record's enable word is built without all
 * five, and this is the other half of saying so.  What is left of the word
 * after they are taken out is checked against the one shape every captured
 * program carries, so a future capture that moved anything else is refused
 * rather than re-derived on an assumption about the other 27 bits.
 */
static int ispfe_pdma_apply_active_ifs(struct ispfe_device *ispfe,
				       const struct ispfe_pdma_cmd *cmd,
				       u8 *payload, bool *applied)
{
	u64 ifs;

	if (!ispfe_lmp_block_is(cmd->reg, ISPFE_LMP_FRAME_CONFIG_REG0))
		return 0;
	if (cmd->len < ISPFE_LMP_FRAME_CONFIG_MIN || *applied)
		return -EINVAL;

	ifs = get_unaligned_le64(payload + ISPFE_LMP_ACTIVE_IFS_AT);
	ifs &= ~ISPFE_LMP_WITHDRAWN_IFS;
	if (ifs != ISPFE_LMP_FIXED_ACTIVE_IFS &&
	    ifs != (ISPFE_LMP_FIXED_ACTIVE_IFS |
		    ISPFE_LMP_ALIGNMENT_FORMATTER_IFS))
		return -EINVAL;

	put_unaligned_le64(ifs, payload + ISPFE_LMP_ACTIVE_IFS_AT);
	*applied = true;

	return 0;
}

/*
 * Build the batch record, rather than editing a captured one.
 *
 * It is a structure and not a tuning: a 0x20-byte header of plane offsets and
 * sixteen 0xa8-byte entries, each an enable word, an interrupt word and twenty
 * 64-bit destination addresses.  Every captured program has a batch size of
 * one, so entry 0 is the whole of it and the other fifteen are zero.  The
 * addresses are relocations, and the plane offsets belong to the linear RGB
 * frame and the three machine-learning outputs -- destinations this driver
 * does not run, whose base addresses the encoder nulls in the same pass, so an
 * offset from one of them describes nothing and is left at zero too.
 *
 * That leaves two words for this function, and the recipe therefore carries no
 * bytes at all for the record: the encoder blanks the whole 0xaa0, and what
 * reaches the hardware is these two words and the relocated addresses.
 *
 * The enable word is one bit per destination.  Three of the ones the vendor's
 * programs set are the image tapouts above -- four on the main camera, which
 * enables the second machine-learning output as well; leaving them on would
 * cost 5.1 MiB of coherent memory and about a megabyte of DMA writes per frame
 * for a picture with no consumer, so they are simply not in the word.  The
 * main camera's contrast-detect autofocus statistics go the same way and for
 * the same reason, which is why its generated header lists two captured
 * addresses the encoder nulls where the ultrawide's lists none.
 *
 * Stating the word rather than masking a replayed one moves a check earlier
 * rather than losing it.  The recipe generator lists every non-zero word of
 * the captured record that no relocation covers, so a capture whose enable or
 * interrupt word differs from these changes the generated header -- and the
 * rule that regenerating the recipes must leave the tree clean is what catches
 * it, at generation time instead of at STREAMON.
 */
static int ispfe_pdma_apply_batch(struct ispfe_device *ispfe,
				  const struct ispfe_pdma_cmd *cmd,
				  u8 *payload, bool *applied)
{
	u32 gates = ISPFE_LMP_STATISTICS_GATES;

	if (!ispfe_lmp_block_is(cmd->reg, ISPFE_LMP_BATCH_CONFIG_REG0))
		return 0;
	if (cmd->len < ISPFE_LMP_BATCH_CONFIG_MIN || *applied)
		return -EINVAL;

	/*
	 * The producer's two follow the one flag that already decides whether
	 * this stream feeds it -- unconditional on the back-end recipe, which
	 * exists for it, and a switch on the full-readout raw one, which can
	 * carry it as a side output.  Asking the same question a second way
	 * here would let the gate and the address disagree: a gate set for a
	 * stream this flag calls off would leave the encoder writing no address
	 * beside it.
	 */
	if (ispfe->active_backend_side_output)
		gates |= ISPFE_LMP_BACKEND_OUTPUT_GATE |
			 ISPFE_LMP_TNR_OUTPUT_GATE;

	put_unaligned_le32(gates, payload + ISPFE_LMP_OUTPUT_GATES_AT);
	put_unaligned_le32(ISPFE_LMP_BATCH_IRQ_REG_UPDATE |
			   ISPFE_LMP_BATCH_IRQ_EOF,
			   payload + ISPFE_LMP_BATCH_IRQS_AT);
	*applied = true;

	return 0;
}

/*
 * The shading grid the hardware reads is tiled: one 32-byte record holds four
 * rows of one column's four gains, so the grid is stored four rows at a time
 * and the twenty-fifth row is alone in a record whose other three rows are
 * padding.  Reading it as a flat array correlates with nothing, which is what
 * hid it during bring-up.
 *
 * A parameters block carries the grid; this is where it becomes the layout.
 * The check that this is the layout is offline and exact: de-tiling the
 * recipe's own captured table and re-tiling it reproduces its 7392 bytes.
 */
static unsigned int ispfe_lsc_word(unsigned int row, unsigned int column,
				   unsigned int channel)
{
	return ((row / ISPFE_LSC_TILE_ROWS) * EXYNOS_ISPFE_LSC_COLUMNS +
		column) * ISPFE_LSC_TILE_WORDS +
	       (row % ISPFE_LSC_TILE_ROWS) * EXYNOS_ISPFE_WB_GAINS + channel;
}

/*
 * Unity everywhere, in the tiled layout: a gain of one at every point on every
 * channel, and zero in the padding rows the last record carries.  Built rather
 * than memset, because %EXYNOS_ISPFE_LSC_GAIN_ONE is not a repeating byte and
 * because the padding must stay zero -- the same index map every other reader
 * of this table uses, so it cannot drift from them.
 */
static void ispfe_lsc_unity(u8 *tiled)
{
	unsigned int row, column, channel;

	memset(tiled, 0, ISPFE_LSC_LUT_BYTES);
	for (row = 0; row < EXYNOS_ISPFE_LSC_ROWS; row++)
		for (column = 0; column < EXYNOS_ISPFE_LSC_COLUMNS; column++)
			for (channel = 0; channel < EXYNOS_ISPFE_WB_GAINS;
			     channel++)
				put_unaligned_le16(EXYNOS_ISPFE_LSC_GAIN_ONE,
						   tiled + 2 *
						   ispfe_lsc_word(row, column,
								  channel));
}

static void ispfe_lsc_tile(u8 *tiled, const u16 *grid)
{
	unsigned int row, column, channel, point = 0;

	memset(tiled, 0, ISPFE_LSC_LUT_BYTES);
	for (row = 0; row < EXYNOS_ISPFE_LSC_ROWS; row++)
		for (column = 0; column < EXYNOS_ISPFE_LSC_COLUMNS; column++)
			for (channel = 0; channel < EXYNOS_ISPFE_WB_GAINS;
			     channel++)
				put_unaligned_le16(grid[point++],
						   tiled + 2 *
						   ispfe_lsc_word(row, column,
								  channel));
}

/*
 * Which of the running program's indirect inputs a lookup table is, or a
 * negative error if it has none.  Found by the register the burst targets
 * rather than by an index, because the two LMP instances put the LUT area in
 * different places and an index would quietly select a different table on the
 * raw recipes.
 */
static int ispfe_lut_input(const struct ispfe_pdma_program *prog, u32 reg,
			   u32 bytes)
{
	unsigned int i;

	for (i = 0; i < prog->num_cmds; i++) {
		const struct ispfe_pdma_cmd *cmd = &prog->cmds[i];

		if (cmd->op != ISPFE_PDMA_INDIRECT_BURST ||
		    (cmd->reg != reg &&
		     cmd->reg != reg + ISPFE_LMP_LUT_INSTANCE_STRIDE))
			continue;
		if (cmd->len != bytes ||
		    ISPFE_BUF_TO_KIND(cmd->buffer) != ISPFE_BUF_KIND_INPUT ||
		    ISPFE_BUF_TO_INDEX(cmd->buffer) >= prog->num_inputs)
			return -EINVAL;
		return ISPFE_BUF_TO_INDEX(cmd->buffer);
	}

	return -ENOENT;
}

static int ispfe_lsc_input(const struct ispfe_pdma_program *prog)
{
	return ispfe_lut_input(prog, ISPFE_LMP_LSC_LUT_REG,
			       ISPFE_LSC_LUT_BYTES);
}

/* One grid sample read back out of the tiled table, for the diagnostic. */
static u16 ispfe_lsc_sample(const u8 *tiled, unsigned int row,
			    unsigned int column, unsigned int channel)
{
	return get_unaligned_le16(tiled +
				  2 * ispfe_lsc_word(row, column, channel));
}

/*
 * The table a stream starts on, and what a disabled block asks for.
 *
 * Unity: shading correction switched off.  A recipe used to carry the vendor's
 * own 7,392-byte grid here and the driver handed that back, which made the
 * kernel ship a calibration of one lens at one readout -- ADR 0009 says no
 * captured tuning ships, and that was the largest table left in the tree.
 *
 * The honest cost is that a consumer sending no grid gets visibly dark
 * corners: this lens falls to about a sixth of its centre response there.
 * That is the same trade the tone curve's identity default makes, and for the
 * same reason -- a picture that says "nothing has calibrated this camera" is
 * better than one silently carrying some other unit's numbers.  libcamera's
 * IPA sends a real grid from the phone's own factory calibration.
 *
 * A recipe *may* still carry a table, and one that does is still honoured:
 * `.data` is NULL only for the inputs the generator was told to neutralise.
 */
static const u8 *ispfe_lsc_default(const struct ispfe_device *ispfe,
				   const struct ispfe_pdma_program *prog)
{
	int input = ispfe_lsc_input(prog);

	if (input < 0 || prog->inputs[input].size < ISPFE_LSC_LUT_BYTES)
		return NULL;
	if (!prog->inputs[input].data)
		return ispfe->lsc_unity;
	return prog->inputs[input].data;
}

/*
 * Where one of the tables the driver states lives in the shared block area,
 * checked against the area it has to fit in rather than trusted.
 *
 * The input it resolves to must be one the recipe declares stated -- carrying
 * no bytes of its own.  A generated recipe where the register and the empty
 * entry disagree would otherwise leave one table zeroed and overwrite another,
 * which is a picture rather than an error.
 */
static int ispfe_lut_area(struct ispfe_device *ispfe, u32 reg, u32 bytes,
			  unsigned int *index, u8 **area)
{
	const struct ispfe_pdma_program *prog = ispfe->prog;
	int input = ispfe_lut_input(prog, reg, bytes);

	if (input < 0)
		return input;
	if (prog->inputs[input].size != bytes || prog->inputs[input].data ||
	    size_add(prog->inputs[input].area_offset, bytes) >
	    prog->blocks_bytes)
		return -EINVAL;
	*index = input;
	*area = (u8 *)ispfe->blocks + prog->inputs[input].area_offset;
	return 0;
}

/*
 * The lookup tables the driver states rather than replays.  A captured table
 * that is an identity or a uniform default is not tuning at all, and
 * generating it says so where a thousand bytes of `const u8` cannot.
 *
 * The linearisation curve is that identity, on all four channels.  Its last
 * record is left at the zero the surrounding memset already wrote, because it
 * is padding rather than a knot.  The histogram's weight map is the uniform
 * default, so nothing in the picture is weighted above anything else.
 *
 * A recipe that carries neither table is refused rather than left with a
 * zeroed one: every captured program writes both, so a recipe without them is
 * a recipe this code has not seen -- and a zeroed weight map would meter
 * nothing at all.
 */
static int ispfe_pdma_state_luts(struct ispfe_device *ispfe)
{
	const struct ispfe_pdma_program *prog = ispfe->prog;
	unsigned int knot, channel, index, i;
	unsigned long stated = 0;
	u8 *area;
	int lsc;
	int ret;

	if (prog->num_inputs > BITS_PER_LONG)
		return -EINVAL;

	ret = ispfe_lut_area(ispfe, ISPFE_LMP_LINEARIZATION_LUT_REG,
			     ISPFE_LMP_LINEARIZATION_LUT_BYTES, &index, &area);
	if (ret) {
		dev_err(ispfe->dev,
			"PDMA recipe carries no linearisation curve (%d)\n", ret);
		return -EINVAL;
	}
	__set_bit(index, &stated);
	for (knot = 0; knot < ISPFE_LMP_LINEARIZATION_KNOTS; knot++)
		for (channel = 0; channel < EXYNOS_ISPFE_WB_GAINS; channel++)
			put_unaligned_le16(knot * ISPFE_LMP_LINEARIZATION_STEP,
					   area + 2 * (knot *
						       EXYNOS_ISPFE_WB_GAINS +
						       channel));

	ret = ispfe_lut_area(ispfe, ISPFE_LMP_HISTOGRAM_LUT_REG,
			     ISPFE_LMP_HISTOGRAM_WEIGHTS, &index, &area);
	if (ret) {
		dev_err(ispfe->dev,
			"PDMA recipe carries no histogram weight map (%d)\n",
			ret);
		return -EINVAL;
	}
	__set_bit(index, &stated);
	memset(area, ISPFE_LMP_HISTOGRAM_WEIGHT_FLAT,
	       ISPFE_LMP_HISTOGRAM_WEIGHTS);

	/*
	 * The shading table is filled by every encode instead, from
	 * @ispfe->shading -- the driver's own unity table until a parameters
	 * block replaces it -- so a recipe carrying no bytes for it is not a
	 * recipe with a hole.  Nothing is written here: the encode does it per
	 * slot, and doing it twice would only add a copy that goes stale.
	 *
	 * A recipe that *does* carry a captured table still works; its bytes
	 * are copied like any other input's and then overwritten by that same
	 * encode.
	 */
	lsc = ispfe_lsc_input(prog);
	if (lsc >= 0 && !prog->inputs[lsc].data)
		__set_bit(lsc, &stated);

	/*
	 * And every empty input is one of those, so a recipe cannot declare a
	 * table stated and get a zeroed area because nothing here fills it.
	 */
	for (i = 0; i < prog->num_inputs; i++)
		if (!prog->inputs[i].data && !test_bit(i, &stated)) {
			dev_err(ispfe->dev,
				"PDMA recipe input %u carries no bytes and nothing states it\n",
				i);
			return -EINVAL;
		}

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
/*
 * The largest ML output 0 a staged program may declare, from the vendor's own
 * allocation class for that destination.  The driver does not run the LMP
 * image tapouts, so nothing is allocated against this and no write can reach
 * it; the bound stays because a staged program is a hand-written geometry and
 * a self-consistent one is the only kind worth accepting.
 */
#define ISPFE_LMP_ML0_MAX_SIZE		0x00480000

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

/*
 * How far into each of those payloads the geometry check below reads, so the
 * collector can refuse a command too short to hold it.
 *
 * Every recipe in the tree satisfies these and the check cannot fire today.
 * It is here because the recipe's command lengths are a *shell script's*
 * declaration -- `--stub` shortens a command to four bytes, and it has already
 * been applied to `lmp/scaler` and `lmp/rgb_scaler` on the back-end recipe.
 * Doing the same to a raw recipe would leave this reading past the payload and
 * into the next command's bytes: still inside the staged buffer, so garbage
 * rather than an oops, and a geometry that validated against nothing.  A
 * length is the cheapest way to make that a refusal rather than an invariant
 * held somewhere else.
 */
#define ISPFE_STAGED_DDS_MIN		0x0c
#define ISPFE_STAGED_RGB_SCALER_MIN	0x1c
#define ISPFE_STAGED_SCALER_MIN		0x44
#define ISPFE_STAGED_FORMATTER_MIN	0x28
#define ISPFE_STAGED_BATCH_MIN		0x10

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

	/* Output 2 is scaled from the same input, so its factors follow it too. */
	dest_width = output2 & U16_MAX;
	dest_height = output2 >> 16;
	if (!dest_width || !dest_height ||
	    !ispfe_pdma_scale_factor_matches(scaler, 0x3c, input_width,
					     dest_width) ||
	    !ispfe_pdma_scale_factor_matches(scaler, 0x40, input_height,
					     dest_height))
		return -EINVAL;

	/* The linear-RGB branch has one automatic 2x pre-bin stage. */
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
			if (cmd->len % 8 || !cmd->payload ||
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
			/*
			 * A command with no payload is @len bytes of zero, so
			 * that is what a staged program has to carry there:
			 * the recipe holds no captured bytes to compare
			 * against, and the encoder's apply functions write
			 * whatever the driver states on top afterwards.
			 */
			for (j = 0; j < cmd->len; j++)
				if (!ispfe_pdma_reloc_byte(prog, i, j) &&
				    !ispfe_pdma_geometry_byte(cmd, j) &&
				    payload[j] != (cmd->payload ?
						   cmd->payload[j] : 0))
					return -EINVAL;
			switch (cmd->reg) {
			case ISPFE_LMP_DDS_CONFIG_REG:
				if (cmd->len < ISPFE_STAGED_DDS_MIN)
					return -EINVAL;
				dds = payload;
				/*
				 * Both vendor recipes deliver 1052x780 here.  The
				 * full-mode /2 experiment instead delivered 2104x1560
				 * and left LMP unable to quiesce [HW 2026-08-20].
				 * DDS is therefore part of the captured hardware
				 * envelope, not editable output geometry.
				 */
				if (!cmd->payload ||
				    get_unaligned_le32(payload) !=
				    get_unaligned_le32(cmd->payload))
					return -EINVAL;
				break;
			case ISPFE_LMP_RGB_SCALER_CONFIG_REG:
				if (cmd->len < ISPFE_STAGED_RGB_SCALER_MIN)
					return -EINVAL;
				rgb = payload;
				break;
			case ISPFE_LMP_SCALER_CONFIG_REG:
				if (cmd->len < ISPFE_STAGED_SCALER_MIN)
					return -EINVAL;
				scaler = payload;
				break;
			case ISPFE_LMP_FORMATTER0_CONFIG_REG:
				if (cmd->len < ISPFE_STAGED_FORMATTER_MIN)
					return -EINVAL;
				formatter = payload;
				break;
			case ISPFE_LMP_BATCH_CONFIG_REG:
				if (cmd->len < ISPFE_STAGED_BATCH_MIN)
					return -EINVAL;
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
			     dma_addr_t bayer, dma_addr_t backend,
			     const struct ispfe_stats_area *stats,
			     const u8 *shading)
{
	const struct ispfe_pdma_program *prog = ispfe->prog;
	const struct ispfe_pdma_reloc *reloc = prog->relocs;
	const struct ispfe_pdma_reloc *last = prog->relocs + prog->num_relocs;
	dma_addr_t lsc = 0;
	u32 bayer_lo = 0, bayer_hi = 0;
	u32 awb_lo = 0, awb_hi = 0;
	u32 backend_image_lo = 0, backend_image_hi = 0;
	u32 backend_header_lo = 0, backend_header_hi = 0;
	unsigned int lmp_wbg_configs = 0;
	unsigned int lmp_stats_configs = 0;
	bool lmp_dpc_applied = false;
	bool lmp_histogram_applied = false;
	bool lmp_batch_applied = false;
	bool lmp_active_ifs_applied = false;
	unsigned int lmp_scalers_applied = 0;
	unsigned int i;
	u8 *program;
	size_t at = 0;
	int ret;

	if (slot >= PDMA_SLOTS)
		return -EINVAL;

	program = ispfe->programs + slot * PDMA_SLOT_STRIDE;

	/*
	 * Give this slot its own copy of the shading table before anything
	 * points at it, so that a table sent for this frame cannot reach a
	 * frame whose program was encoded earlier.  A staged program keeps the
	 * shared block area, because what it is for is running exactly the
	 * bytes that were staged.
	 */
	if (shading && ispfe->lsc_input >= 0 &&
	    !ispfe->active_pdma_program_override) {
		lsc = ispfe->lsc_dma + slot * ISPFE_LSC_SLOT_STRIDE;
		memcpy((u8 *)ispfe->lsc + slot * ISPFE_LSC_SLOT_STRIDE,
		       shading, ISPFE_LSC_LUT_BYTES);
	}

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
						backend, stats, lsc);
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
			if (cmd->payload)
				memcpy(program + at, cmd->payload, cmd->len);
			else
				memset(program + at, 0, cmd->len);
			ret = ispfe_pdma_apply_wbg(ispfe, cmd, program + at,
						   &lmp_wbg_configs);
			if (ret)
				return ret;
			ret = ispfe_pdma_apply_dpc(ispfe, cmd, program + at,
						   &lmp_dpc_applied);
			if (ret)
				return ret;
			ret = ispfe_pdma_apply_stats(ispfe, cmd, program + at,
						     &lmp_stats_configs);
			if (ret)
				return ret;
			ret = ispfe_pdma_apply_histogram(ispfe, cmd,
						 program + at,
						 &lmp_histogram_applied);
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
		/*
		 * Applied to a staged program too, because it is what decides
		 * whether a destination the driver has not allocated is
		 * enabled.  A staged program cannot have moved either word:
		 * ispfe_pdma_staged_validate() already holds every byte of it
		 * that is neither a relocation nor scaler geometry.
		 */
		ret = ispfe_pdma_apply_batch(ispfe, cmd, program + at,
					     &lmp_batch_applied);
		if (ret)
			return ret;
		/*
		 * Applied to a staged program for the same reason as the gates:
		 * it is the other half of saying a destination does not run.
		 */
		ret = ispfe_pdma_apply_active_ifs(ispfe, cmd, program + at,
						  &lmp_active_ifs_applied);
		if (ret)
			return ret;
		ret = ispfe_pdma_apply_scalers(ispfe, cmd, program + at,
					       &lmp_scalers_applied);
		if (ret)
			return ret;

		for (; reloc < last && reloc->cmd == i; reloc++) {
			if (reloc->lo + 4 > cmd->len ||
			    (reloc->hi != ISPFE_PDMA_RELOC_NO_HIGH &&
			     reloc->hi + 4 > cmd->len))
				return -EINVAL;
			/*
			 * A destination this stream is not running gets the
			 * null address that goes with having no gate, which is
			 * how every captured program leaves the five it does
			 * not write.
			 */
			if (ispfe_pdma_buffer_gated(ispfe, reloc->buffer)) {
				dma = 0;
			} else {
				dma = ispfe_pdma_buffer(ispfe, reloc->buffer,
							bayer, backend, stats,
							lsc);
				if (dma == DMA_MAPPING_ERROR)
					return -EINVAL;
			}
			put_unaligned_le32(lower_32_bits(dma),
					   program + at + reloc->lo);
			if (reloc->hi != ISPFE_PDMA_RELOC_NO_HIGH)
				put_unaligned_le32(upper_32_bits(dma),
						   program + at + reloc->hi);
			if (reloc->buffer == ISPFE_BUF_BAYER) {
				bayer_lo = at + reloc->lo;
				bayer_hi = at + reloc->hi;
			} else if (reloc->buffer ==
				   ISPFE_BUF_OUTPUT(ISPFE_PDMA_OUTPUT_AWB)) {
				awb_lo = at + reloc->lo;
				awb_hi = at + reloc->hi;
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
	if ((!ispfe->active_pdma_program_override && prog->lmp_wbg &&
	     (lmp_wbg_configs != ISPFE_LMP_WBG_CONFIGS ||
	      !lmp_dpc_applied)) ||
	    reloc != last || !awb_lo || !awb_hi ||
	    (prog->raw_output && !bayer_lo)) {
		dev_err(ispfe->dev, "PDMA recipe relocations do not match it\n");
		return -EINVAL;
	}
	/*
	 * Said separately because it asserts something about the *recipe* rather
	 * than about this encode: every captured program carries both metering
	 * blocks, on one LMP instance or the other.  A future one that does not
	 * should say which block is missing rather than borrow the relocation
	 * message above.
	 */
	if (!ispfe->active_pdma_program_override &&
	    lmp_stats_configs != ISPFE_LMP_STATS_CONFIGS) {
		dev_err(ispfe->dev,
			"PDMA recipe is missing the %s metering block\n",
			(lmp_stats_configs & ISPFE_LMP_AWB_STATS_CONFIG) ?
			"exposure" : "white balance");
		return -EINVAL;
	}
	/*
	 * And the same for the histogram, whose weight map the driver states:
	 * a recipe without the block would leave the map's shape at whatever
	 * the hardware came up in.
	 */
	if (!ispfe->active_pdma_program_override && !lmp_histogram_applied) {
		dev_err(ispfe->dev, "PDMA recipe is missing the histogram block\n");
		return -EINVAL;
	}
	/*
	 * And for the batch record, which the driver builds entirely: a recipe
	 * without it would leave every destination gated off and no frame
	 * interrupt enabled, since the words that say otherwise are written
	 * here and nowhere else.
	 */
	if (!lmp_batch_applied) {
		dev_err(ispfe->dev, "PDMA recipe is missing the batch record\n");
		return -EINVAL;
	}
	/*
	 * And the frame configuration, which carries the interface mask that
	 * has to agree with those gates.
	 */
	if (!lmp_active_ifs_applied) {
		dev_err(ispfe->dev,
			"PDMA recipe is missing the frame configuration\n");
		return -EINVAL;
	}
	/*
	 * And both scaler stages, whose enables have to be cleared for the
	 * tables this recipe no longer carries.  A recipe that had lost one of
	 * these commands would otherwise ship a scaler still asking for a
	 * coefficient table that is not in the program.
	 */
	if (ispfe->prog->backend_recipe &&
	    lmp_scalers_applied != ISPFE_LMP_SCALERS_APPLIED) {
		dev_err(ispfe->dev, "PDMA recipe is missing the %s scaler\n",
			(lmp_scalers_applied & ISPFE_LMP_SCALER_APPLIED) ?
			"RGB" : "image");
		return -EINVAL;
	}
	if (ispfe->active_backend_side_output &&
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
	ispfe->awb_lo = awb_lo;
	ispfe->awb_hi = awb_hi;
	ispfe->backend_image_lo = backend_image_lo;
	ispfe->backend_image_hi = backend_image_hi;
	ispfe->backend_header_lo = backend_header_lo;
	ispfe->backend_header_hi = backend_header_hi;

	return 0;
}

static void ispfe_params_consume(struct ispfe_device *ispfe,
				 struct ispfe_lmp_wbg_profile *wbg,
				 const u8 **shading);

/*
 * Hand every finished frame to the back end.  This has no deadline of its own
 * -- the frame is already written -- which is why it runs *after* the arm
 * below rather than before it.  Doing it first cost a frame whenever it landed
 * badly: producer_complete() hands the slot on, the back end queues its own
 * work item for it, and that item takes the same mutex the next
 * producer_acquire() needs and holds it across an encode.  Measured, the arm
 * then waited 2.1 ms behind a lock it had itself caused to be taken, against a
 * window of 1.3 ms.
 */
static void ispfe_backend_queue_drain(struct ispfe_device *ispfe)
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
}

/*
 * Encode a program into every slot the back end will take one for.  This is
 * the deadline: a credit at the next frame start drops the frame unless one of
 * these is on the ready list by then.
 *
 * It runs before the drain and does not need it to have run.  Two resources
 * say so, and they are not the same argument.
 *
 * The local two are counted: a buffer is not idle for exactly as long as its
 * back-end slot is a producer's, and PDMA_BUF_SLOTS is four against the back
 * end's three input slots (BECORE_INPUT_SLOT_COUNT, its own private constant,
 * so nothing here can assert it), leaving one program slot and one buffer
 * always spare.
 *
 * The back-end slot is the one that matters and the counting says nothing
 * about it.  producer_complete() moves a slot to *ready*, not free; only the
 * back end's own run frees one.  So draining first never handed this an idle
 * slot either -- what it handed it was the wait for that run, taken inside the
 * 1.3 ms window.  Arming first spends the same dependency against a whole
 * frame period instead: the run is ~17 ms of a 34 ms frame, so its result is
 * ~31 ms old by the time this asks.  If it ever is not, producer_acquire()
 * returns -EBUSY, this encodes nothing, and the credit that finds an empty
 * ready list schedules the work again -- one frame, not a stall.
 */
static void ispfe_backend_queue_arm(struct ispfe_device *ispfe)
{
	for (;;) {
		const struct ispfe_stats_area *stats = NULL;
		struct ispfe_backend_buffer *buf = NULL;
		struct ispfe_lmp_wbg_profile wbg;
		const u8 *shading = NULL;
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
			stats = ispfe_stats_take(ispfe, program_slot);
			/*
			 * The gains this program is about to be encoded with,
			 * taken under the same lock that may have just moved
			 * them, and handed to the back end on the buffer the
			 * program writes.  Its Bayer denoiser scales its noise
			 * factors by them, and what it wants is the balance
			 * *this* frame was taken through.
			 */
			ispfe_params_consume(ispfe, &wbg, &shading);
			buf->ticket.gains = (struct exynos_becore_input_gains) {
				.red = wbg.red,
				.green_red = wbg.green_red,
				.green_blue = wbg.green_blue,
				.blue = wbg.blue,
			};
			ret = ispfe_pdma_encode(ispfe, program_slot,
						ispfe->frame_dma,
						buf->ticket.dma, stats,
						shading);
			if (ret)
				ispfe_stats_untake(ispfe, program_slot);
		}
		if (ret && acquired)
			exynos_becore_input_producer_abort(ispfe->backend_input,
							   &buf->ticket);

		spin_lock_irq(&ispfe->slock);
		if (ret || !ispfe->backend_queue_active) {
			ispfe_stats_untake_locked(ispfe, program_slot);
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

static void ispfe_backend_queue_fill(struct ispfe_device *ispfe)
{
	ispfe_backend_queue_arm(ispfe);
	ispfe_backend_queue_drain(ispfe);
}

/*
 * Both queue-fill work items -- this one and ispfe_fill_work() -- run on
 * system_highpri_wq, and the reason is a deadline rather than importance.  A
 * frame retires at LMP EOF and its replacement has to be encoded before the
 * next frame start credits a program: about 1.3 ms later, measured.  The fill
 * takes 30 us.  So what it cannot afford is being *scheduled behind* something
 * bulky, and this driver's own statistics copy is exactly that: milliseconds
 * of memcpy with no deadline of its own.  Putting that copy on an unbound pool
 * stopped it holding the per-CPU worker, but an unbound worker woken by the
 * same interrupt is placed on the same idle CPU and then keeps it for a full
 * slice.  A highpri pool's workers are nice -20, so the fill preempts the copy
 * rather than waiting a slice out behind it.
 *
 * Which is safe only because neither fill hogs what it preempts.  This one
 * sleeps on the back end's mutex, which releases the pool; the raw one does
 * not sleep at all but is bounded at four encodes, around 120 us.  The one
 * exception is a producer_acquire() that finds a slot the offline diagnostic
 * left dirty and syncs 27 MB of it, 2.3 ms -- reachable only if a debugfs run
 * preceded the capture in the same boot.
 *
 * Neither is why the work exists rather than the interrupt.  For this one that
 * is the back end's mutex, which producer_acquire() and producer_complete()
 * take; for the raw one it is the size of the encode, said where it lives.
 */
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
	    upper_32_bits(ispfe->lsc_dma + ISPFE_LSC_AREA_BYTES - 1) ||
	    upper_32_bits(ispfe->programs_dma + PDMA_PROGRAMS_SIZE - 1) ||
	    upper_32_bits(ispfe->frame_dma + ispfe->frame_size - 1) ||
	    upper_32_bits(ispfe->spare_frame_dma + ispfe->frame_size - 1) ||
	    upper_32_bits(ispfe->awb_spare_dma +
			  ispfe_pdma_outputs[ISPFE_PDMA_OUTPUT_AWB].size - 1))
		return -ERANGE;

	for (i = 0; i < ARRAY_SIZE(ispfe_pdma_outputs); i++)
		if (ispfe->pdma_output[i].cpu &&
		    upper_32_bits(ispfe->pdma_output[i].dma +
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
			if (!input->data)
				continue;
			memcpy((u8 *)ispfe->blocks + input->area_offset,
			       input->data, input->size);
		}
		ret = ispfe_pdma_state_luts(ispfe);
		if (ret)
			return ret;
	}

	/*
	 * No statistics area: this is where every slot starts, and it is the
	 * whole of what the debugfs diagnostic and the dump slot ever use, so
	 * both keep writing into the shared allocations the drained snapshot
	 * reads.  A queue chooses an area when it encodes a slot for a buffer.
	 *
	 * The shading table is the stream's default here for the same reason a
	 * slot's own copy exists at all: what a slot is encoded with is what
	 * the frame it carries will be corrected by, and no frame has asked for
	 * anything else yet.
	 */
	memset(ispfe->programs, 0, PDMA_PROGRAMS_SIZE);
	for (i = 0; i < PDMA_SLOTS; i++) {
		ret = ispfe_pdma_encode(ispfe, i,
					i == PDMA_DUMP_SLOT ?
					ispfe->spare_frame_dma :
					ispfe->frame_dma,
					backend, NULL, ispfe->shading);
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
	writel_relaxed(ispfe->active_fc_axi_max_ost,
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
	put_unaligned_le32(lower_32_bits(dma + ispfe_backend_image_offset(ispfe)),
			   program + ispfe->backend_image_lo);
	put_unaligned_le32(upper_32_bits(dma + ispfe_backend_image_offset(ispfe)),
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
		if (WARN_ON_ONCE(!ispfe->awb_lo || !ispfe->awb_hi)) {
			cmpxchg(&ispfe->snapshot_state,
				ISPFE_SNAPSHOT_REDIRECTING, ISPFE_SNAPSHOT_IDLE);
			return;
		}
		put_unaligned_le32(lower_32_bits(ispfe->awb_spare_dma),
				   program + ispfe->awb_lo);
		put_unaligned_le32(upper_32_bits(ispfe->awb_spare_dma),
				   program + ispfe->awb_hi);
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
		if (ispfe->active_backend_side_output) {
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

/*
 * Freed only while the front end is stopped, and only after the work item that
 * copies out of them has been waited for -- it takes no lock this path holds,
 * so waiting here is safe and is what makes the lists below unreachable.
 */
static void ispfe_stats_areas_free(struct ispfe_device *ispfe)
{
	unsigned int i;
	unsigned int grid;

	cancel_work_sync(&ispfe->stats_work);

	scoped_guard(spinlock_irqsave, &ispfe->slock) {
		INIT_LIST_HEAD(&ispfe->stats_free);
		INIT_LIST_HEAD(&ispfe->stats_captured);
		memset(ispfe->stats_slot, 0, sizeof(ispfe->stats_slot));
	}

	for (i = 0; i < ARRAY_SIZE(ispfe->stats_areas); i++) {
		struct ispfe_stats_area *area = &ispfe->stats_areas[i];

		for (grid = 0; grid < ISPFE_STATS_GRIDS; grid++) {
			unsigned int output = ispfe_stats_grids[grid].output;

			/*
			 * Separately, because an allocation that got its
			 * pages and failed its mapping leaves one without the
			 * other and this is the path that cleans it up.
			 */
			if (area->grid[grid]) {
				dma_vunmap_noncontiguous(ispfe->dev,
							 area->grid[grid]);
				area->grid[grid] = NULL;
			}
			if (area->sgt[grid]) {
				dma_free_noncontiguous(ispfe->dev,
						       ispfe_pdma_outputs[output].size,
						       area->sgt[grid],
						       DMA_BIDIRECTIONAL);
				area->sgt[grid] = NULL;
			}
		}
		INIT_LIST_HEAD(&area->list);
	}

	/*
	 * The cancel above can have swallowed a schedule that buf_queue made,
	 * and the start this reallocation belongs to may still fail -- in
	 * which case nothing would wake the node until the next QBUF.  Put it
	 * back, so a buffer queued a moment ago is answered either way.
	 */
	if (READ_ONCE(ispfe->stats_streaming))
		queue_work(system_dfl_long_wq, &ispfe->stats_work);
}

/*
 * One allocation per grid per area, of the same size the shared allocation for
 * that output has: only the described part is ever copied out, but the writer
 * is the hardware's and this driver does not get to decide where it stops.
 */
static int ispfe_stats_areas_alloc(struct ispfe_device *ispfe)
{
	unsigned int i;
	unsigned int grid;

	if (ispfe->stats_areas[0].grid[0])
		return 0;

	for (i = 0; i < ARRAY_SIZE(ispfe->stats_areas); i++) {
		struct ispfe_stats_area *area = &ispfe->stats_areas[i];

		for (grid = 0; grid < ISPFE_STATS_GRIDS; grid++) {
			unsigned int output = ispfe_stats_grids[grid].output;
			size_t size = ispfe_pdma_outputs[output].size;

			if (size < ispfe_stats_grids[grid].size ||
			    size < ispfe_stats_grids[grid].clear)
				return -EINVAL;
			/*
			 * __GFP_ZERO, and it is not belt and braces:
			 * dma_alloc_coherent() added it for us and
			 * dma_alloc_noncontiguous() does not.  Only the 64-byte
			 * header of a grid is ever written by this driver, so
			 * without it every byte the hardware does not write is
			 * page-allocator memory -- and it goes to userspace,
			 * because a truncated grid still passes the header
			 * check in ispfe_stats_publish().
			 */
			area->sgt[grid] =
				dma_alloc_noncontiguous(ispfe->dev, size,
							DMA_BIDIRECTIONAL,
							GFP_KERNEL | __GFP_ZERO,
							0);
			if (!area->sgt[grid])
				return -ENOMEM;
			/*
			 * One address for the hardware, because the allocation
			 * is mapped into a single IOVA range however many
			 * physical chunks are behind it.  @nents is 1 for the
			 * same reason and is not worth testing -- the DMA core
			 * assigns it unconditionally.
			 *
			 * @orig_nents is the one to be careful of: it really is
			 * greater than one here, so the sync below has to stay
			 * dma_sync_sgtable_*(), which walks it.  A
			 * dma_sync_sg_*() over @nents would sync the first
			 * physical chunk and silently skip the rest.
			 */
			area->dma[grid] = sg_dma_address(area->sgt[grid]->sgl);
			area->grid[grid] =
				dma_vmap_noncontiguous(ispfe->dev, size,
						       area->sgt[grid]);
			if (!area->grid[grid])
				return -ENOMEM;
			if (upper_32_bits(area->dma[grid] + size - 1))
				return -ERANGE;
			/*
			 * Hand the whole allocation to the device once, here,
			 * so that everything after this point can rely on
			 * "the body is never dirty" without asking which
			 * allocator ran.  The zeroing above is a CPU write to
			 * all of it, and only the IOMMU path happens to clean
			 * afterwards; the direct fallback memsets through the
			 * cacheable linear map and leaves it dirty.  This is
			 * at probe, not on the frame path, so it is free.
			 */
			dma_sync_sgtable_for_device(ispfe->dev,
						    area->sgt[grid],
						    DMA_BIDIRECTIONAL);
		}
	}

	scoped_guard(spinlock_irqsave, &ispfe->slock)
		for (i = 0; i < ARRAY_SIZE(ispfe->stats_areas); i++)
			list_add_tail(&ispfe->stats_areas[i].list,
				      &ispfe->stats_free);

	return 0;
}

static void ispfe_buffers_free(struct ispfe_device *ispfe)
{
	unsigned int i;

	ispfe_stats_areas_free(ispfe);
	if (ispfe->awb_spare) {
		dma_free_coherent(ispfe->dev,
				  ispfe_pdma_outputs[ISPFE_PDMA_OUTPUT_AWB].size,
				  ispfe->awb_spare, ispfe->awb_spare_dma);
		ispfe->awb_spare = NULL;
	}
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
		ispfe->pdma_output[i].dma = 0;
	}
	if (ispfe->programs) {
		dma_free_coherent(ispfe->dev, PDMA_PROGRAMS_SIZE,
				  ispfe->programs, ispfe->programs_dma);
		ispfe->programs = NULL;
		ispfe->bayer_lo = 0;
		ispfe->bayer_hi = 0;
		ispfe->awb_lo = 0;
		ispfe->awb_hi = 0;
		ispfe->backend_image_lo = 0;
		ispfe->backend_image_hi = 0;
		ispfe->backend_header_lo = 0;
		ispfe->backend_header_hi = 0;
	}
	if (ispfe->lsc) {
		dma_free_coherent(ispfe->dev, ISPFE_LSC_AREA_BYTES,
				  ispfe->lsc, ispfe->lsc_dma);
		ispfe->lsc = NULL;
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

	memset(ispfe->awb_spare, 0,
	       ispfe_pdma_outputs[ISPFE_PDMA_OUTPUT_AWB].size);
	for (i = 0; i < ARRAY_SIZE(ispfe_pdma_outputs); i++) {
		const struct ispfe_pdma_output *output = &ispfe_pdma_outputs[i];

		if (!ispfe->pdma_output[i].cpu)
			continue;
		memset(ispfe->pdma_output[i].cpu, 0, output->size);
	}
}

/*
 * The working-area allocations are not checked here, and do not need to be:
 * ispfe_buffers_free() releases every allocation together and
 * ispfe_buffers_alloc() calls it on any failure, so there is no state in which
 * the pointers below are live and the outputs are not.  What used to be
 * checked was whether the LMP image tapouts were allocated *and should not
 * be*, which stopped being a question when the driver stopped running them.
 */
static bool ispfe_buffers_ready(struct ispfe_device *ispfe)
{
	if (!ispfe->frame || !ispfe->spare_frame || !ispfe->ring ||
	    !ispfe->programs || !ispfe->blocks || !ispfe->lsc ||
	    !ispfe->awb_spare || !ispfe->stats_areas[0].grid[0])
		return false;
	if (ispfe->active_backend_side_output && !ispfe->tnr_pyramid)
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
	size_t awb_size = ispfe_pdma_outputs[ISPFE_PDMA_OUTPUT_AWB].size;
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
	ispfe->lsc = dma_alloc_coherent(ispfe->dev, ISPFE_LSC_AREA_BYTES,
					&ispfe->lsc_dma, GFP_KERNEL);
	if (!ispfe->lsc) {
		ispfe_buffers_free(ispfe);
		return -ENOMEM;
	}
	ispfe->awb_spare =
		dma_alloc_coherent(ispfe->dev, awb_size,
				   &ispfe->awb_spare_dma, GFP_KERNEL);
	if (!ispfe->awb_spare) {
		ispfe_buffers_free(ispfe);
		return -ENOMEM;
	}
	ret = ispfe_stats_areas_alloc(ispfe);
	if (ret) {
		ispfe_buffers_free(ispfe);
		return ret;
	}
	if (ispfe->active_backend_side_output) {
		ispfe->tnr_pyramid = dma_alloc_coherent(
			ispfe->dev, ISPFE_TNR_PYRAMID_SIZE,
			&ispfe->tnr_pyramid_dma, GFP_KERNEL);
		if (!ispfe->tnr_pyramid) {
			ispfe_buffers_free(ispfe);
			return -ENOMEM;
		}
	}
	for (i = 0; i < ARRAY_SIZE(ispfe_pdma_outputs); i++) {
		if (ispfe_pdma_outputs[i].tapout)
			continue;
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
static int
ispfe_start(struct ispfe_device *ispfe, bool backend_consumer,
	    const struct exynos_becore_input_stream_config *stream_config)
{
	struct platform_device *pdev = to_platform_device(ispfe->dev);
	struct ispfe_source source = ispfe->src;
	struct v4l2_subdev_state *state;
	const struct v4l2_mbus_framefmt *fmt;
	u32 fc_axi_max_ost = ispfe->fc_axi_max_ost;
	u32 backend_recipe = ispfe->backend_recipe;
	bool pdma_program_override = ispfe->pdma_program_override;
	struct ispfe_lmp_wbg_profile lmp_wbg = {};
	int ret;

	/* A new attempt invalidates every result published by an older session. */
	WRITE_ONCE(ispfe->snapshot_state, ISPFE_SNAPSHOT_IDLE);

	if (backend_consumer != !!stream_config)
		return -EINVAL;

	/*
	 * The geometry the receiver is programmed with is the format on the
	 * subdev's source pad, read once here so that a later S_FMT cannot move
	 * it under a running stream.
	 */
	state = v4l2_subdev_lock_and_get_active_state(&ispfe->sd);
	fmt = v4l2_subdev_state_get_format(state, ispfe->source_pad);
	source.width = fmt->width;
	source.height = fmt->height;
	v4l2_subdev_unlock_state(state);

	/*
	 * The attached BE-core consumer is a fixed in-kernel pipeline, not a
	 * debugfs experiment.  Select its already validated producer profile
	 * here so the recipe-specific diagnostic knobs cannot change it.
	 */
	if (backend_consumer) {
		backend_recipe = 1;
		pdma_program_override = false;
	}

	ispfe->prog = ispfe_program_for(source.width, source.height,
					backend_recipe);
	if (!ispfe->prog) {
		dev_err(ispfe->dev,
			"no PDMA recipe captured for %ux%u\n",
			source.width, source.height);
		return -EINVAL;
	}
	if (ispfe->prog->lmp_wbg)
		lmp_wbg = *ispfe->prog->lmp_wbg;
	/*
	 * A stream starts on the driver's own default, which is unity: the grid
	 * *is* per-recipe -- it is a calibration of one lens at one readout, and
	 * two readouts of one camera differ by up to 30% at a grid point over
	 * the 54 captured vendor programs -- and that is exactly why the kernel
	 * has no business choosing one.  Correcting nothing is the only
	 * position that is not somebody else's calibration.
	 */
	ispfe->lsc_input = ispfe_lsc_input(ispfe->prog);
	if (ispfe->lsc_input == -EINVAL)
		return -EINVAL;
	if (ispfe->lsc_input >= 0) {
		const u8 *table = ispfe_lsc_default(ispfe, ispfe->prog);

		/*
		 * Refused here rather than later: a recipe that names a
		 * shading table it does not carry has no default to restore,
		 * and every path that restores one runs after this.
		 */
		if (!table)
			return -EINVAL;
		memcpy(ispfe->shading, table, ISPFE_LSC_LUT_BYTES);
	}
	if (backend_consumer) {
		/*
		 * lmp_wbg with the rest: the back end is told the gains every
		 * frame was taken through, and a recipe with no white balance
		 * stage has none to tell it.  Refusing here says so once,
		 * where refusing at the first frame would only say -ERANGE.
		 */
		if (!ispfe->prog->backend_recipe ||
		    !ispfe->prog->fixed_resources ||
		    !ispfe->prog->lmp_wbg)
			return -EINVAL;
		source.loch = ispfe->prog->required_loch;
		source.fcctx = ispfe->prog->required_fcctx;
		fc_axi_max_ost = ispfe->prog->required_fc_axi_max_ost;
		if (stream_config->red_balance <
		    EXYNOS_BECORE_WBG_GAIN_MIN_Q12 ||
		    stream_config->red_balance >
		    EXYNOS_BECORE_WBG_GAIN_MAX_Q12 ||
		    stream_config->blue_balance <
		    EXYNOS_BECORE_WBG_GAIN_MIN_Q12 ||
		    stream_config->blue_balance >
		    EXYNOS_BECORE_WBG_GAIN_MAX_Q12)
			return -ERANGE;
		lmp_wbg.red = stream_config->red_balance;
		lmp_wbg.blue = stream_config->blue_balance;
	}

	/*
	 * Validate, then snapshot.  Every one of these ends up as an array
	 * index, a shift or an MMIO offset, and they stay writable while a
	 * stream is up -- so checking them and then reading them again later
	 * would leave the check meaning nothing.
	 */
	if (source.link >= CSIS_NUM_LINKS ||
	    source.phy >= PHY_NUM_INSTANCES ||
	    source.lanes < 1 ||
	    source.lanes > ispfe_phy_lanes(source.phy) ||
	    source.loch >= LOCH_COUNT || source.fcctx >= FC_NUM_CTX ||
	    backend_recipe > 1 || source.width - 1 >= U16_MAX ||
	    source.height - 1 >= U16_MAX)
		return -EINVAL;
	if (ispfe->prog->backend_recipe && ispfe->owner == ISPFE_OWNER_V4L2)
		return -EOPNOTSUPP;
	if (ispfe->prog->fixed_resources &&
	    (source.loch != ispfe->prog->required_loch ||
	     source.fcctx != ispfe->prog->required_fcctx ||
	     fc_axi_max_ost != ispfe->prog->required_fc_axi_max_ost)) {
		dev_err(ispfe->dev,
			"PDMA recipe needs loch %u, FC %u, AXI max OST %#x\n",
			ispfe->prog->required_loch, ispfe->prog->required_fcctx,
			ispfe->prog->required_fc_axi_max_ost);
		return -EINVAL;
	}
	if (pdma_program_override) {
		if (ispfe->prog->backend_recipe)
			return -EINVAL;
		ret = ispfe_pdma_staged_validate(ispfe);
		if (ret) {
			dev_err(ispfe->dev,
				"staged PDMA program does not match the %ux%u recipe\n",
				source.width, source.height);
			return ret;
		}
	}

	ispfe->active = source;
	ispfe->active_fc_axi_max_ost = fc_axi_max_ost;
	ispfe->active_lmp_wbg = lmp_wbg;
	/*
	 * Every captured program meters through the same thresholds, so unlike
	 * the white balance there is no per-recipe profile to select -- what a
	 * stream starts at is the vendor's, and a parameters buffer moves it.
	 */
	ispfe->active_lmp_metering = ispfe_lmp_metering_captured;
	ispfe->active_backend_recipe = ispfe->prog->backend_recipe;
	/*
	 * A recipe that was captured with the LMP main-Bayer side output keeps
	 * it: that is the back-end producer's own program.  Grafting one onto a
	 * recipe captured without it is a diagnostic, and it is not free -- the
	 * full-mode raw recipe carrying it receives frames but never completes
	 * an LMP one, so an ordinary raw capture must not pay for it.
	 */
	ispfe->active_backend_side_output = ispfe->prog->backend_output &&
		(!ispfe->prog->patch_backend_output ||
		 ispfe->backend_side_output);
	ispfe->active_pdma_program_override = pdma_program_override;
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
	if (ispfe->active_backend_side_output &&
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

	/*
	 * The frame counter belongs to the stream, so it restarts with the
	 * others rather than on the one path that used to do it.  Only
	 * ispfe_start_streaming() reset it, which made it a boot-lifetime
	 * counter for every other owner: two back-to-back back-end streams
	 * reported the hardware's own frame_id as 93 both times -- it does
	 * restart -- while this counter went 1397 and then 1548, growing by 151
	 * against the 150 frames streamed in between.  A number that does not
	 * restart with the thing it counts cannot be paired with anything, and
	 * V4L2's `sequence` is the frame number.
	 *
	 * Here and not earlier in this function because ispfe_frame_sync()
	 * states as a property that this and frame_start reset together, and a
	 * start refused by one of the validation returns above would otherwise
	 * zero one and not the other.
	 */
	ispfe->sequence = 0;
	atomic_set(&ispfe->frame_start, 0);
	atomic_set(&ispfe->frame_end, 0);
	atomic_set(&ispfe->fc_events, 0);
	atomic_set(&ispfe->core_events, 0);
	atomic_set(&ispfe->lmp_events, 0);
	atomic_set(&ispfe->pdma_events, 0);
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
	/*
	 * The interrupts are gone, so an area a slot was still writing into
	 * will never be finished by a frame.  Release those and wake the
	 * metadata node: without a frame to wait for, a buffer queued to it is
	 * completed rather than held.
	 */
	scoped_guard(spinlock_irqsave, &ispfe->slock)
		ispfe_stats_untake_all_locked(ispfe);
	queue_work(system_dfl_long_wq, &ispfe->stats_work);
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
		WRITE_ONCE(ispfe->owner, ISPFE_OWNER_DEBUGFS);
		ret = ispfe_start(ispfe, false, NULL);
		if (ret)
			WRITE_ONCE(ispfe->owner, ISPFE_OWNER_NONE);
	} else {
		ispfe_stop(ispfe);
		WRITE_ONCE(ispfe->owner, ISPFE_OWNER_NONE);
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
		WRITE_ONCE(ispfe->owner, ISPFE_OWNER_DEBUGFS);
		ret = ispfe_sensor_power(ispfe, true);
		if (ret)
			goto err_owner;
		ret = ispfe_start(ispfe, false, NULL);
		if (ret)
			goto err_power;
		ret = v4l2_subdev_enable_streams(&ispfe->sd,
						 ispfe->source_pad, BIT_ULL(0));
		if (ret)
			goto err_stop;
		ispfe->sensor_streaming = true;
		return 0;
	}

	ispfe_stop(ispfe);
	ret = v4l2_subdev_disable_streams(&ispfe->sd, ispfe->source_pad,
					  BIT_ULL(0));
	if (ret)
		dev_err(ispfe->dev, "cannot stop the sensor: %d\n", ret);
	ispfe_sensor_power(ispfe, false);
	ispfe->sensor_streaming = false;
	WRITE_ONCE(ispfe->owner, ISPFE_OWNER_NONE);

	return 0;

err_stop:
	ispfe_stop(ispfe);
err_power:
	ispfe_sensor_power(ispfe, false);
err_owner:
	WRITE_ONCE(ispfe->owner, ISPFE_OWNER_NONE);
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
	ispfe_stats_untake_all_locked(ispfe);
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
	ispfe_stats_untake_all_locked(ispfe);
	for (i = 0; i < ARRAY_SIZE(ispfe->backend_buffers); i++) {
		INIT_LIST_HEAD(&ispfe->backend_buffers[i].list);
		ispfe->backend_buffers[i].state = ISPFE_BACKEND_BUFFER_IDLE;
	}
	spin_unlock_irq(&ispfe->slock);
}

/*
 * Give the front end back unless a reservation is still holding it.
 *
 * A consumer that reserved at its STREAMON keeps the front end across a stop
 * and across a failed start -- what it asked for was the producer, not this
 * one stream -- and hands it back from .unreserve alone.
 */
static void ispfe_backend_release_owner(struct ispfe_device *ispfe)
{
	if (!ispfe->backend_reserved)
		WRITE_ONCE(ispfe->owner, ISPFE_OWNER_NONE);
}

static int ispfe_backend_queue_start(struct ispfe_device *ispfe,
	bool consumer,
	const struct exynos_becore_input_stream_config *stream_config)
{
	bool ready;
	int ret;

	if (ispfe->streaming)
		return -EBUSY;
	/*
	 * The consumer may start inside its own reservation, and only the
	 * consumer: the debugfs diagnostic reaches this function too, and
	 * letting it in on `owner == BACKEND` would let it arm the receiver
	 * inside somebody else's claim.
	 */
	if (ispfe->owner != ISPFE_OWNER_NONE &&
	    !(consumer && ispfe->backend_reserved &&
	      ispfe->owner == ISPFE_OWNER_BACKEND))
		return -EBUSY;
	if (consumer != !!stream_config)
		return -EINVAL;
	if (!consumer && ispfe->backend_recipe != 1)
		return -EINVAL;

	WRITE_ONCE(ispfe->owner, ISPFE_OWNER_BACKEND);
	ispfe_backend_queue_reset(ispfe);
	spin_lock_irq(&ispfe->slock);
	ispfe->backend_queue_active = true;
	spin_unlock_irq(&ispfe->slock);

	ret = ispfe_sensor_power(ispfe, true);
	if (ret)
		goto err_queue;
	ret = ispfe_start(ispfe, consumer, stream_config);
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

	ret = v4l2_subdev_enable_streams(&ispfe->sd, ispfe->source_pad,
					 BIT_ULL(0));
	if (ret)
		goto err_stop;
	ispfe->sensor_streaming = true;
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
	ispfe_backend_release_owner(ispfe);
	return ret;
}

static void ispfe_backend_queue_stop(struct ispfe_device *ispfe)
{
	int ret;

	spin_lock_irq(&ispfe->slock);
	ispfe->backend_queue_active = false;
	spin_unlock_irq(&ispfe->slock);
	cancel_work_sync(&ispfe->backend_fill_work);
	ispfe_stop(ispfe);
	cancel_work_sync(&ispfe->backend_fill_work);
	ret = v4l2_subdev_disable_streams(&ispfe->sd, ispfe->source_pad,
					  BIT_ULL(0));
	if (ret)
		dev_err(ispfe->dev, "cannot stop the sensor: %d\n", ret);
	ispfe_sensor_power(ispfe, false);
	ispfe->sensor_streaming = false;
	ispfe_backend_queue_abort_all(ispfe);
	ispfe_backend_release_owner(ispfe);
}

/*
 * Hold the front end for a consumer without starting it, and give it back.
 *
 * The pair exists so that the consumer's STREAMON can mean something: see the
 * producer ops in <media/exynos-becore.h>.  What it leaves this driver in is a
 * state it has never had -- owned, with `streaming` false and no sensor
 * running -- so every reader that means "a frame is coming" has to say so, and
 * the ones that do already check `streaming` beside `owner`.
 */
static int ispfe_backend_reserve(void *data)
{
	struct ispfe_device *ispfe = data;

	guard(mutex)(&ispfe->lock);
	if (ispfe->streaming || ispfe->owner != ISPFE_OWNER_NONE)
		return -EBUSY;

	WRITE_ONCE(ispfe->owner, ISPFE_OWNER_BACKEND);
	ispfe->backend_reserved = true;

	return 0;
}

static void ispfe_backend_unreserve(void *data)
{
	struct ispfe_device *ispfe = data;

	guard(mutex)(&ispfe->lock);
	if (!ispfe->backend_reserved)
		return;

	ispfe->backend_reserved = false;
	/*
	 * A consumer that stops before it unreserves has already been through
	 * ispfe_backend_queue_stop(), which left the owner alone because the
	 * reservation was still up.  One that unreserves without ever having
	 * started has nothing else to undo.  Either way this is the release.
	 */
	ispfe_backend_release_owner(ispfe);
}

static int ispfe_backend_stream_start(void *data,
				      const struct exynos_becore_input_stream_config *stream_config)
{
	struct ispfe_device *ispfe = data;
	int ret;

	guard(mutex)(&ispfe->lock);
	if (ispfe->backend_queue_active)
		return -EBUSY;

	ret = ispfe_backend_queue_start(ispfe, true, stream_config);
	if (!ret)
		ispfe->backend_queue_consumer = true;

	return ret;
}

static void ispfe_backend_stream_stop(void *data)
{
	struct ispfe_device *ispfe = data;

	guard(mutex)(&ispfe->lock);
	if (!ispfe->backend_queue_consumer)
		return;

	ispfe->backend_queue_consumer = false;
	ispfe_backend_queue_stop(ispfe);
}

static const struct exynos_becore_input_producer_ops ispfe_backend_ops = {
	.reserve = ispfe_backend_reserve,
	.unreserve = ispfe_backend_unreserve,
	.start_streaming = ispfe_backend_stream_start,
	.stop_streaming = ispfe_backend_stream_stop,
};

static int ispfe_backend_queue_set(void *data, u64 val)
{
	struct ispfe_device *ispfe = data;

	if (val > 1)
		return -EINVAL;

	guard(mutex)(&ispfe->lock);
	/*
	 * `backend_reserved` as well as `backend_queue_consumer`: a consumer
	 * that has reserved and not yet started owns the front end and has no
	 * stream flag up, so this would otherwise read as idle.
	 */
	if (ispfe->backend_queue_consumer || ispfe->backend_reserved)
		return -EBUSY;
	if (!!val == ispfe->backend_queue_active)
		return 0;
	if (val)
		return ispfe_backend_queue_start(ispfe, false, NULL);

	ispfe_backend_queue_stop(ispfe);
	return 0;
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
	u64 credits;
	unsigned int isolation, i, flight;
	dma_addr_t backend_dma;
	unsigned long backend_programs, slots;
	int backend_error;
	bool backend_active;

	guard(mutex)(&ispfe->lock);
	backend_dma = ispfe->backend_buffer.dma;

	seq_printf(s, "streaming    %u\n", ispfe->streaming);
	seq_printf(s, "sensor_stream %u\n", ispfe->sensor_streaming);
	seq_printf(s, "owner        %s%s\n",
		   ispfe->owner == ISPFE_OWNER_V4L2 ? "v4l2" :
		   ispfe->owner == ISPFE_OWNER_BACKEND ? "backend" :
		   ispfe->owner == ISPFE_OWNER_DEBUGFS ? "debugfs" : "none",
		   /*
		    * Owned and idle is a state worth being able to see: it is
		    * what a consumer's STREAMON leaves behind until it has a
		    * buffer to stream, and it reads exactly like a stopped
		    * front end everywhere else in this file.
		    */
		   ispfe->backend_reserved ? " (reserved)" : "");
	scoped_guard(spinlock_irqsave, &ispfe->slock) {
		flight = ispfe->flight_count;
		slots = ispfe->slots_used;
		backend_flight = ispfe->backend_flight_count;
		backend_programs = ispfe->backend_programs_used;
		backend_credits = ispfe->backend_credit_count;
		credits = ispfe->credit_count;
		backend_completed = ispfe->backend_completed;
		backend_dropped = ispfe->backend_dropped;
		backend_error = ispfe->backend_queue_error;
		backend_active = ispfe->backend_queue_active;
		list_for_each(pos, &ispfe->backend_ready)
			backend_ready++;
		list_for_each(pos, &ispfe->backend_done)
			backend_done++;
	}
	seq_printf(s,
		   "queue        %u in flight, slots %#lx, credits %llu, seq %u\n",
		   flight, slots, credits, ispfe->sequence);
	seq_printf(s,
		   "backend_queue active %u, consumer %u, ready %u, flight %u, done %u, programs %#lx, credits %llu, completed %u, dropped %u, error %d\n",
		   backend_active, ispfe->backend_queue_consumer, backend_ready,
		   backend_flight, backend_done, backend_programs,
		   backend_credits, backend_completed, backend_dropped,
		   backend_error);
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
	seq_printf(s, "lmp_wbg       active R/Gr/Gb/B %u/%u/%u/%u Q12\n",
		   ispfe->active_lmp_wbg.red,
		   ispfe->active_lmp_wbg.green_red,
		   ispfe->active_lmp_wbg.green_blue,
		   ispfe->active_lmp_wbg.blue);
	seq_printf(s, "lmp_metering  awb sat/dark %d/%d, luma %u/%u/%u/%u in %u..%u, coring %u\n",
		   ispfe->active_lmp_metering.awb_saturation,
		   ispfe->active_lmp_metering.awb_dark,
		   ispfe->active_lmp_metering.awb_luma_coeff[0],
		   ispfe->active_lmp_metering.awb_luma_coeff[1],
		   ispfe->active_lmp_metering.awb_luma_coeff[2],
		   ispfe->active_lmp_metering.awb_luma_coeff[3],
		   ispfe->active_lmp_metering.awb_luma_low,
		   ispfe->active_lmp_metering.awb_luma_high,
		   ispfe->active_lmp_metering.awb_diff_coring);
	seq_printf(s, "              ae  sat/dark %d/%d\n",
		   ispfe->active_lmp_metering.ae_saturation,
		   ispfe->active_lmp_metering.ae_dark);
	/*
	 * Four samples of the 33 x 25 grid rather than the grid: the centre and
	 * the two horizontal edges of the middle row say what a shading table
	 * is doing, and the corner says how far it goes.  Green-red, because
	 * the four channels move together and one of them is enough to see a
	 * table change.
	 */
	seq_printf(s, "lmp_shading   %s, Gr edge/centre/edge %u/%u/%u, corner %u Q12\n",
		   ispfe->lsc_input < 0 ? "not in this recipe" :
		   ispfe->prog && ispfe_lsc_default(ispfe, ispfe->prog) &&
		   !memcmp(ispfe->shading,
			   ispfe_lsc_default(ispfe, ispfe->prog),
			   ISPFE_LSC_LUT_BYTES) ?
		   "the driver's default" : "from userspace",
		   ispfe_lsc_sample(ispfe->shading, EXYNOS_ISPFE_LSC_ROWS / 2,
				    0, EXYNOS_ISPFE_WB_GREEN_RED),
		   ispfe_lsc_sample(ispfe->shading, EXYNOS_ISPFE_LSC_ROWS / 2,
				    EXYNOS_ISPFE_LSC_COLUMNS / 2,
				    EXYNOS_ISPFE_WB_GREEN_RED),
		   ispfe_lsc_sample(ispfe->shading, EXYNOS_ISPFE_LSC_ROWS / 2,
				    EXYNOS_ISPFE_LSC_COLUMNS - 1,
				    EXYNOS_ISPFE_WB_GREEN_RED),
		   ispfe_lsc_sample(ispfe->shading, 0, 0,
				    EXYNOS_ISPFE_WB_GREEN_RED));
	seq_printf(s, "fc_axi_max_ost %#x requested, %#x active\n",
		   ispfe->fc_axi_max_ost, ispfe->active_fc_axi_max_ost);
	seq_printf(s, "backend_recipe %u requested, %u active\n",
		   ispfe->backend_recipe, ispfe->active_backend_recipe);
	seq_printf(s, "backend_side_output %u requested, %u active\n",
		   ispfe->backend_side_output,
		   ispfe->active_backend_side_output);
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
	seq_printf(s, "awb_reloc    %#x/%#x, spare %pad\n", ispfe->awb_lo,
		   ispfe->awb_hi, &ispfe->awb_spare_dma);
	seq_printf(s,
		   "stats        streaming %u, published %u, empty %u, dropped %u, stale %u\n",
		   READ_ONCE(ispfe->stats_streaming), ispfe->stats_published,
		   ispfe->stats_empty, ispfe->stats_dropped,
		   ispfe->stats_stale);
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
	for (i = 0; i < ARRAY_SIZE(ispfe_pdma_outputs); i++) {
		if (!ispfe->pdma_output[i].cpu) {
			seq_printf(s, "aux%02u_iova  %-18s  size %zu\n", i,
				   ispfe_pdma_outputs[i].tapout ?
				   "gated" : "unallocated",
				   ispfe_pdma_outputs[i].size);
			continue;
		}
		seq_printf(s, "aux%02u_iova  %pad  size %zu\n", i,
			   &ispfe->pdma_output[i].dma,
			   ispfe_pdma_outputs[i].size);
	}
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

/*
 * The live shading table, in the hardware's own tiled layout, readable while
 * streaming -- which is the point: it is the only way to see that what a
 * parameters buffer asked for is what a frame will be corrected by, and the
 * tiling is exactly the part a test cannot check any other way.  Copied under
 * the lock that swaps it, because a torn read of half of one table and half of
 * another would be a diagnostic that lies.
 */
static ssize_t ispfe_lmp_shading_read(struct file *file, char __user *buf,
				      size_t count, loff_t *ppos)
{
	struct ispfe_device *ispfe = file->private_data;
	ssize_t ret;
	u8 *copy;

	if (ispfe->lsc_input < 0)
		return -ENODATA;

	copy = kmalloc(ISPFE_LSC_LUT_BYTES, GFP_KERNEL);
	if (!copy)
		return -ENOMEM;
	scoped_guard(spinlock_irqsave, &ispfe->slock)
		memcpy(copy, ispfe->shading, ISPFE_LSC_LUT_BYTES);
	ret = simple_read_from_buffer(buf, count, ppos, copy,
				      ISPFE_LSC_LUT_BYTES);
	kfree(copy);

	return ret;
}

static const struct file_operations ispfe_lmp_shading_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = ispfe_lmp_shading_read,
	.llseek = default_llseek,
};

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

static int ispfe_lmp_awb_stats_open(struct inode *inode, struct file *file)
{
	struct ispfe_device *ispfe = inode->i_private;
	struct ispfe_awb_snapshot_file *snapshot;
	int ret = 0;

	snapshot = kzalloc_obj(*snapshot, GFP_KERNEL);
	if (!snapshot)
		return -ENOMEM;
	snapshot->size = ispfe_pdma_outputs[ISPFE_PDMA_OUTPUT_AWB].size;
	snapshot->data = vmalloc(snapshot->size);
	if (!snapshot->data) {
		kfree(snapshot);
		return -ENOMEM;
	}

	mutex_lock(&ispfe->lock);
	if (ispfe->streaming) {
		ret = -EBUSY;
	} else {
		/* Pairs with the EOF-side publication of the drained snapshot. */
		if (smp_load_acquire(&ispfe->snapshot_state) !=
		    ISPFE_SNAPSHOT_READY)
			ret = -EAGAIN;
		else if (!ispfe->pdma_output[ISPFE_PDMA_OUTPUT_AWB].cpu)
			ret = -ENODATA;
		else
			memcpy(snapshot->data,
			       ispfe->pdma_output[ISPFE_PDMA_OUTPUT_AWB].cpu,
			       snapshot->size);
	}
	mutex_unlock(&ispfe->lock);

	if (ret) {
		vfree(snapshot->data);
		kfree(snapshot);
		return ret;
	}
	file->private_data = snapshot;

	return 0;
}

static ssize_t ispfe_lmp_awb_stats_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	struct ispfe_awb_snapshot_file *snapshot = file->private_data;

	return simple_read_from_buffer(buf, count, ppos, snapshot->data,
				       snapshot->size);
}

static int ispfe_lmp_awb_stats_release(struct inode *inode, struct file *file)
{
	struct ispfe_awb_snapshot_file *snapshot = file->private_data;

	vfree(snapshot->data);
	kfree(snapshot);

	return 0;
}

static const struct file_operations ispfe_lmp_awb_stats_fops = {
	.owner = THIS_MODULE,
	.open = ispfe_lmp_awb_stats_open,
	.read = ispfe_lmp_awb_stats_read,
	.release = ispfe_lmp_awb_stats_release,
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
	debugfs_create_u32("backend_recipe", 0644, d,
			   &ispfe->backend_recipe);
	debugfs_create_bool("backend_side_output", 0644, d,
			    &ispfe->backend_side_output);
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
	debugfs_create_file("lmp_shading", 0444, d, ispfe,
			    &ispfe_lmp_shading_fops);
	debugfs_create_file("program_override", 0644, d, ispfe,
			    &ispfe_program_override_fops);
	debugfs_create_file("lmp_awb_stats", 0444, d, ispfe,
			    &ispfe_lmp_awb_stats_fops);
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
	ispfe_fill_pix(v4l2_subdev_state_get_format(state, ispfe->source_pad),
		       pix);
	v4l2_subdev_unlock_state(state);
}

/*
 * Hand the queue's buffers back and put its counters and slot bitmap where a
 * fresh stream expects them.  All of that is this node's own.
 *
 * The statistics areas armed against the front end's program slots go back
 * too, and those are *not* this node's own: the array is indexed by slot and
 * the back end arms it from the same range, so it belongs to whoever holds
 * ispfe->owner.  Untaking them is only safe where the front end was ours and
 * the stream that armed them is ending, because then nothing is going to
 * finish them -- and both callers are now inside the claim
 * .prepare_streaming took, which is what makes that unconditional.  It used
 * to be a parameter, for two early exits that ran before the front end was
 * claimed and no longer exist.
 */
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
		ispfe->credit_count = 0;
		ispfe->slots_used = 0;
		ispfe_stats_untake_all_locked(ispfe);
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
 *
 * Its work item runs on system_highpri_wq for the frame deadline explained
 * above ispfe_backend_fill_work(), which this path shares: it retires frames
 * the same way and the same statistics node feeds off it.
 */
static void ispfe_queue_fill(struct ispfe_device *ispfe)
{
	for (;;) {
		const struct ispfe_stats_area *stats;
		struct ispfe_buffer *buf;
		const u8 *shading = NULL;
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
		stats = ispfe_stats_take(ispfe, slot);
		/*
		 * The raw recipes carry no white balance stage, so those gains
		 * change nothing here; the shading table is theirs too, though
		 * what it corrects is the LMP path rather than the Bayer this
		 * queue writes.  A buffer is consumed either way, because one
		 * that is never taken is a DQBUF that never returns -- and
		 * userspace has no way to know which program a stream picked.
		 */
		ispfe_params_consume(ispfe, NULL, &shading);
		ret = ispfe_pdma_encode(ispfe, slot, dma,
					ispfe->backend_buffer.dma, stats,
					shading);
		if (ret) {
			scoped_guard(spinlock_irqsave, &ispfe->slock) {
				ispfe_stats_untake_locked(ispfe, slot);
				__clear_bit(slot, &ispfe->slots_used);
			}
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
	 * Not vb2_is_streaming(): with any non-zero min_queued_buffers, a
	 * STREAMON with nothing queued succeeds -- the check there is on
	 * buffers *allocated* -- and defers the actual start to the next QBUF,
	 * which enqueues every buffer into the driver *before* calling
	 * start_streaming.  So the queue can be streaming while there is still
	 * no program area to encode into.  Gate on the driver's own state
	 * instead; ispfe_start_streaming() drains what accumulated.
	 *
	 * And not on `owner` alone either, since the front end is claimed from
	 * .prepare_streaming: owning it now means STREAMON returned, not that
	 * the receiver is armed.  `streaming` is what ispfe_start() sets, and
	 * the pair is the same test ispfe_stats_producing() makes for the same
	 * distinction.
	 */
	if (ispfe->owner == ISPFE_OWNER_V4L2 && ispfe->streaming)
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
	struct ispfe_link *link = ispfe_active_link(ispfe);
	int ret;

	if (!link || !link->sensor)
		return on ? -ENODEV : 0;

	if (on) {
		ret = v4l2_subdev_call(link->sensor, video, pre_streamon, 0);
		return ret == -ENOIOCTLCMD ? 0 : ret;
	}

	ret = v4l2_subdev_call(link->sensor, video, post_streamoff);
	if (ret && ret != -ENOIOCTLCMD)
		dev_err(ispfe->dev, "cannot power the sensor down: %d\n", ret);

	return 0;
}

/*
 * Claim the front end at STREAMON, before any buffer reaches the driver.
 *
 * vb2 calls this from VIDIOC_STREAMON unconditionally, where it defers
 * start_streaming until %min_queued_buffers are *queued* -- so this is the
 * only hook that can make a successful STREAMON mean "the front end is ours".
 * Between the two nothing is armed and no frame is coming; what changes is
 * that a sink link change, a second consumer and a debugfs capture are all
 * refused from the moment userspace was told the stream was on, rather than
 * from whenever it gets round to queueing a buffer.
 *
 * Claimed before the walk rather than after it, for two reasons that are the
 * same reason: what runs inside the walk needs to know whose start this is.
 * ispfe_vdev_link_validate() reads it to tell its own capture from a processed
 * one that merely reaches this pad, and ispfe_link_setup() reads it to refuse
 * a link change under a stream -- which it can only do for a start that has
 * already claimed, and the walk takes the graph mutex that .link_setup runs
 * under.
 *
 * No buffer is handed back on either failure because the driver holds none:
 * vb2 enqueues into the driver from vb2_start_streaming(), which is downstream
 * of here on every path.
 *
 * The walk can succeed with *no* camera selected, which is a state worth
 * naming: .has_pad_interdep answers false for every pair when there is no
 * active link, so the walk never reaches a MUST_CONNECT sink and nothing
 * refuses it.  STREAMON then returns success and the first buffer fails with
 * -ENODEV out of ispfe_sensor_power(), with the claim held until STREAMOFF --
 * so a caller that disabled every link by hand cannot re-enable one without
 * stopping first.  Deliberate to reach, self-healing on STREAMOFF, and the
 * honest reading of a STREAMON that was told yes.
 */
static int ispfe_prepare_streaming(struct vb2_queue *q)
{
	struct ispfe_device *ispfe = vb2_get_drv_priv(q);
	int ret;

	if (ispfe->streaming || ispfe->owner != ISPFE_OWNER_NONE)
		return -EBUSY;

	WRITE_ONCE(ispfe->owner, ISPFE_OWNER_V4L2);

	ret = video_device_pipeline_start(&ispfe->vdev, &ispfe->pipe);
	if (ret) {
		WRITE_ONCE(ispfe->owner, ISPFE_OWNER_NONE);
		return ret;
	}

	return 0;
}

/*
 * And give it back.  The rule is that every *successful* prepare is matched by
 * exactly one unprepare, which is worth stating that way because the three
 * places vb2 calls it from do not share a simpler one: STREAMOFF and close
 * when `q->streaming` is set, and vb2_core_streamon()'s own unwind when a
 * start attempted from there fails -- where `q->streaming` was never set,
 * because it is assigned after the attempt.  So this is the counterpart on
 * every path, including a STREAMON that never received a buffer and a deferred
 * start that failed at the first QBUF.
 *
 * The order is the take's, not its reverse, and it is safe for a reason the
 * back end's teardown cannot use: this runs under ispfe->lock, which is the
 * raw queue's own %vb2_queue.lock, and every other claimer of the front end
 * takes it before it can act on the owner it reads.
 */
static void ispfe_unprepare_streaming(struct vb2_queue *q)
{
	struct ispfe_device *ispfe = vb2_get_drv_priv(q);

	WRITE_ONCE(ispfe->owner, ISPFE_OWNER_NONE);
	video_device_pipeline_stop(&ispfe->vdev);
}

static int ispfe_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct ispfe_device *ispfe = vb2_get_drv_priv(q);
	int ret;

	ret = ispfe_sensor_power(ispfe, true);
	if (ret)
		goto err_return;

	ret = ispfe_start(ispfe, false, NULL);
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
	ret = v4l2_subdev_enable_streams(&ispfe->sd, ispfe->source_pad,
					 BIT_ULL(0));
	if (ret)
		goto err_stop;
	ispfe->sensor_streaming = true;

	return 0;

err_stop:
	ispfe_stop(ispfe);
	/*
	 * This is the only label a fill can have run under -- the others are
	 * reached before ispfe_queue_fill() is called, and so is this one's
	 * geometry check, which makes the wait a no-op there rather than
	 * wrong.  It is needed because the work item holds a buffer off all
	 * three lists for the length of an encode, so a copy running now would
	 * still be holding one when they are emptied below, and vb2 would take
	 * that buffer back from under it.  ispfe_stop() has freed the
	 * interrupts, so nothing queues the work again once this returns.
	 * ispfe_stop_streaming() waits at the same point and for the same
	 * reason.
	 */
	cancel_work_sync(&ispfe->fill_work);
err_power:
	ispfe_sensor_power(ispfe, false);
err_return:
	/*
	 * The front end is ours on every path here -- .prepare_streaming
	 * claimed it and .unprepare_streaming is what gives it back -- so the
	 * areas are ours to untake, and untaking under the claim is what makes
	 * that true rather than merely likely.
	 */
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
	ret = v4l2_subdev_disable_streams(&ispfe->sd, ispfe->source_pad,
					  BIT_ULL(0));
	if (ret)
		dev_err(ispfe->dev, "cannot stop the sensor: %d\n", ret);
	ispfe_sensor_power(ispfe, false);
	ispfe->sensor_streaming = false;
	cancel_work_sync(&ispfe->fill_work);
	/*
	 * The claim and the pipeline are .unprepare_streaming's, which vb2 runs
	 * straight after this.  The buffers are not: vb2 audits them the moment
	 * this returns.
	 */
	ispfe_queue_return_all(ispfe, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops ispfe_vb2_ops = {
	.queue_setup = ispfe_queue_setup,
	.buf_prepare = ispfe_buf_prepare,
	.buf_queue = ispfe_buf_queue,
	.prepare_streaming = ispfe_prepare_streaming,
	.start_streaming = ispfe_start_streaming,
	.stop_streaming = ispfe_stop_streaming,
	.unprepare_streaming = ispfe_unprepare_streaming,
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
	code = v4l2_subdev_state_get_format(state, ispfe->source_pad)->code;
	ispfe_fill_pix(v4l2_subdev_state_get_format(state, ispfe->source_pad),
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
 * would have the receiver write frames of one size into buffers of another,
 * which is what this refuses.
 *
 * **It is a check on this node's own capture, and this node is not the only
 * thing that starts a pipeline containing it.** The back end's processed path
 * starts one from its own sink pad, and that walk reaches this pad too,
 * because the receiver's source pad links to both and the walk follows every
 * enabled link of a pad it has taken. A queue with no capture in flight has no
 * buffers to be too small, so validating it there would refuse a processed
 * capture over the size of buffers nobody is filling.
 *
 * `owner` is what separates the two, and it is why ispfe_prepare_streaming()
 * claims it before it walks: inside that walk, this node owning the front end
 * means this node is the one starting.
 */
static int ispfe_vdev_link_validate(struct media_link *link)
{
	struct video_device *vdev =
		media_entity_to_video_device(link->sink->entity);
	struct ispfe_device *ispfe = video_get_drvdata(vdev);
	struct v4l2_pix_format pix;

	if (READ_ONCE(ispfe->owner) != ISPFE_OWNER_V4L2)
		return 0;

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

/* ---- the statistics metadata node --------------------------------------- */

/*
 * LMP meters every frame the front end receives, and until now those results
 * only left the driver through a debugfs drain that stops the stream to get
 * one untorn frame out.  This is the per-frame form of the same thing: one
 * buffer per frame, on a V4L2_BUF_TYPE_META_CAPTURE node, carrying the grids
 * in the units the hardware measured them in.
 *
 * It does not own the front end and cannot start one.  Statistics belong to
 * whichever frame is being captured, so this node is fed by whatever is
 * running -- the raw capture node or the back end's consumer -- and produces
 * nothing of its own when nothing is.
 */

struct ispfe_stats_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

static struct ispfe_stats_buffer *
to_ispfe_stats_buffer(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct ispfe_stats_buffer, vb);
}

/*
 * Copy one frame's grids into a buffer and hand it back.  @area is NULL when
 * there was no frame -- the buffer then says so with no measurement flags set,
 * rather than being held for a frame nobody is going to run.
 *
 * A grid is only published when the hardware's own metadata says it metered
 * the geometry this block meters.  The header is cleared before the frame is
 * armed, so a grid that was not written this frame reports nothing and a stale
 * result cannot be presented as a fresh one.
 */
static void ispfe_stats_publish(struct ispfe_device *ispfe,
				struct ispfe_stats_buffer *buf,
				const struct ispfe_stats_area *area)
{
	struct exynos_ispfe_stats_buffer *out =
		vb2_plane_vaddr(&buf->vb.vb2_buf, 0);
	size_t used = offsetof(struct exynos_ispfe_stats_buffer, awb);
	unsigned int grid;

	out->version = EXYNOS_ISPFE_STATS_VERSION_V1;
	out->stats_type = 0;
	out->frame_sequence = area ? area->sequence : 0;
	out->reserved = 0;

	for (grid = 0; area && grid < ISPFE_STATS_GRIDS; grid++) {
		const struct ispfe_stats_grid *desc = &ispfe_stats_grids[grid];

		/*
		 * Nothing here has been read since the hardware wrote it, so
		 * every cached line of it is stale -- including the header the
		 * test below reads.  Invalidating is what makes the copy read
		 * the frame rather than the last one, and it is why the copy
		 * can be a cached one at all: uncached, the same 288 KiB took
		 * 4.8 ms a grid.
		 *
		 * Do not conclude from a test that this line is optional.
		 * Removing it and running `camera-ispfe-stats --check` over
		 * 200 buffers **passed**: five areas of a megabyte each
		 * cycle 5 MiB through more cache than the core has, so by
		 * the time an area comes round again its lines have usually
		 * been evicted and the read reaches DRAM by accident.  It
		 * caught a torn grid once in 220.  The argument for this line
		 * is the ownership
		 * one -- the hardware wrote this memory and the CPU has no way
		 * to know -- and not a measurement.
		 */
		dma_sync_sgtable_for_cpu(ispfe->dev, area->sgt[grid],
					 DMA_BIDIRECTIONAL);

		/*
		 * Copied first and tested afterwards, so that the test is of
		 * the bytes the caller will read rather than of the bytes that
		 * were in the area a moment earlier.  The two are the same
		 * memory and the second load is not the first: the hardware
		 * owns the source and a predicate that read it directly would
		 * be approving one value and publishing another.
		 *
		 * That matters because a predicate does not only say yes or
		 * no.  The flicker result's says how many sums are in it, and
		 * a consumer indexes the array with the count it is handed --
		 * so approving a count of 3120 and then copying out a torn
		 * 0xffffffff would send it past the end of an 18 KiB buffer
		 * with the flag set to say it may.
		 *
		 * The cost is a copy for a grid that turns out not to hold a
		 * result, which happens only when the hardware wrote nothing
		 * at all -- and a buffer with no frame behind it never reaches
		 * this loop.
		 */
		memcpy((u8 *)out + desc->offset, area->grid[grid], desc->size);

		if (!desc->written((u8 *)out + desc->offset))
			continue;

		out->stats_type |= desc->flag;
		used = max(used, desc->offset + desc->size);
	}

	/*
	 * V4L2's `sequence` is the frame number, so that is what this carries:
	 * a consumer pairing a frame with the sensor settings it was taken at
	 * has nothing else to go on, and it cannot read the payload -- on a
	 * split pipeline the half that owns the sensor is not the half that
	 * maps statistics.
	 *
	 * A buffer with no frame behind it repeats the last frame's number
	 * rather than inventing one.  That keeps the sequence non-decreasing,
	 * which is the one thing a queue may not break, and it costs nothing to
	 * read: such a buffer is published only when no statistics are coming at
	 * all, never during a capture, and it says so itself with no
	 * measurement flags set and @frame_sequence zero.
	 */
	if (area)
		ispfe->stats_sequence = area->sequence;

	buf->vb.vb2_buf.timestamp = area ? area->timestamp : ktime_get_ns();
	buf->vb.sequence = ispfe->stats_sequence;
	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, used);
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

/*
 * Whether a frame that could carry statistics for this node is coming.  Not
 * simply "the front end is streaming": the debugfs diagnostic arms the
 * receiver with no queue behind it, so it never pairs a slot with an area and
 * a buffer waiting for one of its frames would wait for ever.
 */
static bool ispfe_stats_producing(const struct ispfe_device *ispfe)
{
	enum ispfe_owner owner = READ_ONCE(ispfe->owner);

	return READ_ONCE(ispfe->streaming) &&
	       (owner == ISPFE_OWNER_V4L2 || owner == ISPFE_OWNER_BACKEND);
}

/*
 * From a work item rather than from the interrupt that retires the frame,
 * because a grid is a quarter of a megabyte and copying it is not something a
 * hard interrupt handler should be doing -- and rather than from QBUF, because
 * a buffer that comes back DONE inside the call that queued it is not what a
 * caller expects.
 *
 * On system_dfl_long_wq rather than the per-CPU one, and it still belongs
 * there now that the copy is cheap.  It was 9.5 ms while the grids were
 * uncached -- 576 KiB at 62 MB/s, measured -- and it never sleeps, so a
 * per-CPU pool could run nothing else behind it for all of that.  What was
 * behind it is ispfe_backend_fill_work(), which has a frame deadline of about
 * 1.3 ms, and that is the whole of why the front end dropped one credit in
 * three whenever this node was streaming.  Cacheable grids took the copy to a
 * measured 0.079 ms mean, so the old margin is back several times over; what
 * has not changed is that this is a bulk copy with no deadline of its own,
 * and an unbound pool is where one belongs.
 *
 * All six sites that queue this item name that one workqueue, and they have to:
 * a work_struct split across two of them can run on two CPUs at once, because
 * the non-reentrancy check compares the pool's workqueue against the queueing
 * one.  ispfe_stats_publish() updates @stats_sequence outside @slock and is
 * the one field here that relies on there being a single instance.
 */
static void ispfe_stats_work_fn(struct work_struct *work)
{
	struct ispfe_device *ispfe =
		container_of(work, struct ispfe_device, stats_work);

	for (;;) {
		struct ispfe_stats_buffer *buf = NULL;
		struct ispfe_stats_area *area = NULL;

		scoped_guard(spinlock_irqsave, &ispfe->slock) {
			if (!ispfe->stats_streaming)
				return;
			area = list_first_entry_or_null(&ispfe->stats_captured,
							struct ispfe_stats_area,
							list);
			if (area) {
				list_del_init(&area->list);
			} else if (ispfe_stats_producing(ispfe)) {
				/*
				 * Nothing metered yet: wait for the frame that
				 * will schedule this again.  Only when there is
				 * no frame coming at all is an empty buffer the
				 * right answer.
				 */
				return;
			}
			buf = list_first_entry_or_null(&ispfe->stats_pending,
						       struct ispfe_stats_buffer,
						       list);
			if (buf)
				list_del(&buf->list);
			else if (!area)
				return;
		}

		/*
		 * A frame with nobody to give it to is dropped rather than
		 * queued: statistics describe the frame that has just gone
		 * past, and an old one is worse than none.
		 */
		if (buf)
			ispfe_stats_publish(ispfe, buf, area);

		guard(spinlock_irqsave)(&ispfe->slock);
		if (buf && area)
			ispfe->stats_published++;
		else if (buf)
			ispfe->stats_empty++;
		else
			ispfe->stats_dropped++;
		if (area)
			ispfe_stats_area_put_locked(ispfe, area);
	}
}

static void ispfe_stats_return_all(struct ispfe_device *ispfe,
				   enum vb2_buffer_state state)
{
	struct ispfe_stats_buffer *buf;
	struct ispfe_stats_buffer *tmp;

	guard(spinlock_irqsave)(&ispfe->slock);
	list_for_each_entry_safe(buf, tmp, &ispfe->stats_pending, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
}

static int ispfe_stats_queue_setup(struct vb2_queue *q, unsigned int *nbufs,
				   unsigned int *nplanes, unsigned int sizes[],
				   struct device *alloc_devs[])
{
	if (*nplanes) {
		if (*nplanes != 1 ||
		    sizes[0] < sizeof(struct exynos_ispfe_stats_buffer))
			return -EINVAL;
		return 0;
	}

	*nplanes = 1;
	sizes[0] = sizeof(struct exynos_ispfe_stats_buffer);

	return 0;
}

static int ispfe_stats_buf_prepare(struct vb2_buffer *vb)
{
	if (vb2_plane_size(vb, 0) < sizeof(struct exynos_ispfe_stats_buffer))
		return -EINVAL;

	return 0;
}

static void ispfe_stats_buf_queue(struct vb2_buffer *vb)
{
	struct ispfe_device *ispfe = vb2_get_drv_priv(vb->vb2_queue);
	struct ispfe_stats_buffer *buf =
		to_ispfe_stats_buffer(to_vb2_v4l2_buffer(vb));

	scoped_guard(spinlock_irqsave, &ispfe->slock)
		list_add_tail(&buf->list, &ispfe->stats_pending);

	/*
	 * Unconditionally, and the work decides: whether a frame is coming is
	 * the front end's business and it can change under this call, so the
	 * one place that answers it is the one place that acts on the answer.
	 */
	queue_work(system_dfl_long_wq, &ispfe->stats_work);
}

/*
 * The front end may already be streaming, in which case the slots it has
 * armed are writing into the shared allocations and only pick up an area of
 * their own as they are encoded afresh -- which happens once per frame.  So
 * statistics begin within a few frames of STREAMON rather than at it, and
 * nothing has to be re-encoded under a running stream to arrange it.
 */
static int ispfe_stats_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct ispfe_device *ispfe = vb2_get_drv_priv(q);

	scoped_guard(spinlock_irqsave, &ispfe->slock) {
		/*
		 * Anything a previous session metered and did not get to hand
		 * out is quiescent -- its frame retired to put it there -- so
		 * this is where it goes back, rather than being published to
		 * whoever is streaming now.
		 */
		list_splice_tail_init(&ispfe->stats_captured,
				      &ispfe->stats_free);
		ispfe->stats_streaming = true;
		ispfe->stats_session++;
		ispfe->stats_sequence = 0;
	}
	queue_work(system_dfl_long_wq, &ispfe->stats_work);

	return 0;
}

static void ispfe_stats_stop_streaming(struct vb2_queue *q)
{
	struct ispfe_device *ispfe = vb2_get_drv_priv(q);

	/*
	 * Clear the flag first, then wait for the work: it takes a buffer off
	 * the pending list and completes it later, so returning buffers while
	 * it ran could leave one active past this function, which is the one
	 * thing vb2 forbids here.  A frame retiring after the cancel can
	 * schedule it again, and it does nothing because the flag is clear.
	 */
	scoped_guard(spinlock_irqsave, &ispfe->slock) {
		ispfe->stats_streaming = false;
		/*
		 * The armed areas are deliberately *not* returned here.  The
		 * front end is a different queue and keeps streaming, and
		 * every slot encoded before this still carries its area's
		 * address in the program the hardware is running -- so putting
		 * one back in the pool would let the next session hand it to a
		 * second slot, and a grid would come back with its header from
		 * one frame and its body from another.  Each is reclaimed
		 * where it becomes quiescent instead, when its frame retires
		 * in ispfe_stats_capture_locked().
		 *
		 * The captured ones have already retired, so they can go.
		 */
		list_splice_tail_init(&ispfe->stats_captured,
				      &ispfe->stats_free);
	}
	cancel_work_sync(&ispfe->stats_work);
	ispfe_stats_return_all(ispfe, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops ispfe_stats_vb2_ops = {
	.queue_setup = ispfe_stats_queue_setup,
	.buf_prepare = ispfe_stats_buf_prepare,
	.buf_queue = ispfe_stats_buf_queue,
	.start_streaming = ispfe_stats_start_streaming,
	.stop_streaming = ispfe_stats_stop_streaming,
};

static int ispfe_stats_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	strscpy(cap->driver, "exynos-ispfe", sizeof(cap->driver));
	strscpy(cap->card, "zumapro ISPFE statistics", sizeof(cap->card));

	return 0;
}

static int ispfe_stats_g_fmt(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	memset(&f->fmt.meta, 0, sizeof(f->fmt.meta));
	f->fmt.meta.dataformat = V4L2_META_FMT_ISPFE_STATS;
	f->fmt.meta.buffersize = sizeof(struct exynos_ispfe_stats_buffer);

	return 0;
}

static int ispfe_stats_enum_fmt(struct file *file, void *priv,
				struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_META_FMT_ISPFE_STATS;

	return 0;
}

static const struct v4l2_ioctl_ops ispfe_stats_ioctl_ops = {
	.vidioc_querycap = ispfe_stats_querycap,
	.vidioc_enum_fmt_meta_cap = ispfe_stats_enum_fmt,
	.vidioc_g_fmt_meta_cap = ispfe_stats_g_fmt,
	.vidioc_s_fmt_meta_cap = ispfe_stats_g_fmt,
	.vidioc_try_fmt_meta_cap = ispfe_stats_g_fmt,
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

static const struct video_device ispfe_stats_template = {
	.name = "exynos-ispfe statistics",
	.fops = &ispfe_fops,
	.ioctl_ops = &ispfe_stats_ioctl_ops,
	.release = video_device_release_empty,
	.device_caps = V4L2_CAP_META_CAPTURE | V4L2_CAP_STREAMING,
	.vfl_dir = VFL_DIR_RX,
};

/* ---- the parameters node ------------------------------------------------ */

/*
 * The front end's tuning is a captured vendor program, and this is the first
 * piece of it that userspace states instead: LMP's white balance gains.
 *
 * They are the piece that has to move, because white balance is an estimate of
 * the illuminant and nothing else in the system can make it.  They are also
 * the cheapest, because the driver already states them rather than replaying
 * them -- ispfe_pdma_apply_wbg() has written them into the program from
 * ispfe->active_lmp_wbg since the encoder existed, and all this adds is a way
 * for that value to come from outside.
 *
 * A buffer is taken by the next program encode, and the front end encodes one
 * program per frame, so a gain set here reaches the frame after next.  That is
 * the same latency the sensor's own controls have and for the same reason.
 */

struct ispfe_params_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	/*
	 * A driver-owned copy of the buffer, because the plane is userspace's
	 * memory and it can rewrite it between the validator's read and the
	 * walk's -- and between the walk's and any later one.  A size rewritten
	 * to zero after validation makes the walk's `offset += header->size`
	 * stand still, which is an unkillable loop inside QBUF holding the
	 * queue's own lock.  Validate the copy, walk the copy.
	 */
	struct v4l2_isp_params_buffer *config;
	/* What the walk resolved, which is what an encode will program. */
	struct ispfe_lmp_wbg_profile wbg;
	struct ispfe_lmp_metering_profile metering;
	/*
	 * This buffer's shading table, already in the hardware's tiled layout,
	 * because tiling is the expensive half and QBUF is the place to pay
	 * for it.  Consuming the buffer swaps this pointer with the device's
	 * live one rather than copying seven kilobytes under a spinlock, so
	 * after a consume this holds whatever table the device had before.
	 */
	u8 *shading;
	bool has_wbg;
	bool has_metering;
	bool has_shading;
	bool restore_default;
	bool restore_metering_default;
	bool restore_shading_default;
};

static struct ispfe_params_buffer *
to_ispfe_params_buffer(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct ispfe_params_buffer, vb);
}

static const struct v4l2_isp_params_block_type_info
ispfe_params_block_info[] = {
	[EXYNOS_ISPFE_PARAM_BLOCK_WHITE_BALANCE] = {
		.size = sizeof(struct exynos_ispfe_params_white_balance),
	},
	[EXYNOS_ISPFE_PARAM_BLOCK_METERING] = {
		.size = sizeof(struct exynos_ispfe_params_metering),
	},
	[EXYNOS_ISPFE_PARAM_BLOCK_LENS_SHADING] = {
		.size = sizeof(struct exynos_ispfe_params_lens_shading),
	},
};

static_assert(ARRAY_SIZE(ispfe_params_block_info) ==
	      EXYNOS_ISPFE_PARAM_BLOCK_SENTINEL);

#define ISPFE_PARAMS_BUFFER_SIZE \
	v4l2_isp_params_buffer_size(EXYNOS_ISPFE_PARAMS_MAX_SIZE)

/*
 * A gain of zero is a channel switched off rather than balanced, and the upper
 * bound is defect pixel correction's: it takes the red and blue gains rounded
 * to Q7 into a ten-bit field, so a larger one cannot be described to it and
 * ispfe_pdma_apply_dpc() would refuse the frame.  Refusing the buffer instead
 * says which one was wrong.
 */
static int ispfe_params_check_wb(struct device *dev,
				 const struct exynos_ispfe_params_white_balance *wb)
{
	unsigned int i;

	for (i = 0; i < EXYNOS_ISPFE_WB_GAINS; i++) {
		if (wb->gains[i] < 1 ||
		    wb->gains[i] > EXYNOS_ISPFE_WB_GAIN_MAX) {
			dev_dbg(dev,
				"white balance gain %u is %u, outside 1..%u\n",
				i, wb->gains[i], EXYNOS_ISPFE_WB_GAIN_MAX);
			return -ERANGE;
		}
	}

	return 0;
}

/*
 * A threshold outside the sample's own domain cannot describe a sample, and a
 * dark threshold above its own saturation threshold excludes every sample in
 * the frame -- which is a metering configuration that can only produce empty
 * grids, so it is refused here rather than reported as three zero counts a
 * frame later.  The luma window is unsigned and its own width, because it
 * gates a weighted sum of four samples rather than one.
 */
static int ispfe_params_check_metering(struct device *dev,
				       const struct exynos_ispfe_params_metering *m)
{
	unsigned int i;

	if (m->reserved) {
		dev_dbg(dev, "metering reserved word is %u, not zero\n",
			m->reserved);
		return -EINVAL;
	}
	/*
	 * Nine bits, which is the field and not a policy: the vendor's own
	 * accessor reads each coefficient as `& 0x1ff` and its encoder writes
	 * them under the same mask.  A tenth bit would be dropped silently and
	 * a weight of 512 would reach the hardware as zero, which turns the
	 * luma into a constant and changes what the grid means with nothing
	 * anywhere saying so.
	 */
	for (i = 0; i < EXYNOS_ISPFE_WB_GAINS; i++)
		if (m->awb_luma_coeff[i] > EXYNOS_ISPFE_METERING_LUMA_COEFF_MAX) {
			dev_dbg(dev, "metering luma weight %u is %u, above %u\n",
				i, m->awb_luma_coeff[i],
				EXYNOS_ISPFE_METERING_LUMA_COEFF_MAX);
			return -ERANGE;
		}
	if (m->awb_luma_threshold_low > m->awb_luma_threshold_high) {
		dev_dbg(dev, "metering luma window %u..%u is inverted\n",
			m->awb_luma_threshold_low, m->awb_luma_threshold_high);
		return -ERANGE;
	}
	if (m->awb_dark_threshold > m->awb_saturation_threshold ||
	    m->ae_dark_threshold > m->ae_saturation_threshold) {
		dev_dbg(dev,
			"metering dark threshold above saturation: awb %d/%d, ae %d/%d\n",
			m->awb_dark_threshold, m->awb_saturation_threshold,
			m->ae_dark_threshold, m->ae_saturation_threshold);
		return -ERANGE;
	}

	return 0;
}

/*
 * A gain of zero is a colour switched off rather than corrected, which no
 * shading table wants; the upper bound is the field's own width.  The grid is
 * not required to be smooth, monotonic or centred on unity: those are
 * properties of a lens, and a consumer correcting something else -- a filter,
 * a cover glass -- is using the block rather than misusing it.
 */
static int
ispfe_params_check_lens_shading(struct device *dev,
				const struct exynos_ispfe_params_lens_shading *lsc)
{
	unsigned int row, column, channel;

	for (row = 0; row < EXYNOS_ISPFE_LSC_ROWS; row++)
		for (column = 0; column < EXYNOS_ISPFE_LSC_COLUMNS; column++)
			for (channel = 0; channel < EXYNOS_ISPFE_WB_GAINS;
			     channel++)
				if (!lsc->gains[row][column][channel]) {
					dev_dbg(dev,
						"shading gain at row %u column %u channel %u is zero\n",
						row, column, channel);
					return -ERANGE;
				}

	return 0;
}

static int ispfe_params_walk(struct ispfe_device *ispfe,
			     const struct v4l2_isp_params_buffer *config,
			     struct ispfe_params_buffer *buf)
{
	size_t offset = 0;

	while (offset < config->data_size) {
		const struct v4l2_isp_params_block_header *header =
			(const struct v4l2_isp_params_block_header *)
			(config->data + offset);

		switch (header->type) {
		case EXYNOS_ISPFE_PARAM_BLOCK_WHITE_BALANCE: {
			const struct exynos_ispfe_params_white_balance *wb =
				(const struct exynos_ispfe_params_white_balance *)header;
			int ret;

			/*
			 * A disabled block asks for the driver's default back
			 * rather than for white balance to stop, so it carries
			 * no values and none are checked.  Which default that
			 * is depends on the program a stream selects, and no
			 * stream need have started when this runs -- ispfe->prog
			 * is NULL until the first one -- so the block is
			 * recorded as a request and resolved at the encode,
			 * where a program exists by construction.
			 */
			if (header->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
				buf->restore_default = true;
				break;
			}

			ret = ispfe_params_check_wb(ispfe->dev, wb);
			if (ret)
				return ret;

			buf->wbg.red = wb->gains[EXYNOS_ISPFE_WB_RED];
			buf->wbg.green_red = wb->gains[EXYNOS_ISPFE_WB_GREEN_RED];
			buf->wbg.green_blue = wb->gains[EXYNOS_ISPFE_WB_GREEN_BLUE];
			buf->wbg.blue = wb->gains[EXYNOS_ISPFE_WB_BLUE];
			buf->has_wbg = true;
			break;
		}
		case EXYNOS_ISPFE_PARAM_BLOCK_METERING: {
			const struct exynos_ispfe_params_metering *m =
				(const struct exynos_ispfe_params_metering *)header;
			unsigned int i;
			int ret;

			/*
			 * As for white balance: a disabled block asks for the
			 * driver's default back rather than for metering to
			 * stop, so it carries no values and none are checked.
			 * Unlike white balance the default needs no program to
			 * resolve it -- every captured recipe meters through
			 * the same thresholds -- but it is still recorded and
			 * applied at the encode, so that the two blocks behave
			 * the same way from outside.
			 */
			if (header->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
				buf->restore_metering_default = true;
				break;
			}

			ret = ispfe_params_check_metering(ispfe->dev, m);
			if (ret)
				return ret;

			buf->metering.awb_saturation = m->awb_saturation_threshold;
			buf->metering.awb_dark = m->awb_dark_threshold;
			for (i = 0; i < EXYNOS_ISPFE_WB_GAINS; i++)
				buf->metering.awb_luma_coeff[i] =
					m->awb_luma_coeff[i];
			buf->metering.awb_luma_low = m->awb_luma_threshold_low;
			buf->metering.awb_luma_high = m->awb_luma_threshold_high;
			buf->metering.awb_diff_coring =
				m->awb_diff_coring_threshold;
			buf->metering.ae_saturation = m->ae_saturation_threshold;
			buf->metering.ae_dark = m->ae_dark_threshold;
			buf->has_metering = true;
			break;
		}
		case EXYNOS_ISPFE_PARAM_BLOCK_LENS_SHADING: {
			const struct exynos_ispfe_params_lens_shading *lsc =
				(const struct exynos_ispfe_params_lens_shading *)header;
			int ret;

			/*
			 * As for the two above: a disabled block asks for the
			 * driver's default back rather than for shading
			 * correction to stop.  Which default that is depends on
			 * the program a stream selects -- the table is a
			 * calibration of one lens at one readout -- so it is
			 * recorded and resolved at the encode.
			 */
			if (header->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
				buf->restore_shading_default = true;
				break;
			}

			ret = ispfe_params_check_lens_shading(ispfe->dev, lsc);
			if (ret)
				return ret;

			/*
			 * Tiled here rather than at the encode, because this
			 * runs once per buffer in process context where the
			 * encode runs once per frame.
			 */
			ispfe_lsc_tile(buf->shading, &lsc->gains[0][0][0]);
			buf->has_shading = true;
			break;
		}
		default:
			return -EINVAL;
		}

		offset += header->size;
	}

	return 0;
}

/*
 * Take the earliest buffer waiting and make it the live state, if one is
 * waiting.  Called from the encode paths, which run once per frame.  @wbg, if
 * given, receives the gains the encode that follows will program -- read under
 * the same lock hold that may have just moved them, so that what a caller
 * reports about a frame is what that frame's program actually carries.
 *
 * The buffer is taken *and completed* inside one hold of ispfe->slock, gated
 * on params_streaming, and that is the whole of the correctness argument:
 * completing it after dropping the lock lets stop_streaming() run in between,
 * find the list already empty and return, leaving a buffer active past it --
 * which is the one thing vb2 forbids there.  vb2_buffer_done() takes only the
 * queue's own done_lock and is IRQ-safe, so there is no inversion to avoid;
 * ispfe_stats_return_all() completes under this same lock.
 */
static void ispfe_params_consume(struct ispfe_device *ispfe,
				 struct ispfe_lmp_wbg_profile *wbg,
				 const u8 **shading)
{
	struct ispfe_params_buffer *buf;

	guard(spinlock_irqsave)(&ispfe->slock);

	buf = ispfe->params_streaming ?
		list_first_entry_or_null(&ispfe->params_pending,
					 struct ispfe_params_buffer, list) :
		NULL;
	if (buf) {
		list_del(&buf->list);

		/*
		 * Resolved here rather than at buf_prepare, because which
		 * default applies is the running program's and no program need
		 * have existed then.  A program without white balance at all
		 * -- the raw recipes -- has no default to restore and the
		 * request is dropped, which is the same nothing the gains
		 * themselves would do there.
		 */
		if (buf->restore_default && ispfe->prog && ispfe->prog->lmp_wbg)
			ispfe->active_lmp_wbg = *ispfe->prog->lmp_wbg;
		else if (buf->has_wbg)
			ispfe->active_lmp_wbg = buf->wbg;

		if (buf->restore_metering_default)
			ispfe->active_lmp_metering = ispfe_lmp_metering_captured;
		else if (buf->has_metering)
			ispfe->active_lmp_metering = buf->metering;

		/*
		 * Swapped rather than copied: seven kilobytes under this lock
		 * would be the largest thing it ever holds, and after the swap
		 * the buffer owns the table the device had.  So every
		 * allocation still has exactly one owner, and the one the
		 * device is about to encode from is not in a buffer userspace
		 * gets back on the next line.  Restoring the default writes
		 * into the device's own table instead, which is a copy the
		 * same size but only when a stream asks for it.
		 */
		if (buf->restore_shading_default && ispfe->lsc_input >= 0 &&
		    ispfe->prog)
			memcpy(ispfe->shading,
			       ispfe_lsc_default(ispfe, ispfe->prog),
			       ISPFE_LSC_LUT_BYTES);
		else if (buf->has_shading)
			swap(ispfe->shading, buf->shading);

		buf->vb.vb2_buf.timestamp = ktime_get_ns();
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	}

	if (wbg)
		*wbg = ispfe->active_lmp_wbg;
	if (shading)
		*shading = ispfe->shading;
}

static void ispfe_params_return_all(struct ispfe_device *ispfe,
				    enum vb2_buffer_state state)
{
	struct ispfe_params_buffer *buf;
	struct ispfe_params_buffer *tmp;

	guard(spinlock_irqsave)(&ispfe->slock);

	list_for_each_entry_safe(buf, tmp, &ispfe->params_pending, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
}

static int ispfe_params_buf_init(struct vb2_buffer *vb)
{
	struct ispfe_params_buffer *buf =
		to_ispfe_params_buffer(to_vb2_v4l2_buffer(vb));

	buf->config = kvmalloc(ISPFE_PARAMS_BUFFER_SIZE, GFP_KERNEL);
	buf->shading = kzalloc(ISPFE_LSC_LUT_BYTES, GFP_KERNEL);
	if (!buf->config || !buf->shading) {
		kvfree(buf->config);
		buf->config = NULL;
		kfree(buf->shading);
		buf->shading = NULL;
		return -ENOMEM;
	}

	return 0;
}

static void ispfe_params_buf_cleanup(struct vb2_buffer *vb)
{
	struct ispfe_params_buffer *buf =
		to_ispfe_params_buffer(to_vb2_v4l2_buffer(vb));

	kvfree(buf->config);
	buf->config = NULL;
	kfree(buf->shading);
	buf->shading = NULL;
}

static int ispfe_params_queue_setup(struct vb2_queue *q, unsigned int *nbufs,
				    unsigned int *nplanes, unsigned int sizes[],
				    struct device *alloc_devs[])
{
	if (*nplanes) {
		if (*nplanes != 1 || sizes[0] < ISPFE_PARAMS_BUFFER_SIZE)
			return -EINVAL;
		return 0;
	}

	*nplanes = 1;
	sizes[0] = ISPFE_PARAMS_BUFFER_SIZE;

	return 0;
}

static int ispfe_params_buf_prepare(struct vb2_buffer *vb)
{
	struct ispfe_device *ispfe = vb2_get_drv_priv(vb->vb2_queue);
	struct ispfe_params_buffer *buf =
		to_ispfe_params_buffer(to_vb2_v4l2_buffer(vb));
	const struct v4l2_isp_params_buffer *config = buf->config;
	int ret;

	ret = v4l2_isp_params_validate_buffer_size(ispfe->dev, vb,
						   ISPFE_PARAMS_BUFFER_SIZE);
	if (ret)
		return ret;

	/* Validate what will be programmed, not what may change underneath. */
	memcpy(buf->config, vb2_plane_vaddr(vb, 0), ISPFE_PARAMS_BUFFER_SIZE);

	/*
	 * Every per-buffer verdict, because buffers are reused for the life of
	 * a REQBUFS: a flag left set makes a buffer that once carried a block
	 * carry it for ever, and for the shading table that is worse than
	 * stale -- the consume *swaps* it, so a buffer re-queued without one
	 * would trade the live table for its own on every frame and the
	 * correction would alternate between two grids.
	 */
	buf->has_wbg = false;
	buf->restore_default = false;
	buf->has_metering = false;
	buf->restore_metering_default = false;
	buf->has_shading = false;
	buf->restore_shading_default = false;

	/*
	 * A buffer carrying no blocks changes nothing and has nothing in it to
	 * check, which is what an all-zero buffer is -- what a queue's own
	 * buffers hold before anyone fills them, and what v4l2-compliance
	 * queues.
	 */
	if (config->version != V4L2_ISP_PARAMS_VERSION_V0 &&
	    config->version != V4L2_ISP_PARAMS_VERSION_V1)
		return -EINVAL;
	if (!config->data_size)
		return 0;

	ret = v4l2_isp_params_validate_buffer(ispfe->dev, vb, config,
					      ispfe_params_block_info,
					      ARRAY_SIZE(ispfe_params_block_info));
	if (ret)
		return ret;

	/*
	 * Nothing is programmed here and the values are copied out rather than
	 * read later, because the buffer is userspace's memory and can change
	 * under the encode that would otherwise read it.
	 */
	return ispfe_params_walk(ispfe, config, buf);
}

static void ispfe_params_buf_queue(struct vb2_buffer *vb)
{
	struct ispfe_device *ispfe = vb2_get_drv_priv(vb->vb2_queue);
	struct ispfe_params_buffer *buf =
		to_ispfe_params_buffer(to_vb2_v4l2_buffer(vb));

	scoped_guard(spinlock_irqsave, &ispfe->slock)
		list_add_tail(&buf->list, &ispfe->params_pending);
}

static int ispfe_params_start_streaming(struct vb2_queue *q,
					unsigned int count)
{
	struct ispfe_device *ispfe = vb2_get_drv_priv(q);

	scoped_guard(spinlock_irqsave, &ispfe->slock)
		ispfe->params_streaming = true;

	return 0;
}

static void ispfe_params_stop_streaming(struct vb2_queue *q)
{
	struct ispfe_device *ispfe = vb2_get_drv_priv(q);

	scoped_guard(spinlock_irqsave, &ispfe->slock)
		ispfe->params_streaming = false;

	ispfe_params_return_all(ispfe, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops ispfe_params_vb2_ops = {
	.queue_setup = ispfe_params_queue_setup,
	.buf_init = ispfe_params_buf_init,
	.buf_cleanup = ispfe_params_buf_cleanup,
	.buf_prepare = ispfe_params_buf_prepare,
	.buf_queue = ispfe_params_buf_queue,
	.start_streaming = ispfe_params_start_streaming,
	.stop_streaming = ispfe_params_stop_streaming,
};

static int ispfe_params_querycap(struct file *file, void *priv,
				 struct v4l2_capability *cap)
{
	strscpy(cap->driver, "exynos-ispfe", sizeof(cap->driver));
	strscpy(cap->card, "zumapro ISPFE parameters", sizeof(cap->card));

	return 0;
}

static int ispfe_params_g_fmt(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	memset(&f->fmt.meta, 0, sizeof(f->fmt.meta));
	f->fmt.meta.dataformat = V4L2_META_FMT_ISPFE_PARAMS;
	f->fmt.meta.buffersize = ISPFE_PARAMS_BUFFER_SIZE;

	return 0;
}

static int ispfe_params_enum_fmt(struct file *file, void *priv,
				 struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_META_FMT_ISPFE_PARAMS;

	return 0;
}

static const struct v4l2_ioctl_ops ispfe_params_ioctl_ops = {
	.vidioc_querycap = ispfe_params_querycap,
	.vidioc_enum_fmt_meta_out = ispfe_params_enum_fmt,
	.vidioc_g_fmt_meta_out = ispfe_params_g_fmt,
	.vidioc_s_fmt_meta_out = ispfe_params_g_fmt,
	.vidioc_try_fmt_meta_out = ispfe_params_g_fmt,
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

static const struct video_device ispfe_params_template = {
	.name = "exynos-ispfe parameters",
	.fops = &ispfe_fops,
	.ioctl_ops = &ispfe_params_ioctl_ops,
	.release = video_device_release_empty,
	.device_caps = V4L2_CAP_META_OUTPUT | V4L2_CAP_STREAMING,
	.vfl_dir = VFL_DIR_TX,
};

/* No link into the graph, for the reason the statistics node has none. */
static int ispfe_params_register(struct ispfe_device *ispfe)
{
	struct vb2_queue *q = &ispfe->params_queue;
	int ret;

	INIT_LIST_HEAD(&ispfe->params_pending);

	q->type = V4L2_BUF_TYPE_META_OUTPUT;
	q->io_modes = VB2_MMAP;
	q->drv_priv = ispfe;
	q->ops = &ispfe_params_vb2_ops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->buf_struct_size = sizeof(struct ispfe_params_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &ispfe->params_lock;
	q->dev = ispfe->dev;
	q->min_queued_buffers = 0;

	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	ispfe->params_vdev = ispfe_params_template;
	ispfe->params_vdev.v4l2_dev = &ispfe->v4l2_dev;
	ispfe->params_vdev.queue = q;
	ispfe->params_vdev.lock = &ispfe->params_lock;
	ispfe->params_vdev.entity.function = MEDIA_ENT_F_IO_V4L;
	video_set_drvdata(&ispfe->params_vdev, ispfe);

	ispfe->params_pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&ispfe->params_vdev.entity, 1,
				     &ispfe->params_pad);
	if (ret)
		return ret;

	ret = video_register_device(&ispfe->params_vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		media_entity_cleanup(&ispfe->params_vdev.entity);
		return ret;
	}

	return 0;
}

/*
 * No link into the graph.  The pad the capture node hangs off carries Bayer to
 * that node, and a second enabled link from it would put this node into the
 * pipeline every capture starts -- which it is not part of.  What produces
 * statistics is LMP, which the graph does not model as an entity, so the
 * honest description is an entity with a pad and nothing claiming to feed it.
 */
static int ispfe_stats_register(struct ispfe_device *ispfe)
{
	struct vb2_queue *q = &ispfe->stats_queue;
	int ret;

	q->type = V4L2_BUF_TYPE_META_CAPTURE;
	q->io_modes = VB2_MMAP;
	q->dev = ispfe->dev;
	q->drv_priv = ispfe;
	q->ops = &ispfe_stats_vb2_ops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->buf_struct_size = sizeof(struct ispfe_stats_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &ispfe->stats_lock;
	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	ispfe->stats_vdev = ispfe_stats_template;
	ispfe->stats_vdev.v4l2_dev = &ispfe->v4l2_dev;
	ispfe->stats_vdev.queue = q;
	ispfe->stats_vdev.lock = &ispfe->stats_lock;
	ispfe->stats_vdev.entity.function = MEDIA_ENT_F_IO_V4L;
	video_set_drvdata(&ispfe->stats_vdev, ispfe);

	ispfe->stats_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&ispfe->stats_vdev.entity, 1,
				     &ispfe->stats_pad);
	if (ret)
		return ret;

	ret = video_register_device(&ispfe->stats_vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		media_entity_cleanup(&ispfe->stats_vdev.entity);

	return ret;
}

/* ---- the receiver subdevice -------------------------------------------- */

static struct ispfe_device *sd_to_ispfe(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ispfe_device, sd);
}

static int ispfe_sd_init_state(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state)
{
	struct ispfe_device *ispfe = sd_to_ispfe(sd);
	struct v4l2_mbus_framefmt *source =
		v4l2_subdev_state_get_format(state, ispfe->source_pad);
	struct v4l2_mbus_framefmt *sink;
	unsigned int pad;

	/* Every camera starts from the same default; each keeps its own. */
	for (pad = 0; pad < ispfe->source_pad; pad++) {
		sink = v4l2_subdev_state_get_format(state, pad);

		sink->code = ISPFE_DEFAULT_CODE;
		sink->width = ISPFE_DEFAULT_WIDTH;
		sink->height = ISPFE_DEFAULT_HEIGHT;
		sink->field = V4L2_FIELD_NONE;
		sink->colorspace = V4L2_COLORSPACE_RAW;
		sink->ycbcr_enc = V4L2_YCBCR_ENC_601;
		sink->quantization = V4L2_QUANTIZATION_FULL_RANGE;
		sink->xfer_func = V4L2_XFER_FUNC_NONE;
	}

	*source = *v4l2_subdev_state_get_format(state,
						ispfe_active_sink_pad(ispfe));

	return 0;
}

static int ispfe_sd_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	struct ispfe_device *ispfe = sd_to_ispfe(sd);

	/* Nothing converts, so the source offers exactly what the sink took. */
	if (code->pad == ispfe->source_pad) {
		if (code->index)
			return -EINVAL;
		code->code = v4l2_subdev_state_get_format(state,
				ispfe_active_sink_pad(ispfe))->code;
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
	struct ispfe_device *ispfe = sd_to_ispfe(sd);

	if (fse->index)
		return -EINVAL;

	if (fse->pad == ispfe->source_pad) {
		const struct v4l2_mbus_framefmt *sink =
			v4l2_subdev_state_get_format(state,
					ispfe_active_sink_pad(ispfe));

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

	if (format->pad == ispfe->source_pad)
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

	sink = v4l2_subdev_state_get_format(state, format->pad);
	*sink = format->format;

	/*
	 * The source carries what the receiver will actually deliver, so it
	 * follows the camera that is selected and not whichever sink was last
	 * written.  Setting a format on a camera that is not the enabled one
	 * is allowed and remembered; it just does not change the output until
	 * that camera is selected.
	 */
	if (format->pad == ispfe_active_sink_pad(ispfe)) {
		source = v4l2_subdev_state_get_format(state,
						      ispfe->source_pad);
		*source = *sink;
	}

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
	struct ispfe_link *link = ispfe_active_link(ispfe);

	if (!link || !link->sensor)
		return -ENODEV;

	return v4l2_subdev_enable_streams(link->sensor, link->sensor_pad,
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
	struct ispfe_link *link = ispfe_active_link(ispfe);
	int ret;

	if (!link || !link->sensor)
		return 0;

	ret = v4l2_subdev_disable_streams(link->sensor, link->sensor_pad,
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

/*
 * One event deep.  A consumer of frame start uses it to place a control write
 * inside the frame it names, so a queue of stale ones is worth less than the
 * knowledge that the newest was dropped -- which is what the sequence number
 * in each event says.
 */
#define ISPFE_FRAME_SYNC_EVENTS		1

static int ispfe_sd_subscribe_event(struct v4l2_subdev *sd,
				    struct v4l2_fh *fh,
				    struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_FRAME_SYNC)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub, ISPFE_FRAME_SYNC_EVENTS, NULL);
}

static const struct v4l2_subdev_core_ops ispfe_subdev_core_ops = {
	.subscribe_event = ispfe_sd_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops ispfe_subdev_ops = {
	.core = &ispfe_subdev_core_ops,
	.video = &ispfe_subdev_video_ops,
	.pad = &ispfe_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops ispfe_subdev_internal_ops = {
	.init_state = ispfe_sd_init_state,
};

/*
 * Which camera feeds the receiver.  One sink link at a time: enabling a second
 * is refused rather than silently switching, so a consumer has to say what it
 * is giving up before it says what it wants -- and that refusal is this
 * driver's, not the core's.  The core only blocks a change to a link whose pad
 * is in a running pipeline, and .has_pad_interdep deliberately keeps the
 * *inactive* sinks out of it, so nothing above would stop the other link being
 * enabled underneath a capture.
 *
 * Which is also why the whole selection is refused while the front end has an
 * owner.  Both V4L2 paths take a media pipeline now -- the raw node from its
 * own capture node, the back end from its subdevice's sink pad -- so the core
 * already refuses a link change under either of them, and this is the second
 * lock on the same door.  The debugfs capture is the one path that still takes
 * none, and it is where the refusal is load-bearing rather than belt and
 * braces.
 *
 * What the pipeline buys beyond the refusal is the *race*.  `owner` is read
 * without ispfe->lock, deliberately: the media core calls this holding its
 * graph mutex, while a stream start takes that mutex inside
 * video_device_pipeline_start(), so taking ispfe->lock here would close an
 * ABBA.  That leaves a link change racing a start that has not yet claimed
 * ownership -- and it is not benign, because ispfe_link_to_src() is six
 * unsynchronised stores and ispfe_start() copies the struct, so an interleave
 * arms the receiver on one camera's bank with another's PHY.  Every field is
 * individually valid, so nothing downstream rejects it; the capture is simply
 * wrong.
 *
 * **Both V4L2 paths close that race the same way now**, by claiming before
 * they walk: the raw node from .prepare_streaming, the back end by reserving
 * its producer from its own .prepare_streaming.  A link change racing either
 * blocks on this graph mutex and then finds an owner -- and even one that got
 * in first completes its six stores before the walk can finish, because it
 * holds the graph mutex for the whole of this function.  Once the walk has
 * run, the *active* camera's link is pinned by the core as well, both of its
 * pads being in the pipeline.  The inactive cameras' links never are, by
 * .has_pad_interdep, so those changes reach here and are refused below --
 * which is why this check cannot be replaced by the pipeline.
 *
 * The claims are also the reason a successful STREAMON on either node means
 * this, rather than meaning it from whenever userspace queues a buffer: vb2
 * defers .start_streaming until min_queued_buffers are queued, and both
 * queues set it.
 *
 * The debugfs capture takes no pipeline at all and stays the same shape as the
 * unlocked debugfs writes this driver already accepts, so this is the whole of
 * its protection rather than a second lock on the same door.
 */
static int ispfe_link_setup(struct media_entity *entity,
			    const struct media_pad *local,
			    const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct ispfe_device *ispfe = sd_to_ispfe(sd);
	struct v4l2_subdev_state *state;
	struct ispfe_link *link;

	/* The graph below the source pad is fixed; only a sink link chooses. */
	if (!(local->flags & MEDIA_PAD_FL_SINK))
		return 0;

	if (READ_ONCE(ispfe->owner) != ISPFE_OWNER_NONE)
		return -EBUSY;

	link = ispfe_link_by_pad(ispfe, local->index);
	if (!link)
		return -EINVAL;

	if (!(flags & MEDIA_LNK_FL_ENABLED)) {
		if (ispfe_active_link(ispfe) == link)
			WRITE_ONCE(ispfe->active_link, NULL);
		return 0;
	}

	if (ispfe_active_link(ispfe) && ispfe_active_link(ispfe) != link)
		return -EBUSY;

	WRITE_ONCE(ispfe->active_link, link);

	/*
	 * Only when the camera actually changes.  `src` is what a debugfs sweep
	 * of `phy` or the mode words writes into, and re-selecting the camera
	 * already selected -- which is what disabling and re-enabling one link
	 * is -- must not throw that away.  Switching to a *different* camera
	 * resets them, because a sweep belongs to the camera it was made on.
	 */
	if (ispfe->src.link != link->bank)
		ispfe_link_to_src(ispfe, link);

	/*
	 * The source pad carries what the receiver will deliver, and it is what
	 * ispfe_start() arms the hardware from and what the capture node sizes
	 * its buffers by -- so selecting a camera has to bring that camera's
	 * geometry with it.  Without this the source would keep the *previous*
	 * camera's, and link validation would not catch it: it compares the
	 * sensor against the sink, and never the sink against the source.
	 */
	state = v4l2_subdev_lock_and_get_active_state(sd);
	if (state) {
		*v4l2_subdev_state_get_format(state, ispfe->source_pad) =
			*v4l2_subdev_state_get_format(state, link->pad);
		v4l2_subdev_unlock_state(state);
	}

	return 0;
}

/*
 * Only the *enabled* sink is connected to the source inside this entity.
 *
 * Without this the default applies, which says every sink is interdependent
 * with every source -- and then the pipeline walk reaches the inactive sink
 * pads too, because it adds the local pad of a link whether or not the link is
 * enabled.  Each of those carries MUST_CONNECT, so a capture on one camera
 * would be refused with -ENOLINK naming the pad of another.
 */
static bool ispfe_has_pad_interdep(struct media_entity *entity,
				   unsigned int pad0, unsigned int pad1)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct ispfe_device *ispfe = sd_to_ispfe(sd);
	struct ispfe_link *link = ispfe_active_link(ispfe);
	unsigned int sink = pad0 == ispfe->source_pad ? pad1 : pad0;

	/* The core has already established that one of the two is the source. */
	return link && sink == link->pad;
}

static const struct media_entity_operations ispfe_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.link_setup = ispfe_link_setup,
	.has_pad_interdep = ispfe_has_pad_interdep,
};

/* ---- binding the sensor ------------------------------------------------ */

/*
 * One async connection per camera, each carrying the link it was added for.
 * The connection's own fwnode is the *sensor's* endpoint, which says nothing
 * about which of the receiver's sink pads the data arrives on, so the mapping
 * has to be recorded when the connection is made.
 */
struct ispfe_asc {
	struct v4l2_async_connection base;
	struct ispfe_link *link;
};

static struct ispfe_asc *to_ispfe_asc(struct v4l2_async_connection *asc)
{
	return container_of(asc, struct ispfe_asc, base);
}

static int ispfe_notify_bound(struct v4l2_async_notifier *nf,
			      struct v4l2_subdev *sd,
			      struct v4l2_async_connection *asc)
{
	struct ispfe_device *ispfe =
		container_of(nf, struct ispfe_device, notifier);
	struct ispfe_link *link = to_ispfe_asc(asc)->link;
	int pad;

	pad = media_entity_get_fwnode_pad(&sd->entity, asc->match.fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (pad < 0) {
		dev_err(ispfe->dev, "%s has no source pad for that endpoint\n",
			sd->name);
		return pad;
	}

	guard(mutex)(&ispfe->lock);
	link->sensor = sd;
	link->sensor_pad = pad;

	return 0;
}

static void ispfe_notify_unbind(struct v4l2_async_notifier *nf,
				struct v4l2_subdev *sd,
				struct v4l2_async_connection *asc)
{
	struct ispfe_device *ispfe =
		container_of(nf, struct ispfe_device, notifier);

	guard(mutex)(&ispfe->lock);
	to_ispfe_asc(asc)->link->sensor = NULL;
}

/*
 * A link per camera, and the enabled one is the selector: this block has twelve
 * link banks and the driver runs one stream, so which camera feeds the receiver
 * is a choice, made by enabling that camera's link and refused anywhere else.
 *
 * The links downstream of the source pad stay immutable, because the graph
 * below the receiver really is fixed.
 *
 * With a single camera there is no choice to make, so its link is immutable
 * too and the graph is as rigid as it was before this could hold two.
 */
static int ispfe_notify_complete(struct v4l2_async_notifier *nf)
{
	struct ispfe_device *ispfe =
		container_of(nf, struct ispfe_device, notifier);
	struct ispfe_link *active = ispfe_active_link(ispfe);
	unsigned int i;
	int ret;

	for (i = 0; i < CSIS_NUM_LINKS; i++) {
		struct ispfe_link *link = &ispfe->links[i];
		u32 flags = 0;

		if (!link->present)
			continue;

		if (link == active)
			flags = MEDIA_LNK_FL_ENABLED;
		if (ispfe->num_links == 1)
			flags |= MEDIA_LNK_FL_IMMUTABLE;

		ret = media_create_pad_link(&link->sensor->entity,
					    link->sensor_pad,
					    &ispfe->sd.entity, link->pad,
					    flags);
		if (ret)
			return ret;
	}

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
 * The device tree says which CSIS link bank a sensor arrives on -- that is the
 * port number -- and what the link looks like.  Everything else about the
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
	struct ispfe_link *link;
	int ret;

	ret = fwnode_graph_parse_endpoint(ep, &fwep);
	if (ret)
		return dev_err_probe(ispfe->dev, ret,
				     "cannot read the endpoint's port\n");

	if (fwep.port >= CSIS_NUM_LINKS || !ispfe_link_cfg[fwep.port].known)
		return dev_err_probe(ispfe->dev, -EINVAL,
				     "no PHY is known for CSIS link %u\n",
				     fwep.port);

	link = &ispfe->links[fwep.port];
	if (link->present)
		return dev_err_probe(ispfe->dev, -EINVAL,
				     "two endpoints on CSIS link %u\n",
				     fwep.port);

	ret = v4l2_fwnode_endpoint_parse(ep, &vep);
	if (ret)
		return dev_err_probe(ispfe->dev, ret,
				     "cannot parse the endpoint\n");

	switch (vep.bus_type) {
	case V4L2_MBUS_CSI2_DPHY:
		link->cphy = false;
		break;
	case V4L2_MBUS_CSI2_CPHY:
		link->cphy = true;
		break;
	default:
		return dev_err_probe(ispfe->dev, -EINVAL,
				     "bus type %u is not CSI-2\n",
				     vep.bus_type);
	}

	cfg = &ispfe_link_cfg[fwep.port];
	link->bank = fwep.port;
	link->phy = cfg->phy;
	link->lanes = vep.bus.mipi_csi2.num_data_lanes;
	link->mode_word0 = cfg->mode_word0;
	link->mode_word1 = cfg->mode_word1;

	if (!link->lanes || link->lanes > ispfe_phy_lanes(link->phy))
		return dev_err_probe(ispfe->dev, -EINVAL,
				     "%u data lanes, PHY %u has %u\n",
				     link->lanes, link->phy,
				     ispfe_phy_lanes(link->phy));

	link->present = true;
	ispfe->num_links++;

	return 0;
}

/*
 * Every endpoint, not just the first.  Which bank a sensor arrives on is its
 * port number, so a board with more than one camera describes them as several
 * ports -- and reading only the first silently ignores the rest, which is not
 * a failure anything downstream can see.
 */
static int ispfe_parse_endpoints(struct ispfe_device *ispfe)
{
	struct fwnode_handle *ep;
	unsigned int i, pad;
	int ret;

	fwnode_graph_for_each_endpoint(dev_fwnode(ispfe->dev), ep) {
		ret = ispfe_parse_endpoint(ispfe, ep);
		if (ret) {
			fwnode_handle_put(ep);
			return ret;
		}
	}

	if (!ispfe->num_links)
		return dev_err_probe(ispfe->dev, -ENXIO,
				     "no sensor endpoint\n");

	/*
	 * A sink pad each, in ascending bank order, so the pad numbering is
	 * stable for a given device tree and the source lands after them.
	 */
	pad = 0;
	for (i = 0; i < CSIS_NUM_LINKS; i++)
		if (ispfe->links[i].present)
			ispfe->links[i].pad = pad++;

	return 0;
}

static int ispfe_media_register(struct ispfe_device *ispfe)
{
	struct vb2_queue *q = &ispfe->queue;
	struct fwnode_handle *ep;
	unsigned int i;
	int ret;

	ret = ispfe_parse_endpoints(ispfe);
	if (ret)
		return ret;

	/* Sink pads first, one per camera, then the source. */
	ispfe->source_pad = ispfe->num_links;

	/*
	 * The lowest-numbered bank is the camera a graph nobody has touched
	 * comes up on, so that a single-camera board needs no link enabling and
	 * a multi-camera one has a defined starting point rather than none.
	 * Every later change of it goes through .link_setup.
	 */
	ispfe->active_link = ispfe_link_by_pad(ispfe, 0);
	ispfe_link_to_src(ispfe, ispfe->active_link);

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
	ispfe->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	ispfe->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	ispfe->sd.entity.ops = &ispfe_subdev_entity_ops;
	/*
	 * Not the bank number any more.  One receiver serves all twelve link
	 * banks, so naming the entity after whichever one a sensor happened to
	 * arrive on was a statement about the board that stopped being true of
	 * the entity the moment it could carry more than one.
	 */
	strscpy(ispfe->sd.name, "exynos-ispfe csis", sizeof(ispfe->sd.name));
	v4l2_set_subdevdata(&ispfe->sd, ispfe);

	/*
	 * MUST_CONNECT, so that a pipeline with no sensor is refused before any
	 * of the receiver is programmed rather than after it is fully armed.
	 */
	for (i = 0; i < ispfe->source_pad; i++)
		ispfe->pads[i].flags = MEDIA_PAD_FL_SINK |
				       MEDIA_PAD_FL_MUST_CONNECT;
	ispfe->pads[ispfe->source_pad].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&ispfe->sd.entity, ispfe->source_pad + 1,
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
	 * A frame lands in the buffer credited a frame earlier, so completing
	 * the first one takes a second credit -- but not a second *buffer*.
	 * Dump credits age what is in flight, so a lone buffer is credited at
	 * one frame start, filled during the next, and retired at that frame's
	 * end against the dump credit in between.
	 */
	q->min_queued_buffers = 1;
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

	ret = media_create_pad_link(&ispfe->sd.entity, ispfe->source_pad,
				    &ispfe->vdev.entity, 0,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		goto err_vdev;

	ret = ispfe_stats_register(ispfe);
	if (ret)
		goto err_vdev;

	ret = ispfe_params_register(ispfe);
	if (ret)
		goto err_stats;

	/*
	 * The whole back end joins this graph -- its input off the same source
	 * pad the raw node hangs from, since the side output and the raw output
	 * are one stream, and its two video nodes on this v4l2_device.  It
	 * needs the mosaic this receiver negotiated and had no way to see it,
	 * and one media device per pipeline is what a consumer expects.
	 */
	ret = exynos_becore_input_register_graph(ispfe->backend_input,
						 &ispfe->v4l2_dev,
						 &ispfe->sd.entity,
						 ispfe->source_pad);
	if (ret)
		goto err_params;

	/*
	 * A connection per camera, each told which link it is for.  The port
	 * number was bounds-checked and marked present by
	 * ispfe_parse_endpoints() above, so indexing links[] with it here is
	 * safe by construction.
	 */
	v4l2_async_nf_init(&ispfe->notifier, &ispfe->v4l2_dev);
	fwnode_graph_for_each_endpoint(dev_fwnode(ispfe->dev), ep) {
		struct fwnode_endpoint fwep = {};
		struct ispfe_asc *asc;

		ret = fwnode_graph_parse_endpoint(ep, &fwep);
		if (ret)
			break;

		asc = v4l2_async_nf_add_fwnode_remote(&ispfe->notifier, ep,
						      struct ispfe_asc);
		if (IS_ERR(asc)) {
			ret = PTR_ERR(asc);
			break;
		}

		asc->link = &ispfe->links[fwep.port];
	}
	if (ret) {
		fwnode_handle_put(ep);
		goto err_nf;
	}

	ispfe->notifier.ops = &ispfe_notifier_ops;
	ret = v4l2_async_nf_register(&ispfe->notifier);
	if (ret)
		goto err_nf;

	return 0;

err_nf:
	v4l2_async_nf_cleanup(&ispfe->notifier);
	/* Sever the callbacks before the node whose teardown can raise one. */
	exynos_becore_input_disconnect(ispfe->backend_input);
	exynos_becore_input_unregister_graph(ispfe->backend_input);
err_params:
	vb2_video_unregister_device(&ispfe->params_vdev);
	media_entity_cleanup(&ispfe->params_vdev.entity);
err_stats:
	/* Releases the queue too, which a bare unregister would not. */
	vb2_video_unregister_device(&ispfe->stats_vdev);
	media_entity_cleanup(&ispfe->stats_vdev.entity);
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
	return ret;
}

static void ispfe_media_unregister(struct ispfe_device *ispfe)
{
	/*
	 * The capture device goes first, because it is what can be driving the
	 * front end and stopping that needs the sensor still bound.  It also
	 * releases the queue, unlike a bare video_unregister_device(), which
	 * would leave the fill work running over the program area freed
	 * afterwards.  The statistics node follows it: stopping the front end
	 * is what releases the areas it is fed from.
	 */
	vb2_video_unregister_device(&ispfe->vdev);
	vb2_video_unregister_device(&ispfe->params_vdev);
	vb2_video_unregister_device(&ispfe->stats_vdev);
	/* The capture teardown above can have scheduled it one last time. */
	cancel_work_sync(&ispfe->stats_work);
	v4l2_async_nf_unregister(&ispfe->notifier);
	v4l2_async_nf_cleanup(&ispfe->notifier);
	/* Before the media device goes, since it holds the back end's three. */
	exynos_becore_input_unregister_graph(ispfe->backend_input);
	media_device_unregister(&ispfe->mdev);
	media_entity_cleanup(&ispfe->params_vdev.entity);
	media_entity_cleanup(&ispfe->stats_vdev.entity);
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
		exynos_becore_input_map(&backend->dev, ispfe->dev,
					&ispfe_backend_ops, ispfe);
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

/*
 * The live shading table is plain kmalloc rather than devm, because consuming
 * a parameters buffer swaps this pointer with that buffer's own allocation:
 * every table then has exactly one owner, and no live one ever sits inside a
 * buffer about to go back to userspace.  What the device holds at teardown is
 * therefore some buffer's former allocation, which devm could not have known
 * about, so the release is registered instead.
 */
static void ispfe_shading_release(void *data)
{
	struct ispfe_device *ispfe = data;

	kfree(ispfe->shading);
	ispfe->shading = NULL;
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
	ispfe->shading = kzalloc(ISPFE_LSC_LUT_BYTES, GFP_KERNEL);
	ispfe->lsc_unity = devm_kmalloc(dev, ISPFE_LSC_LUT_BYTES, GFP_KERNEL);
	ispfe->lsc_input = -ENOENT;
	if (!ispfe->pdma_program_staged || !ispfe->pdma_blocks_staged ||
	    !ispfe->shading || !ispfe->lsc_unity) {
		kfree(ispfe->shading);
		return -ENOMEM;
	}
	ret = devm_add_action_or_reset(dev, ispfe_shading_release, ispfe);
	if (ret)
		return ret;
	ispfe_lsc_unity(ispfe->lsc_unity);

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
	ret = devm_mutex_init(dev, &ispfe->params_lock);
	if (ret)
		return ret;

	ret = devm_mutex_init(dev, &ispfe->stats_lock);
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
	INIT_LIST_HEAD(&ispfe->stats_pending);
	INIT_LIST_HEAD(&ispfe->stats_free);
	INIT_LIST_HEAD(&ispfe->stats_captured);
	for (i = 0; i < ARRAY_SIZE(ispfe->stats_areas); i++)
		INIT_LIST_HEAD(&ispfe->stats_areas[i].list);
	INIT_WORK(&ispfe->stats_work, ispfe_stats_work_fn);
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
	exynos_becore_input_disconnect(ispfe->backend_input);

	/* The shared producer queue does not belong to this video device. */
	scoped_guard(mutex, &ispfe->lock) {
		if (ispfe->owner == ISPFE_OWNER_BACKEND) {
			ispfe->backend_queue_consumer = false;
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
							    ispfe->source_pad,
							    BIT_ULL(0));
				ispfe_sensor_power(ispfe, false);
				ispfe->sensor_streaming = false;
			}
			cancel_work_sync(&ispfe->backend_fill_work);
			ispfe_backend_queue_abort_all(ispfe);
			WRITE_ONCE(ispfe->owner, ISPFE_OWNER_NONE);
		}
	}
	ispfe_media_unregister(ispfe);
	cancel_work_sync(&ispfe->fill_work);

	scoped_guard(mutex, &ispfe->lock) {
		if (ispfe->sensor_streaming) {
			ispfe_stop(ispfe);
			v4l2_subdev_disable_streams(&ispfe->sd,
						    ispfe->source_pad, BIT_ULL(0));
			ispfe_sensor_power(ispfe, false);
			ispfe->sensor_streaming = false;
			WRITE_ONCE(ispfe->owner, ISPFE_OWNER_NONE);
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
