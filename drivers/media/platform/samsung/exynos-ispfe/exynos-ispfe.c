// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google zumapro (Tensor G4) ISPFE camera receive complex.
 *
 * BLK_ISPFE is twelve CSIS links with nine combo D/C-PHYs behind them, the
 * ISPFE front end, and a PDMA that carries per-frame descriptors.  This is the
 * bring-up stage: it powers the block, restores the CMU state the power domain
 * does not, and can put one CSIS link, its PHY, a frame controller context and
 * PDMA into a raw Bayer capture aimed at its own DMA buffer.  Driven from
 * debugfs; there is no V4L2 yet.
 *
 * Copyright 2026 Steffen Deusch
 */

#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>

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
 * The ISPFE frame controller.  Global registers first; the per-context bits in
 * the interrupt, start and stop words run bayer context c at BIT(c) and
 * phase-detect context c at BIT(7 + c), which is how a main-camera session --
 * one of each -- shows up as 0x81 and an ultrawide one on the second context
 * as 0x2.
 */
#define FC_INIT_ENABLE			0x20008
#define FC_INIT_ENABLE_VAL		0x0000000f
#define FC_INIT_START			0x20010
#define FC_INIT_START_VAL		0x00000001
#define FC_INT_SRC			0x2008c
#define FC_INT_MSK			0x200a0
#define FC_CTX_STOP			0x200a4
#define FC_CTX_START			0x200b0
#define FC_INIT_MASK			0x20094
#define FC_INIT_MASK_VAL		0x3fffffff
#define FC_INIT_GO			0x2011c
#define FC_INIT_GO_VAL			0x00000001
#define FC_INT_ACK			0x20110

#define FC_NUM_BAYER_CTX		5
#define FC_BAYER_CTX(c)			(0x20400 + (c) * 0x400)

/* Offsets within one context. */
#define FC_CTX_INT0			0x04
#define FC_CTX_ENABLE			0x10
#define FC_CTX_ENABLE_VAL		0x00000007
#define FC_CTX_ARM			0x18
#define FC_CTX_INT1			0x24
#define FC_CTX_MODE2			0x2c
#define FC_CTX_MODE2_VAL		0x0000001b
#define FC_CTX_MODE			0x30
#define FC_CTX_MODE_CONFIGURE		0x00000012
#define FC_CTX_MODE_RUN_RAW		0x0000001b
#define FC_CTX_MODE_DRAIN		0x00000009
#define FC_CTX_INT2			0x38
#define FC_CTX_LIMIT			0x44
#define FC_CTX_LIMIT_VAL		0x00003fff
#define FC_CTX_LIMIT_DRAIN		0x00000010
#define FC_CTX_INT3			0x4c
#define FC_CTX_DEPTH			0x58
#define FC_CTX_DEPTH_VAL		0x00000003
#define FC_CTX_RESOL			0x74
/*
 * Two words that travel with the sensor mode and are not decoded.  They are
 * the same for a raw and a YUV stream on the same sensor, so they are not
 * about the output, and they differ between the two IMX712s -- which have the
 * same width and 48 lines of height between them -- so they are not a
 * per-sensor constant either.  Carried as measured values for one mode.
 */
#define FC_CTX_MODE_WORD0		0x78
#define FC_CTX_MODE_WORD1		0x7c
#define FC_CTX_ZERO			0x88
/*
 * Where a context is told which CSIS link and virtual channel feed it.  Three
 * sensors settle the layout: the main camera on link 0 writes 0x00000200, the
 * ultrawide on link 1 writes 0x01000200, and the front camera -- which is on
 * link 6 but on bayer context 0, so nothing here follows from the context
 * number -- writes 0x06000200.  The channel field is the same sensor's
 * phase-detect stream writing 0x1200 against its image stream's 0x200.
 */
#define FC_CTX_SOURCE			0x8c
#define FC_CTX_SOURCE_LINK(n)		((n) << 24)
#define FC_CTX_SOURCE_CHANNEL(n)	((n) << 12)
#define FC_CTX_SOURCE_COMMON		0x00000200
/* All four written with one, meaning unknown; there are three buffers. */
static const u32 fc_ctx_ones[] = { 0x60, 0x64, 0x6c, 0x70 };

/*
 * The line-memory pool.  Nine instances -- five bayer and four phase-detect --
 * and each one appears in three places at three different strides, which is
 * what ties them together: a control block that binds the instance to a frame
 * controller context, an interrupt group, and an interrupt bank.  The main
 * camera's raw stream uses bayer instance 4 and phase-detect instance 0; the
 * ultrawide's preview used bayer instance 2 while the main camera held 3.
 * Which instance a stream gets is an allocation, not a property of the link or
 * the context, so it is a debugfs control here.
 */
#define LMP_CTRL			0x5000c
#define LMP_CTRL_RESET			0x00000000
#define LMP_CTRL_CONFIGURE		0x00000c10
#define LMP_CTRL_RUN			0x00000c11
#define LMP_SIGNATURE			0x50010
#define LMP_SIGNATURE_VAL		0x00dddead

#define LMP_NUM_BAYER			5
#define LMP_BAYER_BIND(n)		(0x27c00 + (n) * 0x400)
#define LMP_BAYER_INT(n)		(0x50058 + (n) * 0x30)
#define LMP_BAYER_BANK(n)		(0x31000 + (n) * 0x2000)
#define LMP_NUM_PDAF			4
#define LMP_PDAF_INT(m)			(0x50148 + (m) * 0x18)

#define LMP_BIND_ENABLE			0x00
#define LMP_BIND_CONTEXT		0x04
#define LMP_INT_SRC			0x00
#define LMP_INT_ARM			0x08
#define LMP_INT_ARM_VAL			0x103ffffe
#define LMP_INT_MSK			0x0c
#define LMP_INT_MSK_BAYER_VAL		0x3fffffff
#define LMP_INT_MSK_PDAF_VAL		0x000003ff
#define LMP_INT_DEBUG			0x501ac
#define LMP_BANK_SRC			0x300
#define LMP_BANK_MSK0			0x308
#define LMP_BANK_MSK0_VAL		0x000007fc
#define LMP_BANK_MSK1			0x30c
#define LMP_BANK_MSK1_ARM		0x000007f8
#define LMP_BANK_MSK1_RUN		0x000007fc

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
#define LMP_ALLOC_SLOT(s)		(0x30 + (s) * 8)
#define LMP_ALLOC_NUM_SLOTS		((LMP_ALLOC_B - LMP_ALLOC_A - 0x30) / 8)
#define LMP_ALLOC_SLOT_VAL0		0x00000040
#define LMP_ALLOC_SLOT_VAL1		0x00000020
#define LMP_ALLOC_GATE			0x30f00
#define LMP_ALLOC_GATE_VAL		0x00000001
#define LMP_ALLOC_MODE			0x30100
#define LMP_ALLOC_MODE_VAL		0x00000053

/*
 * PDMA: five contexts of a small DRAM control ring, and the only path by which
 * a frame buffer address reaches this hardware -- no register write anywhere in
 * the system carries one.  The ring is 1000 bytes whatever the sensor,
 * resolution or frame rate, and its head advances exactly one 16-byte record
 * per frame.  A PDMA context is used with the frame controller context of the
 * same index in every session captured.
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
#define PDMA_WRAP_MSK			0x8c
#define PDMA_WRAP_MSK_VAL		0x0000001f

/*
 * One PDMA record.  The command word and the trailing word are byte-identical
 * between a 4000x3000 and a 2000x1500 capture of the same sensor, so neither
 * is geometry, stride or size; only the address varies, and it varied three
 * ways round with one record written per frame.  Replayed as measured.
 */
struct ispfe_pdma_desc {
	__le32 cmd;
	__le32 addr_lo;
	__le32 addr_hi;
	__le32 tail;
} __packed;

#define PDMA_DESC_CMD			0x0000c003
#define PDMA_DESC_TAIL			0x000019c8
#define PDMA_NUM_RECORDS \
	(PDMA_SIZE_VAL / sizeof(struct ispfe_pdma_desc))

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
	ISPFE_NUM_WINDOWS,
};

static const char * const ispfe_window_names[ISPFE_NUM_WINDOWS] = {
	[ISPFE_WIN_CORE]      = "isp-fe",
	[ISPFE_WIN_CSIS]      = "csis-link-phy",
	[ISPFE_WIN_SYSREG]    = "isp-fe-sysreg",
	[ISPFE_WIN_PDMA]      = "pdma",
	[ISPFE_WIN_PDMA_WRAP] = "pdma-wrap",
	[ISPFE_WIN_CMU]       = "cmu",
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
	u32 ctx;
	u32 lmp;
	u32 slot;

	/* Not derived from anything: measured for one sensor mode. */
	u32 mode_word0;
	u32 mode_word1;
};

struct ispfe_device {
	struct device *dev;
	struct notifier_block genpd_nb;
	struct dentry *debugfs;
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
	bool streaming;
	int link_irq;
	int fc_irq;
	int lmp_irq;
	int pdma_irq;
	/*
	 * request_irq() keeps the name it is given rather than copying it, so
	 * this cannot be a local: /proc/interrupts would print whatever took
	 * over the stack slot.
	 */
	char link_name[8];

	/* The frame buffer, and the descriptor ring that points the ISP at it. */
	void *frame;
	dma_addr_t frame_dma;
	size_t frame_size;
	struct ispfe_pdma_desc *ring;
	dma_addr_t ring_dma;
	u32 head;

	/* Everything below is written from an interrupt. */
	atomic_t frame_start;
	atomic_t frame_end;
	atomic_t fc_events;
	atomic_t lmp_events;
	atomic_t pdma_events;
	u32 int0_seen;
	u32 int1_seen;
	u32 fc_seen;
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
 * BLK_ISPFE holds more than this device: the three ISPFE SysMMUs sit in it
 * too, and because they are this device's IOMMUs they are also its runtime-PM
 * suppliers -- so they resume, and touch their own registers, *before* our
 * ->runtime_resume would get a chance to bring the CMU up.  Hang the restore
 * off the domain instead, where genpd runs it after the PMU has powered the
 * block and before any device in it is resumed.  That is also where the vendor
 * stack does it: its power-domain enable path is the PMU sequence, the TZPC
 * restore, and then the CMU restore, in that order.
 */
static int ispfe_genpd_notify(struct notifier_block *nb, unsigned long action,
			      void *unused)
{
	struct ispfe_device *ispfe = container_of(nb, struct ispfe_device,
						  genpd_nb);

	if (action == GENPD_NOTIFY_ON)
		ispfe_cmu_restore(ispfe);

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
	ispfe_cmu_restore(dev_get_drvdata(dev));

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

static void ispfe_phy_stop(struct ispfe_device *ispfe)
{
	void __iomem *phy = ispfe_phy(ispfe);
	u32 lane;

	ispfe_phy_reset_set(ispfe, false);

	for (lane = 0; lane < ispfe->active.lanes; lane++)
		writel_relaxed(0, phy + PHY_LANE(lane));
	writel_relaxed(0, phy);
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

static void ispfe_link_stop(struct ispfe_device *ispfe)
{
	void __iomem *link = ispfe_link(ispfe);
	u32 ctrl;

	writel_relaxed(0, link + CSIS_INT0_MSK);
	writel_relaxed(0, link + CSIS_INT1_MSK);
	writel_relaxed(0, link + CSIS_FS_MSK);
	writel_relaxed(0, link + CSIS_FE_MSK);

	/* How the vendor stack takes a link down: keep CSI_EN, add a reset. */
	ctrl = readl_relaxed(link + CSIS_CMN_CTRL);
	writel_relaxed(ctrl | CSIS_CMN_CTRL_SW_RESET, link + CSIS_CMN_CTRL);
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

	if (fs)
		atomic_inc(&ispfe->frame_start);
	if (fe) {
		atomic_inc(&ispfe->frame_end);

		/*
		 * Advance the descriptor head one record per frame, which is
		 * what the vendor stack does from its own frame interrupt.
		 * Every record names the same buffer, so this is belt and
		 * braces rather than the thing that makes a frame land.
		 */
		ispfe->head += sizeof(*ispfe->ring);
		if (ispfe->head >= PDMA_NUM_RECORDS * sizeof(*ispfe->ring))
			ispfe->head = 0;
		writel_relaxed(ispfe->head,
			       ispfe->base[ISPFE_WIN_PDMA] +
			       PDMA_CTX(ispfe->active.ctx) + PDMA_HEAD);
	}

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
 * Everything the front end wants before a stream: the
 * frame controller, the line-memory pool and its allocator, and PDMA's
 * per-context enables.  Ordered as the vendor stack does it, including the
 * signature word at LMP_SIGNATURE, which is not configuration.
 *
 * The twelve-link reset that opens it is the same pulse the vendor stack
 * issues across every CSIS bank at device enable, and it is why a link can be
 * configured without knowing what the last session left in it.
 */
static void ispfe_device_init(struct ispfe_device *ispfe)
{
	void __iomem *core = ispfe->base[ISPFE_WIN_CORE];
	void __iomem *csis = ispfe->base[ISPFE_WIN_CSIS];
	unsigned int i;

	for (i = 0; i < CSIS_NUM_LINKS; i++)
		writel_relaxed(CSIS_CMN_CTRL_RESET_ALL,
			       csis + i * CSIS_LINK_STRIDE + CSIS_CMN_CTRL);

	writel_relaxed(FC_INIT_ENABLE_VAL, core + FC_INIT_ENABLE);
	writel_relaxed(FC_INIT_START_VAL, core + FC_INIT_START);

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

	for (i = 0; i < LMP_NUM_BAYER; i++) {
		void __iomem *lmp = core + LMP_BAYER_INT(i);

		writel_relaxed(0, lmp + LMP_INT_SRC);
		writel_relaxed(0, lmp + 0x04);
		writel_relaxed(0, lmp + LMP_INT_ARM);
		writel_relaxed(LMP_INT_MSK_BAYER_VAL, lmp + LMP_INT_MSK);
		writel_relaxed(0, lmp + 0x10);
	}
	for (i = 0; i < LMP_NUM_PDAF; i++) {
		void __iomem *lmp = core + LMP_PDAF_INT(i);

		writel_relaxed(0, lmp + LMP_INT_SRC);
		writel_relaxed(0, lmp + 0x04);
		writel_relaxed(0, lmp + LMP_INT_ARM);
		writel_relaxed(LMP_INT_MSK_PDAF_VAL, lmp + LMP_INT_MSK);
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

	for (i = 0; i < FC_NUM_BAYER_CTX; i++) {
		void __iomem *wrap = ispfe->base[ISPFE_WIN_PDMA_WRAP] +
				     PDMA_WRAP_CTX(i);

		writel_relaxed(PDMA_WRAP_INIT0_VAL, wrap + PDMA_WRAP_INIT0);
		writel_relaxed(PDMA_WRAP_INIT1_VAL, wrap + PDMA_WRAP_INIT1);
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
static void ispfe_ring_fill(struct ispfe_device *ispfe)
{
	unsigned int i;

	for (i = 0; i < PDMA_NUM_RECORDS; i++) {
		ispfe->ring[i].cmd = cpu_to_le32(PDMA_DESC_CMD);
		ispfe->ring[i].addr_lo =
			cpu_to_le32(lower_32_bits(ispfe->frame_dma));
		ispfe->ring[i].addr_hi =
			cpu_to_le32(upper_32_bits(ispfe->frame_dma));
		ispfe->ring[i].tail = cpu_to_le32(PDMA_DESC_TAIL);
	}
}

static void ispfe_pdma_start(struct ispfe_device *ispfe)
{
	void __iomem *pdma = ispfe->base[ISPFE_WIN_PDMA] +
			     PDMA_CTX(ispfe->active.ctx);
	void __iomem *wrap = ispfe->base[ISPFE_WIN_PDMA_WRAP] +
			     PDMA_WRAP_CTX(ispfe->active.ctx);

	ispfe->head = 0;

	writel_relaxed(PDMA_INT0_MSK_VAL, pdma + PDMA_INT0_MSK);
	writel_relaxed(PDMA_INT1_MSK_VAL, pdma + PDMA_INT1_MSK);
	writel_relaxed(PDMA_WRAP_MSK_VAL, wrap + PDMA_WRAP_MSK);

	writel_relaxed(lower_32_bits(ispfe->ring_dma), pdma + PDMA_BASE_LO);
	writel_relaxed(upper_32_bits(ispfe->ring_dma), pdma + PDMA_BASE_HI);
	writel_relaxed(PDMA_SIZE_VAL, pdma + PDMA_SIZE);
	writel_relaxed(0, pdma + PDMA_HEAD);
	dma_wmb();
	writel_relaxed(1, pdma + PDMA_ENABLE);
	writel_relaxed(1, pdma + PDMA_INT0_ARM);
}

static void ispfe_pdma_stop(struct ispfe_device *ispfe)
{
	void __iomem *pdma = ispfe->base[ISPFE_WIN_PDMA] +
			     PDMA_CTX(ispfe->active.ctx);
	void __iomem *wrap = ispfe->base[ISPFE_WIN_PDMA_WRAP] +
			     PDMA_WRAP_CTX(ispfe->active.ctx);

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
	void __iomem *ctx = core + FC_BAYER_CTX(ispfe->active.ctx);
	void __iomem *bind = core + LMP_BAYER_BIND(ispfe->active.lmp);
	void __iomem *bank = core + LMP_BAYER_BANK(ispfe->active.lmp);
	void __iomem *alloc = core + LMP_ALLOC_A;
	unsigned int i;

	writel_relaxed(LMP_BANK_MSK0_VAL, bank + LMP_BANK_MSK0);
	writel_relaxed(LMP_BANK_MSK1_ARM, bank + LMP_BANK_MSK1);

	writel_relaxed(LMP_ALLOC_SLOT_VAL0,
		       alloc + LMP_ALLOC_SLOT(ispfe->active.slot));
	writel_relaxed(LMP_ALLOC_SLOT_VAL1,
		       alloc + LMP_ALLOC_SLOT(ispfe->active.slot) + 4);
	writel_relaxed(LMP_ALLOC_SLOT_VAL0,
		       core + LMP_ALLOC_B + LMP_ALLOC_SLOT(ispfe->active.slot));
	writel_relaxed(LMP_ALLOC_SLOT_VAL1,
		       core + LMP_ALLOC_B + LMP_ALLOC_SLOT(ispfe->active.slot) + 4);
	writel_relaxed(LMP_ALLOC_CTRL_IMX712, alloc + LMP_ALLOC_CTRL);
	writel_relaxed(LMP_ALLOC_CTRL_B, core + LMP_ALLOC_B + LMP_ALLOC_CTRL);

	writel_relaxed(LMP_INT_ARM_VAL,
		       core + LMP_BAYER_INT(ispfe->active.lmp) + LMP_INT_ARM);

	writel_relaxed(FC_CTX_ENABLE_VAL, ctx + FC_CTX_ENABLE);
	writel_relaxed(FC_CTX_MODE_CONFIGURE, ctx + FC_CTX_MODE);
	writel_relaxed(FC_CTX_LIMIT_VAL, ctx + FC_CTX_LIMIT);
	writel_relaxed(FC_CTX_DEPTH_VAL, ctx + FC_CTX_DEPTH);

	writel_relaxed(1, ctx + FC_CTX_ARM);
	writel_relaxed(BIT(ispfe->active.ctx), core + FC_CTX_START);

	writel_relaxed(ispfe->active.mode_word0, ctx + FC_CTX_MODE_WORD0);
	writel_relaxed(ispfe->active.mode_word1, ctx + FC_CTX_MODE_WORD1);
	for (i = 0; i < ARRAY_SIZE(fc_ctx_ones); i++)
		writel_relaxed(1, ctx + fc_ctx_ones[i]);
	writel_relaxed(FC_CTX_SOURCE_LINK(ispfe->active.link) |
		       FC_CTX_SOURCE_CHANNEL(0) | FC_CTX_SOURCE_COMMON,
		       ctx + FC_CTX_SOURCE);
	writel_relaxed(0, ctx + FC_CTX_ZERO);
	writel_relaxed(CSIS_ISP_RESOL(ispfe->active.width, ispfe->active.height),
		       ctx + FC_CTX_RESOL);
	writel_relaxed(FC_CTX_MODE2_VAL, ctx + FC_CTX_MODE2);

	writel_relaxed(ispfe->active.ctx, bind + LMP_BIND_CONTEXT);
	writel_relaxed(1, bind + LMP_BIND_ENABLE);

	writel_relaxed(BIT(ispfe->active.ctx), core + FC_INT_MSK);
}

static void ispfe_fc_run(struct ispfe_device *ispfe)
{
	void __iomem *core = ispfe->base[ISPFE_WIN_CORE];

	writel_relaxed(LMP_BANK_MSK1_RUN,
		       core + LMP_BAYER_BANK(ispfe->active.lmp) + LMP_BANK_MSK1);
	writel_relaxed(FC_CTX_MODE_RUN_RAW,
		       core + FC_BAYER_CTX(ispfe->active.ctx) + FC_CTX_MODE);
}

static void ispfe_fc_stop(struct ispfe_device *ispfe)
{
	void __iomem *core = ispfe->base[ISPFE_WIN_CORE];
	void __iomem *ctx = core + FC_BAYER_CTX(ispfe->active.ctx);

	writel_relaxed(0, core + FC_INT_MSK);
	writel_relaxed(BIT(ispfe->active.ctx), core + FC_CTX_STOP);
	writel_relaxed(0, ctx + FC_CTX_ARM);
	writel_relaxed(0, ctx + FC_CTX_ENABLE);
	writel_relaxed(0, ctx + FC_CTX_MODE);
	writel_relaxed(0, ctx + FC_CTX_LIMIT);
	writel_relaxed(0, ctx + FC_CTX_DEPTH);
	writel_relaxed(0, core + LMP_BAYER_BIND(ispfe->active.lmp) +
			  LMP_BIND_ENABLE);
	writel_relaxed(0, core + LMP_BAYER_BANK(ispfe->active.lmp) +
			  LMP_BANK_MSK1);
	writel_relaxed(0, core + LMP_BAYER_BANK(ispfe->active.lmp) +
			  LMP_BANK_MSK0);
}

static irqreturn_t ispfe_fc_isr(int irq, void *data)
{
	struct ispfe_device *ispfe = data;
	void __iomem *core = ispfe->base[ISPFE_WIN_CORE];
	void __iomem *ctx = core + FC_BAYER_CTX(ispfe->active.ctx);
	u32 src, int1;

	src = readl_relaxed(core + FC_INT_SRC);
	if (!src)
		return IRQ_NONE;

	int1 = readl_relaxed(ctx + FC_CTX_INT1);
	WRITE_ONCE(ispfe->fc_seen, READ_ONCE(ispfe->fc_seen) | int1);
	writel_relaxed(readl_relaxed(ctx + FC_CTX_INT0), ctx + FC_CTX_INT0);
	writel_relaxed(int1, ctx + FC_CTX_INT1);
	writel_relaxed(readl_relaxed(ctx + FC_CTX_INT2), ctx + FC_CTX_INT2);
	writel_relaxed(readl_relaxed(ctx + FC_CTX_INT3), ctx + FC_CTX_INT3);
	writel_relaxed(0, core + FC_INT_ACK);
	writel_relaxed(src, core + FC_INT_SRC);

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
	unsigned int i;
	u32 seen;

	seen = readl_relaxed(core + LMP_INT_DEBUG);
	writel_relaxed(seen, core + LMP_INT_DEBUG);

	for (i = 0; i < LMP_NUM_BAYER; i++) {
		void __iomem *lmp = core + LMP_BAYER_INT(i);
		void __iomem *bank = core + LMP_BAYER_BANK(i);
		u32 src = readl_relaxed(lmp + LMP_INT_SRC);
		u32 bsrc = readl_relaxed(bank + LMP_BANK_SRC);

		seen |= src | bsrc;
		writel_relaxed(src, lmp + LMP_INT_SRC);
		writel_relaxed(bsrc, bank + LMP_BANK_SRC);
	}

	if (!seen)
		return IRQ_NONE;

	atomic_inc(&ispfe->lmp_events);

	return IRQ_HANDLED;
}

/* All five PDMA contexts share one line too, for the same reason. */
static irqreturn_t ispfe_pdma_isr(int irq, void *data)
{
	struct ispfe_device *ispfe = data;
	unsigned int i;
	u32 seen = 0;

	for (i = 0; i < FC_NUM_BAYER_CTX; i++) {
		void __iomem *pdma = ispfe->base[ISPFE_WIN_PDMA] + PDMA_CTX(i);
		void __iomem *wrap = ispfe->base[ISPFE_WIN_PDMA_WRAP] +
				     PDMA_WRAP_CTX(i);
		u32 int0 = readl_relaxed(pdma + PDMA_INT0_SRC);
		u32 int1 = readl_relaxed(pdma + PDMA_INT1_SRC);
		u32 wr = readl_relaxed(wrap + PDMA_WRAP_SRC);

		seen |= int0 | int1 | wr;
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
	if (ispfe->ring) {
		dma_free_coherent(ispfe->dev, PAGE_SIZE, ispfe->ring,
				  ispfe->ring_dma);
		ispfe->ring = NULL;
	}
	if (ispfe->frame) {
		dma_free_coherent(ispfe->dev, ispfe->frame_size, ispfe->frame,
				  ispfe->frame_dma);
		ispfe->frame = NULL;
		ispfe->frame_size = 0;
	}
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


	if (size == SIZE_MAX)
		return -EOVERFLOW;

	if (ispfe->frame && ispfe->frame_size == size) {
		memset(ispfe->frame, ISPFE_FRAME_POISON, size);
		ispfe_ring_fill(ispfe);
		return 0;
	}

	ispfe_buffers_free(ispfe);

	ispfe->frame = dma_alloc_coherent(ispfe->dev, size, &ispfe->frame_dma,
					  GFP_KERNEL);
	if (!ispfe->frame)
		return -ENOMEM;
	ispfe->frame_size = size;

	ispfe->ring = dma_alloc_coherent(ispfe->dev, PAGE_SIZE,
					 &ispfe->ring_dma, GFP_KERNEL);
	if (!ispfe->ring) {
		ispfe_buffers_free(ispfe);
		return -ENOMEM;
	}

	/*
	 * Poisoned rather than zeroed, so that a frame that never arrived is
	 * distinguishable from one that arrived black.
	 */
	memset(ispfe->frame, ISPFE_FRAME_POISON, size);
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

	ispfe->fc_irq = platform_get_irq_byname(pdev, "fc");
	ispfe->lmp_irq = platform_get_irq_byname(pdev, "lmp-bayer");
	ispfe->pdma_irq = platform_get_irq_byname(pdev, "pdma");
	if (ispfe->fc_irq < 0 || ispfe->lmp_irq < 0 || ispfe->pdma_irq < 0) {
		ret = -ENODEV;
		goto err_link;
	}

	ret = request_irq(ispfe->fc_irq, ispfe_fc_isr, 0, "ispfe-fc", ispfe);
	if (ret)
		goto err_link;

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
err_link:
	free_irq(ispfe->link_irq, ispfe);
	return ret;
}

static void ispfe_free_irqs(struct ispfe_device *ispfe)
{
	free_irq(ispfe->pdma_irq, ispfe);
	free_irq(ispfe->lmp_irq, ispfe);
	free_irq(ispfe->fc_irq, ispfe);
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
	    ispfe->src.ctx >= FC_NUM_BAYER_CTX ||
	    ispfe->src.lmp >= LMP_NUM_BAYER ||
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
	atomic_set(&ispfe->lmp_events, 0);
	atomic_set(&ispfe->pdma_events, 0);
	ispfe->int0_seen = 0;
	ispfe->int1_seen = 0;
	ispfe->fc_seen = 0;

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
	ispfe_pdma_start(ispfe);
	ispfe_fc_start(ispfe);
	ispfe_phy_start(ispfe);
	ispfe_link_start(ispfe);
	ispfe_link_unmask(ispfe);
	ispfe_fc_run(ispfe);

	ispfe->streaming = true;

	return 0;

err_put:
	pm_runtime_put(ispfe->dev);
	return ret;
}

static void ispfe_stop(struct ispfe_device *ispfe)
{
	ispfe_link_stop(ispfe);
	ispfe_phy_stop(ispfe);
	ispfe_fc_stop(ispfe);
	ispfe_pdma_stop(ispfe);
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

static int ispfe_status_show(struct seq_file *s, void *unused)
{
	struct ispfe_device *ispfe = s->private;

	guard(mutex)(&ispfe->lock);

	seq_printf(s, "streaming    %u\n", ispfe->streaming);
	seq_printf(s, "frame_start  %u\n", atomic_read(&ispfe->frame_start));
	seq_printf(s, "frame_end    %u\n", atomic_read(&ispfe->frame_end));
	seq_printf(s, "fc_events    %u\n", atomic_read(&ispfe->fc_events));
	seq_printf(s, "lmp_events   %u\n", atomic_read(&ispfe->lmp_events));
	seq_printf(s, "pdma_events  %u\n", atomic_read(&ispfe->pdma_events));
	seq_printf(s, "int0_seen    %#010x\n", READ_ONCE(ispfe->int0_seen));
	seq_printf(s, "int1_seen    %#010x\n", READ_ONCE(ispfe->int1_seen));
	seq_printf(s, "fc_seen      %#010x\n", READ_ONCE(ispfe->fc_seen));
	seq_printf(s, "frame_iova   %pad\n", &ispfe->frame_dma);
	seq_printf(s, "frame_size   %zu\n", ispfe->frame_size);
	seq_printf(s, "ring_iova    %pad\n", &ispfe->ring_dma);
	seq_printf(s, "ring_head    %#x\n", ispfe->head);

	if (ispfe->streaming) {
		void __iomem *core = ispfe->base[ISPFE_WIN_CORE];

		seq_printf(s, "cmn_ctrl     %#010x\n",
			   readl_relaxed(ispfe_link(ispfe) + CSIS_CMN_CTRL));
		seq_printf(s, "fc_mode      %#010x\n",
			   readl_relaxed(core + FC_BAYER_CTX(ispfe->active.ctx) +
					 FC_CTX_MODE));
		seq_printf(s, "pdma_head    %#010x\n",
			   readl_relaxed(ispfe->base[ISPFE_WIN_PDMA] +
					 PDMA_CTX(ispfe->active.ctx) + PDMA_HEAD));
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
	debugfs_create_u32("ctx", 0644, d, &ispfe->src.ctx);
	debugfs_create_u32("lmp", 0644, d, &ispfe->src.lmp);
	debugfs_create_u32("slot", 0644, d, &ispfe->src.slot);
	debugfs_create_u32("mode_word0", 0644, d, &ispfe->src.mode_word0);
	debugfs_create_u32("mode_word1", 0644, d, &ispfe->src.mode_word1);
	debugfs_create_file("enable", 0644, d, ispfe, &ispfe_enable_fops);
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
		 * The exact configuration the vendor stack put this sensor in,
		 * rather than a plausible mixture of several.  Nothing in the
		 * captures separates "per sensor mode" from "per context" for
		 * the two mode words, and they were only ever measured in
		 * context 1's register block -- so use context 1, the PDMA
		 * context beside it, and the line-memory instance and slot that
		 * went with them.
		 */
		.ctx = 1, .lmp = 2, .slot = 0,
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

	debugfs_remove_recursive(ispfe->debugfs);

	scoped_guard(mutex, &ispfe->lock) {
		if (ispfe->streaming)
			ispfe_stop(ispfe);
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
	},
};
module_platform_driver(ispfe_driver);

MODULE_DESCRIPTION("Google zumapro ISPFE camera receive complex");
MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_LICENSE("GPL");
