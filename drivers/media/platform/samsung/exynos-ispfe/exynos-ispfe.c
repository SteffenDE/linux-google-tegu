// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google zumapro (Tensor G4) ISPFE camera receive complex.
 *
 * BLK_ISPFE is twelve CSIS links with nine combo D/C-PHYs behind them, the
 * ISPFE front end, and a PDMA that carries per-frame descriptors.  This is the
 * bring-up stage: it powers the block, restores its CMU, and can bring one
 * CSIS link and its PHY up so that a sensor's frames reach the receiver.
 * Nothing lands in memory yet and there is no V4L2.
 *
 * Copyright 2026 Steffen Deusch
 */

#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
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
	/*
	 * request_irq() keeps the name it is given rather than copying it, so
	 * this cannot be a local: /proc/interrupts would print whatever took
	 * over the stack slot.
	 */
	char link_name[8];

	/* Everything below is written from the link interrupt. */
	atomic_t frame_start;
	atomic_t frame_end;
	u32 int0_seen;
	u32 int1_seen;
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
	    ispfe->src.width - 1 >= U16_MAX || ispfe->src.height - 1 >= U16_MAX)
		return -EINVAL;

	ispfe->active = ispfe->src;

	snprintf(ispfe->link_name, sizeof(ispfe->link_name), "csis%u",
		 ispfe->active.link);
	ret = platform_get_irq_byname(pdev, ispfe->link_name);
	if (ret < 0)
		return ret;
	ispfe->link_irq = ret;

	atomic_set(&ispfe->frame_start, 0);
	atomic_set(&ispfe->frame_end, 0);
	ispfe->int0_seen = 0;
	ispfe->int1_seen = 0;

	ret = pm_runtime_resume_and_get(ispfe->dev);
	if (ret)
		return ret;

	/*
	 * Requested before anything is unmasked, and after the domain is up:
	 * the handler touches the link's registers.
	 */
	ret = request_irq(ispfe->link_irq, ispfe_link_isr, 0, ispfe->link_name,
			  ispfe);
	if (ret)
		goto err_put;

	ispfe_phy_start(ispfe);
	ispfe_link_start(ispfe);
	ispfe_link_unmask(ispfe);
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
	free_irq(ispfe->link_irq, ispfe);
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
	seq_printf(s, "int0_seen    %#010x\n", READ_ONCE(ispfe->int0_seen));
	seq_printf(s, "int1_seen    %#010x\n", READ_ONCE(ispfe->int1_seen));

	if (ispfe->streaming)
		seq_printf(s, "cmn_ctrl     %#010x\n",
			   readl_relaxed(ispfe_link(ispfe) + CSIS_CMN_CTRL));

	return 0;
}
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
	debugfs_create_file("enable", 0644, d, ispfe, &ispfe_enable_fops);
	debugfs_create_file("status", 0444, d, ispfe, &ispfe_status_fops);
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
	 * tegu's ultrawide IMX712 as the default source, because it is the
	 * simplest of the three: one recovered mode at full readout and an
	 * ordinary four-lane D-PHY, where the main camera is three-lane C-PHY.
	 */
	ispfe->src = (struct ispfe_source){
		.link = 1, .phy = 1, .lanes = 4, .width = 4208, .height = 3120,
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

	scoped_guard(mutex, &ispfe->lock)
		if (ispfe->streaming)
			ispfe_stop(ispfe);
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
