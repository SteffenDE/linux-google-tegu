// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Zumapro camera back-end core bring-up driver
 *
 * The register sequences here are the common processor lifecycle observed on
 * the Pixel 9a vendor stack.  The debugfs diagnostic accepts a structurally
 * fixed, relocatable RGBP/YUVP program for offline bring-up; it is deliberately
 * not a camera ABI.  Separate GTNR-startup and MCSC recipes can be normalized
 * into dormant command lists while those downstream stages are brought up.
 * Powering a block down is safe only after every owned processor has accepted
 * a software reset.
 *
 * The driver is more than this file.  This one holds the device: power, the
 * VOTF link, the interrupt path, the allocations, the run loop, the producer
 * API and probe.  The rest is:
 *
 *   -params.c     the parameters node and the encoders that read a buffer
 *   -generated.c  the words the driver states where no buffer describes one
 *   -geometry.c   the three rasters and the surfaces derived from them
 *   -recipe.c     the captured program, and how a word's four sources apply
 *   -video.c      the subdev and the capture node
 *   -debugfs.c    the bring-up surface, which is deliberately not an ABI
 *
 * exynos-becore-common.h is what they share; exynos-becore-regs.h is the
 * silicon.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/log2.h>
#include <linux/media/samsung/exynos-becore-config.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/refcount.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <media/exynos-becore.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-isp.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#include "exynos-becore-regs.h"
#include "exynos-becore-common.h"
#include "exynos-becore-recipe.h"
#include "exynos-becore-gtnr-recipe.h"
#include "exynos-becore-mcsc-recipe.h"
#include "exynos-becore-sharpen.h"
#include "exynos-becore-yuvnr.h"
#include "exynos-becore-byrdns.h"
#include "exynos-becore-dmsc.h"

/*
 * Token geometry, taken whole from the one captured link that has the same
 * shape as this one rather than assembled from two.  Lyric computes a token
 * from a per-IP table -- extra lines plus process lines, halved for chroma,
 * quartered for an SBWC producer -- and for YUVP into MCSC every term is 1,
 * because both blocks declare one process line.  That gives a one-line ring,
 * which is not a geometry any measured link uses.
 *
 * What works is `YUVP -> TNR`'s pair, 12 and 4 against 48 and 16.  It is the
 * vendor's only captured link with a two-plane SBWC producer, which is the
 * shape this one has.  MCSC's own captured pairing, 64 and 32, was taken
 * against GDC0, whose producer token is 1, and putting YUVP in front of it
 * fails: the link half-forms, one plane reaching VOTF_CONNECT and the other
 * sitting in WAIT_TOKEN_ACK, and at a limit large enough to paper over that
 * the consumer runs about a third ahead of the data and the picture goes to
 * fill part way down.
 *
 * What separates the two is a whole number of producer tokens per consumer
 * one.  48 over 12 is 4 and 16 over 4 is 4; 64 over 12 is 5.33, and it is
 * exactly the plane whose ratio is not whole that loses.  Every captured
 * endpoint pair holds that ratio -- 48/12 and 16/4, 64/1 and 32/1, 16/1 and
 * 8/1 -- and so does every plane count against its own producer token: 3000
 * over 12 and 1500 over 4, 510 over 1 and 255 over 1.
 *
 * The consumer's token divides nothing.  All three links were captured at
 * 3000/1500 or 510/255 lines, where 48 gives 62.5 tokens, 16 gives 93.75, 64
 * and 32 give 46.875 and 16 and 8 give 31.875 -- not one of the six captured
 * planes comes out whole, and the link this driver runs is byte-identical to
 * the memory path with a chroma plane of 1560 lines in tokens of 16, which is
 * 97.5.  So a partial last consumer token is what the hardware does, and the
 * two ends emphatically do not agree on how many tokens a frame is.
 *
 * That leaves two invariants.  The pair's ratio is a property of these two
 * tables and becore_c2serv_tokens_validate() checks it at probe.  The chain
 * height against the producer's token is the one a negotiated raster can
 * break, and becore_c2serv_chain_fits() decides it per frame, because it says
 * which path the frame takes rather than whether the device works.
 */
static const u32 becore_c2serv_tws_lines_in_token[BECORE_C2SERV_LINK_PLANES] = {
	12, 4,
};

static const u32 becore_c2serv_trs_lines_in_token[BECORE_C2SERV_LINK_PLANES] = {
	48, 16,
};

/*
 * A debug override, or the captured value when none is set.  The link this
 * driver invents has no encoded program to check it against, so its geometry
 * has to be swept against what the fabric reports -- and a sweep that needs a
 * rebuild for every point is a sweep nobody finishes.
 */
static u32 becore_c2serv_override(u32 requested, u32 captured)
{
	/*
	 * Every field these feed is eight bits wide, and a value that wrapped
	 * silently would be read back as something the sweep never asked for
	 * -- a limit of 256 becoming 0 looks exactly like the deadlock the
	 * sweep is trying to characterise.
	 */
	return min(requested ? requested : captured, 0xffu);
}

/*
 * Each plane's token is set on its own, because the two ratios that matter
 * differ: the captured consumer pair is 64 and 32, and the vendor's only
 * SBWC-producer link pairs 12 and 4 against 48 and 16 -- a third, not a half,
 * because TNR's entry in the parameter table carries sixteen extra lines on
 * the luma plane and none on the chroma one.  Deriving plane 1 from plane 0
 * made that second shape inexpressible, which is exactly the shape worth
 * trying.
 */
static u32 becore_c2serv_token(const u32 *requested, const u32 *captured,
			       unsigned int plane)
{
	u32 value = min(requested[plane], 0xffu);

	return value ? value : captured[plane];
}

/*
 * How far the producer may run ahead, in tokens.  The vendor writes 1 on
 * every endpoint of every captured link; Samsung's own driver writes 0xff and
 * leans on a token of 1 instead.  Either way the ring is tens of lines.
 */
#define BECORE_C2SERV_LIMIT		1

#define BECORE_RESET_TIMEOUT_US		1000

/* What Lyric gives its own reset polls, and most of a frame at this rate. */
#define BECORE_C2SERV_FLUSH_US		20000

/*
 * How long a reset is given once it has missed the bound above.  Two frame
 * times at this sensor's rate, which is far longer than any reset observed to
 * complete and short enough that a genuinely stuck one is still an error
 * rather than a hang.
 */
#define BECORE_RESET_LATE_US		50000

#define BECORE_YUVP_STAGE_BLOCKS	(BIT(BECORE_RGBP) | BIT(BECORE_YUVP))

#define BECORE_RUN_TIMEOUT_MS		1000

/*
 * What the driver derives from 17 nodes an axis has to be the length the
 * vendor's own burst was, which is what the generator recorded when it took
 * that burst out of the recipe.
 */
static_assert(BECORE_CLUT_LATTICE_WORDS == BECORE_YUVP_CLUT_WORDS);
static_assert(BECORE_YUVP_CLUT_HEADER < BECORE_YUVP_HEADER_COUNT);

/*
 * The INTCAM operating point the vendor holds while a 4000 x 3000 rear
 * ultrawide stream runs [HW 2026-08-20].  It has to be an exact rung of the
 * measured ladder -- 664000 533000 465000 310000 233000 111000 kHz -- because
 * the ACPM clock passes the requested rate straight to firmware rather than
 * rounding it up to a supported OPP the way downstream's PM QoS does.
 */
#define BECORE_INTCAM_ACTIVE_RATE	233000000UL

/*
 * The register block and the 96 KiB buffer are two halves of one thing, and
 * nothing else in this driver says so: state it where both are in scope, so a
 * future edit to either has to answer for the other.
 */
static_assert(BECORE_LTM_SLCGRID_COLUMNS * BECORE_LTM_SLCGRID_ROWS *
	      BECORE_LTM_SLCGRID_DEPTH * BECORE_LTM_SLCGRID_CELL_SHORTS *
	      sizeof(__le16) == BECORE_GRID_SIZE);

#define BECORE_MCSC_INPUT_VOTF_STALL_LINES	GENMASK_U32(29, 16)

u32 becore_mcsc_votf_enable(const u32 *requested_token)
{
	u32 token = becore_c2serv_token(requested_token,
					becore_c2serv_trs_lines_in_token, 0);

	return FIELD_PREP(BECORE_MCSC_INPUT_VOTF_STALL_LINES, token) | 1;
}

/*
 * One phase boundary.  The caller passes the previous boundary and receives
 * this one, so a phase costs a single clock read and no phase can be counted
 * twice or missed.
 */
static ktime_t becore_timing_mark(u64 *phase_ns, enum becore_timing_phase phase,
				  ktime_t since)
{
	ktime_t now = ktime_get();

	phase_ns[phase] = ktime_to_ns(ktime_sub(now, since));

	return now;
}

static const char * const becore_pm_domain_names[] = {
	"yuvp",
	"rgbp",
	"gdc",
	"mcsc",
};

static const char * const becore_irq_names[] = {
	"rgbp-int0", "rgbp-int1",
	"mcfp-int0", "mcfp-int1",
	"yuvp-int0", "yuvp-int1",
	"mcsc-int0", "mcsc-int1",
};

static const char * const becore_ssmt_names[] = {
	"ssmt-rgbp",
	"ssmt-yuvp",
	"ssmt-mcfp0",
	"ssmt-mcfp1",
	"ssmt-mcfp2",
	"ssmt-mcfp3",
	"ssmt-mcfp4",
	"ssmt-mcsc0",
	"ssmt-mcsc1",
	"ssmt-mcsc2",
	"ssmt-mcsc3",
	"ssmt-mcsc4",
	"ssmt-mcsc5",
	"ssmt-mcsc6",
};

/*
 * The three VOTF C2SERV windows this node owns, out of the seven the camera
 * complex has.  A window's local IP ID is the top sixteen bits of its own
 * base address, and the endpoint counts are the vendor's own hardware
 * parameters: RGBP has one producer, YUVP two, MCSC four consumers.  None of
 * the three has endpoints in both directions, which is why a producer here
 * can only ever reach a consumer somewhere else.
 */

const struct becore_c2serv_desc becore_c2serv[BECORE_NUM_C2SERV] = {
	[BECORE_C2SERV_RGBP] = { "rgbp-c2serv", 0x1c46, 1, 0 },
	[BECORE_C2SERV_YUVP] = { "yuvp-c2serv", 0x1c86, 2, 0 },
	[BECORE_C2SERV_MCSC] = { "mcsc-c2serv", 0x1d05, 0, 4 },
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

/* MCSC keeps SET_CTRL asserted while its command queue is active. */
static const struct becore_regval becore_mcsc_init[] = {
	{ BECORE_SET_CTRL, 0x1 },
	{ BECORE_GLOBAL_ENABLE_CLEAR, 0x1 },
	{ BECORE_FRO_GLOBAL_ENABLE, 0x3fffff },
	{ BECORE_CMDQ_ENABLE, 0x1 },
	{ BECORE_STAT_RDMACL_EN, 0x1 },
	{ BECORE_GLOBAL_ENABLE, 0x1 },
};

/*
 * Every stage of RGBP and YUVP ends its register page with a stream CRC:
 * CRC_SEED in bits 7:0, writable, and CRC_RESULT in bits 15:8, read-only.
 * Samsung's own tables for this IP family carry both fields at these widths,
 * and Lyric's Zuma descriptors name the registers -- but nothing has ever
 * exercised them, on this SoC or here, so whether the result register behaves
 * as the table says is unverified.  Reading the whole word rather than the
 * result field is what answers that: the seed has to read back where the
 * table puts it.
 *
 * With a deterministic input this is a per-stage signature of the stream
 * leaving each block, for one MMIO write and one read -- no WDMA, no output
 * buffer, no image decode -- so a discrepancy localises to the first stage
 * whose CRC moved.
 *
 * A zero seed produces a zero result on every stage, including ones that are
 * plainly running [HW 2026-08-22], so the feature is inert until a seed is
 * written and the driver skips it entirely rather than spending 117 MMIO
 * accesses per frame on a diagnostic nobody asked for.  Eleven of these
 * offsets are in register pages no recipe touches, so leaving them alone by
 * default also keeps that first contact inside the harness.
 *
 * The offsets come from Lyric's own descriptors rather than a sibling SFR
 * header: where the two disagree the target's map wins, and they do disagree,
 * over where local tone mapping ends.
 */

static const struct becore_stream_crc becore_rgbp_stream_crc[] = {
	{ 0x30fc, "byr_dtp" },
	{ 0x31fc, "byr_dns" },
	{ 0x32fc, "byr_dmsc" },
	{ 0x38fc, "rgb_gamma_rgb" },
	{ 0x3afc, "rgb_gtm" },
	{ 0x3bfc, "rgb_rgb_to_yuv" },
	{ 0x3cfc, "yuv_yuv444_to422" },
	{ 0x3efc, "y_decomp" },
	{ 0x40fc, "rgb_ccm33" },
	{ 0x45fc, "yuv_sc" },
	{ 0x47fc, "y_gamma_lr" },
	{ 0x49fc, "y_upsc" },
	{ 0x4bfc, "y_gamma_hr" },
};

static const struct becore_stream_crc becore_yuvp_stream_crc[] = {
	{ 0x10fc, "yuv_cinfifo0" },
	{ 0x30fc, "yuv_dtp" },
	{ 0x37fc, "yuv_yuv_nr" },
	{ 0x38fc, "yuv_yuv422_to444" },
	{ 0x39fc, "yuv_yuv_to_rgb" },
	{ 0x3afc, "rgb_rgb_to_yuv" },
	{ 0x3cfc, "yuv_dither420" },
	{ 0x3efc, "rgb_invccm33" },
	{ 0x41fc, "rgb_degamma_rgb" },
	{ 0x44fc, "rgb_gamma_rgb" },
	{ 0x46fc, "yuv_gamma_oetf" },
	{ 0x4dfc, "rgb_prc" },
	{ 0x51fc, "yuv_sharp_enhancer_sharpen" },
	{ 0x54fc, "yuv_sharp_enhancer_hfmixer" },
	{ 0x56fc, "yuv_sharp_enhancer_noise_gen" },
	{ 0x57fc, "yuv_sharp_enhancer_noise_mixer" },
	{ 0x58fc, "yuv_sharp_enhancer_cont_det" },
	{ 0x5afc, "yuv_sharp_enhancer" },
	{ 0x66fc, "rgb_diablo_ltm" },
	{ 0x6afc, "yuv_yuv_to_rgb_zuma" },
	{ 0x6dfc, "rgb_degamma_rgb_main" },
	{ 0x72fc, "rgb_degamma_rgb_linear" },
	{ 0x74fc, "rgb_rgb_to_yuv420" },
	{ 0x75fc, "rgb_rgb_to_yuv422" },
	{ 0x76fc, "yuv_yuv420_to422" },
	{ 0x7afc, "rgb_diablo_ccm" },
};

static_assert(ARRAY_SIZE(becore_rgbp_stream_crc) <= BECORE_STREAM_CRC_MAX);
static_assert(ARRAY_SIZE(becore_yuvp_stream_crc) <= BECORE_STREAM_CRC_MAX);

const struct becore_stream_crc *
becore_stream_crc_table(enum becore_block_id id, size_t *count)
{
	if (id == BECORE_RGBP) {
		*count = ARRAY_SIZE(becore_rgbp_stream_crc);
		return becore_rgbp_stream_crc;
	}
	if (id == BECORE_YUVP) {
		*count = ARRAY_SIZE(becore_yuvp_stream_crc);
		return becore_yuvp_stream_crc;
	}
	*count = 0;

	return NULL;
}

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

/*
 * Wait for one software reset to clear, and tell a late one from a refusing
 * one.
 *
 * The two want different fixes and the millisecond bound this used to have
 * could not distinguish them: a processor that clears at three milliseconds
 * and one that never clears both read back 1 when the poll gives up, and
 * reading 1 was taken to mean the block was refusing.  So a reset that misses
 * the expected bound is polled to a far more generous one and the outcome is
 * named -- a warning with how long it actually took, or an error saying it
 * never cleared at all.
 *
 * Waiting the extra time is not merely diagnostic.  A reset that completes is
 * a reset that completed, and giving up on one that was about to finish is
 * what turns a slow teardown into a device quarantined until reboot.
 */
static int becore_poll_reset(struct device *dev, void __iomem *reg,
			     const char *name)
{
	ktime_t start = ktime_get();
	u32 value;
	int ret;

	ret = readl_poll_timeout(reg, value, !value, 1,
				 BECORE_RESET_TIMEOUT_US);
	if (!ret)
		return 0;

	ret = readl_poll_timeout(reg, value, !value, 10,
				 BECORE_RESET_LATE_US);
	if (!ret) {
		dev_warn(dev, "%s reset cleared late, after %lld us\n",
			 name, ktime_us_delta(ktime_get(), start));
		return 0;
	}

	dev_err(dev, "%s reset never cleared in %u us (0x%08x)\n",
		name, BECORE_RESET_LATE_US, value);

	return ret;
}

static int becore_wait_reset(struct becore_block *block)
{
	return becore_poll_reset(block->becore->dev,
				 block->base + BECORE_SW_RESET, block->name);
}

/*
 * What the chain says about itself when a reset will not complete.
 *
 * A processor refusing to reset is the end of a story this driver cannot
 * otherwise read: `reset_failed` latches, the device is quarantined until
 * reboot, and nothing says *why*.  Lyric names three registers that do --
 * `ip_busy_monitor_0`, `ip_stall_out_status_0` and, on YUVP, the input FIFO's
 * own stall counter -- and the C2SERV windows carry the other half, since a
 * VOTF producer and consumer that have stalled against each other are exactly
 * what the failing case has in it.
 *
 * MCSC is deliberately not asked for the first two: Lyric's descriptor table
 * does not reach them on that block, and an unimplemented offset here is an
 * SError rather than a zero.
 *
 * The C2SERV windows are read whatever the driver thinks of their link state,
 * because that state is not what makes them readable -- becore_c2serv_prepare()
 * reads and writes the same window before it sets `ready`, and both windows are
 * mapped or the probe fails.  Gating on `ready` would silence exactly the caller
 * that needs this most: the runtime suspend, which unprepares the windows -- and
 * so clears `ready` -- immediately before the reset that can fail.
 *
 * Read-only and only on the failure path, so a healthy teardown pays nothing.
 */
static void becore_report_stuck(struct becore_device *becore)
{
	unsigned int i;

	for (i = 0; i < BECORE_NUM_BLOCKS; i++) {
		struct becore_block *block = &becore->blocks[i];

		if (i == BECORE_MCSC) {
			dev_err(becore->dev,
				"%s stuck: reset %#010x processing %#010x cinfifo %#010x\n",
				block->name,
				readl_relaxed(block->base + BECORE_SW_RESET),
				readl_relaxed(block->base + BECORE_SET_CTRL),
				readl_relaxed(block->base + BECORE_C_LOADER_ENABLE));
			continue;
		}

		dev_err(becore->dev,
			"%s stuck: reset %#010x processing %#010x cinfifo %#010x busy %#010x stall %#010x\n",
			block->name,
			readl_relaxed(block->base + BECORE_SW_RESET),
			readl_relaxed(block->base + BECORE_SET_CTRL),
			readl_relaxed(block->base + BECORE_C_LOADER_ENABLE),
			readl_relaxed(block->base + BECORE_IP_BUSY_MONITOR_0),
			readl_relaxed(block->base + BECORE_IP_STALL_OUT_STATUS_0));

		if (i == BECORE_YUVP)
			dev_err(becore->dev, "%s stuck: cinfifo stalls %u\n",
				block->name,
				readl_relaxed(block->base +
					      BECORE_YUVP_CINFIFO_STALL_CNT));
	}

	for (i = 0; i < BECORE_NUM_C2SERV; i++) {
		void __iomem *base = becore->c2serv[i];
		unsigned int n;

		dev_err(becore->dev, "%s stuck: reset %#010x ring %#010x/%#010x\n",
			becore_c2serv[i].name,
			readl_relaxed(base + BECORE_C2SERV_SW_RESET),
			readl_relaxed(base + BECORE_C2SERV_RING_CLK_EN),
			readl_relaxed(base + BECORE_C2SERV_RING_ENABLE));

		for (n = 0; n < BECORE_C2SERV_LINK_PLANES; n++)
			dev_err(becore->dev,
				"%s stuck: plane %u tws busy %#x full %#x, trs busy %#x lost %#x\n",
				becore_c2serv[i].name, n,
				readl_relaxed(base + BECORE_C2SERV_TWS(n) +
					      BECORE_C2SERV_TWS_BUSY),
				readl_relaxed(base + BECORE_C2SERV_TWS(n) +
					      BECORE_C2SERV_TWS_FULLNESS),
				readl_relaxed(base + BECORE_C2SERV_TRS(n) +
					      BECORE_C2SERV_TRS_BUSY),
				readl_relaxed(base + BECORE_C2SERV_TRS(n) +
					      BECORE_C2SERV_TRS_LOST_CONNECTION));
	}
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

	if (first_error)
		becore_report_stuck(becore);

	return first_error;
}

static int becore_stream_crc_validate(struct device *dev)
{
	unsigned int id;

	for (id = 0; id < BECORE_NUM_BLOCKS; id++) {
		const struct becore_stream_crc *table;
		size_t count;
		size_t i;

		table = becore_stream_crc_table(id, &count);
		if (count > BECORE_STREAM_CRC_MAX)
			return dev_err_probe(dev, -EINVAL,
					     "block %u has %zu stream CRCs\n",
					     id, count);
		for (i = 0; i < count; i++) {
			if (table[i].offset & 3 ||
			    table[i].offset >= BECORE_BLOCK_WINDOW)
				return dev_err_probe(dev, -EINVAL,
						     "stream CRC %u:%zu is outside the block\n",
						     id, i);
			if (i && table[i].offset <= table[i - 1].offset)
				return dev_err_probe(dev, -EINVAL,
						     "stream CRC %u:%zu is out of order\n",
						     id, i);
		}
	}

	return 0;
}

/*
 * The VOTF token geometry, and the chain raster against it.  Checked at probe
 * rather than at link time because neither failure is a failed register write:
 * see becore_c2serv_tws_lines_in_token for what each of the two refusals
 * costs when it is not made, and for the captured links they come from.
 *
 * The debugfs overrides are deliberately not checked.  They exist to sweep
 * this geometry against what the fabric reports, and a sweep that cannot reach
 * a failing point cannot characterise the failure -- which is how these two
 * numbers were found in the first place.
 */
static int becore_c2serv_tokens_validate(struct device *dev)
{
	unsigned int n;

	for (n = 0; n < BECORE_C2SERV_LINK_PLANES; n++) {
		u32 tws = becore_c2serv_tws_lines_in_token[n];
		u32 trs = becore_c2serv_trs_lines_in_token[n];

		if (!tws || !trs || trs % tws)
			return dev_err_probe(dev, -EINVAL,
					     "VOTF plane %u gives the consumer %u lines to the producer's %u, which is not a whole number of tokens\n",
					     n, trs, tws);
	}

	return 0;
}

/*
 * And the raster against them, which is a property of the fabric and not of
 * the device: the memory path carries any height correctly, and a frame the
 * fabric cannot describe simply goes that way instead.  Refusing to probe over
 * it would deny the memory path, the offline oracle and the debugfs sweep for
 * a switch none of them uses.
 *
 * The overrides are not consulted, for the reason above them: a sweep has to
 * be able to reach a failing point, and this decides which path a frame takes
 * rather than what the fabric is told.
 */
static bool becore_c2serv_chain_fits(const struct becore_raster *chain)
{
	unsigned int n;

	for (n = 0; n < BECORE_C2SERV_LINK_PLANES; n++) {
		u32 tws = becore_c2serv_tws_lines_in_token[n];
		u32 lines = n ? DIV_ROUND_UP(chain->height, 2) : chain->height;

		if (!tws || lines % tws)
			return false;
	}

	return true;
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

/*
 * Nothing in mainline asks INTCAM for anything, so RGBP, YUVP and MCSC run a
 * 4000 x 3000 stream at the bottom rung of its ladder: 111 MHz, with clk_summary
 * reporting the domain "deviceless" for the whole of a capture [HW 2026-08-22].
 * The front end beside this one already votes for CAM and for memory bandwidth;
 * this is the back end's half of the same contract.
 *
 * The rate is raised rather than set, so a larger vote from anything else --
 * a future GDC, GSE or LME consumer, all of which share INTCAM -- survives,
 * and
 * the entry rate is restored rather than assumed, so a failed unwind cannot
 * quietly become the next stream's idle baseline.
 */
static int becore_qos_disable(struct becore_device *becore)
{
	int ret = 0;

	if (becore->intcam_rate_active) {
		ret = clk_set_rate(becore->intcam_clk,
				   becore->saved_intcam_rate);
		if (ret)
			return ret;
		becore->intcam_rate_active = false;
		becore->saved_intcam_rate = 0;
	}

	return 0;
}

static int becore_qos_enable(struct becore_device *becore)
{
	int cleanup_ret;
	int ret;

	/* Retry a restore the preceding stream left incomplete. */
	ret = becore_qos_disable(becore);
	if (ret)
		return ret;

	becore->saved_intcam_rate = clk_get_rate(becore->intcam_clk);
	if (!becore->saved_intcam_rate)
		return -EIO;

	/* A transport error can arrive after firmware accepted the request. */
	becore->intcam_rate_active = true;
	ret = clk_set_rate(becore->intcam_clk,
			   max(becore->saved_intcam_rate,
			       BECORE_INTCAM_ACTIVE_RATE));
	if (ret) {
		cleanup_ret = becore_qos_disable(becore);
		if (cleanup_ret)
			dev_err(becore->dev,
				"cannot restore INTCAM after error: %d\n",
				cleanup_ret);
		return ret;
	}

	return 0;
}

static void becore_qos_restore(struct becore_device *becore)
{
	int ret = becore_qos_disable(becore);

	if (ret)
		dev_err(becore->dev, "cannot restore INTCAM: %d\n", ret);
}

/*
 * A stream holds the block powered for its whole length rather than letting
 * each frame power it up and down.  becore_run_frame()'s own get and put then
 * only move the usage count, which never reaches zero, so no callback runs and
 * the four processors are reset once per stream instead of once per frame --
 * which is what Pablo does too, resetting in enable/disable rather than shot.
 *
 * Measured, the per-frame cycle was 1,228 us of resume and 93 us of suspend
 * against a frame of 31.6 ms.  That was noise when the frame was 340 ms; it is
 * 56% of the 2.3 ms that now separates the run from the sensor's period.
 *
 * The one-shot debugfs diagnostics keep powering up and down per run: they are
 * not a stream and nothing holds this reference for them.
 */
int becore_stream_power_get(struct becore_device *becore)
{
	int ret;

	if (WARN_ON_ONCE(becore->stream_powered))
		return -EBUSY;

	ret = pm_runtime_resume_and_get(becore->dev);
	if (ret)
		return ret;
	if (becore->reset_failed) {
		/*
		 * Drop the count without an idle notification, so the device
		 * stays powered rather than attempting a suspend whose reset
		 * has already failed.  reset_failed is sticky and refuses every
		 * entry point from here, so nothing runs on it again.
		 */
		pm_runtime_put_noidle(becore->dev);
		return -EIO;
	}
	becore->stream_powered = true;

	ret = becore_qos_enable(becore);
	if (ret) {
		becore->stream_powered = false;
		if (pm_runtime_put_sync(becore->dev) < 0)
			pm_runtime_get_noresume(becore->dev);
	}

	return ret;
}

void becore_stream_power_put(struct becore_device *becore)
{
	int ret;

	becore_qos_restore(becore);
	if (!becore->stream_powered)
		return;
	becore->stream_powered = false;

	ret = pm_runtime_put_sync(becore->dev);
	if (ret < 0) {
		/* Match probe: never leave a failed-reset device at usage zero. */
		pm_runtime_get_noresume(becore->dev);
		dev_err(becore->dev, "cannot power down after stream: %d\n", ret);
	}
}

/*
 * Both directions start by flushing every endpoint, consumers first, which is
 * what the vendor does on every instance regardless of how many endpoints it
 * actually has -- so all thirty-two of these writes are attested on this
 * hardware.  Flushing only the ones an instance has would be tidier and would
 * rest on the software reset clearing the rest, which Samsung's own register
 * notes attach to SW_CORE_RESET at +0x1c rather than to the SW_RESET at +0x18
 * that both this driver and the vendor write.  An unverifiable assumption is
 * not worth twenty-odd writes once per stream.
 */
static void becore_c2serv_flush(struct becore_device *becore,
				enum becore_c2serv_id id)
{
	void __iomem *base = becore->c2serv[id];
	unsigned int i;

	for (i = 0; i < BECORE_C2SERV_ENDPOINTS; i++)
		writel_relaxed(1, base + BECORE_C2SERV_TRS(i) +
				  BECORE_C2SERV_TRS_FLUSH);
	for (i = 0; i < BECORE_C2SERV_ENDPOINTS; i++)
		writel_relaxed(1, base + BECORE_C2SERV_TWS(i) +
				  BECORE_C2SERV_TWS_FLUSH);
}

/*
 * Sample every endpoint's enable bit, producer n in bit n and consumer n in
 * bit 16 + n.  Called once straight out of the software reset, which is the
 * only moment the answer says what the hardware's own defaults are.
 */
static u32 becore_c2serv_enables(struct becore_device *becore,
				 enum becore_c2serv_id id)
{
	void __iomem *base = becore->c2serv[id];
	unsigned int i;
	u32 mask = 0;

	for (i = 0; i < BECORE_C2SERV_ENDPOINTS; i++) {
		if (readl(base + BECORE_C2SERV_TWS(i) +
			       BECORE_C2SERV_TWS_ENABLE) & 1)
			mask |= BIT(i);
		if (readl(base + BECORE_C2SERV_TRS(i) +
			       BECORE_C2SERV_TRS_ENABLE) & 1)
			mask |= BIT(16 + i);
	}

	return mask;
}

/*
 * Turn every endpoint off.  A software reset does not leave a window quiet:
 * both enables reset to 1, so a freshly reset window has all thirty-two
 * endpoints live, each pointing at destination 0 with a default token of two
 * lines.  The vendor never has to care, because it programs a real link
 * before it ever starts the ring.  This driver prepares windows long before
 * it links them, so it has to make the prepared state quiet itself -- that is
 * what lets the ring be started later without every unused endpoint joining
 * in.
 */
static void becore_c2serv_disable_endpoints(struct becore_device *becore,
					    enum becore_c2serv_id id)
{
	void __iomem *base = becore->c2serv[id];
	unsigned int i;

	for (i = 0; i < BECORE_C2SERV_ENDPOINTS; i++) {
		writel_relaxed(0, base + BECORE_C2SERV_TWS(i) +
				  BECORE_C2SERV_TWS_ENABLE);
		writel_relaxed(0, base + BECORE_C2SERV_TRS(i) +
				  BECORE_C2SERV_TRS_ENABLE);
	}
}

static int becore_c2serv_reset(struct becore_device *becore,
			       enum becore_c2serv_id id)
{
	void __iomem *base = becore->c2serv[id];

	writel_relaxed(1, base + BECORE_C2SERV_SW_RESET);

	return becore_poll_reset(becore->dev, base + BECORE_C2SERV_SW_RESET,
				 becore_c2serv[id].name);
}

/*
 * Make one C2SERV window ready: flush every endpoint, software-reset the
 * window, select the register bank and let the window name itself.  The ring
 * is deliberately left stopped, and that is a hardware requirement rather
 * than caution.
 *
 * The YUVP program this driver runs is the vendor's, and the vendor captured
 * it while YUVP was a VOTF producer, so its combined WDMA still carries
 * VOTF_EN = 3.  Writing the frame to memory works only
 * because the fabric underneath is dead.  Start the ring with that bit still
 * set and no producer destination programmed, and the first frame faults:
 * a SysMMU write page fault on the YUVP domain at a stale address, a YUVP
 * that then will not reset, and a camera down until reboot [HW 2026-08-22].
 *
 * So RING_CLK_EN and RING_ENABLE belong with the link, alongside the
 * transport each endpoint's DMA is told to use, and not here.  The vendor
 * starts them right after SEL_REGISTER and stops them in the mirror order --
 * ring clock first, then the ring, then reset -- which is where they go when
 * there is something on the ring to carry.
 *
 * SEL_REGISTER selects the immediate bank over the shadowed one and
 * SEL_REGISTER_MODE makes a write take effect at once rather than at the next
 * shadow trigger.  Every endpoint register therefore has a shadow alias this
 * driver never uses, and a link is programmed by writing the plain offsets --
 * which is what both the vendor's stream and Samsung's own driver do.
 *
 * Whether the window answered is decided by the software reset clearing
 * itself, and by nothing else.  That is the only read of a C2SERV window the
 * vendor's stream makes, so it is the only one whose read-back behaviour is
 * known; the three registers sampled afterwards are recorded for debugfs and
 * deliberately do not gate anything, because a register that turns out to be
 * write-only would otherwise condemn hardware that is working.
 */
static void becore_c2serv_prepare(struct becore_device *becore,
				  enum becore_c2serv_id id)
{
	const struct becore_c2serv_desc *desc = &becore_c2serv[id];
	struct becore_c2serv_state *state = &becore->c2serv_state[id];
	void __iomem *base = becore->c2serv[id];

	becore_c2serv_flush(becore, id);

	if (becore_c2serv_reset(becore, id)) {
		memset(state, 0, sizeof(*state));
		return;
	}

	writel_relaxed(1, base + BECORE_C2SERV_SEL_REGISTER_MODE);
	writel_relaxed(1, base + BECORE_C2SERV_SEL_REGISTER);
	writel_relaxed(desc->local_ip, base + BECORE_C2SERV_LOCAL_IP);

	state->reset_enables = becore_c2serv_enables(becore, id);
	becore_c2serv_disable_endpoints(becore, id);

	state->ring_clk_en = readl(base + BECORE_C2SERV_RING_CLK_EN);
	state->ring_enable = readl(base + BECORE_C2SERV_RING_ENABLE);
	state->local_ip = readl(base + BECORE_C2SERV_LOCAL_IP);
	state->ready = true;
}

/*
 * What the fabric thinks of one endpoint's connection: 0 idle, 1 the consumer
 * waiting, 2 the producer waiting, 3 connected.  Selecting an endpoint is a
 * write, so this is not a passive read -- it is only done between runs and on
 * a link that is standing still.
 */
static u32 becore_c2serv_conn(struct becore_device *becore,
			      enum becore_c2serv_id id, unsigned int n,
			      bool consumer)
{
	void __iomem *base = becore->c2serv[id];
	u32 select = BECORE_C2SERV_DEBUG_ENABLE |
		     FIELD_PREP(BECORE_C2SERV_DEBUG_ENDPOINT, n);

	if (consumer)
		select |= BECORE_C2SERV_DEBUG_CONSUMER;

	writel(select, base + BECORE_C2SERV_DEBUG);

	return readl(base + BECORE_C2SERV_DEBUG_DOUT);
}

static void becore_c2serv_read_tws(struct becore_device *becore,
				   enum becore_c2serv_id id, unsigned int n,
				   struct becore_c2serv_tws_state *tws)
{
	void __iomem *ep = becore->c2serv[id] + BECORE_C2SERV_TWS(n);

	tws->conn_raw = becore_c2serv_conn(becore, id, n, false);
	tws->conn = FIELD_GET(BECORE_C2SERV_DEBUG_STATE, tws->conn_raw);
	tws->rcv_valid = readl(becore->c2serv[id] + BECORE_C2SERV_RCV_VALID);
	tws->enable = readl(ep + BECORE_C2SERV_TWS_ENABLE);
	tws->limit = readl(ep + BECORE_C2SERV_TWS_LIMIT);
	tws->dest = readl(ep + BECORE_C2SERV_TWS_DEST);
	tws->lines_in_token = readl(ep + BECORE_C2SERV_TWS_LINES_IN_TOKEN);
	tws->busy = readl(ep + BECORE_C2SERV_TWS_BUSY);
	tws->fullness = readl(ep + BECORE_C2SERV_TWS_FULLNESS);
}

static void becore_c2serv_read_trs(struct becore_device *becore,
				   enum becore_c2serv_id id, unsigned int n,
				   struct becore_c2serv_trs_state *trs)
{
	void __iomem *ep = becore->c2serv[id] + BECORE_C2SERV_TRS(n);

	trs->conn_raw = becore_c2serv_conn(becore, id, n, true);
	trs->conn = FIELD_GET(BECORE_C2SERV_DEBUG_STATE, trs->conn_raw);
	trs->rcv_valid = readl(becore->c2serv[id] + BECORE_C2SERV_RCV_VALID);
	trs->enable = readl(ep + BECORE_C2SERV_TRS_ENABLE);
	trs->limit = readl(ep + BECORE_C2SERV_TRS_LIMIT);
	trs->lines_in_first_token =
		readl(ep + BECORE_C2SERV_TRS_LINES_IN_FIRST_TOKEN);
	trs->lines_in_token = readl(ep + BECORE_C2SERV_TRS_LINES_IN_TOKEN);
	trs->lines_count = readl(ep + BECORE_C2SERV_TRS_LINES_COUNT);
	trs->busy = readl(ep + BECORE_C2SERV_TRS_BUSY);
	trs->lost_connection = readl(ep + BECORE_C2SERV_TRS_LOST_CONNECTION);
}

static void becore_c2serv_link_sample(struct becore_device *becore,
				      struct becore_c2serv_link_state *link);

/*
 * Light the link up, and put it out again.  Order matters at the moment it
 * goes live: geometry first, then the ring, then the four endpoints, and the
 * consumer's ring before the producer's so the receiver is listening before
 * anything can be sent.  The vendor never has to choose, because its ring is
 * up from stream setup and only the enables move per frame; this driver
 * cannot do that, because a live ring with YUVP's WDMA asking for VOTF and
 * nothing to answer it faults the frame.
 *
 * Stopping happens after every run whether the run worked or not, and stops
 * the ring in the same order it started it, clock before ring, which is what
 * the vendor's teardown does.  The vendor leaves a link standing between frames and only
 * clears the latched lost-connection bit; this takes it down each time, so
 * that the fabric is quiet whenever a run is not in progress and a failure
 * cannot leak into the next frame or into a run that is not using it.  That
 * is worth the dozen writes until the link is trusted.
 */
static void becore_c2serv_link_start(struct becore_device *becore)
{
	void __iomem *tws_base = becore->c2serv[BECORE_C2SERV_YUVP];
	void __iomem *trs_base = becore->c2serv[BECORE_C2SERV_MCSC];
	unsigned int n;

	/*
	 * Ring first, endpoints second, consumer before producer.  This is the
	 * vendor's steady state -- its ring is up from stream setup and only
	 * the enables move per frame -- and the order is load-bearing: an
	 * endpoint enabled onto a stopped ring never exchanges the credit that
	 * lets the producer emit its first token, and both sides then wait for
	 * ever.  Enabling onto a running ring is safe here because no endpoint
	 * carries a destination until the link above programmed one, and
	 * because the blocks have not been started yet.
	 */
	writel_relaxed(1, trs_base + BECORE_C2SERV_RING_CLK_EN);
	writel_relaxed(1, trs_base + BECORE_C2SERV_RING_ENABLE);
	writel_relaxed(1, tws_base + BECORE_C2SERV_RING_CLK_EN);
	writel_relaxed(1, tws_base + BECORE_C2SERV_RING_ENABLE);

	for (n = 0; n < BECORE_C2SERV_LINK_PLANES; n++) {
		writel_relaxed(1, trs_base + BECORE_C2SERV_TRS(n) +
				  BECORE_C2SERV_TRS_ENABLE);
		writel_relaxed(1, tws_base + BECORE_C2SERV_TWS(n) +
				  BECORE_C2SERV_TWS_ENABLE);
	}

	/*
	 * Sample the link as armed, before either block has been started.  A
	 * connection that is already made here and one that is still waiting
	 * are different problems, and after a run has timed out there is no
	 * way left to tell them apart.
	 */
	becore_c2serv_link_sample(becore, &becore->c2serv_armed);
}

/*
 * Wait for one VOTF endpoint to stop being busy after a flush.
 *
 * The bound is Lyric's own for a reset poll -- 20 ms, most of a frame -- and a
 * timeout is a warning rather than an error: the teardown continues either
 * way, and an endpoint that will not go idle produces a failed reset
 * downstream that says so on its own.
 */
static void becore_c2serv_wait_idle(struct becore_device *becore,
				    void __iomem *busy, const char *window,
				    const char *what, unsigned int plane)
{
	u32 value;

	if (readl_poll_timeout(busy, value, !value, 1, BECORE_C2SERV_FLUSH_US))
		dev_warn(becore->dev,
			 "%s %s plane %u still busy %#x after its flush\n",
			 window, what, plane, value);
}

static void becore_c2serv_link_stop(struct becore_device *becore, bool failed)
{
	void __iomem *tws_base = becore->c2serv[BECORE_C2SERV_YUVP];
	void __iomem *trs_base = becore->c2serv[BECORE_C2SERV_MCSC];
	unsigned int n;

	/*
	 * Flush before anything else, which is the vendor's order and matters
	 * for a stalled link: a flush is what abandons the tokens in flight,
	 * and taking the ring away first leaves both blocks waiting on a
	 * fabric that has stopped answering -- after which neither they nor
	 * their C2SERV windows will reset, and the device is out of use until
	 * the next boot.
	 */
	for (n = 0; n < BECORE_C2SERV_LINK_PLANES; n++) {
		writel_relaxed(1, trs_base + BECORE_C2SERV_TRS(n) +
				  BECORE_C2SERV_TRS_FLUSH);
		writel_relaxed(1, tws_base + BECORE_C2SERV_TWS(n) +
				  BECORE_C2SERV_TWS_FLUSH);
	}

	/*
	 * A flush is a request, not an act, and this waits for it.
	 *
	 * Every endpoint carries a `busy` bit, and after a stream killed
	 * mid-frame both ends of this link report it set with the ring empty --
	 * a producer holding an unfinished token and a consumer waiting for one
	 * that will never come.  Writing flush and moving straight on to take
	 * the enables and the ring away left them there, and neither the
	 * windows nor the two processors whose FIFOs feed them would then
	 * complete a software reset: the device was quarantined until reboot.
	 *
	 * So the endpoints are given the same bound Lyric gives its own resets
	 * to go idle before anything else is touched.  One that does not is
	 * reported rather than waited on forever, because the teardown has to
	 * finish either way.
	 */
	for (n = 0; n < BECORE_C2SERV_LINK_PLANES; n++) {
		becore_c2serv_wait_idle(becore, tws_base + BECORE_C2SERV_TWS(n) +
					BECORE_C2SERV_TWS_BUSY,
					becore_c2serv[BECORE_C2SERV_YUVP].name,
					"tws", n);
		becore_c2serv_wait_idle(becore, trs_base + BECORE_C2SERV_TRS(n) +
					BECORE_C2SERV_TRS_BUSY,
					becore_c2serv[BECORE_C2SERV_MCSC].name,
					"trs", n);
	}

	for (n = 0; n < BECORE_C2SERV_LINK_PLANES; n++) {
		writel_relaxed(0, tws_base + BECORE_C2SERV_TWS(n) +
				  BECORE_C2SERV_TWS_ENABLE);
		writel_relaxed(0, trs_base + BECORE_C2SERV_TRS(n) +
				  BECORE_C2SERV_TRS_ENABLE);
	}

	writel_relaxed(0, tws_base + BECORE_C2SERV_RING_CLK_EN);
	writel_relaxed(0, tws_base + BECORE_C2SERV_RING_ENABLE);
	writel_relaxed(0, trs_base + BECORE_C2SERV_RING_CLK_EN);
	writel_relaxed(0, trs_base + BECORE_C2SERV_RING_ENABLE);

	/*
	 * A run that failed may have left an endpoint mid-token, and a flush
	 * is only documented to empty the DMA.  Reset both windows so that
	 * whatever went wrong cannot be inherited by the next frame -- there
	 * is no resume between the frames of a stream to do it for us.  The
	 * blocks themselves are reset by the failure path that follows.
	 */
	if (failed) {
		becore_c2serv_reset(becore, BECORE_C2SERV_YUVP);
		becore_c2serv_reset(becore, BECORE_C2SERV_MCSC);
		becore_c2serv_disable_endpoints(becore, BECORE_C2SERV_YUVP);
		becore_c2serv_disable_endpoints(becore, BECORE_C2SERV_MCSC);
	}
}

/* What the fabric had to say, sampled while the link is still standing. */
static void becore_c2serv_link_sample(struct becore_device *becore,
				      struct becore_c2serv_link_state *link)
{
	unsigned int n;

	for (n = 0; n < BECORE_C2SERV_LINK_PLANES; n++) {
		becore_c2serv_read_tws(becore, BECORE_C2SERV_YUVP, n,
				       &link->tws[n]);
		becore_c2serv_read_trs(becore, BECORE_C2SERV_MCSC, n,
				       &link->trs[n]);
	}
	link->sampled = true;
}

/*
 * Describe the YUVP-to-MCSC link in the fabric, one endpoint pair per plane,
 * in the order and with the interleaved wrapper write the vendor uses for
 * every captured connection.  This programs the link but does not arm it --
 * the endpoint enables and the ring are becore_c2serv_link_start()'s -- and
 * it runs only for a frame that is going to use the fabric, so the memory
 * path's register writes are exactly what they were before there was a
 * fabric at all.
 *
 * The line count is the plane's height, as in every captured link, and it is
 * the chain raster's -- the same number the blocks either side are programmed
 * with, so the fabric and they cannot disagree about the frame.  It runs per
 * frame because that is where the vendor programs a link, and because the
 * things it depends on are resolved per run.
 *
 * The token geometry beside it does not move with the height, and does not
 * have to: it says how the producer packets a plane, not how tall the plane
 * is.  A height it cannot packet does not reach here at all --
 * becore_c2serv_chain_fits() sends that frame through memory.
 */
static void becore_c2serv_program_link(struct becore_device *becore)
{
	struct becore_c2serv_state *producer =
		&becore->c2serv_state[BECORE_C2SERV_YUVP];
	struct becore_c2serv_state *consumer =
		&becore->c2serv_state[BECORE_C2SERV_MCSC];
	void __iomem *tws_base = becore->c2serv[BECORE_C2SERV_YUVP];
	void __iomem *trs_base = becore->c2serv[BECORE_C2SERV_MCSC];
	u16 consumer_ip = becore_c2serv[BECORE_C2SERV_MCSC].local_ip;
	u32 tws_limit = becore_c2serv_override(becore->votf_tws_limit,
					       BECORE_C2SERV_LIMIT);
	u32 trs_limit = becore_c2serv_override(becore->votf_trs_limit,
					       BECORE_C2SERV_LIMIT);
	unsigned int n;

	if (!producer->ready || !consumer->ready)
		return;

	for (n = 0; n < BECORE_C2SERV_LINK_PLANES; n++) {
		void __iomem *tws = tws_base + BECORE_C2SERV_TWS(n);
		void __iomem *trs = trs_base + BECORE_C2SERV_TRS(n);
		u32 lines = n ? DIV_ROUND_UP(becore->chain.height, 2) :
				becore->chain.height;

		u32 tws_token = becore_c2serv_token(becore->votf_tws_token,
					becore_c2serv_tws_lines_in_token, n);
		u32 trs_token = becore_c2serv_token(becore->votf_trs_token,
					becore_c2serv_trs_lines_in_token, n);

		writel_relaxed(tws_limit, tws + BECORE_C2SERV_TWS_LIMIT);
		writel_relaxed(BECORE_C2SERV_WRAPPER_CONNECT,
			       tws_base + BECORE_C2SERV_WRAPPER);
		writel_relaxed(BECORE_C2SERV_DEST(consumer_ip, n),
			       tws + BECORE_C2SERV_TWS_DEST);
		writel_relaxed(tws_token,
			       tws + BECORE_C2SERV_TWS_LINES_IN_TOKEN);

		/*
		 * The consumer half opens by clearing the latched
		 * lost-connection bit and writing the wrapper, in that order,
		 * which is where the vendor puts both.
		 */
		writel_relaxed(1, trs + BECORE_C2SERV_TRS_LOST_CONNECTION);
		writel_relaxed(BECORE_C2SERV_WRAPPER_CONNECT,
			       trs_base + BECORE_C2SERV_WRAPPER);
		writel_relaxed(trs_limit, trs + BECORE_C2SERV_TRS_LIMIT);
		writel_relaxed(trs_token,
			       trs + BECORE_C2SERV_TRS_LINES_IN_FIRST_TOKEN);
		writel_relaxed(trs_token,
			       trs + BECORE_C2SERV_TRS_LINES_IN_TOKEN);
		writel_relaxed(lines, trs + BECORE_C2SERV_TRS_LINES_COUNT);
	}

}

/*
 * And put it back: flush, then reset.  Only a window that came up is touched,
 * so a resume that gave up before reaching the fabric is not followed by
 * writes to blocks that just failed to answer a reset.
 */
static void becore_c2serv_unprepare(struct becore_device *becore)
{
	unsigned int i;

	for (i = 0; i < BECORE_NUM_C2SERV; i++) {
		if (!becore->c2serv_state[i].ready)
			continue;

		becore_c2serv_flush(becore, i);
		becore_c2serv_reset(becore, i);
		memset(&becore->c2serv_state[i], 0,
		       sizeof(becore->c2serv_state[i]));
	}
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
	becore_write_table(&becore->blocks[BECORE_MCSC], becore_mcsc_init,
			   ARRAY_SIZE(becore_mcsc_init));

	for (i = 0; i < BECORE_NUM_C2SERV; i++)
		becore_c2serv_prepare(becore, i);

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

	becore_c2serv_unprepare(becore);

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
	unsigned int i;
	bool complete_run = false;

	if (id == BECORE_MCFP)
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
	if (!(becore->expected_mask & BIT(id)))
		goto unlock;

	if (status & BECORE_INT_CMDQ_HOLD) {
		becore->cmdq_hold_mask |= BIT(id);
		if ((becore->cmdq_hold_mask & becore->expected_mask) ==
		    becore->expected_mask &&
		    !becore->start_issued && !becore->abort_run) {
			becore->start_issued = true;
			/* Release the downstream end of the chain first. */
			for (i = BECORE_NUM_BLOCKS; i-- > 0;)
				if (becore->expected_mask & BIT(i))
					writel(1, becore->blocks[i].base +
					       BECORE_CMDQ_ADD_TO_QUEUE_0);
		}
	}
	if (status & BECORE_INT_FRAME_END) {
		becore->frame_done_mask |= BIT(id);
		if ((becore->frame_done_mask & becore->expected_mask) ==
		    becore->expected_mask)
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
	static const char * const block_names[] = {
		"rgbp", "mcfp", "yuvp", "mcsc",
	};
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

	becore->sysreg_mcsc =
		devm_platform_ioremap_resource_byname(pdev, "sysreg-mcsc");
	if (IS_ERR(becore->sysreg_mcsc))
		return dev_err_probe(dev, PTR_ERR(becore->sysreg_mcsc),
				     "cannot map sysreg-mcsc\n");

	/*
	 * A C2SERV window's local IP ID is the top sixteen bits of its own
	 * base address, so the ID the vendor's hardware-parameter table gives
	 * and the address the device tree gives have to agree.  Checking that
	 * here is what proves the window is the one it is believed to be --
	 * writing the ID and reading it back cannot, because the ID is a
	 * constant this driver already holds.
	 */
	for (i = 0; i < BECORE_NUM_C2SERV; i++) {
		const struct becore_c2serv_desc *desc = &becore_c2serv[i];
		struct resource *res;

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   desc->name);
		if (!res)
			return dev_err_probe(dev, -ENODEV, "no %s\n",
					     desc->name);

		if (res->start >> 16 != desc->local_ip)
			return dev_err_probe(dev, -EINVAL,
					     "%s is at %pa, which is IP %#06llx and not %#06x\n",
					     desc->name, &res->start,
					     (u64)res->start >> 16,
					     desc->local_ip);

		becore->c2serv[i] = devm_ioremap_resource(dev, res);
		if (IS_ERR(becore->c2serv[i]))
			return dev_err_probe(dev, PTR_ERR(becore->c2serv[i]),
					     "cannot map %s\n", desc->name);
	}

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

static void becore_free_dma_buffer(struct becore_device *becore,
				   struct becore_dma_buffer *buffer)
{
	if (buffer->cpu)
		dma_free_coherent(becore->dev, buffer->size, buffer->cpu,
				  buffer->dma);
	buffer->cpu = NULL;
	buffer->dma = 0;
	buffer->size = 0;
	buffer->staged_bytes = 0;
}

/*
 * Not dmam_alloc_coherent(), and the reason is the size rather than the
 * lifetime: three of these are sized from a raster, and devres can only
 * release an allocation once, at unbind.  An explicit pair can run again when
 * the raster it was sized from moves.
 *
 * What devres also took care of is that a buffer was allocated once.  Nothing
 * enforces that now except the callers, so say it here: allocating over a live
 * buffer would leak tens of megabytes with no other symptom.
 */
static int becore_alloc_dma_buffer(struct becore_device *becore,
				   struct becore_dma_buffer *buffer,
				   size_t size, const char *name)
{
	if (WARN_ON_ONCE(buffer->cpu))
		return -EEXIST;
	buffer->cpu = dma_alloc_coherent(becore->dev, size, &buffer->dma,
					 GFP_KERNEL);
	if (!buffer->cpu)
		return dev_err_probe(becore->dev, -ENOMEM,
				     "cannot allocate %s buffer\n", name);
	buffer->size = size;
	if (upper_32_bits(buffer->dma) ||
	    upper_32_bits(buffer->dma + buffer->size - 1)) {
		becore_free_dma_buffer(becore, buffer);
		return dev_err_probe(becore->dev, -ERANGE,
				     "%s buffer is outside 32-bit DMA\n", name);
	}

	return 0;
}

/*
 * The neutral grid, and the reason the guide curve's identity ramp above is a
 * measured no-op rather than a hopeful one: with every cell at unity gain and
 * zero bias, the curve has nothing to modulate. The two are written by
 * different code and coupled only by that fact, so anything that makes this
 * grid non-neutral makes the ramp a live tone curve in the same breath.
 */
static int becore_ltm_grid_generate(struct becore_device *becore)
{
	struct becore_dma_buffer *grid = &becore->grid;
	u32 row, column, group, channel;

	static_assert(sizeof(struct becore_ltm_grid_cell) ==
		      BECORE_LTM_GRID_CELL_BYTES);
	if (!grid->cpu || grid->size != BECORE_GRID_SIZE ||
	    BECORE_LTM_GRID_WIDTH_CELLS * BECORE_LTM_GRID_CELL_BYTES >
		BECORE_LTM_GRID_ROW_BYTES ||
	    BECORE_LTM_GRID_HEIGHT_CELLS > BECORE_LTM_GRID_ROWS)
		return -EINVAL;

	/*
	 * Lyric's LTM translator stores four Q14 gains followed by four signed
	 * offsets in each 16-byte group.  Unity gains and zero offsets provide a
	 * neutral policy surface; inactive cells and physical-row padding stay 0.
	 */
	memset(grid->cpu, 0, grid->size);
	for (row = 0; row < BECORE_LTM_GRID_HEIGHT_CELLS; row++) {
		u8 *row_base = (u8 *)grid->cpu +
			       row * BECORE_LTM_GRID_ROW_BYTES;

		for (column = 0; column < BECORE_LTM_GRID_WIDTH_CELLS;
		     column++) {
			struct becore_ltm_grid_cell *cell =
				(void *)(row_base +
					 column * BECORE_LTM_GRID_CELL_BYTES);

			for (group = 0; group < ARRAY_SIZE(cell->groups); group++)
				for (channel = 0;
				     channel < ARRAY_SIZE(cell->groups[group].gain);
				     channel++)
					cell->groups[group].gain[channel] =
						cpu_to_le16(BECORE_LTM_UNITY_Q14);
		}
	}

	grid->staged_bytes = grid->size;
	becore->grid_generation = 1;

	return 0;
}

static void becore_free_shared_input(struct becore_device *becore)
{
	unsigned int i;

	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_dma_buffer *input = &becore->inputs[i].buffer;

		if (input->cpu)
			dma_vunmap_noncontiguous(becore->dev, input->cpu);
		if (input->sgt)
			dma_free_noncontiguous(becore->dev, input->size,
					       input->sgt, DMA_BIDIRECTIONAL);
		input->cpu = NULL;
		input->sgt = NULL;
		input->dma = 0;
		input->size = 0;
		input->staged_bytes = 0;
	}
}

static int becore_alloc_shared_input(struct becore_device *becore)
{
	unsigned int i;
	int ret;

	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_dma_buffer *input = &becore->inputs[i].buffer;

		input->size = becore_input_allocation_size(&becore->array);
		input->sgt = dma_alloc_noncontiguous(becore->dev, input->size,
						     DMA_BIDIRECTIONAL,
						     GFP_KERNEL, 0);
		if (!input->sgt) {
			ret = -ENOMEM;
			goto err_free;
		}

		input->dma = sg_dma_address(input->sgt->sgl);
		input->cpu = dma_vmap_noncontiguous(becore->dev, input->size,
						    input->sgt);
		if (!input->cpu) {
			ret = -ENOMEM;
			goto err_free;
		}
		if (upper_32_bits(input->dma) ||
		    upper_32_bits(input->dma + input->size - 1)) {
			ret = -ERANGE;
			goto err_free;
		}
	}

	return 0;

err_free:
	becore_free_shared_input(becore);
	return dev_err_probe(becore->dev, ret,
			     "cannot allocate Bayer input slot %u\n", i);
}

/*
 * The three surfaces the chain and the scaled output size, allocated and freed
 * together because they move together: none of them survives a raster change,
 * and each is written by a DMA programmed from the same numbers.
 *
 * The input slots are deliberately not here.  Their size is a contract with
 * the producer -- ISPFE checks the size it is handed at its own probe and then
 * holds mappings of these exact pages in its own IOMMU domain -- so they are
 * sized once and a raster they cannot hold is refused rather than resized.
 */
void becore_free_surfaces(struct becore_device *becore)
{
	becore_free_dma_buffer(becore, &becore->mcsc_output);
	becore_free_dma_buffer(becore, &becore->gtnr_output);
	becore_free_dma_buffer(becore, &becore->output);
	becore->mcsc_dest_dma = 0;
	becore->active_output_dma = 0;
	becore->active_output_size = 0;
	becore->active_capture_size = 0;
	/*
	 * What a run left in those surfaces went with them, and the two
	 * debugfs readers are gated on these rather than on a pointer: a
	 * completed length surviving the buffer it described would read the
	 * next allocation at the previous one's length.
	 */
	becore->completed_generation = 0;
	becore->completed_output_size = 0;
	becore->mcsc_completed_generation = 0;
	becore->mcsc_completed_output_size = 0;
	/*
	 * And so did the encoded programs, which carry these surfaces'
	 * addresses.  The RGBP and YUVP readers gate on completed_generation
	 * above; GTNR's and MCSC's have freshness counters of their own, and a
	 * program that names a freed buffer must not read back as current --
	 * these files are what says what the driver encoded.
	 */
	becore->gtnr_encoded_generation = 0;
	becore->mcsc_encoded_generation = 0;
}

int becore_alloc_surfaces(struct becore_device *becore)
{
	int ret;

	ret = becore_alloc_dma_buffer(becore, &becore->output,
				      becore_yuvp_output_allocation_size(&becore->chain),
				      "YUVP output");
	if (ret)
		return ret;
	ret = becore_alloc_dma_buffer(becore, &becore->gtnr_output,
				      becore_gtnr_surface_size(&becore->chain),
				      "GTNR output");
	if (ret)
		return ret;
	ret = becore_alloc_dma_buffer(becore, &becore->mcsc_output,
				      becore_mcsc_output_size(&becore->scaled),
				      "MCSC output");
	if (ret)
		return ret;

	/*
	 * What a run has produced, reported until the first one replaces it.
	 * Derived from the same two rasters the surfaces were just sized from,
	 * so it belongs with them rather than with the caller.
	 */
	becore->mcsc_dest_dma = becore->mcsc_output.dma;
	becore->active_output_dma = becore->output.dma;
	becore->active_output_size = becore_active_output_size(becore);
	becore->active_capture_size =
		becore_mcsc_output_active_size(&becore->scaled);

	return 0;
}

/*
 * One teardown for everything becore_alloc_dma_buffer() and the input slots
 * produced, registered before the first of them so a partial failure unwinds
 * through the same path.  Every free below tolerates a buffer that was never
 * allocated.
 */
static void becore_free_buffers(void *data)
{
	struct becore_device *becore = data;

	becore_free_surfaces(becore);
	becore_free_dma_buffer(becore, &becore->grid);
	becore->grid_generation = 0;
	becore_free_shared_input(becore);
}

static int becore_alloc_cmdq_buffer(struct becore_device *becore,
				    struct becore_cmdq_program *program,
				    u32 header_count, const char *name)
{
	program->capacity = header_count;
	program->size = becore_cmdq_program_size(header_count);
	program->cpu = dmam_alloc_coherent(becore->dev, program->size,
					   &program->dma, GFP_KERNEL);
	if (!program->cpu)
		return dev_err_probe(becore->dev, -ENOMEM,
				     "cannot allocate %s CMDQ program\n",
				     name);
	if (upper_32_bits(program->dma) ||
	    upper_32_bits(program->dma + program->size - 1))
		return dev_err_probe(becore->dev, -ERANGE,
				     "%s CMDQ program is outside 32-bit DMA\n",
				     name);

	return 0;
}

static int becore_alloc_cmdq_program(struct becore_device *becore,
				     enum becore_block_id id,
				     u32 header_count)
{
	return becore_alloc_cmdq_buffer(becore, &becore->program[id],
					header_count, becore->blocks[id].name);
}

static int becore_alloc_diagnostic(struct becore_device *becore)
{
	int ret;

	ret = becore_yuvnr_table_validate(becore->dev);
	if (ret)
		return ret;

	ret = becore_dmsc_table_validate(becore->dev);
	if (ret)
		return ret;

	ret = becore_byr_dns_table_validate(becore->dev);
	if (ret)
		return ret;

	ret = becore_sharpen_table_validate(becore->dev);
	if (ret)
		return ret;

	ret = becore_generated_tables_validate(becore->dev);
	if (ret)
		return ret;
	ret = becore_raster_validate(becore->dev, "array", &becore->array,
				     BECORE_ARRAY_EXTENT_MAX);
	if (ret)
		return ret;
	ret = becore_chain_validate(becore->dev, &becore->chain);
	if (ret)
		return ret;
	ret = becore_c2serv_tokens_validate(becore->dev);
	if (ret)
		return ret;
	ret = becore_raster_validate(becore->dev, "scaled", &becore->scaled,
				     BECORE_RASTER_EXTENT_MAX);
	if (ret)
		return ret;
	ret = becore_input_profiles_validate(becore->dev, &becore->array);
	if (ret)
		return ret;
	ret = becore_output_profiles_validate(becore->dev);
	if (ret)
		return ret;
	ret = becore_stream_crc_validate(becore->dev);
	if (ret)
		return ret;
	ret = becore_noise_knots_resolve(becore->dev);
	if (ret)
		return ret;
	becore->recipe = devm_kzalloc(becore->dev, BECORE_RECIPE_BYTES,
				      GFP_KERNEL);
	if (!becore->recipe)
		return -ENOMEM;
	ret = becore_recipe_generate(becore);
	if (ret)
		return dev_err_probe(becore->dev, ret,
				     "invalid built-in RGBP/YUVP recipe\n");
	becore->gtnr_recipe = devm_kzalloc(becore->dev,
					   BECORE_GTNR_RECIPE_BYTES, GFP_KERNEL);
	if (!becore->gtnr_recipe)
		return -ENOMEM;
	becore->mcsc_recipe = devm_kzalloc(becore->dev,
					   BECORE_MCSC_RECIPE_BYTES, GFP_KERNEL);
	if (!becore->mcsc_recipe)
		return -ENOMEM;
	ret = becore_mcsc_recipe_generate(becore);
	if (ret)
		return dev_err_probe(becore->dev, ret,
				     "invalid built-in MCSC recipe\n");

	ret = devm_add_action_or_reset(becore->dev, becore_free_buffers,
				       becore);
	if (ret)
		return ret;
	ret = becore_alloc_shared_input(becore);
	if (ret)
		return ret;
	ret = becore_alloc_dma_buffer(becore, &becore->grid,
				      BECORE_GRID_SIZE, "YUVP grid");
	if (ret)
		return ret;
	ret = becore_ltm_grid_generate(becore);
	if (ret)
		return dev_err_probe(becore->dev, ret,
				     "cannot generate neutral YUVP grid\n");
	ret = becore_alloc_surfaces(becore);
	if (ret)
		return ret;
	ret = becore_alloc_cmdq_program(becore, BECORE_RGBP,
					BECORE_RGBP_HEADER_COUNT);
	if (ret)
		return ret;

	/*
	 * Sized for the longest YUVP program, which is the recipe plus the
	 * colour LUT's burst.  What is encoded is shorter whenever userspace
	 * has sent no lattice, which is the ordinary case.
	 */
	ret = becore_alloc_cmdq_program(becore, BECORE_YUVP,
					BECORE_YUVP_PROGRAM_HEADERS);
	if (ret)
		return ret;
	ret = becore_alloc_cmdq_buffer(becore, &becore->gtnr_program,
				       BECORE_GTNR_HEADER_COUNT, "GTNR startup");
	if (ret)
		return ret;

	return becore_alloc_cmdq_buffer(becore, &becore->mcsc_program,
					BECORE_MCSC_HEADER_COUNT, "MCSC");
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
 * @ops: producer lifetime operations for an ordinary processed stream
 * @producer_data: producer-private callback argument
 *
 * Each producer_acquire() returns one generation-tagged slot and its address
 * in @producer's DMA domain.  BE-core retains distinct mappings of the same
 * pages and never exposes those IOVAs to the producer.
 */
struct exynos_becore_input *
exynos_becore_input_map(struct device *backend, struct device *producer,
			const struct exynos_becore_input_producer_ops *ops,
			void *producer_data)
{
	struct becore_device *becore;
	struct exynos_becore_input *input;
	unsigned int i;
	int ret;

	if (!backend || !producer || !ops || !ops->start_streaming ||
	    !ops->stop_streaming)
		return ERR_PTR(-EINVAL);
	becore = dev_get_drvdata(backend);
	if (!becore || !becore->inputs[0].buffer.sgt)
		return ERR_PTR(-EPROBE_DEFER);

	input = kzalloc_obj(*input, GFP_KERNEL);
	if (!input)
		return ERR_PTR(-ENOMEM);
	input->becore = becore;
	input->producer = get_device(producer);
	input->ops = ops;
	input->producer_data = producer_data;
	refcount_set(&input->callback_users, 1);
	init_waitqueue_head(&input->callback_wait);

	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_dma_buffer *slot = &becore->inputs[i].buffer;

		ret = becore_clone_sgtable(&input->sgts[i], slot->sgt);
		if (ret)
			goto err_mappings;
		ret = dma_map_sgtable(producer, &input->sgts[i],
				      DMA_FROM_DEVICE, 0);
		if (ret) {
			sg_free_table(&input->sgts[i]);
			goto err_mappings;
		}
		input->dmas[i] = sg_dma_address(input->sgts[i].sgl);
		if (input->sgts[i].nents != 1 ||
		    sg_dma_len(input->sgts[i].sgl) < slot->size ||
		    upper_32_bits(input->dmas[i]) ||
		    upper_32_bits(input->dmas[i] + slot->size - 1)) {
			ret = -ERANGE;
			dma_unmap_sgtable(producer, &input->sgts[i],
					  DMA_FROM_DEVICE, 0);
			sg_free_table(&input->sgts[i]);
			goto err_mappings;
		}
	}
	mutex_lock(&becore->lock);
	/*
	 * Two separate refusals. Probe returns success on two paths that never
	 * reach the video setup -- a processor that would not reset, and one
	 * that would not power down -- and on both of them there is no
	 * subdevice and no node here for a producer to put on its graph. And
	 * reset_failed is not probe-only: a runtime suspend or a run teardown
	 * can set it long afterwards, which a producer that has not attached
	 * yet must still be refused for.
	 */
	if (!becore->video_ready || becore->reset_failed) {
		ret = -EIO;
		mutex_unlock(&becore->lock);
		goto err_mappings;
	}
	if (becore->input_producer) {
		ret = -EBUSY;
		mutex_unlock(&becore->lock);
		goto err_mappings;
	}
	becore->input_producer = input;
	mutex_unlock(&becore->lock);

	return input;

err_mappings:
	while (i--) {
		dma_unmap_sgtable(producer, &input->sgts[i],
				  DMA_FROM_DEVICE, 0);
		sg_free_table(&input->sgts[i]);
	}
	put_device(input->producer);
	kfree(input);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(exynos_becore_input_map);

/**
 * exynos_becore_input_disconnect() - sever and join the producer callbacks
 * @input: attachment returned by exynos_becore_input_map()
 *
 * No callback can begin after this returns.  An ordinary BE-core stream is
 * cancelled and errored, but the caller must still quiesce its DMA before
 * exynos_becore_input_unmap() releases or retains the shared mappings.
 */
void exynos_becore_input_disconnect(struct exynos_becore_input *input)
{
	struct becore_device *becore;
	unsigned long flags;
	bool cancel = false;
	bool streaming;

	if (!input)
		return;
	becore = input->becore;

	mutex_lock(&becore->lock);
	if (input->disconnected) {
		mutex_unlock(&becore->lock);
		wait_event(input->callback_wait,
			   refcount_read(&input->callback_users) == 1);
		return;
	}
	if (WARN_ON_ONCE(becore->input_producer != input)) {
		mutex_unlock(&becore->lock);
		return;
	}

	input->disconnected = true;
	becore->input_producer = NULL;
	streaming = becore->video_streaming;
	becore->video_streaming = false;
	becore->producer_streaming = false;
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

	/*
	 * Powered down only once the run in flight has been abandoned and the
	 * worker has joined, which is the order becore_stop_streaming() uses.
	 * A suspend resets all four processors and both C2SERV instances, and
	 * asking a chain that is still mid-frame to reset is not a thing to do
	 * on purpose.
	 */
	mutex_lock(&becore->lock);
	becore_stream_power_put(becore);
	mutex_unlock(&becore->lock);

	if (streaming) {
		becore_video_controls_ungrab(becore);
		vb2_queue_error(&becore->queue);
		becore_video_return_all(becore, VB2_BUF_STATE_ERROR);
	}
	wait_event(input->callback_wait,
		   refcount_read(&input->callback_users) == 1);
}
EXPORT_SYMBOL_GPL(exynos_becore_input_disconnect);

struct exynos_becore_input *
becore_input_callback_get(struct becore_device *becore)
{
	struct exynos_becore_input *input = becore->input_producer;

	lockdep_assert_held(&becore->lock);
	if (!input || input->disconnected)
		return NULL;
	refcount_inc(&input->callback_users);

	return input;
}

void becore_input_callback_put(struct exynos_becore_input *input)
{
	refcount_dec(&input->callback_users);
	wake_up_all(&input->callback_wait);
}

void exynos_becore_input_unmap(struct exynos_becore_input *input)
{
	struct becore_device *becore;
	unsigned int i;

	if (!input)
		return;
	becore = input->becore;
	exynos_becore_input_disconnect(input);

	mutex_lock(&becore->lock);
	if (WARN_ON_ONCE(!input->disconnected || becore->input_producer)) {
		mutex_unlock(&becore->lock);
		return;
	}
	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		if (becore->inputs[i].state == BECORE_INPUT_BACKEND ||
		    becore->inputs[i].state == BECORE_INPUT_QUARANTINED) {
			dev_crit(becore->dev,
				 "retaining producer mappings for active/quarantined input slot %u\n",
				 i);
			mutex_unlock(&becore->lock);
			return;
		}
	}
	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_input_slot *slot = &becore->inputs[i];

		if (slot->state != BECORE_INPUT_BACKEND) {
			slot->state = BECORE_INPUT_FREE;
			slot->buffer.staged_bytes = 0;
			slot->producer_cookie = 0;
			slot->ready_sequence = 0;
		}
	}
	mutex_unlock(&becore->lock);

	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		dma_unmap_sgtable(input->producer, &input->sgts[i],
				  DMA_FROM_DEVICE, 0);
		sg_free_table(&input->sgts[i]);
	}
	put_device(input->producer);
	WARN_ON_ONCE(!refcount_dec_and_test(&input->callback_users));
	kfree(input);
}
EXPORT_SYMBOL_GPL(exynos_becore_input_unmap);

size_t exynos_becore_input_size(struct exynos_becore_input *input)
{
	return input->becore->inputs[0].buffer.size;
}
EXPORT_SYMBOL_GPL(exynos_becore_input_size);

/*
 * The whole back end joins the *producer's* graph: the sink pad, and the two
 * video nodes with it.
 *
 * One media device per pipeline is what a controller-aware consumer expects.
 * A libcamera pipeline handler matches *a* media device and walks that
 * device's graph, so a back end on a second one is a pipeline it cannot see;
 * and one graph is the only arrangement in which the link between the two
 * blocks can exist at all, because media_create_pad_link() needs both
 * entities in it.
 *
 * So this device has no media device of its own, and its nodes exist exactly
 * while a producer is bound.  Probe builds everything that does not need a
 * v4l2_device -- the queues, the controls, the entities and their pads -- and
 * this adds the three things that do.
 *
 * Both video devices are stamped from their templates at probe, and
 * video_register_device() consumes fields of them that cannot be handed to it
 * twice -- valid_ioctls is an input on the way in and the result on the way
 * out, and the embedded struct device is kobject_init()ed -- so this registers
 * once for the device's lifetime, and a failure here consumes them just as a
 * success does.  The unregister below is teardown rather than a point to
 * rebind from, which is the only shape either driver has: both are built in
 * and suppress unbind.
 */
int exynos_becore_input_register_graph(struct exynos_becore_input *input,
				       struct v4l2_device *v4l2_dev,
				       struct media_entity *source,
				       u16 source_pad)
{
	struct becore_device *becore;
	int ret;

	if (!input || !v4l2_dev || !source)
		return -EINVAL;
	becore = input->becore;
	if (becore->sd_registered)
		return -EBUSY;

	ret = v4l2_device_register_subdev(v4l2_dev, &becore->sd);
	if (ret)
		return ret;

	ret = media_create_pad_link(source, source_pad, &becore->sd.entity, 0,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		goto err_sd;

	/*
	 * This pointer outlives the registration: v4l2_release() and the
	 * device's own release callback both walk it, and either can run long
	 * after an unregister if userspace still holds the node open. So it is
	 * set here and never cleared.
	 */
	becore->vdev.v4l2_dev = v4l2_dev;
	ret = video_register_device(&becore->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_sd;

	becore->params_vdev.v4l2_dev = v4l2_dev;
	ret = video_register_device(&becore->params_vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_vdev;

	becore->sd_registered = true;

	return 0;

err_vdev:
	/* Releases the queue with it, which a bare unregister would not. */
	vb2_video_unregister_device(&becore->vdev);
	/* Which stops a stream, and a stop schedules this once. */
	cancel_work_sync(&becore->params_work);
err_sd:
	/* Drops the link created above with the entity it was created on. */
	v4l2_device_unregister_subdev(&becore->sd);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_becore_input_register_graph);

void exynos_becore_input_unregister_graph(struct exynos_becore_input *input)
{
	struct becore_device *becore;

	if (!input)
		return;
	becore = input->becore;
	if (!becore->sd_registered)
		return;

	/*
	 * These release the queues too, which a bare unregister would not:
	 * the capture teardown is what stops a stream still running.
	 */
	vb2_video_unregister_device(&becore->params_vdev);
	vb2_video_unregister_device(&becore->vdev);
	/*
	 * The capture teardown above can have scheduled it one last time, and
	 * so can the offline loop afterwards -- debugfs outlives the graph. So
	 * this bounds the work rather than ending it; what runs later finds an
	 * empty list.
	 */
	cancel_work_sync(&becore->params_work);
	v4l2_device_unregister_subdev(&becore->sd);
	becore->sd_registered = false;
}
EXPORT_SYMBOL_GPL(exynos_becore_input_unregister_graph);

static struct becore_input_slot *
becore_input_ticket(struct exynos_becore_input *input,
		    const struct exynos_becore_input_buffer *buffer)
{
	struct becore_input_slot *slot;

	if (!buffer || buffer->slot >= BECORE_INPUT_SLOT_COUNT)
		return NULL;
	slot = &input->becore->inputs[buffer->slot];
	if (slot->state != BECORE_INPUT_PRODUCER ||
	    slot->producer_cookie != buffer->cookie ||
	    input->dmas[buffer->slot] != buffer->dma ||
	    slot->buffer.size != buffer->size)
		return NULL;

	return slot;
}

int exynos_becore_input_producer_acquire(struct exynos_becore_input *input,
					 struct exynos_becore_input_buffer *buffer)
{
	struct becore_device *becore = input->becore;
	struct becore_input_slot *slot = NULL;
	u64 cookie;
	unsigned int i;
	int ret = 0;

	if (!buffer)
		return -EINVAL;

	mutex_lock(&becore->lock);
	if (becore->input_producer != input) {
		ret = -EINVAL;
		goto unlock;
	}
	if (becore->reset_failed) {
		ret = -EIO;
		goto unlock;
	}
	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++)
		if (becore->inputs[i].state == BECORE_INPUT_FREE) {
			slot = &becore->inputs[i];
			break;
		}
	if (!slot) {
		ret = -EBUSY;
		goto unlock;
	}

	/*
	 * Only a slot the debugfs path staged can hold dirty lines that would
	 * otherwise land on top of what the front end is about to write.  In an
	 * ordinary capture this is never taken, which is the point: the sweep
	 * costs 2.3 ms over a 27 MB slot and used to run on every frame.
	 */
	if (slot->cpu_dirty) {
		dma_sync_sgtable_for_device(becore->dev, slot->buffer.sgt,
					    DMA_BIDIRECTIONAL);
		slot->cpu_dirty = false;
	}
	slot->buffer.staged_bytes = 0;
	slot->ready_sequence = 0;
	/*
	 * The balance this frame will be taken through unless the producer
	 * says otherwise, which is the one STREAMON handed it.  Seeding rather
	 * than requiring means a producer with nothing to say about white
	 * balance -- and every producer had nothing to say until one of them
	 * grew a parameters node -- needs to do nothing at all.
	 */
	slot->gains = becore_stream_gains(becore);
	cookie = ++becore->producer_sequence;
	if (!cookie)
		cookie = ++becore->producer_sequence;
	slot->producer_cookie = cookie;
	slot->state = BECORE_INPUT_PRODUCER;
	*buffer = (struct exynos_becore_input_buffer) {
		.dma = input->dmas[i],
		.size = slot->buffer.size,
		.cookie = cookie,
		.slot = i,
		.gains = slot->gains,
	};

unlock:
	mutex_unlock(&becore->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_becore_input_producer_acquire);

int exynos_becore_input_producer_complete(struct exynos_becore_input *input,
					  const struct exynos_becore_input_buffer *buffer)
{
	struct becore_device *becore = input->becore;
	struct becore_input_slot *slot;
	int ret = 0;

	mutex_lock(&becore->lock);
	if (becore->input_producer != input) {
		ret = -EINVAL;
		goto unlock;
	}
	slot = becore_input_ticket(input, buffer);
	if (!slot) {
		ret = -EINVAL;
		goto unlock;
	}
	if (!becore_gains_valid(&buffer->gains)) {
		/*
		 * Nothing can reach this today -- every gain the front end can
		 * hold is already bounded by the same constant on its own side
		 * -- which is exactly why it says so out loud.  The caller's
		 * only recourse is to fail the stream, and a stream that stops
		 * with a hung DQBUF and nothing in the log is worse than one
		 * that is merely broken.
		 */
		dev_err_ratelimited(becore->dev,
				    "producer frame carries gains %u/%u/%u/%u Q12, outside %u..%u\n",
				    buffer->gains.red, buffer->gains.green_red,
				    buffer->gains.green_blue, buffer->gains.blue,
				    EXYNOS_BECORE_WBG_GAIN_MIN_Q12,
				    EXYNOS_BECORE_WBG_GAIN_MAX_Q12);
		ret = -ERANGE;
		goto unlock;
	}

	/*
	 * The caller has quiesced the producer at a completed-frame boundary.
	 * Nothing is synced: ISPFE wrote these pages to DRAM and RGBP will read
	 * them from DRAM, and no CPU mapping was read or written in between.
	 */
	slot->buffer.staged_bytes = slot->buffer.size;
	/*
	 * The producer's raster and not becore->array, which the offline loop
	 * may have set to something else entirely.  A frame is at the raster
	 * whoever wrote it wrote it at, and the run refuses to encode for any
	 * other -- see becore_recipe_validate().
	 */
	slot->raster = becore->producer_array;
	/*
	 * And the producer's gains, for the same reason the raster is the
	 * producer's: a frame is at the balance whoever took it took it at.
	 */
	slot->gains = buffer->gains;
	/* And which frame it is, for the buffer this slot ends up as. */
	slot->producer_frame = buffer->sequence;
	slot->producer_timestamp = buffer->timestamp;
	slot->producer_cookie = 0;
	slot->ready_sequence = ++becore->input_sequence;
	slot->state = BECORE_INPUT_READY;
	if (becore->video_streaming && becore->producer_streaming)
		schedule_work(&becore->video_work);

unlock:
	mutex_unlock(&becore->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_becore_input_producer_complete);

void exynos_becore_input_producer_abort(struct exynos_becore_input *input,
					const struct exynos_becore_input_buffer *buffer)
{
	struct becore_device *becore = input->becore;
	struct becore_input_slot *slot;

	mutex_lock(&becore->lock);
	if (becore->input_producer == input) {
		slot = becore_input_ticket(input, buffer);
		if (!slot)
			goto unlock;
		slot->buffer.staged_bytes = 0;
		slot->producer_cookie = 0;
		slot->ready_sequence = 0;
		slot->state = BECORE_INPUT_FREE;
	}

unlock:
	mutex_unlock(&becore->lock);
}
EXPORT_SYMBOL_GPL(exynos_becore_input_producer_abort);

static void becore_clear_pending_irqs(struct becore_block *block)
{
	writel_relaxed(U32_MAX, block->base + BECORE_INT0_CLEAR);
	writel_relaxed(U32_MAX, block->base + BECORE_INT1_CLEAR);
	writel_relaxed(U32_MAX, block->base + BECORE_CMDQ_INT_CLEAR);
}

static void becore_publish_program(struct becore_device *becore,
				   enum becore_block_id id)
{
	struct becore_cmdq_program *program = id == BECORE_MCSC ?
		&becore->mcsc_program : &becore->program[id];
	struct becore_block *block = &becore->blocks[id];

	writel_relaxed(lower_32_bits(program->dma),
		       block->base + BECORE_CMDQ_QUE_CMD_L);
	writel_relaxed(BECORE_CMDQ_MODE | program->header_count,
		       block->base + BECORE_CMDQ_QUE_CMD_M);
	writel_relaxed(0xff, block->base + BECORE_CMDQ_QUE_CMD_H);
	writel_relaxed(1, block->base + BECORE_CMDQ_QUE_CMD_START);
}

/*
 * The seed is written after the programs are encoded and the result read before
 * the next frame overwrites it.  Under a stream the processors are no longer
 * reset between frames, so what bounds the window is the run itself rather than
 * a reset either side of it; the seed is rewritten every frame regardless.
 */
static void becore_stream_crc_arm(struct becore_device *becore)
{
	u32 seed = READ_ONCE(becore->stream_crc_seed) &
		   BECORE_STREAM_CRC_SEED_MASK;
	unsigned int id;

	becore->stream_crc_armed_seed = seed;
	for (id = 0; id < BECORE_NUM_BLOCKS; id++) {
		struct becore_block *block = &becore->blocks[id];
		const struct becore_stream_crc *table;
		size_t count;
		size_t i;

		table = becore_stream_crc_table(id, &count);
		for (i = 0; i < count; i++) {
			block->stream_crc_armed[i] = 0;
			block->stream_crc_result[i] = 0;
			if (!seed)
				continue;
			writel_relaxed(seed, block->base + table[i].offset);
			block->stream_crc_armed[i] =
				readl_relaxed(block->base + table[i].offset);
		}
	}
}

static void becore_stream_crc_capture(struct becore_device *becore)
{
	unsigned int id;

	becore->stream_crc_generation = becore->run_generation;
	if (!becore->stream_crc_armed_seed)
		return;

	for (id = 0; id < BECORE_NUM_BLOCKS; id++) {
		struct becore_block *block = &becore->blocks[id];
		const struct becore_stream_crc *table;
		size_t count;
		size_t i;

		table = becore_stream_crc_table(id, &count);
		for (i = 0; i < count; i++)
			block->stream_crc_result[i] =
				readl_relaxed(block->base + table[i].offset);
	}
}

static void becore_measure_buffer(const struct becore_dma_buffer *buffer,
				  u32 *changed_bytes, u32 *first_changed)
{
	const u8 *output = buffer->cpu;
	u32 changed = 0;
	u32 first = U32_MAX;
	u32 i;

	for (i = 0; i < buffer->size; i++) {
		if (output[i] == 0xa5)
			continue;
		if (first == U32_MAX)
			first = i;
		changed++;
	}
	*changed_bytes = changed;
	*first_changed = first;
}

static int becore_run_stage(struct becore_device *becore, u32 blocks)
{
	unsigned long flags;
	unsigned long waited;
	unsigned int i;
	int ret;

	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		if (blocks & BIT(i))
			becore_clear_pending_irqs(&becore->blocks[i]);
	reinit_completion(&becore->run_completion);

	spin_lock_irqsave(&becore->run_lock, flags);
	becore->expected_mask = blocks;
	becore->start_issued = false;
	if (becore->abort_run) {
		spin_unlock_irqrestore(&becore->run_lock, flags);
		return -ECANCELED;
	}
	spin_unlock_irqrestore(&becore->run_lock, flags);

	/* Publish every DMA ownership transition before starting the next stage. */
	dma_wmb();
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		if (blocks & BIT(i))
			becore_publish_program(becore, i);
	mutex_unlock(&becore->lock);

	waited = wait_for_completion_timeout(&becore->run_completion,
					     msecs_to_jiffies(BECORE_RUN_TIMEOUT_MS));

	mutex_lock(&becore->lock);
	spin_lock_irqsave(&becore->run_lock, flags);
	if (!waited)
		ret = -ETIMEDOUT;
	else if (becore->irq_error)
		ret = -EIO;
	else if (becore->abort_run)
		ret = -ECANCELED;
	else if ((becore->frame_done_mask & blocks) != blocks)
		ret = -EIO;
	else
		ret = 0;
	spin_unlock_irqrestore(&becore->run_lock, flags);

	return ret;
}

static struct becore_input_slot *
becore_next_input(struct becore_device *becore, bool allow_staged)
{
	struct becore_input_slot *next = NULL;
	unsigned int i;

	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_input_slot *slot = &becore->inputs[i];

		if (slot->state != BECORE_INPUT_READY)
			continue;
		if (!next || slot->ready_sequence < next->ready_sequence)
			next = slot;
	}
	if (next)
		return next;

	/* Slot zero remains the upload-and-repeat diagnostic oracle. */
	if (allow_staged && becore->inputs[0].state == BECORE_INPUT_FREE)
		return &becore->inputs[0];

	return NULL;
}

int becore_run_frame(struct becore_device *becore, u32 input_profile,
		     u32 output_profile, bool ready_only,
		     struct vb2_buffer *capture, bool packed_output,
		     bool run_mcsc)
{
	bool use_votf;

	unsigned long flags;
	bool diagnostic_output = !capture;
	bool input_claimed = false;
	u64 phase_ns[BECORE_TIMING_PHASE_COUNT] = {};
	bool quiesced;
	ktime_t start;
	ktime_t mark;
	int pm_ret;
	int ret;
	u32 i;

	mutex_lock(&becore->lock);
	if (diagnostic_output && becore->video_streaming) {
		ret = -EBUSY;
		goto unlock;
	}
	if (!diagnostic_output && !becore->video_streaming) {
		ret = -ECANCELED;
		goto unlock;
	}
	if (becore->running) {
		ret = -EBUSY;
		goto unlock;
	}
	if (becore->reset_failed) {
		ret = -EIO;
		goto record_error;
	}
	if (output_profile >= BECORE_YUVP_OUTPUT_PROFILE_COUNT ||
	    input_profile >= BECORE_RGBP_INPUT_PROFILE_COUNT) {
		ret = -EINVAL;
		goto record_error;
	}
	if (packed_output && output_profile != BECORE_YUVP_OUTPUT_P010) {
		ret = -EINVAL;
		goto record_error;
	}
	if (run_mcsc && (output_profile != BECORE_YUVP_OUTPUT_SBWCL ||
			 packed_output)) {
		ret = -EINVAL;
		goto record_error;
	}
	becore->run_input = becore_next_input(becore, !ready_only);
	if (!becore->run_input) {
		if (ready_only) {
			ret = -ENODATA;
			goto unlock;
		}
		ret = -EBUSY;
		goto record_error;
	}
	/*
	 * Only a diagnostic run over the staged slot may take an input profile
	 * other than the live one: the producer writes SBWC-compressed Bayer,
	 * and a slot it completed carries a non-zero ready sequence where the
	 * staged slot zero is handed out from FREE and carries none.
	 */
	if (input_profile != BECORE_RGBP_INPUT_SBWC &&
	    (!diagnostic_output || becore->run_input->ready_sequence)) {
		ret = -EINVAL;
		goto record_error;
	}
	if (capture) {
		becore->mcsc_dest_dma =
			vb2_dma_contig_plane_dma_addr(capture, 0);
		/*
		 * MCSC's address registers are 32 bits, and a capture is the
		 * one destination the driver did not allocate, so it gets the
		 * bounds check the driver's own buffers get at probe.
		 */
		if (!becore->mcsc_dest_dma ||
		    upper_32_bits(becore->mcsc_dest_dma) ||
		    upper_32_bits(becore->mcsc_dest_dma +
				  becore_mcsc_output_active_size(&becore->scaled) - 1)) {
			ret = -EINVAL;
			goto record_error;
		}
	} else if (run_mcsc) {
		/*
		 * The driver's own surface gets the same check, because it is
		 * no longer allocated for the device's lifetime: a geometry
		 * change that could not take its surfaces back leaves this one
		 * freed, and a run must refuse rather than program a DMA at
		 * zero.
		 */
		becore->mcsc_dest_dma = becore->mcsc_output.dma;
		if (!becore->mcsc_dest_dma ||
		    upper_32_bits(becore->mcsc_dest_dma) ||
		    upper_32_bits(becore->mcsc_dest_dma +
				  becore_mcsc_output_active_size(&becore->scaled) - 1)) {
			ret = -EINVAL;
			goto record_error;
		}
	} else {
		becore->mcsc_dest_dma = becore->mcsc_output.dma;
	}
	becore->active_input_profile = input_profile;
	becore->active_output_profile = output_profile;
	becore->active_output_packed = packed_output;
	becore->active_output_dma = becore->output.dma;
	becore->active_output_size = becore_active_output_size(becore);
	becore->active_capture_size = run_mcsc ?
		becore_mcsc_output_active_size(&becore->scaled) : becore->active_output_size;
	if (!becore->active_output_dma ||
	    upper_32_bits(becore->active_output_dma) ||
	    upper_32_bits(becore->active_output_dma +
			  becore->active_output_size - 1) ||
	    becore->active_output_size > becore->output.size) {
		ret = -EINVAL;
		goto record_error;
	}
	ret = becore_recipe_validate(becore);
	if (ret)
		goto record_error;
	if (run_mcsc) {
		ret = becore_mcsc_recipe_validate(becore);
		if (ret)
			goto record_error;
	}
	becore->run_input->state = BECORE_INPUT_BACKEND;
	input_claimed = true;

	start = ktime_get();
	ret = pm_runtime_resume_and_get(becore->dev);
	mark = becore_timing_mark(phase_ns, BECORE_TIMING_RESUME, start);
	if (ret)
		goto record_error;
	if (becore->reset_failed) {
		/* The runtime callback deliberately retains every supplier. */
		ret = -EIO;
		goto record_error;
	}
	/*
	 * The fabric needs all three blocks in one stage, both endpoints
	 * transported over it, and both windows ready.  A consumer that ran
	 * after the producer finished could not drain a ring the producer is
	 * filling, so with a limit of one token the producer would stall at
	 * the first one -- which is why this is a single switch and not three.
	 *
	 * It is resolved here rather than with the rest of the run's shape
	 * because readiness is a property of the resume just above: a one-shot
	 * arrives with its windows torn down, and only becomes able to use the
	 * fabric once it has powered the block back up.
	 */
	use_votf = run_mcsc && READ_ONCE(becore->votf) &&
		   output_profile == BECORE_YUVP_OUTPUT_SBWCL &&
		   becore->c2serv_state[BECORE_C2SERV_YUVP].ready &&
		   becore->c2serv_state[BECORE_C2SERV_MCSC].ready &&
		   becore_c2serv_chain_fits(&becore->chain);
	if (!use_votf && run_mcsc && READ_ONCE(becore->votf) &&
	    !becore_c2serv_chain_fits(&becore->chain))
		dev_warn_once(becore->dev,
			      "chain raster %ux%u is not a whole number of VOTF producer tokens; using memory\n",
			      becore->chain.width, becore->chain.height);
	becore->active_votf = use_votf;
	if (use_votf)
		becore_c2serv_program_link(becore);

	/*
	 * Take this frame's parameters immediately before it is encoded: past
	 * every refusal above, so a buffer is never spent on a frame that does
	 * not happen, and before the encoder reads them, so a buffer queued for
	 * this frame reaches this frame rather than the next one.  The offline
	 * path takes them too, which is what makes a parameters block auditable
	 * -- stage a frame, queue a block, run, and read the encoded program
	 * back out.
	 */
	becore_params_consume(becore);

	ret = becore_encode_programs(becore);
	if (ret)
		goto put_power;
	if (run_mcsc) {
		enum becore_mcsc_input_transport transport = use_votf ?
			BECORE_MCSC_INPUT_CAPTURED_VOTF :
			BECORE_MCSC_INPUT_MEMORY;

		ret = becore_encode_mcsc(becore, transport);
		if (ret)
			goto put_power;
		becore->mcsc_encoded_generation = becore->mcsc_recipe_generation;
		becore->mcsc_encoded_transport = transport;
	}
	mark = becore_timing_mark(phase_ns, BECORE_TIMING_ENCODE, mark);

	if (diagnostic_output) {
		memset(becore->output.cpu, 0xa5, becore->output.size);
		if (run_mcsc)
			memset(becore->mcsc_output.cpu, 0xa5,
			       becore->mcsc_output.size);
	}
	becore->output_changed_bytes = 0;
	becore->output_first_changed = U32_MAX;
	becore->mcsc_output_changed_bytes = 0;
	becore->mcsc_output_first_changed = U32_MAX;
	becore->mcsc_completed_generation = 0;
	becore->mcsc_completed_output_size = 0;
	for (i = 0; i < BECORE_NUM_BLOCKS; i++) {
		WRITE_ONCE(becore->blocks[i].last_int0, 0);
		WRITE_ONCE(becore->blocks[i].last_int1, 0);
		WRITE_ONCE(becore->blocks[i].last_cmdq_int, 0);
		atomic64_set(&becore->blocks[i].int0_count, 0);
		atomic64_set(&becore->blocks[i].int1_count, 0);
	}
	spin_lock_irqsave(&becore->run_lock, flags);
	becore->cmdq_hold_mask = 0;
	becore->frame_done_mask = 0;
	becore->expected_mask = 0;
	becore->abort_run = false;
	becore->irq_error = false;
	becore->running = true;
	becore->run_generation++;
	spin_unlock_irqrestore(&becore->run_lock, flags);
	becore->active_mcsc = run_mcsc;

	becore_stream_crc_arm(becore);

	/*
	 * A staged slot was filled through the cached vmap, so its lines have
	 * to reach DRAM before RGBP reads them.  A producer-written slot was
	 * never touched by the CPU and needs nothing.
	 */
	if (becore->run_input->cpu_dirty) {
		dma_sync_sgtable_for_device(becore->dev,
					    becore->run_input->buffer.sgt,
					    DMA_TO_DEVICE);
		becore->run_input->cpu_dirty = false;
	}
	mark = becore_timing_mark(phase_ns, BECORE_TIMING_ARM, mark);
	if (use_votf) {
		/*
		 * One stage, because the two halves of a VOTF link have to be
		 * running at the same time.  becore_run_stage() already starts
		 * every block in the mask together and waits for all their
		 * frame-ends, so the whole of what this needs already exists.
		 */
		becore_c2serv_link_start(becore);
		ret = becore_run_stage(becore, BECORE_YUVP_STAGE_BLOCKS |
					       BIT(BECORE_MCSC));
		becore_c2serv_link_sample(becore, &becore->c2serv_done);
		becore_c2serv_link_stop(becore, ret);
		mark = becore_timing_mark(phase_ns, BECORE_TIMING_STAGE1, mark);
	} else {
		ret = becore_run_stage(becore, BECORE_YUVP_STAGE_BLOCKS);
		mark = becore_timing_mark(phase_ns, BECORE_TIMING_STAGE1, mark);
		if (!ret && run_mcsc) {
			ret = becore_run_stage(becore, BIT(BECORE_MCSC));
			mark = becore_timing_mark(phase_ns,
						  BECORE_TIMING_STAGE2, mark);
		}
	}

	/* A failed run's CRCs say how far the stream got, so read them too. */
	becore_stream_crc_capture(becore);

	spin_lock_irqsave(&becore->run_lock, flags);
	becore->running = false;
	spin_unlock_irqrestore(&becore->run_lock, flags);
	mark = becore_timing_mark(phase_ns, BECORE_TIMING_CRC, mark);

	/*
	 * Without a stream this suspends: runtime_suspend synchronizes IRQs and
	 * resets all four processors.  Under one it only drops the usage count,
	 * because the stream holds its own reference.
	 */
	pm_ret = pm_runtime_put_sync(becore->dev);
	if (pm_ret < 0) {
		/* Match probe: never leave a failed-reset device at usage zero. */
		pm_runtime_get_noresume(becore->dev);
		quiesced = false;
		if (!ret)
			ret = pm_ret;
	} else if (ret && becore->stream_powered) {
		/*
		 * The put above reset nothing, so a run that failed leaves the
		 * processors unproven -- and on a timeout they are genuinely
		 * still going.  The slot cannot go back to a producer that
		 * would begin writing it, and producer_acquire() does not gate
		 * on streaming, so prove quiescence here rather than waiting
		 * for the teardown that follows.  Losing the init tables with
		 * it costs nothing: every path that reaches this stops the
		 * stream, and the next STREAMON resumes through them again.
		 */
		int reset_ret = becore_reset_all(becore);

		becore->reset_failed = !!reset_ret;
		quiesced = !reset_ret;
	} else {
		quiesced = true;
	}

	if (!quiesced) {
		/*
		 * The input mapping may not be returned after an unproven stop.
		 * The capture is worse and is not resolved here: MCSC writes it
		 * directly, so unlike the driver-owned output it *was* a live
		 * destination, and the caller returns it with ERROR.  Nothing
		 * below holds its pages -- vb2_queue_error() pushes userspace
		 * straight to REQBUFS(0), which frees them.  What bounds the
		 * damage is the SysMMU: dma_free_attrs() unmaps the IOVA before
		 * releasing the pages, so a write still in flight faults rather
		 * than landing in recycled memory, until that IOVA is reused.
		 * reset_failed is sticky and stops the device being used again,
		 * but it cannot stop a transfer already started.  Proving
		 * quiescence needs an idle-status read this driver does not do.
		 */
		becore->run_input->state = BECORE_INPUT_QUARANTINED;
		becore->output_quarantined = true;
		becore->completed_generation = 0;
		becore->completed_output_size = 0;
		becore->mcsc_completed_generation = 0;
		becore->mcsc_completed_output_size = 0;
	}

	/*
	 * Number the frame here, where the slot that made it is still in hand.
	 *
	 * A processed frame *is* the frame the producer made, so it carries
	 * that frame's number and that frame's end -- not a count of this
	 * node's buffers and not the moment the chain finished.  Both are what
	 * pair a picture with the statistics metered from it and with the
	 * sensor settings it was taken at.
	 *
	 * The test is `ready_sequence` and not the frame number being non-zero,
	 * which is a different question with a coincidental answer: a producer
	 * that never fills the field leaves it zero, and one in-tree caller
	 * does exactly that.  `ready_sequence` is what producer_complete() sets
	 * and the free path clears, so it is the one that means "a producer
	 * filled this slot".  It has to be read before the slot is released,
	 * which is why this is here and not in the caller.
	 *
	 * A run with no producer behind it -- the offline loop's staged input,
	 * taken from BECORE_INPUT_FREE and possibly a producer's slot before
	 * that -- is no frame of anybody's and gets this node's own counter and
	 * its own completion time.
	 */
	if (!ret && capture) {
		struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(capture);

		if (becore->run_input->ready_sequence) {
			vbuf->sequence = becore->run_input->producer_frame;
			capture->timestamp =
				becore->run_input->producer_timestamp;
		} else {
			vbuf->sequence = becore->video_sequence;
			capture->timestamp = ktime_get_ns();
		}
		becore->video_sequence++;
	}

	if (quiesced) {
		becore->run_input->state = BECORE_INPUT_FREE;
		becore->run_input->ready_sequence = 0;
	}
	becore->run_input = NULL;
	input_claimed = false;
	if (quiesced)
		dma_rmb();
	becore_timing_mark(phase_ns, BECORE_TIMING_SUSPEND, mark);
	phase_ns[BECORE_TIMING_FRAME] =
		ktime_to_ns(ktime_sub(ktime_get(), start));
	for (i = 0; i < BECORE_TIMING_PHASE_COUNT; i++) {
		becore->timing[i].last_ns = phase_ns[i];
		becore->timing[i].total_ns += phase_ns[i];
	}
	becore->timing_runs++;
	if (diagnostic_output && pm_ret >= 0) {
		becore_measure_buffer(&becore->output,
				      &becore->output_changed_bytes,
				      &becore->output_first_changed);
		becore->completed_generation = becore->run_generation;
		becore->completed_output_size = becore->active_output_size;
		if (run_mcsc) {
			becore_measure_buffer(&becore->mcsc_output,
					      &becore->mcsc_output_changed_bytes,
					      &becore->mcsc_output_first_changed);
			if (!ret) {
				becore->mcsc_completed_generation =
					becore->run_generation;
				becore->mcsc_completed_output_size =
					becore_mcsc_output_size(&becore->scaled);
			}
		}
	}
	becore->last_run_result = ret;
	/* Anything queued while this ran has nothing left to wait for. */
	becore_params_drain_idle(becore);
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
	if (input_claimed) {
		becore->run_input->state = BECORE_INPUT_FREE;
		becore->run_input->ready_sequence = 0;
	}
	becore->run_input = NULL;
	becore->last_run_result = ret;
	becore_params_drain_idle(becore);
unlock:
	mutex_unlock(&becore->lock);
	return ret;
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
	/*
	 * The three rasters, compiled in as defaults rather than as fields of
	 * the tables that describe formats.  Nothing negotiates any of them
	 * yet, and what changes when something does is these six assignments
	 * -- everything downstream already reads a value.
	 */
	becore->array.width = BECORE_ARRAY_WIDTH;
	becore->array.height = BECORE_ARRAY_HEIGHT;
	/*
	 * Until a stream reads the sink pad this is the only thing said about
	 * what the producer sends, and it is more than a guess: the producer
	 * checks the length of the slots it is handed against its own constant
	 * at its probe, and that length is this raster's.
	 */
	becore->producer_array = becore->array;
	becore->chain.width = BECORE_CHAIN_WIDTH;
	becore->chain.height = BECORE_CHAIN_HEIGHT;
	becore->scaled.width = BECORE_OUTPUT_WIDTH;
	becore->scaled.height = BECORE_OUTPUT_HEIGHT;
	mutex_init(&becore->lock);
	mutex_init(&becore->video_lock);
	mutex_init(&becore->params_lock);
	spin_lock_init(&becore->run_lock);
	spin_lock_init(&becore->queue_lock);
	init_completion(&becore->run_completion);
	INIT_LIST_HEAD(&becore->queued_outputs);
	INIT_LIST_HEAD(&becore->queued_params);
	INIT_WORK(&becore->video_work, becore_video_work);
	INIT_WORK(&becore->params_work, becore_params_work);
	becore->output_first_changed = U32_MAX;
	becore->mcsc_output_first_changed = U32_MAX;
	becore->active_input_profile = BECORE_RGBP_INPUT_SBWC;
	becore->active_output_profile = BECORE_YUVP_OUTPUT_SBWCL;
	becore->votf = 1;
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
	becore->blocks[BECORE_MCSC] = (struct becore_block) {
		.becore = becore,
		.name = "MCSC",
		.int0_mask_prepare = 0x3ffffc02,
		.int0_mask = 0x3ffffc06,
		.int1_mask = 0x501,
		.cmdq_int_mask = 0x1,
	};
	platform_set_drvdata(pdev, becore);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "no 32-bit DMA\n");

	becore->intcam_clk = devm_clk_get(dev, "intcam");
	if (IS_ERR(becore->intcam_clk))
		return dev_err_probe(dev, PTR_ERR(becore->intcam_clk),
				     "cannot get INTCAM clock\n");

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

	ret = becore_video_init(becore);
	if (ret)
		return ret;

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
