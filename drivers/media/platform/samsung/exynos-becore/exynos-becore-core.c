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
 * The driver is more than this file.  This one holds the device: power, probe,
 * debugfs, the run loop and the interrupt path.  exynos-becore-params.c holds
 * the parameters node and the encoders that read a buffer from it, and
 * exynos-becore-generated.c the words the driver states for a register no
 * buffer describes.  exynos-becore-common.h is what they share;
 * exynos-becore-regs.h is the silicon.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/io.h>
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

#define BECORE_YUVP_STAGE_BLOCKS	(BIT(BECORE_RGBP) | BIT(BECORE_YUVP))

#define BECORE_RUN_TIMEOUT_MS		1000

/* What a queued capture's address must be aligned to; see becore_buf_prepare. */
#define BECORE_CAPTURE_ALIGN		32

/* The lattice, plus the pair that opens the port and the pair that closes it. */
#define BECORE_YUVP_CLUT_BURST_HEADERS \
	(DIV_ROUND_UP(BECORE_CLUT_LATTICE_WORDS, BECORE_CMDQ_PAYLOAD_WORDS) + 2)
#define BECORE_YUVP_PROGRAM_HEADERS \
	(BECORE_YUVP_HEADER_COUNT + BECORE_YUVP_CLUT_BURST_HEADERS)

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

/*
 * The four Bayer orders, numbered the way Samsung's OTF_INPUT_ORDER_BAYER_*
 * enum numbers them, which is what BYR_DNS and BYR_DMSC take: the index into
 * this table is the register value. The three cameras' captured programs write
 * 0, 1 and 2, and the one sensor with a driver settles which is which -- the
 * ultrawide's IMX712 reads out RGGB with both flip bits clear, and its
 * captured phase is 1.
 *
 * The fourth is unused by any of the three and is here because the field is
 * two bits wide and a table with a hole in it is worse than one without.
 */
static const u32 becore_input_codes[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,	/* the main camera's */
	MEDIA_BUS_FMT_SRGGB10_1X10,	/* the ultrawide's */
	MEDIA_BUS_FMT_SBGGR10_1X10,	/* the front's */
	MEDIA_BUS_FMT_SGBRG10_1X10,
};

#define BECORE_INPUT_DEFAULT_CODE	MEDIA_BUS_FMT_SRGGB10_1X10

/* The CFA phase a media-bus code means, or -EINVAL for one we cannot place. */
int becore_bayer_phase(u32 code)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_input_codes); i++)
		if (becore_input_codes[i] == code)
			return i;

	return -EINVAL;
}

/*
 * How the Bayer arrives, from the live ultrawide program: the DMA fields and
 * nothing about its size.  As in Pablo's common DMA API the payload and header
 * geometry are derived rather than carried, and Lyric additionally writes the
 * 256-pixel-aligned SBWC storage width after enabling the RDMA.
 *
 * The size lives in becore->array, because it is a property of the picture and
 * not of the format it is stored in -- the same format describes any of them.
 * Both are resolved once and passed to each derivation rather than read from
 * module scope, which is what lets the offline loop encode for a geometry the
 * hardware is not running.
 */
static const struct becore_rgbp_input_profile
becore_rgbp_inputs[BECORE_RGBP_INPUT_PROFILE_COUNT] = {
	[BECORE_RGBP_INPUT_SBWC] = {
		.data_format = 0x18,
		.comp_control = 0x9,
		.sbwc_block_width = 256,
		.bytes_per_pixel = 2,
		.header_stride = 0x40,
		.businfo = 0,
	},
	/*
	 * The same image without compression, so that a frame chosen rather
	 * than captured can be put through the offline loop.  Lyric's own
	 * BuildFormatConfig picks between the two formats and writes 0x1a when
	 * it is not compressing -- a three-instruction branch, `tst` on
	 * IsSbwcBufferCompression and a `csel` of 0x18 against 0x1a -- so this
	 * is the vendor's own counterpart of the profile above rather than a
	 * guess: same signedness, same depth, one 16-bit little-endian sample
	 * per pixel right-aligned in the low twelve bits.
	 *
	 * An SBWC block width of one makes the storage width the active width,
	 * which is what an uncompressed stride is, and there is no header
	 * plane.  Only the offline loop may select it: the live producer writes
	 * compressed Bayer, so reading its pages under this profile would
	 * decode payload as pixels.
	 */
	[BECORE_RGBP_INPUT_LINEAR] = {
		.data_format = 0x1a,
		.comp_control = 0,
		.sbwc_block_width = 1,
		.bytes_per_pixel = 2,
		.header_stride = 0,
		.businfo = 0,
	},
};

const u32 becore_rgbp_input_regs[] = {
	[BECORE_RGBP_CROP_SIZE] = BECORE_RGBP_CROP_SIZE_REG,
	[BECORE_RGBP_CROP_START] = BECORE_RGBP_CROP_START_REG,
	[BECORE_RGBP_SC_DST_SIZE] = BECORE_RGBP_SC_DST_SIZE_REG,
	[BECORE_RGBP_SC_H_RATIO] = BECORE_RGBP_SC_H_RATIO_REG,
	[BECORE_RGBP_SC_V_RATIO] = BECORE_RGBP_SC_V_RATIO_REG,
	[BECORE_RGBP_CHAIN_SRC_SIZE] = BECORE_RGBP_CHAIN_SRC_SIZE_REG,
	[BECORE_RGBP_CHAIN_DST_SIZE] = BECORE_RGBP_CHAIN_DST_SIZE_REG,
	[BECORE_RGBP_INPUT_FORMAT] = BECORE_RGBP_INPUT_FORMAT_REG,
	[BECORE_RGBP_INPUT_COMP] = BECORE_RGBP_INPUT_COMP_REG,
	[BECORE_RGBP_INPUT_ACTIVE_WIDTH] = BECORE_RGBP_INPUT_WIDTH_REG,
	[BECORE_RGBP_INPUT_HEIGHT] = BECORE_RGBP_INPUT_HEIGHT_REG,
	[BECORE_RGBP_INPUT_STRIDE] = BECORE_RGBP_INPUT_STRIDE_REG,
	[BECORE_RGBP_INPUT_HEADER_STRIDE] = BECORE_RGBP_INPUT_HEADER_STRIDE_REG,
	[BECORE_RGBP_INPUT_BUSINFO] = BECORE_RGBP_INPUT_BUSINFO_REG,
	[BECORE_RGBP_INPUT_ENABLE] = BECORE_RGBP_INPUT_ENABLE_REG,
	[BECORE_RGBP_INPUT_STORAGE_WIDTH] = BECORE_RGBP_INPUT_WIDTH_REG,
};

/* Pixel uses a combined, two-plane WDMA where Pablo v1.1 uses split blocks. */
static const struct becore_yuvp_output_profile becore_yuvp_outputs[] = {
	[BECORE_YUVP_OUTPUT_SBWCL] = {
		.data_format = 0x2000,
		.mode = 0xa,
		.lossy_byte32num = 2,
		.votf_enable = 3,
		.block_width = 32,
		.block_height = 4,
		.luma_height_align = 16,
		.plane_gap = 0x40,
		.businfo = 0,
	},
	[BECORE_YUVP_OUTPUT_P010] = {
		.data_format = 0x2000,
		.mode = 0,
		.lossy_byte32num = 0,
		.votf_enable = 0,
		.bytes_per_pixel = 2,
		.businfo = 0,
	},
};

static const u32 becore_yuvp_output_regs[] = {
	[BECORE_YUVP_OUTPUT_VOTF] = BECORE_YUVP_OUTPUT_VOTF_REG,
	[BECORE_YUVP_OUTPUT_FORMAT] = BECORE_YUVP_OUTPUT_FORMAT_REG,
	[BECORE_YUVP_OUTPUT_LOSSY] = BECORE_YUVP_OUTPUT_LOSSY_REG,
	[BECORE_YUVP_OUTPUT_MODE] = BECORE_YUVP_OUTPUT_MODE_REG,
	[BECORE_YUVP_OUTPUT_WIDTH] = BECORE_YUVP_OUTPUT_WIDTH_REG,
	[BECORE_YUVP_OUTPUT_HEIGHT] = BECORE_YUVP_OUTPUT_HEIGHT_REG,
	[BECORE_YUVP_OUTPUT_STRIDE1] = BECORE_YUVP_OUTPUT_STRIDE1_REG,
	[BECORE_YUVP_OUTPUT_STRIDE2] = BECORE_YUVP_OUTPUT_STRIDE2_REG,
	[BECORE_YUVP_OUTPUT_BUSINFO] = BECORE_YUVP_OUTPUT_BUSINFO_REG,
	[BECORE_YUVP_OUTPUT_ENABLE] = BECORE_YUVP_OUTPUT_ENABLE_REG,
};

/*
 * The first physical-ultrawide GTNR frame has no temporal inputs or map DMAs.
 * It reads YUVP's 4160x3120 lossy-SBWC surface and writes a separate surface
 * with the same bounded layout.  Keep this startup shape dormant until GTNR's
 * power, reset, interrupt, and cross-block completion lifecycle is established.
 */
static const struct becore_gtnr_dma_profile becore_gtnr_input = {
	.votf_enable = 1,
	.businfo = 1,
	.max_mo = 0x100,
	.max_bl = 0x10,
	.enable = 1,
};

static const struct becore_gtnr_dma_profile becore_gtnr_output = {
	.businfo = 0,
	.max_mo = 0x100,
	.max_bl = 0x10,
	.enable = 1,
};

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

static const u32 becore_gtnr_dma_regs[] = {
	[BECORE_GTNR_INPUT_VOTF] = BECORE_GTNR_INPUT_VOTF_REG,
	[BECORE_GTNR_INPUT_FORMAT] = BECORE_GTNR_INPUT_FORMAT_REG,
	[BECORE_GTNR_INPUT_LOSSY] = BECORE_GTNR_INPUT_LOSSY_REG,
	[BECORE_GTNR_INPUT_COMP] = BECORE_GTNR_INPUT_COMP_REG,
	[BECORE_GTNR_INPUT_WIDTH] = BECORE_GTNR_INPUT_WIDTH_REG,
	[BECORE_GTNR_INPUT_HEIGHT] = BECORE_GTNR_INPUT_HEIGHT_REG,
	[BECORE_GTNR_INPUT_STRIDE1] = BECORE_GTNR_INPUT_STRIDE1_REG,
	[BECORE_GTNR_INPUT_STRIDE2] = BECORE_GTNR_INPUT_STRIDE2_REG,
	[BECORE_GTNR_INPUT_BUSINFO] = BECORE_GTNR_INPUT_BUSINFO_REG,
	[BECORE_GTNR_INPUT_MAX_MO] = BECORE_GTNR_INPUT_MAX_MO_REG,
	[BECORE_GTNR_INPUT_MAX_BL] = BECORE_GTNR_INPUT_MAX_BL_REG,
	[BECORE_GTNR_INPUT_ENABLE] = BECORE_GTNR_INPUT_ENABLE_REG,
	[BECORE_GTNR_OUTPUT_FORMAT] = BECORE_GTNR_OUTPUT_FORMAT_REG,
	[BECORE_GTNR_OUTPUT_LOSSY] = BECORE_GTNR_OUTPUT_LOSSY_REG,
	[BECORE_GTNR_OUTPUT_COMP] = BECORE_GTNR_OUTPUT_COMP_REG,
	[BECORE_GTNR_OUTPUT_WIDTH] = BECORE_GTNR_OUTPUT_WIDTH_REG,
	[BECORE_GTNR_OUTPUT_HEIGHT] = BECORE_GTNR_OUTPUT_HEIGHT_REG,
	[BECORE_GTNR_OUTPUT_STRIDE1] = BECORE_GTNR_OUTPUT_STRIDE1_REG,
	[BECORE_GTNR_OUTPUT_STRIDE2] = BECORE_GTNR_OUTPUT_STRIDE2_REG,
	[BECORE_GTNR_OUTPUT_BUSINFO] = BECORE_GTNR_OUTPUT_BUSINFO_REG,
	[BECORE_GTNR_OUTPUT_MAX_MO] = BECORE_GTNR_OUTPUT_MAX_MO_REG,
	[BECORE_GTNR_OUTPUT_MAX_BL] = BECORE_GTNR_OUTPUT_MAX_BL_REG,
	[BECORE_GTNR_OUTPUT_ENABLE] = BECORE_GTNR_OUTPUT_ENABLE_REG,
};

/*
 * The physical-ultrawide 4000x3000 request carries YUVP's 4160x3120 lossy
 * SBWC intermediate into MCSC and writes output zero as linear NV21.  The
 * captured program used VOTF; a sequential run replaces only that transport
 * control so MCSC reads the completed driver-owned surface from memory.
 */
static const struct becore_mcsc_dma_profile becore_mcsc_input = {
	.votf_enable = 0x00400001,
	.businfo = 2,
	.max_bl = 0x10,
	.enable = 1,
};

#define BECORE_MCSC_INPUT_VOTF_STALL_LINES	GENMASK_U32(29, 16)

static u32 becore_mcsc_votf_enable(const u32 *requested_token)
{
	u32 token = becore_c2serv_token(requested_token,
					becore_c2serv_trs_lines_in_token, 0);

	return FIELD_PREP(BECORE_MCSC_INPUT_VOTF_STALL_LINES, token) | 1;
}


static const struct becore_mcsc_dma_profile becore_mcsc_output = {
	.data_format = 0x800,
	.comp_control = 0,
	.businfo = 0,
	.max_bl = 4,
	.enable = 1,
	.dither = 0x10,
};

static const u32 becore_mcsc_dma_regs[] = {
	[BECORE_MCSC_INPUT_VOTF] = BECORE_MCSC_INPUT_VOTF_REG,
	[BECORE_MCSC_INPUT_FORMAT] = BECORE_MCSC_INPUT_FORMAT_REG,
	[BECORE_MCSC_INPUT_LOSSY] = BECORE_MCSC_INPUT_LOSSY_REG,
	[BECORE_MCSC_INPUT_COMP] = BECORE_MCSC_INPUT_COMP_REG,
	[BECORE_MCSC_INPUT_WIDTH] = BECORE_MCSC_INPUT_WIDTH_REG,
	[BECORE_MCSC_INPUT_HEIGHT] = BECORE_MCSC_INPUT_HEIGHT_REG,
	[BECORE_MCSC_INPUT_STRIDE1] = BECORE_MCSC_INPUT_STRIDE1_REG,
	[BECORE_MCSC_INPUT_STRIDE2] = BECORE_MCSC_INPUT_STRIDE2_REG,
	[BECORE_MCSC_INPUT_BUSINFO] = BECORE_MCSC_INPUT_BUSINFO_REG,
	[BECORE_MCSC_INPUT_MAX_BL] = BECORE_MCSC_INPUT_MAX_BL_REG,
	[BECORE_MCSC_INPUT_ENABLE] = BECORE_MCSC_INPUT_ENABLE_REG,
	[BECORE_MCSC_OUTPUT_FORMAT] = BECORE_MCSC_OUTPUT_FORMAT_REG,
	[BECORE_MCSC_OUTPUT_COMP] = BECORE_MCSC_OUTPUT_COMP_REG,
	[BECORE_MCSC_OUTPUT_WIDTH] = BECORE_MCSC_OUTPUT_WIDTH_REG,
	[BECORE_MCSC_OUTPUT_HEIGHT] = BECORE_MCSC_OUTPUT_HEIGHT_REG,
	[BECORE_MCSC_OUTPUT_STRIDE1] = BECORE_MCSC_OUTPUT_STRIDE1_REG,
	[BECORE_MCSC_OUTPUT_STRIDE2] = BECORE_MCSC_OUTPUT_STRIDE2_REG,
	[BECORE_MCSC_OUTPUT_BUSINFO] = BECORE_MCSC_OUTPUT_BUSINFO_REG,
	[BECORE_MCSC_OUTPUT_MAX_BL] = BECORE_MCSC_OUTPUT_MAX_BL_REG,
	[BECORE_MCSC_OUTPUT_ENABLE] = BECORE_MCSC_OUTPUT_ENABLE_REG,
	[BECORE_MCSC_OUTPUT_DITHER] = BECORE_MCSC_OUTPUT_DITHER_REG,
	[BECORE_MCSC_DJAG_IMG_SIZE] = BECORE_MCSC_DJAG_IMG_SIZE_REG,
	[BECORE_MCSC_DJAG_PS_SRC_POS] = BECORE_MCSC_DJAG_PS_SRC_POS_REG,
	[BECORE_MCSC_DJAG_PS_SRC_SIZE] = BECORE_MCSC_DJAG_PS_SRC_SIZE_REG,
	[BECORE_MCSC_DJAG_PS_DST_SIZE] = BECORE_MCSC_DJAG_PS_DST_SIZE_REG,
	[BECORE_MCSC_DJAG_PS_H_RATIO] = BECORE_MCSC_DJAG_PS_H_RATIO_REG,
	[BECORE_MCSC_DJAG_PS_V_RATIO] = BECORE_MCSC_DJAG_PS_V_RATIO_REG,
};

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

static void becore_video_return_all(struct becore_device *becore,
				    enum vb2_buffer_state state);
static void becore_params_drain_idle(struct becore_device *becore);
static void becore_video_controls_ungrab(struct becore_device *becore);

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
struct becore_c2serv_desc {
	const char *name;
	u16 local_ip;
	u8 producers;
	u8 consumers;
};

static const struct becore_c2serv_desc becore_c2serv[BECORE_NUM_C2SERV] = {
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
struct becore_stream_crc {
	u32 offset;
	const char *name;
};

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

#define BECORE_STREAM_CRC_SEED_MASK	GENMASK_U32(7, 0)
#define BECORE_STREAM_CRC_RESULT_MASK	GENMASK_U32(15, 8)
#define BECORE_STREAM_CRC_RESULT_SHIFT	8

static const struct becore_stream_crc *
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

/* The bytes the last encode filled, which is not the allocation. */
static size_t becore_cmdq_encoded_size(const struct becore_cmdq_program *program)
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

static u32
becore_rgbp_input_storage_width(const struct becore_rgbp_input_profile *profile,
				const struct becore_raster *array)
{
	return ALIGN(array->width, profile->sbwc_block_width);
}

static u32
becore_rgbp_input_stride(const struct becore_rgbp_input_profile *profile,
			 const struct becore_raster *array)
{
	return becore_rgbp_input_storage_width(profile, array) *
	       profile->bytes_per_pixel;
}

static size_t
becore_rgbp_input_image_offset(const struct becore_rgbp_input_profile *profile,
			       const struct becore_raster *array)
{
	return (size_t)profile->header_stride * array->height;
}

static size_t
becore_rgbp_input_size(const struct becore_rgbp_input_profile *profile,
		       const struct becore_raster *array)
{
	size_t image_bytes = (size_t)becore_rgbp_input_stride(profile, array) *
			     array->height;

	return ALIGN(becore_rgbp_input_image_offset(profile, array) +
		     image_bytes, SZ_4K);
}

/*
 * Every input slot is sized for the live path, because that is what fills
 * them: ISPFE writes SBWC-compressed main Bayer straight into a slot, and its
 * own probe checks the size it is handed.  A harness profile therefore has to
 * fit inside that allocation rather than resize it, which is what
 * becore_input_profiles_validate() checks once at probe.
 */
static size_t becore_input_allocation_size(const struct becore_raster *array)
{
	return becore_rgbp_input_size(&becore_rgbp_inputs[BECORE_RGBP_INPUT_SBWC],
				      array);
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

/* The array raster is becore_raster_validate()'s; this checks the profiles. */
static int becore_input_profiles_validate(struct device *dev,
					  const struct becore_raster *array)
{
	unsigned int i;

	for (i = 0; i < BECORE_RGBP_INPUT_PROFILE_COUNT; i++) {
		const struct becore_rgbp_input_profile *profile =
			&becore_rgbp_inputs[i];

		if (!profile->bytes_per_pixel ||
		    !is_power_of_2(profile->sbwc_block_width))
			return dev_err_probe(dev, -EINVAL,
					     "input profile %u is degenerate\n",
					     i);
		if (becore_rgbp_input_size(profile, array) >
		    becore_input_allocation_size(array))
			return dev_err_probe(dev, -EINVAL,
					     "input profile %u wants %zu bytes, the slot holds %zu\n",
					     i, becore_rgbp_input_size(profile, array),
					     becore_input_allocation_size(array));
	}

	return 0;
}

/*
 * The same for the surfaces YUVP writes, and it is the divisors that make it
 * worth having: a compressed profile's stride and both plane sizes divide by
 * its block geometry, and its stride is only the expression written above
 * while the compressed rate is even.
 */
static int becore_output_profiles_validate(struct device *dev)
{
	unsigned int i;

	for (i = 0; i < BECORE_YUVP_OUTPUT_PROFILE_COUNT; i++) {
		const struct becore_yuvp_output_profile *profile =
			&becore_yuvp_outputs[i];

		if (!profile->mode) {
			if (!profile->bytes_per_pixel)
				return dev_err_probe(dev, -EINVAL,
						     "output profile %u is uncompressed and has no pixel size\n",
						     i);
			continue;
		}
		if (!profile->block_width || !profile->block_height ||
		    !profile->luma_height_align)
			return dev_err_probe(dev, -EINVAL,
					     "output profile %u is compressed and has no block geometry\n",
					     i);
		if (!profile->lossy_byte32num || profile->lossy_byte32num & 1)
			return dev_err_probe(dev, -EINVAL,
					     "output profile %u has a compressed rate of %u, and becore_yuvp_output_stride() states only even ones\n",
					     i, profile->lossy_byte32num);
	}

	return 0;
}

/*
 * Two conventions the Exynos ISP blocks share, so they live here rather than
 * with any one of them.
 *
 * Geometry registers pack two 16-bit halves into one word with the width in
 * the high one. Scaling ratios are a 20-bit fixed-point fraction of the
 * destination, truncated -- Samsung spells this GET_ZOOM_RATIO(in, out),
 * ((in) << MCSC_PRECISION) / (out) with MCSC_PRECISION 20. The captured
 * programs are the check on the rounding: DJAG's 3536 << 20 over 4000 is
 * 926941.18 and the vendor writes 926941.
 */
u32 becore_pack_size(u32 high, u32 low)
{
	return (high << 16) | low;
}

/*
 * A ratio is a Q20 fraction in a u32, so it describes a downscale of just
 * under 4096 and no further.  Past that the shifted numerator leaves 32 bits
 * and the truncation lands somewhere small: an in/out of exactly 4096 reads
 * back as **zero**, a scaler told to step nothing per output pixel, and 4097
 * reads back as unity.  Either way becore_sc_coeff_set() then picks the x8/8
 * filter for it, because both are at or below its first band.  Neither the
 * register nor the filter is a failed write, so say so rather than write it.
 */
int becore_zoom_ratio(u32 in, u32 out, u32 *ratio)
{
	u64 scaled;

	if (!out)
		return -EINVAL;
	scaled = div_u64((u64)in << BECORE_RATIO_SHIFT, out);
	if (scaled > U32_MAX)
		return -ERANGE;
	*ratio = scaled;

	return 0;
}

/*
 * Where MCSC's scaler starts, which is a function of its ratio rather than a
 * constant: half a source step when it stretches and nothing when it shrinks.
 * Ten captured POLY_SC0 programs say so -- the six that shrink write zero, and
 * the four that stretch write half the ratio, odd values included, 0x0ffdf4
 * giving 0x0007fefa.
 *
 * **It is three scalers' rule and not all four's**, which is the thing not to
 * generalise from the register's name. DJAG's pre-scaler has the same field in
 * the same shape and does not follow it: seven captured programs run it as a
 * stretcher, at ratios from 889105 to 974302 against a unity of 1048576, and
 * every one of them writes an origin of zero where this would put 444552 to
 * 487151. So becore_mcsc_djag_value() states zero because that is what the
 * silicon is told, not because nothing has moved yet.
 *
 * Two sources say so and they agree.  Samsung's kernel driver carries
 * `if (hratio < RATIO_X8_8) h_phase_offset = hratio >> 1;` in
 * is_scaler_set_poly_scaler_coef() and is_scaler_set_post_scaler_coef(), while
 * every caller of is_scaler_set_djag_init_phase_offset() passes a literal 0 --
 * four of them, unconditionally, on the line after the ratio is set.  And
 * Lyric, which is what actually programs this silicon, has no ratio arithmetic
 * in its DJAG function at all: lyric::McscCropAndUpScaleBlock's writes two
 * literal zeros and then the round mode's 1.
 */
u32 becore_scaler_init_phase(u32 ratio)
{
	return ratio < BECORE_RATIO_UNITY ? ratio >> 1 : 0;
}

/*
 * What a raster has to be before anything derives a register from it.  Every
 * clause here is a silent failure rather than a loud one, which is why they
 * are checked at all: none of them is reachable while the three rasters are
 * probe-time constants, and all of them become reachable the moment something
 * negotiates one.
 *
 * An extent has to fit the 16-bit half becore_pack_size() puts it in.  At
 * 65536 the shift walks into the other half instead of overflowing: an array
 * that wide encodes chain_src_img_size as 0x00000c30, a zero width, and no
 * register write fails.
 *
 * An odd extent has no whole last pair of anything.  The array is read out in
 * Bayer quads, the chain is the 4:2:0 surface YUVP writes and GTNR and MCSC
 * read back, and the scaled output is NV21 -- all three side their chroma on a
 * 2x2 grid the last row or column would fall off.
 */
#define BECORE_RASTER_EXTENT_MAX	U16_MAX

/*
 * The array's bound is tighter, and it comes from the two blocks that state a
 * radial fall-off centre.  BYR_DNS writes -(array extent / 2) and YUVNR writes
 * -(crop extent / 2), both into a 15-bit signed field, so the largest array
 * either can describe is twice that field's negative span.  The crop is cut
 * out of the array, so bounding the array bounds YUVNR's copy with it.
 */
#define BECORE_ARRAY_EXTENT_MAX		(BECORE_RGBP_DNS_CENTRE_MASK + 1)

/* One bound, so the two fields it is the bound for have to be one width. */
static_assert(BECORE_YUVNR_CENTRE_MASK == BECORE_RGBP_DNS_CENTRE_MASK);

/* See becore_chain_validate(); the chroma plane is what asks for the four. */
#define BECORE_CHAIN_WIDTH_ALIGN	4U

static int becore_raster_validate(struct device *dev, const char *name,
				  const struct becore_raster *raster, u32 max)
{
	if (!raster->width || !raster->height)
		return dev_err_probe(dev, -EINVAL,
				     "%s raster %ux%u has a zero extent\n",
				     name, raster->width, raster->height);
	if ((raster->width | raster->height) & 1)
		return dev_err_probe(dev, -EINVAL,
				     "%s raster %ux%u has an odd extent\n",
				     name, raster->width, raster->height);
	if (raster->width > max || raster->height > max)
		return dev_err_probe(dev, -EINVAL,
				     "%s raster %ux%u exceeds %u\n",
				     name, raster->width, raster->height, max);

	return 0;
}

/*
 * The chain raster is the one with a width alignment of its own, and it is
 * four rather than the two an even extent gives.  The chain surface carries
 * 4:2:0 chroma interleaved at half the luma width, and the DMA blocks that
 * plane the same way it blocks luma, so a merely even width leaves the chroma
 * with an odd sample count.
 *
 * MCSC's stage then never completes at all: not an error interrupt but a
 * timeout, which spends the run's whole budget and leaves the processors
 * unproven, so it is much better refused.  Measured across widths at every
 * residue -- 2578, 2582, 2586, 4158, 4162, 1506 and 1510 all hang where 2576,
 * 2580, 2584, 4160 and 1508 all run -- and it is the width alone: the chain's
 * height and both of the scaled output's extents are fine at merely even.
 * Every captured chain raster is a multiple of four.  [HW 2026-08-24]
 */
static int becore_chain_validate(struct device *dev,
				 const struct becore_raster *chain)
{
	int ret = becore_raster_validate(dev, "chain", chain,
					 BECORE_RASTER_EXTENT_MAX);

	if (ret)
		return ret;
	if (chain->width % BECORE_CHAIN_WIDTH_ALIGN)
		return dev_err_probe(dev, -EINVAL,
				     "chain raster %ux%u has a width that is not a multiple of %u\n",
				     chain->width, chain->height,
				     BECORE_CHAIN_WIDTH_ALIGN);

	return 0;
}

/*
 * DMSCCROP's window: the largest centred rectangle of the Bayer array that
 * has the aspect ratio the chain hands downstream. Cropping is what makes the
 * two aspects agree, so only one axis is ever narrowed; the other keeps the
 * whole array.
 *
 * This is a policy choice rather than an arithmetic one, and the vendor makes
 * a different one on some readouts -- it takes the aspect fit on every 4:3
 * request, but keeps an eight-pixel margin on the rear camera's 16:9 modes.
 * What is *not* a choice is that everything below reads the crop rather than
 * the array: the scaler's ratios are the crop over the destination, and that
 * holds bit-exactly on all eighteen captured programs where taking the array
 * would be right only where the two coincide.
 *
 * Both sizes are kept even, and the margin is widened to a multiple of four so
 * that the centred origin lands on an even pixel: a window on the wrong Bayer
 * phase swaps the colours with no other symptom. The odd-origin test below is
 * therefore unreachable by construction and is kept as a guard on that
 * reasoning rather than on the arithmetic.
 */
int becore_rgbp_crop(const struct becore_raster *array,
		     const struct becore_raster *chain,
		     struct becore_rect *crop)
{
	u32 array_w = array->width;
	u32 array_h = array->height;
	u32 out_w = chain->width;
	u32 out_h = chain->height;
	u32 width;

	if (!out_w || !out_h || !array_w || !array_h)
		return -EINVAL;
	/*
	 * Widening the margin to a multiple of four keeps the crop's sizes
	 * even only while the array's own dimensions are, and a Bayer array
	 * with an odd dimension has no whole last quad in any case.
	 */
	if ((array_w | array_h) & 1)
		return -EINVAL;

	width = 2 * (u32)DIV_ROUND_CLOSEST_ULL((u64)array_h * out_w,
					       2 * out_h);
	if (width <= array_w) {
		crop->width = width;
		crop->height = array_h;
	} else {
		u64 fit = (u64)array_w * out_h;

		crop->width = array_w;
		crop->height = 2 * (u32)DIV_ROUND_CLOSEST_ULL(fit, 2 * out_w);
	}
	if (crop->height > array_h || !crop->width || !crop->height)
		return -ERANGE;
	/*
	 * A centred window is only on the right Bayer phase when its margin
	 * is a multiple of four, so widen it by the two pixels that are in
	 * the way rather than accept an off-phase origin. That costs at most
	 * a two-pixel aspect error, which is what the vendor accepts as well:
	 * on both 16:9 readouts where the exact fit lands here it stops
	 * cropping altogether, and widening reaches the same answer.
	 */
	crop->width += (array_w - crop->width) % 4;
	crop->height += (array_h - crop->height) % 4;
	crop->x = (array_w - crop->width) / 2;
	crop->y = (array_h - crop->height) / 2;
	if ((crop->x | crop->y) & 1)
		return -ERANGE;

	return 0;
}

/*
 * Every word RGBP's typed input profile supplies, computed whether or not the
 * caller wants the value: passing a null pointer is the probe-time validation
 * pass, and short-circuiting it there would mean the four crop-derived words
 * are the only ones never checked before the hardware sees them.
 */
int becore_rgbp_input_value(const struct becore_rgbp_input_profile *profile,
			    const struct becore_raster *array,
			    const struct becore_raster *chain,
			    u32 index, u32 reg, u32 *value)
{
	u32 result;

	if (index >= BECORE_RGBP_INPUT_WORD_COUNT ||
	    reg != becore_rgbp_input_regs[index])
		return -EINVAL;

	switch (index) {
	case BECORE_RGBP_CHAIN_SRC_SIZE:
		result = becore_pack_size(array->width, array->height);
		break;
	case BECORE_RGBP_CHAIN_DST_SIZE:
	case BECORE_RGBP_SC_DST_SIZE:
		result = becore_pack_size(chain->width, chain->height);
		break;
	case BECORE_RGBP_CROP_SIZE:
	case BECORE_RGBP_CROP_START:
	case BECORE_RGBP_SC_H_RATIO:
	case BECORE_RGBP_SC_V_RATIO: {
		struct becore_rect crop;
		int ret = becore_rgbp_crop(array, chain, &crop);

		if (ret)
			return ret;
		if (index == BECORE_RGBP_CROP_SIZE)
			result = becore_pack_size(crop.width, crop.height);
		else if (index == BECORE_RGBP_CROP_START)
			result = becore_pack_size(crop.x, crop.y);
		else if (index == BECORE_RGBP_SC_H_RATIO)
			ret = becore_zoom_ratio(crop.width, chain->width,
						&result);
		else
			ret = becore_zoom_ratio(crop.height, chain->height,
						&result);
		if (ret)
			return ret;
		break;
	}
	case BECORE_RGBP_INPUT_FORMAT:
		result = profile->data_format;
		break;
	case BECORE_RGBP_INPUT_COMP:
		result = profile->comp_control;
		break;
	case BECORE_RGBP_INPUT_ACTIVE_WIDTH:
		result = array->width;
		break;
	case BECORE_RGBP_INPUT_HEIGHT:
		result = array->height;
		break;
	case BECORE_RGBP_INPUT_STRIDE:
		result = becore_rgbp_input_stride(profile, array);
		break;
	case BECORE_RGBP_INPUT_HEADER_STRIDE:
		result = profile->header_stride;
		break;
	case BECORE_RGBP_INPUT_BUSINFO:
		result = profile->businfo;
		break;
	case BECORE_RGBP_INPUT_ENABLE:
		result = 1;
		break;
	case BECORE_RGBP_INPUT_STORAGE_WIDTH:
		result = becore_rgbp_input_storage_width(profile, array);
		break;
	default:
		return -EINVAL;
	}

	if (value)
		*value = result;

	return 0;
}

const struct becore_rgbp_input_profile *
becore_rgbp_input_profile(const struct becore_device *becore)
{
	return &becore_rgbp_inputs[becore->active_input_profile];
}

static const struct becore_yuvp_output_profile *
becore_yuvp_output_profile(const struct becore_device *becore)
{
	return &becore_yuvp_outputs[becore->active_output_profile];
}

static u32
becore_yuvp_output_stride(const struct becore_yuvp_output_profile *profile,
			  const struct becore_raster *chain)
{
	if (!profile->mode)
		return chain->width * profile->bytes_per_pixel;

	/*
	 * A compressed surface's stride is a count of whole blocks, each of a
	 * fixed compressed size -- Samsung's is_hw_dma_get_payload_stride()
	 * for a lossy 32x4 surface.  A width that does not fill its last block
	 * still occupies one.
	 *
	 * That function branches on a 32- or 64-byte alignment, and the two
	 * branches collapse to this one expression while lossy_byte32num is
	 * even: the odd-rate correction term vanishes and both alignments
	 * become no-ops.  Ours is 2, so the alignment does not have to be
	 * known -- which is the only reason it is not carried here.  An odd
	 * rate would need both, and is refused where the profiles are checked.
	 */
	return DIV_ROUND_UP(chain->width, profile->block_width) *
	       profile->lossy_byte32num * profile->block_width;
}

/*
 * The chain surface: what YUVP writes and GTNR and MCSC read back.  One
 * surface, so one description, and the profile that writes it is the one that
 * knows it -- a reader that states the format again is a second answer to a
 * question already answered, and it is the answer that goes wrong silently,
 * because a reader that disagrees about a layout still reads a frame's worth
 * of bytes and produces a picture.
 *
 * It is always the lossy-SBWC profile, not whichever one is active: MCSC is
 * refused a run against any other (see becore_run_frame()), and GTNR's program
 * is written against this one.  So this names the surface rather than taking
 * the caller's, which also lets a value function with no device pointer ask.
 */
static const struct becore_yuvp_output_profile *becore_chain_surface(void)
{
	return &becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];
}

static u32 becore_chain_stride(const struct becore_raster *chain)
{
	return becore_yuvp_output_stride(becore_chain_surface(), chain);
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

static size_t
becore_yuvp_output_plane2_offset(const struct becore_yuvp_output_profile *profile,
				 const struct becore_raster *chain)
{
	u32 stride = becore_yuvp_output_stride(profile, chain);
	u32 luma_height;
	u32 luma_rows;

	if (!profile->mode)
		return ALIGN((size_t)stride * chain->height, SZ_4K);

	luma_height = ALIGN(chain->height, profile->luma_height_align);
	luma_rows = DIV_ROUND_UP(luma_height, profile->block_height);

	return (size_t)stride * luma_rows + profile->plane_gap;
}

static size_t
becore_yuvp_output_size(const struct becore_yuvp_output_profile *profile,
			const struct becore_raster *chain)
{
	u32 stride = becore_yuvp_output_stride(profile, chain);
	u32 chroma_height = DIV_ROUND_UP(chain->height, 2);
	u32 chroma_rows;
	size_t chroma_bytes;

	if (profile->mode)
		chroma_rows = DIV_ROUND_UP(chroma_height, profile->block_height);
	else
		chroma_rows = chroma_height;
	chroma_bytes = (size_t)stride * chroma_rows;

	return ALIGN(becore_yuvp_output_plane2_offset(profile, chain) +
		     chroma_bytes, SZ_4K);
}

static size_t
becore_yuvp_packed_plane2_offset(const struct becore_yuvp_output_profile *profile,
				 const struct becore_raster *chain)
{
	return (size_t)becore_yuvp_output_stride(profile, chain) * chain->height;
}

static size_t
becore_yuvp_packed_output_size(const struct becore_yuvp_output_profile *profile,
			       const struct becore_raster *chain)
{
	u32 chroma_height = DIV_ROUND_UP(chain->height, 2);

	return becore_yuvp_packed_plane2_offset(profile, chain) +
	       (size_t)becore_yuvp_output_stride(profile, chain) * chroma_height;
}

static size_t becore_active_output_plane2_offset(const struct becore_device *becore)
{
	const struct becore_yuvp_output_profile *profile =
		becore_yuvp_output_profile(becore);

	if (becore->active_output_packed)
		return becore_yuvp_packed_plane2_offset(profile, &becore->chain);

	return becore_yuvp_output_plane2_offset(profile, &becore->chain);
}

static size_t becore_active_output_size(const struct becore_device *becore)
{
	const struct becore_yuvp_output_profile *profile =
		becore_yuvp_output_profile(becore);

	if (becore->active_output_packed)
		return becore_yuvp_packed_output_size(profile, &becore->chain);

	return becore_yuvp_output_size(profile, &becore->chain);
}

static size_t
becore_yuvp_output_allocation_size(const struct becore_raster *chain)
{
	const struct becore_yuvp_output_profile *sbwcl =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];
	const struct becore_yuvp_output_profile *p010 =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_P010];

	return max(becore_yuvp_output_size(sbwcl, chain),
		   becore_yuvp_output_size(p010, chain));
}

static int becore_yuvp_output_value(struct becore_device *becore, u32 index,
				    u32 reg, u32 *value)
{
	const struct becore_yuvp_output_profile *profile =
		becore_yuvp_output_profile(becore);

	if (index >= BECORE_YUVP_OUTPUT_WORD_COUNT ||
	    reg != becore_yuvp_output_regs[index])
		return -EINVAL;
	if (!value)
		return 0;

	switch (index) {
	case BECORE_YUVP_OUTPUT_VOTF:
		*value = profile->votf_enable;
		break;
	case BECORE_YUVP_OUTPUT_FORMAT:
		*value = profile->data_format;
		break;
	case BECORE_YUVP_OUTPUT_LOSSY:
		*value = profile->lossy_byte32num;
		break;
	case BECORE_YUVP_OUTPUT_MODE:
		*value = profile->mode;
		break;
	case BECORE_YUVP_OUTPUT_WIDTH:
		*value = becore->chain.width;
		break;
	case BECORE_YUVP_OUTPUT_HEIGHT:
		*value = becore->chain.height;
		break;
	case BECORE_YUVP_OUTPUT_STRIDE1:
	case BECORE_YUVP_OUTPUT_STRIDE2:
		*value = becore_yuvp_output_stride(profile, &becore->chain);
		break;
	case BECORE_YUVP_OUTPUT_BUSINFO:
		*value = profile->businfo;
		break;
	case BECORE_YUVP_OUTPUT_ENABLE:
		*value = 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int becore_gtnr_dma_value(const struct becore_raster *chain, u32 index,
				 u32 reg, u32 *value)
{
	if (index >= BECORE_GTNR_DMA_WORD_COUNT ||
	    reg != becore_gtnr_dma_regs[index])
		return -EINVAL;
	if (!value)
		return 0;

	switch (index) {
	case BECORE_GTNR_INPUT_VOTF:
		*value = becore_gtnr_input.votf_enable;
		break;
	case BECORE_GTNR_INPUT_FORMAT:
		*value = becore_chain_surface()->data_format;
		break;
	case BECORE_GTNR_INPUT_LOSSY:
		*value = becore_chain_surface()->lossy_byte32num;
		break;
	case BECORE_GTNR_INPUT_COMP:
		*value = becore_chain_surface()->mode;
		break;
	case BECORE_GTNR_INPUT_WIDTH:
		*value = chain->width;
		break;
	case BECORE_GTNR_INPUT_HEIGHT:
		*value = chain->height;
		break;
	case BECORE_GTNR_INPUT_STRIDE1:
	case BECORE_GTNR_INPUT_STRIDE2:
		*value = becore_chain_stride(chain);
		break;
	case BECORE_GTNR_INPUT_BUSINFO:
		*value = becore_gtnr_input.businfo;
		break;
	case BECORE_GTNR_INPUT_MAX_MO:
		*value = becore_gtnr_input.max_mo;
		break;
	case BECORE_GTNR_INPUT_MAX_BL:
		*value = becore_gtnr_input.max_bl;
		break;
	case BECORE_GTNR_INPUT_ENABLE:
		*value = becore_gtnr_input.enable;
		break;
	case BECORE_GTNR_OUTPUT_FORMAT:
		*value = becore_chain_surface()->data_format;
		break;
	case BECORE_GTNR_OUTPUT_LOSSY:
		*value = becore_chain_surface()->lossy_byte32num;
		break;
	case BECORE_GTNR_OUTPUT_COMP:
		*value = becore_chain_surface()->mode;
		break;
	case BECORE_GTNR_OUTPUT_WIDTH:
		*value = chain->width;
		break;
	case BECORE_GTNR_OUTPUT_HEIGHT:
		*value = chain->height;
		break;
	case BECORE_GTNR_OUTPUT_STRIDE1:
	case BECORE_GTNR_OUTPUT_STRIDE2:
		*value = becore_chain_stride(chain);
		break;
	case BECORE_GTNR_OUTPUT_BUSINFO:
		*value = becore_gtnr_output.businfo;
		break;
	case BECORE_GTNR_OUTPUT_MAX_MO:
		*value = becore_gtnr_output.max_mo;
		break;
	case BECORE_GTNR_OUTPUT_MAX_BL:
		*value = becore_gtnr_output.max_bl;
		break;
	case BECORE_GTNR_OUTPUT_ENABLE:
		*value = becore_gtnr_output.enable;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * The scaled output's stride: its width rounded up to 64 bytes, one byte to a
 * luma sample.  4000 becomes 4032, which is what the vendor's every captured
 * program writes and what the driver used to carry as 0xfc0.  It is the
 * node's `bytesperline` as well, so a stated one would be a promise to
 * userspace that the DMA had stopped keeping.
 */
#define BECORE_MCSC_OUTPUT_STRIDE_ALIGN	64

static u32 becore_mcsc_output_stride(const struct becore_raster *output)
{
	return ALIGN(output->width, BECORE_MCSC_OUTPUT_STRIDE_ALIGN);
}

static size_t
becore_mcsc_output_plane2_offset(const struct becore_raster *output)
{
	return (size_t)becore_mcsc_output_stride(output) * output->height;
}

static size_t
becore_mcsc_output_active_size(const struct becore_raster *output)
{
	size_t chroma = (size_t)becore_mcsc_output_stride(output) *
			DIV_ROUND_UP(output->height, 2);

	return becore_mcsc_output_plane2_offset(output) + chroma;
}

static size_t becore_mcsc_output_size(const struct becore_raster *output)
{
	return ALIGN(becore_mcsc_output_active_size(output), SZ_4K);
}

/*
 * How much of DJAG's input the output is taken from. The captured program
 * crops 3536 x 2652 out of the 4160 x 3120 YUVP surface and scales that up to
 * fill 4000 x 3000, reserving a margin the vendor spends on electronic
 * stabilisation. This driver has no stabilisation to spend it on, so it takes
 * the whole raster and downscales instead, covering the full field.
 *
 * These say that against the raster rather than beside it. A margin carried
 * separately would have gone on describing the raster the driver was built
 * against, and DJAG would have cropped a window larger than its own input the
 * first time the raster moved.
 */
static u32 becore_mcsc_djag_crop_width(const struct becore_raster *chain)
{
	return chain->width;
}

static u32 becore_mcsc_djag_crop_height(const struct becore_raster *chain)
{
	return chain->height;
}

/*
 * The crop is centred in the scaler's input, so its origin is derived rather
 * than carried. An odd margin would land the window off a chroma boundary on a
 * 4:2:0 output, so refuse it instead of silently rounding.
 */
static int becore_mcsc_djag_origin(const struct becore_raster *chain,
				   u32 *x, u32 *y)
{
	if (becore_mcsc_djag_crop_width(chain) > chain->width ||
	    becore_mcsc_djag_crop_height(chain) > chain->height)
		return -ERANGE;
	*x = (chain->width - becore_mcsc_djag_crop_width(chain)) / 2;
	*y = (chain->height - becore_mcsc_djag_crop_height(chain)) / 2;
	if ((*x | *y) & 1)
		return -ERANGE;

	return 0;
}

static int
becore_mcsc_dma_value(const struct becore_raster *chain,
		      const struct becore_raster *output, u32 index, u32 reg,
		      enum becore_mcsc_input_transport transport,
		      const u32 *requested_token, u32 *value)
{
	if (index >= BECORE_MCSC_DMA_WORD_COUNT ||
	    reg != becore_mcsc_dma_regs[index])
		return -EINVAL;
	if (!value)
		return 0;

	switch (index) {
	case BECORE_MCSC_INPUT_VOTF:
		/*
		 * Lyric builds this word two ways, and which one depends on
		 * the direction rather than on the block: a consumer gets its
		 * stall-line count in bits 29:16 with the enable in bit 0, and
		 * a producer gets a constant 2 with the enable in bit 0 -- the
		 * captured 0x00400001 here and 3 on YUVP's side.  The stall
		 * count has to be the same number as this endpoint's C2SERV
		 * token or the block and the fabric wait for different things,
		 * so generating it keeps them together when the token is
		 * swept.  There is one such register per DMA engine and none
		 * per plane.
		 */
		*value = transport == BECORE_MCSC_INPUT_MEMORY ? 0 :
			 becore_mcsc_votf_enable(requested_token);
		break;
	case BECORE_MCSC_INPUT_FORMAT:
		*value = becore_chain_surface()->data_format;
		break;
	case BECORE_MCSC_INPUT_LOSSY:
		*value = becore_chain_surface()->lossy_byte32num;
		break;
	case BECORE_MCSC_INPUT_COMP:
		*value = becore_chain_surface()->mode;
		break;
	case BECORE_MCSC_INPUT_WIDTH:
		*value = chain->width;
		break;
	case BECORE_MCSC_INPUT_HEIGHT:
		*value = chain->height;
		break;
	case BECORE_MCSC_INPUT_STRIDE1:
	case BECORE_MCSC_INPUT_STRIDE2:
		*value = becore_chain_stride(chain);
		break;
	case BECORE_MCSC_INPUT_BUSINFO:
		*value = becore_mcsc_input.businfo;
		break;
	case BECORE_MCSC_INPUT_MAX_BL:
		*value = becore_mcsc_input.max_bl;
		break;
	case BECORE_MCSC_INPUT_ENABLE:
		*value = becore_mcsc_input.enable;
		break;
	case BECORE_MCSC_OUTPUT_FORMAT:
		*value = becore_mcsc_output.data_format;
		break;
	case BECORE_MCSC_OUTPUT_COMP:
		*value = becore_mcsc_output.comp_control;
		break;
	case BECORE_MCSC_OUTPUT_WIDTH:
		*value = output->width;
		break;
	case BECORE_MCSC_OUTPUT_HEIGHT:
		*value = output->height;
		break;
	case BECORE_MCSC_OUTPUT_STRIDE1:
	case BECORE_MCSC_OUTPUT_STRIDE2:
		*value = becore_mcsc_output_stride(output);
		break;
	case BECORE_MCSC_OUTPUT_BUSINFO:
		*value = becore_mcsc_output.businfo;
		break;
	case BECORE_MCSC_OUTPUT_MAX_BL:
		*value = becore_mcsc_output.max_bl;
		break;
	case BECORE_MCSC_OUTPUT_ENABLE:
		*value = becore_mcsc_output.enable;
		break;
	case BECORE_MCSC_OUTPUT_DITHER:
		*value = becore_mcsc_output.dither;
		break;
	case BECORE_MCSC_DJAG_IMG_SIZE:
		*value = becore_pack_size(chain->width, chain->height);
		break;
	case BECORE_MCSC_DJAG_PS_SRC_POS: {
		u32 x, y;
		int ret = becore_mcsc_djag_origin(chain, &x, &y);

		if (ret)
			return ret;
		*value = becore_pack_size(x, y);
		break;
	}
	case BECORE_MCSC_DJAG_PS_SRC_SIZE:
		*value = becore_pack_size(becore_mcsc_djag_crop_width(chain),
					  becore_mcsc_djag_crop_height(chain));
		break;
	case BECORE_MCSC_DJAG_PS_DST_SIZE:
		*value = becore_pack_size(output->width, output->height);
		break;
	case BECORE_MCSC_DJAG_PS_H_RATIO:
		return becore_zoom_ratio(becore_mcsc_djag_crop_width(chain),
					 output->width, value);
	case BECORE_MCSC_DJAG_PS_V_RATIO:
		return becore_zoom_ratio(becore_mcsc_djag_crop_height(chain),
					 output->height, value);
	default:
		return -EINVAL;
	}

	return 0;
}

static size_t
becore_gtnr_surface_plane2_offset(const struct becore_raster *chain)
{
	return becore_yuvp_output_plane2_offset(becore_chain_surface(), chain);
}

static size_t becore_gtnr_surface_size(const struct becore_raster *chain)
{
	return becore_yuvp_output_size(becore_chain_surface(), chain);
}

static dma_addr_t becore_gtnr_address_dma(struct becore_device *becore, u32 reg)
{
	switch (reg) {
	case BECORE_GTNR_INPUT_PLANE1_REG:
		return becore->output.dma;
	case BECORE_GTNR_INPUT_PLANE2_REG:
		return becore->output.dma + becore_gtnr_surface_plane2_offset(&becore->chain);
	case BECORE_GTNR_OUTPUT_PLANE1_REG:
		return becore->gtnr_output.dma;
	case BECORE_GTNR_OUTPUT_PLANE2_REG:
		return becore->gtnr_output.dma +
		       becore_gtnr_surface_plane2_offset(&becore->chain);
	default:
		return DMA_MAPPING_ERROR;
	}
}

static dma_addr_t becore_mcsc_address_dma(struct becore_device *becore, u32 reg)
{
	switch (reg) {
	case BECORE_MCSC_INPUT_PLANE1_REG:
		return becore->output.dma;
	case BECORE_MCSC_INPUT_PLANE2_REG:
		return becore->output.dma +
		       becore_yuvp_output_plane2_offset(becore_chain_surface(),
							&becore->chain);
	case BECORE_MCSC_OUTPUT_PLANE1_REG:
		return becore->mcsc_dest_dma;
	case BECORE_MCSC_OUTPUT_PLANE2_REG:
		return becore->mcsc_dest_dma +
		       becore_mcsc_output_plane2_offset(&becore->scaled);
	default:
		return DMA_MAPPING_ERROR;
	}
}

static u32 becore_typed_word_count(enum becore_block_id id)
{
	if (id == BECORE_RGBP)
		return BECORE_RGBP_INPUT_WORD_COUNT;
	if (id == BECORE_YUVP)
		return BECORE_YUVP_OUTPUT_WORD_COUNT;

	return 0;
}

static int becore_typed_value(struct becore_device *becore,
			      enum becore_block_id id, u32 index, u32 reg,
			      u32 *value)
{
	if (id == BECORE_RGBP)
		return becore_rgbp_input_value(becore_rgbp_input_profile(becore),
					       &becore->array, &becore->chain,
					       index, reg, value);
	if (id == BECORE_YUVP)
		return becore_yuvp_output_value(becore, index, reg, value);

	return -EINVAL;
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

static int becore_recipe_records_validate(struct becore_device *becore,
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

static int becore_recipe_validate(struct becore_device *becore)
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
	 */
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
static int becore_override_check(u32 reg)
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

static int becore_gtnr_recipe_validate(struct becore_device *becore)
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

static int becore_encode_gtnr(struct becore_device *becore)
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

static int becore_recipe_generate(struct becore_device *becore)
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

static int becore_mcsc_recipe_generate(struct becore_device *becore)
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

static int becore_mcsc_recipe_validate(struct becore_device *becore)
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

static int
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
static int becore_stream_power_get(struct becore_device *becore)
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

static void becore_stream_power_put(struct becore_device *becore)
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
	u32 value;
	int ret;

	writel_relaxed(1, base + BECORE_C2SERV_SW_RESET);
	ret = readl_poll_timeout(base + BECORE_C2SERV_SW_RESET, value, !value,
				 1, BECORE_RESET_TIMEOUT_US);
	if (ret)
		dev_err(becore->dev, "%s reset timed out (%#010x)\n",
			becore_c2serv[id].name, value);

	return ret;
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
static void becore_free_surfaces(struct becore_device *becore)
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

static int becore_alloc_surfaces(struct becore_device *becore)
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
	becore_stream_power_put(becore);
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
	if (streaming) {
		becore_video_controls_ungrab(becore);
		vb2_queue_error(&becore->queue);
		becore_video_return_all(becore, VB2_BUF_STATE_ERROR);
	}
	wait_event(input->callback_wait,
		   refcount_read(&input->callback_users) == 1);
}
EXPORT_SYMBOL_GPL(exynos_becore_input_disconnect);

static struct exynos_becore_input *
becore_input_callback_get(struct becore_device *becore)
{
	struct exynos_becore_input *input = becore->input_producer;

	lockdep_assert_held(&becore->lock);
	if (!input || input->disconnected)
		return NULL;
	refcount_inc(&input->callback_users);

	return input;
}

static void becore_input_callback_put(struct exynos_becore_input *input)
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

	/*
	 * The caller has quiesced the producer at a completed-frame boundary.
	 * Nothing is synced: ISPFE wrote these pages to DRAM and RGBP will read
	 * them from DRAM, and no CPU mapping was read or written in between.
	 */
	slot->buffer.staged_bytes = slot->buffer.size;
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
	if (*ppos == 0)
		*staged_bytes = 0;
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

static int becore_run_frame(struct becore_device *becore, u32 input_profile,
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
	} else {
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

/* ---- the input subdevice ------------------------------------------------ */

/*
 * A sink pad on the producer's graph, and the reason the back end needs one:
 * everything it knows about the frame arriving from the front end is a
 * compiled-in profile, including the Bayer phase, which differs between the
 * three cameras. The front end already negotiates the mosaic on its own pads.
 * The fact exists in the system and the back end could not see it.
 *
 * The pad carries no format of its own. A sink whose producer is a fixed
 * hardware path has nothing to negotiate: what arrives is what the front end
 * sends, so get_fmt reports the remote pad's format and set_fmt is get_fmt.
 * That is deliberate -- offering a settable format here would let userspace
 * tell the driver something the hardware contradicts, with no way to arbitrate.
 */
/* The format the producer says it is sending, or -EPIPE if nothing is. */
static int becore_input_format(struct becore_device *becore,
			       struct v4l2_mbus_framefmt *format)
{
	struct v4l2_subdev_format remote = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct media_pad *pad;
	struct v4l2_subdev *sd;
	int ret;

	pad = media_pad_remote_pad_first(&becore->sink_pad);
	if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
		return -EPIPE;
	sd = media_entity_to_v4l2_subdev(pad->entity);
	remote.pad = pad->index;
	ret = v4l2_subdev_call_state_active(sd, pad, get_fmt, &remote);
	if (ret)
		return ret;

	*format = remote.format;

	return 0;
}

/*
 * Take the producer's mosaic and the raster it sends them on, or keep what the
 * driver already has if there is no producer to ask -- which is the offline
 * loop, where the staged frame is the ultrawide's and so is the default.
 *
 * The two are not kept the same way, and the difference is which of them means
 * anything without a producer. input_code is per-stream state whose only
 * source is the remote pad, so with nothing there it returns to the
 * compiled-in default; the array raster is device state that has to describe
 * the slots whatever is or is not attached, so it is left as it stands.
 *
 * A code the table cannot place is refused rather than guessed. The phase
 * decides which of the four quads the demosaic reads as red, so a wrong one is
 * not a subtle error, and a refusal at STREAMON is a far better failure than a
 * picture with its colours swapped.
 *
 * A raster is refused on three counts, and only the first is about the number
 * itself. It has to be one the register fields can carry; it has to lay a
 * frame out in exactly the slot the producer was handed; and there has to be
 * a crop of it that reaches the chain, which is the one derivation between the
 * two rasters that can fail on its own. Everything else the raster feeds is
 * range-checked where it is encoded.
 *
 * The middle one is an equality and not a bound, which looks stricter than it
 * needs to be and is not. A slot the producer filled stages its whole
 * allocation, and becore_recipe_validate() then requires the staged length to
 * equal what the raster and the profile say the frame is -- so a raster that
 * merely *fits* is accepted here and refuses every frame afterwards, which is
 * a far worse failure than a refusal at STREAMON.
 */
static int becore_latch_input_format(struct becore_device *becore)
{
	struct v4l2_mbus_framefmt format;
	struct becore_raster array;
	struct becore_rect crop;
	int ret;

	lockdep_assert_held(&becore->lock);

	ret = becore_input_format(becore, &format);
	if (ret == -EPIPE) {
		becore->input_code = BECORE_INPUT_DEFAULT_CODE;
		return 0;
	}
	if (ret)
		return ret;
	if (becore_bayer_phase(format.code) < 0) {
		dev_err(becore->dev, "producer sends mosaic 0x%04x, which this driver cannot place\n",
			format.code);
		return -EINVAL;
	}

	array.width = format.width;
	array.height = format.height;
	ret = becore_raster_validate(becore->dev, "array", &array,
				     BECORE_ARRAY_EXTENT_MAX);
	if (ret)
		return ret;
	ret = becore_input_profiles_validate(becore->dev, &array);
	if (ret)
		return ret;
	if (becore_input_allocation_size(&array) !=
	    becore->inputs[0].buffer.size) {
		dev_err(becore->dev,
			"producer sends %ux%u, which lays out in %zu bytes a slot where the slots are %zu\n",
			array.width, array.height,
			becore_input_allocation_size(&array),
			becore->inputs[0].buffer.size);
		return -ENOSPC;
	}
	ret = becore_rgbp_crop(&array, &becore->chain, &crop);
	if (ret) {
		dev_err(becore->dev,
			"no crop of %ux%u reaches the %ux%u chain\n",
			array.width, array.height,
			becore->chain.width, becore->chain.height);
		return ret;
	}

	becore->input_code = format.code;
	becore->array = array;

	return 0;
}

static int becore_sd_init_state(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state)
{
	struct becore_device *becore = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *sink =
		v4l2_subdev_state_get_format(state, 0);

	sink->code = BECORE_INPUT_DEFAULT_CODE;
	sink->width = becore->array.width;
	sink->height = becore->array.height;
	sink->field = V4L2_FIELD_NONE;
	sink->colorspace = V4L2_COLORSPACE_RAW;
	sink->ycbcr_enc = V4L2_YCBCR_ENC_601;
	sink->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	sink->xfer_func = V4L2_XFER_FUNC_NONE;

	return 0;
}

static int becore_sd_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(becore_input_codes))
		return -EINVAL;
	code->code = becore_input_codes[code->index];

	return 0;
}

/*
 * The producer decides, so a set is a get, and what is reported is what link
 * validation last saw the producer send.
 *
 * **This must not ask the remote.** `v4l2_subdev_link_validate()` locks both
 * subdevs' states and then calls the sink's `get_fmt`, so a `get_fmt` that
 * fetches the source's format takes a lock its own caller is holding. That is
 * a self-deadlock, and it hangs `STREAMON` on the *producer's* video node --
 * where the back end is only a pad on the graph and nothing about it is being
 * used. The stored format is kept in step from `link_validate` below instead.
 */
static int becore_sd_get_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *format)
{
	if (format->pad)
		return -EINVAL;
	format->format = *v4l2_subdev_state_get_format(state, 0);

	return 0;
}

/*
 * Take the producer's format rather than compare against it.
 *
 * A sink whose producer is a fixed hardware path has nothing to negotiate:
 * what arrives is what the front end sends, so the default validation -- which
 * refuses a link whose two ends disagree -- would refuse every pipeline until
 * userspace had told the back end what the hardware was already doing.
 *
 * Nothing is refused here, and that is deliberate too. The back end cannot
 * place every mosaic or work at every raster, but the producer's raw path does
 * not go through the back end at all, so neither is a reason to stop a raw
 * capture. becore_latch_input_format() refuses both at the back end's own
 * STREAMON, which is where it matters.
 */
static int becore_sd_link_validate(struct v4l2_subdev *sd,
				   struct media_link *link,
				   struct v4l2_subdev_format *source_fmt,
				   struct v4l2_subdev_format *sink_fmt)
{
	struct v4l2_subdev_state *state = v4l2_subdev_get_locked_active_state(sd);

	if (state)
		*v4l2_subdev_state_get_format(state, 0) = source_fmt->format;

	return 0;
}

static const struct v4l2_subdev_pad_ops becore_subdev_pad_ops = {
	.enum_mbus_code = becore_sd_enum_mbus_code,
	.get_fmt = becore_sd_get_fmt,
	.set_fmt = becore_sd_get_fmt,
	.link_validate = becore_sd_link_validate,
};

static const struct v4l2_subdev_ops becore_subdev_ops = {
	.pad = &becore_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops becore_subdev_internal_ops = {
	.init_state = becore_sd_init_state,
};

static const struct media_entity_operations becore_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* ---- processed NV21 capture queue -------------------------------------- */

static void becore_video_fill_pix(const struct becore_raster *output,
				  struct v4l2_pix_format *pix)
{
	pix->width = output->width;
	pix->height = output->height;
	pix->pixelformat = V4L2_PIX_FMT_NV21;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = becore_mcsc_output_stride(output);
	pix->sizeimage = becore_mcsc_output_active_size(output);
	/* The captured recipe uses a full-range BT.601 RGB-to-YUV matrix. */
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->flags = 0;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static void becore_video_return_all(struct becore_device *becore,
				    enum vb2_buffer_state state)
{
	struct becore_video_buffer *buf, *tmp;
	LIST_HEAD(done);

	spin_lock_irq(&becore->queue_lock);
	list_splice_tail_init(&becore->queued_outputs, &done);
	spin_unlock_irq(&becore->queue_lock);

	list_for_each_entry_safe(buf, tmp, &done, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
}

/*
 * The two gains the Bayer denoiser's noise factors are multiplied by, kept
 * where the encoder can reach them without taking the control handler's lock
 * under the device's.  Both controls are grabbed for the length of a stream,
 * so this cannot move under a frame.
 */
static int becore_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct becore_device *becore =
		container_of(ctrl->handler, struct becore_device, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_RED_BALANCE:
		WRITE_ONCE(becore->encode_balance_red, ctrl->val);
		return 0;
	case V4L2_CID_BLUE_BALANCE:
		WRITE_ONCE(becore->encode_balance_blue, ctrl->val);
		return 0;
	}

	return -EINVAL;
}

static const struct v4l2_ctrl_ops becore_ctrl_ops = {
	.s_ctrl = becore_s_ctrl,
};

static void becore_video_controls_snapshot(struct becore_device *becore,
					   struct exynos_becore_input_stream_config *config)
{
	v4l2_ctrl_lock(becore->red_balance);
	config->red_balance = becore->red_balance->val;
	config->blue_balance = becore->blue_balance->val;
	__v4l2_ctrl_grab(becore->red_balance, true);
	__v4l2_ctrl_grab(becore->blue_balance, true);
	v4l2_ctrl_unlock(becore->red_balance);
}

static void becore_video_controls_ungrab(struct becore_device *becore)
{
	v4l2_ctrl_lock(becore->red_balance);
	__v4l2_ctrl_grab(becore->red_balance, false);
	__v4l2_ctrl_grab(becore->blue_balance, false);
	v4l2_ctrl_unlock(becore->red_balance);
}

static void becore_video_discard_ready(struct becore_device *becore)
{
	unsigned int i;

	mutex_lock(&becore->lock);
	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_input_slot *slot = &becore->inputs[i];

		if (slot->state != BECORE_INPUT_READY)
			continue;
		slot->state = BECORE_INPUT_FREE;
		slot->buffer.staged_bytes = 0;
		slot->ready_sequence = 0;
	}
	mutex_unlock(&becore->lock);
}

static void becore_params_drain_idle(struct becore_device *becore)
{
	if (!READ_ONCE(becore->video_streaming))
		schedule_work(&becore->params_work);
}

static void becore_video_stop_producer(struct becore_device *becore)
{
	struct exynos_becore_input *input = NULL;

	mutex_lock(&becore->lock);
	if (becore->producer_streaming) {
		becore->producer_streaming = false;
		input = becore_input_callback_get(becore);
	}
	mutex_unlock(&becore->lock);

	if (!input)
		return;
	input->ops->stop_streaming(input->producer_data);
	becore_video_discard_ready(becore);
	becore_input_callback_put(input);
}

static void becore_video_fail(struct becore_device *becore,
			      struct becore_video_buffer *buf)
{
	mutex_lock(&becore->lock);
	becore->video_streaming = false;
	becore_stream_power_put(becore);
	mutex_unlock(&becore->lock);

	becore_video_stop_producer(becore);
	becore_video_controls_ungrab(becore);
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	vb2_queue_error(&becore->queue);
	becore_video_return_all(becore, VB2_BUF_STATE_ERROR);
}

static void becore_video_work(struct work_struct *work)
{
	struct becore_device *becore =
		container_of(work, struct becore_device, video_work);

	for (;;) {
		struct becore_video_buffer *buf;
		int ret;

		mutex_lock(&becore->lock);
		if (!becore->video_streaming) {
			mutex_unlock(&becore->lock);
			return;
		}
		mutex_unlock(&becore->lock);

		spin_lock_irq(&becore->queue_lock);
		if (list_empty(&becore->queued_outputs)) {
			spin_unlock_irq(&becore->queue_lock);
			return;
		}
		buf = list_first_entry(&becore->queued_outputs,
				       struct becore_video_buffer, list);
		list_del(&buf->list);
		spin_unlock_irq(&becore->queue_lock);

		ret = becore_run_frame(becore, BECORE_RGBP_INPUT_SBWC,
				       BECORE_YUVP_OUTPUT_SBWCL,
				       true, &buf->vb.vb2_buf, false, true);
		if (ret == -ENODATA || ret == -EBUSY) {
			spin_lock_irq(&becore->queue_lock);
			list_add(&buf->list, &becore->queued_outputs);
			spin_unlock_irq(&becore->queue_lock);
			return;
		}
		if (ret) {
			bool stopping;

			mutex_lock(&becore->lock);
			stopping = !becore->video_streaming;
			mutex_unlock(&becore->lock);
			if (ret == -ECANCELED && stopping) {
				vb2_buffer_done(&buf->vb.vb2_buf,
						VB2_BUF_STATE_ERROR);
				return;
			}
			becore_video_fail(becore, buf);
			return;
		}

		buf->vb.vb2_buf.timestamp = ktime_get_ns();
		mutex_lock(&becore->lock);
		buf->vb.sequence = becore->video_sequence++;
		mutex_unlock(&becore->lock);
		buf->vb.field = V4L2_FIELD_NONE;
		vb2_set_plane_payload(&buf->vb.vb2_buf, 0,
				      becore->active_capture_size);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	}
}

static int becore_queue_setup(struct vb2_queue *q, unsigned int *nbufs,
			      unsigned int *nplanes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct becore_device *becore = vb2_get_drv_priv(q);
	struct v4l2_pix_format pix;

	becore_video_fill_pix(&becore->scaled, &pix);
	if (*nplanes) {
		if (*nplanes != 1 || sizes[0] < pix.sizeimage)
			return -EINVAL;
		return 0;
	}

	*nplanes = 1;
	sizes[0] = pix.sizeimage;

	return 0;
}

static int becore_buf_prepare(struct vb2_buffer *vb)
{
	struct becore_device *becore = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_pix_format pix;
	dma_addr_t dma;

	becore_video_fill_pix(&becore->scaled, &pix);
	if (vb2_plane_size(vb, 0) < pix.sizeimage)
		return -EINVAL;

	/*
	 * An imported buffer's address is not the driver's to choose, so every
	 * property MCSC needs of it is checked here rather than at the point of
	 * use: a run that refuses a buffer takes the whole stream down with it,
	 * where QBUF refusing one costs the caller only that buffer.
	 */
	dma = vb2_dma_contig_plane_dma_addr(vb, 0);
	if (!dma || upper_32_bits(dma) ||
	    upper_32_bits(dma + becore_mcsc_output_active_size(&becore->scaled) - 1)) {
		dev_err_ratelimited(becore->dev,
				    "buffer at %pad is outside 32-bit DMA\n",
				    &dma);
		return -EINVAL;
	}
	/*
	 * The raw node refuses a misaligned import because the front end drops
	 * the low bits rather than faulting, and MCSC's own stride is a
	 * multiple of this, so hold a capture to the same rule.  Every buffer
	 * either node allocates is page-aligned and cannot reach this.
	 */
	if (!IS_ALIGNED(dma, BECORE_CAPTURE_ALIGN)) {
		dev_err_ratelimited(becore->dev,
				    "buffer at %pad is not %u-byte aligned\n",
				    &dma, BECORE_CAPTURE_ALIGN);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, pix.sizeimage);

	return 0;
}

static void becore_buf_queue(struct vb2_buffer *vb)
{
	struct becore_device *becore = vb2_get_drv_priv(vb->vb2_queue);
	struct becore_video_buffer *buf =
		to_becore_video_buffer(to_vb2_v4l2_buffer(vb));

	spin_lock_irq(&becore->queue_lock);
	list_add_tail(&buf->list, &becore->queued_outputs);
	spin_unlock_irq(&becore->queue_lock);

	if (READ_ONCE(becore->video_streaming))
		schedule_work(&becore->video_work);
}

static int becore_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct becore_device *becore = vb2_get_drv_priv(q);
	struct exynos_becore_input_stream_config stream_config;
	struct exynos_becore_input *input;
	struct becore_raster was_array;
	u32 was_code;
	int ret;

	mutex_lock(&becore->lock);
	was_array = becore->array;
	was_code = becore->input_code;
	if (becore->video_streaming || becore->running) {
		ret = -EBUSY;
		goto unlock;
	}
	/*
	 * A debug override must not be able to leak into an ordinary capture,
	 * and a forgotten one is easy to leave behind, so the ordinary path
	 * refuses to start rather than quietly running a swept program.
	 */
	if (becore->override_count) {
		ret = -EPERM;
		goto unlock;
	}
	/*
	 * Latch the producer's format first, because everything below is
	 * checked against it: the record walk evaluates every word the array
	 * raster derives, and doing that before the latch would prove it of
	 * the previous stream's raster.
	 */
	ret = becore_latch_input_format(becore);
	if (ret)
		goto unlock;
	ret = becore_recipe_records_validate(becore, false);
	if (ret)
		goto unlock;
	ret = becore_mcsc_recipe_validate(becore);
	if (ret)
		goto unlock;
	if (becore->grid.staged_bytes != BECORE_GRID_SIZE) {
		ret = -EINVAL;
		goto unlock;
	}
	if (becore->reset_failed) {
		ret = -EIO;
		goto unlock;
	}
	input = becore_input_callback_get(becore);
	if (!input) {
		ret = -ENODEV;
		goto unlock;
	}

	ret = becore_stream_power_get(becore);
	if (ret) {
		becore_input_callback_put(input);
		goto unlock;
	}

	becore->video_sequence = 0;
	/* The shared driver-owned output now becomes the video bounce buffer. */
	becore->completed_generation = 0;
	becore->completed_output_size = 0;
	becore_video_controls_snapshot(becore, &stream_config);
	becore->video_streaming = true;
	mutex_unlock(&becore->lock);

	ret = input->ops->start_streaming(input->producer_data, &stream_config);
	if (ret) {
		becore_input_callback_put(input);
		mutex_lock(&becore->lock);
		becore->video_streaming = false;
		/*
		 * The producer refused, so nothing it said about the frame it
		 * sends was ever acted on -- and today refusing is what it does
		 * for every raster but one, because the program that writes a
		 * slot is a captured PDMA recipe selected by exactly matching
		 * its source pad. Leaving what was latched behind would let the
		 * offline loop, which has no producer to disagree with, encode
		 * a geometry no frame was ever taken at, in a phase no sensor
		 * read out.
		 */
		becore->array = was_array;
		becore->input_code = was_code;
		becore_stream_power_put(becore);
		mutex_unlock(&becore->lock);
		cancel_work_sync(&becore->video_work);
		becore_video_controls_ungrab(becore);
		becore_video_return_all(becore, VB2_BUF_STATE_QUEUED);
		becore_params_drain_idle(becore);
		return ret;
	}

	mutex_lock(&becore->lock);
	if (becore->input_producer == input && !input->disconnected &&
	    becore->video_streaming) {
		becore->producer_streaming = true;
	} else {
		/* The producer went away under the dropped lock; same rule. */
		becore->array = was_array;
		becore->input_code = was_code;
		ret = -ENODEV;
	}
	mutex_unlock(&becore->lock);
	if (ret)
		input->ops->stop_streaming(input->producer_data);
	becore_input_callback_put(input);
	if (ret) {
		mutex_lock(&becore->lock);
		becore_stream_power_put(becore);
		mutex_unlock(&becore->lock);
		cancel_work_sync(&becore->video_work);
		becore_video_controls_ungrab(becore);
		becore_video_return_all(becore, VB2_BUF_STATE_QUEUED);
		/*
		 * vb2 does not call stop_streaming after a failed start, so
		 * anything queued while video_streaming was briefly true would
		 * otherwise wait for a frame that is not coming.
		 */
		becore_params_drain_idle(becore);
		return ret;
	}
	schedule_work(&becore->video_work);

	return 0;

unlock:
	/*
	 * Nothing was proven at what was latched, so none of it persists. The
	 * mosaic matters here as much as the raster: its one reader is the
	 * demosaic's phase, the offline loop never re-latches, and a phase left
	 * behind by a stream that never started is a picture with its colours
	 * swapped and nothing saying so.
	 */
	becore->array = was_array;
	becore->input_code = was_code;
	mutex_unlock(&becore->lock);
	becore_video_return_all(becore, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void becore_stop_streaming(struct vb2_queue *q)
{
	struct becore_device *becore = vb2_get_drv_priv(q);
	unsigned long flags;
	bool cancel = false;

	mutex_lock(&becore->lock);
	becore->video_streaming = false;
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
	becore_video_stop_producer(becore);
	becore_video_controls_ungrab(becore);
	becore_video_return_all(becore, VB2_BUF_STATE_ERROR);
	mutex_lock(&becore->lock);
	becore_stream_power_put(becore);
	mutex_unlock(&becore->lock);
	/* Nothing is going to run a frame for a queued parameters buffer now. */
	becore_params_drain_idle(becore);
}

static const struct vb2_ops becore_vb2_ops = {
	.queue_setup = becore_queue_setup,
	.buf_prepare = becore_buf_prepare,
	.buf_queue = becore_buf_queue,
	.start_streaming = becore_start_streaming,
	.stop_streaming = becore_stop_streaming,
};

static int becore_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strscpy(cap->driver, "exynos-becore", sizeof(cap->driver));
	strscpy(cap->card, "zumapro BE-core MCSC NV21", sizeof(cap->card));

	return 0;
}

static int becore_enum_fmt(struct file *file, void *priv,
			   struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_NV21;

	return 0;
}

static int becore_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct becore_device *becore = video_drvdata(file);

	becore_video_fill_pix(&becore->scaled, &f->fmt.pix);

	return 0;
}

static int becore_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct becore_device *becore = video_drvdata(file);

	if (vb2_is_busy(&becore->queue))
		return -EBUSY;

	becore_video_fill_pix(&becore->scaled, &f->fmt.pix);

	return 0;
}

static int becore_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	struct becore_device *becore = video_drvdata(file);

	if (fsize->index || fsize->pixel_format != V4L2_PIX_FMT_NV21)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = becore->scaled.width;
	fsize->discrete.height = becore->scaled.height;

	return 0;
}

static const struct v4l2_ioctl_ops becore_ioctl_ops = {
	.vidioc_querycap = becore_querycap,
	.vidioc_enum_fmt_vid_cap = becore_enum_fmt,
	.vidioc_g_fmt_vid_cap = becore_g_fmt,
	.vidioc_s_fmt_vid_cap = becore_s_fmt,
	.vidioc_try_fmt_vid_cap = becore_g_fmt,
	.vidioc_enum_framesizes = becore_enum_framesizes,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

const struct v4l2_file_operations becore_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

static const struct video_device becore_video_template = {
	.name = "exynos-becore MCSC NV21 capture",
	.fops = &becore_fops,
	.ioctl_ops = &becore_ioctl_ops,
	.release = video_device_release_empty,
	.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING,
	.vfl_dir = VFL_DIR_RX,
};

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
	seq_printf(s, "input            %zu/%zu bytes, iova %pad\n",
		   becore->inputs[0].buffer.staged_bytes,
		   becore->inputs[0].buffer.size, &becore->inputs[0].buffer.dma);
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
			seq_printf(s, "@%llu", slot->ready_sequence);
	}
	seq_putc(s, '\n');
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
		   "params           ccm %u, ltm curve %u, colour LUT %u, gamma %u, sharpener %u, noise reducer %u\n",
		   becore->params.ccm_valid, becore->params.ltm_curve_valid,
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

/*
 * The chain and the scaled output, read and set as one.
 *
 * Everything from RGBP's crop down is derived from these two and from the
 * array, and they are only meaningful together: set one at a time, the driver
 * would spend the gap describing a geometry nobody asked for.  The array is
 * not settable here -- it is the producer's, taken from the sink pad at
 * STREAMON -- and it is printed so a reader sees all three at once.
 *
 * This is what makes a second geometry something to try rather than something
 * to rebuild for.  Every derivation between the three rasters has been
 * runtime arithmetic for a while now, but the surfaces those numbers size
 * were taken once at probe, so the only way to reach another readout was to
 * change the constants and boot.
 *
 * Two states refuse a set outright rather than let it fail somewhere useful.
 * reset_failed and output_quarantined both mean a processor was not proven
 * stopped, so a DMA may still be in flight into the surfaces this would free
 * -- which is precisely the case the stop path bounds by leaving the IOVA
 * mapped until the pages are released, and freeing them here would spend that
 * bound.  A queue with buffers refuses too: MCSC writes a capture buffer
 * directly, so a scaled raster that grew past what REQBUFS allocated would
 * overrun it.
 *
 * On a failure the previous geometry goes back and is allocated again.  If
 * even that fails the surfaces stay freed, and a run refuses rather than
 * programming a DMA at zero -- becore_run_frame() checks both destination
 * addresses before it writes any of them.
 */
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

static int becore_geometry_apply(struct becore_device *becore,
				 const struct becore_raster *chain,
				 const struct becore_raster *scaled)
{
	struct becore_raster old_chain = becore->chain;
	struct becore_raster old_scaled = becore->scaled;
	struct becore_rect crop;
	u32 ratio;
	int ret;

	lockdep_assert_held(&becore->lock);

	ret = becore_chain_validate(becore->dev, chain);
	if (ret)
		return ret;
	ret = becore_raster_validate(becore->dev, "scaled", scaled,
				     BECORE_RASTER_EXTENT_MAX);
	if (ret)
		return ret;
	/*
	 * The three derivations that a *pair* of rasters can fail on their own,
	 * which bounding each extent separately cannot reach: RGBP's crop of
	 * the array onto the chain; YUVNR's binning, which is that crop over
	 * the chain at Q10 in fourteen bits, so the crop may be at most 15.999
	 * times the chain; and DJAG's ratio of the chain onto the scaled
	 * output, which is Q20 and refuses past a 4096x downscale. Everything
	 * else is either bounded by one extent or a plain repacking of one.
	 *
	 * YUVNR's is the reason a crop check alone is not enough: with the
	 * default array, any chain narrower than 260 passes the crop and the
	 * ratio and then refuses every run afterwards with a bare -EINVAL.
	 */
	ret = becore_rgbp_crop(&becore->array, chain, &crop);
	if (ret) {
		dev_err(becore->dev, "no crop of %ux%u reaches a %ux%u chain\n",
			becore->array.width, becore->array.height,
			chain->width, chain->height);
		return ret;
	}
	ret = becore_yuvnr_geometry_value(&becore->array, chain,
					  BECORE_YUVNR_BINNING, &ratio);
	if (ret) {
		dev_err(becore->dev,
			"a %ux%u crop of %ux%u is too much for a %ux%u chain to bin\n",
			crop.width, crop.height,
			becore->array.width, becore->array.height,
			chain->width, chain->height);
		return ret;
	}
	ret = becore_zoom_ratio(chain->width, scaled->width, &ratio);
	if (!ret)
		ret = becore_zoom_ratio(chain->height, scaled->height, &ratio);
	if (ret) {
		dev_err(becore->dev, "no scaler ratio takes %ux%u to %ux%u\n",
			chain->width, chain->height,
			scaled->width, scaled->height);
		return ret;
	}

	becore->chain = *chain;
	becore->scaled = *scaled;
	becore_free_surfaces(becore);
	ret = becore_alloc_surfaces(becore);
	if (!ret)
		return 0;

	becore->chain = old_chain;
	becore->scaled = old_scaled;
	becore_free_surfaces(becore);
	if (becore_alloc_surfaces(becore)) {
		/*
		 * A second failure can stop partway too, so free once more:
		 * "the back end has no surfaces" is a state a run refuses
		 * cleanly, and a half-allocated set is not.
		 */
		becore_free_surfaces(becore);
		dev_crit(becore->dev,
			 "cannot take the previous surfaces back; the back end has none\n");
	}

	return ret;
}

static ssize_t becore_geometry_write(struct file *file, const char __user *buf,
				     size_t count, loff_t *ppos)
{
	struct becore_device *becore =
		((struct seq_file *)file->private_data)->private;
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
	/* The %c matches only if something follows the four, and refuses it. */
	if (sscanf(line, "%u %u %u %u %c", &chain.width, &chain.height,
		   &scaled.width, &scaled.height, &tail) != 4)
		return -EINVAL;

	mutex_lock(&becore->video_lock);
	mutex_lock(&becore->lock);
	if (becore->reset_failed || becore->output_quarantined)
		ret = -EIO;
	else if (becore->running || becore->video_streaming ||
		 vb2_is_busy(&becore->queue))
		ret = -EBUSY;
	else
		ret = becore_geometry_apply(becore, &chain, &scaled);
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

static void becore_video_cleanup(void *data)
{
	struct becore_device *becore = data;

	/*
	 * Both drivers are built in and suppress unbind, so this runs only on
	 * a probe-failure unwind, before any producer has put these entities on
	 * a graph and so before either node has streamed.  The two cancels are
	 * for the offline loop, which is the one thing that can queue work with
	 * no node registered at all.
	 */
	cancel_work_sync(&becore->params_work);
	cancel_work_sync(&becore->video_work);
	media_entity_cleanup(&becore->params_vdev.entity);
	media_entity_cleanup(&becore->vdev.entity);
	v4l2_subdev_cleanup(&becore->sd);
	media_entity_cleanup(&becore->sd.entity);
	v4l2_ctrl_handler_free(&becore->ctrl_handler);
}

/*
 * Everything a video node needs except the v4l2_device it hangs from, which
 * this device does not own -- see exynos_becore_input_register_graph().
 */
static int becore_video_init(struct becore_device *becore)
{
	struct v4l2_ctrl_handler *handler = &becore->ctrl_handler;
	struct vb2_queue *q = &becore->queue;
	int ret;

	ret = v4l2_ctrl_handler_init(handler, 2);
	if (ret)
		return ret;
	becore->red_balance =
		v4l2_ctrl_new_std(handler, &becore_ctrl_ops, V4L2_CID_RED_BALANCE,
				  EXYNOS_BECORE_WBG_GAIN_MIN_Q12,
				  EXYNOS_BECORE_WBG_GAIN_MAX_Q12, 1,
				  EXYNOS_BECORE_WBG_RED_DEFAULT_Q12);
	becore->blue_balance =
		v4l2_ctrl_new_std(handler, &becore_ctrl_ops, V4L2_CID_BLUE_BALANCE,
				  EXYNOS_BECORE_WBG_GAIN_MIN_Q12,
				  EXYNOS_BECORE_WBG_GAIN_MAX_Q12, 1,
				  EXYNOS_BECORE_WBG_BLUE_DEFAULT_Q12);
	/*
	 * The control op has not run yet, so seed these with the same defaults
	 * the controls were created at.  The offline loop encodes without ever
	 * setting one.
	 */
	becore->encode_balance_red = EXYNOS_BECORE_WBG_RED_DEFAULT_Q12;
	becore->encode_balance_blue = EXYNOS_BECORE_WBG_BLUE_DEFAULT_Q12;
	if (handler->error) {
		ret = handler->error;
		goto err_ctrl;
	}

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->dev = becore->dev;
	q->drv_priv = becore;
	q->ops = &becore_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct becore_video_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers = 1;
	q->lock = &becore->video_lock;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_ctrl;

	/* The producer's v4l2_device is filled in when it registers. */
	becore->vdev = becore_video_template;
	becore->vdev.ctrl_handler = &becore->ctrl_handler;
	becore->vdev.queue = q;
	becore->vdev.lock = &becore->video_lock;
	becore->vdev.entity.function = MEDIA_ENT_F_IO_V4L;
	video_set_drvdata(&becore->vdev, becore);

	/*
	 * The input subdevice is initialised here and registered later, by the
	 * producer, on the producer's graph: this device has nothing to link it
	 * to on its own.
	 */
	v4l2_subdev_init(&becore->sd, &becore_subdev_ops);
	becore->sd.internal_ops = &becore_subdev_internal_ops;
	becore->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	becore->sd.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	becore->sd.entity.ops = &becore_subdev_entity_ops;
	becore->sd.owner = THIS_MODULE;
	becore->sd.dev = becore->dev;
	strscpy(becore->sd.name, "exynos-becore input", sizeof(becore->sd.name));
	v4l2_set_subdevdata(&becore->sd, becore);
	becore->input_code = BECORE_INPUT_DEFAULT_CODE;

	becore->sink_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&becore->sd.entity, 1, &becore->sink_pad);
	if (ret)
		goto err_ctrl;

	ret = v4l2_subdev_init_finalize(&becore->sd);
	if (ret)
		goto err_sd_entity;

	becore->vdev_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&becore->vdev.entity, 1,
				     &becore->vdev_pad);
	if (ret)
		goto err_sd;

	ret = becore_params_init(becore);
	if (ret)
		goto err_entity;

	ret = devm_add_action_or_reset(becore->dev,
				       becore_video_cleanup, becore);
	if (ret)
		return ret;
	becore->video_ready = true;

	return 0;

err_entity:
	media_entity_cleanup(&becore->vdev.entity);
err_sd:
	v4l2_subdev_cleanup(&becore->sd);
err_sd_entity:
	media_entity_cleanup(&becore->sd.entity);
err_ctrl:
	v4l2_ctrl_handler_free(&becore->ctrl_handler);
	return ret;
}

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
