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
#include <media/media-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-isp.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#include "exynos-becore-recipe.h"
#include "exynos-becore-gtnr-recipe.h"
#include "exynos-becore-mcsc-recipe.h"

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

/*
 * The VOTF fabric.  Each processor that owns a fabric endpoint has its own
 * C2SERV window, and a window is one node on a token ring: it names itself by
 * writing its local IP ID to LOCAL_IP, and a producer names its consumer by
 * that ID shifted up four bits with the consumer's DMA number in the low
 * nibble.  Producer endpoints (TWS) and consumer endpoints (TRS) live in
 * separate register regions.
 *
 * A region packs as many endpoint blocks into each 256-byte page as fit whole
 * -- nine producers of 0x1c bytes, five consumers of 0x2c -- and starts a new
 * page rather than letting one straddle, so the offset of endpoint n is not
 * linear in n.  The two formulas below reproduce all thirty-two flush offsets
 * the vendor writes, wraps included.
 */
#define BECORE_C2SERV_DEBUG		0x0000
#define BECORE_C2SERV_DEBUG_DOUT	0x0004
#define BECORE_C2SERV_RCV_VALID		0x0008
#define BECORE_C2SERV_RING_CLK_EN	0x000c
#define BECORE_C2SERV_RING_ENABLE	0x0010
#define BECORE_C2SERV_LOCAL_IP		0x0014
#define BECORE_C2SERV_SW_RESET		0x0018
#define BECORE_C2SERV_SEL_REGISTER	0x0024
#define BECORE_C2SERV_SEL_REGISTER_MODE	0x0028

#define BECORE_C2SERV_TWS_PER_PAGE	9
#define BECORE_C2SERV_TWS(n)		(0x0100 + \
					 ((n) / BECORE_C2SERV_TWS_PER_PAGE) * 0x100 + \
					 ((n) % BECORE_C2SERV_TWS_PER_PAGE) * 0x1c)
#define BECORE_C2SERV_TWS_ENABLE	0x00
#define BECORE_C2SERV_TWS_LIMIT		0x04
#define BECORE_C2SERV_TWS_DEST		0x08
#define BECORE_C2SERV_TWS_LINES_IN_TOKEN	0x0c
#define BECORE_C2SERV_TWS_FLUSH		0x10
#define BECORE_C2SERV_TWS_BUSY		0x14
#define BECORE_C2SERV_TWS_FULLNESS	0x18

#define BECORE_C2SERV_TRS_PER_PAGE	5
#define BECORE_C2SERV_TRS(n)		(0x0300 + \
					 ((n) / BECORE_C2SERV_TRS_PER_PAGE) * 0x100 + \
					 ((n) % BECORE_C2SERV_TRS_PER_PAGE) * 0x2c)
#define BECORE_C2SERV_TRS_ENABLE	0x00
#define BECORE_C2SERV_TRS_RECOVER	0x04
#define BECORE_C2SERV_TRS_LIMIT		0x08
#define BECORE_C2SERV_TRS_CROP_START	0x0c
#define BECORE_C2SERV_TRS_CROP_ENABLE	0x10
#define BECORE_C2SERV_TRS_LINES_IN_FIRST_TOKEN	0x14
#define BECORE_C2SERV_TRS_LINES_IN_TOKEN	0x18
#define BECORE_C2SERV_TRS_LINES_COUNT	0x1c
#define BECORE_C2SERV_TRS_FLUSH		0x20
#define BECORE_C2SERV_TRS_BUSY		0x24
#define BECORE_C2SERV_TRS_LOST_CONNECTION	0x28

/*
 * The wrapper between the DMAs and the fabric.  Every captured endpoint setup
 * writes 8 here, on the same window as the endpoint it is programming and in
 * a fixed position in the sequence; Samsung's register table puts the
 * wrapper's software reset next door at +0xd300.
 */
#define BECORE_C2SERV_WRAPPER		0xd304
#define BECORE_C2SERV_WRAPPER_CONNECT	0x8


/*
 * A window will say what state one endpoint's connection is in: select the
 * endpoint through DEBUG and read the answer out of DEBUG_DOUT.  This is the
 * one measurement that separates "the handshake never completed" from
 * "connected, and then starved" -- the difference between a link that is
 * mis-programmed and one that is merely mis-tuned.
 */
#define BECORE_C2SERV_DEBUG_ENABLE	BIT(0)
#define BECORE_C2SERV_DEBUG_ENDPOINT	GENMASK_U32(4, 1)
#define BECORE_C2SERV_DEBUG_CONSUMER	BIT(5)
#define BECORE_C2SERV_DEBUG_STATE	GENMASK_U32(3, 0)

/* Both regions are sixteen deep on every instance the vendor flushes. */
#define BECORE_C2SERV_ENDPOINTS		16

/*
 * The link this driver is building: YUVP's combined two-plane output into
 * MCSC's input, one endpoint per plane, which is the shape both captured
 * vendor links have.  Endpoint n on one side pairs with endpoint n on the
 * other, so a plane needs no mapping table of its own.
 */
#define BECORE_C2SERV_LINK_PLANES	2

/*
 * Token geometry, taken whole from the one captured link that has the same
 * shape as this one rather than assembled from two.  Lyric computes a token
 * from a per-IP table -- extra lines plus process lines, halved for chroma,
 * quartered for an SBWC producer -- and for YUVP into MCSC every term is 1,
 * because both blocks declare one process line.  That gives a one-line ring,
 * which is not a geometry any measured link uses.
 *
 * What works is `YUVP -> TNR`'s pair, 12 and 4 against 48 and 16.  It is the
 * vendor's only captured link with a two-plane SBWC producer -- the shape
 * this one has -- and the reason it works is arithmetic: 3120 lines in tokens
 * of 12 is 260 tokens, and in tokens of 48 is 65, an exact factor of four, so
 * both ends agree about how many tokens a frame is.  MCSC's own captured
 * pairing, 64 and 32 against GDC0's producer, does not divide that way once
 * YUVP is the producer: 260 producer tokens against 48.75 consumer ones.
 *
 * That mismatch is not a subtlety, it is the whole difference between working
 * and not.  With 64/32 the link half-forms -- one plane connects and the
 * other sits in WAIT_TOKEN_ACK -- and at a limit large enough to paper over
 * it the consumer runs about a third ahead of the data and the picture goes
 * to fill part way down.  With 48/16 the frame is byte-identical to the
 * memory path at the vendor's own limit of 1.
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

/*
 * A producer names its destination as the consumer's local IP ID followed by
 * the consumer DMA number, which is how 0x1cc5 and DMA 1 becomes 0x1cc51.
 */
#define BECORE_C2SERV_DEST(ip, dma)	(((ip) << 4) | (dma))


#define BECORE_RESET_TIMEOUT_US		1000
/* The longer of the two per-stage CRC lists; checked against both at probe. */
#define BECORE_STREAM_CRC_MAX		26
/* Debug register overrides: enough to sweep a small LUT, not a whole block. */
#define BECORE_OVERRIDE_MAX		32
#define BECORE_OVERRIDE_TEXT_MAX	1024

#define BECORE_INT_FRAME_END		BIT(1)
#define BECORE_INT_CMDQ_HOLD		BIT(2)
#define BECORE_INT_EXPECTED		(BECORE_INT_FRAME_END | BECORE_INT_CMDQ_HOLD)
#define BECORE_YUVP_STAGE_BLOCKS	(BIT(BECORE_RGBP) | BIT(BECORE_YUVP))

#define BECORE_CMDQ_HEADER_BYTES		16
#define BECORE_CMDQ_PAYLOAD_BYTES	64
#define BECORE_CMDQ_PAYLOAD_WORDS	(BECORE_CMDQ_PAYLOAD_BYTES / 4)
#define BECORE_CMDQ_MODE		0x9000

/* Fixed neutral LTM policy for the proven 4000x3000 processing profile. */
#define BECORE_LTM_GRID_ROW_BYTES	0x800
#define BECORE_LTM_GRID_ROWS		48
#define BECORE_LTM_GRID_CELL_BYTES	0x100
#define BECORE_LTM_GRID_WIDTH_CELLS	4
#define BECORE_LTM_GRID_HEIGHT_CELLS	24
#define BECORE_LTM_UNITY_Q14		BIT(14)
#define BECORE_GRID_SIZE			(BECORE_LTM_GRID_ROW_BYTES * \
					 BECORE_LTM_GRID_ROWS)
#define BECORE_RUN_TIMEOUT_MS		1000

/* What a queued capture's address must be aligned to; see becore_buf_prepare. */
#define BECORE_CAPTURE_ALIGN		32

/*
 * The INTCAM operating point the vendor holds while a 4000 x 3000 rear
 * ultrawide stream runs [HW 2026-08-20].  It has to be an exact rung of the
 * measured ladder -- 664000 533000 465000 310000 233000 111000 kHz -- because
 * the ACPM clock passes the requested rate straight to firmware rather than
 * rounding it up to a supported OPP the way downstream's PM QoS does.
 */
#define BECORE_INTCAM_ACTIVE_RATE	233000000UL

/* Each block's device-tree reg window; a debug read has to stay inside it. */
#define BECORE_BLOCK_WINDOW		0x10000

#define BECORE_RGBP_PHYS_BASE		0x1c440000
#define BECORE_YUVP_PHYS_BASE		0x1c840000
#define BECORE_GTNR_PHYS_BASE		0x1cc40000
#define BECORE_MCSC_PHYS_BASE		0x1d040000

#define BECORE_RGBP_INPUT_IMAGE_REG	(BECORE_RGBP_PHYS_BASE + 0x1c50)
#define BECORE_RGBP_INPUT_HEADER_REG	(BECORE_RGBP_PHYS_BASE + 0x1d10)
#define BECORE_RGBP_INPUT_ENABLE_REG	(BECORE_RGBP_PHYS_BASE + 0x1c00)
#define BECORE_RGBP_INPUT_COMP_REG	(BECORE_RGBP_PHYS_BASE + 0x1c04)
#define BECORE_RGBP_INPUT_FORMAT_REG	(BECORE_RGBP_PHYS_BASE + 0x1c10)
/*
 * RGBP's chain geometry and the crop that narrows it, named from Samsung
 * RGBP v1.20. All seven derive from three rasters: the sensor's Bayer array,
 * the window DMSCCROP takes out of it, and what the chain hands YUVP. At the
 * shipped profile that is 4208 x 3120, a centred 4160 x 3120 at (24, 0), and
 * 4160 x 3120 -- so the crop removes 48 columns and the scaler below it runs
 * at unity. None of those numbers is a constant here: the scaler's ratios are
 * the crop over the destination, and they select its filter coefficients as
 * well as its scaling.
 */
#define BECORE_RGBP_CHAIN_SRC_SIZE_REG	(BECORE_RGBP_PHYS_BASE + 0x0200)
#define BECORE_RGBP_CHAIN_DST_SIZE_REG	(BECORE_RGBP_PHYS_BASE + 0x0204)
#define BECORE_RGBP_CROP_START_REG	(BECORE_RGBP_PHYS_BASE + 0x0234)
#define BECORE_RGBP_CROP_SIZE_REG	(BECORE_RGBP_PHYS_BASE + 0x0238)
#define BECORE_RGBP_SC_DST_SIZE_REG	(BECORE_RGBP_PHYS_BASE + 0x441c)
#define BECORE_RGBP_SC_H_RATIO_REG	(BECORE_RGBP_PHYS_BASE + 0x4420)
#define BECORE_RGBP_SC_V_RATIO_REG	(BECORE_RGBP_PHYS_BASE + 0x4424)
/*
 * Ten registers that say which RGBP blocks run, named from Samsung RGBP v1.20.
 * None needs a value from the capture: an enable the program leaves clear is
 * zero and an asserted bypass is one.
 *
 * OTF_CROP_CTRL is the one that reads backwards. Its only field is
 * RGB_DMSCCROP_BYPASS, whose vendor default is 1, and the capture clears it --
 * so zero here is what keeps the 48-column demosaic crop above *running*.
 * Do not fold it in with the asserted bypasses.
 *
 * UPSC_CTRL0 is a control word rather than a bare enable: bit 0 enables, bit 8
 * bypasses and two more disable clock gates. Zero leaves the upscaler neither
 * enabled nor bypassed, which is what the capture does.
 *
 * DECOMP's frame size is not a tuning either. It is the Bayer input's, packed
 * with the height in the high half: RGBP v1.20 gives DECOMP width bits [13:0]
 * and height bits [29:16], the other way round from CHAIN_SRC_IMG_SIZE.
 */
#define BECORE_RGBP_CINFIFO_FRAME_IN_REG (BECORE_RGBP_PHYS_BASE + 0x0084)
#define BECORE_RGBP_SATFLAG_ENABLE_REG	(BECORE_RGBP_PHYS_BASE + 0x0218)
#define BECORE_RGBP_DMSCCROP_BYPASS_REG	(BECORE_RGBP_PHYS_BASE + 0x0230)
#define BECORE_RGBP_WDMADECOMP_EN_REG	(BECORE_RGBP_PHYS_BASE + 0x2000)
#define BECORE_RGBP_WDMAY_EN_REG	(BECORE_RGBP_PHYS_BASE + 0x2400)
#define BECORE_RGBP_WDMAUV_EN_REG	(BECORE_RGBP_PHYS_BASE + 0x2600)
#define BECORE_RGBP_DECOMP_BYPASS_REG	(BECORE_RGBP_PHYS_BASE + 0x3e00)
#define BECORE_RGBP_DECOMP_SIZE_REG	(BECORE_RGBP_PHYS_BASE + 0x3e08)
#define BECORE_RGBP_GAMMALR_BYPASS_REG	(BECORE_RGBP_PHYS_BASE + 0x4600)
#define BECORE_RGBP_UPSC_CTRL0_REG	(BECORE_RGBP_PHYS_BASE + 0x4800)
#define BECORE_RGBP_GAMMAHR_BYPASS_REG	(BECORE_RGBP_PHYS_BASE + 0x4a00)
/*
 * RGBP's global tone map, named from Samsung RGBP v1.20. The block runs, but
 * the curve it runs is an exact identity: at every one of its 64 knots the
 * captured output is the input times 32. That factor is the block's own field
 * widths -- v1.20 gives its input knots 13 bits and its output values 18, and
 * 18 - 13 is 5 -- so times 32 is this block's unity, and there is no tuning
 * here to recover. What is left is the knot grid, a description of where the
 * curve is sampled, and the luma weights, which are BT.601's.
 *
 * IN_POINTS packs two knots per register with the even one in the low half,
 * which the vendor's own field descriptors state; OUT_POINTS needs a register
 * each because 18 bits do not fit beside anything.
 */
/*
 * Three RGBP blocks and one YUVP register whose values are named constants
 * rather than anyone's tuning.
 *
 * RGB_RGBTOYUV is BT.601 at Q13, full range rather than studio, and it is
 * column-major by input channel -- (Y, U, V) of R, then of G, then of B. Read
 * row-major, six of the nine coefficients look wrong. The coefficients are
 * kept as exact rationals of Kr = 299/1000 and Kb = 114/1000 so that no float
 * reaches a register, and Q13 is deliberately not twice the front end's Q12
 * copy: the two are independent roundings of the same reals.
 *
 * YUV444TO422 is [1, 2, 1] / 4 scaled by 32, the classic chroma decimation
 * filter, written unconditionally with no tuning import anywhere.
 *
 * BYR_DNS's binning is Q10 unity -- this path does not bin -- and its radial
 * centre is the sensor's full array halved and negated. Samsung's own
 * rgbp_hw_s_dns_size() writes -(full_width >> 1 & ~1) plus the crop as an
 * offset; the two agree exactly while the crop is centred, which mainline's
 * is, and the input profile's raster is the full array. A crop that was *not*
 * centred would need the offset term as well, so this is a statement about the
 * current geometry rather than a general derivation. The noise curve above it
 * is real tuning and stays.
 *
 * SHARPENHANCER's LPF_NORM is log2 of its three low-pass kernels' sums packed
 * at bits 0, 8 and 16 -- 16, 512 and 4096, which is what the captured taps
 * really sum to. Note what is and is not checked: the taps themselves stay in
 * the recipe and are not read here, so the power-of-two test below is a
 * property of these three constants and not of the kernels. Deriving the sums
 * from the tap registers would make it a real check, and would want them out
 * of the recipe first.
 */
/*
 * Two blocks whose captured words are literal constants rather than a scene.
 *
 * MCSC's DJAG runs at a neutral profile, and the thirteen words stated here
 * have three separate provenances -- worth keeping apart, because they are not
 * equally strong:
 *
 *  - Seven are Samsung's init_djag_cfgs in is-hw-djag-v2.c *and* the POR reset
 *    value is-sfr-mcsc-v10_1.h gives for the same field: the three LFSR seeds,
 *    the two dither-ramp words, the saturation/dither thresholds and the
 *    coring threshold. Two independent sources agreeing is the strongest case
 *    in this file.
 *  - Three are POR only, because init_djag_cfgs has no such field: both
 *    pre-scaler phase offsets and the round mode. Samsung's own code writes
 *    them as hardcoded literals rather than from a setfile.
 *  - Three are neither, and are stated from what the block is being asked to
 *    do. CTRL is 0x403 because is_scaler_set_djag_enable() sets exactly bits
 *    0, 1 and 10 from one enable and every other bit is POR-clear: DJAG, its
 *    pre-scaler and EZ post are on. RECOM_CTRL and RECOM_WEIGHT are zero
 *    because the detail-restoration sub-block is off -- and it is off by two
 *    independent signals, its control clear *and* its weight zero where POR is
 *    0x400. Two signals rather than one is what makes that a fact rather than
 *    a hypothesis; a lone enable bit is exactly what misled us about YUVP's
 *    tone mapping.
 *
 * What stays in the recipe is not all "differs from the profile". The
 * shooting-detection thresholds, the cross-filter weights, CP_ARBI's mode, the
 * dither white/black guard band and RECOM's biquad shift do differ, and they
 * are keyed on a scaling ratio we hold one row of. But six of RECOM's radial
 * registers are already at POR and are left alone only because a disabled
 * sub-block's radial configuration is not worth a claim.
 *
 * RGBP's DMSC is a real demosaic and most of it is tuning. Fourteen registers
 * are literals GetDefaultDmsc writes *after* the tuning path has run, so they
 * do not vary with the scene. Three cautions:
 *
 *  - Unlike DJAG these are the vendor's compiled-in defaults and not a
 *    hardware reset state, since RGBP v1.20 publishes no POR values. They are
 *    a bring-up default under ADR 0009, not a derivation.
 *  - GetDefaultDmsc writes 23 such literals, not fourteen. The rest are left
 *    in the recipe; this is a conservative subset, not the whole set.
 *  - Every one is a read-modify-write. For thirteen the preserved bits are
 *    reserved and the literal determines every defined field. BASE_CONFIG is
 *    the exception: it preserves bit 0 and bit 16, and bit 16 is a real field,
 *    SKIP_BYR2RGB_EN. Its captured 0x36a therefore also asserts that the
 *    tuning path left those two clear.
 *
 * EDGE_DESAT_RED_PRESERVE_GAIN looks like it belongs to that group and does
 * not: TranslateDmsc computes it as clamp(f * 1023, 0, 0x3ff) into a 10-bit
 * field, so it is tuning that happens to be 0x100 here. It stays in the
 * recipe.
 *
 * Watch the name: GetDefaultDmsc(DmscRgbpOutput&) is a different function from
 * the front end's, which takes a DmscOutput& and programs ISPFE.
 */
/*
 * RGBP's BYR_DNS and YUVP's YUVNR carry the same object: an eight-knot
 * piecewise-linear curve of noise standard deviation against pixel level, one
 * curve for luma and one for chroma. The knots are genuine tuning and stay in
 * the recipe. What follows from them is the eight slopes, the shift they are
 * taken at, and -- because the chroma curve is measured on the same domain as
 * the luma one -- the chroma domain itself.
 *
 * A slope is (dY << 11) / dX, and the eighth field repeats the seventh because
 * there are eight fields for seven segments. The trap is the rounding: **DNS
 * rounds to nearest and YUVNR truncates**. Both were verified on both channels
 * of both blocks. Using one rule for the other block still reproduces every
 * exact division and misses by one everywhere else, so a spot check on a
 * couple of knots passes and the curve is quietly wrong in between.
 *
 * These are the first values that are not a function of the geometry or of a
 * constant, but of the block's own neighbouring registers, which is why
 * becore_recipe_fixed_value() exists.
 */
#define BECORE_RGBP_DNS_X_G_REG		(BECORE_RGBP_DNS_BASE + 0x110)
#define BECORE_RGBP_DNS_Y_G_REG		(BECORE_RGBP_DNS_BASE + 0x120)
#define BECORE_RGBP_DNS_SLOPE_G_REG	(BECORE_RGBP_DNS_BASE + 0x130)
#define BECORE_RGBP_DNS_SHIFT_G_REG	(BECORE_RGBP_DNS_BASE + 0x140)
#define BECORE_RGBP_DNS_X_RB_REG	(BECORE_RGBP_DNS_BASE + 0x144)
#define BECORE_RGBP_DNS_Y_RB_REG	(BECORE_RGBP_DNS_BASE + 0x154)
#define BECORE_RGBP_DNS_SLOPE_RB_REG	(BECORE_RGBP_DNS_BASE + 0x164)
#define BECORE_RGBP_DNS_SHIFT_RB_REG	(BECORE_RGBP_DNS_BASE + 0x174)
#define BECORE_YUVP_NR_BASE		(BECORE_YUVP_PHYS_BASE + 0x3000)
#define BECORE_YUVP_NR_X_Y_REG		(BECORE_YUVP_NR_BASE + 0x220)
#define BECORE_YUVP_NR_Y_Y_REG		(BECORE_YUVP_NR_BASE + 0x230)
#define BECORE_YUVP_NR_SLOPE_Y_REG	(BECORE_YUVP_NR_BASE + 0x240)
#define BECORE_YUVP_NR_SHIFT_Y_REG	(BECORE_YUVP_NR_BASE + 0x260)
#define BECORE_YUVP_NR_X_UV_REG		(BECORE_YUVP_NR_BASE + 0x274)
#define BECORE_YUVP_NR_Y_UV_REG		(BECORE_YUVP_NR_BASE + 0x284)
#define BECORE_YUVP_NR_SLOPE_UV_REG	(BECORE_YUVP_NR_BASE + 0x2a4)
#define BECORE_YUVP_NR_SHIFT_UV_REG	(BECORE_YUVP_NR_BASE + 0x2b4)
#define BECORE_NOISE_KNOTS		8
#define BECORE_NOISE_TABLE_REGS		(BECORE_NOISE_KNOTS / 2)
#define BECORE_NOISE_TABLE_LAST		((BECORE_NOISE_TABLE_REGS - 1) * 4)
#define BECORE_NOISE_SLOPE_SHIFT	11
#define BECORE_NOISE_SLOPE_MASK		GENMASK(12, 0)
#define BECORE_NOISE_SHIFT_NIBBLES	8
/*
 * A scaler that starts on a pixel. RGBP's SC, MCSC's POLY_SC0 and its POST_PC0
 * chroma converter each put two 20-bit init phase offsets at the same place in
 * their register map, and all six words are the field table's POR zero: no
 * sub-pixel origin. Nothing here is scene-dependent, and a sub-pixel origin
 * would need a reason none of these blocks has at this geometry -- though note
 * that is a single-geometry observation, since POLY_SC0 and POST_PC0 run at
 * unity here and DJAG does the scaling.
 *
 * The two MCSC blocks put a round-mode bit after the offsets, also at POR, and
 * those are stated too. RGBP's scaler does not: its round mode is bit 0 of
 * YUV_SC_CTRL1 two registers *earlier*, its POR is zero, and the capture sets
 * it -- so RGBP rounds against the reset value and that word stays in the
 * recipe. RGBP has no register at +0x4410 at all.
 *
 * The rest of what MCSC still replayed above its scalers is the shape of the
 * job rather than a value: it reads memory, not an OTF stream, so CINFIFO and
 * both IP_USE gates are clear and INPUT_TYPE is memory; it drives one output,
 * so the four other WDMA channels and the HF statistics RDMA are off; and the
 * raster it reads is YUVP's output, which the driver already describes.
 */
#define BECORE_RGBP_SC_PHASE_FIRST	(BECORE_RGBP_PHYS_BASE + 0x4408)
#define BECORE_RGBP_SC_PHASE_LAST	(BECORE_RGBP_PHYS_BASE + 0x440c)
#define BECORE_MCSC_SC0_PHASE_FIRST	(BECORE_MCSC_PHYS_BASE + 0x5018)
#define BECORE_MCSC_PC0_PHASE_FIRST	(BECORE_MCSC_PHYS_BASE + 0x6014)
/* Two offsets everywhere; the MCSC blocks add a round mode after them. */
#define BECORE_SCALER_PHASE_LAST	0x08
#define BECORE_MCSC_OTF_GATE_FIRST	(BECORE_MCSC_PHYS_BASE + 0x0080)
#define BECORE_MCSC_OTF_GATE_LAST	(BECORE_MCSC_PHYS_BASE + 0x0084)
#define BECORE_MCSC_INPUT_TYPE_REG	(BECORE_MCSC_PHYS_BASE + 0x0200)
#define BECORE_MCSC_IN_WIDTH_REG	(BECORE_MCSC_PHYS_BASE + 0x0210)
#define BECORE_MCSC_IN_HEIGHT_REG	(BECORE_MCSC_PHYS_BASE + 0x0214)
#define BECORE_MCSC_CINFIFO_FIRST	(BECORE_MCSC_PHYS_BASE + 0x1000)
#define BECORE_MCSC_CINFIFO_LAST	(BECORE_MCSC_PHYS_BASE + 0x1004)
#define BECORE_MCSC_STAT_RDMA_FIRST	(BECORE_MCSC_PHYS_BASE + 0x1a00)
#define BECORE_MCSC_STAT_RDMA_LAST	(BECORE_MCSC_PHYS_BASE + 0x1a04)
#define BECORE_MCSC_WDMA_W1_FIRST	(BECORE_MCSC_PHYS_BASE + 0x2200)
#define BECORE_MCSC_WDMA_W2_FIRST	(BECORE_MCSC_PHYS_BASE + 0x2400)
#define BECORE_MCSC_WDMA_W3_FIRST	(BECORE_MCSC_PHYS_BASE + 0x2600)
#define BECORE_MCSC_WDMA_W4_FIRST	(BECORE_MCSC_PHYS_BASE + 0x2800)
/* Each unused channel is quiesced by its enable and its compression control. */
#define BECORE_MCSC_WDMA_OFF_LAST	0x04
#define BECORE_MCSC_DJAG_BASE		(BECORE_MCSC_PHYS_BASE + 0x4000)
#define BECORE_MCSC_DJAG_CTRL_REG	(BECORE_MCSC_DJAG_BASE + 0x000)
#define BECORE_MCSC_DJAG_PS_FIRST	(BECORE_MCSC_DJAG_BASE + 0x01c)
#define BECORE_MCSC_DJAG_PS_LAST	(BECORE_MCSC_DJAG_BASE + 0x024)
#define BECORE_MCSC_DJAG_TUNE_FIRST	(BECORE_MCSC_DJAG_BASE + 0x050)
#define BECORE_MCSC_DJAG_TUNE_LAST	(BECORE_MCSC_DJAG_BASE + 0x068)
#define BECORE_MCSC_DJAG_RECOM_CTRL_REG	(BECORE_MCSC_DJAG_BASE + 0x080)
#define BECORE_MCSC_DJAG_RECOM_WEIGHT_REG (BECORE_MCSC_DJAG_BASE + 0x088)
#define BECORE_DJAG_DITHER_FIELD_BITS	6
#define BECORE_DJAG_SAT_CTRL		5
#define BECORE_DJAG_DITHER_THRES	5
#define BECORE_DJAG_DITHER_THRES_SHIFT	14
#define BECORE_DJAG_CP_HF_THRES		40
#define BECORE_RGBP_DMSC_BASE		(BECORE_RGBP_PHYS_BASE + 0x3000)
#define BECORE_RGBP_DNS_BASE		(BECORE_RGBP_PHYS_BASE + 0x3000)
#define BECORE_RGBP_DNS_BINNING_REG	(BECORE_RGBP_DNS_BASE + 0x1a4)
#define BECORE_RGBP_DNS_CENTRE_REG	(BECORE_RGBP_DNS_BASE + 0x1c0)
#define BECORE_RGBP_DNS_BINNING_UNITY	1024	/* Q10 */
#define BECORE_RGBP_DNS_CENTRE_MASK	GENMASK(14, 0)
#define BECORE_RGBP_CSC_BASE		(BECORE_RGBP_PHYS_BASE + 0x3b00)
/* YUVP carries the same twenty words, bit for bit, 0x100 lower. */
#define BECORE_YUVP_CSC_BASE		(BECORE_YUVP_PHYS_BASE + 0x3a00)
#define BECORE_YUVP_CSC_FIRST		(BECORE_YUVP_CSC_BASE + 0x00)
#define BECORE_YUVP_CSC_LAST		(BECORE_YUVP_CSC_BASE + 0x4c)
#define BECORE_RGBP_CSC_FIRST		(BECORE_RGBP_CSC_BASE + 0x00)
#define BECORE_RGBP_CSC_LAST		(BECORE_RGBP_CSC_BASE + 0x4c)
#define BECORE_RGBP_CSC_Q		13
#define BECORE_RGBP_CSC_FIELD_MASK	GENMASK(13, 0)
#define BECORE_RGBP_CSC_MAX		0xfff	/* full range, not studio */
#define BECORE_RGBP_CSC_CHROMA_OFFSET	0x800
#define BECORE_RGBP_CHROMA_LPF_BASE	(BECORE_RGBP_PHYS_BASE + 0x3c00)
#define BECORE_RGBP_CHROMA_LPF_CTRL_REG	(BECORE_RGBP_CHROMA_LPF_BASE + 0x00)
#define BECORE_RGBP_CHROMA_LPF_FIRST	(BECORE_RGBP_CHROMA_LPF_BASE + 0x08)
#define BECORE_RGBP_CHROMA_LPF_LAST	(BECORE_RGBP_CHROMA_LPF_BASE + 0x0c)
#define BECORE_YUVP_LPF_NORM_REG	(BECORE_YUVP_PHYS_BASE + 0x5150)
/*
 * RGBP's forward gamma is not a tuning curve: at all 65 of its knots the
 * captured output is round(sqrt(x) * 4096) on a 0..4096 input, an exact
 * gamma-2.0 encode. It is there to move linear light into a gamma domain for
 * the blocks downstream -- YUVP's DEGAMMARGB undoes it with an approximate
 * square, and YUVP's own forward gamma is where the creative tone curve lives.
 * So this block needs no values from anywhere.
 *
 * The grid it is sampled on is the same 65-point grid the vendor's tone curve
 * uses -- 8 steps of 8, 12 of 16, 8 of 32, 16 of 64 and 20 of 128 at Q12,
 * tiling 0..4096 exactly and finest near black, where a square root moves
 * fastest. It is a sampling choice, not a rendering one, and it is not the
 * hardware's reset grid, which is coarser below 64.
 *
 * Two shapes to know. The table has a genuine four-register hole between
 * logical points 23 and 24 -- 0x37ec jumps to 0x3800 -- so it is written out
 * as two runs rather than walked by stride. And the 65th knot of each table
 * would need 1 << 12 exactly, one past its 12-bit field, so the vendor stores
 * its distance from the 64th in the register after the table, with the sign in
 * a _DELTA_SIGN that a rising curve never needs.
 */
#define BECORE_RGBP_GAMMA_BASE		(BECORE_RGBP_PHYS_BASE + 0x3600)
#define BECORE_RGBP_GAMMA_CTRL_FIRST	(BECORE_RGBP_GAMMA_BASE + 0x000)
#define BECORE_RGBP_GAMMA_CTRL_LAST	(BECORE_RGBP_GAMMA_BASE + 0x004)
#define BECORE_RGBP_GAMMA_TBL_FIRST	(BECORE_RGBP_GAMMA_BASE + 0x00c)
#define BECORE_RGBP_GAMMA_TBL_LAST	(BECORE_RGBP_GAMMA_BASE + 0x08c)
#define BECORE_RGBP_GAMMA_X_LOW_FIRST	(BECORE_RGBP_GAMMA_BASE + 0x1c0)
#define BECORE_RGBP_GAMMA_X_LOW_LAST	(BECORE_RGBP_GAMMA_BASE + 0x1ec)
#define BECORE_RGBP_GAMMA_X_HIGH_FIRST	(BECORE_RGBP_GAMMA_BASE + 0x200)
#define BECORE_RGBP_GAMMA_X_HIGH_LAST	(BECORE_RGBP_GAMMA_BASE + 0x254)
#define BECORE_RGBP_GAMMA_KNOTS		65
#define BECORE_RGBP_GAMMA_SEGMENTS	(BECORE_RGBP_GAMMA_KNOTS - 1)
#define BECORE_RGBP_GAMMA_Q		12
/* The grid is stored at Q12; the block is told to meet its wider input. */
#define BECORE_RGBP_GAMMA_X_LSHIFT	5
#define BECORE_RGBP_GTM_BASE		(BECORE_RGBP_PHYS_BASE + 0x3900)
#define BECORE_RGBP_GTM_LAST		(BECORE_RGBP_PHYS_BASE + 0x3a94)
#define BECORE_RGBP_GTM_BYPASS		0x000
#define BECORE_RGBP_GTM_GAIN_MODE_EN	0x004
#define BECORE_RGBP_GTM_IN_POINTS	0x008
#define BECORE_RGBP_GTM_OUT_POINTS	0x088
#define BECORE_RGBP_GTM_Y_WEIGHT_0	0x188
#define BECORE_RGBP_GTM_Y_WEIGHT_1	0x18c
#define BECORE_RGBP_GTM_V_BLEND_RATIO	0x190
#define BECORE_RGBP_GTM_INPUT_RSHIFT	0x194
#define BECORE_RGBP_GTM_KNOTS		64
#define BECORE_RGBP_GTM_OUT_SHIFT	5
/* BT.601 luma, Q8: the three sum to 256. */
#define BECORE_RGBP_GTM_Y_WEIGHT_R	77
#define BECORE_RGBP_GTM_Y_WEIGHT_G	150
#define BECORE_RGBP_GTM_Y_WEIGHT_B	29
#define BECORE_RGBP_INPUT_WIDTH_REG	(BECORE_RGBP_PHYS_BASE + 0x1c20)
#define BECORE_RGBP_INPUT_HEIGHT_REG	(BECORE_RGBP_PHYS_BASE + 0x1c24)
#define BECORE_RGBP_INPUT_STRIDE_REG	(BECORE_RGBP_PHYS_BASE + 0x1c28)
#define BECORE_RGBP_INPUT_HEADER_STRIDE_REG \
	(BECORE_RGBP_PHYS_BASE + 0x1c34)
#define BECORE_RGBP_INPUT_BUSINFO_REG	(BECORE_RGBP_PHYS_BASE + 0x1c4c)
/*
 * Two gates YUVP leaves in a state that does not depend on the scene, named
 * from Samsung YUVP v1.20: DTP asserts its own BYPASS -- it is a test-pattern
 * generator, and nothing wants one -- and COUTFIFO0 is left disabled.
 *
 * The third candidate was 0x6000..0x65fc, which Samsung's tables call DRCDIST
 * and whose first register reads as RGB_DRCDIST_BYPASS = 1 -- 359 table words,
 * 21% of the captured program, apparently switched off. They are not. Zeroing
 * them produces a completely black frame, and so does zeroing them while
 * keeping the block's grid geometry, step multipliers and CONFIG words, which
 * was the obvious explanation [HW 2026-08-21].
 *
 * It is not a dynamic-range block with a bypass. It is **local tone mapping**,
 * and the captured words say so on their own: 0x6014..0x601c hold 1225, 2404
 * and 467, the BT.601 luma weights in Q12; 0x6020..0x611c hold 128 strictly
 * monotone samples rising to 32737 of 32768; and 0x64e0..0x65fc hold 144 cells
 * of 0x0100, unity in Q8. The block takes RGB, forms a luma, looks up a tone
 * curve and applies a gain grid. Zeroing a tone curve is why the frame went
 * black. Vendor code agrees: it has a YuvpLtmBlock whose CMDQ configuration
 * takes the image size -- which is why four of these words track the chain
 * geometry between cameras -- and which carries a hardware-backed array of
 * 49152 shorts, exactly the 96 KiB LTM grid this driver already generates as
 * an identity.
 *
 * So the block runs. Its two curves and its CONFIG words are live per-frame
 * tuning and stay in the recipe; the gate, the luma weights, the grid geometry
 * and the vendor's own identity fills are stated below.
 *
 * Lyric's embedded register descriptors name the whole range rgb_diablo_ltm_*,
 * and that is what fixes where each stated run ends: the gain LUT does not
 * stop at 0x65fc -- that is only where the captured program's header ended --
 * it runs to 0x66d4, which is where 122 more unity entries put it.
 *
 *   0x6000  ltm_enable                    the block runs
 *   0x6014  lumacalc_rgby_coeff_r/g/b     BT.601 luma at Q12
 *   0x6120  slcgrid_*                     9 words, from the image size
 *   0x62dc  crecon_satctrl_lut            258 entries, all zero
 *   0x64e0  crecon_luma_lut               130 entries of unity Q8
 *   0x65e4  crecon_gain_lut               122 entries of unity Q8
 *
 * GetDefaultLtm fills the luma and gain LUTs with 0x0100 and clears the
 * saturation LUT, so the two identity fills are the vendor's own and not an
 * artefact of the scene this was captured from -- and neither differs between
 * the rear and front cameras, where the four grid steps do.
 */
#define BECORE_YUVP_COUTFIFO0_EN_REG	(BECORE_YUVP_PHYS_BASE + 0x1200)
#define BECORE_YUVP_DTP_BYPASS_REG	(BECORE_YUVP_PHYS_BASE + 0x3000)
#define BECORE_YUVP_LTM_BASE		(BECORE_YUVP_PHYS_BASE + 0x6000)
#define BECORE_YUVP_LTM_ENABLE_REG	(BECORE_YUVP_LTM_BASE + 0x000)
#define BECORE_YUVP_LTM_LUMA_FIRST	(BECORE_YUVP_LTM_BASE + 0x014)
#define BECORE_YUVP_LTM_LUMA_LAST	(BECORE_YUVP_LTM_BASE + 0x01c)
#define BECORE_YUVP_LTM_GRID_FIRST	(BECORE_YUVP_LTM_BASE + 0x120)
#define BECORE_YUVP_LTM_GRID_LAST	(BECORE_YUVP_LTM_BASE + 0x140)
#define BECORE_YUVP_LTM_SATCTRL_FIRST	(BECORE_YUVP_LTM_BASE + 0x2dc)
#define BECORE_YUVP_LTM_SATCTRL_LAST	(BECORE_YUVP_LTM_BASE + 0x4dc)
#define BECORE_YUVP_LTM_UNITY_FIRST	(BECORE_YUVP_LTM_BASE + 0x4e0)
#define BECORE_YUVP_LTM_UNITY_LAST	(BECORE_YUVP_LTM_BASE + 0x6d4)
/*
 * The grid is a fixed 32 x 24 x 8 bilateral grid: 32 * 24 * 8 cells of eight
 * shorts is 49152, exactly the array YuvpLtmBlock::ConfigureWith carries. It
 * keeps that shape whatever the frame is, so its cells are square only on a
 * 4:3 one -- which is why the horizontal and vertical steps below come out
 * equal here and why this looked like one number for a while.
 */
#define BECORE_LTM_LUMA_Q12_R		1225
#define BECORE_LTM_LUMA_Q12_G		2404
#define BECORE_LTM_LUMA_Q12_B		467
#define BECORE_LTM_SLCGRID_COLUMNS	32
#define BECORE_LTM_SLCGRID_ROWS		24
#define BECORE_LTM_SLCGRID_DEPTH	8
#define BECORE_LTM_SLCGRID_CELL_SHORTS	8
#define BECORE_LTM_UNITY_Q8_PAIR	0x01000100
/*
 * The register block and the 96 KiB buffer are two halves of one thing, and
 * nothing else in this driver says so: state it where both are in scope, so a
 * future edit to either has to answer for the other.
 */
static_assert(BECORE_LTM_SLCGRID_COLUMNS * BECORE_LTM_SLCGRID_ROWS *
	      BECORE_LTM_SLCGRID_DEPTH * BECORE_LTM_SLCGRID_CELL_SHORTS *
	      sizeof(__le16) == BECORE_GRID_SIZE);
/*
 * YUVP's inverse colour matrix: nine signed Q10 coefficients that read as
 * unity in every captured program and do *not* invert the DIABLO_CCM matrix
 * beside them.  The same situation as RGBP's GTM -- the block is not being
 * used, so what it holds is the identity rather than a calibration.  Its
 * config at +0x00 is 0x02020224 and no published table names its fields, so
 * that word stays in the recipe.
 *
 * Whether the nine are packed row- or column-major is unobservable while they
 * are the identity, and the diagonal is at 0, 4 and 8 either way.
 */
#define BECORE_YUVP_INVCCM33_BASE	(BECORE_YUVP_PHYS_BASE + 0x3e00)
#define BECORE_YUVP_INVCCM33_FIRST	(BECORE_YUVP_INVCCM33_BASE + 0x004)
#define BECORE_YUVP_INVCCM33_LAST	(BECORE_YUVP_INVCCM33_BASE + 0x024)
#define BECORE_INVCCM33_Q		10
#define BECORE_INVCCM33_COEFFICIENTS	9

static_assert((BECORE_YUVP_INVCCM33_LAST - BECORE_YUVP_INVCCM33_FIRST) / 4 +
	      1 == BECORE_INVCCM33_COEFFICIENTS);

/*
 * DIABLO_CCM's gate and the three offsets after its matrix.  The nine
 * coefficients between them are the live AWB matrix -- one payload per frame,
 * and per-frame in the invariance census -- so they stay in the recipe.  These
 * four do not move: ApplyDefaults clears the config's bit 0 and nothing sets
 * it, and the offsets are nominally live (AwbFrameData floats 9..11) but zero
 * in every captured program.
 */
#define BECORE_YUVP_CCM_BASE		(BECORE_YUVP_PHYS_BASE + 0x7a00)
#define BECORE_YUVP_CCM_CONFIG_REG	(BECORE_YUVP_CCM_BASE + 0x000)
#define BECORE_YUVP_CCM_MATRIX_FIRST	(BECORE_YUVP_CCM_BASE + 0x004)
#define BECORE_YUVP_CCM_MATRIX_LAST	(BECORE_YUVP_CCM_BASE + 0x024)
#define BECORE_YUVP_CCM_OFFSET_FIRST	(BECORE_YUVP_CCM_BASE + 0x028)
#define BECORE_YUVP_CCM_OFFSET_LAST	(BECORE_YUVP_CCM_BASE + 0x030)

static_assert((BECORE_YUVP_CCM_MATRIX_LAST - BECORE_YUVP_CCM_MATRIX_FIRST) / 4 +
	      1 == EXYNOS_BECORE_CCM_COEFFICIENTS);
static_assert((BECORE_YUVP_CCM_OFFSET_LAST - BECORE_YUVP_CCM_OFFSET_FIRST) / 4 +
	      1 == EXYNOS_BECORE_CCM_OFFSETS);

/*
 * The tone mapper's guide curve: 128 Q15 samples, two to a register with the
 * lower-numbered one in the low half.  It is the per-frame output of the
 * vendor's tone-mapping node rather than a tuning table -- driven by the
 * exposure estimate and the front end's bilateral-grid statistics -- which is
 * why it is a parameters block and not something the driver can state.
 */
#define BECORE_YUVP_LTM_GMAP_FIRST	(BECORE_YUVP_PHYS_BASE + 0x6020)
#define BECORE_YUVP_LTM_GMAP_LAST	(BECORE_YUVP_PHYS_BASE + 0x611c)
#define BECORE_LTM_CURVE_PER_REG	2

static_assert((BECORE_YUVP_LTM_GMAP_LAST - BECORE_YUVP_LTM_GMAP_FIRST) / 4 +
	      1 == EXYNOS_BECORE_LTM_CURVE_POINTS / BECORE_LTM_CURVE_PER_REG);

/*
 * The colour LUT above 0x7b00, which Lyric's descriptors call
 * yuv_diablo_clut_*. Its input stage is three 1D LUTs, one per YUV channel,
 * each 22 registers at 0x7b50, 0x7ba8 and 0x7c00.
 *
 * They carry an exact identity. Google's own packer says how to read them:
 * clut_packing.h stores three consecutive entries per register as 10-bit
 * fields, low first, each entry floor(f * 1024) of a float in [0, 1). Twenty-
 * one registers hold entries 0..62 that way; the twenty-second holds entry 63
 * and then 1024 minus it, because the curve's last point is 1.0 and does not
 * fit the field. Entry n is 16 * n at every one of the 64 points, so the LUTs
 * are the identity ramp over the 10-bit range.
 *
 * The array lengths are not a guess from where a captured header stopped:
 * YuvpClutBlock::ConfigureWith writes each of them as a
 * std::array<unsigned char, 88>, and 88 bytes is 22 registers.
 */
#define BECORE_YUVP_CLUT_BASE		(BECORE_YUVP_PHYS_BASE + 0x7b00)
#define BECORE_YUVP_CLUT_BYPASS_REG	(BECORE_YUVP_CLUT_BASE + 0x000)
#define BECORE_YUVP_CLUT_EN_CONFIG_REG	(BECORE_YUVP_CLUT_BASE + 0x004)
#define BECORE_YUVP_CLUT_MATRIX_FIRST	(BECORE_YUVP_CLUT_BASE + 0x03c)
#define BECORE_YUVP_CLUT_MATRIX_LAST	(BECORE_YUVP_CLUT_BASE + 0x04c)
#define BECORE_YUVP_CLUT_1DLUT_Y_FIRST	(BECORE_YUVP_CLUT_BASE + 0x050)
#define BECORE_YUVP_CLUT_1DLUT_U_FIRST	(BECORE_YUVP_CLUT_BASE + 0x0a8)
#define BECORE_YUVP_CLUT_1DLUT_V_FIRST	(BECORE_YUVP_CLUT_BASE + 0x100)
/* 64 entries, three per register plus the last one's distance from unity. */
#define BECORE_CLUT_1DLUT_ENTRIES	64
#define BECORE_CLUT_1DLUT_PER_REG	3
#define BECORE_CLUT_1DLUT_REGS		22
#define BECORE_CLUT_1DLUT_LAST		((BECORE_CLUT_1DLUT_REGS - 1) * 4)
#define BECORE_CLUT_FIELD_BITS		10
#define BECORE_CLUT_FIELD_MAX		0x3ff
#define BECORE_CLUT_ONE			(1 << BECORE_CLUT_FIELD_BITS)
/* The identity's step: 1024 / 64, so entry 63 is 1008 and its delta is 16. */
#define BECORE_CLUT_1DLUT_STEP \
	(BECORE_CLUT_ONE / BECORE_CLUT_1DLUT_ENTRIES)

static_assert((BECORE_CLUT_1DLUT_REGS - 1) * BECORE_CLUT_1DLUT_PER_REG ==
	      BECORE_CLUT_1DLUT_ENTRIES - 1);
/* The distance from the last entry to unity has to fit a field of its own. */
static_assert(BECORE_CLUT_ONE - (BECORE_CLUT_1DLUT_ENTRIES - 1) *
	      BECORE_CLUT_1DLUT_STEP <= BECORE_CLUT_FIELD_MAX);
/*
 * en_config gates the block's four optional stages, one bit each: the
 * YUV-to-RGB matrix at bit 0 and the three input curves above it.
 * TranslateStaticClut fills each bit from the corresponding tuning message's
 * own enable flag, and the vendor enables only the matrix -- the curves are
 * written and left off, which is consistent with their being the identity.
 */
#define BECORE_CLUT_EN_MATRIX		BIT(0)
/*
 * Nine coefficients as 13-bit signed fields, two per register, row-major. The
 * ninth has no partner, so the fifth register's upper half is unused.
 * TranslateStaticClut encodes each through QCodec(3, 10): three integer bits
 * and ten fractional.
 */
#define BECORE_CLUT_MATRIX_Q		10
#define BECORE_CLUT_MATRIX_FIELD_MASK	GENMASK(12, 0)
#define BECORE_CLUT_MATRIX_COEFFICIENTS	9

static_assert((BECORE_YUVP_CLUT_MATRIX_LAST - BECORE_YUVP_CLUT_MATRIX_FIRST) /
	      4 + 1 == (BECORE_CLUT_MATRIX_COEFFICIENTS + 1) / 2);

/*
 * The 17^3 lattice, and the two registers it is streamed through: an index
 * that arms the write port at entry zero and the port itself. There is no way
 * to read a lattice back out, so the port is write-only in both senses.
 *
 * The stream is one (U, V) pair per node in node order, packed three 10-bit
 * fields to a word with the low field first -- so a node's pair straddles word
 * boundaries and the whole thing is a flat list of values rather than a table
 * with a row length. 9,826 values do not divide by three, so the last word
 * carries two spare fields, and the only sensible padding is the value that
 * means no chroma.
 */
#define BECORE_YUVP_CLUT_INDEX_REG	(BECORE_YUVP_CLUT_BASE + 0x010)
#define BECORE_YUVP_CLUT_FIFO_REG	(BECORE_YUVP_CLUT_BASE + 0x014)
#define BECORE_CLUT_FIFO_OPEN		0x01000000
#define BECORE_CLUT_FIELDS_PER_WORD	3
#define BECORE_CLUT_LATTICE_VALUES	(EXYNOS_BECORE_CLUT_NODES * 2)
#define BECORE_CLUT_LATTICE_WORDS \
	DIV_ROUND_UP(BECORE_CLUT_LATTICE_VALUES, BECORE_CLUT_FIELDS_PER_WORD)
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
static_assert(EXYNOS_BECORE_CLUT_MAX == BECORE_CLUT_FIELD_MAX);

#define BECORE_YUVP_GRID_REG		(BECORE_YUVP_PHYS_BASE + 0x1c50)
#define BECORE_YUVP_OUTPUT_PLANE1_REG	(BECORE_YUVP_PHYS_BASE + 0x2450)
#define BECORE_YUVP_OUTPUT_PLANE2_REG	(BECORE_YUVP_PHYS_BASE + 0x2490)
#define BECORE_YUVP_OUTPUT_ENABLE_REG	(BECORE_YUVP_PHYS_BASE + 0x2400)
#define BECORE_YUVP_OUTPUT_MODE_REG	(BECORE_YUVP_PHYS_BASE + 0x2404)
#define BECORE_YUVP_OUTPUT_FORMAT_REG	(BECORE_YUVP_PHYS_BASE + 0x2410)
#define BECORE_YUVP_OUTPUT_LOSSY_REG	(BECORE_YUVP_PHYS_BASE + 0x2418)
#define BECORE_YUVP_OUTPUT_WIDTH_REG	(BECORE_YUVP_PHYS_BASE + 0x2420)
#define BECORE_YUVP_OUTPUT_HEIGHT_REG	(BECORE_YUVP_PHYS_BASE + 0x2424)
#define BECORE_YUVP_OUTPUT_STRIDE1_REG	(BECORE_YUVP_PHYS_BASE + 0x2428)
#define BECORE_YUVP_OUTPUT_STRIDE2_REG	(BECORE_YUVP_PHYS_BASE + 0x242c)
#define BECORE_YUVP_OUTPUT_VOTF_REG	(BECORE_YUVP_PHYS_BASE + 0x243c)
#define BECORE_YUVP_OUTPUT_BUSINFO_REG	(BECORE_YUVP_PHYS_BASE + 0x244c)

#define BECORE_GTNR_INPUT_PLANE1_REG	(BECORE_GTNR_PHYS_BASE + 0x1e50)
#define BECORE_GTNR_INPUT_PLANE2_REG	(BECORE_GTNR_PHYS_BASE + 0x1e90)
#define BECORE_GTNR_INPUT_VOTF_REG	(BECORE_GTNR_PHYS_BASE + 0x1e3c)
#define BECORE_GTNR_INPUT_FORMAT_REG	(BECORE_GTNR_PHYS_BASE + 0x1e10)
#define BECORE_GTNR_INPUT_LOSSY_REG	(BECORE_GTNR_PHYS_BASE + 0x1e18)
#define BECORE_GTNR_INPUT_COMP_REG	(BECORE_GTNR_PHYS_BASE + 0x1e04)
#define BECORE_GTNR_INPUT_WIDTH_REG	(BECORE_GTNR_PHYS_BASE + 0x1e20)
#define BECORE_GTNR_INPUT_HEIGHT_REG	(BECORE_GTNR_PHYS_BASE + 0x1e24)
#define BECORE_GTNR_INPUT_STRIDE1_REG	(BECORE_GTNR_PHYS_BASE + 0x1e28)
#define BECORE_GTNR_INPUT_STRIDE2_REG	(BECORE_GTNR_PHYS_BASE + 0x1e2c)
#define BECORE_GTNR_INPUT_BUSINFO_REG	(BECORE_GTNR_PHYS_BASE + 0x1e4c)
#define BECORE_GTNR_INPUT_MAX_MO_REG	(BECORE_GTNR_PHYS_BASE + 0x1e40)
#define BECORE_GTNR_INPUT_MAX_BL_REG	(BECORE_GTNR_PHYS_BASE + 0x1e48)
#define BECORE_GTNR_INPUT_ENABLE_REG	(BECORE_GTNR_PHYS_BASE + 0x1e00)
#define BECORE_GTNR_OUTPUT_PLANE1_REG	(BECORE_GTNR_PHYS_BASE + 0x3050)
#define BECORE_GTNR_OUTPUT_PLANE2_REG	(BECORE_GTNR_PHYS_BASE + 0x3090)
#define BECORE_GTNR_OUTPUT_FORMAT_REG	(BECORE_GTNR_PHYS_BASE + 0x3010)
#define BECORE_GTNR_OUTPUT_LOSSY_REG	(BECORE_GTNR_PHYS_BASE + 0x3018)
#define BECORE_GTNR_OUTPUT_COMP_REG	(BECORE_GTNR_PHYS_BASE + 0x3004)
#define BECORE_GTNR_OUTPUT_WIDTH_REG	(BECORE_GTNR_PHYS_BASE + 0x3020)
#define BECORE_GTNR_OUTPUT_HEIGHT_REG	(BECORE_GTNR_PHYS_BASE + 0x3024)
#define BECORE_GTNR_OUTPUT_STRIDE1_REG	(BECORE_GTNR_PHYS_BASE + 0x3028)
#define BECORE_GTNR_OUTPUT_STRIDE2_REG	(BECORE_GTNR_PHYS_BASE + 0x302c)
#define BECORE_GTNR_OUTPUT_BUSINFO_REG	(BECORE_GTNR_PHYS_BASE + 0x304c)
#define BECORE_GTNR_OUTPUT_MAX_MO_REG	(BECORE_GTNR_PHYS_BASE + 0x3040)
#define BECORE_GTNR_OUTPUT_MAX_BL_REG	(BECORE_GTNR_PHYS_BASE + 0x3048)
#define BECORE_GTNR_OUTPUT_ENABLE_REG	(BECORE_GTNR_PHYS_BASE + 0x3000)

#define BECORE_MCSC_INPUT_PLANE1_REG	(BECORE_MCSC_PHYS_BASE + 0x1850)
#define BECORE_MCSC_INPUT_PLANE2_REG	(BECORE_MCSC_PHYS_BASE + 0x1890)
#define BECORE_MCSC_INPUT_VOTF_REG	(BECORE_MCSC_PHYS_BASE + 0x183c)
#define BECORE_MCSC_INPUT_FORMAT_REG	(BECORE_MCSC_PHYS_BASE + 0x1810)
#define BECORE_MCSC_INPUT_LOSSY_REG	(BECORE_MCSC_PHYS_BASE + 0x1818)
#define BECORE_MCSC_INPUT_COMP_REG	(BECORE_MCSC_PHYS_BASE + 0x1804)
#define BECORE_MCSC_INPUT_WIDTH_REG	(BECORE_MCSC_PHYS_BASE + 0x1820)
#define BECORE_MCSC_INPUT_HEIGHT_REG	(BECORE_MCSC_PHYS_BASE + 0x1824)
#define BECORE_MCSC_INPUT_STRIDE1_REG	(BECORE_MCSC_PHYS_BASE + 0x1828)
#define BECORE_MCSC_INPUT_STRIDE2_REG	(BECORE_MCSC_PHYS_BASE + 0x182c)
#define BECORE_MCSC_INPUT_BUSINFO_REG	(BECORE_MCSC_PHYS_BASE + 0x184c)
#define BECORE_MCSC_INPUT_MAX_BL_REG	(BECORE_MCSC_PHYS_BASE + 0x1848)
#define BECORE_MCSC_INPUT_ENABLE_REG	(BECORE_MCSC_PHYS_BASE + 0x1800)
#define BECORE_MCSC_OUTPUT_PLANE1_REG	(BECORE_MCSC_PHYS_BASE + 0x2050)
#define BECORE_MCSC_OUTPUT_PLANE2_REG	(BECORE_MCSC_PHYS_BASE + 0x2090)
#define BECORE_MCSC_OUTPUT_FORMAT_REG	(BECORE_MCSC_PHYS_BASE + 0x2010)
#define BECORE_MCSC_OUTPUT_COMP_REG	(BECORE_MCSC_PHYS_BASE + 0x2004)
#define BECORE_MCSC_OUTPUT_WIDTH_REG	(BECORE_MCSC_PHYS_BASE + 0x2020)
#define BECORE_MCSC_OUTPUT_HEIGHT_REG	(BECORE_MCSC_PHYS_BASE + 0x2024)
#define BECORE_MCSC_OUTPUT_STRIDE1_REG	(BECORE_MCSC_PHYS_BASE + 0x2028)
#define BECORE_MCSC_OUTPUT_STRIDE2_REG	(BECORE_MCSC_PHYS_BASE + 0x202c)
#define BECORE_MCSC_OUTPUT_BUSINFO_REG	(BECORE_MCSC_PHYS_BASE + 0x204c)
#define BECORE_MCSC_OUTPUT_MAX_BL_REG	(BECORE_MCSC_PHYS_BASE + 0x2048)
#define BECORE_MCSC_OUTPUT_ENABLE_REG	(BECORE_MCSC_PHYS_BASE + 0x2000)
#define BECORE_MCSC_OUTPUT_DITHER_REG	(BECORE_MCSC_PHYS_BASE + 0x2f00)
/*
 * DJAG's pre-scaler geometry -- Samsung MCSC v10.1 names these
 * YUV_DJAG_IMG_SIZE, YUV_DJAG_PS_SRC_POS/SRC_SIZE/DST_SIZE and
 * YUV_DJAG_PS_H/V_RATIO. This is the block that crops and scales, and
 * POLY_SC0 downstream of it therefore maps the output raster onto itself.
 *
 * Each of the first four registers packs two 16-bit halves with the width in
 * the high one, and the two ratios are the crop as a 20-bit fixed-point
 * fraction of the destination -- Samsung's own GET_ZOOM_RATIO(in, out), which
 * is ((in) << MCSC_PRECISION) / (out) with MCSC_PRECISION 20. This driver
 * crops the whole 4160 x 3120 raster into 4000 x 3000, which truncates to
 * 0x0010a3d7 on both axes; the vendor cropped 3536 x 2652 out of it and
 * carried 0x000e24dd, spending the difference on stabilisation.
 */
#define BECORE_MCSC_DJAG_IMG_SIZE_REG	(BECORE_MCSC_PHYS_BASE + 0x4004)
#define BECORE_MCSC_DJAG_PS_SRC_POS_REG	(BECORE_MCSC_PHYS_BASE + 0x4008)
#define BECORE_MCSC_DJAG_PS_SRC_SIZE_REG (BECORE_MCSC_PHYS_BASE + 0x400c)
#define BECORE_MCSC_DJAG_PS_DST_SIZE_REG	(BECORE_MCSC_PHYS_BASE + 0x4010)
#define BECORE_MCSC_DJAG_PS_H_RATIO_REG	(BECORE_MCSC_PHYS_BASE + 0x4014)
#define BECORE_MCSC_DJAG_PS_V_RATIO_REG	(BECORE_MCSC_PHYS_BASE + 0x4018)
#define BECORE_RATIO_SHIFT		20
/*
 * MCSC's chain below DJAG, named from Samsung MCSC v10.1. DJAG has already
 * produced the output raster, so the POLY_SC0 scaler and the POST_PC0 chroma
 * converter each map that raster onto itself: neither crops and neither
 * scales. Carried as constants these did not move when the output geometry
 * did, and 4000 x 3000 happened to be right; derived, they cannot disagree
 * with the surface MCSC writes.
 */
#define BECORE_MCSC_SC0_SRC_POS_REG	(BECORE_MCSC_PHYS_BASE + 0x5004)
#define BECORE_MCSC_SC0_SRC_SIZE_REG	(BECORE_MCSC_PHYS_BASE + 0x5008)
#define BECORE_MCSC_SC0_DST_SIZE_REG	(BECORE_MCSC_PHYS_BASE + 0x500c)
#define BECORE_MCSC_SC0_H_RATIO_REG	(BECORE_MCSC_PHYS_BASE + 0x5010)
#define BECORE_MCSC_SC0_V_RATIO_REG	(BECORE_MCSC_PHYS_BASE + 0x5014)
#define BECORE_MCSC_PC0_IMG_SIZE_REG	(BECORE_MCSC_PHYS_BASE + 0x6004)
#define BECORE_MCSC_PC0_DST_SIZE_REG	(BECORE_MCSC_PHYS_BASE + 0x6008)
#define BECORE_MCSC_PC0_H_RATIO_REG	(BECORE_MCSC_PHYS_BASE + 0x600c)
#define BECORE_MCSC_PC0_V_RATIO_REG	(BECORE_MCSC_PHYS_BASE + 0x6010)
/*
 * The two poly-phase scalers' filter coefficients. RGBP's SC and MCSC's
 * POLY_SC0 carry bit-identical tables, at 0x4500/0x4548 and 0x5024/0x506c
 * respectively: nine phases of four vertical and eight horizontal taps, two
 * taps to a register with the lower-numbered one in the low half.
 */
#define BECORE_RGBP_SC_V_COEFF_FIRST	(BECORE_RGBP_PHYS_BASE + 0x4500)
#define BECORE_RGBP_SC_V_COEFF_LAST	(BECORE_RGBP_PHYS_BASE + 0x4544)
#define BECORE_RGBP_SC_H_COEFF_FIRST	(BECORE_RGBP_PHYS_BASE + 0x4548)
#define BECORE_RGBP_SC_H_COEFF_LAST	(BECORE_RGBP_PHYS_BASE + 0x45d4)
#define BECORE_MCSC_SC0_V_COEFF_FIRST	(BECORE_MCSC_PHYS_BASE + 0x5024)
#define BECORE_MCSC_SC0_V_COEFF_LAST	(BECORE_MCSC_PHYS_BASE + 0x5068)
#define BECORE_MCSC_SC0_H_COEFF_FIRST	(BECORE_MCSC_PHYS_BASE + 0x506c)
#define BECORE_MCSC_SC0_H_COEFF_LAST	(BECORE_MCSC_PHYS_BASE + 0x50f8)
#define BECORE_SC_PHASES		9
#define BECORE_SC_SETS			7
#define BECORE_SC_V_TAPS		4
#define BECORE_SC_H_TAPS		8
#define BECORE_SC_COEFF_MASK		GENMASK(10, 0)

/* Samsung's seven tap sets, in the order the ratio bands select them. */
enum becore_sc_set {
	BECORE_SC_SET_X8_8,
	BECORE_SC_SET_X7_8,
	BECORE_SC_SET_X6_8,
	BECORE_SC_SET_X5_8,
	BECORE_SC_SET_X4_8,
	BECORE_SC_SET_X3_8,
	BECORE_SC_SET_X2_8,
};

static_assert(BECORE_SC_SET_X2_8 + 1 == BECORE_SC_SETS);

/*
 * becore_sc_coeff_value() turns a register's index within its range into a
 * (phase, tap pair), which is only right while each range is exactly one
 * register per tap pair per phase. Both blocks carry both scalers, so say it
 * once for all four ranges rather than trusting four address literals. The
 * span is the byte distance between a range's first and last register, which
 * is one register short of the count.
 */
#define BECORE_SC_COEFF_SPAN(taps) \
	((taps) / 2 * BECORE_SC_PHASES * 4 - 4)
static_assert(BECORE_RGBP_SC_V_COEFF_LAST - BECORE_RGBP_SC_V_COEFF_FIRST ==
	      BECORE_SC_COEFF_SPAN(BECORE_SC_V_TAPS));
static_assert(BECORE_RGBP_SC_H_COEFF_LAST - BECORE_RGBP_SC_H_COEFF_FIRST ==
	      BECORE_SC_COEFF_SPAN(BECORE_SC_H_TAPS));
static_assert(BECORE_MCSC_SC0_V_COEFF_LAST - BECORE_MCSC_SC0_V_COEFF_FIRST ==
	      BECORE_SC_COEFF_SPAN(BECORE_SC_V_TAPS));
static_assert(BECORE_MCSC_SC0_H_COEFF_LAST - BECORE_MCSC_SC0_H_COEFF_FIRST ==
	      BECORE_SC_COEFF_SPAN(BECORE_SC_H_TAPS));

enum becore_block_id {
	BECORE_RGBP,
	BECORE_MCFP,
	BECORE_YUVP,
	BECORE_MCSC,
	BECORE_NUM_BLOCKS,
};

enum becore_c2serv_id {
	BECORE_C2SERV_RGBP,
	BECORE_C2SERV_YUVP,
	BECORE_C2SERV_MCSC,
	BECORE_NUM_C2SERV,
};

struct becore_regval {
	u32 offset;
	u32 value;
};

struct becore_override {
	u32 reg;
	u32 value;
};

struct becore_rect {
	u32 x;
	u32 y;
	u32 width;
	u32 height;
};

struct becore_rgbp_input_profile {
	u32 width;
	u32 height;
	u32 data_format;
	u32 comp_control;
	u32 sbwc_block_width;
	u32 bytes_per_pixel;
	u32 header_stride;
	u32 businfo;
};

enum becore_rgbp_input_profile_id {
	BECORE_RGBP_INPUT_SBWC,
	BECORE_RGBP_INPUT_LINEAR,
	BECORE_RGBP_INPUT_PROFILE_COUNT,
};

/*
 * The active dimensions and DMA fields are from the live ultrawide program.
 * As in Pablo's common DMA API, the payload and header geometry are derived
 * from the image profile.  Lyric additionally writes the 256-pixel-aligned
 * SBWC storage width after enabling the RDMA.
 *
 * Every word RGBP's input section programs comes from one of these, and so do
 * the crop, the scaler ratios and the frame sizes further down the chain, so
 * the profile is resolved once and passed to each of them rather than read
 * from module scope -- the shape becore_yuvp_outputs[] already has.
 */
static const struct becore_rgbp_input_profile
becore_rgbp_inputs[BECORE_RGBP_INPUT_PROFILE_COUNT] = {
	[BECORE_RGBP_INPUT_SBWC] = {
		.width = 4208,
		.height = 3120,
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
		.width = 4208,
		.height = 3120,
		.data_format = 0x1a,
		.comp_control = 0,
		.sbwc_block_width = 1,
		.bytes_per_pixel = 2,
		.header_stride = 0,
		.businfo = 0,
	},
};

enum becore_rgbp_input_word {
	BECORE_RGBP_CROP_SIZE,
	BECORE_RGBP_CROP_START,
	BECORE_RGBP_SC_DST_SIZE,
	BECORE_RGBP_SC_H_RATIO,
	BECORE_RGBP_SC_V_RATIO,
	BECORE_RGBP_CHAIN_SRC_SIZE,
	BECORE_RGBP_CHAIN_DST_SIZE,
	BECORE_RGBP_INPUT_FORMAT,
	BECORE_RGBP_INPUT_COMP,
	BECORE_RGBP_INPUT_ACTIVE_WIDTH,
	BECORE_RGBP_INPUT_HEIGHT,
	BECORE_RGBP_INPUT_STRIDE,
	BECORE_RGBP_INPUT_HEADER_STRIDE,
	BECORE_RGBP_INPUT_BUSINFO,
	BECORE_RGBP_INPUT_ENABLE,
	BECORE_RGBP_INPUT_STORAGE_WIDTH,
	BECORE_RGBP_INPUT_WORD_COUNT,
};

/*
 * A value the driver generates from what the block it programs is *for*.
 *
 * Unlike a typed word, which is matched to its register by its position in the
 * program, one of these is resolved by register address alone -- so an entry
 * cannot drift onto a neighbouring word, and a range can cover a whole block
 * without naming each of its registers.
 */
enum becore_generated_kind {
	BECORE_GEN_OFF,		/* an enable the program leaves clear */
	BECORE_GEN_BYPASS,	/* an asserted bypass bit */
	BECORE_GEN_RUNNING,	/* a bypass the program clears: the block runs */
	BECORE_GEN_DECOMP_SIZE,	/* a frame size, from the Bayer input */
	BECORE_GEN_CSC,		/* RGB to YUV: BT.601, full range, Q13 */
	BECORE_GEN_CHROMA_LPF,	/* 4:4:4 to 4:2:2, a fixed binomial filter */
	BECORE_GEN_DJAG,	/* MCSC DJAG at Samsung's neutral profile */
	BECORE_GEN_DMSC,	/* what GetDefaultDmsc writes after the tuning */
	BECORE_GEN_DNS_GEOMETRY,	/* binning and radial centre, from the array */
	BECORE_GEN_GAMMA,	/* RGBP's forward gamma, a square-root encode */
	BECORE_GEN_LPF_NORM,	/* log2 of the sharpener's three kernel sums */
	BECORE_GEN_NOISE_SLOPE,	/* a noise curve's slopes, from its own knots */
	BECORE_GEN_NOISE_SHIFT,	/* the shift those slopes are taken at */
	BECORE_GEN_NOISE_DOMAIN,	/* a chroma domain repeating the luma one */
	BECORE_GEN_SCALER_PHASE,	/* a scaler starting on a pixel, rounding */
	BECORE_GEN_MCSC_INPUT_SIZE,	/* the raster MCSC reads, from YUVP */
	BECORE_GEN_GTM,		/* RGBP's tone map, an identity */
	BECORE_GEN_LTM,		/* YUVP's tone mapping: gate, luma, grid, identity */
	BECORE_GEN_CLUT_1DLUT,	/* the colour LUT's per-channel input identity */
	BECORE_GEN_CLUT,	/* the colour LUT's gate and its YUV-to-RGB matrix */
	BECORE_GEN_INVCCM33,	/* the inverse colour matrix, an exact identity */
	BECORE_GEN_CCM,		/* the colour matrix's gate and its zero offsets */
	BECORE_GEN_CHAIN_SIZE,	/* a raster size, from the output profile */
	BECORE_GEN_CHAIN_ORIGIN,	/* a chain stage that does not crop */
	BECORE_GEN_CHAIN_RATIO,	/* a chain stage that does not scale */
	BECORE_GEN_SC_V_COEFF,	/* poly-phase vertical taps, from the ratio */
	BECORE_GEN_SC_H_COEFF,	/* poly-phase horizontal taps, from the ratio */
};

struct becore_generated_range {
	u32 first;		/* physical register, inclusive */
	u32 last;		/* inclusive; equal to first for one register */
	u32 kind;
};

/*
 * How many words each block hands to becore_generated_value(). Held here
 * rather than in the generated table so that a recipe which quietly stopped
 * carrying one of them fails validation instead of programming the capture.
 */
#define BECORE_RGBP_GENERATED_WORDS	291
#define BECORE_YUVP_GENERATED_WORDS	391
#define BECORE_MCSC_GENERATED_WORDS	99

static const struct becore_generated_range becore_rgbp_generated[] = {
	{ BECORE_RGBP_CINFIFO_FRAME_IN_REG, BECORE_RGBP_CINFIFO_FRAME_IN_REG,
	  BECORE_GEN_OFF },
	{ BECORE_RGBP_SATFLAG_ENABLE_REG, BECORE_RGBP_SATFLAG_ENABLE_REG,
	  BECORE_GEN_OFF },
	{ BECORE_RGBP_DMSCCROP_BYPASS_REG, BECORE_RGBP_DMSCCROP_BYPASS_REG,
	  BECORE_GEN_RUNNING },
	{ BECORE_RGBP_WDMADECOMP_EN_REG, BECORE_RGBP_WDMADECOMP_EN_REG,
	  BECORE_GEN_OFF },
	{ BECORE_RGBP_WDMAY_EN_REG, BECORE_RGBP_WDMAY_EN_REG,
	  BECORE_GEN_OFF },
	{ BECORE_RGBP_WDMAUV_EN_REG, BECORE_RGBP_WDMAUV_EN_REG,
	  BECORE_GEN_OFF },
	{ BECORE_RGBP_DNS_SLOPE_G_REG,
	  BECORE_RGBP_DNS_SLOPE_G_REG + BECORE_NOISE_TABLE_LAST,
	  BECORE_GEN_NOISE_SLOPE },
	{ BECORE_RGBP_DNS_SHIFT_G_REG, BECORE_RGBP_DNS_SHIFT_G_REG,
	  BECORE_GEN_NOISE_SHIFT },
	{ BECORE_RGBP_DNS_X_RB_REG,
	  BECORE_RGBP_DNS_X_RB_REG + BECORE_NOISE_TABLE_LAST,
	  BECORE_GEN_NOISE_DOMAIN },
	{ BECORE_RGBP_DNS_SLOPE_RB_REG,
	  BECORE_RGBP_DNS_SLOPE_RB_REG + BECORE_NOISE_TABLE_LAST,
	  BECORE_GEN_NOISE_SLOPE },
	{ BECORE_RGBP_DNS_SHIFT_RB_REG, BECORE_RGBP_DNS_SHIFT_RB_REG,
	  BECORE_GEN_NOISE_SHIFT },
	{ BECORE_RGBP_DMSC_BASE + 0x20c, BECORE_RGBP_DMSC_BASE + 0x20c,
	  BECORE_GEN_DMSC },			/* BASE_CONFIG */
	{ BECORE_RGBP_DMSC_BASE + 0x238, BECORE_RGBP_DMSC_BASE + 0x238,
	  BECORE_GEN_DMSC },			/* EXTRACT_COLORS_CONFIG */
	{ BECORE_RGBP_DMSC_BASE + 0x24c, BECORE_RGBP_DMSC_BASE + 0x250,
	  BECORE_GEN_DMSC },			/* GREEN_HUE, GREEN_SAT */
	{ BECORE_RGBP_DMSC_BASE + 0x258, BECORE_RGBP_DMSC_BASE + 0x258,
	  BECORE_GEN_DMSC },			/* POST_PROCESS_CONFIG */
	{ BECORE_RGBP_DMSC_BASE + 0x264, BECORE_RGBP_DMSC_BASE + 0x264,
	  BECORE_GEN_DMSC },			/* DIR_DETECT_SELECTION */
	{ BECORE_RGBP_DMSC_BASE + 0x26c, BECORE_RGBP_DMSC_BASE + 0x274,
	  BECORE_GEN_DMSC },			/* ADD_COLORS_GREEN..SHARPENING */
	{ BECORE_RGBP_DMSC_BASE + 0x288, BECORE_RGBP_DMSC_BASE + 0x288,
	  BECORE_GEN_DMSC },			/* NEAR_EDGE_DESAT_EN */
	{ BECORE_RGBP_DMSC_BASE + 0x2a0, BECORE_RGBP_DMSC_BASE + 0x2a0,
	  BECORE_GEN_DMSC },			/* RED_PRESERVE_EN; its GAIN
						 * at 0x2a4 is tuning
						 */
	{ BECORE_RGBP_DMSC_BASE + 0x2a8, BECORE_RGBP_DMSC_BASE + 0x2ac,
	  BECORE_GEN_DMSC },			/* RED_PRESERVE_THRES, _LIMIT */
	{ BECORE_RGBP_DMSC_BASE + 0x2b4, BECORE_RGBP_DMSC_BASE + 0x2b4,
	  BECORE_GEN_DMSC },			/* ADD_YBLUR */
	{ BECORE_RGBP_DNS_BINNING_REG, BECORE_RGBP_DNS_BINNING_REG,
	  BECORE_GEN_DNS_GEOMETRY },
	{ BECORE_RGBP_DNS_CENTRE_REG, BECORE_RGBP_DNS_CENTRE_REG,
	  BECORE_GEN_DNS_GEOMETRY },
	{ BECORE_RGBP_GAMMA_CTRL_FIRST, BECORE_RGBP_GAMMA_CTRL_LAST,
	  BECORE_GEN_GAMMA },
	{ BECORE_RGBP_GAMMA_TBL_FIRST, BECORE_RGBP_GAMMA_TBL_LAST,
	  BECORE_GEN_GAMMA },
	{ BECORE_RGBP_GAMMA_X_LOW_FIRST, BECORE_RGBP_GAMMA_X_LOW_LAST,
	  BECORE_GEN_GAMMA },
	{ BECORE_RGBP_GAMMA_X_HIGH_FIRST, BECORE_RGBP_GAMMA_X_HIGH_LAST,
	  BECORE_GEN_GAMMA },
	{ BECORE_RGBP_GTM_BASE, BECORE_RGBP_GTM_LAST, BECORE_GEN_GTM },
	{ BECORE_RGBP_CSC_FIRST, BECORE_RGBP_CSC_LAST, BECORE_GEN_CSC },
	{ BECORE_RGBP_CHROMA_LPF_CTRL_REG, BECORE_RGBP_CHROMA_LPF_CTRL_REG,
	  BECORE_GEN_CHROMA_LPF },
	{ BECORE_RGBP_CHROMA_LPF_FIRST, BECORE_RGBP_CHROMA_LPF_LAST,
	  BECORE_GEN_CHROMA_LPF },
	{ BECORE_RGBP_DECOMP_BYPASS_REG, BECORE_RGBP_DECOMP_BYPASS_REG,
	  BECORE_GEN_BYPASS },
	{ BECORE_RGBP_DECOMP_SIZE_REG, BECORE_RGBP_DECOMP_SIZE_REG,
	  BECORE_GEN_DECOMP_SIZE },
	{ BECORE_RGBP_GAMMALR_BYPASS_REG, BECORE_RGBP_GAMMALR_BYPASS_REG,
	  BECORE_GEN_BYPASS },
	{ BECORE_RGBP_UPSC_CTRL0_REG, BECORE_RGBP_UPSC_CTRL0_REG,
	  BECORE_GEN_OFF },
	{ BECORE_RGBP_GAMMAHR_BYPASS_REG, BECORE_RGBP_GAMMAHR_BYPASS_REG,
	  BECORE_GEN_BYPASS },
	{ BECORE_RGBP_SC_PHASE_FIRST, BECORE_RGBP_SC_PHASE_LAST,
	  BECORE_GEN_SCALER_PHASE },
	{ BECORE_RGBP_SC_V_COEFF_FIRST, BECORE_RGBP_SC_V_COEFF_LAST,
	  BECORE_GEN_SC_V_COEFF },
	{ BECORE_RGBP_SC_H_COEFF_FIRST, BECORE_RGBP_SC_H_COEFF_LAST,
	  BECORE_GEN_SC_H_COEFF },
};

static const struct becore_generated_range becore_yuvp_generated[] = {
	{ BECORE_YUVP_COUTFIFO0_EN_REG, BECORE_YUVP_COUTFIFO0_EN_REG,
	  BECORE_GEN_OFF },
	{ BECORE_YUVP_DTP_BYPASS_REG, BECORE_YUVP_DTP_BYPASS_REG,
	  BECORE_GEN_BYPASS },
	{ BECORE_YUVP_CSC_FIRST, BECORE_YUVP_CSC_LAST, BECORE_GEN_CSC },
	{ BECORE_YUVP_NR_SLOPE_Y_REG,
	  BECORE_YUVP_NR_SLOPE_Y_REG + BECORE_NOISE_TABLE_LAST,
	  BECORE_GEN_NOISE_SLOPE },
	{ BECORE_YUVP_NR_SHIFT_Y_REG, BECORE_YUVP_NR_SHIFT_Y_REG,
	  BECORE_GEN_NOISE_SHIFT },
	{ BECORE_YUVP_NR_X_UV_REG,
	  BECORE_YUVP_NR_X_UV_REG + BECORE_NOISE_TABLE_LAST,
	  BECORE_GEN_NOISE_DOMAIN },
	{ BECORE_YUVP_NR_SLOPE_UV_REG,
	  BECORE_YUVP_NR_SLOPE_UV_REG + BECORE_NOISE_TABLE_LAST,
	  BECORE_GEN_NOISE_SLOPE },
	{ BECORE_YUVP_NR_SHIFT_UV_REG, BECORE_YUVP_NR_SHIFT_UV_REG,
	  BECORE_GEN_NOISE_SHIFT },
	{ BECORE_YUVP_LPF_NORM_REG, BECORE_YUVP_LPF_NORM_REG,
	  BECORE_GEN_LPF_NORM },
	{ BECORE_YUVP_LTM_ENABLE_REG, BECORE_YUVP_LTM_ENABLE_REG,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_LUMA_FIRST, BECORE_YUVP_LTM_LUMA_LAST,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_GRID_FIRST, BECORE_YUVP_LTM_GRID_LAST,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_SATCTRL_FIRST, BECORE_YUVP_LTM_SATCTRL_LAST,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_LTM_UNITY_FIRST, BECORE_YUVP_LTM_UNITY_LAST,
	  BECORE_GEN_LTM },
	{ BECORE_YUVP_INVCCM33_FIRST, BECORE_YUVP_INVCCM33_LAST,
	  BECORE_GEN_INVCCM33 },
	{ BECORE_YUVP_CCM_CONFIG_REG, BECORE_YUVP_CCM_CONFIG_REG,
	  BECORE_GEN_CCM },
	{ BECORE_YUVP_CCM_OFFSET_FIRST, BECORE_YUVP_CCM_OFFSET_LAST,
	  BECORE_GEN_CCM },
	{ BECORE_YUVP_CLUT_BYPASS_REG, BECORE_YUVP_CLUT_EN_CONFIG_REG,
	  BECORE_GEN_CLUT },
	{ BECORE_YUVP_CLUT_MATRIX_FIRST, BECORE_YUVP_CLUT_MATRIX_LAST,
	  BECORE_GEN_CLUT },
	{ BECORE_YUVP_CLUT_1DLUT_Y_FIRST,
	  BECORE_YUVP_CLUT_1DLUT_Y_FIRST + BECORE_CLUT_1DLUT_LAST,
	  BECORE_GEN_CLUT_1DLUT },
	{ BECORE_YUVP_CLUT_1DLUT_U_FIRST,
	  BECORE_YUVP_CLUT_1DLUT_U_FIRST + BECORE_CLUT_1DLUT_LAST,
	  BECORE_GEN_CLUT_1DLUT },
	{ BECORE_YUVP_CLUT_1DLUT_V_FIRST,
	  BECORE_YUVP_CLUT_1DLUT_V_FIRST + BECORE_CLUT_1DLUT_LAST,
	  BECORE_GEN_CLUT_1DLUT },
};

static const struct becore_generated_range becore_mcsc_generated[] = {
	{ BECORE_MCSC_OTF_GATE_FIRST, BECORE_MCSC_OTF_GATE_LAST,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_INPUT_TYPE_REG, BECORE_MCSC_INPUT_TYPE_REG,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_IN_WIDTH_REG, BECORE_MCSC_IN_HEIGHT_REG,
	  BECORE_GEN_MCSC_INPUT_SIZE },
	{ BECORE_MCSC_CINFIFO_FIRST, BECORE_MCSC_CINFIFO_LAST,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_STAT_RDMA_FIRST, BECORE_MCSC_STAT_RDMA_LAST,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_WDMA_W1_FIRST,
	  BECORE_MCSC_WDMA_W1_FIRST + BECORE_MCSC_WDMA_OFF_LAST,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_WDMA_W2_FIRST,
	  BECORE_MCSC_WDMA_W2_FIRST + BECORE_MCSC_WDMA_OFF_LAST,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_WDMA_W3_FIRST,
	  BECORE_MCSC_WDMA_W3_FIRST + BECORE_MCSC_WDMA_OFF_LAST,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_WDMA_W4_FIRST,
	  BECORE_MCSC_WDMA_W4_FIRST + BECORE_MCSC_WDMA_OFF_LAST,
	  BECORE_GEN_OFF },
	{ BECORE_MCSC_SC0_PHASE_FIRST,
	  BECORE_MCSC_SC0_PHASE_FIRST + BECORE_SCALER_PHASE_LAST,
	  BECORE_GEN_SCALER_PHASE },
	{ BECORE_MCSC_PC0_PHASE_FIRST,
	  BECORE_MCSC_PC0_PHASE_FIRST + BECORE_SCALER_PHASE_LAST,
	  BECORE_GEN_SCALER_PHASE },
	{ BECORE_MCSC_DJAG_CTRL_REG, BECORE_MCSC_DJAG_CTRL_REG,
	  BECORE_GEN_DJAG },
	{ BECORE_MCSC_DJAG_PS_FIRST, BECORE_MCSC_DJAG_PS_LAST,
	  BECORE_GEN_DJAG },
	{ BECORE_MCSC_DJAG_TUNE_FIRST, BECORE_MCSC_DJAG_TUNE_LAST,
	  BECORE_GEN_DJAG },
	{ BECORE_MCSC_DJAG_RECOM_CTRL_REG, BECORE_MCSC_DJAG_RECOM_CTRL_REG,
	  BECORE_GEN_DJAG },
	{ BECORE_MCSC_DJAG_RECOM_WEIGHT_REG, BECORE_MCSC_DJAG_RECOM_WEIGHT_REG,
	  BECORE_GEN_DJAG },
	{ BECORE_MCSC_SC0_SRC_POS_REG, BECORE_MCSC_SC0_SRC_POS_REG,
	  BECORE_GEN_CHAIN_ORIGIN },
	{ BECORE_MCSC_SC0_SRC_SIZE_REG, BECORE_MCSC_SC0_DST_SIZE_REG,
	  BECORE_GEN_CHAIN_SIZE },
	{ BECORE_MCSC_SC0_H_RATIO_REG, BECORE_MCSC_SC0_V_RATIO_REG,
	  BECORE_GEN_CHAIN_RATIO },
	{ BECORE_MCSC_PC0_IMG_SIZE_REG, BECORE_MCSC_PC0_DST_SIZE_REG,
	  BECORE_GEN_CHAIN_SIZE },
	{ BECORE_MCSC_PC0_H_RATIO_REG, BECORE_MCSC_PC0_V_RATIO_REG,
	  BECORE_GEN_CHAIN_RATIO },
	{ BECORE_MCSC_SC0_V_COEFF_FIRST, BECORE_MCSC_SC0_V_COEFF_LAST,
	  BECORE_GEN_SC_V_COEFF },
	{ BECORE_MCSC_SC0_H_COEFF_FIRST, BECORE_MCSC_SC0_H_COEFF_LAST,
	  BECORE_GEN_SC_H_COEFF },
};

static const u32 becore_rgbp_input_regs[] = {
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

struct becore_yuvp_output_profile {
	u32 width;
	u32 height;
	u32 data_format;
	u32 mode;
	u32 lossy_byte32num;
	u32 votf_enable;
	u32 bytes_per_pixel;
	u32 block_height;
	u32 luma_height_align;
	u32 plane_gap;
	u32 businfo;
};

enum becore_yuvp_output_profile_id {
	BECORE_YUVP_OUTPUT_SBWCL,
	BECORE_YUVP_OUTPUT_P010,
	BECORE_YUVP_OUTPUT_PROFILE_COUNT,
};

/* Pixel uses a combined, two-plane WDMA where Pablo v1.1 uses split blocks. */
static const struct becore_yuvp_output_profile becore_yuvp_outputs[] = {
	[BECORE_YUVP_OUTPUT_SBWCL] = {
		.width = 4160,
		.height = 3120,
		.data_format = 0x2000,
		.mode = 0xa,
		.lossy_byte32num = 2,
		.votf_enable = 3,
		.bytes_per_pixel = 2,
		.block_height = 4,
		.luma_height_align = 16,
		.plane_gap = 0x40,
		.businfo = 0,
	},
	[BECORE_YUVP_OUTPUT_P010] = {
		.width = 4160,
		.height = 3120,
		.data_format = 0x2000,
		.mode = 0,
		.lossy_byte32num = 0,
		.votf_enable = 0,
		.bytes_per_pixel = 2,
		.businfo = 0,
	},
};

enum becore_yuvp_output_word {
	BECORE_YUVP_OUTPUT_VOTF,
	BECORE_YUVP_OUTPUT_FORMAT,
	BECORE_YUVP_OUTPUT_LOSSY,
	BECORE_YUVP_OUTPUT_MODE,
	BECORE_YUVP_OUTPUT_WIDTH,
	BECORE_YUVP_OUTPUT_HEIGHT,
	BECORE_YUVP_OUTPUT_STRIDE1,
	BECORE_YUVP_OUTPUT_STRIDE2,
	BECORE_YUVP_OUTPUT_BUSINFO,
	BECORE_YUVP_OUTPUT_ENABLE,
	BECORE_YUVP_OUTPUT_WORD_COUNT,
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

struct becore_gtnr_dma_profile {
	u32 width;
	u32 height;
	u32 data_format;
	u32 comp_control;
	u32 lossy_byte32num;
	u32 votf_enable;
	u32 stride;
	u32 businfo;
	u32 max_mo;
	u32 max_bl;
	u32 enable;
};

/*
 * The first physical-ultrawide GTNR frame has no temporal inputs or map DMAs.
 * It reads YUVP's 4160x3120 lossy-SBWC surface and writes a separate surface
 * with the same bounded layout.  Keep this startup shape dormant until GTNR's
 * power, reset, interrupt, and cross-block completion lifecycle is established.
 */
static const struct becore_gtnr_dma_profile becore_gtnr_input = {
	.width = 4160,
	.height = 3120,
	.data_format = 0x2000,
	.comp_control = 0xa,
	.lossy_byte32num = 2,
	.votf_enable = 1,
	.stride = 0x2080,
	.businfo = 1,
	.max_mo = 0x100,
	.max_bl = 0x10,
	.enable = 1,
};

static const struct becore_gtnr_dma_profile becore_gtnr_output = {
	.width = 4160,
	.height = 3120,
	.data_format = 0x2000,
	.comp_control = 0xa,
	.lossy_byte32num = 2,
	.stride = 0x2080,
	.businfo = 0,
	.max_mo = 0x100,
	.max_bl = 0x10,
	.enable = 1,
};

enum becore_gtnr_dma_word {
	BECORE_GTNR_INPUT_VOTF,
	BECORE_GTNR_INPUT_FORMAT,
	BECORE_GTNR_INPUT_LOSSY,
	BECORE_GTNR_INPUT_COMP,
	BECORE_GTNR_INPUT_WIDTH,
	BECORE_GTNR_INPUT_HEIGHT,
	BECORE_GTNR_INPUT_STRIDE1,
	BECORE_GTNR_INPUT_STRIDE2,
	BECORE_GTNR_INPUT_BUSINFO,
	BECORE_GTNR_INPUT_MAX_MO,
	BECORE_GTNR_INPUT_MAX_BL,
	BECORE_GTNR_INPUT_ENABLE,
	BECORE_GTNR_OUTPUT_FORMAT,
	BECORE_GTNR_OUTPUT_LOSSY,
	BECORE_GTNR_OUTPUT_COMP,
	BECORE_GTNR_OUTPUT_WIDTH,
	BECORE_GTNR_OUTPUT_HEIGHT,
	BECORE_GTNR_OUTPUT_STRIDE1,
	BECORE_GTNR_OUTPUT_STRIDE2,
	BECORE_GTNR_OUTPUT_BUSINFO,
	BECORE_GTNR_OUTPUT_MAX_MO,
	BECORE_GTNR_OUTPUT_MAX_BL,
	BECORE_GTNR_OUTPUT_ENABLE,
	BECORE_GTNR_DMA_WORD_COUNT,
};

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

struct becore_mcsc_dma_profile {
	u32 width;
	u32 height;
	u32 data_format;
	u32 comp_control;
	u32 lossy_byte32num;
	u32 votf_enable;
	u32 stride;
	u32 businfo;
	u32 max_bl;
	u32 enable;
	u32 dither;
};

/*
 * The physical-ultrawide 4000x3000 request carries YUVP's 4160x3120 lossy
 * SBWC intermediate into MCSC and writes output zero as linear NV21.  The
 * captured program used VOTF; a sequential run replaces only that transport
 * control so MCSC reads the completed driver-owned surface from memory.
 */
static const struct becore_mcsc_dma_profile becore_mcsc_input = {
	.width = 4160,
	.height = 3120,
	.data_format = 0x2000,
	.comp_control = 0xa,
	.lossy_byte32num = 2,
	.votf_enable = 0x00400001,
	.stride = 0x2080,
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
	.width = 4000,
	.height = 3000,
	.data_format = 0x800,
	.comp_control = 0,
	.stride = 0xfc0,
	.businfo = 0,
	.max_bl = 4,
	.enable = 1,
	.dither = 0x10,
};

/*
 * How much of DJAG's input the output is taken from. The captured program
 * crops 3536 x 2652 out of the 4160 x 3120 YUVP surface and scales that up to
 * fill 4000 x 3000, reserving a margin the vendor spends on electronic
 * stabilisation. This driver has no stabilisation to spend it on, so it takes
 * the whole raster and downscales instead, covering the full field. The
 * window is centred, so only its size is a parameter.
 */
struct becore_mcsc_djag_profile {
	u32 crop_width;
	u32 crop_height;
};

static const struct becore_mcsc_djag_profile becore_mcsc_djag = {
	.crop_width = 4160,
	.crop_height = 3120,
};

enum becore_mcsc_input_transport {
	BECORE_MCSC_INPUT_CAPTURED_VOTF,
	BECORE_MCSC_INPUT_MEMORY,
};

enum becore_mcsc_dma_word {
	BECORE_MCSC_INPUT_VOTF,
	BECORE_MCSC_INPUT_FORMAT,
	BECORE_MCSC_INPUT_LOSSY,
	BECORE_MCSC_INPUT_COMP,
	BECORE_MCSC_INPUT_WIDTH,
	BECORE_MCSC_INPUT_HEIGHT,
	BECORE_MCSC_INPUT_STRIDE1,
	BECORE_MCSC_INPUT_STRIDE2,
	BECORE_MCSC_INPUT_BUSINFO,
	BECORE_MCSC_INPUT_MAX_BL,
	BECORE_MCSC_INPUT_ENABLE,
	BECORE_MCSC_DJAG_IMG_SIZE,
	BECORE_MCSC_DJAG_PS_SRC_POS,
	BECORE_MCSC_DJAG_PS_SRC_SIZE,
	BECORE_MCSC_DJAG_PS_DST_SIZE,
	BECORE_MCSC_DJAG_PS_H_RATIO,
	BECORE_MCSC_DJAG_PS_V_RATIO,
	BECORE_MCSC_OUTPUT_FORMAT,
	BECORE_MCSC_OUTPUT_COMP,
	BECORE_MCSC_OUTPUT_WIDTH,
	BECORE_MCSC_OUTPUT_HEIGHT,
	BECORE_MCSC_OUTPUT_STRIDE1,
	BECORE_MCSC_OUTPUT_STRIDE2,
	BECORE_MCSC_OUTPUT_BUSINFO,
	BECORE_MCSC_OUTPUT_MAX_BL,
	BECORE_MCSC_OUTPUT_ENABLE,
	BECORE_MCSC_OUTPUT_DITHER,
	BECORE_MCSC_DMA_WORD_COUNT,
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

struct becore_device;

#define BECORE_INPUT_SLOT_COUNT	3

struct becore_dma_buffer {
	void *cpu;
	dma_addr_t dma;
	size_t size;
	size_t staged_bytes;
	struct sg_table *sgt;
};

struct becore_ltm_gain_offset_group {
	__le16 gain[4];
	__le16 offset[4];
};

#define BECORE_LTM_GRID_GROUPS_PER_CELL \
	(BECORE_LTM_GRID_CELL_BYTES / \
	 sizeof(struct becore_ltm_gain_offset_group))

struct becore_ltm_grid_cell {
	struct becore_ltm_gain_offset_group
		groups[BECORE_LTM_GRID_GROUPS_PER_CELL];
};

enum becore_input_slot_state {
	BECORE_INPUT_FREE,
	BECORE_INPUT_PRODUCER,
	BECORE_INPUT_READY,
	BECORE_INPUT_BACKEND,
	BECORE_INPUT_QUARANTINED,
};

/*
 * A slot's pages are written by ISPFE's DMA and read by RGBP's, and both are
 * non-coherent masters that go to DRAM: the CPU is not part of that exchange,
 * so it needs no cache maintenance between them.  The one thing that does is
 * the debugfs staging path, which writes a slot through the cached vmap
 * dma_vmap_noncontiguous() returns.  cpu_dirty says a slot is in that state
 * and is the only reason a sync is issued.
 */
struct becore_input_slot {
	struct becore_dma_buffer buffer;
	enum becore_input_slot_state state;
	bool cpu_dirty;
	u64 producer_cookie;
	u64 ready_sequence;
};

/*
 * A CMDQ program's allocation and its contents are two different lengths. The
 * allocation is sized once, for the longest program the block can be asked to
 * run; the header count is what the last encode emitted, and it is what
 * CMDQ_QUE_CMD_M tells the hardware to execute.
 *
 * The payload area starts after the header list, so the two lengths also give
 * two different layouts -- which is why the debugfs readback returns the
 * encoded length rather than the allocation. A reader handed the allocation
 * would look for the payloads in the wrong place.
 */
struct becore_cmdq_program {
	void *cpu;
	dma_addr_t dma;
	size_t size;
	u32 capacity;
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
	u32 stream_crc_armed[BECORE_STREAM_CRC_MAX];
	u32 stream_crc_result[BECORE_STREAM_CRC_MAX];
};

struct becore_irq {
	struct becore_block *block;
	int irq;
	bool int1;
};

struct becore_video_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

/*
 * What userspace has most recently asked for, in the units the blocks are
 * specified in rather than as register words.  A frame carries only what
 * changed, so a value persists until a later buffer replaces it, and a value
 * that was never sent leaves the driver's own default in place.
 */
struct becore_params_state {
	s16 ccm[EXYNOS_BECORE_CCM_COEFFICIENTS];
	s16 ccm_offsets[EXYNOS_BECORE_CCM_OFFSETS];
	u16 ltm_curve[EXYNOS_BECORE_LTM_CURVE_POINTS];
	u16 clut_u[EXYNOS_BECORE_CLUT_NODES];
	u16 clut_v[EXYNOS_BECORE_CLUT_NODES];
	bool ccm_valid;
	bool ltm_curve_valid;
	bool clut_valid;
};

struct becore_params_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	/*
	 * A kernel copy taken at buf_prepare: validation is worthless against
	 * memory userspace can still write to after it has been checked.
	 */
	struct v4l2_isp_params_buffer *config;
};

static inline struct becore_params_buffer *
to_becore_params_buffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct becore_params_buffer, vb);
}

static inline struct becore_video_buffer *
to_becore_video_buffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct becore_video_buffer, vb);
}

/*
 * Where a run's wall clock goes.  The phases tile the whole of a run without
 * overlapping, so they sum to BECORE_TIMING_FRAME: each mark is taken once, at
 * the boundary between two phases, and hands its timestamp to the next.
 */
enum becore_timing_phase {
	BECORE_TIMING_RESUME,
	BECORE_TIMING_ENCODE,
	BECORE_TIMING_ARM,
	BECORE_TIMING_STAGE1,
	BECORE_TIMING_STAGE2,
	BECORE_TIMING_CRC,
	BECORE_TIMING_SUSPEND,
	BECORE_TIMING_FRAME,
	BECORE_TIMING_PHASE_COUNT,
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

struct becore_timing {
	u64 last_ns;
	u64 total_ns;
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

/* One producer endpoint, as the hardware reads it back. */
struct becore_c2serv_tws_state {
	u32 conn;
	u32 conn_raw;
	u32 rcv_valid;
	u32 enable;
	u32 limit;
	u32 dest;
	u32 lines_in_token;
	u32 busy;
	u32 fullness;
};

/* One consumer endpoint, as the hardware reads it back. */
struct becore_c2serv_trs_state {
	u32 conn;
	u32 conn_raw;
	u32 rcv_valid;
	u32 enable;
	u32 limit;
	u32 lines_in_first_token;
	u32 lines_in_token;
	u32 lines_count;
	u32 busy;
	u32 lost_connection;
};

/*
 * What a window said after being made ready.  Only `ready` is a verdict: it
 * comes from the software reset clearing itself, which is the one C2SERV read
 * the vendor's own stream makes.  The rest are read because they are cheap
 * and say something, not because anything is known about how they read back
 * -- and the two ring bits should read zero, because this driver does not
 * start the ring.
 *
 * `reset_enables` is the endpoint enable mask as it stands after the software
 * reset and before any endpoint is written: bit n for producer n, bit 16 + n
 * for consumer n.  It is sampled after SEL_REGISTER rather than before,
 * because until the immediate bank is selected a read answers for the shadow
 * alias instead.  Samsung's field table says both enables reset to 1, which
 * would mean a freshly reset window has every endpoint live; the whole safety
 * of preparing a window rests on that, so it is measured here rather than
 * assumed.
 */
struct becore_c2serv_state {
	bool ready;
	u32 ring_clk_en;
	u32 ring_enable;
	u32 local_ip;
	u32 reset_enables;
};

/*
 * The link's own registers, kept apart from the window state on purpose: a
 * one-shot suspends as soon as it finishes, and tearing the windows down
 * would otherwise erase the sample taken from a run that had just failed --
 * which is the only thing that says why.  Nothing clears this but the next
 * sample.
 */
struct becore_c2serv_link_state {
	bool sampled;
	struct becore_c2serv_tws_state tws[BECORE_C2SERV_LINK_PLANES];
	struct becore_c2serv_trs_state trs[BECORE_C2SERV_LINK_PLANES];
};

struct becore_device {
	struct device *dev;
	struct clk *intcam_clk;
	unsigned long saved_intcam_rate;
	bool intcam_rate_active;
	struct becore_block blocks[BECORE_NUM_BLOCKS];
	struct becore_irq irqs[BECORE_NUM_BLOCKS * 2];
	void __iomem *ssmt[14];
	void __iomem *sysreg_rgbp;
	void __iomem *sysreg_mcsc;
	void __iomem *c2serv[BECORE_NUM_C2SERV];
	struct becore_c2serv_state c2serv_state[BECORE_NUM_C2SERV];
	/* The link as armed, and as it stood when the run finished. */
	struct becore_c2serv_link_state c2serv_armed;
	struct becore_c2serv_link_state c2serv_done;
	struct dev_pm_domain_list *pm_domains;
	struct dentry *debugfs;
	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *red_balance;
	struct v4l2_ctrl *blue_balance;
	struct video_device vdev;
	struct media_pad vdev_pad;
	struct vb2_queue queue;
	struct video_device params_vdev;
	struct media_pad params_pad;
	struct vb2_queue params_queue;
	/* Serializes V4L2 ioctls and vb2 queue setup on the parameters node. */
	struct mutex params_lock;
	struct list_head queued_params;
	struct work_struct params_work;
	struct becore_params_state params;
	/* Serializes V4L2 ioctls and vb2 queue setup/teardown. */
	struct mutex video_lock;
	/* Protects the pending processed-output and parameters buffer lists. */
	spinlock_t queue_lock;
	struct list_head queued_outputs;
	struct work_struct video_work;
	/* Serializes staging, input ownership, execution, and output inspection. */
	struct mutex lock;
	/* Protects IRQ-driven command-hold/frame completion state. */
	spinlock_t run_lock;
	struct completion run_completion;
	struct becore_input_slot inputs[BECORE_INPUT_SLOT_COUNT];
	struct becore_input_slot *run_input;
	struct becore_dma_buffer grid;
	struct becore_dma_buffer output;
	struct becore_dma_buffer gtnr_output;
	struct becore_dma_buffer mcsc_output;
	struct exynos_becore_input *input_producer;
	u64 producer_sequence;
	u64 input_sequence;
	struct becore_cmdq_program program[BECORE_NUM_BLOCKS];
	struct becore_cmdq_program gtnr_program;
	struct becore_cmdq_program mcsc_program;
	u8 *recipe;
	u8 *gtnr_recipe;
	u8 *mcsc_recipe;
	size_t recipe_staged_bytes;
	size_t gtnr_recipe_staged_bytes;
	size_t mcsc_recipe_staged_bytes;
	u32 recipe_generation;
	u32 gtnr_recipe_generation;
	u32 gtnr_encoded_generation;
	u32 mcsc_recipe_generation;
	u32 mcsc_encoded_generation;
	u32 grid_generation;
	u32 run_generation;
	u32 completed_generation;
	u32 video_sequence;
	u32 params_sequence;
	u32 cmdq_hold_mask;
	u32 frame_done_mask;
	u32 expected_mask;
	u32 output_changed_bytes;
	u32 output_first_changed;
	u32 mcsc_output_changed_bytes;
	u32 mcsc_output_first_changed;
	u32 stream_crc_seed;
	u32 stream_crc_armed_seed;
	u32 stream_crc_generation;
	struct becore_override overrides[BECORE_OVERRIDE_MAX];
	u32 override_count;
	char override_text[BECORE_OVERRIDE_TEXT_MAX];
	size_t override_text_len;
	/* Compared, never dereferenced: which descriptor owns the text. */
	const struct file *override_writer;
	u32 input_profile;
	u32 active_input_profile;
	u32 output_profile;
	u32 active_output_profile;
	/*
	 * Carry YUVP into MCSC over the fabric rather than through DRAM.  On
	 * by default; clearing it puts the frame back through memory, which
	 * is a slower path that is otherwise identical and is worth keeping
	 * as the thing to compare a result against.
	 */
	u32 votf;
	/* Debug geometry: zero means the captured value, per plane. */
	u32 votf_tws_limit;
	u32 votf_trs_limit;
	u32 votf_tws_token[BECORE_C2SERV_LINK_PLANES];
	u32 votf_trs_token[BECORE_C2SERV_LINK_PLANES];
	bool active_votf;
	u32 mcsc_completed_generation;
	u32 mcsc_completed_output_size;
	enum becore_mcsc_input_transport mcsc_encoded_transport;
	dma_addr_t active_output_dma;
	size_t active_output_size;
	size_t active_capture_size;
	size_t completed_output_size;
	int last_run_result;
	bool running;
	bool start_issued;
	bool abort_run;
	bool irq_error;
	bool irqs_enabled;
	bool reset_failed;
	bool active_output_packed;
	bool active_mcsc;
	bool output_quarantined;
	bool video_streaming;
	bool producer_streaming;
	bool stream_powered;
	/*
	 * Where MCSC writes this run.  The driver-owned buffer except during a
	 * capture, which redirects it at the queued vb2 buffer; every path that
	 * encodes MCSC sets it first rather than inheriting the last run's.
	 */
	dma_addr_t mcsc_dest_dma;
	/* Written only by a completed run, read under the same lock. */
	struct becore_timing timing[BECORE_TIMING_PHASE_COUNT];
	u32 timing_runs;
};

struct exynos_becore_input {
	struct becore_device *becore;
	struct device *producer;
	const struct exynos_becore_input_producer_ops *ops;
	void *producer_data;
	refcount_t callback_users;
	wait_queue_head_t callback_wait;
	bool disconnected;
	struct sg_table sgts[BECORE_INPUT_SLOT_COUNT];
	dma_addr_t dmas[BECORE_INPUT_SLOT_COUNT];
};

static void becore_video_return_all(struct becore_device *becore,
				    enum vb2_buffer_state state);
static void becore_params_drain_idle(struct becore_device *becore);
static void becore_video_controls_ungrab(struct becore_device *becore);
static void becore_params_consume(struct becore_device *becore);

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
becore_rgbp_input_storage_width(const struct becore_rgbp_input_profile *profile)
{
	return ALIGN(profile->width, profile->sbwc_block_width);
}

static u32
becore_rgbp_input_stride(const struct becore_rgbp_input_profile *profile)
{
	return becore_rgbp_input_storage_width(profile) *
	       profile->bytes_per_pixel;
}

static size_t
becore_rgbp_input_image_offset(const struct becore_rgbp_input_profile *profile)
{
	return (size_t)profile->header_stride * profile->height;
}

static size_t
becore_rgbp_input_size(const struct becore_rgbp_input_profile *profile)
{
	size_t image_bytes = (size_t)becore_rgbp_input_stride(profile) *
			     profile->height;

	return ALIGN(becore_rgbp_input_image_offset(profile) + image_bytes,
		     SZ_4K);
}

/*
 * Every input slot is sized for the live path, because that is what fills
 * them: ISPFE writes SBWC-compressed main Bayer straight into a slot, and its
 * own probe checks the size it is handed.  A harness profile therefore has to
 * fit inside that allocation rather than resize it, which is what
 * becore_input_profiles_validate() checks once at probe.
 */
static size_t becore_input_allocation_size(void)
{
	return becore_rgbp_input_size(&becore_rgbp_inputs[BECORE_RGBP_INPUT_SBWC]);
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

static int becore_input_profiles_validate(struct device *dev)
{
	unsigned int i;

	for (i = 0; i < BECORE_RGBP_INPUT_PROFILE_COUNT; i++) {
		const struct becore_rgbp_input_profile *profile =
			&becore_rgbp_inputs[i];

		if (!profile->width || !profile->height ||
		    !profile->bytes_per_pixel ||
		    !is_power_of_2(profile->sbwc_block_width))
			return dev_err_probe(dev, -EINVAL,
					     "input profile %u is degenerate\n",
					     i);
		if (becore_rgbp_input_size(profile) >
		    becore_input_allocation_size())
			return dev_err_probe(dev, -EINVAL,
					     "input profile %u wants %zu bytes, the slot holds %zu\n",
					     i, becore_rgbp_input_size(profile),
					     becore_input_allocation_size());
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
static u32 becore_pack_size(u32 high, u32 low)
{
	return (high << 16) | low;
}

static u32 becore_zoom_ratio(u32 in, u32 out)
{
	if (!out)
		return 0;

	return (u32)div_u64((u64)in << BECORE_RATIO_SHIFT, out);
}

/*
 * What RGBP hands downstream. It is the geometry YUVP then carries, so it is
 * taken from the YUVP profile rather than repeated here -- if the two ever
 * disagreed the chain would be describing two different images. Both YUVP
 * output profiles share it deliberately: this is the chain's geometry, not a
 * property of how the surface is encoded, so the diagnostic P010 selector must
 * not move it.
 */
static u32 becore_rgbp_out_width(void)
{
	return becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL].width;
}

static u32 becore_rgbp_out_height(void)
{
	return becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL].height;
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
static int becore_rgbp_crop(const struct becore_rgbp_input_profile *profile,
			    struct becore_rect *crop)
{
	u32 array_w = profile->width;
	u32 array_h = profile->height;
	u32 out_w = becore_rgbp_out_width();
	u32 out_h = becore_rgbp_out_height();
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
static int becore_rgbp_input_value(const struct becore_rgbp_input_profile *profile,
				   u32 index, u32 reg, u32 *value)
{
	u32 result;

	if (index >= BECORE_RGBP_INPUT_WORD_COUNT ||
	    reg != becore_rgbp_input_regs[index])
		return -EINVAL;

	switch (index) {
	case BECORE_RGBP_CHAIN_SRC_SIZE:
		result = becore_pack_size(profile->width, profile->height);
		break;
	case BECORE_RGBP_CHAIN_DST_SIZE:
	case BECORE_RGBP_SC_DST_SIZE:
		result = becore_pack_size(becore_rgbp_out_width(),
					  becore_rgbp_out_height());
		break;
	case BECORE_RGBP_CROP_SIZE:
	case BECORE_RGBP_CROP_START:
	case BECORE_RGBP_SC_H_RATIO:
	case BECORE_RGBP_SC_V_RATIO: {
		struct becore_rect crop;
		int ret = becore_rgbp_crop(profile, &crop);

		if (ret)
			return ret;
		if (index == BECORE_RGBP_CROP_SIZE)
			result = becore_pack_size(crop.width, crop.height);
		else if (index == BECORE_RGBP_CROP_START)
			result = becore_pack_size(crop.x, crop.y);
		else if (index == BECORE_RGBP_SC_H_RATIO)
			result = becore_zoom_ratio(crop.width,
						   becore_rgbp_out_width());
		else
			result = becore_zoom_ratio(crop.height,
						   becore_rgbp_out_height());
		break;
	}
	case BECORE_RGBP_INPUT_FORMAT:
		result = profile->data_format;
		break;
	case BECORE_RGBP_INPUT_COMP:
		result = profile->comp_control;
		break;
	case BECORE_RGBP_INPUT_ACTIVE_WIDTH:
		result = profile->width;
		break;
	case BECORE_RGBP_INPUT_HEIGHT:
		result = profile->height;
		break;
	case BECORE_RGBP_INPUT_STRIDE:
		result = becore_rgbp_input_stride(profile);
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
		result = becore_rgbp_input_storage_width(profile);
		break;
	default:
		return -EINVAL;
	}

	if (value)
		*value = result;

	return 0;
}

static const struct becore_rgbp_input_profile *
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
becore_yuvp_output_stride(const struct becore_yuvp_output_profile *profile)
{
	return profile->width * profile->bytes_per_pixel;
}

static size_t
becore_yuvp_output_plane2_offset(const struct becore_yuvp_output_profile *profile)
{
	u32 stride = becore_yuvp_output_stride(profile);
	u32 luma_height;
	u32 luma_rows;

	if (!profile->mode)
		return ALIGN((size_t)stride * profile->height, SZ_4K);

	luma_height = ALIGN(profile->height, profile->luma_height_align);
	luma_rows = DIV_ROUND_UP(luma_height, profile->block_height);

	return (size_t)stride * luma_rows + profile->plane_gap;
}

static size_t
becore_yuvp_output_size(const struct becore_yuvp_output_profile *profile)
{
	u32 stride = becore_yuvp_output_stride(profile);
	u32 chroma_height = DIV_ROUND_UP(profile->height, 2);
	u32 chroma_rows;
	size_t chroma_bytes;

	if (profile->mode)
		chroma_rows = DIV_ROUND_UP(chroma_height, profile->block_height);
	else
		chroma_rows = chroma_height;
	chroma_bytes = (size_t)stride * chroma_rows;

	return ALIGN(becore_yuvp_output_plane2_offset(profile) + chroma_bytes,
		     SZ_4K);
}

static size_t
becore_yuvp_packed_plane2_offset(const struct becore_yuvp_output_profile *profile)
{
	return (size_t)becore_yuvp_output_stride(profile) * profile->height;
}

static size_t
becore_yuvp_packed_output_size(const struct becore_yuvp_output_profile *profile)
{
	u32 chroma_height = DIV_ROUND_UP(profile->height, 2);

	return becore_yuvp_packed_plane2_offset(profile) +
	       (size_t)becore_yuvp_output_stride(profile) * chroma_height;
}

static size_t becore_active_output_plane2_offset(const struct becore_device *becore)
{
	const struct becore_yuvp_output_profile *profile =
		becore_yuvp_output_profile(becore);

	if (becore->active_output_packed)
		return becore_yuvp_packed_plane2_offset(profile);

	return becore_yuvp_output_plane2_offset(profile);
}

static size_t becore_active_output_size(const struct becore_device *becore)
{
	const struct becore_yuvp_output_profile *profile =
		becore_yuvp_output_profile(becore);

	if (becore->active_output_packed)
		return becore_yuvp_packed_output_size(profile);

	return becore_yuvp_output_size(profile);
}

static size_t becore_yuvp_output_allocation_size(void)
{
	const struct becore_yuvp_output_profile *sbwcl =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];
	const struct becore_yuvp_output_profile *p010 =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_P010];

	return max(becore_yuvp_output_size(sbwcl),
		   becore_yuvp_output_size(p010));
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
		*value = profile->width;
		break;
	case BECORE_YUVP_OUTPUT_HEIGHT:
		*value = profile->height;
		break;
	case BECORE_YUVP_OUTPUT_STRIDE1:
	case BECORE_YUVP_OUTPUT_STRIDE2:
		*value = becore_yuvp_output_stride(profile);
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

static int becore_gtnr_dma_value(u32 index, u32 reg, u32 *value)
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
		*value = becore_gtnr_input.data_format;
		break;
	case BECORE_GTNR_INPUT_LOSSY:
		*value = becore_gtnr_input.lossy_byte32num;
		break;
	case BECORE_GTNR_INPUT_COMP:
		*value = becore_gtnr_input.comp_control;
		break;
	case BECORE_GTNR_INPUT_WIDTH:
		*value = becore_gtnr_input.width;
		break;
	case BECORE_GTNR_INPUT_HEIGHT:
		*value = becore_gtnr_input.height;
		break;
	case BECORE_GTNR_INPUT_STRIDE1:
	case BECORE_GTNR_INPUT_STRIDE2:
		*value = becore_gtnr_input.stride;
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
		*value = becore_gtnr_output.data_format;
		break;
	case BECORE_GTNR_OUTPUT_LOSSY:
		*value = becore_gtnr_output.lossy_byte32num;
		break;
	case BECORE_GTNR_OUTPUT_COMP:
		*value = becore_gtnr_output.comp_control;
		break;
	case BECORE_GTNR_OUTPUT_WIDTH:
		*value = becore_gtnr_output.width;
		break;
	case BECORE_GTNR_OUTPUT_HEIGHT:
		*value = becore_gtnr_output.height;
		break;
	case BECORE_GTNR_OUTPUT_STRIDE1:
	case BECORE_GTNR_OUTPUT_STRIDE2:
		*value = becore_gtnr_output.stride;
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

static size_t becore_mcsc_output_plane2_offset(void)
{
	return (size_t)becore_mcsc_output.stride * becore_mcsc_output.height;
}

static size_t becore_mcsc_output_active_size(void)
{
	size_t chroma = (size_t)becore_mcsc_output.stride *
			DIV_ROUND_UP(becore_mcsc_output.height, 2);

	return becore_mcsc_output_plane2_offset() + chroma;
}

static size_t becore_mcsc_output_size(void)
{
	return ALIGN(becore_mcsc_output_active_size(), SZ_4K);
}

/*
 * The crop is centred in the scaler's input, so its origin is derived rather
 * than carried. An odd margin would land the window off a chroma boundary on a
 * 4:2:0 output, so refuse it instead of silently rounding.
 */
static int becore_mcsc_djag_origin(u32 *x, u32 *y)
{
	if (becore_mcsc_djag.crop_width > becore_mcsc_input.width ||
	    becore_mcsc_djag.crop_height > becore_mcsc_input.height)
		return -ERANGE;
	*x = (becore_mcsc_input.width - becore_mcsc_djag.crop_width) / 2;
	*y = (becore_mcsc_input.height - becore_mcsc_djag.crop_height) / 2;
	if ((*x | *y) & 1)
		return -ERANGE;

	return 0;
}

static int
becore_mcsc_dma_value(u32 index, u32 reg,
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
		*value = becore_mcsc_input.data_format;
		break;
	case BECORE_MCSC_INPUT_LOSSY:
		*value = becore_mcsc_input.lossy_byte32num;
		break;
	case BECORE_MCSC_INPUT_COMP:
		*value = becore_mcsc_input.comp_control;
		break;
	case BECORE_MCSC_INPUT_WIDTH:
		*value = becore_mcsc_input.width;
		break;
	case BECORE_MCSC_INPUT_HEIGHT:
		*value = becore_mcsc_input.height;
		break;
	case BECORE_MCSC_INPUT_STRIDE1:
	case BECORE_MCSC_INPUT_STRIDE2:
		*value = becore_mcsc_input.stride;
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
		*value = becore_mcsc_output.width;
		break;
	case BECORE_MCSC_OUTPUT_HEIGHT:
		*value = becore_mcsc_output.height;
		break;
	case BECORE_MCSC_OUTPUT_STRIDE1:
	case BECORE_MCSC_OUTPUT_STRIDE2:
		*value = becore_mcsc_output.stride;
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
		*value = becore_pack_size(becore_mcsc_input.width,
						 becore_mcsc_input.height);
		break;
	case BECORE_MCSC_DJAG_PS_SRC_POS: {
		u32 x, y;
		int ret = becore_mcsc_djag_origin(&x, &y);

		if (ret)
			return ret;
		*value = becore_pack_size(x, y);
		break;
	}
	case BECORE_MCSC_DJAG_PS_SRC_SIZE:
		*value = becore_pack_size(becore_mcsc_djag.crop_width,
						 becore_mcsc_djag.crop_height);
		break;
	case BECORE_MCSC_DJAG_PS_DST_SIZE:
		*value = becore_pack_size(becore_mcsc_output.width,
						 becore_mcsc_output.height);
		break;
	case BECORE_MCSC_DJAG_PS_H_RATIO:
		*value = becore_zoom_ratio(becore_mcsc_djag.crop_width,
						  becore_mcsc_output.width);
		break;
	case BECORE_MCSC_DJAG_PS_V_RATIO:
		*value = becore_zoom_ratio(becore_mcsc_djag.crop_height,
						  becore_mcsc_output.height);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static size_t becore_gtnr_surface_plane2_offset(void)
{
	const struct becore_yuvp_output_profile *profile =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];

	return becore_yuvp_output_plane2_offset(profile);
}

static size_t becore_gtnr_surface_size(void)
{
	const struct becore_yuvp_output_profile *profile =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];

	return becore_yuvp_output_size(profile);
}

static dma_addr_t becore_gtnr_address_dma(struct becore_device *becore, u32 reg)
{
	switch (reg) {
	case BECORE_GTNR_INPUT_PLANE1_REG:
		return becore->output.dma;
	case BECORE_GTNR_INPUT_PLANE2_REG:
		return becore->output.dma + becore_gtnr_surface_plane2_offset();
	case BECORE_GTNR_OUTPUT_PLANE1_REG:
		return becore->gtnr_output.dma;
	case BECORE_GTNR_OUTPUT_PLANE2_REG:
		return becore->gtnr_output.dma +
		       becore_gtnr_surface_plane2_offset();
	default:
		return DMA_MAPPING_ERROR;
	}
}

static dma_addr_t becore_mcsc_address_dma(struct becore_device *becore, u32 reg)
{
	const struct becore_yuvp_output_profile *input =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];

	switch (reg) {
	case BECORE_MCSC_INPUT_PLANE1_REG:
		return becore->output.dma;
	case BECORE_MCSC_INPUT_PLANE2_REG:
		return becore->output.dma + becore_yuvp_output_plane2_offset(input);
	case BECORE_MCSC_OUTPUT_PLANE1_REG:
		return becore->mcsc_dest_dma;
	case BECORE_MCSC_OUTPUT_PLANE2_REG:
		return becore->mcsc_dest_dma +
		       becore_mcsc_output_plane2_offset();
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
					       index, reg, value);
	if (id == BECORE_YUVP)
		return becore_yuvp_output_value(becore, index, reg, value);

	return -EINVAL;
}

/*
 * The register a value word programs. Pair-mode headers carry it beside the
 * value; sequential-mode ones step from the header's target.
 */
static int becore_shape_register(const struct becore_cmdq_shape *shape,
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

/*
 * The knot grid: 16 knots every 16 codes across the first 256, then 8 every
 * 32, 24 every 64 and 16 every 128. Each knot is the left edge of its segment
 * and the segments tile the 12-bit input exactly, so the last knot is 3968
 * rather than 4095. Finer where the eye is, which is the ordinary reason a
 * tone curve is sampled unevenly; the grid decides where the curve is
 * measured, not what it does.
 *
 * tools/camera-becore-recipe.py carries the same arithmetic and checks it
 * against the vendor capture, which is the only thing that checks it -- the
 * recipe holds zero for these words, so nothing here can be validated against
 * it. Change one copy and change the other.
 */
static const struct {
	u8 knots;
	u16 step;
} becore_rgbp_gtm_grid[] = {
	{ 16, 16 }, { 8, 32 }, { 24, 64 }, { 16, 128 },
};

static_assert(16 + 8 + 24 + 16 == BECORE_RGBP_GTM_KNOTS);
static_assert(16 * 16 + 8 * 32 + 24 * 64 + 16 * 128 == 1 << 12);

static int becore_rgbp_gtm_knot(u32 index, u32 *knot)
{
	u32 x = 0;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_rgbp_gtm_grid); i++) {
		if (index < becore_rgbp_gtm_grid[i].knots) {
			*knot = x + index * becore_rgbp_gtm_grid[i].step;
			return 0;
		}
		x += becore_rgbp_gtm_grid[i].knots *
		     becore_rgbp_gtm_grid[i].step;
		index -= becore_rgbp_gtm_grid[i].knots;
	}

	return -EINVAL;
}

/*
 * YUVP local tone mapping, by offset from BECORE_YUVP_LTM_BASE. What the block
 * is for, rather than what one scene wanted from it: it forms a guide luma
 * from RGB, looks that up in a tone curve and applies a spatial gain grid. The
 * curve and the CONFIG words are the tuning and stay in the recipe; the gate,
 * the luma weights, the grid the frame is divided into and the vendor's own
 * unity fills are all stateable.
 */
/*
 * How far the block steps through the grid per raster pixel, at Q16.
 *
 * The reciprocal of a cell size is *not* this: the two agree only where the
 * cells divide the raster exactly, and 32 columns over 2608 gives 804 where
 * 65536 / (2608 / 32) gives 809. The grid is 32 x 24 x 8 at every readout the
 * vendor was captured at, 16:9 ones included, so its cells are square only on
 * a 4:3 frame and the horizontal and vertical steps are independent. The
 * half-step is the floor of half the step.
 */
static int becore_ltm_grid_scale(u32 cells, u32 extent, u32 *scale)
{
	u32 step;

	if (!extent)
		return -EINVAL;
	step = (u32)DIV_ROUND_CLOSEST_ULL((u64)cells << 16, extent);
	if (step > U16_MAX)
		return -ERANGE;
	*scale = step;

	return 0;
}

static int becore_yuvp_ltm_value(u32 offset, u32 *value)
{
	u32 scale;
	int ret;

	if (offset & 3)
		return -EINVAL;

	switch (offset) {
	case 0x000:				/* LTM_ENABLE: the block runs */
		*value = 1;
		return 0;
	case 0x014:				/* LUMACALC_RGBY_COEFF_R */
		*value = BECORE_LTM_LUMA_Q12_R;
		return 0;
	case 0x018:				/* ..._COEFF_G */
		*value = BECORE_LTM_LUMA_Q12_G;
		return 0;
	case 0x01c:				/* ..._COEFF_B */
		*value = BECORE_LTM_LUMA_Q12_B;
		return 0;
	case 0x120:				/* SLCGRID_START_X_POS */
	case 0x124:				/* SLCGRID_START_Y_POS */
		*value = 0;
		return 0;
	case 0x128:				/* SLCGRID_GRID_DEPTH */
		*value = BECORE_LTM_SLCGRID_DEPTH;
		return 0;
	case 0x12c:				/* SLCGRID_GRID_WIDTH */
		*value = BECORE_LTM_SLCGRID_COLUMNS;
		return 0;
	case 0x130:				/* SLCGRID_GRID_HEIGHT */
		*value = BECORE_LTM_SLCGRID_ROWS;
		return 0;
	}

	if (offset >= 0x2dc && offset <= 0x4dc) {
		/* CRECON_SATCTRL_LUT: saturation control contributes nothing. */
		*value = 0;
		return 0;
	}

	if (offset >= 0x4e0 && offset <= 0x6d4) {
		/* CRECON_LUMA_LUT and _GAIN_LUT: unity, as GetDefaultLtm fills them. */
		*value = BECORE_LTM_UNITY_Q8_PAIR;
		return 0;
	}

	/* The remaining four are the steps it walks the grid with. */
	switch (offset) {
	case 0x134:				/* SLCGRID_GRID_X_SCALE */
	case 0x138:				/* ..._X_SCALE_HALF */
		ret = becore_ltm_grid_scale(BECORE_LTM_SLCGRID_COLUMNS,
					    becore_rgbp_out_width(), &scale);
		if (ret)
			return ret;
		*value = offset == 0x134 ? scale : scale >> 1;
		return 0;
	case 0x13c:				/* SLCGRID_GRID_Y_SCALE */
	case 0x140:				/* ..._Y_SCALE_HALF */
		ret = becore_ltm_grid_scale(BECORE_LTM_SLCGRID_ROWS,
					    becore_rgbp_out_height(), &scale);
		if (ret)
			return ret;
		*value = offset == 0x13c ? scale : scale >> 1;
		return 0;
	}

	return -EINVAL;
}

/*
 * DIABLO_CCM's gate and its three offsets, by offset from the block's base.
 *
 * All four are zero, but say which four: a mis-stated range in the table above
 * would otherwise be answered rather than refused, and the generator that has
 * to agree with this refuses it.
 */
static int becore_yuvp_ccm_value(u32 offset, u32 *value)
{
	if (offset != 0x000 && (offset < 0x028 || offset > 0x030 ||
				offset & 3))
		return -EINVAL;

	*value = 0;

	return 0;
}

/* One coefficient of the inverse colour matrix, by offset from the first. */
static int becore_yuvp_invccm33_value(u32 offset, u32 *value)
{
	u32 index = offset / 4;

	if (offset & 3 || index >= BECORE_INVCCM33_COEFFICIENTS)
		return -EINVAL;

	*value = index % 4 ? 0 : 1 << BECORE_INVCCM33_Q;

	return 0;
}

/*
 * One register of a colour-LUT input curve, by offset from the curve's first.
 *
 * The identity is what the block does when nothing is asking it to reshape a
 * channel, and it is also what the vendor writes: en_config leaves all three
 * of these curves disabled, so their contents are a default rather than a
 * tuning choice. Packing is Google's own, from clut_packing.h.
 */
static int becore_yuvp_clut_1dlut_value(u32 offset, u32 *value)
{
	u32 packed = 0;
	u32 index;
	u32 entry;
	u32 field;

	if (offset & 3 || offset > BECORE_CLUT_1DLUT_LAST)
		return -EINVAL;

	if (offset == BECORE_CLUT_1DLUT_LAST) {
		/*
		 * The curve's last point is 1.0, one past a 10-bit field, so
		 * the register after entry 63 holds its distance from unity
		 * instead -- the same trick the gamma tables use for their
		 * sixty-fifth knot.
		 */
		entry = (BECORE_CLUT_1DLUT_ENTRIES - 1) * BECORE_CLUT_1DLUT_STEP;
		*value = entry | ((BECORE_CLUT_ONE - entry)
				  << BECORE_CLUT_FIELD_BITS);
		return 0;
	}

	index = offset / 4 * BECORE_CLUT_1DLUT_PER_REG;
	for (field = 0; field < BECORE_CLUT_1DLUT_PER_REG; field++) {
		entry = (index + field) * BECORE_CLUT_1DLUT_STEP;
		packed |= entry << (BECORE_CLUT_FIELD_BITS * field);
	}
	*value = packed;

	return 0;
}

/*
 * Read one fixed word back out of the recipe by register address.
 *
 * Every other generated value is a function of the hardware description or of
 * a constant, and resolves from the register alone. A noise curve's slopes are
 * a function of its knots, which are tuning and stay in the recipe -- so
 * resolving one means finding a sibling register's value.
 *
 * What makes that safe is not the order this runs in: it reads the
 * compiled-in table, so its answer does not depend on validation having run.
 * It is that the two ends agree by construction. Validation refuses any staged
 * recipe whose fixed words differ from these, and encode copies the staged
 * payload and then overwrites only address, typed and generated words -- so a
 * knot that is programmed is always the knot a slope was derived from.
 *
 * A command list's last write to a register is the one the hardware keeps, and
 * this recipe really does write one YUVP register twice with two values, so
 * take the last match rather than the first. A register carried only by a
 * repeated-target header, or one that is not a fixed word at all, is invisible
 * here and returns -EINVAL, which fails the encode rather than guessing.
 */
static int becore_recipe_fixed_value(enum becore_block_id id, u32 reg,
				     u32 *value)
{
	const struct becore_cmdq_shape *shape;
	bool found = false;
	u32 count;
	u32 i;
	u32 word;

	switch (id) {
	case BECORE_RGBP:
		shape = becore_rgbp_shape;
		count = BECORE_RGBP_HEADER_COUNT;
		break;
	case BECORE_YUVP:
		shape = becore_yuvp_shape;
		count = BECORE_YUVP_HEADER_COUNT;
		break;
	case BECORE_MCSC:
		shape = becore_mcsc_shape;
		count = BECORE_MCSC_HEADER_COUNT;
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		for (word = 0; word < shape[i].valid_words; word++) {
			u32 candidate;

			if (!(shape[i].fixed_mask & BIT(word)))
				continue;
			if (becore_shape_register(&shape[i], word, &candidate) ||
			    candidate != reg)
				continue;
			*value = shape[i].fixed_values[word];
			found = true;
		}
	}

	return found ? 0 : -EINVAL;
}

/*
 * Each entry names the knots its slopes come from, so a chroma curve whose own
 * domain registers this table generates points at the luma copy it repeats
 * rather than at itself -- those would read back as zero.
 */
struct becore_noise_curve {
	enum becore_block_id block;
	u32 x_first;
	u32 y_first;
	u32 slope_first;
	u32 shift_reg;
	u32 domain_first;	/* 0 when the curve owns its own domain */
	bool round;		/* to nearest; false truncates */
};

static const struct becore_noise_curve becore_noise_curves[] = {
	{ BECORE_RGBP, BECORE_RGBP_DNS_X_G_REG, BECORE_RGBP_DNS_Y_G_REG,
	  BECORE_RGBP_DNS_SLOPE_G_REG, BECORE_RGBP_DNS_SHIFT_G_REG, 0, true },
	{ BECORE_RGBP, BECORE_RGBP_DNS_X_G_REG, BECORE_RGBP_DNS_Y_RB_REG,
	  BECORE_RGBP_DNS_SLOPE_RB_REG, BECORE_RGBP_DNS_SHIFT_RB_REG,
	  BECORE_RGBP_DNS_X_RB_REG, true },
	{ BECORE_YUVP, BECORE_YUVP_NR_X_Y_REG, BECORE_YUVP_NR_Y_Y_REG,
	  BECORE_YUVP_NR_SLOPE_Y_REG, BECORE_YUVP_NR_SHIFT_Y_REG, 0, false },
	{ BECORE_YUVP, BECORE_YUVP_NR_X_Y_REG, BECORE_YUVP_NR_Y_UV_REG,
	  BECORE_YUVP_NR_SLOPE_UV_REG, BECORE_YUVP_NR_SHIFT_UV_REG,
	  BECORE_YUVP_NR_X_UV_REG, false },
};

static int becore_noise_curve_for(enum becore_block_id id, u32 reg, u32 kind,
				  size_t *found)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_noise_curves); i++) {
		const struct becore_noise_curve *curve = &becore_noise_curves[i];
		u32 first;
		u32 regs;

		if (curve->block != id)
			continue;
		if (kind == BECORE_GEN_NOISE_SLOPE) {
			first = curve->slope_first;
			regs = BECORE_NOISE_TABLE_REGS;
		} else if (kind == BECORE_GEN_NOISE_DOMAIN) {
			first = curve->domain_first;
			regs = BECORE_NOISE_TABLE_REGS;
		} else {
			first = curve->shift_reg;
			regs = 1;
		}
		if (first && reg >= first && reg < first + regs * 4) {
			*found = i;
			return 0;
		}
	}

	return -EINVAL;
}

/*
 * The knots resolved once, rather than scanned for on every frame.
 *
 * The recipe's fixed words are compile-time constants, so a curve's knots are
 * too, but finding one means walking every header of its block. That walk is
 * cheap once and expensive per frame -- the encode path runs it for all 396
 * headers of a submission -- so it happens at probe and the result is what the
 * per-frame arithmetic reads.
 */
struct becore_noise_knots {
	s32 x[BECORE_NOISE_KNOTS];
	s32 y[BECORE_NOISE_KNOTS];
};

static struct becore_noise_knots
	becore_noise_knots[ARRAY_SIZE(becore_noise_curves)];
static bool becore_noise_knots_ready;

/* Knot `index` of an eight-knot table: two 16-bit knots per register. */
static int becore_noise_read_knot(enum becore_block_id id, u32 first,
				  u32 index, s32 *knot)
{
	u32 word;
	int ret;

	if (index >= BECORE_NOISE_KNOTS)
		return -EINVAL;
	ret = becore_recipe_fixed_value(id, first + (index / 2) * 4, &word);
	if (ret)
		return ret;
	*knot = (index & 1) ? (word >> 16) & 0xffff : word & 0xffff;

	return 0;
}

/*
 * Resolve every curve's knots, and check each domain rises while we are here.
 * A curve that cannot be resolved is a wiring mistake in the table above --
 * most likely a chroma curve pointed at its own generated domain instead of
 * the luma copy it repeats -- and it is worth failing probe over rather than
 * discovering at the first STREAMON.
 */
static int becore_noise_knots_resolve(struct device *dev)
{
	size_t i;
	u32 index;
	int ret;

	for (i = 0; i < ARRAY_SIZE(becore_noise_curves); i++) {
		const struct becore_noise_curve *curve = &becore_noise_curves[i];

		for (index = 0; index < BECORE_NOISE_KNOTS; index++) {
			ret = becore_noise_read_knot(curve->block,
						     curve->x_first, index,
						     &becore_noise_knots[i].x[index]);
			if (ret)
				return dev_err_probe(dev, ret,
						     "noise curve %zu has no knot domain\n",
						     i);
			ret = becore_noise_read_knot(curve->block,
						     curve->y_first, index,
						     &becore_noise_knots[i].y[index]);
			if (ret)
				return dev_err_probe(dev, ret,
						     "noise curve %zu has no knot range\n",
						     i);
		}
		for (index = 1; index < BECORE_NOISE_KNOTS; index++) {
			if (becore_noise_knots[i].x[index] <=
			    becore_noise_knots[i].x[index - 1])
				return dev_err_probe(dev, -ERANGE,
						     "noise curve %zu's domain does not rise\n",
						     i);
		}
	}
	becore_noise_knots_ready = true;

	return 0;
}

/*
 * One segment's slope. There are eight slope fields for seven segments, so the
 * eighth repeats the seventh: past the last knot the curve does not turn.
 */
static int becore_noise_slope(size_t curve_index, u32 index, u32 *slope)
{
	const struct becore_noise_curve *curve =
		&becore_noise_curves[curve_index];
	const struct becore_noise_knots *knots =
		&becore_noise_knots[curve_index];
	s32 magnitude;
	s32 quotient;
	s32 delta;
	s32 dx;

	if (index >= BECORE_NOISE_KNOTS - 1)
		index = BECORE_NOISE_KNOTS - 2;
	dx = knots->x[index + 1] - knots->x[index];
	if (dx <= 0)
		return -ERANGE;
	delta = knots->y[index + 1] - knots->y[index];
	magnitude = (delta < 0 ? -delta : delta) << BECORE_NOISE_SLOPE_SHIFT;
	if (curve->round)
		quotient = (2 * magnitude + dx) / (2 * dx);
	else
		quotient = magnitude / dx;
	if (delta < 0)
		quotient = -quotient;
	if (quotient > (s32)(BECORE_NOISE_SLOPE_MASK >> 1) ||
	    quotient < -(s32)(BECORE_NOISE_SLOPE_MASK >> 1) - 1)
		return -ERANGE;
	*slope = (u32)quotient & BECORE_NOISE_SLOPE_MASK;

	return 0;
}

static int becore_noise_value(enum becore_block_id id, u32 reg, u32 kind,
			      u32 *value)
{
	size_t curve_index;
	u32 packed = 0;
	u32 index;
	u32 high;
	u32 low;
	u32 i;
	int ret;

	if (!becore_noise_knots_ready)
		return -EINVAL;
	ret = becore_noise_curve_for(id, reg, kind, &curve_index);
	if (ret)
		return ret;
	if (kind == BECORE_GEN_NOISE_SHIFT) {
		for (i = 0; i < BECORE_NOISE_SHIFT_NIBBLES; i++)
			packed |= (u32)BECORE_NOISE_SLOPE_SHIFT << (4 * i);
		*value = packed;
		return 0;
	}
	if (kind == BECORE_GEN_NOISE_DOMAIN) {
		const struct becore_noise_knots *knots =
			&becore_noise_knots[curve_index];

		index = (reg - becore_noise_curves[curve_index].domain_first) /
			4 * 2;
		if (index + 1 >= BECORE_NOISE_KNOTS)
			return -EINVAL;
		*value = (((u32)knots->x[index + 1] & 0xffff) << 16) |
			 ((u32)knots->x[index] & 0xffff);
		return 0;
	}

	index = (reg - becore_noise_curves[curve_index].slope_first) / 4 * 2;
	ret = becore_noise_slope(curve_index, index, &low);
	if (ret)
		return ret;
	ret = becore_noise_slope(curve_index, index + 1, &high);
	if (ret)
		return ret;
	*value = (high << 16) | low;

	return 0;
}

/* Samsung's init_djag_cfgs, which is also this field's POR reset value. */
static const u16 becore_djag_lfsr_seeds[] = { 44257, 4671, 47792 };
static const u8 becore_djag_dither_ramp[] = { 0, 0, 1, 2, 3, 4, 6, 7, 8 };

/*
 * The literals GetDefaultDmsc(DmscRgbpOutput&) writes once the tuning path has
 * run, so they are the same whatever the scene was.
 */
static const struct becore_regval becore_dmsc_defaults[] = {
	{ 0x20c, 0x36a },		/* BASE_CONFIG */
	{ 0x238, 0x07f },		/* EXTRACT_COLORS_CONFIG */
	{ 0x24c, 0x00d500b4 },		/* GREEN_HUE: min 0xb4, max 0xd5 */
	{ 0x250, 0x0dac0046 },		/* GREEN_SAT: min 0x46, max 0xdac */
	{ 0x258, 0xfff },		/* POST_PROCESS_CONFIG */
	{ 0x264, 0x508 },		/* DIR_DETECT_SELECTION */
	{ 0x26c, 0x000600a0 },		/* ADD_COLORS_GREEN */
	{ 0x270, 0x120 },		/* FALSE_COLORS */
	{ 0x274, 0x003 },		/* SHARPENING_CONFIG */
	{ 0x288, 0x001 },		/* NEAR_EDGE_DESAT_EN */
	{ 0x2a0, 0x001 },		/* EDGE_DESAT_RED_PRESERVE_EN */
	{ 0x2a8, 0x000 },		/* EDGE_DESAT_RED_PRESERVE_THRES */
	{ 0x2ac, 0x800 },		/* EDGE_DESAT_RED_PRESERVE_LIMIT */
	{ 0x2b4, 0x011 },		/* ADD_YBLUR */
};

static int becore_mcsc_djag_value(u32 offset, u32 *value)
{
	u32 packed = 0;
	u32 first;
	u32 count;
	u32 i;

	switch (offset) {
	case 0x000:		/* CTRL: DJAG, its pre-scaler and EZ post on */
		*value = BIT(0) | BIT(1) | BIT(10);
		return 0;
	case 0x01c:		/* PS_H_INIT_PHASE_OFFSET */
	case 0x020:		/* PS_V_INIT_PHASE_OFFSET */
	case 0x080:		/* RECOM_CTRL: detail restoration is off */
	case 0x088:		/* RECOM_WEIGHT: and off by its weight too */
		*value = 0;
		return 0;
	case 0x024:		/* PS_ROUND_MODE */
		*value = 1;
		return 0;
	case 0x050:		/* LFSR_SEED_0 */
	case 0x054:		/* LFSR_SEED_1 */
	case 0x058:		/* LFSR_SEED_2 */
		*value = becore_djag_lfsr_seeds[(offset - 0x050) / 4];
		return 0;
	case 0x05c:		/* DITHER_VALUE_04: five 6-bit steps */
	case 0x060:		/* DITHER_VALUE_58: the remaining four */
		first = offset == 0x05c ? 0 : 5;
		count = offset == 0x05c ? 5 : 4;
		for (i = 0; i < count; i++)
			packed |= (u32)becore_djag_dither_ramp[first + i] <<
				  (BECORE_DJAG_DITHER_FIELD_BITS * i);
		*value = packed;
		return 0;
	case 0x064:		/* DITHER_THRES, which also carries SAT_CTRL */
		*value = BECORE_DJAG_SAT_CTRL |
			 BECORE_DJAG_DITHER_THRES <<
			 BECORE_DJAG_DITHER_THRES_SHIFT;
		return 0;
	case 0x068:		/* CP_HF_THRES */
		*value = BECORE_DJAG_CP_HF_THRES;
		return 0;
	}

	return -EINVAL;
}

static int becore_rgbp_dmsc_value(u32 offset, u32 *value)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_dmsc_defaults); i++) {
		if (becore_dmsc_defaults[i].offset != offset)
			continue;
		*value = becore_dmsc_defaults[i].value;
		return 0;
	}

	return -EINVAL;
}

/* Where each block's scaler puts its two init phase offsets and round mode. */
static const u32 becore_scaler_phase_first[] = {
	BECORE_RGBP_SC_PHASE_FIRST,
	BECORE_MCSC_SC0_PHASE_FIRST,
	BECORE_MCSC_PC0_PHASE_FIRST,
};

static int becore_scaler_phase_value(u32 reg, u32 *value)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_scaler_phase_first); i++) {
		u32 first = becore_scaler_phase_first[i];

		if (reg < first || reg > first + BECORE_SCALER_PHASE_LAST)
			continue;
		switch (reg - first) {
		case 0x00:	/* H_INIT_PHASE_OFFSET: no sub-pixel origin */
		case 0x04:	/* V_INIT_PHASE_OFFSET */
			*value = 0;
			return 0;
		case 0x08:	/* ROUND_MODE, on the MCSC blocks only */
			*value = 1;
			return 0;
		}
		return -EINVAL;
	}

	return -EINVAL;
}

/*
 * BT.601 as exact rationals, column-major by input channel. Kr is 299/1000 and
 * Kb 114/1000; the chroma rows are those over 2 * (1 - Kb) and 2 * (1 - Kr),
 * whose denominators are 1772 and 1402.
 */
static const struct becore_csc_coefficient {
	s32 numerator;
	s32 denominator;
} becore_csc_matrix[3][3] = {
	{ { 299, 1000 }, { -299, 1772 }, { 1, 2 } },
	{ { 587, 1000 }, { -587, 1772 }, { -587, 1402 } },
	{ { 114, 1000 }, { 1, 2 }, { -114, 1402 } },
};

/*
 * BT.601 the other way, as exact rationals and row-major: R = Y + 2(1 - Kr) V,
 * G = Y - Kb/(1 - Kr - Kb) * 2(1 - Kb) U - Kr/(1 - Kr - Kb) * 2(1 - Kr) V, and
 * B = Y + 2(1 - Kb) U. The colour LUT converts to RGB before it looks a colour
 * up, which is why a YUV block carries this at all.
 */
static const struct becore_csc_coefficient
becore_clut_yuv2rgb[3][3] = {
	{ { 1, 1 }, { 0, 1 },			{ 1402, 1000 } },
	{ { 1, 1 }, { -114 * 1772, 587 * 1000 }, { -299 * 1402, 587 * 1000 } },
	{ { 1, 1 }, { 1772, 1000 },		{ 0, 1 } },
};

static_assert(ARRAY_SIZE(becore_clut_yuv2rgb) *
	      ARRAY_SIZE(becore_clut_yuv2rgb[0]) ==
	      BECORE_CLUT_MATRIX_COEFFICIENTS);

/* [1, 2, 1] / 4 scaled by 32, one byte per tap. */
static const u8 becore_chroma_lpf_taps[] = { 0, 32, 64, 32, 0 };

/* The sharpener's three low-pass kernels sum to these; all powers of two. */
static const u32 becore_sharpenhancer_lpf_sums[] = { 16, 512, 4096 };

/*
 * round(coefficient << q), away from zero, as a signed field of its own.
 *
 * The rounded numerator does not fit 32 bits for every table here -- the
 * inverse matrix below carries 419198 -- so it is formed at 64 bits rather
 * than left to a bound each new coefficient would have to be checked against.
 * Denominators stay small enough to divide by.
 */
static u32 becore_csc_coefficient(const struct becore_csc_coefficient *coef,
				  u32 q, u32 mask)
{
	s64 magnitude = coef->numerator < 0 ? -coef->numerator : coef->numerator;

	magnitude = div_s64(magnitude * (1 << q) * 2 + coef->denominator,
			    2 * coef->denominator);
	if (coef->numerator < 0)
		magnitude = -magnitude;

	return (u32)magnitude & mask;
}

static int becore_rgbp_csc_value(u32 offset, u32 *value)
{
	if (offset & 3)
		return -EINVAL;
	if (offset >= 0x04 && offset < 0x28) {
		u32 index = (offset - 0x04) / 4;

		*value = becore_csc_coefficient(&becore_csc_matrix[index / 3]
								 [index % 3],
						BECORE_RGBP_CSC_Q,
						BECORE_RGBP_CSC_FIELD_MASK);
		return 0;
	}

	switch (offset) {
	case 0x00:		/* BYPASS: the block runs */
	case 0x28:		/* YMIN */
	case 0x30:		/* UMIN */
	case 0x38:		/* VMIN */
	case 0x40:		/* LS: no post-matrix shift */
	case 0x44:		/* YOS: full range puts luma's offset at zero */
		*value = 0;
		return 0;
	case 0x2c:		/* YMAX */
	case 0x34:		/* UMAX */
	case 0x3c:		/* VMAX */
		*value = BECORE_RGBP_CSC_MAX;
		return 0;
	case 0x48:		/* UOS */
	case 0x4c:		/* VOS */
		*value = BECORE_RGBP_CSC_CHROMA_OFFSET;
		return 0;
	}

	return -EINVAL;
}

/*
 * The colour LUT's gate and the matrix in front of its lattice, by offset from
 * BECORE_YUVP_CLUT_BASE.
 *
 * The lattice itself is not here and cannot be: 4,913 (R, G, B) nodes of a
 * (U, V) pair is per-camera tuning, and it belongs in a parameters buffer
 * rather than in this driver. Which is also why the gate says *bypass*: the
 * entries are the chroma the block outputs rather than an offset to it, so
 * there is no neutral fill that leaves the picture alone, and the only
 * identity this block has is not running. becore_params_value() clears the
 * bypass when a lattice arrives.
 *
 * The rest of it is stateable and is what the block is for: convert YUV to RGB
 * with the ordinary BT.601 inverse, and take the input curves as written.
 */
static int becore_yuvp_clut_value(u32 offset, u32 *value)
{
	u32 index;

	if (offset & 3)
		return -EINVAL;

	switch (offset) {
	case 0x000:		/* BYPASS: nothing until a lattice arrives */
		*value = 1;
		return 0;
	case 0x004:		/* EN_CONFIG: the matrix, and no input curve */
		*value = BECORE_CLUT_EN_MATRIX;
		return 0;
	}

	if (offset < 0x03c || offset > 0x04c)
		return -EINVAL;

	/*
	 * Two coefficients per register, the lower-numbered one in the low
	 * half. The ninth is the last, so the upper half of the fifth register
	 * has nothing to carry.
	 */
	index = (offset - 0x03c) / 4 * 2;
	*value = becore_csc_coefficient(&becore_clut_yuv2rgb[index / 3]
							    [index % 3],
					BECORE_CLUT_MATRIX_Q,
					BECORE_CLUT_MATRIX_FIELD_MASK);
	index++;
	if (index < BECORE_CLUT_MATRIX_COEFFICIENTS)
		*value |= becore_csc_coefficient(&becore_clut_yuv2rgb[index / 3]
								     [index % 3],
						 BECORE_CLUT_MATRIX_Q,
						 BECORE_CLUT_MATRIX_FIELD_MASK)
			  << 16;

	return 0;
}

static int becore_rgbp_chroma_lpf_value(u32 offset, u32 *value)
{
	u32 packed = 0;
	size_t i;

	switch (offset) {
	case 0x00:		/* ISP_BYPASS: the block runs */
		*value = 0;
		return 0;
	case 0x08:		/* COEFFS: the first four taps, one byte each */
		for (i = 0; i < 4; i++)
			packed |= (u32)becore_chroma_lpf_taps[i] << (8 * i);
		*value = packed;
		return 0;
	case 0x0c:		/* COEFFS_1: the fifth */
		*value = becore_chroma_lpf_taps[4];
		return 0;
	}

	return -EINVAL;
}

static int
becore_rgbp_dns_geometry_value(const struct becore_rgbp_input_profile *profile,
			       u32 offset, u32 *value)
{
	s32 x;
	s32 y;

	switch (offset) {
	case 0x1a4:		/* BINNING: Q10, x in [0:13], y in [16:29] */
		*value = (BECORE_RGBP_DNS_BINNING_UNITY << 16) |
			 BECORE_RGBP_DNS_BINNING_UNITY;
		return 0;
	case 0x1c0:		/* RADIAL_CENTER: 15-bit signed, x low, y high */
		x = -(s32)((profile->width >> 1) & ~1u);
		y = -(s32)((profile->height >> 1) & ~1u);
		*value = ((y & BECORE_RGBP_DNS_CENTRE_MASK) << 16) |
			 (x & BECORE_RGBP_DNS_CENTRE_MASK);
		return 0;
	}

	return -EINVAL;
}

/* log2 of each kernel's sum, packed at bits 0, 8 and 16. */
static int becore_yuvp_lpf_norm_value(u32 *value)
{
	u32 packed = 0;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_sharpenhancer_lpf_sums); i++) {
		u32 total = becore_sharpenhancer_lpf_sums[i];

		if (!is_power_of_2(total))
			return -EINVAL;
		packed |= (u32)(ilog2(total)) << (8 * i);
	}
	*value = packed;

	return 0;
}

/* The x grid: finest where a square root moves fastest, tiling Q12 exactly. */
static int becore_rgbp_gamma_knot(u32 index, u32 *x)
{
	static const struct {
		u8 count;
		u16 step;
	} segments[] = {
		{ 8, 8 }, { 12, 16 }, { 8, 32 }, { 16, 64 }, { 20, 128 },
	};
	u32 value = 0;
	size_t i;

	/*
	 * The grid tiles 0..1 << Q exactly, and index 64 relies on falling out
	 * of the loop below with nothing left. A miscount here would be silent
	 * -- the recipe carries zero for these words -- so make it loud.
	 */
	static_assert(8 + 12 + 8 + 16 + 20 == BECORE_RGBP_GAMMA_SEGMENTS);
	static_assert(8 * 8 + 12 * 16 + 8 * 32 + 16 * 64 + 20 * 128 ==
		      1 << BECORE_RGBP_GAMMA_Q);

	for (i = 0; i < ARRAY_SIZE(segments); i++) {
		if (index < segments[i].count) {
			*x = value + index * segments[i].step;
			return 0;
		}
		value += segments[i].count * segments[i].step;
		index -= segments[i].count;
	}
	if (index)
		return -EINVAL;
	*x = value;		/* the 65th knot closes the grid at 1 << Q */

	return 0;
}

/* round(sqrt(x / 4096) * 4096), which is exactly round(sqrt(x << 12)). */
static u32 becore_rgbp_gamma_encode(u32 x)
{
	u32 n = x << BECORE_RGBP_GAMMA_Q;
	u32 root = int_sqrt(n);

	return n - root * root > root ? root + 1 : root;
}

static int becore_rgbp_gamma_point(u32 index, bool encode, u32 *value)
{
	u32 x;
	int ret;

	if (index >= BECORE_RGBP_GAMMA_KNOTS)
		return -EINVAL;
	ret = becore_rgbp_gamma_knot(index, &x);
	if (ret)
		return ret;
	*value = encode ? becore_rgbp_gamma_encode(x) : x;

	return 0;
}

/* Two points per register, the lower-numbered one in the low half. */
static int becore_rgbp_gamma_pair(u32 index, bool encode, u32 *value)
{
	u32 low;
	u32 high;
	int ret;

	ret = becore_rgbp_gamma_point(index, encode, &low);
	if (ret)
		return ret;
	ret = becore_rgbp_gamma_point(index + 1, encode, &high);
	if (ret)
		return ret;
	*value = (high << 16) | low;

	return 0;
}

/*
 * The 65th point, as its distance from the 64th: it does not fit the field.
 *
 * The vendor stores a magnitude and puts the direction in a separate
 * _DELTA_SIGN register. Neither of these curves falls, so that register stays
 * unwritten -- but take the magnitude rather than the difference anyway, so a
 * curve that did fall would encode a small number here instead of wrapping.
 */
static int becore_rgbp_gamma_delta(bool encode, u32 *value)
{
	u32 last;
	u32 prev;
	int ret;

	ret = becore_rgbp_gamma_point(BECORE_RGBP_GAMMA_KNOTS - 1, encode, &last);
	if (ret)
		return ret;
	ret = becore_rgbp_gamma_point(BECORE_RGBP_GAMMA_KNOTS - 2, encode, &prev);
	if (ret)
		return ret;
	*value = last > prev ? last - prev : prev - last;

	return 0;
}

static int becore_rgbp_gamma_value(u32 offset, u32 *value)
{
	if (offset & 3)
		return -EINVAL;

	switch (offset) {
	case 0x000:		/* BYPASS: the block runs */
	case 0x004:		/* PEDESTAL_EN */
		*value = 0;
		return 0;
	case 0x08c:		/* R_GAMMA_TBL last-knot delta */
		return becore_rgbp_gamma_delta(true, value);
	case 0x250:		/* X_PNTS_TBL last-knot delta */
		return becore_rgbp_gamma_delta(false, value);
	case 0x254:		/* X_PNTS_LSHIFT */
		*value = BECORE_RGBP_GAMMA_X_LSHIFT;
		return 0;
	}

	if (offset >= 0x00c && offset < 0x08c)
		return becore_rgbp_gamma_pair((offset - 0x00c) / 4 * 2, true,
					      value);
	if (offset >= 0x1c0 && offset <= 0x1ec)
		return becore_rgbp_gamma_pair((offset - 0x1c0) / 4 * 2, false,
					      value);
	/*
	 * The four-register hole between points 23 and 24 is 0x1f0..0x1fc. No
	 * range covers it and the capture writes nothing there, so it is never
	 * asked for; if it were, the -EINVAL below would reject it.
	 */
	if (offset >= 0x200 && offset < 0x250)
		return becore_rgbp_gamma_pair((offset - 0x200) / 4 * 2 + 24,
					      false, value);

	return -EINVAL;
}

/* The identity itself: out[i] == in[i] << 5 at every knot. */
static int becore_rgbp_gtm_value(u32 offset, u32 *value)
{
	u32 knot;
	u32 low;
	int ret;

	if (offset & 3)
		return -EINVAL;

	switch (offset) {
	case BECORE_RGBP_GTM_BYPASS:		/* the block runs */
	case BECORE_RGBP_GTM_GAIN_MODE_EN:
	case BECORE_RGBP_GTM_V_BLEND_RATIO:
	case BECORE_RGBP_GTM_INPUT_RSHIFT:
		*value = 0;
		return 0;
	case BECORE_RGBP_GTM_Y_WEIGHT_0:
		*value = (BECORE_RGBP_GTM_Y_WEIGHT_G << 16) |
			 BECORE_RGBP_GTM_Y_WEIGHT_R;
		return 0;
	case BECORE_RGBP_GTM_Y_WEIGHT_1:
		*value = BECORE_RGBP_GTM_Y_WEIGHT_B;
		return 0;
	}

	if (offset >= BECORE_RGBP_GTM_IN_POINTS &&
	    offset < BECORE_RGBP_GTM_OUT_POINTS) {
		u32 index = (offset - BECORE_RGBP_GTM_IN_POINTS) / 4 * 2;

		ret = becore_rgbp_gtm_knot(index, &low);
		if (ret)
			return ret;
		ret = becore_rgbp_gtm_knot(index + 1, &knot);
		if (ret)
			return ret;
		*value = (knot << 16) | low;
		return 0;
	}

	if (offset >= BECORE_RGBP_GTM_OUT_POINTS &&
	    offset < BECORE_RGBP_GTM_Y_WEIGHT_0) {
		ret = becore_rgbp_gtm_knot((offset -
					    BECORE_RGBP_GTM_OUT_POINTS) / 4,
					   &knot);
		if (ret)
			return ret;
		*value = knot << BECORE_RGBP_GTM_OUT_SHIFT;
		return 0;
	}

	return -EINVAL;
}

/*
 * DJAG is the stage that crops and scales on this path, so what it hands down
 * is already the output raster: there is exactly one raster below it, and
 * POLY_SC0 and POST_PC0 map that raster onto itself. Their ratio is therefore
 * unity by construction rather than by measurement, and this cannot be
 * written as a ratio of two independent extents because there is no second
 * extent to name.
 *
 * Which means the poly-phase filter set below is pinned to x8/8 here whatever
 * the geometry, since the ratio selects it. That is right for this
 * configuration and would be wrong for one where DJAG is switched off and
 * POLY_SC0 does the scaling, which is what the vendor's own 640x480 request
 * does -- there it runs in the x5/8 band. Making that configuration
 * expressible means giving the chain a source raster of its own, and until
 * then the honest thing is to say so rather than to route unity through a
 * helper that looks like a derivation.
 *
 * The one thing that is per axis is which extent the register describes, so
 * take it from the matching one: a vertical register computed from a width
 * would be a real defect the moment the two differ.
 */
static u32 becore_mcsc_chain_ratio(bool vertical)
{
	u32 extent = vertical ? becore_mcsc_output.height :
				becore_mcsc_output.width;

	return becore_zoom_ratio(extent, extent);
}

/*
 * Samsung publishes its poly-phase coefficients because they are a function of
 * the scaling ratio rather than of the scene: get_scaler_coef_ver2() picks one
 * of seven sets by comparing the ratio against x8/8, x7/8 and so on down to
 * x2/8. All seven are here because a scaler that downscales needs whichever
 * one its ratio selects -- the rear ultrawide and the rear camera run at
 * unity, the front camera at 4096/4000 and its 640x480 mode at exactly 2x.
 *
 * These are Samsung's numbers verbatim from is-hw-api-mcscaler-v9_1.c, indexed
 * [set][tap][phase]. The MCSC table rather than the RGBP one, although the two
 * publish the same coefficients: RGBP's are at 2048 and Zuma's fields hold
 * 512, and quartering them reproduces two transcription errors that the
 * hardware contradicts -- horizontal x7/8 tap 0 phase 1, where Samsung's row
 * breaks its own monotone run, and horizontal x5/8 tap 0 phase 0, where it has
 * the wrong sign. Each error also corrupts whichever tap absorbs the
 * renormalisation, so each costs two words. MCSC publishes them already at
 * 512, so nothing is scaled or renormalised here and there is no residual.
 *
 * Checked against every capture: 54 coefficient words in each of eighteen
 * captured RGBP programs, over three cameras and seven sensor readouts,
 * exercising x8/8, x7/8, x5/8 and x4/8, all matching bit for bit.
 */
static const s16 becore_sc_v_coeff[BECORE_SC_SETS][BECORE_SC_V_TAPS][BECORE_SC_PHASES] = {
	[BECORE_SC_SET_X8_8] = {
		{     0,   -15,   -25,   -31,   -33,   -33,   -31,   -27,   -23 },
		{   512,   508,   495,   473,   443,   408,   367,   324,   279 },
		{     0,    20,    45,    75,   110,   148,   190,   234,   279 },
		{     0,    -1,    -3,    -5,    -8,   -11,   -14,   -19,   -23 },
	},
	[BECORE_SC_SET_X7_8] = {
		{    32,    17,     3,    -7,   -14,   -18,   -20,   -20,   -19 },
		{   448,   446,   437,   421,   399,   373,   343,   310,   275 },
		{    32,    55,    79,   107,   138,   170,   204,   240,   275 },
		{     0,    -6,    -7,    -9,   -11,   -13,   -15,   -18,   -19 },
	},
	[BECORE_SC_SET_X6_8] = {
		{    61,    46,    31,    19,     9,     2,    -3,    -7,    -9 },
		{   390,   390,   383,   371,   356,   337,   315,   291,   265 },
		{    61,    83,   106,   130,   156,   183,   210,   238,   265 },
		{     0,    -7,    -8,    -8,    -9,   -10,   -10,   -10,    -9 },
	},
	[BECORE_SC_SET_X5_8] = {
		{    85,    71,    56,    43,    32,    23,    16,     9,     5 },
		{   341,   341,   336,   328,   317,   304,   288,   271,   251 },
		{    86,   105,   124,   145,   166,   187,   209,   231,   251 },
		{     0,    -5,    -4,    -4,    -3,    -2,    -1,     1,     5 },
	},
	[BECORE_SC_SET_X4_8] = {
		{   104,    89,    76,    63,    52,    42,    33,    26,    20 },
		{   304,   302,   298,   293,   285,   275,   264,   251,   236 },
		{   104,   120,   136,   153,   170,   188,   205,   221,   236 },
		{     0,     1,     2,     3,     5,     7,    10,    14,    20 },
	},
	[BECORE_SC_SET_X3_8] = {
		{   118,   103,    90,    78,    67,    57,    48,    40,    33 },
		{   276,   273,   270,   266,   260,   253,   244,   234,   223 },
		{   118,   129,   143,   157,   171,   185,   199,   211,   223 },
		{     0,     7,     9,    11,    14,    17,    21,    27,    33 },
	},
	[BECORE_SC_SET_X2_8] = {
		{   127,   111,   100,    88,    78,    68,    59,    50,    43 },
		{   258,   252,   250,   247,   242,   237,   230,   222,   213 },
		{   127,   135,   147,   159,   171,   182,   193,   204,   213 },
		{     0,    14,    15,    18,    21,    25,    30,    36,    43 },
	},
};

static const s16 becore_sc_h_coeff[BECORE_SC_SETS][BECORE_SC_H_TAPS][BECORE_SC_PHASES] = {
	[BECORE_SC_SET_X8_8] = {
		{     0,    -2,    -4,    -5,    -6,    -6,    -6,    -6,    -5 },
		{     0,     8,    14,    20,    23,    25,    26,    25,    23 },
		{     0,   -25,   -46,   -62,   -73,   -80,   -83,   -82,   -78 },
		{   512,   509,   499,   482,   458,   429,   395,   357,   316 },
		{     0,    30,    64,   101,   142,   185,   228,   273,   316 },
		{     0,    -9,   -19,   -30,   -41,   -53,   -63,   -71,   -78 },
		{     0,     2,     5,     8,    12,    15,    19,    21,    23 },
		{     0,    -1,    -1,    -2,    -3,    -3,    -4,    -5,    -5 },
	},
	[BECORE_SC_SET_X7_8] = {
		{    12,     9,     7,     5,     3,     2,     1,     0,    -1 },
		{   -32,   -24,   -16,    -9,    -3,     2,     7,    10,    13 },
		{    56,    29,     6,   -14,   -30,   -43,   -53,   -60,   -65 },
		{   444,   445,   438,   426,   410,   390,   365,   338,   309 },
		{    52,    82,   112,   144,   177,   211,   244,   277,   309 },
		{   -32,   -39,   -46,   -52,   -58,   -63,   -66,   -66,   -65 },
		{    12,    13,    14,    15,    16,    16,    16,    15,    13 },
		{     0,    -3,    -3,    -3,    -3,    -3,    -2,    -2,    -1 },
	},
	[BECORE_SC_SET_X6_8] = {
		{     8,     9,     8,     8,     8,     7,     7,     5,     5 },
		{   -44,   -40,   -36,   -32,   -27,   -22,   -18,   -13,    -9 },
		{   100,    77,    57,    38,    20,     5,    -9,   -20,   -30 },
		{   384,   382,   377,   369,   358,   344,   329,   310,   290 },
		{   100,   123,   147,   171,   196,   221,   245,   268,   290 },
		{   -44,   -47,   -49,   -49,   -48,   -47,   -43,   -37,   -30 },
		{     8,     8,     7,     5,     3,     1,    -2,    -5,    -9 },
		{     0,     0,     1,     2,     2,     3,     3,     4,     5 },
	},
	[BECORE_SC_SET_X5_8] = {
		{    -3,    -3,    -1,     0,     1,     2,     2,     3,     3 },
		{   -31,   -32,   -33,   -32,   -31,   -30,   -28,   -25,   -23 },
		{   130,   113,    97,    81,    66,    52,    38,    26,    15 },
		{   320,   319,   315,   311,   304,   296,   286,   274,   261 },
		{   130,   147,   165,   182,   199,   216,   232,   247,   261 },
		{   -31,   -29,   -26,   -22,   -17,   -11,    -3,     5,    15 },
		{    -3,    -6,    -8,   -11,   -13,   -16,   -18,   -21,   -23 },
		{     0,     3,     3,     3,     3,     3,     3,     3,     3 },
	},
	[BECORE_SC_SET_X4_8] = {
		{   -11,   -10,    -9,    -8,    -7,    -6,    -5,    -5,    -4 },
		{     0,    -4,    -7,   -10,   -12,   -14,   -15,   -16,   -17 },
		{   140,   129,   117,   106,    95,    85,    74,    64,    55 },
		{   255,   254,   253,   250,   246,   241,   236,   229,   222 },
		{   140,   151,   163,   174,   185,   195,   204,   214,   222 },
		{     0,     5,    10,    16,    22,    29,    37,    46,    55 },
		{   -12,   -13,   -14,   -15,   -16,   -16,   -17,   -17,   -17 },
		{     0,     0,    -1,    -1,    -1,    -2,    -2,    -3,    -4 },
	},
	[BECORE_SC_SET_X3_8] = {
		{    -5,    -5,    -5,    -5,    -5,    -5,    -5,    -5,    -5 },
		{    31,    27,    23,    19,    16,    12,    10,     7,     5 },
		{   133,   126,   119,   112,   105,    98,    91,    84,    78 },
		{   195,   195,   194,   193,   191,   189,   185,   182,   178 },
		{   133,   139,   146,   152,   158,   163,   169,   174,   178 },
		{    31,    37,    41,    47,    53,    59,    65,    71,    78 },
		{    -6,    -4,    -3,    -2,    -2,     0,     1,     3,     5 },
		{     0,    -3,    -3,    -4,    -4,    -4,    -4,    -4,    -5 },
	},
	[BECORE_SC_SET_X2_8] = {
		{    10,     9,     7,     6,     5,     4,     4,     3,     2 },
		{    52,    48,    45,    41,    38,    35,    31,    29,    26 },
		{   118,   114,   110,   106,   102,    98,    94,    89,    85 },
		{   152,   152,   151,   150,   149,   148,   146,   145,   143 },
		{   118,   122,   125,   129,   132,   135,   138,   140,   143 },
		{    52,    56,    60,    64,    68,    72,    77,    81,    85 },
		{    10,    11,    13,    15,    17,    19,    21,    23,    26 },
		{     0,     0,     1,     1,     1,     1,     1,     2,     2 },
	},
};

/*
 * The set get_scaler_coef_ver2() selects for a ratio. Its bands are
 * is-hw-api-mcscaler-v3.h's RATIO_X*_8: at or below the first is x8/8, and
 * anything past the last saturates at x2/8 rather than failing, because the
 * filter is a best fit and not a description of the scaling.
 */
static const u32 becore_sc_ratio_bands[BECORE_SC_SETS] = {
	1048576, 1198373, 1398101, 1677722, 2097152, 2796203, 4194304,
};

static u32 becore_sc_coeff_set(u32 ratio)
{
	u32 set;

	for (set = 0; set + 1 < BECORE_SC_SETS; set++)
		if (ratio <= becore_sc_ratio_bands[set])
			break;

	return set;
}

/* Two taps of one phase, the lower-numbered one in the low half. */
static int becore_sc_coeff_value(bool vertical, u32 ratio, u32 index,
				 u32 *value)
{
	u32 set = becore_sc_coeff_set(ratio);
	u32 taps = vertical ? BECORE_SC_V_TAPS : BECORE_SC_H_TAPS;
	u32 pairs = taps / 2;
	u32 phase = index / pairs;
	u32 pair = index % pairs;
	s16 low, high;

	if (phase >= BECORE_SC_PHASES)
		return -EINVAL;
	if (vertical) {
		low = becore_sc_v_coeff[set][pair * 2][phase];
		high = becore_sc_v_coeff[set][pair * 2 + 1][phase];
	} else {
		low = becore_sc_h_coeff[set][pair * 2][phase];
		high = becore_sc_h_coeff[set][pair * 2 + 1][phase];
	}
	*value = (((u32)high & BECORE_SC_COEFF_MASK) << 16) |
		 ((u32)low & BECORE_SC_COEFF_MASK);

	return 0;
}

/*
 * The ratio the scaler these coefficients belong to is programmed with. Taken
 * from the same place the driver takes the value it writes into the ratio
 * register, so the filter and the ratio cannot describe different scalings.
 */
static int becore_sc_ratio(const struct becore_device *becore,
			   enum becore_block_id id, bool vertical, u32 *ratio)
{
	u32 index;

	if (id == BECORE_RGBP) {
		index = vertical ? BECORE_RGBP_SC_V_RATIO :
				   BECORE_RGBP_SC_H_RATIO;
		return becore_rgbp_input_value(becore_rgbp_input_profile(becore),
					       index,
					       becore_rgbp_input_regs[index],
					       ratio);
	}
	if (id != BECORE_MCSC)
		return -EINVAL;
	*ratio = becore_mcsc_chain_ratio(vertical);

	return 0;
}

static u32 becore_generated_word_count(enum becore_block_id id)
{
	if (id == BECORE_RGBP)
		return BECORE_RGBP_GENERATED_WORDS;
	if (id == BECORE_YUVP)
		return BECORE_YUVP_GENERATED_WORDS;
	if (id == BECORE_MCSC)
		return BECORE_MCSC_GENERATED_WORDS;

	return 0;
}

/*
 * The by-register class is only unambiguous while the ranges are. A first
 * match wins below, so an overlap would silently hand a register the wrong
 * intent -- and these ranges are wide enough that an overlap is an easy edit
 * to make. Check the tables once, at probe, rather than trusting the reader.
 */
static int becore_generated_tables_validate(struct device *dev)
{
	static const struct becore_generated_range *tables[] = {
		becore_rgbp_generated,
		becore_yuvp_generated,
		becore_mcsc_generated,
	};
	static const size_t counts[] = {
		ARRAY_SIZE(becore_rgbp_generated),
		ARRAY_SIZE(becore_yuvp_generated),
		ARRAY_SIZE(becore_mcsc_generated),
	};
	size_t block;
	size_t i;
	size_t j;

	for (block = 0; block < ARRAY_SIZE(tables); block++) {
		for (i = 0; i < counts[block]; i++) {
			const struct becore_generated_range *a =
				&tables[block][i];

			if (a->first > a->last)
				return dev_err_probe(dev, -EINVAL,
						     "generated range %zu:%zu is inverted\n",
						     block, i);
			for (j = 0; j < i; j++) {
				const struct becore_generated_range *b =
					&tables[block][j];

				if (a->first <= b->last && b->first <= a->last)
					return dev_err_probe(dev, -EINVAL,
							     "generated ranges %zu:%zu and %zu:%zu overlap\n",
							     block, j, block, i);
			}
		}
	}

	return 0;
}

static int becore_generated_value(const struct becore_device *becore,
				  enum becore_block_id id, u32 reg, u32 *value)
{
	const struct becore_rgbp_input_profile *input =
		becore_rgbp_input_profile(becore);
	const struct becore_generated_range *table;
	size_t count;
	size_t i;

	if (id == BECORE_RGBP) {
		table = becore_rgbp_generated;
		count = ARRAY_SIZE(becore_rgbp_generated);
	} else if (id == BECORE_YUVP) {
		table = becore_yuvp_generated;
		count = ARRAY_SIZE(becore_yuvp_generated);
	} else if (id == BECORE_MCSC) {
		table = becore_mcsc_generated;
		count = ARRAY_SIZE(becore_mcsc_generated);
	} else {
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		u32 result;

		if (reg < table[i].first || reg > table[i].last)
			continue;
		switch (table[i].kind) {
		case BECORE_GEN_OFF:
			result = 0;
			break;
		case BECORE_GEN_BYPASS:
			result = 1;
			break;
		case BECORE_GEN_RUNNING:
			result = 0;
			break;
		case BECORE_GEN_DECOMP_SIZE:
			result = becore_pack_size(input->height, input->width);
			break;
		case BECORE_GEN_NOISE_SLOPE:
		case BECORE_GEN_NOISE_SHIFT:
		case BECORE_GEN_NOISE_DOMAIN:
			if (becore_noise_value(id, reg, table[i].kind, &result))
				return -EINVAL;
			break;
		case BECORE_GEN_DJAG:
			if (becore_mcsc_djag_value(reg - BECORE_MCSC_DJAG_BASE,
						   &result))
				return -EINVAL;
			break;
		case BECORE_GEN_DMSC:
			if (becore_rgbp_dmsc_value(reg - BECORE_RGBP_DMSC_BASE,
						   &result))
				return -EINVAL;
			break;
		case BECORE_GEN_CSC:
			if (becore_rgbp_csc_value(reg - (id == BECORE_RGBP ?
							 BECORE_RGBP_CSC_BASE :
							 BECORE_YUVP_CSC_BASE),
						  &result))
				return -EINVAL;
			break;
		case BECORE_GEN_SCALER_PHASE:
			if (becore_scaler_phase_value(reg, &result))
				return -EINVAL;
			break;
		case BECORE_GEN_MCSC_INPUT_SIZE:
			if (reg == BECORE_MCSC_IN_WIDTH_REG)
				result = becore_rgbp_out_width();
			else if (reg == BECORE_MCSC_IN_HEIGHT_REG)
				result = becore_rgbp_out_height();
			else
				return -EINVAL;
			break;
		case BECORE_GEN_CHROMA_LPF:
			if (becore_rgbp_chroma_lpf_value(reg -
						BECORE_RGBP_CHROMA_LPF_BASE,
						&result))
				return -EINVAL;
			break;
		case BECORE_GEN_DNS_GEOMETRY: {
			u32 offset = reg - BECORE_RGBP_DNS_BASE;

			if (becore_rgbp_dns_geometry_value(input, offset,
							   &result))
				return -EINVAL;
			break;
		}
		case BECORE_GEN_LPF_NORM:
			if (becore_yuvp_lpf_norm_value(&result))
				return -EINVAL;
			break;
		case BECORE_GEN_GAMMA:
			if (becore_rgbp_gamma_value(reg -
						    BECORE_RGBP_GAMMA_BASE,
						    &result))
				return -EINVAL;
			break;
		case BECORE_GEN_GTM:
			if (becore_rgbp_gtm_value(reg - BECORE_RGBP_GTM_BASE,
						  &result))
				return -EINVAL;
			break;
		case BECORE_GEN_LTM:
			if (becore_yuvp_ltm_value(reg - BECORE_YUVP_LTM_BASE,
						  &result))
				return -EINVAL;
			break;
		case BECORE_GEN_CLUT_1DLUT:
			if (becore_yuvp_clut_1dlut_value(reg - table[i].first,
							 &result))
				return -EINVAL;
			break;
		case BECORE_GEN_INVCCM33:
			if (becore_yuvp_invccm33_value(reg -
						BECORE_YUVP_INVCCM33_FIRST,
						&result))
				return -EINVAL;
			break;
		case BECORE_GEN_CCM:
			if (becore_yuvp_ccm_value(reg - BECORE_YUVP_CCM_BASE,
						  &result))
				return -EINVAL;
			break;
		case BECORE_GEN_CLUT:
			if (becore_yuvp_clut_value(reg -
						   BECORE_YUVP_CLUT_BASE,
						   &result))
				return -EINVAL;
			break;
		case BECORE_GEN_SC_V_COEFF:
		case BECORE_GEN_SC_H_COEFF: {
			bool vertical = table[i].kind == BECORE_GEN_SC_V_COEFF;
			u32 ratio;

			if (becore_sc_ratio(becore, id, vertical, &ratio))
				return -EINVAL;
			if (becore_sc_coeff_value(vertical, ratio,
						  (reg - table[i].first) / 4,
						  &result))
				return -EINVAL;
			break;
		}
		case BECORE_GEN_CHAIN_ORIGIN:
			result = becore_pack_size(0, 0);
			break;
		case BECORE_GEN_CHAIN_SIZE:
			result = becore_pack_size(becore_mcsc_output.width,
						  becore_mcsc_output.height);
			break;
		case BECORE_GEN_CHAIN_RATIO: {
			bool vertical = reg == BECORE_MCSC_SC0_V_RATIO_REG ||
					reg == BECORE_MCSC_PC0_V_RATIO_REG;

			result = becore_mcsc_chain_ratio(vertical);
			break;
		}
		default:
			return -EINVAL;
		}
		if (value)
			*value = result;

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
		return input->dma + becore_rgbp_input_image_offset(profile);
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
		becore_rgbp_input_size(becore_rgbp_input_profile(becore));

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
 * One register of a parameters block, or -ENOENT if none of them reaches it.
 *
 * The blocks name values, not registers, so this is where the two meet: a
 * matrix coefficient is a 16-bit two's complement value in the low half of its
 * own word -- which is how every captured program writes one, negatives
 * included -- and the tone curve packs two samples per word with the
 * lower-numbered one low.  Anything
 * userspace never sent keeps whatever the recipe or the driver already put
 * there, which is what makes a parameters buffer additive rather than a
 * wholesale replacement of the program.
 */
static int becore_params_value(const struct becore_params_state *params,
			       u32 reg, u32 *value)
{
	u32 index;

	if (params->ccm_valid &&
	    reg >= BECORE_YUVP_CCM_MATRIX_FIRST &&
	    reg <= BECORE_YUVP_CCM_MATRIX_LAST) {
		index = (reg - BECORE_YUVP_CCM_MATRIX_FIRST) / 4;
		*value = (u16)params->ccm[index];
		return 0;
	}
	if (params->ccm_valid &&
	    reg >= BECORE_YUVP_CCM_OFFSET_FIRST &&
	    reg <= BECORE_YUVP_CCM_OFFSET_LAST) {
		index = (reg - BECORE_YUVP_CCM_OFFSET_FIRST) / 4;
		*value = (u16)params->ccm_offsets[index];
		return 0;
	}
	if (params->ltm_curve_valid &&
	    reg >= BECORE_YUVP_LTM_GMAP_FIRST &&
	    reg <= BECORE_YUVP_LTM_GMAP_LAST) {
		index = (reg - BECORE_YUVP_LTM_GMAP_FIRST) / 4 *
			BECORE_LTM_CURVE_PER_REG;
		*value = params->ltm_curve[index] |
			 ((u32)params->ltm_curve[index + 1] << 16);
		return 0;
	}
	/*
	 * The colour LUT's lattice is not a register and is emitted separately,
	 * but the gate in front of it is one: the driver's own default asserts
	 * BYPASS because there is no lattice, and a block that brings one
	 * clears it.  The two cannot disagree -- the same flag decides both.
	 */
	if (params->clut_valid && reg == BECORE_YUVP_CLUT_BYPASS_REG) {
		*value = 0;
		return 0;
	}

	return -ENOENT;
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
	const struct becore_params_state *params = &becore->params;
	u32 word;

	for (word = 0; word < shape->valid_words; word++) {
		u32 value;
		u32 reg;

		if (shape->mode == 0x00090000 && !(word & 1))
			continue;
		if ((shape->address_mask | shape->typed_mask) & BIT(word))
			continue;
		if (becore_shape_register(shape, word, &reg) ||
		    becore_params_value(params, reg, &value))
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
	const struct becore_yuvp_output_profile *input =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];
	u32 address_count = 0;
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
	if (becore->output.size < becore_gtnr_surface_size() ||
	    becore->gtnr_output.size != becore_gtnr_surface_size() ||
	    becore_gtnr_input.width != input->width ||
	    becore_gtnr_input.height != input->height ||
	    becore_gtnr_input.stride != becore_yuvp_output_stride(input) ||
	    becore_gtnr_output.width != input->width ||
	    becore_gtnr_output.height != input->height ||
	    becore_gtnr_output.stride != becore_yuvp_output_stride(input))
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
		    shape->generated_mask ||
		    (shape->address_mask & ~used_mask) ||
		    (shape->typed_mask & ~used_mask) ||
		    (shape->fixed_mask & ~used_mask) ||
		    (shape->address_mask & shape->typed_mask) ||
		    (shape->address_mask & shape->fixed_mask) ||
		    (shape->typed_mask & shape->fixed_mask))
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
				if (becore_gtnr_dma_value(typed_count, reg, NULL))
					return -EINVAL;
				typed_count++;
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

	if (address_count != 4 || typed_count != BECORE_GTNR_DMA_WORD_COUNT)
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
				if (becore_gtnr_dma_value(typed_count, reg, &value))
					return -EINVAL;
				put_unaligned_le32(value, payload + word * 4);
				typed_count++;
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
	if (becore->output.size < becore_yuvp_output_size(input) ||
	    becore->mcsc_output.size != becore_mcsc_output_size())
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
				if (becore_mcsc_dma_value(typed_count, reg,
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
				if (becore_mcsc_dma_value(typed_count, reg, transport,
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
 * taken from the profile this run resolved rather than from a constant, so
 * the fabric and the block cannot disagree about the frame.  That is why this
 * runs per frame from the run path and not once from the resume: a stream
 * holds one runtime-PM reference across all its frames, so a resume happens
 * before any run has chosen a profile.  Per frame is also where the vendor
 * programs a link.
 */
static void becore_c2serv_program_link(struct becore_device *becore)
{
	const struct becore_yuvp_output_profile *profile =
		becore_yuvp_output_profile(becore);
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
		u32 lines = n ? DIV_ROUND_UP(profile->height, 2) :
				profile->height;

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

static void becore_free_shared_input(void *data)
{
	struct becore_device *becore = data;
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
	}
}

static int becore_alloc_shared_input(struct becore_device *becore)
{
	unsigned int i;
	int ret;

	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_dma_buffer *input = &becore->inputs[i].buffer;

		input->size = becore_input_allocation_size();
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

	ret = devm_add_action_or_reset(becore->dev,
				       becore_free_shared_input, becore);
	return ret;

err_free:
	becore_free_shared_input(becore);
	return dev_err_probe(becore->dev, ret,
			     "cannot allocate Bayer input slot %u\n", i);
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
	size_t output_size = becore_yuvp_output_allocation_size();
	int ret;

	ret = becore_generated_tables_validate(becore->dev);
	if (ret)
		return ret;
	ret = becore_input_profiles_validate(becore->dev);
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
	ret = becore_alloc_dma_buffer(becore, &becore->output,
				      output_size, "YUVP output");
	if (ret)
		return ret;
	ret = becore_alloc_dma_buffer(becore, &becore->gtnr_output,
				      becore_gtnr_surface_size(), "GTNR output");
	if (ret)
		return ret;
	ret = becore_alloc_dma_buffer(becore, &becore->mcsc_output,
				      becore_mcsc_output_size(), "MCSC output");
	if (ret)
		return ret;
	becore->mcsc_dest_dma = becore->mcsc_output.dma;
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
	if (becore->reset_failed) {
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
				  becore_mcsc_output_active_size() - 1)) {
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
		becore_mcsc_output_active_size() : becore->active_output_size;
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
		   becore->c2serv_state[BECORE_C2SERV_MCSC].ready;
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
					becore_mcsc_output_size();
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

/* ---- processed NV21 capture queue -------------------------------------- */

static void becore_video_fill_pix(struct v4l2_pix_format *pix)
{
	pix->width = becore_mcsc_output.width;
	pix->height = becore_mcsc_output.height;
	pix->pixelformat = V4L2_PIX_FMT_NV21;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = becore_mcsc_output.stride;
	pix->sizeimage = becore_mcsc_output_active_size();
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
	struct v4l2_pix_format pix;

	becore_video_fill_pix(&pix);
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

	becore_video_fill_pix(&pix);
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
	    upper_32_bits(dma + becore_mcsc_output_active_size() - 1)) {
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
	int ret;

	mutex_lock(&becore->lock);
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
	becore_video_fill_pix(&f->fmt.pix);

	return 0;
}

static int becore_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct becore_device *becore = video_drvdata(file);

	if (vb2_is_busy(&becore->queue))
		return -EBUSY;

	becore_video_fill_pix(&f->fmt.pix);

	return 0;
}

static int becore_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || fsize->pixel_format != V4L2_PIX_FMT_NV21)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = becore_mcsc_output.width;
	fsize->discrete.height = becore_mcsc_output.height;

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

static const struct v4l2_file_operations becore_fops = {
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
	seq_printf(s, "params           ccm %u, ltm curve %u, colour LUT %u\n",
		   becore->params.ccm_valid, becore->params.ltm_curve_valid,
		   becore->params.clut_valid);
	seq_printf(s, "input_profile    %u requested, %u active, %zu bytes\n",
		   READ_ONCE(becore->input_profile),
		   becore->active_input_profile,
		   becore_rgbp_input_size(becore_rgbp_input_profile(becore)));
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

/* ---------------------------------------------------------------------------
 * The parameters node
 *
 * Under ADR 0009 the kernel owns the hardware description and the register
 * encoding; the per-frame image-quality *values* come from userspace through a
 * V4L2_BUF_TYPE_META_OUTPUT node, one typed block per hardware block.  Two of
 * them exist so far, and both are chosen because they are demonstrably live
 * policy rather than calibration: the colour matrix is the white balance's own
 * output and the tone curve is the exposure estimate's, and both move frame to
 * frame in a moving scene.
 *
 * A buffer never carries a register, an address or a command -- only values,
 * in the units the block is specified in.
 */

static const struct v4l2_isp_params_block_type_info
becore_params_block_info[] = {
	[EXYNOS_BECORE_PARAM_BLOCK_CCM] = {
		.size = sizeof(struct exynos_becore_params_ccm),
	},
	[EXYNOS_BECORE_PARAM_BLOCK_LTM_CURVE] = {
		.size = sizeof(struct exynos_becore_params_ltm_curve),
	},
	[EXYNOS_BECORE_PARAM_BLOCK_CLUT] = {
		.size = sizeof(struct exynos_becore_params_clut),
	},
};

static_assert(ARRAY_SIZE(becore_params_block_info) ==
	      EXYNOS_BECORE_PARAM_BLOCK_SENTINEL);

#define BECORE_PARAMS_BUFFER_SIZE \
	v4l2_isp_params_buffer_size(EXYNOS_BECORE_PARAMS_MAX_SIZE)

/*
 * Each row of the matrix has to sum to unity, which is what makes it preserve
 * neutrals; the vendor's own encoder guarantees it by renormalising the third
 * coefficient of each row.  A matrix that tints grey is far more likely to be
 * an arithmetic mistake upstream than an intention, so refuse it rather than
 * program it -- and refusing at buf_prepare tells userspace which buffer was
 * wrong, where a silently accepted one shows up as a colour cast three
 * abstraction layers away.
 */
static int becore_params_check_ccm(struct device *dev,
				   const struct exynos_becore_params_ccm *ccm)
{
	unsigned int row;

	for (row = 0; row < EXYNOS_BECORE_CCM_COEFFICIENTS / 3; row++) {
		s32 sum = ccm->matrix[row * 3] + ccm->matrix[row * 3 + 1] +
			  ccm->matrix[row * 3 + 2];

		if (sum != EXYNOS_BECORE_CCM_ONE) {
			dev_dbg(dev, "CCM row %u sums to %d, not %d\n", row,
				sum, EXYNOS_BECORE_CCM_ONE);
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * A tone curve that goes backwards inverts contrast over that interval, which
 * no tone mapper wants and which is what a sign or ordering error looks like.
 */
static int
becore_params_check_ltm_curve(struct device *dev,
			      const struct exynos_becore_params_ltm_curve *ltm)
{
	unsigned int i;

	for (i = 0; i < EXYNOS_BECORE_LTM_CURVE_POINTS; i++) {
		if (ltm->curve[i] > EXYNOS_BECORE_LTM_CURVE_ONE) {
			dev_dbg(dev, "tone curve point %u exceeds unity\n", i);
			return -EINVAL;
		}
		if (i && ltm->curve[i] < ltm->curve[i - 1]) {
			dev_dbg(dev, "tone curve decreases at point %u\n", i);
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * The lattice entries are the chroma the block outputs rather than an offset
 * to it, so there is no fill that leaves the picture alone and nothing here
 * can check that a table is *sensible*.  Two things it can check:
 *
 * The range, because the hardware field is ten bits and three of them share a
 * word -- a sample that did not fit would corrupt a neighbouring node rather
 * than itself.
 *
 * And the two ends of the grey axis.  Black and white have no hue, every
 * lattice the vendor ships is exactly neutral at both, and a lattice that is
 * not is what a stream written one sample out of step looks like.  The
 * interior of the diagonal is deliberately not checked: the vendor's own
 * tables are a count or two off neutral in the mid-greys, which is a tuning
 * choice rather than a mistake.
 */
static int becore_params_check_clut(struct device *dev,
				    const struct exynos_becore_params_clut *clut)
{
	static const unsigned int grey[] = { 0, EXYNOS_BECORE_CLUT_NODES - 1 };
	unsigned int i;

	for (i = 0; i < EXYNOS_BECORE_CLUT_NODES; i++) {
		if (clut->lut_u[i] > EXYNOS_BECORE_CLUT_MAX ||
		    clut->lut_v[i] > EXYNOS_BECORE_CLUT_MAX) {
			dev_dbg(dev, "colour LUT node %u exceeds %u\n", i,
				EXYNOS_BECORE_CLUT_MAX);
			return -EINVAL;
		}
	}

	for (i = 0; i < ARRAY_SIZE(grey); i++) {
		if (clut->lut_u[grey[i]] != EXYNOS_BECORE_CLUT_NEUTRAL ||
		    clut->lut_v[grey[i]] != EXYNOS_BECORE_CLUT_NEUTRAL) {
			dev_dbg(dev, "colour LUT node %u is not neutral\n",
				grey[i]);
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * Walk the blocks once.  `apply` distinguishes the buf_prepare pass, which
 * only says whether the buffer is acceptable, from the per-frame pass, which
 * installs it -- so that the two cannot drift apart into a buffer that
 * validates and then programs something else.
 */
static int becore_params_walk(struct becore_device *becore,
			      const struct v4l2_isp_params_buffer *config,
			      bool apply)
{
	size_t offset = 0;

	while (offset < config->data_size) {
		const struct v4l2_isp_params_block_header *header =
			(const void *)(config->data + offset);
		bool disable = header->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE;
		int ret;

		offset += header->size;
		switch (header->type) {
		case EXYNOS_BECORE_PARAM_BLOCK_CCM: {
			const struct exynos_becore_params_ccm *ccm =
				(const void *)header;

			if (disable) {
				if (apply)
					becore->params.ccm_valid = false;
				break;
			}
			ret = becore_params_check_ccm(becore->dev, ccm);
			if (ret)
				return ret;
			if (!apply)
				break;
			memcpy(becore->params.ccm, ccm->matrix,
			       sizeof(becore->params.ccm));
			memcpy(becore->params.ccm_offsets, ccm->offsets,
			       sizeof(becore->params.ccm_offsets));
			becore->params.ccm_valid = true;
			break;
		}
		case EXYNOS_BECORE_PARAM_BLOCK_LTM_CURVE: {
			const struct exynos_becore_params_ltm_curve *ltm =
				(const void *)header;

			if (disable) {
				if (apply)
					becore->params.ltm_curve_valid = false;
				break;
			}
			ret = becore_params_check_ltm_curve(becore->dev, ltm);
			if (ret)
				return ret;
			if (!apply)
				break;
			memcpy(becore->params.ltm_curve, ltm->curve,
			       sizeof(becore->params.ltm_curve));
			becore->params.ltm_curve_valid = true;
			break;
		}
		case EXYNOS_BECORE_PARAM_BLOCK_CLUT: {
			const struct exynos_becore_params_clut *clut =
				(const void *)header;

			/*
			 * Disabling this one really does switch the stage off,
			 * because bypass is what the driver has instead of a
			 * default lattice.
			 */
			if (disable) {
				if (apply)
					becore->params.clut_valid = false;
				break;
			}
			ret = becore_params_check_clut(becore->dev, clut);
			if (ret)
				return ret;
			if (!apply)
				break;
			memcpy(becore->params.clut_u, clut->lut_u,
			       sizeof(becore->params.clut_u));
			memcpy(becore->params.clut_v, clut->lut_v,
			       sizeof(becore->params.clut_v));
			becore->params.clut_valid = true;
			break;
		}
		default:
			/* v4l2_isp_params_validate_buffer() rejects these. */
			return -EINVAL;
		}
	}

	return 0;
}

/* Make one buffer the current configuration and hand it back. */
static void becore_params_install(struct becore_device *becore,
				  struct becore_params_buffer *buf)
{
	/*
	 * The blocks were checked at buf_prepare against this same walk, over
	 * this same kernel copy, so this cannot fail -- and if it somehow did,
	 * the frame would run on the configuration it already had.
	 */
	if (becore_params_walk(becore, buf->config, true))
		dev_warn_once(becore->dev, "a validated params buffer did not apply\n");

	/*
	 * Both are the driver's to fill, even on an output queue: vb2 zeroes
	 * the sequence at prepare and never copies the application's, and the
	 * queue asks for a monotonic timestamp.  The counter is not reset by a
	 * session, because it must never go backwards -- a per-session one
	 * does, and a consumer that has already seen a higher number has no way
	 * to tell that from a buffer arriving out of order.
	 */
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = becore->params_sequence++;
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

/*
 * Take the oldest queued buffer, if there is one, and make it the current
 * configuration.  A frame with nothing queued keeps the configuration it
 * already had, which is what lets userspace send only what changed.
 */
static void becore_params_consume(struct becore_device *becore)
{
	struct becore_params_buffer *buf;

	spin_lock_irq(&becore->queue_lock);
	buf = list_first_entry_or_null(&becore->queued_params,
				       struct becore_params_buffer, list);
	if (buf)
		list_del(&buf->list);
	spin_unlock_irq(&becore->queue_lock);

	if (buf)
		becore_params_install(becore, buf);
}

static void becore_params_return_all(struct becore_device *becore,
				     enum vb2_buffer_state state)
{
	struct becore_params_buffer *buf;
	struct becore_params_buffer *tmp;

	spin_lock_irq(&becore->queue_lock);
	list_for_each_entry_safe(buf, tmp, &becore->queued_params, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irq(&becore->queue_lock);
}

static int becore_params_queue_setup(struct vb2_queue *q, unsigned int *nbufs,
				     unsigned int *nplanes, unsigned int sizes[],
				     struct device *alloc_devs[])
{
	if (*nplanes) {
		if (*nplanes != 1 || sizes[0] < BECORE_PARAMS_BUFFER_SIZE)
			return -EINVAL;
		return 0;
	}
	*nplanes = 1;
	sizes[0] = BECORE_PARAMS_BUFFER_SIZE;

	return 0;
}

static int becore_params_buf_init(struct vb2_buffer *vb)
{
	struct becore_params_buffer *buf =
		to_becore_params_buffer(to_vb2_v4l2_buffer(vb));

	buf->config = kvzalloc(BECORE_PARAMS_BUFFER_SIZE, GFP_KERNEL);

	return buf->config ? 0 : -ENOMEM;
}

static void becore_params_buf_cleanup(struct vb2_buffer *vb)
{
	struct becore_params_buffer *buf =
		to_becore_params_buffer(to_vb2_v4l2_buffer(vb));

	kvfree(buf->config);
	buf->config = NULL;
}

static int becore_params_buf_prepare(struct vb2_buffer *vb)
{
	struct becore_device *becore = vb2_get_drv_priv(vb->vb2_queue);
	struct becore_params_buffer *buf =
		to_becore_params_buffer(to_vb2_v4l2_buffer(vb));
	const struct v4l2_isp_params_buffer *user = vb2_plane_vaddr(vb, 0);
	int ret;

	ret = v4l2_isp_params_validate_buffer_size(becore->dev, vb,
						   BECORE_PARAMS_BUFFER_SIZE);
	if (ret)
		return ret;

	/* Validate what will be programmed, not what may change underneath. */
	memcpy(buf->config, user, BECORE_PARAMS_BUFFER_SIZE);

	/*
	 * A buffer carrying no blocks changes nothing, and there is nothing in
	 * it to check -- so take it rather than insisting its payload length
	 * agree with a block list that is not there.  That also makes an
	 * all-zero buffer legal, which is what a queue's own buffers are before
	 * anyone fills them and what v4l2-compliance queues.
	 */
	if (buf->config->version != V4L2_ISP_PARAMS_VERSION_V0 &&
	    buf->config->version != V4L2_ISP_PARAMS_VERSION_V1)
		return -EINVAL;
	if (!buf->config->data_size)
		return 0;

	ret = v4l2_isp_params_validate_buffer(becore->dev, vb, buf->config,
					      becore_params_block_info,
					      ARRAY_SIZE(becore_params_block_info));
	if (ret)
		return ret;

	return becore_params_walk(becore, buf->config, false);
}

/*
 * A buffer waits for a frame only while there is a frame coming.  When the
 * capture queue is streaming, frames follow one another and holding the buffer
 * until the next one is what makes a configuration frame-accurate.  When it is
 * not, "the next frame" is whenever somebody triggers the offline loop, or
 * never -- so apply it and hand it back instead of leaving userspace waiting
 * on a frame nobody is going to run.
 *
 * From a work item rather than from here, because a buffer that comes back
 * DONE inside the QBUF that queued it is not something a caller expects: vb2
 * allows it, and v4l2-compliance warns about it and then loses track of the
 * queue.  The work runs immediately afterwards and drains whatever is waiting.
 */
static void becore_params_work(struct work_struct *work)
{
	struct becore_device *becore =
		container_of(work, struct becore_device, params_work);

	for (;;) {
		struct becore_params_buffer *buf;

		mutex_lock(&becore->lock);
		/*
		 * becore_run_stage() drops this lock while it waits, so a run
		 * can be in flight here -- and installing a configuration the
		 * frame in flight has already encoded past would return the
		 * buffer as though that frame had used it.  Leave it for the
		 * run to take, or for the next schedule if it does not.
		 */
		if (becore->video_streaming || becore->running) {
			mutex_unlock(&becore->lock);
			return;
		}
		spin_lock_irq(&becore->queue_lock);
		buf = list_first_entry_or_null(&becore->queued_params,
					       struct becore_params_buffer,
					       list);
		if (buf)
			list_del(&buf->list);
		spin_unlock_irq(&becore->queue_lock);
		if (buf)
			becore_params_install(becore, buf);
		mutex_unlock(&becore->lock);
		if (!buf)
			return;
	}
}

static void becore_params_buf_queue(struct vb2_buffer *vb)
{
	struct becore_device *becore = vb2_get_drv_priv(vb->vb2_queue);
	struct becore_params_buffer *buf =
		to_becore_params_buffer(to_vb2_v4l2_buffer(vb));

	spin_lock_irq(&becore->queue_lock);
	list_add_tail(&buf->list, &becore->queued_params);
	spin_unlock_irq(&becore->queue_lock);

	if (!READ_ONCE(becore->video_streaming))
		schedule_work(&becore->params_work);
}

/*
 * The end of a parameters session is where the configuration goes back to the
 * driver's own defaults -- not the start of one, because vb2 hands over
 * buffers queued before STREAMON before it calls start_streaming, so a reset
 * there would wipe what those buffers had just installed.  A capture session
 * does not reset it either: parameters belong to the parameters node.
 */
static void becore_params_stop_streaming(struct vb2_queue *q)
{
	struct becore_device *becore = vb2_get_drv_priv(q);

	/*
	 * Under the device lock, because becore_params_consume() takes a
	 * buffer off the list and completes it later in the same hold, and it
	 * is reached from a frame rather than from this queue.  Returning
	 * buffers without the lock can therefore find the list already empty
	 * and leave one active past this function, which is the one thing vb2
	 * forbids here.  The work item is cancelled afterwards: it takes the
	 * same lock, so it cannot be cancelled from under it.
	 */
	mutex_lock(&becore->lock);
	becore_params_return_all(becore, VB2_BUF_STATE_ERROR);
	memset(&becore->params, 0, sizeof(becore->params));
	mutex_unlock(&becore->lock);
	cancel_work_sync(&becore->params_work);
}

static const struct vb2_ops becore_params_vb2_ops = {
	.queue_setup = becore_params_queue_setup,
	.buf_init = becore_params_buf_init,
	.buf_cleanup = becore_params_buf_cleanup,
	.buf_prepare = becore_params_buf_prepare,
	.buf_queue = becore_params_buf_queue,
	.stop_streaming = becore_params_stop_streaming,
};

static int becore_params_querycap(struct file *file, void *priv,
				  struct v4l2_capability *cap)
{
	strscpy(cap->driver, "exynos-becore", sizeof(cap->driver));
	strscpy(cap->card, "zumapro BE-core parameters", sizeof(cap->card));

	return 0;
}

static void becore_params_fill_fmt(struct v4l2_meta_format *meta)
{
	memset(meta, 0, sizeof(*meta));
	meta->dataformat = V4L2_META_FMT_BECORE_PARAMS;
	meta->buffersize = BECORE_PARAMS_BUFFER_SIZE;
}

static int becore_params_g_fmt(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	becore_params_fill_fmt(&f->fmt.meta);

	return 0;
}

static int becore_params_enum_fmt(struct file *file, void *priv,
				  struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_META_FMT_BECORE_PARAMS;

	return 0;
}

static const struct v4l2_ioctl_ops becore_params_ioctl_ops = {
	.vidioc_querycap = becore_params_querycap,
	.vidioc_enum_fmt_meta_out = becore_params_enum_fmt,
	.vidioc_g_fmt_meta_out = becore_params_g_fmt,
	.vidioc_s_fmt_meta_out = becore_params_g_fmt,
	.vidioc_try_fmt_meta_out = becore_params_g_fmt,
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

static const struct video_device becore_params_template = {
	.name = "exynos-becore parameters",
	.fops = &becore_fops,
	.ioctl_ops = &becore_params_ioctl_ops,
	.release = video_device_release_empty,
	.device_caps = V4L2_CAP_META_OUTPUT | V4L2_CAP_STREAMING,
	.vfl_dir = VFL_DIR_TX,
};

static int becore_params_register(struct becore_device *becore)
{
	struct vb2_queue *q = &becore->params_queue;
	size_t type;
	int ret;

	/*
	 * v4l2_isp_params_validate_buffer() walks the blocks by their declared
	 * size, so a block type whose size is zero would never terminate.  The
	 * sizes below are all sizeof() of a real struct, and this is what keeps
	 * that true if someone adds a type and leaves the entry out.
	 */
	for (type = 0; type < ARRAY_SIZE(becore_params_block_info); type++) {
		if (becore_params_block_info[type].size <
		    sizeof(struct v4l2_isp_params_block_header))
			return dev_err_probe(becore->dev, -EINVAL,
					     "params block type %zu has no size\n",
					     type);
	}

	q->type = V4L2_BUF_TYPE_META_OUTPUT;
	q->io_modes = VB2_MMAP;
	q->dev = becore->dev;
	q->drv_priv = becore;
	q->ops = &becore_params_vb2_ops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->buf_struct_size = sizeof(struct becore_params_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &becore->params_lock;
	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	becore->params_vdev = becore_params_template;
	becore->params_vdev.v4l2_dev = &becore->v4l2_dev;
	becore->params_vdev.queue = q;
	becore->params_vdev.lock = &becore->params_lock;
	becore->params_vdev.entity.function = MEDIA_ENT_F_IO_V4L;
	video_set_drvdata(&becore->params_vdev, becore);

	becore->params_pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&becore->params_vdev.entity, 1,
				     &becore->params_pad);
	if (ret)
		return ret;

	ret = video_register_device(&becore->params_vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		media_entity_cleanup(&becore->params_vdev.entity);

	return ret;
}

static void becore_video_unregister(void *data)
{
	struct becore_device *becore = data;

	vb2_video_unregister_device(&becore->params_vdev);
	media_entity_cleanup(&becore->params_vdev.entity);
	vb2_video_unregister_device(&becore->vdev);
	/* The capture teardown above can have scheduled it one last time. */
	cancel_work_sync(&becore->params_work);
	media_device_unregister(&becore->mdev);
	media_entity_cleanup(&becore->vdev.entity);
	v4l2_ctrl_handler_free(&becore->ctrl_handler);
	v4l2_device_unregister(&becore->v4l2_dev);
	media_device_cleanup(&becore->mdev);
}

static int becore_video_register(struct becore_device *becore)
{
	struct v4l2_ctrl_handler *handler = &becore->ctrl_handler;
	struct vb2_queue *q = &becore->queue;
	int ret;

	becore->mdev.dev = becore->dev;
	strscpy(becore->mdev.model, "zumapro BE-core",
		sizeof(becore->mdev.model));
	media_device_init(&becore->mdev);
	becore->v4l2_dev.mdev = &becore->mdev;

	ret = v4l2_device_register(becore->dev, &becore->v4l2_dev);
	if (ret)
		goto err_mdev;

	ret = v4l2_ctrl_handler_init(handler, 2);
	if (ret)
		goto err_v4l2;
	becore->red_balance =
		v4l2_ctrl_new_std(handler, NULL, V4L2_CID_RED_BALANCE,
				  EXYNOS_BECORE_WBG_GAIN_MIN_Q12,
				  EXYNOS_BECORE_WBG_GAIN_MAX_Q12, 1,
				  EXYNOS_BECORE_WBG_RED_DEFAULT_Q12);
	becore->blue_balance =
		v4l2_ctrl_new_std(handler, NULL, V4L2_CID_BLUE_BALANCE,
				  EXYNOS_BECORE_WBG_GAIN_MIN_Q12,
				  EXYNOS_BECORE_WBG_GAIN_MAX_Q12, 1,
				  EXYNOS_BECORE_WBG_BLUE_DEFAULT_Q12);
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

	becore->vdev = becore_video_template;
	becore->vdev.v4l2_dev = &becore->v4l2_dev;
	becore->vdev.ctrl_handler = &becore->ctrl_handler;
	becore->vdev.queue = q;
	becore->vdev.lock = &becore->video_lock;
	becore->vdev.entity.function = MEDIA_ENT_F_IO_V4L;
	video_set_drvdata(&becore->vdev, becore);

	becore->vdev_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&becore->vdev.entity, 1,
				     &becore->vdev_pad);
	if (ret)
		goto err_ctrl;

	ret = video_register_device(&becore->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_entity;

	ret = becore_params_register(becore);
	if (ret)
		goto err_vdev;

	ret = media_device_register(&becore->mdev);
	if (ret)
		goto err_params;

	return devm_add_action_or_reset(becore->dev,
					becore_video_unregister, becore);

err_params:
	video_unregister_device(&becore->params_vdev);
	media_entity_cleanup(&becore->params_vdev.entity);
err_vdev:
	video_unregister_device(&becore->vdev);
err_entity:
	media_entity_cleanup(&becore->vdev.entity);
err_ctrl:
	v4l2_ctrl_handler_free(&becore->ctrl_handler);
err_v4l2:
	v4l2_device_unregister(&becore->v4l2_dev);
err_mdev:
	media_device_cleanup(&becore->mdev);
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
	becore->active_output_size = becore_active_output_size(becore);
	becore->active_capture_size = becore_mcsc_output_active_size();
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
	becore->active_output_dma = becore->output.dma;

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

	ret = becore_video_register(becore);
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
