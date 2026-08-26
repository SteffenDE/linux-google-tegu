/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google Zumapro camera back-end core: the hardware's register map.
 *
 * Everything here describes the silicon -- register offsets, block bases, the
 * spans of a range of registers, the masks, shifts and fixed-point positions
 * of their fields, the counts that come from a block's own shape, and the
 * static assertions that hold those against each other and against the UAPI.
 *
 * What *this driver* chose stays in the .c: timeouts, debugfs limits, the
 * buffers it sizes, and the words that are constants only because of a
 * decision it made elsewhere. A value the vendor writes is not one of those
 * -- it was recovered from the silicon like the register it goes in, and it
 * belongs beside that register, where it is the thing that explains it.
 */

#ifndef __EXYNOS_BECORE_REGS_H__
#define __EXYNOS_BECORE_REGS_H__

#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/math.h>
#include <linux/media/samsung/exynos-becore-config.h>
/* Not greppable but load-bearing: GENMASK_U32() expands to GENMASK_TYPE(u32,). */
#include <linux/types.h>

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
 * A producer names its destination as the consumer's local IP ID followed by
 * the consumer DMA number, which is how 0x1cc5 and DMA 1 becomes 0x1cc51.
 */
#define BECORE_C2SERV_DEST(ip, dma)	(((ip) << 4) | (dma))

#define BECORE_INT_FRAME_END		BIT(1)
#define BECORE_INT_CMDQ_HOLD		BIT(2)
#define BECORE_INT_EXPECTED		(BECORE_INT_FRAME_END | BECORE_INT_CMDQ_HOLD)

#define BECORE_CMDQ_HEADER_BYTES		16
#define BECORE_CMDQ_PAYLOAD_BYTES	64
#define BECORE_CMDQ_PAYLOAD_WORDS	(BECORE_CMDQ_PAYLOAD_BYTES / 4)
#define BECORE_CMDQ_MODE		0x9000

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
 * Sixteen registers that say which RGBP blocks run, named from Samsung RGBP
 * v1.20 and from Lyric's own register descriptors. None needs a value from the
 * capture: an enable the program leaves clear is zero and an asserted bypass
 * is one.
 *
 * OTF_CROP_CTRL is the one that reads backwards. Its only field is
 * RGB_DMSCCROP_BYPASS, whose vendor default is 1, and the capture clears it --
 * so zero here is what keeps the 48-column demosaic crop above *running*.
 * Do not fold it in with the asserted bypasses.
 *
 * UPSC_CTRL0 and SC_CTRL0 are control words rather than bare enables: bit 0
 * enables, bit 8 bypasses and two more disable clock gates. Zero leaves the
 * scaler neither enabled nor bypassed, which is what the capture does and what
 * the chain needs -- setting SC_CTRL0's bit 0 in the offline loop leaves the
 * main output byte-identical and empties the low-resolution branch below it,
 * because an enabled scaler scales rather than passing its raster through.
 *
 * DTP's mode selects a test pattern instead of the sensor's pixels. Zero is
 * the generator off; forcing it to 1 signs every stage of both processors with
 * a constant, which is the pattern rather than the picture.
 *
 * The two bypasses at +0x3100 and +0x3200 are DNS's and DMSC's, and zero keeps
 * both blocks running: asserting either changes that block's stream signature
 * and every one below it. DMSC's other two mode words select a cheaper
 * demosaic (LOW_POWER_EN) and a whole-block operating mode whose 1 reproduces
 * the bypass exactly, so zero is full-quality demosaic in the normal mode.
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
#define BECORE_RGBP_DTP_MODE_REG	(BECORE_RGBP_PHYS_BASE + 0x3000)
#define BECORE_RGBP_DNS_PHASE_REG	(BECORE_RGBP_PHYS_BASE + 0x31c4)
#define BECORE_RGBP_DMSC_BYPASS_REG	(BECORE_RGBP_PHYS_BASE + 0x3200)
#define BECORE_RGBP_DMSC_PHASE_REG	(BECORE_RGBP_PHYS_BASE + 0x3278)
/* LOW_POWER_EN at +0x3204 is a field of the parameters block, not a mode. */
#define BECORE_RGBP_DMSC_OP_MODE_REG	(BECORE_RGBP_PHYS_BASE + 0x3208)
#define BECORE_RGBP_DECOMP_BYPASS_REG	(BECORE_RGBP_PHYS_BASE + 0x3e00)
#define BECORE_RGBP_DECOMP_SIZE_REG	(BECORE_RGBP_PHYS_BASE + 0x3e08)
#define BECORE_RGBP_SC_CTRL0_REG	(BECORE_RGBP_PHYS_BASE + 0x4400)
#define BECORE_RGBP_GAMMALR_BYPASS_REG	(BECORE_RGBP_PHYS_BASE + 0x4600)
#define BECORE_RGBP_UPSC_CTRL0_REG	(BECORE_RGBP_PHYS_BASE + 0x4800)
#define BECORE_RGBP_GAMMAHR_BYPASS_REG	(BECORE_RGBP_PHYS_BASE + 0x4a00)
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
/*
 * The block's radial geometry, which describes the *crop* rather than the
 * raster YUVP reads: the fall-off reasons in the crop's coordinates and
 * @BECORE_YUVNR_BINNING is what converts a chain pixel into one.  Up here
 * with the rest of the block's geography because the generated-range table
 * names it, and that table is built long before the arithmetic that fills it.
 */
#define BECORE_YUVNR_BINNING		0x332c
#define BECORE_YUVNR_RADIAL_CENTER	0x3354
#define BECORE_YUVNR_BINNING_Q		1024
#define BECORE_YUVNR_BINNING_MASK	GENMASK(13, 0)
#define BECORE_YUVNR_BINNING_Y_SHIFT	14
#define BECORE_YUVNR_BUCKET_SHIFT	28
#define BECORE_YUVNR_CENTRE_MASK	GENMASK(14, 0)

/*
 * The temporal filter's gain curve: six knots two to a register, five slopes
 * one per register.  Up here for the same reason -- the range table names
 * both.
 */
#define BECORE_YUVNR_TNR_KNOTS		EXYNOS_BECORE_YUVNR_MCFP_LUT_POINTS
#define BECORE_YUVNR_TNR_KNOT_FIRST	0x33f8
#define BECORE_YUVNR_TNR_SLOPE_FIRST	0x340c
#define BECORE_YUVNR_TNR_SLOPE_MASK	GENMASK(13, 0)
#define BECORE_YUVNR_TNR_GAIN_MAX	4096

/*
 * The luma-gain curve's x grid: 32 knots, 0, 128, 256 ... 3840, 4096.
 *
 * `luma_gain_x` is a tuning field like any other -- it lives in the shipped
 * `YuvNrStaticParam` and is not interpolated -- so what makes it stateable is
 * that it does not move, on three independent counts: it is bit-identical in
 * all 34 shipped tuning files that carry this block, bit-identical in all 426
 * captured programs on three cameras, and equal to the value
 * `SetDefaultTuningCommon` compiles in.
 *
 * All three are needed, and the field beside it in the same message says why:
 * `std_lut_x` is filled by that same function and the shipped tunings carry
 * *four* distinct noise-curve axes between them. Fourteen of the 34 do hold
 * the compiled-in default, which is what makes it the wrong thing to argue
 * from on its own -- a value being the vendor's default says nothing about
 * whether a tuning file moves it.
 */
#define BECORE_YUVP_NR_LUMA_GRID_FIRST	(BECORE_YUVP_NR_BASE + 0x524)
#define BECORE_YUVP_NR_LUMA_GRID_LAST	(BECORE_YUVP_NR_BASE + 0x560)
#define BECORE_NR_LUMA_GRID_KNOTS	32
#define BECORE_NR_LUMA_GRID_STEP	128
#define BECORE_NR_LUMA_GRID_FULL_SCALE	4096
#define BECORE_NR_LUMA_GRID_PER_REG	2

static_assert((BECORE_YUVP_NR_LUMA_GRID_LAST -
	       BECORE_YUVP_NR_LUMA_GRID_FIRST) / 4 + 1 ==
	      BECORE_NR_LUMA_GRID_KNOTS / BECORE_NR_LUMA_GRID_PER_REG);

#define BECORE_NOISE_KNOTS		8
#define BECORE_NOISE_TABLE_REGS		(BECORE_NOISE_KNOTS / 2)
#define BECORE_NOISE_TABLE_LAST		((BECORE_NOISE_TABLE_REGS - 1) * 4)
#define BECORE_NOISE_SLOPE_SHIFT	11
#define BECORE_NOISE_SLOPE_MASK		GENMASK(12, 0)
#define BECORE_NOISE_SHIFT_NIBBLES	8
/* The knot registers are twelve bits, and a slope has to describe what fits. */
#define BECORE_NOISE_KNOT_MAX		4095
/*
 * Where a scaler starts. RGBP's SC, MCSC's POLY_SC0 and its POST_PC0 chroma
 * converter each put two 20-bit init phase offsets at the same place in their
 * register map. All six are zero in every capture -- but so is the ratio
 * beside them in all but four, and where POLY_SC0 does stretch it writes half
 * that ratio. becore_scaler_init_phase() is the rule; POLY_SC0 and POST_PC0
 * run at unity here because DJAG does the scaling, so it comes out zero.
 * Nothing here is scene-dependent.
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
/*
 * And the two channels that *are* used are quiesced first, by the same pair of
 * registers, before the typed DMA words below configure and start them. The
 * value is not a state the hardware ends in -- the typed words overwrite both
 * within the same command buffer, which is why becore_override_check() refuses
 * these two registers outright -- so what the driver states here is the order,
 * not a setting.
 */
#define BECORE_MCSC_RDMA_R0_FIRST	(BECORE_MCSC_PHYS_BASE + 0x1800)
#define BECORE_MCSC_WDMA_W0_FIRST	(BECORE_MCSC_PHYS_BASE + 0x2000)
/*
 * MCSC has five output ports, and each carries a poly-phase scaler at
 * +0x5N00 and a post-processing chroma converter at +0x6N00. The driver drives
 * port 0 alone: its POST_PC0 is off, and so are the four ports' converters
 * after it. Lyric's descriptors name port 0's pair; the recipe's own header
 * alternates +0x51N00 and +0x61N00 for N = 1..4, which is what places the
 * other four. The coefficient control belongs to the converter that is off,
 * and setting it alone changes nothing; with the converter on, it does.
 */
#define BECORE_MCSC_PC0_CTRL_REG	(BECORE_MCSC_PHYS_BASE + 0x6000)
#define BECORE_MCSC_PC0_COEFF_CTRL_REG	(BECORE_MCSC_PHYS_BASE + 0x6020)
#define BECORE_MCSC_PC1_CTRL_REG	(BECORE_MCSC_PHYS_BASE + 0x6100)
#define BECORE_MCSC_PC4_CTRL_REG	(BECORE_MCSC_PHYS_BASE + 0x6400)
/*
 * Two more MCSC words that describe the job rather than tune it: no hardware
 * frame connector drives our frame start, and the pipeline is not a secure
 * one, so its sequence id is zero.
 */
#define BECORE_MCSC_SECU_SEQID_REG	(BECORE_MCSC_PHYS_BASE + 0x0b00)
#define BECORE_MCSC_HWFC_START_REG	(BECORE_MCSC_PHYS_BASE + 0x704c)
#define BECORE_MCSC_DJAG_BASE		(BECORE_MCSC_PHYS_BASE + 0x4000)
#define BECORE_MCSC_DJAG_CTRL_REG	(BECORE_MCSC_DJAG_BASE + 0x000)
#define BECORE_MCSC_DJAG_PS_FIRST	(BECORE_MCSC_DJAG_BASE + 0x01c)
#define BECORE_MCSC_DJAG_PS_LAST	(BECORE_MCSC_DJAG_BASE + 0x024)
#define BECORE_MCSC_DJAG_TUNE_FIRST	(BECORE_MCSC_DJAG_BASE + 0x050)
#define BECORE_MCSC_DJAG_TUNE_LAST	(BECORE_MCSC_DJAG_BASE + 0x068)
#define BECORE_MCSC_DJAG_RECOM_CTRL_REG	(BECORE_MCSC_DJAG_BASE + 0x080)
#define BECORE_MCSC_DJAG_RECOM_WEIGHT_REG (BECORE_MCSC_DJAG_BASE + 0x088)
/* The radial gain inside that stage, off with it: centre, two biquad factors
 * and its own enable. Setting all four in the offline loop leaves the frame
 * byte-identical, which is what a gain inside a stage that does not run does.
 */
#define BECORE_MCSC_DJAG_RADIAL_CENTRE_REG (BECORE_MCSC_DJAG_BASE + 0x0a4)
#define BECORE_MCSC_DJAG_RADIAL_FIRST	(BECORE_MCSC_DJAG_BASE + 0x0ac)
#define BECORE_MCSC_DJAG_RADIAL_LAST	(BECORE_MCSC_DJAG_BASE + 0x0b4)
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
#define BECORE_RGBP_DNS_BIQUAD_REG	(BECORE_RGBP_DNS_BASE + 0x1a8)
#define BECORE_RGBP_DNS_BIQUAD_MAX	7
/* The subtracter arrives at Q8, so an octave is 256 and the fraction is kept. */
#define BECORE_RGBP_DNS_BIQUAD_Q	256
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
/*
 * YUVP carries the same colour-space converter three times over, and the two
 * copies here are the ones the 12-bit Q13 block above does not cover.
 *
 * YUV_YUVTORGB at +0x3900 and its _ZUMA twin at +0x6a00 are the BT.601 inverse
 * at Q12 in 14-bit signed fields. Their nine coefficients are in the same
 * order as the forward block's -- input slow, output fast, so the vendor's
 * coeff_<in>_<out> names run 0_0, 0_1, 0_2, 1_0 and the forward block's run
 * r1, r2, r3, g1 -- and the subscripts still swap below, because the driver's
 * two matrix tables are stored in opposite conventions and not because the two
 * register blocks disagree. Two things after them are not coefficients:
 * LSHIFT is the post-matrix shift, which
 * Lyric's TranslateCsc writes as a bare 1 whatever the tuning says, and the
 * three OFFSETs are the pedestal the block subtracts from its input -- zero
 * for luma and minus a half for each chroma channel, in a 13-bit signed field.
 * Half of the block's own Q rather than a literal 0x800, so the same rounding
 * that produces the matrix produces the pedestal.
 */
#define BECORE_YUVP_YUV2RGB_BASE	(BECORE_YUVP_PHYS_BASE + 0x3900)
#define BECORE_YUVP_YUV2RGB_BYPASS_REG	(BECORE_YUVP_YUV2RGB_BASE + 0x00)
/* +0x04 is not a register: the vendor's table jumps bypass to coeff_0_0. */
#define BECORE_YUVP_YUV2RGB_FIRST	(BECORE_YUVP_YUV2RGB_BASE + 0x08)
#define BECORE_YUVP_YUV2RGB_LAST	(BECORE_YUVP_YUV2RGB_BASE + 0x38)
#define BECORE_YUVP_YUV2RGB_ZUMA_BASE	(BECORE_YUVP_PHYS_BASE + 0x6a00)
#define BECORE_YUVP_YUV2RGB_ZUMA_BYPASS_REG (BECORE_YUVP_YUV2RGB_ZUMA_BASE + 0x00)
#define BECORE_YUVP_YUV2RGB_ZUMA_FIRST	(BECORE_YUVP_YUV2RGB_ZUMA_BASE + 0x08)
#define BECORE_YUVP_YUV2RGB_ZUMA_LAST	(BECORE_YUVP_YUV2RGB_ZUMA_BASE + 0x38)
#define BECORE_YUVP_YUV2RGB_Q		12
#define BECORE_YUVP_YUV2RGB_FIELD_MASK	GENMASK(13, 0)
#define BECORE_YUVP_YUV2RGB_OFFSET_MASK	GENMASK(12, 0)
#define BECORE_YUVP_YUV2RGB_LSHIFT	1
/*
 * RGB_RGBTOYUV420 at +0x7400 is the forward matrix again, but this is the copy
 * at the end of the chain that produces the frame, so it runs wider than the
 * 4:4:4 one and it subsamples: Q14 coefficients in 16-bit fields, 14-bit
 * clipping limits, and a chroma decimation the other copy has no registers
 * for.
 *
 * Its first register reads 0x100 and that bit is what blocked the block. The
 * vendor's own register descriptor calls the whole word `bypass`, and that is
 * a partial name rather than a wrong value: bit 0 is the bypass and bit 8 is
 * the output bit depth. Lyric's TranslateCsc for this output writes bit 8 as
 * "output bit depth == 10", and it is set because YUVP hands MCSC a P010
 * surface. The same function decides the dither below from the same pair of
 * depths -- the matrix accumulates at 14 bits and the frame leaves at 10 --
 * which is why the two blocks are stated together and why the dither is not
 * simply "a block that runs".
 *
 * Only this copy of the converter has that second field. The translators for
 * the other three -- YUVP's +0x3a00 and both inverse blocks, and RGBP's
 * +0x3b00 -- read-modify-write bit 0 and touch nothing else, so calling their
 * word a bypass is as much as the evidence supports and no more.
 *
 * Three of the twenty-three are GetDefaultCsc literals whose *value* is read
 * straight out of the vendor's defaults and whose *meaning* no field table
 * gives: LS = 1, UV_COEFF = 0x02010201 and VER_SAMPLING_POSITION = 8. One
 * statement of GetDefaultCsc writes the first two together. LS being 1 here
 * and 0 in the Q13 copy, whose coefficients sit one binary point lower, makes
 * a post-matrix shift the obvious reading -- but the Q12 inverse block's own
 * shift is also 1, so the correlation is not a rule and the driver does not
 * claim one.
 */
#define BECORE_YUVP_CSC420_BASE		(BECORE_YUVP_PHYS_BASE + 0x7400)
#define BECORE_YUVP_CSC420_FIRST	(BECORE_YUVP_CSC420_BASE + 0x00)
#define BECORE_YUVP_CSC420_LAST		(BECORE_YUVP_CSC420_BASE + 0x54)
#define BECORE_YUVP_CSC420_VER_SAMPLING_REG (BECORE_YUVP_CSC420_BASE + 0xa0)
#define BECORE_YUVP_CSC420_Q		14
#define BECORE_YUVP_CSC420_FIELD_MASK	GENMASK(15, 0)
#define BECORE_YUVP_CSC420_LIMIT_MASK	GENMASK(14, 0)
#define BECORE_YUVP_CSC420_MAX		0x3fff	/* full range at this width */
#define BECORE_YUVP_CSC420_LS		1
#define BECORE_YUVP_CSC420_CTRL_BYPASS	BIT(0)
#define BECORE_YUVP_CSC420_CTRL_OUT10	BIT(8)
/*
 * GetDefaultCsc's two remaining literals, which no field table explains: four
 * bytes of chroma decimation weight, and where the chroma sample sits between
 * the two luma rows it comes from.
 */
#define BECORE_YUVP_CSC420_UV_COEFF	0x02010201
#define BECORE_YUVP_CSC420_VER_SAMPLING	8
#define BECORE_YUVP_DITHER420_BASE	(BECORE_YUVP_PHYS_BASE + 0x3c00)
#define BECORE_YUVP_DITHER420_FIRST	(BECORE_YUVP_DITHER420_BASE + 0x00)
#define BECORE_YUVP_DITHER420_LAST	(BECORE_YUVP_DITHER420_BASE + 0x04)
#define BECORE_RGBP_CHROMA_LPF_BASE	(BECORE_RGBP_PHYS_BASE + 0x3c00)
#define BECORE_RGBP_CHROMA_LPF_CTRL_REG	(BECORE_RGBP_CHROMA_LPF_BASE + 0x00)
#define BECORE_RGBP_CHROMA_LPF_FIRST	(BECORE_RGBP_CHROMA_LPF_BASE + 0x08)
#define BECORE_RGBP_CHROMA_LPF_LAST	(BECORE_RGBP_CHROMA_LPF_BASE + 0x0c)
/*
 * The block's own bypass. It is asserted unless a parameters block brings
 * tuning, and that it does something is measured rather than assumed: forcing
 * it through the debugfs override over the vendor's own tuning softens the
 * picture's detail and leaves its exposure and colour alone.
 */
#define BECORE_YUVP_SHARPEN_BYPASS_REG	(BECORE_YUVP_PHYS_BASE + 0x5000)
/*
 * The sharpener's two geometry words, which describe the *crop* and how a
 * chain pixel maps into it -- the same pair YUVNR carries, and the same
 * conversion: BECORE_YUVNR_BINNING is 1024 x crop / chain and this step is the
 * identical ratio at eight fractional bits instead of ten.
 *
 * SENSOR is the crop's size with the height in the high half, the way DECOMP's
 * frame size is and the way CHAIN_SRC_IMG_SIZE is not. STEP packs the vertical
 * ratio above the horizontal one, which is only visible where the two differ:
 * of the sixteen captured camera/geometry pairs exactly one does, the
 * ultrawide's 4208 x 2368 readout at 0x018c018b, and it is the vertical axis
 * that reads 0x18c.
 *
 * Both hold on all sixteen, and the second was found by running the offline
 * loop at a second chain raster on hardware: the driver replayed 0x01000100
 * where the vendor writes 0x01980198, which is a word that would have been
 * wrong at every readout but the one it was captured at.
 */
#define BECORE_YUVP_SHARPEN_SENSOR_REG	(BECORE_YUVP_PHYS_BASE + 0x500c)
#define BECORE_YUVP_SHARPEN_STEP_REG	(BECORE_YUVP_PHYS_BASE + 0x5014)
#define BECORE_SHARPEN_STEP_Q		256
#define BECORE_SHARPEN_STEP_MASK	GENMASK(15, 0)
/*
 * Three registers of the sharpener that nothing writes. Each falls in a gap
 * between two runs of exynos-becore-sharpen.h -- the table
 * tools/camera-sharpener-encode.py recovers from the vendor's own
 * SharpenerBlock::ConfigureWith -- so the vendor programs the register either
 * side of each and not these: CONT_CONFIG3 but not _CONFIG4, SKIN_FACE_
 * BRIGHTNESS_GAIN but not the SKIN_B one after it, APPLY_DESAT_1 but not
 * _DESAT_3. They are invariant across all 426 captured programs, and forcing
 * each to a saturated value in the offline loop leaves the frame
 * byte-identical, so they are the block's reset state rather than a tuning
 * whose zero happens to be neutral.
 */
#define BECORE_YUVP_SHARPEN_CONT_CONFIG4_REG (BECORE_YUVP_PHYS_BASE + 0x5930)
#define BECORE_YUVP_SHARPEN_SKIN_B_GAIN_REG (BECORE_YUVP_PHYS_BASE + 0x5958)
#define BECORE_YUVP_SHARPEN_APPLY_DESAT3_REG (BECORE_YUVP_PHYS_BASE + 0x5aa8)
#define BECORE_YUVP_LPF_FIRST		(BECORE_YUVP_PHYS_BASE + 0x5100)
#define BECORE_YUVP_LPF_LAST		(BECORE_YUVP_PHYS_BASE + 0x514c)
#define BECORE_YUVP_LPF_NORM_REG	(BECORE_YUVP_PHYS_BASE + 0x5150)
#define BECORE_SHARPEN_TAPS_PER_REG	2
/*
 * The sharpener's per-scene inputs: a segmentation confidence map, five face
 * rectangles and five regions of interest. Each range is contiguous and the
 * tuning either side of it is not, so the extents carry the claim.
 */
#define BECORE_YUVP_CONFMAP_FIRST	(BECORE_YUVP_PHYS_BASE + 0x5338)
#define BECORE_YUVP_CONFMAP_LAST	(BECORE_YUVP_PHYS_BASE + 0x54bc)
#define BECORE_YUVP_FACE_REGION_FIRST	(BECORE_YUVP_PHYS_BASE + 0x595c)
#define BECORE_YUVP_FACE_REGION_LAST	(BECORE_YUVP_PHYS_BASE + 0x59a8)
#define BECORE_YUVP_ROI_REGION_FIRST	(BECORE_YUVP_PHYS_BASE + 0x59f8)
#define BECORE_YUVP_ROI_REGION_LAST	(BECORE_YUVP_PHYS_BASE + 0x5a44)
#define BECORE_YUVP_CONFMAP_WORDS	98
#define BECORE_YUVP_REGION_WORDS	20

/*
 * Four of the six edges have a zero on both sides of them in every captured
 * program, so a range one register wide in the wrong direction would emit a
 * program identical to today's and only the register names would catch it.
 * The counts are a second statement of the same fact.
 */
static_assert((BECORE_YUVP_CONFMAP_LAST - BECORE_YUVP_CONFMAP_FIRST) / 4 + 1 ==
	      BECORE_YUVP_CONFMAP_WORDS);
static_assert((BECORE_YUVP_FACE_REGION_LAST - BECORE_YUVP_FACE_REGION_FIRST) /
	      4 + 1 == BECORE_YUVP_REGION_WORDS);
static_assert((BECORE_YUVP_ROI_REGION_LAST - BECORE_YUVP_ROI_REGION_FIRST) /
	      4 + 1 == BECORE_YUVP_REGION_WORDS);

/*
 * Ten 16-bit seeds over six registers: SEED0_0..2 then SEED1_0..2, packed two
 * to a register, so the third register of each group carries one seed and a
 * reserved half.
 */
#define BECORE_YUVP_NOISE_SEED_FIRST	(BECORE_YUVP_PHYS_BASE + 0x5610)
#define BECORE_YUVP_NOISE_SEED_LAST	(BECORE_YUVP_PHYS_BASE + 0x5624)
#define BECORE_YUVP_NOISE_SEEDS		5
#define BECORE_YUVP_NOISE_SEED_STEP	11111
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
 * Three gates YUVP leaves in a state that does not depend on the scene, named
 * from Samsung YUVP v1.20 and Lyric's descriptors: DTP asserts its own BYPASS
 * -- it is a test-pattern generator, and nothing wants one -- and both output
 * FIFOs are left disabled. Lyric calls them COUTFIFO_GSE and COUTFIFO_MCSC,
 * which says what each would feed on the fly; neither path exists here,
 * because the frame reaches MCSC through memory or through the VOTF fabric
 * rather than through a FIFO. Enabling the MCSC one in the offline loop leaves
 * the frame byte-identical, which is what a FIFO nothing drains does.
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
 * So the block runs, and all 414 of its registers are now stated here: nothing
 * of it is left in the recipe. Of its two curves the guide curve is a
 * parameters block, with the identity ramp below under it, and the tone-adjust
 * curve is the vendor's own sampled S-curve.
 *
 * That the second curve and the CONFIG words could be stated at all is an
 * invariance result rather than a decode: over 426 programs, three cameras and
 * eighteen sessions the block has exactly 65 per-frame registers and 64 of
 * them are the guide curve. What looked like live per-frame tuning is a static
 * profile, and it retires the way DJAG did rather than waiting for an IPA.
 *
 * Lyric's embedded register descriptors name the whole range rgb_diablo_ltm_*,
 * and that is what fixes where each stated run ends: the gain LUT does not
 * stop at 0x65fc -- that is only where the captured program's header ended --
 * it runs to 0x66d4, which is where 122 more unity entries put it.
 *
 *   0x6000  ltm_enable                    the block runs
 *   0x6004  enable_*, lumacalc_*_weight   GetDefaultLtm's literals
 *   0x6014  lumacalc_rgby_coeff_r/g/b     BT.601 luma at Q12
 *   0x6120  slcgrid_*                     9 words, from the image size
 *   0x61a4  trans_slope_frac_bit, _bias   from the grid this driver writes
 *   0x61ac  trans_scale_lut_00..08        17 entries, two to a register
 *   0x61d0  toneadj_use_output_luma       set
 *   0x61d4  toneadj_lut_000..064          the shipped S-curve, 129 samples
 *   0x62d8  crecon_input_luma_thres       zero: a threshold, not a fill
 *   0x62dc  crecon_satctrl_lut            258 entries, all zero
 *   0x64e0  crecon_luma_lut               130 entries of unity Q8
 *   0x65e4  crecon_gain_lut               122 entries of unity Q8
 *
 * GetDefaultLtm fills the luma and gain LUTs with 0x0100, clears the
 * saturation LUT and holds the enables and luma weights above, so all of that
 * is the vendor's own compiled-in default and not an artefact of the scene
 * this was captured from -- and none of it differs between the rear and front
 * cameras, where the four grid steps do.
 */
#define BECORE_YUVP_COUTFIFO_GSE_EN_REG	(BECORE_YUVP_PHYS_BASE + 0x1200)
#define BECORE_YUVP_COUTFIFO_MCSC_EN_REG (BECORE_YUVP_PHYS_BASE + 0x1400)
#define BECORE_YUVP_DTP_BYPASS_REG	(BECORE_YUVP_PHYS_BASE + 0x3000)
#define BECORE_YUVP_LTM_BASE		(BECORE_YUVP_PHYS_BASE + 0x6000)
#define BECORE_YUVP_LTM_ENABLE_REG	(BECORE_YUVP_LTM_BASE + 0x000)
#define BECORE_YUVP_LTM_LUMA_FIRST	(BECORE_YUVP_LTM_BASE + 0x014)
#define BECORE_YUVP_LTM_LUMA_LAST	(BECORE_YUVP_LTM_BASE + 0x01c)
#define BECORE_YUVP_LTM_GRID_FIRST	(BECORE_YUVP_LTM_BASE + 0x120)
#define BECORE_YUVP_LTM_GRID_LAST	(BECORE_YUVP_LTM_BASE + 0x140)
#define BECORE_YUVP_LTM_CONFIG_FIRST	(BECORE_YUVP_LTM_BASE + 0x004)
#define BECORE_YUVP_LTM_CONFIG_LAST	(BECORE_YUVP_LTM_BASE + 0x010)
#define BECORE_YUVP_LTM_TRANS_FIRST	(BECORE_YUVP_LTM_BASE + 0x1a4)
#define BECORE_YUVP_LTM_TRANS_LAST	(BECORE_YUVP_LTM_BASE + 0x1d0)
#define BECORE_YUVP_LTM_TONEADJ_FIRST	(BECORE_YUVP_LTM_BASE + 0x1d4)
#define BECORE_YUVP_LTM_TONEADJ_LAST	(BECORE_YUVP_LTM_BASE + 0x2d4)
/* Two entries to a register, which is the LUT's own shape. */
#define BECORE_LTM_TONEADJ_PER_REG	2
/*
 * 129 samples over 65 registers, so the last register's high half is padding
 * and not a 130th entry: TranslateLtmToneAdjust writes kToneAdjustLutEntryCnt
 * shorts and stops.
 */
static_assert((BECORE_YUVP_LTM_TONEADJ_LAST -
	       BECORE_YUVP_LTM_TONEADJ_FIRST) / 4 + 1 ==
	      DIV_ROUND_UP(EXYNOS_BECORE_LTM_TONE_ADJUST_POINTS,
			   BECORE_LTM_TONEADJ_PER_REG));

#define BECORE_YUVP_LTM_LUMA_THRES_REG	(BECORE_YUVP_LTM_BASE + 0x2d8)
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
 * DIABLO_CCM: a gate, nine coefficients and three offsets.
 *
 * The nine are the live AWB matrix -- one payload per frame, and per-frame in
 * the invariance census -- so under ADR 0009 the kernel writes an identity and
 * a parameters block carries the real one.  Row-major, each row summing to
 * %EXYNOS_BECORE_CCM_ONE, which the captured matrix does on all three rows.
 *
 * The identity is load-bearing rather than decorative, and that is measured
 * rather than assumed: the config's bit 0 is clear in every captured program
 * and ApplyDefaults never sets it, which reads like a disabled block -- but
 * zeroing the nine coefficients through the debugfs override changes the
 * picture, so the block runs whatever that bit means.  Writing nothing here
 * would leave nine zeros in a live stage.
 *
 * The gate and the offsets do not move: the offsets are nominally live
 * (AwbFrameData floats 9..11) but zero in every captured program.
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
 * YUVP's DEGAMMARGB: the inverse of RGBP's forward square-root encode, laid
 * out exactly as the forward gamma below it -- two gates, one table of 32
 * pair registers closed by a delta-encoded last knot, then the shared grid --
 * with one table rather than three, because a fixed inverse has no
 * per-channel form.
 */
#define BECORE_YUVP_DEGAMMA_BASE	(BECORE_YUVP_PHYS_BASE + 0x3f00)
#define BECORE_YUVP_DEGAMMA_GATE_FIRST	(BECORE_YUVP_DEGAMMA_BASE + 0x000)
#define BECORE_YUVP_DEGAMMA_GATE_LAST	(BECORE_YUVP_DEGAMMA_BASE + 0x004)
#define BECORE_YUVP_DEGAMMA_TBL_FIRST	(BECORE_YUVP_DEGAMMA_BASE + 0x00c)
#define BECORE_YUVP_DEGAMMA_TBL_LAST	(BECORE_YUVP_DEGAMMA_BASE + 0x08c)
#define BECORE_YUVP_DEGAMMA_X_LOW_FIRST	(BECORE_YUVP_DEGAMMA_BASE + 0x1c0)
#define BECORE_YUVP_DEGAMMA_X_LOW_LAST	(BECORE_YUVP_DEGAMMA_BASE + 0x1ec)
#define BECORE_YUVP_DEGAMMA_X_HIGH_FIRST (BECORE_YUVP_DEGAMMA_BASE + 0x200)
#define BECORE_YUVP_DEGAMMA_X_HIGH_LAST	(BECORE_YUVP_DEGAMMA_BASE + 0x250)
/* How many knots the toe covers before the exact square takes over. */
#define BECORE_YUVP_DEGAMMA_TOE_KNOTS	15

/*
 * YUVP's forward GAMMARGB: the creative tone curve, and the grid it is
 * sampled on.
 *
 * It is the same hardware primitive as RGBP's forward gamma one IP upstream
 * -- 65 knots, two per register with the lower-numbered one low, and a last
 * knot stored as its distance from the previous one because it would need
 * 1 << Q exactly -- and it does a completely different job.  RGBP's is a
 * fixed square-root encode that moves linear light into a gamma domain;
 * DEGAMMARGB undoes it; this one is where the picture is actually graded, so
 * its three per-channel tables are policy and belong to userspace.
 *
 * The grid does not: it is at Q14 what RGBP's is at Q12, and it says where a
 * curve is measured rather than what the curve does.  Both it and the green
 * table have a genuine four-register reserved hole in them, which is why the
 * ranges below are named rather than strided.
 */
#define BECORE_YUVP_GAMMA_BASE		(BECORE_YUVP_PHYS_BASE + 0x4200)
#define BECORE_YUVP_GAMMA_GATE_FIRST	(BECORE_YUVP_GAMMA_BASE + 0x000)
#define BECORE_YUVP_GAMMA_GATE_LAST	(BECORE_YUVP_GAMMA_BASE + 0x004)
#define BECORE_YUVP_GAMMA_R_FIRST	(BECORE_YUVP_GAMMA_BASE + 0x00c)
#define BECORE_YUVP_GAMMA_R_LAST	(BECORE_YUVP_GAMMA_BASE + 0x088)
#define BECORE_YUVP_GAMMA_R_DELTA_REG	(BECORE_YUVP_GAMMA_BASE + 0x08c)
#define BECORE_YUVP_GAMMA_G_LOW_FIRST	(BECORE_YUVP_GAMMA_BASE + 0x0a0)
#define BECORE_YUVP_GAMMA_G_LOW_LAST	(BECORE_YUVP_GAMMA_BASE + 0x0ec)
#define BECORE_YUVP_GAMMA_G_HIGH_FIRST	(BECORE_YUVP_GAMMA_BASE + 0x100)
#define BECORE_YUVP_GAMMA_G_HIGH_LAST	(BECORE_YUVP_GAMMA_BASE + 0x12c)
#define BECORE_YUVP_GAMMA_G_DELTA_REG	(BECORE_YUVP_GAMMA_BASE + 0x130)
#define BECORE_YUVP_GAMMA_B_FIRST	(BECORE_YUVP_GAMMA_BASE + 0x138)
#define BECORE_YUVP_GAMMA_B_LAST	(BECORE_YUVP_GAMMA_BASE + 0x1b4)
#define BECORE_YUVP_GAMMA_B_DELTA_REG	(BECORE_YUVP_GAMMA_BASE + 0x1b8)
#define BECORE_YUVP_GAMMA_X_LOW_FIRST	(BECORE_YUVP_GAMMA_BASE + 0x1c0)
#define BECORE_YUVP_GAMMA_X_LOW_LAST	(BECORE_YUVP_GAMMA_BASE + 0x1ec)
#define BECORE_YUVP_GAMMA_X_HIGH_FIRST	(BECORE_YUVP_GAMMA_BASE + 0x200)
#define BECORE_YUVP_GAMMA_X_HIGH_LAST	(BECORE_YUVP_GAMMA_BASE + 0x250)
#define BECORE_YUVP_GAMMA_Q		14
/* Two knots per register, and a last knot that has a register to itself. */
#define BECORE_GAMMA_KNOTS_PER_REG	2
#define BECORE_YUVP_GAMMA_G_SPLIT_KNOT	40

static_assert(EXYNOS_BECORE_GAMMA_POINTS == BECORE_RGBP_GAMMA_KNOTS);
/*
 * Every knot but the last is half of a register.  An odd count would divide
 * silently and drop one, because the packing counts registers rather than
 * knots.
 */
static_assert((EXYNOS_BECORE_GAMMA_POINTS - 1) % BECORE_GAMMA_KNOTS_PER_REG ==
	      0);
static_assert(EXYNOS_BECORE_GAMMA_ONE == 1 << BECORE_YUVP_GAMMA_Q);
/*
 * Each range has to hold exactly the knots the table below says it starts at,
 * or the packing would run off the end of a curve. The green table's hole is
 * the reason this is worth asserting rather than reading: it splits at knot 40
 * and nothing about the addresses says so.
 */
static_assert((BECORE_YUVP_GAMMA_R_LAST - BECORE_YUVP_GAMMA_R_FIRST) / 4 + 1 ==
	      (EXYNOS_BECORE_GAMMA_POINTS - 1) / BECORE_GAMMA_KNOTS_PER_REG);
static_assert((BECORE_YUVP_GAMMA_B_LAST - BECORE_YUVP_GAMMA_B_FIRST) / 4 + 1 ==
	      (EXYNOS_BECORE_GAMMA_POINTS - 1) / BECORE_GAMMA_KNOTS_PER_REG);
static_assert((BECORE_YUVP_GAMMA_G_LOW_LAST - BECORE_YUVP_GAMMA_G_LOW_FIRST) /
	      4 + 1 == BECORE_YUVP_GAMMA_G_SPLIT_KNOT /
	      BECORE_GAMMA_KNOTS_PER_REG);
static_assert((BECORE_YUVP_GAMMA_G_HIGH_LAST -
	       BECORE_YUVP_GAMMA_G_HIGH_FIRST) / 4 + 1 ==
	      (EXYNOS_BECORE_GAMMA_POINTS - 1 -
	       BECORE_YUVP_GAMMA_G_SPLIT_KNOT) / BECORE_GAMMA_KNOTS_PER_REG);

/*
 * The same for the inverse below this block: one table and one grid, 32 pair
 * registers each with the last knot in a register of its own. Asserted rather
 * than read off the addresses, because a range one register short packs a
 * knot into a register the capture never wrote, and one too long runs off the
 * end of the curve. They live here because they need the forward block's
 * knot-count constants, which are declared above.
 */
static_assert((BECORE_YUVP_DEGAMMA_TBL_LAST - BECORE_YUVP_DEGAMMA_TBL_FIRST) /
	      4 == (EXYNOS_BECORE_GAMMA_POINTS - 1) /
	      BECORE_GAMMA_KNOTS_PER_REG);
static_assert((BECORE_YUVP_DEGAMMA_X_LOW_LAST -
	       BECORE_YUVP_DEGAMMA_X_LOW_FIRST) / 4 + 1 +
	      (BECORE_YUVP_DEGAMMA_X_HIGH_LAST -
	       BECORE_YUVP_DEGAMMA_X_HIGH_FIRST) / 4 ==
	      (EXYNOS_BECORE_GAMMA_POINTS - 1) / BECORE_GAMMA_KNOTS_PER_REG);
static_assert(BECORE_YUVP_DEGAMMA_TOE_KNOTS < EXYNOS_BECORE_GAMMA_POINTS);

/*
 * The tone mapper's guide curve: 128 Q15 samples, two to a register with the
 * lower-numbered one in the low half.  It is the per-frame output of the
 * vendor's tone-mapping node rather than a tuning table -- driven by the
 * exposure estimate and the front end's bilateral-grid statistics -- which is
 * why it is a parameters block and not something the driver can state.
 *
 * The driver's default under it is the identity ramp, and here that is not a
 * judgement call: with the neutral grid this driver writes, the curve makes no
 * difference to the picture at all.  Sending the ramp through the parameters
 * node produces a frame byte-identical to the one the vendor's own captured
 * curve produces, on the same staged input, and the captured curve is nowhere
 * near a ramp -- it departs from one by 14162 of 32767 at its furthest.  So
 * the ramp is measured to be a no-op today rather than argued to be harmless.
 *
 * It will stop being a no-op the moment something sends a real grid, which is
 * the intended end state: an IPA owns both, and a ramp under a real grid is
 * a curve that does nothing rather than a curve that does damage.
 */
#define BECORE_YUVP_LTM_GMAP_FIRST	(BECORE_YUVP_PHYS_BASE + 0x6020)
#define BECORE_YUVP_LTM_GMAP_LAST	(BECORE_YUVP_PHYS_BASE + 0x611c)
#define BECORE_LTM_CURVE_PER_REG	2

static_assert((BECORE_YUVP_LTM_GMAP_LAST - BECORE_YUVP_LTM_GMAP_FIRST) / 4 +
	      1 == EXYNOS_BECORE_LTM_CURVE_POINTS / BECORE_LTM_CURVE_PER_REG);

/* The luma is all gray weight and no lightness weight, Q8. */
#define BECORE_LTM_LUMACALC_GRAY_Q8	256

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
static_assert(EXYNOS_BECORE_CLUT_MAX == BECORE_CLUT_FIELD_MAX);

#define BECORE_YUVP_CHAIN_IMG_SIZE_REG	(BECORE_YUVP_PHYS_BASE + 0x0200)
#define BECORE_YUVP_GRID_DMA_EN_REG	(BECORE_YUVP_PHYS_BASE + 0x1c00)
#define BECORE_YUVP_GRID_DMA_FORMAT_REG	(BECORE_YUVP_PHYS_BASE + 0x1c10)
#define BECORE_YUVP_GRID_DMA_WIDTH_REG	(BECORE_YUVP_PHYS_BASE + 0x1c20)
#define BECORE_YUVP_GRID_DMA_STRIDE_REG	(BECORE_YUVP_PHYS_BASE + 0x1c28)
#define BECORE_YUVP_GRID_DMA_BUSINFO_REG (BECORE_YUVP_PHYS_BASE + 0x1c4c)
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
 * fraction of the destination. Samsung names that GET_ZOOM_RATIO(in, out) and
 * writes it as ((in) << MCSC_PRECISION) / (out) with MCSC_PRECISION 20, an
 * integer divide -- but this block's captured words are the *rounded*
 * quotient, not the truncated one, at every window in the corpus that tells
 * the two apart. RGBP's scaler is the truncation; see becore_zoom_ratio() and
 * becore_mcsc_djag_ratio() for which readouts say so.
 *
 * This driver crops the whole 4160 x 3120 raster into 4000 x 3000, which
 * gives 0x0010a3d7 on both axes either way; the vendor cropped 3536 x 2652
 * out of it and carried 0x000e24dd, spending the difference on stabilisation.
 */
#define BECORE_MCSC_DJAG_IMG_SIZE_REG	(BECORE_MCSC_PHYS_BASE + 0x4004)
#define BECORE_MCSC_DJAG_PS_SRC_POS_REG	(BECORE_MCSC_PHYS_BASE + 0x4008)
#define BECORE_MCSC_DJAG_PS_SRC_SIZE_REG (BECORE_MCSC_PHYS_BASE + 0x400c)
#define BECORE_MCSC_DJAG_PS_DST_SIZE_REG	(BECORE_MCSC_PHYS_BASE + 0x4010)
#define BECORE_MCSC_DJAG_PS_H_RATIO_REG	(BECORE_MCSC_PHYS_BASE + 0x4014)
#define BECORE_MCSC_DJAG_PS_V_RATIO_REG	(BECORE_MCSC_PHYS_BASE + 0x4018)
#define BECORE_RATIO_SHIFT		20
/* A scaler that neither shrinks nor stretches, which is the sign of both. */
#define BECORE_RATIO_UNITY		BIT(BECORE_RATIO_SHIFT)
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

#endif /* __EXYNOS_BECORE_REGS_H__ */
