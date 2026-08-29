// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S5KGN8 CMOS image sensor.
 *
 * The part answers with model id 0x08E8 at register 0x0000, in the SMIA/CCS
 * layout, and Google's camera stack calls it "barghest".  The mapping from
 * that id to a marketing name is inferred rather than read from a datasheet;
 * what this driver is actually written against is a recording of the vendor
 * stack programming this sensor, register by register, in a live session.
 *
 * It is a 16-bit-register, 16-bit-value part with a *paged* register file:
 * writing 0xfcfc selects a page, and the same register number means different
 * things on different ones.  The CCS block lives on page 0x4000, and the mode
 * list below carries its own page selects, which is why they appear in it as
 * ordinary writes.  Everything this driver does outside that list selects page
 * 0x4000 first rather than assuming the page it left behind.
 *
 * On that page the PLL and the geometry are textbook CCS: 0x0300..0x0306, the
 * frame at 0x0340/0x0342, the window at 0x0344..0x034a, the output size at
 * 0x034c/0x034e, the binning at 0x0380..0x0386 and 0x0900, and the per-frame
 * exposure and gain set at 0x0202/0x0204/0x020e behind the grouped-parameter
 * hold at 0x0104.
 *
 * Copyright 2026 Steffen Deusch
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

/*
 * The page select, and the page the CCS block lives on.  Read before any page
 * has been selected, 0x0000 still returns the model id -- that is how the
 * vendor stack identifies the part, and this driver does the same.
 */
#define S5KGN8_PAGE			CCI_REG16(0xfcfc)
#define   S5KGN8_PAGE_CCS		0x4000

#define S5KGN8_MODEL_ID			CCI_REG16(0x0000)
#define S5KGN8_MODEL_ID_VAL		0x08e8

/*
 * The vendor's first write to the part after identifying it, with the pause
 * that follows it in the recording.  What it does is not known -- 0x6010 is
 * not a CCS register -- but it is written once per power-up, before anything
 * else, and every frame ever captured from this sensor was taken after it.
 */
#define S5KGN8_INIT_6010		CCI_REG16(0x6010)
#define S5KGN8_INIT_6010_VAL		0x0001
#define S5KGN8_INIT_SETTLE_US		10000

/*
 * Values, not bits: on a 16-bit-value part a write to 0x0100 covers 0x0101 as
 * well, and the recording writes the pair.  0x0101 is CCS image_orientation,
 * so streaming starts with the readout unflipped -- which is what makes the
 * mosaic GRBG rather than GBRG.
 */
#define S5KGN8_MODE_SELECT		CCI_REG16(0x0100)
#define   S5KGN8_MODE_STANDBY		0x0000
#define   S5KGN8_MODE_STREAMING		0x0100

/*
 * Grouped parameter hold: everything written while it is set lands together.
 * The low half of the pair stays 1 in both, so the values are 0x0101 and
 * 0x0001 rather than 1 and 0.
 */
#define S5KGN8_GROUPED_HOLD		CCI_REG16(0x0104)
#define   S5KGN8_HOLD_ON		0x0101
#define   S5KGN8_HOLD_OFF		0x0001

#define S5KGN8_EXPOSURE			CCI_REG16(0x0202)
#define S5KGN8_ANALOGUE_GAIN		CCI_REG16(0x0204)
#define S5KGN8_DIGITAL_GAIN		CCI_REG16(0x020e)
#define S5KGN8_FRAME_LENGTH		CCI_REG16(0x0340)

/*
 * Two registers the vendor writes to zero inside every per-frame group and
 * nothing decodes.  Replayed as measured, because every frame captured from
 * this sensor was taken with them written there.
 */
#define S5KGN8_UNDECODED_0702		CCI_REG16(0x0702)
#define S5KGN8_UNDECODED_0704		CCI_REG16(0x0704)

/*
 * The captured per-frame values, used as the control defaults so that a
 * capture taken with no controls set is the one the receive path was brought
 * up against.  The frame length is the one the vendor's first exposure group
 * raises the mode list's 9320 to, before the stream even starts.
 */
#define S5KGN8_EXPOSURE_DEFAULT		2808
#define S5KGN8_ANA_GAIN_DEFAULT		53
#define S5KGN8_DGTL_GAIN_DEFAULT	0x0100

/*
 * What the part's own capability registers would say is unknown: they are
 * *not* read here, because on a paged register file a read of 0x0084 or
 * 0x1084 means nothing without knowing which page implements it, and the
 * recording never reads them.  Every limit below therefore has a source.
 *
 * The analogue gain range has three.  The mode list writes 0x0086 -- CCS
 * analogue_gain_code_max -- with 2048; Android reports this camera's maximum
 * analogue sensitivity as ISO 1865 against a range starting at 29, which is
 * 64x; and s5kjn1.c, the other Samsung part in this directory, exposes gain as
 * a 1..64 multiplier and writes it to 0x0204 shifted left by five, which is
 * the same 32..2048 in codes.  So the family law is code/32, 32 is unity and
 * 2048 is the 64x all three agree on.  The control is still the raw code, as
 * V4L2 analogue gain is everywhere else; the law belongs to whatever maps it
 * to a gain.  The recorded session only ever used 51..382 of that range.
 *
 * The digital gain range is the CCS default and is not measured: the recording
 * only ever writes unity, and reading the part's own capability registers is
 * exactly what a paged register file makes untrustworthy.
 */
#define S5KGN8_ANA_GAIN_MIN		32
#define S5KGN8_ANA_GAIN_MAX		2048
#define S5KGN8_DGTL_GAIN_MIN		256
#define S5KGN8_DGTL_GAIN_MAX		4095

/*
 * Coarse integration.  The margin is measured rather than assumed: one frame
 * in the recording sets frame_length_lines to 16903 and coarse integration to
 * 16852 in the same group, and the sensor went on delivering frames -- so a
 * coarse integration of frame_length - 51 is legal.  The minimum is not
 * measured at all; the shortest the session ever used was 2808 lines.
 */
#define S5KGN8_COARSE_INT_MIN		1
#define S5KGN8_COARSE_INT_MARGIN	51

/* frame_length_lines and line_length_pck are both 16-bit CCS fields. */
#define S5KGN8_FRAME_LENGTH_MAX		0xffff

/*
 * The external clock, and it is *not* what the part is told it is.  0x0136, in
 * the power-up sequence below, is written 0x1800 -- 24.0 MHz in the CCS Q8.8
 * encoding -- while the pad carries oscclk undivided at 24.576 MHz.  The frame
 * timing says which of the two the PLL runs on, at two different frame lengths:
 *
 *   5600 * 9540  / (196.608 MHz * 8) = 33.9661 ms, measured 33.9660
 *   5600 * 11773 / (196.608 MHz * 8) = 41.9164 ms, measured 41.9195
 *
 * against 34.7812 and 42.9224 ms on 24.0 MHz.  So the register is a stale
 * value in the vendor's list and the arithmetic below uses the real rate.
 */
#define S5KGN8_EXTCLK_RATE		24576000UL

/*
 * pre_pll_clk_div 4, pll_multiplier 288, vt_sys_clk_div 1 and vt_pix_clk_div 9
 * give 24.576 / 4 * 288 / 9 = 196.608 MHz, and the factor of eight below is
 * the pixels per vt_pix clock that closes the two frame times above.
 */
#define S5KGN8_PIXEL_RATE		(196608000LL * 8)

/*
 * Three C-PHY lanes.  Both ends of the link agree: the mode list writes CCS
 * csi_lane_mode 2, and the receiver's CMN_CTRL carries lanes-1 = 2 -- and at
 * this pixel rate and 10 bits three D-PHY lanes could not carry the data, so
 * the trios are C-PHY.
 *
 * No link frequency is reported.  The output PLL's registers do not sit where
 * CCS puts them on this part -- 0x030c reads as a zero divider -- and the one
 * field mapping that gives sane numbers produces a symbol rate too low to
 * carry this mode's own line, so nothing here is established well enough to
 * put a number on.  V4L2_CID_LINK_FREQ is left out rather than guessed at.
 */
#define S5KGN8_DATA_LANES		3

/*
 * The window the one recovered mode reads out, in array coordinates:
 * x_addr_start 80, y_addr_start 64, x_addr_end 8142, y_addr_end 6110.  Odd on
 * both axes, and deliberately so -- CCS binning gives (window + 1) / 2, which
 * is 4032 x 3024, and the 16 and 12 at 0x0350/0x0352 trim that to the mode's
 * 4000 x 3000 output.  This is the analogue crop, so it is what the source
 * pad's V4L2_SEL_TGT_CROP reports; the format on the same pad is the binned
 * output size.
 */
#define S5KGN8_WINDOW_LEFT		80U
#define S5KGN8_WINDOW_TOP		64U
#define S5KGN8_WINDOW_WIDTH		8063U
#define S5KGN8_WINDOW_HEIGHT		6047U

/*
 * And the array that has to contain it.  What the part says about itself has
 * never been read -- on a paged register file a read of CCS's x_addr_max means
 * nothing without knowing which page implements it -- so this is the smallest
 * array the one recovered mode proves, not a capability.  It is stated at all
 * because V4L2_SEL_TGT_NATIVE_SIZE has to have a zero origin, which the window
 * does not, and because a crop outside its own bounds is what makes a consumer
 * compute a negative offset.
 */
#define S5KGN8_NATIVE_WIDTH		(S5KGN8_WINDOW_LEFT + S5KGN8_WINDOW_WIDTH)
#define S5KGN8_NATIVE_HEIGHT		(S5KGN8_WINDOW_TOP + S5KGN8_WINDOW_HEIGHT)

/*
 * In the order the module's power sequence brings them up.  The names are
 * positional, not functional: three of the five are camera-PMIC LDOs whose
 * roles inside the module have not been established, and only their order is
 * evidence.  On this board they are L12S_CAMIO (1.8 V, shared by all three
 * camera modules), CAM_MAIN_2V9_B, CAM_MAIN_2V9_A, CAM_MAIN_2V25, and the
 * switched rail the downstream device tree calls vdig_enable -- without which
 * the sensor does not answer on i2c at all.
 */
static const struct regulator_bulk_data s5kgn8_supplies[] = {
	{ .supply = "iovdd" },
	{ .supply = "avdd" },
	{ .supply = "avdd2" },
	{ .supply = "avdd3" },
	{ .supply = "dvdd" },
};

struct s5kgn8_mode {
	u32 width;
	u32 height;
	/* line_length_pck, as the register list programs it. */
	u32 llp;
	/* frame_length_lines the vendor applies with its exposure group. */
	u32 fll_def;
	s64 pixel_rate;
	u32 code;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5kgn8 {
	struct regmap *regmap;
	struct clk *extclk;
	struct gpio_desc *reset;
	struct regulator_bulk_data *supplies;

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *again;
	struct v4l2_ctrl *dgain;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *pixel_rate;

	const struct s5kgn8_mode *mode;
};

static inline struct s5kgn8 *sd_to_s5kgn8(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5kgn8, sd);
}

static inline struct s5kgn8 *ctrl_to_s5kgn8(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct s5kgn8, hdl);
}

/*
 * The part's power-up initialisation, whose middle the i2c capture does not
 * contain.  That middle is a 5,620-byte image uploaded through a single
 * register, and the tracepoint the capture was decoded from drops a message
 * that large -- so what the recording shows in its place is a 54 ms gap that
 * reads as an idle pause.  The whole sequence is in the vendor HAL instead, as
 * one array of register/value pairs that Lyric logs as "BARGHEST ... using v1
 * init settings"; tools/camera-sensor-init-extract.py reads it back out, and
 * the two tables below are what it emits.
 *
 * It runs once per power-up: the wake write, these six -- which point the
 * part's indirect port at internal address 0x20030800 -- and then these
 * thirty-two.  The image belongs between them; it is uploaded two commits on,
 * behind the bus rate that message needs.
 *
 * All thirty-eight used to be the head of the mode list, because the i2c
 * capture is where they were read from and that is where they appear in it.
 * They are not the mode.  They carry the PLL, and the mode list never writes
 * it again:
 *
 *   0x0304/0x0306 = 4/288      pre_pll_clk_div and pll_multiplier
 *   0x0300/0x0302 = 9/1        vt_pix_clk_div and vt_sys_clk_div
 *   0x0136 = 0x1800            the external clock it is told it has, and that
 *                              is not the one it gets
 */
static const struct cci_reg_sequence s5kgn8_init_port[] = {
	{ CCI_REG16(0xfcfc), 0x2000 }, { CCI_REG16(0x0ea8), 0x0100 },
	{ CCI_REG16(0x0e4e), 0x0300 }, { CCI_REG16(0xfcfc), 0x4000 },
	{ CCI_REG16(0x6028), 0x2003 }, { CCI_REG16(0x602a), 0x0800 },
};

static const struct cci_reg_sequence s5kgn8_init_settings[] = {
	{ CCI_REG16(0xfcfc), 0x4000 }, { CCI_REG16(0x0a72), 0x0100 },
	{ CCI_REG16(0x0a70), 0x0001 }, { CCI_REG16(0xfcfc), 0x2000 },
	{ CCI_REG16(0x101e), 0x0003 }, { CCI_REG16(0xb3b6), 0x00fc },
	{ CCI_REG16(0x0e3c), 0x0902 }, { CCI_REG16(0x0e0a), 0x0762 },
	{ CCI_REG16(0x4752), 0x0000 }, { CCI_REG16(0xfcfc), 0x4000 },
	{ CCI_REG16(0x0312), 0x0000 }, { CCI_REG16(0x0310), 0x00df },
	{ CCI_REG16(0x030e), 0x0004 }, { CCI_REG16(0x011c), 0x0101 },
	{ CCI_REG16(0xfcfc), 0x2000 }, { CCI_REG16(0x0e08), 0x0000 },
	{ CCI_REG16(0x0e64), 0x03f0 }, { CCI_REG16(0x0e62), 0x03f0 },
	{ CCI_REG16(0x0ecc), 0x0100 }, { CCI_REG16(0x0eca), 0x0803 },
	{ CCI_REG16(0x0ec8), 0x0803 }, { CCI_REG16(0x0ec6), 0x0803 },
	{ CCI_REG16(0x0ec4), 0x0803 }, { CCI_REG16(0xfcfc), 0x4000 },
	{ CCI_REG16(0x030a), 0x0002 }, { CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0300), 0x0009 }, { CCI_REG16(0x030c), 0x0000 },
	{ CCI_REG16(0x0306), 0x0120 }, { CCI_REG16(0x0304), 0x0004 },
	{ CCI_REG16(0x013e), 0x0240 }, { CCI_REG16(0x0136), 0x1800 },
};

/*
 * The 2x2-binned preview mode, 4000x3000 at 29.4 fps, transcribed from a
 * recording of the vendor stack programming this sensor.  584 writes in the
 * order they were recorded, page selects included; the thirty-eight this
 * list used to start with belong to the power-up above.  The geometry inside
 * it decodes as
 *
 *   0x0112 = 0x0a0a            RAW10 in, RAW10 out
 *   0x0114 = 0x0201            CCS csi_lane_mode 2 in the high byte, so
 *                              three lanes; the low byte is 0x0115, which
 *                              CCS does not define
 *   0x0342 = 5600              line_length_pck
 *   0x0340 = 9320              frame_length_lines, and then 9540: the list
 *                              carries the vendor's first exposure group at
 *                              entries 362..369 and it raises the frame
 *                              before the stream starts
 *   0x0344..0x034a             window (80, 64) to (8142, 6110)
 *   0x034c = 4000, 0x034e = 3000
 *   0x0350 = 16, 0x0352 = 12   the margin binning leaves over
 *   0x0380..0x0386 = 2         x/y even/odd increment, so 2x2 binning
 *   0x0900 = 0x2222            binning mode
 *
 * Nothing here is reordered or deduplicated.  The geometry block appears
 * twice, several registers are written more than once, and which write lands
 * last is the one that counts.
 *
 * The recording's second streaming segment writes the same 584 registers in
 * the same order, and 583 of them with the same values.  The one that differs
 * is entry 368, the frame length inside the vendor's own exposure group, which
 * is 9540 here and 11773 there -- a value the controls own, so the table's
 * copy of it is overwritten on every stream start anyway.
 */
static const struct cci_reg_sequence s5kgn8_mode_4000x3000[] = {
	{ CCI_REG16(0xfcfc), 0x2000 }, { CCI_REG16(0xcf2a), 0x0400 },
	{ CCI_REG16(0xfcfc), 0x2001 }, { CCI_REG16(0x3080), 0x0200 },
	{ CCI_REG16(0x3084), 0x0000 }, { CCI_REG16(0x3086), 0x27c0 },
	{ CCI_REG16(0xfcfc), 0x2000 }, { CCI_REG16(0x34b6), 0x0000 },
	{ CCI_REG16(0x34b8), 0x0002 }, { CCI_REG16(0x3536), 0x0000 },
	{ CCI_REG16(0x3538), 0x0000 }, { CCI_REG16(0x4750), 0x0101 },
	{ CCI_REG16(0x4910), 0x0101 }, { CCI_REG16(0x49ba), 0x0014 },
	{ CCI_REG16(0x49be), 0x000f }, { CCI_REG16(0x49c6), 0x0014 },
	{ CCI_REG16(0x49c8), 0x0016 }, { CCI_REG16(0x49cc), 0x0046 },
	{ CCI_REG16(0x49d4), 0x0016 }, { CCI_REG16(0x4a2a), 0x0016 },
	{ CCI_REG16(0x4a2e), 0x001e }, { CCI_REG16(0x4a36), 0x0016 },
	{ CCI_REG16(0x4a38), 0x001a }, { CCI_REG16(0x4a3c), 0x0046 },
	{ CCI_REG16(0x4a44), 0x001a }, { CCI_REG16(0x4a46), 0x0016 },
	{ CCI_REG16(0x4a4a), 0x0001 }, { CCI_REG16(0x4a52), 0x0016 },
	{ CCI_REG16(0x4a54), 0x001a }, { CCI_REG16(0x4a58), 0x0028 },
	{ CCI_REG16(0x4a60), 0x001a }, { CCI_REG16(0x4a74), 0x0050 },
	{ CCI_REG16(0x4a7c), 0x0046 }, { CCI_REG16(0x4b0a), 0x0015 },
	{ CCI_REG16(0x4b0e), 0x0014 }, { CCI_REG16(0x4b16), 0x0015 },
	{ CCI_REG16(0x4b18), 0x0018 }, { CCI_REG16(0x4b1c), 0x001e },
	{ CCI_REG16(0x4b24), 0x0018 }, { CCI_REG16(0x4b7a), 0x0016 },
	{ CCI_REG16(0x4b7e), 0x0014 }, { CCI_REG16(0x4b86), 0x0016 },
	{ CCI_REG16(0x4b88), 0x0019 }, { CCI_REG16(0x4b8c), 0x0028 },
	{ CCI_REG16(0x4b94), 0x0019 }, { CCI_REG16(0x4b96), 0x0016 },
	{ CCI_REG16(0x4b9a), 0x0001 }, { CCI_REG16(0x4ba2), 0x0016 },
	{ CCI_REG16(0x4ba4), 0x0019 }, { CCI_REG16(0x4ba8), 0x0028 },
	{ CCI_REG16(0x4bb0), 0x0019 }, { CCI_REG16(0x4bda), 0x0002 },
	{ CCI_REG16(0x4c22), 0x001e }, { CCI_REG16(0x4c26), 0x0514 },
	{ CCI_REG16(0x4c2e), 0x001e }, { CCI_REG16(0x4c30), 0x0032 },
	{ CCI_REG16(0x4c34), 0x0046 }, { CCI_REG16(0x4c3c), 0x0032 },
	{ CCI_REG16(0x4c76), 0x001e }, { CCI_REG16(0x4c7a), 0x0152 },
	{ CCI_REG16(0x4c82), 0x001e }, { CCI_REG16(0x4cba), 0x0003 },
	{ CCI_REG16(0x4cc8), 0x0003 }, { CCI_REG16(0x4cca), 0x0018 },
	{ CCI_REG16(0x4cce), 0x0022 }, { CCI_REG16(0x4cd6), 0x0037 },
	{ CCI_REG16(0x4cd8), 0x0018 }, { CCI_REG16(0x4cdc), 0x0022 },
	{ CCI_REG16(0x4d06), 0x0077 }, { CCI_REG16(0x4d14), 0x0077 },
	{ CCI_REG16(0x4d22), 0x006d }, { CCI_REG16(0x4d30), 0x006d },
	{ CCI_REG16(0x4de6), 0x0019 }, { CCI_REG16(0x5464), 0x0014 },
	{ CCI_REG16(0x5468), 0x0016 }, { CCI_REG16(0x5470), 0x0014 },
	{ CCI_REG16(0x5472), 0x0030 }, { CCI_REG16(0x5476), 0x0032 },
	{ CCI_REG16(0x547e), 0x0038 }, { CCI_REG16(0x5480), 0x0038 },
	{ CCI_REG16(0x5484), 0x003a }, { CCI_REG16(0x548c), 0x0038 },
	{ CCI_REG16(0x6038), 0x0026 }, { CCI_REG16(0x6040), 0x0026 },
	{ CCI_REG16(0x6088), 0x0038 }, { CCI_REG16(0x6094), 0x0046 },
	{ CCI_REG16(0x60a2), 0x0046 }, { CCI_REG16(0x60e8), 0x00be },
	{ CCI_REG16(0x60ea), 0x0070 }, { CCI_REG16(0x60ee), 0x0142 },
	{ CCI_REG16(0x60f6), 0x00c4 }, { CCI_REG16(0x6104), 0x00b9 },
	{ CCI_REG16(0x747e), 0x0000 }, { CCI_REG16(0x7482), 0x8024 },
	{ CCI_REG16(0x748a), 0x0000 }, { CCI_REG16(0x748c), 0x0000 },
	{ CCI_REG16(0x7490), 0x8028 }, { CCI_REG16(0x7498), 0x0000 },
	{ CCI_REG16(0x83b6), 0x0001 }, { CCI_REG16(0x83c4), 0x0460 },
	{ CCI_REG16(0xb1ec), 0x04f8 }, { CCI_REG16(0xb204), 0x0702 },
	{ CCI_REG16(0xb21a), 0x1414 }, { CCI_REG16(0xb21c), 0x3c14 },
	{ CCI_REG16(0xb220), 0x1418 }, { CCI_REG16(0xb222), 0x3838 },
	{ CCI_REG16(0xb224), 0x3c38 }, { CCI_REG16(0xb228), 0x383c },
	{ CCI_REG16(0xb232), 0x0c0c }, { CCI_REG16(0xb23a), 0x0202 },
	{ CCI_REG16(0xb24c), 0x1212 }, { CCI_REG16(0xb254), 0x0f0f },
	{ CCI_REG16(0xb270), 0x693a }, { CCI_REG16(0xb274), 0x6c3b },
	{ CCI_REG16(0xb27c), 0x7fff }, { CCI_REG16(0xb27e), 0x016f },
	{ CCI_REG16(0xb282), 0x01ef }, { CCI_REG16(0xb28a), 0x017f },
	{ CCI_REG16(0xb2a6), 0x077f }, { CCI_REG16(0xb2a8), 0x00fd },
	{ CCI_REG16(0xb2ac), 0x00ff }, { CCI_REG16(0xb2b4), 0x01fd },
	{ CCI_REG16(0xb2e4), 0x0492 }, { CCI_REG16(0xb2e8), 0x0249 },
	{ CCI_REG16(0xb2f0), 0x0492 }, { CCI_REG16(0xb300), 0x0082 },
	{ CCI_REG16(0xb304), 0x0041 }, { CCI_REG16(0xb30c), 0x0082 },
	{ CCI_REG16(0xb364), 0x0500 }, { CCI_REG16(0xb366), 0x04c0 },
	{ CCI_REG16(0xb36c), 0x0580 }, { CCI_REG16(0xb36e), 0x05c0 },
	{ CCI_REG16(0xb37c), 0x0500 }, { CCI_REG16(0xb37e), 0x04c0 },
	{ CCI_REG16(0xb380), 0x0500 }, { CCI_REG16(0xb382), 0x04c0 },
	{ CCI_REG16(0xb3a0), 0x0600 }, { CCI_REG16(0xb3a2), 0x00e2 },
	{ CCI_REG16(0xb3aa), 0x0100 }, { CCI_REG16(0xb3ae), 0x0e0e },
	{ CCI_REG16(0xbea6), 0x8676 }, { CCI_REG16(0xbea8), 0x0026 },
	{ CCI_REG16(0xbeaa), 0xa008 }, { CCI_REG16(0xbeac), 0x0631 },
	{ CCI_REG16(0xc590), 0x0000 }, { CCI_REG16(0xc5a8), 0x0000 },
	{ CCI_REG16(0xc708), 0x2875 }, { CCI_REG16(0xc70c), 0x2875 },
	{ CCI_REG16(0xc70e), 0x0f0f }, { CCI_REG16(0xc716), 0xb000 },
	{ CCI_REG16(0xc71a), 0x8000 }, { CCI_REG16(0xc71c), 0x2000 },
	{ CCI_REG16(0xc9fe), 0x0000 }, { CCI_REG16(0xca02), 0x01d8 },
	{ CCI_REG16(0xca0a), 0x0000 }, { CCI_REG16(0xca48), 0x0000 },
	{ CCI_REG16(0xca4a), 0x0000 }, { CCI_REG16(0xca4c), 0x0000 },
	{ CCI_REG16(0xca4e), 0x0000 }, { CCI_REG16(0xcc7a), 0x1f21 },
	{ CCI_REG16(0xcc7c), 0x2324 }, { CCI_REG16(0xcc7e), 0x2628 },
	{ CCI_REG16(0xcc80), 0x2a2c }, { CCI_REG16(0xcc82), 0x2e2f },
	{ CCI_REG16(0xcc84), 0x3031 }, { CCI_REG16(0xcc86), 0x3132 },
	{ CCI_REG16(0xcc88), 0x3334 }, { CCI_REG16(0xcc8a), 0x3536 },
	{ CCI_REG16(0xcc8c), 0x3637 }, { CCI_REG16(0xcc8e), 0x3738 },
	{ CCI_REG16(0xcc90), 0x3838 }, { CCI_REG16(0xcc92), 0x3939 },
	{ CCI_REG16(0xcc9a), 0x3b1b }, { CCI_REG16(0xcc9c), 0x1f22 },
	{ CCI_REG16(0xcc9e), 0x2426 }, { CCI_REG16(0xcca0), 0x282a },
	{ CCI_REG16(0xcca2), 0x2b2c }, { CCI_REG16(0xcca4), 0x2e30 },
	{ CCI_REG16(0xcca6), 0x3131 }, { CCI_REG16(0xcca8), 0x3233 },
	{ CCI_REG16(0xccaa), 0x3435 }, { CCI_REG16(0xccac), 0x3636 },
	{ CCI_REG16(0xccae), 0x3737 }, { CCI_REG16(0xccb0), 0x3838 },
	{ CCI_REG16(0xccb2), 0x3839 }, { CCI_REG16(0xccb4), 0x3939 },
	{ CCI_REG16(0xccb6), 0x3a3a }, { CCI_REG16(0xccb8), 0x3a3a },
	{ CCI_REG16(0xcf40), 0x8822 }, { CCI_REG16(0xd954), 0xffff },
	{ CCI_REG16(0xd956), 0x0202 }, { CCI_REG16(0xd994), 0x0108 },
	{ CCI_REG16(0xda76), 0x0000 }, { CCI_REG16(0xda86), 0x0000 },
	{ CCI_REG16(0xfcfc), 0x2001 }, { CCI_REG16(0x0e64), 0x0001 },
	{ CCI_REG16(0x0e66), 0x0100 }, { CCI_REG16(0x0e68), 0x01ff },
	{ CCI_REG16(0x0e6a), 0x1000 }, { CCI_REG16(0x0e6c), 0x0fe4 },
	{ CCI_REG16(0x0e6e), 0x0200 }, { CCI_REG16(0x0e70), 0x02ff },
	{ CCI_REG16(0x0e72), 0x0fe4 }, { CCI_REG16(0x0e74), 0x1012 },
	{ CCI_REG16(0x0e76), 0x0300 }, { CCI_REG16(0x0e78), 0x03ff },
	{ CCI_REG16(0x0e7a), 0x1012 }, { CCI_REG16(0x0e7c), 0x1026 },
	{ CCI_REG16(0x0e7e), 0x0400 }, { CCI_REG16(0x0e80), 0x0e00 },
	{ CCI_REG16(0x0e82), 0x1026 }, { CCI_REG16(0x0e84), 0x101e },
	{ CCI_REG16(0x0e86), 0x0e00 }, { CCI_REG16(0x0e88), 0x4000 },
	{ CCI_REG16(0x0e8a), 0x101e }, { CCI_REG16(0x0e8c), 0x1024 },
	{ CCI_REG16(0x214c), 0x0600 }, { CCI_REG16(0x22ce), 0xffff },
	{ CCI_REG16(0x22d0), 0x0000 }, { CCI_REG16(0x22d2), 0xffff },
	{ CCI_REG16(0x22d4), 0x0000 }, { CCI_REG16(0x22de), 0x0000 },
	{ CCI_REG16(0x22e0), 0x0000 }, { CCI_REG16(0x22e2), 0x0000 },
	{ CCI_REG16(0x22e4), 0x0000 }, { CCI_REG16(0x22ee), 0xffff },
	{ CCI_REG16(0x22f0), 0x0000 }, { CCI_REG16(0x22f2), 0xffff },
	{ CCI_REG16(0x22f4), 0x0000 }, { CCI_REG16(0x22fe), 0x0000 },
	{ CCI_REG16(0x2300), 0x0000 }, { CCI_REG16(0x2302), 0x0000 },
	{ CCI_REG16(0x2304), 0x0000 }, { CCI_REG16(0x234e), 0x0000 },
	{ CCI_REG16(0x2352), 0x0000 }, { CCI_REG16(0x236e), 0x0000 },
	{ CCI_REG16(0x2372), 0x0000 }, { CCI_REG16(0x3130), 0x0000 },
	{ CCI_REG16(0x3132), 0x0000 }, { CCI_REG16(0x3182), 0x0000 },
	{ CCI_REG16(0x4094), 0x0101 }, { CCI_REG16(0x40ba), 0x2f00 },
	{ CCI_REG16(0x40bc), 0x01e0 }, { CCI_REG16(0x40be), 0x03c0 },
	{ CCI_REG16(0x40c0), 0x01e0 }, { CCI_REG16(0x40c2), 0x01e0 },
	{ CCI_REG16(0x40e0), 0x0202 }, { CCI_REG16(0x40f2), 0x0000 },
	{ CCI_REG16(0x4578), 0x0886 }, { CCI_REG16(0x45ca), 0x0010 },
	{ CCI_REG16(0x45cc), 0x000c }, { CCI_REG16(0x4770), 0x0000 },
	{ CCI_REG16(0x47ce), 0x0400 }, { CCI_REG16(0x4ae2), 0x0100 },
	{ CCI_REG16(0x4ae6), 0x0100 }, { CCI_REG16(0x4aec), 0x017f },
	{ CCI_REG16(0x4aee), 0x01ff }, { CCI_REG16(0x4af0), 0x02ff },
	{ CCI_REG16(0x4af2), 0x03ff }, { CCI_REG16(0x4af4), 0x05ff },
	{ CCI_REG16(0x4af6), 0x07ff }, { CCI_REG16(0x4af8), 0x0bff },
	{ CCI_REG16(0x4afa), 0x0fff }, { CCI_REG16(0x4afc), 0x17ff },
	{ CCI_REG16(0x4afe), 0x1fff }, { CCI_REG16(0x4b00), 0x2fff },
	{ CCI_REG16(0x4b02), 0x3fff }, { CCI_REG16(0x4b04), 0x5fff },
	{ CCI_REG16(0x4b2e), 0x0400 }, { CCI_REG16(0x4b38), 0x0000 },
	{ CCI_REG16(0x4b3a), 0x0007 }, { CCI_REG16(0x4b3c), 0x000b },
	{ CCI_REG16(0x4b46), 0x0018 }, { CCI_REG16(0x4b48), 0x0016 },
	{ CCI_REG16(0x4b4a), 0x0016 }, { CCI_REG16(0x4b4c), 0x0015 },
	{ CCI_REG16(0x4b4e), 0x001e }, { CCI_REG16(0x4b50), 0x001d },
	{ CCI_REG16(0x4b52), 0x001d }, { CCI_REG16(0x4b54), 0x001d },
	{ CCI_REG16(0x4b56), 0x001d }, { CCI_REG16(0x4b58), 0x001d },
	{ CCI_REG16(0x4b5a), 0x001d }, { CCI_REG16(0x4b5c), 0x001e },
	{ CCI_REG16(0x4b5e), 0x001f }, { CCI_REG16(0x4b66), 0x0018 },
	{ CCI_REG16(0x4b68), 0x0016 }, { CCI_REG16(0x4b6a), 0x0016 },
	{ CCI_REG16(0x4b6c), 0x0015 }, { CCI_REG16(0x4b6e), 0x001e },
	{ CCI_REG16(0x4b70), 0x001d }, { CCI_REG16(0x4b72), 0x001d },
	{ CCI_REG16(0x4b74), 0x001d }, { CCI_REG16(0x4b76), 0x001d },
	{ CCI_REG16(0x4b78), 0x001d }, { CCI_REG16(0x4b7a), 0x001d },
	{ CCI_REG16(0x4b7c), 0x001e }, { CCI_REG16(0x4b7e), 0x001f },
	{ CCI_REG16(0x5e22), 0x0202 }, { CCI_REG16(0xfcfc), 0x2003 },
	{ CCI_REG16(0x48b0), 0x0000 }, { CCI_REG16(0x48b4), 0x0040 },
	{ CCI_REG16(0x48b6), 0x0040 }, { CCI_REG16(0x4938), 0x0000 },
	{ CCI_REG16(0x493c), 0x0040 }, { CCI_REG16(0x493e), 0x0040 },
	{ CCI_REG16(0x4944), 0x8000 }, { CCI_REG16(0x49c0), 0x0000 },
	{ CCI_REG16(0x49c4), 0x0040 }, { CCI_REG16(0x49c6), 0x0040 },
	{ CCI_REG16(0x4a48), 0x0000 }, { CCI_REG16(0x4a4c), 0x0040 },
	{ CCI_REG16(0x4a4e), 0x0040 }, { CCI_REG16(0xfcfc), 0x4000 },
	{ CCI_REG16(0x0086), 0x0800 }, { CCI_REG16(0x0112), 0x0a0a },
	{ CCI_REG16(0x0114), 0x0201 }, { CCI_REG16(0x0118), 0x0104 },
	{ CCI_REG16(0x021e), 0x0000 }, { CCI_REG16(0x0248), 0x0001 },
	{ CCI_REG16(0x0262), 0x0200 }, { CCI_REG16(0x0272), 0x2b10 },
	{ CCI_REG16(0x0340), 0x2468 }, { CCI_REG16(0x0342), 0x15e0 },
	{ CCI_REG16(0x0344), 0x0050 }, { CCI_REG16(0x0346), 0x0040 },
	{ CCI_REG16(0x0348), 0x1fce }, { CCI_REG16(0x034a), 0x17de },
	{ CCI_REG16(0x034c), 0x0fa0 }, { CCI_REG16(0x034e), 0x0bb8 },
	{ CCI_REG16(0x0350), 0x0010 }, { CCI_REG16(0x0352), 0x000c },
	{ CCI_REG16(0x0380), 0x0002 }, { CCI_REG16(0x0382), 0x0002 },
	{ CCI_REG16(0x0384), 0x0002 }, { CCI_REG16(0x0386), 0x0002 },
	{ CCI_REG16(0x0720), 0x0001 }, { CCI_REG16(0x0722), 0x0800 },
	{ CCI_REG16(0x072c), 0x07d0 }, { CCI_REG16(0x072e), 0x02ee },
	{ CCI_REG16(0x0900), 0x2222 }, { CCI_REG16(0x0b04), 0x0001 },
	{ CCI_REG16(0xfd5c), 0x0011 }, { CCI_REG16(0xfd8e), 0x0d00 },
	{ CCI_REG16(0xff60), 0x8200 }, { CCI_REG16(0xff62), 0x8684 },
	{ CCI_REG16(0xff64), 0x8a88 }, { CCI_REG16(0xff66), 0x8e8c },
	{ CCI_REG16(0xff68), 0x8200 }, { CCI_REG16(0xff6a), 0x8684 },
	{ CCI_REG16(0xff6c), 0x8a88 }, { CCI_REG16(0xff6e), 0x8e8c },
	{ CCI_REG16(0xff70), 0x8a90 }, { CCI_REG16(0xff72), 0x868c },
	{ CCI_REG16(0xff74), 0x8288 }, { CCI_REG16(0xff76), 0x0284 },
	{ CCI_REG16(0xff78), 0x8a90 }, { CCI_REG16(0xff7a), 0x868c },
	{ CCI_REG16(0xff7c), 0x8288 }, { CCI_REG16(0xff7e), 0x0284 },
	{ CCI_REG16(0xfcfc), 0x4000 }, { CCI_REG16(0x6000), 0x0085 },
	{ CCI_REG16(0x0104), 0x0101 }, { CCI_REG16(0x0702), 0x0000 },
	{ CCI_REG16(0x0704), 0x0000 }, { CCI_REG16(0x020e), 0x0100 },
	{ CCI_REG16(0x0204), 0x0035 }, { CCI_REG16(0x0202), 0x0af8 },
	{ CCI_REG16(0x0340), 0x2544 }, { CCI_REG16(0x0104), 0x0001 },
	{ CCI_REG16(0xfcfc), 0x2000 }, { CCI_REG16(0x4910), 0x0101 },
	{ CCI_REG16(0x49ba), 0x0014 }, { CCI_REG16(0x49be), 0x000f },
	{ CCI_REG16(0x49c8), 0x0016 }, { CCI_REG16(0x49cc), 0x0046 },
	{ CCI_REG16(0x4a2a), 0x0016 }, { CCI_REG16(0x4a2e), 0x001e },
	{ CCI_REG16(0x4a38), 0x001a }, { CCI_REG16(0x4a3c), 0x0046 },
	{ CCI_REG16(0x4a46), 0x0016 }, { CCI_REG16(0x4a4a), 0x0001 },
	{ CCI_REG16(0x4a54), 0x001a }, { CCI_REG16(0x4a58), 0x0028 },
	{ CCI_REG16(0x4a74), 0x0050 }, { CCI_REG16(0x4b0a), 0x0015 },
	{ CCI_REG16(0x4b0e), 0x0014 }, { CCI_REG16(0x4b18), 0x0018 },
	{ CCI_REG16(0x4b1c), 0x001e }, { CCI_REG16(0x4b7a), 0x0016 },
	{ CCI_REG16(0x4b7e), 0x0014 }, { CCI_REG16(0x4b88), 0x0019 },
	{ CCI_REG16(0x4b8c), 0x0028 }, { CCI_REG16(0x4b96), 0x0016 },
	{ CCI_REG16(0x4b9a), 0x0001 }, { CCI_REG16(0x4ba4), 0x0019 },
	{ CCI_REG16(0x4ba8), 0x0028 }, { CCI_REG16(0x4c22), 0x001e },
	{ CCI_REG16(0x4c26), 0x0514 }, { CCI_REG16(0x4c30), 0x0032 },
	{ CCI_REG16(0x4c34), 0x0046 }, { CCI_REG16(0x4c76), 0x001e },
	{ CCI_REG16(0x4c7a), 0x0152 }, { CCI_REG16(0x4cca), 0x0018 },
	{ CCI_REG16(0x4cce), 0x0022 }, { CCI_REG16(0x4cd8), 0x0018 },
	{ CCI_REG16(0x4cdc), 0x0022 }, { CCI_REG16(0x5464), 0x0014 },
	{ CCI_REG16(0x5468), 0x0016 }, { CCI_REG16(0x5472), 0x0030 },
	{ CCI_REG16(0x5476), 0x0032 }, { CCI_REG16(0x5480), 0x0038 },
	{ CCI_REG16(0x5484), 0x003a }, { CCI_REG16(0x6088), 0x0038 },
	{ CCI_REG16(0x60ea), 0x0070 }, { CCI_REG16(0x60ee), 0x0142 },
	{ CCI_REG16(0x747e), 0x0000 }, { CCI_REG16(0x7482), 0x8024 },
	{ CCI_REG16(0x748c), 0x0000 }, { CCI_REG16(0x7490), 0x8028 },
	{ CCI_REG16(0xb1ec), 0x04f8 }, { CCI_REG16(0xb204), 0x0702 },
	{ CCI_REG16(0xb21a), 0x1414 }, { CCI_REG16(0xb21c), 0x3c14 },
	{ CCI_REG16(0xb222), 0x3838 }, { CCI_REG16(0xb224), 0x3c38 },
	{ CCI_REG16(0xb24c), 0x1212 }, { CCI_REG16(0xb254), 0x0f0f },
	{ CCI_REG16(0xb270), 0x693a }, { CCI_REG16(0xb274), 0x6c3b },
	{ CCI_REG16(0xb27e), 0x016f }, { CCI_REG16(0xb282), 0x01ef },
	{ CCI_REG16(0xb2a8), 0x00fd }, { CCI_REG16(0xb2ac), 0x00ff },
	{ CCI_REG16(0xb2e4), 0x0492 }, { CCI_REG16(0xb2e8), 0x0249 },
	{ CCI_REG16(0xb300), 0x0082 }, { CCI_REG16(0xb304), 0x0041 },
	{ CCI_REG16(0xb364), 0x0500 }, { CCI_REG16(0xb366), 0x04c0 },
	{ CCI_REG16(0xb36c), 0x0580 }, { CCI_REG16(0xb36e), 0x05c0 },
	{ CCI_REG16(0xb3aa), 0x0100 }, { CCI_REG16(0xb3ae), 0x0e0e },
	{ CCI_REG16(0xbea8), 0x0026 }, { CCI_REG16(0xc590), 0x0000 },
	{ CCI_REG16(0xc5a8), 0x0000 }, { CCI_REG16(0xc9fe), 0x0000 },
	{ CCI_REG16(0xca02), 0x01d8 }, { CCI_REG16(0xca48), 0x0000 },
	{ CCI_REG16(0xca4a), 0x0000 }, { CCI_REG16(0xca4c), 0x0000 },
	{ CCI_REG16(0xca4e), 0x0000 }, { CCI_REG16(0xcc7a), 0x1f21 },
	{ CCI_REG16(0xcc7c), 0x2324 }, { CCI_REG16(0xcc7e), 0x2628 },
	{ CCI_REG16(0xcc80), 0x2a2c }, { CCI_REG16(0xcc82), 0x2e2f },
	{ CCI_REG16(0xcc84), 0x3031 }, { CCI_REG16(0xcc86), 0x3132 },
	{ CCI_REG16(0xcc88), 0x3334 }, { CCI_REG16(0xcc8a), 0x3536 },
	{ CCI_REG16(0xcc8c), 0x3637 }, { CCI_REG16(0xcc8e), 0x3738 },
	{ CCI_REG16(0xcc90), 0x3838 }, { CCI_REG16(0xcc92), 0x3939 },
	{ CCI_REG16(0xcc9a), 0x3b1b }, { CCI_REG16(0xcc9c), 0x1f22 },
	{ CCI_REG16(0xcc9e), 0x2426 }, { CCI_REG16(0xcca0), 0x282a },
	{ CCI_REG16(0xcca2), 0x2b2c }, { CCI_REG16(0xcca4), 0x2e30 },
	{ CCI_REG16(0xcca6), 0x3131 }, { CCI_REG16(0xcca8), 0x3233 },
	{ CCI_REG16(0xccaa), 0x3435 }, { CCI_REG16(0xccac), 0x3636 },
	{ CCI_REG16(0xccae), 0x3737 }, { CCI_REG16(0xccb0), 0x3838 },
	{ CCI_REG16(0xccb2), 0x3839 }, { CCI_REG16(0xccb4), 0x3939 },
	{ CCI_REG16(0xccb6), 0x3a3a }, { CCI_REG16(0xccb8), 0x3a3a },
	{ CCI_REG16(0xcf40), 0x8822 }, { CCI_REG16(0xfcfc), 0x2001 },
	{ CCI_REG16(0x5e22), 0x0202 }, { CCI_REG16(0x0e64), 0x0001 },
	{ CCI_REG16(0x0e66), 0x0100 }, { CCI_REG16(0x0e68), 0x01ff },
	{ CCI_REG16(0x0e6a), 0x1000 }, { CCI_REG16(0x0e6c), 0x0fe4 },
	{ CCI_REG16(0x0e6e), 0x0200 }, { CCI_REG16(0x0e70), 0x02ff },
	{ CCI_REG16(0x0e72), 0x0fe4 }, { CCI_REG16(0x0e74), 0x1012 },
	{ CCI_REG16(0x0e76), 0x0300 }, { CCI_REG16(0x0e78), 0x03ff },
	{ CCI_REG16(0x0e7a), 0x1012 }, { CCI_REG16(0x0e7c), 0x1026 },
	{ CCI_REG16(0x0e7e), 0x0400 }, { CCI_REG16(0x0e80), 0x0e00 },
	{ CCI_REG16(0x0e82), 0x1026 }, { CCI_REG16(0x0e84), 0x101e },
	{ CCI_REG16(0x0e86), 0x0e00 }, { CCI_REG16(0x0e88), 0x4000 },
	{ CCI_REG16(0x0e8a), 0x101e }, { CCI_REG16(0x0e8c), 0x1024 },
	{ CCI_REG16(0x22ce), 0xffff }, { CCI_REG16(0x22d0), 0x0000 },
	{ CCI_REG16(0x22d2), 0xffff }, { CCI_REG16(0x22d4), 0x0000 },
	{ CCI_REG16(0x22de), 0x0000 }, { CCI_REG16(0x22e0), 0x0000 },
	{ CCI_REG16(0x22e2), 0x0000 }, { CCI_REG16(0x22e4), 0x0000 },
	{ CCI_REG16(0x22ee), 0xffff }, { CCI_REG16(0x22f0), 0x0000 },
	{ CCI_REG16(0x22f2), 0xffff }, { CCI_REG16(0x22f4), 0x0000 },
	{ CCI_REG16(0x22fe), 0x0000 }, { CCI_REG16(0x2300), 0x0000 },
	{ CCI_REG16(0x2302), 0x0000 }, { CCI_REG16(0x2304), 0x0000 },
	{ CCI_REG16(0x3130), 0x0000 }, { CCI_REG16(0x3132), 0x0000 },
	{ CCI_REG16(0x4094), 0x0101 }, { CCI_REG16(0x40ba), 0x2f00 },
	{ CCI_REG16(0x40bc), 0x01e0 }, { CCI_REG16(0x40be), 0x03c0 },
	{ CCI_REG16(0x40c0), 0x01e0 }, { CCI_REG16(0x40c2), 0x01e0 },
	{ CCI_REG16(0x4578), 0x0886 }, { CCI_REG16(0x45cc), 0x000c },
	{ CCI_REG16(0x4770), 0x0000 }, { CCI_REG16(0x47ce), 0x0400 },
	{ CCI_REG16(0x4afe), 0x1fff }, { CCI_REG16(0x4b00), 0x2fff },
	{ CCI_REG16(0x4b02), 0x3fff }, { CCI_REG16(0x4b04), 0x5fff },
	{ CCI_REG16(0x4b46), 0x0018 }, { CCI_REG16(0x4b48), 0x0016 },
	{ CCI_REG16(0x4b4a), 0x0016 }, { CCI_REG16(0x4b4c), 0x0015 },
	{ CCI_REG16(0x4b4e), 0x001e }, { CCI_REG16(0x4b50), 0x001d },
	{ CCI_REG16(0x4b52), 0x001d }, { CCI_REG16(0x4b54), 0x001d },
	{ CCI_REG16(0x4b56), 0x001d }, { CCI_REG16(0x4b58), 0x001d },
	{ CCI_REG16(0x4b5a), 0x001d }, { CCI_REG16(0x4b5c), 0x001e },
	{ CCI_REG16(0x4b5e), 0x001f }, { CCI_REG16(0x4b66), 0x0018 },
	{ CCI_REG16(0x4b68), 0x0016 }, { CCI_REG16(0x4b6a), 0x0016 },
	{ CCI_REG16(0x4b6c), 0x0015 }, { CCI_REG16(0x4b6e), 0x001e },
	{ CCI_REG16(0x4b70), 0x001d }, { CCI_REG16(0x4b72), 0x001d },
	{ CCI_REG16(0x4b74), 0x001d }, { CCI_REG16(0x4b76), 0x001d },
	{ CCI_REG16(0x4b78), 0x001d }, { CCI_REG16(0x4b7a), 0x001d },
	{ CCI_REG16(0x4b7c), 0x001e }, { CCI_REG16(0x4b7e), 0x001f },
	{ CCI_REG16(0xfcfc), 0x4000 }, { CCI_REG16(0x0086), 0x0800 },
	{ CCI_REG16(0x0342), 0x15e0 }, { CCI_REG16(0x0344), 0x0050 },
	{ CCI_REG16(0x0346), 0x0040 }, { CCI_REG16(0x0348), 0x1fce },
	{ CCI_REG16(0x034a), 0x17de }, { CCI_REG16(0x0352), 0x000c },
	{ CCI_REG16(0x0380), 0x0002 }, { CCI_REG16(0x0382), 0x0002 },
	{ CCI_REG16(0x0384), 0x0002 }, { CCI_REG16(0x0386), 0x0002 },
	{ CCI_REG16(0x072c), 0x07d0 }, { CCI_REG16(0x072e), 0x02ee },
	{ CCI_REG16(0x0900), 0x2222 }, { CCI_REG16(0x0b04), 0x0001 },
	{ CCI_REG16(0xfd5c), 0x0011 }, { CCI_REG16(0xfd8e), 0x0d00 },
	{ CCI_REG16(0xfcfc), 0x4000 }, { CCI_REG16(0x6000), 0x0085 },
};

static const struct s5kgn8_mode s5kgn8_modes[] = {
	{
		.width = 4000,
		.height = 3000,
		.llp = 5600,
		.fll_def = 9540,
		.pixel_rate = S5KGN8_PIXEL_RATE,
		/*
		 * Confirmed from the pixels of a captured raw frame, not just
		 * from the vendor's metadata: the two green sites agree to
		 * within 0.1 of a code while red and blue differ from them and
		 * from each other.  A 2x2 bin of a GRBG mosaic is still GRBG,
		 * because it averages each colour with its own kind.
		 */
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.regs = s5kgn8_mode_4000x3000,
		.num_regs = ARRAY_SIZE(s5kgn8_mode_4000x3000),
	},
};

/* ---- register helpers --------------------------------------------------- */

/*
 * Select the CCS page.  Written unconditionally rather than tracked, because
 * the state it would track is the sensor's and the sensor loses it at every
 * runtime-PM suspend; one extra write per control group is cheaper than a
 * cached page that is wrong once.
 */
static void s5kgn8_select_ccs_page(struct s5kgn8 *sensor, int *err)
{
	cci_write(sensor->regmap, S5KGN8_PAGE, S5KGN8_PAGE_CCS, err);
}

/*
 * Every per-frame value the vendor stack changes is written inside the
 * grouped-parameter hold, so that exposure, gain and frame length take effect
 * on the same frame boundary rather than one apiece.
 */
static int s5kgn8_write_held(struct s5kgn8 *sensor, u32 reg, u64 val)
{
	int ret = 0, release;

	/*
	 * Returned rather than accumulated: everything below is page-relative,
	 * and the release in particular is issued whatever else failed -- so a
	 * page select that did not land would send it to whichever of the mode
	 * list's other three pages was still selected.
	 */
	s5kgn8_select_ccs_page(sensor, &ret);
	if (ret)
		return ret;

	cci_write(sensor->regmap, S5KGN8_GROUPED_HOLD, S5KGN8_HOLD_ON, &ret);
	cci_write(sensor->regmap, reg, val, &ret);
	/*
	 * Released with its own error, because cci_write() skips a write when
	 * the accumulated one is already set -- and a sensor left holding stops
	 * latching per-frame updates for the rest of the session, which reads
	 * as "the controls do nothing" rather than as an i2c failure.
	 */
	release = cci_write(sensor->regmap, S5KGN8_GROUPED_HOLD,
			    S5KGN8_HOLD_OFF, NULL);

	return ret ? ret : release;
}

/*
 * The whole per-frame set, in the order and the single hold the recording puts
 * it in, so that the values a stream starts with land on one frame boundary
 * exactly as they were captured.
 *
 * The control values are read straight out of the handler rather than through
 * v4l2_ctrl_g_ctrl(): this runs from enable_streams, which already holds the
 * subdev state lock, and that is the control handler's lock too, so the
 * locking accessor deadlocks the caller against itself.  That is a hung task
 * rather than an error return, so the assertion below is worth its line.
 */
static int s5kgn8_apply_frame_group(struct s5kgn8 *sensor)
{
	int ret = 0, release;

	lockdep_assert_held(sensor->hdl.lock);

	/* Returned, not accumulated, for the reason s5kgn8_write_held() gives. */
	s5kgn8_select_ccs_page(sensor, &ret);
	if (ret)
		return ret;

	cci_write(sensor->regmap, S5KGN8_GROUPED_HOLD, S5KGN8_HOLD_ON, &ret);
	cci_write(sensor->regmap, S5KGN8_UNDECODED_0702, 0, &ret);
	cci_write(sensor->regmap, S5KGN8_UNDECODED_0704, 0, &ret);
	cci_write(sensor->regmap, S5KGN8_DIGITAL_GAIN,
		  sensor->dgain->cur.val, &ret);
	cci_write(sensor->regmap, S5KGN8_ANALOGUE_GAIN,
		  sensor->again->cur.val, &ret);
	cci_write(sensor->regmap, S5KGN8_EXPOSURE,
		  sensor->exposure->cur.val, &ret);
	cci_write(sensor->regmap, S5KGN8_FRAME_LENGTH,
		  sensor->mode->height + sensor->vblank->cur.val, &ret);
	release = cci_write(sensor->regmap, S5KGN8_GROUPED_HOLD,
			    S5KGN8_HOLD_OFF, NULL);

	return ret ? ret : release;
}

/*
 * Clamp a default onto a control's range.  Every control here has a step of
 * one, so there is no grid to round onto; what this is for is narrowing the
 * handler's s64 bounds to the u32 the register takes in one place.
 */
static u32 s5kgn8_clamp(u32 val, u32 min, u32 max)
{
	return clamp(val, min, max);
}

/* ---- controls ----------------------------------------------------------- */

static int s5kgn8_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5kgn8 *sensor = ctrl_to_s5kgn8(ctrl);
	struct device *dev = regmap_get_device(sensor->regmap);
	const struct s5kgn8_mode *mode = sensor->mode;
	int ret = 0;

	/*
	 * Lengthening the frame lengthens what can be integrated inside it, so
	 * the exposure range follows frame length rather than being fixed.
	 */
	if (ctrl->id == V4L2_CID_VBLANK) {
		u32 max = mode->height + ctrl->val - S5KGN8_COARSE_INT_MARGIN;

		ret = __v4l2_ctrl_modify_range(sensor->exposure,
					       sensor->exposure->minimum, max,
					       sensor->exposure->step,
					       s5kgn8_clamp(S5KGN8_EXPOSURE_DEFAULT,
							    sensor->exposure->minimum,
							    max));
		if (ret)
			return ret;
	}

	/*
	 * Registers can only be written while the sensor is powered.  Tested
	 * against one rather than zero: with runtime PM compiled out this
	 * returns -EINVAL, which is not a reference to put back.
	 */
	if (pm_runtime_get_if_in_use(dev) != 1)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = s5kgn8_write_held(sensor, S5KGN8_EXPOSURE, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = s5kgn8_write_held(sensor, S5KGN8_ANALOGUE_GAIN,
					ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = s5kgn8_write_held(sensor, S5KGN8_DIGITAL_GAIN, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = s5kgn8_write_held(sensor, S5KGN8_FRAME_LENGTH,
					mode->height + ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5kgn8_ctrl_ops = {
	.s_ctrl = s5kgn8_set_ctrl,
};

static int s5kgn8_init_controls(struct s5kgn8 *sensor)
{
	const struct v4l2_ctrl_ops *ops = &s5kgn8_ctrl_ops;
	struct device *dev = regmap_get_device(sensor->regmap);
	const struct s5kgn8_mode *mode = sensor->mode;
	struct v4l2_ctrl_handler *hdl = &sensor->hdl;
	struct v4l2_fwnode_device_properties props;
	u32 hblank, vblank, exposure_max;
	int ret;

	ret = v4l2_fwnode_device_parse(dev, &props);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(hdl, 8);

	sensor->pixel_rate = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE,
					       mode->pixel_rate,
					       mode->pixel_rate, 1,
					       mode->pixel_rate);
	if (sensor->pixel_rate)
		sensor->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Read-only, because line_length_pck is one of the registers inside the
	 * transcribed mode list and no control changes it.
	 */
	hblank = mode->llp - mode->width;
	sensor->hblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK, hblank,
					   hblank, 1, hblank);
	if (sensor->hblank)
		sensor->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * The frame can be lengthened but not shortened: the vendor's own frame
	 * length is the shortest this mode has been seen to run at, and the
	 * part has not been asked for a frame_length_lines minimum.
	 */
	vblank = mode->fll_def - mode->height;
	sensor->vblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK, vblank,
					   S5KGN8_FRAME_LENGTH_MAX -
					   mode->height, 1, vblank);

	exposure_max = mode->fll_def - S5KGN8_COARSE_INT_MARGIN;
	sensor->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					     S5KGN8_COARSE_INT_MIN,
					     exposure_max, 1,
					     s5kgn8_clamp(S5KGN8_EXPOSURE_DEFAULT,
							  S5KGN8_COARSE_INT_MIN,
							  exposure_max));

	sensor->again = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN,
					  S5KGN8_ANA_GAIN_MIN,
					  S5KGN8_ANA_GAIN_MAX, 1,
					  S5KGN8_ANA_GAIN_DEFAULT);

	sensor->dgain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_DIGITAL_GAIN,
					  S5KGN8_DGTL_GAIN_MIN,
					  S5KGN8_DGTL_GAIN_MAX, 1,
					  S5KGN8_DGTL_GAIN_DEFAULT);

	v4l2_ctrl_new_fwnode_properties(hdl, ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return dev_err_probe(dev, ret, "cannot build the controls\n");
	}

	sensor->sd.ctrl_handler = hdl;

	return 0;
}

/* ---- subdev ------------------------------------------------------------- */

static int s5kgn8_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5kgn8 *sensor = sd_to_s5kgn8(sd);
	struct device *dev = regmap_get_device(sensor->regmap);
	const struct s5kgn8_mode *mode = sensor->mode;
	int ret, stop;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(sensor->regmap, mode->regs, mode->num_regs,
				  NULL);
	if (ret) {
		dev_err(dev, "cannot write the mode list: %d\n", ret);
		goto err_put;
	}

	/*
	 * Stream on immediately after the list, with nothing in between.  The
	 * list's last two writes are a page select and one undecoded register,
	 * and the recording's next write to this part is this one; putting a
	 * group of nine in the gap is a shape the sensor has never been seen in.
	 *
	 * The mode list ends on the CCS page, which is where this belongs.
	 */
	ret = cci_write(sensor->regmap, S5KGN8_MODE_SELECT,
			S5KGN8_MODE_STREAMING, NULL);
	if (ret) {
		dev_err(dev, "cannot start the stream: %d\n", ret);
		goto err_put;
	}

	/*
	 * Exposure, both gains and the frame length, so that a stream runs with
	 * the values that were asked for rather than with the ones the mode
	 * list carries.  After the stream-on and not before it, which is where
	 * the recording puts its own copy of this group: the first frame is
	 * therefore the mode list's exposure, and every frame after it is the
	 * control's.
	 */
	ret = s5kgn8_apply_frame_group(sensor);
	if (ret)
		goto err_stop;

	return 0;

err_stop:
	/*
	 * The part is transmitting, and nothing else will stop it.  The v4l2
	 * core never marked the stream enabled, so it answers a later
	 * disable_streams() with -EALREADY and this driver's teardown is not
	 * reached; the only other thing that ends the transmission is the
	 * autosuspend a second later cutting power to a live sensor.  A retry
	 * inside that second is the real hazard -- it finds the device still
	 * runtime-active and writes the whole mode list into a streaming part.
	 */
	stop = 0;
	s5kgn8_select_ccs_page(sensor, &stop);
	cci_write(sensor->regmap, S5KGN8_MODE_SELECT, S5KGN8_MODE_STANDBY,
		  &stop);
	if (stop)
		dev_err(dev, "cannot stop a stream that failed to start: %d\n",
			stop);
err_put:
	pm_runtime_put_autosuspend(dev);
	return ret;
}

/*
 * Reported but not returned.  A teardown that fails still has to leave the
 * subdev's own streaming state clear, or the next attempt is refused with
 * -EALREADY and the pipeline is stuck until the machine reboots -- a much worse
 * outcome than a sensor that missed its stop.
 */
static int s5kgn8_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5kgn8 *sensor = sd_to_s5kgn8(sd);
	struct device *dev = regmap_get_device(sensor->regmap);
	int ret = 0;

	s5kgn8_select_ccs_page(sensor, &ret);
	cci_write(sensor->regmap, S5KGN8_MODE_SELECT, S5KGN8_MODE_STANDBY,
		  &ret);
	if (ret)
		dev_err(dev, "cannot stop the stream: %d\n", ret);

	pm_runtime_put_autosuspend(dev);

	return 0;
}

/*
 * Powering the sensor is separated from starting it, because the receiver has
 * to be armed against a link that is already alive: it wants the lanes idle and
 * the master clock running before it completes its own start, but it must not
 * see a frame until it is armed.  So the bridge powers the sensor here, arms
 * itself, and only then enables the stream -- which is the order every frame
 * captured from this sensor has been taken in.
 */
static int s5kgn8_pre_streamon(struct v4l2_subdev *sd, u32 flags)
{
	struct s5kgn8 *sensor = sd_to_s5kgn8(sd);

	return pm_runtime_resume_and_get(regmap_get_device(sensor->regmap));
}

static int s5kgn8_post_streamoff(struct v4l2_subdev *sd)
{
	struct s5kgn8 *sensor = sd_to_s5kgn8(sd);

	pm_runtime_put_autosuspend(regmap_get_device(sensor->regmap));

	return 0;
}

static int s5kgn8_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct s5kgn8 *sensor = sd_to_s5kgn8(sd);

	if (code->index)
		return -EINVAL;

	code->code = sensor->mode->code;

	return 0;
}

static int s5kgn8_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct s5kgn8_mode *mode;

	if (fse->index >= ARRAY_SIZE(s5kgn8_modes))
		return -EINVAL;

	mode = &s5kgn8_modes[fse->index];
	if (fse->code != mode->code)
		return -EINVAL;

	fse->min_width = mode->width;
	fse->max_width = mode->width;
	fse->min_height = mode->height;
	fse->max_height = mode->height;

	return 0;
}

/*
 * The analogue crop, which is the window the mode reads out rather than the
 * size it delivers: the format on this pad is the output, and dividing the two
 * is how a consumer recovers the 2x2 binning between them.
 */
static void s5kgn8_fill_crop(struct v4l2_rect *crop)
{
	crop->left = S5KGN8_WINDOW_LEFT;
	crop->top = S5KGN8_WINDOW_TOP;
	crop->width = S5KGN8_WINDOW_WIDTH;
	crop->height = S5KGN8_WINDOW_HEIGHT;
}

static void s5kgn8_fill_format(const struct s5kgn8_mode *mode,
			       struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = mode->code;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

/*
 * One mode, so there is nothing to select between and nothing to re-range: the
 * format is filled in from it whatever is asked for.
 */
static int s5kgn8_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *format)
{
	struct s5kgn8 *sensor = sd_to_s5kgn8(sd);
	const struct s5kgn8_mode *mode = sensor->mode;

	s5kgn8_fill_format(mode, &format->format);
	*v4l2_subdev_state_get_format(state, format->pad) = format->format;
	s5kgn8_fill_crop(v4l2_subdev_state_get_crop(state, format->pad));

	return 0;
}

static int s5kgn8_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, sel->pad);
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
		/* The one mode recovered so far reads out one window. */
		s5kgn8_fill_crop(&sel->r);
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		/*
		 * Zero origin, and large enough to contain the window: a crop
		 * outside its own bounds is what makes a consumer subtract the
		 * two and get a negative offset.
		 */
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = S5KGN8_NATIVE_WIDTH;
		sel->r.height = S5KGN8_NATIVE_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5kgn8_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	s5kgn8_fill_format(&s5kgn8_modes[0],
			   v4l2_subdev_state_get_format(state, 0));
	s5kgn8_fill_crop(v4l2_subdev_state_get_crop(state, 0));

	return 0;
}

static const struct v4l2_subdev_video_ops s5kgn8_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
	.pre_streamon = s5kgn8_pre_streamon,
	.post_streamoff = s5kgn8_post_streamoff,
};

static const struct v4l2_subdev_pad_ops s5kgn8_pad_ops = {
	.enum_mbus_code = s5kgn8_enum_mbus_code,
	.enum_frame_size = s5kgn8_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5kgn8_set_format,
	.get_selection = s5kgn8_get_selection,
	.enable_streams = s5kgn8_enable_streams,
	.disable_streams = s5kgn8_disable_streams,
};

static const struct v4l2_subdev_ops s5kgn8_subdev_ops = {
	.video = &s5kgn8_video_ops,
	.pad = &s5kgn8_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5kgn8_internal_ops = {
	.init_state = s5kgn8_init_state,
};

static const struct media_entity_operations s5kgn8_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* ---- power -------------------------------------------------------------- */

/*
 * The order the vendor stack was recorded powering this module up in: the five
 * rails a millisecond apart, then the reset released, then the master clock.
 * Powering down retraces it, with the same intervals.
 *
 * The recorded list has two more entries before those five, and they are not
 * enabled here because they are not this module's supplies: they are the two
 * board rails feeding the camera PMIC, shared by all three camera modules.
 * The device tree makes them the parents of the rails that *are* this
 * module's, so the regulator core brings each one up before its dependent,
 * with its own 2 ms enable ramp -- twice the millisecond the recorded list
 * leaves between steps.  What is preserved is therefore the constraint (an
 * input rail before what it feeds) rather than the position, and it is the
 * arrangement both of this board's other camera modules already stream on.
 *
 * The master clock pad is part of that order rather than a boot-time setting,
 * which is why the pinctrl state is selected here and not left to the driver
 * core: it is muxed to its CIS_CLK function only once the module is powered
 * and out of reset, and driven low the rest of the time.
 */
static int s5kgn8_power_on(struct s5kgn8 *sensor)
{
	struct device *dev = regmap_get_device(sensor->regmap);
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(s5kgn8_supplies); i++) {
		ret = regulator_enable(sensor->supplies[i].consumer);
		if (ret) {
			dev_err(dev, "cannot enable %s: %d\n",
				s5kgn8_supplies[i].supply, ret);
			goto err_supplies;
		}
		fsleep(1000);
	}

	gpiod_set_value_cansleep(sensor->reset, 0);
	fsleep(1000);

	ret = pinctrl_pm_select_default_state(dev);
	if (ret)
		goto err_reset;

	ret = clk_prepare_enable(sensor->extclk);
	if (ret)
		goto err_pinctrl;

	/*
	 * The recording leaves 10 ms between the master clock and the first
	 * register access.
	 */
	fsleep(10000);

	return 0;

err_pinctrl:
	pinctrl_pm_select_sleep_state(dev);
err_reset:
	gpiod_set_value_cansleep(sensor->reset, 1);
err_supplies:
	while (i--)
		regulator_disable(sensor->supplies[i].consumer);

	return ret;
}

static void s5kgn8_power_off(struct s5kgn8 *sensor)
{
	struct device *dev = regmap_get_device(sensor->regmap);
	unsigned int i = ARRAY_SIZE(s5kgn8_supplies);

	clk_disable_unprepare(sensor->extclk);
	pinctrl_pm_select_sleep_state(dev);
	fsleep(10000);

	gpiod_set_value_cansleep(sensor->reset, 1);
	fsleep(1000);

	while (i--) {
		regulator_disable(sensor->supplies[i].consumer);
		fsleep(1000);
	}
}

/*
 * Initialising the part.  The recording writes 0x6010 once per power-up,
 * immediately after reading the model id, and waits 10 ms; every occurrence in
 * it follows a fresh identification, never a second stream start on a part
 * that is already awake.  So it belongs to the power-up rather than to
 * enable_streams, which with a one-second autosuspend delay would replay it
 * onto a part that never suspended.  The two tables that follow it are the
 * rest of that same once-per-power-up sequence.
 *
 * Both power-ups call it: probe's, which does not come through runtime PM, and
 * every one after it.  Probe's is not optional -- the device is left active for
 * an autosuspend delay afterwards, so a capture started inside that second
 * resumes nothing and would otherwise program the mode list onto a part that
 * had never been initialised, and a device whose runtime PM userspace has
 * forbidden would never get it at all.
 */
static int s5kgn8_init_part(struct s5kgn8 *sensor)
{
	int ret = 0;

	s5kgn8_select_ccs_page(sensor, &ret);
	cci_write(sensor->regmap, S5KGN8_INIT_6010, S5KGN8_INIT_6010_VAL, &ret);
	if (ret)
		return ret;

	fsleep(S5KGN8_INIT_SETTLE_US);

	cci_multi_reg_write(sensor->regmap, s5kgn8_init_port,
			    ARRAY_SIZE(s5kgn8_init_port), &ret);
	cci_multi_reg_write(sensor->regmap, s5kgn8_init_settings,
			    ARRAY_SIZE(s5kgn8_init_settings), &ret);

	return ret;
}

static int s5kgn8_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5kgn8 *sensor = sd_to_s5kgn8(sd);
	int ret;

	ret = s5kgn8_power_on(sensor);
	if (ret)
		return ret;

	ret = s5kgn8_init_part(sensor);
	if (ret) {
		dev_err(dev, "cannot initialise the sensor: %d\n", ret);
		s5kgn8_power_off(sensor);
		return ret;
	}

	return 0;
}

static int s5kgn8_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);

	s5kgn8_power_off(sd_to_s5kgn8(sd));

	return 0;
}

static const struct dev_pm_ops s5kgn8_pm_ops = {
	RUNTIME_PM_OPS(s5kgn8_runtime_suspend, s5kgn8_runtime_resume, NULL)
};

/* ---- probe -------------------------------------------------------------- */

/*
 * Read before any page has been selected, exactly as the recording does: the
 * part answers the model id on whatever page it resets to, and selecting one
 * first would be an assumption about a register this has not identified.
 */
static int s5kgn8_identify(struct s5kgn8 *sensor)
{
	struct device *dev = regmap_get_device(sensor->regmap);
	u64 id;
	int ret;

	ret = cci_read(sensor->regmap, S5KGN8_MODEL_ID, &id, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read the model id\n");

	if (id != S5KGN8_MODEL_ID_VAL)
		return dev_err_probe(dev, -ENODEV,
				     "model id %#06llx, expected %#06x\n", id,
				     S5KGN8_MODEL_ID_VAL);

	return 0;
}

/*
 * The endpoint says what the receiver on the other end was configured for, and
 * the transcribed mode list only produces one thing: three C-PHY lanes.  Refuse
 * anything else rather than stream into a receiver set up for a link this mode
 * does not drive.  No link frequency is checked because none is claimed.
 */
static int s5kgn8_parse_endpoint(struct s5kgn8 *sensor)
{
	struct device *dev = regmap_get_device(sensor->regmap);
	struct v4l2_fwnode_endpoint bus = {
		.bus_type = V4L2_MBUS_CSI2_CPHY,
	};
	struct fwnode_handle *ep;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!ep)
		return dev_err_probe(dev, -ENXIO, "no endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(dev, ret, "cannot parse the endpoint\n");

	if (bus.bus.mipi_csi2.num_data_lanes != S5KGN8_DATA_LANES)
		ret = dev_err_probe(dev, -EINVAL, "%u data lanes, expected %u\n",
				    bus.bus.mipi_csi2.num_data_lanes,
				    S5KGN8_DATA_LANES);

	v4l2_fwnode_endpoint_free(&bus);

	return ret;
}

static int s5kgn8_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5kgn8 *sensor;
	unsigned long rate;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->mode = &s5kgn8_modes[0];

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(dev, PTR_ERR(sensor->regmap),
				     "cannot make the register map\n");

	sensor->extclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(sensor->extclk))
		return dev_err_probe(dev, PTR_ERR(sensor->extclk),
				     "no master clock\n");

	/*
	 * The power-up sequence's PLL dividers are constants, so the pixel rate
	 * this driver reports is only true at one input rate.
	 */
	rate = clk_get_rate(sensor->extclk);
	if (rate != S5KGN8_EXTCLK_RATE)
		return dev_err_probe(dev, -EINVAL,
				     "master clock is %lu Hz, need %lu Hz\n",
				     rate, S5KGN8_EXTCLK_RATE);

	/* Held asserted until the rails are up. */
	sensor->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(dev, PTR_ERR(sensor->reset),
				     "no reset GPIO\n");

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(s5kgn8_supplies),
					    s5kgn8_supplies, &sensor->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "cannot get the supplies\n");

	ret = s5kgn8_parse_endpoint(sensor);
	if (ret)
		return ret;

	/*
	 * The driver core applies the default pin state before probe runs, so
	 * the master clock pad is already muxed to CIS_CLK with no rail behind
	 * it.  Park it before the first power-on so that the recorded order --
	 * rails, reset, then the clock pad -- holds from the very first one.
	 */
	ret = pinctrl_pm_select_sleep_state(dev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot park the clock pad\n");

	ret = s5kgn8_power_on(sensor);
	if (ret)
		return dev_err_probe(dev, ret, "cannot power the sensor\n");

	ret = s5kgn8_identify(sensor);
	if (ret)
		goto err_power;

	/* In the recording's own order: the model id, then this. */
	ret = s5kgn8_init_part(sensor);
	if (ret) {
		dev_err_probe(dev, ret, "cannot initialise the sensor\n");
		goto err_power;
	}

	v4l2_i2c_subdev_init(&sensor->sd, client, &s5kgn8_subdev_ops);
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->sd.internal_ops = &s5kgn8_internal_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->sd.entity.ops = &s5kgn8_entity_ops;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto err_power;

	ret = s5kgn8_init_controls(sensor);
	if (ret)
		goto err_entity;

	/*
	 * Share one lock between the subdev state and the control handler, so
	 * that enable_streams -- which holds the state lock -- may read the
	 * control values without taking the handler's lock a second time.
	 */
	sensor->sd.state_lock = sensor->hdl.lock;

	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret)
		goto err_ctrls;

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret) {
		dev_err_probe(dev, ret, "cannot register the subdev\n");
		goto err_pm;
	}

	pm_runtime_put_autosuspend(dev);

	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&sensor->sd);
err_ctrls:
	v4l2_ctrl_handler_free(&sensor->hdl);
err_entity:
	media_entity_cleanup(&sensor->sd.entity);
err_power:
	s5kgn8_power_off(sensor);

	return ret;
}

static void s5kgn8_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5kgn8 *sensor = sd_to_s5kgn8(sd);
	struct device *dev = &client->dev;

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	v4l2_ctrl_handler_free(&sensor->hdl);
	media_entity_cleanup(&sd->entity);

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		s5kgn8_power_off(sensor);
	pm_runtime_set_suspended(dev);
}

static const struct of_device_id s5kgn8_of_match[] = {
	{ .compatible = "samsung,s5kgn8" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5kgn8_of_match);

static struct i2c_driver s5kgn8_i2c_driver = {
	.driver = {
		.name = "s5kgn8",
		.of_match_table = s5kgn8_of_match,
		.pm = pm_ptr(&s5kgn8_pm_ops),
	},
	.probe = s5kgn8_probe,
	.remove = s5kgn8_remove,
};
module_i2c_driver(s5kgn8_i2c_driver);

MODULE_DESCRIPTION("Samsung S5KGN8 image sensor driver");
MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_LICENSE("GPL");
