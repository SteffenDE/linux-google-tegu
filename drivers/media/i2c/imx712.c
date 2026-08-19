// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sony IMX712 CMOS image sensor.
 *
 * The part is an SMIA/CCS sensor with 16-bit register addresses and 8-bit
 * values, no paging.  Everything that defines a mode is standard: the PLL
 * block at 0x0300, the geometry at 0x0340..0x0386, and the per-frame
 * exposure/gain set at 0x0202/0x0204/0x020e behind the grouped-parameter hold
 * at 0x0104.
 *
 * The mode register list below is a transcription of what the vendor camera
 * stack writes to the sensor, recorded register by register from a live
 * session, and it is kept in the order it was recorded: several of those
 * registers are written twice with different values, so the sequence carries
 * information the sorted set does not.
 *
 * Copyright 2026 Steffen Deusch
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/math.h>
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

/* Identification: 0x0016/0x0017 read 0x07/0x12 on both parts in this phone. */
#define IMX712_MODEL_ID			CCI_REG16(0x0016)
#define IMX712_MODEL_ID_VAL		0x0712

#define IMX712_MODE_SELECT		CCI_REG8(0x0100)
#define   IMX712_MODE_STANDBY		0x00
#define   IMX712_MODE_STREAMING		0x01

/* Grouped parameter hold: everything written while it is set lands together. */
#define IMX712_GROUPED_HOLD		CCI_REG8(0x0104)

#define IMX712_EXPOSURE			CCI_REG16(0x0202)
#define IMX712_ANALOGUE_GAIN		CCI_REG16(0x0204)
#define IMX712_DIGITAL_GAIN		CCI_REG16(0x020e)
#define IMX712_FRAME_LENGTH		CCI_REG16(0x0340)

/*
 * CCS capability registers.  These describe the part rather than configure it,
 * so they are read once at probe and used to build the control ranges instead
 * of hard-coding limits nobody has measured.  Each has a fallback below for
 * the case where the sensor does not implement it.
 */
#define IMX712_ANA_GAIN_CODE_MIN	CCI_REG16(0x0084)
#define IMX712_ANA_GAIN_CODE_MAX	CCI_REG16(0x0086)
#define IMX712_ANA_GAIN_CODE_STEP	CCI_REG16(0x0088)
#define IMX712_DGTL_GAIN_MIN		CCI_REG16(0x1084)
#define IMX712_DGTL_GAIN_MAX		CCI_REG16(0x1086)
#define IMX712_DGTL_GAIN_STEP		CCI_REG16(0x1088)
#define IMX712_COARSE_INT_MIN		CCI_REG16(0x1004)
#define IMX712_COARSE_INT_MAX_MARGIN	CCI_REG16(0x1006)

/*
 * One register inside the vendor's first exposure group that nothing decodes.
 * It is written once, to zero, between the gain and the hold release; replayed
 * as measured because every frame captured so far was taken with it there.
 */
#define IMX712_UNDECODED_3004		CCI_REG8(0x3004)

/*
 * The captured per-frame values, used as the control defaults so that a
 * capture taken with no controls set is the one the receive path was brought
 * up against.
 */
#define IMX712_EXPOSURE_DEFAULT		1924
#define IMX712_ANA_GAIN_DEFAULT		777
#define IMX712_DGTL_GAIN_DEFAULT	0x0100

/*
 * Fallbacks for the CCS capability registers.  The gain fallbacks bracket the
 * captured value without claiming a maximum nobody has read; the integration
 * margin is the one the bring-up captures were validated at, where a 3269-line
 * frame accepted a coarse integration time of 3198.
 */
#define IMX712_ANA_GAIN_MAX_FALLBACK	IMX712_ANA_GAIN_DEFAULT
#define IMX712_DGTL_GAIN_MAX_FALLBACK	0x0fff
#define IMX712_COARSE_INT_MIN_FALLBACK	1
#define IMX712_COARSE_INT_MARGIN_FALLBACK 71

/* frame_length_lines and line_length_pck are both 16-bit CCS fields. */
#define IMX712_FRAME_LENGTH_MAX		0xffff

/*
 * The external clock the mode list tells the sensor it has: 0x0136/0x0137 are
 * written 0x18/0x93, which is 24.574 MHz in the CCS Q8.8 encoding.  What the
 * SoC actually delivers is oscclk undivided, 24.576 MHz, and the difference is
 * below the resolution of the register.  The PLL arithmetic below uses the
 * real rate.
 */
#define IMX712_EXTCLK_RATE		24576000UL

/*
 * The pixel clock the mode list's PLL produces, and the factor that closes the
 * timing model.  pre_pll_clk_div 3, pll_multiplier 122, vt_sys_clk_div 2 and
 * vt_pix_clk_div 4 give 24.576 / 3 * 122 / (2 * 4) = 124.928 MHz.  Four pixels
 * per vt_pix clock make the frame period 5192 * 3269 / 499712000 = 33.97 ms,
 * against a measured frame interval of 34.03 ms over a whole session.
 */
#define IMX712_PIXELS_PER_VT_CLK	4
#define IMX712_PIXEL_RATE		499712000LL

/*
 * The output PLL: op_pre_pll_clk_div 3, op_pll_multiplier 135 and
 * op_sys_clk_div 1 give 1105.92 MHz, so 552.96 MHz on the clock lane and
 * 1.10592 Gbps on each of the four data lanes.
 *
 * The mode list corroborates that once and then muddies it.  Its first write to
 * 0x0820/0x0821 puts 0x1147 in the top half of the CCS requested_link_rate
 * field, which is 4423 Mbps -- those four lanes added up.  It later writes
 * 0x0820 again, on its own, with 0x47, which that reading does not explain and
 * which nothing else here decodes.  Both writes are replayed in the order they
 * were recorded; the link frequency reported to userspace comes from the PLL
 * arithmetic above rather than from either of them.
 */
#define IMX712_LINK_FREQ		552960000LL
#define IMX712_DATA_LANES		4

#define IMX712_NATIVE_WIDTH		4208U
#define IMX712_NATIVE_HEIGHT		3120U

/* In the order the recording brings them up. */
static const struct regulator_bulk_data imx712_supplies[] = {
	{ .supply = "iovdd" },	/* 1.8 V interface, shared by all modules */
	{ .supply = "avdd" },	/* 2.8 V analogue */
	{ .supply = "dvdd" },	/* 1.2 V digital */
};

struct imx712_mode {
	u32 width;
	u32 height;
	/* line_length_pck, as the register list programs it. */
	u32 llp;
	/* frame_length_lines the vendor applies with its exposure group. */
	u32 fll_def;
	u32 code;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct imx712_limits {
	u32 ana_gain_min;
	u32 ana_gain_max;
	u32 ana_gain_step;
	u32 dgtl_gain_min;
	u32 dgtl_gain_max;
	u32 dgtl_gain_step;
	u32 exposure_min;
	u32 exposure_margin;
};

struct imx712 {
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

	const struct imx712_mode *mode;
	struct imx712_limits limits;
};

static inline struct imx712 *sd_to_imx712(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx712, sd);
}

static inline struct imx712 *ctrl_to_imx712(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct imx712, hdl);
}

/*
 * The full-array preview mode, 4208x3120 at 29.4 fps, transcribed from a
 * recording of the vendor stack programming this sensor.  126 registers in the
 * order they were written; the geometry inside it decodes as
 *
 *   0x0112/0x0113 = 0x0a/0x0a  RAW10 in, RAW10 out
 *   0x0114 = 3                 four CSI-2 data lanes
 *   0x0342 = 5192              line_length_pck
 *   0x0340 = 3208              frame_length_lines, raised to 3269 by the
 *                              exposure group the controls now own
 *   0x0344..0x034b             crop 0,0 to 4207,3119 -- the whole array
 *   0x034c = 4208, 0x034e = 3120
 *   0x0900 = 0                 no binning
 *   0x0101 = 0                 no flip, so the mosaic reads out RGGB
 *   0x0301/0x0303/0x0305/0x0306 = 4/2/3/122   pixel-clock PLL
 *   0x030b/0x030d/0x030e = 1/3/135            output PLL
 *
 * Nothing here is reordered or deduplicated.  0x484c..0x4853 and 0x0820 are
 * each written twice with different values, and which write lands last is the
 * one that counts.
 */
static const struct cci_reg_sequence imx712_mode_4208x3120[] = {
	{ CCI_REG8(0x0136), 0x18 }, { CCI_REG8(0x0137), 0x93 }, { CCI_REG8(0x3040), 0x01 },
	{ CCI_REG8(0x3041), 0x01 }, { CCI_REG8(0x0101), 0x00 }, { CCI_REG8(0x300b), 0x00 },
	{ CCI_REG8(0x4350), 0x3c }, { CCI_REG8(0x4516), 0x3e }, { CCI_REG8(0x4518), 0x56 },
	{ CCI_REG8(0x451a), 0x02 }, { CCI_REG8(0x451b), 0xdb }, { CCI_REG8(0x4527), 0x00 },
	{ CCI_REG8(0x4528), 0xff }, { CCI_REG8(0x45be), 0x4b }, { CCI_REG8(0x45bf), 0xcc },
	{ CCI_REG8(0x45c6), 0x4b }, { CCI_REG8(0x45c7), 0xcc }, { CCI_REG8(0x4702), 0x0a },
	{ CCI_REG8(0x4828), 0x08 }, { CCI_REG8(0x484c), 0x2b }, { CCI_REG8(0x484d), 0x2b },
	{ CCI_REG8(0x484e), 0x2b }, { CCI_REG8(0x484f), 0x2b }, { CCI_REG8(0x4850), 0x20 },
	{ CCI_REG8(0x4851), 0x20 }, { CCI_REG8(0x4852), 0x20 }, { CCI_REG8(0x4853), 0x20 },
	{ CCI_REG8(0x4856), 0x05 }, { CCI_REG8(0x4857), 0x05 }, { CCI_REG8(0x4858), 0x05 },
	{ CCI_REG8(0x4859), 0x05 }, { CCI_REG8(0x485a), 0x04 }, { CCI_REG8(0x485b), 0x04 },
	{ CCI_REG8(0x485c), 0x04 }, { CCI_REG8(0x485d), 0x04 }, { CCI_REG8(0x4879), 0x00 },
	{ CCI_REG8(0x4899), 0x0a }, { CCI_REG8(0x48a9), 0x2e }, { CCI_REG8(0x6a10), 0x0a },
	{ CCI_REG8(0x70b0), 0x00 }, { CCI_REG8(0x70a8), 0xff }, { CCI_REG8(0x4012), 0x00 },
	{ CCI_REG8(0x4013), 0xdf }, { CCI_REG8(0x7088), 0x00 }, { CCI_REG8(0x4108), 0x01 },
	{ CCI_REG8(0x4109), 0x07 }, { CCI_REG8(0x0106), 0x01 }, { CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a }, { CCI_REG8(0x0114), 0x03 }, { CCI_REG8(0x0342), 0x14 },
	{ CCI_REG8(0x0343), 0x48 }, { CCI_REG8(0x0340), 0x0c }, { CCI_REG8(0x0341), 0x88 },
	{ CCI_REG8(0x0344), 0x00 }, { CCI_REG8(0x0345), 0x00 }, { CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 }, { CCI_REG8(0x0348), 0x10 }, { CCI_REG8(0x0349), 0x6f },
	{ CCI_REG8(0x034a), 0x0c }, { CCI_REG8(0x034b), 0x2f }, { CCI_REG8(0x0900), 0x00 },
	{ CCI_REG8(0x0901), 0x11 }, { CCI_REG8(0x0902), 0x00 }, { CCI_REG8(0x3130), 0x01 },
	{ CCI_REG8(0x034c), 0x10 }, { CCI_REG8(0x034d), 0x70 }, { CCI_REG8(0x034e), 0x0c },
	{ CCI_REG8(0x034f), 0x30 }, { CCI_REG8(0x0301), 0x04 }, { CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0305), 0x03 }, { CCI_REG8(0x0306), 0x00 }, { CCI_REG8(0x0307), 0x7a },
	{ CCI_REG8(0x030b), 0x01 }, { CCI_REG8(0x030d), 0x03 }, { CCI_REG8(0x030e), 0x00 },
	{ CCI_REG8(0x030f), 0x87 }, { CCI_REG8(0x0700), 0x00 }, { CCI_REG8(0x0701), 0x30 },
	{ CCI_REG8(0x0820), 0x11 }, { CCI_REG8(0x0821), 0x47 }, { CCI_REG8(0x3100), 0x05 },
	{ CCI_REG8(0x7013), 0x01 }, { CCI_REG8(0x0202), 0x0c }, { CCI_REG8(0x0203), 0x7e },
	{ CCI_REG8(0x0204), 0x00 }, { CCI_REG8(0x0205), 0x00 }, { CCI_REG8(0x020e), 0x01 },
	{ CCI_REG8(0x020f), 0x00 }, { CCI_REG8(0x0b06), 0x01 }, { CCI_REG8(0x484c), 0x54 },
	{ CCI_REG8(0x484d), 0x54 }, { CCI_REG8(0x484e), 0x54 }, { CCI_REG8(0x484f), 0x54 },
	{ CCI_REG8(0x4850), 0x20 }, { CCI_REG8(0x4851), 0x20 }, { CCI_REG8(0x4852), 0x20 },
	{ CCI_REG8(0x4853), 0x20 }, { CCI_REG8(0x500c), 0x01 }, { CCI_REG8(0x500d), 0x72 },
	{ CCI_REG8(0x500e), 0x00 }, { CCI_REG8(0x500f), 0x28 }, { CCI_REG8(0x0808), 0x02 },
	{ CCI_REG8(0x080a), 0x00 }, { CCI_REG8(0x080b), 0x87 }, { CCI_REG8(0x080c), 0x00 },
	{ CCI_REG8(0x080d), 0x47 }, { CCI_REG8(0x080e), 0x00 }, { CCI_REG8(0x080f), 0x87 },
	{ CCI_REG8(0x0810), 0x00 }, { CCI_REG8(0x0811), 0x47 }, { CCI_REG8(0x0812), 0x00 },
	{ CCI_REG8(0x0813), 0x47 }, { CCI_REG8(0x0814), 0x00 }, { CCI_REG8(0x0815), 0x47 },
	{ CCI_REG8(0x0816), 0x01 }, { CCI_REG8(0x0817), 0x27 }, { CCI_REG8(0x0818), 0x00 },
	{ CCI_REG8(0x0819), 0x3f }, { CCI_REG8(0x0820), 0x47 }, { CCI_REG8(0x3122), 0x00 },
	{ CCI_REG8(0x3123), 0x77 }, { CCI_REG8(0x3120), 0x00 }, { CCI_REG8(0x3121), 0x0f },
};

static const struct imx712_mode imx712_modes[] = {
	{
		.width = IMX712_NATIVE_WIDTH,
		.height = IMX712_NATIVE_HEIGHT,
		.llp = 5192,
		.fll_def = 3269,
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.regs = imx712_mode_4208x3120,
		.num_regs = ARRAY_SIZE(imx712_mode_4208x3120),
	},
};

static const s64 imx712_link_freq[] = {
	IMX712_LINK_FREQ,
};

/* ---- register helpers --------------------------------------------------- */

/*
 * Every per-frame value the vendor stack changes is written inside the
 * grouped-parameter hold, so that exposure, gain and frame length take effect
 * on the same frame boundary rather than one apiece.
 */
static int imx712_write_held(struct imx712 *sensor, u32 reg, u64 val)
{
	int ret = 0, release;

	cci_write(sensor->regmap, IMX712_GROUPED_HOLD, 1, &ret);
	cci_write(sensor->regmap, reg, val, &ret);
	/*
	 * Released with its own error, because cci_write() skips a write when
	 * the accumulated one is already set -- and a sensor left holding stops
	 * latching per-frame updates for the rest of the session, which reads
	 * as "the controls do nothing" rather than as an i2c failure.
	 */
	release = cci_write(sensor->regmap, IMX712_GROUPED_HOLD, 0, NULL);

	return ret ? ret : release;
}

/*
 * The whole per-frame set, in the order and the single hold the recording puts
 * it in, so that the values a stream starts with land on one frame boundary
 * exactly as they were captured.
 */
static int imx712_apply_frame_group(struct imx712 *sensor)
{
	int ret = 0, release;

	cci_write(sensor->regmap, IMX712_GROUPED_HOLD, 1, &ret);
	cci_write(sensor->regmap, IMX712_DIGITAL_GAIN,
		  v4l2_ctrl_g_ctrl(sensor->dgain), &ret);
	cci_write(sensor->regmap, IMX712_ANALOGUE_GAIN,
		  v4l2_ctrl_g_ctrl(sensor->again), &ret);
	cci_write(sensor->regmap, IMX712_EXPOSURE,
		  v4l2_ctrl_g_ctrl(sensor->exposure), &ret);
	cci_write(sensor->regmap, IMX712_FRAME_LENGTH,
		  sensor->mode->height + v4l2_ctrl_g_ctrl(sensor->vblank),
		  &ret);
	cci_write(sensor->regmap, IMX712_UNDECODED_3004, 0, &ret);
	release = cci_write(sensor->regmap, IMX712_GROUPED_HOLD, 0, NULL);

	return ret ? ret : release;
}

/*
 * Read a CCS min/max/step capability triple, and keep it only if it describes
 * a usable range.  A part that does not implement these reads back zeros, and
 * a zero maximum would otherwise become a control that cannot be set at all.
 */
static void imx712_read_range(struct imx712 *sensor, u32 reg_min, u32 reg_max,
			      u32 reg_step, const char *what,
			      u32 *min, u32 *max, u32 *step)
{
	struct device *dev = regmap_get_device(sensor->regmap);
	u64 v_min, v_max, v_step;
	int ret = 0;

	cci_read(sensor->regmap, reg_min, &v_min, &ret);
	cci_read(sensor->regmap, reg_max, &v_max, &ret);
	cci_read(sensor->regmap, reg_step, &v_step, &ret);
	if (ret || !v_step || v_max <= v_min) {
		dev_info(dev,
			 "%s capability unusable (%llu..%llu/%llu), keeping %u..%u/%u\n",
			 what, v_min, v_max, v_step, *min, *max, *step);
		return;
	}

	*min = v_min;
	*max = v_max;
	*step = v_step;
	dev_dbg(dev, "%s %u..%u/%u\n", what, *min, *max, *step);
}

/*
 * What the part says about itself.  Everything here has a fallback, because
 * these registers describe capabilities rather than configure anything and
 * nothing in the capture recorded them being read.
 */
static void imx712_read_limits(struct imx712 *sensor)
{
	struct device *dev = regmap_get_device(sensor->regmap);
	struct imx712_limits *lim = &sensor->limits;
	u64 val;

	lim->ana_gain_min = 0;
	lim->ana_gain_max = IMX712_ANA_GAIN_MAX_FALLBACK;
	lim->ana_gain_step = 1;
	imx712_read_range(sensor, IMX712_ANA_GAIN_CODE_MIN,
			  IMX712_ANA_GAIN_CODE_MAX, IMX712_ANA_GAIN_CODE_STEP,
			  "analogue gain", &lim->ana_gain_min,
			  &lim->ana_gain_max, &lim->ana_gain_step);

	lim->dgtl_gain_min = IMX712_DGTL_GAIN_DEFAULT;
	lim->dgtl_gain_max = IMX712_DGTL_GAIN_MAX_FALLBACK;
	lim->dgtl_gain_step = 1;
	imx712_read_range(sensor, IMX712_DGTL_GAIN_MIN, IMX712_DGTL_GAIN_MAX,
			  IMX712_DGTL_GAIN_STEP, "digital gain",
			  &lim->dgtl_gain_min, &lim->dgtl_gain_max,
			  &lim->dgtl_gain_step);

	lim->exposure_min = IMX712_COARSE_INT_MIN_FALLBACK;
	if (!cci_read(sensor->regmap, IMX712_COARSE_INT_MIN, &val, NULL) &&
	    val && val < IMX712_NATIVE_HEIGHT)
		lim->exposure_min = val;

	lim->exposure_margin = IMX712_COARSE_INT_MARGIN_FALLBACK;
	if (!cci_read(sensor->regmap, IMX712_COARSE_INT_MAX_MARGIN, &val,
		      NULL) && val && val < IMX712_NATIVE_HEIGHT)
		lim->exposure_margin = val;

	/*
	 * A window this mode's frame cannot contain is worse than no reading at
	 * all: the control handler refuses an inverted range, so probe would
	 * fail with the part's own capability registers as the cause.
	 */
	if (lim->exposure_min + lim->exposure_margin >= sensor->mode->fll_def) {
		dev_info(dev,
			 "coarse integration %u..frame_length-%u does not fit %u lines; keeping %u..frame_length-%u\n",
			 lim->exposure_min, lim->exposure_margin,
			 sensor->mode->fll_def, IMX712_COARSE_INT_MIN_FALLBACK,
			 IMX712_COARSE_INT_MARGIN_FALLBACK);
		lim->exposure_min = IMX712_COARSE_INT_MIN_FALLBACK;
		lim->exposure_margin = IMX712_COARSE_INT_MARGIN_FALLBACK;
	}

	dev_dbg(dev, "coarse integration %u..frame_length-%u\n",
		lim->exposure_min, lim->exposure_margin);
}

/* Round a default down onto the control's own grid so it is a legal value. */
static u32 imx712_snap(u32 val, u32 min, u32 max, u32 step)
{
	val = clamp(val, min, max);

	return min + rounddown(val - min, step);
}

/* ---- controls ----------------------------------------------------------- */

static int imx712_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx712 *sensor = ctrl_to_imx712(ctrl);
	struct device *dev = regmap_get_device(sensor->regmap);
	const struct imx712_mode *mode = sensor->mode;
	int ret = 0;

	/*
	 * Lengthening the frame lengthens what can be integrated inside it, so
	 * the exposure range follows frame length rather than being fixed.
	 */
	if (ctrl->id == V4L2_CID_VBLANK) {
		u32 max = mode->height + ctrl->val - sensor->limits.exposure_margin;

		ret = __v4l2_ctrl_modify_range(sensor->exposure,
					       sensor->exposure->minimum, max,
					       sensor->exposure->step,
					       imx712_snap(IMX712_EXPOSURE_DEFAULT,
							   sensor->exposure->minimum,
							   max, 1));
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
		ret = imx712_write_held(sensor, IMX712_EXPOSURE, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx712_write_held(sensor, IMX712_ANALOGUE_GAIN,
					ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx712_write_held(sensor, IMX712_DIGITAL_GAIN, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = imx712_write_held(sensor, IMX712_FRAME_LENGTH,
					mode->height + ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx712_ctrl_ops = {
	.s_ctrl = imx712_set_ctrl,
};

static int imx712_init_controls(struct imx712 *sensor)
{
	const struct v4l2_ctrl_ops *ops = &imx712_ctrl_ops;
	struct device *dev = regmap_get_device(sensor->regmap);
	const struct imx712_limits *lim = &sensor->limits;
	const struct imx712_mode *mode = sensor->mode;
	struct v4l2_ctrl_handler *hdl = &sensor->hdl;
	struct v4l2_fwnode_device_properties props;
	u32 hblank, vblank, exposure_max;
	struct v4l2_ctrl *ctrl;
	int ret;

	ret = v4l2_fwnode_device_parse(dev, &props);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(hdl, 10);

	ctrl = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE,
				 IMX712_PIXEL_RATE, IMX712_PIXEL_RATE, 1,
				 IMX712_PIXEL_RATE);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(imx712_link_freq) - 1, 0,
				      imx712_link_freq);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Read-only, because line_length_pck is one of the registers inside the
	 * transcribed mode list and no capture ever changes it.
	 */
	hblank = mode->llp - mode->width;
	sensor->hblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK, hblank,
					   hblank, 1, hblank);
	if (sensor->hblank)
		sensor->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * The frame can be lengthened but not shortened: the vendor's own frame
	 * length is the smallest this mode has been seen to run at, and the
	 * part has not been asked for a frame_length_lines minimum.
	 */
	vblank = mode->fll_def - mode->height;
	sensor->vblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK, vblank,
					   IMX712_FRAME_LENGTH_MAX -
					   mode->height, 1, vblank);

	exposure_max = mode->fll_def - lim->exposure_margin;
	sensor->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					     lim->exposure_min, exposure_max, 1,
					     imx712_snap(IMX712_EXPOSURE_DEFAULT,
							 lim->exposure_min,
							 exposure_max, 1));

	sensor->again = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN,
					  lim->ana_gain_min, lim->ana_gain_max,
					  lim->ana_gain_step,
					  imx712_snap(IMX712_ANA_GAIN_DEFAULT,
						      lim->ana_gain_min,
						      lim->ana_gain_max,
						      lim->ana_gain_step));

	sensor->dgain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_DIGITAL_GAIN,
					  lim->dgtl_gain_min,
					  lim->dgtl_gain_max,
					  lim->dgtl_gain_step,
					  imx712_snap(IMX712_DGTL_GAIN_DEFAULT,
						      lim->dgtl_gain_min,
						      lim->dgtl_gain_max,
						      lim->dgtl_gain_step));

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

static int imx712_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx712 *sensor = sd_to_imx712(sd);
	struct device *dev = regmap_get_device(sensor->regmap);
	const struct imx712_mode *mode = sensor->mode;
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(sensor->regmap, mode->regs, mode->num_regs,
				  NULL);
	if (ret) {
		dev_err(dev, "cannot write the mode list: %d\n", ret);
		goto err_put;
	}

	/* Exposure, both gains and the frame length, as the mode's defaults. */
	ret = imx712_apply_frame_group(sensor);
	if (ret)
		goto err_put;

	ret = cci_write(sensor->regmap, IMX712_MODE_SELECT,
			IMX712_MODE_STREAMING, NULL);
	if (ret) {
		dev_err(dev, "cannot start the stream: %d\n", ret);
		goto err_put;
	}

	return 0;

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
static int imx712_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx712 *sensor = sd_to_imx712(sd);
	struct device *dev = regmap_get_device(sensor->regmap);
	int ret;

	ret = cci_write(sensor->regmap, IMX712_MODE_SELECT,
			IMX712_MODE_STANDBY, NULL);
	if (ret)
		dev_err(dev, "cannot stop the stream: %d\n", ret);

	pm_runtime_put_autosuspend(dev);

	return 0;
}

/*
 * Powering the sensor is separated from starting it, because the receiver has
 * to be armed against a link that is already alive: it wants the lanes in LP-11
 * and the master clock running before it completes its own start, but it must
 * not see a frame until it is armed.  So the bridge powers the sensor here,
 * arms itself, and only then enables the stream -- which is the order every
 * frame captured from this sensor has been taken in.
 */
static int imx712_pre_streamon(struct v4l2_subdev *sd, u32 flags)
{
	struct imx712 *sensor = sd_to_imx712(sd);

	return pm_runtime_resume_and_get(regmap_get_device(sensor->regmap));
}

static int imx712_post_streamoff(struct v4l2_subdev *sd)
{
	struct imx712 *sensor = sd_to_imx712(sd);

	pm_runtime_put_autosuspend(regmap_get_device(sensor->regmap));

	return 0;
}

static int imx712_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx712 *sensor = sd_to_imx712(sd);

	if (code->index)
		return -EINVAL;

	code->code = sensor->mode->code;

	return 0;
}

static int imx712_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct imx712_mode *mode;

	if (fse->index >= ARRAY_SIZE(imx712_modes))
		return -EINVAL;

	mode = &imx712_modes[fse->index];
	if (fse->code != mode->code)
		return -EINVAL;

	fse->min_width = mode->width;
	fse->max_width = mode->width;
	fse->min_height = mode->height;
	fse->max_height = mode->height;

	return 0;
}

static void imx712_fill_format(const struct imx712_mode *mode,
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

static int imx712_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *format)
{
	struct imx712 *sensor = sd_to_imx712(sd);
	const struct imx712_mode *mode;

	mode = v4l2_find_nearest_size(imx712_modes, ARRAY_SIZE(imx712_modes),
				      width, height, format->format.width,
				      format->format.height);

	imx712_fill_format(mode, &format->format);
	*v4l2_subdev_state_get_format(state, format->pad) = format->format;
	*v4l2_subdev_state_get_crop(state, format->pad) = (struct v4l2_rect){
		.width = mode->width,
		.height = mode->height,
	};

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		sensor->mode = mode;

	return 0;
}

static int imx712_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, sel->pad);
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		/*
		 * The one recovered mode reads the whole array out, so there is
		 * nothing to crop and the bounds are the array itself.
		 */
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = IMX712_NATIVE_WIDTH;
		sel->r.height = IMX712_NATIVE_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int imx712_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	const struct imx712_mode *mode = &imx712_modes[0];
	struct v4l2_rect *crop = v4l2_subdev_state_get_crop(state, 0);

	imx712_fill_format(mode, v4l2_subdev_state_get_format(state, 0));
	crop->left = 0;
	crop->top = 0;
	crop->width = mode->width;
	crop->height = mode->height;

	return 0;
}

static const struct v4l2_subdev_video_ops imx712_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
	.pre_streamon = imx712_pre_streamon,
	.post_streamoff = imx712_post_streamoff,
};

static const struct v4l2_subdev_pad_ops imx712_pad_ops = {
	.enum_mbus_code = imx712_enum_mbus_code,
	.enum_frame_size = imx712_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx712_set_format,
	.get_selection = imx712_get_selection,
	.enable_streams = imx712_enable_streams,
	.disable_streams = imx712_disable_streams,
};

static const struct v4l2_subdev_ops imx712_subdev_ops = {
	.video = &imx712_video_ops,
	.pad = &imx712_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx712_internal_ops = {
	.init_state = imx712_init_state,
};

static const struct media_entity_operations imx712_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* ---- power -------------------------------------------------------------- */

/*
 * The order the vendor stack was recorded powering this module up in:
 * interface rail, analogue, digital, a millisecond apart, then the reset
 * released, then the master clock.  Powering down retraces it, with the same
 * intervals.
 *
 * The master clock pad is part of that order rather than a boot-time setting,
 * which is why the pinctrl state is selected here and not left to the driver
 * core: it is muxed to its CIS_CLK function only once the module is powered
 * and out of reset, and driven low the rest of the time.
 */
static int imx712_power_on(struct imx712 *sensor)
{
	struct device *dev = regmap_get_device(sensor->regmap);
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(imx712_supplies); i++) {
		ret = regulator_enable(sensor->supplies[i].consumer);
		if (ret) {
			dev_err(dev, "cannot enable %s: %d\n",
				imx712_supplies[i].supply, ret);
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

	/* The recording leaves 10 ms between the master clock and the first
	 * register write.
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

static void imx712_power_off(struct imx712 *sensor)
{
	struct device *dev = regmap_get_device(sensor->regmap);
	unsigned int i = ARRAY_SIZE(imx712_supplies);

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

static int imx712_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);

	return imx712_power_on(sd_to_imx712(sd));
}

static int imx712_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);

	imx712_power_off(sd_to_imx712(sd));

	return 0;
}

static const struct dev_pm_ops imx712_pm_ops = {
	RUNTIME_PM_OPS(imx712_runtime_suspend, imx712_runtime_resume, NULL)
};

/* ---- probe -------------------------------------------------------------- */

static int imx712_identify(struct imx712 *sensor)
{
	struct device *dev = regmap_get_device(sensor->regmap);
	u64 id;
	int ret;

	ret = cci_read(sensor->regmap, IMX712_MODEL_ID, &id, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read the model id\n");

	if (id != IMX712_MODEL_ID_VAL)
		return dev_err_probe(dev, -ENODEV,
				     "model id %#04llx, expected %#04x\n", id,
				     IMX712_MODEL_ID_VAL);

	return 0;
}

/*
 * The endpoint says what the receiver on the other end was configured for, and
 * the transcribed mode list only produces one thing: four D-PHY lanes carrying
 * 1.10592 Gbps each.  Refuse anything else rather than stream into a receiver
 * set up for a link this mode does not drive.
 */
static int imx712_parse_endpoint(struct imx712 *sensor)
{
	struct device *dev = regmap_get_device(sensor->regmap);
	struct v4l2_fwnode_endpoint bus = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
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

	if (bus.bus.mipi_csi2.num_data_lanes != IMX712_DATA_LANES) {
		ret = dev_err_probe(dev, -EINVAL, "%u data lanes, expected %u\n",
				    bus.bus.mipi_csi2.num_data_lanes,
				    IMX712_DATA_LANES);
		goto out;
	}

	if (bus.nr_of_link_frequencies != 1 ||
	    bus.link_frequencies[0] != IMX712_LINK_FREQ)
		ret = dev_err_probe(dev, -EINVAL,
				    "link-frequencies must be %lld\n",
				    IMX712_LINK_FREQ);

out:
	v4l2_fwnode_endpoint_free(&bus);

	return ret;
}

static int imx712_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx712 *sensor;
	unsigned long rate;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->mode = &imx712_modes[0];

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(dev, PTR_ERR(sensor->regmap),
				     "cannot make the register map\n");

	sensor->extclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(sensor->extclk))
		return dev_err_probe(dev, PTR_ERR(sensor->extclk),
				     "no master clock\n");

	/*
	 * The mode list's PLL dividers are constants, so the pixel rate and the
	 * link frequency this driver reports are only true at one input rate.
	 */
	rate = clk_get_rate(sensor->extclk);
	if (rate != IMX712_EXTCLK_RATE)
		return dev_err_probe(dev, -EINVAL,
				     "master clock is %lu Hz, need %lu Hz\n",
				     rate, IMX712_EXTCLK_RATE);

	/* Held asserted until the rails are up. */
	sensor->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(dev, PTR_ERR(sensor->reset),
				     "no reset GPIO\n");

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(imx712_supplies),
					    imx712_supplies, &sensor->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "cannot get the supplies\n");

	ret = imx712_parse_endpoint(sensor);
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

	ret = imx712_power_on(sensor);
	if (ret)
		return dev_err_probe(dev, ret, "cannot power the sensor\n");

	ret = imx712_identify(sensor);
	if (ret)
		goto err_power;

	imx712_read_limits(sensor);

	v4l2_i2c_subdev_init(&sensor->sd, client, &imx712_subdev_ops);
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->sd.internal_ops = &imx712_internal_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->sd.entity.ops = &imx712_entity_ops;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto err_power;

	ret = imx712_init_controls(sensor);
	if (ret)
		goto err_entity;

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
	imx712_power_off(sensor);

	return ret;
}

static void imx712_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx712 *sensor = sd_to_imx712(sd);
	struct device *dev = &client->dev;

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	v4l2_ctrl_handler_free(&sensor->hdl);
	media_entity_cleanup(&sd->entity);

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		imx712_power_off(sensor);
	pm_runtime_set_suspended(dev);
}

static const struct of_device_id imx712_of_match[] = {
	{ .compatible = "sony,imx712" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx712_of_match);

static struct i2c_driver imx712_i2c_driver = {
	.driver = {
		.name = "imx712",
		.of_match_table = imx712_of_match,
		.pm = pm_ptr(&imx712_pm_ops),
	},
	.probe = imx712_probe,
	.remove = imx712_remove,
};
module_i2c_driver(imx712_i2c_driver);

MODULE_DESCRIPTION("Sony IMX712 image sensor driver");
MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_LICENSE("GPL");
