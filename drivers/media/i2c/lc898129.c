// SPDX-License-Identifier: GPL-2.0-only
/*
 * ON Semiconductor LC898129 lens voice-coil actuator.
 *
 * Copyright (C) 2026 Steffen Deusch
 *
 * The part is an "OIS & CL-AF Control LSI": three functions on one I2C
 * address -- autofocus, optical stabilisation and the module's EEPROM -- with
 * a 32-bit DSP running firmware out of its own flash.  This driver is the
 * autofocus half and nothing else.  It needs none of that firmware's
 * cooperation beyond what the part does for itself at power-up: the AF channel
 * is closed-loop, so the host writes a target and the chip's own servo reaches
 * it against a Hall sensor, and a bare target write moves the lens on a module
 * this driver has not otherwise touched.
 *
 * There is no register map in the datasheet -- it is a summary of
 * specification, with characteristics and timing and no command list -- so the
 * one register this driver writes was read out of a recording of the vendor
 * camera stack driving this module, where it is the only one that moves the
 * lens.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/unaligned.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

/*
 * The AF target.  Its value is a constant selector in the top half over a
 * 16-bit position in the bottom: the vendor writes it 892 times in one
 * session, 26 distinct values, always shaped 0002xxxx.  What the selector
 * selects is not known; it never varies, and this driver has no reason to vary
 * it.  Reading the register back returns what was written -- measured on this
 * driver's own writes, not in that session, which never reads it.
 */
#define LC898129_AF_TARGET		0xf01a
#define LC898129_AF_TARGET_SELECT	0x00020000

/*
 * The span the part can both reach and confirm, which is what the Hall
 * position it reports says it is.  Read back while stepping the target, that
 * position is 0xbffd at code 0 and rises by very nearly exactly eight counts
 * per code -- 7.95 to 8.17 measured over six steps -- reaching 0xffff, the top
 * of the sixteen bits it is reported in, at code 0x0800.  So this is the range
 * the closed loop has feedback across, and the lens tracks linearly over all
 * of it: stepping down to zero found no mechanical stop.
 *
 * The part accepts more: the vendor's own autofocus reached 0x086d, past the
 * top of this, so the travel is longer than the feedback.  A servo driven past
 * what its Hall sensor can report is a servo with no position, though, and
 * nothing here has measured where that ends.
 *
 * Not a range to trim to what one session used, either.  That session's
 * minimum of 0x03fd is short of infinity focus on this module, which is at
 * about 0x0390: a phase-detect stack does not sweep, it jumps to where the
 * phase says the subject is, so the positions in a recording are that scene's
 * subject distances rather than the actuator's limits.
 *
 * The polarity is V4L2's -- a larger value focuses closer -- and what says so
 * is two subjects at once rather than one defocusing, which would not
 * distinguish the directions: in one frame the nearer subject nulls its phase
 * at a higher code than the further one.
 */
#define LC898129_FOCUS_MIN		0x0000
#define LC898129_FOCUS_MAX		0x0800
#define LC898129_FOCUS_STEP		1

struct lc898129 {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
};

static inline struct lc898129 *ctrl_to_lc898129(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct lc898129, ctrls);
}

/*
 * Sixteen-bit register address, thirty-two-bit value, both big-endian on the
 * wire: the six bytes of a captured write are f0 1a 00 02 hi lo, and the
 * module's downstream device tree node declares the same two widths.
 */
static int lc898129_write(struct lc898129 *lc898129, u16 reg, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&lc898129->sd);
	u8 buf[6];
	int ret;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val, buf + 2);

	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	return ret == sizeof(buf) ? 0 : -EIO;
}

static int lc898129_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct lc898129 *lc898129 = ctrl_to_lc898129(ctrl);

	if (ctrl->id != V4L2_CID_FOCUS_ABSOLUTE)
		return -EINVAL;

	/*
	 * The write goes out now rather than at the next frame, because the
	 * servo is the chip's: it starts moving when it has a target and this
	 * driver has nothing to synchronise it against.
	 *
	 * It fails while the camera module is unpowered, which is not a state
	 * this driver can do anything about -- see the comment on the lack of
	 * regulators in lc898129_probe().  Measured, because a controller that
	 * reported a NACK as success would leave the control claiming a
	 * position the lens is not at: nothing drives the bus at all with the
	 * module down, so the transfer times out rather than being refused,
	 * and the control returns -ETIMEDOUT after this controller's fixed
	 * 100 ms.  The control framework does not commit a value whose s_ctrl
	 * failed, so the cached position stays at the last one that landed.
	 *
	 * What it does not do is notice the module power-cycling underneath
	 * it.  The part resets to its own position and this driver is not told,
	 * so the cached value can be stale until something writes again -- an
	 * autofocus loop writing every frame does, an application setting a
	 * position once does not.
	 */
	return lc898129_write(lc898129, LC898129_AF_TARGET,
			      LC898129_AF_TARGET_SELECT | ctrl->val);
}

static const struct v4l2_ctrl_ops lc898129_ctrl_ops = {
	.s_ctrl = lc898129_set_ctrl,
};

static const struct v4l2_subdev_ops lc898129_subdev_ops = { };

static int lc898129_probe(struct i2c_client *client)
{
	struct lc898129 *lc898129;
	int ret;

	lc898129 = devm_kzalloc(&client->dev, sizeof(*lc898129), GFP_KERNEL);
	if (!lc898129)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&lc898129->sd, client, &lc898129_subdev_ops);
	lc898129->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	lc898129->sd.entity.function = MEDIA_ENT_F_LENS;

	v4l2_ctrl_handler_init(&lc898129->ctrls, 1);
	v4l2_ctrl_new_std(&lc898129->ctrls, &lc898129_ctrl_ops,
			  V4L2_CID_FOCUS_ABSOLUTE, LC898129_FOCUS_MIN,
			  LC898129_FOCUS_MAX, LC898129_FOCUS_STEP,
			  LC898129_FOCUS_MIN);
	if (lc898129->ctrls.error) {
		ret = lc898129->ctrls.error;
		goto err_free_ctrls;
	}
	lc898129->sd.ctrl_handler = &lc898129->ctrls;

	ret = media_entity_pads_init(&lc898129->sd.entity, 0, NULL);
	if (ret)
		goto err_free_ctrls;

	/*
	 * No supplies and no reset, deliberately.  This chip sits on the camera
	 * module's own rails and comes up with the image sensor: on a board
	 * where nothing has powered that module, neither the actuator at 0x24
	 * nor the sensor at 0x56 answers on I2C at all.  A second consumer
	 * enabling a subset of those rails, in an order that is not the
	 * sequence the sensor driver replays, is a way to break the sensor and
	 * not a way to power this.
	 *
	 * So there is nothing here for runtime PM to gate either.  What follows
	 * from that is the honest failure mode above: setting the control with
	 * the module unpowered returns the I2C error rather than pretending.
	 */
	ret = v4l2_async_register_subdev(&lc898129->sd);
	if (ret)
		goto err_cleanup_entity;

	return 0;

err_cleanup_entity:
	media_entity_cleanup(&lc898129->sd.entity);
err_free_ctrls:
	v4l2_ctrl_handler_free(&lc898129->ctrls);

	return ret;
}

static void lc898129_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lc898129 *lc898129 = container_of(sd, struct lc898129, sd);

	v4l2_async_unregister_subdev(&lc898129->sd);
	media_entity_cleanup(&lc898129->sd.entity);
	v4l2_ctrl_handler_free(&lc898129->ctrls);
}

static const struct of_device_id lc898129_of_match[] = {
	{ .compatible = "onnn,lc898129" },
	{ }
};
MODULE_DEVICE_TABLE(of, lc898129_of_match);

static struct i2c_driver lc898129_i2c_driver = {
	.driver = {
		.name = "lc898129",
		.of_match_table = lc898129_of_match,
	},
	.probe = lc898129_probe,
	.remove = lc898129_remove,
};
module_i2c_driver(lc898129_i2c_driver);

MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_DESCRIPTION("ON Semiconductor LC898129 lens voice-coil actuator");
MODULE_LICENSE("GPL");
