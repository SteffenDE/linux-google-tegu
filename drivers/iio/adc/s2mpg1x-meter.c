// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2026 Steffen Deusch
 *
 * Read-only ODPM (on-device power metering) driver for the Samsung S2MPG1x
 * PMICs on Google Tensor SoCs.  Each PMIC has a 12-channel power meter; a
 * channel continuously accumulates the power of a muxed rail into a 41-bit
 * accumulator alongside a shared 20-bit sample counter.
 *
 * The accumulators free-run and are copied to readable registers on demand by
 * pulsing CTRL2.ASYNC_RD.  Average power over an interval is therefore the
 * change in ACC_DATA divided by the change in ACC_COUNT, scaled by the rail's
 * per-LSB resolution.  We never change a rail's power state: the only writes
 * are to the meter's own mux/enable registers (the measurement selector), so
 * the driver is read-only with respect to the regulators.  The mux is
 * programmed from DT rather than inherited, because the always-on PMIC keeps
 * whatever the previous boot left in these registers.
 */

#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/iio/iio.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/s2mpg14.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>

/*
 * Re-snapshot at most this often, so that reading all channels back-to-back
 * (e.g. "cat in_power*_input") shares one coherent accumulator sample and the
 * reported power is the average since the previous read pass.
 */
#define S2MPG1X_METER_MIN_REFRESH_MS	100

struct s2mpg1x_meter_chan {
	u8 muxsel;	/* rail mux selection */
	u32 res_pw;	/* power resolution, pW per accumulator LSB */
	const char *label;
};

struct s2mpg1x_meter {
	struct regmap *regmap;
	struct mutex lock;	/* serialises latch + snapshot state + cache */
	unsigned int n;
	u8 hw_idx[S2MPG14_METER_CHANNELS];	/* enabled channels, in IIO order */
	/* The following are all indexed by hardware channel (MUXSEL index). */
	struct s2mpg1x_meter_chan chan[S2MPG14_METER_CHANNELS];
	u64 prev_acc[S2MPG14_METER_CHANNELS];
	s64 cache_uw[S2MPG14_METER_CHANNELS];
	u32 prev_count;
	ktime_t last_refresh;
	bool primed;
};

/*
 * S2MPG14 buck power resolution (pW per LSB), derived from the downstream
 * IQ30 constants and confirmed on hardware against in_powerN_scale:
 * CMS 0.006868132, CMD 0.013736264, CMT 0.020604396 mW/LSB.
 * External (VSEN shunt) and LDO rails are not handled yet.
 */
static u32 s2mpg14_muxsel_power_pw(u8 muxsel)
{
	switch (muxsel) {
	case 0x01: /* BUCK1 */
	case 0x06: /* BUCK6 */
	case 0x08: /* BUCK8 */
	case 0x09: /* BUCK9 */
		return 6868132;
	case 0x02: /* BUCK2 */
	case 0x04: /* BUCK4 */
		return 13736264;
	case 0x03: /* BUCK3 */
	case 0x05: /* BUCK5 */
	case 0x07: /* BUCK7 */
		return 20604396;
	default:
		return 0;
	}
}

/*
 * Pulse ASYNC_RD to copy the live accumulators into the readable registers.
 * The bit self-clears once the transfer completes, within one acquisition
 * window (~8 ms at 125 Hz).
 */
static int s2mpg1x_meter_latch(struct s2mpg1x_meter *m)
{
	unsigned int val;
	int ret;

	ret = regmap_update_bits(m->regmap, S2MPG14_METER_CTRL2,
				 S2MPG14_METER_ASYNC_RD_MASK,
				 S2MPG14_METER_ASYNC_RD_MASK);
	if (ret)
		return ret;

	return regmap_read_poll_timeout(m->regmap, S2MPG14_METER_CTRL2, val,
					!(val & S2MPG14_METER_ASYNC_RD_MASK),
					500, 10000);
}

static int s2mpg1x_meter_read_acc(struct s2mpg1x_meter *m, u8 hw, u64 *out)
{
	u8 buf[S2MPG14_METER_ACC_DATA_BYTES];
	u64 v = 0;
	int i, ret;

	ret = regmap_bulk_read(m->regmap,
			       S2MPG14_METER_ACC_DATA_CH0_1 +
			       hw * S2MPG14_METER_ACC_DATA_BYTES,
			       buf, sizeof(buf));
	if (ret)
		return ret;

	for (i = 0; i < S2MPG14_METER_ACC_DATA_BYTES; i++)
		v |= (u64)buf[i] << (8 * i);

	*out = v & GENMASK_ULL(S2MPG14_METER_ACC_DATA_BITS - 1, 0);
	return 0;
}

static int s2mpg1x_meter_read_count(struct s2mpg1x_meter *m, u32 *out)
{
	u8 buf[S2MPG14_METER_ACC_COUNT_BYTES];
	int ret;

	ret = regmap_bulk_read(m->regmap, S2MPG14_METER_ACC_COUNT_1,
			       buf, sizeof(buf));
	if (ret)
		return ret;

	*out = (buf[0] | buf[1] << 8 | buf[2] << 16) &
	       GENMASK(S2MPG14_METER_ACC_COUNT_BITS - 1, 0);
	return 0;
}

/*
 * Caller holds m->lock.  Latches and snapshots the accumulators, then
 * recomputes per-rail power as the average since the previous snapshot.
 * Deltas use modular subtraction, correct across a single wrap of the
 * counter/accumulator; reads more than ~2 h apart (the saturation time at
 * 125 Hz) would under-count, which is far beyond any expected polling rate.
 */
static int s2mpg1x_meter_refresh(struct s2mpg1x_meter *m)
{
	ktime_t now = ktime_get();
	u32 count, d_count;
	unsigned int i;
	int ret;

	if (m->primed &&
	    ktime_before(now, ktime_add_ms(m->last_refresh,
					   S2MPG1X_METER_MIN_REFRESH_MS)))
		return 0;

	ret = s2mpg1x_meter_latch(m);
	if (ret)
		return ret;

	ret = s2mpg1x_meter_read_count(m, &count);
	if (ret)
		return ret;

	d_count = (count - m->prev_count) & GENMASK(S2MPG14_METER_ACC_COUNT_BITS - 1, 0);

	for (i = 0; i < m->n; i++) {
		u8 hw = m->hw_idx[i];
		u64 acc, d_acc;

		ret = s2mpg1x_meter_read_acc(m, hw, &acc);
		if (ret)
			return ret;

		if (m->primed && d_count) {
			d_acc = (acc - m->prev_acc[hw]) &
				GENMASK_ULL(S2MPG14_METER_ACC_DATA_BITS - 1, 0);
			m->cache_uw[hw] = div64_u64(div64_u64(d_acc, d_count) *
						    m->chan[hw].res_pw, 1000000);
		}
		m->prev_acc[hw] = acc;
	}

	m->prev_count = count;
	m->last_refresh = now;
	m->primed = true;
	return 0;
}

static int s2mpg1x_meter_read_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan,
				  int *val, int *val2, long mask)
{
	struct s2mpg1x_meter *m = iio_priv(indio_dev);
	u32 rem;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		guard(mutex)(&m->lock);
		ret = s2mpg1x_meter_refresh(m);
		if (ret)
			return ret;
		/* cache is in microwatts; IIO power base unit is milliwatts */
		*val = div_u64_rem(m->cache_uw[chan->address], 1000, &rem);
		*val2 = rem * 1000;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int s2mpg1x_meter_read_label(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    char *label)
{
	struct s2mpg1x_meter *m = iio_priv(indio_dev);

	return sysfs_emit(label, "%s\n", m->chan[chan->address].label);
}

static const struct iio_info s2mpg1x_meter_info = {
	.read_raw = s2mpg1x_meter_read_raw,
	.read_label = s2mpg1x_meter_read_label,
};

static int s2mpg1x_meter_parse_channels(struct device *dev,
					struct s2mpg1x_meter *m)
{
	unsigned int n = 0;

	device_for_each_child_node_scoped(dev, child) {
		struct s2mpg1x_meter_chan *c;
		const char *label;
		u32 reg, muxsel;
		int ret;

		if (n >= S2MPG14_METER_CHANNELS)
			return dev_err_probe(dev, -EINVAL,
					     "too many meter channels\n");

		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret || reg >= S2MPG14_METER_CHANNELS)
			return dev_err_probe(dev, -EINVAL,
					     "bad/missing channel reg\n");

		ret = fwnode_property_read_u32(child, "samsung,muxsel", &muxsel);
		if (ret)
			return dev_err_probe(dev, ret,
					     "channel %u: missing samsung,muxsel\n",
					     reg);

		c = &m->chan[reg];
		c->res_pw = s2mpg14_muxsel_power_pw(muxsel);
		if (!c->res_pw)
			return dev_err_probe(dev, -EINVAL,
					     "channel %u: unsupported muxsel 0x%02x\n",
					     reg, muxsel);

		ret = fwnode_property_read_string(child, "label", &label);
		if (ret)
			return dev_err_probe(dev, ret,
					     "channel %u: missing label\n", reg);

		c->muxsel = muxsel;
		c->label = devm_kstrdup(dev, label, GFP_KERNEL);
		if (!c->label)
			return -ENOMEM;
		m->hw_idx[n++] = reg;
	}

	if (!n)
		return dev_err_probe(dev, -EINVAL, "no meter channels\n");

	m->n = n;
	return 0;
}

/* Program the mux for the configured channels and enable the meter. */
static int s2mpg1x_meter_hw_init(struct s2mpg1x_meter *m)
{
	unsigned int i;
	int ret;

	/* Enable current sensing for all main-PMIC bucks (BUCK1..9). */
	ret = regmap_write(m->regmap, S2MPG14_METER_BUCKEN1, 0xff);
	if (ret)
		return ret;
	ret = regmap_write(m->regmap, S2MPG14_METER_BUCKEN2, 0x01);
	if (ret)
		return ret;

	for (i = 0; i < m->n; i++) {
		u8 hw = m->hw_idx[i];

		ret = regmap_write(m->regmap, S2MPG14_METER_MUXSEL0 + hw,
				   m->chan[hw].muxsel);
		if (ret)
			return ret;
	}

	/* Set the sample rate and enable, preserving the other CTRL1 bits. */
	return regmap_update_bits(m->regmap, S2MPG14_METER_CTRL1,
				  S2MPG14_METER_EN_MASK |
				  S2MPG14_METER_INT_SAMP_RATE_MASK,
				  S2MPG14_METER_EN_MASK |
				  (S2MPG14_METER_INT_SAMP_RATE_125HZ <<
				   S2MPG14_METER_INT_SAMP_RATE_SHIFT));
}

static int s2mpg1x_meter_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_chan_spec *channels;
	struct s2mpg1x_meter *m;
	struct iio_dev *indio_dev;
	unsigned int i;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*m));
	if (!indio_dev)
		return -ENOMEM;

	m = iio_priv(indio_dev);
	mutex_init(&m->lock);

	m->regmap = dev_get_regmap(dev->parent, "meter");
	if (!m->regmap)
		return dev_err_probe(dev, -ENODEV, "no meter regmap\n");

	ret = s2mpg1x_meter_parse_channels(dev, m);
	if (ret)
		return ret;

	ret = s2mpg1x_meter_hw_init(m);
	if (ret)
		return dev_err_probe(dev, ret, "meter init failed\n");

	channels = devm_kcalloc(dev, m->n, sizeof(*channels), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	for (i = 0; i < m->n; i++) {
		channels[i].type = IIO_POWER;
		channels[i].indexed = 1;
		channels[i].channel = m->hw_idx[i];
		channels[i].address = m->hw_idx[i];
		channels[i].info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED);
	}

	/* Prime the accumulator baseline so the first read has a valid delta. */
	scoped_guard(mutex, &m->lock)
		s2mpg1x_meter_refresh(m);

	indio_dev->name = dev_name(dev);
	indio_dev->info = &s2mpg1x_meter_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = channels;
	indio_dev->num_channels = m->n;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct platform_device_id s2mpg1x_meter_id[] = {
	{ "s2mpg14-meter", S2MPG14 },
	{ }
};
MODULE_DEVICE_TABLE(platform, s2mpg1x_meter_id);

/*
 * The device is instantiated by the parent MFD and matched by the
 * platform_device_id above; the of_compatible only assigns the DT node.
 */
static const struct of_device_id s2mpg1x_meter_of_match[] __used = {
	{ .compatible = "samsung,s2mpg14-meter" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpg1x_meter_of_match);

static struct platform_driver s2mpg1x_meter_driver = {
	.driver = {
		.name = "s2mpg1x-meter",
	},
	.probe = s2mpg1x_meter_probe,
	.id_table = s2mpg1x_meter_id,
};
module_platform_driver(s2mpg1x_meter_driver);

MODULE_AUTHOR("Steffen Deusch");
MODULE_DESCRIPTION("Samsung S2MPG1x ODPM power meter");
MODULE_LICENSE("GPL");
