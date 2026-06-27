// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2026 Steffen Deusch
 *
 * Read-only ODPM (on-device power metering) driver for the Samsung S2MPG1x
 * PMICs on Google Tensor SoCs.  Each PMIC has a 12-channel power meter; a
 * channel accumulates the power of a muxed rail into a 41-bit accumulator
 * alongside a shared 20-bit sample counter.
 *
 * Writing CTRL2.ASYNC_RD copies the accumulators into the readable registers
 * and restarts accumulation, so a read returns the average power over the
 * window since the previous read: ACC_DATA / ACC_COUNT scaled by the rail's
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
 * Re-latch at most this often, so that reading all channels back-to-back
 * (e.g. "cat in_power*_input") shares one coherent measurement window and the
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
	struct mutex lock;	/* serialises latch + cache */
	unsigned long dev_type;	/* enum sec_device_type */
	unsigned int n;
	u8 hw_idx[S2MPG14_METER_CHANNELS];	/* enabled channels, in IIO order */
	/* chan[] and cache_uw[] are indexed by hardware channel (MUXSEL index). */
	struct s2mpg1x_meter_chan chan[S2MPG14_METER_CHANNELS];
	s64 cache_uw[S2MPG14_METER_CHANNELS];
	ktime_t last_refresh;
	bool valid;
};

/*
 * Internal buck/LDO power resolution (pW per LSB), derived from the
 * downstream IQ30 constants.  The buck classes were confirmed on hardware
 * against in_powerN_scale: CMS 0.006868132, CMD 0.013736264,
 * CMT 0.020604396 mW/LSB.
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

static u32 s2mpg15_muxsel_power_pw(u8 muxsel)
{
	switch (muxsel) {
	case 0x01: /* BUCK1 (CMD) */
	case 0x02: /* BUCK2 (CMD) */
	case 0x0c: /* BUCK12 (CMD) */
		return 13736264;
	case 0x03: /* BUCK3 (CMS) */
	case 0x04: /* BUCK4 (CMS) */
	case 0x05: /* BUCK5 (CMS) */
	case 0x09: /* BUCK9 (CMS) */
		return 6868132;
	case 0x22: /* LDO2 (NLDO 1200mA) */
		return 1831502;
	case 0x35: /* LDO21 (NLDO 800mA) */
		return 2442002;
	case 0x36: /* LDO22 (PLDO 150mA) */
		return 915751;
	default:
		return 0;
	}
}

/* External (shunt) rails occupy this muxsel range. */
#define S2MPG1X_METER_MUXSEL_EXT_FIRST	0x5c
#define S2MPG1X_METER_MUXSEL_EXT_LAST	0x5e

static bool s2mpg1x_muxsel_is_external(u8 muxsel)
{
	return muxsel >= S2MPG1X_METER_MUXSEL_EXT_FIRST &&
	       muxsel <= S2MPG1X_METER_MUXSEL_EXT_LAST;
}

/*
 * External (VSEN) rails measure the drop across an off-chip sense resistor,
 * so the per-LSB power depends on the shunt value.  Derive it the same way
 * as the downstream odpm driver from these IQ30 (value << 30) calibration
 * constants: VRAIL = _IQ30(2.1978021), VSHUNT = _IQ30(0.7935698),
 * TRIM = BIT(3).
 */
#define S2MPG1X_METER_EXT_RES_VRAIL	2359872035U
#define S2MPG1X_METER_EXT_RES_VSHUNT	852089084U
#define S2MPG1X_METER_EXT_RES_TRIM	8U

static u32 s2mpg1x_shunt_power_pw(u32 shunt_uohms)
{
	u64 raw_iq60 = div64_u64((u64)S2MPG1X_METER_EXT_RES_VRAIL *
				 S2MPG1X_METER_EXT_RES_VSHUNT *
				 S2MPG1X_METER_EXT_RES_TRIM, shunt_uohms);
	u32 res_mw_iq30 = (raw_iq60 * 10) >> 30;

	/* mW/LSB (IQ30) -> pW/LSB, rounded */
	return div64_u64((u64)res_mw_iq30 * 1000000000ULL + (1U << 29), 1U << 30);
}

/*
 * Pulse ASYNC_RD to copy the live accumulators into the readable registers
 * and restart accumulation.  The bit self-clears once the transfer completes,
 * within one acquisition window (~8 ms at 125 Hz).
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
 * Caller holds m->lock.  Latches the accumulators (which restarts the window)
 * and recomputes per-rail power as ACC_DATA / ACC_COUNT for the elapsed
 * window.  Reads within MIN_REFRESH_MS reuse the cached window.
 */
static int s2mpg1x_meter_refresh(struct s2mpg1x_meter *m)
{
	ktime_t now = ktime_get();
	u32 count;
	unsigned int i;
	int ret;

	if (m->valid &&
	    ktime_before(now, ktime_add_ms(m->last_refresh,
					   S2MPG1X_METER_MIN_REFRESH_MS)))
		return 0;

	ret = s2mpg1x_meter_latch(m);
	if (ret)
		return ret;

	ret = s2mpg1x_meter_read_count(m, &count);
	if (ret)
		return ret;

	for (i = 0; i < m->n; i++) {
		u8 hw = m->hw_idx[i];
		u64 acc, avg;

		ret = s2mpg1x_meter_read_acc(m, hw, &acc);
		if (ret)
			return ret;

		avg = count ? div64_u64(acc, count) : 0;
		/*
		 * The per-sample code cannot exceed the accumulator's structural
		 * full scale (41-bit ACC over a 20-bit count); a larger value
		 * means a corrupt latch, so keep the previous reading.
		 */
		if (avg < S2MPG14_METER_MAX_SAMPLE_CODE)
			m->cache_uw[hw] = div64_u64(avg * m->chan[hw].res_pw,
						    1000000);
	}

	m->last_refresh = now;
	m->valid = true;
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
		if (s2mpg1x_muxsel_is_external(muxsel)) {
			u32 shunt_uohms;

			ret = fwnode_property_read_u32(child, "shunt-resistor-micro-ohms",
						       &shunt_uohms);
			if (ret || !shunt_uohms)
				return dev_err_probe(dev, -EINVAL,
						     "channel %u: external rail needs shunt-resistor-micro-ohms\n",
						     reg);
			c->res_pw = s2mpg1x_shunt_power_pw(shunt_uohms);
		} else if (m->dev_type == S2MPG15) {
			c->res_pw = s2mpg15_muxsel_power_pw(muxsel);
		} else {
			c->res_pw = s2mpg14_muxsel_power_pw(muxsel);
		}
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
	unsigned int ctrl1_mask, ctrl1_val;
	u8 ext_ch_mask = 0;
	unsigned int i;
	int ret;

	/* Accumulate power (not current) on all 12 channels. */
	ret = regmap_write(m->regmap, S2MPG14_METER_CTRL4, 0x00);
	if (ret)
		return ret;
	ret = regmap_update_bits(m->regmap, S2MPG14_METER_CTRL5,
				 S2MPG14_METER_ACC_MODE_HI_MASK, 0x00);
	if (ret)
		return ret;

	/*
	 * Enable current sensing for all bucks: BUCKEN1 covers BUCK1..8,
	 * BUCKEN2 the rest.  Enable BUCK9 (bit0) and BUCK12 (bit3) -- the
	 * highest-numbered bucks either PMIC meters (s2mpg15 BUCK12S = AUR).
	 */
	ret = regmap_write(m->regmap, S2MPG14_METER_BUCKEN1, 0xff);
	if (ret)
		return ret;
	ret = regmap_write(m->regmap, S2MPG14_METER_BUCKEN2, 0x09);
	if (ret)
		return ret;

	for (i = 0; i < m->n; i++) {
		u8 hw = m->hw_idx[i];
		u8 muxsel = m->chan[hw].muxsel;

		ret = regmap_write(m->regmap, S2MPG14_METER_MUXSEL0 + hw, muxsel);
		if (ret)
			return ret;

		if (s2mpg1x_muxsel_is_external(muxsel))
			ext_ch_mask |= BIT(muxsel - S2MPG1X_METER_MUXSEL_EXT_FIRST);
	}

	if (ext_ch_mask) {
		/*
		 * Program the external sample rate and channel-enable bits
		 * while the external meter is disabled, then turn it on
		 * together with the meter below.
		 */
		ret = regmap_update_bits(m->regmap, S2MPG14_METER_CTRL1,
					 S2MPG14_METER_EXT_EN_MASK, 0);
		if (ret)
			return ret;
		ret = regmap_update_bits(m->regmap, S2MPG14_METER_CTRL2,
					 S2MPG14_METER_EXT_CH_EN_MASK |
					 S2MPG14_METER_EXT_SAMP_RATE_MASK,
					 (ext_ch_mask <<
					  S2MPG14_METER_EXT_CH_EN_SHIFT) |
					 S2MPG14_METER_EXT_SAMP_RATE_31_25HZ);
		if (ret)
			return ret;
	}

	/* Set the sample rate and enable, preserving the other CTRL1 bits. */
	ctrl1_mask = S2MPG14_METER_EN_MASK | S2MPG14_METER_INT_SAMP_RATE_MASK;
	ctrl1_val = S2MPG14_METER_EN_MASK |
		    (S2MPG14_METER_INT_SAMP_RATE_125HZ <<
		     S2MPG14_METER_INT_SAMP_RATE_SHIFT);
	if (ext_ch_mask) {
		ctrl1_mask |= S2MPG14_METER_EXT_EN_MASK;
		ctrl1_val |= S2MPG14_METER_EXT_EN_MASK;
	}

	return regmap_update_bits(m->regmap, S2MPG14_METER_CTRL1, ctrl1_mask,
				  ctrl1_val);
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
	m->dev_type = platform_get_device_id(pdev)->driver_data;

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

	/* Latch once to start a clean measurement window. */
	scoped_guard(mutex, &m->lock) {
		if (!s2mpg1x_meter_latch(m))
			m->last_refresh = ktime_get();
	}

	indio_dev->name = dev_name(dev);
	indio_dev->info = &s2mpg1x_meter_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = channels;
	indio_dev->num_channels = m->n;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct platform_device_id s2mpg1x_meter_id[] = {
	{ "s2mpg14-meter", S2MPG14 },
	{ "s2mpg15-meter", S2MPG15 },
	{ }
};
MODULE_DEVICE_TABLE(platform, s2mpg1x_meter_id);

/*
 * The device is instantiated by the parent MFD and matched by the
 * platform_device_id above; the of_compatible only assigns the DT node.
 */
static const struct of_device_id s2mpg1x_meter_of_match[] __used = {
	{ .compatible = "samsung,s2mpg14-meter" },
	{ .compatible = "samsung,s2mpg15-meter" },
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
