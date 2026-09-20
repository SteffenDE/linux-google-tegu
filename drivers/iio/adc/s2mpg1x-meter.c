// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2026 Steffen Deusch
 *
 * Read-only ODPM (on-device power metering) driver for the Samsung S2MPG1x
 * PMICs on Google Tensor SoCs.  Each PMIC has a 12-channel power meter; a
 * channel accumulates the power of a muxed rail into a 41-bit accumulator
 * alongside a shared 20-bit sample counter.
 *
 * The block reports each channel twice.  LPF_DATA holds a low-pass-filtered
 * instantaneous power, which is a plain read: nothing is consumed and no
 * control write is needed, so it is what in_powerN_input returns.  The
 * accumulator holds the summed power of a window; writing CTRL2.ASYNC_RD
 * copies it into the readable registers and restarts it, which is a
 * measurement a reader takes away from every other reader and so is not done
 * on their behalf.
 *
 * The filter's per-channel coefficients are left as found.  The always-on PMIC
 * keeps them across boots and another operating system can program them, so
 * the time constant behind in_powerN_input is whatever was last written to
 * LPF_C0_0; the vendor driver does not program them either.
 *
 * We never change a rail's power state: the only writes
 * are to the meter's own mux/enable registers (the measurement selector), so
 * the driver is read-only with respect to the regulators.  The mux is
 * programmed from DT rather than inherited, because the always-on PMIC keeps
 * whatever the previous boot left in these registers.
 */

#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/s2mpg14.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

/*
 * Re-latch at most this often.  A latch waits for ASYNC_RD to self-clear, so
 * sweeping every channel back-to-back does the transfer once and answers the
 * rest of the sweep from the same window.  The totals it feeds are cumulative,
 * so the only cost is that a reader can be this far behind.
 */
#define S2MPG1X_METER_MIN_REFRESH_MS	100

/*
 * The 20-bit ACC_COUNT saturates after ~2.3 h at the 125 Hz internal rate, and
 * once saturated the accumulator is stuck (ASYNC_RD can no longer restart it).
 * A deadline timer re-latches once the meter has gone this long unlatched, so
 * an idle meter does not reach saturation while the system runs.  It gives no
 * protection across a system sleep -- the work is not run and the deadline does
 * not advance -- which is the one interval long enough to saturate, so the
 * window that spans a sleep is checked rather than assumed.
 */
#define S2MPG1X_METER_REFRESH_MS	(60 * 60 * 1000)

/*
 * The internal sample rate hw_init() programs, in hertz.  It is nominal: the
 * meter's own oscillator is untrimmed, so it is used to judge whether a
 * window's sample count is plausible, never to convert one into energy.
 */
#define S2MPG1X_METER_SAMP_RATE_HZ	125

/*
 * Tolerance on that nominal rate, as a reciprocal.  The vendor driver allows
 * the same +-12.5% before it stops believing the counter.
 */
#define S2MPG1X_METER_RATE_TOLERANCE	8

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
	/* chan[] and energy_nj[] are indexed by hardware channel. */
	struct s2mpg1x_meter_chan chan[S2MPG14_METER_CHANNELS];
	u64 energy_nj[S2MPG14_METER_CHANNELS];
	u64 suspend_energy_nj[S2MPG14_METER_CHANNELS];
	u64 suspend_time_us;
	unsigned int lost_windows;
	ktime_t last_refresh;	/* boottime, so a sleep is inside a window */
	ktime_t suspend_boot;	/* both clocks as the system suspended, so */
	ktime_t suspend_mono;	/* their divergence says whether it slept */
	bool in_suspend;	/* the open window is a sleep, and only that */
	bool valid;
	bool stopping;		/* gates the self-rearming refresh work */
	struct delayed_work refresh_work;
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

/*
 * The filtered power of one channel.  A plain register read: the LPF bank runs
 * continuously and reading it neither latches nor restarts anything.
 */
static int s2mpg1x_meter_read_lpf(struct s2mpg1x_meter *m, u8 hw, u32 *out)
{
	u8 buf[S2MPG14_METER_LPF_DATA_BYTES];
	int ret;

	ret = regmap_bulk_read(m->regmap,
			       S2MPG14_METER_LPF_DATA_CH0_1 +
			       hw * S2MPG14_METER_LPF_DATA_BYTES,
			       buf, sizeof(buf));
	if (ret)
		return ret;

	*out = (buf[0] | buf[1] << 8 | buf[2] << 16) &
	       GENMASK(S2MPG14_METER_LPF_DATA_BITS - 1, 0);
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

static int s2mpg1x_meter_hw_init(struct s2mpg1x_meter *m);

/*
 * Caller holds m->lock.  Start a window here and now, discarding whatever the
 * accumulators hold.  Used after a window turns out not to be a measurement.
 */
static int s2mpg1x_meter_restart(struct s2mpg1x_meter *m)
{
	int ret;

	m->valid = false;
	ret = s2mpg1x_meter_hw_init(m);
	if (ret)
		return ret;

	m->last_refresh = ktime_get_boottime();
	m->valid = true;
	return 0;
}

/*
 * Is a window of @delta_us plausibly described by @count samples?
 *
 * More samples than the elapsed time can hold means the counter is not
 * measuring this window at all -- saturated and frozen, or restarted
 * underneath us -- and that is worth catching at any window length, because a
 * frozen accumulator would otherwise be added to the totals again and again.
 * Fewer samples than expected only means something once the window is long
 * enough that the count's own quantisation is small, so the lower bound waits
 * for that.
 */
static bool s2mpg1x_meter_count_plausible(u32 count, u64 delta_us)
{
	u64 expected = mul_u64_u64_div_u64(delta_us,
					   S2MPG1X_METER_SAMP_RATE_HZ,
					   USEC_PER_SEC);
	u64 margin = expected / S2MPG1X_METER_RATE_TOLERANCE;

	if (count > expected + margin)
		return false;

	if (delta_us >= S2MPG1X_METER_MIN_REFRESH_MS * USEC_PER_MSEC &&
	    count + margin < expected)
		return false;

	return true;
}

/*
 * Caller holds m->lock.  Latches the accumulators, which restarts the window,
 * and adds the closed window's energy to the running per-rail totals.  A call
 * within MIN_REFRESH_MS of the last one does nothing unless @force says to
 * close the window exactly here, which is what the suspend boundaries need.
 *
 * Energy is the window's mean power times the time the AP measured, not the
 * accumulator's sample count times a nominal sample period: the count cancels
 * out of the arithmetic and the CPU's clock is the better of the two
 * references.  The count is then free to be what says whether the window is a
 * measurement at all.
 */
static int s2mpg1x_meter_refresh(struct s2mpg1x_meter *m, bool force)
{
	u64 window_nj[S2MPG14_METER_CHANNELS];
	unsigned int i;
	u64 delta_us;
	ktime_t now;
	u32 count;
	int ret;

	if (!force && m->valid &&
	    ktime_before(ktime_get_boottime(),
			 ktime_add_ms(m->last_refresh,
				      S2MPG1X_METER_MIN_REFRESH_MS)))
		return 0;

	ret = s2mpg1x_meter_latch(m);
	if (ret)
		return ret;

	/* Sampled after the latch: it is the latch that ends the window. */
	now = ktime_get_boottime();

	ret = s2mpg1x_meter_read_count(m, &count);
	if (ret)
		return ret;

	if (!m->valid) {
		/* No window is open yet; this latch opens the first one. */
		m->last_refresh = now;
		m->valid = true;
		return 0;
	}

	delta_us = ktime_us_delta(now, m->last_refresh);
	if (!s2mpg1x_meter_count_plausible(count, delta_us)) {
		m->lost_windows++;
		return s2mpg1x_meter_restart(m);
	}

	for (i = 0; i < m->n; i++) {
		u8 hw = m->hw_idx[i];
		u64 acc, power_nw;

		ret = s2mpg1x_meter_read_acc(m, hw, &acc);
		if (ret) {
			/*
			 * The latch already closed the window, so it is gone
			 * for every rail. Report the loss rather than commit
			 * it to some rails and not others.
			 */
			m->lost_windows++;
			m->last_refresh = now;
			return ret;
		}

		/*
		 * A per-sample code cannot exceed the accumulator's structural
		 * full scale (41-bit ACC over a 20-bit count), so an
		 * accumulator above that ceiling times the count is a corrupt
		 * read rather than a rail, and is dropped like any other
		 * window that is not a measurement.
		 */
		if (count && acc >= (u64)count * S2MPG14_METER_MAX_SAMPLE_CODE) {
			m->lost_windows++;
			m->last_refresh = now;
			return -EIO;
		}

		/* res_pw is picowatts per accumulated LSB. */
		power_nw = count ? mul_u64_u64_div_u64(acc, m->chan[hw].res_pw,
						       (u64)count * 1000) : 0;
		window_nj[i] = mul_u64_u64_div_u64(power_nw, delta_us,
						   USEC_PER_SEC);
	}

	for (i = 0; i < m->n; i++) {
		u8 hw = m->hw_idx[i];

		m->energy_nj[hw] += window_nj[i];
		if (m->in_suspend)
			m->suspend_energy_nj[hw] += window_nj[i];
	}

	/*
	 * The suspend total carries its own elapsed time, so that a reader
	 * divides an energy by exactly the interval it was measured over
	 * rather than by a sleep length measured somewhere else.
	 */
	if (m->in_suspend)
		m->suspend_time_us += delta_us;

	m->last_refresh = now;
	return 0;
}

/*
 * Re-latch only if the meter has gone unread for the deadline, so an idle
 * meter cannot saturate (see S2MPG1X_METER_REFRESH_MS).  A recent userspace
 * read pushes the deadline out instead, so active polling never forces an
 * extra latch.
 */
static void s2mpg1x_meter_refresh_work(struct work_struct *work)
{
	struct s2mpg1x_meter *m = container_of(to_delayed_work(work),
					       struct s2mpg1x_meter, refresh_work);

	scoped_guard(mutex, &m->lock) {
		s64 idle_ms = ktime_ms_delta(ktime_get_boottime(),
					     m->last_refresh);
		unsigned long delay;

		if (m->in_suspend) {
			/*
			 * The window open across a sleep belongs to the PM
			 * callbacks; latching it here would spend part of it
			 * on whichever moment of the suspend sequence this
			 * runs in.
			 */
			delay = msecs_to_jiffies(S2MPG1X_METER_REFRESH_MS);
		} else if (idle_ms >= S2MPG1X_METER_REFRESH_MS) {
			s2mpg1x_meter_refresh(m, false);
			delay = msecs_to_jiffies(S2MPG1X_METER_REFRESH_MS);
		} else {
			delay = msecs_to_jiffies(S2MPG1X_METER_REFRESH_MS - idle_ms);
		}

		/* Re-arm under the lock so stop() can race-free cancel us. */
		if (!m->stopping)
			schedule_delayed_work(&m->refresh_work, delay);
	}
}

static void s2mpg1x_meter_stop(void *data)
{
	struct s2mpg1x_meter *m = data;

	scoped_guard(mutex, &m->lock)
		m->stopping = true;
	cancel_delayed_work_sync(&m->refresh_work);
}

static ssize_t s2mpg1x_meter_read_energy(struct iio_dev *indio_dev,
					 uintptr_t private,
					 const struct iio_chan_spec *chan,
					 char *buf)
{
	struct s2mpg1x_meter *m = iio_priv(indio_dev);
	int ret;

	guard(mutex)(&m->lock);
	ret = s2mpg1x_meter_refresh(m, false);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%llu\n",
			  m->energy_nj[chan->address] / NSEC_PER_USEC);
}

/*
 * The suspend total moves only at a resume boundary, so reading it does not
 * latch: a latch here would fold awake time into the answer to "what did this
 * rail use while the system was asleep".
 */
static ssize_t s2mpg1x_meter_read_suspend_energy(struct iio_dev *indio_dev,
						 uintptr_t private,
						 const struct iio_chan_spec *chan,
						 char *buf)
{
	struct s2mpg1x_meter *m = iio_priv(indio_dev);

	guard(mutex)(&m->lock);

	return sysfs_emit(buf, "%llu\n",
			  m->suspend_energy_nj[chan->address] / NSEC_PER_USEC);
}

/*
 * Energy is carried per channel rather than as an IIO_ENERGY channel of its
 * own: a second channel set would double every label and index for a quantity
 * that is the same rail, and the suspend total below has no channel type to be.
 */
static const struct iio_chan_spec_ext_info s2mpg1x_meter_ext_info[] = {
	{
		.name = "energy",
		.read = s2mpg1x_meter_read_energy,
		.shared = IIO_SEPARATE,
	},
	{
		.name = "suspend_energy",
		.read = s2mpg1x_meter_read_suspend_energy,
		.shared = IIO_SEPARATE,
	},
	{ }
};

static ssize_t suspend_time_us_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct s2mpg1x_meter *m = iio_priv(indio_dev);

	guard(mutex)(&m->lock);

	return sysfs_emit(buf, "%llu\n", m->suspend_time_us);
}

static IIO_DEVICE_ATTR_RO(suspend_time_us, 0);

static ssize_t lost_windows_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct s2mpg1x_meter *m = iio_priv(indio_dev);

	guard(mutex)(&m->lock);

	return sysfs_emit(buf, "%u\n", m->lost_windows);
}

static IIO_DEVICE_ATTR_RO(lost_windows, 0);

static struct attribute *s2mpg1x_meter_attrs[] = {
	&iio_dev_attr_lost_windows.dev_attr.attr,
	&iio_dev_attr_suspend_time_us.dev_attr.attr,
	NULL,
};

static const struct attribute_group s2mpg1x_meter_attr_group = {
	.attrs = s2mpg1x_meter_attrs,
};

static int s2mpg1x_meter_read_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan,
				  int *val, int *val2, long mask)
{
	struct s2mpg1x_meter *m = iio_priv(indio_dev);
	u32 rem;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED: {
		u8 hw = chan->address;
		u64 uw;
		u32 lpf;

		/*
		 * Held across the read because hw_init() reconfigures the block
		 * -- soft reset, mux, mode -- and a read landing inside that
		 * returns a settling filter as a plausible number.
		 */
		guard(mutex)(&m->lock);
		ret = s2mpg1x_meter_read_lpf(m, hw, &lpf);
		if (ret)
			return ret;

		/*
		 * A filtered sample carries the same per-LSB power as one of
		 * the accumulator's samples; res_pw is in picowatts.
		 */
		uw = div_u64((u64)lpf * m->chan[hw].res_pw, 1000000);

		/* IIO's power base unit is milliwatts. */
		*val = div_u64_rem(uw, 1000, &rem);
		*val2 = rem * 1000;
		return IIO_VAL_INT_PLUS_MICRO;
	}
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
	.attrs = &s2mpg1x_meter_attr_group,
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

	/*
	 * The main PMIC is always-on and powers the SoC, so its meter keeps
	 * accumulating across AP reboots.  ACC_COUNT (20-bit) saturates after
	 * ~2.3 h at 125 Hz, and ASYNC_RD only copies the accumulators -- it
	 * cannot clear a saturated counter -- so a meter inherited from a prior
	 * boot reads a frozen since-cold-boot average (ACC_DATA/ACC_COUNT with a
	 * pinned count).  Soft-reset the accumulators so counting restarts from
	 * zero before we configure and enable the meter.
	 */
	ret = regmap_update_bits(m->regmap, S2MPG14_METER_CTRL5,
				 S2MPG14_METER_SOFT_RST_MASK,
				 S2MPG14_METER_SOFT_RST_MASK);
	if (ret)
		return ret;
	usleep_range(2, 102);

	/*
	 * Accumulate power (not current) on all 12 channels; the same write
	 * also clears the soft-reset bit so the meter can run.
	 */
	ret = regmap_write(m->regmap, S2MPG14_METER_CTRL4, 0x00);
	if (ret)
		return ret;
	ret = regmap_update_bits(m->regmap, S2MPG14_METER_CTRL5,
				 S2MPG14_METER_ACC_MODE_HI_MASK |
				 S2MPG14_METER_SOFT_RST_MASK, 0x00);
	if (ret)
		return ret;

	/*
	 * Report power (not current) in the filtered data registers too.  Only
	 * the mode is programmed; the filter coefficients are left as found.
	 */
	ret = regmap_write(m->regmap, S2MPG14_METER_CTRL6, 0x00);
	if (ret)
		return ret;
	ret = regmap_update_bits(m->regmap, S2MPG14_METER_CTRL7,
				 S2MPG14_METER_LPF_MODE_HI_MASK, 0x00);
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
	platform_set_drvdata(pdev, indio_dev);

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
		channels[i].ext_info = s2mpg1x_meter_ext_info;
	}

	/* Latch once to start a clean measurement window. */
	scoped_guard(mutex, &m->lock) {
		if (!s2mpg1x_meter_latch(m)) {
			m->last_refresh = ktime_get_boottime();
			m->valid = true;
		}
	}

	/* Keep the accumulator from saturating while userspace is not reading. */
	INIT_DELAYED_WORK(&m->refresh_work, s2mpg1x_meter_refresh_work);
	ret = devm_add_action_or_reset(dev, s2mpg1x_meter_stop, m);
	if (ret)
		return ret;
	schedule_delayed_work(&m->refresh_work,
			      msecs_to_jiffies(S2MPG1X_METER_REFRESH_MS));

	indio_dev->name = dev_name(dev);
	indio_dev->info = &s2mpg1x_meter_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = channels;
	indio_dev->num_channels = m->n;

	return devm_iio_device_register(dev, indio_dev);
}

/*
 * The always-on PMIC keeps the meter accumulating while the AP sleeps, so the
 * window left open here holds what the sleep cost.  Close the awake window so
 * that it holds the sleep and as little else as possible, and remember both
 * clocks: their divergence by the time we resume is the time actually spent
 * suspended.
 *
 * A failure leaves the window open unattributed rather than claiming a sleep it
 * cannot bound, and never vetoes the suspend: this driver measures the system,
 * and stopping it from sleeping would be the worse error by far.
 */
static int s2mpg1x_meter_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct s2mpg1x_meter *m = iio_priv(indio_dev);
	int ret;

	guard(mutex)(&m->lock);
	ret = s2mpg1x_meter_refresh(m, true);
	if (ret) {
		dev_warn(dev, "meter not closed before suspend: %d\n", ret);
		return 0;
	}

	m->suspend_boot = ktime_get_boottime();
	m->suspend_mono = ktime_get();
	m->in_suspend = true;
	return 0;
}

/*
 * Close the window the sleep ran in.  BOOTTIME advances across a sleep and
 * MONOTONIC does not, so their divergence over these two callbacks is the time
 * the system was actually suspended.  A cycle that aborted -- a wakeup arriving
 * during device suspend, or a later device refusing -- still runs this callback
 * for every device that suspended, and has no such divergence: its window is
 * ordinary awake time and is not attributed to sleep.
 *
 * This window is bounded by the callbacks, not by the sleep, so it also holds
 * the device suspend and resume either side of it. The suspend total therefore
 * comes with the elapsed time of the windows that make it up, so that what a
 * reader divides by is the interval the energy was actually measured over.
 *
 * Errors are reported and swallowed. A driver that fails to resume is recorded
 * as the device that broke the cycle, and a metering driver claiming that about
 * a bus hiccup would mislead exactly the tooling that reads these counters.
 */
static int s2mpg1x_meter_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct s2mpg1x_meter *m = iio_priv(indio_dev);
	s64 slept_ns;
	int ret;

	guard(mutex)(&m->lock);

	slept_ns = ktime_to_ns(ktime_sub(ktime_sub(ktime_get_boottime(),
						   m->suspend_boot),
					 ktime_sub(ktime_get(),
						   m->suspend_mono)));
	if (slept_ns <= 0)
		m->in_suspend = false;

	ret = s2mpg1x_meter_refresh(m, true);
	m->in_suspend = false;
	if (ret) {
		/*
		 * A meter that will not latch cannot be recovered by reading
		 * it, and a saturated counter is one reason it might not, so
		 * put it back to a known state here rather than leaving every
		 * later read to fail the same way.
		 */
		dev_warn(dev, "meter not closed after resume: %d\n", ret);
		s2mpg1x_meter_restart(m);
	}

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(s2mpg1x_meter_pm_ops, s2mpg1x_meter_suspend,
				s2mpg1x_meter_resume);

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
		.pm = pm_sleep_ptr(&s2mpg1x_meter_pm_ops),
	},
	.probe = s2mpg1x_meter_probe,
	.id_table = s2mpg1x_meter_id,
};
module_platform_driver(s2mpg1x_meter_driver);

MODULE_AUTHOR("Steffen Deusch");
MODULE_DESCRIPTION("Samsung S2MPG1x ODPM power meter");
MODULE_LICENSE("GPL");
