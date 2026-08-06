// SPDX-License-Identifier: GPL-2.0-only
/*
 * AoC USF sensor framework -> IIO bridge.
 *
 * Bridges the sensors hosted by the AoC USF (Unified Sensor Framework)
 * firmware to Linux IIO. The AoC sensor registry is loaded out of band by a
 * userspace daemon; once it is loaded this driver enumerates the sensors over
 * the com.google.usf AOCC channel and exposes them as IIO devices. This is the
 * data-plane half described in docs/subsystems/sensors/reference/usf-iio-bridge.md.
 *
 * The registry-load daemon "pokes" this driver once the registry is ready by
 * writing the "enumerate" sysfs attribute; the driver stays inert until then.
 * The autopoll module parameter is a kernel-only bring-up escape hatch that
 * enumerates at probe instead.
 */

#define pr_fmt(fmt) "usf-iio: " fmt

#include <linux/completion.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/timekeeping.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>

#include <soc/google/aoc_channel.h>
#include <soc/google/aoc_clock.h>

#include "usf-proto.h"

#define USF_WAKE_SERVICE	"com.google.usf"
#define USF_NONWAKE_SERVICE	"com.google.usf.non_wake_up"

#define USF_CTL_TIMEOUT_MS	5000	/* per request/response round-trip */
#define USF_RETRY_MS		1000	/* bootstrap retry cadence */
#define USF_MAX_RETRIES		15	/* ~15 s before giving up */
#define USF_RESP_MAX		1024	/* one AOCC MTU */

#define USF_LIST_MAX		64	/* enumerated sensor handles */
#define USF_NAME_MAX		64
#define USF_SETTLE_TRIES	25	/* GetSensorList settle: 25 * 200 ms = 5 s */
#define USF_SETTLE_MS		200
#define USF_MAX_DEV		12	/* max IIO devices registered */

static bool autopoll;
module_param(autopoll, bool, 0644);
MODULE_PARM_DESC(autopoll,
		 "enumerate sensors at probe instead of waiting for the sysfs poke");

static char *sensor = "";
module_param(sensor, charp, 0644);
MODULE_PARM_DESC(sensor,
		 "if set, only expose USF sensors whose name contains this substring (default: all supported)");

static int rate = 50;
module_param(rate, int, 0644);
MODULE_PARM_DESC(rate,
		 "initial sampling rate in Hz; per-device sampling_frequency overrides it (AoC caps per sensor)");

struct usf_sensor;

struct usf_iio {
	struct device *dev;
	struct delayed_work enum_work;
	unsigned int retries;
	bool enumerated;

	/* AOCC transport: control + wake samples on @wake, the rest on @nonwake */
	struct aocc_channel *wake;
	struct aocc_channel *nonwake;

	/* Control request/response, serialised by @ctl_lock. */
	struct mutex ctl_lock;
	struct usf_fbb fbb;		/* request builder scratch (~1.5 KB) */
	u32 txn;			/* monotonic transaction id */

	/* Response slot, filled by the rx callback, protected by @resp_lock. */
	spinlock_t resp_lock;
	struct completion resp_done;
	u32 resp_txn;			/* awaited txn, or U32_MAX for none */
	u32 resp_ready;			/* txn whose payload is in resp[], else U32_MAX */
	u32 resp_len;
	u8 resp[USF_RESP_MAX];

	/* Discovered USF server handles (resolved at runtime; never hardcoded). */
	u32 sensor_mgr;
	u32 sample_chan;

	/* Registered IIO devices, one per exposed sensor. */
	struct usf_sensor *sensors[USF_MAX_DEV];
	int nsensors;
	spinlock_t sample_lock;	/* fences sample push/lookup vs buffer teardown */
	bool ts_logged;		/* one-shot log of the first timestamp mapping */
};

/* Per-IIO-device state (in iio_priv). */
struct usf_sensor {
	struct usf_iio *usf;
	struct iio_dev *indio;	/* owning IIO device */
	u32 handle;		/* USF sensor handle (dst for Create/Reconfig) */
	u32 client_id;		/* AP-chosen opaque id echoed in samples */
	u32 sampling_id;	/* live stream id (0 = not streaming) */
	s64 period_ns;
	int samp_freq;		/* requested rate in Hz (sampling_frequency) */
	u8 ndata;		/* data channels (1 scalar, 3 vector) */

	/*
	 * Scale: samples quantise to raw = round(value / resolution), so raw is
	 * a native-unit LSB count and IIO SCALE carries the resolution (folded
	 * with the Android->IIO unit factor). res_q = resolution << USF_SCALE_Q;
	 * 0 means the firmware gave no resolution -> micro-unit fallback.
	 */
	u64 res_q;
	u64 scale_nano;		/* IIO_CHAN_INFO_SCALE in nano units */
};

/* Fixed-point fraction bits for the resolution divisor (res_q). */
#define USF_SCALE_Q	30
#define USF_NANO	1000000000ULL

/* Forced scan masks (all data channels; timestamp is tracked separately). */
static const unsigned long usf_scan_masks_3axis[] = { GENMASK(2, 0), 0 };
static const unsigned long usf_scan_masks_scalar[] = { BIT(0), 0 };

/*
 * Sample axes arrive as f32 in Android sensor units. The data path quantises
 * each to raw = round(value / resolution) as an s32, and IIO SCALE = resolution
 * (times the Android->IIO unit factor), so the value is recovered as raw*SCALE
 * without the fixed-1e-6 saturation that clipped high-range sensors (ALS, mag).
 */
#define USF_AXIS(_type, _mod, _idx) {				\
	.type = _type,						\
	.modified = 1,						\
	.channel2 = IIO_MOD_##_mod,				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _idx,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 32,					\
		.storagebits = 32,				\
		.endianness = IIO_LE,				\
	},							\
}
#define USF_3AXIS_CHANNELS(_type)		\
	USF_AXIS(_type, X, 0),			\
	USF_AXIS(_type, Y, 1),			\
	USF_AXIS(_type, Z, 2),			\
	IIO_CHAN_SOFT_TIMESTAMP(3)

static const struct iio_chan_spec usf_accel_channels[] = {
	USF_3AXIS_CHANNELS(IIO_ACCEL),
};
static const struct iio_chan_spec usf_gyro_channels[] = {
	USF_3AXIS_CHANNELS(IIO_ANGL_VEL),
};
static const struct iio_chan_spec usf_magn_channels[] = {
	USF_3AXIS_CHANNELS(IIO_MAGN),
};

#define USF_SCALAR_CHANNEL(_type) {				\
	.type = _type,						\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = 0,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 32,					\
		.storagebits = 32,				\
		.endianness = IIO_LE,				\
	},							\
}
#define USF_SCALAR_CHANNELS(_type)		\
	USF_SCALAR_CHANNEL(_type),		\
	IIO_CHAN_SOFT_TIMESTAMP(1)

static const struct iio_chan_spec usf_light_channels[] = {
	USF_SCALAR_CHANNELS(IIO_LIGHT),
};
static const struct iio_chan_spec usf_prox_channels[] = {
	USF_SCALAR_CHANNELS(IIO_PROXIMITY),
};
static const struct iio_chan_spec usf_pressure_channels[] = {
	USF_SCALAR_CHANNELS(IIO_PRESSURE),
};

/* Map a USF sensor name (substring) to an IIO device type + channels. */
static const struct usf_type_map {
	const char *match;	/* case-insensitive substring of the USF name */
	const char *iio_name;
	const struct iio_chan_spec *channels;
	int num_channels;
	int ndata;		/* data channels (1 scalar, 3 vector) */
	const unsigned long *scan_masks;
	/*
	 * Android-native -> IIO unit factor, times 1e9: SCALE(nano) =
	 * resolution * unit_nano. 1e9 = 1:1 (accel m/s^2, gyro rad/s, light
	 * lux, prox count); mag uT -> Gauss is /100; pressure hPa -> kPa is /10.
	 */
	u64 unit_nano;
} usf_type_maps[] = {
	{ "Accelerometer", "usf_accel", usf_accel_channels,
	  ARRAY_SIZE(usf_accel_channels), 3, usf_scan_masks_3axis, 1000000000ULL },
	{ "Gyroscope", "usf_gyro", usf_gyro_channels,
	  ARRAY_SIZE(usf_gyro_channels), 3, usf_scan_masks_3axis, 1000000000ULL },
	{ "Magnetometer", "usf_magn", usf_magn_channels,
	  ARRAY_SIZE(usf_magn_channels), 3, usf_scan_masks_3axis, 10000000ULL },
	{ "Ambient Light", "usf_als", usf_light_channels,
	  ARRAY_SIZE(usf_light_channels), 1, usf_scan_masks_scalar, 1000000000ULL },
	/* "Proximity" alone also matches gesture/voice/AAD sensors; be specific */
	{ "TMD3733 Proximity", "usf_prox", usf_prox_channels,
	  ARRAY_SIZE(usf_prox_channels), 1, usf_scan_masks_scalar, 1000000000ULL },
	{ "Barometer", "usf_baro", usf_pressure_channels,
	  ARRAY_SIZE(usf_pressure_channels), 1, usf_scan_masks_scalar, 100000000ULL },
};

static int usf_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long mask)
{
	struct usf_sensor *s = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		/* SCALE = resolution * unit factor, discovered per sensor. */
		*val = s->scale_nano / USF_NANO;
		*val2 = s->scale_nano % USF_NANO;
		return IIO_VAL_INT_PLUS_NANO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = s->samp_freq;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

/*
 * Set the sampling rate. The AoC picks the nearest supported ODR, so the value
 * read back is what was requested, not necessarily what streams. A live stream
 * is retuned in place (ReconfigSampling keeps the sampling_id); otherwise the
 * rate takes effect at the next buffer enable.
 */
static int usf_write_raw(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int val, int val2, long mask)
{
	struct usf_sensor *s = iio_priv(indio_dev);
	struct usf_iio *usf = s->usf;
	const u8 *req;
	size_t reqlen;
	s64 period_ns;
	u32 txn, sid;
	int ret;

	if (mask != IIO_CHAN_INFO_SAMP_FREQ)
		return -EINVAL;
	if (val <= 0 || val > 1000000)		/* keep period_ns >= 1000 ns */
		return -EINVAL;

	s->samp_freq = val;
	period_ns = 1000000000LL / val;

	/*
	 * Read sampling_id *inside* ctl_lock, and hold ctl_lock across the retune
	 * send. usf_stop_sampling() (buffer disable) clears sampling_id before it
	 * takes ctl_lock to send its StopSampling, so if we observe a live sid here a
	 * concurrent stop cannot have disabled yet and its disable is serialised
	 * after our enable. Otherwise we would risk leaving the AoC streaming a
	 * sampling_id that nothing stops again.
	 */
	ret = 0;
	mutex_lock(&usf->ctl_lock);
	spin_lock(&usf->sample_lock);
	sid = s->sampling_id;
	spin_unlock(&usf->sample_lock);
	if (sid) {
		txn = usf->txn++;
		ret = usf_build_reconfig(&usf->fbb, txn, s->handle, sid,
					 period_ns, 0, true, &req, &reqlen);
		if (!ret)
			ret = aocc_kernel_write(usf->wake, req, reqlen);
		if (ret >= 0)
			s->period_ns = period_ns;
	}
	mutex_unlock(&usf->ctl_lock);

	if (ret < 0) {
		dev_warn(usf->dev, "ReconfigSampling(rate) failed: %d\n", ret);
		return ret;
	}
	return 0;
}

static const struct iio_info usf_iio_info = {
	.read_raw = usf_read_raw,
	.write_raw = usf_write_raw,
};

/* Case-insensitive substring test. */
static bool usf_name_has(const char *name, const char *sub)
{
	size_t nl = strlen(name), sl = strlen(sub), i, j;

	if (!sl)
		return true;
	for (i = 0; i + sl <= nl; i++) {
		for (j = 0; j < sl; j++)
			if (tolower(name[i + j]) != tolower(sub[j]))
				break;
		if (j == sl)
			return true;
	}
	return false;
}

/*
 * IEEE-754 binary32 -> round(|value| * mul * 2^q) as a u64 magnitude, integer
 * math only (the kernel must not touch the FPU). @neg (if given) gets the sign.
 * Saturates to U64_MAX on overflow / non-finite so callers can clamp.
 */
static u64 usf_f32_scaled(u32 bits, u64 mul, unsigned int q, bool *neg)
{
	u32 mant = bits & 0x7fffff;
	int exp = (bits >> 23) & 0xff;
	u64 num;
	int e;

	if (neg)
		*neg = bits & 0x80000000u;
	if (exp == 0)
		return 0;			/* zero / subnormal ~= 0 */
	if (exp == 0xff)
		return U64_MAX;			/* inf / NaN */

	num = ((u64)1 << 23) | mant;		/* value = num * 2^(exp-127-23) */
	if (mul > 1) {
		if (num > U64_MAX / mul)
			return U64_MAX;
		num *= mul;
	}
	e = exp - 127 - 23 + (int)q;		/* fold the 2^q into the exponent */
	if (e >= 0) {
		if (e >= 64 || num > (U64_MAX >> e))
			return U64_MAX;
		return num << e;
	}
	e = -e;
	if (e >= 64)
		return 0;
	return (num + (1ULL << (e - 1))) >> e;	/* round-nearest */
}

static s32 usf_clamp_s32(u64 mag, bool neg)
{
	if (mag > S32_MAX)
		return neg ? S32_MIN : S32_MAX;
	return neg ? -(s32)mag : (s32)mag;
}

/* Quantise one sample axis to a raw scan value for its device's scale. */
static s32 usf_scale_sample(const struct usf_sensor *s, u32 val_bits)
{
	bool neg;
	u64 val_q;

	if (!s->res_q)		/* no resolution reported: micro-unit fallback */
		return usf_clamp_s32(usf_f32_scaled(val_bits, 1000000, 0, &neg),
				     neg);

	val_q = usf_f32_scaled(val_bits, 1, USF_SCALE_Q, &neg);
	if (val_q == U64_MAX)
		return neg ? S32_MIN : S32_MAX;
	return usf_clamp_s32((val_q + s->res_q / 2) / s->res_q, neg);
}

/*
 * Map a sample's AoC timestamp to CLOCK_BOOTTIME. The AoC stamps samples in ns
 * off the architected counter it shares with the AP, so aoc_ts_to_boottime_ns()
 * recovers the exact boottime; on the rare miss (firmware not ready, or a stamp
 * too far from now to trust) fall back to the current boottime. The first
 * mapping is logged once so the mapping can be checked on HW (a healthy sample
 * is a few ms old).
 */
static s64 usf_sample_boottime(struct usf_iio *usf, u64 aoc_ts)
{
	u64 boot = aoc_ts_to_boottime_ns(aoc_ts);

	if (!usf->ts_logged) {
		u64 now = ktime_get_boottime_ns();

		usf->ts_logged = true;
		dev_info(usf->dev,
			 "timestamp map: aoc_ts=%llu -> boottime=%llu (now=%llu, age=%lld ns)%s\n",
			 aoc_ts, boot, now, (s64)(now - boot),
			 boot ? "" : " [untrusted, substituting now]");
	}
	return (s64)(boot ? boot : ktime_get_boottime_ns());
}

/* Parse a type-9 compact sample batch and push each record to the IIO buffer. */
static void usf_handle_sample(struct usf_iio *usf, const u8 *pay, u32 plen)
{
	struct usf_sensor *match = NULL;
	struct iio_dev *indio;
	struct usf_sample_hdr hdr;
	u32 scount, dcount, sid, i, d;
	size_t stride, off;
	int k, n;
	/*
	 * The device scan is n data s32 at offsets 0.. then an 8-aligned s64
	 * timestamp. One 3-axis-sized buffer serves both scalar and vector
	 * devices: iio_push writes the timestamp using the device's scan_bytes,
	 * so for a scalar device it lands at offset 8 (over the unused axes).
	 */
	struct {
		s32 chan[3];
		aligned_s64 timestamp;
	} scan;

	if (plen < sizeof(hdr))
		return;
	memcpy(&hdr, pay, sizeof(hdr));
	sid = hdr.sampling_id;
	scount = usf_sample_count(&hdr);
	dcount = usf_sample_dcount(&hdr);
	stride = 8 + (size_t)dcount * 4;

	/*
	 * Hold sample_lock across the sampling_id lookup and the pushes:
	 * usf_stop_sampling() clears sampling_id under the same lock on buffer
	 * disable, so once it returns no push can be in flight to race the IIO
	 * buffer teardown. iio_push_to_buffers is non-sleeping (kfifo), so this
	 * is safe under a spinlock.
	 */
	spin_lock(&usf->sample_lock);
	for (k = 0; k < usf->nsensors; k++) {
		if (usf->sensors[k]->sampling_id &&
		    usf->sensors[k]->sampling_id == sid) {
			match = usf->sensors[k];
			break;
		}
	}
	if (!match) {
		spin_unlock(&usf->sample_lock);
		return;			/* not one of our active streams */
	}
	indio = match->indio;
	n = min_t(int, match->ndata, dcount);

	for (i = 0; i < scount; i++) {
		u64 tsp;
		s64 ts;

		off = 16 + (size_t)i * stride;
		if (off + stride > plen)
			break;
		tsp = get_unaligned_le64(pay + off);
		ts = usf_sample_boottime(usf, tsp & USF_SAMPLE_TS_MASK);

		memset(&scan, 0, sizeof(scan));
		for (d = 0; d < (u32)n; d++)
			scan.chan[d] = usf_scale_sample(match,
				get_unaligned_le32(pay + off + 8 + d * 4));

		iio_push_to_buffers_with_ts(indio, &scan, sizeof(scan), ts);
	}
	spin_unlock(&usf->sample_lock);
}

/*
 * AOCC rx callback. Runs in the per-service demux kthread; must not sleep.
 * Responses (type 2) matching the awaited txn wake the control path; async
 * samples (type 9) are demuxed to the IIO buffer by sampling_id.
 */
static void usf_iio_channel_rx(void *ctx, const void *payload, size_t len)
{
	struct usf_iio *usf = ctx;
	const u8 *pay;
	u32 type, plen, rtxn;
	unsigned long flags;
	bool matched = false;

	if (!usf_parse_outer(payload, len, &type, &pay, &plen) || !pay)
		return;
	if (type == USF_T_SAMPLE) {
		usf_handle_sample(usf, pay, plen);
		return;
	}
	if (type != USF_T_RESPONSE)
		return;

	rtxn = usf_resp_txn(pay, plen);

	spin_lock_irqsave(&usf->resp_lock, flags);
	if (usf->resp_txn != U32_MAX && rtxn == usf->resp_txn) {
		usf->resp_len = min_t(u32, plen, USF_RESP_MAX);
		memcpy(usf->resp, pay, usf->resp_len);
		usf->resp_ready = rtxn;	/* mark which txn's payload resp[] holds */
		usf->resp_txn = U32_MAX;
		matched = true;
	}
	spin_unlock_irqrestore(&usf->resp_lock, flags);

	if (matched)
		complete(&usf->resp_done);
}

/*
 * Send one request on the wake channel and wait for its response. Caller holds
 * ctl_lock (which also owns @fbb, so @req must stay valid across the write).
 * On success @resp/@resp_len point at usf->resp (valid until the next call).
 */
static int usf_ctl(struct usf_iio *usf, u32 txn, const u8 *req, size_t reqlen,
		   const u8 **resp, u32 *resp_len)
{
	unsigned long flags;
	long left;
	int ret;

	reinit_completion(&usf->resp_done);
	spin_lock_irqsave(&usf->resp_lock, flags);
	usf->resp_txn = txn;
	usf->resp_ready = U32_MAX;	/* discard any stale payload */
	spin_unlock_irqrestore(&usf->resp_lock, flags);

	ret = aocc_kernel_write(usf->wake, req, reqlen);
	if (ret < 0)
		goto clear;

	left = wait_for_completion_timeout(&usf->resp_done,
					   msecs_to_jiffies(USF_CTL_TIMEOUT_MS));

	/*
	 * Trust resp[] only if it actually holds this txn's payload. The
	 * completion token alone is not enough: a stalled rx could complete()
	 * for a prior txn after that call already timed out, leaking a wakeup
	 * into this one. resp_ready closes that window deterministically.
	 */
	spin_lock_irqsave(&usf->resp_lock, flags);
	usf->resp_txn = U32_MAX;
	if (usf->resp_ready == txn) {
		*resp = usf->resp;
		*resp_len = usf->resp_len;
		ret = 0;
	} else {
		ret = left ? -EIO : -ETIMEDOUT;
	}
	spin_unlock_irqrestore(&usf->resp_lock, flags);
	return ret;

clear:
	spin_lock_irqsave(&usf->resp_lock, flags);
	usf->resp_txn = U32_MAX;
	spin_unlock_irqrestore(&usf->resp_lock, flags);
	return ret;
}

/* GetServer(uuid) -> server handle (0 = not found / error). */
static u32 usf_get_server(struct usf_iio *usf, const u8 uuid[16],
			  const char *label)
{
	const u8 *req, *resp, *body;
	size_t reqlen;
	u32 rlen, blen, txn, handle = 0;
	int ret;

	mutex_lock(&usf->ctl_lock);
	txn = usf->txn++;
	ret = usf_build_get_server(&usf->fbb, txn, uuid, &req, &reqlen);
	if (!ret)
		ret = usf_ctl(usf, txn, req, reqlen, &resp, &rlen);
	if (!ret) {
		body = usf_resp_body(resp, rlen, &blen);
		if (body)
			handle = usf_fb_u32(body, blen, 0, 0);
	}
	mutex_unlock(&usf->ctl_lock);

	if (ret)
		dev_warn(usf->dev, "GetServer(%s) failed: %d\n", label, ret);
	else
		dev_info(usf->dev, "GetServer(%s) -> handle %u%s\n", label,
			 handle, handle ? "" : " (NOT FOUND)");
	return handle;
}

/* CreateSampling: start the AoC stream for this sensor. */
static int usf_start_sampling(struct usf_sensor *s)
{
	struct usf_iio *usf = s->usf;
	const u8 *req, *resp, *body;
	size_t reqlen;
	u32 rlen, blen, txn, sid = 0;
	s64 period_ns;
	int ret;

	period_ns = 1000000000LL / (s->samp_freq > 0 ? s->samp_freq : 50);
	s->client_id = 0xC0DE0000u | s->handle;
	s->period_ns = period_ns;

	mutex_lock(&usf->ctl_lock);
	txn = usf->txn++;
	ret = usf_build_create_sampling(&usf->fbb, txn, s->handle, period_ns,
					s->client_id, &req, &reqlen);
	if (!ret)
		ret = usf_ctl(usf, txn, req, reqlen, &resp, &rlen);
	if (!ret) {
		body = usf_resp_body(resp, rlen, &blen);
		if (body)
			sid = usf_fb_u32(body, blen, 0, 0);
	}
	mutex_unlock(&usf->ctl_lock);

	if (ret) {
		dev_err(usf->dev, "CreateSampling failed: %d\n", ret);
		return ret;
	}

	spin_lock(&usf->sample_lock);
	s->sampling_id = sid;
	spin_unlock(&usf->sample_lock);
	dev_info(usf->dev, "%s streaming: sampling_id=%u period=%lld ns\n",
		 s->indio->name, sid, period_ns);
	return 0;
}

/*
 * StopSampling: end the AoC stream for this sensor.
 *
 * This must be the dedicated StopSampling request, sent synchronously and
 * checked for the firmware's response. The firmware silently ignores
 * ReconfigSampling(enable=0), and an unchecked send hid that: every buffer
 * disable leaked a live session, and the leaked streams accumulated into a
 * multi-kHz mailbox interrupt storm.
 */
static int usf_stop_sampling(struct usf_sensor *s)
{
	struct usf_iio *usf = s->usf;
	const u8 *req, *resp;
	size_t reqlen;
	u32 txn, sid, rlen;
	int ret;

	spin_lock(&usf->sample_lock);
	sid = s->sampling_id;
	s->sampling_id = 0;		/* stop accepting samples immediately */
	spin_unlock(&usf->sample_lock);
	if (!sid)
		return 0;

	mutex_lock(&usf->ctl_lock);
	txn = usf->txn++;
	ret = usf_build_stop_sampling(&usf->fbb, txn, s->handle, sid,
				      &req, &reqlen);
	if (!ret)
		ret = usf_ctl(usf, txn, req, reqlen, &resp, &rlen);
	mutex_unlock(&usf->ctl_lock);

	if (ret)
		dev_warn(usf->dev,
			 "StopSampling(handle=0x%x id=%u) unacknowledged (%d), session may leak\n",
			 s->handle, sid, ret);
	return 0;
}

static int usf_buffer_postenable(struct iio_dev *indio_dev)
{
	return usf_start_sampling(iio_priv(indio_dev));
}

static int usf_buffer_predisable(struct iio_dev *indio_dev)
{
	return usf_stop_sampling(iio_priv(indio_dev));
}

static const struct iio_buffer_setup_ops usf_buffer_ops = {
	.postenable = usf_buffer_postenable,
	.predisable = usf_buffer_predisable,
};

/* GetSensorList with settle-retry -> handles[]. Returns count or -errno. */
static int usf_sensor_list(struct usf_iio *usf, u32 *handles, int max)
{
	const u8 *req, *resp, *body, *vec;
	size_t reqlen;
	u32 rlen, blen, txn, n;
	int ret, tries, i, count;

	for (tries = 0; tries < USF_SETTLE_TRIES; tries++) {
		count = 0;
		mutex_lock(&usf->ctl_lock);
		txn = usf->txn++;
		ret = usf_build_no_body(&usf->fbb, USF_MSG_GET_SENSOR_LIST, txn,
					usf->sensor_mgr, &req, &reqlen);
		if (!ret)
			ret = usf_ctl(usf, txn, req, reqlen, &resp, &rlen);
		if (!ret) {
			body = usf_resp_body(resp, rlen, &blen);
			vec = body ? usf_fb_vec(body, blen, 0, &n) : NULL;
			if (vec) {
				count = min_t(u32, n, (u32)max);
				for (i = 0; i < count; i++)
					handles[i] = get_unaligned_le32(vec + i * 4);
			}
		}
		mutex_unlock(&usf->ctl_lock);

		if (ret)
			return ret;
		if (count > 0)
			return count;
		/* Registry loaded, but AoC's async chip probe hasn't settled. */
		msleep(USF_SETTLE_MS);
	}
	return 0;
}

/*
 * GetSensorInfo(handle) -> name into @out, and the id9 resolution (f32 bits, 0
 * if absent) into @res_bits. Returns 0 or -errno.
 */
static int usf_sensor_info(struct usf_iio *usf, u32 handle, char *out,
			   size_t outsz, u32 *res_bits)
{
	const u8 *req, *resp, *body;
	size_t reqlen;
	u32 rlen, blen, txn;
	int ret;

	*res_bits = 0;
	mutex_lock(&usf->ctl_lock);
	txn = usf->txn++;
	ret = usf_build_no_body(&usf->fbb, USF_MSG_GET_SENSOR_INFO, txn, handle,
				&req, &reqlen);
	if (!ret)
		ret = usf_ctl(usf, txn, req, reqlen, &resp, &rlen);
	if (!ret) {
		body = usf_resp_body(resp, rlen, &blen);
		if (!body || usf_fb_string(body, blen, 0, out, outsz) < 0) {
			out[0] = '\0';
			ret = -ENODATA;
		} else {
			*res_bits = usf_fb_u32(body, blen, 9, 0);
		}
	}
	mutex_unlock(&usf->ctl_lock);
	return ret;
}

/*
 * Derive the IIO scale from the sensor's reported resolution (id9). raw =
 * round(value / resolution) [res_q = resolution << USF_SCALE_Q] and SCALE =
 * resolution * unit factor. If the firmware gives no usable resolution, fall
 * back to fixed micro-units (SCALE = 1e-6 * unit factor), matching the original
 * behaviour for that sensor.
 */
static void usf_set_scale(struct usf_sensor *s, const struct usf_type_map *map,
			  u32 res_bits)
{
	u64 res_q = usf_f32_scaled(res_bits, 1, USF_SCALE_Q, NULL);
	u64 scale = usf_f32_scaled(res_bits, map->unit_nano, 0, NULL);

	if (res_q && res_q != U64_MAX && scale && scale != U64_MAX) {
		s->res_q = res_q;
		s->scale_nano = scale;
	} else {
		s->res_q = 0;			/* micro-unit fallback */
		s->scale_nano = map->unit_nano / 1000000;
	}
}

static int usf_register_sensor(struct usf_iio *usf, u32 handle,
			       const char *name,
			       const struct usf_type_map *map, u32 res_bits)
{
	struct iio_dev *indio;
	struct usf_sensor *s;
	int ret;

	if (usf->nsensors >= USF_MAX_DEV)
		return -ENOSPC;

	indio = devm_iio_device_alloc(usf->dev, sizeof(*s));
	if (!indio)
		return -ENOMEM;
	s = iio_priv(indio);
	s->usf = usf;
	s->indio = indio;
	s->handle = handle;
	s->ndata = map->ndata;
	s->samp_freq = rate > 0 ? rate : 50;
	usf_set_scale(s, map, res_bits);

	indio->name = map->iio_name;
	indio->info = &usf_iio_info;
	indio->modes = INDIO_DIRECT_MODE;
	indio->channels = map->channels;
	indio->num_channels = map->num_channels;
	indio->available_scan_masks = map->scan_masks;

	ret = devm_iio_kfifo_buffer_setup(usf->dev, indio, &usf_buffer_ops);
	if (ret)
		return ret;

	ret = devm_iio_device_register(usf->dev, indio);
	if (ret)
		return ret;

	usf->sensors[usf->nsensors++] = s;
	dev_info(usf->dev,
		 "registered %s for USF '%s' (handle 0x%x) scale=%llu.%09llu%s\n",
		 map->iio_name, name, handle, s->scale_nano / USF_NANO,
		 s->scale_nano % USF_NANO, s->res_q ? "" : " (micro fallback)");
	return 0;
}

/*
 * Enumerate the AoC sensors and register an IIO device for each supported one
 * (optionally filtered by the sensor= module param).
 */
static void usf_enumerate(struct usf_iio *usf)
{
	u32 handles[USF_LIST_MAX];
	char name[USF_NAME_MAX];
	bool filtered = sensor && sensor[0];
	int count, i, m;
	u32 res_bits;

	count = usf_sensor_list(usf, handles, USF_LIST_MAX);
	if (count <= 0) {
		dev_warn(usf->dev, "GetSensorList returned %d\n", count);
		return;
	}
	dev_info(usf->dev, "enumerated %d USF sensors\n", count);

	for (i = 0; i < count; i++) {
		if (usf_sensor_info(usf, handles[i], name, sizeof(name), &res_bits))
			continue;
		dev_info(usf->dev, "  handle 0x%02x : %s\n", handles[i], name);

		if (filtered && !usf_name_has(name, sensor))
			continue;
		for (m = 0; m < (int)ARRAY_SIZE(usf_type_maps); m++) {
			if (usf_name_has(name, usf_type_maps[m].match)) {
				usf_register_sensor(usf, handles[i], name,
						    &usf_type_maps[m], res_bits);
				break;
			}
		}
	}
	if (!usf->nsensors)
		dev_warn(usf->dev, "no registerable sensors%s%s\n",
			 filtered ? " matching " : "", filtered ? sensor : "");
}

static int usf_open_channels(struct usf_iio *usf)
{
	int ret;

	usf->wake = aocc_kernel_open_channel(USF_WAKE_SERVICE,
					     usf_iio_channel_rx, usf);
	if (IS_ERR(usf->wake)) {
		ret = PTR_ERR(usf->wake);
		usf->wake = NULL;
		return ret;
	}

	usf->nonwake = aocc_kernel_open_channel(USF_NONWAKE_SERVICE,
						usf_iio_channel_rx, usf);
	if (IS_ERR(usf->nonwake)) {
		ret = PTR_ERR(usf->nonwake);
		usf->nonwake = NULL;
		aocc_kernel_close_channel(usf->wake);
		usf->wake = NULL;
		return ret;
	}

	return 0;
}

static void usf_close_channels(struct usf_iio *usf)
{
	if (usf->nonwake) {
		aocc_kernel_close_channel(usf->nonwake);
		usf->nonwake = NULL;
	}
	if (usf->wake) {
		aocc_kernel_close_channel(usf->wake);
		usf->wake = NULL;
	}
}

/* Reschedule the bootstrap if we have retries left; otherwise give up. */
static bool usf_retry(struct usf_iio *usf, const char *why)
{
	if (usf->retries++ < USF_MAX_RETRIES) {
		dev_dbg(usf->dev, "%s; retry %u/%u\n", why, usf->retries,
			USF_MAX_RETRIES);
		schedule_delayed_work(&usf->enum_work,
				      msecs_to_jiffies(USF_RETRY_MS));
		return true;
	}
	dev_warn(usf->dev, "giving up after %u retries: %s\n", usf->retries, why);
	return false;
}

static void usf_iio_enumerate_work(struct work_struct *work)
{
	struct usf_iio *usf = container_of(to_delayed_work(work),
					   struct usf_iio, enum_work);
	u32 h_reg, h_sensor, h_sample;
	int ret;

	if (usf->enumerated)
		return;

	ret = usf_open_channels(usf);
	if (ret) {
		/* AoC / AOCC not up yet: transient, keep retrying. */
		usf_retry(usf, "AOCC channels not ready");
		return;
	}

	h_reg = usf_get_server(usf, usf_uuid_registry, "Registry");
	h_sensor = usf_get_server(usf, usf_uuid_sensor_mgr, "SensorMgr");
	h_sample = usf_get_server(usf, usf_uuid_sample_channel, "SampleChannel");

	if (!h_sensor || !h_sample) {
		/* Servers dormant: the registry is not loaded yet. */
		usf_close_channels(usf);
		usf_retry(usf, "USF servers not responding (registry loaded?)");
		return;
	}

	usf->sensor_mgr = h_sensor;
	usf->sample_chan = h_sample;
	usf->enumerated = true;
	dev_info(usf->dev,
		 "USF bootstrap OK (registry=%u sensor_mgr=%u sample_chan=%u)\n",
		 h_reg, h_sensor, h_sample);

	usf_enumerate(usf);
}

static ssize_t enumerate_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct usf_iio *usf = dev_get_drvdata(dev);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;
	if (val && !usf->enumerated) {
		usf->retries = 0;
		schedule_delayed_work(&usf->enum_work, 0);
	}
	return count;
}
static DEVICE_ATTR_WO(enumerate);

static struct attribute *usf_iio_attrs[] = {
	&dev_attr_enumerate.attr,
	NULL,
};
ATTRIBUTE_GROUPS(usf_iio);

static int usf_iio_probe(struct platform_device *pdev)
{
	struct usf_iio *usf;

	usf = devm_kzalloc(&pdev->dev, sizeof(*usf), GFP_KERNEL);
	if (!usf)
		return -ENOMEM;

	usf->dev = &pdev->dev;
	usf->txn = 1;
	usf->resp_txn = U32_MAX;
	usf->resp_ready = U32_MAX;
	mutex_init(&usf->ctl_lock);
	spin_lock_init(&usf->resp_lock);
	spin_lock_init(&usf->sample_lock);
	init_completion(&usf->resp_done);
	INIT_DELAYED_WORK(&usf->enum_work, usf_iio_enumerate_work);
	platform_set_drvdata(pdev, usf);

	if (autopoll)
		schedule_delayed_work(&usf->enum_work, 0);

	return 0;
}

static void usf_iio_remove(struct platform_device *pdev)
{
	struct usf_iio *usf = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&usf->enum_work);
	usf_close_channels(usf);
}

static struct platform_driver usf_iio_driver = {
	.driver = {
		.name = "aoc-usf-iio",
		.dev_groups = usf_iio_groups,
	},
	.probe = usf_iio_probe,
	.remove = usf_iio_remove,
};

static struct platform_device *usf_iio_pdev;

static int __init usf_iio_init(void)
{
	int ret;

	ret = platform_driver_register(&usf_iio_driver);
	if (ret)
		return ret;

	/*
	 * No DT node describes this bridge (the AoC firmware, not the DT, owns
	 * the sensors), so self-instantiate a platform device to hang the sysfs
	 * trigger and the IIO devices off of.
	 */
	usf_iio_pdev = platform_device_register_simple("aoc-usf-iio", -1, NULL, 0);
	if (IS_ERR(usf_iio_pdev)) {
		platform_driver_unregister(&usf_iio_driver);
		return PTR_ERR(usf_iio_pdev);
	}

	return 0;
}
module_init(usf_iio_init);

static void __exit usf_iio_exit(void)
{
	platform_device_unregister(usf_iio_pdev);
	platform_driver_unregister(&usf_iio_driver);
}
module_exit(usf_iio_exit);

MODULE_DESCRIPTION("AoC USF sensor framework to IIO bridge");
MODULE_LICENSE("GPL");
