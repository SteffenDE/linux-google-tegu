// SPDX-License-Identifier: GPL-2.0-only
/*
 * AoC USF sensor framework -> IIO bridge.
 *
 * Bridges the sensors hosted by the AoC USF (Unified Sensor Framework)
 * firmware to Linux IIO. The AoC sensor registry is loaded out of band by a
 * userspace daemon; once it is loaded this driver enumerates the sensors over
 * the com.google.usf AOCC channel and exposes them as IIO devices. This is the
 * data-plane half described in research/usf-iio-bridge.md.
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
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <linux/iio/iio.h>

#include <soc/google/aoc_channel.h>

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

static bool autopoll;
module_param(autopoll, bool, 0644);
MODULE_PARM_DESC(autopoll,
		 "enumerate sensors at probe instead of waiting for the sysfs poke");

static char *sensor = "gyro";
module_param(sensor, charp, 0644);
MODULE_PARM_DESC(sensor,
		 "USF sensor name substring to expose for P2a bring-up (e.g. gyro, accel)");

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

	/* The single IIO device registered for P2a (extended to many later). */
	struct iio_dev *sensor_indio;
};

/* Per-IIO-device state (in iio_priv). */
struct usf_sensor {
	struct usf_iio *usf;
	u32 handle;		/* USF sensor handle (dst for Create/Reconfig) */
};

/*
 * Sample axes arrive as f32 in Android sensor units; the data path quantises
 * each to micro-units (value * 1e6) as an s32 raw, so SCALE is a fixed 1e-6.
 * Unit-correct per-type scaling (mag Gauss, pressure kPa, ...) comes later.
 */
#define USF_AXIS(_type, _mod, _idx) {				\
	.type = _type,						\
	.modified = 1,						\
	.channel2 = IIO_MOD_##_mod,				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
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

/* Map a USF sensor name (substring) to an IIO device type + channels. */
static const struct usf_type_map {
	const char *match;
	const char *iio_name;
	const struct iio_chan_spec *channels;
	int num_channels;
} usf_type_maps[] = {
	{ "Accelerometer", "usf_accel", usf_accel_channels,
	  ARRAY_SIZE(usf_accel_channels) },
	{ "Gyroscope", "usf_gyro", usf_gyro_channels,
	  ARRAY_SIZE(usf_gyro_channels) },
	{ "Magnetometer", "usf_magn", usf_magn_channels,
	  ARRAY_SIZE(usf_magn_channels) },
};

static int usf_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = 1;		/* 1e-6: raw is (value * 1e6) */
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static const struct iio_info usf_iio_info = {
	.read_raw = usf_read_raw,
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
 * AOCC rx callback. Runs in the per-service demux kthread; must not sleep.
 * Responses (type 2) matching the awaited txn wake the control path; async
 * samples (type 9) are handled by the data path (follow-up commit).
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
	if (type != USF_T_RESPONSE)
		return;			/* samples (type 9) handled later */

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

/* GetSensorInfo(handle) -> name into @out. Returns 0 or -errno. */
static int usf_sensor_name(struct usf_iio *usf, u32 handle, char *out,
			   size_t outsz)
{
	const u8 *req, *resp, *body;
	size_t reqlen;
	u32 rlen, blen, txn;
	int ret;

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
		}
	}
	mutex_unlock(&usf->ctl_lock);
	return ret;
}

static int usf_register_sensor(struct usf_iio *usf, u32 handle,
			       const char *name,
			       const struct usf_type_map *map)
{
	struct iio_dev *indio;
	struct usf_sensor *s;
	int ret;

	indio = devm_iio_device_alloc(usf->dev, sizeof(*s));
	if (!indio)
		return -ENOMEM;
	s = iio_priv(indio);
	s->usf = usf;
	s->handle = handle;

	indio->name = map->iio_name;
	indio->info = &usf_iio_info;
	indio->modes = INDIO_DIRECT_MODE;
	indio->channels = map->channels;
	indio->num_channels = map->num_channels;

	ret = devm_iio_device_register(usf->dev, indio);
	if (ret)
		return ret;

	usf->sensor_indio = indio;
	dev_info(usf->dev, "registered %s for USF '%s' (handle 0x%x)\n",
		 map->iio_name, name, handle);
	return 0;
}

/* Enumerate the AoC sensors and register the one selected by the sensor param. */
static void usf_enumerate(struct usf_iio *usf)
{
	u32 handles[USF_LIST_MAX];
	char name[USF_NAME_MAX];
	int count, i, m;

	count = usf_sensor_list(usf, handles, USF_LIST_MAX);
	if (count <= 0) {
		dev_warn(usf->dev, "GetSensorList returned %d\n", count);
		return;
	}
	dev_info(usf->dev, "enumerated %d USF sensors\n", count);

	for (i = 0; i < count; i++) {
		if (usf_sensor_name(usf, handles[i], name, sizeof(name)))
			continue;
		dev_info(usf->dev, "  handle 0x%02x : %s\n", handles[i], name);

		if (usf->sensor_indio || !usf_name_has(name, sensor))
			continue;	/* P2a: register the first chosen match */
		for (m = 0; m < (int)ARRAY_SIZE(usf_type_maps); m++) {
			if (usf_name_has(name, usf_type_maps[m].match)) {
				usf_register_sensor(usf, handles[i], name,
						    &usf_type_maps[m]);
				break;
			}
		}
	}
	if (!usf->sensor_indio)
		dev_warn(usf->dev, "no registerable sensor matching '%s'\n",
			 sensor);
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
