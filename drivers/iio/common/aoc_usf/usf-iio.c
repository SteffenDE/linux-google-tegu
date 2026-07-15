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

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

static bool autopoll;
module_param(autopoll, bool, 0644);
MODULE_PARM_DESC(autopoll,
		 "enumerate sensors at probe instead of waiting for the sysfs poke");

struct usf_iio {
	struct device *dev;
	struct work_struct enum_work;
	bool enumerated;
};

static void usf_iio_enumerate_work(struct work_struct *work)
{
	struct usf_iio *usf = container_of(work, struct usf_iio, enum_work);

	if (usf->enumerated)
		return;

	/* Bootstrap + enumeration + IIO registration land here (follow-ups). */
	dev_info(usf->dev, "enumerate trigger (stub)\n");
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
	if (val)
		schedule_work(&usf->enum_work);
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
	INIT_WORK(&usf->enum_work, usf_iio_enumerate_work);
	platform_set_drvdata(pdev, usf);

	if (autopoll)
		schedule_work(&usf->enum_work);

	return 0;
}

static void usf_iio_remove(struct platform_device *pdev)
{
	struct usf_iio *usf = platform_get_drvdata(pdev);

	cancel_work_sync(&usf->enum_work);
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
