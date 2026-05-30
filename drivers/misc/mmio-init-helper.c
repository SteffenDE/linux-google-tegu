// SPDX-License-Identifier: GPL-2.0-only
/* Tiny one-shot MMIO write helper for early hardware bring-up. */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>

static int mmio_init_helper_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	void __iomem *base;
	u32 mask;
	u32 old;
	u32 readback;
	u32 write;
	u32 value;
	bool has_mask;
	int ret;

	ret = device_property_read_u32(dev, "value", &value);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read value\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return dev_err_probe(dev, -EINVAL, "missing MMIO resource\n");

	if (resource_size(res) < sizeof(u32))
		return dev_err_probe(dev, -EINVAL, "MMIO resource is too small\n");

	base = devm_ioremap(dev, res->start, resource_size(res));
	if (!base)
		return dev_err_probe(dev, -ENOMEM, "failed to map %pR\n", res);

	has_mask = !device_property_read_u32(dev, "mask", &mask);
	if (has_mask) {
		old = readl(base);
		write = (old & ~mask) | (value & mask);
	} else {
		old = 0;
		mask = 0;
		write = value;
	}

	writel(write, base);
	readback = readl(base);

	if (has_mask)
		dev_info(dev,
			 "updated %pR mask 0x%08x value 0x%08x: 0x%08x -> 0x%08x, read back 0x%08x\n",
			 res, mask, value, old, write, readback);
	else
		dev_info(dev, "wrote 0x%08x to %pR, read back 0x%08x\n",
			 write, res, readback);

	return 0;
}

static const struct of_device_id mmio_init_helper_of_match[] = {
	{ .compatible = "linux,mmio-init-helper" },
	{ }
};
MODULE_DEVICE_TABLE(of, mmio_init_helper_of_match);

static struct platform_driver mmio_init_helper_driver = {
	.probe = mmio_init_helper_probe,
	.driver = {
		.name = "mmio-init-helper",
		.of_match_table = mmio_init_helper_of_match,
	},
};

static int __init mmio_init_helper_driver_init(void)
{
	return platform_driver_register(&mmio_init_helper_driver);
}
subsys_initcall(mmio_init_helper_driver_init);

MODULE_DESCRIPTION("One-shot MMIO init helper");
MODULE_LICENSE("GPL");
