// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Tensor S2MPU support for a host without protected DMA domains.
 * Copyright 2026 Steffen Deusch
 *
 * Follow the non-pKVM path of Google's GPL S2MPU driver: permit physical
 * accesses after each power-on. The master's stage-1 IOMMU still owns DMA
 * translations and permissions. This driver does not provide guest isolation.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#define S2MPU_PROT_EN_PER_VID_CLR	0x54
#define S2MPU_VERSION		0x60
#define S2MPU_VERSION_MASK	GENMASK(31, 16)
#define S2MPU_VERSION_9		0x90000000
#define S2MPU_ALL_VIDS		GENMASK(7, 0)

struct google_s2mpu {
	void __iomem *base;
	struct clk *clk;
};

static int google_s2mpu_runtime_resume(struct device *dev)
{
	struct google_s2mpu *s2mpu = dev_get_drvdata(dev);
	u32 version;
	int ret;

	ret = clk_prepare_enable(s2mpu->clk);
	if (ret)
		return ret;

	version = readl(s2mpu->base + S2MPU_VERSION);
	if ((version & S2MPU_VERSION_MASK) != S2MPU_VERSION_9) {
		dev_err(dev, "unsupported S2MPU version %#x\n", version);
		clk_disable_unprepare(s2mpu->clk);
		return -ENODEV;
	}

	/* Protection resets to blocking, including stage-1 page-table walks. */
	writel(S2MPU_ALL_VIDS, s2mpu->base + S2MPU_PROT_EN_PER_VID_CLR);
	readl(s2mpu->base + S2MPU_VERSION);
	return 0;
}

static int google_s2mpu_runtime_suspend(struct device *dev)
{
	struct google_s2mpu *s2mpu = dev_get_drvdata(dev);

	clk_disable_unprepare(s2mpu->clk);
	return 0;
}

static int google_s2mpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct google_s2mpu *s2mpu;
	int ret;

	s2mpu = devm_kzalloc(dev, sizeof(*s2mpu), GFP_KERNEL);
	if (!s2mpu)
		return -ENOMEM;

	s2mpu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(s2mpu->base))
		return PTR_ERR(s2mpu->base);

	s2mpu->clk = devm_clk_get(dev, "bus");
	if (IS_ERR(s2mpu->clk))
		return dev_err_probe(dev, PTR_ERR(s2mpu->clk), "missing bus clock\n");

	platform_set_drvdata(pdev, s2mpu);
	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		pm_runtime_disable(dev);
		return ret;
	}
	pm_runtime_put_sync(dev);
	return 0;
}

static void google_s2mpu_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		google_s2mpu_runtime_suspend(dev);
	pm_runtime_set_suspended(dev);
}

/*
 * Late, like the System MMU that links to this device: the device list then
 * closes the S2MPU after the MMU at suspend and reopens it before the MMU at
 * resume, the order the runtime path produces through the link.
 */
static const struct dev_pm_ops google_s2mpu_pm_ops = {
	RUNTIME_PM_OPS(google_s2mpu_runtime_suspend, google_s2mpu_runtime_resume, NULL)
	LATE_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

static const struct of_device_id google_s2mpu_of_match[] = {
	{ .compatible = "google,zumapro-s2mpu" },
	{ }
};
MODULE_DEVICE_TABLE(of, google_s2mpu_of_match);

static struct platform_driver google_s2mpu_driver = {
	.probe = google_s2mpu_probe,
	.remove = google_s2mpu_remove,
	.driver = {
		.name = "google-s2mpu",
		.of_match_table = google_s2mpu_of_match,
		.pm = pm_ptr(&google_s2mpu_pm_ops),
	},
};
module_platform_driver(google_s2mpu_driver);

MODULE_DESCRIPTION("Google Tensor S2MPU host access controller");
MODULE_LICENSE("GPL");
