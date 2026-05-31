// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung System MMU group support.
 *
 * Newer Samsung/Google display pipelines describe multiple System MMU
 * controllers that should share one IOMMU group.  The real System MMU driver
 * consumes the group object through the phandle in samsung,iommu-group.
 */

#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

static int samsung_sysmmu_group_probe(struct platform_device *pdev)
{
	struct iommu_group *group;
	struct device *dev = &pdev->dev;
	int ret;

	group = iommu_group_alloc();
	if (IS_ERR(group))
		return dev_err_probe(dev, PTR_ERR(group),
				     "failed to allocate IOMMU group\n");

	ret = iommu_group_set_name(group, dev_name(dev));
	if (ret) {
		iommu_group_put(group);
		return dev_err_probe(dev, ret, "failed to name IOMMU group\n");
	}

	platform_set_drvdata(pdev, group);

	return 0;
}

static void samsung_sysmmu_group_remove(struct platform_device *pdev)
{
	struct iommu_group *group = platform_get_drvdata(pdev);

	iommu_group_put(group);
}

static const struct of_device_id samsung_sysmmu_group_of_match[] = {
	{ .compatible = "samsung,sysmmu-group" },
	{ }
};
MODULE_DEVICE_TABLE(of, samsung_sysmmu_group_of_match);

static struct platform_driver samsung_sysmmu_group_driver = {
	.probe = samsung_sysmmu_group_probe,
	.remove = samsung_sysmmu_group_remove,
	.driver = {
		.name = "samsung-sysmmu-group",
		.of_match_table = samsung_sysmmu_group_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(samsung_sysmmu_group_driver);

MODULE_DESCRIPTION("Samsung System MMU group support");
MODULE_LICENSE("GPL");
