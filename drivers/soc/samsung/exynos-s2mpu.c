// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung/Google Exynos S2MPU bring-up bypass driver.
 *
 * An S2MPU (Stage-2 Memory Protection Unit) sits between a DMA master and the
 * system bus.  While enforcing it blocks all of that master's DMA traffic until
 * software programs its memory-protection table.  On zumapro (Google Tensor G4)
 * the downstream stack programs those tables from pKVM at EL2.  Mainline has no
 * such owner, so a master like the GPU would fault the instant it touches DRAM
 * through an enforcing S2MPU.
 *
 * This driver takes the same escape hatch the downstream uses when pKVM is not
 * present (google-modules pkvm_s2mpu.c: "bypass S2MPU if pKVM is not there"):
 * it disables enforcement for every VID, letting the master's DMA through
 * unchecked.  It is a bring-up shim, not a real S2MPU/IOMMU driver -- it
 * provides no memory protection.
 *
 * NOTE: on tegu the bootloader already hands the G3D S2MPU over open, so this
 * bypass is a no-op on the current boot path and was not what GPU first light
 * needed (that was the panthor ASN hash).  It is kept for boot paths that leave
 * the unit enforcing.
 *
 * The S2MPU returns to its all-blocked reset state whenever its power domain is
 * turned off, so the bypass is (re-)applied both at probe and on every power-on
 * of the domain via a genpd ON notifier.
 *
 * Copyright 2026 Steffen Deusch
 */

#include <linux/bits.h>
#include <linux/container_of.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>

/*
 * google,s2mpu-v9 register used to open the unit: writing a VID's bit to
 * PROT_EN_PER_VID_CLR disables protection enforcement for that VID.  The unit
 * has 8 VIDs, so 0xff disables enforcement for all of them.
 */
#define S2MPU_V9_CTRL_PROT_EN_PER_VID_SET	0x50
#define S2MPU_V9_CTRL_PROT_EN_PER_VID_CLR	0x54
#define S2MPU_V9_NR_VIDS			8
#define S2MPU_V9_ALL_VIDS			GENMASK(S2MPU_V9_NR_VIDS - 1, 0)

struct exynos_s2mpu {
	void __iomem *base;
	struct notifier_block genpd_nb;
};

static void exynos_s2mpu_bypass(struct exynos_s2mpu *s2mpu)
{
	writel(S2MPU_V9_ALL_VIDS,
	       s2mpu->base + S2MPU_V9_CTRL_PROT_EN_PER_VID_CLR);
}

static int exynos_s2mpu_genpd_notify(struct notifier_block *nb,
				     unsigned long action, void *data)
{
	struct exynos_s2mpu *s2mpu =
		container_of(nb, struct exynos_s2mpu, genpd_nb);

	/* Re-open once the domain (and the S2MPU) has been powered back on. */
	if (action == GENPD_NOTIFY_ON)
		exynos_s2mpu_bypass(s2mpu);

	return NOTIFY_OK;
}

static int exynos_s2mpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_s2mpu *s2mpu;
	u32 before, after;
	int ret;

	s2mpu = devm_kzalloc(dev, sizeof(*s2mpu), GFP_KERNEL);
	if (!s2mpu)
		return -ENOMEM;

	s2mpu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(s2mpu->base))
		return PTR_ERR(s2mpu->base);

	platform_set_drvdata(pdev, s2mpu);

	/*
	 * Register the ON notifier before the first bypass so a power-on that
	 * races probe cannot leave the unit closed.  The notifier covers future
	 * power-cycles; the explicit call below covers first light, where the
	 * GPU power domain is always-on and so never triggers a transition.
	 */
	s2mpu->genpd_nb.notifier_call = exynos_s2mpu_genpd_notify;
	ret = dev_pm_genpd_add_notifier(dev, &s2mpu->genpd_nb);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register power-domain notifier\n");

	before = readl(s2mpu->base + S2MPU_V9_CTRL_PROT_EN_PER_VID_SET);
	exynos_s2mpu_bypass(s2mpu);
	after = readl(s2mpu->base + S2MPU_V9_CTRL_PROT_EN_PER_VID_SET);
	dev_info(dev, "protection disabled (bypass): PROT_EN 0x%02x -> 0x%02x\n",
		 before, after);

	return 0;
}

static void exynos_s2mpu_remove(struct platform_device *pdev)
{
	dev_pm_genpd_remove_notifier(&pdev->dev);
}

static const struct of_device_id exynos_s2mpu_of_match[] = {
	{ .compatible = "google,zumapro-s2mpu" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_s2mpu_of_match);

static struct platform_driver exynos_s2mpu_driver = {
	.driver = {
		.name = "exynos-s2mpu",
		.of_match_table = exynos_s2mpu_of_match,
	},
	.probe = exynos_s2mpu_probe,
	.remove = exynos_s2mpu_remove,
};
module_platform_driver(exynos_s2mpu_driver);

MODULE_DESCRIPTION("Exynos/Google S2MPU bring-up bypass driver");
MODULE_LICENSE("GPL");
