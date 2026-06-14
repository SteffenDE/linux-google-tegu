// SPDX-License-Identifier: GPL-2.0
//
// Exynos Generic power domain support.
//
// Copyright (c) 2012 Samsung Electronics Co., Ltd.
//		http://www.samsung.com
//
// Implementation of Exynos specific power domain control which is used in
// conjunction with runtime-pm. Support for both device-tree and non-device-tree
// based power domain support is included.

#include <linux/arm-smccc.h>
#include <linux/bits.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pm_domain.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>

struct exynos_pm_domain_config {
	/* Value for LOCAL_PWR_CFG and STATUS fields for each domain */
	u32 local_pwr_cfg;
	bool secure_pmu;
};

/*
 * Exynos specific wrapper around the generic power domain
 */
struct exynos_pm_domain {
	void __iomem *base;
	void __iomem *cmu_option;
	phys_addr_t base_addr;
	struct generic_pm_domain pd;
	u32 local_pwr_cfg;
	u32 secure_pwr_id;
	bool secure_pmu;
};

#define EXYNOS_PD_SMC_CMD		0x82000410
#define EXYNOS_PD_SMC_SAVE		0
#define EXYNOS_PD_SMC_RESTORE		1
#define EXYNOS_PD_SMC_TZPC_GROUP	2
#define EXYNOS_PRIV_REG_SMC_CMD		0x82000504
#define EXYNOS_PRIV_REG_WRITE		1
#define EXYNOS_PD_CMU_RESET_DISABLE	BIT(24)

static void exynos_pd_secure_control(struct exynos_pm_domain *pd, bool power_on)
{
	struct arm_smccc_res res;

	if (!pd->secure_pwr_id)
		return;

	arm_smccc_smc(EXYNOS_PD_SMC_CMD,
		      power_on ? EXYNOS_PD_SMC_RESTORE : EXYNOS_PD_SMC_SAVE,
		      pd->secure_pwr_id, EXYNOS_PD_SMC_TZPC_GROUP,
		      0, 0, 0, 0, &res);

	if (res.a0)
		pr_warn("Power domain %s secure %s returned %lu\n",
			pd->pd.name, power_on ? "restore" : "save", res.a0);
}

static int exynos_pd_write_pmu(struct exynos_pm_domain *pd, u32 value)
{
	struct arm_smccc_res res;

	if (!pd->secure_pmu) {
		writel_relaxed(value, pd->base);
		return 0;
	}

	arm_smccc_smc(EXYNOS_PRIV_REG_SMC_CMD, pd->base_addr,
		      EXYNOS_PRIV_REG_WRITE, value, 0, 0, 0, 0, &res);

	if (res.a0) {
		pr_err("Power domain %s secure PMU write returned %lu\n",
		       pd->pd.name, res.a0);
		return -EIO;
	}

	return 0;
}

static int exynos_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	struct exynos_pm_domain *pd;
	void __iomem *base;
	u32 timeout, pwr;
	char *op;
	int ret;

	pd = container_of(domain, struct exynos_pm_domain, pd);
	base = pd->base;

	if (!power_on) {
		exynos_pd_secure_control(pd, false);

		if (pd->cmu_option)
			writel_relaxed(readl_relaxed(pd->cmu_option) &
				       ~EXYNOS_PD_CMU_RESET_DISABLE,
				       pd->cmu_option);
	}

	pwr = power_on ? pd->local_pwr_cfg : 0;
	ret = exynos_pd_write_pmu(pd, pwr);
	if (ret)
		return ret;

	/* Wait max 1ms */
	timeout = 10;

	while ((readl_relaxed(base + 0x4) & pd->local_pwr_cfg) != pwr) {
		if (!timeout) {
			op = (power_on) ? "enable" : "disable";
			pr_err("Power domain %s %s failed\n", domain->name, op);
			return -ETIMEDOUT;
		}
		timeout--;
		cpu_relax();
		usleep_range(80, 100);
	}

	if (power_on)
		exynos_pd_secure_control(pd, true);

	return 0;
}

static int exynos_pd_power_on(struct generic_pm_domain *domain)
{
	return exynos_pd_power(domain, true);
}

static int exynos_pd_power_off(struct generic_pm_domain *domain)
{
	return exynos_pd_power(domain, false);
}

static const struct exynos_pm_domain_config exynos4210_cfg = {
	.local_pwr_cfg		= 0x7,
};

static const struct exynos_pm_domain_config exynos5433_cfg = {
	.local_pwr_cfg		= 0xf,
};

static const struct exynos_pm_domain_config zumapro_cfg = {
	.local_pwr_cfg		= BIT(0),
	.secure_pmu		= true,
};

static const struct of_device_id exynos_pm_domain_of_match[] = {
	{
		.compatible = "samsung,exynos4210-pd",
		.data = &exynos4210_cfg,
	}, {
		.compatible = "samsung,exynos5433-pd",
		.data = &exynos5433_cfg,
	}, {
		.compatible = "google,zumapro-pd",
		.data = &zumapro_cfg,
	},
	{ },
};

static const char *exynos_get_domain_name(struct device *dev,
					  struct device_node *node)
{
	const char *name;

	if (of_property_read_string(node, "label", &name) < 0)
		name = kbasename(node->full_name);
	return devm_kstrdup_const(dev, name, GFP_KERNEL);
}

static int exynos_pd_probe(struct platform_device *pdev)
{
	const struct exynos_pm_domain_config *pm_domain_cfg;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *res;
	struct of_phandle_args child, parent;
	struct exynos_pm_domain *pd;
	int on, ret;

	pm_domain_cfg = of_device_get_match_data(dev);
	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->pd.name = exynos_get_domain_name(dev, np);
	if (!pd->pd.name)
		return -ENOMEM;

	pd->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pd->base))
		return PTR_ERR(pd->base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	pd->base_addr = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cmu");
	if (res) {
		pd->cmu_option = devm_ioremap_resource(dev, res);
		if (IS_ERR(pd->cmu_option))
			return PTR_ERR(pd->cmu_option);
	}

	of_property_read_u32(np, "samsung,secure-pd-id", &pd->secure_pwr_id);

	pd->pd.power_off = exynos_pd_power_off;
	pd->pd.power_on = exynos_pd_power_on;
	pd->local_pwr_cfg = pm_domain_cfg->local_pwr_cfg;
	pd->secure_pmu = pm_domain_cfg->secure_pmu;
	if (of_property_read_bool(np, "samsung,always-on"))
		pd->pd.flags |= GENPD_FLAG_ALWAYS_ON;

	/*
	 * Some Samsung platforms with bootloaders turning on the splash-screen
	 * and handing it over to the kernel, requires the power-domains to be
	 * reset during boot.
	 */
	if (IS_ENABLED(CONFIG_ARM) &&
	    of_device_is_compatible(np, "samsung,exynos4210-pd"))
		exynos_pd_power_off(&pd->pd);

	on = readl_relaxed(pd->base + 0x4) & pd->local_pwr_cfg;

	ret = pm_genpd_init(&pd->pd, NULL, !on);
	if (ret)
		return ret;

	/*
	 * When the bootloader hands a secure power domain over already powered
	 * (e.g. the always-on G3D adopted for GPU bring-up), genpd records it as
	 * on and never calls our .power_on, so the secure-side context that
	 * callback restores via SMC is never established for this boot.  That
	 * context includes the TZPC protection setup the IP's bus master needs to
	 * reach DRAM, which the bootloader only programs for the masters it uses
	 * itself.  Establish it now for secure domains adopted in the on state.
	 */
	if (on && pd->secure_pwr_id)
		exynos_pd_secure_control(pd, true);

	ret = of_genpd_add_provider_simple(np, &pd->pd);

	if (ret == 0 && of_parse_phandle_with_args(np, "power-domains",
				      "#power-domain-cells", 0, &parent) == 0) {
		child.np = np;
		child.args_count = 0;

		if (of_genpd_add_subdomain(&parent, &child))
			pr_warn("%pOF failed to add subdomain: %pOF\n",
				parent.np, child.np);
		else
			pr_info("%pOF has as child subdomain: %pOF.\n",
				parent.np, child.np);
	}

	pm_runtime_enable(dev);
	return ret;
}

static struct platform_driver exynos_pd_driver = {
	.probe	= exynos_pd_probe,
	.driver	= {
		.name		= "exynos-pd",
		.of_match_table	= exynos_pm_domain_of_match,
		.suppress_bind_attrs = true,
	}
};

static __init int exynos4_pm_init_power_domain(void)
{
	return platform_driver_register(&exynos_pd_driver);
}
core_initcall(exynos4_pm_init_power_domain);
