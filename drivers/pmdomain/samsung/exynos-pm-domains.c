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
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>

struct exynos_pm_domain_config {
	/* Value for LOCAL_PWR_CFG and STATUS fields for each domain */
	u32 local_pwr_cfg;
	bool secure_pmu;
	/* How long STATUS is given to follow CONFIGURATION */
	unsigned int wait_us;
	const struct exynos_pm_domain_restore *restore;
};

struct exynos_pm_domain_restore {
	u32 divider_offset;
	u32 mux_offsets[3];
	u32 num_muxes;
	u32 qch_first;
	u32 qch_last;
	u32 qch_hole;
	u32 bus_drcg;
	u32 bus_drcg1_offset;
	u32 bus_drcg1;
	u32 bus_memclk_offset;
	u32 num_s2mpus;
};

/*
 * Exynos specific wrapper around the generic power domain
 */
struct exynos_pm_domain {
	void __iomem *base;
	void __iomem *cmu;
	void __iomem *cmu_option;
	void __iomem *bus;
	void __iomem *s2mpu[3];
	phys_addr_t base_addr;
	struct generic_pm_domain pd;
	const struct exynos_pm_domain_restore *restore;
	u32 local_pwr_cfg;
	u32 secure_pwr_id;
	bool secure_pmu;
	unsigned int wait_us;
};

#define EXYNOS_PD_SMC_CMD		0x82000410
#define EXYNOS_PD_SMC_SAVE		0
#define EXYNOS_PD_SMC_RESTORE		1
#define EXYNOS_PD_SMC_TZPC_GROUP	2
#define EXYNOS_PRIV_REG_SMC_CMD		0x82000504
#define EXYNOS_PRIV_REG_WRITE		1
#define EXYNOS_PD_CMU_RESET_DISABLE	BIT(24)
#define EXYNOS_PD_CMU_OPTION_ON		0xf11ff03f
#define EXYNOS_PD_CMU_DIVIDER_ON		0x1
#define EXYNOS_PD_CMU_MUX_ON		0x10
#define EXYNOS_PD_CMU_QCH_ON		0x2
#define EXYNOS_PD_BUS_DRCG_EN		0x104
#define EXYNOS_PD_S2MPU_PROT_CLR	0x54
#define EXYNOS_PD_S2MPU_OPEN		0xff
#define EXYNOS_PD_STATUS		0x4
#define EXYNOS_PD_POLL_US		100

static void exynos_pd_restore(struct exynos_pm_domain *pd)
{
	const struct exynos_pm_domain_restore *restore = pd->restore;
	u32 i, off;

	if (!restore)
		return;

	writel(EXYNOS_PD_CMU_DIVIDER_ON,
	       pd->cmu + restore->divider_offset);
	for (i = 0; i < restore->num_muxes; i++)
		writel(EXYNOS_PD_CMU_MUX_ON, pd->cmu + restore->mux_offsets[i]);

	for (off = restore->qch_first; off <= restore->qch_last; off += 4)
		if (off != restore->qch_hole)
			writel(EXYNOS_PD_CMU_QCH_ON, pd->cmu + off);

	writel(EXYNOS_PD_CMU_OPTION_ON, pd->cmu_option);
	writel(restore->bus_drcg, pd->bus + EXYNOS_PD_BUS_DRCG_EN);
	if (restore->bus_drcg1_offset)
		writel(restore->bus_drcg1,
		       pd->bus + restore->bus_drcg1_offset);
	writel(0, pd->bus + restore->bus_memclk_offset);

	for (i = 0; i < restore->num_s2mpus; i++)
		writel(EXYNOS_PD_S2MPU_OPEN,
		       pd->s2mpu[i] + EXYNOS_PD_S2MPU_PROT_CLR);

	/* Complete every restore before a domain consumer is allowed to resume. */
	readl(pd->cmu_option);
}

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
	u32 status, pwr;
	ktime_t start;
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

	start = ktime_get();
	ret = readl_poll_timeout(base + EXYNOS_PD_STATUS, status,
				 (status & pd->local_pwr_cfg) == pwr,
				 EXYNOS_PD_POLL_US, pd->wait_us);
	if (ret) {
		op = (power_on) ? "enable" : "disable";
		/*
		 * CONFIGURATION as well as STATUS: on a secure-PMU domain the
		 * write went through an SMC that reports success without
		 * proving anything landed, and "the PMU never accepted the
		 * command" and "the block will not quiesce" are different
		 * faults with the same symptom.  Both registers are in PMU
		 * ALIVE, so reading them is safe whatever the block is doing.
		 */
		pr_err("Power domain %s %s failed after %lld us, config %#010x status %#010x\n",
		       domain->name, op, ktime_us_delta(ktime_get(), start),
		       readl_relaxed(base), status);
		return ret;
	}

	if (power_on)
		exynos_pd_secure_control(pd, true);
	if (power_on)
		exynos_pd_restore(pd);

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
	.wait_us		= 1000,
};

static const struct exynos_pm_domain_config exynos5433_cfg = {
	.local_pwr_cfg		= 0xf,
	.wait_us		= 1000,
};

/*
 * Five times the budget the others get, because that is what the vendor's own
 * PMU sequencer allows on this SoC: pmucal_rae_wait() polls at 1 us and gives
 * up after 5000 of them, and BLK_ISPFE has been seen to need 520 us of that
 * just to power on.  1 ms leaves nothing over for a block that has been
 * streaming, and the failure is not a clean -ETIMEDOUT: an aborted transition
 * takes ACPM down with it.
 */
static const struct exynos_pm_domain_config zumapro_cfg = {
	.local_pwr_cfg		= BIT(0),
	.secure_pmu		= true,
	.wait_us		= 5000,
};

/*
 * These are the values restored by downstream PMUCAL on every BLK_GDC,
 * BLK_RGBP, BLK_TNR, BLK_MCSC and BLK_YUVP power-up, corroborated by the Pixel
 * 9a camera startup trace.  The Q-channel ranges include the System MMUs, so
 * the restore belongs here in genpd: an IOMMU resumes before the camera device
 * whose DMA it serves.
 */
static const struct exynos_pm_domain_restore zumapro_gdc_restore = {
	.divider_offset	= 0x1800,
	.mux_offsets	= { 0x600, 0x610, 0x620 },
	.num_muxes	= 3,
	.qch_first	= 0x3034,
	.qch_last	= 0x30e8,
	.qch_hole	= U32_MAX,
	.bus_drcg	= 0xffffffff,
	.bus_drcg1_offset = 0x400,
	.bus_drcg1	= 0x00000001,
	.bus_memclk_offset = 0x10c,
	.num_s2mpus	= 1,
};

static const struct exynos_pm_domain_restore zumapro_rgbp_restore = {
	.divider_offset	= 0x1800,
	.mux_offsets	= { 0x600, 0x610 },
	.num_muxes	= 2,
	.qch_first	= 0x3030,
	.qch_last	= 0x311c,
	.qch_hole	= 0x3118,
	.bus_drcg	= 0x07ffffff,
	.bus_drcg1_offset = 0x400,
	.bus_drcg1	= 0x007fffff,
	.bus_memclk_offset = 0x108,
	.num_s2mpus	= 2,
};

static const struct exynos_pm_domain_restore zumapro_yuvp_restore = {
	.divider_offset	= 0x1800,
	.mux_offsets	= { 0x600 },
	.num_muxes	= 1,
	.qch_first	= 0x3018,
	.qch_last	= 0x3070,
	.qch_hole	= U32_MAX,
	.bus_drcg	= 0x0000ffff,
	.bus_memclk_offset = 0x108,
	.num_s2mpus	= 1,
};

static const struct exynos_pm_domain_restore zumapro_tnr_restore = {
	.divider_offset	= 0x1800,
	.mux_offsets	= { 0x600, 0x610 },
	.num_muxes	= 2,
	.qch_first	= 0x3028,
	.qch_last	= 0x311c,
	.qch_hole	= U32_MAX,
	.bus_drcg	= 0xffffffff,
	.bus_drcg1_offset = 0x10c,
	.bus_drcg1	= 0xffffffff,
	.bus_memclk_offset = 0x108,
	.num_s2mpus	= 3,
};

static const struct exynos_pm_domain_restore zumapro_mcsc_restore = {
	.divider_offset	= 0x1800,
	.mux_offsets	= { 0x610 },
	.num_muxes	= 1,
	.qch_first	= 0x3014,
	.qch_last	= 0x309c,
	.qch_hole	= U32_MAX,
	.bus_drcg	= 0x03ffffff,
	.bus_memclk_offset = 0x108,
	.num_s2mpus	= 1,
};

static const struct exynos_pm_domain_config zumapro_gdc_cfg = {
	.local_pwr_cfg	= BIT(0),
	.secure_pmu	= true,
	.wait_us	= 5000,
	.restore	= &zumapro_gdc_restore,
};

static const struct exynos_pm_domain_config zumapro_rgbp_cfg = {
	.local_pwr_cfg	= BIT(0),
	.secure_pmu	= true,
	.wait_us	= 5000,
	.restore	= &zumapro_rgbp_restore,
};

static const struct exynos_pm_domain_config zumapro_yuvp_cfg = {
	.local_pwr_cfg	= BIT(0),
	.secure_pmu	= true,
	.wait_us	= 5000,
	.restore	= &zumapro_yuvp_restore,
};

static const struct exynos_pm_domain_config zumapro_tnr_cfg = {
	.local_pwr_cfg	= BIT(0),
	.secure_pmu	= true,
	.wait_us	= 5000,
	.restore	= &zumapro_tnr_restore,
};

static const struct exynos_pm_domain_config zumapro_mcsc_cfg = {
	.local_pwr_cfg	= BIT(0),
	.secure_pmu	= true,
	.wait_us	= 5000,
	.restore	= &zumapro_mcsc_restore,
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
	}, {
		.compatible = "google,zumapro-gdc-pd",
		.data = &zumapro_gdc_cfg,
	}, {
		.compatible = "google,zumapro-rgbp-pd",
		.data = &zumapro_rgbp_cfg,
	}, {
		.compatible = "google,zumapro-yuvp-pd",
		.data = &zumapro_yuvp_cfg,
	}, {
		.compatible = "google,zumapro-tnr-pd",
		.data = &zumapro_tnr_cfg,
	}, {
		.compatible = "google,zumapro-mcsc-pd",
		.data = &zumapro_mcsc_cfg,
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
	unsigned int i;
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
	pd->restore = pm_domain_cfg->restore;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cmu");
	if (res) {
		pd->cmu = devm_ioremap_resource(dev, res);
		if (IS_ERR(pd->cmu))
			return PTR_ERR(pd->cmu);
		pd->cmu_option = pd->cmu;
		if (pd->restore)
			pd->cmu_option += 0x800;
	}

	if (pd->restore) {
		if (!pd->cmu)
			return -EINVAL;

		pd->bus = devm_platform_ioremap_resource_byname(pdev, "bus");
		if (IS_ERR(pd->bus))
			return PTR_ERR(pd->bus);

		for (i = 0; i < pd->restore->num_s2mpus; i++) {
			char name[] = "s2mpu0";

			name[5] += i;
			pd->s2mpu[i] =
				devm_platform_ioremap_resource_byname(pdev, name);
			if (IS_ERR(pd->s2mpu[i]))
				return PTR_ERR(pd->s2mpu[i]);
		}
	}

	of_property_read_u32(np, "samsung,secure-pd-id", &pd->secure_pwr_id);

	pd->pd.power_off = exynos_pd_power_off;
	pd->pd.power_on = exynos_pd_power_on;
	pd->local_pwr_cfg = pm_domain_cfg->local_pwr_cfg;
	pd->secure_pmu = pm_domain_cfg->secure_pmu;
	/* readl_poll_timeout() never gives up on a zero timeout. */
	pd->wait_us = pm_domain_cfg->wait_us ?: 1000;
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
	if (on)
		exynos_pd_restore(pd);

	ret = pm_genpd_init(&pd->pd, NULL, !on);
	if (ret)
		return ret;

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
