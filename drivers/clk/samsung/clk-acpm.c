// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos ACPM protocol based clock driver.
 *
 * Copyright 2025 Linaro Ltd.
 */

#include <linux/array_size.h>
#include <linux/clk-provider.h>
#include <linux/container_of.h>
#include <linux/device/devres.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>

struct acpm_clk {
	u32 id;
	struct clk_hw hw;
	unsigned int mbox_chan_id;
	struct acpm_handle *handle;
};

struct acpm_clk_variant {
	const char *name;
};

struct acpm_clk_driver_data {
	const struct acpm_clk_variant *clks;
	unsigned int nr_clks;
	unsigned int mbox_chan_id;
};

#define to_acpm_clk(clk) container_of(clk, struct acpm_clk, hw)

#define ACPM_CLK(cname)					\
	{						\
		.name		= cname,		\
	}

static const struct acpm_clk_variant gs101_acpm_clks[] = {
	ACPM_CLK("mif"),
	ACPM_CLK("int"),
	ACPM_CLK("cpucl0"),
	ACPM_CLK("cpucl1"),
	ACPM_CLK("cpucl2"),
	ACPM_CLK("g3d"),
	ACPM_CLK("g3dl2"),
	ACPM_CLK("tpu"),
	ACPM_CLK("intcam"),
	ACPM_CLK("tnr"),
	ACPM_CLK("cam"),
	ACPM_CLK("mfc"),
	ACPM_CLK("disp"),
	ACPM_CLK("bo"),
};

static const struct acpm_clk_driver_data acpm_clk_gs101 = {
	.clks = gs101_acpm_clks,
	.nr_clks = ARRAY_SIZE(gs101_acpm_clks),
	.mbox_chan_id = 0,
};

/*
 * Zuma/Zumapro (Tensor G4) ACPM DVFS domains, in the firmware enum order from
 * the downstream cal-if (drivers/soc/google/cal-if/zuma/acpm_dvfs_zuma.h):
 * MIF, INT, CPUCL0..2, then DSU and BCI, then G3D and G3DL2.  The DSU/BCI
 * insertion is why the GPU domains sit at indices 7/8 here, versus 5/6 on
 * gs101 -- the GPU cal_ids (0xb040007 = G3D, 0xb040008 = G3DL2) carry those
 * indices in their low 16 bits (cmucal MASK_OF_ID), and the driver passes the
 * array index straight to ACPM as the DVFS domain id, so the array order must
 * match the firmware enum exactly.  Only the prefix through G3DL2 is listed:
 * everything up to here is needed to keep index == domain-id for the GPU, and
 * later domains (TPU, CAM, ...) can be appended when a consumer lands.
 */
static const struct acpm_clk_variant zumapro_acpm_clks[] = {
	ACPM_CLK("mif"),	/* 0 */
	ACPM_CLK("int"),	/* 1 */
	ACPM_CLK("cpucl0"),	/* 2 */
	ACPM_CLK("cpucl1"),	/* 3 */
	ACPM_CLK("cpucl2"),	/* 4 */
	ACPM_CLK("dsu"),	/* 5 */
	ACPM_CLK("bci"),	/* 6 */
	ACPM_CLK("g3d"),	/* 7 */
	ACPM_CLK("g3dl2"),	/* 8 */
};

static const struct acpm_clk_driver_data acpm_clk_zumapro = {
	.clks = zumapro_acpm_clks,
	.nr_clks = ARRAY_SIZE(zumapro_acpm_clks),
	/*
	 * DVFS requests go on ACPM mailbox channel 1 on zumapro (channel 0 is
	 * framework/control).  Confirmed by the on-silicon channel dump
	 * (research/tracing/acpm-channels.md) and the downstream
	 * samsung,exynos-dvfs-manager acpm-ipc-channel = <1>.
	 */
	.mbox_chan_id = 1,
};

static unsigned long acpm_clk_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	struct acpm_clk *clk = to_acpm_clk(hw);

	return clk->handle->ops.dvfs_ops.get_rate(clk->handle,
					clk->mbox_chan_id, clk->id);
}

static int acpm_clk_determine_rate(struct clk_hw *hw,
				   struct clk_rate_request *req)
{
	/*
	 * We can't figure out what rate it will be, so just return the
	 * rate back to the caller. acpm_clk_recalc_rate() will be called
	 * after the rate is set and we'll know what rate the clock is
	 * running at then.
	 */
	return 0;
}

static int acpm_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			     unsigned long parent_rate)
{
	struct acpm_clk *clk = to_acpm_clk(hw);

	return clk->handle->ops.dvfs_ops.set_rate(clk->handle,
					clk->mbox_chan_id, clk->id, rate);
}

static const struct clk_ops acpm_clk_ops = {
	.recalc_rate = acpm_clk_recalc_rate,
	.determine_rate = acpm_clk_determine_rate,
	.set_rate = acpm_clk_set_rate,
};

static int acpm_clk_register(struct device *dev, struct acpm_clk *aclk,
			     const char *name)
{
	struct clk_init_data init = {};

	init.name = name;
	init.ops = &acpm_clk_ops;
	aclk->hw.init = &init;

	return devm_clk_hw_register(dev, &aclk->hw);
}

static int acpm_clk_probe(struct platform_device *pdev)
{
	const struct acpm_clk_driver_data *data;
	struct acpm_handle *acpm_handle;
	struct clk_hw_onecell_data *clk_data;
	struct clk_hw **hws;
	struct device *dev = &pdev->dev;
	struct acpm_clk *aclks;
	unsigned int mbox_chan_id;
	int i, err, count;

	data = (const struct acpm_clk_driver_data *)
		platform_get_device_id(pdev)->driver_data;
	if (!data)
		return -ENODEV;

	acpm_handle = devm_acpm_get_by_node(dev, dev->parent->of_node);
	if (IS_ERR(acpm_handle))
		return dev_err_probe(dev, PTR_ERR(acpm_handle),
				     "Failed to get acpm handle\n");

	count = data->nr_clks;
	mbox_chan_id = data->mbox_chan_id;

	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws, count),
				GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->num = count;
	hws = clk_data->hws;

	aclks = devm_kcalloc(dev, count, sizeof(*aclks), GFP_KERNEL);
	if (!aclks)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		struct acpm_clk *aclk = &aclks[i];

		/*
		 * The code assumes the clock IDs start from zero,
		 * are sequential and do not have gaps.
		 */
		aclk->id = i;
		aclk->handle = acpm_handle;
		aclk->mbox_chan_id = mbox_chan_id;

		hws[i] = &aclk->hw;

		err = acpm_clk_register(dev, aclk, data->clks[i].name);
		if (err)
			return dev_err_probe(dev, err,
					     "Failed to register clock\n");
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					   clk_data);
}

static const struct platform_device_id acpm_clk_id[] = {
	{ "gs101-acpm-clk", (kernel_ulong_t)&acpm_clk_gs101 },
	{ "zumapro-acpm-clk", (kernel_ulong_t)&acpm_clk_zumapro },
	{}
};
MODULE_DEVICE_TABLE(platform, acpm_clk_id);

static struct platform_driver acpm_clk_driver = {
	.driver	= {
		.name = "acpm-clocks",
	},
	.probe = acpm_clk_probe,
	.id_table = acpm_clk_id,
};
module_platform_driver(acpm_clk_driver);

MODULE_AUTHOR("Tudor Ambarus <tudor.ambarus@linaro.org>");
MODULE_DESCRIPTION("Samsung Exynos ACPM clock driver");
MODULE_LICENSE("GPL");
