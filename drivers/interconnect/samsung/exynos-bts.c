// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos Bus Traffic Shaper interconnect provider
 *
 * The IP-BTS register operations and Zumapro policy data are ported from the
 * downstream Google/Samsung GPL driver.  The integration uses the mainline
 * interconnect and common-clock frameworks instead of Exynos-private PM QoS.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/interconnect-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>

#include <dt-bindings/interconnect/google,zumapro.h>

#define BTS_CON			0x00
#define BTS_TIMEOUT_R0		0x08
#define BTS_TIMEOUT_W0		0x10
#define BTS_RCON		0x20
#define BTS_RBLK_UPPER		0x24
#define BTS_WCON		0x40
#define BTS_WBLK_UPPER		0x44

#define BTS_CON_QOS_EN		BIT(0)
#define BTS_CON_EX_QURGENT_EN	BIT(7)
#define BTS_CON_QURGENT_EN	BIT(23)
#define BTS_XCON_BLOCKING_EN	BIT(0)
#define BTS_XCON_QOS_OVERRIDE	BIT(8)
#define BTS_XCON_QOS		GENMASK(15, 12)
#define BTS_BLOCK_UPPER		GENMASK(15, 0)

#define BTS_NUM_CHANNELS	4U
#define BTS_MIF_BUS_WIDTH	16U
#define BTS_INT_BUS_WIDTH	32U
#define BTS_NOCL2A_CHANNELS	2U
#define BTS_MIF_UTIL		65U
#define BTS_INT_UTIL		65U
#define BTS_RT_UTIL		40U

/* Separate graph-wide LWIS DPM floors measured with the ISPFE vote. */
#define ZUMAPRO_ISPFE_MIF_FLOOR	1000000U
#define ZUMAPRO_ISPFE_INT_FLOOR	200000U

struct exynos_bts_profile {
	u16 rmo;
	u16 wmo;
	u8 arqos;
	u8 awqos;
	u8 qurgent_th_r;
	u8 qurgent_th_w;
	bool qurgent;
	bool ex_qurgent;
};

struct exynos_bts_block_data {
	const char *name;
	const struct exynos_bts_profile *default_profile;
	const struct exynos_bts_profile *camera_profile;
};

struct exynos_bts_block {
	const struct exynos_bts_block_data *data;
	void __iomem *base;
};

struct exynos_bts {
	struct device *dev;
	/* Serializes clock floors and the global camera scenario. */
	struct mutex lock;
	struct clk *mif_clk;
	struct clk *int_clk;
	unsigned long saved_mif_rate;
	unsigned long saved_int_rate;
	unsigned long target_mif_rate;
	unsigned long target_int_rate;
	bool rates_saved;
	bool camera_active;

	struct exynos_bts_block *blocks;
	unsigned int num_blocks;

	struct icc_provider provider;
	struct icc_onecell_data *data;
};

/* Profiles from the downstream Zumapro zuma-bts.dtsi. */
static const struct exynos_bts_profile zumapro_dpu_default = {
	.rmo = 0x60,
	.wmo = 0x20,
	.arqos = 0x8,
	.awqos = 0x8,
	.qurgent_th_r = 0x20,
	.qurgent_th_w = 0x20,
	.qurgent = true,
};

static const struct exynos_bts_profile zumapro_dpu_camera = {
	.rmo = 0x30,
	.wmo = 0x20,
	.arqos = 0x8,
	.awqos = 0x8,
	.qurgent_th_r = 0x20,
	.qurgent_th_w = 0x20,
	.qurgent = true,
};

static const struct exynos_bts_profile zumapro_ispfe0_default = {
	.rmo = 0x26,
	.wmo = 0x40,
	.arqos = 0xc,
	.awqos = 0xc,
	.qurgent_th_r = 0x20,
	.qurgent_th_w = 0x20,
	.qurgent = true,
};

static const struct exynos_bts_profile zumapro_ispfe12_default = {
	.rmo = 0xffff,
	.wmo = 0x40,
	.arqos = 0xc,
	.awqos = 0xc,
	.qurgent_th_r = 0x20,
	.qurgent_th_w = 0x20,
	.qurgent = true,
};

static const struct exynos_bts_profile zumapro_ispfe3_default = {
	.rmo = 0xffff,
	.wmo = 0x8,
	.arqos = 0xc,
	.awqos = 0xc,
	.qurgent_th_r = 0x20,
	.qurgent_th_w = 0x20,
	.qurgent = true,
};

static const struct exynos_bts_profile zumapro_g2d_default = {
	.rmo = 0x18,
	.wmo = 0x14,
	.arqos = 0x4,
	.awqos = 0x4,
	.qurgent_th_r = 0xff,
	.qurgent_th_w = 0xff,
};

static const struct exynos_bts_profile zumapro_g2d_camera = {
	.rmo = 0x8,
	.wmo = 0x8,
	.arqos = 0x4,
	.awqos = 0x4,
	.qurgent_th_r = 0xff,
	.qurgent_th_w = 0xff,
};

static const struct exynos_bts_block_data zumapro_blocks[] = {
	{
		.name = "dpu0",
		.default_profile = &zumapro_dpu_default,
		.camera_profile = &zumapro_dpu_camera,
	}, {
		.name = "dpu1",
		.default_profile = &zumapro_dpu_default,
		.camera_profile = &zumapro_dpu_camera,
	}, {
		.name = "ispfe0",
		.default_profile = &zumapro_ispfe0_default,
	}, {
		.name = "ispfe1",
		.default_profile = &zumapro_ispfe12_default,
	}, {
		.name = "ispfe2",
		.default_profile = &zumapro_ispfe12_default,
	}, {
		.name = "ispfe3",
		.default_profile = &zumapro_ispfe3_default,
	}, {
		.name = "g2d0",
		.default_profile = &zumapro_g2d_default,
		.camera_profile = &zumapro_g2d_camera,
	}, {
		.name = "g2d1",
		.default_profile = &zumapro_g2d_default,
		.camera_profile = &zumapro_g2d_camera,
	}, {
		.name = "g2d2",
		.default_profile = &zumapro_g2d_default,
		.camera_profile = &zumapro_g2d_camera,
	},
};

/* Live Android devfreq OPP ladders, in kHz, highest first. */
static const u32 zumapro_mif_rates[] = {
	3744000, 3172000, 2730000, 2288000, 2028000, 1716000, 1539000,
	1352000, 1014000, 845000, 676000, 546000, 421000,
};

static const u32 zumapro_int_rates[] = {
	1066000, 799000, 664000, 533000, 356000, 200000, 178000,
};

struct exynos_bts_bus1_map {
	u32 bus1;
	u32 int_freq;
};

static const struct exynos_bts_bus1_map zumapro_bus1_int_map[] = {
	{ 620000, 1066000 },
	{ 533000, 664000 },
	{ 400000, 533000 },
	{ 267000, 356000 },
	{ 200000, 200000 },
	{ 134000, 178000 },
};

static void exynos_bts_program_block(void __iomem *base,
				     const struct exynos_bts_profile *profile)
{
	u32 val;

	/* AxQoS override and value. */
	val = readl(base + BTS_RCON);
	val &= ~(BTS_XCON_QOS_OVERRIDE | BTS_XCON_QOS);
	val |= BTS_XCON_QOS_OVERRIDE |
	       FIELD_PREP(BTS_XCON_QOS, profile->arqos);
	writel(val, base + BTS_RCON);

	val = readl(base + BTS_WCON);
	val &= ~(BTS_XCON_QOS_OVERRIDE | BTS_XCON_QOS);
	val |= BTS_XCON_QOS_OVERRIDE |
	       FIELD_PREP(BTS_XCON_QOS, profile->awqos);
	writel(val, base + BTS_WCON);

	val = readl(base + BTS_CON);
	val |= BTS_CON_QOS_EN;
	writel(val, base + BTS_CON);

	/* Maximum outstanding read and write transactions. */
	val = readl(base + BTS_RBLK_UPPER);
	val &= ~BTS_BLOCK_UPPER;
	val |= FIELD_PREP(BTS_BLOCK_UPPER, profile->rmo);
	writel(val, base + BTS_RBLK_UPPER);

	val = readl(base + BTS_WBLK_UPPER);
	val &= ~BTS_BLOCK_UPPER;
	val |= FIELD_PREP(BTS_BLOCK_UPPER, profile->wmo);
	writel(val, base + BTS_WBLK_UPPER);

	/* The same urgent threshold applies to all four virtual channels. */
	val = profile->qurgent_th_r * 0x01010101U;
	writel(val, base + BTS_TIMEOUT_R0);
	val = profile->qurgent_th_w * 0x01010101U;
	writel(val, base + BTS_TIMEOUT_W0);

	val = readl(base + BTS_CON);
	val &= ~(BTS_CON_QURGENT_EN | BTS_CON_EX_QURGENT_EN);
	if (profile->qurgent)
		val |= BTS_CON_QURGENT_EN;
	if (profile->ex_qurgent)
		val |= BTS_CON_EX_QURGENT_EN;
	writel(val, base + BTS_CON);

	/* None of the scoped Zumapro profiles enables transaction blocking. */
	val = readl(base + BTS_RCON);
	val &= ~BTS_XCON_BLOCKING_EN;
	writel(val, base + BTS_RCON);
	val = readl(base + BTS_WCON);
	val &= ~BTS_XCON_BLOCKING_EN;
	writel(val, base + BTS_WCON);
}

static void exynos_bts_program_scenario(struct exynos_bts *bts, bool camera)
{
	unsigned int i;

	for (i = 0; i < bts->num_blocks; i++) {
		const struct exynos_bts_block_data *data = bts->blocks[i].data;
		const struct exynos_bts_profile *profile = data->default_profile;

		if (camera && data->camera_profile)
			profile = data->camera_profile;
		exynos_bts_program_block(bts->blocks[i].base, profile);
	}
}

static u32 exynos_bts_ceil_rate(const u32 *rates, size_t count, u32 floor)
{
	size_t i;

	for (i = count; i-- > 0;)
		if (floor <= rates[i])
			return rates[i];

	return rates[0];
}

static u32 exynos_bts_bus1_to_int(u32 freq)
{
	size_t i = ARRAY_SIZE(zumapro_bus1_int_map);

	while (--i > 0)
		if (freq <= zumapro_bus1_int_map[i].bus1)
			break;

	return zumapro_bus1_int_map[i].int_freq;
}

static u32 exynos_bts_mif_floor(u32 total_bw, u32 rt_bw)
{
	u64 mif_freq;
	u64 rt_freq;

	/* Preserve downstream truncation without its u32 multiplication wrap. */
	mif_freq = (u64)(total_bw / BTS_MIF_BUS_WIDTH) * 100 / BTS_MIF_UTIL;
	rt_freq = (u64)(rt_bw / BTS_MIF_BUS_WIDTH) * 100 / BTS_RT_UTIL;

	return max(mif_freq, rt_freq);
}

static u32 exynos_bts_int_floor(u32 write_bw, u32 peak_bw)
{
	u64 bus1_freq;
	u64 int_freq;
	u64 freq;

	/* ISPFE is a 32-bit write-only client on NOCL2AA. */
	bus1_freq = (u64)(peak_bw / BTS_INT_BUS_WIDTH) * 100 /
		BTS_INT_UTIL;
	freq = (write_bw / BTS_INT_BUS_WIDTH) / BTS_NOCL2A_CHANNELS;
	freq = freq * 100 / BTS_INT_UTIL;
	bus1_freq = max(bus1_freq, freq);

	/* NOCL1A sees the aggregate traffic over four channels. */
	int_freq = (write_bw / BTS_INT_BUS_WIDTH) / BTS_NUM_CHANNELS;
	int_freq = int_freq * 100 / BTS_INT_UTIL;

	return max_t(u64, int_freq, exynos_bts_bus1_to_int(bus1_freq));
}

static int exynos_bts_set_active(struct exynos_bts *bts, u32 avg_bw,
				 u32 peak_bw)
{
	unsigned long old_mif;
	unsigned long mif_rate;
	unsigned long int_rate;
	u32 floor;
	int rollback_ret;
	int ret;

	floor = exynos_bts_mif_floor(avg_bw, avg_bw);
	floor = max(floor, ZUMAPRO_ISPFE_MIF_FLOOR);
	mif_rate = (unsigned long)exynos_bts_ceil_rate(zumapro_mif_rates,
						 ARRAY_SIZE(zumapro_mif_rates),
						 floor) * 1000;
	floor = exynos_bts_int_floor(avg_bw, peak_bw);
	floor = max(floor, ZUMAPRO_ISPFE_INT_FLOOR);
	int_rate = (unsigned long)exynos_bts_ceil_rate(zumapro_int_rates,
						 ARRAY_SIZE(zumapro_int_rates),
						 floor) * 1000;

	if (!bts->rates_saved) {
		bts->saved_mif_rate = clk_get_rate(bts->mif_clk);
		bts->saved_int_rate = clk_get_rate(bts->int_clk);
		if (!bts->saved_mif_rate || !bts->saved_int_rate)
			return -EIO;
		bts->rates_saved = true;
	}
	mif_rate = max(mif_rate, bts->saved_mif_rate);
	int_rate = max(int_rate, bts->saved_int_rate);
	old_mif = bts->target_mif_rate ?: bts->saved_mif_rate;

	ret = clk_set_rate(bts->mif_clk, mif_rate);
	if (ret)
		return ret;

	ret = clk_set_rate(bts->int_clk, int_rate);
	if (ret) {
		rollback_ret = clk_set_rate(bts->mif_clk, old_mif);
		if (rollback_ret) {
			/* Retain enough state for the ICC rollback to retry. */
			bts->target_mif_rate = mif_rate;
			dev_err(bts->dev,
				"cannot roll MIF back to %lu Hz: %d\n",
				old_mif, rollback_ret);
		}
		return ret;
	}

	bts->target_mif_rate = mif_rate;
	bts->target_int_rate = int_rate;
	if (!bts->camera_active) {
		exynos_bts_program_scenario(bts, true);
		bts->camera_active = true;
	}

	dev_dbg(bts->dev, "ISPFE %u/%u KB/s -> MIF %lu INT %lu Hz\n",
		avg_bw, peak_bw, mif_rate, int_rate);

	return 0;
}

static int exynos_bts_set_idle(struct exynos_bts *bts)
{
	int rollback_ret;
	int ret;

	if (!bts->rates_saved && !bts->camera_active)
		return 0;

	/* Downstream removes the bandwidth vote before the global scenario. */
	if (bts->rates_saved) {
		ret = clk_set_rate(bts->mif_clk, bts->saved_mif_rate);
		if (ret)
			return ret;

		ret = clk_set_rate(bts->int_clk, bts->saved_int_rate);
		if (ret) {
			/* Keep the active state coherent until ICC retries/rolls back. */
			if (bts->target_mif_rate) {
				rollback_ret = clk_set_rate(bts->mif_clk,
							    bts->target_mif_rate);
				if (rollback_ret)
					dev_err(bts->dev,
						"cannot restore active MIF rate %lu Hz: %d\n",
						bts->target_mif_rate,
						rollback_ret);
			}
			return ret;
		}
	}

	if (bts->camera_active)
		exynos_bts_program_scenario(bts, false);
	bts->rates_saved = false;
	bts->camera_active = false;
	bts->saved_mif_rate = 0;
	bts->saved_int_rate = 0;
	bts->target_mif_rate = 0;
	bts->target_int_rate = 0;

	return 0;
}

static int exynos_bts_icc_set(struct icc_node *src, struct icc_node *dst)
{
	struct exynos_bts *bts = src->data;
	bool active = src->avg_bw || src->peak_bw;
	int ret;

	if (src->id != ZUMAPRO_BTS_MASTER_ISPFE ||
	    dst->id != ZUMAPRO_BTS_SLAVE_MIF)
		return 0;

	/* Downstream rejects a total/peak pair unless both are zero or nonzero. */
	if (!src->avg_bw != !src->peak_bw)
		return -EINVAL;

	mutex_lock(&bts->lock);
	if (active)
		ret = exynos_bts_set_active(bts, src->avg_bw, src->peak_bw);
	else
		ret = exynos_bts_set_idle(bts);
	mutex_unlock(&bts->lock);

	return ret;
}

static int exynos_bts_icc_get_bw(struct icc_node *node, u32 *avg, u32 *peak)
{
	*avg = 0;
	*peak = 0;

	return 0;
}

static void exynos_bts_unregister(void *data)
{
	struct exynos_bts *bts = data;
	int ret;

	icc_provider_deregister(&bts->provider);
	mutex_lock(&bts->lock);
	ret = exynos_bts_set_idle(bts);
	if (ret)
		dev_warn(bts->dev,
			 "cannot restore idle clock rates while removing: %d\n",
			 ret);
	exynos_bts_program_scenario(bts, false);
	mutex_unlock(&bts->lock);
	icc_nodes_remove(&bts->provider);
}

static int exynos_bts_register(struct exynos_bts *bts)
{
	struct icc_provider *provider = &bts->provider;
	struct icc_node *master;
	struct icc_node *slave;
	int ret;

	bts->data = devm_kzalloc(bts->dev,
				 struct_size(bts->data, nodes, 2), GFP_KERNEL);
	if (!bts->data)
		return -ENOMEM;
	bts->data->num_nodes = 2;

	provider->dev = bts->dev;
	provider->set = exynos_bts_icc_set;
	provider->get_bw = exynos_bts_icc_get_bw;
	provider->aggregate = icc_std_aggregate;
	provider->xlate = of_icc_xlate_onecell;
	provider->data = bts->data;
	icc_provider_init(provider);

	master = icc_node_create(ZUMAPRO_BTS_MASTER_ISPFE);
	if (IS_ERR(master))
		return PTR_ERR(master);
	master->name = "ispfe";
	master->data = bts;
	icc_node_add(master, provider);
	bts->data->nodes[ZUMAPRO_BTS_MASTER_ISPFE] = master;

	slave = icc_node_create(ZUMAPRO_BTS_SLAVE_MIF);
	if (IS_ERR(slave)) {
		ret = PTR_ERR(slave);
		goto err_nodes;
	}
	slave->name = "mif";
	slave->data = bts;
	icc_node_add(slave, provider);
	bts->data->nodes[ZUMAPRO_BTS_SLAVE_MIF] = slave;

	ret = icc_link_create(master, slave->id);
	if (ret)
		goto err_nodes;

	ret = icc_provider_register(provider);
	if (ret)
		goto err_nodes;

	return devm_add_action_or_reset(bts->dev, exynos_bts_unregister, bts);

err_nodes:
	icc_nodes_remove(provider);
	return ret;
}

static int exynos_bts_resume(struct device *dev)
{
	struct exynos_bts *bts = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&bts->lock);
	/* Retry an ICC rollback which failed before the system suspended. */
	if (bts->rates_saved && !bts->camera_active) {
		ret = exynos_bts_set_idle(bts);
		if (ret)
			goto unlock;
	}
	exynos_bts_program_scenario(bts, bts->camera_active);
	if (bts->camera_active) {
		ret = clk_set_rate(bts->mif_clk, bts->target_mif_rate);
		if (!ret)
			ret = clk_set_rate(bts->int_clk, bts->target_int_rate);
	}

unlock:
	mutex_unlock(&bts->lock);

	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(exynos_bts_pm_ops, NULL, exynos_bts_resume);

static int exynos_bts_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_bts *bts;
	unsigned int i;
	int ret;

	bts = devm_kzalloc(dev, sizeof(*bts), GFP_KERNEL);
	if (!bts)
		return -ENOMEM;

	bts->dev = dev;
	bts->num_blocks = ARRAY_SIZE(zumapro_blocks);
	bts->blocks = devm_kcalloc(dev, bts->num_blocks,
				   sizeof(*bts->blocks), GFP_KERNEL);
	if (!bts->blocks)
		return -ENOMEM;

	mutex_init(&bts->lock);
	platform_set_drvdata(pdev, bts);

	for (i = 0; i < bts->num_blocks; i++) {
		const char *name = zumapro_blocks[i].name;

		bts->blocks[i].data = &zumapro_blocks[i];
		bts->blocks[i].base =
			devm_platform_ioremap_resource_byname(pdev, name);
		if (IS_ERR(bts->blocks[i].base))
			return dev_err_probe(dev, PTR_ERR(bts->blocks[i].base),
					     "cannot map %s\n", name);
	}

	bts->mif_clk = devm_clk_get(dev, "mif");
	if (IS_ERR(bts->mif_clk))
		return dev_err_probe(dev, PTR_ERR(bts->mif_clk),
				     "cannot get MIF clock\n");
	bts->int_clk = devm_clk_get(dev, "int");
	if (IS_ERR(bts->int_clk))
		return dev_err_probe(dev, PTR_ERR(bts->int_clk),
				     "cannot get INT clock\n");

	ret = exynos_bts_register(bts);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register interconnect\n");

	/* Downstream installs the default profile at BTS probe. */
	exynos_bts_program_scenario(bts, false);

	return 0;
}

static const struct of_device_id exynos_bts_of_match[] = {
	{ .compatible = "google,zumapro-bts" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_bts_of_match);

static struct platform_driver exynos_bts_driver = {
	.probe = exynos_bts_probe,
	.driver = {
		.name = "exynos-bts",
		.of_match_table = exynos_bts_of_match,
		.pm = pm_sleep_ptr(&exynos_bts_pm_ops),
		.sync_state = icc_sync_state,
	},
};
module_platform_driver(exynos_bts_driver);

MODULE_DESCRIPTION("Samsung Exynos Bus Traffic Shaper interconnect driver");
MODULE_LICENSE("GPL");
