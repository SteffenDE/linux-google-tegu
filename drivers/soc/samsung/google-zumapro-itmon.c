// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Google Tensor G4 interconnect transaction monitor
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 * Copyright 2026 Steffen Deusch
 *
 * This is a deliberately small diagnostic port of downstream's Zumapro ITMON
 * driver.  It programs the same node masks, protocol checkers and response
 * timeouts, but leaves the vendor debug-snapshot policy framework behind.
 * The first error is dumped into the printk ring and panics so ramoops can
 * preserve the report across a warm reset.
 */

#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/panic.h>
#include <linux/panic_notifier.h>
#include <linux/platform_device.h>
#include <linux/syscore_ops.h>

#define ITMON_DBG_CTL			0x1800
#define ITMON_ERR_LOG_STAT		0x1804
#define ITMON_ERR_LOG_POP		0x1808
#define ITMON_ERR_LOG_CLR		0x180c
#define ITMON_ERR_LOG_EN_NODE		0x1810
#define ITMON_ERR_LOG_INFO(n)		(0x1840 + ((n) * 4))
#define ITMON_TMOUT_CTL(n)		(0x1900 + ((n) * 4))
#define ITMON_PRTCHK_M_CTL(n)		(0x1c00 + ((n) * 4))
#define ITMON_PRTCHK_S_CTL(n)		(0x1d00 + ((n) * 4))

#define ITMON_DBG_INT_EN		BIT(0)
#define ITMON_DBG_ERR_LOG_EN		BIT(1)
#define ITMON_DBG_TMOUT_EN		BIT(2)
#define ITMON_DBG_PRTCHK_EN		BIT(3)
#define ITMON_DBG_FIXED_DET_EN		BIT(5)
#define ITMON_DBG_ENABLE		(ITMON_DBG_INT_EN | ITMON_DBG_ERR_LOG_EN | \
					 ITMON_DBG_TMOUT_EN | ITMON_DBG_PRTCHK_EN | \
					 ITMON_DBG_FIXED_DET_EN)

#define ITMON_PRTCHK_ENABLE		GENMASK(3, 0)
#define ITMON_TMOUT_ENABLE		BIT(0)
#define ITMON_TMOUT_VALUE		0xfffff
#define ITMON_MAX_LOGS			64
#define ITMON_NUM_GROUPS		10

#define ITMON_INFO_ERR_CODE(v)		((v) & GENMASK(3, 0))
#define ITMON_INFO_WRITE(v)		((v) & BIT(12))
#define ITMON_INFO_DETECTOR(v)		(((v) >> 16) & 0xff)
#define ITMON_INFO_MASTER(v)		(((v) >> 24) & 0xff)

struct zumapro_itmon_group_cfg {
	const char *name;
	u32 err_mask;
	u32 m_prt_mask;
	u32 s_prt_mask;
	u32 tmout_mask;
};

struct zumapro_itmon;

struct zumapro_itmon_group {
	struct zumapro_itmon *itmon;
	const struct zumapro_itmon_group_cfg *cfg;
	void __iomem *base;
	int irq;
};

struct zumapro_itmon {
	struct device *dev;
	struct zumapro_itmon_group groups[ITMON_NUM_GROUPS];
	struct notifier_block panic_nb;
	struct syscore syscore;
	atomic_t triggered;
};

/*
 * Source: downstream zuma-itmon.c.  Node IDs and checker/timeout offsets are
 * contiguous within each mask.
 */
static const struct zumapro_itmon_group_cfg zumapro_itmon_groups[] = {
	{ "nocl1a-data",   GENMASK(17, 0), GENMASK(13, 0), GENMASK(3, 0), 0 },
	{ "nocl1b-data",   GENMASK(6, 0),  GENMASK(5, 0),  BIT(0), 0 },
	{ "nocl2aa-data",  GENMASK(17, 0), GENMASK(15, 0), GENMASK(1, 0), 0 },
	{ "nocl2ab-data",  GENMASK(18, 0), GENMASK(16, 0), GENMASK(1, 0), 0 },
	{ "nocl0-data",    GENMASK(15, 0), GENMASK(8, 0),
						      GENMASK(6, 0), 0 },
	{ "nocl1a-config", GENMASK(5, 0),  BIT(0), GENMASK(4, 0),
						      GENMASK(4, 0) },
	{ "nocl1b-config", GENMASK(5, 0),  BIT(0), GENMASK(4, 0),
						      GENMASK(4, 0) },
	{ "nocl2aa-config", GENMASK(8, 0), BIT(0), GENMASK(7, 0),
						      GENMASK(7, 0) },
	{ "nocl2ab-config", GENMASK(7, 0), BIT(0), GENMASK(6, 0),
						      GENMASK(6, 0) },
	/* SLC is timeout slot 12 but protocol-checker slot 16. */
	{ "nocl0-config",  GENMASK(19, 0), GENMASK(2, 0), GENMASK(16, 0),
						      GENMASK(12, 0) },
};

static const char *zumapro_itmon_err_name(u32 code)
{
	switch (code) {
	case 0:
		return "slave error";
	case 1:
		return "decode error";
	case 2:
		return "unsupported transaction";
	case 3:
		return "powered-down target";
	case 4:
		return "intended-access violation";
	case 6:
		return "response timeout";
	case 8 ... 13:
		return "protocol error";
	case 15:
		return "timeout freeze";
	default:
		return "unknown error";
	}
}

static void zumapro_itmon_write_mask(void __iomem *base, u32 offset,
				     u32 mask, u32 value)
{
	unsigned int bit;

	for (bit = 0; bit < 32; bit++)
		if (mask & BIT(bit))
			writel(value, base + offset + bit * 4);
}

static void zumapro_itmon_init_group(struct zumapro_itmon_group *group,
				     bool clear, bool enable)
{
	const struct zumapro_itmon_group_cfg *cfg = group->cfg;
	void __iomem *base = group->base;

	if (clear) {
		writel(0, base + ITMON_DBG_CTL);
		writel(1, base + ITMON_ERR_LOG_CLR);
	}

	writel(cfg->err_mask, base + ITMON_ERR_LOG_EN_NODE);
	zumapro_itmon_write_mask(base, ITMON_PRTCHK_M_CTL(0), cfg->m_prt_mask,
				 ITMON_PRTCHK_ENABLE);
	zumapro_itmon_write_mask(base, ITMON_PRTCHK_S_CTL(0), cfg->s_prt_mask,
				 ITMON_PRTCHK_ENABLE);
	zumapro_itmon_write_mask(base, ITMON_TMOUT_CTL(0), cfg->tmout_mask,
				 (ITMON_TMOUT_VALUE << 4) | ITMON_TMOUT_ENABLE);
	writel(enable ? ITMON_DBG_ENABLE : 0, base + ITMON_DBG_CTL);
	readl(base + ITMON_DBG_CTL);
}

static void zumapro_itmon_init(struct zumapro_itmon *itmon, bool clear,
			       bool enable)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(itmon->groups); i++)
		zumapro_itmon_init_group(&itmon->groups[i], clear, enable);
}

static bool zumapro_itmon_dump_group(struct zumapro_itmon_group *group,
				     bool pop)
{
	struct device *dev = group->itmon->dev;
	void __iomem *base = group->base;
	bool found = false;
	u32 info[7];
	u32 stat;
	unsigned int i, log;
	u64 addr;

	for (log = 0; log < ITMON_MAX_LOGS; log++) {
		stat = readl(base + ITMON_ERR_LOG_STAT);
		if (!stat)
			break;

		for (i = 0; i < ARRAY_SIZE(info); i++)
			info[i] = readl(base + ITMON_ERR_LOG_INFO(i));

		addr = ((u64)(info[3] & GENMASK(15, 0)) << 32) | info[2];
		dev_emerg(dev,
			  "%s: %s, %s, master %u, detector %u, addr %#llx\n",
			  group->cfg->name,
			  ITMON_INFO_WRITE(info[0]) ? "write" : "read",
			  zumapro_itmon_err_name(ITMON_INFO_ERR_CODE(info[0])),
			  ITMON_INFO_MASTER(info[0]),
			  ITMON_INFO_DETECTOR(info[0]), addr);
		dev_emerg(dev,
			  "%s: raw %08x %08x %08x %08x %08x %08x %08x, enabled %08x\n",
			  group->cfg->name, info[0], info[1], info[2], info[3],
			  info[4], info[5], info[6],
			  readl(base + ITMON_ERR_LOG_EN_NODE));
		found = true;

		if (!pop)
			break;
		writel(1, base + ITMON_ERR_LOG_POP);
	}

	if (pop && found)
		writel(1, base + ITMON_ERR_LOG_CLR);

	return found;
}

static bool zumapro_itmon_dump(struct zumapro_itmon *itmon, bool pop)
{
	bool found = false;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(itmon->groups); i++)
		found |= zumapro_itmon_dump_group(&itmon->groups[i], pop);

	return found;
}

static irqreturn_t zumapro_itmon_irq(int irq, void *data)
{
	struct zumapro_itmon_group *group = data;
	struct zumapro_itmon *itmon = group->itmon;
	bool found;

	if (atomic_xchg(&itmon->triggered, 1))
		return IRQ_HANDLED;

	dev_emerg(itmon->dev, "ITMON interrupt %d (%s)\n", irq,
		  group->cfg->name);
	found = zumapro_itmon_dump(itmon, true);
	panic("Zumapro ITMON %s%s", group->cfg->name,
	      found ? " interconnect error" : " interrupt without an error log");
}

static int zumapro_itmon_panic(struct notifier_block *nb,
			       unsigned long action, void *data)
{
	struct zumapro_itmon *itmon =
		container_of(nb, struct zumapro_itmon, panic_nb);

	if (!atomic_read(&itmon->triggered))
		zumapro_itmon_dump(itmon, false);

	return NOTIFY_DONE;
}

static void zumapro_itmon_syscore_resume(void *data)
{
	struct zumapro_itmon *itmon = data;

	/* The NoC monitor registers lose state across SYSTEM_SUSPEND. */
	zumapro_itmon_init(itmon, false, true);
}

static const struct syscore_ops zumapro_itmon_syscore_ops = {
	.resume = zumapro_itmon_syscore_resume,
};

static int zumapro_itmon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zumapro_itmon *itmon;
	unsigned int i;
	int ret;

	itmon = devm_kzalloc(dev, sizeof(*itmon), GFP_KERNEL);
	if (!itmon)
		return -ENOMEM;

	itmon->dev = dev;
	atomic_set(&itmon->triggered, 0);

	for (i = 0; i < ARRAY_SIZE(itmon->groups); i++) {
		struct zumapro_itmon_group *group = &itmon->groups[i];

		group->itmon = itmon;
		group->cfg = &zumapro_itmon_groups[i];
		group->base = devm_platform_ioremap_resource_byname(pdev,
								    group->cfg->name);
		if (IS_ERR(group->base))
			return PTR_ERR(group->base);

		group->irq = platform_get_irq_byname(pdev, group->cfg->name);
		if (group->irq < 0)
			return group->irq;
	}

	/* Clear stale boot-firmware state before any interrupt is unmasked. */
	zumapro_itmon_init(itmon, true, false);

	for (i = 0; i < ARRAY_SIZE(itmon->groups); i++) {
		struct zumapro_itmon_group *group = &itmon->groups[i];

		ret = devm_request_irq(dev, group->irq, zumapro_itmon_irq,
				       IRQF_NOBALANCING, group->cfg->name, group);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request %s IRQ\n",
					     group->cfg->name);
		irq_set_affinity_hint(group->irq, cpu_online_mask);
	}

	itmon->panic_nb.notifier_call = zumapro_itmon_panic;
	ret = atomic_notifier_chain_register(&panic_notifier_list,
					     &itmon->panic_nb);
	if (ret)
		return ret;

	itmon->syscore.ops = &zumapro_itmon_syscore_ops;
	itmon->syscore.data = itmon;
	register_syscore(&itmon->syscore);
	platform_set_drvdata(pdev, itmon);
	zumapro_itmon_init(itmon, false, true);

	dev_info(dev, "enabled ten NoC transaction monitors\n");
	return 0;
}

static const struct of_device_id zumapro_itmon_of_match[] = {
	{ .compatible = "google,zumapro-itmon" },
	{ }
};
MODULE_DEVICE_TABLE(of, zumapro_itmon_of_match);

static struct platform_driver zumapro_itmon_driver = {
	.probe = zumapro_itmon_probe,
	.driver = {
		.name = "google-zumapro-itmon",
		.of_match_table = zumapro_itmon_of_match,
		.suppress_bind_attrs = true,
	},
};

static int __init zumapro_itmon_driver_init(void)
{
	return platform_driver_register(&zumapro_itmon_driver);
}
/* Let the debug UART register its syscore restore before ITMON is rearmed. */
late_initcall(zumapro_itmon_driver_init);

MODULE_DESCRIPTION("Google Tensor G4 interconnect transaction monitor");
MODULE_LICENSE("GPL");
