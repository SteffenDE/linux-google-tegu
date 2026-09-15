// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 Samsung Electronics Co., Ltd.
 * Copyright 2020 Google LLC.
 * Copyright 2024 Linaro Ltd.
 */

#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox/exynos-message.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define EXYNOS_MBOX_INTMR0		0x28	/* Interrupt Mask Register 0 */
#define EXYNOS_MBOX_INTSR0		0x2c	/* Interrupt Status Register 0 */
#define EXYNOS_MBOX_INTMSR0		0x30	/* Masked Interrupt Status 0 */
#define EXYNOS_MBOX_INTGR1		0x40	/* Interrupt Generation Register 1 */
#define EXYNOS_MBOX_INTMR1		0x48	/* Interrupt Mask Register 1 */
#define EXYNOS_MBOX_INTSR1		0x4c	/* Interrupt Status Register 1 */
#define EXYNOS_MBOX_INTMSR1		0x50	/* Masked Interrupt Status 1 */

#define EXYNOS_MBOX_INTMR0_MASK		GENMASK(15, 0)
#define EXYNOS_MBOX_INTGR1_MASK		GENMASK(15, 0)

#define EXYNOS_MBOX_CHAN_COUNT		HWEIGHT32(EXYNOS_MBOX_INTGR1_MASK)

/**
 * struct exynos_mbox - driver's private data.
 * @regs:	mailbox registers base address.
 * @mbox:	pointer to the mailbox controller.
 */
struct exynos_mbox {
	void __iomem *regs;
	struct mbox_controller *mbox;
};

/**
 * exynos_mbox_read_regs() - snapshot read-only mailbox state
 * @chan: mailbox channel whose controller should be sampled
 * @snapshot: caller-owned destination
 *
 * Return: true when all registers were sampled, false for an invalid channel.
 */
bool exynos_mbox_read_regs(struct mbox_chan *chan,
			   struct exynos_mbox_regs *snapshot)
{
	struct exynos_mbox *exynos_mbox;
	void __iomem *regs;

	if (!chan || !chan->mbox || !chan->mbox->dev || !snapshot)
		return false;

	exynos_mbox = dev_get_drvdata(chan->mbox->dev);
	if (!exynos_mbox)
		return false;

	regs = exynos_mbox->regs;
	snapshot->intsr0 = readl(regs + EXYNOS_MBOX_INTSR0);
	snapshot->intmr0 = readl(regs + EXYNOS_MBOX_INTMR0);
	snapshot->intmsr0 = readl(regs + EXYNOS_MBOX_INTMSR0);
	snapshot->intsr1 = readl(regs + EXYNOS_MBOX_INTSR1);
	snapshot->intmr1 = readl(regs + EXYNOS_MBOX_INTMR1);
	snapshot->intmsr1 = readl(regs + EXYNOS_MBOX_INTMSR1);

	return true;
}
EXPORT_SYMBOL_GPL(exynos_mbox_read_regs);

/**
 * exynos_mbox_dump_regs() - dump mailbox state without taking locks
 * @chan: mailbox channel whose controller should be dumped
 *
 * This is intended for fatal diagnostics where the mailbox peer has stopped
 * responding.  Keep the reads to the status and mask registers: unlike the
 * generation and clear registers, these are safe to inspect.
 */
void exynos_mbox_dump_regs(struct mbox_chan *chan)
{
	struct exynos_mbox_regs snapshot;
	struct device *dev;

	if (!chan || !chan->mbox || !chan->mbox->dev)
		return;

	dev = chan->mbox->dev;
	if (!exynos_mbox_read_regs(chan, &snapshot))
		return;

	dev_emerg(dev,
		  "registers: INTSR0:%08x INTMR0:%08x INTMSR0:%08x INTSR1:%08x INTMR1:%08x INTMSR1:%08x\n",
		  snapshot.intsr0, snapshot.intmr0, snapshot.intmsr0,
		  snapshot.intsr1, snapshot.intmr1, snapshot.intmsr1);
}
EXPORT_SYMBOL_GPL(exynos_mbox_dump_regs);

static int exynos_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct device *dev = chan->mbox->dev;
	struct exynos_mbox *exynos_mbox = dev_get_drvdata(dev);
	struct exynos_mbox_msg *msg = data;

	if (msg->chan_id >= exynos_mbox->mbox->num_chans) {
		dev_err(dev, "Invalid channel ID %d\n", msg->chan_id);
		return -EINVAL;
	}

	if (msg->chan_type != EXYNOS_MBOX_CHAN_TYPE_DOORBELL) {
		dev_err(dev, "Unsupported channel type [%d]\n", msg->chan_type);
		return -EINVAL;
	}

	writel(BIT(msg->chan_id), exynos_mbox->regs + EXYNOS_MBOX_INTGR1);

	return 0;
}

static const struct mbox_chan_ops exynos_mbox_chan_ops = {
	.send_data = exynos_mbox_send_data,
};

static struct mbox_chan *exynos_mbox_of_xlate(struct mbox_controller *mbox,
					      const struct of_phandle_args *sp)
{
	int i;

	if (sp->args_count != 0)
		return ERR_PTR(-EINVAL);

	/*
	 * Return the first available channel. When we don't pass the
	 * channel ID from device tree, each channel populated by the driver is
	 * just a software construct or a virtual channel. We use 'void *data'
	 * in send_data() to pass the channel identifiers.
	 */
	for (i = 0; i < mbox->num_chans; i++)
		if (mbox->chans[i].cl == NULL)
			return &mbox->chans[i];
	return ERR_PTR(-EINVAL);
}

static const struct of_device_id exynos_mbox_match[] = {
	{ .compatible = "google,gs101-mbox" },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_mbox_match);

static int exynos_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_mbox *exynos_mbox;
	struct mbox_controller *mbox;
	struct mbox_chan *chans;
	struct clk *pclk;

	exynos_mbox = devm_kzalloc(dev, sizeof(*exynos_mbox), GFP_KERNEL);
	if (!exynos_mbox)
		return -ENOMEM;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	chans = devm_kcalloc(dev, EXYNOS_MBOX_CHAN_COUNT, sizeof(*chans),
			     GFP_KERNEL);
	if (!chans)
		return -ENOMEM;

	exynos_mbox->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(exynos_mbox->regs))
		return PTR_ERR(exynos_mbox->regs);

	pclk = devm_clk_get_enabled(dev, "pclk");
	if (IS_ERR(pclk))
		return dev_err_probe(dev, PTR_ERR(pclk),
				     "Failed to enable clock.\n");

	mbox->num_chans = EXYNOS_MBOX_CHAN_COUNT;
	mbox->chans = chans;
	mbox->dev = dev;
	mbox->ops = &exynos_mbox_chan_ops;
	mbox->of_xlate = exynos_mbox_of_xlate;

	exynos_mbox->mbox = mbox;

	platform_set_drvdata(pdev, exynos_mbox);

	/* Mask out all interrupts. We support just polling channels for now. */
	writel(EXYNOS_MBOX_INTMR0_MASK, exynos_mbox->regs + EXYNOS_MBOX_INTMR0);

	return devm_mbox_controller_register(dev, mbox);
}

static struct platform_driver exynos_mbox_driver = {
	.probe	= exynos_mbox_probe,
	.driver	= {
		.name = "exynos-acpm-mbox",
		.of_match_table	= exynos_mbox_match,
	},
};
module_platform_driver(exynos_mbox_driver);

MODULE_AUTHOR("Tudor Ambarus <tudor.ambarus@linaro.org>");
MODULE_DESCRIPTION("Samsung Exynos mailbox driver");
MODULE_LICENSE("GPL");
