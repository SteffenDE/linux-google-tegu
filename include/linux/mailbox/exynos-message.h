/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos mailbox message.
 *
 * Copyright 2024 Linaro Ltd.
 */

#ifndef _LINUX_EXYNOS_MESSAGE_H_
#define _LINUX_EXYNOS_MESSAGE_H_

#include <linux/kconfig.h>
#include <linux/types.h>

#define EXYNOS_MBOX_CHAN_TYPE_DOORBELL		0
#define EXYNOS_MBOX_CHAN_TYPE_DATA		1

struct mbox_chan;

struct exynos_mbox_msg {
	unsigned int chan_id;
	unsigned int chan_type;
};

#if IS_REACHABLE(CONFIG_EXYNOS_MBOX)
void exynos_mbox_dump_regs(struct mbox_chan *chan);
int exynos_mbox_set_chan_polling(struct mbox_chan *chan, unsigned int chan_id,
				 bool polling);
int exynos_mbox_clear_chan_irq(struct mbox_chan *chan, unsigned int chan_id);
#else
static inline void exynos_mbox_dump_regs(struct mbox_chan *chan)
{
}

static inline int exynos_mbox_set_chan_polling(struct mbox_chan *chan,
					       unsigned int chan_id,
					       bool polling)
{
	return -EOPNOTSUPP;
}

static inline int exynos_mbox_clear_chan_irq(struct mbox_chan *chan,
					     unsigned int chan_id)
{
	return -EOPNOTSUPP;
}
#endif

#endif /* _LINUX_EXYNOS_MESSAGE_H_ */
