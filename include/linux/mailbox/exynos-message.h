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

struct exynos_mbox_regs {
	u32 intsr0;
	u32 intmr0;
	u32 intmsr0;
	u32 intsr1;
	u32 intmr1;
	u32 intmsr1;
};

#if IS_REACHABLE(CONFIG_EXYNOS_MBOX)
bool exynos_mbox_read_regs(struct mbox_chan *chan,
			    struct exynos_mbox_regs *snapshot);
void exynos_mbox_dump_regs(struct mbox_chan *chan);
#else
static inline bool exynos_mbox_read_regs(struct mbox_chan *chan,
					  struct exynos_mbox_regs *snapshot)
{
	return false;
}

static inline void exynos_mbox_dump_regs(struct mbox_chan *chan)
{
}
#endif

#endif /* _LINUX_EXYNOS_MESSAGE_H_ */
