/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Minimal stub for the tegu AoC bring-up.  The downstream ITMON bus-error
 * monitor is not ported, so the AoC ITMON notifier is registered as a
 * no-op and never invoked.  struct itmon_notifier is kept intact so the
 * AoC callback compiles unchanged.
 */
#ifndef __SOC_GOOGLE_EXYNOS_ITMON_H
#define __SOC_GOOGLE_EXYNOS_ITMON_H

#include <linux/notifier.h>

struct itmon_notifier {
	char *port;			/* block the client IP belongs to */
	char *client;			/* client whose access faulted */
	char *dest;			/* destination the client accessed */
	bool read;			/* transaction type */
	unsigned long target_addr;	/* physical address accessed */
	unsigned int errcode;		/* error code */
	bool onoff;			/* target block on/off */
	char *pd_name;			/* target block power-domain name */
};

static inline void itmon_notifier_chain_register(struct notifier_block *n) { }
static inline void itmon_notifier_chain_unregister(struct notifier_block *n) { }

#endif /* __SOC_GOOGLE_EXYNOS_ITMON_H */
