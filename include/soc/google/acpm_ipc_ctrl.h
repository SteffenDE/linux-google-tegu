/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Minimal stub for the tegu AoC bring-up.  The downstream ACPM IPC channel
 * API is not shimmed here, so the AoC "reset over ACPM" callback is never
 * registered.  Initial firmware boot + service enumeration does not need it
 * (it matters for AoC reset/recovery).  tegu drives ACPM elsewhere
 * (drivers/firmware/samsung/exynos-acpm*); wire a real shim when AoC reset
 * is brought up.
 */
#ifndef __SOC_GOOGLE_ACPM_IPC_CTRL_H
#define __SOC_GOOGLE_ACPM_IPC_CTRL_H

#include <linux/of.h>

typedef void (*ipc_callback)(unsigned int *cmd, unsigned int size);

static inline int acpm_ipc_request_channel(struct device_node *np,
					   ipc_callback handler,
					   unsigned int *id, unsigned int *size)
{
	if (id)
		*id = 0;
	if (size)
		*size = 0;
	return 0;
}

static inline int acpm_ipc_release_channel(struct device_node *np,
					   unsigned int id)
{
	return 0;
}

#endif /* __SOC_GOOGLE_ACPM_IPC_CTRL_H */
