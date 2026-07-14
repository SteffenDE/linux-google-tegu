/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Minimal stub for the tegu AoC bring-up.  The downstream debug-snapshot
 * subsystem is not ported; AoC only uses the emergency-reboot entry point,
 * which becomes a no-op returning failure (matching the downstream
 * CONFIG-disabled fallback).
 */
#ifndef __SOC_GOOGLE_DEBUG_SNAPSHOT_H
#define __SOC_GOOGLE_DEBUG_SNAPSHOT_H

static inline int dbg_snapshot_emergency_reboot(const char *str)
{
	return -1;
}

static inline int dbg_snapshot_emergency_reboot_timeout(const char *str,
							int tick)
{
	return -1;
}

#endif /* __SOC_GOOGLE_DEBUG_SNAPSHOT_H */
