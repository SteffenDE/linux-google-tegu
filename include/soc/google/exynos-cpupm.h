/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Minimal stub for the tegu AoC bring-up.  The downstream exynos-cpupm
 * driver is not ported; disable/enable_power_mode() become no-ops, so
 * AoC's "keep the AP awake for ~2s during firmware boot" optimization is
 * simply skipped.  Revisit if AoC boot timing needs it.
 */
#ifndef __SOC_GOOGLE_EXYNOS_CPUPM_H
#define __SOC_GOOGLE_EXYNOS_CPUPM_H

enum {
	POWERMODE_TYPE_CLUSTER = 0,
	POWERMODE_TYPE_SYSTEM,
	POWERMODE_TYPE_END,
};

static inline void disable_power_mode(int cpu, int type) { }
static inline void enable_power_mode(int cpu, int type) { }

#endif /* __SOC_GOOGLE_EXYNOS_CPUPM_H */
