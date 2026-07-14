/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Minimal stub for the tegu AoC bring-up.  The downstream PMU-IF regmap
 * is not shimmed; reads return 0.  (tegu writes PMU registers via EL3 SMC
 * elsewhere; wire a real shim here if AoC turns out to need a live PMU
 * read.)
 */
#ifndef __SOC_GOOGLE_EXYNOS_PMU_IF_H
#define __SOC_GOOGLE_EXYNOS_PMU_IF_H

static inline int exynos_pmu_read(unsigned int offset, unsigned int *val)
{
	if (val)
		*val = 0;
	return 0;
}

static inline int exynos_pmu_write(unsigned int offset, unsigned int val)
{
	return 0;
}

static inline int exynos_pmu_update(unsigned int offset, unsigned int mask,
				    unsigned int val)
{
	return 0;
}

#endif /* __SOC_GOOGLE_EXYNOS_PMU_IF_H */
