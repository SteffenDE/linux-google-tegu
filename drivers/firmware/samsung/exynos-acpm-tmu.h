/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2017 Samsung Electronics Co., Ltd.
 * Copyright 2025 Google LLC.
 */
#ifndef __EXYNOS_ACPM_TMU_H__
#define __EXYNOS_ACPM_TMU_H__

#include <linux/types.h>

struct acpm_handle;

int acpm_tmu_read_temp(struct acpm_handle *handle, unsigned int acpm_chan_id,
		       u8 tzid, int *temp, int *stat);

#endif /* __EXYNOS_ACPM_TMU_H__ */
