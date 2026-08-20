/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MEDIA_EXYNOS_BECORE_H__
#define __MEDIA_EXYNOS_BECORE_H__

#include <linux/types.h>

struct device;

/* Private attachment returned to one camera-front-end producer. */
struct exynos_becore_input;

struct exynos_becore_input *
exynos_becore_input_map(struct device *backend, struct device *producer);
void exynos_becore_input_unmap(struct exynos_becore_input *input);

dma_addr_t exynos_becore_input_dma(struct exynos_becore_input *input);
size_t exynos_becore_input_size(struct exynos_becore_input *input);

int exynos_becore_input_producer_begin(struct exynos_becore_input *input);
int exynos_becore_input_producer_complete(struct exynos_becore_input *input);
void exynos_becore_input_producer_abort(struct exynos_becore_input *input);

#endif /* __MEDIA_EXYNOS_BECORE_H__ */
