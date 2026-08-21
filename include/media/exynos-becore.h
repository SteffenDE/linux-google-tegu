/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MEDIA_EXYNOS_BECORE_H__
#define __MEDIA_EXYNOS_BECORE_H__

#include <linux/types.h>

struct device;

/* Private attachment returned to one camera-front-end producer. */
struct exynos_becore_input;

struct exynos_becore_input_producer_ops {
	int (*start_streaming)(void *data);
	void (*stop_streaming)(void *data);
};

/* One driver-owned input slot reserved for a front-end producer. */
struct exynos_becore_input_buffer {
	dma_addr_t dma;
	size_t size;
	u64 cookie;
	unsigned int slot;
};

struct exynos_becore_input *
exynos_becore_input_map(struct device *backend, struct device *producer,
			const struct exynos_becore_input_producer_ops *ops,
			void *producer_data);
void exynos_becore_input_disconnect(struct exynos_becore_input *input);
void exynos_becore_input_unmap(struct exynos_becore_input *input);

size_t exynos_becore_input_size(struct exynos_becore_input *input);

int exynos_becore_input_producer_acquire(struct exynos_becore_input *input,
					 struct exynos_becore_input_buffer *buffer);
int exynos_becore_input_producer_complete(struct exynos_becore_input *input,
					  const struct exynos_becore_input_buffer *buffer);
void exynos_becore_input_producer_abort(struct exynos_becore_input *input,
					const struct exynos_becore_input_buffer *buffer);

#endif /* __MEDIA_EXYNOS_BECORE_H__ */
