/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Google LLC
 */

#ifndef _ION_PHYSICAL_HEAP_H
#define _ION_PHYSICAL_HEAP_H

#include <linux/genalloc.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/types.h>

struct dma_heap;
struct ion_physical_heap;

/*
 * Per-allocation descriptor handed to the AoC map/unmap callbacks and stashed
 * in dma_buf->priv. The single-entry @sg_table describes the physically
 * contiguous carveout allocation; @priv is a stable, unique correlation token
 * (the descriptor pointer) used by AoC to pair a map at alloc with an unmap at
 * free.
 */
struct ion_physical_heap_buffer {
	struct ion_physical_heap *heap;
	struct list_head attachments;
	struct mutex lock;
	unsigned long len;
	phys_addr_t paddr;
	struct sg_table sg_table;
	void *priv;
	int vmap_cnt;
	void *vaddr;
};

typedef void(ion_physical_heap_allocate_callback)(struct ion_physical_heap_buffer *buffer,
						  void *ctx);
typedef void(ion_physical_heap_free_callback)(struct ion_physical_heap_buffer *buffer,
					      void *ctx);

struct dma_heap *ion_physical_heap_create(phys_addr_t base, size_t size,
					  size_t align, const char *name,
					  ion_physical_heap_allocate_callback alloc_cb,
					  ion_physical_heap_free_callback free_cb,
					  void *ctx);

struct ion_physical_heap {
	struct gen_pool *pool;
	struct dma_heap *heap;
	phys_addr_t base;
	size_t size;
	size_t align;
	const char *name;

	ion_physical_heap_allocate_callback *allocate_cb;
	void *allocate_ctx;

	ion_physical_heap_free_callback *free_cb;
	void *free_ctx;
};

#endif /* _ION_PHYSICAL_HEAP_H */
