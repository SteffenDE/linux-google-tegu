// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2011,2020 Google LLC
 *
 * Physically-contiguous carveout dma-heap for the AoC. Owns a gen_pool
 * (genalloc) over [base, base + size] and hands out single-segment,
 * physically contiguous allocations on the mainline dma-heap API.
 */
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/iosys-map.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "ion_physical_heap.h"

struct ion_physical_heap_attachment {
	struct device *dev;
	struct sg_table table;
	struct list_head list;
	bool mapped;
};

static int _clear_pages(struct page **pages, int num, pgprot_t pgprot)
{
	void *addr = vmap(pages, num, VM_MAP, pgprot);

	if (!addr)
		return -ENOMEM;
	memset(addr, 0, PAGE_SIZE * num);
	vunmap(addr);

	return 0;
}

static int _sglist_zero(struct scatterlist *sgl, unsigned int nents,
			pgprot_t pgprot)
{
	int p = 0;
	int ret = 0;
	struct sg_page_iter piter;
	struct page *pages[32];

	for_each_sg_page(sgl, &piter, nents, 0) {
		pages[p++] = sg_page_iter_page(&piter);
		if (p == ARRAY_SIZE(pages)) {
			ret = _clear_pages(pages, p, pgprot);
			if (ret)
				return ret;
			p = 0;
		}
	}
	if (p)
		ret = _clear_pages(pages, p, pgprot);

	return ret;
}

static int _pages_zero(struct page *page, size_t size, pgprot_t pgprot)
{
	struct scatterlist sg;

	sg_init_table(&sg, 1);
	sg_set_page(&sg, page, size, 0);
	return _sglist_zero(&sg, 1, pgprot);
}

static int ion_physical_heap_attach(struct dma_buf *dmabuf,
				    struct dma_buf_attachment *attachment)
{
	struct ion_physical_heap_buffer *buffer = dmabuf->priv;
	struct ion_physical_heap_attachment *a;
	int ret;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	ret = sg_alloc_table(&a->table, 1, GFP_KERNEL);
	if (ret) {
		kfree(a);
		return ret;
	}
	sg_set_page(a->table.sgl, sg_page(buffer->sg_table.sgl), buffer->len, 0);

	a->dev = attachment->dev;
	INIT_LIST_HEAD(&a->list);
	a->mapped = false;

	attachment->priv = a;

	mutex_lock(&buffer->lock);
	list_add(&a->list, &buffer->attachments);
	mutex_unlock(&buffer->lock);

	return 0;
}

static void ion_physical_heap_detach(struct dma_buf *dmabuf,
				     struct dma_buf_attachment *attachment)
{
	struct ion_physical_heap_buffer *buffer = dmabuf->priv;
	struct ion_physical_heap_attachment *a = attachment->priv;

	mutex_lock(&buffer->lock);
	list_del(&a->list);
	mutex_unlock(&buffer->lock);

	sg_free_table(&a->table);
	kfree(a);
}

static struct sg_table *ion_physical_heap_map_dma_buf(struct dma_buf_attachment *attachment,
						      enum dma_data_direction direction)
{
	struct ion_physical_heap_attachment *a = attachment->priv;
	struct sg_table *table = &a->table;
	int ret;

	ret = dma_map_sgtable(attachment->dev, table, direction, 0);
	if (ret)
		return ERR_PTR(-ENOMEM);
	a->mapped = true;
	return table;
}

static void ion_physical_heap_unmap_dma_buf(struct dma_buf_attachment *attachment,
					    struct sg_table *table,
					    enum dma_data_direction direction)
{
	struct ion_physical_heap_attachment *a = attachment->priv;

	a->mapped = false;
	dma_unmap_sgtable(attachment->dev, table, direction, 0);
}

static int ion_physical_heap_dma_buf_begin_cpu_access(struct dma_buf *dmabuf,
						      enum dma_data_direction direction)
{
	struct ion_physical_heap_buffer *buffer = dmabuf->priv;
	struct ion_physical_heap_attachment *a;

	mutex_lock(&buffer->lock);

	if (buffer->vmap_cnt)
		invalidate_kernel_vmap_range(buffer->vaddr, buffer->len);

	list_for_each_entry(a, &buffer->attachments, list) {
		if (!a->mapped)
			continue;
		dma_sync_sgtable_for_cpu(a->dev, &a->table, direction);
	}
	mutex_unlock(&buffer->lock);

	return 0;
}

static int ion_physical_heap_dma_buf_end_cpu_access(struct dma_buf *dmabuf,
						    enum dma_data_direction direction)
{
	struct ion_physical_heap_buffer *buffer = dmabuf->priv;
	struct ion_physical_heap_attachment *a;

	mutex_lock(&buffer->lock);

	if (buffer->vmap_cnt)
		flush_kernel_vmap_range(buffer->vaddr, buffer->len);

	list_for_each_entry(a, &buffer->attachments, list) {
		if (!a->mapped)
			continue;
		dma_sync_sgtable_for_device(a->dev, &a->table, direction);
	}
	mutex_unlock(&buffer->lock);

	return 0;
}

static int ion_physical_heap_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct ion_physical_heap_buffer *buffer = dmabuf->priv;
	unsigned long pfn = PFN_DOWN(buffer->paddr);

	if ((vma->vm_flags & (VM_SHARED | VM_MAYSHARE)) == 0)
		return -EINVAL;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	return remap_pfn_range(vma, vma->vm_start, pfn + vma->vm_pgoff,
			       vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

static void *ion_physical_heap_do_vmap(struct ion_physical_heap_buffer *buffer)
{
	pgoff_t pagecount = PFN_UP(buffer->len);
	unsigned long pfn = page_to_pfn(sg_page(buffer->sg_table.sgl));
	struct page **pages;
	void *vaddr;
	pgoff_t i;

	pages = kvmalloc_array(pagecount, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < pagecount; i++)
		pages[i] = pfn_to_page(pfn + i);

	vaddr = vmap(pages, pagecount, VM_MAP, pgprot_writecombine(PAGE_KERNEL));
	kvfree(pages);

	if (!vaddr)
		return ERR_PTR(-ENOMEM);

	return vaddr;
}

static int ion_physical_heap_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct ion_physical_heap_buffer *buffer = dmabuf->priv;
	void *vaddr;
	int ret = 0;

	mutex_lock(&buffer->lock);
	if (buffer->vmap_cnt) {
		buffer->vmap_cnt++;
		iosys_map_set_vaddr(map, buffer->vaddr);
		goto out;
	}

	vaddr = ion_physical_heap_do_vmap(buffer);
	if (IS_ERR(vaddr)) {
		ret = PTR_ERR(vaddr);
		goto out;
	}
	buffer->vaddr = vaddr;
	buffer->vmap_cnt++;
	iosys_map_set_vaddr(map, buffer->vaddr);
out:
	mutex_unlock(&buffer->lock);

	return ret;
}

static void ion_physical_heap_vunmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct ion_physical_heap_buffer *buffer = dmabuf->priv;

	mutex_lock(&buffer->lock);
	if (!--buffer->vmap_cnt) {
		vunmap(buffer->vaddr);
		buffer->vaddr = NULL;
	}
	mutex_unlock(&buffer->lock);
	iosys_map_clear(map);
}

static void ion_physical_heap_dma_buf_release(struct dma_buf *dmabuf)
{
	struct ion_physical_heap_buffer *buffer = dmabuf->priv;
	struct ion_physical_heap *physical_heap = buffer->heap;

	if (physical_heap->free_cb)
		physical_heap->free_cb(buffer, physical_heap->free_ctx);

	if (buffer->vmap_cnt > 0) {
		WARN(1, "%s: buffer still mapped in the kernel\n", __func__);
		vunmap(buffer->vaddr);
		buffer->vaddr = NULL;
	}

	_pages_zero(sg_page(buffer->sg_table.sgl), buffer->len,
		    pgprot_writecombine(PAGE_KERNEL));

	if (buffer->paddr)
		gen_pool_free(physical_heap->pool, buffer->paddr,
			      ALIGN(buffer->len, physical_heap->align));

	sg_free_table(&buffer->sg_table);
	mutex_destroy(&buffer->lock);
	kfree(buffer);
}

static const struct dma_buf_ops ion_physical_heap_buf_ops = {
	.attach = ion_physical_heap_attach,
	.detach = ion_physical_heap_detach,
	.map_dma_buf = ion_physical_heap_map_dma_buf,
	.unmap_dma_buf = ion_physical_heap_unmap_dma_buf,
	.begin_cpu_access = ion_physical_heap_dma_buf_begin_cpu_access,
	.end_cpu_access = ion_physical_heap_dma_buf_end_cpu_access,
	.mmap = ion_physical_heap_mmap,
	.vmap = ion_physical_heap_vmap,
	.vunmap = ion_physical_heap_vunmap,
	.release = ion_physical_heap_dma_buf_release,
};

static struct dma_buf *ion_physical_heap_allocate(struct dma_heap *heap,
						  unsigned long len,
						  u32 fd_flags,
						  u64 heap_flags)
{
	struct ion_physical_heap *physical_heap = dma_heap_get_drvdata(heap);
	unsigned long aligned_size = ALIGN(len, physical_heap->align);
	struct ion_physical_heap_buffer *buffer;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct dma_buf *dmabuf;
	phys_addr_t paddr;
	int ret;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&buffer->attachments);
	mutex_init(&buffer->lock);
	buffer->heap = physical_heap;
	buffer->len = len;

	ret = sg_alloc_table(&buffer->sg_table, 1, GFP_KERNEL);
	if (ret)
		goto free_buffer;

	paddr = gen_pool_alloc(physical_heap->pool, aligned_size);
	if (!paddr) {
		pr_err("%s: failed to allocate from AOC physical heap size %lu\n",
		       __func__, len);
		ret = -ENOMEM;
		goto free_table;
	}
	buffer->paddr = paddr;

	sg_set_page(buffer->sg_table.sgl, pfn_to_page(PFN_DOWN(paddr)), len, 0);
	sg_dma_address(buffer->sg_table.sgl) = paddr;
	sg_dma_len(buffer->sg_table.sgl) = len;

	/*
	 * Stable, unique per-buffer correlation token that AoC uses to pair the
	 * map issued from alloc_cb with the unmap issued from free_cb.
	 */
	buffer->priv = buffer;

	if (physical_heap->allocate_cb)
		physical_heap->allocate_cb(buffer, physical_heap->allocate_ctx);

	exp_info.exp_name = dma_heap_get_name(heap);
	exp_info.ops = &ion_physical_heap_buf_ops;
	exp_info.size = len;
	exp_info.flags = fd_flags;
	exp_info.priv = buffer;

	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto free_cb;
	}

	return dmabuf;

free_cb:
	if (physical_heap->free_cb)
		physical_heap->free_cb(buffer, physical_heap->free_ctx);
	gen_pool_free(physical_heap->pool, paddr, aligned_size);
free_table:
	sg_free_table(&buffer->sg_table);
free_buffer:
	mutex_destroy(&buffer->lock);
	kfree(buffer);

	return ERR_PTR(ret);
}

static const struct dma_heap_ops ion_physical_heap_ops = {
	.allocate = ion_physical_heap_allocate,
};

struct dma_heap *ion_physical_heap_create(phys_addr_t base, size_t size,
					  size_t align, const char *name,
					  ion_physical_heap_allocate_callback alloc_cb,
					  ion_physical_heap_free_callback free_cb,
					  void *ctx)
{
	struct ion_physical_heap *physical_heap;
	struct dma_heap_export_info exp_info;
	struct dma_heap *heap;
	int ret;

	ret = _pages_zero(pfn_to_page(PFN_DOWN(base)), size,
			  pgprot_writecombine(PAGE_KERNEL));
	if (ret)
		return ERR_PTR(ret);

	physical_heap = kzalloc(sizeof(*physical_heap), GFP_KERNEL);
	if (!physical_heap)
		return ERR_PTR(-ENOMEM);

	physical_heap->pool = gen_pool_create(get_order(align) + PAGE_SHIFT, -1);
	if (!physical_heap->pool) {
		kfree(physical_heap);
		return ERR_PTR(-ENOMEM);
	}

	physical_heap->base = base;
	gen_pool_add(physical_heap->pool, physical_heap->base, size, -1);

	physical_heap->size = size;
	physical_heap->align = align;
	physical_heap->name = name;

	physical_heap->allocate_cb = alloc_cb;
	physical_heap->allocate_ctx = ctx;

	physical_heap->free_cb = free_cb;
	physical_heap->free_ctx = ctx;

	exp_info.name = name;
	exp_info.ops = &ion_physical_heap_ops;
	exp_info.priv = physical_heap;

	heap = dma_heap_add(&exp_info);
	if (IS_ERR(heap)) {
		gen_pool_destroy(physical_heap->pool);
		kfree(physical_heap);
		return heap;
	}

	physical_heap->heap = heap;

	return heap;
}
