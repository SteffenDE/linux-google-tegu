// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung System MMU v9 support.
 *
 * This is a deliberately small VM0-only bring-up driver for the System MMU v9
 * blocks used by newer Samsung/Google display pipelines.  It keeps the v9 page
 * table format and stream-table programming separate from the older
 * exynos-iommu driver.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_iommu.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/string_choices.h>

#include "iommu-pages.h"

typedef u64 sysmmu_iova_t;
typedef u32 sysmmu_pte_t;

#define SYSMMU_VM0				0
#define SYSMMU_MASK_VM0				BIT(SYSMMU_VM0)
#define SYSMMU_MAX_PMMUS			8
#define SYSMMU_DEFAULT_QOS			(-1)
#define SYSMMU_DEFAULT_STREAM_NONE		U32_MAX
#define SYSMMU_UNUSED_STREAM_INDEX		U32_MAX

#define REG_MMU_CTRL				0x0000
#define REG_MMU_STATUS				0x0008
#define REG_MMU_VERSION				0x0034
#define REG_MMU_NUM_CONTEXT			0x0100
#define REG_MMU_NUM_CONTEXT_PER_VM(n)		(0x0140 + (n) * 0x4)
#define REG_MMU_STREAM_CFG(n)			(0x2000 + (n) * 0x10)
#define REG_MMU_STREAM_MATCH_CFG(n)		(0x2004 + (n) * 0x10)
#define REG_MMU_STREAM_MATCH_SID_VALUE(n)	(0x2008 + (n) * 0x10)
#define REG_MMU_STREAM_MATCH_SID_MASK(n)	(0x200c + (n) * 0x10)
#define REG_MMU_PMMU_INDICATOR			0x2ffc
#define REG_MMU_PMMU_INFO			0x3000
#define REG_MMU_SWALKER_INFO			0x3004
#define REG_MMU_CTRL_VM				0x8000
#define REG_MMU_ALL_INV_VM			0x8010
#define REG_MMU_RANGE_INV_START_VPN_VM		0x8020
#define REG_MMU_RANGE_INV_END_VPN_AND_TRIG_VM	0x8024
#define REG_MMU_FAULT_STATUS_VM			0x8060
#define REG_MMU_FAULT_CLEAR_VM			0x8064
#define REG_MMU_FAULT_VA_VM			0x8070
#define REG_MMU_FAULT_INFO0_VM			0x8074
#define REG_MMU_FAULT_INFO1_VM			0x8078
#define REG_MMU_FAULT_INFO2_VM			0x807c
#define REG_MMU_CONTEXT0_CFG_FLPT_BASE_VM	0x8404
#define REG_MMU_CONTEXT0_CFG_ATTRIBUTE_VM	0x8408

#define MMU_VERSION_MAJOR(val)			((val) >> 12)
#define MMU_VERSION_MINOR(val)			(((val) >> 8) & 0xf)
#define MMU_VERSION_REVISION(val)		((val) & 0xff)
#define MMU_VERSION_RAW(reg)			(((reg) >> 16) & 0xffff)
#define MMU_NUM_CONTEXT(reg)			((reg) & 0x1f)
#define MMU_NUM_CONTEXT_PER_VM(reg)		((reg) & 0x1f)
#define MMU_SWALKER_INFO_NUM_PMMU(reg)		((reg) & 0xffff)
#define MMU_PMMU_INFO_NUM_STREAM_TABLE(reg)	(((reg) >> 16) & 0xffff)
#define MMU_VM_ADDR(base, idx)			((base) + (idx) * 0x1000)

#define MMU_CTRL_ENABLE				BIT(0)
#define MMU_STREAM_CFG_MASK(reg)		((reg) & (GENMASK(31, 16) | \
							 GENMASK(8, 8) | \
							 GENMASK(6, 0)))
#define MMU_STREAM_MATCH_CFG_MASK(reg)		((reg) & (GENMASK(9, 8) | BIT(0)))
#define MMU_SET_PMMU_INDICATOR(val)		((val) & 0xf)

#define MMU_FAULT_STATUS_MASK			GENMASK(4, 0)
#define MMU_FAULT_INFO0_WRITE			BIT(20)
#define MMU_FAULT_INFO0_VA36			BIT(21)
#define MMU_FAULT_INFO0_VA_HIGH(reg)		((u64)((reg) & GENMASK(25, 22)) << 10)
#define MMU_FAULT_INFO2_PMMU_ID(reg)		(((reg) >> 24) & 0xff)
#define MMU_FAULT_INFO2_STREAM_ID(reg)		((reg) & 0xffffff)

#define CFG_USE_AP				BIT(2)
#define CFG_QOS(n)				(((n) & 0xf) << 7)
#define CFG_QOS_OVERRIDE			BIT(11)
#define CFG_PT_CACHEABLE_SHIFT			16
#define CFG_PT_CACHEABLE_MASK			GENMASK(19, CFG_PT_CACHEABLE_SHIFT)
#define CFG_PT_CACHEABLE_NORMAL_NC		(0x2 << CFG_PT_CACHEABLE_SHIFT)

#define PT_BASE_SHIFT				12
#define PG_ENT_SHIFT				4
#define SECT_ORDER				20
#define LPAGE_ORDER				16
#define SPAGE_ORDER				12
#define SECT_SIZE				(1UL << SECT_ORDER)
#define LPAGE_SIZE				(1UL << LPAGE_ORDER)
#define SPAGE_SIZE				(1UL << SPAGE_ORDER)
#define SECT_MASK				(~(SECT_SIZE - 1))
#define LPAGE_MASK				(~(LPAGE_SIZE - 1))
#define SPAGE_MASK				(~(SPAGE_SIZE - 1))
#define SECT_ENT_MASK				~((SECT_SIZE >> PG_ENT_SHIFT) - 1)
#define LPAGE_ENT_MASK				~((LPAGE_SIZE >> PG_ENT_SHIFT) - 1)
#define SPAGE_ENT_MASK				~((SPAGE_SIZE >> PG_ENT_SHIFT) - 1)
#define NUM_LV1ENTRIES				65536
#define NUM_LV2ENTRIES				(SECT_SIZE / SPAGE_SIZE)
#define LV1TABLE_SIZE				(NUM_LV1ENTRIES * sizeof(sysmmu_pte_t))
#define LV2TABLE_SIZE				(NUM_LV2ENTRIES * sizeof(sysmmu_pte_t))
#define SPAGES_PER_LPAGE			(LPAGE_SIZE / SPAGE_SIZE)

#define FLPD_FLAG_MASK				7
#define SLPD_FLAG_MASK				3
#define UNMAPPED_FLAG				0
#define SLPD_FLAG				1
#define SECT_FLAG				2
#define LPAGE_FLAG				1
#define SPAGE_FLAG				2
#define FLPD_AP_READ				BIT(4)
#define FLPD_AP_WRITE				BIT(5)
#define FLPD_SHAREABLE				BIT(6)
#define SLPD_AP_READ				BIT(2)
#define SLPD_AP_WRITE				BIT(3)
#define SLPD_SHAREABLE				BIT(4)

struct samsung_sysmmu_v9_stream_config {
	u32 index;
	u32 cfg;
	u32 match_cfg;
	u32 match_id_value;
	u32 match_id_mask;
};

struct samsung_sysmmu_v9_stream_props {
	u32 default_cfg;
	unsigned int count;
	struct samsung_sysmmu_v9_stream_config *cfg;
};

struct samsung_sysmmu_v9_drvdata {
	struct device *dev;
	struct iommu_device iommu;
	struct list_head domain_node;
	void __iomem *sfrbase;
	struct clk *clk;
	spinlock_t lock;
	phys_addr_t pgtable;
	struct iommu_domain *attached_domain;
	struct device *master;
	unsigned int attached_count;
	bool rpm_active;
	u32 version;
	u32 max_vm;
	u32 num_pmmu;
	int qos;
	bool secure_irq;
	u32 secure_base;
	const char *port_name;
	struct samsung_sysmmu_v9_stream_props *props;
};

struct samsung_sysmmu_v9_client {
	struct samsung_sysmmu_v9_drvdata **sysmmus;
	struct device_link **links;
	unsigned int count;
};

struct samsung_sysmmu_v9_domain {
	struct iommu_domain domain;
	struct iommu_group *group;
	struct device *dma_dev;
	sysmmu_pte_t *pgtable;
	int *lv2entcnt;
	spinlock_t pgtablelock;
};

static const struct iommu_ops samsung_sysmmu_v9_ops;
static struct device *samsung_sysmmu_v9_dma_dev;

static void samsung_sysmmu_v9_clear_dma_dev(struct device *dev)
{
	if (samsung_sysmmu_v9_dma_dev == dev)
		samsung_sysmmu_v9_dma_dev = NULL;
}

static inline struct samsung_sysmmu_v9_domain *
to_samsung_sysmmu_v9_domain(struct iommu_domain *domain)
{
	return container_of(domain, struct samsung_sysmmu_v9_domain, domain);
}

static inline u32 lv1ent_offset(sysmmu_iova_t iova)
{
	return iova >> SECT_ORDER;
}

static inline u32 lv2ent_offset(sysmmu_iova_t iova)
{
	return (iova >> SPAGE_ORDER) & (NUM_LV2ENTRIES - 1);
}

static inline bool lv1ent_unmapped(sysmmu_pte_t *sent)
{
	return (*sent & FLPD_FLAG_MASK) == UNMAPPED_FLAG;
}

static inline bool lv1ent_page(sysmmu_pte_t *sent)
{
	return (*sent & FLPD_FLAG_MASK) == SLPD_FLAG;
}

static inline bool lv1ent_section(sysmmu_pte_t *sent)
{
	return (*sent & FLPD_FLAG_MASK) == SECT_FLAG;
}

static inline bool lv2ent_unmapped(sysmmu_pte_t *pent)
{
	return (*pent & SLPD_FLAG_MASK) == UNMAPPED_FLAG;
}

static inline bool lv2ent_large(sysmmu_pte_t *pent)
{
	return (*pent & SLPD_FLAG_MASK) == LPAGE_FLAG;
}

static inline bool lv2ent_small(sysmmu_pte_t *pent)
{
	return (*pent & SLPD_FLAG_MASK) == SPAGE_FLAG;
}

static inline phys_addr_t lv2table_base(sysmmu_pte_t *sent)
{
	return (phys_addr_t)(*sent & ~0x3fU) << PG_ENT_SHIFT;
}

static inline sysmmu_pte_t *section_entry(sysmmu_pte_t *pgtable,
					  sysmmu_iova_t iova)
{
	return pgtable + lv1ent_offset(iova);
}

static inline sysmmu_pte_t *page_entry(sysmmu_pte_t *sent,
				       sysmmu_iova_t iova)
{
	return (sysmmu_pte_t *)phys_to_virt(lv2table_base(sent)) +
	       lv2ent_offset(iova);
}

static inline sysmmu_pte_t make_sysmmu_pte(phys_addr_t paddr, int pgsize,
					   int attr)
{
	return (paddr >> PG_ENT_SHIFT) | pgsize | attr;
}

static inline phys_addr_t pte_to_phys(sysmmu_pte_t pte, unsigned long mask)
{
	return (phys_addr_t)(pte & mask) << PG_ENT_SHIFT;
}

static void samsung_sysmmu_v9_flush_pgtable(struct samsung_sysmmu_v9_domain *domain,
					    void *start, size_t len)
{
	iommu_pages_flush_incoherent(domain->dma_dev, start, 0, len);
}

static void samsung_sysmmu_v9_invalidate_all(struct samsung_sysmmu_v9_drvdata *data)
{
	writel_relaxed(0x1, data->sfrbase + REG_MMU_ALL_INV_VM);
}

static void samsung_sysmmu_v9_invalidate_range(struct samsung_sysmmu_v9_drvdata *data,
					       dma_addr_t start, dma_addr_t end)
{
	writel_relaxed(ALIGN_DOWN(start, SPAGE_SIZE) >> PG_ENT_SHIFT,
		       data->sfrbase + REG_MMU_RANGE_INV_START_VPN_VM);
	writel_relaxed((ALIGN_DOWN(end, SPAGE_SIZE) >> PG_ENT_SHIFT) | 0x1,
		       data->sfrbase + REG_MMU_RANGE_INV_END_VPN_AND_TRIG_VM);
}

static void samsung_sysmmu_v9_sync_range(struct samsung_sysmmu_v9_domain *domain,
					 dma_addr_t start, dma_addr_t end)
{
	struct samsung_sysmmu_v9_drvdata *data;
	struct list_head *group_list;
	unsigned long flags;

	if (!domain->group || start > end)
		return;

	group_list = iommu_group_get_iommudata(domain->group);
	if (!group_list)
		return;

	list_for_each_entry(data, group_list, domain_node) {
		spin_lock_irqsave(&data->lock, flags);
		if (data->attached_count && data->rpm_active)
			samsung_sysmmu_v9_invalidate_range(data, start, end);
		spin_unlock_irqrestore(&data->lock, flags);
	}
}

static void samsung_sysmmu_v9_set_stream(struct samsung_sysmmu_v9_drvdata *data,
					 unsigned int pmmu_id)
{
	struct samsung_sysmmu_v9_stream_props *props = &data->props[pmmu_id];
	unsigned int i;

	writel_relaxed(MMU_SET_PMMU_INDICATOR(pmmu_id),
		       data->sfrbase + REG_MMU_PMMU_INDICATOR);
	readl_relaxed(data->sfrbase + REG_MMU_PMMU_INDICATOR);

	if (props->default_cfg != SYSMMU_DEFAULT_STREAM_NONE)
		writel_relaxed(MMU_STREAM_CFG_MASK(props->default_cfg),
			       data->sfrbase + REG_MMU_STREAM_CFG(0));

	for (i = 0; i < props->count; i++) {
		struct samsung_sysmmu_v9_stream_config *cfg = &props->cfg[i];

		if (cfg->index == SYSMMU_UNUSED_STREAM_INDEX)
			continue;

		writel_relaxed(MMU_STREAM_CFG_MASK(cfg->cfg),
			       data->sfrbase + REG_MMU_STREAM_CFG(cfg->index));
		writel_relaxed(MMU_STREAM_MATCH_CFG_MASK(cfg->match_cfg),
			       data->sfrbase + REG_MMU_STREAM_MATCH_CFG(cfg->index));
		writel_relaxed(cfg->match_id_value,
			       data->sfrbase +
			       REG_MMU_STREAM_MATCH_SID_VALUE(cfg->index));
		writel_relaxed(cfg->match_id_mask,
			       data->sfrbase +
			       REG_MMU_STREAM_MATCH_SID_MASK(cfg->index));
	}
}

static void samsung_sysmmu_v9_init_config(struct samsung_sysmmu_v9_drvdata *data)
{
	u32 cfg;
	unsigned int i;

	cfg = readl_relaxed(data->sfrbase + REG_MMU_CONTEXT0_CFG_ATTRIBUTE_VM);
	cfg &= ~CFG_PT_CACHEABLE_MASK;
	cfg |= CFG_PT_CACHEABLE_NORMAL_NC | CFG_USE_AP;

	if (data->qos != SYSMMU_DEFAULT_QOS) {
		cfg &= ~CFG_QOS(0xf);
		cfg |= CFG_QOS_OVERRIDE | CFG_QOS(data->qos);
	}

	writel_relaxed(cfg, data->sfrbase + REG_MMU_CONTEXT0_CFG_ATTRIBUTE_VM);

	for (i = 0; i < data->num_pmmu; i++)
		samsung_sysmmu_v9_set_stream(data, i);
}

static void samsung_sysmmu_v9_enable(struct samsung_sysmmu_v9_drvdata *data)
{
	u32 ctrl;

	writel_relaxed(data->pgtable >> PT_BASE_SHIFT,
		       data->sfrbase + REG_MMU_CONTEXT0_CFG_FLPT_BASE_VM);
	samsung_sysmmu_v9_init_config(data);

	ctrl = readl_relaxed(data->sfrbase + REG_MMU_CTRL_VM);
	ctrl |= MMU_CTRL_ENABLE;
	writel_relaxed(ctrl, data->sfrbase + REG_MMU_CTRL_VM);
	samsung_sysmmu_v9_invalidate_all(data);
}

static void samsung_sysmmu_v9_disable(struct samsung_sysmmu_v9_drvdata *data)
{
	u32 ctrl;

	ctrl = readl_relaxed(data->sfrbase + REG_MMU_CTRL_VM);
	ctrl &= ~MMU_CTRL_ENABLE;
	writel_relaxed(ctrl, data->sfrbase + REG_MMU_CTRL_VM);
	writel_relaxed(0, data->sfrbase + REG_MMU_CONTEXT0_CFG_FLPT_BASE_VM);
	samsung_sysmmu_v9_invalidate_all(data);
}

static struct iommu_domain *
samsung_sysmmu_v9_domain_alloc_paging(struct device *dev)
{
	struct samsung_sysmmu_v9_domain *domain;
	int ret;

	if (!samsung_sysmmu_v9_dma_dev)
		return NULL;

	domain = kzalloc_obj(*domain);
	if (!domain)
		return NULL;

	domain->pgtable = iommu_alloc_pages_sz(GFP_KERNEL, LV1TABLE_SIZE);
	if (!domain->pgtable)
		goto err_free_domain;

	ret = iommu_pages_start_incoherent(domain->pgtable,
					   samsung_sysmmu_v9_dma_dev);
	if (ret)
		goto err_free_pgtable;

	domain->lv2entcnt = kcalloc(NUM_LV1ENTRIES, sizeof(*domain->lv2entcnt),
				    GFP_KERNEL);
	if (!domain->lv2entcnt)
		goto err_free_incoherent_pgtable;

	domain->dma_dev = samsung_sysmmu_v9_dma_dev;
	spin_lock_init(&domain->pgtablelock);
	samsung_sysmmu_v9_flush_pgtable(domain, domain->pgtable,
					LV1TABLE_SIZE);

	domain->domain.pgsize_bitmap = SECT_SIZE | LPAGE_SIZE | SPAGE_SIZE;
	domain->domain.geometry.aperture_start = 0;
	domain->domain.geometry.aperture_end = DMA_BIT_MASK(36);
	domain->domain.geometry.force_aperture = true;

	return &domain->domain;

err_free_incoherent_pgtable:
	iommu_pages_free_incoherent(domain->pgtable, samsung_sysmmu_v9_dma_dev);
	domain->pgtable = NULL;
err_free_pgtable:
	iommu_free_pages(domain->pgtable);
err_free_domain:
	kfree(domain);
	return NULL;
}

static void samsung_sysmmu_v9_domain_free(struct iommu_domain *iommu_domain)
{
	struct samsung_sysmmu_v9_domain *domain =
		to_samsung_sysmmu_v9_domain(iommu_domain);
	unsigned int i;

	WARN_ON(domain->group);

	for (i = 0; i < NUM_LV1ENTRIES; i++) {
		sysmmu_pte_t *sent = &domain->pgtable[i];

		if (lv1ent_page(sent))
			iommu_pages_free_incoherent(phys_to_virt(lv2table_base(sent)),
						    domain->dma_dev);
	}

	iommu_pages_free_incoherent(domain->pgtable, domain->dma_dev);
	kfree(domain->lv2entcnt);
	kfree(domain);
}

static sysmmu_pte_t *
samsung_sysmmu_v9_alloc_lv2table(struct samsung_sysmmu_v9_domain *domain,
				 gfp_t gfp)
{
	sysmmu_pte_t *pent;
	int ret;

	pent = iommu_alloc_pages_sz(gfp, LV2TABLE_SIZE);
	if (!pent)
		return ERR_PTR(-ENOMEM);

	ret = iommu_pages_start_incoherent(pent, domain->dma_dev);
	if (ret) {
		iommu_free_pages(pent);
		return ERR_PTR(ret);
	}

	samsung_sysmmu_v9_flush_pgtable(domain, pent, LV2TABLE_SIZE);

	return pent;
}

static sysmmu_pte_t *
samsung_sysmmu_v9_lv2entry_locked(struct samsung_sysmmu_v9_domain *domain,
				  sysmmu_pte_t *sent, sysmmu_iova_t iova,
				  int *pgcounter, sysmmu_pte_t *new_lv2,
				  bool *new_lv2_used)
{
	if (lv1ent_section(sent))
		return ERR_PTR(-EADDRINUSE);

	if (!lv1ent_unmapped(sent))
		return page_entry(sent, iova);

	if (!new_lv2)
		return ERR_PTR(-EAGAIN);

	*sent = make_sysmmu_pte(virt_to_phys(new_lv2), SLPD_FLAG, 0);
	*pgcounter = 0;
	*new_lv2_used = true;
	samsung_sysmmu_v9_flush_pgtable(domain, sent, sizeof(*sent));

	return page_entry(sent, iova);
}

static int samsung_sysmmu_v9_lv1set_section(struct samsung_sysmmu_v9_domain *domain,
					    sysmmu_pte_t *sent, sysmmu_iova_t iova,
					    phys_addr_t paddr, int prot,
					    int *pgcounter,
					    sysmmu_pte_t **lv2_to_free)
{
	int attr = (prot & IOMMU_CACHE) ? FLPD_SHAREABLE : 0;

	if (lv1ent_section(sent))
		return -EADDRINUSE;

	if (lv1ent_page(sent)) {
		if (*pgcounter)
			return -EADDRINUSE;

		*lv2_to_free = phys_to_virt(lv2table_base(sent));
	}

	if (prot & IOMMU_READ)
		attr |= FLPD_AP_READ;
	if (prot & IOMMU_WRITE)
		attr |= FLPD_AP_WRITE;

	*sent = make_sysmmu_pte(paddr, SECT_FLAG, attr);
	*pgcounter = NUM_LV2ENTRIES;
	samsung_sysmmu_v9_flush_pgtable(domain, sent, sizeof(*sent));

	return 0;
}

static int samsung_sysmmu_v9_lv2set_page(struct samsung_sysmmu_v9_domain *domain,
					 sysmmu_pte_t *pent, phys_addr_t paddr,
					 size_t size, int prot, int *pgcounter)
{
	int attr = (prot & IOMMU_CACHE) ? SLPD_SHAREABLE : 0;
	unsigned int i;

	if (prot & IOMMU_READ)
		attr |= SLPD_AP_READ;
	if (prot & IOMMU_WRITE)
		attr |= SLPD_AP_WRITE;

	if (size == SPAGE_SIZE) {
		if (!lv2ent_unmapped(pent))
			return -EADDRINUSE;

		*pent = make_sysmmu_pte(paddr, SPAGE_FLAG, attr);
		(*pgcounter)++;
		samsung_sysmmu_v9_flush_pgtable(domain, pent, sizeof(*pent));
		return 0;
	}

	for (i = 0; i < SPAGES_PER_LPAGE; i++) {
		if (!lv2ent_unmapped(pent + i)) {
			memset(pent, 0, sizeof(*pent) * i);
			samsung_sysmmu_v9_flush_pgtable(domain, pent,
							sizeof(*pent) * i);
			return -EADDRINUSE;
		}
		*(pent + i) = make_sysmmu_pte(paddr, LPAGE_FLAG, attr);
	}

	*pgcounter += SPAGES_PER_LPAGE;
	samsung_sysmmu_v9_flush_pgtable(domain, pent,
					sizeof(*pent) * SPAGES_PER_LPAGE);

	return 0;
}

static int samsung_sysmmu_v9_map_one_locked(struct samsung_sysmmu_v9_domain *domain,
					    sysmmu_iova_t iova, phys_addr_t paddr,
					    size_t size, int prot,
					    sysmmu_pte_t *new_lv2,
					    bool *new_lv2_used,
					    sysmmu_pte_t **lv2_to_free)
{
	int *lv2entcnt = &domain->lv2entcnt[lv1ent_offset(iova)];
	sysmmu_pte_t *entry;

	if (size != SECT_SIZE && size != LPAGE_SIZE && size != SPAGE_SIZE)
		return -EINVAL;

	entry = section_entry(domain->pgtable, iova);
	if (size == SECT_SIZE)
		return samsung_sysmmu_v9_lv1set_section(domain, entry, iova,
							paddr, prot, lv2entcnt,
							lv2_to_free);

	entry = samsung_sysmmu_v9_lv2entry_locked(domain, entry, iova,
						  lv2entcnt, new_lv2,
						  new_lv2_used);
	if (IS_ERR(entry))
		return PTR_ERR(entry);

	return samsung_sysmmu_v9_lv2set_page(domain, entry, paddr, size, prot,
					     lv2entcnt);
}

static int samsung_sysmmu_v9_map_pages(struct iommu_domain *iommu_domain,
				       unsigned long l_iova, phys_addr_t paddr,
				       size_t pgsize, size_t pgcount, int prot,
				       gfp_t gfp, size_t *mapped)
{
	struct samsung_sysmmu_v9_domain *domain =
		to_samsung_sysmmu_v9_domain(iommu_domain);
	sysmmu_iova_t iova = l_iova;
	unsigned long flags;
	size_t done = 0;
	int ret = 0;

	prot &= IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE;

	while (pgcount--) {
		sysmmu_pte_t *lv2_to_free = NULL;
		sysmmu_pte_t *new_lv2 = NULL;
		bool new_lv2_used = false;

retry:
		spin_lock_irqsave(&domain->pgtablelock, flags);
		ret = samsung_sysmmu_v9_map_one_locked(domain, iova + done,
						       paddr + done, pgsize, prot,
						       new_lv2, &new_lv2_used,
						       &lv2_to_free);
		spin_unlock_irqrestore(&domain->pgtablelock, flags);

		if (ret == -EAGAIN) {
			new_lv2 = samsung_sysmmu_v9_alloc_lv2table(domain, gfp);
			if (IS_ERR(new_lv2)) {
				ret = PTR_ERR(new_lv2);
				break;
			}
			goto retry;
		}

		if (new_lv2 && !new_lv2_used)
			iommu_pages_free_incoherent(new_lv2, domain->dma_dev);
		if (lv2_to_free) {
			samsung_sysmmu_v9_sync_range(domain, iova + done,
						     iova + done + pgsize - 1);
			iommu_pages_free_incoherent(lv2_to_free, domain->dma_dev);
		}
		if (ret)
			break;
		done += pgsize;
	}

	*mapped = done;
	return ret;
}

static size_t samsung_sysmmu_v9_unmap_one(struct samsung_sysmmu_v9_domain *domain,
					  sysmmu_iova_t iova, size_t size)
{
	int *lv2entcnt = &domain->lv2entcnt[lv1ent_offset(iova)];
	sysmmu_pte_t *sent;
	sysmmu_pte_t *pent;

	sent = section_entry(domain->pgtable, iova);
	if (lv1ent_unmapped(sent))
		return min_t(size_t, size, SECT_SIZE);

	if (lv1ent_section(sent)) {
		if (size < SECT_SIZE)
			return 0;

		*sent = 0;
		*lv2entcnt = 0;
		samsung_sysmmu_v9_flush_pgtable(domain, sent, sizeof(*sent));
		return SECT_SIZE;
	}

	pent = page_entry(sent, iova);
	if (lv2ent_unmapped(pent))
		return min_t(size_t, size, SPAGE_SIZE);

	if (lv2ent_small(pent)) {
		*pent = 0;
		(*lv2entcnt)--;
		samsung_sysmmu_v9_flush_pgtable(domain, pent, sizeof(*pent));
		return SPAGE_SIZE;
	}

	if (size < LPAGE_SIZE)
		return 0;

	memset(pent, 0, sizeof(*pent) * SPAGES_PER_LPAGE);
	*lv2entcnt -= SPAGES_PER_LPAGE;
	samsung_sysmmu_v9_flush_pgtable(domain, pent,
					sizeof(*pent) * SPAGES_PER_LPAGE);

	return LPAGE_SIZE;
}

static size_t samsung_sysmmu_v9_unmap_pages(struct iommu_domain *iommu_domain,
					    unsigned long iova, size_t pgsize,
					    size_t pgcount,
					    struct iommu_iotlb_gather *gather)
{
	struct samsung_sysmmu_v9_domain *domain =
		to_samsung_sysmmu_v9_domain(iommu_domain);
	unsigned long flags;
	size_t unmapped = 0;

	spin_lock_irqsave(&domain->pgtablelock, flags);
	while (pgcount--) {
		size_t size = samsung_sysmmu_v9_unmap_one(domain,
							  iova + unmapped,
							  pgsize);

		if (!size)
			break;

		iommu_iotlb_gather_add_page(iommu_domain, gather,
					    iova + unmapped, size);
		unmapped += size;
	}
	spin_unlock_irqrestore(&domain->pgtablelock, flags);

	return unmapped;
}

static phys_addr_t samsung_sysmmu_v9_iova_to_phys(struct iommu_domain *iommu_domain,
						  dma_addr_t d_iova)
{
	struct samsung_sysmmu_v9_domain *domain =
		to_samsung_sysmmu_v9_domain(iommu_domain);
	sysmmu_iova_t iova = d_iova;
	sysmmu_pte_t *entry;
	phys_addr_t phys = 0;
	unsigned long flags;

	spin_lock_irqsave(&domain->pgtablelock, flags);

	entry = section_entry(domain->pgtable, iova);
	if (lv1ent_section(entry)) {
		phys = pte_to_phys(*entry, SECT_ENT_MASK) + (iova & (SECT_SIZE - 1));
	} else if (lv1ent_page(entry)) {
		entry = page_entry(entry, iova);
		if (lv2ent_large(entry))
			phys = pte_to_phys(*entry, LPAGE_ENT_MASK) +
			       (iova & (LPAGE_SIZE - 1));
		else if (lv2ent_small(entry))
			phys = pte_to_phys(*entry, SPAGE_ENT_MASK) +
			       (iova & (SPAGE_SIZE - 1));
	}

	spin_unlock_irqrestore(&domain->pgtablelock, flags);

	return phys;
}

static void samsung_sysmmu_v9_detach_data(struct samsung_sysmmu_v9_drvdata *data)
{
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);
	if (!data->attached_count) {
		spin_unlock_irqrestore(&data->lock, flags);
		return;
	}

	if (--data->attached_count == 0) {
		if (data->rpm_active)
			samsung_sysmmu_v9_disable(data);
		list_del_init(&data->domain_node);
		data->pgtable = 0;
		data->attached_domain = NULL;
		data->master = NULL;
	}
	spin_unlock_irqrestore(&data->lock, flags);
}

static void samsung_sysmmu_v9_detach_dev(struct iommu_domain *iommu_domain,
					 struct device *dev)
{
	struct samsung_sysmmu_v9_domain *domain =
		to_samsung_sysmmu_v9_domain(iommu_domain);
	struct samsung_sysmmu_v9_client *client = dev_iommu_priv_get(dev);
	unsigned int i;

	if (!client)
		return;

	for (i = 0; i < client->count; i++)
		samsung_sysmmu_v9_detach_data(client->sysmmus[i]);

	domain->group = NULL;
}

static int samsung_sysmmu_v9_attach_dev(struct iommu_domain *iommu_domain,
					struct device *dev,
					struct iommu_domain *old)
{
	struct samsung_sysmmu_v9_domain *domain =
		to_samsung_sysmmu_v9_domain(iommu_domain);
	struct samsung_sysmmu_v9_client *client = dev_iommu_priv_get(dev);
	struct iommu_group *group = dev->iommu_group;
	struct list_head *group_list;
	phys_addr_t pgtable = virt_to_phys(domain->pgtable);
	unsigned int i;

	if (!client || !client->count)
		return -ENODEV;

	if (old && old != iommu_domain &&
	    old->ops == samsung_sysmmu_v9_ops.default_domain_ops)
		samsung_sysmmu_v9_detach_dev(old, dev);

	if (!group)
		return -ENODEV;
	if (domain->group && domain->group != group)
		return -EBUSY;

	group_list = iommu_group_get_iommudata(group);
	if (!group_list)
		return -EPROBE_DEFER;

	for (i = 0; i < client->count; i++) {
		struct samsung_sysmmu_v9_drvdata *data = client->sysmmus[i];
		unsigned long flags;

		spin_lock_irqsave(&data->lock, flags);
		if (data->attached_count++ == 0) {
			data->pgtable = pgtable;
			data->attached_domain = iommu_domain;
			data->master = dev;
			list_add_tail(&data->domain_node, group_list);
			if (data->rpm_active)
				samsung_sysmmu_v9_enable(data);
		} else if (data->pgtable != pgtable) {
			data->attached_count--;
			spin_unlock_irqrestore(&data->lock, flags);
			goto err_detach;
		}
		spin_unlock_irqrestore(&data->lock, flags);
	}

	domain->group = group;
	dev_dbg(dev, "attached Samsung SysMMU v9 pgtable %pa\n", &pgtable);

	return 0;

err_detach:
	while (i--)
		samsung_sysmmu_v9_detach_data(client->sysmmus[i]);
	return -EBUSY;
}

static void samsung_sysmmu_v9_flush_iotlb_all(struct iommu_domain *iommu_domain)
{
	struct samsung_sysmmu_v9_domain *domain =
		to_samsung_sysmmu_v9_domain(iommu_domain);
	struct samsung_sysmmu_v9_drvdata *data;
	struct list_head *group_list;
	unsigned long flags;

	if (!domain->group)
		return;

	group_list = iommu_group_get_iommudata(domain->group);
	if (!group_list)
		return;

	list_for_each_entry(data, group_list, domain_node) {
		spin_lock_irqsave(&data->lock, flags);
		if (data->attached_count && data->rpm_active)
			samsung_sysmmu_v9_invalidate_all(data);
		spin_unlock_irqrestore(&data->lock, flags);
	}
}

static int samsung_sysmmu_v9_iotlb_sync_map(struct iommu_domain *iommu_domain,
					    unsigned long iova, size_t size)
{
	return 0;
}

static void samsung_sysmmu_v9_iotlb_sync(struct iommu_domain *iommu_domain,
					 struct iommu_iotlb_gather *gather)
{
	struct samsung_sysmmu_v9_domain *domain =
		to_samsung_sysmmu_v9_domain(iommu_domain);

	if (gather->start > gather->end)
		return;

	samsung_sysmmu_v9_sync_range(domain, gather->start, gather->end);
}

static struct iommu_device *samsung_sysmmu_v9_probe_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct samsung_sysmmu_v9_client *client = dev_iommu_priv_get(dev);
	unsigned int i;

	if (!fwspec)
		return ERR_PTR(-ENODEV);
	if (!client || !client->count)
		return ERR_PTR(-ENODEV);

	if (client->links)
		return &client->sysmmus[0]->iommu;

	client->links = kcalloc(client->count, sizeof(*client->links),
				GFP_KERNEL);
	if (!client->links)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < client->count; i++) {
		client->links[i] =
			device_link_add(dev, client->sysmmus[i]->dev,
					DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME);
		if (!client->links[i])
			goto err_links;
	}

	return &client->sysmmus[0]->iommu;

err_links:
	while (i--)
		device_link_del(client->links[i]);
	kfree(client->links);
	client->links = NULL;
	return ERR_PTR(-EINVAL);
}

static void samsung_sysmmu_v9_release_device(struct device *dev)
{
	struct samsung_sysmmu_v9_client *client = dev_iommu_priv_get(dev);
	unsigned int i;

	if (!client)
		return;

	for (i = 0; i < client->count; i++)
		if (client->links && client->links[i])
			device_link_del(client->links[i]);

	kfree(client->links);
	kfree(client->sysmmus);
	kfree(client);
	dev_iommu_priv_set(dev, NULL);
}

static void samsung_sysmmu_v9_group_data_release(void *iommu_data)
{
	kfree(iommu_data);
}

static struct iommu_group *samsung_sysmmu_v9_device_group(struct device *dev)
{
	static DEFINE_MUTEX(group_data_lock);
	struct device_node *np;
	struct platform_device *pdev;
	struct iommu_group *group;
	struct list_head *list;

	np = of_parse_phandle(dev->of_node, "samsung,iommu-group", 0);
	if (np) {
		pdev = of_find_device_by_node(np);
		of_node_put(np);
		if (!pdev)
			return ERR_PTR(-EPROBE_DEFER);

		group = platform_get_drvdata(pdev);
		platform_device_put(pdev);
		if (!group)
			return ERR_PTR(-EPROBE_DEFER);

		group = iommu_group_ref_get(group);
	} else {
		group = generic_device_group(dev);
		if (IS_ERR(group))
			return group;
	}

	mutex_lock(&group_data_lock);
	if (iommu_group_get_iommudata(group)) {
		mutex_unlock(&group_data_lock);
		return group;
	}

	list = kzalloc(sizeof(*list), GFP_KERNEL);
	if (!list) {
		mutex_unlock(&group_data_lock);
		iommu_group_put(group);
		return ERR_PTR(-ENOMEM);
	}

	INIT_LIST_HEAD(list);
	iommu_group_set_iommudata(group, list,
				  samsung_sysmmu_v9_group_data_release);
	mutex_unlock(&group_data_lock);

	return group;
}

static int samsung_sysmmu_v9_of_xlate(struct device *dev,
				      const struct of_phandle_args *args)
{
	struct platform_device *pdev;
	struct samsung_sysmmu_v9_drvdata *data;
	struct samsung_sysmmu_v9_client *client;
	struct samsung_sysmmu_v9_drvdata **sysmmus;
	unsigned int i;

	pdev = of_find_device_by_node(args->np);
	if (!pdev)
		return -EPROBE_DEFER;

	data = platform_get_drvdata(pdev);
	platform_device_put(pdev);
	if (!data)
		return -EPROBE_DEFER;

	client = dev_iommu_priv_get(dev);
	if (!client) {
		/*
		 * of_xlate runs before the client device probes; attaching
		 * devres to it here would make really_probe() fail with
		 * "Resources present before probing", so allocate plainly
		 * and free in release_device.
		 */
		client = kzalloc(sizeof(*client), GFP_KERNEL);
		if (!client)
			return -ENOMEM;
		dev_iommu_priv_set(dev, client);
	}

	for (i = 0; i < client->count; i++)
		if (client->sysmmus[i] == data)
			return 0;

	sysmmus = krealloc_array(client->sysmmus, client->count + 1,
				 sizeof(*client->sysmmus), GFP_KERNEL);
	if (!sysmmus)
		return -ENOMEM;

	client->sysmmus = sysmmus;
	client->sysmmus[client->count++] = data;

	return 0;
}

static bool samsung_sysmmu_v9_capable(struct device *dev, enum iommu_cap cap)
{
	return cap == IOMMU_CAP_CACHE_COHERENCY;
}

static const struct iommu_ops samsung_sysmmu_v9_ops = {
	.owner = THIS_MODULE,
	.capable = samsung_sysmmu_v9_capable,
	.domain_alloc_paging = samsung_sysmmu_v9_domain_alloc_paging,
	.probe_device = samsung_sysmmu_v9_probe_device,
	.release_device = samsung_sysmmu_v9_release_device,
	.device_group = samsung_sysmmu_v9_device_group,
	.get_resv_regions = of_iommu_get_resv_regions,
	.of_xlate = samsung_sysmmu_v9_of_xlate,
	.default_domain_ops = &(const struct iommu_domain_ops) {
		.attach_dev = samsung_sysmmu_v9_attach_dev,
		.map_pages = samsung_sysmmu_v9_map_pages,
		.unmap_pages = samsung_sysmmu_v9_unmap_pages,
		.flush_iotlb_all = samsung_sysmmu_v9_flush_iotlb_all,
		.iotlb_sync_map = samsung_sysmmu_v9_iotlb_sync_map,
		.iotlb_sync = samsung_sysmmu_v9_iotlb_sync,
		.iova_to_phys = samsung_sysmmu_v9_iova_to_phys,
		.free = samsung_sysmmu_v9_domain_free,
	},
};

static int samsung_sysmmu_v9_read_u32_compat(struct device_node *np,
					     const char *new_name,
					     const char *old_name,
					     u32 *value)
{
	if (!of_property_read_u32(np, new_name, value))
		return 0;

	return of_property_read_u32(np, old_name, value);
}

static bool samsung_sysmmu_v9_property_present_compat(struct device_node *np,
						      const char *new_name,
						      const char *old_name)
{
	return of_property_read_bool(np, new_name) ||
	       of_property_read_bool(np, old_name);
}

static int samsung_sysmmu_v9_parse_stream_property(struct device *dev,
						   struct samsung_sysmmu_v9_drvdata *data,
						   unsigned int pmmu_id)
{
	struct samsung_sysmmu_v9_stream_props *props = &data->props[pmmu_id];
	struct samsung_sysmmu_v9_stream_config *cfg;
	char new_name[48];
	char old_name[48];
	const char *prop_name;
	unsigned int num_stream;
	unsigned int i;
	u32 pmmu;
	int count;
	int ret;

	props->default_cfg = SYSMMU_DEFAULT_STREAM_NONE;

	snprintf(new_name, sizeof(new_name), "samsung,pmmu%u-default-stream",
		 pmmu_id);
	snprintf(old_name, sizeof(old_name), "pmmu%u,default_stream", pmmu_id);
	samsung_sysmmu_v9_read_u32_compat(dev->of_node, new_name, old_name,
					  &props->default_cfg);

	snprintf(new_name, sizeof(new_name), "samsung,pmmu%u-stream-property",
		 pmmu_id);
	snprintf(old_name, sizeof(old_name), "pmmu%u,stream_property", pmmu_id);
	prop_name = of_find_property(dev->of_node, new_name, NULL) ? new_name :
							       old_name;

	count = of_property_count_elems_of_size(dev->of_node, prop_name,
						sizeof(*cfg));
	if (count <= 0)
		return 0;

	cfg = devm_kcalloc(dev, count, sizeof(*cfg), GFP_KERNEL);
	if (!cfg)
		return -ENOMEM;

	ret = of_property_read_variable_u32_array(dev->of_node, prop_name,
						  (u32 *)cfg,
						  count * sizeof(*cfg) /
						  sizeof(u32),
						  count * sizeof(*cfg) /
						  sizeof(u32));
	if (ret < 0)
		return ret;

	writel_relaxed(MMU_SET_PMMU_INDICATOR(pmmu_id),
		       data->sfrbase + REG_MMU_PMMU_INDICATOR);
	readl_relaxed(data->sfrbase + REG_MMU_PMMU_INDICATOR);
	pmmu = readl_relaxed(data->sfrbase + REG_MMU_PMMU_INFO);
	num_stream = MMU_PMMU_INFO_NUM_STREAM_TABLE(pmmu);

	for (i = 0; i < count; i++)
		if (cfg[i].index >= num_stream)
			cfg[i].index = SYSMMU_UNUSED_STREAM_INDEX;

	props->count = count;
	props->cfg = cfg;

	return 0;
}

static int samsung_sysmmu_v9_parse_dt(struct device *dev,
				      struct samsung_sysmmu_v9_drvdata *data)
{
	u32 value;
	unsigned int i;
	int ret;

	data->qos = SYSMMU_DEFAULT_QOS;
	if (!samsung_sysmmu_v9_read_u32_compat(dev->of_node, "samsung,qos",
					       "qos", &value) &&
	    value <= 15)
		data->qos = value;

	data->secure_irq =
		samsung_sysmmu_v9_property_present_compat(dev->of_node,
							  "samsung,secure-irq",
							  "sysmmu,secure-irq");

	if (!samsung_sysmmu_v9_read_u32_compat(dev->of_node,
					       "samsung,secure-base",
					       "sysmmu,secure_base",
					       &value))
		data->secure_base = value;

	of_property_read_string(dev->of_node, "samsung,port-name",
				&data->port_name);
	if (!data->port_name)
		of_property_read_string(dev->of_node, "port-name",
					&data->port_name);

	data->props = devm_kcalloc(dev, data->num_pmmu, sizeof(*data->props),
				   GFP_KERNEL);
	if (!data->props)
		return -ENOMEM;

	for (i = 0; i < data->num_pmmu; i++) {
		ret = samsung_sysmmu_v9_parse_stream_property(dev, data, i);
		if (ret)
			return ret;
	}

	return 0;
}

static int samsung_sysmmu_v9_get_hw_info(struct samsung_sysmmu_v9_drvdata *data)
{
	struct device *dev = data->dev;
	u32 reg;
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;

	data->version = MMU_VERSION_RAW(readl_relaxed(data->sfrbase +
						     REG_MMU_VERSION));
	data->max_vm = MMU_NUM_CONTEXT(readl_relaxed(data->sfrbase +
						    REG_MMU_NUM_CONTEXT));
	data->num_pmmu =
		MMU_SWALKER_INFO_NUM_PMMU(readl_relaxed(data->sfrbase +
							REG_MMU_SWALKER_INFO));

	if (!data->max_vm || data->num_pmmu > SYSMMU_MAX_PMMUS) {
		ret = -ENODEV;
		goto out_put;
	}

	reg = readl_relaxed(data->sfrbase +
			    REG_MMU_NUM_CONTEXT_PER_VM(SYSMMU_VM0));
	if (!MMU_NUM_CONTEXT_PER_VM(reg))
		ret = -ENODEV;

out_put:
	pm_runtime_put(dev);
	return ret;
}

static const char * const samsung_sysmmu_v9_fault_names[] = {
	"page-table walk access fault",
	"page fault",
	"access fault",
	"context fault",
	"unknown fault",
};

static irqreturn_t samsung_sysmmu_v9_irq_thread(int irq, void *dev_id)
{
	struct samsung_sysmmu_v9_drvdata *data = dev_id;
	struct iommu_domain *domain;
	struct device *master;
	phys_addr_t pgtable;
	unsigned long flags;
	u32 status, info0, info1, info2;
	u64 va;

	spin_lock_irqsave(&data->lock, flags);

	if (!data->rpm_active) {
		spin_unlock_irqrestore(&data->lock, flags);
		return IRQ_NONE;
	}

	status = readl_relaxed(data->sfrbase + REG_MMU_FAULT_STATUS_VM) &
		 MMU_FAULT_STATUS_MASK;
	if (!status) {
		spin_unlock_irqrestore(&data->lock, flags);
		dev_err_ratelimited(data->dev, "spurious fault interrupt\n");
		return IRQ_NONE;
	}

	va = readl_relaxed(data->sfrbase + REG_MMU_FAULT_VA_VM);
	info0 = readl_relaxed(data->sfrbase + REG_MMU_FAULT_INFO0_VM);
	info1 = readl_relaxed(data->sfrbase + REG_MMU_FAULT_INFO1_VM);
	info2 = readl_relaxed(data->sfrbase + REG_MMU_FAULT_INFO2_VM);
	if (info0 & MMU_FAULT_INFO0_VA36)
		va |= MMU_FAULT_INFO0_VA_HIGH(info0);

	/*
	 * Clearing the fault releases the stalled transaction; with an
	 * unchanged page table it simply faults again.  Leaving it stalled is
	 * not an option: a transaction parked on the AXI port deadlocks the
	 * interconnect as soon as the client block is reset underneath it.
	 */
	writel(status, data->sfrbase + REG_MMU_FAULT_CLEAR_VM);

	domain = data->attached_domain;
	master = data->master;
	pgtable = data->pgtable;

	spin_unlock_irqrestore(&data->lock, flags);

	dev_err_ratelimited(data->dev,
			    "%s %s at %#llx (AXI ID %#x, PMMU %u, stream ID %#x, pgtable %pa)\n",
			    str_read_write(!(info0 & MMU_FAULT_INFO0_WRITE)),
			    samsung_sysmmu_v9_fault_names[__ffs(status)], va,
			    info1, MMU_FAULT_INFO2_PMMU_ID(info2),
			    MMU_FAULT_INFO2_STREAM_ID(info2), &pgtable);

	if (domain)
		report_iommu_fault(domain, master, va,
				   (info0 & MMU_FAULT_INFO0_WRITE) ?
				   IOMMU_FAULT_WRITE : IOMMU_FAULT_READ);

	return IRQ_HANDLED;
}

static int samsung_sysmmu_v9_probe(struct platform_device *pdev)
{
	struct samsung_sysmmu_v9_drvdata *data;
	struct device *dev = &pdev->dev;
	int irq;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/*
	 * The page-table walker takes 36-bit physical addresses (the FLPT
	 * base register holds a 24-bit PPN); page tables are mapped through
	 * this device and must not bounce, so the default 32-bit platform
	 * mask is not enough.
	 */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));
	if (ret)
		return ret;

	data->sfrbase = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(data->sfrbase))
		return PTR_ERR(data->sfrbase);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_threaded_irq(dev, irq, NULL,
					samsung_sysmmu_v9_irq_thread,
					IRQF_ONESHOT, dev_name(dev), data);
	if (ret)
		return ret;

	data->clk = devm_clk_get_optional(dev, "gate");
	if (IS_ERR(data->clk))
		return PTR_ERR(data->clk);

	data->dev = dev;
	INIT_LIST_HEAD(&data->domain_node);
	spin_lock_init(&data->lock);
	platform_set_drvdata(pdev, data);

	if (!samsung_sysmmu_v9_dma_dev)
		samsung_sysmmu_v9_dma_dev = dev;

	pm_runtime_enable(dev);
	ret = samsung_sysmmu_v9_get_hw_info(data);
	if (ret)
		goto err_pm_disable;

	ret = samsung_sysmmu_v9_parse_dt(dev, data);
	if (ret)
		goto err_pm_disable;

	ret = iommu_device_sysfs_add(&data->iommu, dev, NULL, dev_name(dev));
	if (ret)
		goto err_pm_disable;

	ret = iommu_device_register(&data->iommu, &samsung_sysmmu_v9_ops, dev);
	if (ret)
		goto err_sysfs_remove;

	dev_info(dev, "Samsung System MMU v9 %u.%u.%u initialized%s%s\n",
		 MMU_VERSION_MAJOR(data->version),
		 MMU_VERSION_MINOR(data->version),
		 MMU_VERSION_REVISION(data->version),
		 data->port_name ? " for " : "",
		 data->port_name ?: "");

	return 0;

err_sysfs_remove:
	iommu_device_sysfs_remove(&data->iommu);
err_pm_disable:
	samsung_sysmmu_v9_clear_dma_dev(dev);
	pm_runtime_disable(dev);
	return ret;
}

static void samsung_sysmmu_v9_remove(struct platform_device *pdev)
{
	struct samsung_sysmmu_v9_drvdata *data = platform_get_drvdata(pdev);

	iommu_device_unregister(&data->iommu);
	iommu_device_sysfs_remove(&data->iommu);
	samsung_sysmmu_v9_clear_dma_dev(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
}

static int __maybe_unused samsung_sysmmu_v9_runtime_resume(struct device *dev)
{
	struct samsung_sysmmu_v9_drvdata *data = dev_get_drvdata(dev);
	unsigned long flags;
	int ret;

	ret = clk_prepare_enable(data->clk);
	if (ret)
		return ret;

	spin_lock_irqsave(&data->lock, flags);
	data->rpm_active = true;
	if (data->attached_count)
		samsung_sysmmu_v9_enable(data);
	spin_unlock_irqrestore(&data->lock, flags);

	return 0;
}

static int __maybe_unused samsung_sysmmu_v9_runtime_suspend(struct device *dev)
{
	struct samsung_sysmmu_v9_drvdata *data = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);
	if (data->attached_count)
		samsung_sysmmu_v9_disable(data);
	data->rpm_active = false;
	spin_unlock_irqrestore(&data->lock, flags);

	clk_disable_unprepare(data->clk);

	return 0;
}

static const struct dev_pm_ops samsung_sysmmu_v9_pm_ops = {
	SET_RUNTIME_PM_OPS(samsung_sysmmu_v9_runtime_suspend,
			   samsung_sysmmu_v9_runtime_resume, NULL)
};

static const struct of_device_id samsung_sysmmu_v9_of_match[] = {
	{ .compatible = "samsung,sysmmu-v9" },
	{ }
};
MODULE_DEVICE_TABLE(of, samsung_sysmmu_v9_of_match);

static struct platform_driver samsung_sysmmu_v9_driver = {
	.probe = samsung_sysmmu_v9_probe,
	.remove = samsung_sysmmu_v9_remove,
	.driver = {
		.name = "samsung-sysmmu-v9",
		.of_match_table = samsung_sysmmu_v9_of_match,
		.pm = &samsung_sysmmu_v9_pm_ops,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(samsung_sysmmu_v9_driver);

MODULE_DESCRIPTION("Samsung System MMU v9 support");
MODULE_LICENSE("GPL");
