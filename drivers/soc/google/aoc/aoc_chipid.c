// SPDX-License-Identifier: GPL-2.0-only
/*
 * gs-chipid getters for the tegu AoC bring-up.
 *
 * The AoC firmware wants the SoC product-id/type/revision, which the AP
 * forwards in fw_data as kAOCChipProductId/kAOCChipType/kAOCChipRevision.  The
 * downstream gs-chipid driver reads them from the chipid block; rather than pull
 * that driver in, this is a self-contained reader of the same registers.
 *
 * Register layout matches drv_data_zuma in the downstream gs-chipid.c (Zuma Pro
 * reuses the "google,zuma-chipid" compatible).  Base is the chipid node at
 * 0x10000000:
 *   product_id = reg(0x00) & 0xfffff000    (tegu / Zuma Pro reads 0x09875000)
 *   type       = reg(0x00) & 0x000000ff    (SoC variant, low byte; 0 on A0)
 *   revision   = (main_rev << 4) | sub_rev,
 *                main_rev = reg(0x00) & 0xf, sub_rev = (reg(0x10) >> 16) & 0xf
 * The AP-HW-tune fuse array is not consumed by the fw_data path, so it stays a
 * stub returning nothing.  Declarations live in aoc.h / aoc.c.
 */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/printk.h>
#include <linux/types.h>

/*
 * These are the getters the AoC core calls (extern-declared in aoc.h / aoc.c).
 * Declared here too so this standalone reader builds warning-clean without
 * pulling in the heavy aoc.h.
 */
u32 gs_chipid_get_revision(void);
u32 gs_chipid_get_type(void);
u32 gs_chipid_get_product_id(void);
int gs_chipid_get_ap_hw_tune_array(const u8 **array);

#define GS_CHIPID_PRODUCT_REG		0x00
#define GS_CHIPID_SUB_REV_REG		0x10
#define GS_CHIPID_PRODUCT_ID_MASK	0xfffff000
#define GS_CHIPID_TYPE_MASK		0x000000ff
#define GS_CHIPID_REV_PART_MASK		0xf
#define GS_CHIPID_SUB_REV_SHIFT		16

static u32 gs_chipid_product_id;
static u32 gs_chipid_type;
static u32 gs_chipid_revision;
static bool gs_chipid_valid;

static void gs_chipid_read(void)
{
	struct device_node *np;
	void __iomem *base;
	u32 reg0, reg10, main_rev, sub_rev;

	if (smp_load_acquire(&gs_chipid_valid))
		return;

	np = of_find_compatible_node(NULL, NULL, "google,zuma-chipid");
	if (!np) {
		pr_warn("aoc chipid: no google,zuma-chipid node; chip fw_data stays 0\n");
		return;
	}

	base = of_iomap(np, 0);
	of_node_put(np);
	if (!base) {
		pr_warn("aoc chipid: failed to map chipid block; chip fw_data stays 0\n");
		return;
	}

	reg0 = readl(base + GS_CHIPID_PRODUCT_REG);
	reg10 = readl(base + GS_CHIPID_SUB_REV_REG);
	iounmap(base);

	gs_chipid_product_id = reg0 & GS_CHIPID_PRODUCT_ID_MASK;
	gs_chipid_type = reg0 & GS_CHIPID_TYPE_MASK;
	main_rev = reg0 & GS_CHIPID_REV_PART_MASK;
	sub_rev = (reg10 >> GS_CHIPID_SUB_REV_SHIFT) & GS_CHIPID_REV_PART_MASK;
	gs_chipid_revision = (main_rev << 4) | sub_rev;
	/* Publish the values before the ready flag any later caller may observe. */
	smp_store_release(&gs_chipid_valid, true);

	pr_info("aoc chipid: product_id=0x%08x type=0x%x revision=0x%02x\n",
		gs_chipid_product_id, gs_chipid_type, gs_chipid_revision);
}

u32 gs_chipid_get_revision(void)
{
	gs_chipid_read();
	return gs_chipid_revision;
}

u32 gs_chipid_get_type(void)
{
	gs_chipid_read();
	return gs_chipid_type;
}

u32 gs_chipid_get_product_id(void)
{
	gs_chipid_read();
	return gs_chipid_product_id;
}

int gs_chipid_get_ap_hw_tune_array(const u8 **array)
{
	if (array)
		*array = NULL;
	return 0;
}
