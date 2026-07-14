// SPDX-License-Identifier: GPL-2.0-only
/*
 * Stub gs-chipid getters for the tegu AoC bring-up.
 *
 * The real gs-chipid driver reads the chip type/revision/product-id (and the
 * AP HW-tune array) from the chipid block, and AoC forwards them to firmware
 * via fw_data.  Until it is ported these return zero: AoC boots and enumerates
 * its services; chip-specific tuning is simply not applied.  Replace with a
 * real read (chipid block or the mainline exynos-chipid soc_device) if AoC
 * turns out to need the true values.  Declarations live in aoc.h / aoc.c.
 */
#include <linux/types.h>

u32 gs_chipid_get_revision(void) { return 0; }
u32 gs_chipid_get_type(void) { return 0; }
u32 gs_chipid_get_product_id(void) { return 0; }

int gs_chipid_get_ap_hw_tune_array(const u8 **array)
{
	if (array)
		*array = NULL;
	return 0;
}
