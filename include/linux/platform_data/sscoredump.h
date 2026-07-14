/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Minimal stub for the tegu AoC bring-up.  The Samsung subsystem-coredump
 * (sscoredump) driver is not ported.  These types let aoc.c compile; the
 * platform device it registers has no bound driver, so sscd_report stays
 * NULL and aoc.c's coredump path self-skips (it checks for a NULL report).
 */
#ifndef __LINUX_PLATFORM_DATA_SSCOREDUMP_H
#define __LINUX_PLATFORM_DATA_SSCOREDUMP_H

#include <linux/types.h>

struct platform_device;

#define SSCD_FLAGS_ELFARM64HDR	0x0001

struct sscd_segment {
	void *addr;
	size_t size;
	void *paddr;
	void *vaddr;
};

struct sscd_platform_data {
	int (*sscd_report)(struct platform_device *pdev,
			   struct sscd_segment *segs, u16 nsegs,
			   unsigned long flags, const char *crash_info);
};

#endif /* __LINUX_PLATFORM_DATA_SSCOREDUMP_H */
