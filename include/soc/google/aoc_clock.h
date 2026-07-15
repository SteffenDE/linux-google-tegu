/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * AoC time-domain correlation for in-kernel clients.
 *
 * The AoC firmware timestamps its events (e.g. USF sensor samples) in its own
 * clock domain: nanoseconds off the SoC's architected counter (offset by a
 * constant the firmware publishes). CLOCK_BOOTTIME is derived from that same
 * counter, so the mapping is exact with no drift. This helper performs it for
 * drivers that consume AoC-stamped data and need Linux monotonic-boot timestamps.
 */
#ifndef _SOC_GOOGLE_AOC_CLOCK_H_
#define _SOC_GOOGLE_AOC_CLOCK_H_

#include <linux/types.h>

/**
 * aoc_ts_to_boottime_ns() - map an AoC-domain timestamp to CLOCK_BOOTTIME
 * @aoc_ts: a timestamp in the AoC clock domain: nanoseconds off the architected
 *          counter (offset by the firmware's system_clock_offset). This is the
 *          ns form of what the aoc_clock sysfs attribute reports as ticks.
 *
 * Expresses "now" in the same domain and differences the two to translate
 * @aoc_ts into CLOCK_BOOTTIME nanoseconds. Safe to call from atomic context.
 *
 * Return: CLOCK_BOOTTIME nanoseconds, or 0 if the AoC firmware is not ready or
 * the timestamp is too far from the current time to be trusted (the caller
 * should substitute the current boottime in that case).
 */
u64 aoc_ts_to_boottime_ns(u64 aoc_ts);

#endif /* _SOC_GOOGLE_AOC_CLOCK_H_ */
