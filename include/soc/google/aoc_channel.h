/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * In-kernel client API for AoC Channelized Comms (AOCC).
 *
 * The AOCC char driver (drivers/soc/google/aoc/aoc_channel_dev.c) multiplexes
 * many logical channels over a single AoC service (e.g. "com.google.usf") and
 * exposes each channel to userspace as /dev/acd-<service>. This header lets an
 * in-kernel driver open its own channel on the same service and coexist with
 * the userspace clients, receiving demultiplexed messages through a callback.
 */
#ifndef _SOC_GOOGLE_AOC_CHANNEL_H_
#define _SOC_GOOGLE_AOC_CHANNEL_H_

#include <linux/types.h>

struct aocc_channel;

/**
 * aocc_kernel_open_channel() - open an in-kernel AOCC channel on a service
 * @service_name: AOCC service name, e.g. "com.google.usf" or
 *                "com.google.usf.non_wake_up".
 * @rx: callback invoked for every message demultiplexed to this channel.
 *      @payload / @len exclude the 4-byte channel header. It runs in the
 *      service's demux kthread context under an internal lock, so it must not
 *      sleep and must not call aocc_kernel_open_channel() or
 *      aocc_kernel_close_channel().
 * @ctx: opaque context passed back to @rx.
 *
 * Return: a channel handle on success; ERR_PTR(-ENODEV) if the service has not
 * been probed yet (the caller may retry / defer); or another ERR_PTR on error.
 */
struct aocc_channel *
aocc_kernel_open_channel(const char *service_name,
			 void (*rx)(void *ctx, const void *payload, size_t len),
			 void *ctx);

/**
 * aocc_kernel_write() - send one message on an in-kernel AOCC channel
 * @chan: handle from aocc_kernel_open_channel()
 * @payload: message bytes; the channel header is prepended internally
 * @len: length of @payload
 *
 * May sleep. Return: bytes accepted (> 0) or a negative errno.
 */
int aocc_kernel_write(struct aocc_channel *chan, const void *payload,
		      size_t len);

/**
 * aocc_kernel_close_channel() - close an in-kernel AOCC channel
 * @chan: handle from aocc_kernel_open_channel() (NULL / ERR_PTR tolerated)
 */
void aocc_kernel_close_channel(struct aocc_channel *chan);

#endif /* _SOC_GOOGLE_AOC_CHANNEL_H_ */
