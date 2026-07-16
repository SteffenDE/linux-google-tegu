/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google Whitechapel AoC ALSA Driver
 *
 * Copyright (c) 2019 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef AOC_ALSA_DRV_H
#define AOC_ALSA_DRV_H

#undef pr_fmt
#define pr_fmt(fmt) "alsa: " fmt

#include <linux/poll.h>
#include "aoc.h"

typedef enum {
    AOC_SERVICE_EVENT_DOWN = 0,
    AOC_SERVICE_EVENT_MAX,
} aoc_aud_service_event_t;

typedef void(*service_event_cb_t)(aoc_aud_service_event_t event, void *cookies);

struct aoc_state_client_t {
	bool inuse;
	bool online;
	bool exit;
};

int alloc_aoc_audio_service(const char *name, struct aoc_service_dev **dev,
		service_event_cb_t cb, void *cookies);
int free_aoc_audio_service(const char *name, struct aoc_service_dev *dev);
int8_t aoc_audio_service_num(void);
__poll_t aoc_audio_state_poll(struct file *f, poll_table *wait,
		struct aoc_state_client_t *client);

bool aoc_audio_current_state(void);

/*
 * Speaker-protection interlock. The speaker backend refuses to start until
 * userspace has loaded the AoC excursion/thermal limiter and armed this flag;
 * it is cleared whenever the AoC goes down so protection must be re-armed
 * before playback can resume. Paired with the AoC online state above.
 */
bool aoc_speaker_protection_is_armed(void);
void aoc_speaker_protection_set_armed(bool armed);

struct aoc_state_client_t *alloc_audio_state_client(void);
void free_audio_state_client(struct aoc_state_client_t *client);
void audio_free_isr(struct aoc_service_dev *dev);
#endif
