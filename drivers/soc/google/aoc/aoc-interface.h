/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google Whitechapel AoC Core Driver
 *
 * Copyright (c) 2021 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * tegu is zumapro (zuma family).  The tree has no per-SoC CONFIG_SOC_*
 * symbol (drivers match via DT compatible), and this driver is only built
 * for zumapro, so select the zuma interface table directly rather than
 * gating on CONFIG_SOC_ZUMA.
 */
#include "aoc-interface-zuma.h"

