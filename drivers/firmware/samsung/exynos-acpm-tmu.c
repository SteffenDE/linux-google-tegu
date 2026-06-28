// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2017 Samsung Electronics Co., Ltd.
 * Copyright 2025 Google LLC.
 *
 * ACPM TMU plugin interface.
 *
 * On the Tensor (gs101/zumapro) SoCs the Thermal Management Unit is owned by
 * the ACPM firmware.  The AP does not touch the TMU registers directly; it
 * asks the firmware for a per-zone temperature over a dedicated ACPM IPC
 * channel.  This is the protocol-driver side of that exchange, modelled on
 * exynos-acpm-pmic.c.
 */
#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/types.h>

#include "exynos-acpm.h"
#include "exynos-acpm-tmu.h"

/* TMU IPC command word 1 (request and response share the layout). */
#define ACPM_TMU_TYPE			GENMASK(7, 0)
#define ACPM_TMU_RET			GENMASK(15, 8)
#define ACPM_TMU_TZID			GENMASK(23, 16)
#define ACPM_TMU_TEMP			GENMASK(31, 24)

/* TMU IPC command word 2 (response). */
#define ACPM_TMU_STAT			GENMASK(7, 0)

/* IPC request types. */
#define ACPM_TMU_IPC_READ_TEMP		0x02

/**
 * acpm_tmu_read_temp() - read one thermal zone's temperature from the firmware.
 * @handle:		ACPM handle.
 * @acpm_chan_id:	ACPM IPC channel carrying the TMU plugin.
 * @tzid:		firmware thermal-zone id.
 * @temp:		output, temperature in degrees Celsius.
 * @stat:		output, firmware status byte (may be NULL).
 *
 * Return: 0 on success, negative errno on IPC failure.
 */
int acpm_tmu_read_temp(struct acpm_handle *handle, unsigned int acpm_chan_id,
		       u8 tzid, int *temp, int *stat)
{
	struct acpm_xfer xfer;
	u32 cmd[4] = {0};
	int ret;

	cmd[1] = FIELD_PREP(ACPM_TMU_TYPE, ACPM_TMU_IPC_READ_TEMP) |
		 FIELD_PREP(ACPM_TMU_TZID, tzid);

	xfer.txd = cmd;
	xfer.rxd = cmd;
	xfer.txcnt = ARRAY_SIZE(cmd);
	xfer.rxcnt = ARRAY_SIZE(cmd);
	xfer.acpm_chan_id = acpm_chan_id;

	ret = acpm_do_xfer(handle, &xfer);
	if (ret)
		return ret;

	*temp = FIELD_GET(ACPM_TMU_TEMP, xfer.rxd[1]);
	if (stat)
		*stat = FIELD_GET(ACPM_TMU_STAT, xfer.rxd[2]);

	return 0;
}
