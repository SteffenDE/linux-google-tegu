/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2026 Steffen Deusch
 *
 * Samsung S2MPG15 PMIC (Google Tensor G4 "zumapro" sub PMIC).
 *
 * Like s2mpg14.h this is intentionally partial: only the blocks the
 * mainline driver currently uses are described.  The meter block is
 * register-identical to the S2MPG14 and is shared from s2mpg14.h
 * (S2MPG14_METER_*) by the common s2mpg1x meter driver, so it is not
 * repeated here.
 */

#ifndef __LINUX_MFD_S2MPG15_H
#define __LINUX_MFD_S2MPG15_H

/* Common registers (type 0x000) */
enum s2mpg15_common_reg {
	S2MPG15_COMMON_CHIPID = 0x0b,
	S2MPG15_COMMON_IBIM2 = 0x10,
};

/* PMIC registers (type 0x100) */
enum s2mpg15_pmic_reg {
	S2MPG15_PMIC_INT1 = 0x00,
	/* LxS_CTRL: bit 7 is the rail enable, bits 5:0 the voltage selector. */
	S2MPG15_PMIC_L5S_CTRL = 0x33,	/* L5S_PROX,    sensor 3.3 V */
	S2MPG15_PMIC_L7S_CTRL = 0x35,	/* L7S_SENSORS, sensor 1.8 V */
	S2MPG15_PMIC_BB_USONIC = 0xea,
};

/* Regulator ids -- only the sensor rails the AoC currently powers. */
enum s2mpg15_regulators {
	S2MPG15_LDO5,
	S2MPG15_LDO7,
	S2MPG15_REGULATOR_MAX,
};

#endif /* __LINUX_MFD_S2MPG15_H */
