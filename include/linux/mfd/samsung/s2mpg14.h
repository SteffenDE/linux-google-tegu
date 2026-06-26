/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2025 Steffen Deusch
 *
 * Samsung S2MPG14 PMIC (Google Tensor G4 "zumapro" main PMIC).
 *
 * Register addresses are taken from the downstream Pixel kernel header
 * s2mpg14-register.h.  This is intentionally a partial description: only
 * the blocks and rails that mainline currently uses are listed.  Extend it
 * register by register as consumers appear instead of transcribing the
 * whole downstream header at once.
 */

#ifndef __LINUX_MFD_S2MPG14_H
#define __LINUX_MFD_S2MPG14_H

#include <linux/bits.h>

/* Common registers (type 0x000) */
enum s2mpg14_common_reg {
	S2MPG14_COMMON_VGPIO0,
	S2MPG14_COMMON_VGPIO1,
	S2MPG14_COMMON_VGPIO2,
	S2MPG14_COMMON_VGPIO3,
	S2MPG14_COMMON_I3C_DAA,
	S2MPG14_COMMON_IBI0,
	S2MPG14_COMMON_IBI1,
	S2MPG14_COMMON_IBI2,
	S2MPG14_COMMON_IBI3,
	S2MPG14_COMMON_CHIPID = 0x0b,
	S2MPG14_COMMON_I3C_CFG1 = 0x0c,
	S2MPG14_COMMON_I3C_CFG2 = 0x0d,
	S2MPG14_COMMON_I3C_STA = 0x0e,
	S2MPG14_COMMON_IBIM1 = 0x0f,
	S2MPG14_COMMON_IBIM2 = 0x10,
	S2MPG14_COMMON_TEST_MODE2 = 0x25,
};

/* PMIC registers (type 0x100) */
enum s2mpg14_pmic_reg {
	S2MPG14_PMIC_INT1,
	S2MPG14_PMIC_INT2,
	S2MPG14_PMIC_INT3,
	S2MPG14_PMIC_INT4,
	S2MPG14_PMIC_INT5,
	S2MPG14_PMIC_INT1M,
	S2MPG14_PMIC_INT2M,
	S2MPG14_PMIC_INT3M,
	S2MPG14_PMIC_INT4M,
	S2MPG14_PMIC_INT5M,
	S2MPG14_PMIC_STATUS1,
	S2MPG14_PMIC_STATUS2,
	S2MPG14_PMIC_PWRONSRC,
	S2MPG14_PMIC_OFFSRC1,
	S2MPG14_PMIC_OFFSRC2,
	S2MPG14_PMIC_L4M_CTRL = 0x2e,
	S2MPG14_PMIC_L25M_CTRL = 0x43,
	S2MPG14_PMIC_SW_RESET = 0xe4,
};

/* Meter registers (type 0x00a) */
enum s2mpg14_meter_reg {
	S2MPG14_METER_CTRL1 = 0x08,
	S2MPG14_METER_CTRL2 = 0x09,
	S2MPG14_METER_CTRL4 = 0x0b,
	S2MPG14_METER_CTRL5 = 0x0c,
	S2MPG14_METER_BUCKEN1 = 0x0f,
	S2MPG14_METER_BUCKEN2 = 0x10,
	S2MPG14_METER_MUXSEL0 = 0x11,
	S2MPG14_METER_ACC_DATA_CH0_1 = 0x63,
	S2MPG14_METER_ACC_COUNT_1 = 0xab,
	S2MPG14_METER_LPF_DATA_CH0_1 = 0xae,
	S2MPG14_METER_VBAT_DATA1 = 0xd2,
	S2MPG14_METER_VBAT_DATA2 = 0xd3,
	S2MPG14_METER_EXT_SIGNED_DATA_1 = 0xe4,
	S2MPG14_METER_EXT_SIGNED_DATA_2 = 0xe5,
};

/* METER_CTRL1 */
#define S2MPG14_METER_EN_MASK		BIT(0)
#define S2MPG14_METER_INT_SAMP_RATE_SHIFT 2
#define S2MPG14_METER_INT_SAMP_RATE_MASK (0x7 << S2MPG14_METER_INT_SAMP_RATE_SHIFT)
#define S2MPG14_METER_INT_SAMP_RATE_125HZ 4

/* METER_CTRL2: write ASYNC_RD to latch the accumulators, self-clears */
#define S2MPG14_METER_ASYNC_RD_MASK	BIT(7)

/*
 * METER_CTRL4 (channels 0-7) + METER_CTRL5[3:0] (channels 8-11): per-channel
 * accumulation mode, 0 = power, 1 = current.
 */
#define S2MPG14_METER_ACC_MODE_HI_MASK	0x0f

/* The meter exposes 12 channels; data is accumulated and low-pass filtered. */
#define S2MPG14_METER_CHANNELS		12
#define S2MPG14_METER_ACC_DATA_BYTES	6
#define S2MPG14_METER_ACC_COUNT_BYTES	3
#define S2MPG14_METER_ACC_DATA_BITS	41
#define S2MPG14_METER_ACC_COUNT_BITS	20

/*
 * Regulators.  Deliberately partial: just the touchscreen rails for now.
 * Append new IDs here (and a matching descriptor in s2mps11.c) when a
 * consumer for another rail shows up.
 */
enum s2mpg14_regulators {
	S2MPG14_LDO4,
	S2MPG14_LDO25,
	S2MPG14_REGULATOR_MAX,
};

#endif /* __LINUX_MFD_S2MPG14_H */
