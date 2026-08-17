// SPDX-License-Identifier: GPL-2.0+
//
// SLG51000/SLG51002 High PSRR, Multi-Output Regulators
// Copyright (C) 2019  Dialog Semiconductor
//
// Author: Eric Jeong <eric.jeong.opensource@diasemi.com>

#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include "slg51000-regulator.h"

#define SLG51000_LDOHP_LV_MIN           1200000
#define SLG51000_LDOHP_HV_MIN           2400000

#define SLG51000_MAX_REGULATORS         7
#define SLG51002_MAX_REGULATORS         8

/* Sized for the larger part; a given chip uses info->n_regulators of these. */
#define SLG51000_MAX_LDOS               SLG51002_MAX_REGULATORS
/* One event/status register pair per LDO, plus the system controller's. */
#define SLG51000_MAX_EVT_REGISTER       (SLG51000_MAX_LDOS + 1)

enum slg51000_regulators {
	SLG51000_REGULATOR_LDO1 = 0,
	SLG51000_REGULATOR_LDO2,
	SLG51000_REGULATOR_LDO3,
	SLG51000_REGULATOR_LDO4,
	SLG51000_REGULATOR_LDO5,
	SLG51000_REGULATOR_LDO6,
	SLG51000_REGULATOR_LDO7,
	SLG51000_REGULATOR_LDO8,
};

struct slg51000_evt_sta {
	unsigned int ereg;
	unsigned int sreg;
};

/**
 * struct slg51000_chip_info - per-variant description
 * @regmap_cfg: register map, which differs in the per-LDO tail registers
 * @rdesc: regulator descriptors, @n_regulators of them
 * @evt_sta: event/status register pairs, one per LDO then the system
 *	controller's, so @n_regulators + 1 entries
 * @min_regs: each LDO's MINV register, the first of the MINV/MAXV pair the
 *	probe reads to learn the selector range
 * @n_regulators: number of LDOs on this part
 * @ldo12_vrange: LDO1 and LDO2 are dual-range and pick 1.2 V or 2.4 V as
 *	their base from a bit in MISC1 (SLG51000 only; on the SLG51002 that
 *	address is LDO1_CONF1 and both are plain 1.2 V-based high-voltage LDOs)
 * @trim2_regs: per-LDO TRIM2 register, or 0 for an LDO that cannot be
 *	strapped into bypass.  Bypass turns an LDO into a pass-through of its
 *	input, so a driver that missed the strap would report a regulated
 *	voltage the rail does not have.
 */
struct slg51000_chip_info {
	const struct regmap_config *regmap_cfg;
	const struct regulator_desc *rdesc;
	const struct slg51000_evt_sta *evt_sta;
	const unsigned int *min_regs;
	const unsigned int *trim2_regs;
	unsigned int n_regulators;
	bool ldo12_vrange;
};

struct slg51000 {
	struct device *dev;
	struct regmap *regmap;
	const struct slg51000_chip_info *info;
	struct regulator_desc *rdesc;
	struct regulator_dev *rdev[SLG51000_MAX_LDOS];
	struct gpio_desc *cs_gpiod;
	int chip_irq;
};

static const struct slg51000_evt_sta slg51000_es_reg[] = {
	{SLG51000_LDO1_EVENT, SLG51000_LDO1_STATUS},
	{SLG51000_LDO2_EVENT, SLG51000_LDO2_STATUS},
	{SLG51000_LDO3_EVENT, SLG51000_LDO3_STATUS},
	{SLG51000_LDO4_EVENT, SLG51000_LDO4_STATUS},
	{SLG51000_LDO5_EVENT, SLG51000_LDO5_STATUS},
	{SLG51000_LDO6_EVENT, SLG51000_LDO6_STATUS},
	{SLG51000_LDO7_EVENT, SLG51000_LDO7_STATUS},
	{SLG51000_SYSCTL_EVENT, SLG51000_SYSCTL_STATUS},
};

static const struct slg51000_evt_sta slg51002_es_reg[] = {
	{SLG51000_LDO1_EVENT, SLG51000_LDO1_STATUS},
	{SLG51000_LDO2_EVENT, SLG51000_LDO2_STATUS},
	{SLG51000_LDO3_EVENT, SLG51000_LDO3_STATUS},
	{SLG51000_LDO4_EVENT, SLG51000_LDO4_STATUS},
	{SLG51000_LDO5_EVENT, SLG51000_LDO5_STATUS},
	{SLG51000_LDO6_EVENT, SLG51000_LDO6_STATUS},
	{SLG51000_LDO7_EVENT, SLG51000_LDO7_STATUS},
	{SLG51002_LDO8_EVENT, SLG51002_LDO8_STATUS},
	{SLG51000_SYSCTL_EVENT, SLG51000_SYSCTL_STATUS},
};

static const unsigned int slg51000_min_regs[] = {
	SLG51000_LDO1_MINV, SLG51000_LDO2_MINV, SLG51000_LDO3_MINV,
	SLG51000_LDO4_MINV, SLG51000_LDO5_MINV, SLG51000_LDO6_MINV,
	SLG51000_LDO7_MINV,
};

/*
 * The low-voltage LDOs are the bypass-capable ones: LDO5 and LDO6 on the
 * SLG51000, LDO6 through LDO8 on the SLG51002.
 */
static const unsigned int slg51000_trim2_regs[SLG51000_MAX_REGULATORS] = {
	[SLG51000_REGULATOR_LDO5] = SLG51000_LDO5_TRIM2,
	[SLG51000_REGULATOR_LDO6] = SLG51000_LDO6_TRIM2,
};

static const unsigned int slg51002_trim2_regs[SLG51002_MAX_REGULATORS] = {
	[SLG51000_REGULATOR_LDO6] = SLG51002_LDO6_TRIM2,
	[SLG51000_REGULATOR_LDO7] = SLG51002_LDO7_TRIM2,
	[SLG51000_REGULATOR_LDO8] = SLG51002_LDO8_TRIM2,
};

static const unsigned int slg51002_min_regs[] = {
	SLG51000_LDO1_MINV, SLG51000_LDO2_MINV, SLG51000_LDO3_MINV,
	SLG51000_LDO4_MINV, SLG51000_LDO5_MINV, SLG51000_LDO6_MINV,
	SLG51000_LDO7_MINV, SLG51002_LDO8_MINV,
};

static const struct regmap_range slg51000_writeable_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_MATRIX_CONF_A,
			 SLG51000_SYSCTL_MATRIX_CONF_A),
	regmap_reg_range(SLG51000_LDO1_VSEL, SLG51000_LDO1_VSEL),
	regmap_reg_range(SLG51000_LDO1_MINV, SLG51000_LDO1_MAXV),
	regmap_reg_range(SLG51000_LDO1_IRQ_MASK, SLG51000_LDO1_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO2_VSEL, SLG51000_LDO2_VSEL),
	regmap_reg_range(SLG51000_LDO2_MINV, SLG51000_LDO2_MAXV),
	regmap_reg_range(SLG51000_LDO2_IRQ_MASK, SLG51000_LDO2_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO3_VSEL, SLG51000_LDO3_VSEL),
	regmap_reg_range(SLG51000_LDO3_MINV, SLG51000_LDO3_MAXV),
	regmap_reg_range(SLG51000_LDO3_IRQ_MASK, SLG51000_LDO3_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO4_VSEL, SLG51000_LDO4_VSEL),
	regmap_reg_range(SLG51000_LDO4_MINV, SLG51000_LDO4_MAXV),
	regmap_reg_range(SLG51000_LDO4_IRQ_MASK, SLG51000_LDO4_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO5_VSEL, SLG51000_LDO5_VSEL),
	regmap_reg_range(SLG51000_LDO5_MINV, SLG51000_LDO5_MAXV),
	regmap_reg_range(SLG51000_LDO5_IRQ_MASK, SLG51000_LDO5_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO6_VSEL, SLG51000_LDO6_VSEL),
	regmap_reg_range(SLG51000_LDO6_MINV, SLG51000_LDO6_MAXV),
	regmap_reg_range(SLG51000_LDO6_IRQ_MASK, SLG51000_LDO6_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO7_VSEL, SLG51000_LDO7_VSEL),
	regmap_reg_range(SLG51000_LDO7_MINV, SLG51000_LDO7_MAXV),
	regmap_reg_range(SLG51000_LDO7_IRQ_MASK, SLG51000_LDO7_IRQ_MASK),
	regmap_reg_range(SLG51000_OTP_IRQ_MASK, SLG51000_OTP_IRQ_MASK),
};

static const struct regmap_range slg51000_readable_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_PATN_ID_B0,
			 SLG51000_SYSCTL_PATN_ID_B2),
	regmap_reg_range(SLG51000_SYSCTL_SYS_CONF_A,
			 SLG51000_SYSCTL_SYS_CONF_A),
	regmap_reg_range(SLG51000_SYSCTL_SYS_CONF_D,
			 SLG51000_SYSCTL_MATRIX_CONF_B),
	regmap_reg_range(SLG51000_SYSCTL_REFGEN_CONF_C,
			 SLG51000_SYSCTL_UVLO_CONF_A),
	regmap_reg_range(SLG51000_SYSCTL_FAULT_LOG1, SLG51000_SYSCTL_IRQ_MASK),
	regmap_reg_range(SLG51000_IO_GPIO1_CONF, SLG51000_IO_GPIO_STATUS),
	regmap_reg_range(SLG51000_LUTARRAY_LUT_VAL_0,
			 SLG51000_LUTARRAY_LUT_VAL_11),
	regmap_reg_range(SLG51000_MUXARRAY_INPUT_SEL_0,
			 SLG51000_MUXARRAY_INPUT_SEL_63),
	regmap_reg_range(SLG51000_PWRSEQ_RESOURCE_EN_0,
			 SLG51000_PWRSEQ_INPUT_SENSE_CONF_B),
	regmap_reg_range(SLG51000_LDO1_VSEL, SLG51000_LDO1_VSEL),
	regmap_reg_range(SLG51000_LDO1_MINV, SLG51000_LDO1_MAXV),
	regmap_reg_range(SLG51000_LDO1_MISC1, SLG51000_LDO1_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO1_EVENT, SLG51000_LDO1_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO2_VSEL, SLG51000_LDO2_VSEL),
	regmap_reg_range(SLG51000_LDO2_MINV, SLG51000_LDO2_MAXV),
	regmap_reg_range(SLG51000_LDO2_MISC1, SLG51000_LDO2_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO2_EVENT, SLG51000_LDO2_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO3_VSEL, SLG51000_LDO3_VSEL),
	regmap_reg_range(SLG51000_LDO3_MINV, SLG51000_LDO3_MAXV),
	regmap_reg_range(SLG51000_LDO3_CONF1, SLG51000_LDO3_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO3_EVENT, SLG51000_LDO3_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO4_VSEL, SLG51000_LDO4_VSEL),
	regmap_reg_range(SLG51000_LDO4_MINV, SLG51000_LDO4_MAXV),
	regmap_reg_range(SLG51000_LDO4_CONF1, SLG51000_LDO4_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO4_EVENT, SLG51000_LDO4_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO5_VSEL, SLG51000_LDO5_VSEL),
	regmap_reg_range(SLG51000_LDO5_MINV, SLG51000_LDO5_MAXV),
	regmap_reg_range(SLG51000_LDO5_TRIM2, SLG51000_LDO5_TRIM2),
	regmap_reg_range(SLG51000_LDO5_CONF1, SLG51000_LDO5_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO5_EVENT, SLG51000_LDO5_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO6_VSEL, SLG51000_LDO6_VSEL),
	regmap_reg_range(SLG51000_LDO6_MINV, SLG51000_LDO6_MAXV),
	regmap_reg_range(SLG51000_LDO6_TRIM2, SLG51000_LDO6_TRIM2),
	regmap_reg_range(SLG51000_LDO6_CONF1, SLG51000_LDO6_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO6_EVENT, SLG51000_LDO6_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO7_VSEL, SLG51000_LDO7_VSEL),
	regmap_reg_range(SLG51000_LDO7_MINV, SLG51000_LDO7_MAXV),
	regmap_reg_range(SLG51000_LDO7_CONF1, SLG51000_LDO7_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO7_EVENT, SLG51000_LDO7_IRQ_MASK),
	regmap_reg_range(SLG51000_OTP_EVENT, SLG51000_OTP_EVENT),
	regmap_reg_range(SLG51000_OTP_IRQ_MASK, SLG51000_OTP_IRQ_MASK),
	regmap_reg_range(SLG51000_OTP_LOCK_OTP_PROG, SLG51000_OTP_LOCK_CTRL),
	regmap_reg_range(SLG51000_LOCK_GLOBAL_LOCK_CTRL1,
			 SLG51000_LOCK_GLOBAL_LOCK_CTRL1),
};

static const struct regmap_range slg51000_volatile_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_FAULT_LOG1, SLG51000_SYSCTL_STATUS),
	regmap_reg_range(SLG51000_IO_GPIO_STATUS, SLG51000_IO_GPIO_STATUS),
	regmap_reg_range(SLG51000_LDO1_EVENT, SLG51000_LDO1_STATUS),
	regmap_reg_range(SLG51000_LDO2_EVENT, SLG51000_LDO2_STATUS),
	regmap_reg_range(SLG51000_LDO3_EVENT, SLG51000_LDO3_STATUS),
	regmap_reg_range(SLG51000_LDO4_EVENT, SLG51000_LDO4_STATUS),
	regmap_reg_range(SLG51000_LDO5_EVENT, SLG51000_LDO5_STATUS),
	regmap_reg_range(SLG51000_LDO6_EVENT, SLG51000_LDO6_STATUS),
	regmap_reg_range(SLG51000_LDO7_EVENT, SLG51000_LDO7_STATUS),
	regmap_reg_range(SLG51000_OTP_EVENT, SLG51000_OTP_EVENT),
};

static const struct regmap_access_table slg51000_writeable_table = {
	.yes_ranges	= slg51000_writeable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51000_writeable_ranges),
};

static const struct regmap_access_table slg51000_readable_table = {
	.yes_ranges	= slg51000_readable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51000_readable_ranges),
};

static const struct regmap_access_table slg51000_volatile_table = {
	.yes_ranges	= slg51000_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51000_volatile_ranges),
};

static const struct regmap_config slg51000_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x8000,
	.wr_table = &slg51000_writeable_table,
	.rd_table = &slg51000_readable_table,
	.volatile_table = &slg51000_volatile_table,
};

/*
 * The SLG51002 tables differ from the SLG51000 ones above only in the per-LDO
 * tail registers -- which move because the two parts split their LDOs into
 * high- and low-voltage groups differently, see the header -- and in the
 * eighth LDO.
 */
static const struct regmap_range slg51002_writeable_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_MATRIX_CONF_A,
			 SLG51000_SYSCTL_MATRIX_CONF_A),
	regmap_reg_range(SLG51000_LDO1_VSEL, SLG51000_LDO1_VSEL),
	regmap_reg_range(SLG51000_LDO1_MINV, SLG51000_LDO1_MAXV),
	regmap_reg_range(SLG51000_LDO1_IRQ_MASK, SLG51000_LDO1_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO2_VSEL, SLG51000_LDO2_VSEL),
	regmap_reg_range(SLG51000_LDO2_MINV, SLG51000_LDO2_MAXV),
	regmap_reg_range(SLG51000_LDO2_IRQ_MASK, SLG51000_LDO2_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO3_VSEL, SLG51000_LDO3_VSEL),
	regmap_reg_range(SLG51000_LDO3_MINV, SLG51000_LDO3_MAXV),
	regmap_reg_range(SLG51000_LDO3_IRQ_MASK, SLG51000_LDO3_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO4_VSEL, SLG51000_LDO4_VSEL),
	regmap_reg_range(SLG51000_LDO4_MINV, SLG51000_LDO4_MAXV),
	regmap_reg_range(SLG51000_LDO4_IRQ_MASK, SLG51000_LDO4_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO5_VSEL, SLG51000_LDO5_VSEL),
	regmap_reg_range(SLG51000_LDO5_MINV, SLG51000_LDO5_MAXV),
	regmap_reg_range(SLG51000_LDO5_IRQ_MASK, SLG51000_LDO5_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO6_VSEL, SLG51000_LDO6_VSEL),
	regmap_reg_range(SLG51000_LDO6_MINV, SLG51000_LDO6_MAXV),
	regmap_reg_range(SLG51000_LDO6_IRQ_MASK, SLG51000_LDO6_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO7_VSEL, SLG51000_LDO7_VSEL),
	regmap_reg_range(SLG51000_LDO7_MINV, SLG51000_LDO7_MAXV),
	regmap_reg_range(SLG51000_LDO7_IRQ_MASK, SLG51000_LDO7_IRQ_MASK),
	regmap_reg_range(SLG51002_LDO8_VSEL, SLG51002_LDO8_VSEL),
	regmap_reg_range(SLG51002_LDO8_MINV, SLG51002_LDO8_MAXV),
	regmap_reg_range(SLG51002_LDO8_IRQ_MASK, SLG51002_LDO8_IRQ_MASK),
	regmap_reg_range(SLG51000_OTP_IRQ_MASK, SLG51000_OTP_IRQ_MASK),
};

static const struct regmap_range slg51002_readable_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_PATN_ID_B0,
			 SLG51000_SYSCTL_PATN_ID_B2),
	regmap_reg_range(SLG51000_SYSCTL_SYS_CONF_A,
			 SLG51000_SYSCTL_SYS_CONF_A),
	regmap_reg_range(SLG51000_SYSCTL_SYS_CONF_D,
			 SLG51000_SYSCTL_MATRIX_CONF_B),
	regmap_reg_range(SLG51000_SYSCTL_REFGEN_CONF_C,
			 SLG51000_SYSCTL_UVLO_CONF_A),
	regmap_reg_range(SLG51000_SYSCTL_FAULT_LOG1, SLG51000_SYSCTL_IRQ_MASK),
	regmap_reg_range(SLG51000_IO_GPIO1_CONF, SLG51000_IO_GPIO_STATUS),
	regmap_reg_range(SLG51000_LUTARRAY_LUT_VAL_0,
			 SLG51000_LUTARRAY_LUT_VAL_11),
	regmap_reg_range(SLG51000_MUXARRAY_INPUT_SEL_0,
			 SLG51000_MUXARRAY_INPUT_SEL_63),
	regmap_reg_range(SLG51000_PWRSEQ_RESOURCE_EN_0,
			 SLG51000_PWRSEQ_INPUT_SENSE_CONF_B),
	regmap_reg_range(SLG51000_LDO1_VSEL, SLG51000_LDO1_VSEL),
	regmap_reg_range(SLG51000_LDO1_MINV, SLG51000_LDO1_MAXV),
	regmap_reg_range(SLG51002_LDO1_TRIM2, SLG51002_LDO1_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO1_EVENT, SLG51000_LDO1_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO2_VSEL, SLG51000_LDO2_VSEL),
	regmap_reg_range(SLG51000_LDO2_MINV, SLG51000_LDO2_MAXV),
	regmap_reg_range(SLG51002_LDO2_TRIM2, SLG51002_LDO2_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO2_EVENT, SLG51000_LDO2_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO3_VSEL, SLG51000_LDO3_VSEL),
	regmap_reg_range(SLG51000_LDO3_MINV, SLG51000_LDO3_MAXV),
	regmap_reg_range(SLG51002_LDO3_TRIM2, SLG51002_LDO3_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO3_EVENT, SLG51000_LDO3_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO4_VSEL, SLG51000_LDO4_VSEL),
	regmap_reg_range(SLG51000_LDO4_MINV, SLG51000_LDO4_MAXV),
	regmap_reg_range(SLG51002_LDO4_TRIM2, SLG51002_LDO4_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO4_EVENT, SLG51000_LDO4_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO5_VSEL, SLG51000_LDO5_VSEL),
	regmap_reg_range(SLG51000_LDO5_MINV, SLG51000_LDO5_MAXV),
	regmap_reg_range(SLG51002_LDO5_TRIM2, SLG51002_LDO5_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO5_EVENT, SLG51000_LDO5_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO6_VSEL, SLG51000_LDO6_VSEL),
	regmap_reg_range(SLG51000_LDO6_MINV, SLG51000_LDO6_MAXV),
	regmap_reg_range(SLG51002_LDO6_TRIM2, SLG51002_LDO6_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO6_EVENT, SLG51000_LDO6_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO7_VSEL, SLG51000_LDO7_VSEL),
	regmap_reg_range(SLG51000_LDO7_MINV, SLG51000_LDO7_MAXV),
	regmap_reg_range(SLG51002_LDO7_TRIM2, SLG51002_LDO7_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO7_EVENT, SLG51000_LDO7_IRQ_MASK),
	regmap_reg_range(SLG51002_LDO8_VSEL, SLG51002_LDO8_VSEL),
	regmap_reg_range(SLG51002_LDO8_MINV, SLG51002_LDO8_MAXV),
	regmap_reg_range(SLG51002_LDO8_TRIM2, SLG51002_LDO8_VSEL_ACTUAL),
	regmap_reg_range(SLG51002_LDO8_EVENT, SLG51002_LDO8_IRQ_MASK),
	regmap_reg_range(SLG51000_OTP_EVENT, SLG51000_OTP_EVENT),
	regmap_reg_range(SLG51000_OTP_IRQ_MASK, SLG51000_OTP_IRQ_MASK),
	regmap_reg_range(SLG51000_OTP_LOCK_OTP_PROG, SLG51000_OTP_LOCK_CTRL),
	regmap_reg_range(SLG51000_LOCK_GLOBAL_LOCK_CTRL1,
			 SLG51002_LOCK_GLOBAL_LOCK_CTRL2),
};

static const struct regmap_range slg51002_volatile_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_FAULT_LOG1, SLG51000_SYSCTL_STATUS),
	regmap_reg_range(SLG51000_IO_GPIO_STATUS, SLG51000_IO_GPIO_STATUS),
	regmap_reg_range(SLG51000_LDO1_EVENT, SLG51000_LDO1_STATUS),
	regmap_reg_range(SLG51000_LDO2_EVENT, SLG51000_LDO2_STATUS),
	regmap_reg_range(SLG51000_LDO3_EVENT, SLG51000_LDO3_STATUS),
	regmap_reg_range(SLG51000_LDO4_EVENT, SLG51000_LDO4_STATUS),
	regmap_reg_range(SLG51000_LDO5_EVENT, SLG51000_LDO5_STATUS),
	regmap_reg_range(SLG51000_LDO6_EVENT, SLG51000_LDO6_STATUS),
	regmap_reg_range(SLG51000_LDO7_EVENT, SLG51000_LDO7_STATUS),
	regmap_reg_range(SLG51002_LDO8_EVENT, SLG51002_LDO8_STATUS),
	regmap_reg_range(SLG51000_OTP_EVENT, SLG51000_OTP_EVENT),
};

static const struct regmap_access_table slg51002_writeable_table = {
	.yes_ranges	= slg51002_writeable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51002_writeable_ranges),
};

static const struct regmap_access_table slg51002_readable_table = {
	.yes_ranges	= slg51002_readable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51002_readable_ranges),
};

static const struct regmap_access_table slg51002_volatile_table = {
	.yes_ranges	= slg51002_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51002_volatile_ranges),
};

static const struct regmap_config slg51002_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = SLG51002_LOCK_GLOBAL_LOCK_CTRL2,
	.wr_table = &slg51002_writeable_table,
	.rd_table = &slg51002_readable_table,
	.volatile_table = &slg51002_volatile_table,
};

static const struct regulator_ops slg51000_regl_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
};

static const struct regulator_ops slg51000_switch_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static int slg51000_of_parse_cb(struct device_node *np,
				const struct regulator_desc *desc,
				struct regulator_config *config)
{
	struct gpio_desc *ena_gpiod;

	ena_gpiod = fwnode_gpiod_get_index(of_fwnode_handle(np), "enable", 0,
					   GPIOD_OUT_LOW |
						GPIOD_FLAGS_BIT_NONEXCLUSIVE,
					   "gpio-en-ldo");
	if (!IS_ERR(ena_gpiod))
		config->ena_gpiod = ena_gpiod;

	return 0;
}

#define __SLG51000_REGL_DESC(_id, _name, _s_name, _min, _step, _vsel) \
	[SLG51000_REGULATOR_##_id] = {                             \
		.name = #_name,                                    \
		.supply_name = _s_name,				   \
		.id = SLG51000_REGULATOR_##_id,                    \
		.of_match = of_match_ptr(#_name),                  \
		.of_parse_cb = slg51000_of_parse_cb,               \
		.ops = &slg51000_regl_ops,                         \
		.regulators_node = of_match_ptr("regulators"),     \
		.n_voltages = 256,                                 \
		.min_uV = _min,                                    \
		.uV_step = _step,                                  \
		.linear_min_sel = 0,                               \
		.vsel_mask = SLG51000_VSEL_MASK,                   \
		.vsel_reg = _vsel,                                 \
		.enable_reg = SLG51000_SYSCTL_MATRIX_CONF_A,       \
		.enable_mask = BIT(SLG51000_REGULATOR_##_id),      \
		.type = REGULATOR_VOLTAGE,                         \
		.owner = THIS_MODULE,                              \
	}

#define SLG51000_REGL_DESC(_id, _name, _s_name, _min, _step) \
	__SLG51000_REGL_DESC(_id, _name, _s_name, _min, _step, \
			     SLG51000_##_id##_VSEL)

static const struct regulator_desc slg51000_regls_desc[SLG51000_MAX_REGULATORS] = {
	SLG51000_REGL_DESC(LDO1, ldo1, NULL,   2400000,  5000),
	SLG51000_REGL_DESC(LDO2, ldo2, NULL,   2400000,  5000),
	SLG51000_REGL_DESC(LDO3, ldo3, "vin3", 1200000, 10000),
	SLG51000_REGL_DESC(LDO4, ldo4, "vin4", 1200000, 10000),
	SLG51000_REGL_DESC(LDO5, ldo5, "vin5",  400000,  5000),
	SLG51000_REGL_DESC(LDO6, ldo6, "vin6",  400000,  5000),
	SLG51000_REGL_DESC(LDO7, ldo7, "vin7", 1200000, 10000),
};

/*
 * The SLG51002 splits its LDOs differently: LDO1..LDO5 are the high-voltage
 * ones and LDO6..LDO8 the low-voltage ones, where on the SLG51000 LDO5 and
 * LDO6 are low-voltage and LDO7 high-voltage.  LDO1 and LDO2 share one input
 * pin here rather than having none named.
 */
static const struct regulator_desc slg51002_regls_desc[SLG51002_MAX_REGULATORS] = {
	SLG51000_REGL_DESC(LDO1, ldo1, "vin1_2", 1200000, 10000),
	SLG51000_REGL_DESC(LDO2, ldo2, "vin1_2", 1200000, 10000),
	SLG51000_REGL_DESC(LDO3, ldo3, "vin3",   1200000, 10000),
	SLG51000_REGL_DESC(LDO4, ldo4, "vin4",   1200000, 10000),
	SLG51000_REGL_DESC(LDO5, ldo5, "vin5",   1200000, 10000),
	SLG51000_REGL_DESC(LDO6, ldo6, "vin6",    400000,  5000),
	SLG51000_REGL_DESC(LDO7, ldo7, "vin7",    400000,  5000),
	__SLG51000_REGL_DESC(LDO8, ldo8, "vin8",  400000,  5000,
			     SLG51002_LDO8_VSEL),
};

static const struct slg51000_chip_info slg51000_chip_info = {
	.regmap_cfg	= &slg51000_regmap_config,
	.rdesc		= slg51000_regls_desc,
	.evt_sta	= slg51000_es_reg,
	.min_regs	= slg51000_min_regs,
	.trim2_regs	= slg51000_trim2_regs,
	.n_regulators	= SLG51000_MAX_REGULATORS,
	.ldo12_vrange	= true,
};

static const struct slg51000_chip_info slg51002_chip_info = {
	.regmap_cfg	= &slg51002_regmap_config,
	.rdesc		= slg51002_regls_desc,
	.evt_sta	= slg51002_es_reg,
	.min_regs	= slg51002_min_regs,
	.trim2_regs	= slg51002_trim2_regs,
	.n_regulators	= SLG51002_MAX_REGULATORS,
};

static int slg51000_regulator_init(struct slg51000 *chip)
{
	const struct slg51000_chip_info *info = chip->info;
	struct regulator_config config = { };
	struct regulator_desc *rdesc;
	unsigned int reg, val;
	u8 vsel_range[2];
	int id, ret = 0;

	for (id = 0; id < info->n_regulators; id++) {
		rdesc = &chip->rdesc[id];
		config.regmap = chip->regmap;
		config.dev = chip->dev;
		config.driver_data = chip;

		ret = regmap_bulk_read(chip->regmap, info->min_regs[id],
				       vsel_range, 2);
		if (ret < 0) {
			dev_err(chip->dev,
				"Failed to read the MIN register\n");
			return ret;
		}

		/*
		 * LDO1 and LDO2 are dual-range on some variants and pick
		 * their base from a bit in MISC1.
		 */
		if (info->ldo12_vrange && (id == SLG51000_REGULATOR_LDO1 ||
					   id == SLG51000_REGULATOR_LDO2)) {
			if (id == SLG51000_REGULATOR_LDO1)
				reg = SLG51000_LDO1_MISC1;
			else
				reg = SLG51000_LDO2_MISC1;

			ret = regmap_read(chip->regmap, reg, &val);
			if (ret < 0) {
				dev_err(chip->dev,
					"Failed to read voltage range of ldo%d\n",
					id + 1);
				return ret;
			}

			rdesc->linear_min_sel = vsel_range[0];
			rdesc->n_voltages = vsel_range[1] + 1;
			if (val & SLG51000_SEL_VRANGE_MASK)
				rdesc->min_uV = SLG51000_LDOHP_HV_MIN
						+ (vsel_range[0]
						   * rdesc->uV_step);
			else
				rdesc->min_uV = SLG51000_LDOHP_LV_MIN
						+ (vsel_range[0]
						   * rdesc->uV_step);
			goto register_it;
		}

		/*
		 * A low-voltage LDO can be strapped into bypass, in which case
		 * it passes its input through and is a switch, not a
		 * regulator.  Which LDOs those are moves with the part.
		 */
		if (info->trim2_regs[id]) {
			ret = regmap_read(chip->regmap, info->trim2_regs[id],
					  &val);
			if (ret < 0) {
				dev_err(chip->dev,
					"Failed to read LDO mode register\n");
				return ret;
			}

			if (val & SLG51000_SEL_BYP_MODE_MASK) {
				rdesc->ops = &slg51000_switch_ops;
				rdesc->n_voltages = 0;
				rdesc->min_uV = 0;
				rdesc->uV_step = 0;
				rdesc->linear_min_sel = 0;
				goto register_it;
			}
		}

		rdesc->linear_min_sel = vsel_range[0];
		rdesc->n_voltages = vsel_range[1] + 1;
		rdesc->min_uV = rdesc->min_uV + (vsel_range[0] * rdesc->uV_step);

register_it:
		chip->rdev[id] = devm_regulator_register(chip->dev, rdesc,
							 &config);
		if (IS_ERR(chip->rdev[id])) {
			ret = PTR_ERR(chip->rdev[id]);
			dev_err(chip->dev,
				"Failed to register regulator(%s):%d\n",
				rdesc->name, ret);
			return ret;
		}
	}

	return 0;
}

static irqreturn_t slg51000_irq_handler(int irq, void *data)
{
	struct slg51000 *chip = data;
	const struct slg51000_chip_info *info = chip->info;
	struct regmap *regmap = chip->regmap;
	enum { R0 = 0, R1, R2, REG_MAX };
	u8 evt[SLG51000_MAX_EVT_REGISTER][REG_MAX];
	/* The system controller's pair follows the per-LDO ones. */
	const unsigned int sctl_evt = info->n_regulators;
	int ret, i, handled = IRQ_NONE;
	unsigned int evt_otp, mask_otp;

	/* Read event[R0], status[R1] and mask[R2] register */
	for (i = 0; i < info->n_regulators + 1; i++) {
		ret = regmap_bulk_read(regmap, info->evt_sta[i].ereg, evt[i],
				       REG_MAX);
		if (ret < 0) {
			dev_err(chip->dev,
				"Failed to read event registers(%d)\n", ret);
			return IRQ_NONE;
		}
	}

	ret = regmap_read(regmap, SLG51000_OTP_EVENT, &evt_otp);
	if (ret < 0) {
		dev_err(chip->dev,
			"Failed to read otp event registers(%d)\n", ret);
		return IRQ_NONE;
	}

	ret = regmap_read(regmap, SLG51000_OTP_IRQ_MASK, &mask_otp);
	if (ret < 0) {
		dev_err(chip->dev,
			"Failed to read otp mask register(%d)\n", ret);
		return IRQ_NONE;
	}

	if ((evt_otp & SLG51000_EVT_CRC_MASK) &&
	    !(mask_otp & SLG51000_IRQ_CRC_MASK)) {
		dev_info(chip->dev,
			 "OTP has been read or OTP crc is not zero\n");
		handled = IRQ_HANDLED;
	}

	for (i = 0; i < info->n_regulators; i++) {
		if (!(evt[i][R2] & SLG51000_IRQ_ILIM_FLAG_MASK) &&
		    (evt[i][R0] & SLG51000_EVT_ILIM_FLAG_MASK)) {
			regulator_notifier_call_chain(chip->rdev[i],
					    REGULATOR_EVENT_OVER_CURRENT, NULL);

			if (evt[i][R1] & SLG51000_STA_ILIM_FLAG_MASK)
				dev_warn(chip->dev,
					 "Over-current limit(ldo%d)\n", i + 1);
			handled = IRQ_HANDLED;
		}
	}

	if (!(evt[sctl_evt][R2] & SLG51000_IRQ_HIGH_TEMP_WARN_MASK) &&
	    (evt[sctl_evt][R0] & SLG51000_EVT_HIGH_TEMP_WARN_MASK)) {
		for (i = 0; i < info->n_regulators; i++) {
			if (!(evt[i][R1] & SLG51000_STA_ILIM_FLAG_MASK) &&
			    (evt[i][R1] & SLG51000_STA_VOUT_OK_FLAG_MASK)) {
				regulator_notifier_call_chain(chip->rdev[i],
					       REGULATOR_EVENT_OVER_TEMP, NULL);
			}
		}
		handled = IRQ_HANDLED;
		if (evt[sctl_evt][R1] & SLG51000_STA_HIGH_TEMP_WARN_MASK)
			dev_warn(chip->dev, "High temperature warning!\n");
	}

	return handled;
}

static void slg51000_clear_fault_log(struct slg51000 *chip)
{
	unsigned int val = 0;
	int ret = 0;

	ret = regmap_read(chip->regmap, SLG51000_SYSCTL_FAULT_LOG1, &val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read Fault log register\n");
		return;
	}

	if (val & SLG51000_FLT_OVER_TEMP_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_OVER_TEMP\n");
	if (val & SLG51000_FLT_POWER_SEQ_CRASH_REQ_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_POWER_SEQ_CRASH_REQ\n");
	if (val & SLG51000_FLT_RST_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_RST\n");
	if (val & SLG51000_FLT_POR_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_POR\n");
}

static int slg51000_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct slg51000 *chip;
	struct gpio_desc *cs_gpiod;
	int error, ret;

	chip = devm_kzalloc(dev, sizeof(struct slg51000), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->info = i2c_get_match_data(client);
	if (!chip->info)
		return -ENODEV;

	/*
	 * slg51000_regulator_init() patches each descriptor from the chip's
	 * own trim registers, so it needs a copy per device rather than the
	 * shared template.
	 */
	chip->rdesc = devm_kmemdup_array(dev, chip->info->rdesc,
					 chip->info->n_regulators,
					 sizeof(*chip->rdesc), GFP_KERNEL);
	if (!chip->rdesc)
		return -ENOMEM;

	cs_gpiod = devm_gpiod_get_optional(dev, "dlg,cs",
					   GPIOD_OUT_HIGH |
						GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(cs_gpiod))
		return PTR_ERR(cs_gpiod);

	if (cs_gpiod) {
		dev_info(dev, "Found chip selector property\n");
		chip->cs_gpiod = cs_gpiod;
	}

	usleep_range(10000, 11000);

	i2c_set_clientdata(client, chip);
	chip->chip_irq = client->irq;
	chip->dev = dev;
	chip->regmap = devm_regmap_init_i2c(client, chip->info->regmap_cfg);
	if (IS_ERR(chip->regmap)) {
		error = PTR_ERR(chip->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n",
			error);
		return error;
	}

	ret = slg51000_regulator_init(chip);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to init regulator(%d)\n", ret);
		return ret;
	}

	slg51000_clear_fault_log(chip);

	if (chip->chip_irq) {
		ret = devm_request_threaded_irq(dev, chip->chip_irq, NULL,
						slg51000_irq_handler,
						(IRQF_TRIGGER_HIGH |
						IRQF_ONESHOT),
						dev_name(dev), chip);
		if (ret != 0) {
			dev_err(dev, "Failed to request IRQ: %d\n",
				chip->chip_irq);
			return ret;
		}
	} else {
		dev_info(dev, "No IRQ configured\n");
	}

	return ret;
}

static const struct i2c_device_id slg51000_i2c_id[] = {
	{ "slg51000", (kernel_ulong_t)&slg51000_chip_info },
	{ "slg51002", (kernel_ulong_t)&slg51002_chip_info },
	{}
};
MODULE_DEVICE_TABLE(i2c, slg51000_i2c_id);

static const struct of_device_id slg51000_of_match[] = {
	{ .compatible = "dlg,slg51000", .data = &slg51000_chip_info },
	{ .compatible = "dlg,slg51002", .data = &slg51002_chip_info },
	{}
};
MODULE_DEVICE_TABLE(of, slg51000_of_match);

static struct i2c_driver slg51000_regulator_driver = {
	.driver = {
		.name = "slg51000-regulator",
		.of_match_table = slg51000_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = slg51000_i2c_probe,
	.id_table = slg51000_i2c_id,
};

module_i2c_driver(slg51000_regulator_driver);

MODULE_AUTHOR("Eric Jeong <eric.jeong.opensource@diasemi.com>");
MODULE_DESCRIPTION("SLG51000/SLG51002 regulator driver");
MODULE_LICENSE("GPL");

