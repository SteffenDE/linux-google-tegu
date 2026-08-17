// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2026 Steffen Deusch
 *
 * GPIO controller in the Samsung S2MPG14 and S2MPG15 PMICs, as found on
 * Google Tensor SoC boards.  The S2MPG14 has six pins and the S2MPG15 ten;
 * the register layout is otherwise the same.
 *
 * Pins are reached through the MFD parent's "gpio" regmap, which on these
 * parts is a separate access type over the ACPM mailbox rather than a window
 * in the PMIC block.  Each pin has one control register holding its direction,
 * output value, pulls and drive strength, so a direction or value change is a
 * read-modify-write of a single byte.  Reading an input goes to a separate
 * status register instead, since the control register's output-value bit only
 * reflects what was driven.
 *
 * The pins also have interrupt and monitor-select registers.  Neither is used
 * here.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/mfd/samsung/s2mpg14.h>
#include <linux/mfd/samsung/s2mpg15.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>

struct s2mpg1x_gpio_variant {
	unsigned int ngpio;
	unsigned int ctrl_base;
	unsigned int status_base;
};

struct s2mpg1x_gpio {
	struct gpio_chip gc;
	struct regmap *regmap;
	const struct s2mpg1x_gpio_variant *variant;
};

static const struct s2mpg1x_gpio_variant s2mpg14_gpio_variant = {
	.ngpio		= S2MPG14_GPIO_NR,
	.ctrl_base	= S2MPG14_GPIO0_SET,
	.status_base	= S2MPG14_GPIO_STATUS,
};

static const struct s2mpg1x_gpio_variant s2mpg15_gpio_variant = {
	.ngpio		= S2MPG15_GPIO_NR,
	.ctrl_base	= S2MPG15_GPIO0_SET,
	.status_base	= S2MPG15_GPIO_STATUS1,
};

static unsigned int s2mpg1x_gpio_ctrl_reg(struct s2mpg1x_gpio *chip,
					  unsigned int offset)
{
	return chip->variant->ctrl_base + offset;
}

static int s2mpg1x_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct s2mpg1x_gpio *chip = gpiochip_get_data(gc);
	unsigned int val;
	int ret;

	ret = regmap_read(chip->regmap, s2mpg1x_gpio_ctrl_reg(chip, offset),
			  &val);
	if (ret)
		return ret;

	return val & S2MPG1X_GPIO_SET_OEN ? GPIO_LINE_DIRECTION_OUT
					  : GPIO_LINE_DIRECTION_IN;
}

static int s2mpg1x_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct s2mpg1x_gpio *chip = gpiochip_get_data(gc);
	unsigned int reg, val;
	int ret;

	/*
	 * The control register carries both the direction and, for an output,
	 * the level being driven, so one read settles the common case.
	 */
	ret = regmap_read(chip->regmap, s2mpg1x_gpio_ctrl_reg(chip, offset),
			  &val);
	if (ret)
		return ret;

	if (val & S2MPG1X_GPIO_SET_OEN)
		return !!(val & S2MPG1X_GPIO_SET_OUT);

	/*
	 * An input's level is only in the status registers, eight pins to a
	 * register.
	 */
	reg = chip->variant->status_base + offset / 8;
	ret = regmap_read(chip->regmap, reg, &val);
	if (ret)
		return ret;

	return !!(val & BIT(offset % 8));
}

static int s2mpg1x_gpio_set(struct gpio_chip *gc, unsigned int offset,
			    int value)
{
	struct s2mpg1x_gpio *chip = gpiochip_get_data(gc);

	return regmap_update_bits(chip->regmap,
				  s2mpg1x_gpio_ctrl_reg(chip, offset),
				  S2MPG1X_GPIO_SET_OUT,
				  value ? S2MPG1X_GPIO_SET_OUT : 0);
}

static int s2mpg1x_gpio_direction_input(struct gpio_chip *gc,
					unsigned int offset)
{
	struct s2mpg1x_gpio *chip = gpiochip_get_data(gc);

	return regmap_clear_bits(chip->regmap,
				 s2mpg1x_gpio_ctrl_reg(chip, offset),
				 S2MPG1X_GPIO_SET_OEN);
}

static int s2mpg1x_gpio_direction_output(struct gpio_chip *gc,
					 unsigned int offset, int value)
{
	struct s2mpg1x_gpio *chip = gpiochip_get_data(gc);
	int ret;

	/* Drive the level before enabling the output, so it never glitches. */
	ret = s2mpg1x_gpio_set(gc, offset, value);
	if (ret)
		return ret;

	return regmap_set_bits(chip->regmap,
			       s2mpg1x_gpio_ctrl_reg(chip, offset),
			       S2MPG1X_GPIO_SET_OEN);
}

static int s2mpg1x_gpio_set_config(struct gpio_chip *gc, unsigned int offset,
				   unsigned long config)
{
	struct s2mpg1x_gpio *chip = gpiochip_get_data(gc);
	unsigned int reg = s2mpg1x_gpio_ctrl_reg(chip, offset);
	unsigned int pulls = S2MPG1X_GPIO_SET_PULL_UP |
			     S2MPG1X_GPIO_SET_PULL_DOWN;
	u32 arg = pinconf_to_config_argument(config);

	switch (pinconf_to_config_param(config)) {
	case PIN_CONFIG_BIAS_DISABLE:
		return regmap_update_bits(chip->regmap, reg, pulls, 0);
	case PIN_CONFIG_BIAS_PULL_UP:
		return regmap_update_bits(chip->regmap, reg, pulls,
					  S2MPG1X_GPIO_SET_PULL_UP);
	case PIN_CONFIG_BIAS_PULL_DOWN:
		return regmap_update_bits(chip->regmap, reg, pulls,
					  S2MPG1X_GPIO_SET_PULL_DOWN);
	case PIN_CONFIG_DRIVE_STRENGTH:
		/* One bit: the nominal drive, or twice it. */
		if (arg == 2)
			return regmap_clear_bits(chip->regmap, reg,
						 S2MPG1X_GPIO_SET_DRV_STR);
		if (arg == 4)
			return regmap_set_bits(chip->regmap, reg,
					       S2MPG1X_GPIO_SET_DRV_STR);
		return -ENOTSUPP;
	default:
		return -ENOTSUPP;
	}
}

static int s2mpg1x_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpg1x_gpio *chip;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->variant = device_get_match_data(dev);
	if (!chip->variant)
		return -ENODEV;

	chip->regmap = dev_get_regmap(dev->parent, "gpio");
	if (!chip->regmap)
		return dev_err_probe(dev, -ENODEV, "no gpio regmap\n");

	chip->gc.label = dev_name(dev);
	chip->gc.parent = dev;
	chip->gc.owner = THIS_MODULE;
	chip->gc.base = -1;
	chip->gc.ngpio = chip->variant->ngpio;
	/* Every access is an ACPM mailbox round trip. */
	chip->gc.can_sleep = true;
	chip->gc.get_direction = s2mpg1x_gpio_get_direction;
	chip->gc.direction_input = s2mpg1x_gpio_direction_input;
	chip->gc.direction_output = s2mpg1x_gpio_direction_output;
	chip->gc.get = s2mpg1x_gpio_get;
	chip->gc.set = s2mpg1x_gpio_set;
	chip->gc.set_config = s2mpg1x_gpio_set_config;

	return devm_gpiochip_add_data(dev, &chip->gc, chip);
}

static const struct of_device_id s2mpg1x_gpio_of_match[] = {
	{
		.compatible = "samsung,s2mpg14-gpio",
		.data = &s2mpg14_gpio_variant,
	}, {
		.compatible = "samsung,s2mpg15-gpio",
		.data = &s2mpg15_gpio_variant,
	}, {
	}
};
MODULE_DEVICE_TABLE(of, s2mpg1x_gpio_of_match);

static struct platform_driver s2mpg1x_gpio_driver = {
	.driver = {
		.name = "s2mpg1x-gpio",
		.of_match_table = s2mpg1x_gpio_of_match,
	},
	.probe = s2mpg1x_gpio_probe,
};
module_platform_driver(s2mpg1x_gpio_driver);

MODULE_AUTHOR("Steffen Deusch");
MODULE_DESCRIPTION("Samsung S2MPG14/S2MPG15 PMIC GPIO controller");
MODULE_LICENSE("GPL");
