// SPDX-License-Identifier: GPL-2.0-only
//
// NVMEM driver for the Maxim MAX77779 battery-backed scratchpad
//
// The MAX77779 charging PMIC carries a small battery-backed RAM that survives
// a reset, and even a power cut, as long as the battery stays connected. The
// bootloader uses it to exchange the reboot mode and reason with the kernel,
// which is what makes it worth exposing: it outlives the DRAM-resident
// post-mortem buffers, which a cold reset clears.
//
// Like the fuel gauge, the scratchpad is reachable as an independent I2C
// device (default address 0x60), so this driver owns the I2C client and its
// regmap directly. Should the rest of the MAX77779 ever gain an MFD core in
// the shape of the sibling MAX77759, the nvmem registration below is already
// separated from how the regmap was obtained, so growing a second probe path
// that takes the parent's regmap is a small addition rather than a rewrite.
//
// Copyright (C) 2026 Steffen Deusch

#include <linux/align.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-provider.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/types.h>

#define MAX77779_SP_PAGE_CTRL		0x70
#define MAX77779_SP_PAGE_CTRL_PAGE	GENMASK(1, 0)
#define MAX77779_SP_DATA		0x80
#define MAX77779_SP_MAX_ADDR		0xff

/*
 * The registers in this sub-block are 16-bit, so the 128 registers of the
 * 0x80..0xff data window map one 256-byte page, and PAGE_CTRL selects which of
 * the four pages that is. Only page 0 is exposed: it is the page whose
 * contents are shared with the bootloader.
 */
#define MAX77779_SP_PAGE_SIZE		256

struct max77779_nvmem {
	struct regmap *regmap;
	/* Serialises the page select against the access it applies to, guards buf */
	struct mutex lock;
	/* Bounce buffer for the register-aligned window covering a transfer */
	u8 buf[MAX77779_SP_PAGE_SIZE];
};

static bool max77779_nvmem_readable_reg(struct device *dev, unsigned int reg)
{
	return reg == MAX77779_SP_PAGE_CTRL ||
	       (reg >= MAX77779_SP_DATA && reg <= MAX77779_SP_MAX_ADDR);
}

/*
 * Battery-backed RAM written by both the AP and the bootloader, so nothing
 * here may be cached or assumed unchanged since the last read.
 */
static const struct regmap_config max77779_nvmem_regmap_config = {
	.name = "max77779-scratch",
	.reg_bits = 8,
	.val_bits = 16,
	/*
	 * The low byte of a register is the lower of the two scratchpad
	 * addresses it covers, which is little-endian regardless of what the
	 * host happens to be.
	 */
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.max_register = MAX77779_SP_MAX_ADDR,
	.readable_reg = max77779_nvmem_readable_reg,
	.writeable_reg = max77779_nvmem_readable_reg,
	.volatile_reg = max77779_nvmem_readable_reg,
};

/*
 * A scratchpad byte and its neighbour share one 16-bit register, so a transfer
 * has to be widened to the register-aligned window enclosing it. Returns the
 * byte offset that window starts at.
 */
static unsigned int max77779_nvmem_window(unsigned int offset, size_t bytes,
					  unsigned int *reg, size_t *len)
{
	unsigned int start = ALIGN_DOWN(offset, 2);

	*reg = MAX77779_SP_DATA + start / 2;
	*len = ALIGN(offset + bytes, 2) - start;

	return start;
}

static int max77779_nvmem_select_page0(struct max77779_nvmem *nvmem)
{
	return regmap_update_bits(nvmem->regmap, MAX77779_SP_PAGE_CTRL,
				  MAX77779_SP_PAGE_CTRL_PAGE, 0);
}

static int max77779_nvmem_reg_read(void *priv, unsigned int offset,
				   void *val, size_t bytes)
{
	struct max77779_nvmem *nvmem = priv;
	unsigned int reg, start;
	size_t len;
	int ret;

	if (offset >= MAX77779_SP_PAGE_SIZE ||
	    bytes > MAX77779_SP_PAGE_SIZE - offset)
		return -EINVAL;

	start = max77779_nvmem_window(offset, bytes, &reg, &len);

	guard(mutex)(&nvmem->lock);

	ret = max77779_nvmem_select_page0(nvmem);
	if (ret)
		return ret;

	ret = regmap_raw_read(nvmem->regmap, reg, nvmem->buf, len);
	if (ret)
		return ret;

	memcpy(val, nvmem->buf + (offset - start), bytes);

	return 0;
}

static int max77779_nvmem_reg_write(void *priv, unsigned int offset,
				    void *val, size_t bytes)
{
	struct max77779_nvmem *nvmem = priv;
	unsigned int reg, start;
	size_t len;
	int ret;

	if (offset >= MAX77779_SP_PAGE_SIZE ||
	    bytes > MAX77779_SP_PAGE_SIZE - offset)
		return -EINVAL;

	start = max77779_nvmem_window(offset, bytes, &reg, &len);

	guard(mutex)(&nvmem->lock);

	ret = max77779_nvmem_select_page0(nvmem);
	if (ret)
		return ret;

	/*
	 * Widening happens exactly when an end of the transfer falls inside a
	 * register, and then the neighbouring byte has to be read back so the
	 * write does not clear it.
	 */
	if (len != bytes) {
		ret = regmap_raw_read(nvmem->regmap, reg, nvmem->buf, len);
		if (ret)
			return ret;
	}

	memcpy(nvmem->buf + (offset - start), val, bytes);

	return regmap_raw_write(nvmem->regmap, reg, nvmem->buf, len);
}

static int max77779_nvmem_register(struct device *dev, struct regmap *regmap)
{
	struct nvmem_config config = {
		.dev = dev,
		.name = dev_name(dev),
		.id = NVMEM_DEVID_NONE,
		.type = NVMEM_TYPE_BATTERY_BACKED,
		.ignore_wp = true,
		.size = MAX77779_SP_PAGE_SIZE,
		.word_size = sizeof(u8),
		.stride = sizeof(u8),
		.reg_read = max77779_nvmem_reg_read,
		.reg_write = max77779_nvmem_reg_write,
	};
	struct max77779_nvmem *nvmem;
	int ret;

	nvmem = devm_kzalloc(dev, sizeof(*nvmem), GFP_KERNEL);
	if (!nvmem)
		return -ENOMEM;

	nvmem->regmap = regmap;

	ret = devm_mutex_init(dev, &nvmem->lock);
	if (ret)
		return ret;

	config.priv = nvmem;

	return PTR_ERR_OR_ZERO(devm_nvmem_register(dev, &config));
}

static int max77779_nvmem_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client, &max77779_nvmem_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to initialise regmap\n");

	return max77779_nvmem_register(dev, regmap);
}

static const struct of_device_id max77779_nvmem_of_id[] = {
	{ .compatible = "maxim,max77779sp", },
	{ }
};
MODULE_DEVICE_TABLE(of, max77779_nvmem_of_id);

static const struct i2c_device_id max77779_nvmem_i2c_id[] = {
	{ "max77779sp", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77779_nvmem_i2c_id);

static struct i2c_driver max77779_nvmem_i2c_driver = {
	.driver = {
		.name = "max77779-nvmem",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = max77779_nvmem_of_id,
	},
	.probe = max77779_nvmem_i2c_probe,
	.id_table = max77779_nvmem_i2c_id,
};

module_i2c_driver(max77779_nvmem_i2c_driver);

MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_DESCRIPTION("NVMEM driver for the Maxim MAX77779 scratchpad");
MODULE_LICENSE("GPL");
