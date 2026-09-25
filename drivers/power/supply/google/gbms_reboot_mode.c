// SPDX-License-Identifier: GPL-2.0-only
/*
 * The reboot mode, in the store the bootloader reads after a cold reset.
 *
 * The bootloader looks for the mode in two places, because neither survives
 * both kinds of reset: the PMU's SYSIP_DAT0 outlives a warm reset and is
 * cleared by a cold one, and the MAX77779 scratchpad is battery-backed and
 * outlives both. The PMU copy is syscon-reboot-mode's. This is the scratchpad
 * copy, which the vendor's pixel-zuma-reboot.c writes as the same mode with
 * BIT(31) set: battery-backed RAM comes up holding whatever it last held, so
 * the bootloader has to be told the field means something. It clears the field
 * once it has acted on it.
 *
 * The scratchpad belongs to max77779_sp as a gbms_storage provider, and the
 * mode is its GBMS_TAG_RSBM -- the first four bytes of a page that goes on to
 * hold the reboot reason and, from byte 10, battery state Android keeps there.
 * So the write goes through the storage layer, as the vendor's does, rather
 * than past it.
 *
 * The modes themselves come from the device tree, through the reboot-mode
 * core, already carrying the valid bit.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot-mode.h>

#include "gbms_storage.h"

static int gbms_reboot_mode_write(struct reboot_mode_driver *reboot,
				  unsigned int magic)
{
	u32 mode = magic;
	int ret;

	ret = gbms_storage_write(GBMS_TAG_RSBM, &mode, sizeof(mode));
	if (ret < 0) {
		dev_err(reboot->dev, "cannot store reboot mode %#x (%d)\n",
			mode, ret);
		return ret;
	}

	return 0;
}

static int gbms_reboot_mode_probe(struct platform_device *pdev)
{
	struct reboot_mode_driver *reboot;
	u32 mode;
	int ret;

	/*
	 * Read the tag once, so that a provider which has not registered yet is
	 * a deferred probe now rather than a write that fails at reboot, when
	 * there is no one left to tell. The storage layer answers -ENOENT for a
	 * tag no registered provider serves, or -EPROBE_DEFER while one is
	 * still to arrive; either way the provider is a device probe, so wait.
	 */
	ret = gbms_storage_read(GBMS_TAG_RSBM, &mode, sizeof(mode));
	if (ret == -ENOENT || ret == -EPROBE_DEFER)
		return dev_err_probe(&pdev->dev, -EPROBE_DEFER,
				     "reboot mode store not up yet\n");
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret,
				     "cannot read reboot mode store\n");

	reboot = devm_kzalloc(&pdev->dev, sizeof(*reboot), GFP_KERNEL);
	if (!reboot)
		return -ENOMEM;

	reboot->dev = &pdev->dev;
	reboot->write = gbms_reboot_mode_write;

	return devm_reboot_mode_register(&pdev->dev, reboot);
}

static const struct of_device_id gbms_reboot_mode_match[] = {
	{ .compatible = "google,gbms-reboot-mode" },
	{ }
};
MODULE_DEVICE_TABLE(of, gbms_reboot_mode_match);

static struct platform_driver gbms_reboot_mode_driver = {
	.driver = {
		.name = "gbms-reboot-mode",
		.of_match_table = gbms_reboot_mode_match,
	},
	.probe = gbms_reboot_mode_probe,
};
module_platform_driver(gbms_reboot_mode_driver);

MODULE_DESCRIPTION("Reboot mode in the Google BMS storage");
MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_LICENSE("GPL");
