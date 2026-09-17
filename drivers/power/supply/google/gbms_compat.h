/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Compatibility shims for the Google BMS port.
 *
 * The vendor sources target 6.1.145. This header carries the pieces they use
 * that no longer exist in our tree, so that the drivers themselves stay as
 * close to the vendor snapshot as possible -- see the port commit for the
 * drift that could be fixed in place instead.
 *
 * Nothing here is a new interface: each entry reproduces the semantics of the
 * API it replaces, and each is a place where a real conversion is owed before
 * the driver is trusted on hardware.
 */

#ifndef __GBMS_COMPAT_H_
#define __GBMS_COMPAT_H_

#include <linux/err.h>
#include <linux/of.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/legacy.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/usb/pd.h>
#include <linux/usb/tcpm.h>
#include <linux/power_supply.h>

/*
 * The vendor kernel extends enum power_supply_charge_type with a CV-phase
 * value, deliberately placed at 50 so that it cannot collide with upstream
 * additions. GBMS reports it through POWER_SUPPLY_PROP_CHARGE_TYPE and
 * gbms_chg_type_s() names it in the GBMS-specific sysfs nodes. Note the
 * vendor also patches POWER_SUPPLY_CHARGE_TYPE_TEXT and we do not, so the
 * standard charge_type attribute reads "50" here where a vendor kernel reads
 * "Taper". Keeping the value here rather than in include/linux/power_supply.h
 * is what lets this port carry no power_supply core patch.
 */
#define POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT	50

/*
 * <linux/of_gpio.h> and the integer-based OF GPIO lookups were removed
 * upstream in favour of the gpiod API. The drivers here hold GPIOs as integers
 * throughout (uc_data->bst_on, chip->ceb_gpio, ...) and read them from DT
 * properties that do not carry the "-gpios" suffix gpiod lookups require
 * ("max77779,bst-on", "google,buck_chg_en", "rt,intr_gpio"), so they cannot be
 * converted by swapping the lookup alone.
 *
 * These reproduce what of_get_named_gpio{,_flags}() did: resolve the specifier
 * through the controller's phandle and return the global GPIO number. The
 * translation assumes the standard two-cell form <&controller pin flags>,
 * which is what every controller these drivers name uses; a controller with a
 * custom of_xlate would need the real gpiod conversion.
 */
enum of_gpio_flags {
	OF_GPIO_ACTIVE_LOW		= 0x1,
	OF_GPIO_SINGLE_ENDED		= 0x2,
	OF_GPIO_OPEN_DRAIN		= 0x4,
	OF_GPIO_TRANSITORY		= 0x8,
	OF_GPIO_PULL_UP			= 0x10,
	OF_GPIO_PULL_DOWN		= 0x20,
	OF_GPIO_PULL_DISABLE		= 0x40,
};

static inline int of_get_named_gpio_flags(const struct device_node *np,
					  const char *propname, int index,
					  enum of_gpio_flags *flags)
{
	struct of_phandle_args args;
	struct gpio_device *gdev;
	struct gpio_desc *desc;
	int gpio, ret;

	ret = of_parse_phandle_with_args(np, propname, "#gpio-cells", index,
					 &args);
	if (ret)
		return ret;

	if (args.args_count < 1) {
		of_node_put(args.np);
		return -EINVAL;
	}

	gdev = gpio_device_find_by_fwnode(of_fwnode_handle(args.np));
	of_node_put(args.np);
	if (!gdev)
		return -EPROBE_DEFER;

	/*
	 * Rejects a pin past the controller's ngpio, which of_gpio_simple_xlate()
	 * did and a bare base + offset would not: the sum would name a line
	 * belonging to some other chip instead of failing.
	 */
	desc = gpio_device_get_desc(gdev, args.args[0]);
	if (IS_ERR(desc)) {
		gpio_device_put(gdev);
		return PTR_ERR(desc);
	}

	gpio = desc_to_gpio(desc);
	gpio_device_put(gdev);

	if (flags && args.args_count > 1)
		*flags = args.args[1];

	return gpio;
}

static inline int of_get_named_gpio(const struct device_node *np,
				    const char *propname, int index)
{
	return of_get_named_gpio_flags(np, propname, index, NULL);
}

/*
 * strncpy() is gone from the kernel, and its replacements each differ from it
 * where gbms_decode_eeprom_sn() uses it: strscpy_pad() guarantees a NUL and so
 * drops the last byte of a full-width field, and memcpy() does not stop at an
 * embedded NUL. That call copies a 23-byte barcode into a buffer the caller
 * terminates itself, so reproduce what strncpy() did.
 *
 * The other former strncpy() site, max77779_fg's serial_number, does NOT use
 * this: there the missing terminator is an out-of-bounds sysfs read, so it
 * takes strscpy() instead. See the comment there.
 */
static inline void gbms_strncpy(char *dst, const char *src, size_t count)
{
	size_t len = strnlen(src, count);

	memcpy(dst, src, len);
	memset(dst + len, 0, count - len);
}

/*
 * GPIOF_DIR_OUT and GPIOF_DIR_IN are gone from <linux/gpio/legacy.h>. Every
 * remaining use in these drivers is a gpio_chip .get_direction return value,
 * where GPIO_LINE_DIRECTION_OUT/_IN carry the same values and the same
 * meaning, so the call sites use those directly instead.
 */

/*
 * power_supply_get_by_phandle_array() took the FIRST phandle of the property
 * and then filled the caller's array with every power supply whose parent is
 * that device node, returning how many it found. The array is dimensioned by
 * power supplies per node, not by phandles -- which is why all three callers
 * (google_charger.c, google_dc_pps.c, p9221_charger.c) loop over the result
 * and keep the one whose name matches.
 *
 * Upstream's power_supply_get_by_reference() applies the same parent-fwnode
 * predicate but returns only the first match, and power_supply_class is not
 * reachable from a module, so the full walk cannot be reproduced here without
 * patching the power_supply core -- which ADR 0011 measured as unnecessary and
 * is worth keeping true.
 *
 * So this resolves at most one power supply per node. That is right on tegu
 * today, where mainline registers only "tcpm-source-psy-..." under
 * max77759tcpc@25; the vendor stack also registers "usb" there, and against
 * that the first match would be the wrong one and the right one would never be
 * seen. Revisit before a second power supply appears under a referenced node.
 */
static inline int of_power_supply_get_by_phandle_array(struct device_node *np,
						       const char *property,
						       struct power_supply **psy,
						       ssize_t size)
{
	struct power_supply *found;

	if (size < 1)
		return -EINVAL;

	found = power_supply_get_by_reference(of_fwnode_handle(np), property);
	if (IS_ERR(found))
		return PTR_ERR(found);
	if (!found)
		return -EAGAIN;

	psy[0] = found;

	return 1;
}

/*
 * The vendor sources take the partner's source capabilities through a pair
 * that allocates: tcpm_get_partner_src_caps() hands back a buffer and
 * tcpm_put_partner_src_caps() frees it. Downstream implements that pair in the
 * MAX77759 TCPC driver over a driver-global cache, ignoring the port argument
 * entirely.
 *
 * Our TCPM exports the upstream shape instead -- tcpm_get_partner_source_caps()
 * fills an array the caller owns, which needs no allocation, no free function
 * and no GFP argument for the 28 bytes PDO_MAX_OBJECTS costs. The vendor's
 * allocating contract is reproduced here so that their call sites are
 * unchanged and this stays the only place that knows about the difference.
 */
static inline int tcpm_get_partner_src_caps(struct tcpm_port *port,
					    u32 **src_pdo)
{
	u32 *pdo;
	int ret;

	pdo = kcalloc(PDO_MAX_OBJECTS, sizeof(*pdo), GFP_KERNEL);
	if (!pdo)
		return -ENOMEM;

	ret = tcpm_get_partner_source_caps(port, pdo, PDO_MAX_OBJECTS);
	if (ret < 0) {
		kfree(pdo);
		return ret;
	}

	*src_pdo = pdo;

	return ret;
}

static inline void tcpm_put_partner_src_caps(u32 **src_pdo)
{
	kfree(*src_pdo);
	*src_pdo = NULL;
}

#endif /* __GBMS_COMPAT_H_ */
