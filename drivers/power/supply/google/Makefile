# SPDX-License-Identifier: GPL-2.0
#
# Google Battery Management System (GBMS)
#
# Ported from the Pixel 9a (tegu) AOSP tree, private/google-modules/bms.
# The sources are carried as close to the vendor snapshot as possible so that
# a later snapshot can be diffed against this directory; only this Makefile
# and Kconfig are written for the in-tree build.
#

# The vendor sources include their own headers as <misc/gvotable.h> etc.,
# relative to the root of the bms module.
ccflags-y += -I$(src)

# Shared infrastructure, used by the BMS drivers and nothing else.
obj-$(CONFIG_GOOGLE_VOTABLE)		+= gvotable.o
gvotable-y				:= misc/gvotable.o

obj-$(CONFIG_GOOGLE_LOGBUFFER)		+= logbuffer.o
logbuffer-y				:= misc/logbuffer.o

# GBMS core: the property extension, the tag storage and the pack EEPROM.
obj-$(CONFIG_GOOGLE_BMS)		+= google-bms.o
google-bms-y				:= google_bms.o
google-bms-y				+= gbms_storage.o
google-bms-y				+= google_eeprom.o
google-bms-y				+= google_eeprom_01.o
google-bms-y				+= google_eeprom_02.o

obj-$(CONFIG_GOOGLE_BATTERY)		+= google-battery.o
google-battery-y			:= google_battery.o
google-battery-y			+= google_ttf.o

obj-$(CONFIG_GOOGLE_CHARGER)		+= google-charger.o
google-charger-y			:= google_charger.o
google-charger-y			+= google_dc_pps.o

obj-$(CONFIG_GOOGLE_CCD)		+= google_ccd.o

# Not a vendor file: local glue, so mainline's TCPM can supply the buck-mode
# ballot the vendor's port controller driver would have cast.
obj-$(CONFIG_GOOGLE_TCPM_BUCK)		+= google-tcpm-buck.o
google-tcpm-buck-y			:= google_tcpm_buck.o

# MAX77779 IF-PMIC: charger, fuel gauge, V/I monitor, scratchpad, I2C master
# and the PMIC core that owns their shared IRQ.
obj-$(CONFIG_CHARGER_MAX77779)		+= max77779-charger.o
max77779-charger-y			:= max77779_charger.o
max77779-charger-y			+= max77779_usecase.o

obj-$(CONFIG_CHARGER_MAX77779_I2C)	+= max77779-charger-i2c.o
max77779-charger-i2c-y			:= max77779_charger_i2c.o

obj-$(CONFIG_FG_MAX77779)		+= max77779-fg-gbms.o
max77779-fg-gbms-y			:= max77779_fg.o
max77779-fg-gbms-y			+= max77779_fg_model.o
max77779-fg-gbms-y			+= maxfg_common.o

obj-$(CONFIG_FG_MAX77779_I2C)		+= max77779-fg-i2c.o
max77779-fg-i2c-y			:= max77779_fg_i2c.o

obj-$(CONFIG_VIMON_MAX77779)		+= max77779-vimon.o
max77779-vimon-y			:= max77779_vimon.o

obj-$(CONFIG_VIMON_MAX77779_I2C)	+= max77779-vimon-i2c.o
max77779-vimon-i2c-y			:= max77779_vimon_i2c.o

obj-$(CONFIG_MAX77779_SP)		+= max77779-sp.o
max77779-sp-y				:= max77779_sp.o

obj-$(CONFIG_MAX77779_SP_I2C)		+= max77779-sp-i2c.o
max77779-sp-i2c-y			:= max77779_sp_i2c.o

obj-$(CONFIG_MAX77779_I2CM)		+= max77779_i2cm.o
obj-$(CONFIG_MAX77779_I2CM_I2C)		+= max77779_i2cm_i2c.o

obj-$(CONFIG_MAX77779_PMIC)		+= max77779_pmic.o
obj-$(CONFIG_MAX77779_PMIC_I2C)		+= max77779_pmic_i2c.o
obj-$(CONFIG_MAX77779_PMIC_IRQ)		+= max77779_pmic_irq.o
obj-$(CONFIG_MAX77779_PMIC_PINCTRL)	+= max77779_pmic_pinctrl.o
obj-$(CONFIG_MAX77779_PMIC_SGPIO)	+= max77779_pmic_sgpio.o

# Second wired charger on the tegu board, reached through google_ccd.
obj-$(CONFIG_CHARGER_RT9471_GBMS)	+= rt9471_charger.o

# Wireless receiver.
obj-$(CONFIG_CHARGER_P9221)		+= p9221.o
p9221-y					:= p9221_charger.o
p9221-y					+= p9221_chip.o

# The vendor's own warning suppressions, carried over from
# private/google-modules/bms/Makefile.
#
# -Wno-enum-conversion and -Wno-switch are structural rather than sloppiness:
# GBMS extends enum power_supply_property with enum gbms_property (its values
# start at POWER_SUPPLY_PROP_SERIAL_NUMBER + 100), so every switch over a
# property and every call that passes one warns by construction.
WENUMS = -Wno-enum-conversion -Wno-switch

CFLAGS_google_bms.o		+= -Wno-enum-conversion
CFLAGS_google_battery.o		+= $(WENUMS)
CFLAGS_google_ttf.o		+= -Wno-format
CFLAGS_google_charger.o		+= -Wno-enum-conversion
CFLAGS_google_ccd.o		+= $(WENUMS)
CFLAGS_max77779_charger.o	+= -Wno-unused-function $(WENUMS)
CFLAGS_max77779_fg.o		+= -Wno-unused-function $(WENUMS)
CFLAGS_max77779_vimon.o		+= -Wno-unused-function $(WENUMS)
CFLAGS_max77779_sp.o		+= -Wno-unused-function $(WENUMS)
CFLAGS_p9221_charger.o		+= $(WENUMS)
