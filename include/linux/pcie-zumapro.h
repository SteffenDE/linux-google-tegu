/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Bring-up hooks the Google Zumapro PCIe root-complex driver exports for the
 * Samsung Exynos Modem 5300 boot driver.  The modem boot protocol needs two
 * things no other endpoint does: the RC's MSI target address moved into the
 * modem's MSI carveout (the mask ROM derives where to report boot progress
 * from its MSI capability address), and a mid-boot link bounce (the CP
 * bootloader re-links after the first-stage download).  Both callers pass the
 * RC's platform device resolved from a DT phandle.
 */
#ifndef _LINUX_PCIE_ZUMAPRO_H
#define _LINUX_PCIE_ZUMAPRO_H

#include <linux/types.h>

struct device;

#if IS_ENABLED(CONFIG_PCIE_ZUMAPRO)
int zumapro_pcie_set_msi_target(struct device *rc_dev, phys_addr_t target);
int zumapro_pcie_reserve_msi_base(struct device *rc_dev, unsigned int count);
int zumapro_pcie_modem_link_down(struct device *rc_dev);
int zumapro_pcie_modem_link_up(struct device *rc_dev);
int zumapro_pcie_modem_wake(struct device *rc_dev);
#else
static inline int zumapro_pcie_set_msi_target(struct device *rc_dev,
					      phys_addr_t target)
{
	return -ENODEV;
}
static inline int zumapro_pcie_reserve_msi_base(struct device *rc_dev,
						unsigned int count)
{
	return -ENODEV;
}
static inline int zumapro_pcie_modem_link_down(struct device *rc_dev)
{
	return -ENODEV;
}
static inline int zumapro_pcie_modem_link_up(struct device *rc_dev)
{
	return -ENODEV;
}
static inline int zumapro_pcie_modem_wake(struct device *rc_dev)
{
	return -ENODEV;
}
#endif

#endif /* _LINUX_PCIE_ZUMAPRO_H */
