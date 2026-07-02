// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2014 Broadcom Corporation
 */
#ifndef BRCMFMAC_PCIE_H
#define BRCMFMAC_PCIE_H

struct brcmf_bus;

struct brcmf_pciedev {
	struct brcmf_bus *bus;
	struct brcmf_pciedev_info *devinfo;
};

/* Bracket H2D ring submissions so in-band device-sleep can wake the device
 * before access and let it micro-sleep again when idle.  Submissions made
 * from the PCIe IRQ thread (rx buffer reposts) must use the noblock variant:
 * that thread delivers the wake acknowledgment, so waiting for it there
 * deadlocks until timeout.
 */
int brcmf_pcie_pm_enter_active(struct brcmf_bus *bus);
int brcmf_pcie_pm_enter_active_noblock(struct brcmf_bus *bus);
void brcmf_pcie_pm_leave_active(struct brcmf_bus *bus);
bool brcmf_pcie_pm_attach_hold(struct brcmf_bus *bus);

#endif /* BRCMFMAC_PCIE_H */
