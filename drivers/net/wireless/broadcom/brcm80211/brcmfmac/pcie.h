// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2014 Broadcom Corporation
 */
#ifndef BRCMFMAC_PCIE_H
#define BRCMFMAC_PCIE_H

#include <linux/types.h>

struct brcmf_bus;

struct brcmf_pciedev {
	struct brcmf_bus *bus;
	struct brcmf_pciedev_info *devinfo;
};

void brcmf_pcie_handle_mbdata(struct brcmf_bus *bus, u32 mb_data);
int brcmf_pcie_pm_enter_active(struct brcmf_bus *bus);
void brcmf_pcie_pm_leave_active(struct brcmf_bus *bus);

#endif /* BRCMFMAC_PCIE_H */
