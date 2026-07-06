/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Google Zumapro PCIe PHY hook for the root-complex driver: the PHY owns the
 * SoC-control block whose clock mux must be parked on the always-on OSC
 * before an intentional link drop (an ELBI/DBI access on the CLKREQ#-gated
 * clock stalls the interconnect).
 */
#ifndef _LINUX_PHY_PHY_ZUMAPRO_PCIE_H
#define _LINUX_PHY_PHY_ZUMAPRO_PCIE_H

struct phy;

#if IS_ENABLED(CONFIG_PHY_ZUMAPRO_PCIE)
void zumapro_pcie_phy_safe_clk(struct phy *phy, bool safe);
void zumapro_pcie_phy_keep_refclk(struct phy *phy, bool keep);
#else
static inline void zumapro_pcie_phy_safe_clk(struct phy *phy, bool safe)
{
}
static inline void zumapro_pcie_phy_keep_refclk(struct phy *phy, bool keep)
{
}
#endif

#endif /* _LINUX_PHY_PHY_ZUMAPRO_PCIE_H */
