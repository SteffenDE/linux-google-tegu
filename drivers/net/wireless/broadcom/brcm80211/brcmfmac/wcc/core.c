// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2022 Broadcom Corporation
 */
#include <linux/errno.h>
#include <linux/types.h>
#include <core.h>
#include <bus.h>
#include <fwvid.h>
#include <fwil.h>
#include <cfg80211.h>
#include <feature.h>

#include "vops.h"
#include "fwil_types.h"

#define BRCMF_WCC_E_LAST		213

static int brcmf_wcc_set_sae_pwd(struct brcmf_if *ifp,
				 struct cfg80211_crypto_settings *crypto)
{
	return brcmf_set_wsec(ifp, crypto->sae_pwd, crypto->sae_pwd_len,
			      BRCMF_WSEC_PASSPHRASE);
}

/* Relay an outbound SAE authentication frame produced by the host supplicant
 * to the firmware. Non-authentication management frames take the default path.
 */
static int
brcmf_wcc_mgmt_tx(struct wiphy *wiphy, struct wireless_dev *wdev,
		  struct cfg80211_mgmt_tx_params *params, u64 *cookie)
{
	struct brcmf_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	const struct ieee80211_mgmt *mgmt = (const struct ieee80211_mgmt *)params->buf;
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_assoc_mgr_cmd_le *cmd;
	u32 cmd_len;
	bool ack = false;
	int err;

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);

	if (!ieee80211_is_auth(mgmt->frame_control) ||
	    !brcmf_feat_is_enabled(vif->ifp, BRCMF_FEAT_SAE_EXT))
		return brcmf_cfg80211_mgmt_tx(wiphy, wdev, params, cookie);

	*cookie = 0;
	cmd_len = sizeof(*cmd) + params->len;
	cmd = kzalloc(cmd_len, GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	cmd->version = cpu_to_le16(BRCMF_ASSOC_MGR_VERSION_0);
	cmd->length = cpu_to_le16(params->len);
	cmd->cmd = cpu_to_le16(BRCMF_ASSOC_MGR_CMD_SEND_AUTH);
	memcpy(&cmd->params, params->buf, params->len);

	err = brcmf_fil_iovar_data_set(vif->ifp, "assoc_mgr_cmd", cmd, cmd_len);
	if (err)
		bphy_err(cfg->pub, "failed to relay SAE auth frame (%d)\n", err);
	else
		ack = true;

	kfree(cmd);
	cfg80211_mgmt_tx_status(wdev, *cookie, params->buf, params->len, ack,
				GFP_KERNEL);
	return 0;
}

/* Report the host supplicant's SAE result to the firmware: resume the paused
 * association on success, abort it on failure.
 */
static int
brcmf_wcc_external_auth(struct wiphy *wiphy, struct net_device *dev,
			struct cfg80211_external_auth_params *params)
{
	struct brcmf_if *ifp = netdev_priv(dev);
	struct brcmf_pub *drvr = ifp->drvr;
	struct brcmf_assoc_mgr_cmd_le cmd;
	int err;

	if (params->status &&
	    params->status != WLAN_STATUS_SAE_HASH_TO_ELEMENT &&
	    params->status != WLAN_STATUS_SAE_PK) {
		bphy_err(drvr, "SAE authentication failed: status=%u\n",
			 params->status);
		return brcmf_fil_cmd_data_set(ifp, BRCMF_C_DISASSOC, NULL, 0);
	}

	cmd.version = cpu_to_le16(BRCMF_ASSOC_MGR_VERSION_0);
	cmd.length = cpu_to_le16(sizeof(cmd));
	cmd.cmd = cpu_to_le16(BRCMF_ASSOC_MGR_CMD_PAUSE_ON_EVT);
	cmd.params = cpu_to_le16(BRCMF_ASSOC_MGR_PARAMS_EVENT_NONE);
	err = brcmf_fil_iovar_data_set(ifp, "assoc_mgr_cmd", &cmd, sizeof(cmd));
	if (err)
		bphy_err(drvr, "failed to resume association (%d)\n", err);

	return err;
}

static void brcmf_wcc_get_cfg80211_ops(struct brcmf_pub *drvr)
{
	drvr->ops->mgmt_tx = brcmf_wcc_mgmt_tx;
	drvr->ops->external_auth = brcmf_wcc_external_auth;
}

static int brcmf_wcc_alloc_fweh_info(struct brcmf_pub *drvr)
{
	struct brcmf_fweh_info *fweh;

	fweh = kzalloc_flex(*fweh, evt_handler, BRCMF_WCC_E_LAST);
	if (!fweh)
		return -ENOMEM;

	fweh->num_event_codes = BRCMF_WCC_E_LAST;
	drvr->fweh = fweh;
	return 0;
}

const struct brcmf_fwvid_ops brcmf_wcc_ops = {
	.set_sae_password = brcmf_wcc_set_sae_pwd,
	.alloc_fweh_info = brcmf_wcc_alloc_fweh_info,
	.get_cfg80211_ops = brcmf_wcc_get_cfg80211_ops,
};
