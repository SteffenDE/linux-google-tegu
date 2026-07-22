// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2022 Broadcom Corporation
 */
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <core.h>
#include <bus.h>
#include <fwvid.h>
#include <fwil.h>
#include <fweh.h>
#include <cfg80211.h>
#include <feature.h>

#include "vops.h"
#include "fwil_types.h"

#define BRCMF_WCC_E_LAST		213

/* Firmware event code that asks the host to start a SAE exchange. */
#define BRCMF_WCC_E_EXT_AUTH_START	194

static const struct brcmf_fweh_event_map brcmf_wcc_event_map = {
	.items = {
		{ BRCMF_E_EXT_AUTH_REQ, BRCMF_WCC_E_EXT_AUTH_START },
	},
	.n_items = 1,
};

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

/* Firmware asks the host to authenticate: hand the request to the supplicant
 * and pause the association until it has driven the SAE exchange.
 */
static s32
brcmf_wcc_notify_auth_start(struct brcmf_if *ifp,
			    const struct brcmf_event_msg *e, void *data)
{
	struct brcmf_pub *drvr = ifp->drvr;
	struct brcmf_auth_start_evt *evt = data;
	struct cfg80211_external_auth_params params;
	struct brcmf_assoc_mgr_cmd_le cmd;
	u32 ssid_len;
	int tries;
	int err;

	if (e->datalen < sizeof(*evt)) {
		bphy_err(drvr, "auth start event too short\n");
		return -EINVAL;
	}

	memset(&params, 0, sizeof(params));
	params.action = NL80211_EXTERNAL_AUTH_START;
	params.key_mgmt_suite = WLAN_AKM_SUITE_SAE;
	params.status = WLAN_STATUS_SUCCESS;
	ssid_len = min_t(u32, le32_to_cpu(evt->ssid.SSID_len),
			 IEEE80211_MAX_SSID_LEN);
	params.ssid.ssid_len = ssid_len;
	memcpy(params.ssid.ssid, evt->ssid.SSID, ssid_len);
	memcpy(params.bssid, e->addr, ETH_ALEN);

	/* The firmware can raise this event while the connect request that
	 * triggered it is still running (it fires during the join command,
	 * before cfg80211 has recorded the owning socket), in which case
	 * cfg80211_external_auth_request() rejects it with -EINVAL. Retry over
	 * a short window to let the connect path catch up, otherwise the
	 * firmware's paused authentication times out.
	 */
	for (tries = 0; tries < 20; tries++) {
		err = cfg80211_external_auth_request(ifp->ndev, &params,
						     GFP_KERNEL);
		if (err != -EINVAL)
			break;
		msleep(10);
	}
	if (err) {
		bphy_err(drvr, "failed to hand SAE auth to the supplicant (%d)\n",
			 err);
		return err;
	}

	cmd.version = cpu_to_le16(BRCMF_ASSOC_MGR_VERSION_0);
	cmd.length = cpu_to_le16(sizeof(cmd));
	cmd.cmd = cpu_to_le16(BRCMF_ASSOC_MGR_CMD_PAUSE_ON_EVT);
	cmd.params = cpu_to_le16(BRCMF_ASSOC_MGR_PARAMS_PAUSE_EVENT_AUTH_RESP);
	err = brcmf_fil_iovar_data_set(ifp, "assoc_mgr_cmd", &cmd, sizeof(cmd));
	if (err)
		bphy_err(drvr, "failed to pause association (%d)\n", err);

	return err;
}

/* Firmware forwards a received SAE authentication frame: wrap it back into an
 * 802.11 management frame and pass it up to the supplicant.
 */
static s32
brcmf_wcc_notify_auth_rx(struct brcmf_if *ifp,
			 const struct brcmf_event_msg *e, void *data)
{
	struct brcmf_pub *drvr = ifp->drvr;
	struct brcmf_cfg80211_info *cfg = drvr->config;
	struct wireless_dev *wdev = &ifp->vif->wdev;
	struct ieee80211_mgmt *mgmt;
	struct brcmu_chan ch;
	u32 chanspec, mgmt_len, hdr_len;
	s32 freq;
	int err;

	if (e->auth_type != BRCMF_AUTH_ALGO_SAE)
		return 0;

	if (!e->datalen) {
		bphy_err(drvr, "SAE auth frame event has no payload\n");
		return -EINVAL;
	}

	hdr_len = offsetof(struct ieee80211_mgmt, u);
	mgmt_len = hdr_len + e->datalen;
	mgmt = kzalloc(mgmt_len, GFP_KERNEL);
	if (!mgmt)
		return -ENOMEM;

	mgmt->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT |
					  IEEE80211_STYPE_AUTH);
	memcpy(mgmt->da, ifp->mac_addr, ETH_ALEN);
	memcpy(mgmt->sa, e->addr, ETH_ALEN);
	if (brcmf_fil_cmd_data_get(ifp, BRCMF_C_GET_BSSID, mgmt->bssid, ETH_ALEN))
		memcpy(mgmt->bssid, e->addr, ETH_ALEN);
	memcpy((u8 *)mgmt + hdr_len, data, e->datalen);

	err = brcmf_fil_iovar_int_get(ifp, "chanspec", &chanspec);
	if (err) {
		bphy_err(drvr, "could not read chanspec (%d)\n", err);
		kfree(mgmt);
		return err;
	}

	ch.chspec = (u16)chanspec;
	cfg->d11inf.decchspec(&ch);
	freq = ieee80211_channel_to_frequency(ch.control_ch_num,
			ch.band == BRCMU_CHAN_BAND_2G ? NL80211_BAND_2GHZ :
			ch.band == BRCMU_CHAN_BAND_6G ? NL80211_BAND_6GHZ :
			NL80211_BAND_5GHZ);

	cfg80211_rx_mgmt(wdev, freq, 0, (u8 *)mgmt, mgmt_len,
			 NL80211_RXMGMT_FLAG_EXTERNAL_AUTH);
	kfree(mgmt);
	return 0;
}

static void brcmf_wcc_register_event_handlers(struct brcmf_pub *drvr)
{
	struct brcmf_if *ifp = brcmf_get_ifp(drvr, 0);

	if (!ifp || !brcmf_feat_is_enabled(ifp, BRCMF_FEAT_SAE_EXT))
		return;

	brcmf_fweh_register(drvr, BRCMF_E_EXT_AUTH_REQ,
			    brcmf_wcc_notify_auth_start);
	brcmf_fweh_register(drvr, BRCMF_E_AUTH, brcmf_wcc_notify_auth_rx);
}

static int brcmf_wcc_alloc_fweh_info(struct brcmf_pub *drvr)
{
	struct brcmf_fweh_info *fweh;

	fweh = kzalloc_flex(*fweh, evt_handler, BRCMF_WCC_E_LAST);
	if (!fweh)
		return -ENOMEM;

	fweh->num_event_codes = BRCMF_WCC_E_LAST;
	fweh->event_map = &brcmf_wcc_event_map;
	drvr->fweh = fweh;
	return 0;
}

const struct brcmf_fwvid_ops brcmf_wcc_ops = {
	.set_sae_password = brcmf_wcc_set_sae_pwd,
	.alloc_fweh_info = brcmf_wcc_alloc_fweh_info,
	.get_cfg80211_ops = brcmf_wcc_get_cfg80211_ops,
	.register_event_handlers = brcmf_wcc_register_event_handlers,
};
