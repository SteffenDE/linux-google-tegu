/* SPDX-License-Identifier: ISC */
/*
 * Copyright (c) 2022 Broadcom Corporation
 */

#ifndef WCC_FWIL_TYPES_H_
#define WCC_FWIL_TYPES_H_

#include <fwil_types.h>

/* Host-driven SAE: the firmware relays the authentication exchange through the
 * "assoc_mgr_cmd" iovar and pauses the association until the host supplicant
 * has completed it.
 */
#define BRCMF_ASSOC_MGR_VERSION_0			0
#define BRCMF_ASSOC_MGR_CMD_PAUSE_ON_EVT		0
#define BRCMF_ASSOC_MGR_CMD_SEND_AUTH			3
#define BRCMF_ASSOC_MGR_PARAMS_EVENT_NONE		0
#define BRCMF_ASSOC_MGR_PARAMS_PAUSE_EVENT_AUTH_RESP	1

/* 802.11 authentication algorithm carried in the auth firmware event. */
#define BRCMF_AUTH_ALGO_SAE				3

/**
 * struct brcmf_assoc_mgr_cmd_le - "assoc_mgr_cmd" iovar payload.
 *
 * @version: iovar version (BRCMF_ASSOC_MGR_VERSION_0).
 * @length: length of the parameter payload meaningful to @cmd.
 * @cmd: requested operation.
 * @params: fixed 16-bit parameter, or the first bytes of a variable payload
 *	(the raw authentication frame for BRCMF_ASSOC_MGR_CMD_SEND_AUTH).
 */
struct brcmf_assoc_mgr_cmd_le {
	__le16 version;
	__le16 length;
	__le16 cmd;
	__le16 params;
};

/**
 * struct brcmf_auth_start_evt - payload of the auth-start firmware event.
 *
 * @version: event structure version.
 * @len: structure length.
 * @ssid: SSID of the BSS being authenticated to.
 * @bssid: BSSID of the BSS.
 * @pad: alignment padding.
 * @key_mgmt_suite: AKM suite selector.
 * @opt_tlvs: optional trailing TLVs.
 */
struct brcmf_auth_start_evt {
	__le16 version;
	__le16 len;
	struct brcmf_ssid_le ssid;
	u8 bssid[ETH_ALEN];
	u8 pad[2];
	__le32 key_mgmt_suite;
	u8 opt_tlvs[];
};

#endif /* WCC_FWIL_TYPES_H_ */
