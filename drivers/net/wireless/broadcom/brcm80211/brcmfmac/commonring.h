// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2014 Broadcom Corporation
 */
#ifndef BRCMFMAC_COMMONRING_H
#define BRCMFMAC_COMMONRING_H

/*
 * H2D ring phase, carried in msgbuf_common_hdr.flags.  It marks which lap of
 * the ring an item belongs to, so the device can tell an item the host wrote
 * this lap from a stale one left over from the previous lap -- something the
 * write index alone cannot express.  The device only looks at it once the host
 * advertises BRCMF_HOSTCAP_H2D_VALID_PHASE.
 *
 * BRCMF_COMMONRING_PHASE_INIT is the value the device expects on the first lap:
 * BCM4383 firmware seeds its own copy with 0x80 when it creates a flow ring
 * (pciedev_process_flow_ring_create_rqst).  Note bcmdhd initialises its H2D
 * rings to 0 instead, which disagrees; if the device turns out to expect 0,
 * this is the one value to flip.  A wrong choice is not silently wrong -- every
 * item mismatches and the firmware logs each one.
 */
#define BRCMF_COMMONRING_PHASE_BIT	0x80
#define BRCMF_COMMONRING_PHASE_INIT	0x80

struct brcmf_commonring {
	u16 r_ptr;
	u16 w_ptr;
	u16 f_ptr;
	u32 seqnum;
	u16 depth;
	u16 item_len;

	/*
	 * @phase: the lap marker for the slot the write pointer is about to
	 *	hand out, flipped every time it wraps.
	 * @item_phase: @phase captured when the current reservation was made,
	 *	i.e. the marker belonging to the items being filled in now.
	 *	Reservations never straddle a wrap (see
	 *	brcmf_commonring_reserve_for_write_multiple()), so one value
	 *	covers every item in a reservation.
	 */
	u8 phase;
	u8 item_phase;

	void *buf_addr;

	int (*cr_ring_bell)(void *ctx);
	int (*cr_update_rptr)(void *ctx);
	int (*cr_update_wptr)(void *ctx);
	int (*cr_write_rptr)(void *ctx);
	int (*cr_write_wptr)(void *ctx);

	void *cr_ctx;

	spinlock_t lock;
	unsigned long flags;
	bool inited;
	bool was_full;

	atomic_t outstanding_tx;
};


void brcmf_commonring_register_cb(struct brcmf_commonring *commonring,
				  int (*cr_ring_bell)(void *ctx),
				  int (*cr_update_rptr)(void *ctx),
				  int (*cr_update_wptr)(void *ctx),
				  int (*cr_write_rptr)(void *ctx),
				  int (*cr_write_wptr)(void *ctx), void *ctx);
void brcmf_commonring_config(struct brcmf_commonring *commonring, u16 depth,
			     u16 item_len, void *buf_addr);
void brcmf_commonring_lock(struct brcmf_commonring *commonring);
void brcmf_commonring_unlock(struct brcmf_commonring *commonring);
bool brcmf_commonring_write_available(struct brcmf_commonring *commonring);
void *brcmf_commonring_reserve_for_write(struct brcmf_commonring *commonring);
void *
brcmf_commonring_reserve_for_write_multiple(struct brcmf_commonring *commonring,
					    u16 n_items, u16 *alloced);
int brcmf_commonring_write_complete(struct brcmf_commonring *commonring);
void brcmf_commonring_write_cancel(struct brcmf_commonring *commonring,
				   u16 n_items);
void *brcmf_commonring_get_read_ptr(struct brcmf_commonring *commonring,
				    u16 *n_items);
int brcmf_commonring_read_complete(struct brcmf_commonring *commonring,
				   u16 n_items);

#define brcmf_commonring_n_items(commonring) (commonring->depth)
#define brcmf_commonring_len_item(commonring) (commonring->item_len)


#endif /* BRCMFMAC_COMMONRING_H */
