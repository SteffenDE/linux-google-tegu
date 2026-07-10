// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos Modem 5300 PCIe boot transport (Google Tensor G4 boards).
 *
 * This driver is the *transport* half of the CP boot: it owns the PCIe
 * endpoint, the doorbell, the MSI carveout and the shared-memory download
 * ring, and exposes them through /dev/umts_boot0.  It carries bytes; it does
 * not parse firmware.  A userspace helper (cbd / cbd-lite) sources the factory
 * image and the per-device NV/handover from the vendor partitions and speaks
 * the SIT "std_dl" download protocol over the chardev.  The split matches
 * downstream cpif (google-modules/radio/samsung/s5300): the kernel adds the
 * 12-byte EXYNOS link header and manages the ring; userspace builds every
 * std_dl frame (including the MAIN CRC-verify frame).
 *
 * The whole ~100 MB firmware is streamed on every cold boot -- MAIN is not
 * resident in the modem's DRAM.  (An earlier assumption that MAIN persisted
 * across boots was a wrong-instrumentation artifact: only the one-shot PBL
 * load was counted, never the std_dl stream.)
 *
 * Boot flow (userspace drives each step; the kernel executes it):
 *  1. The RC driver has rail-cycled the CP and trained the link; the mask ROM
 *     enumerates as 144d:a5a5 and parks waiting for the doorbell.  probe()
 *     claims the endpoint, forces the doorbell BAR, moves the RC MSI target
 *     into the carveout, allocates the EP MSI vectors and registers /dev.
 *  2. IOCTL_HANDOVER_BLOCK_INFO stages the 161-byte handover block (IMEIs +
 *     signature) the CP reads during boot.
 *  3. IOCTL_POWER_ON publishes the srinfo/capability pointers and clears the
 *     control words.
 *  4. IOCTL_LOAD_CP_IMAGE copies the first-stage bootloader (PBL) into the IPC
 *     carveout.  IOCTL_START_CP_BOOTLOADER arms the boot ring (magic 0xBDBD),
 *     publishes the PBL address through the MSI block, rings the doorbell,
 *     polls boot_stage to DONE, then bounces the link so the CP's BL1 download
 *     server comes up.
 *  5. write()/read() stream the std_dl frames: each write() is one frame that
 *     the kernel wraps in an EXYNOS header and copies onto the NORM_RAW txq;
 *     the CP writes 4-byte acks onto the rxq and raises MSI, and read()
 *     returns them (link header stripped).
 *  6. IOCTL_COMPLETE_NORMAL_BOOTUP waits for the CP's INIT_START/PHONE_START
 *     handshake; once MAIN is running the CP goes ONLINE (status 4).
 *
 * Once ONLINE the CP speaks the SIT control protocol on the legacy FMT queue
 * (downstream io-device umts_ipc0, channel 0xF5).  That is exposed as a WWAN
 * SIT port: userspace (a RIL/ModemManager plugin) writes and reads bare SIT
 * app messages, and this driver wraps them in the 12-byte EXYNOS link header,
 * moves them across the FMT ring and rings the CP's data doorbell.  The bulk
 * data path (PKTPROC rings -> netdev) is still to come.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/if_arp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/pci.h>
#include <linux/pcie-zumapro.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sizes.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wwan.h>

#define S5300_PCI_VENDOR_ID		0x144d
#define S5300_PCI_DEVICE_ID		0xa5a5

/*
 * MSI capability offset in the mask ROM's config space (DesignWare EP
 * default; downstream hardcodes it, print_msi_register()).
 */
#define S5300_ROM_MSI_CAP		0x50

/*
 * The 4K MSI carveout doubles as the boot status block (downstream
 * modem_ctrl.h struct msi_reg_type).  Offset 0 is the MSI termination
 * address; the fields above it are plain DMA targets.
 */
#define S5300_MSI_ERR_REPORT		0x08
#define S5300_MSI_BOOT_STAGE		0x10
#define S5300_MSI_IMG_ADDR_LO		0x14
#define S5300_MSI_IMG_ADDR_HI		0x18
#define S5300_MSI_IMG_SIZE		0x1c

/* All enum boot_stage_bit stages set, ROM through BL1 jump. */
#define S5300_BOOT_STAGE_DONE		0x3fff

/*
 * The mask ROM only tolerates MME=2 (4 vectors); the RC reserves vectors 0-3
 * for itself first (zumapro_pcie_reserve_msi_base) so the modem lands at data
 * base 4.  MAIN fires its post-link-ack notify on message 4 = the EP's vector
 * 0, so request_irq() on vector 0 catches both the ROM ack and MAIN's
 * INIT_START.
 */
#define S5300_MSI_VECTORS		4

/*
 * IPC region layout (downstream create_legacy_link_device() with tegu's DT
 * offsets, and the DRAM_V1 control-message words).
 */
#define S5300_IPC_MAGIC			0x00
#define S5300_IPC_ACCESS		0x04
#define S5300_IPC_Q_HEAD_TAIL		0x08	/* 8 words: FMT/RAW head+tail */
#define S5300_IPC_Q_WORDS		8
#define S5300_IPC_SRINFO_OFS_PTR	0x64
#define S5300_IPC_CAP_OFS_PTR		0x70
#define S5300_IPC_CAP_BASE		0xa0
#define S5300_IPC_CAP_WORDS		4	/* AP cap x2, CP cap x2 */

/*
 * AP/CP capability words at capability_offset (dumped.dts capability_offset =
 * 0xa0), interleaved [ap0][cp0][ap1][cp1] (downstream layout,
 * AP_CP_CAP_PART_LEN*2*part + {0, PART_LEN}).  The CP negotiates against these:
 * observed on downstream, the CP only starts servicing runtime IPC (sending on
 * umts_ipc0, draining the FMT ring) after INIT_START publishes them.
 */
#define S5300_IPC_CAP_AP0		(S5300_IPC_CAP_BASE + 0x0)
#define S5300_IPC_CAP_CP0		(S5300_IPC_CAP_BASE + 0x4)
#define S5300_IPC_CAP_AP1		(S5300_IPC_CAP_BASE + 0x8)
#define S5300_IPC_CAP_CP1		(S5300_IPC_CAP_BASE + 0xc)

/*
 * AP capability part 0 (downstream set_ap_capabilities()): bit0 PKTPROC_UL,
 * bit1 CH_EXTENSION, bit2 PKTPROC_36BIT.  We advertise 0x3 (PKTPROC_UL |
 * CH_EXTENSION): the CP reads bit0 as "AP does UL over pktproc" and consumes
 * our UL rings, so it is published (at INIT_START) only after
 * s5300_pktproc_ul_setup() has provisioned both UL queues; the CP then fills in
 * end_bit_owner/cp_quota, which s5300_pktproc_ul_activate() reads at
 * PHONE_START.  No PCIe IOMMU is needed on tegu (hw_iocc + fixed reserved
 * buffers).
 *
 * 2026-07-08: an earlier control-channel stall traced to the *DL* info block
 * (num_queues=1/shared, a mode this firmware never runs), compounded by a
 * sit-smoke artifact (a redundant SET_RADIO_POWER on an already-on radio whose
 * response the CP defers a park cycle).  With DL corrected to 4-queue/exclusive
 * the control plane + data call are healthy on HW at 0x2, so advertise 0x3 to
 * bring up the UL data path.
 */
#define S5300_AP_CAPABILITY_0		0x3
#define S5300_IPC_AP2CP_MSG		0x800
#define S5300_IPC_CP2AP_MSG		0x804
#define S5300_IPC_AP2CP_STATUS		0x808
#define S5300_IPC_CP2AP_STATUS		0x80c
/*
 * ap2cp_united_status ds_det field (downstream sbi_ds_det_pos=14, mask 0x3;
 * get_ds_detect() returns 1 on this device).  Load-bearing for runtime IPC --
 * see s5300_init_control_messages().
 */
#define S5300_IPC_DS_DET		(1 << 14)
/*
 * cp2ap_united_status bit 2: the global TX flow-control gate (downstream
 * SHM_FLOWCTL_BIT, shmem_tx_state_handler).  The CP updates the word and rings
 * MSI vector 1: 1 = suspend all uplink, 0 = resume.
 */
#define S5300_CP2AP_FLOWCTL		BIT(2)
/* ap2cp_handover_block_info = <DRAM_V1 2092> (zuma-cp-s5300-sit.dtsi). */
#define S5300_IPC_HANDOVER		0x82c
#define S5300_HANDOVER_SIZE		161	/* sizeof(t_handover_block_info) */

#define S5300_IPC_SRINFO_OFFSET		0x400000
#define S5300_IPC_MAGIC_ONLINE		0xaa	/* SHM_IPC_MAGIC (running) */
#define S5300_IPC_MAGIC_BOOT		0xbdbd	/* SHM_BOOT_MAGIC (download) */

/*
 * NORM_RAW legacy ring, the boot std_dl channel (downstream
 * create_legacy_link_device() with tegu's legacy_raw_* offsets).  head/tail
 * are byte offsets into the respective buffer.  txq is AP->CP (head owned by
 * AP), rxq is CP->AP (head owned by CP).
 */
#define S5300_RAW_TXQ_HEAD		0x18
#define S5300_RAW_TXQ_TAIL		0x1c
#define S5300_RAW_RXQ_HEAD		0x20
#define S5300_RAW_RXQ_TAIL		0x24
#define S5300_RAW_BUF_OFFSET		0x3000
#define S5300_RAW_TXQ_SIZE		0x1fd000
#define S5300_RAW_RXQ_OFFSET		(S5300_RAW_BUF_OFFSET + S5300_RAW_TXQ_SIZE)
#define S5300_RAW_RXQ_SIZE		0x200000

/*
 * Legacy FMT queue, the runtime SIT control channel (downstream
 * create_legacy_link_device() with tegu's legacy_fmt_* offsets).  Same ring
 * shape as NORM_RAW, its own 4K TX/RX buffers just below the RAW buffers.
 */
#define S5300_FMT_TXQ_HEAD		0x08
#define S5300_FMT_TXQ_TAIL		0x0c
#define S5300_FMT_RXQ_HEAD		0x10
#define S5300_FMT_RXQ_TAIL		0x14
#define S5300_FMT_TXQ_OFFSET		0x1000
#define S5300_FMT_TXQ_SIZE		0x1000
#define S5300_FMT_RXQ_OFFSET		0x2000
#define S5300_FMT_RXQ_SIZE		0x1000
#define S5300_FMT_CH			0xf5	/* EXYNOS_CH_ID_FMT_0 (umts_ipc0) */

/* Largest SIT app message the corpus shows is the 983-byte setup-data-call. */
#define S5300_FMT_MAX			SZ_2K

/*
 * OEM/GEMS channel (downstream io-device oem_ipc, an IPC_FMT multi-channel iod
 * based at EXYNOS_CH_ID_OEM_0 = 0x81, so oem_ipc1 = 0x82) transported on the
 * legacy FMT ring post-ONLINE, same frame shape as the SIT control channel --
 * only the channel byte differs.  The CP streams UE-capability-config file
 * requests here; left unanswered its internal LTE-RRC message queue overflows
 * and the modem asserts (PAL_QUEUE_FULL).  A userspace daemon answers it via
 * /dev/umts_oem1.  Config chunks are ~4 KB, so a frame fills most of one 4 KB
 * FMT ring slot: cap the payload so header+payload+pad still fits the ring
 * (leaving the circ one-slot gap).
 */
#define S5300_OEM_CH			0x82	/* EXYNOS_CH_ID_OEM_0 + 1 (oem_ipc1) */
#define S5300_OEM_MAX			(S5300_FMT_TXQ_SIZE - S5300_HDR_SIZE - 8)
#define S5300_OEM_RXQ_MAX		64	/* bound the un-drained rx backlog */
/* FMT chardev backpressure: a full-ring send re-nudges the CP every poll tick
 * and blocks (per frame) up to the timeout before giving up. */
#define S5300_FMT_TX_POLL_MS		20
#define S5300_FMT_TX_TIMEOUT_MS		2000

/*
 * RFS file channel (EXYNOS_CH_ID_RFS_0, umts_rfs0) transported on the NORM_RAW
 * ring post-ONLINE.  Frames are single-fragment; the biggest the corpus shows
 * is a ~2 KB READ_RESP/WRITE chunk (downstream rfsd reads 0x840 at a time).
 */
#define S5300_RFS_CH			0x29	/* EXYNOS_CH_ID_RFS_0 */
#define S5300_RFS_MAX			SZ_4K

/*
 * rmnet PDP data channels: EXYNOS_CH_EX_ID_PDP_0 (181) through +29 (210) =
 * rmnet0..29.  The CP places small DL packets (e.g. DNS replies) on the legacy
 * NORM_RAW ring on these channels instead of PKTPROC; both feed the data netdev.
 */
#define S5300_PDP_CH_MIN		0xb5	/* EXYNOS_CH_EX_ID_PDP_0 = 181 (rmnet0) */
#define S5300_PDP_CH_MAX		0xd2	/* +29 = 210 (rmnet29) */

/*
 * EXYNOS link header (downstream include/exynos_ipc.h struct
 * exynos_link_header).  12 bytes, single-frame config, boot channel 0xF1.
 */
#define S5300_HDR_SIZE			12
#define S5300_HDR_SYNC			0xabcd	/* EXYNOS_START_MASK */
#define S5300_HDR_CFG_SINGLE		0xc000	/* EXYNOS_SINGLE_MASK << 8 */
#define S5300_BOOT_CH			0xf1	/* EXYNOS_CH_ID_BOOT */

/* Interrupt-word encoding (downstream link_device_memory.h). */
#define S5300_INT_VALID			BIT(7)
#define S5300_CMD_VALID			BIT(6)
#define S5300_CMD_MASK			GENMASK(5, 0)
#define S5300_CMD(x)			(S5300_INT_VALID | S5300_CMD_VALID | (x))
#define S5300_CMD_INIT_START		0x1
#define S5300_CMD_INIT_END		0x2
#define S5300_CMD_CRASH_RESET		0x7
#define S5300_CMD_PHONE_START		0x8
#define S5300_CMD_CRASH_EXIT		0x9
#define S5300_CMD_PIF_INIT_DONE		0xd

/*
 * Non-command interrupt word: a queue-has-data / flow-control mask OR'd with
 * INT_VALID (downstream mask2int()).  Only the FMT bits are used here; RAW
 * data goes through PKTPROC once the data path lands.
 */
#define S5300_MASK_REQ_ACK_FMT		0x0020
#define S5300_MASK_RES_ACK_FMT		0x0008
#define S5300_MASK_SEND_FMT		0x0002
#define S5300_MASK_REQ_ACK_RAW		0x0010
#define S5300_MASK_RES_ACK_RAW		0x0004
#define S5300_MASK_SEND_RAW		0x0001
#define S5300_MASK_SEND_DATA		0x0001	/* pktproc UL; CP scans both rings */
#define S5300_MASK(x)			(S5300_INT_VALID | (x))

/* Doorbell values: bit 16 triggers, low bits select the mailbox index. */
#define S5300_DB_TRIGGER		BIT(16)
#define S5300_DB_MSG			(S5300_DB_TRIGGER | 0x0)
#define S5300_DB_LINK_ACK		(S5300_DB_TRIGGER | 0xe)

/*
 * PKTPROC PS-data path (research/pktproc-plan.md).  The bulk data rings live in
 * a separate carveout (the "pktproc" region, AP-phys 0xe8000000); the CP
 * addresses it at pktproc_cp_base and self-translates to the AP-phys before the
 * TLP, so no AP-side window is needed.  tegu is V2/SKTBUF with hw_iocc and no CP
 * IOMMU, which collapses DL to a fixed reserved-buffer ring: descriptor i always
 * points at buff + max_pkt*i, and RX is a plain copy-out.
 *
 * DL info/desc/buff sub-regions (offsets within the pktproc carveout); the CP
 * sees each at S5300_PKTPROC_CP_BASE + the same offset.
 */
#define S5300_PKTPROC_CP_BASE		0x20000000
#define S5300_PKTPROC_DL_INFO_OFF	0x0
#define S5300_PKTPROC_DL_DESC_OFF	0x1000
#define S5300_PKTPROC_DL_BUFF_OFF	0x100000
/* DL buffers fill the span from their start to the UL sub-region; deriving it
 * (rather than a second magic size) keeps num_desc tied to the map, so the ring
 * always covers every buffer slot the CP is handed.
 */
#define S5300_PKTPROC_DL_BUFF_SIZE	(S5300_PKTPROC_UL_INFO_OFF - S5300_PKTPROC_DL_BUFF_OFF)
#define S5300_PKTPROC_MAX_PKT		0x630	/* pktproc_dl_max_packet_size */
#define S5300_PKTPROC_DESC_SZ		16	/* sizeof(pktproc_desc_sktbuf) */
#define S5300_PKTPROC_DESC_MODE_SKTBUF	1
#define S5300_PKTPROC_IRQ_EXCLUSIVE	1	/* irq_mode: tegu firmware = exclusive */
/*
 * tegu DL: 4 queues, exclusive irq_mode (DT-confirmed pktproc_dl_num_queue=4 /
 * use_exclusive_irq=1).  The CP arms DL *unconditionally* (no capability bit),
 * so this info block MUST match what the firmware runs -- a 1-queue/shared block
 * degrades the CP's notify engine and stalls the SIT control channel (proven on
 * HW).  We still poll rear_ptr from the shared MSI-0 handler, so the exclusive
 * per-queue vectors going unhandled costs DL latency, not correctness.  num_desc
 * spans the whole per-queue buffer region (see S5300_PKTPROC_DL_NUM_DESC); the
 * CP uses what we advertise.
 */
#define S5300_PKTPROC_DL_NUM_Q		4
#define S5300_PKTPROC_DL_BUFF_BY_Q	(S5300_PKTPROC_DL_BUFF_SIZE / S5300_PKTPROC_DL_NUM_Q)
/*
 * One descriptor per buffer slot, so the ring spans the whole per-queue buffer
 * region and wraps in lockstep with the CP's buffer area -- downstream derives
 * num_desc the same way (buff_size_by_q / packet_size).  A hardcoded 512-entry
 * ring recycled only the first ~810 KB of each 6.9 MB buffer quarter and wedged
 * the CP's DL engine after ~one buffer-region of DMA (~21 MB, hw-observed): the
 * CP kept raising the per-queue MSI with nothing to deliver.  We copy each frame
 * out (netdev_alloc_skb + memcpy_fromio), so the slot stride is just MAX_PKT --
 * downstream's larger true_packet_size only pads for build_skb's shared_info.
 */
#define S5300_PKTPROC_DL_NUM_DESC	(S5300_PKTPROC_DL_BUFF_BY_Q / S5300_PKTPROC_MAX_PKT)
#define S5300_PKTPROC_DL_DESC_BY_Q	(S5300_PKTPROC_DL_NUM_DESC * S5300_PKTPROC_DESC_SZ)

/* pktproc_info_v2: word0 (4B) then q_info[]; UL info_ul is word0+word1 (8B). */
#define S5300_PKTPROC_DL_QINFO(q)	(0x4 + (q) * 0x14)
#define S5300_QINFO_CP_DESC		0x0
#define S5300_QINFO_NUM_DESC		0x4
#define S5300_QINFO_CP_BUFF		0x8
#define S5300_QINFO_FORE		0xc
#define S5300_QINFO_REAR		0x10

/* SKTBUF descriptor: word0 = cp_data_paddr[31:0]; word1 carries control@bit8. */
#define S5300_DESC_ADDR_LO		0x0
#define S5300_DESC_W1			0x4
#define S5300_DESC_LEN			0x8	/* u16 length (CP writes) */
#define S5300_DESC_CHID			0xc	/* word @0xc; chid = (val >> 16) & 0xff */
#define S5300_PKTPROC_CTRL_HEAD		0x80	/* control bit7: first descriptor */
#define S5300_PKTPROC_CTRL_RINGEND	0x08	/* control bit3: last descriptor */

/* Data PDP channels with CH_EXTENSION on: EXYNOS_CH_EX_ID_PDP_0.. = 181.. */
#define S5300_PKTPROC_CH_PDP_FIRST	0xb5	/* 181 */
#define S5300_PKTPROC_CH_PDP_COUNT	30

/*
 * PKTPROC UL (uplink transmit).  Separate sub-regions in the same carveout; two
 * queues (the CP requires it), all TX on NORM.  The CP fills end_bit_owner +
 * cp_quota into the info header once it sees the PKTPROC_UL capability.
 */
#define S5300_PKTPROC_UL_INFO_OFF	0x1c00000
#define S5300_PKTPROC_UL_DESC_OFF	0x1c01000
#define S5300_PKTPROC_UL_BUFF_OFF	0x1c90000
#define S5300_PKTPROC_UL_BUFF_SIZE	0x370000
#define S5300_PKTPROC_UL_NUM_Q		2
#define S5300_PKTPROC_UL_DESC_SZ	32	/* sizeof(pktproc_desc_ul) */
#define S5300_PKTPROC_UL_CP_PADDING	76	/* padding_required=1: +CP_PADDING */
#define S5300_PKTPROC_UL_BUFF_BY_Q	(S5300_PKTPROC_UL_BUFF_SIZE / S5300_PKTPROC_UL_NUM_Q)
#define S5300_PKTPROC_UL_QINFO(i)	(0x8 + (i) * 0x14)	/* q_info[i] in info_ul */
/* End of the whole DL+UL map; the reserved-memory carveout must be >= this. */
#define S5300_PKTPROC_MAP_END		(S5300_PKTPROC_UL_BUFF_OFF + S5300_PKTPROC_UL_BUFF_SIZE)
#define S5300_UL_END_BIT_AP		0	/* AP sets the batch end bit */
/*
 * Per-queue geometry (hiprio_ack_only=1, DT-confirmed): queue 0 = HIPRIO with
 * roundup_pow_of_two(MAX_UL_PACKET_SIZE 512 + CP_PADDING) = 0x400-byte packets;
 * queue 1 = NORM with the default 0x800.  num_desc = buff_by_q / max_pkt, and
 * the descriptor rings are laid out contiguously by *actual* size (HIPRIO
 * first), matching downstream pktproc_create_ul(); buffers split evenly.  We
 * transmit only on NORM, but both queues must be provisioned to the geometry
 * the CP expects or its HIPRIO ring can overlap ours.
 */
#define S5300_PKTPROC_UL_HI_MAX_PKT	0x400
#define S5300_PKTPROC_UL_HI_NUM_DESC	(S5300_PKTPROC_UL_BUFF_BY_Q / S5300_PKTPROC_UL_HI_MAX_PKT)
#define S5300_PKTPROC_UL_HI_DESC_SZ	(S5300_PKTPROC_UL_HI_NUM_DESC * S5300_PKTPROC_UL_DESC_SZ)
#define S5300_PKTPROC_UL_TXQ		1	/* NORM queue carries all TX */
#define S5300_PKTPROC_UL_MAX_PKT	0x800	/* NORM default_max_packet_size */
#define S5300_PKTPROC_UL_NUM_DESC	(S5300_PKTPROC_UL_BUFF_BY_Q / S5300_PKTPROC_UL_MAX_PKT)
#define S5300_PKTPROC_UL_DESC_BASE	(S5300_PKTPROC_UL_DESC_OFF + S5300_PKTPROC_UL_HI_DESC_SZ)

/* PBL lands at IPC base + this offset (round_up(raw buffer offset, 64K)). */
#define S5300_BOOT_IMG_OFFSET		0x10000

/* boot_stage / CP2AP_WAKEUP polling, downstream check_cp_status(). */
#define S5300_POLL_INTERVAL_MS		20
#define S5300_POLL_COUNT		200

/* PHONE_START handshake timeout, downstream MIF_INIT_TIMEOUT. */
#define S5300_INIT_TIMEOUT		(15 * HZ)

/* enum modem_state (downstream modem_prj.h); userspace polls for ONLINE. */
#define S5300_STATE_OFFLINE		0
#define S5300_STATE_BOOTING		3
#define S5300_STATE_ONLINE		4

/*
 * Largest single std_dl frame userspace writes is one 0xC000 data chunk plus
 * its 12-byte std_dl header; cap generously and keep a resident TX staging
 * buffer (writes are serialised by io_lock).
 */
#define S5300_TX_MAX			0xf000
#define S5300_TX_BUF_SIZE		(S5300_HDR_SIZE + S5300_TX_MAX + 8)
#define S5300_RX_FIFO_SIZE		4096

/* First-stage bootloader (BOOT TOC entry); <= 0x16800 in practice. */
#define S5300_PBL_MAX			0x20000

/* Boot chardev ABI (magic 'o'), downstream-compatible for same-binary A/B. */
struct s5300_cp_image {
	__u64	binary;
	__u32	size;
	__u32	m_offset;
	__u32	b_offset;
	__u32	mode;
	__u32	len;
} __packed;

struct s5300_boot_mode {
	int	idx;
};

#define IOCTL_POWER_ON			_IO('o', 0x19)
#define IOCTL_START_CP_BOOTLOADER	_IOW('o', 0x22, struct s5300_boot_mode)
#define IOCTL_COMPLETE_NORMAL_BOOTUP	_IO('o', 0x23)
#define IOCTL_GET_CP_STATUS		_IO('o', 0x27)
#define IOCTL_LOAD_CP_IMAGE		_IOW('o', 0x40, struct s5300_cp_image)
#define IOCTL_HANDOVER_BLOCK_INFO	_IO('o', 0x57)

struct s5300_chardev {
	struct s5300_modem	*sm;
	struct miscdevice	miscdev;
	u8			channel;	/* EXYNOS channel id */
	u8			ch_seq;		/* per-channel link-header seq */
	bool			raw_ring;	/* true: NORM_RAW ring, false: FMT */
	u32			tx_max;		/* max app payload per frame */
	u8			*tx_buf;	/* header + one frame + pad */
	struct sk_buff_head	rxq;		/* one skb per received frame */
	wait_queue_head_t	read_wq;
};

struct s5300_modem {
	struct device		*dev;
	struct device		*rc_dev;
	struct pci_dev		*pdev;
	struct gpio_desc	*cp2ap_wakeup;

	phys_addr_t		ipc_phys;
	resource_size_t		ipc_size;
	void __iomem		*ipc;
	phys_addr_t		msi_phys;
	void __iomem		*msi;

	u32			db_bus_addr;
	void __iomem		*doorbell;

	struct miscdevice	miscdev;

	/* Runtime SIT control channel on the legacy FMT queue (post-ONLINE). */
	struct wwan_port	*ctrl_port;
	u16			fmt_frame_seq;
	u8			fmt_ch_seq;
	u8			*fmt_tx_buf;	/* header + one SIT app msg + pad */

	/*
	 * OEM/GEMS channel (ch 0x82) on the FMT ring, exposed as /dev/umts_oem1.
	 * fmt_tx_lock serialises the FMT txq's two post-ONLINE writers (SIT +
	 * oem: their WWAN ops_lock / chardev writes are independent) and the
	 * shared fmt_frame_seq; fmt_tx_wq wakes a blocked oem writer when the CP
	 * drains the txq and frees ring space.
	 */
	struct s5300_chardev	oem;
	struct mutex		fmt_tx_lock;
	wait_queue_head_t	fmt_tx_wq;

	/*
	 * RFS file channel (ch 0x29 on the NORM_RAW ring), exposed as
	 * /dev/umts_rfs0 for the userspace server that answers the CP's NV and
	 * carrier-config file requests (post-ONLINE).
	 */
	struct s5300_chardev	rfs;

	/*
	 * Serialises the post-ONLINE writers of the shared NORM_RAW txq: the RFS
	 * chardev write path and the AT WWAN port have independent locks, so their
	 * head RMW and the frame_seq counter would otherwise race.  The boot
	 * std_dl writer uses the same ring but is temporally disjoint (io_lock).
	 */
	struct mutex		raw_tx_lock;

	/* EXYNOS link-header sequence counters (reset per boot). */
	u16			frame_seq;
	u8			ch_seq;
	u8			*tx_buf;	/* header + one std_dl frame + pad */

	/* CP->AP std_dl acks, filled by the IRQ, drained by read(). */
	struct kfifo		rx_fifo;
	spinlock_t		rx_lock;	/* protects rx_fifo */
	wait_queue_head_t	read_wq;

	u32			pbl_size;	/* staged by LOAD, published by START */
	struct completion	init_done;
	struct mutex		io_lock;	/* serialises ioctl/write sequencing */
	spinlock_t		lock;		/* orders ap2cp_msg word + doorbell */
	int			cp_status;	/* enum modem_state */
	bool			online;

	/*
	 * PKTPROC PS-data path (separate cp_rmem_1 carveout).  DL is a fixed
	 * reserved-buffer ring drained from the MSI handler; fore is the AP's
	 * producer of empty buffers (mirrored into the info region for the CP),
	 * done its private consumer, rear the CP's producer (read from info).
	 */
	phys_addr_t		pktproc_phys;
	resource_size_t		pktproc_size;
	void __iomem		*pktproc;
	struct net_device	*ndev;		/* raw-IP data netdev (ch 181..) */
	u32			dl_num_desc;
	u32			dl_fore[S5300_PKTPROC_DL_NUM_Q];
	u32			dl_done[S5300_PKTPROC_DL_NUM_Q];
	/* UL TX ring (NORM queue).  ul_done is touched only under the tx lock. */
	u32			ul_num_desc;
	u32			ul_done;
	u16			ul_cp_quota;
	u8			ul_end_bit_owner;
	bool			ul_active;
	bool			tx_suspended;	/* CP TX flow control (vector-1 cp2ap_status) */
};

/* --- circular-ring helpers (downstream include/circ_queue.h) ------------- */

static inline u32 s5300_circ_space(u32 qsize, u32 in, u32 out)
{
	return (in < out) ? (out - in - 1) : (qsize + out - in - 1);
}

static inline u32 s5300_circ_usage(u32 qsize, u32 in, u32 out)
{
	return (in >= out) ? (in - out) : (qsize - out + in);
}

static inline u32 s5300_circ_new(u32 qsize, u32 p, u32 len)
{
	u32 np = p + len;

	while (np >= qsize)
		np -= qsize;
	return np;
}

static void s5300_circ_write(void __iomem *buff, const u8 *src, u32 qsize,
			     u32 in, u32 len)
{
	if (in + len < qsize) {
		memcpy_toio(buff + in, src, len);
	} else {
		u32 first = qsize - in;

		memcpy_toio(buff + in, src, first);
		memcpy_toio(buff, src + first, len - first);
	}
}

static void s5300_circ_read(u8 *dst, void __iomem *buff, u32 qsize, u32 out,
			    u32 len)
{
	if (out + len <= qsize) {
		memcpy_fromio(dst, buff + out, len);
	} else {
		u32 first = qsize - out;

		memcpy_fromio(dst, buff + out, first);
		memcpy_fromio(dst + first, buff, len - first);
	}
}

/* --- doorbell / MSI plumbing --------------------------------------------- */

/*
 * Force BAR0 to decode the doorbell page.  The write is deliberately not
 * 1M-aligned: whatever BAR0 size the current boot stage exposes, the hardware
 * aligns the value down, and the doorbell always decodes at the DT bus address
 * (downstream programs the same value through both boot phases).  Programmed
 * behind the PCI core's back -- the core saw unassignable ROM BARs anyway.
 * The read-back loop is belt-and-braces: once the identically-IDed root port
 * stops squatting the window (s5300_open_bridge_window) the BAR sticks on the
 * first try.
 */
static int s5300_program_doorbell_bar(struct s5300_modem *sm)
{
	u32 val = 0, base;
	int try;

	for (try = 0; try < 10; try++) {
		pci_write_config_dword(sm->pdev, PCI_BASE_ADDRESS_0,
				       sm->db_bus_addr);
		pci_write_config_dword(sm->pdev, PCI_BASE_ADDRESS_1, 0);
		pci_read_config_dword(sm->pdev, PCI_BASE_ADDRESS_0, &val);
		base = val & PCI_BASE_ADDRESS_MEM_MASK;
		if (base && base <= sm->db_bus_addr &&
		    sm->db_bus_addr - base < SZ_1M) {
			if (try)
				dev_warn(sm->dev,
					 "doorbell BAR stuck after %d retries (%#x)\n",
					 try, val);
			return 0;
		}
		udelay(100);
	}

	dev_err(sm->dev, "doorbell BAR won't hold %#x (reads %#x)\n",
		sm->db_bus_addr, val);
	return -EIO;
}

/*
 * Memory TLPs are address-routed: the root port only forwards them downstream
 * inside its type-1 memory window, and because the doorbell BAR is programmed
 * behind the PCI core's back (no child resource was ever assigned) the core
 * leaves that window closed -- every doorbell access dies at the root port
 * with the EP config-reachable but memory-dead.  Open it explicitly over the
 * doorbell's 1M-aligned range, and evict the root port's own BAR0 from that
 * range (TLPs matching an RP BAR are consumed by the port, not forwarded).
 */
static int s5300_open_bridge_window(struct s5300_modem *sm)
{
	struct pci_dev *bridge = pci_upstream_bridge(sm->pdev);
	u32 base = sm->db_bus_addr & ~(SZ_1M - 1);
	u32 limit = base + SZ_1M - 1;
	u32 want = (((limit >> 16) & 0xfff0) << 16) | ((base >> 16) & 0xfff0);
	u32 val;
	u16 cmd;

	if (!bridge)
		return -ENODEV;

	pci_read_config_dword(bridge, PCI_MEMORY_BASE, &val);
	if (val != want) {
		dev_info(sm->dev,
			 "opening root-port memory window %#x-%#x (was %#010x)\n",
			 base, limit, val);
		pci_write_config_dword(bridge, PCI_MEMORY_BASE, want);
		pci_read_config_dword(bridge, PCI_MEMORY_BASE, &val);
		if (val != want) {
			dev_err(sm->dev,
				"root-port window won't hold (%#010x)\n", val);
			return -EIO;
		}
	}

	pci_read_config_dword(bridge, PCI_BASE_ADDRESS_0, &val);
	if ((val & PCI_BASE_ADDRESS_MEM_MASK) >= base &&
	    (val & PCI_BASE_ADDRESS_MEM_MASK) <= limit) {
		dev_info(sm->dev,
			 "evicting root-port BAR0 (%#010x) from the doorbell window\n",
			 val);
		pci_write_config_dword(bridge, PCI_BASE_ADDRESS_0, 0);
		pci_write_config_dword(bridge, PCI_BASE_ADDRESS_1, 0);
	}

	pci_read_config_word(bridge, PCI_COMMAND, &cmd);
	if ((cmd & (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER)) !=
	    (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER))
		pci_write_config_word(bridge, PCI_COMMAND, cmd |
				      PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);

	return 0;
}

/*
 * Ring the doorbell with the downstream retry shape: right after a (re)train
 * the EP's memory decode may not be settled yet, so an all-ones read-back gets
 * the command register and doorbell BAR repaired and the write retried
 * (downstream s51xx_pcie_send_doorbell_int() retries at 1 ms up to 100x; keep
 * it short here because the IPC path can ring from hard-IRQ context).
 */
static void s5300_send_doorbell(struct s5300_modem *sm, u32 val)
{
	int try;
	u16 cmd;

	for (try = 0; try < 10; try++) {
		writel(val, sm->doorbell);
		if (readl(sm->doorbell) != 0xffffffff)
			return;

		pci_read_config_word(sm->pdev, PCI_COMMAND, &cmd);
		if (cmd != 0xffff &&
		    (cmd & (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER)) !=
		    (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER))
			pci_write_config_word(sm->pdev, PCI_COMMAND, cmd |
					      PCI_COMMAND_MEMORY |
					      PCI_COMMAND_MASTER);
		s5300_program_doorbell_bar(sm);
		s5300_open_bridge_window(sm);
		udelay(100);
	}

	dev_err(sm->dev, "doorbell %#x kept reading back all-ones\n", val);
}

/*
 * The ROM derives its boot-status DMA target from the MSI message address
 * registers; downstream re-drives them whenever they read back zero
 * (print_msi_register(): "MSI Message Reg == 0x0 - set MSI again!!!").
 */
static void s5300_verify_msi_target(struct s5300_modem *sm)
{
	u32 lo = 0;
	int try;

	for (try = 0; try < 5; try++) {
		pci_read_config_dword(sm->pdev,
				      sm->pdev->msi_cap + PCI_MSI_ADDRESS_LO,
				      &lo);
		if (lo == lower_32_bits(sm->msi_phys))
			return;
		dev_warn(sm->dev, "MSI address reads %#x, re-driving\n", lo);
		pci_restore_msi_state(sm->pdev);
	}

	dev_err(sm->dev, "MSI address won't hold %pap\n", &sm->msi_phys);
}

/* Downstream pcie_send_ap2cp_irq(): interrupt word, then the doorbell. */
static void s5300_send_ipc_irq(struct s5300_modem *sm, u32 val)
{
	unsigned long flags;

	spin_lock_irqsave(&sm->lock, flags);
	writel(val, sm->ipc + S5300_IPC_AP2CP_MSG);
	s5300_send_doorbell(sm, S5300_DB_MSG);
	spin_unlock_irqrestore(&sm->lock, flags);
}

/*
 * Downstream init_control_messages(): publish the srinfo and capability
 * offsets and zero the status/capability words before the CP boots.  The AP
 * capability words stay zero (tegu DT: ap_capability_0/1 = 0).
 */
static void s5300_init_control_messages(struct s5300_modem *sm)
{
	int i;

	writel(S5300_IPC_SRINFO_OFFSET, sm->ipc + S5300_IPC_SRINFO_OFS_PTR);
	writel(S5300_IPC_CAP_BASE, sm->ipc + S5300_IPC_CAP_OFS_PTR);
	writel(0, sm->ipc + S5300_IPC_AP2CP_MSG);
	writel(0, sm->ipc + S5300_IPC_CP2AP_MSG);
	/*
	 * ap2cp_united_status ds_det field (bits 14-15) = 1 (downstream
	 * get_ds_detect(); the live working device reads 0x4000 here).  This is
	 * load-bearing: with ds_det=0 the CP never runs its deep-sleep link
	 * handshake, so after MAIN loads on the already-up link it never takes a
	 * link-up ISR to arm its runtime IPC and the FMT control queue is never
	 * drained ("no data in UL buffer").  With ds_det=1 the CP arms its
	 * runtime IPC and the SIT control channel round-trips.  HW-validated.
	 */
	writel(S5300_IPC_DS_DET, sm->ipc + S5300_IPC_AP2CP_STATUS);
	writel(0, sm->ipc + S5300_IPC_CP2AP_STATUS);
	for (i = 0; i < S5300_IPC_CAP_WORDS; i++)
		writel(0, sm->ipc + S5300_IPC_CAP_BASE + 4 * i);
}

/*
 * Downstream set_ap_capabilities(): publish the AP capability words before
 * answering INIT_START with PIF_INIT_DONE, so the CP can read them as it comes
 * up.  The CP will not enable its runtime IPC until this handshake completes.
 */
static void s5300_set_ap_capabilities(struct s5300_modem *sm)
{
	writel(S5300_AP_CAPABILITY_0, sm->ipc + S5300_IPC_CAP_AP0);
	writel(0, sm->ipc + S5300_IPC_CAP_AP1);
	dev_info(sm->dev, "AP capability part0 %#x\n", S5300_AP_CAPABILITY_0);
}

/*
 * Downstream cmd_phone_start_handler() capability check: read what the CP
 * advertised (it fills these once it has read ours) and warn if the CP is
 * missing a bit the AP claims -- downstream crashes the CP on that; we only log
 * (no crash recovery yet).  init_ap_capabilities() itself is a no-op here (it
 * only sets up PKTPROC_UL, deferred to the data path).
 */
static void s5300_check_cp_capabilities(struct s5300_modem *sm)
{
	u32 ap0 = S5300_AP_CAPABILITY_0;
	u32 cp0 = readl(sm->ipc + S5300_IPC_CAP_CP0);
	u32 cp1 = readl(sm->ipc + S5300_IPC_CAP_CP1);

	dev_info(sm->dev, "capability AP %#x CP %#x/%#x\n", ap0, cp0, cp1);
	if ((ap0 ^ cp0) & ap0)
		dev_warn(sm->dev, "CP lacks AP capability bits %#x\n",
			 (ap0 ^ cp0) & ap0);
}

/*
 * Downstream link_start_normal_boot() -> init_legacy_link(): clear the queue
 * pointers, enable memory access, then stamp the boot magic.  Run before the
 * std_dl stream so the CP's download server sees empty rings.
 */
static void s5300_init_boot_ring(struct s5300_modem *sm)
{
	int i;

	sm->frame_seq = 0;
	sm->ch_seq = 0;

	writel(0, sm->ipc + S5300_IPC_MAGIC);
	writel(0, sm->ipc + S5300_IPC_ACCESS);
	for (i = 0; i < S5300_IPC_Q_WORDS; i++)
		writel(0, sm->ipc + S5300_IPC_Q_HEAD_TAIL + 4 * i);
	writel(1, sm->ipc + S5300_IPC_ACCESS);
	writel(S5300_IPC_MAGIC_BOOT, sm->ipc + S5300_IPC_MAGIC);
}

/*
 * Downstream init_legacy_link() from the PHONE_START handler: swap the boot
 * magic for the running magic once the CP asks to start IPC.
 */
static void s5300_init_ipc_queues(struct s5300_modem *sm)
{
	u32 magic, access;
	int i;

	sm->fmt_frame_seq = 0;
	sm->fmt_ch_seq = 0;
	sm->rfs.ch_seq = 0;

	writel(0, sm->ipc + S5300_IPC_MAGIC);
	writel(0, sm->ipc + S5300_IPC_ACCESS);
	for (i = 0; i < S5300_IPC_Q_WORDS; i++)
		writel(0, sm->ipc + S5300_IPC_Q_HEAD_TAIL + 4 * i);
	writel(S5300_IPC_MAGIC_ONLINE, sm->ipc + S5300_IPC_MAGIC);
	writel(1, sm->ipc + S5300_IPC_ACCESS);

	magic = readl(sm->ipc + S5300_IPC_MAGIC);
	access = readl(sm->ipc + S5300_IPC_ACCESS);
	if (magic != S5300_IPC_MAGIC_ONLINE || access != 1)
		dev_err(sm->dev, "IPC init readback failed: magic %#x access %u\n",
			magic, access);
}

/* Defined below; called from the RX drain and the chardev write path here. */
static void s5300_chardev_rx(struct s5300_chardev *cd, void __iomem *buff,
			     u32 ringsize, u32 out, u32 payload);
static int s5300_raw_ring_tx(struct s5300_modem *sm, u8 *staging, u8 ch,
			     u8 *ch_seq, u32 max, const u8 *data, u32 len);

/*
 * Drain the NORM_RAW rxq and strip the 12-byte link header.  Three channels
 * ride this ring: boot std_dl acks (handed to read() on /dev/umts_boot0) and,
 * once ONLINE, the RFS file channel (ch 0x29, to /dev/umts_rfs0) and the vendor
 * AT/router channel (ch 0x15, to the AT port).  Boot is temporally disjoint from
 * the runtime channels, so only BOOT-channel frames are handed to
 * /dev/umts_boot0; unknown runtime RAW channels are logged and dropped.  Runs
 * from the MSI handler; the CP drains by head/tail, so we send no RES_ACK (see
 * the FMT drain -- an ack on the shared ap2cp_msg word clobbers a pending SEND_*).
 */
static void s5300_drain_rxq(struct s5300_modem *sm)
{
	void __iomem *buff = sm->ipc + S5300_RAW_RXQ_OFFSET;
	bool woke = false;
	u32 in, out;

	in = readl(sm->ipc + S5300_RAW_RXQ_HEAD);
	out = readl(sm->ipc + S5300_RAW_RXQ_TAIL);

	if (in >= S5300_RAW_RXQ_SIZE || out >= S5300_RAW_RXQ_SIZE) {
		dev_err(sm->dev, "rxq pointers out of range (in %#x out %#x)\n",
			in, out);
		return;
	}

	while (in != out) {
		u8 frame[S5300_HDR_SIZE + 64];
		u32 rest = s5300_circ_usage(S5300_RAW_RXQ_SIZE, in, out);
		u32 flen, total, payload, body;
		u8 hdr[S5300_HDR_SIZE];

		if (rest < S5300_HDR_SIZE)
			break;

		s5300_circ_read(hdr, buff, S5300_RAW_RXQ_SIZE, out,
				S5300_HDR_SIZE);
		if (hdr[0] != (S5300_HDR_SYNC & 0xff) ||
		    hdr[1] != (S5300_HDR_SYNC >> 8)) {
			dev_err(sm->dev, "rxq bad sync %#04x, flushing\n",
				hdr[0] | (hdr[1] << 8));
			out = in;
			break;
		}

		flen = hdr[6] | (hdr[7] << 8);		/* header + payload */
		total = round_up(flen, 8);		/* CP writes 8-aligned */
		if (flen < S5300_HDR_SIZE || total > rest) {
			dev_err(sm->dev, "rxq bad len %u (rest %u)\n", flen,
				rest);
			out = in;
			break;
		}
		payload = flen - S5300_HDR_SIZE;
		body = s5300_circ_new(S5300_RAW_RXQ_SIZE, out, S5300_HDR_SIZE);

		if (hdr[8] == S5300_RFS_CH) {
			/* Single-frame RE'd protocol; bound a corrupt CP length
			 * (the chardev enqueues + wakes a reader). */
			if (payload && payload <= S5300_RFS_MAX)
				s5300_chardev_rx(&sm->rfs, buff,
						 S5300_RAW_RXQ_SIZE, out, payload);
		} else if (READ_ONCE(sm->online) && sm->ndev && payload &&
			   hdr[8] >= S5300_PDP_CH_MIN && hdr[8] <= S5300_PDP_CH_MAX) {
			/*
			 * Raw-IP DL data the CP places on the legacy NORM_RAW ring
			 * instead of PKTPROC for small packets (DNS replies); cpif's
			 * rx_multi_pdp does the same on these PDP channels.  Feed the
			 * data netdev like s5300_pktproc_dl_drain().  Like every
			 * other channel we send the CP NOTHING on RX: downstream
			 * acks nothing either, and s5300_send_ipc_irq() overwrites the
			 * shared ap2cp_msg word, so a spurious ack would clobber a
			 * pending SEND_* notification and desync the CP.
			 */
			struct sk_buff *skb = netdev_alloc_skb(sm->ndev, payload);
			u8 ver = 0;

			if (skb) {
				s5300_circ_read(skb_put(skb, payload), buff,
						S5300_RAW_RXQ_SIZE, body, payload);
				ver = skb->data[0] >> 4;
			}
			if (skb && (ver == 4 || ver == 6)) {
				skb->protocol = htons(ver == 6 ? ETH_P_IPV6
							       : ETH_P_IP);
				skb->dev = sm->ndev;
				skb_reset_mac_header(skb);
				skb_reset_network_header(skb);
				sm->ndev->stats.rx_packets++;
				sm->ndev->stats.rx_bytes += payload;
				netif_rx(skb);
				dev_info_once(sm->dev, "raw-ring PDP data (ch %#x) -> %s\n",
					      hdr[8], netdev_name(sm->ndev));
			} else if (skb) {
				dev_kfree_skb_any(skb);
				sm->ndev->stats.rx_length_errors++;
			} else {
				sm->ndev->stats.rx_dropped++;
			}
		} else if (!READ_ONCE(sm->online) && hdr[8] == S5300_BOOT_CH &&
			   payload && payload <= sizeof(frame) - S5300_HDR_SIZE) {
			s5300_circ_read(frame, buff, S5300_RAW_RXQ_SIZE, out, flen);
			kfifo_in_spinlocked(&sm->rx_fifo, frame + S5300_HDR_SIZE,
					    payload, &sm->rx_lock);
			woke = true;
		} else if (payload) {
			u8 first = 0;

			s5300_circ_read(&first, buff, S5300_RAW_RXQ_SIZE, body, 1);
			dev_info_ratelimited(sm->dev,
					     "raw rxq drop unhandled ch %#x payload %u first %#x\n",
					     hdr[8], payload, first);
		}

		out = s5300_circ_new(S5300_RAW_RXQ_SIZE, out, total);
	}

	writel(out, sm->ipc + S5300_RAW_RXQ_TAIL);

	if (woke)
		wake_up_interruptible(&sm->read_wq);
}

/*
 * Copy one channel frame's payload out of a ring into an skb and queue it for
 * the chardev's reader (runs in the drain, hard IRQ: GFP_ATOMIC).  @buff/@out
 * point at the frame's link header; the payload starts one header in.  The
 * backlog is bounded so a daemon that never opens the node cannot exhaust
 * memory -- excess frames are dropped (a draining daemon never hits this).
 */
static void s5300_chardev_rx(struct s5300_chardev *cd, void __iomem *buff,
			     u32 ringsize, u32 out, u32 payload)
{
	struct sk_buff *skb;

	if (skb_queue_len(&cd->rxq) >= S5300_OEM_RXQ_MAX) {
		dev_warn_ratelimited(cd->sm->dev, "%s rxq full, dropping %u\n",
				     cd->miscdev.name, payload);
		return;
	}
	skb = alloc_skb(payload, GFP_ATOMIC);
	if (!skb) {
		dev_warn_ratelimited(cd->sm->dev,
				     "%s alloc_skb(%u) failed, dropping\n",
				     cd->miscdev.name, payload);
		return;
	}
	s5300_circ_read(skb_put(skb, payload), buff, ringsize,
			s5300_circ_new(ringsize, out, S5300_HDR_SIZE), payload);
	skb_queue_tail(&cd->rxq, skb);
	wake_up_interruptible(&cd->read_wq);
}

/*
 * Drain formatted IPC frames the CP left on the FMT rxq.  Downstream routes
 * several IPC_FMT channels through this ring (umts_ipc0, oem_ipc*, wfc0, ...);
 * ch 0xf5 is the SIT control port (WWAN_PORT_SIT) and ch 0x82 is oem_ipc1
 * (/dev/umts_oem1).  Consume every frame so the ring cannot stall, delivering
 * 0xf5 to the SIT port and 0x82 to the oem chardev and dropping the rest.  We
 * send no RES_ACK (see the tail comment below).
 */
static void s5300_drain_fmt_rxq(struct s5300_modem *sm)
{
	void __iomem *buff = sm->ipc + S5300_FMT_RXQ_OFFSET;
	u32 in, out;

	in = readl(sm->ipc + S5300_FMT_RXQ_HEAD);
	out = readl(sm->ipc + S5300_FMT_RXQ_TAIL);

	if (in >= S5300_FMT_RXQ_SIZE || out >= S5300_FMT_RXQ_SIZE) {
		dev_err(sm->dev, "fmt rxq pointers out of range (in %#x out %#x)\n",
			in, out);
		return;
	}

	while (in != out) {
		u32 rest = s5300_circ_usage(S5300_FMT_RXQ_SIZE, in, out);
		u32 flen, total, payload;
		u8 hdr[S5300_HDR_SIZE];
		struct sk_buff *skb;

		if (rest < S5300_HDR_SIZE)
			break;

		s5300_circ_read(hdr, buff, S5300_FMT_RXQ_SIZE, out, S5300_HDR_SIZE);
		if (hdr[0] != (S5300_HDR_SYNC & 0xff) ||
		    hdr[1] != (S5300_HDR_SYNC >> 8)) {
			dev_err(sm->dev, "fmt rxq bad sync %#04x, flushing\n",
				hdr[0] | (hdr[1] << 8));
			out = in;
			break;
		}

		flen = hdr[6] | (hdr[7] << 8);
		total = round_up(flen, 8);
		if (flen < S5300_HDR_SIZE || total > rest) {
			dev_err(sm->dev, "fmt rxq bad len %u (rest %u)\n", flen,
				rest);
			out = in;
			break;
		}
		payload = flen - S5300_HDR_SIZE;

		if (hdr[8] == S5300_OEM_CH) {
			if (payload)
				s5300_chardev_rx(&sm->oem, buff,
						 S5300_FMT_RXQ_SIZE, out, payload);
		} else if (hdr[8] != S5300_FMT_CH) {
			u8 first = 0;

			if (payload)
				s5300_circ_read(&first, buff, S5300_FMT_RXQ_SIZE,
						s5300_circ_new(S5300_FMT_RXQ_SIZE,
							       out, S5300_HDR_SIZE),
						1);
			dev_info_ratelimited(sm->dev,
					     "fmt rxq drop unhandled ch %#x payload %u first %#x\n",
					     hdr[8], payload, first);
		} else if (payload) {
			skb = alloc_skb(payload, GFP_ATOMIC);
			if (!skb) {
				dev_warn(sm->dev,
					 "fmt rxq alloc_skb(%u) failed, dropping\n",
					 payload);
			} else {
				s5300_circ_read(skb_put(skb, payload), buff,
						S5300_FMT_RXQ_SIZE,
						s5300_circ_new(S5300_FMT_RXQ_SIZE,
							       out, S5300_HDR_SIZE),
						payload);
				wwan_port_rx(sm->ctrl_port, skb);
			}
		}

		out = s5300_circ_new(S5300_FMT_RXQ_SIZE, out, total);
	}

	writel(out, sm->ipc + S5300_FMT_RXQ_TAIL);

	/*
	 * Do not answer REQ_ACK with RES_ACK.  Downstream never sends RES_ACK
	 * (the CP drains the ring by head/tail regardless), and writing
	 * RES_ACK_FMT to the shared ap2cp_msg word clobbers a still-unread
	 * SEND_FMT staged for our reply frame -- which wedges a full FMT ring,
	 * since once it is full every send short-circuits before re-notifying
	 * and the CP is never told to drain (hw-observed oem-reply stall).
	 */
}

/* PKTPROC ring helpers, defined with the data-netdev code further down. */
static void s5300_pktproc_dl_init(struct s5300_modem *sm);
static void s5300_pktproc_dl_drain(struct s5300_modem *sm);
static void s5300_pktproc_ul_setup(struct s5300_modem *sm);
static void s5300_pktproc_ul_activate(struct s5300_modem *sm);

/*
 * MSI vector 1: the CP's cp2ap_status interrupt (downstream
 * shmem_tx_state_handler).  Its one runtime payload is the global TX
 * flow-control bit in cp2ap_united_status: the CP asks the AP to stop feeding
 * it uplink while its ingress quiesces, and to resume afterwards.  Downstream
 * reacts by stopping the data netdevs; the legacy FMT/RAW rings stay open
 * (they are CP-paced -- the CP polls them at its leisure), so only the PKTPROC
 * UL path is gated here.
 */
static irqreturn_t s5300_tx_state_irq(int irq, void *data)
{
	struct s5300_modem *sm = data;
	u32 status = readl(sm->ipc + S5300_IPC_CP2AP_STATUS);
	bool suspend = status & S5300_CP2AP_FLOWCTL;

	if (suspend == READ_ONCE(sm->tx_suspended))
		return IRQ_HANDLED;

	WRITE_ONCE(sm->tx_suspended, suspend);
	if (sm->ndev) {
		if (suspend)
			netif_stop_queue(sm->ndev);
		else
			netif_wake_queue(sm->ndev);
	}
	dev_info_ratelimited(sm->dev, "CP TX flow control: %s (cp2ap status %#x)\n",
			     suspend ? "suspend" : "resume", status);
	return IRQ_HANDLED;
}

static irqreturn_t s5300_irq_handler(int irq, void *data)
{
	struct s5300_modem *sm = data;
	u32 val, cmd;

	/* std_dl acks (and any CP->AP data) ride the NORM_RAW rxq. */
	s5300_drain_rxq(sm);

	val = readl(sm->ipc + S5300_IPC_CP2AP_MSG);

	/* Once ONLINE the SIT control channel delivers on the FMT rxq, and the
	 * CP DMAs PS data into the PKTPROC DL ring (drained to the netdev).
	 */
	if (READ_ONCE(sm->online)) {
		s5300_drain_fmt_rxq(sm);
		s5300_pktproc_dl_drain(sm);
		/* The CP advancing the FMT txq tail frees space for oem writers. */
		wake_up_interruptible(&sm->fmt_tx_wq);
	}

	if (!(val & S5300_INT_VALID))
		return IRQ_HANDLED;

	if (!(val & S5300_CMD_VALID)) {
		/* Plain data notification; the rxq drain above handled it. */
		dev_dbg(sm->dev, "IPC data interrupt %#x\n", val);
		return IRQ_HANDLED;
	}

	cmd = val & S5300_CMD_MASK;
	switch (cmd) {
	case S5300_CMD_INIT_START:
		dev_info(sm->dev, "CP INIT_START\n");
		s5300_pktproc_dl_init(sm);
		s5300_pktproc_ul_setup(sm);
		s5300_set_ap_capabilities(sm);
		s5300_send_ipc_irq(sm, S5300_CMD(S5300_CMD_PIF_INIT_DONE));
		break;
	case S5300_CMD_PHONE_START:
		dev_info(sm->dev, "CP PHONE_START\n");
		if (!READ_ONCE(sm->online)) {
			s5300_check_cp_capabilities(sm);
			s5300_pktproc_ul_activate(sm);
			s5300_init_ipc_queues(sm);
			/* A rebooted CP starts un-flow-controlled. */
			if (READ_ONCE(sm->tx_suspended)) {
				WRITE_ONCE(sm->tx_suspended, false);
				netif_wake_queue(sm->ndev);
			}
			/* Publish only after the FMT ring is armed (magic 0xAA). */
			WRITE_ONCE(sm->online, true);
			sm->cp_status = S5300_STATE_ONLINE;
		}
		/* Re-entrant PHONE_START just gets the INIT_END again. */
		s5300_send_ipc_irq(sm, S5300_CMD(S5300_CMD_INIT_END));
		complete_all(&sm->init_done);
		break;
	case S5300_CMD_CRASH_RESET:
	case S5300_CMD_CRASH_EXIT:
		dev_err(sm->dev, "CP crash notification %#x (err_report %#x)\n",
			cmd, readl(sm->msi + S5300_MSI_ERR_REPORT));
		sm->cp_status = S5300_STATE_OFFLINE;
		break;
	default:
		dev_warn(sm->dev, "unknown CP command %#x\n", cmd);
		break;
	}

	return IRQ_HANDLED;
}

static int s5300_setup_doorbell(struct s5300_modem *sm)
{
	struct pci_bus_region region;
	/*
	 * pcibios_bus_to_resource() matches host-bridge windows by the resource
	 * type of the passed-in res, so it must be pre-typed MEM -- zero flags
	 * match no window and the bus address comes back untranslated.
	 */
	struct resource res = { .flags = IORESOURCE_MEM };
	int i, ret;

	/*
	 * The PCI core could not place the mask ROM's six 1M BARs in the small
	 * CH0 window; drop them from resource management entirely so
	 * pci_enable_device() has nothing unclaimed to trip over, then program
	 * BAR0 directly (downstream s51xx_pcie_probe() does the same).
	 */
	for (i = 0; i < PCI_STD_NUM_BARS; i++) {
		sm->pdev->resource[i].start = 0;
		sm->pdev->resource[i].end = 0;
		sm->pdev->resource[i].flags = 0;
	}
	ret = s5300_program_doorbell_bar(sm);
	if (ret)
		return ret;

	ret = s5300_open_bridge_window(sm);
	if (ret)
		return ret;

	region.start = sm->db_bus_addr;
	region.end = sm->db_bus_addr + SZ_4K - 1;
	pcibios_bus_to_resource(sm->pdev->bus, &res, &region);
	if (!res.start)
		return dev_err_probe(sm->dev, -EINVAL,
				     "doorbell bus address %#x maps to no CPU window\n",
				     sm->db_bus_addr);

	sm->doorbell = devm_ioremap(sm->dev, res.start, SZ_4K);
	if (!sm->doorbell)
		return -ENOMEM;

	dev_info(sm->dev, "doorbell at bus %#x, cpu %pR\n", sm->db_bus_addr,
		 &res);

	return 0;
}

static int s5300_poll_boot_stage(struct s5300_modem *sm)
{
	u32 val;
	int i;

	for (i = 0; i < S5300_POLL_COUNT; i++) {
		val = readl(sm->msi + S5300_MSI_BOOT_STAGE);
		if (val == S5300_BOOT_STAGE_DONE)
			return 0;
		msleep(S5300_POLL_INTERVAL_MS);
	}

	dev_err(sm->dev, "boot_stage stuck at %#x (err_report %#x)\n", val,
		readl(sm->msi + S5300_MSI_ERR_REPORT));
	return -ETIMEDOUT;
}

static int s5300_poll_cp_wakeup(struct s5300_modem *sm)
{
	int i;

	for (i = 0; i < S5300_POLL_COUNT; i++) {
		if (gpiod_get_value_cansleep(sm->cp2ap_wakeup))
			return 0;
		msleep(S5300_POLL_INTERVAL_MS);
	}

	dev_err(sm->dev, "CP2AP_WAKEUP never asserted after link bounce\n");
	return -ETIMEDOUT;
}

/* --- boot ioctls --------------------------------------------------------- */

static int s5300_power_on(struct s5300_modem *sm)
{
	unsigned long flags;

	reinit_completion(&sm->init_done);
	WRITE_ONCE(sm->online, false);
	sm->cp_status = S5300_STATE_OFFLINE;
	spin_lock_irqsave(&sm->rx_lock, flags);
	kfifo_reset(&sm->rx_fifo);
	spin_unlock_irqrestore(&sm->rx_lock, flags);

	writel(0, sm->msi + S5300_MSI_BOOT_STAGE);
	s5300_init_control_messages(sm);

	dev_info(sm->dev, "power on: control messages published\n");
	return 0;
}

static int s5300_load_cp_image(struct s5300_modem *sm, void __user *arg)
{
	struct s5300_cp_image img;
	void *buf;
	u32 dst;
	int ret = 0;

	if (copy_from_user(&img, arg, sizeof(img)))
		return -EFAULT;

	/*
	 * The first-stage image lands in the IPC carveout at boot_img_offset +
	 * m_offset (downstream link_load_cp_image() PCIE path); START then
	 * points the ROM at boot_img_offset.  cbd sends the whole PBL in one
	 * call, but honour chunked loads too.  The PBL is its own size domain
	 * (up to ~0x16800, larger than a std_dl frame), so bounce it through a
	 * dedicated buffer rather than the TX frame staging.
	 */
	dst = S5300_BOOT_IMG_OFFSET + img.m_offset;
	if (!img.len || img.len > S5300_PBL_MAX ||
	    (u64)dst + img.len > sm->ipc_size) {
		dev_err(sm->dev, "PBL chunk out of range (dst %#x len %u)\n",
			dst, img.len);
		return -EINVAL;
	}
	if (img.m_offset)
		dev_warn(sm->dev, "PBL m_offset %#x (START publishes base only)\n",
			 img.m_offset);

	buf = kmalloc(img.len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	if (copy_from_user(buf, u64_to_user_ptr(img.binary), img.len)) {
		ret = -EFAULT;
		goto out;
	}
	memcpy_toio(sm->ipc + dst, buf, img.len);
	sm->pbl_size = img.size;

	dev_info(sm->dev, "staged PBL chunk: %u bytes at ipc+%#x (total %u)\n",
		 img.len, dst, img.size);
out:
	kfree(buf);
	return ret;
}

static int s5300_start_bootloader(struct s5300_modem *sm)
{
	struct pci_dev *pdev = sm->pdev;
	int ret;

	if (!sm->pbl_size)
		return dev_err_probe(sm->dev, -EINVAL, "no PBL staged\n");

	sm->cp_status = S5300_STATE_BOOTING;

	/* Arm the download ring (magic 0xBDBD) before the first-stage kick. */
	s5300_init_boot_ring(sm);

	/* Publish the PBL location through the MSI block. */
	writel(lower_32_bits(sm->ipc_phys + S5300_BOOT_IMG_OFFSET),
	       sm->msi + S5300_MSI_IMG_ADDR_LO);
	writel(upper_32_bits(sm->ipc_phys + S5300_BOOT_IMG_OFFSET),
	       sm->msi + S5300_MSI_IMG_ADDR_HI);
	writel(sm->pbl_size, sm->msi + S5300_MSI_IMG_SIZE);

	/* The ROM reads these on the doorbell; make sure the writes stuck. */
	s5300_verify_msi_target(sm);

	/* Config state (forced BAR0, MSI capability) must survive the bounce. */
	pci_save_state(pdev);

	dev_info(sm->dev, "first-stage download (%u bytes at %pap+%#x)\n",
		 sm->pbl_size, &sm->ipc_phys, S5300_BOOT_IMG_OFFSET);
	s5300_send_doorbell(sm, S5300_DB_MSG);

	ret = s5300_poll_boot_stage(sm);
	if (ret)
		return ret;
	dev_info(sm->dev, "first-stage bootloader up, bouncing the link\n");

	/*
	 * The CP bootloader expects a link drop and retrain before it serves
	 * the download (downstream start_normal_boot()); CP2AP_WAKEUP signals
	 * it is ready to re-link.
	 */
	ret = zumapro_pcie_modem_link_down(sm->rc_dev);
	if (ret)
		return ret;
	ret = s5300_poll_cp_wakeup(sm);
	if (ret)
		return ret;
	ret = zumapro_pcie_modem_link_up(sm->rc_dev);
	if (ret) {
		dev_err(sm->dev, "link retrain after bounce failed: %d\n", ret);
		return ret;
	}
	pci_restore_state(pdev);
	/*
	 * Downstream does not trust restore for the forced BAR: it re-reads and
	 * rewrites it after every link-up.  Re-verify the BAR, bridge window
	 * and MSI target on the fresh link.
	 */
	s5300_program_doorbell_bar(sm);
	s5300_open_bridge_window(sm);
	s5300_verify_msi_target(sm);

	s5300_send_doorbell(sm, S5300_DB_LINK_ACK);

	dev_info(sm->dev, "CP download server up, awaiting std_dl stream\n");
	return 0;
}

static int s5300_handover_block(struct s5300_modem *sm, void __user *arg)
{
	u8 buf[S5300_HANDOVER_SIZE];

	if (copy_from_user(buf, arg, sizeof(buf)))
		return -EFAULT;

	memcpy_toio(sm->ipc + S5300_IPC_HANDOVER, buf, sizeof(buf));
	dev_info(sm->dev, "staged %zu-byte handover block\n", sizeof(buf));
	return 0;
}

static int s5300_complete_boot(struct s5300_modem *sm)
{
	if (!wait_for_completion_timeout(&sm->init_done, S5300_INIT_TIMEOUT)) {
		dev_err(sm->dev,
			"CP handshake timed out (cp2ap %#x boot_stage %#x err %#x)\n",
			readl(sm->ipc + S5300_IPC_CP2AP_MSG),
			readl(sm->msi + S5300_MSI_BOOT_STAGE),
			readl(sm->msi + S5300_MSI_ERR_REPORT));
		return -ETIMEDOUT;
	}

	dev_info(sm->dev, "CP is ONLINE\n");
	return 0;
}

/* --- chardev fops -------------------------------------------------------- */

static int s5300_dev_open(struct inode *inode, struct file *file)
{
	struct s5300_modem *sm = container_of(file->private_data,
					      struct s5300_modem, miscdev);

	file->private_data = sm;
	return 0;
}

/*
 * One write() is one std_dl frame.  Prepend the 12-byte EXYNOS link header,
 * 8-byte pad the frame (BOOT_ALIGNED), and copy it onto the NORM_RAW txq.  No
 * per-frame doorbell: the CP's download server polls the ring (downstream
 * xmit_to_legacy_link()).
 */
static ssize_t s5300_dev_write(struct file *file, const char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct s5300_modem *sm = file->private_data;
	void __iomem *txq = sm->ipc + S5300_RAW_BUF_OFFSET;
	u32 flen, total, pad, in, out, space;
	u8 *frame = sm->tx_buf;
	u16 seq;
	int ret;

	if (count == 0)
		return 0;
	if (count > S5300_TX_MAX)
		return -EMSGSIZE;

	flen = S5300_HDR_SIZE + count;
	total = round_up(flen, 8);
	pad = total - flen;

	ret = mutex_lock_interruptible(&sm->io_lock);
	if (ret)
		return ret;

	seq = ++sm->frame_seq;
	frame[0] = S5300_HDR_SYNC & 0xff;
	frame[1] = S5300_HDR_SYNC >> 8;
	frame[2] = seq & 0xff;
	frame[3] = seq >> 8;
	frame[4] = S5300_HDR_CFG_SINGLE & 0xff;
	frame[5] = S5300_HDR_CFG_SINGLE >> 8;
	frame[6] = flen & 0xff;
	frame[7] = flen >> 8;
	frame[8] = S5300_BOOT_CH;
	frame[9] = ++sm->ch_seq;
	frame[10] = 0;
	frame[11] = 0;

	if (copy_from_user(frame + S5300_HDR_SIZE, buf, count)) {
		ret = -EFAULT;
		goto out;
	}
	if (pad)
		memset(frame + flen, 0, pad);

	in = readl(sm->ipc + S5300_RAW_TXQ_HEAD);
	out = readl(sm->ipc + S5300_RAW_TXQ_TAIL);
	if (in >= S5300_RAW_TXQ_SIZE || out >= S5300_RAW_TXQ_SIZE) {
		dev_err(sm->dev, "txq pointers out of range (in %#x out %#x)\n",
			in, out);
		ret = -EIO;
		goto out;
	}
	space = s5300_circ_space(S5300_RAW_TXQ_SIZE, in, out);
	if (space < total) {
		dev_err(sm->dev, "txq full (space %u need %u)\n", space, total);
		ret = -ENOSPC;
		goto out;
	}

	s5300_circ_write(txq, frame, S5300_RAW_TXQ_SIZE, in, total);
	/* Order the payload store ahead of the head advance the CP polls. */
	wmb();
	writel(s5300_circ_new(S5300_RAW_TXQ_SIZE, in, total),
	       sm->ipc + S5300_RAW_TXQ_HEAD);

	ret = count;
out:
	mutex_unlock(&sm->io_lock);
	return ret;
}

static ssize_t s5300_dev_read(struct file *file, char __user *buf,
			      size_t count, loff_t *ppos)
{
	struct s5300_modem *sm = file->private_data;
	unsigned int copied = 0;
	int ret;

	if (count == 0)
		return 0;

	if (kfifo_is_empty(&sm->rx_fifo)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(sm->read_wq,
					       !kfifo_is_empty(&sm->rx_fifo));
		if (ret)
			return ret;
	}

	ret = kfifo_to_user(&sm->rx_fifo, buf, count, &copied);
	if (ret)
		return ret;
	return copied;
}

static __poll_t s5300_dev_poll(struct file *file, poll_table *wait)
{
	struct s5300_modem *sm = file->private_data;

	poll_wait(file, &sm->read_wq, wait);
	if (!kfifo_is_empty(&sm->rx_fifo))
		return EPOLLIN | EPOLLRDNORM;
	return 0;
}

static long s5300_dev_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	struct s5300_modem *sm = file->private_data;
	void __user *uarg = (void __user *)arg;
	int ret;

	switch (cmd) {
	case IOCTL_GET_CP_STATUS:
		return sm->cp_status;
	case IOCTL_COMPLETE_NORMAL_BOOTUP:
		/* Blocks on the handshake; must not hold io_lock. */
		return s5300_complete_boot(sm);
	}

	ret = mutex_lock_interruptible(&sm->io_lock);
	if (ret)
		return ret;

	switch (cmd) {
	case IOCTL_HANDOVER_BLOCK_INFO:
		ret = s5300_handover_block(sm, uarg);
		break;
	case IOCTL_POWER_ON:
		ret = s5300_power_on(sm);
		break;
	case IOCTL_LOAD_CP_IMAGE:
		ret = s5300_load_cp_image(sm, uarg);
		break;
	case IOCTL_START_CP_BOOTLOADER:
		ret = s5300_start_bootloader(sm);
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	mutex_unlock(&sm->io_lock);
	return ret;
}

static const struct file_operations s5300_fops = {
	.owner		= THIS_MODULE,
	.open		= s5300_dev_open,
	.read		= s5300_dev_read,
	.write		= s5300_dev_write,
	.poll		= s5300_dev_poll,
	.unlocked_ioctl	= s5300_dev_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
};

/* --- SIT control port (runtime FMT queue) -------------------------------- */

static u32 s5300_fmt_txq_space(struct s5300_modem *sm)
{
	u32 in = readl(sm->ipc + S5300_FMT_TXQ_HEAD);
	u32 out = readl(sm->ipc + S5300_FMT_TXQ_TAIL);

	if (in >= S5300_FMT_TXQ_SIZE || out >= S5300_FMT_TXQ_SIZE)
		return 0;
	return s5300_circ_space(S5300_FMT_TXQ_SIZE, in, out);
}

/*
 * Wrap one app message (SIT ch 0xf5 or oem ch 0x82) in the 12-byte EXYNOS link
 * header (single frame, 8-byte padded), copy it onto the FMT txq and ring the
 * CP's FMT data doorbell.  The two post-ONLINE writers and the shared
 * fmt_frame_seq are serialised by fmt_tx_lock; each caller owns its staging
 * buffer and per-channel ch_seq.
 */
static int s5300_fmt_ring_tx(struct s5300_modem *sm, u8 *staging, u8 ch,
			     u8 *ch_seq, u32 max, const u8 *data, u32 len)
{
	void __iomem *txq = sm->ipc + S5300_FMT_TXQ_OFFSET;
	u32 flen, total, pad, in, out, space;
	u8 *frame = staging;
	u16 seq;
	int ret;

	if (!READ_ONCE(sm->online))
		return -ENODEV;
	if (len == 0 || len > max)
		return -EMSGSIZE;

	flen = S5300_HDR_SIZE + len;
	total = round_up(flen, 8);
	pad = total - flen;

	mutex_lock(&sm->fmt_tx_lock);

	seq = ++sm->fmt_frame_seq;
	frame[0] = S5300_HDR_SYNC & 0xff;
	frame[1] = S5300_HDR_SYNC >> 8;
	frame[2] = seq & 0xff;
	frame[3] = seq >> 8;
	frame[4] = S5300_HDR_CFG_SINGLE & 0xff;
	frame[5] = S5300_HDR_CFG_SINGLE >> 8;
	frame[6] = flen & 0xff;
	frame[7] = flen >> 8;
	frame[8] = ch;
	frame[9] = ++*ch_seq;
	frame[10] = 0;
	frame[11] = 0;
	memcpy(frame + S5300_HDR_SIZE, data, len);
	if (pad)
		memset(frame + flen, 0, pad);

	in = readl(sm->ipc + S5300_FMT_TXQ_HEAD);
	out = readl(sm->ipc + S5300_FMT_TXQ_TAIL);
	if (in >= S5300_FMT_TXQ_SIZE || out >= S5300_FMT_TXQ_SIZE) {
		dev_err(sm->dev, "fmt txq pointers out of range (in %#x out %#x)\n",
			in, out);
		ret = -EIO;
		goto out_unlock;
	}
	space = s5300_circ_space(S5300_FMT_TXQ_SIZE, in, out);
	if (space < total) {
		dev_warn_ratelimited(sm->dev, "fmt txq full (space %u need %u)\n",
				     space, total);
		/*
		 * The ring already holds an undrained frame -- the CP owes us the
		 * space.  Once full, every further send short-circuits here without
		 * ringing the doorbell, so if the CP missed the original SEND_FMT
		 * (a later ap2cp_msg write clobbered it, or the notification was
		 * lost) it is never told to drain and the ring wedges permanently
		 * (hw-observed: an oem file reply that fills the 4K ring stalls with
		 * FMT_TXQ head!=tail forever).  Re-assert SEND_FMT so a blocked
		 * writer keeps nudging the CP until it drains.
		 */
		s5300_send_ipc_irq(sm, S5300_MASK(S5300_MASK_SEND_FMT));
		ret = -EBUSY;
		goto out_unlock;
	}

	s5300_circ_write(txq, frame, S5300_FMT_TXQ_SIZE, in, total);
	/* Order the payload store ahead of the head advance the CP reads. */
	wmb();
	writel(s5300_circ_new(S5300_FMT_TXQ_SIZE, in, total),
	       sm->ipc + S5300_FMT_TXQ_HEAD);

	s5300_send_ipc_irq(sm, S5300_MASK(S5300_MASK_SEND_FMT));
	ret = 0;
out_unlock:
	mutex_unlock(&sm->fmt_tx_lock);
	return ret;
}

static int s5300_ctrl_start(struct wwan_port *port)
{
	return 0;
}

static void s5300_ctrl_stop(struct wwan_port *port)
{
}

static int s5300_ctrl_tx(struct wwan_port *port, struct sk_buff *skb)
{
	struct s5300_modem *sm = wwan_port_get_drvdata(port);
	int ret;

	/* Linear: the port is created with caps=NULL, so frag_len is SIZE_MAX. */
	ret = s5300_fmt_ring_tx(sm, sm->fmt_tx_buf, S5300_FMT_CH,
				&sm->fmt_ch_seq, S5300_FMT_MAX,
				skb->data, skb->len);
	if (ret)
		return ret;

	consume_skb(skb);
	return 0;
}

static const struct wwan_port_ops s5300_ctrl_ops = {
	.start	= s5300_ctrl_start,
	.stop	= s5300_ctrl_stop,
	.tx	= s5300_ctrl_tx,
};

static int s5300_chardev_open(struct inode *inode, struct file *file)
{
	struct s5300_chardev *cd = container_of(file->private_data,
						struct s5300_chardev, miscdev);

	file->private_data = cd;
	return 0;
}

/* One read() returns exactly one received frame's payload (datagram-like). */
static ssize_t s5300_chardev_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct s5300_chardev *cd = file->private_data;
	struct sk_buff *skb;
	size_t n;

	skb = skb_dequeue(&cd->rxq);
	if (!skb) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(cd->read_wq,
					     (skb = skb_dequeue(&cd->rxq))))
			return -ERESTARTSYS;
	}

	n = min(count, (size_t)skb->len);
	if (copy_to_user(buf, skb->data, n)) {
		/* Keep the message for a retry rather than losing it. */
		skb_queue_head(&cd->rxq, skb);
		return -EFAULT;
	}
	kfree_skb(skb);
	return n;
}

/*
 * One write() is one app message.  It fills at most one FMT ring slot, so on a
 * full ring (-EBUSY) block until the CP drains it (fmt_tx_wq, woken from the IRQ
 * handler) rather than dropping the frame; O_NONBLOCK maps that to -EAGAIN.
 */
static ssize_t s5300_chardev_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct s5300_chardev *cd = file->private_data;
	struct s5300_modem *sm = cd->sm;
	unsigned long deadline;
	u32 needed;
	u8 *kbuf;
	int ret;

	if (count == 0)
		return 0;
	if (count > cd->tx_max)
		return -EMSGSIZE;

	kbuf = kmalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;
	if (copy_from_user(kbuf, buf, count)) {
		kfree(kbuf);
		return -EFAULT;
	}

	needed = round_up(S5300_HDR_SIZE + count, 8);
	deadline = jiffies + msecs_to_jiffies(S5300_FMT_TX_TIMEOUT_MS);
	for (;;) {
		ret = cd->raw_ring
			? s5300_raw_ring_tx(sm, cd->tx_buf, cd->channel,
					    &cd->ch_seq, cd->tx_max, kbuf, count)
			: s5300_fmt_ring_tx(sm, cd->tx_buf, cd->channel,
					    &cd->ch_seq, cd->tx_max, kbuf, count);
		if (ret != -EBUSY)
			break;
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}
		if (time_after_eq(jiffies, deadline)) {
			ret = -ETIMEDOUT;
			break;
		}
		if (cd->raw_ring) {
			/*
			 * NORM_RAW is request/response ping-pong (RFS): the CP
			 * drains our frame before the next write, so the ring
			 * effectively never fills.  There is no raw TX wq to wake
			 * on, so just re-check after a short sleep.
			 */
			if (msleep_interruptible(S5300_FMT_TX_POLL_MS)) {
				ret = -ERESTARTSYS;
				break;
			}
		} else {
			/*
			 * The CP frees FMT ring space by draining our frame,
			 * which does not MSI us, so re-check on a short tick.
			 * Each retry re-rings SEND_FMT (in s5300_fmt_ring_tx),
			 * so a blocked writer keeps nudging the CP.
			 */
			ret = wait_event_interruptible_timeout(sm->fmt_tx_wq,
					s5300_fmt_txq_space(sm) >= needed,
					msecs_to_jiffies(S5300_FMT_TX_POLL_MS));
			if (ret < 0)
				break;		/* interrupted */
		}
	}

	kfree(kbuf);
	return ret < 0 ? ret : (ssize_t)count;
}

static __poll_t s5300_chardev_poll(struct file *file, poll_table *wait)
{
	struct s5300_chardev *cd = file->private_data;
	__poll_t mask = EPOLLOUT | EPOLLWRNORM;	/* TX backpressure lives in write() */

	poll_wait(file, &cd->read_wq, wait);
	if (!skb_queue_empty(&cd->rxq))
		mask |= EPOLLIN | EPOLLRDNORM;
	return mask;
}

static const struct file_operations s5300_chardev_fops = {
	.owner		= THIS_MODULE,
	.open		= s5300_chardev_open,
	.read		= s5300_chardev_read,
	.write		= s5300_chardev_write,
	.poll		= s5300_chardev_poll,
};

/* --- shared NORM_RAW ring TX (RFS + AT, post-ONLINE) --------------------- */

/*
 * Wrap one frame in the 12-byte EXYNOS link header (single frame, @ch), copy it
 * onto the NORM_RAW txq and ring the CP's RAW data doorbell.  Both post-ONLINE
 * raw writers -- RFS (0x29) and the vendor AT/router (0x15) -- funnel through
 * here, so the txq head RMW and the frame_seq counter are serialised by
 * raw_tx_lock; each caller owns its @staging buffer and @ch_seq.  The boot
 * std_dl writer shares the ring but is temporally disjoint (pre-ONLINE) and
 * io_lock-serialised.
 */
static int s5300_raw_ring_tx(struct s5300_modem *sm, u8 *staging, u8 ch,
			     u8 *ch_seq, u32 max, const u8 *data, u32 len)
{
	void __iomem *txq = sm->ipc + S5300_RAW_BUF_OFFSET;
	u32 flen, total, pad, in, out, space;
	u8 *frame = staging;
	u16 seq;
	int ret;

	if (!READ_ONCE(sm->online))
		return -ENODEV;
	if (len == 0 || len > max)
		return -EMSGSIZE;

	flen = S5300_HDR_SIZE + len;
	total = round_up(flen, 8);
	pad = total - flen;

	mutex_lock(&sm->raw_tx_lock);

	seq = ++sm->frame_seq;
	frame[0] = S5300_HDR_SYNC & 0xff;
	frame[1] = S5300_HDR_SYNC >> 8;
	frame[2] = seq & 0xff;
	frame[3] = seq >> 8;
	frame[4] = S5300_HDR_CFG_SINGLE & 0xff;
	frame[5] = S5300_HDR_CFG_SINGLE >> 8;
	frame[6] = flen & 0xff;
	frame[7] = flen >> 8;
	frame[8] = ch;
	frame[9] = ++*ch_seq;
	frame[10] = 0;
	frame[11] = 0;
	memcpy(frame + S5300_HDR_SIZE, data, len);
	if (pad)
		memset(frame + flen, 0, pad);

	in = readl(sm->ipc + S5300_RAW_TXQ_HEAD);
	out = readl(sm->ipc + S5300_RAW_TXQ_TAIL);
	if (in >= S5300_RAW_TXQ_SIZE || out >= S5300_RAW_TXQ_SIZE) {
		dev_err(sm->dev, "raw txq pointers out of range (in %#x out %#x)\n",
			in, out);
		ret = -EIO;
		goto out_unlock;
	}
	space = s5300_circ_space(S5300_RAW_TXQ_SIZE, in, out);
	if (space < total) {
		dev_warn(sm->dev, "raw txq full (space %u need %u)\n", space, total);
		ret = -EBUSY;
		goto out_unlock;
	}

	s5300_circ_write(txq, frame, S5300_RAW_TXQ_SIZE, in, total);
	/* Order the payload store ahead of the head advance the CP reads. */
	wmb();
	writel(s5300_circ_new(S5300_RAW_TXQ_SIZE, in, total),
	       sm->ipc + S5300_RAW_TXQ_HEAD);

	s5300_send_ipc_irq(sm, S5300_MASK(S5300_MASK_SEND_RAW));
	ret = 0;
out_unlock:
	mutex_unlock(&sm->raw_tx_lock);
	return ret;
}

/* --- probe / remove ------------------------------------------------------ */

/* --- PKTPROC data path (DL/RX; UL is a later stage) ---------------------- */

/*
 * Point descriptor @idx at its fixed reserved buffer (buff + max_pkt*idx in CP
 * address space) and set the ring-boundary control bits.  tegu has no CP IOMMU
 * and a 1:1 descriptor<->buffer mapping, so the address is static; re-arming on
 * refill is idempotent (the CP overwrites only length/channel/status).
 */
static void s5300_pktproc_dl_arm(struct s5300_modem *sm, u32 q, u32 idx)
{
	void __iomem *desc = sm->pktproc + S5300_PKTPROC_DL_DESC_OFF +
			     q * S5300_PKTPROC_DL_DESC_BY_Q +
			     idx * S5300_PKTPROC_DESC_SZ;
	u32 cp_buff = S5300_PKTPROC_CP_BASE + S5300_PKTPROC_DL_BUFF_OFF +
		      q * S5300_PKTPROC_DL_BUFF_BY_Q;
	u32 ctrl = 0;

	if (idx == 0)
		ctrl |= S5300_PKTPROC_CTRL_HEAD;
	if (idx == sm->dl_num_desc - 1)
		ctrl |= S5300_PKTPROC_CTRL_RINGEND;

	writel(cp_buff + S5300_PKTPROC_MAX_PKT * idx, desc + S5300_DESC_ADDR_LO);
	writel(ctrl << 8, desc + S5300_DESC_W1);
}

/*
 * Program the DL info/q_info and prime the ring, from the INIT_START handler
 * before PIF_INIT_DONE (downstream cmd_init_start_handler order): the CP reads
 * num_queues/desc_mode/max_packet_size and the queue's desc/buff bases as it
 * comes up, then DMAs DL packets into the armed buffers.  DL needs no capability
 * bit, so this is safe while the AP capability is still 0x2.
 */
static void s5300_pktproc_dl_init(struct s5300_modem *sm)
{
	void __iomem *info = sm->pktproc + S5300_PKTPROC_DL_INFO_OFF;
	u32 n = S5300_PKTPROC_DL_NUM_DESC;
	u32 q, i;

	sm->dl_num_desc = n;

	memset_io(info, 0, SZ_4K);

	/* info_v2 word0: num_queues:4 | desc_mode:2 | irq_mode:2 | max_pkt:16. */
	writel(S5300_PKTPROC_DL_NUM_Q | (S5300_PKTPROC_DESC_MODE_SKTBUF << 4) |
	       (S5300_PKTPROC_IRQ_EXCLUSIVE << 6) | (S5300_PKTPROC_MAX_PKT << 8),
	       info);

	for (q = 0; q < S5300_PKTPROC_DL_NUM_Q; q++) {
		void __iomem *qi = info + S5300_PKTPROC_DL_QINFO(q);

		writel(S5300_PKTPROC_CP_BASE + S5300_PKTPROC_DL_DESC_OFF +
		       q * S5300_PKTPROC_DL_DESC_BY_Q, qi + S5300_QINFO_CP_DESC);
		writel(n, qi + S5300_QINFO_NUM_DESC);
		writel(S5300_PKTPROC_CP_BASE + S5300_PKTPROC_DL_BUFF_OFF +
		       q * S5300_PKTPROC_DL_BUFF_BY_Q, qi + S5300_QINFO_CP_BUFF);
		writel(0, qi + S5300_QINFO_REAR);

		/* Arm all descriptors; offer n-1 (circ leaves a one-slot gap). */
		for (i = 0; i < n; i++)
			s5300_pktproc_dl_arm(sm, q, i);
		sm->dl_fore[q] = n - 1;
		sm->dl_done[q] = 0;
		writel(n - 1, qi + S5300_QINFO_FORE);
	}

	dev_info(sm->dev, "pktproc DL armed: %u queues x %u desc (exclusive)\n",
		 S5300_PKTPROC_DL_NUM_Q, n);
}

/*
 * Drain filled DL descriptors [done..rear) into skbs and refill the freed
 * slots.  Runs from the MSI handler once ONLINE.  Copy-out (no CP IOMMU): the
 * packet sits at buff_vbase + max_pkt*done, length from the descriptor.  Raw IP:
 * sniff the version nibble and hand it to the data netdev.  Bounded by num_desc
 * so a garbled rear cannot spin.
 */
static void s5300_pktproc_dl_drain(struct s5300_modem *sm)
{
	void __iomem *info = sm->pktproc + S5300_PKTPROC_DL_INFO_OFF;
	u32 n = sm->dl_num_desc;
	u32 q;

	if (!n || !sm->ndev)
		return;

	/* Exclusive irq_mode: the CP fires per-queue MSIs we do not wire up, so
	 * poll every queue's rear_ptr from the shared MSI-0 handler instead.
	 */
	for (q = 0; q < S5300_PKTPROC_DL_NUM_Q; q++) {
		void __iomem *qi = info + S5300_PKTPROC_DL_QINFO(q);
		void __iomem *descs = sm->pktproc + S5300_PKTPROC_DL_DESC_OFF +
				      q * S5300_PKTPROC_DL_DESC_BY_Q;
		void __iomem *buff = sm->pktproc + S5300_PKTPROC_DL_BUFF_OFF +
				     q * S5300_PKTPROC_DL_BUFF_BY_Q;
		u32 rear = readl(qi + S5300_QINFO_REAR) % n;
		u32 done = sm->dl_done[q];
		u32 space, guard, i;

		for (guard = 0; done != rear && guard < n; guard++) {
			void __iomem *d = descs + done * S5300_PKTPROC_DESC_SZ;
			u32 len = readl(d + S5300_DESC_LEN) & 0xffff;
			struct sk_buff *skb;

			if (len == 0 || len > S5300_PKTPROC_MAX_PKT) {
				sm->ndev->stats.rx_length_errors++;
				goto next;
			}
			skb = netdev_alloc_skb(sm->ndev, len);
			if (!skb) {
				sm->ndev->stats.rx_dropped++;
				goto next;
			}
			memcpy_fromio(skb_put(skb, len),
				      buff + done * S5300_PKTPROC_MAX_PKT, len);
			skb->protocol = htons((skb->data[0] >> 4) == 6 ?
					      ETH_P_IPV6 : ETH_P_IP);
			skb->dev = sm->ndev;
			skb_reset_mac_header(skb);
			skb_reset_network_header(skb);
			sm->ndev->stats.rx_packets++;
			sm->ndev->stats.rx_bytes += len;
			netif_rx(skb);
next:
			done = (done + 1 == n) ? 0 : done + 1;
		}
		sm->dl_done[q] = done;

		/* Refill freed slots: circ_get_space(n, fore, done). */
		space = (done + n - sm->dl_fore[q] - 1) % n;
		for (i = 0; i < space; i++) {
			s5300_pktproc_dl_arm(sm, q, sm->dl_fore[q]);
			sm->dl_fore[q] = (sm->dl_fore[q] + 1 == n) ?
					 0 : sm->dl_fore[q] + 1;
		}
		if (space)
			writel(sm->dl_fore[q], qi + S5300_QINFO_FORE);
	}
}

/*
 * Provision both UL queues (geometry only) at INIT_START, before the AP
 * capability advertises PKTPROC_UL: the CP consumes these rings as soon as it
 * reads the bit, so q_info must be valid first.  The CP writes end_bit_owner +
 * cp_quota into the info header afterwards (read at PHONE_START).  The UL
 * descriptor region needs no priming -- fore starts 0, so the CP reads nothing
 * until a TX writes a full descriptor.
 */
static void s5300_pktproc_ul_setup(struct s5300_modem *sm)
{
	void __iomem *info = sm->pktproc + S5300_PKTPROC_UL_INFO_OFF;
	static const u32 num_desc[S5300_PKTPROC_UL_NUM_Q] = {
		S5300_PKTPROC_UL_HI_NUM_DESC, S5300_PKTPROC_UL_NUM_DESC };
	static const u32 desc_off[S5300_PKTPROC_UL_NUM_Q] = {
		0, S5300_PKTPROC_UL_HI_DESC_SZ };
	u32 i;

	sm->ul_num_desc = S5300_PKTPROC_UL_NUM_DESC;	/* NORM (TX) queue */
	sm->ul_done = 0;
	sm->ul_active = false;

	memset_io(info, 0, SZ_4K);
	writel(S5300_PKTPROC_UL_NUM_Q, info);	/* num_queues; CP fills the rest */

	/* HIPRIO (q0) and NORM (q1): desc rings contiguous by actual size, buffers
	 * split evenly.  q0 is left idle but provisioned to the CP's geometry.
	 */
	for (i = 0; i < S5300_PKTPROC_UL_NUM_Q; i++) {
		void __iomem *qi = info + S5300_PKTPROC_UL_QINFO(i);

		writel(S5300_PKTPROC_CP_BASE + S5300_PKTPROC_UL_DESC_OFF + desc_off[i],
		       qi + S5300_QINFO_CP_DESC);
		writel(num_desc[i], qi + S5300_QINFO_NUM_DESC);
		writel(S5300_PKTPROC_CP_BASE + S5300_PKTPROC_UL_BUFF_OFF +
		       i * S5300_PKTPROC_UL_BUFF_BY_Q, qi + S5300_QINFO_CP_BUFF);
		writel(0, qi + S5300_QINFO_FORE);
		writel(0, qi + S5300_QINFO_REAR);
	}
}

/*
 * At PHONE_START: read the CP's end_bit_owner (info word0 bit24) + cp_quota
 * (word1), re-zero the TX queue, and enable UL transmit.
 */
static void s5300_pktproc_ul_activate(struct s5300_modem *sm)
{
	void __iomem *info = sm->pktproc + S5300_PKTPROC_UL_INFO_OFF;
	void __iomem *qi = info + S5300_PKTPROC_UL_QINFO(S5300_PKTPROC_UL_TXQ);

	sm->ul_end_bit_owner = (readl(info) >> 24) & 1;
	sm->ul_cp_quota = readl(info + 4) & 0xffff;
	sm->ul_done = 0;
	writel(0, qi + S5300_QINFO_FORE);
	writel(0, qi + S5300_QINFO_REAR);
	/* Only transmit once PKTPROC_UL is advertised; otherwise the CP does not
	 * consume the UL ring and an up'd rmnet0 would ring spurious doorbells.
	 */
	sm->ul_active = S5300_AP_CAPABILITY_0 & 0x1;

	dev_info(sm->dev, "pktproc UL %s: end_bit_owner=%u cp_quota=%u\n",
		 sm->ul_active ? "active" : "provisioned (UL cap withheld)",
		 sm->ul_end_bit_owner, sm->ul_cp_quota);
}

/*
 * Transmit one skb on the NORM UL queue: copy into the ring buffer, write the
 * 32-byte descriptor, publish fore_ptr and ring the CP.  Runs under the netdev
 * tx lock (serialised), so ul_done needs no extra lock.  Returns false if the
 * ring is full (caller drops).
 */
static bool s5300_pktproc_ul_xmit(struct s5300_modem *sm, struct sk_buff *skb)
{
	void __iomem *info = sm->pktproc + S5300_PKTPROC_UL_INFO_OFF;
	void __iomem *qi = info + S5300_PKTPROC_UL_QINFO(S5300_PKTPROC_UL_TXQ);
	void __iomem *desc = sm->pktproc + S5300_PKTPROC_UL_DESC_BASE +
			     sm->ul_done * S5300_PKTPROC_UL_DESC_SZ;
	void __iomem *buf = sm->pktproc + S5300_PKTPROC_UL_BUFF_OFF +
			    S5300_PKTPROC_UL_TXQ * S5300_PKTPROC_UL_BUFF_BY_Q +
			    sm->ul_done * S5300_PKTPROC_UL_MAX_PKT;
	u32 cp_buf = S5300_PKTPROC_CP_BASE + S5300_PKTPROC_UL_BUFF_OFF +
		     S5300_PKTPROC_UL_TXQ * S5300_PKTPROC_UL_BUFF_BY_Q +
		     sm->ul_done * S5300_PKTPROC_UL_MAX_PKT;
	u32 n = sm->ul_num_desc;
	u32 rear = readl(qi + S5300_QINFO_REAR) % n;
	u32 dsize = skb->len + S5300_PKTPROC_UL_CP_PADDING;
	u32 last = sm->ul_end_bit_owner == S5300_UL_END_BIT_AP ? 1 : 0;

	/* circ_get_space(n, done, rear): need at least one free descriptor. */
	if (((rear + n - sm->ul_done - 1) % n) < 1)
		return false;
	if (skb->len + S5300_PKTPROC_UL_CP_PADDING > S5300_PKTPROC_UL_MAX_PKT)
		return false;

	memcpy_toio(buf, skb->data, skb->len);

	writel(dsize, desc + 0x0);		/* data_size (+CP_PADDING) */
	writel(dsize, desc + 0x4);		/* total_pkt_size */
	writel(cp_buf, desc + 0x8);		/* sktbuf_point[31:0] */
	writel(0, desc + 0xc);			/* sktbuf_point[35:32] + ap2cp */
	writel(last, desc + 0x10);		/* last_desc */
	writel(S5300_PKTPROC_CH_PDP_FIRST << 8, desc + 0x14);	/* lcid @ bits 8-15 */
	writel(0, desc + 0x18);
	writel(0, desc + 0x1c);

	sm->ul_done = (sm->ul_done + 1 == n) ? 0 : sm->ul_done + 1;
	wmb();				/* descriptor + data land before fore_ptr */
	writel(sm->ul_done, qi + S5300_QINFO_FORE);
	s5300_send_ipc_irq(sm, S5300_MASK(S5300_MASK_SEND_DATA));
	return true;
}

static int s5300_ndo_open(struct net_device *ndev)
{
	netif_start_queue(ndev);
	return 0;
}

static int s5300_ndo_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	return 0;
}

static netdev_tx_t s5300_ndo_start_xmit(struct sk_buff *skb,
					struct net_device *ndev)
{
	struct s5300_modem *sm = *(struct s5300_modem **)netdev_priv(ndev);
	unsigned int len;

	/*
	 * CP-driven TX flow control (vector-1 cp2ap_status): the CP asked for
	 * uplink silence, so hold the packet in the qdisc until it resumes.
	 * The re-check closes the race with a resume landing between the flag
	 * read and the queue stop.
	 */
	if (READ_ONCE(sm->tx_suspended)) {
		netif_stop_queue(ndev);
		if (READ_ONCE(sm->tx_suspended))
			return NETDEV_TX_BUSY;
		netif_wake_queue(ndev);
	}

	/* ul_xmit copies only the linear head; we set no SG feature, so this is a
	 * no-op today, but guard the invariant regardless.
	 */
	if (skb_linearize(skb)) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	len = skb->len;

	if (sm->ul_active && s5300_pktproc_ul_xmit(sm, skb)) {
		ndev->stats.tx_packets++;
		ndev->stats.tx_bytes += len;
	} else {
		ndev->stats.tx_dropped++;
	}
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops s5300_netdev_ops = {
	.ndo_open	= s5300_ndo_open,
	.ndo_stop	= s5300_ndo_stop,
	.ndo_start_xmit	= s5300_ndo_start_xmit,
};

static void s5300_netdev_setup(struct net_device *ndev)
{
	ndev->netdev_ops = &s5300_netdev_ops;
	ndev->type = ARPHRD_RAWIP;
	ndev->flags = IFF_POINTOPOINT | IFF_NOARP;
	ndev->hard_header_len = 0;
	ndev->addr_len = 0;
	ndev->mtu = ETH_DATA_LEN;
	ndev->min_mtu = 68;
	ndev->max_mtu = ETH_DATA_LEN;
	ndev->tx_queue_len = 1000;
	ndev->needs_free_netdev = true;
}

static int s5300_map_region(struct s5300_modem *sm, const char *name,
			    phys_addr_t *phys, resource_size_t *size,
			    void __iomem **map)
{
	struct device_node *np;
	struct reserved_mem *rmem;
	int idx;

	idx = of_property_match_string(sm->dev->of_node, "memory-region-names",
				       name);
	if (idx < 0)
		return idx;
	np = of_parse_phandle(sm->dev->of_node, "memory-region", idx);
	if (!np)
		return -ENOENT;
	rmem = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!rmem)
		return -ENOENT;

	*phys = rmem->base;
	if (size)
		*size = rmem->size;
	/*
	 * Non-cached: the CP reads the staged PBL and writes boot/IPC state by
	 * PCIe DMA, and the HSI1 IO-coherency plumbing is not set up yet.
	 */
	*map = devm_ioremap_wc(sm->dev, rmem->base, rmem->size);
	if (!*map)
		return -ENOMEM;

	return 0;
}

static int s5300_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *rc_pdev;
	struct device_node *rc_node;
	struct s5300_modem *sm;
	u16 cmd;
	int ret;

	sm = devm_kzalloc(dev, sizeof(*sm), GFP_KERNEL);
	if (!sm)
		return -ENOMEM;

	sm->dev = dev;
	spin_lock_init(&sm->lock);
	spin_lock_init(&sm->rx_lock);
	mutex_init(&sm->io_lock);
	mutex_init(&sm->fmt_tx_lock);
	mutex_init(&sm->raw_tx_lock);
	init_completion(&sm->init_done);
	init_waitqueue_head(&sm->read_wq);
	init_waitqueue_head(&sm->fmt_tx_wq);
	sm->oem.sm = sm;
	sm->oem.channel = S5300_OEM_CH;
	sm->oem.tx_max = S5300_OEM_MAX;
	skb_queue_head_init(&sm->oem.rxq);
	init_waitqueue_head(&sm->oem.read_wq);
	sm->rfs.sm = sm;
	sm->rfs.channel = S5300_RFS_CH;
	sm->rfs.tx_max = S5300_RFS_MAX;
	sm->rfs.raw_ring = true;
	skb_queue_head_init(&sm->rfs.rxq);
	init_waitqueue_head(&sm->rfs.read_wq);
	sm->cp_status = S5300_STATE_OFFLINE;
	platform_set_drvdata(pdev, sm);

	sm->tx_buf = devm_kmalloc(dev, S5300_TX_BUF_SIZE, GFP_KERNEL);
	if (!sm->tx_buf)
		return -ENOMEM;
	sm->fmt_tx_buf = devm_kmalloc(dev, S5300_HDR_SIZE + S5300_FMT_MAX + 8,
				      GFP_KERNEL);
	if (!sm->fmt_tx_buf)
		return -ENOMEM;
	sm->rfs.tx_buf = devm_kmalloc(dev, S5300_HDR_SIZE + S5300_RFS_MAX + 8,
				      GFP_KERNEL);
	if (!sm->rfs.tx_buf)
		return -ENOMEM;
	sm->oem.tx_buf = devm_kmalloc(dev, S5300_HDR_SIZE + S5300_OEM_MAX + 8,
				      GFP_KERNEL);
	if (!sm->oem.tx_buf)
		return -ENOMEM;
	ret = kfifo_alloc(&sm->rx_fifo, S5300_RX_FIFO_SIZE, GFP_KERNEL);
	if (ret)
		return ret;

	rc_node = of_parse_phandle(dev->of_node, "google,pcie", 0);
	if (!rc_node) {
		ret = dev_err_probe(dev, -EINVAL, "missing google,pcie\n");
		goto err_fifo;
	}
	rc_pdev = of_find_device_by_node(rc_node);
	of_node_put(rc_node);
	if (!rc_pdev) {
		ret = -EPROBE_DEFER;
		goto err_fifo;
	}
	sm->rc_dev = &rc_pdev->dev;
	if (!sm->rc_dev->driver) {
		ret = -EPROBE_DEFER;
		goto err_rc;
	}

	ret = of_property_read_u32(dev->of_node, "samsung,doorbell-addr",
				   &sm->db_bus_addr);
	if (ret) {
		dev_err_probe(dev, ret, "missing samsung,doorbell-addr\n");
		goto err_rc;
	}

	sm->cp2ap_wakeup = devm_gpiod_get(dev, "cp2ap-wakeup", GPIOD_IN);
	if (IS_ERR(sm->cp2ap_wakeup)) {
		ret = dev_err_probe(dev, PTR_ERR(sm->cp2ap_wakeup),
				    "failed to get CP2AP_WAKEUP\n");
		goto err_rc;
	}

	ret = s5300_map_region(sm, "ipc", &sm->ipc_phys, &sm->ipc_size,
			       &sm->ipc);
	if (ret) {
		dev_err_probe(dev, ret, "failed to map IPC carveout\n");
		goto err_rc;
	}
	ret = s5300_map_region(sm, "msi", &sm->msi_phys, NULL, &sm->msi);
	if (ret) {
		dev_err_probe(dev, ret, "failed to map MSI carveout\n");
		goto err_rc;
	}
	ret = s5300_map_region(sm, "pktproc", &sm->pktproc_phys,
			       &sm->pktproc_size, &sm->pktproc);
	if (ret) {
		dev_err_probe(dev, ret, "failed to map PKTPROC carveout\n");
		goto err_rc;
	}
	if (sm->pktproc_size < S5300_PKTPROC_MAP_END) {
		dev_err(dev, "PKTPROC carveout %pa too small for the map (need %#x)\n",
			&sm->pktproc_size, S5300_PKTPROC_MAP_END);
		ret = -EINVAL;
		goto err_rc;
	}

	/*
	 * The zumapro root port carries the same 144d:a5a5 ID as the modem
	 * endpoint, and it registers first -- a bare first-match lookup returns
	 * the root port.  Match the endpoint by port type.
	 */
	sm->pdev = NULL;
	while ((sm->pdev = pci_get_device(S5300_PCI_VENDOR_ID,
					  S5300_PCI_DEVICE_ID, sm->pdev))) {
		if (pci_pcie_type(sm->pdev) == PCI_EXP_TYPE_ENDPOINT)
			break;
	}
	if (!sm->pdev) {
		ret = dev_err_probe(dev, -ENODEV, "CP endpoint not enumerated\n");
		goto err_rc;
	}
	dev_info(dev, "CP endpoint %s\n", pci_name(sm->pdev));

	/*
	 * Downstream keeps every form of link PM off for the whole CP boot; the
	 * core enabled ASPM L1 at enumeration, so take it back out before
	 * poking config space (the mask ROM's config emulation has been seen
	 * returning garbled completions to rapid access bursts).
	 */
	pci_disable_link_state(sm->pdev, PCIE_LINK_STATE_L0S |
			       PCIE_LINK_STATE_L1 | PCIE_LINK_STATE_CLKPM);

	/* Before MSI allocation: the EP capability must carry the carveout. */
	ret = zumapro_pcie_set_msi_target(sm->rc_dev, sm->msi_phys);
	if (ret)
		goto err_pci;

	ret = s5300_setup_doorbell(sm);
	if (ret)
		goto err_pci;

	ret = pci_enable_device(sm->pdev);
	if (ret) {
		dev_err(dev, "pci_enable_device: %d\n", ret);
		goto err_pci;
	}
	pci_set_master(sm->pdev);
	/* MSE by hand: BAR0 is programmed behind the PCI core's back. */
	pci_read_config_word(sm->pdev, PCI_COMMAND, &cmd);
	pci_write_config_word(sm->pdev, PCI_COMMAND, cmd | PCI_COMMAND_MEMORY);

	/*
	 * The core caches the MSI capability offset at enumeration time; if the
	 * mask ROM exposed its capability list late, re-look it up so vector
	 * allocation does not fail on a stale zero.
	 */
	if (!sm->pdev->msi_cap) {
		sm->pdev->msi_cap = pci_find_capability(sm->pdev,
							PCI_CAP_ID_MSI);
		dev_warn(dev, "MSI capability re-lookup: %#x\n",
			 sm->pdev->msi_cap);
	}
	if (!sm->pdev->msi_cap) {
		u16 w40, w50, w52;
		u32 hdr;

		pci_read_config_word(sm->pdev, 0x40, &w40);
		pci_read_config_word(sm->pdev, S5300_ROM_MSI_CAP, &w50);
		pci_read_config_word(sm->pdev, S5300_ROM_MSI_CAP + 2, &w52);
		pci_read_config_dword(sm->pdev, S5300_ROM_MSI_CAP, &hdr);
		dev_warn(dev,
			 "MSI cap probe: w@0x40 %#06x w@0x50 %#06x w@0x52 %#06x dw@0x50 %#010x\n",
			 w40, w50, w52, hdr);
		if ((hdr & PCI_CAP_ID_MASK) == PCI_CAP_ID_MSI)
			sm->pdev->msi_cap = S5300_ROM_MSI_CAP;
	}

	/*
	 * Reserve the RC's own vectors 0-3 so the modem's 4 vectors land at data
	 * base 4 (MME=2), matching downstream byte-for-byte.  MAIN's INIT_START
	 * MSI (message 4) then lands on the EP's vector 0.
	 */
	ret = zumapro_pcie_reserve_msi_base(sm->rc_dev, S5300_MSI_VECTORS);
	if (ret)
		goto err_disable;

	/*
	 * Exactly 4 vectors: the mask ROM aborts the PBL download at any other
	 * MME (8 vectors -> MME=3 regressed boot_stage to 0x1ff on hardware).
	 * Vector 0 = IPC message/command; 1 = TX flow control; 2..3 spare for
	 * pktproc.  Only vector 0 matters until the data path exists.
	 */
	ret = pci_alloc_irq_vectors(sm->pdev, S5300_MSI_VECTORS,
				    S5300_MSI_VECTORS, PCI_IRQ_MSI);
	if (ret < 0) {
		dev_err(dev, "MSI alloc: %d (power state %d, msi_cap %#x)\n",
			ret, sm->pdev->current_state, sm->pdev->msi_cap);
		goto err_disable;
	}
	dev_info(dev, "%d MSI vector(s)\n", ret);

	ret = request_irq(pci_irq_vector(sm->pdev, 0), s5300_irq_handler, 0,
			  "s5300-ipc", sm);
	if (ret)
		goto err_vectors;

	/*
	 * Vector 0 carries the CP's IPC message/command + FMT + PKTPROC DL
	 * notifications; vector 1 is the cp2ap_status TX-flow-control interrupt
	 * (2..3 spare).  A never-requested vector stays disabled in iMSI-RX, so
	 * the CP's suspend request would be silently dropped at the RC.
	 */
	ret = request_irq(pci_irq_vector(sm->pdev, 1), s5300_tx_state_irq, 0,
			  "s5300-tx-state", sm);
	if (ret)
		goto err_irq0;

	sm->miscdev.minor = MISC_DYNAMIC_MINOR;
	sm->miscdev.name = "umts_boot0";
	sm->miscdev.fops = &s5300_fops;
	sm->miscdev.parent = dev;
	ret = misc_register(&sm->miscdev);
	if (ret) {
		dev_err(dev, "misc_register: %d\n", ret);
		goto err_irq;
	}

	/*
	 * The SIT control port for post-ONLINE traffic.  It is created up front
	 * so userspace can open it, but s5300_fmt_ring_tx() rejects writes and
	 * the IRQ only drains the FMT rxq once the CP has reached ONLINE.
	 */
	sm->ctrl_port = wwan_create_port(dev, WWAN_PORT_SIT, &s5300_ctrl_ops,
					 NULL, sm);
	if (IS_ERR(sm->ctrl_port)) {
		ret = PTR_ERR(sm->ctrl_port);
		dev_err(dev, "wwan_create_port: %d\n", ret);
		goto err_misc;
	}

	/*
	 * The RFS file channel (ch 0x29 on the NORM_RAW ring), exposed as
	 * /dev/umts_rfs0 for the userspace server that answers the CP's NV and
	 * carrier-config file requests.  Created up front; only carries traffic
	 * once ONLINE.
	 */
	sm->rfs.miscdev.minor = MISC_DYNAMIC_MINOR;
	sm->rfs.miscdev.name = "umts_rfs0";
	sm->rfs.miscdev.fops = &s5300_chardev_fops;
	sm->rfs.miscdev.parent = dev;
	ret = misc_register(&sm->rfs.miscdev);
	if (ret) {
		dev_err(dev, "misc_register(rfs): %d\n", ret);
		goto err_ctrl_port;
	}

	/*
	 * The oem/GEMS channel (ch 0x82 on the FMT ring), exposed as /dev/umts_oem1
	 * for the userspace daemon that answers the CP's UE-capability-config file
	 * requests.  Created up front; only carries traffic once ONLINE.
	 */
	sm->oem.miscdev.minor = MISC_DYNAMIC_MINOR;
	sm->oem.miscdev.name = "umts_oem1";
	sm->oem.miscdev.fops = &s5300_chardev_fops;
	sm->oem.miscdev.parent = dev;
	ret = misc_register(&sm->oem.miscdev);
	if (ret) {
		dev_err(dev, "misc_register(oem): %d\n", ret);
		goto err_rfs;
	}

	/*
	 * The raw-IP PS-data netdev.  Created up front; the DL ring is armed at
	 * INIT_START and drained once ONLINE.  Userspace assigns the address the
	 * CP returns from SETUP_DATA_CALL and brings it up.
	 */
	sm->ndev = alloc_netdev(sizeof(struct s5300_modem *), "rmnet%d",
				NET_NAME_ENUM, s5300_netdev_setup);
	if (!sm->ndev) {
		ret = -ENOMEM;
		goto err_oem;
	}
	*(struct s5300_modem **)netdev_priv(sm->ndev) = sm;
	SET_NETDEV_DEV(sm->ndev, dev);
	ret = register_netdev(sm->ndev);
	if (ret) {
		dev_err(dev, "register_netdev: %d\n", ret);
		free_netdev(sm->ndev);
		goto err_oem;
	}

	dev_info(dev, "ready: /dev/%s awaiting CP boot\n", sm->miscdev.name);
	return 0;

err_oem:
	misc_deregister(&sm->oem.miscdev);
err_rfs:
	misc_deregister(&sm->rfs.miscdev);
err_ctrl_port:
	wwan_remove_port(sm->ctrl_port);
err_misc:
	misc_deregister(&sm->miscdev);
err_irq:
	free_irq(pci_irq_vector(sm->pdev, 1), sm);
err_irq0:
	free_irq(pci_irq_vector(sm->pdev, 0), sm);
err_vectors:
	pci_free_irq_vectors(sm->pdev);
err_disable:
	pci_disable_device(sm->pdev);
err_pci:
	pci_dev_put(sm->pdev);
err_rc:
	put_device(sm->rc_dev);
err_fifo:
	kfifo_free(&sm->rx_fifo);
	return ret;
}

static void s5300_remove(struct platform_device *pdev)
{
	struct s5300_modem *sm = platform_get_drvdata(pdev);

	/*
	 * Free the IRQ first: it synchronises in-flight handlers, so no MSI can
	 * run s5300_drain_fmt_rxq() -> wwan_port_rx() against a port that
	 * wwan_remove_port() is about to free.
	 */
	free_irq(pci_irq_vector(sm->pdev, 1), sm);
	free_irq(pci_irq_vector(sm->pdev, 0), sm);
	unregister_netdev(sm->ndev);
	misc_deregister(&sm->oem.miscdev);
	misc_deregister(&sm->rfs.miscdev);
	wwan_remove_port(sm->ctrl_port);
	misc_deregister(&sm->miscdev);
	pci_free_irq_vectors(sm->pdev);
	pci_disable_device(sm->pdev);
	pci_dev_put(sm->pdev);
	put_device(sm->rc_dev);
	kfifo_free(&sm->rx_fifo);
}

static const struct of_device_id s5300_of_match[] = {
	{ .compatible = "samsung,s5300-modem" },
	{ },
};
MODULE_DEVICE_TABLE(of, s5300_of_match);

static struct platform_driver s5300_driver = {
	.probe	= s5300_probe,
	.remove	= s5300_remove,
	.driver	= {
		.name		= "s5300-modem",
		.of_match_table	= s5300_of_match,
	},
};
module_platform_driver(s5300_driver);

MODULE_DESCRIPTION("Samsung Exynos Modem 5300 PCIe boot transport");
MODULE_LICENSE("GPL");
