// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos Modem 5300 PCIe boot driver (Google Tensor G4 boards).
 *
 * Bring-up scope: boots the CP to its ONLINE state and answers the shared
 * memory command handshake.  No IPC ports or data path yet -- those follow
 * once the boot handshake is hardware-proven.  The protocol is ported from
 * the downstream GPL cpif driver (google-modules/radio/samsung/s5300,
 * modem_ctrl_s5100.c start_normal_boot()/link_device.c command handlers).
 *
 * Boot flow (all steps observed on downstream hardware traces):
 *  1. The RC driver has already rail-cycled the CP and trained the link; the
 *     mask ROM enumerates as 144d:a5a5 and parks in WAIT_DOORBELL.
 *  2. Move the RC's MSI target into the MSI carveout, allocate the EP's MSI
 *     vectors (the ROM reads the MSI capability address and DMA-writes its
 *     boot progress at fixed offsets above it).
 *  3. Stage the first-stage bootloader (PBL, "BOOT" TOC entry of the factory
 *     modem.bin) at IPC-carveout+0x10000, publish the address through the
 *     MSI block, ring the message doorbell, poll boot_stage to DONE (the CP
 *     verifies the image internally -- no AP-side security call).
 *  4. Bounce the link (the CP bootloader re-links; CP2AP_WAKEUP signals
 *     readiness), restore EP config, ring the link-ack doorbell.
 *  5. Answer the shared-memory command handshake: INIT_START -> PIF_INIT_DONE,
 *     PHONE_START -> (queue init) INIT_END.  The main CP image is never
 *     transferred: it persists in the modem's self-powered DRAM.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/pci.h>
#include <linux/pcie-zumapro.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/workqueue.h>

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
#define S5300_IPC_AP2CP_MSG		0x800
#define S5300_IPC_CP2AP_MSG		0x804
#define S5300_IPC_AP2CP_STATUS		0x808
#define S5300_IPC_CP2AP_STATUS		0x80c
#define S5300_IPC_HANDOVER		0x82c

#define S5300_IPC_SRINFO_OFFSET		0x400000
#define S5300_IPC_MAGIC_VALUE		0xaa	/* SHM_IPC_MAGIC, live IPC */
/*
 * SHM_BOOT_MAGIC (PROTOCOL_SIT flavor): downstream link_start_normal_boot()
 * publishes this in the magic word BEFORE the CP boots and only switches to
 * the IPC magic at PHONE_START; CP MAIN polls it on startup and stays silent
 * without it (hw-observed: boot_stage DONE, link re-trained, cp2ap forever 0).
 */
#define S5300_IPC_BOOT_MAGIC		0xbdbd

/*
 * ap2cp_united_status ds_det field (tegu DT sbi_ds_det_pos/mask); value 1 =
 * downstream get_ds_detect() default (dual-SIM detect 2 - 1).  The only
 * united-status field downstream populates before the CP boots
 * (init_control_messages()).
 */
#define S5300_STATUS_DS_DET_POS		14
#define S5300_STATUS_DS_DET		1

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

/* Doorbell values: bit 16 triggers, low bits select the mailbox index. */
#define S5300_DB_TRIGGER		BIT(16)
#define S5300_DB_MSG			(S5300_DB_TRIGGER | 0x0)
#define S5300_DB_LINK_ACK		(S5300_DB_TRIGGER | 0xe)

/* PBL lands at IPC base + this offset (round_up(raw buffer offset, 64K)). */
#define S5300_BOOT_IMG_OFFSET		0x10000

/* boot_stage / CP2AP_WAKEUP polling, downstream check_cp_status(). */
#define S5300_POLL_INTERVAL_MS		20
#define S5300_POLL_COUNT		200

/* PHONE_START handshake timeout, downstream MIF_INIT_TIMEOUT. */
#define S5300_INIT_TIMEOUT		(15 * HZ)

/*
 * Four MSI vectors, matching downstream (s51xx_pcie_request_msi_int(pdev, 4)).
 * The mask ROM only tolerates this width: requesting eight to reach vector 4
 * sets MME=3 and aborts the PBL download (boot_stage 0x1ff, err_report 0x1000).
 * The CP signals the AP on MSI message-data base 4 (m1n1 trace of a working
 * downstream boot: EP data 4, RC iMSI-RX ENABLE 0xf1 / MASK 0xffffff0c,
 * INIT_START and every MAIN-phase interrupt on bit 4).  We reproduce that by
 * having the RC reserve MSI vectors 0-3 (zumapro_pcie_reserve_msi_base) so this
 * four-vector request lands at base 4 -- our vector 0 is then hwirq 4, the
 * CP's IPC/INIT_START vector, with MME=2.
 */
#define S5300_MSI_VECTORS		4

/*
 * ap2cp handover block (downstream struct t_handover_block_info), written by
 * cbd through IOCTL_HANDOVER_BLOCK_INFO before every CP boot.  Contents
 * captured verbatim from an instrumented downstream boot of this device;
 * cpid[] holds the two IMEIs and cpsig a signature blob, so the values are
 * device-specific and must eventually come from NV storage, never a commit.
 */
struct s5300_handover_info {
	u32 version;
	u32 project_id;
	u32 revision;
	u32 major_id;
	u32 minor_id;
	u32 modem_sku;
	u32 modem_hw;
	u32 cpinfo[3];
	u32 rf_sub;
	u32 rf_config;
	u32 reserved[4];
	char cpid[2][16];
	char cpsig[65];
} __packed;

static const struct s5300_handover_info s5300_handover = {
	.version = 1,
	.project_id = 6,
	.major_id = 1,
	.modem_sku = 2,
	.reserved = { 0x0b, 0x06, 0x00, 0x6816544d },
	.cpid = { "357281833466068", "357281833466076" },
	.cpsig = "1e11532b2e65dc9bb6f2c36dfa300e61864ba7ed05cb6f8e39048de7d6c9fee2",
};

/*
 * PKTPROC info regions (cp_rmem_1, bus 0xE8000000 = CP address 0x20000000).
 * Downstream pktproc_create()/pktproc_create_ul() seed these two pages at
 * module probe, i.e. before every CP boot including fully-cold AP boots (the
 * desc/buff regions stay uninitialized pre-boot there, so only the info
 * pages matter to the CP before INIT_START).  Header and queue geometry
 * captured verbatim from the instrumented downstream boot.
 *
 * DL header bitfield: num_queues=4, desc_mode=1 (SKTBUF), irq_mode=1
 * (exclusive), max_packet_size=0x630.  UL header: num_queues=2, the other
 * fields are CP-written at runtime (garbage-tolerated pre-boot downstream).
 */
#define S5300_PKTPROC_UL_INFO_OFFSET	0x1c00000

struct s5300_pktproc_q_info {
	u32 cp_desc_pbase;
	u32 num_desc;
	u32 cp_buff_pbase;
	u32 fore_ptr;
	u32 rear_ptr;
} __packed;

static const u32 s5300_pktproc_dl_hdr = 0x63054;
static const struct s5300_pktproc_q_info s5300_pktproc_dl_q[4] = {
	{ 0x20001000, 3456, 0x20100000 },
	{ 0x2000e800, 3456, 0x207c0000 },
	{ 0x2001c000, 3456, 0x20e80000 },
	{ 0x20029800, 3456, 0x21540000 },
};

static const u32 s5300_pktproc_ul_hdr[2] = { 2, 0 };	/* num_queues, quota */
static const struct s5300_pktproc_q_info s5300_pktproc_ul_q[2] = {
	{ 0x21c01000, 1760, 0x21c90000 },
	{ 0x21c0ec00,  880, 0x21e48000 },
};

static const char * const s5300_msi_names[S5300_MSI_VECTORS] = {
	"s5300-ipc",		/* base vector (hwirq 4): the CP's INIT_START */
	"s5300-status",
	"s5300-pktproc0",
	"s5300-pktproc1",
};

struct s5300_modem {
	struct device		*dev;
	struct device		*rc_dev;
	struct pci_dev		*pdev;
	struct gpio_desc	*cp2ap_wakeup;
	struct gpio_desc	*cp2ap_active;	/* phone_active, diagnostics */
	struct gpio_desc	*cp2ap_ps_hold;	/* CP power state, diagnostics */

	phys_addr_t		ipc_phys;
	resource_size_t		ipc_size;
	void __iomem		*ipc;
	void __iomem		*ipc_wb;	/* 4K WB alias, diagnostics only */
	phys_addr_t		msi_phys;
	void __iomem		*msi;
	phys_addr_t		pktproc_phys;
	void __iomem		*pktproc;

	u32			db_bus_addr;
	void __iomem		*doorbell;

	const struct firmware	*pbl;
	struct work_struct	boot_work;
	struct completion	init_done;
	spinlock_t		lock;	/* orders ap2cp_msg word + doorbell */
	int			irq_count;
	bool			online;
};

/*
 * Program the doorbell BAR and read it back, like downstream does (it
 * computes the doorbell offset from what actually landed, and its restore
 * path runs a "BAR0 value correction" rewrite).  The ROM-phase BAR is 1M
 * so the hardware aligns the programmed address down; any base that still
 * decodes the doorbell bus address is fine.
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
 * Memory TLPs are address-routed: the root port only forwards them
 * downstream inside its type-1 memory window, and because the doorbell BAR
 * is programmed behind the PCI core's back (no child resource was ever
 * assigned) the core leaves that window closed -- every doorbell access
 * dies at the root port with the EP config-reachable but memory-dead.
 * Downstream opens it implicitly by routing its 4K BAR through
 * pci_assign_resource(); open it explicitly over the doorbell's 1M-aligned
 * range instead.  The root port is the RC's own DBI, not the flaky ROM, so
 * a single verified write suffices.
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

	/*
	 * The core re-assigns the root port's own (dummy, dw_pcie_setup_rc
	 * zeroes it) BAR0 into the bottom of the translation window at
	 * enumeration; TLPs matching an RP BAR are consumed by the port,
	 * not forwarded, so evict it from the doorbell range.
	 */
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
 * the EP's memory decode may not be settled yet, so an all-ones read-back
 * gets the command register and doorbell BAR repaired and the write retried
 * (downstream s51xx_pcie_send_doorbell_int() retries at 1 ms up to 100x;
 * keep it short here because the IPC path rings from hard-IRQ context).
 */
static void s5300_send_doorbell(struct s5300_modem *sm, u32 val)
{
	int try;
	u16 cmd;

	for (try = 0; try < 10; try++) {
		u32 rb, bar0 = 0;
		u16 c = 0;

		writel(val, sm->doorbell);
		rb = readl(sm->doorbell);
		/*
		 * Mirror downstream s51xx_pcie_send_doorbell_int's
		 * TEGU_CP_TRACE line for a byte-for-byte log diff (int_num,
		 * readback, EP cmd, BAR0).
		 */
		pci_read_config_word(sm->pdev, PCI_COMMAND, &c);
		pci_read_config_dword(sm->pdev, PCI_BASE_ADDRESS_0, &bar0);
		dev_info(sm->dev,
			 "TEGU_CP_TRACE doorbell int_num=%#x readback=%#x cmd=%#06x bar0=%#x try=%d\n",
			 val, rb, c, bar0, try);
		if (rb != 0xffffffff)
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
 * offsets and zero the status/capability words before the CP boots.  The
 * AP capability words stay zero (tegu DT: ap_capability_0/1 = 0).
 */
static void s5300_init_control_messages(struct s5300_modem *sm)
{
	int i;

	/*
	 * Downstream link_start_normal_boot(): init_legacy_link() clears the
	 * queue pointers and finishes with mem_access = 1, then the boot magic
	 * overwrites the IPC magic.  The pre-boot state the CP sees is boot
	 * magic + access 1 (hw-captured: word 0x04 reads 1 in the instrumented
	 * downstream dumps both before the PBL kick and after the re-link).
	 */
	writel(S5300_IPC_BOOT_MAGIC, sm->ipc + S5300_IPC_MAGIC);
	writel(1, sm->ipc + S5300_IPC_ACCESS);
	/*
	 * Zero the whole header, not just the FMT/RAW head/tail words at
	 * 0x08/0x18: the golden pre-boot image (instrumented downstream dump)
	 * is all-zero up to the capability words except the two offset
	 * pointers written below, while live-IPC leftovers survive a warm
	 * reboot (hw-observed stale 0x3c) and must not reach the CP.
	 */
	for (i = S5300_IPC_Q_HEAD_TAIL; i < S5300_IPC_CAP_BASE; i += 4)
		writel(0, sm->ipc + i);

	writel(S5300_IPC_SRINFO_OFFSET, sm->ipc + S5300_IPC_SRINFO_OFS_PTR);
	writel(S5300_IPC_CAP_BASE, sm->ipc + S5300_IPC_CAP_OFS_PTR);
	/*
	 * Message words included (downstream init_ctrl_msg() in power_on_cp):
	 * stale VALID bits from a previous boot must not be readable once the
	 * MSI handler is live.
	 */
	writel(0, sm->ipc + S5300_IPC_AP2CP_MSG);
	writel(0, sm->ipc + S5300_IPC_CP2AP_MSG);
	writel(S5300_STATUS_DS_DET << S5300_STATUS_DS_DET_POS,
	       sm->ipc + S5300_IPC_AP2CP_STATUS);
	writel(0, sm->ipc + S5300_IPC_CP2AP_STATUS);
	for (i = 0; i < S5300_IPC_CAP_WORDS; i++)
		writel(0, sm->ipc + S5300_IPC_CAP_BASE + 4 * i);

	memcpy_toio(sm->ipc + S5300_IPC_HANDOVER, &s5300_handover,
		    sizeof(s5300_handover));
}

static void s5300_init_pktproc_info(struct s5300_modem *sm)
{
	memcpy_toio(sm->pktproc, &s5300_pktproc_dl_hdr,
		    sizeof(s5300_pktproc_dl_hdr));
	memcpy_toio(sm->pktproc + sizeof(s5300_pktproc_dl_hdr),
		    s5300_pktproc_dl_q, sizeof(s5300_pktproc_dl_q));

	memcpy_toio(sm->pktproc + S5300_PKTPROC_UL_INFO_OFFSET,
		    s5300_pktproc_ul_hdr, sizeof(s5300_pktproc_ul_hdr));
	memcpy_toio(sm->pktproc + S5300_PKTPROC_UL_INFO_OFFSET +
		    sizeof(s5300_pktproc_ul_hdr),
		    s5300_pktproc_ul_q, sizeof(s5300_pktproc_ul_q));
}

static void s5300_disable_link_pm(struct pci_dev *pdev)
{
	struct pci_dev *bridge = pci_upstream_bridge(pdev);
	int l1ss;

	pcie_capability_clear_word(pdev, PCI_EXP_LNKCTL,
				   PCI_EXP_LNKCTL_ASPMC |
				   PCI_EXP_LNKCTL_CLKREQ_EN);
	l1ss = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_L1SS);
	if (l1ss)
		pci_clear_and_set_config_dword(pdev, l1ss + PCI_L1SS_CTL1,
					       PCI_L1SS_CTL1_L1SS_MASK, 0);

	if (!bridge)
		return;

	pcie_capability_clear_word(bridge, PCI_EXP_LNKCTL,
				   PCI_EXP_LNKCTL_ASPMC |
				   PCI_EXP_LNKCTL_CLKREQ_EN);
	l1ss = pci_find_ext_capability(bridge, PCI_EXT_CAP_ID_L1SS);
	if (l1ss)
		pci_clear_and_set_config_dword(bridge, l1ss + PCI_L1SS_CTL1,
					       PCI_L1SS_CTL1_L1SS_MASK, 0);
}

static void s5300_unmask_msi_vectors(struct s5300_modem *sm)
{
	u32 mask;
	u16 flags;
	int mask_off;

	if (!sm->pdev->msi_cap)
		return;

	pci_read_config_word(sm->pdev, sm->pdev->msi_cap + PCI_MSI_FLAGS,
			     &flags);
	if (!(flags & PCI_MSI_FLAGS_MASKBIT))
		return;

	mask_off = (flags & PCI_MSI_FLAGS_64BIT) ? PCI_MSI_MASK_64 :
		   PCI_MSI_MASK_32;
	pci_read_config_dword(sm->pdev, sm->pdev->msi_cap + mask_off, &mask);
	dev_info(sm->dev, "MSI mask before unmasking vectors 0-%d: %#010x\n",
		 S5300_MSI_VECTORS - 1, mask);
	mask &= ~GENMASK(S5300_MSI_VECTORS - 1, 0);
	pci_write_config_dword(sm->pdev, sm->pdev->msi_cap + mask_off, mask);
	pci_read_config_dword(sm->pdev, sm->pdev->msi_cap + mask_off, &mask);
	dev_info(sm->dev, "MSI mask after unmasking vectors 0-%d: %#010x\n",
		 S5300_MSI_VECTORS - 1, mask);
}

/*
 * Downstream init_legacy_link(), run from the PHONE_START handler: clear the
 * queue pointers, then advertise the magic and access-enable words.
 */
static void s5300_init_ipc_queues(struct s5300_modem *sm)
{
	u32 magic, access;
	int i;

	writel(0, sm->ipc + S5300_IPC_MAGIC);
	writel(0, sm->ipc + S5300_IPC_ACCESS);
	for (i = 0; i < S5300_IPC_Q_WORDS; i++)
		writel(0, sm->ipc + S5300_IPC_Q_HEAD_TAIL + 4 * i);
	writel(S5300_IPC_MAGIC_VALUE, sm->ipc + S5300_IPC_MAGIC);
	writel(1, sm->ipc + S5300_IPC_ACCESS);

	magic = readl(sm->ipc + S5300_IPC_MAGIC);
	access = readl(sm->ipc + S5300_IPC_ACCESS);
	if (magic != S5300_IPC_MAGIC_VALUE || access != 1)
		dev_err(sm->dev, "IPC init readback failed: magic %#x access %u\n",
			magic, access);
}

static irqreturn_t s5300_irq_handler(int irq, void *data)
{
	struct s5300_modem *sm = data;
	int vector;
	u32 val, status, cmd;

	val = readl(sm->ipc + S5300_IPC_CP2AP_MSG);
	status = readl(sm->ipc + S5300_IPC_CP2AP_STATUS);
	for (vector = 0; vector < sm->irq_count; vector++)
		if (pci_irq_vector(sm->pdev, vector) == irq)
			break;
	if (vector == sm->irq_count)
		vector = -1;

	/* Bring-up diagnostic: date every MSI and what the mailbox held. */
	dev_info(sm->dev, "MSI vector %d irq %d (cp2ap %#x status %#x)\n",
		 vector, irq, val, status);
	if (!(val & S5300_INT_VALID))
		return IRQ_HANDLED;

	if (!(val & S5300_CMD_VALID)) {
		/* Plain data notification; no consumers yet. */
		dev_dbg(sm->dev, "IPC data interrupt %#x\n", val);
		return IRQ_HANDLED;
	}

	cmd = val & S5300_CMD_MASK;
	switch (cmd) {
	case S5300_CMD_INIT_START:
		dev_info(sm->dev, "CP INIT_START\n");
		s5300_send_ipc_irq(sm, S5300_CMD(S5300_CMD_PIF_INIT_DONE));
		break;
	case S5300_CMD_PHONE_START:
		dev_info(sm->dev, "CP PHONE_START\n");
		if (!sm->online) {
			s5300_init_ipc_queues(sm);
			sm->online = true;
		}
		/* Re-entrant PHONE_START just gets the INIT_END again. */
		s5300_send_ipc_irq(sm, S5300_CMD(S5300_CMD_INIT_END));
		complete_all(&sm->init_done);
		break;
	case S5300_CMD_CRASH_RESET:
	case S5300_CMD_CRASH_EXIT:
		dev_err(sm->dev, "CP crash notification %#x (err_report %#x)\n",
			cmd, readl(sm->msi + S5300_MSI_ERR_REPORT));
		break;
	default:
		dev_warn(sm->dev, "unknown CP command %#x\n", cmd);
		break;
	}

	return IRQ_HANDLED;
}

/*
 * Force BAR0 to the doorbell page the downstream stack uses.  The write is
 * deliberately not 1M-aligned: whatever BAR0 size the current CP boot stage
 * exposes, the hardware aligns the value down to its own size, and the
 * doorbell register always decodes at bus address 0x14e60000 (downstream
 * pci_db_addr; its driver likewise programs this value into BAR0 and rings
 * that bus address through both boot phases).  Written behind the PCI core's
 * back (the core saw unassignable ROM BARs anyway); the modem never runs
 * with core-managed BARs downstream either.
 */
static int s5300_setup_doorbell(struct s5300_modem *sm)
{
	struct pci_bus_region region;
	/*
	 * pcibios_bus_to_resource() matches host-bridge windows by the
	 * resource type of the passed-in res, so it must be pre-typed MEM --
	 * zero flags match no window and the bus address comes back
	 * untranslated (seen on hardware as "cpu [??? 0x14e60000...]").
	 */
	struct resource res = { .flags = IORESOURCE_MEM };
	int i, ret;

	/*
	 * The PCI core could not place the mask ROM's six 1M BARs in the
	 * small CH0 window; drop them from resource management entirely so
	 * pci_enable_device() has nothing unclaimed to trip over, then
	 * program BAR0 directly (downstream s51xx_pcie_probe() does the
	 * same).
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

/*
 * Undo the pre-bounce D3hot/disable on a failure path, leaving the endpoint
 * enabled in D0.  Between the disable at the bounce point and the re-enable on
 * the far side the device is powered down and off the enable_cnt; an early
 * return there would leave s5300_remove()'s pci_disable_device() to underflow
 * the count.  Best-effort -- these are already error returns.
 */
static void s5300_abort_bounce(struct pci_dev *pdev)
{
	int ret;

	pci_set_power_state(pdev, PCI_D0);
	ret = pci_enable_device(pdev);
	if (ret)
		dev_warn(&pdev->dev,
			 "re-enable after aborted bounce failed: %d\n", ret);
}

/*
 * CP liveness lines (alive bank, readable any time): phone_active goes high
 * once CP MAIN runs and drops on a CP crash; ps_hold tracks the CP power
 * state.  Downstream only attaches interrupts to them after ONLINE, but the
 * levels discriminate "MAIN never started" from "MAIN started and died"
 * during the INIT_START wait.
 */
static void s5300_log_cp_lines(struct s5300_modem *sm, const char *tag)
{
	dev_info(sm->dev, "%s: cp2ap wakeup %d active %d ps_hold %d\n", tag,
		 gpiod_get_value_cansleep(sm->cp2ap_wakeup),
		 sm->cp2ap_active ?
			gpiod_get_value_cansleep(sm->cp2ap_active) : -1,
		 sm->cp2ap_ps_hold ?
			gpiod_get_value_cansleep(sm->cp2ap_ps_hold) : -1);
}

/*
 * Dual-view diagnostic: the IPC header words the CP consults at startup, read
 * through the WC mapping (the DRAM view -- what a non-snooping CP read
 * returns) and through the WB alias (the coherent-domain view -- where an
 * IO-coherent CP write would land).  Divergence tells us on which side of the
 * cache hierarchy the data is hiding.
 */
static void s5300_dump_views(struct s5300_modem *sm, const char *tag)
{
	dev_info(sm->dev,
		 "%s WC: magic %#x acc %#x a2c %#x c2a %#x ast %#x cst %#x ho %#x\n",
		 tag,
		 readl(sm->ipc + S5300_IPC_MAGIC),
		 readl(sm->ipc + S5300_IPC_ACCESS),
		 readl(sm->ipc + S5300_IPC_AP2CP_MSG),
		 readl(sm->ipc + S5300_IPC_CP2AP_MSG),
		 readl(sm->ipc + S5300_IPC_AP2CP_STATUS),
		 readl(sm->ipc + S5300_IPC_CP2AP_STATUS),
		 readl(sm->ipc + S5300_IPC_HANDOVER));
	if (!sm->ipc_wb)
		return;
	dev_info(sm->dev,
		 "%s WB: magic %#x acc %#x a2c %#x c2a %#x ast %#x cst %#x ho %#x\n",
		 tag,
		 readl(sm->ipc_wb + S5300_IPC_MAGIC),
		 readl(sm->ipc_wb + S5300_IPC_ACCESS),
		 readl(sm->ipc_wb + S5300_IPC_AP2CP_MSG),
		 readl(sm->ipc_wb + S5300_IPC_CP2AP_MSG),
		 readl(sm->ipc_wb + S5300_IPC_AP2CP_STATUS),
		 readl(sm->ipc_wb + S5300_IPC_CP2AP_STATUS),
		 readl(sm->ipc_wb + S5300_IPC_HANDOVER));
}

static void s5300_boot_work(struct work_struct *work)
{
	struct s5300_modem *sm = container_of(work, struct s5300_modem,
					      boot_work);
	struct pci_dev *pdev = sm->pdev;
	int ret, sec;
	/*
	 * Heavy bounce (full PERST + PHY re-cal + RC re-setup): the only cycle
	 * that reliably retrains the post-PBL x2/Gen3 link on this hardware.
	 * The light experiments proved a bare-PERST/PHY-alive relink cannot
	 * train past detect, so this is the shippable path.  New this cycle:
	 * modem_link_down snapshots the RC iMSI-RX enable/mask and modem_link_up
	 * restores them after dw_pcie_setup_rc, so the CP's post-link-ack notify
	 * MSI (message 4) is no longer dropped by a core-reset-zeroed ENABLE.
	 */
	bool light = false;

	/*
	 * Diagnostic: dump the control block before touching it.  AP DRAM
	 * often retains across a warm reboot, so this can recover what the
	 * previous downstream boot wrote (notably the ap2cp handover block
	 * at 0x82c, whose full contents only cbd knows).
	 */
	for (ret = 0x7f0; ret < 0x860; ret += 16)
		dev_info(sm->dev, "shmem %#05x: %08x %08x %08x %08x\n", ret,
			 readl(sm->ipc + ret), readl(sm->ipc + ret + 4),
			 readl(sm->ipc + ret + 8), readl(sm->ipc + ret + 12));

	/* Publish the PBL location through the MSI block. */
	writel(0, sm->msi + S5300_MSI_BOOT_STAGE);
	s5300_init_control_messages(sm);
	s5300_init_pktproc_info(sm);
	memcpy_toio(sm->ipc + S5300_BOOT_IMG_OFFSET, sm->pbl->data,
		    sm->pbl->size);
	writel(lower_32_bits(sm->ipc_phys + S5300_BOOT_IMG_OFFSET),
	       sm->msi + S5300_MSI_IMG_ADDR_LO);
	writel(upper_32_bits(sm->ipc_phys + S5300_BOOT_IMG_OFFSET),
	       sm->msi + S5300_MSI_IMG_ADDR_HI);
	writel(sm->pbl->size, sm->msi + S5300_MSI_IMG_SIZE);

	release_firmware(sm->pbl);
	sm->pbl = NULL;

	/* The ROM reads these on the doorbell; make sure the writes stuck. */
	s5300_verify_msi_target(sm);

	/*
	 * Config state (forced BAR0, MSI capability) must survive the bounce.
	 * Downstream also snapshots here (first_save_s51xx_status); the
	 * re-save at the bounce point below (with bus-master cleared, matching
	 * s51xx_pcie_save_state()) supersedes this as the restored state.
	 */
	pci_save_state(pdev);

	dev_info(sm->dev, "starting first-stage download (%#x bytes at %pap+%#x)\n",
		 readl(sm->msi + S5300_MSI_IMG_SIZE), &sm->ipc_phys,
		 S5300_BOOT_IMG_OFFSET);
	/*
	 * The image/descriptor stores above target write-combined mappings;
	 * the writel() barrier inside send_doorbell orders them ahead of the
	 * doorbell trigger.
	 */
	s5300_dump_views(sm, "pre-doorbell");
	s5300_send_doorbell(sm, S5300_DB_MSG);

	ret = s5300_poll_boot_stage(sm);
	if (ret)
		return;
	dev_info(sm->dev, "first-stage bootloader up, bouncing the link\n");
	s5300_log_cp_lines(sm, "post-pbl");

	/*
	 * Downstream settles after boot_stage DONE before the link drop:
	 * s5100_poweroff_pcie() always sleeps 30 ms, and check_cp_status()
	 * adds a 20 ms guard when DONE hit on its very first poll.  50 ms
	 * unconditionally is a superset of that.
	 */
	msleep(50);

	zumapro_pcie_modem_set_light(sm->rc_dev, light);

	/*
	 * D3hot the endpoint before the bounce, mirroring the save half of
	 * s51xx_pcie_save_state() (clear bus-master, re-save the BME-cleared
	 * config, disable, D3hot), paired with the D0/restore/enable/
	 * set_master block after link-up.  Both bounce modes assert PERST,
	 * which resets EP config, so both need this.
	 */
	pci_clear_master(pdev);
	pci_save_state(pdev);
	pci_disable_device(pdev);
	pci_wake_from_d3(pdev, false);
	if (pci_set_power_state(pdev, PCI_D3hot))
		dev_warn(sm->dev, "could not put endpoint in D3hot before bounce\n");
	if (pdev->pm_cap) {
		u16 pmcsr = 0;

		pci_read_config_word(pdev, pdev->pm_cap + PCI_PM_CTRL, &pmcsr);
		dev_info(sm->dev, "EP PMCSR after D3hot: %#06x\n", pmcsr);
	}

	/*
	 * The CP bootloader expects a link drop and retrain before the ack
	 * doorbell (downstream start_normal_boot()); CP2AP_WAKEUP signals it
	 * is ready to re-link.
	 */
	ret = zumapro_pcie_modem_link_down(sm->rc_dev);
	if (ret) {
		s5300_abort_bounce(pdev);
		return;
	}
	ret = s5300_poll_cp_wakeup(sm);
	if (ret) {
		s5300_abort_bounce(pdev);
		return;
	}
	ret = zumapro_pcie_modem_link_up(sm->rc_dev);
	if (ret) {
		dev_err(sm->dev, "link retrain after bounce failed: %d\n", ret);
		s5300_abort_bounce(pdev);
		return;
	}

	/*
	 * Restore half of s51xx_pcie_restore_state(): D0, reload the
	 * BME-cleared config, re-enable, set bus-master.
	 */
	if (pci_set_power_state(pdev, PCI_D0))
		dev_warn(sm->dev,
			 "could not bring endpoint back to D0 after bounce\n");
	pci_restore_state(pdev);
	ret = pci_enable_device(pdev);
	if (ret)
		dev_warn(sm->dev, "pci_enable_device after bounce failed: %d\n",
			 ret);
	pci_set_master(pdev);
	/*
	 * Downstream does not trust restore for the forced BAR: it re-reads
	 * and rewrites it after every link-up (s51xx_pcie_restore_state()).
	 * Re-verify the BAR, bridge window and MSI target on the fresh link.
	 */
	s5300_program_doorbell_bar(sm);
	s5300_open_bridge_window(sm);
	s5300_verify_msi_target(sm);
	/*
	 * Skip the L1SS enable in light mode: its L1-exit dance perturbs the
	 * link, and downstream only enables L1.2 once the CP is ONLINE anyway.
	 */
	if (!light) {
		ret = zumapro_pcie_modem_enable_l1ss(sm->rc_dev, pdev);
		if (ret)
			dev_warn(sm->dev,
				 "post-bounce modem L1SS enable failed: %d\n", ret);
	}

	s5300_dump_views(sm, "pre-link-ack");
	s5300_log_cp_lines(sm, "pre-link-ack");
	zumapro_pcie_modem_msi_status(sm->rc_dev);
	s5300_send_doorbell(sm, S5300_DB_LINK_ACK);

	/*
	 * Downstream INIT_START arrives ~2 s after the link-ack.  While
	 * waiting, log the CP liveness lines once a second and re-ring the
	 * link-ack a few times: the golden m1n1 boot carried three link-acks
	 * (retry cycles), so a repeat is protocol-safe if BL1's mailbox
	 * missed our single edge during its own link recovery.
	 */
	for (sec = 0; sec < S5300_INIT_TIMEOUT / HZ; sec++) {
		if (wait_for_completion_timeout(&sm->init_done, HZ))
			break;
		s5300_log_cp_lines(sm, "waiting");
		zumapro_pcie_modem_msi_status(sm->rc_dev);
		if (sec == 2 || sec == 5 || sec == 9) {
			dev_info(sm->dev, "re-ringing link-ack (t+%ds)\n",
				 sec + 1);
			s5300_send_doorbell(sm, S5300_DB_LINK_ACK);
		}
	}

	if (sec == S5300_INIT_TIMEOUT / HZ) {
		int off, hits = 0;

		/*
		 * cp2ap_united_status discriminates the silence: CP MAIN sets
		 * its status bits there early, so nonzero = MAIN runs but the
		 * handshake is gated; zero = the CP never got past BL1.
		 */
		dev_err(sm->dev, "CP handshake timed out (cp2ap %#x status %#x boot_stage %#x err %#x)\n",
			readl(sm->ipc + S5300_IPC_CP2AP_MSG),
			readl(sm->ipc + S5300_IPC_CP2AP_STATUS),
			readl(sm->msi + S5300_MSI_BOOT_STAGE),
			readl(sm->msi + S5300_MSI_ERR_REPORT));
		s5300_dump_views(sm, "timeout");
		/*
		 * Diagnostic: any CP write ANYWHERE in the first 4K shows
		 * whether the CP spoke at an offset our layout model missed.
		 */
		for (off = 0; off < SZ_4K && hits < 32; off += 4) {
			u32 val = readl(sm->ipc + off);

			if (val)
				dev_err(sm->dev, "  shmem %#05x = %#010x\n",
					off, val), hits++;
		}

		/*
		 * Post-bounce EP config dump.  The PBL disassembly
		 * (research/modem-pbl-re.md) proved BL1 polls a CP-internal
		 * ELBI bit set by the doorbell write and never re-arms across a
		 * link-down; the leading fix candidate is that the ROM armed the
		 * inbound path through an extended-config VSEC our pci_restore
		 * does not replay.  Diff this against the golden post-restore EP
		 * config (out/ubports-modem-pcie-hsi1-mmio-trace.log, offsets
		 * 0x40ffeXXX): 0x104=0, 0x108=0x400000, 0x10c=0x462030,
		 * 0x110=0, 0x114=0xe000, 0x118=0xa0, 0x004=0x0506, 0x052=0x01ab.
		 */
		for (off = 0; off <= 0x11c; off += 0x10) {
			u32 a = 0, b = 0, c = 0, d = 0;

			pci_read_config_dword(pdev, off, &a);
			pci_read_config_dword(pdev, off + 4, &b);
			pci_read_config_dword(pdev, off + 8, &c);
			pci_read_config_dword(pdev, off + 0xc, &d);
			dev_err(sm->dev, "  epcfg %#05x: %08x %08x %08x %08x\n",
				off, a, b, c, d);
		}

		/*
		 * Post-mortem doorbell-delivery probe.  A bare msg re-ring
		 * showed nothing on hardware, but that is ambiguous: BL1 is
		 * past the download stage, so its msg handler may be idle even
		 * if the interrupt still arrives.  Disambiguate by first
		 * zeroing the BL1-owned boot_stage word (downstream
		 * clear_boot_stage() does exactly this at power-on): the PBL is
		 * still staged in the IPC region and its descriptor in the MSI
		 * block survived the bounce (AP DRAM is untouched), so a LIVE
		 * doorbell makes BL1 re-DMA and climb boot_stage back toward
		 * 0x3fff; a DEAD one leaves it pinned at 0.  That splits
		 * "doorbell IRQ dead post-PERST" from "handler idle post-DONE".
		 */
		writel(0, sm->msi + S5300_MSI_BOOT_STAGE);
		dev_err(sm->dev,
			"post-mortem: boot_stage cleared to %#x, re-ringing msg doorbell\n",
			readl(sm->msi + S5300_MSI_BOOT_STAGE));
		s5300_send_doorbell(sm, S5300_DB_MSG);
		/*
		 * Log the raw doorbell read-back.  Per the PBL disassembly the
		 * CP write-1-clears its ELBI trigger bit once serviced; if this
		 * register is that view, a self-clearing read-back (not the
		 * written 0x10000) would mean BL1 is polling and alive.  A plain
		 * latch just echoes the write, so this only informs, not proves.
		 */
		msleep(20);
		dev_err(sm->dev, "post-mortem: doorbell reads back %#010x\n",
			readl(sm->doorbell));
		for (sec = 0; sec < 3; sec++) {
			msleep(1000);
			dev_err(sm->dev,
				"post-mortem +%ds: boot_stage %#x err %#x cp2ap %#x\n",
				sec + 1,
				readl(sm->msi + S5300_MSI_BOOT_STAGE),
				readl(sm->msi + S5300_MSI_ERR_REPORT),
				readl(sm->ipc + S5300_IPC_CP2AP_MSG));
		}
		return;
	}

	dev_info(sm->dev, "CP is ONLINE\n");
}

static int s5300_map_region(struct s5300_modem *sm, const char *name,
			    bool cached, phys_addr_t *phys,
			    resource_size_t *size, void __iomem **map)
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
	 * Downstream cp_shmem maps the IPC region cached (region,cached=1,
	 * plain phys_to_virt) and the MSI region non-cached, so mirror that
	 * split.  The rationale is that the CP's inbound IPC writes are
	 * IO-coherent and land in the CPU-visible cache hierarchy (a non-cached
	 * mapping would read stale DRAM), while the mask ROM's boot_stage
	 * writes in the MSI region are non-coherent and need the NC view.
	 * Whether mainline CH0 inbound traffic is actually IO-coherent is
	 * UNVERIFIED: our RC does no IOCC programming yet (downstream enables it
	 * via DBI 0x8E8 plus sysreg shareability with use-cache-coherency=true),
	 * and no INIT_START has ever reached mainline to confirm the cached view.
	 */
	if (cached) {
		*map = (void __iomem __force *)
			devm_memremap(sm->dev, rmem->base, rmem->size,
				      MEMREMAP_WB);
		if (IS_ERR(*map))
			return PTR_ERR((void *)*map);
	} else {
		*map = devm_ioremap_wc(sm->dev, rmem->base, rmem->size);
	}
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
	const char *fw_name;
	u16 cmd;
	int i, ret;

	sm = devm_kzalloc(dev, sizeof(*sm), GFP_KERNEL);
	if (!sm)
		return -ENOMEM;

	sm->dev = dev;
	spin_lock_init(&sm->lock);
	init_completion(&sm->init_done);
	INIT_WORK(&sm->boot_work, s5300_boot_work);
	platform_set_drvdata(pdev, sm);

	rc_node = of_parse_phandle(dev->of_node, "google,pcie", 0);
	if (!rc_node)
		return dev_err_probe(dev, -EINVAL, "missing google,pcie\n");
	rc_pdev = of_find_device_by_node(rc_node);
	of_node_put(rc_node);
	if (!rc_pdev)
		return -EPROBE_DEFER;
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

	/* Liveness diagnostics only: warn and carry on without them. */
	sm->cp2ap_active = devm_gpiod_get_optional(dev, "cp2ap-active",
						   GPIOD_IN);
	if (IS_ERR(sm->cp2ap_active)) {
		dev_warn(dev, "no CP2AP_CP_ACTIVE line: %ld\n",
			 PTR_ERR(sm->cp2ap_active));
		sm->cp2ap_active = NULL;
	}
	sm->cp2ap_ps_hold = devm_gpiod_get_optional(dev, "cp2ap-ps-hold",
						    GPIOD_IN);
	if (IS_ERR(sm->cp2ap_ps_hold)) {
		dev_warn(dev, "no CP2AP_PS_HOLD line: %ld\n",
			 PTR_ERR(sm->cp2ap_ps_hold));
		sm->cp2ap_ps_hold = NULL;
	}

	/*
	 * EXPERIMENT (cache matrix, 2026-07-04): map IPC and PKTPROC
	 * write-combined for this boot.  Downstream maps them cached, but it
	 * also programs the RC for IO coherency (set_iocc: DBI 0x8E8 + sysreg
	 * shareability), which our RC does not do yet -- and the cached
	 * mapping has only ever been tested together with the base-4 MSI fix,
	 * while every visible-writes (WC) boot ran under the old base-0 MSI
	 * misrouting.  WC guarantees the boot magic/handover block reach DRAM
	 * before the CP can look.  Revert to cached + RC IOCC once the
	 * INIT_START blocker is found.
	 */
	ret = s5300_map_region(sm, "ipc", false, &sm->ipc_phys, &sm->ipc_size,
			       &sm->ipc);
	if (ret) {
		dev_err_probe(dev, ret, "failed to map IPC carveout\n");
		goto err_rc;
	}
	/*
	 * Cacheable 4K alias of the IPC header, read-only, for the dual-view
	 * diagnostics: shows what the coherent domain holds next to the DRAM
	 * view above (mismatched-attribute alias, never written through).
	 */
	sm->ipc_wb = (void __iomem __force *)
		devm_memremap(dev, sm->ipc_phys, SZ_4K, MEMREMAP_WB);
	if (IS_ERR(sm->ipc_wb))
		sm->ipc_wb = NULL;
	ret = s5300_map_region(sm, "msi", false, &sm->msi_phys, NULL,
			       &sm->msi);
	if (ret) {
		dev_err_probe(dev, ret, "failed to map MSI carveout\n");
		goto err_rc;
	}

	/* Downstream cp_shmem: PKTPROC cached=1; WC here, see the IPC note. */
	ret = s5300_map_region(sm, "pktproc", false, &sm->pktproc_phys, NULL,
			       &sm->pktproc);
	if (ret) {
		dev_err_probe(dev, ret, "failed to map PKTPROC carveout\n");
		goto err_rc;
	}

	if (of_property_read_string(dev->of_node, "firmware-name", &fw_name))
		fw_name = "tegu/cp_pbl.bin";
	ret = request_firmware(&sm->pbl, fw_name, dev);
	if (ret) {
		dev_err_probe(dev, ret, "failed to load %s\n", fw_name);
		goto err_rc;
	}
	if (sm->pbl->size > sm->ipc_size - S5300_BOOT_IMG_OFFSET) {
		ret = dev_err_probe(dev, -EFBIG, "PBL too large\n");
		goto err_fw;
	}

	/*
	 * The zumapro root port carries the same 144d:a5a5 ID as the modem
	 * endpoint, and it registers first -- a bare first-match lookup
	 * returns the root port.  Match the endpoint by port type.
	 */
	sm->pdev = NULL;
	while ((sm->pdev = pci_get_device(S5300_PCI_VENDOR_ID,
					  S5300_PCI_DEVICE_ID, sm->pdev))) {
		if (pci_pcie_type(sm->pdev) == PCI_EXP_TYPE_ENDPOINT)
			break;
	}
	if (!sm->pdev) {
		ret = dev_err_probe(dev, -ENODEV,
				    "CP endpoint not enumerated\n");
		goto err_fw;
	}
	dev_info(dev, "CP endpoint %s\n", pci_name(sm->pdev));

	/*
	 * Keep link PM off while talking to the ROM, but do not call
	 * pci_disable_link_state(): this experiment enables the downstream-style
	 * CP L1SS/ASPM state after the BL1 bounce, before the link-ack doorbell.
	 */
	s5300_disable_link_pm(sm->pdev);

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
	 * The core caches the MSI capability offset at enumeration time; if
	 * the mask ROM exposed its capability list late, re-look it up so
	 * vector allocation does not fail on a stale zero.
	 */
	if (!sm->pdev->msi_cap) {
		sm->pdev->msi_cap = pci_find_capability(sm->pdev,
							PCI_CAP_ID_MSI);
		dev_warn(dev, "MSI capability re-lookup: %#x\n",
			 sm->pdev->msi_cap);
	}

	/*
	 * Dormant belt-and-braces: if the walk still comes up empty, probe
	 * the DW-default offset downstream hardcodes (print_msi_register())
	 * and install it directly, logging what the raw reads see.
	 */
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
	 * Reserve RC vectors 0-3 so this four-vector request lands at base 4,
	 * where the CP fires INIT_START and every MAIN-phase interrupt (see
	 * S5300_MSI_VECTORS).  Leaving the allocation at base 0, as before, put
	 * the CP's vector-4 interrupt outside our range -- the most likely
	 * reason the CP looked silent after the bounce.
	 */
	ret = zumapro_pcie_reserve_msi_base(sm->rc_dev, S5300_MSI_VECTORS);
	if (ret)
		dev_warn(dev, "MSI base reservation failed: %d (CP vector 4 may miss)\n",
			 ret);

	ret = pci_alloc_irq_vectors(sm->pdev, S5300_MSI_VECTORS,
				    S5300_MSI_VECTORS, PCI_IRQ_MSI);
	if (ret < 0) {
		dev_err(dev, "MSI alloc: %d (power state %d, msi_cap %#x)\n",
			ret, sm->pdev->current_state, sm->pdev->msi_cap);
		goto err_disable;
	}
	sm->irq_count = ret;
	dev_info(dev, "%d MSI vector(s)\n", sm->irq_count);

	/*
	 * The CP's vector-4 interrupt only reaches us if this allocation landed
	 * at base 4, i.e. the EP message-data base reads 4 and the control word
	 * shows MME=2 (0x40).  Log both so the bring-up test can confirm the
	 * reservation took; MME=3 (0x60) means we regressed to the ROM-breaking
	 * eight-vector width.
	 */
	if (sm->pdev->msi_cap) {
		u16 ctrl, data = 0;
		int data_off;

		pci_read_config_word(sm->pdev,
				     sm->pdev->msi_cap + PCI_MSI_FLAGS, &ctrl);
		data_off = (ctrl & PCI_MSI_FLAGS_64BIT) ? PCI_MSI_DATA_64 :
							  PCI_MSI_DATA_32;
		pci_read_config_word(sm->pdev, sm->pdev->msi_cap + data_off,
				     &data);
		dev_info(dev,
			 "EP MSI ctrl %#06x data base %#06x (CP fires vector 4; want base 4, MME=2)\n",
			 ctrl, data);
	}

	for (i = 0; i < sm->irq_count; i++) {
		ret = request_irq(pci_irq_vector(sm->pdev, i),
				  s5300_irq_handler, 0, s5300_msi_names[i], sm);
		if (ret)
			goto err_irqs;
		dev_info(dev, "requested MSI vector %d irq %d (%s)\n", i,
			 pci_irq_vector(sm->pdev, i), s5300_msi_names[i]);
	}
	s5300_unmask_msi_vectors(sm);

	schedule_work(&sm->boot_work);

	return 0;

err_irqs:
	while (i--)
		free_irq(pci_irq_vector(sm->pdev, i), sm);
	pci_free_irq_vectors(sm->pdev);
err_disable:
	pci_disable_device(sm->pdev);
err_pci:
	pci_dev_put(sm->pdev);
err_fw:
	release_firmware(sm->pbl);
err_rc:
	put_device(sm->rc_dev);
	return ret;
}

static void s5300_remove(struct platform_device *pdev)
{
	struct s5300_modem *sm = platform_get_drvdata(pdev);
	int i;

	cancel_work_sync(&sm->boot_work);
	release_firmware(sm->pbl);
	for (i = 0; i < sm->irq_count; i++)
		free_irq(pci_irq_vector(sm->pdev, i), sm);
	pci_free_irq_vectors(sm->pdev);
	pci_disable_device(sm->pdev);
	pci_dev_put(sm->pdev);
	put_device(sm->rc_dev);
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

MODULE_DESCRIPTION("Samsung Exynos Modem 5300 PCIe boot driver");
MODULE_LICENSE("GPL");
