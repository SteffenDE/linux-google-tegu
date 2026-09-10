/* SPDX-License-Identifier: GPL-2.0 */
/*
 * tegu-pmtrace - a bounded binary record of the operations the AP issues on
 * the way into and out of a system power mode.
 *
 * This is the mainline half of a pair.  The downstream vendor kernel carries
 * the same recorder, with the same record layout, event kinds, phases and
 * sequence labels, installed by scripts/downstream-kernel-patches in the
 * bring-up repo -- so a dump from either side decodes with the same tool and
 * the two are directly diffable.  Keep the numbering below in step with it.
 *
 * Why it is built this way:
 *
 *  - The storage is static and built in, so the buffers are valid from the
 *    first instruction of the first recorder.  There is no initialisation call
 *    a driver's init path could run ahead of, and unlike the downstream build
 *    there is no module load whose records are missed before it.
 *  - A slot is claimed with one atomic increment and then written by its
 *    claimer.  Nothing allocates, sleeps, takes a lock or touches hardware,
 *    so this is safe in the atomic and noirq paths where the interesting
 *    operations happen.
 *  - Every value stored is one the caller already had.  No readback is added
 *    for the trace, in particular none from a domain that may be powered off.
 *  - Boot records go to their own linear buffer, so runtime traffic (ACPM
 *    DVFS and thermal in particular) cannot erase the boot-time setup that is
 *    the point of the exercise.
 */
#ifndef __LINUX_SOC_SAMSUNG_TEGU_PMTRACE_H
#define __LINUX_SOC_SAMSUNG_TEGU_PMTRACE_H

#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/compiler.h>
#include <linux/types.h>

/* Event kinds. The decoder keys off these, so their values are ABI. */
enum tegu_pmt_kind {
	TEGU_PMT_MARK		= 1,	/* free-form marker */
	TEGU_PMT_SEC_WRITE	= 2,	/* set_priv_reg()  addr, val -> ret */
	TEGU_PMT_SEC_RMW	= 3,	/* rmw_priv_reg()  addr, mask, val -> ret */
	TEGU_PMT_SEC_READ	= 4,	/* get_priv_reg()  addr -> ret */
	TEGU_PMT_MMIO_WRITE	= 5,	/* pmucal write that is not a PMU address */
	TEGU_PMT_SEQ_BEGIN	= 6,	/* pmucal sequence entered */
	TEGU_PMT_SEQ_END	= 7,	/* pmucal sequence returned */
	TEGU_PMT_SEQ_STEP	= 8,	/* one pmucal sequence element */
	TEGU_PMT_SEQ_ERR	= 9,	/* a sequence step timed out */
	TEGU_PMT_ACPM_TX	= 10,	/* request as handed to the mailbox */
	TEGU_PMT_ACPM_TX_D	= 11,	/* its remaining payload words */
	TEGU_PMT_ACPM_RX	= 12,	/* reply as read back from the ring */
	TEGU_PMT_ACPM_RX_D	= 13,
	TEGU_PMT_PM		= 14,	/* suspend/resume boundary */
};

/* Families, for the runtime enable mask. */
#define TEGU_PMT_F_MARK		BIT(0)
#define TEGU_PMT_F_SEC		BIT(1)
#define TEGU_PMT_F_PMUCAL	BIT(2)
#define TEGU_PMT_F_SEQ_STEP	BIT(3)
#define TEGU_PMT_F_ACPM		BIT(4)
#define TEGU_PMT_F_PM		BIT(5)
/*
 * Echo every record to the console as it is made.  The ring is in DRAM and
 * debugfs needs a running kernel, so a suspend that never returns leaves it
 * unreadable -- and recovery from that state is a cold reset, which clears
 * DRAM, so pstore cannot carry it either.  The UART is the only way out.
 *
 * Costs roughly 9 ms a record at 115200 baud, so this is for the descent
 * alone: arm it immediately before the write to /sys/power/state, and boot
 * with no_console_suspend or the console is gone before the interesting part.
 */
#define TEGU_PMT_F_SERIAL	BIT(6)

/* Phases, stamped into every record by whoever is recording. */
enum tegu_pmt_phase {
	TEGU_PMT_P_BOOT		= 0,
	TEGU_PMT_P_RUN		= 1,
	TEGU_PMT_P_SYSCORE_SUSPEND = 2,
	TEGU_PMT_P_ARMED	= 3,	/* enter sequence done, AP on its way down */
	TEGU_PMT_P_SYSCORE_RESUME = 4,
	TEGU_PMT_P_RESUMED	= 5,
};

/*
 * Sequence labels. The high half of a record's ctx names the sequence, the
 * low half is the element index within it, so a record can be tied back to
 * the exact table entry that produced it.
 */
#define TEGU_PMT_SEQ_LPM_INIT	0x0001
#define TEGU_PMT_SEQ_SYS_ENTER	0x0100	/* | mode */
#define TEGU_PMT_SEQ_SYS_SAVE	0x0200
#define TEGU_PMT_SEQ_SYS_EXIT	0x0300
#define TEGU_PMT_SEQ_SYS_EWKUP	0x0400
#define TEGU_PMT_SEQ_SYS_RESTORE 0x0500
#define TEGU_PMT_SEQ_LOCAL_ON	0x0600	/* | pd index */
#define TEGU_PMT_SEQ_LOCAL_OFF	0x0700
#define TEGU_PMT_SEQ_CPU	0x0800
#define TEGU_PMT_SEQ_POWERMODE	0x0900	/* CPU_INFORM hint */

#define TEGU_PMT_CTX(label, idx)	(((u32)(label) << 16) | ((idx) & 0xffff))

/* 48 bytes; the decoder reads this layout straight out of the debugfs dump. */
struct tegu_pmt_rec {
	u64 ts;		/* local_clock(), which does not advance while asleep */
	u32 seq;	/* global order, the only reliable ordering across CPUs */
	u16 kind;
	u8 cpu;
	u8 phase;
	u64 addr;	/* physical address, or a channel id for ACPM */
	u32 ctx;	/* TEGU_PMT_CTX() */
	u32 d[5];
};

/* The decoder unpacks exactly this; a layout change has to be deliberate. */
static_assert(sizeof(struct tegu_pmt_rec) == 48,
	      "tegu_pmt_rec layout is shared with the decoder");

#if IS_ENABLED(CONFIG_TEGU_PMTRACE)

extern u32 tegu_pmt_mask;

void tegu_pmt_record(u16 kind, u64 addr, u32 d0, u32 d1, u32 d2, u32 d3, u32 d4);
void tegu_pmt_set_phase(u8 phase);
void tegu_pmt_set_ctx(u32 ctx);
u32 tegu_pmt_get_ctx(void);

#else /* !CONFIG_TEGU_PMTRACE */

#define tegu_pmt_mask 0u
static inline void tegu_pmt_record(u16 kind, u64 addr, u32 d0, u32 d1, u32 d2,
				   u32 d3, u32 d4) { }
static inline void tegu_pmt_set_phase(u8 phase) { }
static inline void tegu_pmt_set_ctx(u32 ctx) { }
static inline u32 tegu_pmt_get_ctx(void) { return 0; }

#endif /* CONFIG_TEGU_PMTRACE */

static inline bool tegu_pmt_on(u32 family)
{
	return READ_ONCE(tegu_pmt_mask) & family;
}

static inline void tegu_pmt_ev(u32 family, u16 kind, u64 addr,
			       u32 d0, u32 d1, u32 d2, u32 d3, u32 d4)
{
	if (tegu_pmt_on(family))
		tegu_pmt_record(kind, addr, d0, d1, d2, d3, d4);
}

/*
 * One ACPM message: a header record naming the channel, the sequence number,
 * how many payload words there are and the first two of them, then as many
 * continuation records as the rest of the payload needs.  @size is the
 * channel's element size in bytes, which is also how much of @cmd the driver
 * itself copies, so nothing here reads past what the caller provided.
 */
static inline void tegu_pmt_acpm(u16 kind, u16 dkind, u32 channel, u32 seq_num,
				 const unsigned int *cmd, unsigned int size,
				 u32 flags)
{
	unsigned int words = size / sizeof(u32);
	unsigned int i;

	if (!tegu_pmt_on(TEGU_PMT_F_ACPM) || !cmd)
		return;

	if (words > 16)
		words = 16;

	tegu_pmt_record(kind, channel, seq_num, words, flags,
			words > 0 ? cmd[0] : 0,
			words > 1 ? cmd[1] : 0);

	for (i = 2; i < words; i += 5)
		tegu_pmt_record(dkind, channel,
				cmd[i],
				i + 1 < words ? cmd[i + 1] : 0,
				i + 2 < words ? cmd[i + 2] : 0,
				i + 3 < words ? cmd[i + 3] : 0,
				i + 4 < words ? cmd[i + 4] : 0);
}

#endif /* __LINUX_SOC_SAMSUNG_TEGU_PMTRACE_H */
