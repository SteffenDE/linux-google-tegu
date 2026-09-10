// SPDX-License-Identifier: GPL-2.0
/*
 * tegu-pmtrace - bounded binary recorder for suspend-path operations.
 *
 * Storage and debugfs for the recorder declared in
 * <linux/soc/samsung/tegu-pmtrace.h>.  Built in rather than modular so the
 * buffers are live before the first PMU write, and so a boot that hangs still
 * leaves whatever it managed to record.
 *
 * The downstream vendor kernel carries the same recorder with the same record
 * layout, so dumps from the two decode with one tool and diff directly.  That
 * is the whole point of it: the interesting question is which operations the
 * two kernels issue that the other does not.
 */

#include <linux/atomic.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/percpu.h>
#include <linux/sched/clock.h>
#include <linux/seq_file.h>
#include <linux/smp.h>
#include <linux/uaccess.h>
#include <linux/soc/samsung/tegu-pmtrace.h>

enum {
	TEGU_PMT_BOOT_RECS = 1 << 15,	/* 1.5 MB, kept for the boot setup */
	TEGU_PMT_LIVE_RECS = 1 << 15,	/* 1.5 MB, wraps */
};

u32 tegu_pmt_mask = TEGU_PMT_F_MARK | TEGU_PMT_F_SEC | TEGU_PMT_F_PMUCAL |
		    TEGU_PMT_F_ACPM | TEGU_PMT_F_PM;
EXPORT_SYMBOL(tegu_pmt_mask);

static struct tegu_pmt_rec tegu_pmt_boot_buf[TEGU_PMT_BOOT_RECS];
static struct tegu_pmt_rec tegu_pmt_live_buf[TEGU_PMT_LIVE_RECS];
static atomic_t tegu_pmt_boot_claimed = ATOMIC_INIT(0);
static atomic_t tegu_pmt_live_claimed = ATOMIC_INIT(0);
static atomic_t tegu_pmt_seq_ctr = ATOMIC_INIT(0);
static bool tegu_pmt_boot_open = true;
static u8 tegu_pmt_phase_val;
static DEFINE_PER_CPU(u32, tegu_pmt_ctx_val);
static struct dentry *tegu_pmt_dir;

void tegu_pmt_set_phase(u8 phase)
{
	WRITE_ONCE(tegu_pmt_phase_val, phase);
}
EXPORT_SYMBOL(tegu_pmt_set_phase);

void tegu_pmt_set_ctx(u32 ctx)
{
	raw_cpu_write(tegu_pmt_ctx_val, ctx);
}
EXPORT_SYMBOL(tegu_pmt_set_ctx);

u32 tegu_pmt_get_ctx(void)
{
	return raw_cpu_read(tegu_pmt_ctx_val);
}
EXPORT_SYMBOL(tegu_pmt_get_ctx);

void tegu_pmt_record(u16 kind, u64 addr, u32 d0, u32 d1, u32 d2, u32 d3, u32 d4)
{
	struct tegu_pmt_rec *r;
	int idx;

	if (READ_ONCE(tegu_pmt_boot_open)) {
		idx = atomic_inc_return(&tegu_pmt_boot_claimed) - 1;
		if (idx < TEGU_PMT_BOOT_RECS) {
			r = &tegu_pmt_boot_buf[idx];
			goto fill;
		}
		/* Boot buffer full: everything from here on wraps in live. */
		WRITE_ONCE(tegu_pmt_boot_open, false);
	}

	idx = atomic_inc_return(&tegu_pmt_live_claimed) - 1;
	r = &tegu_pmt_live_buf[idx & (TEGU_PMT_LIVE_RECS - 1)];
fill:
	r->ts = local_clock();
	r->seq = atomic_inc_return(&tegu_pmt_seq_ctr);
	r->kind = kind;
	r->cpu = raw_smp_processor_id();
	r->phase = READ_ONCE(tegu_pmt_phase_val);
	r->addr = addr;
	r->ctx = raw_cpu_read(tegu_pmt_ctx_val);
	r->d[0] = d0;
	r->d[1] = d1;
	r->d[2] = d2;
	r->d[3] = d3;
	r->d[4] = d4;

	/*
	 * Same fields, same order, in a form the decoder also parses -- so a
	 * descent that never comes back is still readable from the serial log.
	 */
	if (READ_ONCE(tegu_pmt_mask) & TEGU_PMT_F_SERIAL)
		pr_info("tegu_pmt %u %u %u %u %llx %x %x %x %x %x %x\n",
			r->seq, r->kind, r->cpu, r->phase, r->addr, r->ctx,
			d0, d1, d2, d3, d4);
}
EXPORT_SYMBOL(tegu_pmt_record);

static ssize_t tegu_pmt_boot_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	int used = atomic_read(&tegu_pmt_boot_claimed);

	if (used > TEGU_PMT_BOOT_RECS)
		used = TEGU_PMT_BOOT_RECS;

	return simple_read_from_buffer(buf, count, ppos, tegu_pmt_boot_buf,
				       (size_t)used * sizeof(struct tegu_pmt_rec));
}

static ssize_t tegu_pmt_live_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	int claimed = atomic_read(&tegu_pmt_live_claimed);
	size_t len = sizeof(tegu_pmt_live_buf);

	/* Before the first wrap the tail is untouched; do not export it. */
	if (claimed < TEGU_PMT_LIVE_RECS)
		len = (size_t)claimed * sizeof(struct tegu_pmt_rec);

	return simple_read_from_buffer(buf, count, ppos, tegu_pmt_live_buf, len);
}

static const struct file_operations tegu_pmt_boot_fops = {
	.owner = THIS_MODULE,
	.read = tegu_pmt_boot_read,
	.llseek = default_llseek,
};

static const struct file_operations tegu_pmt_live_fops = {
	.owner = THIS_MODULE,
	.read = tegu_pmt_live_read,
	.llseek = default_llseek,
};

static int tegu_pmt_state_show(struct seq_file *s, void *unused)
{
	int boot_claimed = atomic_read(&tegu_pmt_boot_claimed);
	int live_claimed = atomic_read(&tegu_pmt_live_claimed);

	seq_printf(s, "rec_size %zu\n", sizeof(struct tegu_pmt_rec));
	seq_printf(s, "boot_recs %d\n", TEGU_PMT_BOOT_RECS);
	seq_printf(s, "live_recs %d\n", TEGU_PMT_LIVE_RECS);
	seq_printf(s, "boot_open %d\n", READ_ONCE(tegu_pmt_boot_open));
	seq_printf(s, "boot_claimed %d\n", boot_claimed);
	seq_printf(s, "boot_dropped %d\n",
		   boot_claimed > TEGU_PMT_BOOT_RECS ?
		   boot_claimed - TEGU_PMT_BOOT_RECS : 0);
	seq_printf(s, "live_claimed %d\n", live_claimed);
	seq_printf(s, "live_overwritten %d\n",
		   live_claimed > TEGU_PMT_LIVE_RECS ?
		   live_claimed - TEGU_PMT_LIVE_RECS : 0);
	seq_printf(s, "records %d\n", atomic_read(&tegu_pmt_seq_ctr));
	seq_printf(s, "mask 0x%x\n", READ_ONCE(tegu_pmt_mask));
	seq_printf(s, "phase %u\n", READ_ONCE(tegu_pmt_phase_val));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(tegu_pmt_state);

static int __init tegu_pmt_debugfs_init(void)
{
	tegu_pmt_dir = debugfs_create_dir("tegu-pmtrace", NULL);
	if (IS_ERR(tegu_pmt_dir)) {
		tegu_pmt_dir = NULL;
		return 0;
	}

	debugfs_create_file("boot", 0400, tegu_pmt_dir, NULL, &tegu_pmt_boot_fops);
	debugfs_create_file("live", 0400, tegu_pmt_dir, NULL, &tegu_pmt_live_fops);
	debugfs_create_file("state", 0400, tegu_pmt_dir, NULL, &tegu_pmt_state_fops);
	debugfs_create_u32("mask", 0600, tegu_pmt_dir, &tegu_pmt_mask);
	debugfs_create_bool("boot_open", 0600, tegu_pmt_dir, &tegu_pmt_boot_open);
	debugfs_create_u8("phase", 0600, tegu_pmt_dir, &tegu_pmt_phase_val);
	return 0;
}

late_initcall(tegu_pmt_debugfs_init);
