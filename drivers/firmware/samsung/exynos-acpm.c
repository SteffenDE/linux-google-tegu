// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 Samsung Electronics Co., Ltd.
 * Copyright 2020 Google LLC.
 * Copyright 2024 Linaro Ltd.
 */

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/container_of.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/find.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/mailbox/exynos-message.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/panic.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "exynos-acpm.h"
#include "exynos-acpm-dvfs.h"
#include "exynos-acpm-pmic.h"
#include "exynos-acpm-tmu.h"

#define ACPM_PROTOCOL_SEQNUM		GENMASK(21, 16)

#define ACPM_POLL_TIMEOUT_US		(100 * USEC_PER_MSEC)
#define ACPM_TX_TIMEOUT_US		500000

#define ACPM_GS101_INITDATA_BASE	0xa000

#define ACPM_FW_LOG_ENTRY_WORDS		4
#define ACPM_FW_LOG_DUMP_MAX_ENTRIES	512
#define ACPM_FW_LOG_STRING_MAX		80
#define ACPM_QUEUE_DUMP_MAX_SLOTS	32
#define ACPM_QUEUE_DUMP_WORDS		4

/* Firmware tick period in picoseconds, despite the downstream _US name. */
#define ACPM_SYSTICK_PERIOD_PS		20345

/**
 * struct acpm_build_info - firmware build identity embedded in initdata.
 * @build_version: firmware build string.
 * @major: framework ABI major version.
 * @minor: framework ABI minor version.
 */
struct acpm_build_info {
	char build_version[48];
	u8 major;
	u8 minor;
};

/**
 * struct acpm_fw_log_snapshot - host copy of one firmware log tail.
 * @name: diagnostic name of the firmware ring.
 * @entries: preallocated host buffer, in newest-first order.
 * @error: validation failure, or NULL for a usable snapshot.
 * @rear_offset: SRAM offset of the firmware's rear index.
 * @front_offset: SRAM offset of the firmware's front index.
 * @data_offset: SRAM offset of the firmware's entry array.
 * @entry_size: firmware entry stride in bytes.
 * @len: number of entries in the firmware ring.
 * @rear: sampled firmware rear index.
 * @front: sampled firmware front index.
 * @count: number of entries copied into @entries.
 */
struct acpm_fw_log_snapshot {
	const char *name;
	u32 *entries;
	const char *error;
	u32 rear_offset;
	u32 front_offset;
	u32 data_offset;
	u32 entry_size;
	u32 len;
	u32 rear;
	u32 front;
	u32 count;
};

/**
 * struct acpm_shmem - shared memory configuration information.
 * @plugins:	offset to the firmware plugin table.
 * @num_plugins: number of firmware plugins.
 * @chans:	offset to array of struct acpm_chan_shmem.
 * @num_ipc_chans: number of firmware IPC channels.
 * @pid_framework: framework plugin ID.
 * @pid_max:	maximum plugin ID.
 * @num_chans:	number of AP-visible channels.
 * @reserved_ipc: channel counts for other ACPM peers and global limits.
 * @ktime_index: firmware timekeeping index.
 * @log_buf_rear: offset to the normal firmware log rear index.
 * @log_buf_front: offset to the normal firmware log front index.
 * @log_data:	offset to the normal firmware log entries.
 * @log_entry_size: firmware log entry size in bytes.
 * @log_entry_len: number of normal firmware log entries.
 * @reserved_fw: firmware IPC and interrupt metadata.
 * @info:	firmware build identity.
 * @reserved_preempt: preemption metadata.
 * @preempt_log_buf_rear: offset to the preempt log rear index.
 * @preempt_log_buf_front: offset to the preempt log front index.
 * @preempt_log_data: offset to the preempt firmware log entries.
 * @preempt_log_entry_len: number of preempt firmware log entries.
 */
struct acpm_shmem {
	u32 plugins;
	u32 num_plugins;
	u32 chans;
	u32 num_ipc_chans;
	u32 pid_framework;
	u32 pid_max;
	u32 num_chans;
	u32 reserved_ipc[5];
	u32 ktime_index;
	u32 log_buf_rear;
	u32 log_buf_front;
	u32 log_data;
	u32 log_entry_size;
	u32 log_entry_len;
	u32 reserved_fw[5];
	struct acpm_build_info info;
	u32 reserved_preempt[6];
	u32 preempt_log_buf_rear;
	u32 preempt_log_buf_front;
	u32 preempt_log_data;
	u32 preempt_log_entry_len;
};

static_assert(offsetof(struct acpm_shmem, chans) == 0x8);
static_assert(offsetof(struct acpm_shmem, num_chans) == 0x18);
static_assert(offsetof(struct acpm_shmem, log_buf_rear) == 0x34);
static_assert(offsetof(struct acpm_shmem, preempt_log_buf_rear) == 0xa8);

/**
 * struct acpm_chan_shmem - descriptor of a shared memory channel.
 *
 * @id:			channel ID.
 * @reserved:		unused fields.
 * @rx_rear:		rear pointer of APM RX queue (TX for AP).
 * @rx_front:		front pointer of APM RX queue (TX for AP).
 * @rx_base:		base address of APM RX queue (TX for AP).
 * @reserved1:		unused fields.
 * @tx_rear:		rear pointer of APM TX queue (RX for AP).
 * @tx_front:		front pointer of APM TX queue (RX for AP).
 * @tx_base:		base address of APM TX queue (RX for AP).
 * @qlen:		queue length. Applies to both TX/RX queues.
 * @mlen:		message length. Applies to both TX/RX queues.
 * @reserved2:		unused fields.
 * @poll_completion:	true when the channel works on polling.
 */
struct acpm_chan_shmem {
	u32 id;
	u32 reserved[3];
	u32 rx_rear;
	u32 rx_front;
	u32 rx_base;
	u32 reserved1[3];
	u32 tx_rear;
	u32 tx_front;
	u32 tx_base;
	u32 qlen;
	u32 mlen;
	u32 reserved2[2];
	u32 poll_completion;
};

/**
 * struct acpm_queue - exynos acpm queue.
 *
 * @rear:	rear address of the queue.
 * @front:	front address of the queue.
 * @base:	base address of the queue.
 */
struct acpm_queue {
	void __iomem *rear;
	void __iomem *front;
	void __iomem *base;
};

/**
 * struct acpm_rx_data - RX queue data.
 *
 * @cmd:	pointer to where the data shall be saved.
 * @cmdcnt:	allocated capacity of the @cmd buffer in 32-bit words.
 * @rxcnt:	expected length of the response in 32-bit words.
 * @completed:	flag indicating if the firmware response has been fully
 *		processed.
 * @tx_cmd:	copy of the request assigned to this sequence number.
 * @txcnt:	request length in 32-bit words.
 */
struct acpm_rx_data {
	u32 *cmd __counted_by_ptr(cmdcnt);
	size_t cmdcnt;
	size_t rxcnt;
	bool completed;
	u32 *tx_cmd;
	size_t txcnt;
};

#define ACPM_SEQNUM_MAX    64

/**
 * struct acpm_chan - driver internal representation of a channel.
 * @cl:		mailbox client.
 * @chan:	mailbox channel.
 * @acpm:	pointer to driver private data.
 * @tx:		TX queue. The enqueue is done by the host.
 *			- front index is written by the host.
 *			- rear index is written by the firmware.
 *
 * @rx:		RX queue. The enqueue is done by the firmware.
 *			- front index is written by the firmware.
 *			- rear index is written by the host.
 * @tx_lock:	protects TX queue.
 * @rx_lock:	protects RX queue.
 * @qlen:	queue length. Applies to both TX/RX queues.
 * @mlen:	message length. Applies to both TX/RX queues.
 * @seqnum:	sequence number of the last message enqueued on TX queue.
 * @id:		channel ID.
 * @poll_completion:	indicates if the transfer needs to be polled for
 *			completion or interrupt mode is used.
 * @bitmap_seqnum: bitmap that tracks the messages on the TX/RX queues.
 * @rx_data:	internal buffer used to drain the RX queue.
 */
struct acpm_chan {
	struct mbox_client cl;
	struct mbox_chan *chan;
	struct acpm_info *acpm;
	struct acpm_queue tx;
	struct acpm_queue rx;
	struct mutex tx_lock;
	struct mutex rx_lock;

	unsigned int qlen;
	unsigned int mlen;
	u8 seqnum;
	u8 id;
	bool poll_completion;

	DECLARE_BITMAP(bitmap_seqnum, ACPM_SEQNUM_MAX - 1);
	struct acpm_rx_data rx_data[ACPM_SEQNUM_MAX];
};

/**
 * struct acpm_info - driver's private data.
 * @shmem:	pointer to the SRAM configuration data.
 * @sram_base:	base address of SRAM.
 * @chans:	pointer to the ACPM channel parameters retrieved from SRAM.
 * @dev:	pointer to the exynos-acpm device.
 * @handle:	instance of acpm_handle to send to clients.
 * @num_chans:	number of channels available for this controller.
 * @sram_size:	size of the firmware SRAM mapping.
 * @normal_log: preallocated snapshot of the normal firmware log.
 * @preempt_log: preallocated snapshot of the preempt firmware log.
 * @timeout_claimed: elects one timeout caller to collect fatal diagnostics.
 * @timeout_debug: panic with firmware diagnostics on the first timeout.
 */
struct acpm_info {
	struct acpm_shmem __iomem *shmem;
	void __iomem *sram_base;
	struct acpm_chan *chans;
	struct device *dev;
	struct acpm_handle handle;
	u32 num_chans;
	resource_size_t sram_size;
	struct acpm_fw_log_snapshot normal_log;
	struct acpm_fw_log_snapshot preempt_log;
	atomic_t timeout_claimed;
	bool timeout_debug;
};

/**
 * struct acpm_match_data - of_device_id data.
 * @initdata_base:	offset in SRAM where the channels configuration resides.
 * @acpm_clk_dev_name:	base name for the ACPM clocks device that we're registering.
 * @timeout_debug: collect firmware state and panic on the first IPC timeout.
 */
struct acpm_match_data {
	loff_t initdata_base;
	const char *acpm_clk_dev_name;
	bool timeout_debug;
};

#define client_to_acpm_chan(c) container_of(c, struct acpm_chan, cl)
#define handle_to_acpm_info(h) container_of(h, struct acpm_info, handle)

static bool acpm_sram_range_valid(const struct acpm_info *acpm, u32 offset,
				  resource_size_t size)
{
	return offset < acpm->sram_size && size <= acpm->sram_size - offset;
}

static bool acpm_sram_pointer_valid(const struct acpm_info *acpm,
				    const void __iomem *pointer,
				    resource_size_t size)
{
	unsigned long address = (unsigned long)pointer;
	unsigned long base = (unsigned long)acpm->sram_base;
	resource_size_t offset;

	if (address < base)
		return false;
	if (!IS_ALIGNED(address, sizeof(u32)))
		return false;

	offset = address - base;
	return offset < acpm->sram_size && size <= acpm->sram_size - offset;
}

static bool acpm_fw_log_string(struct acpm_info *acpm, u32 encoded_offset,
			       char *buf, size_t buf_size)
{
	u32 offset = encoded_offset & GENMASK(23, 0);
	size_t i;

	if (!buf_size || !acpm_sram_range_valid(acpm, offset, 1))
		return false;

	for (i = 0; i < buf_size - 1 && offset + i < acpm->sram_size; i++) {
		u8 c = readb(acpm->sram_base + offset + i);

		if (!c) {
			buf[i] = '\0';
			return true;
		}

		/* Keep each firmware record on one printable console line. */
		buf[i] = c >= 0x20 && c <= 0x7e ? c : '.';
	}

	buf[i] = '\0';
	return true;
}

static void acpm_snapshot_fw_log(struct acpm_info *acpm,
				 struct acpm_fw_log_snapshot *snapshot,
				 u32 rear_offset, u32 front_offset,
				 u32 data_offset, u32 entry_size, u32 len)
{
	resource_size_t data_size;
	u32 i;

	snapshot->error = NULL;
	snapshot->rear_offset = rear_offset;
	snapshot->front_offset = front_offset;
	snapshot->data_offset = data_offset;
	snapshot->entry_size = entry_size;
	snapshot->len = len;
	snapshot->count = 0;

	if (entry_size < ACPM_FW_LOG_ENTRY_WORDS * sizeof(u32) || !len ||
	    !IS_ALIGNED(rear_offset, sizeof(u32)) ||
	    !IS_ALIGNED(front_offset, sizeof(u32)) ||
	    !IS_ALIGNED(data_offset, sizeof(u32)) ||
	    !IS_ALIGNED(entry_size, sizeof(u32)) ||
	    len > acpm->sram_size / entry_size) {
		snapshot->error = "invalid geometry";
		return;
	}

	data_size = (resource_size_t)entry_size * len;
	if (!acpm_sram_range_valid(acpm, rear_offset, sizeof(u32)) ||
	    !acpm_sram_range_valid(acpm, front_offset, sizeof(u32)) ||
	    !acpm_sram_range_valid(acpm, data_offset, data_size)) {
		snapshot->error = "outside SRAM";
		return;
	}

	snapshot->rear = readl(acpm->sram_base + rear_offset);
	snapshot->front = readl(acpm->sram_base + front_offset);
	if (snapshot->front >= len) {
		snapshot->error = "invalid indices";
		return;
	}

	snapshot->count = min_t(u32, len, ACPM_FW_LOG_DUMP_MAX_ENTRIES);
	for (i = 0; i < snapshot->count; i++) {
		void __iomem *entry;
		u32 index;

		index = (snapshot->front + len - 1 - i) % len;
		entry = acpm->sram_base + data_offset + index * entry_size;
		__ioread32_copy(&snapshot->entries[i * ACPM_FW_LOG_ENTRY_WORDS],
				  entry, ACPM_FW_LOG_ENTRY_WORDS);
	}
}

static void acpm_snapshot_fw_logs(struct acpm_info *acpm)
{
	struct acpm_shmem __iomem *shmem = acpm->shmem;
	u32 entry_size = readl(&shmem->log_entry_size);

	/* The normal ring is the high-volume ring, so preserve it first. */
	acpm_snapshot_fw_log(acpm, &acpm->normal_log,
			     readl(&shmem->log_buf_rear),
			     readl(&shmem->log_buf_front),
			     readl(&shmem->log_data), entry_size,
			     readl(&shmem->log_entry_len));

	acpm_snapshot_fw_log(acpm, &acpm->preempt_log,
			     readl(&shmem->preempt_log_buf_rear),
			     readl(&shmem->preempt_log_buf_front),
			     readl(&shmem->preempt_log_data), entry_size,
			     readl(&shmem->preempt_log_entry_len));
}

static void acpm_print_fw_log(struct acpm_info *acpm,
			      const struct acpm_fw_log_snapshot *snapshot)
{
	struct device *dev = acpm->dev;
	u32 i;

	if (snapshot->error) {
		dev_emerg(dev,
			  "firmware %s log snapshot failed (%s): rear:%#x front:%#x data:%#x entry:%u len:%u sampled-rear:%u sampled-front:%u\n",
			  snapshot->name, snapshot->error, snapshot->rear_offset,
			  snapshot->front_offset, snapshot->data_offset,
			  snapshot->entry_size, snapshot->len, snapshot->rear,
			  snapshot->front);
		return;
	}

	dev_emerg(dev,
		  "firmware %s log snapshot: rear:%u front:%u len:%u entry:%u; dumping %u newest-first\n",
		  snapshot->name, snapshot->rear, snapshot->front, snapshot->len,
		  snapshot->entry_size, snapshot->count);

	for (i = 0; i < snapshot->count; i++) {
		char message[ACPM_FW_LOG_STRING_MAX];
		const u32 *word = &snapshot->entries[i * ACPM_FW_LOG_ENTRY_WORDS];
		u32 index, id;
		u64 ticks, time_ns;
		bool raw, flag26;

		index = (snapshot->front + snapshot->len - 1 - i) % snapshot->len;

		if (!(word[0] | word[1] | word[2] | word[3]))
			continue;

		id = FIELD_GET(GENMASK(31, 28), word[0]);
		raw = word[0] & BIT(27);
		flag26 = word[0] & BIT(26);
		ticks = ((u64)word[1] << 24) | FIELD_GET(GENMASK(23, 0), word[0]);
		time_ns = mul_u64_u32_div(ticks, ACPM_SYSTICK_PERIOD_PS, 1000);

		if (raw) {
			dev_emerg(dev,
				  "firmware %s[%u]: raw id:%u flag26:%u data:%08x %08x %08x %08x\n",
				  snapshot->name, index, id, flag26, word[0], word[1],
				  word[2], word[3]);
		} else if (acpm_fw_log_string(acpm, word[2], message,
					       sizeof(message))) {
			dev_emerg(dev,
				  "firmware %s[%u]: time-ns:%llu id:%u flag26:%u \"%s\" arg:%08x raw:%08x %08x %08x %08x\n",
				  snapshot->name, index, time_ns, id, flag26,
				  message, word[3],
				  word[0], word[1], word[2], word[3]);
		} else {
			dev_emerg(dev,
				  "firmware %s[%u]: time-ns:%llu id:%u flag26:%u bad-string:%08x arg:%08x raw:%08x %08x %08x %08x\n",
				  snapshot->name, index, time_ns, id, flag26,
				  word[2], word[3],
				  word[0], word[1], word[2], word[3]);
		}
	}
}

static void acpm_print_fw_logs(struct acpm_info *acpm)
{
	/* Preempt records include exception and high-priority handler context. */
	acpm_print_fw_log(acpm, &acpm->preempt_log);
	acpm_print_fw_log(acpm, &acpm->normal_log);
}

static void acpm_dump_queue_slots(struct acpm_chan *achan, const char *name,
				  const struct acpm_queue *queue)
{
	struct device *dev = achan->acpm->dev;
	u32 count = min_t(u32, achan->qlen, ACPM_QUEUE_DUMP_MAX_SLOTS);
	u32 words = min_t(u32, achan->mlen / sizeof(u32),
			  ACPM_QUEUE_DUMP_WORDS);
	u32 i;
	resource_size_t size = (resource_size_t)achan->mlen * count;

	if (!count || !words || !IS_ALIGNED(achan->mlen, sizeof(u32)) ||
	    !acpm_sram_pointer_valid(achan->acpm, queue->base, size)) {
		dev_emerg(dev,
			  "ch:%u %s queue cannot be dumped safely: base:%px slots:%u mlen:%u words:%u\n",
			  achan->id, name, queue->base, count, achan->mlen, words);
		return;
	}

	for (i = 0; i < count; i++) {
		u32 data[ACPM_QUEUE_DUMP_WORDS] = {};

		__ioread32_copy(data, queue->base + achan->mlen * i, words);
		dev_emerg(dev,
			  "ch:%u %s-slot:%u data:%08x %08x %08x %08x\n",
			  achan->id, name, i, data[0], data[1], data[2], data[3]);
	}

	if (count != achan->qlen)
		dev_emerg(dev, "ch:%u %s queue truncated at %u of %u slots\n",
			  achan->id, name, count, achan->qlen);
}

static void acpm_dump_channels(struct acpm_info *acpm,
			       const struct acpm_chan *timed_out)
{
	u32 i;

	for (i = 0; i < acpm->num_chans; i++) {
		struct acpm_chan *achan = &acpm->chans[i];
		DECLARE_BITMAP(pending, ACPM_SEQNUM_MAX - 1);
		u32 rx_rear = 0, rx_front = 0, tx_rear = 0, tx_front = 0;
		bool indices_valid;
		unsigned long seq;

		bitmap_copy(pending, achan->bitmap_seqnum, ACPM_SEQNUM_MAX - 1);
		indices_valid =
			acpm_sram_pointer_valid(acpm, achan->rx.rear, sizeof(u32)) &&
			acpm_sram_pointer_valid(acpm, achan->rx.front, sizeof(u32)) &&
			acpm_sram_pointer_valid(acpm, achan->tx.rear, sizeof(u32)) &&
			acpm_sram_pointer_valid(acpm, achan->tx.front, sizeof(u32));
		if (indices_valid) {
			rx_rear = readl(achan->rx.rear);
			rx_front = readl(achan->rx.front);
			tx_rear = readl(achan->tx.rear);
			tx_front = readl(achan->tx.front);
		}

		dev_emerg(acpm->dev,
			  "ch:%u state: pending:%*pb indices-valid:%u RX rear:%u front:%u TX rear:%u front:%u qlen:%u mlen:%u\n",
			  achan->id, ACPM_SEQNUM_MAX - 1, pending,
			  indices_valid, rx_rear, rx_front, tx_rear, tx_front,
			  achan->qlen, achan->mlen);

		for_each_set_bit(seq, pending, ACPM_SEQNUM_MAX - 1) {
			struct acpm_rx_data *rx_data = &achan->rx_data[seq];
			u32 data[ACPM_QUEUE_DUMP_WORDS] = {};
			size_t words;

			words = min_t(size_t, rx_data->txcnt, ARRAY_SIZE(data));
			if (rx_data->tx_cmd)
				memcpy(data, rx_data->tx_cmd, words * sizeof(*data));

			dev_emerg(acpm->dev,
				  "ch:%u seq:%lu outstanding: complete:%u txcnt:%zu data:%08x %08x %08x %08x\n",
				  achan->id, seq + 1, READ_ONCE(rx_data->completed),
				  READ_ONCE(rx_data->txcnt), data[0], data[1],
				  data[2], data[3]);
		}

		if (!bitmap_empty(pending, ACPM_SEQNUM_MAX - 1) ||
		    achan == timed_out) {
			acpm_dump_queue_slots(achan, "TX", &achan->tx);
			acpm_dump_queue_slots(achan, "RX", &achan->rx);
		}
	}
}

static void acpm_timeout_debug(struct acpm_chan *achan,
			       const struct acpm_xfer *xfer, u32 seqnum)
{
	struct acpm_info *acpm = achan->acpm;
	u32 request[ACPM_QUEUE_DUMP_WORDS] = {};
	size_t words;

	if (!acpm->timeout_debug ||
	    atomic_cmpxchg(&acpm->timeout_claimed, 0, 1))
		return;

	words = min_t(size_t, xfer->txcnt, ARRAY_SIZE(request));
	memcpy(request, xfer->txd, words * sizeof(*request));

	/* Preserve volatile firmware evidence before synchronous console output. */
	acpm_snapshot_fw_logs(acpm);

	dev_emerg(acpm->dev,
		  "first IPC timeout: ch:%u seq:%u txcnt:%zu request:%08x %08x %08x %08x\n",
		  achan->id, seqnum, xfer->txcnt, request[0], request[1],
		  request[2], request[3]);

	exynos_mbox_dump_regs(achan->chan);
	acpm_dump_channels(acpm, achan);
	acpm_print_fw_logs(acpm);
	dump_stack();

	/* A diagnostic kernel must come back even without panic= on cmdline. */
	if (!panic_timeout)
		panic_timeout = 5;

	panic("ACPM IPC timeout on channel %u sequence %u", achan->id, seqnum);
}

/**
 * acpm_get_saved_rx() - get the response if it was already saved.
 * @achan:	ACPM channel info.
 * @xfer:	reference to the transfer to get response for.
 * @tx_seqnum:	xfer TX sequence number.
 */
static void acpm_get_saved_rx(struct acpm_chan *achan,
			      const struct acpm_xfer *xfer, u32 tx_seqnum)
{
	const struct acpm_rx_data *rx_data = &achan->rx_data[tx_seqnum - 1];
	u32 rx_seqnum;

	if (!rx_data->rxcnt)
		return;

	rx_seqnum = FIELD_GET(ACPM_PROTOCOL_SEQNUM, rx_data->cmd[0]);

	if (rx_seqnum == tx_seqnum)
		memcpy(xfer->rxd, rx_data->cmd, xfer->rxcnt * sizeof(*xfer->rxd));
}

/**
 * acpm_get_rx() - get response from RX queue.
 * @achan:	ACPM channel info.
 * @xfer:	reference to the transfer to get response for.
 * @native_match: pointer to a boolean set to true if the thread natively
 *                processed its own sequence number during this call.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int acpm_get_rx(struct acpm_chan *achan, const struct acpm_xfer *xfer,
		       bool *native_match)
{
	u32 rx_front, rx_seqnum, tx_seqnum, seqnum;
	const void __iomem *base, *addr;
	struct acpm_rx_data *rx_data;
	u32 i, val, mlen;

	*native_match = false;

	guard(mutex)(&achan->rx_lock);

	rx_front = readl(achan->rx.front);
	i = readl(achan->rx.rear);

	tx_seqnum = FIELD_GET(ACPM_PROTOCOL_SEQNUM, xfer->txd[0]);

	if (i == rx_front)
		return 0;

	base = achan->rx.base;
	mlen = achan->mlen;

	/* Drain RX queue. */
	do {
		/* Read RX seqnum. */
		addr = base + mlen * i;
		val = readl(addr);

		rx_seqnum = FIELD_GET(ACPM_PROTOCOL_SEQNUM, val);
		if (!rx_seqnum)
			return -EIO;
		/*
		 * mssg seqnum starts with value 1, whereas the driver considers
		 * the first mssg at index 0.
		 */
		seqnum = rx_seqnum - 1;
		rx_data = &achan->rx_data[seqnum];

		if (rx_data->rxcnt) {
			if (rx_seqnum == tx_seqnum) {
				__ioread32_copy(xfer->rxd, addr, xfer->rxcnt);
				/*
				 * Signal completion to the polling thread.
				 * Pairs with smp_load_acquire() in polling
				 * loop.
				 */
				smp_store_release(&rx_data->completed, true);
				*native_match = true;
			} else {
				/*
				 * The RX data corresponds to another request.
				 * Save the data to drain the queue, but don't
				 * clear yet the bitmap. It will be cleared
				 * after the response is copied to the request.
				 */
				__ioread32_copy(rx_data->cmd, addr,
						rx_data->rxcnt);
				/*
				 * Signal completion to the polling thread.
				 * Pairs with smp_load_acquire() in polling
				 * loop.
				 */
				smp_store_release(&rx_data->completed, true);
			}
		} else {
			/*
			 * Signal completion to the polling thread.
			 * Pairs with smp_load_acquire() in polling loop.
			 */
			smp_store_release(&rx_data->completed, true);
			if (rx_seqnum == tx_seqnum)
				*native_match = true;
		}

		i = (i + 1) % achan->qlen;
	} while (i != rx_front);

	/* We saved all responses, mark RX empty. */
	writel(rx_front, achan->rx.rear);

	return 0;
}

/**
 * acpm_dequeue_by_polling() - RX dequeue by polling.
 * @achan:	ACPM channel info.
 * @xfer:	reference to the transfer being waited for.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int acpm_dequeue_by_polling(struct acpm_chan *achan,
				   const struct acpm_xfer *xfer)
{
	struct device *dev = achan->acpm->dev;
	bool native_match;
	ktime_t timeout;
	u32 seqnum;
	int ret;

	seqnum = FIELD_GET(ACPM_PROTOCOL_SEQNUM, xfer->txd[0]);

	timeout = ktime_add_us(ktime_get(), ACPM_POLL_TIMEOUT_US);
	do {
		ret = acpm_get_rx(achan, xfer, &native_match);
		if (ret)
			return ret;

		/*
		 * Safely check if our specific transaction has been processed.
		 * smp_load_acquire prevents the CPU from speculatively
		 * executing subsequent instructions before the transaction is
		 * synchronized.
		 */
		if (smp_load_acquire(&achan->rx_data[seqnum - 1].completed)) {
			/* Retrieve payload if another thread cached it for us */
			if (!native_match)
				acpm_get_saved_rx(achan, xfer, seqnum);

			/* Relinquish ownership of the sequence slot */
			clear_bit_unlock(seqnum - 1, achan->bitmap_seqnum);
			return 0;
		}

		/* Determined experimentally. */
		udelay(20);
	} while (ktime_before(ktime_get(), timeout));

	acpm_timeout_debug(achan, xfer, seqnum);
	dev_err(dev, "Timeout! ch:%u s:%u bitmap:%lx.\n",
		achan->id, seqnum, achan->bitmap_seqnum[0]);

	return -ETIME;
}

/**
 * acpm_wait_for_queue_slots() - wait for queue slots.
 *
 * @achan:		ACPM channel info.
 * @next_tx_front:	next front index of the TX queue.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int acpm_wait_for_queue_slots(struct acpm_chan *achan, u32 next_tx_front)
{
	u32 val, ret;

	/*
	 * Wait for RX front to keep up with TX front. Make sure there's at
	 * least one element between them.
	 */
	ret = readl_poll_timeout(achan->rx.front, val, next_tx_front != val, 0,
				 ACPM_TX_TIMEOUT_US);
	if (ret) {
		dev_err(achan->acpm->dev, "RX front can not keep up with TX front.\n");
		return ret;
	}

	ret = readl_poll_timeout(achan->tx.rear, val, next_tx_front != val, 0,
				 ACPM_TX_TIMEOUT_US);
	if (ret)
		dev_err(achan->acpm->dev, "TX queue is full.\n");

	return ret;
}

/**
 * acpm_prepare_xfer() - prepare a transfer before writing the message to the
 * TX queue.
 * @achan:	ACPM channel info.
 * @xfer:	reference to the transfer being prepared.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int acpm_prepare_xfer(struct acpm_chan *achan,
			     const struct acpm_xfer *xfer)
{
	struct acpm_rx_data *rx_data;
	u32 *txd = (u32 *)xfer->txd;
	unsigned long size = ACPM_SEQNUM_MAX - 1;
	unsigned long bit = achan->seqnum;

	bit = find_next_zero_bit(achan->bitmap_seqnum, size, bit);
	if (bit >= size) {
		bit = find_first_zero_bit(achan->bitmap_seqnum, size);
		if (bit >= size) {
			dev_err_ratelimited(achan->acpm->dev,
					    "ACPM sequence number pool exhausted\n");
			return -EBUSY;
		}
	}

	/*
	 * Execute the atomic set to formally claim the bit and establish
	 * LKMM Acquire semantics against the RX thread's clear_bit_unlock().
	 * A loop is unnecessary because allocations are strictly serialized
	 * by tx_lock.
	 */
	if (WARN_ON_ONCE(test_and_set_bit_lock(bit, achan->bitmap_seqnum)))
		return -EIO;

	/* Flag the index based on seqnum. (seqnum: 1~63, bitmap: 0~62) */
	achan->seqnum = bit + 1;
	txd[0] |= FIELD_PREP(ACPM_PROTOCOL_SEQNUM, achan->seqnum);

	/* Clear data for upcoming responses */
	rx_data = &achan->rx_data[bit];
	rx_data->completed = false;
	memset(rx_data->cmd, 0, sizeof(*rx_data->cmd) * rx_data->cmdcnt);
	/* zero means no response expected */
	rx_data->rxcnt = xfer->rxcnt;

	/* Retain the exact outstanding request for first-timeout diagnostics. */
	memset(rx_data->tx_cmd, 0, sizeof(*rx_data->tx_cmd) * rx_data->cmdcnt);
	memcpy(rx_data->tx_cmd, xfer->txd,
	       min(xfer->txcnt, rx_data->cmdcnt) * sizeof(*rx_data->tx_cmd));
	rx_data->txcnt = xfer->txcnt;

	return 0;
}

/**
 * acpm_wait_for_message_response - an helper to group all possible ways of
 * waiting for a synchronous message response.
 *
 * @achan:	ACPM channel info.
 * @xfer:	reference to the transfer being waited for.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int acpm_wait_for_message_response(struct acpm_chan *achan,
					  const struct acpm_xfer *xfer)
{
	/* Just polling mode supported for now. */
	return acpm_dequeue_by_polling(achan, xfer);
}

/**
 * acpm_do_xfer() - do one transfer.
 * @handle:	pointer to the acpm handle.
 * @xfer:	transfer to initiate and wait for response.
 *
 * Return: 0 on success, -errno otherwise.
 */
int acpm_do_xfer(struct acpm_handle *handle, const struct acpm_xfer *xfer)
{
	struct acpm_info *acpm = handle_to_acpm_info(handle);
	struct exynos_mbox_msg msg;
	struct acpm_chan *achan;
	u32 idx, tx_front;
	int ret;

	if (xfer->acpm_chan_id >= acpm->num_chans)
		return -EINVAL;

	achan = &acpm->chans[xfer->acpm_chan_id];

	if (!xfer->txd ||
	    (xfer->txcnt * sizeof(*xfer->txd) > achan->mlen) ||
	    (xfer->rxcnt * sizeof(*xfer->rxd) > achan->mlen))
		return -EINVAL;

	if (!achan->poll_completion) {
		dev_err(achan->acpm->dev, "Interrupt mode not supported\n");
		return -EOPNOTSUPP;
	}

	msg.chan_id = xfer->acpm_chan_id;
	msg.chan_type = EXYNOS_MBOX_CHAN_TYPE_DOORBELL;

	scoped_guard(mutex, &achan->tx_lock) {
		tx_front = readl(achan->tx.front);
		idx = (tx_front + 1) % achan->qlen;

		ret = acpm_wait_for_queue_slots(achan, idx);
		if (ret)
			return ret;

		ret = acpm_prepare_xfer(achan, xfer);
		if (ret)
			return ret;

		/* Write TX command. */
		__iowrite32_copy(achan->tx.base + achan->mlen * tx_front,
				 xfer->txd, xfer->txcnt);

		/* Advance TX front. */
		writel(idx, achan->tx.front);

		ret = mbox_send_message(achan->chan, (void *)&msg);
		if (ret < 0)
			return ret;

		mbox_client_txdone(achan->chan, 0);
	}

	return acpm_wait_for_message_response(achan, xfer);
}

/**
 * acpm_set_xfer() - initialize an ACPM IPC transfer structure.
 * @xfer:	pointer to the ACPM transfer structure that is being initialized.
 * @cmd:	pointer to the buffer containing the command to be transmitted
 *              to the ACPM firmware.
 * @cmdcnt:	length of the command in 32-bit words.
 * @acpm_chan_id: mailbox channel identifier.
 * @response:	boolean flag indicating whether the kernel expects the ACPM
 *              firmware to send a reply to this specific command.
 */
void acpm_set_xfer(struct acpm_xfer *xfer, u32 *cmd, size_t cmdcnt,
		   unsigned int acpm_chan_id, bool response)
{
	xfer->acpm_chan_id = acpm_chan_id;
	xfer->txcnt = cmdcnt;
	xfer->txd = cmd;

	if (response) {
		xfer->rxcnt = cmdcnt;
		xfer->rxd = cmd;
	} else {
		xfer->rxcnt = 0;
		xfer->rxd = NULL;
	}
}

/**
 * acpm_chan_shmem_get_params() - get channel parameters and addresses of the
 * TX/RX queues.
 * @achan:	ACPM channel info.
 * @chan_shmem:	__iomem pointer to a channel described in shared memory.
 */
static void acpm_chan_shmem_get_params(struct acpm_chan *achan,
				struct acpm_chan_shmem __iomem *chan_shmem)
{
	void __iomem *base = achan->acpm->sram_base;
	struct acpm_queue *rx = &achan->rx;
	struct acpm_queue *tx = &achan->tx;

	achan->mlen = readl(&chan_shmem->mlen);
	achan->poll_completion = readl(&chan_shmem->poll_completion);
	achan->id = readl(&chan_shmem->id);
	achan->qlen = readl(&chan_shmem->qlen);

	tx->base = base + readl(&chan_shmem->rx_base);
	tx->rear = base + readl(&chan_shmem->rx_rear);
	tx->front = base + readl(&chan_shmem->rx_front);

	rx->base = base + readl(&chan_shmem->tx_base);
	rx->rear = base + readl(&chan_shmem->tx_rear);
	rx->front = base + readl(&chan_shmem->tx_front);

	dev_vdbg(achan->acpm->dev, "ID = %d poll = %d, mlen = %d, qlen = %d\n",
		 achan->id, achan->poll_completion, achan->mlen, achan->qlen);
}

/**
 * acpm_achan_alloc_cmds() - allocate buffers for retrieving data from the ACPM
 * firmware.
 * @achan:	ACPM channel info.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int acpm_achan_alloc_cmds(struct acpm_chan *achan)
{
	struct device *dev = achan->acpm->dev;
	struct acpm_rx_data *rx_data;
	size_t cmd_size, cmdcnt;
	int i;

	if (achan->mlen == 0)
		return 0;

	cmd_size = sizeof(*(achan->rx_data[0].cmd));
	cmdcnt = DIV_ROUND_UP_ULL(achan->mlen, cmd_size);

	for (i = 0; i < ACPM_SEQNUM_MAX; i++) {
		rx_data = &achan->rx_data[i];
		rx_data->cmdcnt = cmdcnt;
		rx_data->cmd = devm_kcalloc(dev, cmdcnt, cmd_size, GFP_KERNEL);
		if (!rx_data->cmd)
			return -ENOMEM;

		rx_data->tx_cmd = devm_kcalloc(dev, cmdcnt, cmd_size, GFP_KERNEL);
		if (!rx_data->tx_cmd)
			return -ENOMEM;
	}

	return 0;
}

/**
 * acpm_free_mbox_chans() - free mailbox channels.
 * @data:	pointer to driver data.
 */
static void acpm_free_mbox_chans(void *data)
{
	struct acpm_info *acpm = data;
	int i;

	for (i = 0; i < acpm->num_chans; i++)
		if (!IS_ERR_OR_NULL(acpm->chans[i].chan))
			mbox_free_channel(acpm->chans[i].chan);
}

/**
 * acpm_channels_init() - initialize channels based on the configuration data in
 * the shared memory.
 * @acpm:	pointer to driver data.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int acpm_channels_init(struct acpm_info *acpm)
{
	struct acpm_shmem __iomem *shmem = acpm->shmem;
	struct acpm_chan_shmem __iomem *chans_shmem;
	struct device *dev = acpm->dev;
	int i, ret;

	acpm->num_chans = readl(&shmem->num_chans);
	acpm->chans = devm_kcalloc(dev, acpm->num_chans, sizeof(*acpm->chans),
				   GFP_KERNEL);
	if (!acpm->chans)
		return -ENOMEM;

	ret = devm_add_action_or_reset(dev, acpm_free_mbox_chans, acpm);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add mbox free action.\n");

	chans_shmem = acpm->sram_base + readl(&shmem->chans);

	for (i = 0; i < acpm->num_chans; i++) {
		struct acpm_chan_shmem __iomem *chan_shmem = &chans_shmem[i];
		struct acpm_chan *achan = &acpm->chans[i];
		struct mbox_client *cl = &achan->cl;

		achan->acpm = acpm;

		acpm_chan_shmem_get_params(achan, chan_shmem);

		ret = acpm_achan_alloc_cmds(achan);
		if (ret)
			return ret;

		mutex_init(&achan->rx_lock);
		mutex_init(&achan->tx_lock);

		cl->dev = dev;

		achan->chan = mbox_request_channel(cl, 0);
		if (IS_ERR(achan->chan))
			return PTR_ERR(achan->chan);
	}

	return 0;
}

static void acpm_clk_pdev_unregister(void *data)
{
	platform_device_unregister(data);
}

static int acpm_timeout_debug_init(struct acpm_info *acpm)
{
	size_t words = ACPM_FW_LOG_DUMP_MAX_ENTRIES * ACPM_FW_LOG_ENTRY_WORDS;

	if (!acpm->timeout_debug)
		return 0;

	acpm->normal_log.name = "normal";
	acpm->normal_log.entries = devm_kcalloc(acpm->dev, words,
						 sizeof(*acpm->normal_log.entries),
						 GFP_KERNEL);
	if (!acpm->normal_log.entries)
		return -ENOMEM;

	acpm->preempt_log.name = "preempt";
	acpm->preempt_log.entries = devm_kcalloc(acpm->dev, words,
						  sizeof(*acpm->preempt_log.entries),
						  GFP_KERNEL);
	if (!acpm->preempt_log.entries)
		return -ENOMEM;

	return 0;
}

static const struct acpm_ops exynos_acpm_driver_ops = {
	.dvfs = {
		.set_rate = acpm_dvfs_set_rate,
		.get_rate = acpm_dvfs_get_rate,
	},

	.pmic = {
		.read_reg = acpm_pmic_read_reg,
		.bulk_read = acpm_pmic_bulk_read,
		.write_reg = acpm_pmic_write_reg,
		.bulk_write = acpm_pmic_bulk_write,
		.update_reg = acpm_pmic_update_reg,
	},

	.tmu = {
		.init = acpm_tmu_init,
		.read_temp = acpm_tmu_read_temp,
		.set_threshold = acpm_tmu_set_threshold,
		.set_interrupt_enable = acpm_tmu_set_interrupt_enable,
		.tz_control = acpm_tmu_tz_control,
		.clear_tz_irq = acpm_tmu_clear_tz_irq,
		.suspend = acpm_tmu_suspend,
		.resume = acpm_tmu_resume,
	},
};

static int acpm_probe(struct platform_device *pdev)
{
	const struct acpm_match_data *match_data;
	struct platform_device *acpm_clk_pdev;
	struct device *dev = &pdev->dev;
	struct device_node *shmem;
	struct acpm_info *acpm;
	resource_size_t size;
	struct resource res;
	int ret;

	acpm = devm_kzalloc(dev, sizeof(*acpm), GFP_KERNEL);
	if (!acpm)
		return -ENOMEM;

	shmem = of_parse_phandle(dev->of_node, "shmem", 0);
	ret = of_address_to_resource(shmem, 0, &res);
	of_node_put(shmem);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to get shared memory.\n");

	size = resource_size(&res);
	acpm->sram_base = devm_ioremap(dev, res.start, size);
	if (!acpm->sram_base)
		return dev_err_probe(dev, -ENOMEM,
				     "Failed to ioremap shared memory.\n");

	match_data = of_device_get_match_data(dev);
	if (!match_data)
		return dev_err_probe(dev, -EINVAL,
				     "Failed to get match data.\n");

	acpm->shmem = acpm->sram_base + match_data->initdata_base;
	acpm->dev = dev;
	acpm->sram_size = size;
	acpm->timeout_debug = match_data->timeout_debug;
	atomic_set(&acpm->timeout_claimed, 0);

	ret = acpm_timeout_debug_init(acpm);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to allocate timeout log snapshots.\n");

	ret = acpm_channels_init(acpm);
	if (ret)
		return ret;

	acpm->handle.ops = &exynos_acpm_driver_ops;

	platform_set_drvdata(pdev, acpm);

	acpm_clk_pdev = platform_device_register_data(dev,
						match_data->acpm_clk_dev_name,
						PLATFORM_DEVID_NONE, NULL, 0);
	if (IS_ERR(acpm_clk_pdev))
		return dev_err_probe(dev, PTR_ERR(acpm_clk_pdev),
				     "Failed to register ACPM clocks device.\n");

	ret = devm_add_action_or_reset(dev, acpm_clk_pdev_unregister,
				       acpm_clk_pdev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add devm action.\n");

	return devm_of_platform_populate(dev);
}

/**
 * acpm_handle_put() - release the handle acquired by acpm_get_by_phandle.
 * @handle:	Handle acquired by acpm_get_by_phandle.
 */
static void acpm_handle_put(struct acpm_handle *handle)
{
	struct acpm_info *acpm = handle_to_acpm_info(handle);
	struct device *dev = acpm->dev;

	module_put(dev->driver->owner);
	/* Drop reference taken with of_find_device_by_node(). */
	put_device(dev);
}

/**
 * devm_acpm_release() - devres release method.
 * @dev: pointer to device.
 * @res: pointer to resource.
 */
static void devm_acpm_release(struct device *dev, void *res)
{
	acpm_handle_put(*(struct acpm_handle **)res);
}

/**
 * acpm_get_by_node() - get the ACPM handle using node pointer.
 * @dev:	device pointer requesting ACPM handle.
 * @np:		ACPM device tree node.
 *
 * Return: pointer to handle on success, ERR_PTR(-errno) otherwise.
 *
 * Note: handle CANNOT be pointer to const
 */
static struct acpm_handle *acpm_get_by_node(struct device *dev,
					    struct device_node *np)
{
	struct platform_device *pdev;
	struct device_link *link;
	struct acpm_info *acpm;

	pdev = of_find_device_by_node(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	acpm = platform_get_drvdata(pdev);
	if (!acpm) {
		platform_device_put(pdev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	if (!try_module_get(pdev->dev.driver->owner)) {
		platform_device_put(pdev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	link = device_link_add(dev, &pdev->dev, DL_FLAG_AUTOREMOVE_SUPPLIER);
	if (!link) {
		dev_err(&pdev->dev,
			"Failed to create device link to consumer %s.\n",
			dev_name(dev));
		platform_device_put(pdev);
		module_put(pdev->dev.driver->owner);
		return ERR_PTR(-EINVAL);
	}

	return &acpm->handle;
}

/**
 * devm_acpm_get_by_node() - managed get handle using node pointer.
 * @dev: device pointer requesting ACPM handle.
 * @np:  ACPM device tree node.
 *
 * Return: pointer to handle on success, ERR_PTR(-errno) otherwise.
 */
struct acpm_handle *devm_acpm_get_by_node(struct device *dev,
					  struct device_node *np)
{
	struct acpm_handle **ptr, *handle;

	ptr = devres_alloc(devm_acpm_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	handle = acpm_get_by_node(dev, np);
	if (!IS_ERR(handle)) {
		*ptr = handle;
		devres_add(dev, ptr);
	} else {
		devres_free(ptr);
	}

	return handle;
}
EXPORT_SYMBOL_GPL(devm_acpm_get_by_node);

/**
 * devm_acpm_get_by_phandle - Resource managed lookup of the standardized
 * "samsung,acpm-ipc" handle.
 * @dev: consumer device
 *
 * Return: pointer to handle on success, ERR_PTR(-errno) otherwise.
 */
struct acpm_handle *devm_acpm_get_by_phandle(struct device *dev)
{
	struct acpm_handle *handle;
	struct device_node *np;

	np = of_parse_phandle(dev->of_node, "samsung,acpm-ipc", 0);
	if (!np)
		return ERR_PTR(-ENODEV);

	handle = devm_acpm_get_by_node(dev, np);
	of_node_put(np);

	return handle;
}
EXPORT_SYMBOL_GPL(devm_acpm_get_by_phandle);

static const struct acpm_match_data acpm_gs101 = {
	.initdata_base = ACPM_GS101_INITDATA_BASE,
	.acpm_clk_dev_name = "gs101-acpm-clk",
};

static const struct acpm_match_data acpm_zumapro = {
	/* Same SRAM channel-table offset as gs101, confirmed on silicon. */
	.initdata_base = ACPM_GS101_INITDATA_BASE,
	.acpm_clk_dev_name = "zumapro-acpm-clk",
	.timeout_debug = true,
};

static const struct of_device_id acpm_match[] = {
	{
		.compatible = "google,gs101-acpm-ipc",
		.data = &acpm_gs101,
	}, {
		.compatible = "google,zumapro-acpm-ipc",
		.data = &acpm_zumapro,
	},
	{},
};
MODULE_DEVICE_TABLE(of, acpm_match);

static struct platform_driver acpm_driver = {
	.probe	= acpm_probe,
	.driver	= {
		.name = "exynos-acpm-protocol",
		.of_match_table	= acpm_match,
	},
};
module_platform_driver(acpm_driver);

MODULE_AUTHOR("Tudor Ambarus <tudor.ambarus@linaro.org>");
MODULE_DESCRIPTION("Samsung Exynos ACPM mailbox protocol driver");
MODULE_LICENSE("GPL");
