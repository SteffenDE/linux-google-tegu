// SPDX-License-Identifier: GPL-2.0-only
/*
 * Synaptics TouchComm (TCM) protocol version 2 SPI touchscreen driver.
 *
 * The TouchComm message layer implemented here follows the Synaptics
 * reference implementation shipped in Google Pixel downstream kernels
 * (syna_tcm2 / synaptics_touchcom_core_v2.c):
 *
 *  - every message is a 4-byte header [code, length_le16, byte3] where
 *    byte3 carries the host bit (7), a sequence-toggle bit (6) and a
 *    CRC-6 over the first 26 header bits; a full received header always
 *    CRC-6s to zero,
 *  - commands with payload append a CRC-16 (CCITT-FALSE) over header plus
 *    payload; zero-payload commands are bare 4-byte writes,
 *  - the host fetches device messages with CMD_GET_REPORT; the response
 *    header announces the payload length, which is then drained in
 *    max_rd_size-bounded chunks (first chunk needs no ACK, later chunks
 *    are requested with CMD_ACK), each chunk CRC-16-framed,
 *  - the device echoes the host sequence bit in responses; asynchronous
 *    reports use codes >= 0x10 and the same transport.
 *
 * Touch reports are parsed with the device-provided touch report config
 * descriptor: a list of (entity code, bit length) pairs with FOREACH
 * loop markers, fields packed LSB-first in the report bitstream.
 *
 * The controller runs its firmware from internal flash; no firmware
 * download is required for operation.
 */

#include <linux/bitops.h>
#include <linux/crc-itu-t.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define SYNA_TCM_HEADER_SIZE		4
#define SYNA_TCM_CRC_SIZE		2
#define SYNA_TCM_HEADER_BITS		(SYNA_TCM_HEADER_SIZE * 8)

/* Largest message this driver drains (touch reports/configs are tiny). */
#define SYNA_TCM_MSG_MAX		1024
#define SYNA_TCM_OUT_MAX		32
#define SYNA_TCM_CONFIG_MAX		256

/*
 * Per-transfer bus ceiling.  Controllers may split a single transfer that
 * does not fit their FIFO into several bursts (spi-s3c64xx does so in
 * polling mode above its 64-byte FIFO), and nothing guarantees the chip
 * select stays asserted across the splits — a CS drop mid-message makes
 * the device restart its output at the message header.  Staying below
 * typical FIFO sizes keeps every read a single burst; larger payloads are
 * fetched through the protocol's own ACK-driven continued reads instead.
 */
#define SYNA_TCM_XFER_MAX		63

#define SYNA_TCM_RETRIES		5

/* Power sequencing, from the downstream tegu DT. */
#define SYNA_TCM_POWER_DELAY_MS		200
#define SYNA_TCM_RESET_ACTIVE_MS	2
#define SYNA_TCM_RESET_DELAY_MS		50
/* Bus turnaround between a command write and the response read. */
#define SYNA_TCM_TAT_DELAY_US		50

/* TouchComm commands */
#define TCM_CMD_IDENTIFY		0x02
#define TCM_CMD_RESET			0x04
#define TCM_CMD_TCM2_ACK		0x07
#define TCM_CMD_TCM2_GET_REPORT		0x0a
#define TCM_CMD_GET_APPLICATION_INFO	0x20
#define TCM_CMD_GET_TOUCH_REPORT_CONFIG	0x25
#define TCM_CMD_ENTER_DEEP_SLEEP	0x2c
#define TCM_CMD_EXIT_DEEP_SLEEP		0x2d

/* Status codes (< 0x10) and report codes (>= 0x10) */
#define TCM_STATUS_IDLE			0x00
#define TCM_STATUS_OK			0x01
#define TCM_STATUS_CONTINUED_READ	0x03
#define TCM_STATUS_NO_REPORT_AVAILABLE	0x04
#define TCM_STATUS_ACK			0x07
#define TCM_STATUS_RETRY_REQUESTED	0x08
#define TCM_REPORT_IDENTIFY		0x10
#define TCM_REPORT_TOUCH		0x11

/* IDENTIFY payload */
#define TCM_MODE_APPLICATION_FIRMWARE	0x01
#define TCM_MODE_ROMBOOTLOADER		0x04
#define TCM_MODE_BOOTLOADER		0x0b

#define TCM_V1_MARKER			0xa5

/* Touch report config entity codes */
#define TCM_TOUCH_END			0x00
#define TCM_TOUCH_FOREACH_ACTIVE_OBJECT	0x01
#define TCM_TOUCH_FOREACH_OBJECT	0x02
#define TCM_TOUCH_FOREACH_END		0x03
#define TCM_TOUCH_PAD_TO_NEXT_BYTE	0x04
#define TCM_TOUCH_OBJECT_N_INDEX	0x06
#define TCM_TOUCH_OBJECT_N_CLASSIFICATION 0x07
#define TCM_TOUCH_OBJECT_N_X_POSITION	0x08
#define TCM_TOUCH_OBJECT_N_Y_POSITION	0x09
#define TCM_TOUCH_OBJECT_N_Z		0x0a
#define TCM_TOUCH_NUM_OF_ACTIVE_OBJECTS	0x18

/* Object classifications */
#define TCM_OBJ_LIFT			0
#define TCM_OBJ_FINGER			1
#define TCM_OBJ_GLOVED_OBJECT		2

#define SYNA_TCM_MAX_SLOTS		10

struct syna_tcm_object {
	u8 status;
	u16 x;
	u16 y;
	u8 z;
};

struct syna_tcm {
	struct spi_device *spi;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[2];

	/* Serializes all bus message exchanges. */
	struct mutex io_lock;

	u8 seq;
	u16 max_rd_size;

	u16 max_x;
	u16 max_y;
	u16 max_objects;

	/* Last received message: header + reassembled payload. */
	u8 *msg;
	u32 payload_len;
	u8 code;

	/* Raw bus transfer buffers (DMA-safe). */
	u8 *rx;
	u8 *tx;

	u8 config[SYNA_TCM_CONFIG_MAX];
	u32 config_len;

	struct syna_tcm_object objects[SYNA_TCM_MAX_SLOTS];
	unsigned long slots_seen;
};

/*
 * CRC-6 over the leading 'bits' of the buffer, MSB first, as used for the
 * TouchComm v2 header.  Transcribed from the Synaptics reference
 * implementation (synaptics_touchcom_core_dev.h); a fully received header
 * including its CRC field computes to 0.
 */
static u8 syna_tcm_crc6(const u8 *p, unsigned int bits)
{
	static const u16 crc6_table[16] = {
		0,  268,  536,  788, 1072, 1340, 1576, 1828,
		2144, 2412, 2680, 2932, 3152, 3420, 3656, 3908
	};
	u16 r = 0x003f << 2;
	u16 x;

	for (; bits > 8; bits -= 8) {
		r ^= *p++;
		r = (r << 4) ^ crc6_table[r >> 4];
		r = (r << 4) ^ crc6_table[r >> 4];
	}

	if (bits > 0) {
		x = *p;
		while (bits--) {
			if (x & 0x80)
				r ^= 0x80;

			x <<= 1;
			r <<= 1;
			if (r & 0x100)
				r ^= (0x03 << 2);
		}
	}

	return (r >> 2) & 0x3f;
}

static int syna_tcm_spi_read(struct syna_tcm *ts, unsigned int len)
{
	struct spi_transfer xfer = {
		.rx_buf	= ts->rx,
		.len	= len,
	};

	return spi_sync_transfer(ts->spi, &xfer, 1);
}

static int syna_tcm_spi_write(struct syna_tcm *ts, unsigned int len)
{
	struct spi_transfer xfer = {
		.tx_buf	= ts->tx,
		.len	= len,
	};

	return spi_sync_transfer(ts->spi, &xfer, 1);
}

/*
 * Send one command packet.  The sequence toggle advances for new commands
 * and is reused verbatim on retries, so the device can detect resends.
 */
static int syna_tcm_send(struct syna_tcm *ts, u8 command, const u8 *payload,
			 u16 len, bool resend)
{
	unsigned int total = SYNA_TCM_HEADER_SIZE;
	u16 crc;

	if (len > SYNA_TCM_OUT_MAX - SYNA_TCM_HEADER_SIZE - SYNA_TCM_CRC_SIZE)
		return -EINVAL;

	if (!resend)
		ts->seq++;

	ts->tx[0] = command;
	put_unaligned_le16(len, &ts->tx[1]);
	ts->tx[3] = (ts->seq & 0x01) << 6;
	ts->tx[3] |= syna_tcm_crc6(ts->tx, SYNA_TCM_HEADER_BITS - 6);

	if (len) {
		memcpy(&ts->tx[SYNA_TCM_HEADER_SIZE], payload, len);
		total += len;
		crc = crc_itu_t(0xffff, ts->tx, total);
		ts->tx[total] = crc >> 8;
		ts->tx[total + 1] = crc & 0xff;
		total += SYNA_TCM_CRC_SIZE;
	}

	return syna_tcm_spi_write(ts, total);
}

/*
 * Read one packet: a 4-byte header plus, when payload_len is non-zero, the
 * payload and its trailing CRC-16.  Validates the header CRC-6, the echoed
 * sequence bit and the packet CRC-16; protocol-level corruption is reported
 * as -EBADMSG so callers can retry.
 */
static int syna_tcm_read_chunk(struct syna_tcm *ts, unsigned int payload_len,
			       bool check_seq)
{
	unsigned int xfer_len = SYNA_TCM_HEADER_SIZE;
	unsigned int hdr_len;
	int error;

	if (payload_len)
		xfer_len += payload_len + SYNA_TCM_CRC_SIZE;

	if (xfer_len > SYNA_TCM_XFER_MAX)
		return -EINVAL;

	error = syna_tcm_spi_read(ts, xfer_len);
	if (error)
		return error;

	if (syna_tcm_crc6(ts->rx, SYNA_TCM_HEADER_BITS) != 0)
		return -EBADMSG;

	if (check_seq && ((ts->rx[3] >> 6) & 0x01) != (ts->seq & 0x01))
		return -EBADMSG;

	hdr_len = get_unaligned_le16(&ts->rx[1]);
	if (payload_len && hdr_len &&
	    crc_itu_t(0xffff, ts->rx, xfer_len) != 0)
		return -EBADMSG;

	ts->code = ts->rx[0];
	ts->payload_len = hdr_len;

	return 0;
}

/*
 * Read the device's pending message into ts->msg: header first, then the
 * announced payload in chunks.  The first chunk follows the header read
 * without an ACK; further chunks are requested with CMD_ACK.  Mirrors the
 * reference syna_tcm_v2_get_response()/syna_tcm_v2_continued_read().
 */
static int syna_tcm_get_response(struct syna_tcm *ts)
{
	unsigned int chunk_space, offset, remaining, chunk;
	unsigned int total;
	int retries;
	int error;
	u8 code;

	error = syna_tcm_read_chunk(ts, 0, true);
	if (error)
		return error;

	memcpy(ts->msg, ts->rx, SYNA_TCM_HEADER_SIZE);
	code = ts->code;
	total = ts->payload_len;

	if (!total)
		return 0;

	/*
	 * Oversized messages are still drained chunk by chunk (just
	 * truncated in ts->msg): leaving them pending would keep the
	 * level-triggered ATTN asserted forever.
	 */
	if (total > SYNA_TCM_MSG_MAX - SYNA_TCM_HEADER_SIZE)
		dev_warn_ratelimited(&ts->spi->dev,
				     "truncating oversized message %#x len %u\n",
				     code, total);

	chunk_space = SYNA_TCM_XFER_MAX;
	if (ts->max_rd_size)
		chunk_space = min_t(unsigned int, chunk_space, ts->max_rd_size);
	chunk_space -= SYNA_TCM_HEADER_SIZE + SYNA_TCM_CRC_SIZE;

	offset = SYNA_TCM_HEADER_SIZE;
	remaining = total;

	while (remaining) {
		chunk = min(remaining, chunk_space);

		retries = 0;
		do {
			/* The very first chunk streams without an ACK. */
			if (offset > SYNA_TCM_HEADER_SIZE || retries) {
				error = syna_tcm_send(ts, TCM_CMD_TCM2_ACK,
						      NULL, 0, retries > 0);
				if (error)
					return error;
				usleep_range(SYNA_TCM_TAT_DELAY_US,
					     2 * SYNA_TCM_TAT_DELAY_US);
			}

			error = syna_tcm_read_chunk(ts, chunk, true);
		} while (error == -EBADMSG && ++retries < SYNA_TCM_RETRIES);

		if (error)
			return error;

		if (offset + chunk <= SYNA_TCM_MSG_MAX)
			memcpy(&ts->msg[offset], &ts->rx[SYNA_TCM_HEADER_SIZE],
			       chunk);
		offset += chunk;
		remaining -= chunk;
	}

	ts->code = code;
	ts->payload_len = min(total, SYNA_TCM_MSG_MAX - SYNA_TCM_HEADER_SIZE);

	return 0;
}

/*
 * Send a command and collect the message the device answers with.  On
 * return ts->code holds either a status code (response consumed) or a
 * report code >= 0x10 (an asynchronous report arrived instead; the
 * command response will follow on a later CMD_GET_REPORT).
 */
static int syna_tcm_exchange(struct syna_tcm *ts, u8 command,
			     const u8 *payload, u16 len)
{
	int retries = 0;
	int error;

	do {
		error = syna_tcm_send(ts, command, payload, len, retries > 0);
		if (error)
			return error;

		usleep_range(SYNA_TCM_TAT_DELAY_US, 2 * SYNA_TCM_TAT_DELAY_US);

		error = syna_tcm_get_response(ts);
		if (error && error != -EBADMSG)
			return error;

		if (!error) {
			switch (ts->code) {
			case TCM_STATUS_OK:
			case TCM_STATUS_ACK:
			case TCM_STATUS_NO_REPORT_AVAILABLE:
				return 0;
			case TCM_STATUS_RETRY_REQUESTED:
				break;
			default:
				if (ts->code >= TCM_REPORT_IDENTIFY)
					return 0;
				dev_dbg(&ts->spi->dev,
					"command %#x failed, status %#x\n",
					command, ts->code);
				return -EPROTO;
			}
		}
	} while (++retries < SYNA_TCM_RETRIES);

	return -ETIMEDOUT;
}

/* LSB-first bit field extraction; reads beyond the report yield zero. */
static u32 syna_tcm_get_bits(const u8 *report, unsigned int report_bits,
			     unsigned int offset, unsigned int bits)
{
	u32 value = 0;
	unsigned int i;

	if (offset + bits > report_bits)
		return 0;

	for (i = 0; i < bits; i++) {
		unsigned int pos = offset + i;

		value |= ((report[pos / 8] >> (pos % 8)) & 1) << i;
	}

	return value;
}

/*
 * Skip config entries up to and including the next FOREACH_END.  Entity
 * codes carry a bit-length byte; flow-control codes do not.
 */
static unsigned int syna_tcm_skip_foreach(struct syna_tcm *ts,
					  unsigned int idx)
{
	while (idx < ts->config_len) {
		u8 code = ts->config[idx++];

		switch (code) {
		case TCM_TOUCH_FOREACH_END:
			return idx;
		case TCM_TOUCH_END:
			return idx - 1;
		case TCM_TOUCH_FOREACH_ACTIVE_OBJECT:
		case TCM_TOUCH_FOREACH_OBJECT:
		case TCM_TOUCH_PAD_TO_NEXT_BYTE:
			break;
		default:
			idx++;	/* skip the bit-length byte */
			break;
		}
	}

	return idx;
}

static void syna_tcm_report_frame(struct syna_tcm *ts)
{
	struct input_dev *input = ts->input;
	unsigned int i;

	for_each_set_bit(i, &ts->slots_seen, SYNA_TCM_MAX_SLOTS) {
		struct syna_tcm_object *obj = &ts->objects[i];
		bool active = obj->status == TCM_OBJ_FINGER ||
			      obj->status == TCM_OBJ_GLOVED_OBJECT;

		input_mt_slot(input, i);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, active);
		if (active) {
			input_report_abs(input, ABS_MT_POSITION_X, obj->x);
			input_report_abs(input, ABS_MT_POSITION_Y, obj->y);
		}
	}

	input_mt_sync_frame(input);
	input_sync(input);
}

/*
 * Walk the touch report against the device's report config descriptor.
 * Follows the reference syna_tcm_parse_touch_report() loop semantics.
 */
static void syna_tcm_parse_touch(struct syna_tcm *ts, const u8 *report,
				 unsigned int len)
{
	unsigned int report_bits = len * 8;
	unsigned int offset = 0;
	unsigned int idx = 0;
	unsigned int loop_start = 0;
	unsigned int active_objects = 0;
	unsigned int objects = 0;
	unsigned int obj = 0;
	bool have_active_count = false;
	bool active_only = false;
	unsigned int budget = 8 * SYNA_TCM_CONFIG_MAX;
	u8 code, bits;
	u32 data;

	ts->slots_seen = 0;

	while (idx < ts->config_len && budget--) {
		code = ts->config[idx++];

		switch (code) {
		case TCM_TOUCH_END:
			goto done;
		case TCM_TOUCH_FOREACH_ACTIVE_OBJECT:
			obj = 0;
			loop_start = idx;
			active_only = true;
			break;
		case TCM_TOUCH_FOREACH_OBJECT:
			obj = 0;
			loop_start = idx;
			active_only = false;
			break;
		case TCM_TOUCH_FOREACH_END:
			if (offset >= report_bits)
				break;
			if (active_only) {
				if (have_active_count) {
					objects++;
					obj++;
					if (objects < active_objects)
						idx = loop_start;
				} else {
					obj++;
					idx = loop_start;
				}
			} else {
				obj++;
				if (obj < ts->max_objects)
					idx = loop_start;
			}
			break;
		case TCM_TOUCH_PAD_TO_NEXT_BYTE:
			offset = round_up(offset, 8);
			break;
		case TCM_TOUCH_NUM_OF_ACTIVE_OBJECTS:
			bits = ts->config[idx++];
			active_objects = syna_tcm_get_bits(report, report_bits,
							   offset, bits);
			have_active_count = true;
			offset += bits;
			if (!active_objects)
				idx = syna_tcm_skip_foreach(ts, idx);
			break;
		case TCM_TOUCH_OBJECT_N_INDEX:
			bits = ts->config[idx++];
			obj = syna_tcm_get_bits(report, report_bits,
						offset, bits);
			offset += bits;
			break;
		case TCM_TOUCH_OBJECT_N_CLASSIFICATION:
			bits = ts->config[idx++];
			data = syna_tcm_get_bits(report, report_bits,
						 offset, bits);
			offset += bits;
			if (obj < SYNA_TCM_MAX_SLOTS) {
				ts->objects[obj].status = data;
				ts->slots_seen |= BIT(obj);
			}
			break;
		case TCM_TOUCH_OBJECT_N_X_POSITION:
			bits = ts->config[idx++];
			data = syna_tcm_get_bits(report, report_bits,
						 offset, bits);
			offset += bits;
			if (obj < SYNA_TCM_MAX_SLOTS)
				ts->objects[obj].x = data;
			break;
		case TCM_TOUCH_OBJECT_N_Y_POSITION:
			bits = ts->config[idx++];
			data = syna_tcm_get_bits(report, report_bits,
						 offset, bits);
			offset += bits;
			if (obj < SYNA_TCM_MAX_SLOTS)
				ts->objects[obj].y = data;
			break;
		case TCM_TOUCH_OBJECT_N_Z:
			bits = ts->config[idx++];
			data = syna_tcm_get_bits(report, report_bits,
						 offset, bits);
			offset += bits;
			if (obj < SYNA_TCM_MAX_SLOTS)
				ts->objects[obj].z = data;
			break;
		default:
			/* Unknown data entity: skip its bit field. */
			bits = ts->config[idx++];
			offset += bits;
			break;
		}
	}

done:
	syna_tcm_report_frame(ts);
}

static irqreturn_t syna_tcm_irq_thread(int irq, void *data)
{
	struct syna_tcm *ts = data;
	int error;

	mutex_lock(&ts->io_lock);

	error = syna_tcm_exchange(ts, TCM_CMD_TCM2_GET_REPORT, NULL, 0);
	if (error)
		goto out;

	switch (ts->code) {
	case TCM_REPORT_TOUCH:
		syna_tcm_parse_touch(ts, &ts->msg[SYNA_TCM_HEADER_SIZE],
				     ts->payload_len);
		break;
	case TCM_REPORT_IDENTIFY:
		/* Unsolicited identify means the controller reset itself. */
		dev_warn(&ts->spi->dev, "unexpected device reset\n");
		break;
	default:
		break;
	}

out:
	mutex_unlock(&ts->io_lock);

	return IRQ_HANDLED;
}

static void syna_tcm_power_off(void *data)
{
	struct syna_tcm *ts = data;

	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ts->supplies), ts->supplies);
}

static int syna_tcm_power_on(struct syna_tcm *ts)
{
	int error;

	error = regulator_bulk_enable(ARRAY_SIZE(ts->supplies), ts->supplies);
	if (error)
		return error;

	msleep(SYNA_TCM_POWER_DELAY_MS);

	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	msleep(SYNA_TCM_RESET_ACTIVE_MS);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(SYNA_TCM_RESET_DELAY_MS);

	return 0;
}

/*
 * After reset the device queues an IDENTIFY report and asserts ATTN; the
 * first raw header read tells the protocol generation apart: a TouchComm
 * v2 header CRC-6s to zero, a v1 header leads with the 0xa5 marker.
 */
static int syna_tcm_detect(struct syna_tcm *ts)
{
	struct device *dev = &ts->spi->dev;
	int tries;
	int error;

	for (tries = 0; tries < SYNA_TCM_RETRIES; tries++) {
		error = syna_tcm_spi_read(ts, SYNA_TCM_HEADER_SIZE);
		if (error)
			return error;

		if (ts->rx[0] == TCM_V1_MARKER) {
			dev_err(dev, "TouchComm v1 device, not supported\n");
			return -ENODEV;
		}

		if (syna_tcm_crc6(ts->rx, SYNA_TCM_HEADER_BITS) == 0 &&
		    ts->rx[0] == TCM_REPORT_IDENTIFY)
			break;

		msleep(20);
	}

	if (tries == SYNA_TCM_RETRIES) {
		dev_err(dev, "no TouchComm v2 identify header (%*ph)\n",
			SYNA_TCM_HEADER_SIZE, ts->rx);
		return -ENODEV;
	}

	/* Fetch the identification data through the regular transport. */
	error = syna_tcm_exchange(ts, TCM_CMD_TCM2_GET_REPORT, NULL, 0);
	if (error)
		return error;

	if (ts->code != TCM_REPORT_IDENTIFY) {
		error = syna_tcm_exchange(ts, TCM_CMD_IDENTIFY, NULL, 0);
		if (error)
			return error;
		if (ts->code != TCM_STATUS_OK)
			return -EPROTO;
	}

	if (ts->payload_len < 24)
		return -EPROTO;

	if (ts->msg[SYNA_TCM_HEADER_SIZE + 1] != TCM_MODE_APPLICATION_FIRMWARE) {
		dev_err(dev, "device not in application fw mode (mode %#x)\n",
			ts->msg[SYNA_TCM_HEADER_SIZE + 1]);
		return -ENODEV;
	}

	if (ts->payload_len >= 26)
		ts->max_rd_size =
			get_unaligned_le16(&ts->msg[SYNA_TCM_HEADER_SIZE + 24]);

	dev_info(dev, "TouchComm v2, part '%.16s', build %u\n",
		 (char *)&ts->msg[SYNA_TCM_HEADER_SIZE + 2],
		 get_unaligned_le32(&ts->msg[SYNA_TCM_HEADER_SIZE + 18]));

	return 0;
}

static int syna_tcm_setup_app(struct syna_tcm *ts)
{
	struct device *dev = &ts->spi->dev;
	const u8 *info;
	int error;

	error = syna_tcm_exchange(ts, TCM_CMD_GET_APPLICATION_INFO, NULL, 0);
	if (error)
		return error;
	if (ts->code != TCM_STATUS_OK || ts->payload_len < 38)
		return -EPROTO;

	info = &ts->msg[SYNA_TCM_HEADER_SIZE];
	ts->max_x = get_unaligned_le16(&info[32]);
	ts->max_y = get_unaligned_le16(&info[34]);
	ts->max_objects = get_unaligned_le16(&info[36]);

	if (!ts->max_x || !ts->max_y || !ts->max_objects)
		return -ENODEV;

	ts->max_objects = min_t(u16, ts->max_objects, SYNA_TCM_MAX_SLOTS);

	error = syna_tcm_exchange(ts, TCM_CMD_GET_TOUCH_REPORT_CONFIG,
				  NULL, 0);
	if (error)
		return error;
	if (ts->code != TCM_STATUS_OK || !ts->payload_len ||
	    ts->payload_len > SYNA_TCM_CONFIG_MAX)
		return -EPROTO;

	memcpy(ts->config, &ts->msg[SYNA_TCM_HEADER_SIZE], ts->payload_len);
	ts->config_len = ts->payload_len;

	dev_dbg(dev, "%ux%u, %u objects, %u-byte report config\n",
		ts->max_x, ts->max_y, ts->max_objects, ts->config_len);

	return 0;
}

static int syna_tcm_setup_input(struct syna_tcm *ts)
{
	struct input_dev *input;
	int error;

	input = devm_input_allocate_device(&ts->spi->dev);
	if (!input)
		return -ENOMEM;

	input->name = "Synaptics TouchComm Touchscreen";
	input->id.bustype = BUS_SPI;
	input->phys = "syna_tcm/input0";

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);

	error = input_mt_init_slots(input, ts->max_objects,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error)
		return error;

	error = input_register_device(input);
	if (error)
		return error;

	ts->input = input;

	return 0;
}

static int syna_tcm_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct syna_tcm *ts;
	int error;

	if (!spi->irq) {
		dev_err(dev, "no irq specified\n");
		return -EINVAL;
	}

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->msg = devm_kzalloc(dev, SYNA_TCM_MSG_MAX, GFP_KERNEL);
	ts->rx = devm_kzalloc(dev, SYNA_TCM_MSG_MAX, GFP_KERNEL);
	ts->tx = devm_kzalloc(dev, SYNA_TCM_OUT_MAX, GFP_KERNEL);
	if (!ts->msg || !ts->rx || !ts->tx)
		return -ENOMEM;

	ts->spi = spi;
	mutex_init(&ts->io_lock);
	spi_set_drvdata(spi, ts);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	error = spi_setup(spi);
	if (error)
		return error;

	ts->supplies[0].supply = "vdd";
	ts->supplies[1].supply = "avdd";
	error = devm_regulator_bulk_get(dev, ARRAY_SIZE(ts->supplies),
					ts->supplies);
	if (error)
		return dev_err_probe(dev, error, "failed to get supplies\n");

	ts->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset gpio\n");

	error = syna_tcm_power_on(ts);
	if (error)
		return dev_err_probe(dev, error, "failed to power on\n");

	error = devm_add_action_or_reset(dev, syna_tcm_power_off, ts);
	if (error)
		return error;

	error = syna_tcm_detect(ts);
	if (error)
		return error;

	error = syna_tcm_setup_app(ts);
	if (error)
		return dev_err_probe(dev, error, "application setup failed\n");

	error = syna_tcm_setup_input(ts);
	if (error)
		return error;

	error = devm_request_threaded_irq(dev, spi->irq, NULL,
					  syna_tcm_irq_thread,
					  IRQF_ONESHOT, "syna_tcm", ts);
	if (error)
		return dev_err_probe(dev, error, "failed to request irq\n");

	return 0;
}

static int syna_tcm_suspend(struct device *dev)
{
	struct syna_tcm *ts = spi_get_drvdata(to_spi_device(dev));
	int error;

	disable_irq(ts->spi->irq);

	mutex_lock(&ts->io_lock);
	error = syna_tcm_exchange(ts, TCM_CMD_ENTER_DEEP_SLEEP, NULL, 0);
	mutex_unlock(&ts->io_lock);
	if (error)
		dev_warn(dev, "failed to enter deep sleep: %d\n", error);

	return 0;
}

static int syna_tcm_resume(struct device *dev)
{
	struct syna_tcm *ts = spi_get_drvdata(to_spi_device(dev));
	int error;

	mutex_lock(&ts->io_lock);
	error = syna_tcm_exchange(ts, TCM_CMD_EXIT_DEEP_SLEEP, NULL, 0);
	mutex_unlock(&ts->io_lock);
	if (error)
		dev_warn(dev, "failed to exit deep sleep: %d\n", error);

	enable_irq(ts->spi->irq);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(syna_tcm_pm_ops,
				syna_tcm_suspend, syna_tcm_resume);

static const struct of_device_id syna_tcm_of_match[] = {
	{ .compatible = "syna,tcm-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, syna_tcm_of_match);

static const struct spi_device_id syna_tcm_spi_id[] = {
	{ "tcm-spi" },
	{ }
};
MODULE_DEVICE_TABLE(spi, syna_tcm_spi_id);

static struct spi_driver syna_tcm_driver = {
	.driver = {
		.name		= "syna_tcm",
		.of_match_table	= syna_tcm_of_match,
		.pm		= pm_sleep_ptr(&syna_tcm_pm_ops),
	},
	.id_table	= syna_tcm_spi_id,
	.probe		= syna_tcm_probe,
};
module_spi_driver(syna_tcm_driver);

MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_DESCRIPTION("Synaptics TouchComm v2 SPI touchscreen driver");
MODULE_LICENSE("GPL");
