// SPDX-License-Identifier: GPL-2.0
/*
 * Touch Bus Negotiator (TBN) for Google Pixel devices.
 *
 * Arbitrates ownership of the touch SPI bus between the AP and the AoC
 * coprocessor, and delivers low-power touch-wake (LPTW) gesture events that
 * AoC detects while it owns the bus.  When the display turns off the touch
 * driver releases the bus to AoC (tbn_release_bus); on resume it reclaims it
 * (tbn_request_bus).  A tap detected by AoC arrives as a TbnLptwEvent over the
 * com.google.tbn_service AoC channel and is handed to the registered callback.
 *
 * Ported from the downstream google-modules/touch touch_bus_negotiator; only
 * the AoC-channel mode (tbn,mode = 2) used by tegu is implemented.  The
 * reference GPIO and MOCK arbitration modes are dropped.
 *
 * Copyright (C) 2021 Google, Inc.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include "touch_bus_negotiator.h"
/* Exported by drivers/soc/google/aoc/aoc_tbn_service_dev.c (added to the
 * include path for this object in the Makefile). */
#include "aoc_tbn_service_dev.h"

#define TBN_MODULE_NAME "touch_bus_negotiator"
#define TBN_AOC_CHANNEL_THREAD_NAME "tbn_aoc_channel"

#undef pr_fmt
#define pr_fmt(fmt) "gti: tbn: " fmt

static struct tbn_context *tbn_context;

static void handle_tbn_event_response(struct tbn_context *tbn,
				      struct TbnEventResponse *response);

static int aoc_channel_kthread(void *data)
{
	struct tbn_context *tbn = data;
	struct TbnEventHeader header;
	ssize_t len;
	bool service_ready = false;

	while (!kthread_should_stop()) {
		if (service_ready != aoc_tbn_service_ready()) {
			service_ready = !service_ready;
			pr_info("%s: AOC TBN service is %s.\n",
				__func__, service_ready ? "ready" : "not ready");
		}

		if (!service_ready) {
			msleep(1000);
			continue;
		}

		len = aoc_tbn_service_read(&header, sizeof(header));
		if (len < 0) {
			pr_err("%s: failed to read message, err: %zd\n",
			       __func__, len);
			msleep(1000);
			continue;
		}

		if (kthread_should_stop())
			break;

		if (header.operation == TBN_OPERATION_AOC_RESET) {
			if (tbn->event_wq)
				queue_work(tbn->event_wq, &tbn->aoc_reset_work);
		} else if (header.operation == TBN_OPERATION_AOC_SEND_LPTW_EVENT) {
			struct TbnLptwEvent *gesture;

			if (len != sizeof(*gesture)) {
				pr_err("%s: Abnormal gesture data length: %zd\n",
				       __func__, len);
				continue;
			}
			gesture = (struct TbnLptwEvent *)&header;
			pr_info("%s: LPTW event, x=%u y=%u major=%u minor=%u angle=%d\n",
				__func__, gesture->x, gesture->y, gesture->major,
				gesture->minor, gesture->angle);
			if (tbn->lptw_event_cb)
				tbn->lptw_event_cb(gesture, tbn->lptw_event_cbdata);
		} else {
			struct TbnEventResponse *resp;

			if (len != sizeof(*resp)) {
				pr_err("%s: Abnormal resp data length: %zd\n",
				       __func__, len);
				continue;
			}
			resp = (struct TbnEventResponse *)&header;
			handle_tbn_event_response(tbn, resp);
		}
	}

	return 0;
}

static void handle_tbn_event_response(struct tbn_context *tbn,
				      struct TbnEventResponse *response)
{
	mutex_lock(&tbn->event_lock);

	if (response->id != tbn->event.id) {
		pr_err("%s: receive wrong response, id: %d, expected id: %d, "
		       "bus_released:%d bus_requested:%d.\n",
		       __func__, response->id, tbn->event.id,
		       completion_done(&tbn->bus_released),
		       completion_done(&tbn->bus_requested));
		goto exit;
	}

	if (response->err != 0) {
		pr_err("%s: send tbn event failed, err %d!\n",
		       __func__, response->err);
		tbn->event_resp.err = response->err;
	} else {
		tbn->event_resp.lptw_triggered = response->lptw_triggered;
	}

	if (response->operation == TBN_OPERATION_AP_REQUEST_BUS)
		complete_all(&tbn->bus_requested);
	else if (response->operation == TBN_OPERATION_AP_RELEASE_BUS)
		complete_all(&tbn->bus_released);
	else
		pr_err("%s: response unknown operation, op: %d!\n",
		       __func__, response->operation);

exit:
	mutex_unlock(&tbn->event_lock);
}

static void send_tbn_event(struct tbn_context *tbn, enum TbnOperation operation)
{
	ssize_t len;
	int retry = 3;

	if (!aoc_tbn_service_ready()) {
		pr_err("%s: AOC TBN service is not ready.\n", __func__);
		return;
	}

	mutex_lock(&tbn->event_lock);

	tbn->event.operation = operation;
	tbn->event.id++;

	while (retry) {
		len = aoc_tbn_service_write(&tbn->event, sizeof(tbn->event));
		if (len == sizeof(tbn->event))
			break;
		pr_err("%s: failed to send TBN event, retry: %d.\n",
		       __func__, retry);
		retry--;
	}

	mutex_unlock(&tbn->event_lock);
}

static int tbn_handshaking(struct tbn_context *tbn, enum TbnOperation operation)
{
	struct completion *wait_for_completion;
	unsigned int timeout;
	const char *msg;
	int ret = 0;

	if (!tbn || tbn->registered_mask == 0) {
		pr_err("%s: tbn is not ready to serve.\n", __func__);
		return -EINVAL;
	}

	if (operation == TBN_OPERATION_AP_REQUEST_BUS) {
		wait_for_completion = &tbn->bus_requested;
		timeout = TBN_REQUEST_BUS_TIMEOUT_MS;
		msg = "request";
	} else if (operation == TBN_OPERATION_AP_RELEASE_BUS) {
		wait_for_completion = &tbn->bus_released;
		timeout = TBN_RELEASE_BUS_TIMEOUT_MS;
		msg = "release";
	} else {
		pr_err("%s: request unknown operation, op: %d.\n",
		       __func__, operation);
		return -EINVAL;
	}

	if (tbn->mode != TBN_MODE_AOC_CHANNEL)
		return -EINVAL;

	tbn->event_resp.lptw_triggered = false;
	tbn->event_resp.err = 0;

	reinit_completion(wait_for_completion);

	send_tbn_event(tbn, operation);
	if (wait_for_completion_timeout(wait_for_completion,
					msecs_to_jiffies(timeout)) == 0) {
		pr_err("AP %s bus ... timeout!\n", msg);
		complete_all(wait_for_completion);
		ret = -ETIMEDOUT;
	} else if (tbn->event_resp.err == 0) {
		pr_info("AP %s bus ... SUCCESS!\n", msg);
	} else {
		pr_info("AP %s bus ... failed!\n", msg);
		ret = -EBUSY;
	}

	return ret;
}

int tbn_request_bus_with_result(u32 dev_mask, bool *lptw_triggered)
{
	int ret = 0;

	if (!tbn_context)
		return -ENODEV;

	mutex_lock(&tbn_context->dev_mask_mutex);

	if ((dev_mask & tbn_context->registered_mask) == 0) {
		mutex_unlock(&tbn_context->dev_mask_mutex);
		pr_err("%s: dev_mask %#x is invalid.\n", __func__, dev_mask);
		return -EINVAL;
	}

	if (tbn_context->requested_dev_mask == 0) {
		ret = tbn_handshaking(tbn_context, TBN_OPERATION_AP_REQUEST_BUS);
		if (ret == 0 && lptw_triggered != NULL)
			*lptw_triggered = tbn_context->event_resp.lptw_triggered;
	} else {
		dev_dbg(tbn_context->dev,
			"%s: Bus already requested, requested_dev_mask %#x dev_mask %#x.\n",
			__func__, tbn_context->requested_dev_mask, dev_mask);
	}
	tbn_context->requested_dev_mask |= dev_mask;

	mutex_unlock(&tbn_context->dev_mask_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(tbn_request_bus_with_result);

int tbn_request_bus(u32 dev_mask)
{
	return tbn_request_bus_with_result(dev_mask, NULL);
}
EXPORT_SYMBOL_GPL(tbn_request_bus);

int tbn_release_bus(u32 dev_mask)
{
	int ret = 0;

	if (!tbn_context)
		return -ENODEV;

	mutex_lock(&tbn_context->dev_mask_mutex);

	if ((dev_mask & tbn_context->registered_mask) == 0) {
		mutex_unlock(&tbn_context->dev_mask_mutex);
		pr_err("%s: dev_mask %#x is invalid.\n", __func__, dev_mask);
		return -EINVAL;
	}

	if (tbn_context->requested_dev_mask == 0) {
		pr_warn("%s: Bus already released, dev_mask %#x.\n",
			__func__, dev_mask);
		mutex_unlock(&tbn_context->dev_mask_mutex);
		return 0;
	}

	/* Release the bus when the last requested_dev_mask bit releases. */
	if (tbn_context->requested_dev_mask == dev_mask)
		ret = tbn_handshaking(tbn_context, TBN_OPERATION_AP_RELEASE_BUS);
	else
		dev_dbg(tbn_context->dev,
			"%s: Bus is still in use, requested_dev_mask %#x dev_mask %#x.\n",
			__func__, tbn_context->requested_dev_mask, dev_mask);

	tbn_context->requested_dev_mask &= ~dev_mask;

	mutex_unlock(&tbn_context->dev_mask_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(tbn_release_bus);

int register_tbn(u32 *output)
{
	u32 i;

	*output = 0;

	if (!tbn_context) {
		pr_warn("%s: tbn_context doesn't exist.\n", __func__);
		return 0;
	}

	mutex_lock(&tbn_context->dev_mask_mutex);
	for (i = 0; i < tbn_context->max_devices; i++) {
		if (tbn_context->registered_mask & BIT_MASK(i))
			continue;
		tbn_context->registered_mask |= BIT_MASK(i);
		/* Assume screen is on while registering tbn. */
		tbn_context->requested_dev_mask |= BIT_MASK(i);
		*output = BIT_MASK(i);
		break;
	}
	mutex_unlock(&tbn_context->dev_mask_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(register_tbn);

void register_tbn_lptw_callback(void (*callback)(struct TbnLptwEvent *lptw,
						 void *user_data),
				void *cbdata)
{
	if (!tbn_context)
		return;

	tbn_context->lptw_event_cb = callback;
	tbn_context->lptw_event_cbdata = cbdata;
}
EXPORT_SYMBOL_GPL(register_tbn_lptw_callback);

static void tbn_aoc_reset_work(struct work_struct *work)
{
	struct tbn_context *tbn = container_of(work, struct tbn_context,
					       aoc_reset_work);

	pr_warn("%s: AOC has been reset\n", __func__);
	if (tbn->requested_dev_mask == 0)
		tbn_handshaking(tbn, TBN_OPERATION_AP_RELEASE_BUS);
	else
		tbn_handshaking(tbn, TBN_OPERATION_AP_REQUEST_BUS);
}

void unregister_tbn(u32 *output)
{
	if (!tbn_context)
		return;

	mutex_lock(&tbn_context->dev_mask_mutex);
	tbn_context->registered_mask &= ~(*output);
	*output = 0;
	mutex_unlock(&tbn_context->dev_mask_mutex);
}
EXPORT_SYMBOL_GPL(unregister_tbn);

static int tbn_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct tbn_context *tbn;
	int err;

	tbn = devm_kzalloc(dev, sizeof(*tbn), GFP_KERNEL);
	if (!tbn)
		return -ENOMEM;

	tbn->dev = dev;
	tbn->event_resp.lptw_triggered = false;
	dev_set_drvdata(dev, tbn);

	if (of_property_read_u32(np, "tbn,max_devices", &tbn->max_devices))
		tbn->max_devices = 1;

	if (of_property_read_u32(np, "tbn,mode", &tbn->mode))
		tbn->mode = TBN_MODE_AOC_CHANNEL;

	if (tbn->mode != TBN_MODE_AOC_CHANNEL) {
		dev_err(dev,
			"unsupported tbn,mode %u (only AoC channel is implemented)\n",
			tbn->mode);
		return -EINVAL;
	}

	mutex_init(&tbn->dev_mask_mutex);
	mutex_init(&tbn->event_lock);

	init_completion(&tbn->bus_requested);
	init_completion(&tbn->bus_released);
	complete_all(&tbn->bus_requested);
	complete_all(&tbn->bus_released);

	tbn->event_wq = alloc_workqueue("tbn_wq",
					WQ_UNBOUND | WQ_HIGHPRI | WQ_CPU_INTENSIVE,
					1);
	if (!tbn->event_wq) {
		dev_err(dev, "Failed to create work thread for tbn!\n");
		return -ENOMEM;
	}
	INIT_WORK(&tbn->aoc_reset_work, tbn_aoc_reset_work);

	tbn->aoc_channel_task = kthread_run(aoc_channel_kthread, tbn,
					    TBN_AOC_CHANNEL_THREAD_NAME);
	if (IS_ERR(tbn->aoc_channel_task)) {
		err = PTR_ERR(tbn->aoc_channel_task);
		destroy_workqueue(tbn->event_wq);
		return err;
	}
	sched_set_fifo(tbn->aoc_channel_task);

	tbn_context = tbn;
	dev_info(dev, "bus negotiator initialized, mode: %u\n", tbn->mode);

	return 0;
}

static void tbn_remove(struct platform_device *pdev)
{
	struct tbn_context *tbn = platform_get_drvdata(pdev);

	if (!tbn)
		return;

	tbn_context = NULL;
	if (!IS_ERR_OR_NULL(tbn->aoc_channel_task))
		kthread_stop(tbn->aoc_channel_task);
	if (tbn->event_wq)
		destroy_workqueue(tbn->event_wq);
}

static const struct of_device_id tbn_of_match_table[] = {
	{ .compatible = TBN_MODULE_NAME, },
	{ },
};
MODULE_DEVICE_TABLE(of, tbn_of_match_table);

static struct platform_driver tbn_driver = {
	.driver = {
		.name = TBN_MODULE_NAME,
		.of_match_table = tbn_of_match_table,
	},
	.probe = tbn_probe,
	.remove = tbn_remove,
};
module_platform_driver(tbn_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Touch Bus Negotiator");
MODULE_AUTHOR("Google, Inc.");
