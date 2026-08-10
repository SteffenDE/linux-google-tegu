// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung KEPLER GNSS receiver.
 *
 * KEPLER sits alongside the Exynos baseband on Google Tensor SoCs. Its
 * firmware is staged into baseband shared memory and started by the baseband,
 * so nothing here loads firmware; by the time this driver matters the receiver
 * is already running and only needs a pipe. That pipe is an independent SPI
 * bus plus two GPIOs:
 *
 *   gnss2ap  device -> host, high while the receiver has data to move, and,
 *            during a host-initiated write, once it is ready to accept the
 *            transfer. It is the interrupt source, and it is *level*
 *            triggered, so it has to be masked for as long as it stays
 *            asserted or the handler re-enters forever.
 *   ap2gnss  host -> device, high to say the host is ready to move data.
 *
 * Transfers are fixed 64-byte bursts on the read side and one aligned buffer
 * on the write side. The payload is Samsung's proprietary BETP framing, which
 * this driver does not interpret: it hands the raw byte stream to the GNSS
 * core and lets user space frame it.
 *
 * The register-level behaviour follows the vendor gnssif_spi driver
 * (gnss_link_device.c, gnss_spi.c, gnss_keplerctl_device.c).
 *
 * Copyright (C) 2022 Samsung Electronics Co., Ltd.
 * Copyright (C) 2026 Steffen Deusch
 */

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gnss.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm_wakeup.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>

#define KEPLER_RX_CHUNK		64	/* one SPI read burst */
#define KEPLER_RX_FRAME_MAX	SZ_2K	/* hand accumulated RX up at this */
#define KEPLER_TX_MAX		SZ_4K
#define KEPLER_FRAME_ALIGN	4	/* BETP frames are 4-byte aligned */
#define KEPLER_RDY_TIMEOUT_MS	5000
#define KEPLER_WAKE_MS		500

/*
 * The vendor clocks these transfers 32 bits at a time, which works there
 * because its device tree sets swap-mode on this SPI controller and the byte
 * swap puts a byte stream on the wire in memory order. Mainline spi-s3c64xx
 * always clears SWAP_CFG, so 32-bit words would come out byte-reversed and
 * 8-bit words are what reproduce the vendor's wire order. The 4-byte frame
 * alignment above is a BETP requirement and is unaffected either way.
 */
#define KEPLER_BITS_PER_WORD	8

struct kepler_gnss {
	struct spi_device	*spi;
	struct gnss_device	*gdev;
	struct gpio_desc	*gnss2ap;
	struct gpio_desc	*ap2gnss;
	int			irq;

	/* Guards irq_on so the enables below stay balanced. */
	spinlock_t		irq_lock;
	bool			irq_on;

	struct completion	ready;
	atomic_t		wait_ready;
	atomic_t		rx_active;
	atomic_t		tx_active;

	u8			rx_buf[KEPLER_RX_FRAME_MAX] __aligned(4);
};

/*
 * enable_irq() is refcounted and the paths below re-arm the line from several
 * places, so every enable/disable goes through these and is a no-op unless it
 * changes the state.
 */
static void kepler_irq_enable(struct kepler_gnss *kp)
{
	unsigned long flags;

	spin_lock_irqsave(&kp->irq_lock, flags);
	if (!kp->irq_on) {
		enable_irq(kp->irq);
		kp->irq_on = true;
	}
	spin_unlock_irqrestore(&kp->irq_lock, flags);
}

/* Safe from the hard IRQ handler. */
static void kepler_irq_disable_nosync(struct kepler_gnss *kp)
{
	unsigned long flags;

	spin_lock_irqsave(&kp->irq_lock, flags);
	if (kp->irq_on) {
		disable_irq_nosync(kp->irq);
		kp->irq_on = false;
	}
	spin_unlock_irqrestore(&kp->irq_lock, flags);
}

static void kepler_irq_disable_sync(struct kepler_gnss *kp)
{
	bool was_on;

	spin_lock_irq(&kp->irq_lock);
	was_on = kp->irq_on;
	kp->irq_on = false;
	spin_unlock_irq(&kp->irq_lock);

	if (was_on)
		disable_irq(kp->irq);
}

/*
 * Drop the request line unless a write is parked on it.  The write path
 * publishes its claim by setting tx_active and raising the line as one step
 * under the same lock, so this cannot land between the two: otherwise a write
 * starting here has its assertion undone and then waits out its whole timeout
 * for an answer the receiver was never asked for.
 */
static void kepler_ap2gnss_idle(struct kepler_gnss *kp)
{
	unsigned long flags;

	spin_lock_irqsave(&kp->irq_lock, flags);
	if (!atomic_read(&kp->tx_active))
		gpiod_set_value(kp->ap2gnss, 0);
	spin_unlock_irqrestore(&kp->irq_lock, flags);
}

static int kepler_spi_recv(struct kepler_gnss *kp, void *rx, unsigned int len)
{
	struct spi_transfer xfer = {
		.rx_buf		= rx,
		.len		= len,
		.bits_per_word	= KEPLER_BITS_PER_WORD,
	};

	return spi_sync_transfer(kp->spi, &xfer, 1);
}

static int kepler_spi_xfer(struct kepler_gnss *kp, const void *tx, void *rx,
			   unsigned int len)
{
	struct spi_transfer xfer = {
		.tx_buf		= tx,
		.rx_buf		= rx,
		.len		= len,
		.bits_per_word	= KEPLER_BITS_PER_WORD,
	};

	return spi_sync_transfer(kp->spi, &xfer, 1);
}

static irqreturn_t kepler_irq(int irq, void *data)
{
	struct kepler_gnss *kp = data;

	/* Level line: mask it, and let whoever handles it re-arm. */
	kepler_irq_disable_nosync(kp);

	/*
	 * A write is parked waiting for the receiver to say it can accept the
	 * transfer, and this assertion is that answer rather than inbound data.
	 */
	if (atomic_read(&kp->wait_ready)) {
		atomic_set(&kp->wait_ready, 0);
		complete_all(&kp->ready);
		return IRQ_HANDLED;
	}

	/* The read thread drains until the line drops, so it has this covered. */
	if (atomic_read(&kp->rx_active))
		return IRQ_HANDLED;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t kepler_irq_thread(int irq, void *data)
{
	struct kepler_gnss *kp = data;
	unsigned int filled = 0;
	int ret;

	atomic_set(&kp->rx_active, 1);

	do {
		if (filled == 0) {
			/*
			 * Tell the receiver the host is ready to take a burst.
			 * A write already holds the line high for its own
			 * handshake, so only pulse it when one is not running.
			 */
			gpiod_set_value(kp->ap2gnss, 1);
			if (!atomic_read(&kp->tx_active)) {
				/*
				 * Busy-wait rather than sleep: this is a pulse
				 * width the receiver is timing, and letting the
				 * scheduler stretch it defeats the point.
				 */
				udelay(100);
				kepler_ap2gnss_idle(kp);
			}
		}

		ret = kepler_spi_recv(kp, kp->rx_buf + filled, KEPLER_RX_CHUNK);
		if (ret) {
			dev_err_ratelimited(&kp->spi->dev, "read failed: %d\n",
					    ret);
			break;
		}
		filled += KEPLER_RX_CHUNK;

		if (filled >= KEPLER_RX_FRAME_MAX ||
		    atomic_read(&kp->wait_ready)) {
			gnss_insert_raw(kp->gdev, kp->rx_buf, filled);
			filled = 0;

			/* A write wants the bus: let it run before going on. */
			if (atomic_read(&kp->wait_ready)) {
				atomic_set(&kp->wait_ready, 0);
				complete_all(&kp->ready);
				/*
				 * The write owns the bus until it is done, and
				 * a full one takes milliseconds at this clock,
				 * so yield rather than spin the way the vendor
				 * does.
				 */
				while (atomic_read(&kp->tx_active))
					usleep_range(50, 200);
			}
		}
	} while (gpiod_get_value(kp->gnss2ap));

	if (filled)
		gnss_insert_raw(kp->gdev, kp->rx_buf, filled);

	atomic_set(&kp->rx_active, 0);
	kepler_ap2gnss_idle(kp);

	kepler_irq_enable(kp);

	return IRQ_HANDLED;
}

static int kepler_open(struct gnss_device *gdev)
{
	struct kepler_gnss *kp = gnss_get_drvdata(gdev);

	kepler_irq_enable(kp);

	return 0;
}

static void kepler_close(struct gnss_device *gdev)
{
	struct kepler_gnss *kp = gnss_get_drvdata(gdev);

	kepler_irq_disable_sync(kp);
}

static int kepler_write_raw(struct gnss_device *gdev,
			    const unsigned char *buf, size_t count)
{
	struct kepler_gnss *kp = gnss_get_drvdata(gdev);
	unsigned int len;
	void *tx, *rx;
	int ret;

	count = min_t(size_t, count, KEPLER_TX_MAX);
	len = round_up(count, KEPLER_FRAME_ALIGN);

	tx = kzalloc(len, GFP_KERNEL);
	rx = kzalloc(len, GFP_KERNEL);
	if (!tx || !rx) {
		ret = -ENOMEM;
		goto out_free;
	}
	memcpy(tx, buf, count);

	reinit_completion(&kp->ready);
	atomic_set(&kp->wait_ready, 1);

	pm_wakeup_dev_event(&kp->spi->dev, KEPLER_WAKE_MS, false);
	kepler_irq_enable(kp);

	/*
	 * Ask for the bus, then wait for the receiver to say it can take it.
	 * Claiming the line and announcing the claim are one step, so the read
	 * thread cannot drop it underneath us -- see kepler_ap2gnss_idle().
	 */
	spin_lock_irq(&kp->irq_lock);
	atomic_set(&kp->tx_active, 1);
	gpiod_set_value(kp->ap2gnss, 1);
	spin_unlock_irq(&kp->irq_lock);

	if (!wait_for_completion_timeout(&kp->ready,
					 msecs_to_jiffies(KEPLER_RDY_TIMEOUT_MS))) {
		dev_err(&kp->spi->dev,
			"timed out waiting for receiver ready (ap2gnss %d, gnss2ap %d)\n",
			gpiod_get_value(kp->ap2gnss),
			gpiod_get_value(kp->gnss2ap));
		ret = -ETIMEDOUT;
		goto out_deassert;
	}

	ret = kepler_spi_xfer(kp, tx, rx, len);
	if (ret) {
		dev_err(&kp->spi->dev, "write failed: %d\n", ret);
		goto out_deassert;
	}

	/* The transfer is full duplex: what came back is an inbound frame. */
	gnss_insert_raw(gdev, rx, len);
	ret = count;

out_deassert:
	atomic_set(&kp->wait_ready, 0);
	atomic_set(&kp->tx_active, 0);
	gpiod_set_value(kp->ap2gnss, 0);
	kepler_irq_enable(kp);
out_free:
	kfree(tx);
	kfree(rx);

	return ret;
}

static const struct gnss_operations kepler_gnss_ops = {
	.open		= kepler_open,
	.close		= kepler_close,
	.write_raw	= kepler_write_raw,
};

static int kepler_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct kepler_gnss *kp;
	struct gnss_device *gdev;
	int ret;

	kp = devm_kzalloc(dev, sizeof(*kp), GFP_KERNEL);
	if (!kp)
		return -ENOMEM;

	kp->spi = spi;
	spin_lock_init(&kp->irq_lock);
	init_completion(&kp->ready);

	kp->gnss2ap = devm_gpiod_get(dev, "gnss2ap", GPIOD_IN);
	if (IS_ERR(kp->gnss2ap))
		return dev_err_probe(dev, PTR_ERR(kp->gnss2ap),
				     "failed to get gnss2ap\n");

	kp->ap2gnss = devm_gpiod_get(dev, "ap2gnss", GPIOD_OUT_LOW);
	if (IS_ERR(kp->ap2gnss))
		return dev_err_probe(dev, PTR_ERR(kp->ap2gnss),
				     "failed to get ap2gnss\n");

	kp->irq = gpiod_to_irq(kp->gnss2ap);
	if (kp->irq < 0)
		return dev_err_probe(dev, kp->irq, "failed to map gnss2ap irq\n");

	spi->bits_per_word = KEPLER_BITS_PER_WORD;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "spi_setup failed\n");

	gdev = gnss_allocate_device(dev);
	if (!gdev)
		return -ENOMEM;

	/*
	 * Nominal only: the stream is Samsung BETP, which the core has no type
	 * for. It names a sysfs attribute and gates nothing.
	 */
	gdev->type = GNSS_TYPE_NMEA;
	gdev->ops = &kepler_gnss_ops;
	gnss_set_drvdata(gdev, kp);
	kp->gdev = gdev;

	/* Holds the system up for the length of a write, as the vendor does. */
	ret = devm_device_init_wakeup(dev);
	if (ret)
		goto err_put_device;

	/*
	 * NO_AUTOEN because the line may already be asserted, and nothing is
	 * ready to service it until the device is opened.
	 */
	ret = devm_request_threaded_irq(dev, kp->irq, kepler_irq,
					kepler_irq_thread,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT |
					IRQF_NO_AUTOEN,
					dev_name(dev), kp);
	if (ret) {
		dev_err_probe(dev, ret, "failed to request gnss2ap irq\n");
		goto err_put_device;
	}

	ret = gnss_register_device(gdev);
	if (ret)
		goto err_put_device;

	spi_set_drvdata(spi, kp);

	return 0;

err_put_device:
	gnss_put_device(gdev);

	return ret;
}

static void kepler_remove(struct spi_device *spi)
{
	struct kepler_gnss *kp = spi_get_drvdata(spi);

	gnss_deregister_device(kp->gdev);
	kepler_irq_disable_sync(kp);
	gpiod_set_value(kp->ap2gnss, 0);
	gnss_put_device(kp->gdev);
}

static const struct of_device_id kepler_of_match[] = {
	{ .compatible = "samsung,kepler" },
	{ }
};
MODULE_DEVICE_TABLE(of, kepler_of_match);

static const struct spi_device_id kepler_spi_id[] = {
	{ "kepler" },
	{ }
};
MODULE_DEVICE_TABLE(spi, kepler_spi_id);

static struct spi_driver kepler_driver = {
	.driver = {
		.name		= "gnss-kepler",
		.of_match_table	= kepler_of_match,
	},
	.probe		= kepler_probe,
	.remove		= kepler_remove,
	.id_table	= kepler_spi_id,
};
module_spi_driver(kepler_driver);

MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_DESCRIPTION("Samsung KEPLER GNSS receiver driver");
MODULE_LICENSE("GPL");
