// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2026 Steffen Deusch
 *
 * Votes the charger's USB buck mode from TCPM's sink state.
 *
 * The MAX77779 charger decides its operating mode by arbitration: every voter
 * casts a ballot into the "CHARGER_MODE" election and the charger's callback
 * walks the enabled ballots to derive a mode. google_charger votes
 * GBMS_CHGR_MODE_CHGR_BUCK_ON, which only asks for a charge *current* -- it
 * sets the arbitration's chgr_on and leaves buck_on clear. With buck_on clear
 * and no wireless receiver the charger resolves to MAX77779_CHGR_MODE_ALL_OFF
 * and switches charging off, whatever else is voting.
 *
 * What supplies the missing ballot on the vendor's software is its own Type-C
 * port controller driver, which casts GBMS_USB_BUCK_ON when the port attaches
 * as a sink. This board runs mainline's tcpci_maxim instead, which knows
 * nothing about GBMS, so nothing casts it and the charger never turns on.
 *
 * Rather than carry the vendor's ~7500-line port controller stack for one
 * vote, this watches the sink state mainline already tracks. TCPM sets
 * vbus_charge when the port may draw from VBUS -- unconditionally on attach
 * when PD is disabled, and from the negotiated PDO otherwise -- reports it as
 * the ONLINE property of its power supply, and signals a change from inside
 * the same function that sets it. That is the same edge the vendor's driver
 * votes on: its vote is cast from the tcpc->set_vbus callback, which is what
 * TCPM calls to set vbus_charge in the first place.
 *
 * The ballot deliberately carries the vendor's own voter name. The charger's
 * arbitration is defined in terms of that reason, so occupying the same slot
 * reproduces its contract rather than adding a parallel voter to it.
 *
 * Only the sink half of the vendor's behaviour is reproduced. It also casts
 * GBMS_USB_OTG_ON and GBMS_USB_OTG_FRS_ON when the port sources VBUS, and
 * those cannot come from this supply, because ONLINE is sink-only by
 * construction. The connector here is sink and device with PD disabled, so no
 * OTG use case is reachable; enabling one means adding the other half here.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "gbms_compat.h"
#include "google_bms.h"

/* The name the vendor's port controller votes under; see the note above. */
#define GTB_VOTER	"TCPCI"

struct gtb_drv {
	struct device *dev;
	struct power_supply *tcpm_psy;
	struct notifier_block psy_nb;
	struct work_struct vote_work;
	struct mutex lock;
	bool online;
	bool online_valid;
};

/*
 * Read the sink state.  Deliberately not one of the GBMS psy helpers: those
 * return the error code as the value, which turns an unimplemented property
 * into a plausible-looking reading.
 */
static int gtb_read_online(struct gtb_drv *gtb, bool *online)
{
	union power_supply_propval val;
	int ret;

	ret = power_supply_get_property(gtb->tcpm_psy,
					POWER_SUPPLY_PROP_ONLINE, &val);
	if (ret < 0)
		return ret;

	*online = val.intval != 0;
	return 0;
}

/* requires gtb->lock */
static void gtb_cast(struct gtb_drv *gtb, bool online)
{
	struct gvotable_election *el;
	int ret;

	/*
	 * Looked up per vote rather than cached: the handle carries no
	 * reference -- it is the election's own pointer, handed out under a
	 * lock that is dropped before it returns -- so a cached one dangles if
	 * the charger is ever unbound.  The lookup is a hashed walk once per
	 * cable event.
	 */
	el = gvotable_election_get_handle(GBMS_MODE_VOTABLE);
	if (IS_ERR_OR_NULL(el)) {
		dev_err_ratelimited(gtb->dev, "charger mode election is gone\n");
		gtb->online_valid = false;
		return;
	}

	ret = gvotable_cast_long_vote(el, GTB_VOTER, GBMS_USB_BUCK_ON, online);
	if (ret < 0) {
		dev_err(gtb->dev, "cannot vote buck %s (%d)\n",
			online ? "on" : "off", ret);
		gtb->online_valid = false;
		return;
	}

	gtb->online = online;
	gtb->online_valid = true;
	dev_info(gtb->dev, "sink %s, buck vote %s\n",
		 online ? "attached" : "detached", online ? "on" : "off");
}

static void gtb_vote_work(struct work_struct *work)
{
	struct gtb_drv *gtb = container_of(work, struct gtb_drv, vote_work);
	bool online;
	int ret;

	mutex_lock(&gtb->lock);

	ret = gtb_read_online(gtb, &online);
	if (ret < 0) {
		/*
		 * A transient failure leaves the ballot alone and waits for
		 * the next change.  A supply that has gone away will not send
		 * one, and leaving an attached vote standing would have the
		 * charger bucking with nothing tracking the port -- so retract
		 * instead.
		 */
		if (ret == -ENODEV && gtb->online_valid && gtb->online) {
			dev_warn(gtb->dev, "sink state gone, retracting\n");
			gtb_cast(gtb, false);
		} else {
			dev_err_ratelimited(gtb->dev,
					    "cannot read sink state (%d)\n", ret);
		}
		goto unlock;
	}

	if (gtb->online_valid && gtb->online == online)
		goto unlock;

	gtb_cast(gtb, online);

unlock:
	mutex_unlock(&gtb->lock);
}

/*
 * Sleeping here would be allowed -- this is a blocking notifier, and the
 * property read is a plain memory access besides.  The work item is for what
 * the vote does: gvotable_cast_vote() runs the charger's mode callback
 * synchronously, which talks i2c, takes a wakeup source and can reschedule
 * itself.  That does not belong under the notifier chain's rwsem.
 */
static int gtb_psy_changed(struct notifier_block *nb, unsigned long action,
			   void *data)
{
	struct gtb_drv *gtb = container_of(nb, struct gtb_drv, psy_nb);
	struct power_supply *psy = data;

	if (action == PSY_EVENT_PROP_CHANGED && psy == gtb->tcpm_psy)
		schedule_work(&gtb->vote_work);

	return NOTIFY_OK;
}

static int google_tcpm_buck_probe(struct platform_device *pdev)
{
	struct power_supply *psy[1] = { NULL };
	struct device *dev = &pdev->dev;
	struct gvotable_election *el;
	struct gtb_drv *gtb;
	int ret;

	/*
	 * Both producers are device probes, so deferred probe is exactly the
	 * right wait: the TCPM supply appears when the port controller binds
	 * and the election when the charger does.  Requiring them here rather
	 * than polling for them later means a missing one can never become a
	 * silently un-cast vote, which would leave the phone not charging.
	 */
	ret = of_power_supply_get_by_phandle_array(dev->of_node,
						   "google,tcpm-power-supply",
						   psy, ARRAY_SIZE(psy));
	if (ret < 1 || !psy[0])
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "TCPM power supply not up yet\n");

	el = gvotable_election_get_handle(GBMS_MODE_VOTABLE);
	if (IS_ERR_OR_NULL(el)) {
		power_supply_put(psy[0]);
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "charger mode election not up yet\n");
	}

	gtb = devm_kzalloc(dev, sizeof(*gtb), GFP_KERNEL);
	if (!gtb) {
		power_supply_put(psy[0]);
		return -ENOMEM;
	}

	gtb->dev = dev;
	gtb->tcpm_psy = psy[0];
	ret = devm_mutex_init(dev, &gtb->lock);
	if (ret) {
		power_supply_put(psy[0]);
		return ret;
	}

	INIT_WORK(&gtb->vote_work, gtb_vote_work);
	platform_set_drvdata(pdev, gtb);

	gtb->psy_nb.notifier_call = gtb_psy_changed;
	ret = power_supply_reg_notifier(&gtb->psy_nb);
	if (ret < 0) {
		power_supply_put(psy[0]);
		return dev_err_probe(dev, ret, "cannot register psy notifier\n");
	}

	/* Cast the state as it stands: the cable may already be in. */
	schedule_work(&gtb->vote_work);
	dev_info(dev, "watching %s for sink state\n", gtb->tcpm_psy->desc->name);

	return 0;
}

static void google_tcpm_buck_remove(struct platform_device *pdev)
{
	struct gtb_drv *gtb = platform_get_drvdata(pdev);

	power_supply_unreg_notifier(&gtb->psy_nb);
	cancel_work_sync(&gtb->vote_work);

	/*
	 * Retract rather than leave a stale ballot: the charger would
	 * otherwise keep bucking on a state nothing is tracking.
	 */
	mutex_lock(&gtb->lock);
	if (gtb->online_valid && gtb->online)
		gtb_cast(gtb, false);
	mutex_unlock(&gtb->lock);

	power_supply_put(gtb->tcpm_psy);
}

static const struct of_device_id google_tcpm_buck_match[] = {
	{ .compatible = "google,tcpm-buck" },
	{ }
};
MODULE_DEVICE_TABLE(of, google_tcpm_buck_match);

static struct platform_driver google_tcpm_buck_driver = {
	.driver = {
		.name = "google_tcpm_buck",
		.of_match_table = google_tcpm_buck_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = google_tcpm_buck_probe,
	.remove = google_tcpm_buck_remove,
};
module_platform_driver(google_tcpm_buck_driver);

MODULE_DESCRIPTION("Vote the charger's USB buck mode from TCPM sink state");
MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_LICENSE("GPL");
