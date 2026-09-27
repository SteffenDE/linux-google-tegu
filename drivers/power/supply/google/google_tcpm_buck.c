// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2026 Steffen Deusch
 *
 * Votes the charger's USB buck mode and input current limit from TCPM's sink
 * state.
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
 * The input current limit is the second thing the vendor's USB stack owns.
 * Its usb_psy creates the "USB_ICL" election, votes into it what the port
 * says the partner can supply, and writes the result to the charger's
 * CURRENT_MAX, which programs CHGIN_ILIM and takes the charger off its
 * automatic input current. google_charger votes 0 into the same election to
 * suspend the input -- to run from the battery with the cable in -- to drain
 * back to the charge-stop level or for the battery defender, and from its
 * debugfs switch. Without the election those find nothing to vote into, and
 * chg_vote_input_suspend() gives up before casting its other ballots too.
 *
 * So this creates the election under the vendor's name and casts one ballot
 * into it: TCPM's CURRENT_MAX -- the PD contract, the Type-C Rp current, or
 * for Rp-default whatever the TCPC's BC1.2 detection reported -- or the USB
 * 2.0 default when TCPM has no number, detached included. The vendor's
 * protocol, thermal and dead-battery layers under that are not reproduced;
 * the one ballot is what they reduce to while nothing else votes.
 *
 * Only the sink half of the vendor's buck behaviour is reproduced. It also
 * casts GBMS_USB_OTG_ON and GBMS_USB_OTG_FRS_ON when the port sources VBUS,
 * and those cannot come from this supply, because ONLINE is sink-only by
 * construction. The connector here is sink-only, so no OTG use case is
 * reachable; enabling one means adding the other half here.
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

/* The election google_charger looks up by name to suspend the input. */
#define GTB_ICL_ELECTION	"USB_ICL"
#define GTB_ICL_VOTER		"TCPM"

/* USB 2.0's configured maximum, for a port TCPM has no current for. */
#define GTB_ICL_DEFAULT_UA	500000

/* The charger refuses property writes with -EAGAIN while it resumes. */
#define GTB_ICL_RESUME_RETRY_MS	100

/* Any other failure is retried as the vendor's usb_psy does, then dropped. */
#define GTB_ICL_RETRY_MS	20
#define GTB_ICL_RETRIES		3

struct gtb_drv {
	struct device *dev;
	struct power_supply *tcpm_psy;
	struct power_supply *chg_psy;
	struct notifier_block psy_nb;
	struct work_struct vote_work;
	struct mutex lock;
	bool online;
	bool online_valid;

	struct gvotable_election *icl_el;
	struct delayed_work icl_work;
	int icl_ua;		/* last cast, under lock */
	bool icl_valid;
	int applied_ua;		/* last written, icl_work only */
	bool applied_valid;
	int retries;		/* icl_work only */
};

/*
 * Read the sink state.  Deliberately not one of the GBMS psy helpers: those
 * return the error code as the value, which turns an unimplemented property
 * into a plausible-looking reading.
 */
static int gtb_read_prop(struct gtb_drv *gtb, enum power_supply_property psp,
			 int *value)
{
	union power_supply_propval val;
	int ret;

	ret = power_supply_get_property(gtb->tcpm_psy, psp, &val);
	if (ret < 0)
		return ret;

	*value = val.intval;
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

/* requires gtb->lock */
static void gtb_cast_icl(struct gtb_drv *gtb, int current_ua)
{
	const int icl_ua = current_ua > 0 ? current_ua : GTB_ICL_DEFAULT_UA;
	int ret;

	if (gtb->icl_valid && gtb->icl_ua == icl_ua)
		return;

	ret = gvotable_cast_int_vote(gtb->icl_el, GTB_ICL_VOTER, icl_ua, true);
	if (ret < 0) {
		dev_err(gtb->dev, "cannot vote input limit %d uA (%d)\n",
			icl_ua, ret);
		gtb->icl_valid = false;
		return;
	}

	gtb->icl_ua = icl_ua;
	gtb->icl_valid = true;
}

static void gtb_vote_work(struct work_struct *work)
{
	struct gtb_drv *gtb = container_of(work, struct gtb_drv, vote_work);
	int online, current_ua;
	int ret;

	mutex_lock(&gtb->lock);

	ret = gtb_read_prop(gtb, POWER_SUPPLY_PROP_ONLINE, &online);
	if (ret < 0) {
		/*
		 * A transient failure leaves the ballots alone and waits for
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

	if (!gtb->online_valid || gtb->online != !!online)
		gtb_cast(gtb, online);

	/* the notifier can run this before probe has created the election */
	if (!gtb->icl_el)
		goto unlock;

	ret = gtb_read_prop(gtb, POWER_SUPPLY_PROP_CURRENT_MAX, &current_ua);
	if (ret < 0) {
		dev_err_ratelimited(gtb->dev, "cannot read sink current (%d)\n",
				    ret);
		goto unlock;
	}

	gtb_cast_icl(gtb, online ? current_ua : 0);

unlock:
	mutex_unlock(&gtb->lock);
}

/*
 * The result goes to the charger from a work item, as the vendor's does. The
 * callback runs in whoever cast the ballot -- for google_charger, from inside
 * its own elections' callbacks -- and the write casts into the charger's mode
 * election and talks i2c; the work keeps that out of the voters' context.
 */
static int gtb_icl_cb(struct gvotable_election *el, const char *reason,
		      void *vote)
{
	struct gtb_drv *gtb = gvotable_get_data(el);

	/*
	 * No ballot enabled: there is no result, and nothing to program.  This
	 * driver's own ballot stays enabled while it is bound, unless it is
	 * disabled by hand through the election's debugfs.
	 */
	if (!reason)
		return 0;

	mod_delayed_work(system_wq, &gtb->icl_work, 0);
	return 0;
}

static void gtb_icl_work(struct work_struct *work)
{
	struct gtb_drv *gtb = container_of(to_delayed_work(work),
					   struct gtb_drv, icl_work);
	union power_supply_propval val;
	int icl_ua, ret;

	/*
	 * -EAGAIN while there is no result.  A negative one is not a limit
	 * either; only a value forced through debugfs can be one.
	 */
	icl_ua = gvotable_get_current_int_vote(gtb->icl_el);
	if (icl_ua < 0)
		return;

	if (gtb->applied_valid && gtb->applied_ua == icl_ua)
		return;

	/* 0 suspends the charger's USB input, anything else programs it */
	val.intval = icl_ua;
	ret = power_supply_set_property(gtb->chg_psy,
					POWER_SUPPLY_PROP_CURRENT_MAX, &val);
	if (ret == -EAGAIN) {
		schedule_delayed_work(&gtb->icl_work,
				      msecs_to_jiffies(GTB_ICL_RESUME_RETRY_MS));
		return;
	}
	if (ret < 0) {
		gtb->applied_valid = false;
		if (gtb->retries++ < GTB_ICL_RETRIES) {
			schedule_delayed_work(&gtb->icl_work,
					      msecs_to_jiffies(GTB_ICL_RETRY_MS));
			return;
		}
		dev_err(gtb->dev, "cannot set input limit %d uA (%d)\n",
			icl_ua, ret);
		gtb->retries = 0;
		return;
	}

	gtb->retries = 0;
	gtb->applied_ua = icl_ua;
	gtb->applied_valid = true;
	if (icl_ua)
		dev_info(gtb->dev, "input limit %d mA\n", icl_ua / 1000);
	else
		dev_info(gtb->dev, "input suspended\n");
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

static int gtb_create_icl(struct gtb_drv *gtb)
{
	struct gvotable_election *el;

	el = gvotable_create_int_election(GTB_ICL_ELECTION,
					  gvotable_comparator_int_min,
					  gtb_icl_cb, gtb);
	/* NULL when the name is taken, or when an allocation failed */
	if (!el)
		return -EEXIST;

	gvotable_set_vote2str(el, gvotable_v2s_int);
	gtb->icl_el = el;

	return 0;
}

static void gtb_put_supplies(struct gtb_drv *gtb)
{
	power_supply_put(gtb->chg_psy);
	power_supply_put(gtb->tcpm_psy);
}

static int google_tcpm_buck_probe(struct platform_device *pdev)
{
	struct power_supply *psy[1] = { NULL };
	struct device *dev = &pdev->dev;
	struct power_supply *chg_psy;
	struct gvotable_election *el;
	const char *chg_name;
	struct gtb_drv *gtb;
	int ret;

	ret = of_property_read_string(dev->of_node, "google,chg-power-supply",
				      &chg_name);
	if (ret)
		return dev_err_probe(dev, ret,
				     "google,chg-power-supply is missing\n");

	/*
	 * All three producers are device probes, so deferred probe is exactly
	 * the right wait: the TCPM supply appears when the port controller
	 * binds, and the election and the charger's supply when the charger
	 * does.  Requiring them here rather than polling for them later means
	 * a missing one can never become a silently un-cast vote, which would
	 * leave the phone not charging.
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

	chg_psy = power_supply_get_by_name(chg_name);
	if (!chg_psy) {
		power_supply_put(psy[0]);
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "charger supply %s not up yet\n", chg_name);
	}

	gtb = devm_kzalloc(dev, sizeof(*gtb), GFP_KERNEL);
	if (!gtb) {
		power_supply_put(chg_psy);
		power_supply_put(psy[0]);
		return -ENOMEM;
	}

	gtb->dev = dev;
	gtb->tcpm_psy = psy[0];
	gtb->chg_psy = chg_psy;
	ret = devm_mutex_init(dev, &gtb->lock);
	if (ret) {
		gtb_put_supplies(gtb);
		return ret;
	}

	INIT_WORK(&gtb->vote_work, gtb_vote_work);
	INIT_DELAYED_WORK(&gtb->icl_work, gtb_icl_work);
	platform_set_drvdata(pdev, gtb);

	gtb->psy_nb.notifier_call = gtb_psy_changed;
	ret = power_supply_reg_notifier(&gtb->psy_nb);
	if (ret < 0) {
		gtb_put_supplies(gtb);
		return dev_err_probe(dev, ret, "cannot register psy notifier\n");
	}

	/*
	 * Last, because nothing may fail after it: the election is found by
	 * name as soon as it exists, and google_charger keeps the handle from
	 * its first lookup, so it can never be destroyed again.
	 */
	mutex_lock(&gtb->lock);
	ret = gtb_create_icl(gtb);
	mutex_unlock(&gtb->lock);
	if (ret) {
		power_supply_unreg_notifier(&gtb->psy_nb);
		cancel_work_sync(&gtb->vote_work);
		mutex_lock(&gtb->lock);
		if (gtb->online_valid && gtb->online)
			gtb_cast(gtb, false);
		mutex_unlock(&gtb->lock);
		gtb_put_supplies(gtb);
		return dev_err_probe(dev, ret, "cannot create %s\n",
				     GTB_ICL_ELECTION);
	}

	/* Cast the state as it stands: the cable may already be in. */
	schedule_work(&gtb->vote_work);
	dev_info(dev, "watching %s for sink state\n", gtb->tcpm_psy->desc->name);

	return 0;
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
		.suppress_bind_attrs = true,
	},
	.probe = google_tcpm_buck_probe,
};
/*
 * Once bound, this stays: google_charger caches the USB_ICL handle for its
 * lifetime, as it does the charger's own elections, so the election must not
 * go away under it.  No unbind, and no module exit -- a module without one
 * cannot be unloaded.
 */
builtin_platform_driver(google_tcpm_buck_driver);

MODULE_DESCRIPTION("Vote the charger's USB buck mode and input limit from TCPM sink state");
MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_LICENSE("GPL");
