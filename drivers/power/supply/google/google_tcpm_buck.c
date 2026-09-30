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
 * The source half is a regulator. TCPM turns VBUS on through the TCPC's
 * "vbus" supply, and the vendor's port controller does it by casting
 * GBMS_USB_OTG_ON under the same voter it casts GBMS_USB_BUCK_ON under, so
 * that sourcing replaces sinking in one ballot: the charger refuses every
 * OTG use case while buck_on is set. So both come from the one function,
 * under the one lock -- the regulator's enable and disable and the sink
 * state's work item each recompute the ballot rather than cast their own.
 * With no external boost described on the charger, the OTG ballot selects
 * its internal reverse boost, CHG_CNFG_00 mode OTG_BOOST_ON, at the
 * current limit CHG_CNFG_05 already holds.
 *
 * GBMS_USB_OTG_FRS_ON, the fast role swap, is not reproduced.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "gbms_compat.h"
#include "google_bms.h"
#include "max77779.h"

/* The name the vendor's port controller votes under; see the note above. */
#define GTB_VOTER	"TCPCI"

/* The mode the charger's election resolves to for its internal boost. */
#define GTB_MODE_OTG	MAX77779_CHGR_MODE_OTG_BOOST_ON

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
	struct device *chg_dev;		/* the MAX77779 behind chg_psy, or NULL */
	struct notifier_block psy_nb;
	struct work_struct vote_work;
	struct mutex lock;
	bool online;		/* TCPM's sink state, as last read */
	bool sourcing;		/* the vbus regulator is enabled */

	int cast_vote;		/* the "TCPCI" ballot as last cast */
	bool cast_enabled;
	bool cast_valid;

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

/*
 * Cast the one "TCPCI" ballot from the state as it stands: the OTG boost while
 * the port sources, the buck while it sinks, nothing otherwise.  A sink state
 * that says online while the port sources is TCPM's own lag -- a swap to
 * source clears vbus_charge before it enables the supply, and the change
 * reaches the work item later -- so sourcing wins.
 *
 * requires gtb->lock
 */
static int gtb_recompute_mode(struct gtb_drv *gtb)
{
	const int vote = gtb->sourcing ? GBMS_USB_OTG_ON : GBMS_USB_BUCK_ON;
	const bool enabled = gtb->sourcing || gtb->online;
	struct gvotable_election *el;
	int ret;

	if (gtb->cast_valid && gtb->cast_vote == vote &&
	    gtb->cast_enabled == enabled)
		return 0;

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
		gtb->cast_valid = false;
		return -ENODEV;
	}

	/* runs the charger's mode callback before it returns */
	ret = gvotable_cast_long_vote(el, GTB_VOTER, vote, enabled);
	if (ret < 0) {
		dev_err(gtb->dev, "cannot vote %s %s (%d)\n",
			gtb->sourcing ? "otg" : "buck", enabled ? "on" : "off",
			ret);
		gtb->cast_valid = false;
		return ret;
	}

	gtb->cast_vote = vote;
	gtb->cast_enabled = enabled;
	gtb->cast_valid = true;
	if (gtb->sourcing)
		dev_info(gtb->dev, "sourcing, otg vote on\n");
	else
		dev_info(gtb->dev, "sink %s, buck vote %s\n",
			 enabled ? "attached" : "detached", enabled ? "on" : "off");

	return 0;
}

/*
 * The mode CHG_CNFG_00 holds, read from the chip.  Not the election's
 * result: the election installs its head ballot, or its default with none
 * enabled, as the result before the charger's callback runs, and the
 * callback overwrites that only when it writes the register.  So a callback
 * that returns early -- with no ballot enabled it writes nothing at all --
 * leaves a result that says nothing about the register.
 */
static int gtb_charger_mode(struct gtb_drv *gtb)
{
	u8 reg;
	int ret;

	ret = max77779_external_chg_reg_read(gtb->chg_dev, MAX77779_CHG_CNFG_00,
					     &reg);
	if (ret < 0)
		return ret;

	return _max77779_chg_cnfg_00_mode_get(reg);
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
		if (ret == -ENODEV && gtb->online) {
			dev_warn(gtb->dev, "sink state gone, retracting\n");
			gtb->online = false;
			gtb_recompute_mode(gtb);
		} else {
			dev_err_ratelimited(gtb->dev,
					    "cannot read sink state (%d)\n", ret);
		}
		goto unlock;
	}

	gtb->online = online;
	gtb_recompute_mode(gtb);

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

/*
 * TCPM calls these from its state machine, which waits for the TCPC to see
 * VBUS after an enable -- so the enable must not return before the charger
 * has switched.  The charger's callback writes CHG_CNFG_00 inside the cast,
 * and the register is read back to confirm the boost.  A failure retracts
 * the ballot, and TCPM falls back to unattached and tries again -- also what
 * happens to an attach during the charger's resume, when it refuses both.
 */
static int gtb_vbus_enable(struct regulator_dev *rdev)
{
	struct gtb_drv *gtb = rdev_get_drvdata(rdev);
	int mode, ret;

	mutex_lock(&gtb->lock);

	gtb->sourcing = true;
	ret = gtb_recompute_mode(gtb);
	if (ret < 0)
		goto fail;

	mode = gtb_charger_mode(gtb);
	if (mode != GTB_MODE_OTG) {
		dev_err(gtb->dev, "charger did not start the boost (mode %#x)\n",
			mode);
		ret = mode < 0 ? mode : -EIO;
		goto fail;
	}

	mutex_unlock(&gtb->lock);
	return 0;

fail:
	/* do not leave a ballot behind for a later election to act on */
	gtb->sourcing = false;
	gtb_recompute_mode(gtb);
	mutex_unlock(&gtb->lock);
	return ret;
}

static int gtb_vbus_disable(struct regulator_dev *rdev)
{
	struct gtb_drv *gtb = rdev_get_drvdata(rdev);
	int mode;

	mutex_lock(&gtb->lock);

	/*
	 * Nothing here fails the disable.  The regulator core keeps its use
	 * count on a failed one while is_enabled already says off, and the
	 * TCPC then enables by count alone -- without calling in here -- for
	 * as long as the phone is up.  A ballot that could not be retracted
	 * stays invalid and is retried at the next sink state change.
	 */
	gtb->sourcing = false;
	if (gtb_recompute_mode(gtb) < 0)
		goto unlock;

	/*
	 * With no ballot left enabled the charger's callback returns before
	 * writing anything, and CHG_CNFG_00 keeps the boost running.  That is
	 * only reachable when nothing else -- google_charger's standby vote,
	 * this driver's own buck vote -- is standing, but VBUS left on after a
	 * detach must not pass silently.
	 */
	mode = gtb_charger_mode(gtb);
	if (mode == GTB_MODE_OTG)
		dev_err(gtb->dev, "charger left the boost on\n");
	else if (mode < 0)
		dev_err(gtb->dev, "cannot read the charger mode (%d)\n", mode);

unlock:
	mutex_unlock(&gtb->lock);
	return 0;
}

static int gtb_vbus_is_enabled(struct regulator_dev *rdev)
{
	struct gtb_drv *gtb = rdev_get_drvdata(rdev);
	bool sourcing;

	mutex_lock(&gtb->lock);
	sourcing = gtb->sourcing;
	mutex_unlock(&gtb->lock);

	return sourcing;
}

static const struct regulator_ops gtb_vbus_ops = {
	.enable = gtb_vbus_enable,
	.disable = gtb_vbus_disable,
	.is_enabled = gtb_vbus_is_enabled,
};

static const struct regulator_desc gtb_vbus_desc = {
	.name = "vbus",
	.of_match = "vbus-regulator",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &gtb_vbus_ops,
	.n_voltages = 1,
	.fixed_uV = 5000000,
};

static void gtb_put_supplies(struct gtb_drv *gtb)
{
	power_supply_put(gtb->chg_psy);
	power_supply_put(gtb->tcpm_psy);
}

static int google_tcpm_buck_probe(struct platform_device *pdev)
{
	struct power_supply *psy[1] = { NULL };
	struct regulator_config reg_cfg = { };
	struct device *dev = &pdev->dev;
	struct regulator_dev *rdev;
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
	/*
	 * Confirming the boost reads the charger's own register, through an
	 * export that takes its device's driver data on trust.
	 */
	if (chg_psy->dev.parent &&
	    of_device_is_compatible(chg_psy->dev.parent->of_node,
				    "maxim,max77779chrg-i2c"))
		gtb->chg_dev = chg_psy->dev.parent;
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
		gtb->online = false;
		if (gtb->cast_valid && gtb->cast_enabled)
			gtb_recompute_mode(gtb);
		mutex_unlock(&gtb->lock);
		gtb_put_supplies(gtb);
		return dev_err_probe(dev, ret, "cannot create %s\n",
				     GTB_ICL_ELECTION);
	}

	/*
	 * After the election, and not fatal: the TCPC looks the regulator up
	 * at any set_vbus from now on, and one it holds must not be
	 * unregistered under it by a failure further down.  Without it the
	 * port still sinks; TCPM's source path fails its enable instead.
	 */
	if (gtb->chg_dev) {
		reg_cfg.dev = dev;
		reg_cfg.driver_data = gtb;
		rdev = devm_regulator_register(dev, &gtb_vbus_desc, &reg_cfg);
		if (IS_ERR(rdev))
			dev_err(dev, "cannot register the vbus regulator (%pe)\n",
				rdev);
	} else {
		dev_err(dev, "%s is not a MAX77779, not sourcing\n", chg_name);
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

MODULE_DESCRIPTION("Vote the charger's USB mode and input limit for TCPM");
MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_LICENSE("GPL");
