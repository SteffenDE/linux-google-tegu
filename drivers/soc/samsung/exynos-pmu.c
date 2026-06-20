// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2011-2014 Samsung Electronics Co., Ltd.
//		http://www.samsung.com/
//
// Exynos - CPU PMU(Power Management Unit) support

#include <linux/array_size.h>
#include <linux/bitmap.h>
#include <linux/cpuhotplug.h>
#include <linux/cpu_pm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>

#include <linux/soc/samsung/exynos-regs-pmu.h>
#include <linux/soc/samsung/exynos-pmu.h>

#include "exynos-pmu.h"

struct exynos_pmu_context {
	struct device *dev;
	const struct exynos_pmu_data *pmu_data;
	struct regmap *pmureg;
	struct regmap *pmuintrgen;
	/*
	 * Serialization lock for CPU hot plug and cpuidle ACPM hint
	 * programming. Also protects in_cpuhp, sys_insuspend & sys_inreboot
	 * flags.
	 */
	raw_spinlock_t cpupm_lock;
	unsigned long *in_cpuhp;
	bool sys_insuspend;
	bool sys_inreboot;
};

void __iomem *pmu_base_addr;
static struct exynos_pmu_context *pmu_context;
/* forward declaration */
static struct platform_driver exynos_pmu_driver;

void pmu_raw_writel(u32 val, u32 offset)
{
	writel_relaxed(val, pmu_base_addr + offset);
}

u32 pmu_raw_readl(u32 offset)
{
	return readl_relaxed(pmu_base_addr + offset);
}

void exynos_sys_powerdown_conf(enum sys_powerdown mode)
{
	unsigned int i;
	const struct exynos_pmu_data *pmu_data;

	if (!pmu_context || !pmu_context->pmu_data)
		return;

	pmu_data = pmu_context->pmu_data;

	if (pmu_data->powerdown_conf)
		pmu_data->powerdown_conf(mode);

	if (pmu_data->pmu_config) {
		for (i = 0; (pmu_data->pmu_config[i].offset != PMU_TABLE_END); i++)
			pmu_raw_writel(pmu_data->pmu_config[i].val[mode],
					pmu_data->pmu_config[i].offset);
	}

	if (pmu_data->powerdown_conf_extra)
		pmu_data->powerdown_conf_extra(mode);

	if (pmu_data->pmu_config_extra) {
		for (i = 0; pmu_data->pmu_config_extra[i].offset != PMU_TABLE_END; i++)
			pmu_raw_writel(pmu_data->pmu_config_extra[i].val[mode],
				       pmu_data->pmu_config_extra[i].offset);
	}
}

/*
 * Split the data between ARM architectures because it is relatively big
 * and useless on other arch.
 */
#ifdef CONFIG_EXYNOS_PMU_ARM_DRIVERS
#define exynos_pmu_data_arm_ptr(data)	(&data)
#else
#define exynos_pmu_data_arm_ptr(data)	NULL
#endif

static const struct regmap_config regmap_smccfg = {
	.name = "pmu_regs",
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.fast_io = true,
	.use_single_read = true,
	.use_single_write = true,
	.reg_read = tensor_sec_reg_read,
	.reg_write = tensor_sec_reg_write,
	.reg_update_bits = tensor_sec_update_bits,
	.use_raw_spinlock = true,
};

static const struct regmap_config regmap_pmu_intr = {
	.name = "pmu_intr_gen",
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.use_raw_spinlock = true,
};

/*
 * PMU platform driver and devicetree bindings.
 */
static const struct of_device_id exynos_pmu_of_device_ids[] = {
	{
		.compatible = "google,gs101-pmu",
		.data = &gs101_pmu_data,
	}, {
		.compatible = "google,zumapro-pmu",
		.data = &zumapro_pmu_data,
	}, {
		.compatible = "samsung,exynos3250-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos3250_pmu_data),
	}, {
		.compatible = "samsung,exynos4210-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos4210_pmu_data),
	}, {
		.compatible = "samsung,exynos4212-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos4212_pmu_data),
	}, {
		.compatible = "samsung,exynos4412-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos4412_pmu_data),
	}, {
		.compatible = "samsung,exynos5250-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos5250_pmu_data),
	}, {
		.compatible = "samsung,exynos5410-pmu",
	}, {
		.compatible = "samsung,exynos5420-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos5420_pmu_data),
	}, {
		.compatible = "samsung,exynos5433-pmu",
	}, {
		.compatible = "samsung,exynos7-pmu",
	}, {
		.compatible = "samsung,exynos850-pmu",
	},
	{ /*sentinel*/ },
};

static const struct mfd_cell exynos_pmu_devs[] = {
	{ .name = "exynos-clkout", },
};

/**
 * exynos_get_pmu_regmap() - Obtain pmureg regmap
 *
 * Find the pmureg regmap previously configured in probe() and return regmap
 * pointer.
 *
 * Return: A pointer to regmap if found or ERR_PTR error value.
 */
struct regmap *exynos_get_pmu_regmap(void)
{
	struct device_node *np = of_find_matching_node(NULL,
						      exynos_pmu_of_device_ids);
	if (np)
		return exynos_get_pmu_regmap_by_phandle(np, NULL);
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(exynos_get_pmu_regmap);

/**
 * exynos_get_pmu_regmap_by_phandle() - Obtain pmureg regmap via phandle
 * @np: Device node holding PMU phandle property
 * @propname: Name of property holding phandle value
 *
 * Find the pmureg regmap previously configured in probe() and return regmap
 * pointer.
 *
 * Return: A pointer to regmap if found or ERR_PTR error value.
 */
struct regmap *exynos_get_pmu_regmap_by_phandle(struct device_node *np,
						const char *propname)
{
	struct device_node *pmu_np;
	struct device *dev;

	if (propname)
		pmu_np = of_parse_phandle(np, propname, 0);
	else
		pmu_np = np;

	if (!pmu_np)
		return ERR_PTR(-ENODEV);

	/*
	 * Determine if exynos-pmu device has probed and therefore regmap
	 * has been created and can be returned to the caller. Otherwise we
	 * return -EPROBE_DEFER.
	 */
	dev = driver_find_device_by_of_node(&exynos_pmu_driver.driver,
					    (void *)pmu_np);

	if (propname)
		of_node_put(pmu_np);

	if (!dev)
		return ERR_PTR(-EPROBE_DEFER);

	put_device(dev);

	return syscon_node_to_regmap(pmu_np);
}
EXPORT_SYMBOL_GPL(exynos_get_pmu_regmap_by_phandle);

/*
 * CPU_INFORM register "hint" values are required to be programmed in addition to
 * the standard PSCI calls to have functional CPU hotplug and CPU idle states.
 * This is required to workaround limitations in the el3mon/ACPM firmware.
 */
#define CPU_INFORM_CLEAR	0
#define CPU_INFORM_C2		1
#define CPU_INFORM_CPD		2
#define CPU_INFORM_SICD		3
#define CPU_INFORM_SLEEP	4
#define PMU_ALLOWED_C2		1

/*
 * __gs101_cpu_pmu_ prefix functions are common code shared by CPU PM notifiers
 * (CPUIdle) and CPU hotplug callbacks. Functions should be called with IRQs
 * disabled and cpupm_lock held.
 */
static int __gs101_cpu_pmu_online(unsigned int cpu)
	__must_hold(&pmu_context->cpupm_lock)
{
	unsigned int cpuhint = smp_processor_id();
	u32 reg, mask;

	/* clear cpu inform hint */
	regmap_write(pmu_context->pmureg, GS101_CPU_INFORM(cpuhint),
		     CPU_INFORM_CLEAR);

	mask = BIT(cpu);

	regmap_update_bits(pmu_context->pmuintrgen, GS101_GRP2_INTR_BID_ENABLE,
			   mask, (0 << cpu));

	regmap_read(pmu_context->pmuintrgen, GS101_GRP2_INTR_BID_UPEND, &reg);

	regmap_write(pmu_context->pmuintrgen, GS101_GRP2_INTR_BID_CLEAR,
		     reg & mask);

	return 0;
}

/* Called from CPU PM notifier (CPUIdle code path) with IRQs disabled */
static int gs101_cpu_pmu_online(void)
{
	int cpu;

	raw_spin_lock(&pmu_context->cpupm_lock);

	if (pmu_context->sys_inreboot) {
		raw_spin_unlock(&pmu_context->cpupm_lock);
		return NOTIFY_OK;
	}

	cpu = smp_processor_id();
	__gs101_cpu_pmu_online(cpu);
	raw_spin_unlock(&pmu_context->cpupm_lock);

	return NOTIFY_OK;
}

/* Called from CPU hot plug callback with IRQs enabled */
static int gs101_cpuhp_pmu_online(unsigned int cpu)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&pmu_context->cpupm_lock, flags);

	__gs101_cpu_pmu_online(cpu);
	/*
	 * Mark this CPU as having finished the hotplug.
	 * This means this CPU can now enter C2 idle state.
	 */
	clear_bit(cpu, pmu_context->in_cpuhp);
	raw_spin_unlock_irqrestore(&pmu_context->cpupm_lock, flags);

	return 0;
}

/* Common function shared by both CPU hot plug and CPUIdle */
static int __gs101_cpu_pmu_offline(unsigned int cpu)
	__must_hold(&pmu_context->cpupm_lock)
{
	unsigned int cpuhint = smp_processor_id();
	u32 reg, mask;

	/* set cpu inform hint */
	regmap_write(pmu_context->pmureg, GS101_CPU_INFORM(cpuhint),
		     CPU_INFORM_C2);

	mask = BIT(cpu);
	regmap_update_bits(pmu_context->pmuintrgen, GS101_GRP2_INTR_BID_ENABLE,
			   mask, BIT(cpu));

	regmap_read(pmu_context->pmuintrgen, GS101_GRP1_INTR_BID_UPEND, &reg);
	regmap_write(pmu_context->pmuintrgen, GS101_GRP1_INTR_BID_CLEAR,
		     reg & mask);

	mask = (BIT(cpu + 8));
	regmap_read(pmu_context->pmuintrgen, GS101_GRP1_INTR_BID_UPEND, &reg);
	regmap_write(pmu_context->pmuintrgen, GS101_GRP1_INTR_BID_CLEAR,
		     reg & mask);

	return 0;
}

/* Called from CPU PM notifier (CPUIdle code path) with IRQs disabled */
static int gs101_cpu_pmu_offline(void)
{
	int cpu;

	raw_spin_lock(&pmu_context->cpupm_lock);
	cpu = smp_processor_id();

	if (test_bit(cpu, pmu_context->in_cpuhp)) {
		raw_spin_unlock(&pmu_context->cpupm_lock);
		return NOTIFY_BAD;
	}

	/* Ignore CPU_PM_ENTER event in reboot or suspend sequence. */
	if (pmu_context->sys_insuspend || pmu_context->sys_inreboot) {
		raw_spin_unlock(&pmu_context->cpupm_lock);
		return NOTIFY_OK;
	}

	__gs101_cpu_pmu_offline(cpu);
	raw_spin_unlock(&pmu_context->cpupm_lock);

	return NOTIFY_OK;
}

/* Called from CPU hot plug callback with IRQs enabled */
static int gs101_cpuhp_pmu_offline(unsigned int cpu)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&pmu_context->cpupm_lock, flags);
	/*
	 * Mark this CPU as entering hotplug. So as not to confuse
	 * ACPM the CPU entering hotplug should not enter C2 idle state.
	 */
	set_bit(cpu, pmu_context->in_cpuhp);
	__gs101_cpu_pmu_offline(cpu);

	raw_spin_unlock_irqrestore(&pmu_context->cpupm_lock, flags);

	return 0;
}

static int gs101_cpu_pm_notify_callback(struct notifier_block *self,
					unsigned long action, void *v)
{
	switch (action) {
	case CPU_PM_ENTER:
		return gs101_cpu_pmu_offline();

	case CPU_PM_EXIT:
		return gs101_cpu_pmu_online();
	}

	return NOTIFY_OK;
}

static struct notifier_block gs101_cpu_pm_notifier = {
	.notifier_call = gs101_cpu_pm_notify_callback,
	/*
	 * We want to be called first, as the ACPM hint and handshake is what
	 * puts the CPU into C2.
	 */
	.priority = INT_MAX
};

static int exynos_cpupm_reboot_notifier(struct notifier_block *nb,
					unsigned long event, void *v)
{
	unsigned long flags;

	switch (event) {
	case SYS_POWER_OFF:
	case SYS_RESTART:
		raw_spin_lock_irqsave(&pmu_context->cpupm_lock, flags);
		pmu_context->sys_inreboot = true;
		raw_spin_unlock_irqrestore(&pmu_context->cpupm_lock, flags);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block exynos_cpupm_reboot_nb = {
	.priority = INT_MAX,
	.notifier_call = exynos_cpupm_reboot_notifier,
};

/*
 * Build a raw-spinlock mmio regmap for the pmu-intr-gen block referenced by the
 * "google,pmu-intr-gen-syscon" phandle.  A raw spinlock (rather than the syscon
 * default) is needed because the CPU PM notifiers / sleep syscore path that
 * touch it run with IRQs disabled.  Older DTs without the phandle leave
 * pmu_context->pmuintrgen NULL and the function still succeeds.
 */
static int init_pmu_intr_gen_regmap(struct device *dev)
{
	struct device_node *intr_gen_node;
	struct resource intrgen_res;
	void __iomem *virt_addr;
	int ret;

	intr_gen_node = of_parse_phandle(dev->of_node,
					 "google,pmu-intr-gen-syscon", 0);
	if (!intr_gen_node) {
		dev_warn(dev, "pmu-intr-gen syscon unavailable\n");
		return 0;
	}

	ret = of_address_to_resource(intr_gen_node, 0, &intrgen_res);
	if (ret)
		goto out;

	virt_addr = devm_ioremap(dev, intrgen_res.start,
				 resource_size(&intrgen_res));
	if (!virt_addr) {
		ret = -ENOMEM;
		goto out;
	}

	pmu_context->pmuintrgen = devm_regmap_init_mmio(dev, virt_addr,
							&regmap_pmu_intr);
	if (IS_ERR(pmu_context->pmuintrgen)) {
		ret = PTR_ERR(pmu_context->pmuintrgen);
		pmu_context->pmuintrgen = NULL;
		dev_err(dev, "failed to initialize pmu-intr-gen regmap\n");
		goto out;
	}

	/* register custom mmio regmap with syscon */
	ret = of_syscon_register_regmap(intr_gen_node, pmu_context->pmuintrgen);
out:
	of_node_put(intr_gen_node);
	return ret;
}

static int setup_cpuhp_and_cpuidle(struct device *dev)
{
	int ret, cpu;

	ret = init_pmu_intr_gen_regmap(dev);
	if (ret)
		return ret;

	/* older DT without the intr-gen phandle: skip the cpuhp/cpuidle glue */
	if (!pmu_context->pmuintrgen)
		return 0;

	pmu_context->in_cpuhp = devm_bitmap_zalloc(dev, num_possible_cpus(),
						   GFP_KERNEL);
	if (!pmu_context->in_cpuhp)
		return -ENOMEM;

	/* set PMU to power on */
	for_each_online_cpu(cpu)
		gs101_cpuhp_pmu_online(cpu);

	/* register CPU hotplug callbacks */
	cpuhp_setup_state(CPUHP_BP_PREPARE_DYN,	"soc/exynos-pmu:prepare",
			  gs101_cpuhp_pmu_online, NULL);

	cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "soc/exynos-pmu:online",
			  NULL, gs101_cpuhp_pmu_offline);

	/* register CPU PM notifiers for cpuidle */
	cpu_pm_register_notifier(&gs101_cpu_pm_notifier);
	register_reboot_notifier(&exynos_cpupm_reboot_nb);
	return 0;
}

/*
 * Zumapro's firmware does not program the PMU wakeup interrupt enables for the
 * powered-down system sleep state (SYS_SLEEP, suspend_mode_idx 8).  Arm them
 * around system suspend so the SoC has a valid wake source; the external/pin
 * EINT masks are programmed separately by pinctrl.  Values and offsets mirror
 * the downstream exynos-pm "wakeup_int_en"/"wakeup_int_en_offset" DT (also
 * confirmed against a live device-tree dump):
 *
 *   WAKEUP_INT_EN  (0x3944) = 0x1d0bf  - bits 0-3 RTC/TRTC, 4 EINT, 5 EINT_FAR,
 *                                        16 USB_REWA, 12/14 PCIe (the button
 *                                        wake is bit 4/5)
 *   WAKEUP2_INT_EN (0x3964) = 0x01f0   - bits 4-8 APM/AOC/DBGCORE mailboxes
 *
 * Note WAKEUP2_STAT lives at 0x3970 on zumapro (not the gs101 0x3954).  The
 * earlier 0xff00000 here was wrong for sleep: those are the CLUSTER*_CPU*
 * GIC-wakeup bits (20-27), which only matter for an idle/SICD cluster - in
 * deep sleep the CPUs and GIC are off, so none of them can ever fire.
 */
static const struct {
	unsigned int stat_reg;
	unsigned int en_reg;
	u32 mask;
} zumapro_wakeup_mask[] = {
	{ GS101_WAKEUP_STAT, GS101_TOP_INT_EN, 0x1d0bf },
	{ 0x3970, GS101_WAKEUP2_INT_EN, 0x1f0 },
};

static void zumapro_set_wakeup_mask(bool arm)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(zumapro_wakeup_mask); i++) {
		regmap_write(pmu_context->pmureg,
			     zumapro_wakeup_mask[i].stat_reg, 0);
		regmap_write(pmu_context->pmureg, zumapro_wakeup_mask[i].en_reg,
			     arm ? zumapro_wakeup_mask[i].mask : 0);
	}
}

static struct cpumask zumapro_idle_cpus;
/* cpu currently asserting system idle (SICD), or -1 if none */
static int zumapro_sicd_holder = -1;
/* debug counters, printed once per resume */
static u32 zumapro_dbg_c2, zumapro_dbg_sicd, zumapro_dbg_fail;

/*
 * Plain PSCI does not keep zumapro's cores powered down during system idle: the
 * firmware needs every idling core to publish its idle intent through its
 * CPU_INFORM register, and the last core down to request system idle (SICD)
 * with the wakeup mask armed (see zumapro_set_wakeup_mask()); otherwise it
 * powers the cores straight back up.  Mirrors the downstream exynos-cpupm
 * CPU_INFORM hints; no pmu-intr-gen handshake is needed (downstream does not
 * touch it on the idle-enter path).  Only active inside a system suspend, when
 * the mask is armed and all cores are parking; awake idle uses standard PSCI.
 */
static int zumapro_cpu_pm_notify(struct notifier_block *self,
				 unsigned long action, void *v)
{
	unsigned int cpu = smp_processor_id();
	u32 hint;

	raw_spin_lock(&pmu_context->cpupm_lock);

	/*
	 * SICD is the suspend-to-idle (all-cores-parked) hint only.  In deep
	 * sleep (PM_SUSPEND_MEM) the secondary cores are hotplugged off and the
	 * boot core powers down via PSCI SYSTEM_SUSPEND into SYS_SLEEP, which is
	 * a different and deeper power mode - leaving a stale SICD hint here
	 * would tell the firmware the wrong thing.  Only elect a holder for
	 * suspend-to-idle.
	 */
	if (!pmu_context->sys_insuspend ||
	    pm_suspend_target_state != PM_SUSPEND_TO_IDLE) {
		raw_spin_unlock(&pmu_context->cpupm_lock);
		return NOTIFY_OK;
	}

	switch (action) {
	case CPU_PM_ENTER:
		cpumask_set_cpu(cpu, &zumapro_idle_cpus);
		/*
		 * Elect a single SICD holder: the first core to go idle claims
		 * system idle, the rest report plain C2.  Exactly one SICD hint
		 * (plus the armed wakeup mask) is what makes the firmware hold
		 * the cluster; a kernel cpumask "all idle" test never fires
		 * because the cores enter/exit faster than they ever coincide.
		 */
		if (zumapro_sicd_holder < 0) {
			zumapro_sicd_holder = cpu;
			hint = CPU_INFORM_SICD;
			zumapro_dbg_sicd++;
		} else {
			hint = CPU_INFORM_C2;
			zumapro_dbg_c2++;
		}
		if (regmap_write(pmu_context->pmureg, GS101_CPU_INFORM(cpu), hint))
			zumapro_dbg_fail++;
		break;
	case CPU_PM_EXIT:
		cpumask_clear_cpu(cpu, &zumapro_idle_cpus);
		if (zumapro_sicd_holder == cpu)
			zumapro_sicd_holder = -1;
		regmap_write(pmu_context->pmureg, GS101_CPU_INFORM(cpu),
			     CPU_INFORM_CLEAR);
		break;
	}

	raw_spin_unlock(&pmu_context->cpupm_lock);
	return NOTIFY_OK;
}

static struct notifier_block zumapro_cpu_pm_notifier = {
	.notifier_call = zumapro_cpu_pm_notify,
	.priority = INT_MAX,
};

/*
 * Deep sleep (SYS_SLEEP) entry programming.  Beyond the wakeup-source enables
 * (zumapro_set_wakeup_mask()), the firmware needs the boot core's idle intent
 * published in CPU_INFORM and its wakeup routed back through the PMU interrupt
 * generator, otherwise PSCI SYSTEM_SUSPEND powers the SoC down with no path to
 * wake it.  This mirrors the downstream cal_pm_enter(SYS_SLEEP) "enter"
 * sequence: a CPU_INFORM[0]=SLEEP hint and CLUSTER0_CPU0_INT_EN go through the
 * secured PMU (SMC) regmap, the GRP*_INTR_BID registers are plain mmio in the
 * non-secured intr-gen block.
 */
static void zumapro_sys_sleep_arm(bool arm)
{
	unsigned int cl0_int_en =
		GS101_CLUSTER_CPU_INT_EN(GS101_CLUSTER0_OFFSET, 0);
	u32 reg, rb;
	int ret;

	if (arm) {
		/*
		 * Only the boot core's hint is set here, exactly like downstream
		 * cal_pm_enter(SYS_SLEEP) (enter_sleep writes CPU_INFORM0=SLEEP
		 * and nothing else for the secondaries).  The secondary cores'
		 * CPU_INFORM is owned by the firmware: gs101_cpuhp_pmu_offline()
		 * sets each core's C2 hint on the dying CPU before PSCI CPU_OFF,
		 * and the ACPM stamps the "powered-down" ack (bit16) as it
		 * completes the powerdown - captured live as 0x10001/0x10002 on
		 * downstream.  Writing the secondaries here at syscore (after
		 * hotplug) only clobbers that firmware-coherent state, so don't.
		 */
		regmap_write(pmu_context->pmureg, GS101_CPU_INFORM(0),
			     CPU_INFORM_SLEEP);

		regmap_update_bits(pmu_context->pmuintrgen,
				   GS101_GRP2_INTR_BID_ENABLE, BIT(0), BIT(0));
		regmap_read(pmu_context->pmuintrgen,
			    GS101_GRP1_INTR_BID_UPEND, &reg);
		regmap_write(pmu_context->pmuintrgen,
			     GS101_GRP1_INTR_BID_CLEAR, reg & BIT(0));

		/*
		 * Plain read-modify-write (the WRITE SMC), not
		 * regmap_update_bits(): for PMU_ALIVE registers the secure
		 * regmap routes update_bits through the hardware set/clear-bit
		 * alias, which the firmware does not honour for this register -
		 * the bit silently never latches.  Downstream's pmucal issues a
		 * plain masked write here too.
		 */
		regmap_read(pmu_context->pmureg, cl0_int_en, &reg);
		ret = regmap_write(pmu_context->pmureg, cl0_int_en, reg | BIT(3));
		regmap_read(pmu_context->pmureg, cl0_int_en, &rb);
		if (ret || !(rb & BIT(3)))
			pr_warn("zumapro: CLUSTER0_CPU0_INT_EN bit3 did not latch: before=0x%x write_ret=%d readback=0x%x\n",
				reg, ret, rb);
	} else {
		regmap_read(pmu_context->pmureg, cl0_int_en, &reg);
		regmap_write(pmu_context->pmureg, cl0_int_en, reg & ~BIT(3));
		regmap_update_bits(pmu_context->pmuintrgen,
				   GS101_GRP2_INTR_BID_ENABLE, BIT(0), 0);
		regmap_write(pmu_context->pmureg, GS101_CPU_INFORM(0),
			     CPU_INFORM_CLEAR);
	}
}

/*
 * syscore .suspend/.resume run only for deep sleep (PM_SUSPEND_MEM); the
 * suspend-to-idle path never reaches syscore_suspend(), so no state check is
 * needed here.  They run on the boot CPU with IRQs disabled and the secondary
 * cores already hotplugged off - the right moment to arm the SYS_SLEEP entry.
 */
/*
 * Mirror of the downstream zumapro_sleep_dump() register set (exynos-pm.c) so
 * the two SLEEPDUMP logs line up for a direct diff.  Mainline can only read
 * PMU_ALIVE through the secure (SMC) regmap - a direct MMIO read SErrors - so
 * only the smc column is emitted; it compares against downstream's smc= column.
 * Literal offsets (not the GS101_* macros) so CPU_INFORM8 at 0x0880 is reachable.
 */
static const struct {
	u32 off;
	const char *name;
} zumapro_sleep_dump_regs[] = {
	{ 0x0860, "CPU_INFORM0" }, { 0x0864, "CPU_INFORM1" },
	{ 0x0868, "CPU_INFORM2" }, { 0x086c, "CPU_INFORM3" },
	{ 0x0870, "CPU_INFORM4" }, { 0x0874, "CPU_INFORM5" },
	{ 0x0878, "CPU_INFORM6" }, { 0x087c, "CPU_INFORM7" },
	{ 0x0880, "CPU_INFORM8" },
	{ 0x1004, "CLUSTER0_CPU0_STATUS" },
	{ 0x1008, "CLUSTER0_CPU0_STATES" },
	{ 0x1044, "CLUSTER0_CPU0_INT_EN" },
	{ 0x1048, "CLUSTER0_CPU0_INT_TYPE" },
	{ 0x104c, "CLUSTER0_CPU0_INT_DIR" },
	{ 0x3920, "TOP_OUT_STATE" },
	{ 0x3940, "WAKEUP_STAT" },
	{ 0x3944, "WAKEUP_INT_EN" },
	{ 0x3964, "WAKEUP2_INT_EN" },
	{ 0x3970, "WAKEUP2_STAT" },
	{ 0x3a10, "SYSTEM_CTRL" },
	{ 0x3a80, "EINT_WAKEUP_MASK0" },
	{ 0x3a84, "EINT_WAKEUP_MASK1" },
	{ 0x3a88, "EINT_WAKEUP_MASK2" },
	{ 0x3cb0, "DUR_MIF" },
	{ 0x3cc8, "DUR_TCXO" },
};

static void zumapro_sleep_dump(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(zumapro_sleep_dump_regs); i++) {
		u32 smc = 0xdeadbeef;

		regmap_read(pmu_context->pmureg,
			    zumapro_sleep_dump_regs[i].off, &smc);
		pr_info("SLEEPDUMP %-24s(0x%04x) smc=0x%08x\n",
			zumapro_sleep_dump_regs[i].name,
			zumapro_sleep_dump_regs[i].off, smc);
	}
}

/*
 * CMU_TOP (0x26040000) fabric-feed gates that clk-zumapro.c pins CLK_IS_CRITICAL
 * for bring-up (console UART, UFS, USB, MCT).  Critical clocks are never gated
 * by CCF, even in suspend, so the ACPM cannot quiesce the shared peripheral NOC
 * fabric to power it down during SYS_SLEEP - it spin-waits on a domain status
 * bit (observed: APM watchdog stuck at PMU+0x2ca4, PERIC1 region) and resets.
 * Gate these directly here (behind CCF's back, bit21 = clock enable), after the
 * console has flushed, and restore on resume.  Test whether quiescing the fabric
 * lets the ACPM complete the down sequence.
 */
static void __iomem *zumapro_cmu_top_base;
static const struct {
	u32 off;
	const char *name;
} zumapro_sleep_gate_feeds[] = {
	{ 0x20bc, "HSI0_NOC" },
	{ 0x20d8, "HSI2_NOC" },
	{ 0x20e0, "HSI2_UFS_EMBD" },
	{ 0x20f4, "MISC_NOC" },
	{ 0x2110, "PERIC0_IP" },
	{ 0x2114, "PERIC0_NOC" },
};
static u32 zumapro_sleep_gate_saved[ARRAY_SIZE(zumapro_sleep_gate_feeds)];
#define ZUMAPRO_CMU_GATE_EN	BIT(21)

static void zumapro_sleep_gate_feeds_set(bool gate)
{
	int i;

	if (!zumapro_cmu_top_base)
		return;

	for (i = 0; i < ARRAY_SIZE(zumapro_sleep_gate_feeds); i++) {
		void __iomem *r = zumapro_cmu_top_base +
				  zumapro_sleep_gate_feeds[i].off;

		if (gate) {
			zumapro_sleep_gate_saved[i] = readl(r);
			writel(zumapro_sleep_gate_saved[i] & ~ZUMAPRO_CMU_GATE_EN, r);
		} else {
			writel(zumapro_sleep_gate_saved[i], r);
		}
	}
}

static int zumapro_sys_sleep_suspend(void *data)
{
	zumapro_sys_sleep_arm(true);
	zumapro_sleep_dump();
	/* last thing before PSCI: gate the critical fabric feeds (console dies) */
	zumapro_sleep_gate_feeds_set(true);
	return 0;
}

static void zumapro_sys_sleep_resume(void *data)
{
	/* restore the gated fabric feeds first - the console UART needs PERIC0
	 * back before the pr_emerg below can reach the wire */
	zumapro_sleep_gate_feeds_set(false);

	/*
	 * TEMPORARY deep-wake diagnostic.  With no pstore/RTC, and the console
	 * UART clock (plus other peripherals) very likely lost across the
	 * SYS_SLEEP power-down and not restored on this bring-up, we cannot
	 * tell "never woke" from "woke but resumed blind".  If the CPU reaches
	 * here at all, the wake fired - so force a reset: a successful wake
	 * reboots the phone (visible from the bootloader); a no-wake stays
	 * dark.  Remove once the wake path is confirmed.
	 */
	pr_emerg("zumapro: DEEP RESUME REACHED - wake fired, forcing reboot\n");
	emergency_restart();

	zumapro_sys_sleep_arm(false);
}

static const struct syscore_ops zumapro_sys_sleep_syscore_ops = {
	.suspend = zumapro_sys_sleep_suspend,
	.resume  = zumapro_sys_sleep_resume,
};

static struct syscore zumapro_sys_sleep_syscore = {
	.ops = &zumapro_sys_sleep_syscore_ops,
};

/*
 * Boot-time PMU power-sequencing for SYS_SLEEP wake.  Downstream's cmupmucal
 * runs pmucal_lpm_init[] at boot; mainline has no pmucal, so these are never
 * programmed (confirmed on hardware: the non-secure EARLY_WAKEUP siblings read
 * 0 on mainline vs the downstream values).  The EXT_REGULATOR/TCXO settle
 * durations tell the PMU power-up sequencer how long to wait for each PMIC rail
 * and the TCXO to stabilise on wake; left at reset the sequencer cannot bring
 * the SoC back, so after PSCI SYSTEM_SUSPEND it powers down and never returns.
 *
 * Offsets are in the secured PMU_ALIVE block (pmureg / EL3 SMC).  EL3 enforces
 * its own write allowlist and rejects unknown offsets with -EINVAL, and direct
 * MMIO reads SError - so write through pmureg and read back through pmureg (the
 * SMC read path) to confirm what actually landed.  Values from downstream
 * flexpmu_cal_system_zuma.h pmucal_lpm_init[].
 */
static const struct {
	unsigned int reg;
	u32 val;
	const char *name;
} zumapro_lpm_durations[] = {
	{ 0x3cb0, 0x244, "EXT_REGULATOR_MIF" },
	{ 0x3cb4, 0x0a0, "EXT_REGULATOR_TOP" },
	{ 0x3cb8, 0x0a0, "EXT_REGULATOR_CPUCL2" },
	{ 0x3cbc, 0x0a0, "EXT_REGULATOR_CPUCL1" },
	{ 0x3cc0, 0x0be, "EXT_REGULATOR_G3D" },
	{ 0x3cc4, 0x0a0, "EXT_REGULATOR_TPU" },
	{ 0x3cc8, 0x66c, "TCXO" },	/* hw mask 0xfffff; upper bits reset 0 */
};

/*
 * The non-secure half of the wake-critical lpm_init subset: CPU-cluster, DRAM
 * (MIF) and interconnect (NOCL) clock-recovery setup plus the EARLY_WAKEUP
 * routing.  Restricted to register blocks that are always powered while the CPU
 * runs and that were hardware-probed accessible on mainline (CPUCL0/1/2, MIF,
 * NOCL, the EARLY_WAKEUP CMU).  The downstream table writes every lpm_init entry
 * UNCONDITIONALLY (PMUCAL_WRITE ignores the cond fields), so accelerator blocks
 * mainline may power-gate off (G3D/TPU/AUR/ISP) are deliberately omitted - a
 * write into an unpowered block async-SErrors.  Values are 1:1 from
 * flexpmu_cal_system_zuma.h; mask != ~0 is a read-modify-write of those bits,
 * exactly as pmucal_rae_write().
 */
static const struct {
	u32 base;
	u32 offset;
	u32 mask;
	u32 val;
	const char *name;
} zumapro_lpm_clocks[] = {
	/* CPUCL0 (boot cluster) */
	{ 0x29c00000, 0x085c, 0x7f, 0x121, "CPUCL0_HCHGEN_CPU" },
	{ 0x29c00000, 0x0854, 0x7f, 0x101, "CPUCL0_HCHGEN_DSU" },
	{ 0x29c00000, 0x0864, 0x7f, 0x101, "CPUCL0_HCHGEN_BCI" },
	{ 0x29c00000, 0x0834, ~0u, 0x8000ffff, "CPUCL0_CLKDIVSTEP_SMPL_FLT" },
	{ 0x29c00000, 0x0838, ~0u, 0x00f041c3, "CPUCL0_CLKDIVSTEP_CON" },
	{ 0x29c00000, 0x0830, ~0u, 0x00210047, "CPUCL0_CLKDIVSTEP" },
	{ 0x29c00000, 0x0824, 0x1, 0x1, "CPUCL0_SHORTSTOP_DBG" },
	{ 0x29c00000, 0x0820, 0x1, 0x1, "CPUCL0_SHORTSTOP" },
	{ 0x29c20000, 0x0104, ~0u, ~0u, "CPUCL0_DSU_DRCG_EN" },
	{ 0x29c20000, 0x010c, ~0u, ~0u, "CPUCL0_DSU_DRCG_EN_INT" },
	/* CPUCL1 */
	{ 0x29d00000, 0x0854, 0x7f, 0x121, "CPUCL1_HCHGEN_CPU" },
	{ 0x29d00000, 0x083c, ~0u, 0x8000ffff, "CPUCL1_CLKDIVSTEP_SMPL_FLT" },
	{ 0x29d00000, 0x0834, ~0u, 0xc007f8ff, "CPUCL1_CLKDIVSTEP_OCP_FLT" },
	{ 0x29d00000, 0x0838, ~0u, 0xc007f8ff, "CPUCL1_CLKDIVSTEP_VDROOP_FLT" },
	{ 0x29d00000, 0x0840, ~0u, 0xfff041c0, "CPUCL1_CLKDIVSTEP_CON_HEAVY" },
	{ 0x29d00000, 0x0844, ~0u, 0x00f041c3, "CPUCL1_CLKDIVSTEP_CON_LIGHT" },
	{ 0x29d00000, 0x0830, ~0u, 0x00210447, "CPUCL1_CLKDIVSTEP" },
	{ 0x29d00000, 0x0820, 0x1, 0x1, "CPUCL1_SHORTSTOP" },
	/* CPUCL2 */
	{ 0x29d80000, 0x0854, 0x7f, 0x121, "CPUCL2_HCHGEN_CPU" },
	{ 0x29d80000, 0x083c, ~0u, 0x8000ffff, "CPUCL2_CLKDIVSTEP_SMPL_FLT" },
	{ 0x29d80000, 0x0834, ~0u, 0xc007f8ff, "CPUCL2_CLKDIVSTEP_OCP_FLT" },
	{ 0x29d80000, 0x0838, ~0u, 0xc007f8ff, "CPUCL2_CLKDIVSTEP_VDROOP_FLT" },
	{ 0x29d80000, 0x0840, ~0u, 0xfff041c0, "CPUCL2_CLKDIVSTEP_CON_HEAVY" },
	{ 0x29d80000, 0x0844, ~0u, 0x00f041c3, "CPUCL2_CLKDIVSTEP_CON_LIGHT" },
	{ 0x29d80000, 0x0830, ~0u, 0x00210447, "CPUCL2_CLKDIVSTEP" },
	{ 0x29d80000, 0x0820, 0x1, 0x1, "CPUCL2_SHORTSTOP" },
	/* MIF (DRAM) x4 */
	{ 0x27c00000, 0x0850, 0x1, 0x1, "MIF0_HCHGEN_CMUREF" },
	{ 0x27d00000, 0x0850, 0x1, 0x1, "MIF1_HCHGEN_CMUREF" },
	{ 0x27e00000, 0x0850, 0x1, 0x1, "MIF2_HCHGEN_CMUREF" },
	{ 0x27f00000, 0x0850, 0x1, 0x1, "MIF3_HCHGEN_CMUREF" },
	{ 0x27c00000, 0x0820, 0x1, 0x1, "MIF0_SHORTSTOP" },
	{ 0x27d00000, 0x0820, 0x1, 0x1, "MIF1_SHORTSTOP" },
	{ 0x27e00000, 0x0820, 0x1, 0x1, "MIF2_SHORTSTOP" },
	{ 0x27f00000, 0x0820, 0x1, 0x1, "MIF3_SHORTSTOP" },
	{ 0x27c40000, 0xf240, BIT(31), 0, "MIF0_PWRMGMT" },
	{ 0x27d40000, 0xf240, BIT(31), 0, "MIF1_PWRMGMT" },
	{ 0x27e40000, 0xf240, BIT(31), 0, "MIF2_PWRMGMT" },
	{ 0x27f40000, 0xf240, BIT(31), 0, "MIF3_PWRMGMT" },
	{ 0x27c20000, 0x0104, ~0u, ~0u, "MIF0_DRCG_EN" },
	{ 0x27d20000, 0x0104, ~0u, ~0u, "MIF1_DRCG_EN" },
	{ 0x27e20000, 0x0104, ~0u, ~0u, "MIF2_DRCG_EN" },
	{ 0x27f20000, 0x0104, ~0u, ~0u, "MIF3_DRCG_EN" },
	/* NOCL (interconnect) */
	{ 0x26000000, 0x0840, 0x3f, 0x1, "NOCL0_HCHGEN_CMUREF" },
	{ 0x26400000, 0x0840, 0x1, 0x1, "NOCL1A_HCHGEN_CMUREF" },
	{ 0x26800000, 0x0840, 0x3f, 0x1, "NOCL1B_HCHGEN_CMUREF" },
	{ 0x26c00000, 0x0840, 0x1, 0x1, "NOCL2AA_HCHGEN_CMUREF" },
	{ 0x27000000, 0x0840, 0x1, 0x1, "NOCL2AB_HCHGEN_CMUREF" },
	{ 0x26000000, 0x0820, 0x1, 0x1, "NOCL0_SHORTSTOP" },
	{ 0x26020000, 0x0104, ~0u, ~0u, "NOCL0_DRCG_EN" },
	{ 0x26420000, 0x0104, ~0u, ~0u, "NOCL1A_DRCG_EN" },
	{ 0x26820000, 0x0104, ~0u, ~0u, "NOCL1B_DRCG_EN" },
	{ 0x26c20000, 0x0104, ~0u, ~0u, "NOCL2AA_DRCG_EN" },
	{ 0x27020000, 0x0104, ~0u, ~0u, "NOCL2AB_DRCG_EN" },
	/* EARLY_WAKEUP CMU (0x26040000, devmem-confirmed accessible on mainline) */
	{ 0x26040000, 0x0850, 0x1, 0x1, "CMU_HCHGEN_CLKMUX" },
	{ 0x26040000, 0x0880, 0x1, 0x1, "EARLY_WAKEUP_DPU_CTRL" },
	{ 0x26040000, 0x0898, ~0u, 0xfe, "EARLY_WAKEUP_DPU_DEST" },
	{ 0x26040000, 0x0884, 0x1, 0x1, "EARLY_WAKEUP_ISPFE_CTRL" },
	{ 0x26040000, 0x089c, ~0u, 0xfe, "EARLY_WAKEUP_ISPFE_DEST" },
	{ 0x26040000, 0x0888, 0x1, 0x1, "EARLY_WAKEUP_GSE_CTRL" },
	{ 0x26040000, 0x08a0, ~0u, 0x4fc, "EARLY_WAKEUP_GSE_DEST" },
	/*
	 * Always-on / sysreg-domain DRCG enables.  Unlike the accelerator and
	 * display CMUs (G3D/TPU/AUR/ISP/DPU - deliberately omitted, may be
	 * power-gated off on mainline), these CMU SFR blocks live in always-on
	 * domains and stay accessible whenever the SoC runs, so downstream's
	 * lpm_init writes them unconditionally.  Routed as plain MMIO: their
	 * base >> 16 != 0x1546, so pmucal's pmucal_write_reg() uses writel(),
	 * not the secure SMC (APM at 0x1542xxxx is adjacent to but distinct from
	 * the secured PMU_ALIVE at 0x1546xxxx).  Keeping these clocks recoverable
	 * across the SYS_SLEEP power cycle is the remaining untested lpm_init
	 * subset for the wake path.
	 */
	{ 0x15420000, 0x0104, ~0u, ~0u, "APM_DRCG_EN" },
	{ 0x10030000, 0x0104, ~0u, ~0u, "SYSREG_MISC_DRCG_EN" },
	{ 0x11020000, 0x0104, ~0u, ~0u, "HSI0_DRCG_EN" },	/* USB, bootloader-on */
	{ 0x13020000, 0x0104, ~0u, ~0u, "HSI2_DRCG_EN" },	/* UFS, bootloader-on */
	{ 0x10820000, 0x0104, ~0u, ~0u, "PERIC0_DRCG_EN" },	/* console UART */
	{ 0x10c20000, 0x0104, ~0u, ~0u, "PERIC1_DRCG_EN" },
	/*
	 * HSI1 (BLK_HSI1, 0x12020000) is omitted: it carries only PCIe, which
	 * has no mainline consumer and no power-domain model, so we cannot
	 * confirm the bootloader leaves BLK_HSI1 powered.  A direct MMIO write
	 * into an unpowered block async-SErrors (unguardable), so add it only
	 * after devmem confirms 0x12020104 reads back without faulting.
	 */
};

static void zumapro_pmu_lpm_init(struct device *dev)
{
	unsigned int rb;
	int i, ret;

	/* Secure PMU_ALIVE rail/TCXO settle durations (via the EL3 SMC regmap). */
	for (i = 0; i < ARRAY_SIZE(zumapro_lpm_durations); i++) {
		ret = regmap_write(pmu_context->pmureg,
				   zumapro_lpm_durations[i].reg,
				   zumapro_lpm_durations[i].val);
		rb = 0xdeadbeef;
		regmap_read(pmu_context->pmureg,
			    zumapro_lpm_durations[i].reg, &rb);
		dev_info(dev, "lpm_init %s_DURATION(0x%04x)=0x%x: write ret=%d readback=0x%x\n",
			 zumapro_lpm_durations[i].name,
			 zumapro_lpm_durations[i].reg,
			 zumapro_lpm_durations[i].val, ret, rb);
	}

	/* Non-secure CPU/MIF/NOCL clock recovery + EARLY_WAKEUP (plain MMIO). */
	for (i = 0; i < ARRAY_SIZE(zumapro_lpm_clocks); i++) {
		void __iomem *va = ioremap(zumapro_lpm_clocks[i].base, 0x10000);
		u32 v;

		if (!va) {
			dev_warn(dev, "lpm_init %s: ioremap 0x%08x failed\n",
				 zumapro_lpm_clocks[i].name,
				 zumapro_lpm_clocks[i].base);
			continue;
		}
		if (zumapro_lpm_clocks[i].mask == ~0u) {
			writel(zumapro_lpm_clocks[i].val,
			       va + zumapro_lpm_clocks[i].offset);
		} else {
			v = readl(va + zumapro_lpm_clocks[i].offset);
			v &= ~zumapro_lpm_clocks[i].mask;
			v |= zumapro_lpm_clocks[i].val & zumapro_lpm_clocks[i].mask;
			writel(v, va + zumapro_lpm_clocks[i].offset);
		}
		rb = readl(va + zumapro_lpm_clocks[i].offset);
		dev_info(dev, "lpm_init %s(0x%08x+0x%04x) m=0x%x v=0x%x: readback=0x%x\n",
			 zumapro_lpm_clocks[i].name, zumapro_lpm_clocks[i].base,
			 zumapro_lpm_clocks[i].offset, zumapro_lpm_clocks[i].mask,
			 zumapro_lpm_clocks[i].val, rb);
		iounmap(va);
	}
}

/* tegu cluster membership: cluster0={0-3}, cluster1={4-6}, cluster2={7}. */
static const u8 zumapro_cpu_cluster[8] = { 0, 0, 0, 0, 1, 1, 1, 2 };

/*
 * Publish each secondary core's idle hint in CPU_INFORM on the dying CPU,
 * BEFORE its PSCI CPU_OFF, during deep suspend only.  zumapro has no pmu_cpuhp,
 * so nothing otherwise sets the hint and the cores power down as C1 - the ACPM
 * stamps "powered-down" (bit16) but records them as running (CPU_INFORM reads
 * 0x10000, vs downstream's 0x10001/0x10002 where pmucal_cpu_disable set C2/CPD
 * first).  Setting it here lets the ACPM consume the idle transition the same
 * way, which the SYS_SLEEP wake handshake needs.  CPD when this is the last
 * online core of its cluster (cluster power-down), C2 otherwise.  Gated on
 * sys_insuspend so runtime hotplug keeps the existing plain-PSCI behaviour;
 * the matching startup clears the hint as the cores come back on resume.
 */
static int zumapro_cpuhp_sleep_hint_set(unsigned int cpu)
{
	bool cluster_last = true;
	int c;

	if (!pmu_context->sys_insuspend || cpu >= ARRAY_SIZE(zumapro_cpu_cluster))
		return 0;

	for_each_online_cpu(c)
		if (c != cpu && c < ARRAY_SIZE(zumapro_cpu_cluster) &&
		    zumapro_cpu_cluster[c] == zumapro_cpu_cluster[cpu]) {
			cluster_last = false;
			break;
		}

	regmap_write(pmu_context->pmureg, GS101_CPU_INFORM(cpu),
		     cluster_last ? CPU_INFORM_CPD : CPU_INFORM_C2);
	return 0;
}

static int zumapro_cpuhp_sleep_hint_clear(unsigned int cpu)
{
	if (pmu_context->sys_insuspend && cpu < ARRAY_SIZE(zumapro_cpu_cluster))
		regmap_write(pmu_context->pmureg, GS101_CPU_INFORM(cpu),
			     CPU_INFORM_CLEAR);
	return 0;
}

static int exynos_pmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap_config pmu_regmcfg;
	struct regmap *regmap;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	/*
	 * The PMU register block is a syscon that can contain child devices
	 * with their own reg windows - e.g. the Tensor/Zumapro power domains,
	 * whose control registers sit at PMU offsets.  Map it without an
	 * exclusive request_mem_region(), like the generic syscon of_iomap()
	 * path, so those sub-devices can claim their windows instead of the
	 * PMU probe failing with -EBUSY.
	 */
	pmu_base_addr = devm_ioremap(dev, res->start, resource_size(res));
	if (!pmu_base_addr)
		return -ENOMEM;

	pmu_context = devm_kzalloc(&pdev->dev,
			sizeof(struct exynos_pmu_context),
			GFP_KERNEL);
	if (!pmu_context)
		return -ENOMEM;

	pmu_context->pmu_data = of_device_get_match_data(dev);

	/* For SoCs that secure PMU register writes use custom regmap */
	if (pmu_context->pmu_data && pmu_context->pmu_data->pmu_secure) {
		pmu_regmcfg = regmap_smccfg;
		pmu_regmcfg.max_register = resource_size(res) -
					   pmu_regmcfg.reg_stride;
		pmu_regmcfg.wr_table = pmu_context->pmu_data->wr_table;
		pmu_regmcfg.rd_table = pmu_context->pmu_data->rd_table;

		/* Need physical address for SMC call */
		regmap = devm_regmap_init(dev, NULL,
					  (void *)(uintptr_t)res->start,
					  &pmu_regmcfg);

		if (IS_ERR(regmap))
			return dev_err_probe(&pdev->dev, PTR_ERR(regmap),
					     "regmap init failed\n");

		ret = of_syscon_register_regmap(dev->of_node, regmap);
		if (ret)
			return ret;
	} else {
		/* let syscon create mmio regmap */
		regmap = syscon_node_to_regmap(dev->of_node);
		if (IS_ERR(regmap))
			return dev_err_probe(&pdev->dev, PTR_ERR(regmap),
					     "syscon_node_to_regmap failed\n");
	}

	pmu_context->pmureg = regmap;
	pmu_context->dev = dev;
	raw_spin_lock_init(&pmu_context->cpupm_lock);
	pmu_context->sys_inreboot = false;
	pmu_context->sys_insuspend = false;

	if (pmu_context->pmu_data && pmu_context->pmu_data->pmu_cpuhp) {
		ret = setup_cpuhp_and_cpuidle(dev);
		if (ret)
			return ret;
	}

	if (pmu_context->pmu_data && pmu_context->pmu_data->pmu_sicd_wakeup) {
		unsigned int inform0 = 0xdead;

		/*
		 * Downstream ZUMA cpupm tells TF-A that Linux is allowed to use
		 * C2 by writing PMU_INFORM0 at init.  This is separate from the
		 * per-core CPU_INFORM power-mode hints used around idle/suspend.
		 */
		ret = regmap_write(pmu_context->pmureg, GS101_INFORM0, PMU_ALLOWED_C2);
		regmap_read(pmu_context->pmureg, GS101_INFORM0, &inform0);
		dev_info(dev, "PMU_INFORM0 C2 allow: write ret=%d readback=0x%x\n",
			 ret, inform0);

		cpu_pm_register_notifier(&zumapro_cpu_pm_notifier);

		/*
		 * Deep sleep needs the PMU interrupt generator to route the
		 * boot core's wakeup; without the intr-gen regmap the SYS_SLEEP
		 * enter sequence can't run, so only offer it when present.
		 */
		ret = init_pmu_intr_gen_regmap(dev);
		if (ret)
			return ret;
		/*
		 * Persistent CMU_TOP mapping for the syscore fabric-feed gating
		 * (zumapro_sleep_gate_feeds_set); ioremap can't run in the atomic
		 * syscore path, so map it once here.
		 */
		zumapro_cmu_top_base = ioremap(0x26040000, 0x10000);
		if (!zumapro_cmu_top_base)
			dev_warn(dev, "CMU_TOP map failed; sleep fabric gating off\n");
		if (pmu_context->pmuintrgen)
			register_syscore(&zumapro_sys_sleep_syscore);
		else
			dev_warn(dev, "no pmu-intr-gen; deep sleep wake unavailable\n");

		/*
		 * Set the secondary cores' C2/CPD CPU_INFORM hint before their
		 * PSCI CPU_OFF during deep suspend (see zumapro_cpuhp_sleep_hint_set()).
		 * zumapro has no pmu_cpuhp, so this is the only hook that mirrors
		 * downstream's pre-powerdown hint; startup clears it on resume.
		 */
		ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN,
					"soc/zumapro-pmu:sleep-hint",
					zumapro_cpuhp_sleep_hint_clear,
					zumapro_cpuhp_sleep_hint_set);
		if (ret < 0)
			dev_warn(dev, "sleep-hint cpuhp register failed: %d\n", ret);

		/*
		 * Program the SYS_SLEEP wake power-sequencing the firmware needs
		 * to re-power the SoC; mainline otherwise never returns from
		 * PSCI SYSTEM_SUSPEND (see zumapro_pmu_lpm_init()).
		 */
		zumapro_pmu_lpm_init(dev);
	}

	if (pmu_context->pmu_data && pmu_context->pmu_data->pmu_init)
		pmu_context->pmu_data->pmu_init();

	platform_set_drvdata(pdev, pmu_context);

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE, exynos_pmu_devs,
				   ARRAY_SIZE(exynos_pmu_devs), NULL, 0, NULL);
	if (ret)
		return ret;

	if (devm_of_platform_populate(dev))
		dev_err(dev, "Error populating children, reboot and poweroff might not work properly\n");

	dev_dbg(dev, "Exynos PMU Driver probe done\n");
	return 0;
}

static int exynos_cpupm_suspend_noirq(struct device *dev)
{
	raw_spin_lock(&pmu_context->cpupm_lock);
	pmu_context->sys_insuspend = true;
	/* start the idle-hint tracking clean for this suspend */
	cpumask_clear(&zumapro_idle_cpus);
	zumapro_sicd_holder = -1;
	zumapro_dbg_c2 = zumapro_dbg_sicd = zumapro_dbg_fail = 0;
	raw_spin_unlock(&pmu_context->cpupm_lock);

	if (pmu_context->pmu_data && pmu_context->pmu_data->pmu_sicd_wakeup) {
		unsigned int en1 = 0xdead, en2 = 0xdead;

		zumapro_set_wakeup_mask(true);
		regmap_read(pmu_context->pmureg, GS101_TOP_INT_EN, &en1);
		regmap_read(pmu_context->pmureg, GS101_WAKEUP2_INT_EN, &en2);
		pr_info("zumapro: suspend(%s): wakeup mask armed, WAKEUP_INT_EN(0x3944)=0x%x WAKEUP2_INT_EN(0x3964)=0x%x\n",
			pm_suspend_target_state == PM_SUSPEND_MEM ? "deep" : "s2idle",
			en1, en2);
	}

	return 0;
}

static int exynos_cpupm_resume_noirq(struct device *dev)
{
	if (pmu_context->pmu_data && pmu_context->pmu_data->pmu_sicd_wakeup) {
		zumapro_set_wakeup_mask(false);
		pr_info("zumapro: resume: CPU_INFORM hints c2=%u sicd=%u fails=%u\n",
			zumapro_dbg_c2, zumapro_dbg_sicd, zumapro_dbg_fail);
	}

	raw_spin_lock(&pmu_context->cpupm_lock);
	pmu_context->sys_insuspend = false;
	raw_spin_unlock(&pmu_context->cpupm_lock);
	return 0;
}

static const struct dev_pm_ops cpupm_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(exynos_cpupm_suspend_noirq,
				  exynos_cpupm_resume_noirq)
};

static struct platform_driver exynos_pmu_driver = {
	.driver  = {
		.name   = "exynos-pmu",
		.of_match_table = exynos_pmu_of_device_ids,
		.pm = pm_sleep_ptr(&cpupm_pm_ops),
	},
	.probe = exynos_pmu_probe,
};

static int __init exynos_pmu_init(void)
{
	return platform_driver_register(&exynos_pmu_driver);

}
postcore_initcall(exynos_pmu_init);
