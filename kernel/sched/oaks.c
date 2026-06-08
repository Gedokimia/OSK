// SPDX-License-Identifier: GPL-2.0
/*
 * OAKS - Optimized Adaptive Kernel Scheduler
 *
 * Context-driven scheduler extension for MediaTek G85 (Cortex-A75/A55).
 * Targets OSK-16.2-ksun on Redmi 12C / Poco C55.
 *
 * Sits on top of: CFS + BORE (already in tree), EAS, MTK task_turbo,
 *                 MTK perf_ioctl
 */

#include "sched.h"
#include "oaks.h"
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>

/* -----------------------------------------------------------------------
 * Static key — nop when OAKS is inactive (BALANCED, no game running)
 * ----------------------------------------------------------------------- */
DEFINE_STATIC_KEY_FALSE(oaks_active);
EXPORT_SYMBOL(oaks_active);

/* -----------------------------------------------------------------------
 * Global state
 * ----------------------------------------------------------------------- */
struct oaks_state oaks = {
	.ctx			= ATOMIC_INIT(OAKS_CTX_BALANCED),
	.responsive_timeout_j	= HZ / 2,   /* 500ms touch window */
};
EXPORT_SYMBOL(oaks);

/* -----------------------------------------------------------------------
 * Per-context tunable table
 *
 * BATTERY:
 *   Relax preemption pressure. BORE runs at full penalty to punish
 *   background CPU hogs hard. Avoid A75 entirely — let EAS idle it.
 *
 * BALANCED:
 *   Exact KernelBumi defaults. OAKS does not touch anything.
 *
 * RESPONSIVE:
 *   Tighten latency targets for snappy touch response. BORE cap
 *   lowered to 2 so UI threads that burst briefly don't get throttled.
 *   big_util_floor = 12% — tasks above this threshold may go to A75.
 *
 * PERF:
 *   Minimum latency, maximum throughput. BORE suppressed entirely for
 *   tagged render/main threads (see oaks_bore_suppress()). All tasks
 *   in the game tgid are nudged toward A75 if spare cap allows.
 * ----------------------------------------------------------------------- */
const struct oaks_params oaks_param_table[OAKS_CTX_MAX] = {
	[OAKS_CTX_BATTERY] = {
		.latency_ns	 = 6000000,   /* 6ms */
		.min_gran_ns	 = 750000,    /* 0.75ms */
		.wakeup_gran_ns	 = 1000000,   /* 1ms */
		.nr_latency	 = 8,
		.bore_cap	 = 3,
		.big_util_floor	 = 0,
	},
	[OAKS_CTX_BALANCED] = {
		.latency_ns	 = 4000000,   /* 4ms — KernelBumi default */
		.min_gran_ns	 = 500000,    /* 0.5ms */
		.wakeup_gran_ns	 = 500000,    /* 0.5ms */
		.nr_latency	 = 10,
		.bore_cap	 = 3,
		.big_util_floor	 = 0,
	},
	[OAKS_CTX_RESPONSIVE] = {
		.latency_ns	 = 3000000,   /* 3ms */
		.min_gran_ns	 = 300000,    /* 0.3ms */
		.wakeup_gran_ns	 = 300000,    /* 0.3ms */
		.nr_latency	 = 12,
		.bore_cap	 = 2,         /* softer BORE — UI bursts OK */
		.big_util_floor	 = 128,       /* 12.5% util → try A75 */
	},
	[OAKS_CTX_PERF] = {
		.latency_ns	 = 2000000,   /* 2ms */
		.min_gran_ns	 = 250000,    /* 0.25ms */
		.wakeup_gran_ns	 = 200000,    /* 0.2ms */
		.nr_latency	 = 14,
		.bore_cap	 = 0,         /* BORE suppressed for tagged PIDs */
		.big_util_floor	 = 256,       /* 25% util → push to A75 */
	},
};
EXPORT_SYMBOL(oaks_param_table);

/* -----------------------------------------------------------------------
 * oaks_apply_params() — push context tunables to scheduler globals.
 * Caller must hold oaks.lock or be in single-CPU init context.
 * ----------------------------------------------------------------------- */
static void oaks_apply_params(enum oaks_ctx ctx)
{
	const struct oaks_params *p = &oaks_param_table[ctx];

	WRITE_ONCE(sysctl_sched_latency,	    p->latency_ns);
	WRITE_ONCE(sysctl_sched_min_granularity,    p->min_gran_ns);
	WRITE_ONCE(sysctl_sched_wakeup_granularity, p->wakeup_gran_ns);
	/* sched_nr_latency is a file-scope static in fair.c;
	 * expose it via a wrapper if needed or use the sysctl path */
}

/* -----------------------------------------------------------------------
 * oaks_set_ctx() — transition to a new context.
 * Safe to call from any context (IRQ-disabled internally).
 * ----------------------------------------------------------------------- */
void oaks_set_ctx(enum oaks_ctx ctx)
{
	enum oaks_ctx old;
	unsigned long flags;

	if (WARN_ON_ONCE(ctx >= OAKS_CTX_MAX))
		return;

	spin_lock_irqsave(&oaks.lock, flags);

	old = atomic_read(&oaks.ctx);
	if (old == ctx) {
		spin_unlock_irqrestore(&oaks.lock, flags);
		return;
	}

	atomic_set(&oaks.ctx, ctx);
	oaks.ctx_switches[ctx]++;
	oaks_apply_params(ctx);

	/* Enable/disable the hot-path static key */
	if (ctx > OAKS_CTX_BALANCED)
		static_branch_enable(&oaks_active);
	else
		static_branch_disable(&oaks_active);

	spin_unlock_irqrestore(&oaks.lock, flags);

	pr_debug("OAKS: ctx %d → %d\n", old, ctx);
}
EXPORT_SYMBOL(oaks_set_ctx);

enum oaks_ctx oaks_get_ctx(void)
{
	return atomic_read(&oaks.ctx);
}
EXPORT_SYMBOL(oaks_get_ctx);

/* -----------------------------------------------------------------------
 * oaks_notify_touch() — called by input subsystem on touch/key events.
 * Transitions to RESPONSIVE and resets the timeout window.
 * ----------------------------------------------------------------------- */
void oaks_notify_touch(void)
{
	WRITE_ONCE(oaks.last_touch_j, jiffies);
	/* Only upgrade, never downgrade from PERF via touch */
	if (oaks_get_ctx() == OAKS_CTX_BATTERY ||
	    oaks_get_ctx() == OAKS_CTX_BALANCED)
		oaks_set_ctx(OAKS_CTX_RESPONSIVE);
}
EXPORT_SYMBOL(oaks_notify_touch);

/* -----------------------------------------------------------------------
 * oaks_notify_perf_scene() — called by perf_ioctl on game scene enter/exit.
 * Records main and render PIDs for hot-path identification.
 * ----------------------------------------------------------------------- */
void oaks_notify_perf_scene(pid_t main_pid, pid_t render_pid, bool enter)
{
	unsigned long flags;

	spin_lock_irqsave(&oaks.lock, flags);
	if (enter) {
		oaks.perf_task.main_pid   = main_pid;
		oaks.perf_task.render_pid = render_pid;
		spin_unlock_irqrestore(&oaks.lock, flags);
		oaks_set_ctx(OAKS_CTX_PERF);
		pr_info("OAKS: PERF scene entered main=%d render=%d\n",
			main_pid, render_pid);
	} else {
		oaks.perf_task.main_pid   = 0;
		oaks.perf_task.render_pid = 0;
		spin_unlock_irqrestore(&oaks.lock, flags);
		oaks_set_ctx(OAKS_CTX_BALANCED);
		pr_info("OAKS: PERF scene exited\n");
	}
}
EXPORT_SYMBOL(oaks_notify_perf_scene);

/* -----------------------------------------------------------------------
 * oaks_tick() — called from scheduler_tick() every HZ tick.
 * Handles RESPONSIVE → BALANCED timeout.
 * ----------------------------------------------------------------------- */
void oaks_tick(void)
{
	unsigned long last, timeout;

	/* Fast-path: only act in RESPONSIVE */
	if (!static_branch_unlikely(&oaks_active))
		return;
	if (oaks_get_ctx() != OAKS_CTX_RESPONSIVE)
		return;

	last    = READ_ONCE(oaks.last_touch_j);
	timeout = READ_ONCE(oaks.responsive_timeout_j);

	if (time_after(jiffies, last + timeout))
		oaks_set_ctx(OAKS_CTX_BALANCED);
}
EXPORT_SYMBOL(oaks_tick);

/* -----------------------------------------------------------------------
 * oaks_init() — called from sched_init() during boot
 * ----------------------------------------------------------------------- */
void oaks_init(void)
{
	spin_lock_init(&oaks.lock);
	atomic_set(&oaks.ctx, OAKS_CTX_BALANCED);
	pr_info("OAKS: initialized (G85 A75/A55 topology)\n");
}

/* -----------------------------------------------------------------------
 * Sysfs interface — /sys/kernel/oaks/
 *
 *   context                r/w  0=battery 1=balanced 2=responsive 3=perf
 *   responsive_timeout_ms  r/w  100–5000ms (default 500)
 *   stats                  r    transition counters per context
 * ----------------------------------------------------------------------- */
static struct kobject *oaks_kobj;

static ssize_t context_show(struct kobject *kobj, struct kobj_attribute *a,
			    char *buf)
{
	static const char * const names[] = {
		"battery", "balanced", "responsive", "perf"
	};
	int ctx = atomic_read(&oaks.ctx);
	return sprintf(buf, "%d (%s)\n", ctx, names[ctx]);
}

static ssize_t context_store(struct kobject *kobj, struct kobj_attribute *a,
			     const char *buf, size_t count)
{
	unsigned int val;
	if (kstrtouint(buf, 10, &val) || val >= OAKS_CTX_MAX)
		return -EINVAL;
	oaks_set_ctx(val);
	return count;
}
static struct kobj_attribute oaks_ctx_attr =
	__ATTR(context, 0644, context_show, context_store);

static ssize_t timeout_show(struct kobject *kobj, struct kobj_attribute *a,
			    char *buf)
{
	return sprintf(buf, "%u\n",
		jiffies_to_msecs(READ_ONCE(oaks.responsive_timeout_j)));
}

static ssize_t timeout_store(struct kobject *kobj, struct kobj_attribute *a,
			     const char *buf, size_t count)
{
	unsigned int ms;
	if (kstrtouint(buf, 10, &ms) || ms < 100 || ms > 5000)
		return -EINVAL;
	WRITE_ONCE(oaks.responsive_timeout_j, msecs_to_jiffies(ms));
	return count;
}
static struct kobj_attribute oaks_timeout_attr =
	__ATTR(responsive_timeout_ms, 0644, timeout_show, timeout_store);

static ssize_t stats_show(struct kobject *kobj, struct kobj_attribute *a,
			  char *buf)
{
	return sprintf(buf,
		"battery=%lu balanced=%lu responsive=%lu perf=%lu\n"
		"perf_main_pid=%d perf_render_pid=%d\n",
		oaks.ctx_switches[OAKS_CTX_BATTERY],
		oaks.ctx_switches[OAKS_CTX_BALANCED],
		oaks.ctx_switches[OAKS_CTX_RESPONSIVE],
		oaks.ctx_switches[OAKS_CTX_PERF],
		oaks.perf_task.main_pid,
		oaks.perf_task.render_pid);
}
static struct kobj_attribute oaks_stats_attr =
	__ATTR(stats, 0444, stats_show, NULL);

static struct attribute *oaks_attrs[] = {
	&oaks_ctx_attr.attr,
	&oaks_timeout_attr.attr,
	&oaks_stats_attr.attr,
	NULL,
};
static const struct attribute_group oaks_attr_group = {
	.attrs = oaks_attrs,
};

static int __init oaks_sysfs_init(void)
{
	int ret;

	oaks_kobj = kobject_create_and_add("oaks", kernel_kobj);
	if (!oaks_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(oaks_kobj, &oaks_attr_group);
	if (ret)
		kobject_put(oaks_kobj);

	return ret;
}
late_initcall(oaks_sysfs_init);
