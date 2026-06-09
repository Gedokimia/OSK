// SPDX-License-Identifier: GPL-2.0
/*
 * OAKS - Optimized Adaptive Kernel Scheduler
 * OSK-16.2-ksun, Redmi 12C / Poco C55 (MediaTek G85)
 */

#include "sched.h"
#include "oaks.h"
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/mm.h>

DEFINE_STATIC_KEY_FALSE(oaks_active);
EXPORT_SYMBOL(oaks_active);

struct oaks_state oaks = {
	.ctx			= ATOMIC_INIT(OAKS_CTX_BALANCED),
	.responsive_timeout_j	= HZ / 2,
};
EXPORT_SYMBOL(oaks);

const struct oaks_params oaks_param_table[OAKS_CTX_MAX] = {
	[OAKS_CTX_BATTERY] = {
		.latency_ns	 = 6000000,
		.min_gran_ns	 = 750000,
		.wakeup_gran_ns	 = 1000000,
		.nr_latency	 = 8,
	},
	[OAKS_CTX_BALANCED] = {
		.latency_ns	 = 4000000,
		.min_gran_ns	 = 500000,
		.wakeup_gran_ns	 = 500000,
		.nr_latency	 = 10,
	},
	[OAKS_CTX_RESPONSIVE] = {
		.latency_ns	 = 3000000,
		.min_gran_ns	 = 300000,
		.wakeup_gran_ns	 = 300000,
		.nr_latency	 = 12,
	},
	[OAKS_CTX_PERF] = {
		.latency_ns	 = 2000000,
		.min_gran_ns	 = 250000,
		.wakeup_gran_ns	 = 200000,
		.nr_latency	 = 14,
	},
};
EXPORT_SYMBOL(oaks_param_table);

/* Forward declarations for scheduler globals modified by OAKS */
extern unsigned int sysctl_sched_latency;
extern unsigned int sysctl_sched_min_granularity;
extern unsigned int sysctl_sched_wakeup_granularity;
extern unsigned int sched_nr_latency;

static void oaks_apply_params(enum oaks_ctx ctx)
{
	const struct oaks_params *p = &oaks_param_table[ctx];
	WRITE_ONCE(sysctl_sched_latency,	    p->latency_ns);
	WRITE_ONCE(sysctl_sched_min_granularity,    p->min_gran_ns);
	WRITE_ONCE(sysctl_sched_wakeup_granularity, p->wakeup_gran_ns);
}

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
	if (ctx > OAKS_CTX_BALANCED)
		static_branch_enable(&oaks_active);
	else
		static_branch_disable(&oaks_active);
	spin_unlock_irqrestore(&oaks.lock, flags);

	pr_debug("OAKS: ctx %d -> %d\n", old, ctx);
}
EXPORT_SYMBOL(oaks_set_ctx);

enum oaks_ctx oaks_get_ctx(void)
{
	return atomic_read(&oaks.ctx);
}
EXPORT_SYMBOL(oaks_get_ctx);

void oaks_notify_touch(void)
{
	WRITE_ONCE(oaks.last_touch_j, jiffies);
	if (oaks_get_ctx() < OAKS_CTX_RESPONSIVE)
		oaks_set_ctx(OAKS_CTX_RESPONSIVE);
}
EXPORT_SYMBOL(oaks_notify_touch);

void oaks_notify_perf_scene(pid_t main_pid, pid_t render_pid, bool enter)
{
	unsigned long flags;

	spin_lock_irqsave(&oaks.lock, flags);
	if (enter) {
		oaks.perf.main_pid   = main_pid;
		oaks.perf.render_pid = render_pid;
		spin_unlock_irqrestore(&oaks.lock, flags);
		oaks_set_ctx(OAKS_CTX_PERF);
	} else {
		oaks.perf.main_pid   = 0;
		oaks.perf.render_pid = 0;
		spin_unlock_irqrestore(&oaks.lock, flags);
		oaks_set_ctx(OAKS_CTX_BALANCED);
	}
}
EXPORT_SYMBOL(oaks_notify_perf_scene);

void oaks_tick(void)
{
	if (!static_branch_unlikely(&oaks_active))
		return;
	if (oaks_get_ctx() != OAKS_CTX_RESPONSIVE)
		return;
	if (time_after(jiffies,
		       READ_ONCE(oaks.last_touch_j) +
		       READ_ONCE(oaks.responsive_timeout_j)))
		oaks_set_ctx(OAKS_CTX_BALANCED);
}
EXPORT_SYMBOL(oaks_tick);

void oaks_init(void)
{
	spin_lock_init(&oaks.lock);
	atomic_set(&oaks.ctx, OAKS_CTX_BALANCED);
	pr_info("OAKS: initialized (G85: cpu0-5=A55 E-cores, cpu6-7=A75 P-cores)\n");
}

/* -----------------------------------------------------------------------
 * Sysfs: /sys/kernel/oaks/{context, responsive_timeout_ms, stats}
 * ----------------------------------------------------------------------- */
static struct kobject *oaks_kobj;

static ssize_t context_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	static const char * const names[] = {
		"battery", "balanced", "responsive", "perf"
	};
	int ctx = atomic_read(&oaks.ctx);
	return sprintf(buf, "%d (%s)\n", ctx, names[ctx]);
}

static ssize_t context_store(struct kobject *kobj,
			     struct kobj_attribute *attr,
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

static ssize_t timeout_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n",
		jiffies_to_msecs(READ_ONCE(oaks.responsive_timeout_j)));
}

static ssize_t timeout_store(struct kobject *kobj,
			     struct kobj_attribute *attr,
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

static ssize_t stats_show(struct kobject *kobj,
			  struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		"battery=%lu balanced=%lu responsive=%lu perf=%lu\n"
		"perf_main_pid=%d perf_render_pid=%d\n",
		oaks.ctx_switches[OAKS_CTX_BATTERY],
		oaks.ctx_switches[OAKS_CTX_BALANCED],
		oaks.ctx_switches[OAKS_CTX_RESPONSIVE],
		oaks.ctx_switches[OAKS_CTX_PERF],
		oaks.perf.main_pid,
		oaks.perf.render_pid);
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
	oaks_init();
	return ret;
}
late_initcall(oaks_sysfs_init);
