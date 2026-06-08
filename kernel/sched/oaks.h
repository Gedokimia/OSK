/* SPDX-License-Identifier: GPL-2.0 */
/*
 * OAKS - Optimized Adaptive Kernel Scheduler
 *
 * A context-driven scheduler extension for asymmetric CPU topologies.
 * Designed for MediaTek G85 (Cortex-A75 + Cortex-A55) on 4.19 kernels.
 * Inspired by the Capacity Aware Superset Scheduler concept.
 *
 * OAKS sits on top of the existing CFS + BORE infrastructure and adds
 * four execution contexts that tune the scheduler dynamically:
 *
 *   OAKS_CTX_BATTERY    — screen-off / charger disconnected idle
 *   OAKS_CTX_BALANCED   — default; stock KernelBumi tunables unchanged
 *   OAKS_CTX_RESPONSIVE — touch/input active; lower latency targets
 *   OAKS_CTX_PERF       — game scene or sustained load; max throughput
 *
 * Unlike CASS (Sultan's Pixel implementation), OAKS does NOT require a
 * DynamIQ topology or 5.10+ scheduler. It adapts to the G85's simpler
 * two-cluster layout and integrates with MTK's existing task_turbo and
 * perf_ioctl hooks already present in this tree.
 *
 * Exposed via:  /sys/kernel/oaks/{context, responsive_timeout_ms, stats}
 * Sysctl:       /proc/sys/kernel/oaks_bore_cap
 */

#ifndef _KERNEL_SCHED_OAKS_H
#define _KERNEL_SCHED_OAKS_H

#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/spinlock.h>
#include <linux/jump_label.h>

/* -----------------------------------------------------------------------
 * G85 CPU topology constants
 * cpu0-3: Cortex-A55 (efficiency cluster, max_cap ~724)
 * cpu4-5: Cortex-A75 (performance cluster, max_cap 1024)
 * ----------------------------------------------------------------------- */
#define OAKS_LITTLE_FIRST	0
#define OAKS_LITTLE_LAST	3
#define OAKS_BIG_FIRST		4
#define OAKS_BIG_LAST		5
#define OAKS_BIG_CAPACITY	1024UL
#define OAKS_LITTLE_CAPACITY	724UL  /* ~71% of big — G85 ratio */

/* -----------------------------------------------------------------------
 * Execution contexts — ordered by CPU/power aggressiveness
 * ----------------------------------------------------------------------- */
enum oaks_ctx {
	OAKS_CTX_BATTERY	= 0,
	OAKS_CTX_BALANCED	= 1,
	OAKS_CTX_RESPONSIVE	= 2,
	OAKS_CTX_PERF		= 3,
	OAKS_CTX_MAX,
};

/* -----------------------------------------------------------------------
 * Per-context scheduler tunable snapshot.
 * Applied atomically on context transition.
 * ----------------------------------------------------------------------- */
struct oaks_params {
	unsigned int	latency_ns;	  /* sysctl_sched_latency */
	unsigned int	min_gran_ns;	  /* sysctl_sched_min_granularity */
	unsigned int	wakeup_gran_ns;   /* sysctl_sched_wakeup_granularity */
	unsigned int	nr_latency;	  /* sched_nr_latency */
	u8		bore_cap;	  /* max BORE shift (0 = suppress) */
	u8		big_util_floor;   /* util% to prefer A75 (0 = EAS) */
};

/* -----------------------------------------------------------------------
 * Tagged task state — game render/main thread identification
 * ----------------------------------------------------------------------- */
struct oaks_tagged_task {
	pid_t	main_pid;
	pid_t	render_pid;
};

/* -----------------------------------------------------------------------
 * OAKS global state
 * ----------------------------------------------------------------------- */
struct oaks_state {
	atomic_t		ctx;
	spinlock_t		lock;

	struct oaks_tagged_task	perf_task;

	/* RESPONSIVE context timeout */
	unsigned long		last_touch_j;
	unsigned long		responsive_timeout_j;

	/* stats */
	unsigned long		ctx_switches[OAKS_CTX_MAX];
};

/* Static key: disabled when no PERF/RESPONSIVE hooks are active */
DECLARE_STATIC_KEY_FALSE(oaks_active);

extern struct oaks_state oaks;
extern const struct oaks_params oaks_param_table[OAKS_CTX_MAX];

/* -----------------------------------------------------------------------
 * Core API
 * ----------------------------------------------------------------------- */
void oaks_init(void);
void oaks_set_ctx(enum oaks_ctx ctx);
enum oaks_ctx oaks_get_ctx(void);
void oaks_notify_touch(void);
void oaks_notify_perf_scene(pid_t main_pid, pid_t render_pid, bool enter);
void oaks_tick(void);

/* -----------------------------------------------------------------------
 * Hot-path inlines — used directly in fair.c
 * ----------------------------------------------------------------------- */

static inline bool oaks_in_perf(void)
{
	if (!static_branch_unlikely(&oaks_active))
		return false;
	return atomic_read(&oaks.ctx) == OAKS_CTX_PERF;
}

static inline bool oaks_is_responsive(void)
{
	if (!static_branch_unlikely(&oaks_active))
		return false;
	return atomic_read(&oaks.ctx) >= OAKS_CTX_RESPONSIVE;
}

/*
 * oaks_bore_suppress - returns true if BORE penalty should be skipped.
 * In PERF context, tagged game/render threads run at their natural vruntime
 * so the render deadline is never missed due to accumulated burst penalty.
 */
static inline bool oaks_bore_suppress(struct task_struct *p)
{
	if (!oaks_in_perf())
		return false;
	return (p->pid == oaks.perf_task.render_pid ||
		p->pid == oaks.perf_task.main_pid);
}

/*
 * oaks_bore_cap - per-context BORE penalty cap.
 * Falls back to stock cap (3) when OAKS is idle.
 */
static inline u64 oaks_bore_cap(void)
{
	if (!static_branch_unlikely(&oaks_active))
		return 3ULL;
	return oaks_param_table[atomic_read(&oaks.ctx)].bore_cap;
}

/*
 * oaks_want_big_cluster - returns true if task should be nudged to A75.
 * Only acts in PERF context for tagged tasks, and only if a big CPU has
 * spare capacity — never forces placement against EAS.
 */
static inline bool oaks_want_big_cluster(struct task_struct *p)
{
	if (!oaks_in_perf())
		return false;
	return (p->pid == oaks.perf_task.render_pid ||
		p->pid == oaks.perf_task.main_pid ||
		p->tgid == oaks.perf_task.main_pid);
}

/*
 * oaks_oom_guard - returns true if task should be skipped by OOM killer.
 * Protects game tgid during PERF context to prevent mid-game OOM kills.
 */
static inline bool oaks_oom_guard(struct task_struct *p)
{
	if (!oaks_in_perf())
		return false;
	return (p->tgid == oaks.perf_task.main_pid ||
		p->pid  == oaks.perf_task.render_pid);
}

#endif /* _KERNEL_SCHED_OAKS_H */
