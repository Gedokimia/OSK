/* SPDX-License-Identifier: GPL-2.0 */
/*
 * OAKS - Optimized Adaptive Kernel Scheduler
 * Context-driven CFS/BORE tuning for MediaTek G85:
 *   cpu0-5: Cortex-A55 (6x E-cores)
 *   cpu6-7: Cortex-A75 (2x P-cores)
 *
 * OSK-16.2-ksun, Redmi 12C / Poco C55
 */
#ifndef _KERNEL_SCHED_OAKS_H
#define _KERNEL_SCHED_OAKS_H

#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/spinlock.h>
#include <linux/jump_label.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/oom.h>

/* G85 topology — 6E + 2P */
#define OAKS_BIG_FIRST		6
#define OAKS_BIG_LAST		7
#define OAKS_NR_CPUS		8

/* cpu_util_cfs() range; used to normalise util to [0,100]% */
#define OAKS_UTIL_SCALE		1024  /* == SCHED_CAPACITY_SCALE */

/*
 * PSI mem-some 10s average threshold (for context scoring only).
 * PSI avg[] uses FIXED_1=2048 as 100% (FSHIFT=11).
 *   10% stall = 205
 */
#define OAKS_PSI_MEM_SOME_THRESH	205 /* 10% in FIXED_1=2048 units */

enum oaks_ctx {
	OAKS_CTX_BATTERY	= 0,
	OAKS_CTX_BALANCED	= 1,
	OAKS_CTX_RESPONSIVE	= 2,
	OAKS_CTX_PERF		= 3,
	OAKS_CTX_MAX,
};

/*
 * Context detection signal weights.
 * Evaluated every oaks_tick(); highest score wins.
 */
struct oaks_params {
	unsigned int	latency_ns;
	unsigned int	min_gran_ns;
	unsigned int	wakeup_gran_ns;
	unsigned int	nr_latency;
	/* BORE tunables (0 = leave kernel default) */
	unsigned int	bore_burst_penalty_scale; /* sched_burst_penalty_scale */
	unsigned int	bore_initial_score;       /* sched_initial_score */
};

struct oaks_perf_task {
	pid_t	main_pid;
	pid_t	render_pid;
};

/*
 * Context detection state — updated atomically by oaks_tick().
 * Packed to fit in one or two cache lines.
 */
struct oaks_ctx_signals {
	/* Touch: jiffies of last EV_KEY/EV_ABS event */
	unsigned long	last_touch_j;
	/* Render thread last wakeup jiffies (set by oaks_notify_perf_scene) */
	unsigned long	last_render_wake_j;
	/* PSI mem-some 10s avg snapshot, updated each tick (PSI fixed-point) */
	unsigned long	psi_mem_some_avg;
	/* CPU util snapshot of P-cluster (cpu6+7), updated each tick */
	unsigned int	big_cluster_util;
	/* nr_running on big cluster at last tick */
	unsigned int	big_nr_running;
};



struct oaks_state {
	atomic_t			ctx;
	spinlock_t			lock;
	struct oaks_perf_task		perf;
	unsigned long			responsive_timeout_j;
	unsigned long			ctx_switches[OAKS_CTX_MAX];
	struct oaks_ctx_signals		sig;
	/*
	 * ctx_work: deferred workqueue item for static_key transitions.
	 * static_branch_enable/disable require a mutex (jump_label_mutex)
	 * and cpus_read_lock — both sleep. They CANNOT be called from
	 * hardirq context (scheduler_tick, input_handle_event).
	 * All static_key operations are deferred here.
	 *
	 * pending_ctx: the ctx value to apply when ctx_work runs.
	 * Written atomically; ctx_work reads it and calls oaks_set_ctx_work().
	 */
	struct work_struct		ctx_work;
	atomic_t			pending_ctx;
};

DECLARE_STATIC_KEY_FALSE(oaks_active);

extern struct oaks_state oaks;
extern const struct oaks_params oaks_param_table[OAKS_CTX_MAX];

/* Core */
void oaks_init(void);
void oaks_set_ctx(enum oaks_ctx ctx);
enum oaks_ctx oaks_get_ctx(void);

/* Event hooks */
void oaks_notify_touch(void);
void oaks_notify_perf_scene(pid_t main_pid, pid_t render_pid, bool enter);
void oaks_tick(void);

/* ------------------------------------------------------------------ */

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
 * oaks_want_big_cluster - hint to place @p on A75 cluster.
 */
static inline bool oaks_want_big_cluster(struct task_struct *p)
{
	if (!oaks_in_perf())
		return false;
	return (p->pid  == oaks.perf.render_pid ||
		p->tgid == oaks.perf.main_pid);
}

#endif /* _KERNEL_SCHED_OAKS_H */
