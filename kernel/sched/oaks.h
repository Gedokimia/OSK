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

/* G85 topology — 6E + 2P */
#define OAKS_BIG_FIRST		6
#define OAKS_BIG_LAST		7

enum oaks_ctx {
	OAKS_CTX_BATTERY	= 0,
	OAKS_CTX_BALANCED	= 1,
	OAKS_CTX_RESPONSIVE	= 2,
	OAKS_CTX_PERF		= 3,
	OAKS_CTX_MAX,
};

struct oaks_params {
	unsigned int	latency_ns;
	unsigned int	min_gran_ns;
	unsigned int	wakeup_gran_ns;
	unsigned int	nr_latency;
};

struct oaks_perf_task {
	pid_t	main_pid;
	pid_t	render_pid;
};

struct oaks_state {
	atomic_t		ctx;
	spinlock_t		lock;
	struct oaks_perf_task	perf;
	unsigned long		last_touch_j;
	unsigned long		responsive_timeout_j;
	unsigned long		ctx_switches[OAKS_CTX_MAX];
};

DECLARE_STATIC_KEY_FALSE(oaks_active);

extern struct oaks_state oaks;
extern const struct oaks_params oaks_param_table[OAKS_CTX_MAX];

void oaks_init(void);
void oaks_set_ctx(enum oaks_ctx ctx);
enum oaks_ctx oaks_get_ctx(void);
void oaks_notify_touch(void);
void oaks_notify_perf_scene(pid_t main_pid, pid_t render_pid, bool enter);
void oaks_tick(void);

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

static inline bool oaks_oom_guard(struct task_struct *p)
{
	if (!oaks_in_perf())
		return false;
	return (p->tgid == oaks.perf.main_pid ||
		p->pid  == oaks.perf.render_pid);
}

static inline bool oaks_want_big_cluster(struct task_struct *p)
{
	if (!oaks_in_perf())
		return false;
	return (p->pid == oaks.perf.render_pid ||
		p->pid == oaks.perf.main_pid ||
		p->tgid == oaks.perf.main_pid);
}

#endif /* _KERNEL_SCHED_OAKS_H */
