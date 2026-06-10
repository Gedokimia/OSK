// SPDX-License-Identifier: GPL-2.0
/*
 * OAKS - Optimized Adaptive Kernel Scheduler
 * OSK-16.2-ksun, Redmi 12C / Poco C55 (MediaTek G85)
 *
 * Improvements over v1:
 *  - Multi-signal context detection (touch, PSI, big-cluster util, render wakes)
 *  - Hysteresis to prevent rapid ctx oscillation
 *  - Smart in-kernel LMK: shrinker-triggered, adj-tiered, deferred kill
 *  - BORE parameter push per context
 *  - Sysfs: added lmk_stats, lmk_enable, mem_pressure nodes
 */

#include "sched.h"
#include "oaks.h"
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/oom.h>
#include <linux/sched/signal.h>
#include <linux/rcupdate.h>
#include <linux/psi.h>
#include <linux/workqueue.h>

/* -----------------------------------------------------------------------
 * Globals
 * ----------------------------------------------------------------------- */

DEFINE_STATIC_KEY_FALSE(oaks_active);
EXPORT_SYMBOL(oaks_active);

struct oaks_state oaks = {
	.ctx			= ATOMIC_INIT(OAKS_CTX_BALANCED),
	.responsive_timeout_j	= HZ / 2,
};
EXPORT_SYMBOL(oaks);

/*
 * Parameter table — indexed by oaks_ctx.
 *
 * bore_burst_penalty_scale: lower = less burst penalty = more interactive.
 *   Kernel default for 4.19+BORE is typically 1280.
 * bore_initial_score: higher = more aggressive burst dampening on new tasks.
 *   Kernel default 0.
 *
 * Latency/gran values are in nanoseconds and map directly to
 * sysctl_sched_latency / sysctl_sched_min_granularity / sysctl_sched_wakeup_granularity.
 */
const struct oaks_params oaks_param_table[OAKS_CTX_MAX] = {
	[OAKS_CTX_BATTERY] = {
		.latency_ns		= 6000000,
		.min_gran_ns		= 750000,
		.wakeup_gran_ns		= 1000000,
		.nr_latency		= 8,
		.bore_burst_penalty_scale = 1536,
		.bore_initial_score	= 0,
	},
	[OAKS_CTX_BALANCED] = {
		.latency_ns		= 4000000,
		.min_gran_ns		= 500000,
		.wakeup_gran_ns		= 500000,
		.nr_latency		= 10,
		.bore_burst_penalty_scale = 1280,
		.bore_initial_score	= 0,
	},
	[OAKS_CTX_RESPONSIVE] = {
		.latency_ns		= 3000000,
		.min_gran_ns		= 300000,
		.wakeup_gran_ns		= 300000,
		.nr_latency		= 12,
		.bore_burst_penalty_scale = 1024,
		.bore_initial_score	= 0,
	},
	[OAKS_CTX_PERF] = {
		.latency_ns		= 2000000,
		.min_gran_ns		= 250000,
		.wakeup_gran_ns		= 200000,
		.nr_latency		= 14,
		.bore_burst_penalty_scale = 768,
		.bore_initial_score	= 0,
	},
};
EXPORT_SYMBOL(oaks_param_table);

/* -----------------------------------------------------------------------
 * Scheduler globals written by OAKS
 * ----------------------------------------------------------------------- */

/* CFS tunables — non-static globals in kernel/sched/fair.c */
extern unsigned int sysctl_sched_latency;
extern unsigned int sysctl_sched_min_granularity;
extern unsigned int sysctl_sched_wakeup_granularity;
/*
 * sched_nr_latency: static in fair.c — cannot extern. fair.c
 * recomputes it whenever sysctl_sched_latency is written.
 * sysctl_sched_pelt_halflife: does not exist in this 4.19 tree.
 */

#ifdef CONFIG_SCHED_BORE
extern unsigned int sched_burst_penalty_scale; /* uint __read_mostly */
#endif

/* LMK enable flag — writable from sysfs */
static int oaks_lmk_enabled __read_mostly = 1;

/* -----------------------------------------------------------------------
 * Context application
 * ----------------------------------------------------------------------- */

static void oaks_apply_params(enum oaks_ctx ctx)
{
	const struct oaks_params *p = &oaks_param_table[ctx];

	WRITE_ONCE(sysctl_sched_latency,	    p->latency_ns);
	WRITE_ONCE(sysctl_sched_min_granularity,    p->min_gran_ns);
	WRITE_ONCE(sysctl_sched_wakeup_granularity, p->wakeup_gran_ns);

#ifdef CONFIG_SCHED_BORE
	if (p->bore_burst_penalty_scale)
		WRITE_ONCE(sched_burst_penalty_scale, p->bore_burst_penalty_scale);
#endif
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

	/*
	 * static_key: active when ctx > BALANCED so the fast-paths in
	 * oaks_in_perf() / oaks_is_responsive() are near-zero cost in
	 * BATTERY and BALANCED.
	 */
	if (ctx > OAKS_CTX_BALANCED)
		static_branch_enable(&oaks_active);
	else
		static_branch_disable(&oaks_active);

	spin_unlock_irqrestore(&oaks.lock, flags);

	pr_debug("OAKS: ctx %d -> %d (touch=%lu psi=%lu big_util=%u)\n",
		 old, ctx,
		 READ_ONCE(oaks.sig.last_touch_j),
		 READ_ONCE(oaks.sig.psi_mem_some_avg),
		 READ_ONCE(oaks.sig.big_cluster_util));
}
EXPORT_SYMBOL(oaks_set_ctx);

enum oaks_ctx oaks_get_ctx(void)
{
	return atomic_read(&oaks.ctx);
}
EXPORT_SYMBOL(oaks_get_ctx);

/* -----------------------------------------------------------------------
 * Event hooks
 * ----------------------------------------------------------------------- */

void oaks_notify_touch(void)
{
	WRITE_ONCE(oaks.sig.last_touch_j, jiffies);
	/*
	 * Touch always bumps to at least RESPONSIVE. Never downgrade
	 * an active PERF scene — oaks_tick() handles that separately.
	 */
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
		WRITE_ONCE(oaks.sig.last_render_wake_j, jiffies);
		spin_unlock_irqrestore(&oaks.lock, flags);
		oaks_set_ctx(OAKS_CTX_PERF);
	} else {
		oaks.perf.main_pid   = 0;
		oaks.perf.render_pid = 0;
		spin_unlock_irqrestore(&oaks.lock, flags);
		/* Transition to RESPONSIVE briefly, tick will decay to BALANCED */
		oaks_set_ctx(OAKS_CTX_RESPONSIVE);
	}
}
EXPORT_SYMBOL(oaks_notify_perf_scene);

/* -----------------------------------------------------------------------
 * Context detection engine
 *
 * Called from scheduler_tick() via oaks_tick() on every CPU tick.
 * Fast-path: only CPU 0 does full re-evaluation (once per HZ/10).
 * We score signals and derive the target ctx.
 *
 * Score table:
 *   touch recent (<500ms)          → +3 (towards RESPONSIVE/PERF)
 *   touch recent (<2s)             → +1
 *   perf scene active              → +4 (hard-set PERF, bypass scoring)
 *   big cluster util ≥ 70%         → +2
 *   big cluster nr_running ≥ 2     → +1
 *   PSI mem-some avg ≥ threshold   → -1 (memory pressure: back off)
 *   no touch for 5s + no perf      → -4 (decay towards BATTERY)
 *
 * Score → ctx mapping:
 *   ≥ 5  → PERF
 *   3-4  → RESPONSIVE
 *   1-2  → BALANCED
 *   ≤ 0  → BATTERY
 * ----------------------------------------------------------------------- */

/*
 * oaks_sample_big_cluster — PELT util of the A75 big cluster.
 *
 * BUG (was): rq->cpu_load[0] = rq->cfs.load.weight in NICE_0_LOAD
 *   units (~1M on 64-bit). For idle CPUs weight=0, so util was always
 *   0. cpu_load[] is a scheduling weight, NOT a utilisation value.
 *
 * FIX: cpu_util_cfs(rq) returns cfs.avg.util_avg — PELT EMA in
 *   [0, SCHED_CAPACITY_SCALE=1024]. Divide by capacity_orig_of(cpu)
 *   to get true utilisation%, accounting for A75 > A55 capacity.
 *   Add cpu_util_rt() so SCHED_FIFO render threads are counted.
 */
static void oaks_sample_big_cluster(void)
{
	unsigned long util_sum = 0;
	unsigned int nr = 0, avg_pct;
	int cpu;

	for (cpu = OAKS_BIG_FIRST; cpu <= OAKS_BIG_LAST; cpu++) {
		struct rq *rq = cpu_rq(cpu);
		unsigned long cap  = capacity_orig_of(cpu);
		unsigned long util = cpu_util_cfs(rq) + cpu_util_rt(rq);

		util = min(util, cap);
		if (cap > 0)
			util_sum += util * 100 / cap;
		nr += rq->nr_running;
	}

	avg_pct = util_sum / (OAKS_BIG_LAST - OAKS_BIG_FIRST + 1);
	WRITE_ONCE(oaks.sig.big_cluster_util, min(avg_pct, 100U));
	WRITE_ONCE(oaks.sig.big_nr_running,   nr);
}

/*
 * oaks_sample_psi_mem — read PSI MEM_SOME pressure from psi_system.
 *
 * psi_group.avg[state][window] layout (kernel/sched/psi.c):
 *   state:  IO_SOME=0 IO_FULL=1 MEM_SOME=2 MEM_FULL=3 CPU_SOME=4
 *   window: [0]=10s [1]=60s [2]=300s  (EXP_10s/60s/300s macros)
 *
 * Units: FIXED_1=2048 = 100% stall (FSHIFT=11, same as load_avg).
 *   10%=205, 5%=102. Updated every 2s by avgs_work.
 *
 * NOTE: the old comment said 'fixed-point /1000' — wrong.
 *   It also claimed index [1]=10s — wrong, [0]=10s.
 */
static void oaks_sample_psi_mem(void)
{
#ifdef CONFIG_PSI
	/* PSI_MEM_SOME=2, [0]=10s avg, units FIXED_1=2048 */
	unsigned long avg = READ_ONCE(psi_system.avg[PSI_MEM_SOME][0]);
	WRITE_ONCE(oaks.sig.psi_mem_some_avg, avg);
#else
	WRITE_ONCE(oaks.sig.psi_mem_some_avg, 0);
#endif
}

/*
 * oaks_detect_ctx — multi-signal scorer with per-context hysteresis.
 *
 * Score table:
 *   Touch <300ms   +3 | <1500ms  +2 | <4000ms  +1 | >8000ms  -3
 *   A75 util ≥70%  +3 | ≥40%     +2 | nr≥2     +1
 *   PSI mem ≥10%   -2
 *   Active perf scene → hard PERF (bypass scoring)
 *
 * Hysteresis table (prevents rapid BALANCED↔RESPONSIVE oscillation):
 *   To UPGRADE from cur ctx, score must exceed the upper threshold.
 *   To DOWNGRADE, score must fall below the lower threshold.
 *   This is encoded as per-ctx switch cases below.
 */
static enum oaks_ctx oaks_detect_ctx(void)
{
	unsigned long touch_age;
	unsigned int big_util, big_nr;
	unsigned long psi_avg;
	enum oaks_ctx cur;
	int score = 0;

	if (READ_ONCE(oaks.perf.main_pid) != 0)
		return OAKS_CTX_PERF;

	/* Snapshot all signals once */
	touch_age = jiffies - READ_ONCE(oaks.sig.last_touch_j);
	big_util   = READ_ONCE(oaks.sig.big_cluster_util);
	big_nr     = READ_ONCE(oaks.sig.big_nr_running);
	psi_avg    = READ_ONCE(oaks.sig.psi_mem_some_avg);
	cur        = (enum oaks_ctx)atomic_read(&oaks.ctx);

	/* Touch recency */
	if (touch_age < msecs_to_jiffies(300))
		score += 3;
	else if (touch_age < msecs_to_jiffies(1500))
		score += 2;
	else if (touch_age < msecs_to_jiffies(4000))
		score += 1;
	else if (touch_age > msecs_to_jiffies(8000))
		score -= 3;

	/* A75 big cluster utilisation (now via PELT, not cpu_load) */
	if (big_util >= 70)
		score += 3;
	else if (big_util >= 40)
		score += 2;
	if (big_nr >= 2)
		score += 1;

	/* Memory pressure — penalise harder than before */
	if (psi_avg >= OAKS_PSI_MEM_SOME_THRESH)
		score -= 2;

	/* Hysteresis: upgrade/downgrade require score past boundary ±1 */
	switch (cur) {
	case OAKS_CTX_PERF:
		if (score >= 5) return OAKS_CTX_PERF;
		if (score >= 3) return OAKS_CTX_RESPONSIVE;
		if (score >= 0) return OAKS_CTX_BALANCED;
		return OAKS_CTX_BATTERY;
	case OAKS_CTX_RESPONSIVE:
		if (score >= 7) return OAKS_CTX_PERF;
		if (score >= 3) return OAKS_CTX_RESPONSIVE;
		if (score >= 0) return OAKS_CTX_BALANCED;
		return OAKS_CTX_BATTERY;
	case OAKS_CTX_BALANCED:
		if (score >= 7) return OAKS_CTX_PERF;
		if (score >= 5) return OAKS_CTX_RESPONSIVE;
		if (score >= 1) return OAKS_CTX_BALANCED;
		return OAKS_CTX_BATTERY;
	case OAKS_CTX_BATTERY:
	default:
		if (score >= 7) return OAKS_CTX_PERF;
		if (score >= 5) return OAKS_CTX_RESPONSIVE;
		if (score >= 2) return OAKS_CTX_BALANCED;
		return OAKS_CTX_BATTERY;
	}
}

/*
 * oaks_tick — called from scheduler_tick() on every CPU.
 *
 * ROOT CAUSE of big_util/psi always 0:
 *   Sampling was gated behind static_branch_unlikely(&oaks_active).
 *   That key is only enabled when ctx > BALANCED. On an idle device
 *   (ctx=BALANCED, key=false) the function returned before any sampling
 *   occurred. oaks_detect_ctx() always read stale zeros → ctx never
 *   changed from load signals, only from touch events.
 *
 * FIX: rate-limit gate runs BEFORE the static_key check.
 *   Sampling is always unconditional. The static_key fast-path in
 *   oaks_in_perf() / oaks_is_responsive() is unaffected.
 *
 * ALSO FIXED: oaks_tick() was never called — it was not wired into
 *   scheduler_tick(). Wire-up is in kernel/sched/core.c.
 */
#define OAKS_EVAL_INTERVAL_J	(HZ / 10)  /* 100 ms */

void oaks_tick(void)
{
	static unsigned long next_eval_j;

	/*
	 * Rate-limit to one eval per 100ms from one CPU.
	 * MUST be before the static_key check — sampling runs in ALL contexts.
	 */
	if (time_before(jiffies, READ_ONCE(next_eval_j)))
		return;
	if (cpu_online(0) && smp_processor_id() != 0)
		return;
	WRITE_ONCE(next_eval_j, jiffies + OAKS_EVAL_INTERVAL_J);

	oaks_sample_big_cluster();
	oaks_sample_psi_mem();

	oaks_set_ctx(oaks_detect_ctx());

	if (oaks_lmk_enabled &&
	    READ_ONCE(oaks.sig.psi_mem_some_avg) >= OAKS_PSI_MEM_SOME_THRESH)
		oaks_lmk_pressure_check();
}
EXPORT_SYMBOL(oaks_tick);

/* -----------------------------------------------------------------------
 * Smart Low-Memory Killer
 *
 * Design:
 *  - Triggered by oaks_tick() when PSI mem-some avg is high, OR
 *    by the registered shrinker when the page allocator is stressed.
 *  - Victim selection: lowest adj ≥ sweep_min_adj, highest RSS wins.
 *  - Kill is issued via SIGKILL from a workqueue (not in IRQ/shrinker ctx).
 *  - Hysteresis: OAKS_LMK_MIN_INTERVAL_J between sweeps.
 *  - oaks_oom_guard() protects foreground + active perf scene.
 *  - Kills up to OAKS_LMK_MAX_KILL_PER_SWEEP per sweep.
 * ----------------------------------------------------------------------- */

static unsigned long oaks_mem_pressure_pct(void)
{
	struct sysinfo si;
	si_meminfo(&si);
	if (!si.totalram)
		return 0;
	/* used% = (total - free - buffers) / total * 100 */
	return 100 - (si.freeram + si.bufferram) * 100 / si.totalram;
}

/*
 * Pick the best kill target: highest RSS among processes with
 * oom_score_adj >= min_adj that are not guarded.
 * Must be called under rcu_read_lock().
 * Returns task with get_task_struct() held, or NULL.
 */
static struct task_struct *oaks_lmk_pick_victim(short min_adj)
{
	struct task_struct *p, *best = NULL;
	unsigned long best_rss = 0;

	for_each_process(p) {
		struct mm_struct *mm;
		unsigned long rss;
		short adj;

		/*
		 * Skip kernel threads, zombies, and tasks being killed.
		 */
		if (p->flags & (PF_KTHREAD | PF_EXITING))
			continue;

		if (!p->signal)
			continue;

		adj = READ_ONCE(p->signal->oom_score_adj);
		if (adj < min_adj)
			continue;

		if (oaks_oom_guard(p))
			continue;

		mm = p->mm;
		if (!mm)
			continue;

		rss = get_mm_rss(mm);
		if (rss > best_rss) {
			best_rss = rss;
			best = p;
		}
	}

	if (best)
		get_task_struct(best);

	return best;
}

static void oaks_lmk_kill_work_fn(struct work_struct *work)
{
	struct oaks_lmk_state *lmk =
		container_of(work, struct oaks_lmk_state, kill_work);
	short min_adj = READ_ONCE(lmk->sweep_min_adj);
	int killed = 0;

	while (killed < OAKS_LMK_MAX_KILL_PER_SWEEP) {
		struct task_struct *victim;
		unsigned long rss_kb;

		rcu_read_lock();
		victim = oaks_lmk_pick_victim(min_adj);
		rcu_read_unlock();

		if (!victim)
			break;

		rss_kb = get_mm_rss(victim->mm) << (PAGE_SHIFT - 10);

		pr_info("OAKS-LMK: killing pid=%d (%s) adj=%d rss=%lukB\n",
			victim->pid, victim->comm,
			victim->signal ? victim->signal->oom_score_adj : 0,
			rss_kb);

		send_sig(SIGKILL, victim, 0);
		atomic_long_add(rss_kb, &lmk->total_reclaimed_kb);
		atomic_long_inc(&lmk->total_kills);
		killed++;

		put_task_struct(victim);

		/* Bail early if pressure already relieved */
		if (oaks_mem_pressure_pct() < OAKS_LMK_PRESSURE_LOW)
			break;
	}

	WRITE_ONCE(lmk->last_sweep_j, jiffies);
	atomic_set(&lmk->pending, 0);
}

/*
 * oaks_lmk_pressure_check - decide whether to fire a sweep and at
 * what adj tier. Safe to call from any context (schedules work).
 */
void oaks_lmk_pressure_check(void)
{
	struct oaks_lmk_state *lmk = &oaks.lmk;
	unsigned long pct;
	short min_adj;

	if (!oaks_lmk_enabled)
		return;

	/* Hysteresis */
	if (time_before(jiffies, READ_ONCE(lmk->last_sweep_j) +
			OAKS_LMK_MIN_INTERVAL_J))
		return;

	/* Avoid stacking sweeps */
	if (atomic_cmpxchg(&lmk->pending, 0, 1) != 0)
		return;

	pct = oaks_mem_pressure_pct();

	if (pct >= OAKS_LMK_PRESSURE_CRITICAL)
		min_adj = OAKS_LMK_ADJ_NONEMPTY;  /* kill nonempty + cached */
	else if (pct >= OAKS_LMK_PRESSURE_HIGH)
		min_adj = OAKS_LMK_ADJ_CACHED;    /* kill cached only */
	else {
		/* Not enough pressure, cancel */
		atomic_set(&lmk->pending, 0);
		return;
	}

	WRITE_ONCE(lmk->sweep_min_adj, min_adj);
	schedule_work(&lmk->kill_work);
}
EXPORT_SYMBOL(oaks_lmk_pressure_check);

/* -----------------------------------------------------------------------
 * Shrinker: lets the page allocator trigger OAKS-LMK under direct reclaim
 * ----------------------------------------------------------------------- */

static unsigned long oaks_shrinker_count(struct shrinker *s,
					 struct shrink_control *sc)
{
	unsigned long pct = oaks_mem_pressure_pct();

	/* Report a non-zero object count only when we'd actually kill */
	if (pct >= OAKS_LMK_PRESSURE_HIGH)
		return pct; /* arbitrary non-zero; we don't manage a slab */
	return 0;
}

static unsigned long oaks_shrinker_scan(struct shrinker *s,
					struct shrink_control *sc)
{
	oaks_lmk_pressure_check();
	/*
	 * We don't actually shrink slab objects here; return SHRINK_STOP so
	 * the MM doesn't keep hammering us if reclaim isn't making progress.
	 */
	return SHRINK_STOP;
}

static struct shrinker oaks_shrinker = {
	.count_objects = oaks_shrinker_count,
	.scan_objects  = oaks_shrinker_scan,
	.seeks         = DEFAULT_SEEKS,
};

/* -----------------------------------------------------------------------
 * Init
 * ----------------------------------------------------------------------- */

void oaks_init(void)
{
	spin_lock_init(&oaks.lock);
	atomic_set(&oaks.ctx, OAKS_CTX_BALANCED);

	/* LMK */
	INIT_WORK(&oaks.lmk.kill_work, oaks_lmk_kill_work_fn);
	atomic_long_set(&oaks.lmk.total_kills, 0);
	atomic_long_set(&oaks.lmk.total_reclaimed_kb, 0);
	atomic_set(&oaks.lmk.pending, 0);
	oaks.lmk.last_sweep_j = jiffies;

	register_shrinker(&oaks_shrinker);

	oaks_apply_params(OAKS_CTX_BALANCED);

	pr_info("OAKS: initialized — MT6768/G85 cpu0-5=A55 cpu6-7=A75 "
		"lmk=%s\n", oaks_lmk_enabled ? "on" : "off");
}

/* -----------------------------------------------------------------------
 * Sysfs: /sys/kernel/oaks/
 *   context              rw  current ctx (0-3)
 *   responsive_timeout_ms rw  touch-decay timeout
 *   stats                ro  ctx switch counters + perf pids
 *   lmk_stats            ro  kill count + reclaimed KB
 *   lmk_enable           rw  enable/disable LMK (1/0)
 *   mem_pressure         ro  current used-memory %
 * ----------------------------------------------------------------------- */

static struct kobject *oaks_kobj;

static ssize_t context_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	static const char * const names[] = {
		"battery", "balanced", "responsive", "perf"
	};
	int ctx = atomic_read(&oaks.ctx);

	return scnprintf(buf, PAGE_SIZE,
			 "%d (%s) big_util=%u%% psi_mem=%lu touch_age=%ums\n",
			 ctx, names[ctx],
			 READ_ONCE(oaks.sig.big_cluster_util),
			 READ_ONCE(oaks.sig.psi_mem_some_avg),
			 jiffies_to_msecs(jiffies -
					  READ_ONCE(oaks.sig.last_touch_j)));
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
	return scnprintf(buf, PAGE_SIZE, "%u\n",
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
	return scnprintf(buf, PAGE_SIZE,
			 "battery=%lu balanced=%lu responsive=%lu perf=%lu\n"
			 "perf_main_pid=%d perf_render_pid=%d\n"
			 "big_cluster_util=%u%% big_nr_running=%u\n"
			 "psi_mem_some_avg=%lu\n",
			 oaks.ctx_switches[OAKS_CTX_BATTERY],
			 oaks.ctx_switches[OAKS_CTX_BALANCED],
			 oaks.ctx_switches[OAKS_CTX_RESPONSIVE],
			 oaks.ctx_switches[OAKS_CTX_PERF],
			 oaks.perf.main_pid,
			 oaks.perf.render_pid,
			 READ_ONCE(oaks.sig.big_cluster_util),
			 READ_ONCE(oaks.sig.big_nr_running),
			 READ_ONCE(oaks.sig.psi_mem_some_avg));
}
static struct kobj_attribute oaks_stats_attr =
	__ATTR(stats, 0444, stats_show, NULL);

static ssize_t lmk_stats_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			 "total_kills=%ld reclaimed_kb=%ld last_sweep_age_ms=%u\n",
			 atomic_long_read(&oaks.lmk.total_kills),
			 atomic_long_read(&oaks.lmk.total_reclaimed_kb),
			 jiffies_to_msecs(jiffies - READ_ONCE(oaks.lmk.last_sweep_j)));
}
static struct kobj_attribute oaks_lmk_stats_attr =
	__ATTR(lmk_stats, 0444, lmk_stats_show, NULL);

static ssize_t lmk_enable_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", oaks_lmk_enabled);
}

static ssize_t lmk_enable_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(oaks_lmk_enabled, val);
	return count;
}
static struct kobj_attribute oaks_lmk_enable_attr =
	__ATTR(lmk_enable, 0644, lmk_enable_show, lmk_enable_store);

static ssize_t mem_pressure_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%lu%%\n", oaks_mem_pressure_pct());
}
static struct kobj_attribute oaks_mem_pressure_attr =
	__ATTR(mem_pressure, 0444, mem_pressure_show, NULL);

static struct attribute *oaks_attrs[] = {
	&oaks_ctx_attr.attr,
	&oaks_timeout_attr.attr,
	&oaks_stats_attr.attr,
	&oaks_lmk_stats_attr.attr,
	&oaks_lmk_enable_attr.attr,
	&oaks_mem_pressure_attr.attr,
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
	if (ret) {
		kobject_put(oaks_kobj);
		return ret;
	}

	oaks_init();
	return 0;
}
late_initcall(oaks_sysfs_init);
