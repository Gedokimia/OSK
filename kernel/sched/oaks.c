// SPDX-License-Identifier: GPL-2.0
/*
 * OAKS - Optimized Adaptive Kernel Scheduler
 * OSK-16.2-ksun, Redmi 12C / Poco C55 (MediaTek G85)
 *
 * Features:
 *  - Multi-signal context detection: touch, big-cluster PELT util,
 *    PSI memory pressure, frame-deadline misses (vsync), active perf scene
 *  - Per-ctx hysteresis table prevents rapid BALANCED<->RESPONSIVE oscillation
 *  - BORE burst-penalty-scale push per context
 *  - Deferred ctx transitions via workqueue (static_key flips need a
 *    sleeping mutex, so hardirq/touch-irq contexts only set an atomic
 *    and schedule_work; the actual flip runs in process context)
 *  - SF/AudioFlinger/HWC thread classification via uclamp_min floors,
 *    applied at __set_task_comm() time (oaks_classify_thread)
 *  - Render-thread uclamp_min applied/cleared on perf-scene enter/exit
 *  - Vsync-driven frame-miss tracking feeds back into ctx scoring
 *  - Sysfs: context, responsive_timeout_ms, stats
 */

#include "sched.h"
#include "oaks.h"
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/psi.h>
#include <linux/workqueue.h>

/* -----------------------------------------------------------------------
 * Globals
 * ----------------------------------------------------------------------- */

DEFINE_STATIC_KEY_FALSE(oaks_active);
EXPORT_SYMBOL(oaks_active);

struct oaks_state oaks = {
	.ctx			= ATOMIC_INIT(OAKS_CTX_BALANCED),
	.pending_ctx		= ATOMIC_INIT(OAKS_CTX_BALANCED),
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
		/*
		 * Long latency period: background tasks share the CPU in larger
		 * slices with infrequent preemption — reduces context-switch
		 * overhead and scheduler tick CPU cost when screen is off.
		 * High BORE penalty: burst tasks (JIT, sync) are dampened
		 * aggressively so they don't prevent the A55 from entering
		 * deep idle states between background work items.
		 */
		.latency_ns		= 8000000,
		.min_gran_ns		= 1000000,
		.wakeup_gran_ns		= 1500000,
		.nr_latency		= 6,
		.bore_burst_penalty_scale = 1792,
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

/* UCLAMP — defined + exported in kernel/sched/core.c */
extern int set_task_util_min(pid_t pid, unsigned int util_min);
/*
 * sched_nr_latency: static in fair.c — cannot extern. fair.c
 * recomputes it whenever sysctl_sched_latency is written.
 * sysctl_sched_pelt_halflife: does not exist in this 4.19 tree.
 */

#ifdef CONFIG_SCHED_BORE
extern unsigned int sched_burst_penalty_scale; /* uint __read_mostly */
#endif


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

	/*
	 * Re-apply the render thread's uclamp floor whenever ctx changes.
	 * uclamp is per-task and is NOT touched by sysctl_sched_* writes,
	 * so a PERF→RESPONSIVE transition (game still running, e.g. user
	 * stopped touching but render thread still active) must not lose
	 * the render thread's floor. Conversely, leaving PERF entirely
	 * (perf.render_pid cleared by oaks_notify_perf_scene) means this
	 * loop has nothing to do — render_pid is already 0.
	 *
	 * We only re-assert in PERF; RESPONSIVE/BALANCED/BATTERY do not
	 * touch the render thread's uclamp (it keeps whatever was set on
	 * scene entry until oaks_notify_perf_scene(..., false) resets it).
	 */
	if (ctx == OAKS_CTX_PERF) {
		pid_t render_pid = READ_ONCE(oaks.perf.render_pid);

		if (render_pid > 0)
			set_task_util_min(render_pid, OAKS_UCLAMP_SF_RENDER);
	}
}

/*
 * oaks_ctx_work_fn — workqueue handler for deferred ctx transitions.
 *
 * static_branch_enable/disable require jump_label_mutex (a sleeping
 * mutex) and cpus_read_lock. They MUST NOT be called from hardirq
 * context (scheduler_tick, input_handle_event with irqs disabled).
 *
 * This work item is scheduled by oaks_set_ctx() which is safe to call
 * from any context. The actual static_key flip runs here in process ctx.
 */
static void oaks_ctx_work_fn(struct work_struct *work)
{
	enum oaks_ctx ctx = (enum oaks_ctx)atomic_read(&oaks.pending_ctx);
	unsigned long flags;
	enum oaks_ctx old;

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
	spin_unlock_irqrestore(&oaks.lock, flags);

	/*
	 * Static key transition — safe here because we are in process
	 * context (workqueue), not hardirq. The key gates oaks_in_perf()
	 * and oaks_is_responsive() fast-paths to near-zero cost in
	 * BATTERY and BALANCED.
	 */
	if (ctx > OAKS_CTX_BALANCED)
		static_branch_enable(&oaks_active);
	else
		static_branch_disable(&oaks_active);

	pr_debug("OAKS: ctx %d -> %d (touch=%lu psi=%lu big_util=%u)\n",
		 old, ctx,
		 READ_ONCE(oaks.sig.last_touch_j),
		 READ_ONCE(oaks.sig.psi_mem_some_avg),
		 READ_ONCE(oaks.sig.big_cluster_util));
}

/*
 * oaks_set_ctx — request a context transition.
 *
 * Safe to call from ANY context including hardirq and irqs-disabled.
 * Writes the desired ctx atomically and schedules oaks_ctx_work_fn()
 * to perform the actual static_key flip in process context.
 *
 * oaks_apply_params() (writing sysctl_sched_* via WRITE_ONCE) is also
 * deferred to the workqueue for the same reason: fair.c reads these
 * in __schedule() which runs with interrupts enabled but we want
 * consistent updates.
 */
void oaks_set_ctx(enum oaks_ctx ctx)
{
	if (WARN_ON_ONCE(ctx >= OAKS_CTX_MAX))
		return;
	if ((enum oaks_ctx)atomic_read(&oaks.ctx) == ctx)
		return;

	atomic_set(&oaks.pending_ctx, (int)ctx);
	schedule_work(&oaks.ctx_work);
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
	/*
	 * Called from input_handle_event with dev->event_lock held and
	 * interrupts disabled (hardirq context). Only WRITE_ONCE to the
	 * timestamp is safe here. The ctx bump is handled by oaks_set_ctx()
	 * which defers the static_key work to process context.
	 */
	WRITE_ONCE(oaks.sig.last_touch_j, jiffies);
	if (oaks_get_ctx() < OAKS_CTX_RESPONSIVE)
		oaks_set_ctx(OAKS_CTX_RESPONSIVE);
}
EXPORT_SYMBOL(oaks_notify_touch);

/*
 * oaks_notify_frame_end — render thread finished producing a frame.
 * Called from FPSGO_QUEUE producer-end (eglSwapBuffers/vkQueuePresent).
 * Safe from any context: WRITE_ONCE only.
 */
void oaks_notify_frame_end(void)
{
	WRITE_ONCE(oaks.sig.last_frame_end_j, jiffies);
}
EXPORT_SYMBOL(oaks_notify_frame_end);

/*
 * oaks_notify_vsync — called from the FPSGO vsync path on every vsync.
 *
 * Deadline-miss detection: a frame "makes" this vsync if
 * oaks_notify_frame_end() was called at any point after the *previous*
 * vsync. If a perf scene is active (render_pid != 0) and no frame
 * completed in that window, the render thread missed its deadline —
 * increment frame_miss_count. Otherwise decrement (saturating at 0).
 *
 * frame_miss_count is read by oaks_detect_ctx() as a strong PERF-upgrade
 * signal — repeated misses mean current ctx's CFS/BORE params are too
 * loose for the workload, independent of touch/util signals which lag
 * actual demand by tens of ms (PELT decay).
 *
 * When no perf scene is active, frame_miss_count only decays — vsync
 * events with no game running must not accumulate false misses.
 *
 * Safe from any context: WRITE_ONCE/READ_ONCE + atomic ops + jiffies
 * comparisons only, no locks.
 */
void oaks_notify_vsync(unsigned int fps)
{
	unsigned long now = jiffies;
	unsigned long prev_vsync, frame_end;
	bool scene_active = READ_ONCE(oaks.perf.render_pid) != 0;

	if (fps != 0)
		WRITE_ONCE(oaks.sig.vsync_fps, fps);

	prev_vsync = READ_ONCE(oaks.sig.last_vsync_j);
	frame_end  = READ_ONCE(oaks.sig.last_frame_end_j);
	WRITE_ONCE(oaks.sig.last_vsync_j, now);

	if (!scene_active) {
		atomic_dec_if_positive(&oaks.sig.frame_miss_count);
		return;
	}

	if (time_after(frame_end, prev_vsync)) {
		/* A frame completed since the previous vsync — on time */
		atomic_dec_if_positive(&oaks.sig.frame_miss_count);
	} else {
		/* No frame completed — missed this vsync's deadline */
		if (atomic_read(&oaks.sig.frame_miss_count) < OAKS_FRAME_MISS_MAX)
			atomic_inc(&oaks.sig.frame_miss_count);
	}
}
EXPORT_SYMBOL(oaks_notify_vsync);

/*
 * oaks_notify_perf_scene — register/clear the active game's main+render
 * thread PIDs. Called from perf_ioctl FPSGO_QUEUE_CONNECT / FPSGO_QUEUE.
 *
 * On enter:
 *   - Record main_pid (tgid) / render_pid (tid).
 *   - Apply uclamp_min=OAKS_UCLAMP_SF_RENDER to the render thread so EAS
 *     places it on the A75 cluster at ≥50% capacity immediately, without
 *     waiting for oaks_tick() to detect load and transition ctx.
 *   - Force ctx=PERF.
 *
 * On exit:
 *   - Reset the previously-boosted render thread's uclamp_min to 0
 *     (OAKS_UCLAMP_RESET) so it doesn't retain an A75 floor after the
 *     game exits — this would otherwise waste power on a backgrounded
 *     render thread.
 *   - Clear perf.{main,render}_pid.
 *   - Drop to RESPONSIVE; oaks_tick() decays to BALANCED after timeout.
 *
 * Safe to call from process context only (set_task_util_min sleeps via
 * sched_setattr_nocheck → rt_mutex). perf_ioctl handlers run in process
 * context (ioctl syscall), so this is fine.
 */
void oaks_notify_perf_scene(pid_t main_pid, pid_t render_pid, bool enter)
{
	unsigned long flags;
	pid_t old_render_pid;

	spin_lock_irqsave(&oaks.lock, flags);
	old_render_pid = oaks.perf.render_pid;

	if (enter) {
		oaks.perf.main_pid   = main_pid;
		oaks.perf.render_pid = render_pid;
		WRITE_ONCE(oaks.sig.last_render_wake_j, jiffies);
		spin_unlock_irqrestore(&oaks.lock, flags);

		/*
		 * If the render thread changed (new game, or thread churn
		 * within the same game), clear the old thread's floor first
		 * so it doesn't stay pinned to A75 after losing the role.
		 */
		if (old_render_pid > 0 && old_render_pid != render_pid)
			set_task_util_min(old_render_pid, OAKS_UCLAMP_RESET);

		if (render_pid > 0)
			set_task_util_min(render_pid, OAKS_UCLAMP_SF_RENDER);

		oaks_set_ctx(OAKS_CTX_PERF);
	} else {
		oaks.perf.main_pid   = 0;
		oaks.perf.render_pid = 0;
		spin_unlock_irqrestore(&oaks.lock, flags);

		if (old_render_pid > 0)
			set_task_util_min(old_render_pid, OAKS_UCLAMP_RESET);

		/* Transition to RESPONSIVE briefly; tick decays to BALANCED */
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
	int frame_miss;
	enum oaks_ctx cur;
	int score = 0;

	if (READ_ONCE(oaks.perf.main_pid) != 0)
		return OAKS_CTX_PERF;

	/* Snapshot all signals once */
	touch_age = jiffies - READ_ONCE(oaks.sig.last_touch_j);
	big_util   = READ_ONCE(oaks.sig.big_cluster_util);
	big_nr     = READ_ONCE(oaks.sig.big_nr_running);
	psi_avg    = READ_ONCE(oaks.sig.psi_mem_some_avg);
	frame_miss = atomic_read(&oaks.sig.frame_miss_count);
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

	/*
	 * Frame miss — strong upgrade signal independent of CPU util.
	 * A render thread can miss vsync deadlines due to GPU-bound work,
	 * binder IPC latency, or scheduling latency that PELT util doesn't
	 * capture yet (util_avg lags actual demand by tens of ms).
	 * 1-2 misses: transient, +2. 3+ misses: sustained jank, +4 — this
	 * alone can push BALANCED straight to RESPONSIVE/PERF.
	 */
	if (frame_miss >= 3)
		score += 4;
	else if (frame_miss >= 1)
		score += 2;

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

	/*
	 * oaks_set_ctx() is now safe to call from hardirq: it only writes
	 * an atomic and schedules ctx_work. The static_key flip and
	 * sched param updates happen in oaks_ctx_work_fn() (process ctx).
	 */
	oaks_set_ctx(oaks_detect_ctx());
}
EXPORT_SYMBOL(oaks_tick);



/* -----------------------------------------------------------------------
 * Init
 * ----------------------------------------------------------------------- */

/* -----------------------------------------------------------------------
 * Thread classification — SurfaceFlinger, AudioFlinger, display workers
 *
 * Android critical threads name themselves via pthread_setname_np() which
 * calls prctl(PR_SET_NAME) → __set_task_comm(). We intercept this to
 * apply a uclamp_min floor appropriate to each thread class.
 *
 * Rules:
 *  - Matching is by comm prefix/exact — TASK_COMM_LEN=16 so names are
 *    already truncated by the time we see them.
 *  - We call set_task_util_min() which uses sched_setattr_nocheck().
 *    This is safe in process context (called from __set_task_comm
 *    after task_unlock).
 *  - We only set a floor; the scheduler can run threads above it.
 *    uclamp_max is left at 1024 (uncapped) so bursts are not throttled.
 *  - Thread names here are stable across AOSP/HyperOS on MT6768.
 * ----------------------------------------------------------------------- */

struct oaks_thread_entry {
	const char *prefix;    /* comm prefix to match */
	unsigned int uclamp;   /* OAKS_UCLAMP_* value  */
};

static const struct oaks_thread_entry oaks_thread_table[] = {
	/* SurfaceFlinger render path — must run at adequate OPP on big cluster */
	{ "RenderEngine",    OAKS_UCLAMP_SF_RENDER },
	{ "DispSync",        OAKS_UCLAMP_SF_RENDER },
	{ "app",             OAKS_UCLAMP_SF_RENDER }, /* SF per-app thread */
	{ "appSf",           OAKS_UCLAMP_SF_RENDER },
	/* SurfaceFlinger main/event threads */
	{ "surfaceflinger",  OAKS_UCLAMP_SF_MAIN   },
	{ "SurfaceFlinger",  OAKS_UCLAMP_SF_MAIN   },
	{ "EventThread",     OAKS_UCLAMP_SF_MAIN   },
	{ "HwBinder",        OAKS_UCLAMP_DISPLAY_HWC },
	/* AudioFlinger fast paths — latency critical */
	{ "FastMixer",       OAKS_UCLAMP_AUDIO_FAST },
	{ "FastCapture",     OAKS_UCLAMP_AUDIO_FAST },
	/* AudioFlinger mixer / IO threads */
	{ "AudioOut",        OAKS_UCLAMP_AUDIO_MIX  },
	{ "AudioIn",         OAKS_UCLAMP_AUDIO_MIX  },
	{ "AudioFlinger",    OAKS_UCLAMP_AUDIO_MIX  },
	{ "AudioMixer",      OAKS_UCLAMP_AUDIO_MIX  },
	/* Sentinel */
	{ NULL, 0 },
};

/*
 * oaks_classify_thread — called from __set_task_comm() (fs/exec.c).
 * Applies a uclamp_min floor to recognised latency-critical threads.
 * Safe: process context, no locks held on entry.
 */
void oaks_classify_thread(struct task_struct *tsk)
{
	const struct oaks_thread_entry *e;

	if (!tsk)
		return;

	for (e = oaks_thread_table; e->prefix; e++) {
		if (strncmp(tsk->comm, e->prefix, strlen(e->prefix)) == 0) {
			/*
			 * set_task_util_min uses sched_setattr_nocheck —
			 * valid from process context with a live task_struct.
			 * Ignore return: if UCLAMP is disabled the call is a
			 * no-op (sched_setattr returns -EINVAL gracefully).
			 */
			set_task_util_min(tsk->pid, e->uclamp);
			return;
		}
	}
}
EXPORT_SYMBOL(oaks_classify_thread);

void oaks_init(void)
{
	spin_lock_init(&oaks.lock);
	atomic_set(&oaks.ctx, OAKS_CTX_BALANCED);
	atomic_set(&oaks.pending_ctx, OAKS_CTX_BALANCED);
	atomic_set(&oaks.sig.frame_miss_count, 0);
	WRITE_ONCE(oaks.sig.vsync_fps, 60);
	WRITE_ONCE(oaks.sig.last_frame_end_j, jiffies);
	WRITE_ONCE(oaks.sig.last_vsync_j, jiffies);

	/* Deferred ctx transition work — safe to call from any context */
	INIT_WORK(&oaks.ctx_work, oaks_ctx_work_fn);

	oaks_apply_params(OAKS_CTX_BALANCED);

	pr_info("OAKS: initialized — MT6768/G85 cpu0-5=A55 cpu6-7=A75\n");
}

/* -----------------------------------------------------------------------
 * Sysfs: /sys/kernel/oaks/
 *   context               rw  current ctx (0-3)
 *   responsive_timeout_ms rw  touch-decay timeout in ms
 *   stats                 ro  ctx switches, perf pids, cluster util, PSI
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
			 "psi_mem_some_avg=%lu\n"
			 "vsync_fps=%u frame_miss_count=%d\n"
			 "render_wake_age_ms=%u\n",
			 oaks.ctx_switches[OAKS_CTX_BATTERY],
			 oaks.ctx_switches[OAKS_CTX_BALANCED],
			 oaks.ctx_switches[OAKS_CTX_RESPONSIVE],
			 oaks.ctx_switches[OAKS_CTX_PERF],
			 READ_ONCE(oaks.perf.main_pid),
			 READ_ONCE(oaks.perf.render_pid),
			 READ_ONCE(oaks.sig.big_cluster_util),
			 READ_ONCE(oaks.sig.big_nr_running),
			 READ_ONCE(oaks.sig.psi_mem_some_avg),
			 READ_ONCE(oaks.sig.vsync_fps),
			 atomic_read(&oaks.sig.frame_miss_count),
			 jiffies_to_msecs(jiffies -
					  READ_ONCE(oaks.sig.last_render_wake_j)));
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
	if (ret) {
		kobject_put(oaks_kobj);
		return ret;
	}

	oaks_init();
	return 0;
}
late_initcall(oaks_sysfs_init);
