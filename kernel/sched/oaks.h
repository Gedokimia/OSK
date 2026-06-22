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
 * Thread class uclamp_min values [0, 1024 = SCHED_CAPACITY_SCALE].
 *
 * These are applied via set_task_util_min() in oaks_classify_thread()
 * and give the EAS placement logic a minimum OPP floor so latency-
 * critical threads always land on a capable CPU at an adequate frequency.
 *
 * SF_RENDER / AUDIO_FAST: placed on A75 (big cluster) at ≥50% capacity
 *   → guarantees they always run above the low-power OPPs.
 *   A75 at 50% util ≈ 900 MHz, well above the 400MHz floor.
 *
 * AUDIO_MIX / SF_MAIN: A55 is fine; need a moderate OPP floor to avoid
 *   being stuck at the lowest efficiency OPP during audio output.
 *
 * KWORKER_DISPLAY: HWC/ion workers for display path; modest floor.
 */
#define OAKS_UCLAMP_SF_RENDER		512  /* 50% — SurfaceFlinger RenderEngine */
#define OAKS_UCLAMP_SF_MAIN		256  /* 25% — SurfaceFlinger main thread */
#define OAKS_UCLAMP_AUDIO_FAST		512  /* 50% — FastMixer / FastCapture */
#define OAKS_UCLAMP_AUDIO_MIX		256  /* 25% — AudioFlinger mixer */
#define OAKS_UCLAMP_DISPLAY_HWC		200  /* ~20% — HWC / display worker */
#define OAKS_UCLAMP_RESET		  0  /* remove OAKS boost */

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
 * Reads are lock-free (WRITE_ONCE/READ_ONCE); no ordering guarantees
 * needed since these are approximate signals for ctx scoring.
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
	/*
	 * Frame-completion tracking for vsync-deadline-miss detection:
	 *   last_frame_end_j: jiffies of the most recent completed frame
	 *                      (set by oaks_notify_frame_end() on
	 *                      FPSGO_QUEUE producer-end / eglSwapBuffers).
	 *   last_vsync_j:      jiffies of the previous vsync we processed.
	 * oaks_notify_vsync() compares these: if no frame completed since
	 * the previous vsync while a perf scene is active, frame_miss_count
	 * is incremented (deadline miss); otherwise decremented.
	 */
	unsigned long	last_frame_end_j;
	unsigned long	last_vsync_j;
	/*
	 * Frame miss counter, saturating in [0, OAKS_FRAME_MISS_MAX].
	 * Used as a strong signal for ctx upgrades in oaks_detect_ctx().
	 */
	atomic_t	frame_miss_count;
	/* Current vsync FPS (60/90/120), best-effort, default 60 */
	unsigned int	vsync_fps;
};

#define OAKS_FRAME_MISS_MAX	8

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

#ifdef CONFIG_OAKS

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

/*
 * oaks_notify_frame_end - mark that the render thread finished producing
 * a frame (FPSGO_QUEUE producer-end, i.e. eglSwapBuffers/vkQueuePresent).
 * Safe to call from any context (WRITE_ONCE only).
 */
void oaks_notify_frame_end(void);

/*
 * oaks_notify_vsync - called on each vsync event from the FPSGO path.
 * @fps: current display refresh rate (60/90/120), or 0 to keep the
 *       last known value (default 60 if never set).
 *
 * Compares the timestamp of the last completed frame
 * (oaks_notify_frame_end) against the previous vsync timestamp:
 *   - if a frame completed since the last vsync AND a perf scene is
 *     active: frame_miss_count decremented (on time)
 *   - if no frame completed AND a perf scene is active: frame_miss_count
 *     incremented (missed deadline)
 *   - if no perf scene is active: frame_miss_count decays only
 *
 * Safe to call from any context (only WRITE_ONCE + atomic ops + jiffies).
 */
void oaks_notify_vsync(unsigned int fps);

/*
 * oaks_classify_thread - apply thread-class uclamp_min based on comm name.
 * Called from __set_task_comm() whenever a thread names itself.
 * Process context only (task_lock may be dropped before calling).
 */
void oaks_classify_thread(struct task_struct *tsk);

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
 * oaks_oom_guard - return true if the OOM killer must skip @p.
 *
 * Only protects the active perf scene's main and render threads.
 * All other processes are fair game for the OOM killer.
 * Never protects adj <= 0 (foreground) — the OOM killer already
 * handles those correctly via oom_score_adj.
 *
 * Called from mm/oom_kill.c under task_lock(p), RCU read-side.
 * Must be safe to call from any context — only uses atomic reads.
 */
static inline bool oaks_oom_guard(struct task_struct *p)
{
	if (!oaks_in_perf())
		return false;
	if (!p->signal)
		return false;
	return (p->tgid == READ_ONCE(oaks.perf.main_pid) ||
		p->pid  == READ_ONCE(oaks.perf.render_pid));
}

/*
 * oaks_want_big_cluster - hint to place @p on A75 cluster.
 */
static inline bool oaks_want_big_cluster(struct task_struct *p)
{
	if (!oaks_in_perf())
		return false;
	return (p->pid  == READ_ONCE(oaks.perf.render_pid) ||
		p->tgid == READ_ONCE(oaks.perf.main_pid));
}

#else /* !CONFIG_OAKS */

/*
 * CONFIG_OAKS=n stubs.
 * Every symbol called unconditionally elsewhere in the tree (core.c
 * scheduler_tick, cpufreq_schedutil.c, fs/exec.c __set_task_comm) gets
 * a no-op/false-returning inline here so those call sites need no
 * #ifdef of their own. The peripheral hooks in mm/oom_kill.c,
 * mm/compaction.c, kernel/sysctl.c, drivers/input/input.c, and
 * perf_ioctl.c are already wrapped in their own #ifdef CONFIG_OAKS
 * blocks and simply compile out entirely.
 */
static inline void oaks_init(void) { }
static inline void oaks_set_ctx(enum oaks_ctx ctx) { }
static inline enum oaks_ctx oaks_get_ctx(void) { return OAKS_CTX_BALANCED; }
static inline void oaks_notify_touch(void) { }
static inline void oaks_notify_perf_scene(pid_t main_pid, pid_t render_pid,
					   bool enter) { }
static inline void oaks_tick(void) { }
static inline void oaks_notify_frame_end(void) { }
static inline void oaks_notify_vsync(unsigned int fps) { }
static inline void oaks_classify_thread(struct task_struct *tsk) { }
static inline bool oaks_in_perf(void) { return false; }
static inline bool oaks_is_responsive(void) { return false; }
static inline bool oaks_oom_guard(struct task_struct *p) { return false; }
static inline bool oaks_want_big_cluster(struct task_struct *p)
{
	return false;
}

#endif /* CONFIG_OAKS */

#endif /* _KERNEL_SCHED_OAKS_H */
