/* SPDX-License-Identifier: GPL-2.0 */
/*
 * fs/f2fs/gc.h
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *             http://www.samsung.com/
 */
#define GC_THREAD_MIN_WB_PAGES		1	/*
						 * a threshold to determine
						 * whether IO subsystem is idle
						 * or not
						 */
/*
 * OSK F2FS GC thread sleep intervals (all in milliseconds):
 *
 * URGENT (near OOSpace): 200ms.
 *   Faster response than upstream 500ms; on a nearly-full eMMC partition
 *   every extra second of stall increases the risk of ENOSPC during app
 *   writes.  200ms is tight enough to keep free segments above the
 *   watermark without hammering the eMMC continuously.
 *
 * MIN / MAX (normal background GC window): 20s–30s.
 *   Upstream uses 30s–60s.  Tightening to 20s–30s reduces the maximum
 *   time GC can be starved by foreground IO without adding significant
 *   erase-cycle cost on a healthy (>10% free) volume.
 *
 * NOGC (nothing to collect): 5 min — unchanged from upstream.
 */
#define DEF_GC_THREAD_URGENT_SLEEP_TIME	200
#define DEF_GC_THREAD_MIN_SLEEP_TIME	20000
#define DEF_GC_THREAD_MAX_SLEEP_TIME	30000
#define DEF_GC_THREAD_NOGC_SLEEP_TIME	300000
#define LIMIT_INVALID_BLOCK	40 /* percentage over total user space */
#define LIMIT_FREE_BLOCK	40 /* percentage over invalid + free space */

#define DEF_GC_FAILED_PINNED_FILES	2048

/* Search max. number of dirty segments to select a victim segment */
/*
 * OSK: reduce max victim search from 4096 to 2048 segments.
 * A full 4096-segment search on a fragmented 128GB eMMC volume can take
 * several hundred milliseconds, blocking foreground writeback behind it.
 * 2048 covers a 4GB partition cleanly and keeps the search bounded.
 * The urgency path (gc_urgent sysfs) already sets a tighter per-call
 * limit inside f2fs_gc(); this constant caps the global maximum.
 */
#define DEF_MAX_VICTIM_SEARCH 2048

struct f2fs_gc_kthread {
	struct task_struct *f2fs_gc_task;
	wait_queue_head_t gc_wait_queue_head;

	/* for gc sleep time */
	unsigned int urgent_sleep_time;
	unsigned int min_sleep_time;
	unsigned int max_sleep_time;
	unsigned int no_gc_sleep_time;

	/* for changing gc mode */
	unsigned int gc_wake;
};

struct gc_inode_list {
	struct list_head ilist;
	struct radix_tree_root iroot;
};

/*
 * inline functions
 */
static inline block_t free_user_blocks(struct f2fs_sb_info *sbi)
{
	if (free_segments(sbi) < overprovision_segments(sbi))
		return 0;
	else
		return (free_segments(sbi) - overprovision_segments(sbi))
			<< sbi->log_blocks_per_seg;
}

static inline block_t limit_invalid_user_blocks(struct f2fs_sb_info *sbi)
{
	return (long)(sbi->user_block_count * LIMIT_INVALID_BLOCK) / 100;
}

static inline block_t limit_free_user_blocks(struct f2fs_sb_info *sbi)
{
	block_t reclaimable_user_blocks = sbi->user_block_count -
		written_block_count(sbi);
	return (long)(reclaimable_user_blocks * LIMIT_FREE_BLOCK) / 100;
}

static inline void increase_sleep_time(struct f2fs_gc_kthread *gc_th,
							unsigned int *wait)
{
	unsigned int min_time = gc_th->min_sleep_time;
	unsigned int max_time = gc_th->max_sleep_time;

	if (*wait == gc_th->no_gc_sleep_time)
		return;

	if ((long long)*wait + (long long)min_time > (long long)max_time)
		*wait = max_time;
	else
		*wait += min_time;
}

static inline void decrease_sleep_time(struct f2fs_gc_kthread *gc_th,
							unsigned int *wait)
{
	unsigned int min_time = gc_th->min_sleep_time;

	if (*wait == gc_th->no_gc_sleep_time)
		*wait = gc_th->max_sleep_time;

	if ((long long)*wait - (long long)min_time < (long long)min_time)
		*wait = min_time;
	else
		*wait -= min_time;
}

static inline bool has_enough_invalid_blocks(struct f2fs_sb_info *sbi)
{
	block_t invalid_user_blocks = sbi->user_block_count -
					written_block_count(sbi);
	/*
	 * Background GC is triggered with the following conditions.
	 * 1. There are a number of invalid blocks.
	 * 2. There is not enough free space.
	 */
	if (invalid_user_blocks > limit_invalid_user_blocks(sbi) &&
			free_user_blocks(sbi) < limit_free_user_blocks(sbi))
		return true;
	return false;
}
