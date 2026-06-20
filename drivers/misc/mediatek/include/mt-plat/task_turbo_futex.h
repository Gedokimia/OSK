/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019 MediaTek Inc.
 */

#ifndef _TASK_TURBO_FUTEX_H_
#define _TASK_TURBO_FUTEX_H_

#include "turbo_common.h"

inline void futex_plist_add(struct futex_q *q, struct futex_hash_bucket *hb)
{
	struct futex_q *this, *next;
	struct plist_node *current_node = &q->list;
	struct plist_node *this_node;

	/*
	 * OSK: only take the O(n) scan-and-insert path when the CURRENT
	 * (enqueuing) task is itself turbo. The previous condition was
	 * `!sub_feat_enable(SUB_FEAT_LOCK) && !is_turbo_task(current)` --
	 * since SUB_FEAT_LOCK is a global feature flag (task_turbo_feats
	 * bitmask), once it is enabled system-wide (as of task_turbo_feats=15
	 * default) EVERY futex_wait() call -- turbo or not -- fell through
	 * to the linear plist_for_each_entry_safe() scan of the entire wait
	 * chain. On a heavily-contended futex (e.g. a busy JNI/native mutex
	 * with dozens of waiters) this turned every non-turbo thread's
	 * enqueue into an O(n) operation, even though only turbo threads
	 * (bounded to TURBO_PID_COUNT=8 system-wide) need priority-ordered
	 * insertion ahead of non-PI/non-RT waiters.
	 *
	 * Non-turbo tasks now always take the O(1) plist_add() fast path,
	 * regardless of whether SUB_FEAT_LOCK is globally enabled. Turbo
	 * tasks (game RenderThread, emulator JIT main+workers) still get
	 * priority-ordered insertion so they aren't stuck waiting behind
	 * lower-priority holders of a contended lock.
	 */
	if (!is_turbo_task(current)) {
		plist_add(&q->list, &hb->chain);
		return;
	}

	plist_for_each_entry_safe(this, next, &hb->chain, list) {
		if ((!this->pi_state || !this->rt_waiter)
		  && !is_turbo_task(this->task)) {
			this_node = &this->list;
			list_add(&current_node->node_list,
				 this_node->node_list.prev);
			return;
		}
	}

	plist_add(&q->list, &hb->chain);
}
#endif
