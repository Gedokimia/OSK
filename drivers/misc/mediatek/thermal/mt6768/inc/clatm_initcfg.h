/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
*/

#ifndef __CLATM_INITCFG_H__
#define __CLATM_INITCFG_H__

#define CLATM_SET_INIT_CFG			(1)

/*
 * OSK: Active Thermal Management config 0 (CLATM_INIT_CFG_ACTIVE_ATM_COOLER
 * selects this profile), enabled at TRIP_3=72C (tzcpu_initcfg.h).
 *
 * TARGET_TJ 75C -> 80C: ATM regulates CPU/GPU power so the SoC
 *   settles near this junction temperature. Paired with the relaxed
 *   TRIP_3=72C enable point and TRIP_1=100C hard cap, 80C sits
 *   comfortably in the middle with 20C of margin to the 117C reset.
 *
 * EXIT_POINT 10000 -> 8000: the original config encodes the invariant
 *   TRIP_3_TEMP == TARGET_TJ - EXIT_POINT (65 == 75 - 10). Updating
 *   EXIT_POINT to 8000 preserves this same invariant at the new values
 *   (72 == 80 - 8), so ATM's internal "back off the power trim" point
 *   continues to line up with the TRIP_3 enable temperature exactly as
 *   in the original tuning — no new dead-band/oscillation behaviour is
 *   introduced at the enable boundary.
 *
 * MAX_CPU_PWR 2000 -> 2500mW, MAX_GPU_PWR 700 -> 900mW: raises the
 *   power ceiling ATM allows before it starts trimming further. This
 *   is the budget available once TARGET_TJ is reached/exceeded — a
 *   higher ceiling means sustained loads (games) keep more CPU/GPU
 *   headroom while ATM gently regulates toward 80C instead of
 *   aggressively capping at the old 2000/700mW ceiling.
 *
 * MIN_CPU_PWR/MIN_GPU_PWR unchanged: these are the worst-case floor
 *   ATM will reduce to if temperature keeps climbing despite the
 *   higher ceiling — the safety floor is untouched.
 *
 * THETA_RISE/THETA_FALL/FIRST_STEP/MIN_BUDGET_CHG unchanged: these
 *   control step size and reaction speed of the ATM control loop,
 *   independent of the temperature/power targets being relaxed here.
 */
#define CLATM_INIT_CFG_0_TARGET_TJ		(80000)
#define CLATM_INIT_CFG_0_EXIT_POINT		(8000)
#define CLATM_INIT_CFG_0_FIRST_STEP		(2000)
#define CLATM_INIT_CFG_0_THETA_RISE		(2)
#define CLATM_INIT_CFG_0_THETA_FALL		(8)
#define CLATM_INIT_CFG_0_MIN_BUDGET_CHG		(1)
#define CLATM_INIT_CFG_0_MIN_CPU_PWR	(400)
#define CLATM_INIT_CFG_0_MAX_CPU_PWR	(2500)
#define CLATM_INIT_CFG_0_MIN_GPU_PWR	(200)
#define CLATM_INIT_CFG_0_MAX_GPU_PWR	(900)

#define CLATM_INIT_CFG_1_TARGET_TJ		(65000)
#define CLATM_INIT_CFG_1_EXIT_POINT		(10000)
#define CLATM_INIT_CFG_1_FIRST_STEP		(3000)
#define CLATM_INIT_CFG_1_THETA_RISE		(2)
#define CLATM_INIT_CFG_1_THETA_FALL		(8)
#define CLATM_INIT_CFG_1_MIN_BUDGET_CHG		(1)
#define CLATM_INIT_CFG_1_MIN_CPU_PWR		(300)
#define CLATM_INIT_CFG_1_MAX_CPU_PWR		(3000)
#define CLATM_INIT_CFG_1_MIN_GPU_PWR		(800)
#define CLATM_INIT_CFG_1_MAX_GPU_PWR		(2000)

#define CLATM_INIT_CFG_2_TARGET_TJ		(75000)
#define CLATM_INIT_CFG_2_EXIT_POINT		(10000)
#define CLATM_INIT_CFG_2_FIRST_STEP		(3960)
#define CLATM_INIT_CFG_2_THETA_RISE		(2)
#define CLATM_INIT_CFG_2_THETA_FALL		(8)
#define CLATM_INIT_CFG_2_MIN_BUDGET_CHG		(1)
#define CLATM_INIT_CFG_2_MIN_CPU_PWR		(600)
#define CLATM_INIT_CFG_2_MAX_CPU_PWR		(3960)
#define CLATM_INIT_CFG_2_MIN_GPU_PWR		(800)
#define CLATM_INIT_CFG_2_MAX_GPU_PWR		(2000)

#define CLATM_INIT_CFG_ACTIVE_ATM_COOLER	(0)

#define CLATM_INIT_CFG_CATM			(0)

#define CLATM_INIT_CFG_PHPB_CPU_TT		(10)
#define CLATM_INIT_CFG_PHPB_CPU_TP		(10)

#define CLATM_INIT_CFG_PHPB_GPU_TT		(80)
#define CLATM_INIT_CFG_PHPB_GPU_TP		(80)

#define CLATM_INIT_HRTIMER_POLLING_DELAY	(100)

#define CLATM_USE_MIN_CPU_OPP			(1)
#endif	/* __CLATM_INITCFG_H__ */
