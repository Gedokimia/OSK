/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
*/

#ifndef __TZCPU_INITCFG_H__
#define __TZCPU_INITCFG_H__

#define TZCPU_SET_INIT_CFG			(1)

/*
 * OSK: thermal polling interval 40ms -> 50ms.
 * The CPU thermal zone is polled every TZCPU_INITCFG_INTERVAL ms to
 * read SoC thermal sensors and re-evaluate cooling device states.
 * 40ms = 25 polls/sec; 50ms = 20 polls/sec. Thermal time constants
 * for an SoC die are on the order of seconds, so 50ms still reacts
 * far faster than the thermal mass can change, while issuing 20%
 * fewer sensor-read + governor-decision cycles.
 */
#define TZCPU_INITCFG_INTERVAL			(50)
#define TZCPU_INITCFG_NUM_TRIPS			(4)

/*
 * TRIP_0: hardware-reset safety net. NOT changed — 117C is the
 * mtktscpu-sysrst trip that force-reboots the SoC to prevent damage.
 * This is the last line of defense and must retain full margin below
 * the silicon's absolute maximum junction temperature.
 */
#define TZCPU_INITCFG_TRIP_0_TEMP		(117000)
#define TZCPU_INITCFG_TRIP_0_COOLER		"mtktscpu-sysrst"

/*
 * OSK: TRIP_1 (cpu00, full-SoC CPU cap) 95C -> 100C.
 * Still 17C of margin below the 117C hardware reset (TRIP_0).
 * Lets sustained heavy workloads run 5C hotter before the most
 * severe CPU cap engages.
 */
#define TZCPU_INITCFG_TRIP_1_TEMP		(100000)
#define TZCPU_INITCFG_TRIP_1_COOLER		"cpu00"

/*
 * OSK: TRIP_2 (cpu03, big-cluster A75 cap) 85C -> 92C.
 * Keeps an 8C gap below the relaxed TRIP_1 (100C). The A75 cluster
 * can sustain higher temperatures before needing an independent cap
 * separate from the full-SoC TRIP_1 cooler.
 */
#define TZCPU_INITCFG_TRIP_2_TEMP		(92000)
#define TZCPU_INITCFG_TRIP_2_COOLER		"cpu03"

/*
 * OSK: TRIP_3 (cpu_adaptive_0, ATM enable) 65C -> 72C.
 * This is the EARLIEST throttle point — it enables the Adaptive
 * Thermal Manager (see clatm_initcfg.h CFG_0), which then regulates
 * toward CLATM_INIT_CFG_0_TARGET_TJ. Raising the enable point from
 * 65C to 72C gives sustained workloads (games, benchmarks, charging)
 * ~7C more headroom before ATM starts trimming CPU/GPU power, while
 * still engaging well below TRIP_2 (92C).
 */
#define TZCPU_INITCFG_TRIP_3_TEMP		(72000)
#define TZCPU_INITCFG_TRIP_3_COOLER		"cpu_adaptive_0"

#define TZCPU_INITCFG_TRIP_4_TEMP		(63000)
#define TZCPU_INITCFG_TRIP_4_COOLER		""

#define TZCPU_INITCFG_TRIP_5_TEMP		(60000)
#define TZCPU_INITCFG_TRIP_5_COOLER		""

#define TZCPU_INITCFG_TRIP_6_TEMP		(55000)
#define TZCPU_INITCFG_TRIP_6_COOLER		""

#define TZCPU_INITCFG_TRIP_7_TEMP		(50000)
#define TZCPU_INITCFG_TRIP_7_COOLER		""

#define TZCPU_INITCFG_TRIP_8_TEMP		(45000)
#define TZCPU_INITCFG_TRIP_8_COOLER		""

#define TZCPU_INITCFG_TRIP_9_TEMP		(40000)
#define TZCPU_INITCFG_TRIP_9_COOLER		""


#endif	/* __TZCPU_INITCFG_H__ */


