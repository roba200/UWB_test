/**
 * @file      task_sleep.h
 * @brief     Host-commanded low-power sleep with motion/timeout wake-up.
 *
 * @attention Copyright 2019-2020 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 */

#ifndef __TASK_SLEEP__H__
#define __TASK_SLEEP__H__ 1

#ifdef __cplusplus
extern "C" {
#endif

void sleep_helper(void const *argument);
void sleep_terminate(void);

/* Phase 1 - WATCH: runs alongside normal TAG ranging. Arms the accelerometer's
 * motion IRQ, which resets an inactivity countdown on every motion event; only
 * once the countdown truly expires (sustained inactivity) does it trigger the
 * mode switch into Phase 2 (deep sleep, sleep_helper()/ImuTask above). */
void sleep_watch_start(void);
void sleep_watch_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_SLEEP__H__ */
