/**
 * @file      task_sleep.c
 * @brief     Inactivity-triggered real System OFF sleep with motion-only wake.
 *
 *            Two phases, using app.imuTask as the Phase 1 task handle (Phase 2
 *            has no task at all - see below):
 *
 *            Phase 1 - WATCH (WatchTask, sleep_watch_start()/stop()):
 *              Runs alongside normal TAG ranging whenever
 *              app.pConfig->s.inactivityTimeout_sec > 0. Polls the LIS2DW12 for
 *              motion once a second; any motion resets a software "seconds
 *              since last motion" counter back to 0. Ranging is left running
 *              untouched. Once that counter reaches the configured threshold
 *              (sustained inactivity), it fires Ev_Sleep_Task to hand off to
 *              Phase 2. Started automatically every time TAG (re)starts - see
 *              the hook at the end of tag_helper() in task_tag.c - which is
 *              what gives the "auto re-arm after wake" behaviour for free,
 *              including after a motion wake (see Phase 2).
 *
 *            Phase 2 - SLEEP (sleep_helper(), no task, no FreeRTOS involved):
 *              Entered via Ev_Sleep_Task, which runs through main.c's normal
 *              teardown path first (tag_terminate() etc., already resets the
 *              DW3000 into its low-current state). Arms the LIS2DW12's INT1
 *              pin as a GPIO SENSE wake source, closes the UART
 *              (deca_uart_close()), and calls nrf_power_system_off() - this
 *              halts the CPU and never returns. The chip is now genuinely
 *              powered down (~1-3 uA); the only way out is motion asserting
 *              INT1, which causes a full chip RESET (not a resume - all RAM
 *              state is lost). main.c detects this wake reason via
 *              NRF_POWER->RESETREAS and pulses HOST_WAKEUP_Pin as the very
 *              first thing on boot, before the normal boot sequence (which
 *              lands back in TAG mode and re-arms Phase 1) continues.
 *
 *            Both phases explicitly flush the LIS2DW12's wake-up interrupt
 *            (it is latched until WAKE_UP_SRC is read) before relying on it -
 *            for Phase 2 this matters even more than it did for the old
 *            GPIOTE-interrupt design, since GPIO SENSE wakes immediately if
 *            the pin is already at the sensed level when armed.
 *
 * @attention Copyright 2019-2020 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 */

#include <string.h>
#include "port.h"
#include "app.h"
#include "task_sleep.h"
#include "lis2dw12_platform.h"
#include "lis2dw12_reg.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_power.h"

/* How often Phase 1 polls the accelerometer for motion while TAG is ranging. */
#define WATCH_POLL_MS   1000

static stmdev_ctx_t gImuCtx;
static uint8_t      gImuConfigured;
static uint8_t      gImuFound;

/* @brief One-time LIS2DW12 bring-up + wake-up-on-motion configuration, shared
 *        by both phases. Low-power continuous mode @ 12.5 Hz ODR, routed to
 *        INT1. wkup_dur is a couple of samples (not 0) so a single noisy
 *        sample can't trigger a false wake.
 * */
static void imu_configure_wakeup_on_motion(void)
{
    uint8_t whoami = 0;
    uint8_t rst;
    lis2dw12_ctrl4_int1_pad_ctrl_t int1_route;

    if (gImuConfigured)
    {
        return;
    }
    gImuConfigured = 1;

    lis2dw12_platform_spi_init(&gImuCtx);
    nrf_delay_ms(20); /* LIS2DW12 boot time */

    lis2dw12_device_id_get(&gImuCtx, &whoami);
    if (whoami != LIS2DW12_ID)
    {
        gImuFound = 0;
        error_handler(0, _ERR_SPI); /* non-fatal: logged, sleep feature stays disabled */
        return;
    }
    gImuFound = 1;

    lis2dw12_reset_set(&gImuCtx, PROPERTY_ENABLE);
    do
    {
        lis2dw12_reset_get(&gImuCtx, &rst);
    } while (rst);

    lis2dw12_full_scale_set(&gImuCtx, LIS2DW12_2g);
    lis2dw12_power_mode_set(&gImuCtx, LIS2DW12_CONT_LOW_PWR_12bit);
    lis2dw12_data_rate_set(&gImuCtx, LIS2DW12_XL_ODR_12Hz5);
    lis2dw12_filter_path_set(&gImuCtx, LIS2DW12_HIGH_PASS_ON_OUT);
    lis2dw12_wkup_dur_set(&gImuCtx, 2);        /* debounce: ~2 consecutive samples, tune as needed */
    lis2dw12_wkup_threshold_set(&gImuCtx, 1);  /* 1 LSb = FS_XL/2^6 (~31 mg here, most sensitive non-zero value); tune sensitivity as needed */

    lis2dw12_pin_int1_route_get(&gImuCtx, &int1_route);
    int1_route.int1_wu = PROPERTY_ENABLE;
    lis2dw12_pin_int1_route_set(&gImuCtx, &int1_route);
}

/* @brief Reads (and thereby clears, this chip's interrupt is latched until
 *        read) the wake-up source register. Returns non-zero if motion was
 *        detected since the last call.
 * */
static uint8_t imu_poll_motion(void)
{
    lis2dw12_all_sources_t all_source;

    memset(&all_source, 0, sizeof(all_source));
    lis2dw12_all_sources_get(&gImuCtx, &all_source);

    return all_source.wake_up_src.wu_ia ? 1 : 0;
}

/* @brief Phase 1: WatchTask - polls for motion once a second while TAG keeps
 *        ranging; hands off to Phase 2 only after sustained inactivity.
 * */
static void WatchTask(void const *argument)
{
    uint32_t stillSeconds = 0;
    uint32_t thresholdSec;

    UNUSED(argument);

    osMutexDef(watchMutex);
    app.imuTask.MutexId = osMutexCreate(osMutex(watchMutex));

    thresholdSec = app.pConfig->s.inactivityTimeout_sec;

    imu_configure_wakeup_on_motion();

    if (gImuFound)
    {
        imu_poll_motion(); /* flush any stale latch before starting to watch */
    }

    osMutexRelease(app.imuTask.MutexId);

    while (gImuFound && thresholdSec > 0)
    {
        osDelay(WATCH_POLL_MS);

        osMutexWait(app.imuTask.MutexId, 0);

        if (imu_poll_motion())
        {
            stillSeconds = 0;
        }
        else
        {
            stillSeconds += (WATCH_POLL_MS / 1000);
        }

        osMutexRelease(app.imuTask.MutexId);

        if (stillSeconds >= thresholdSec)
        {
            /* Sustained inactivity: hand off to Phase 2 (deep sleep). This
             * tears WatchTask (app.imuTask) down via tag_terminate()'s
             * existing TERMINATE_STD_TASK(app.imuTask) call. */
            xEventGroupSetBits(app.xStartTaskEvent, Ev_Sleep_Task);
            break;
        }
    }

    /* Either disabled (thresholdSec==0), no accelerometer, or already handed
     * off above - park here, safely killable, until externally terminated. */
    for (;;)
    {
        osSignalWait(app.imuTask.Signal, osWaitForever);
    }
}

/* @brief (Re)start Phase 1 watching using the current persisted threshold.
 *        Safe to call whether or not a watch is already running (e.g. the
 *        host just changed the SLEEP value while TAG is ranging).
 * */
void sleep_watch_start(void)
{
    TERMINATE_STD_TASK(app.imuTask);

    osThreadDef(watchTask, WatchTask, PRIO_ImuTask, 0, 512);
    app.imuTask.Handle = osThreadCreate(osThread(watchTask), NULL);

    if (app.imuTask.Handle == NULL)
    {
        error_handler(1, _ERR_Create_Task_Bad);
    }
}

void sleep_watch_stop(void)
{
    TERMINATE_STD_TASK(app.imuTask);
}

/* @brief Terminate Phase 1's WatchTask, if any is currently running (e.g. some
 *        other command interrupted it). Phase 2 below has no task/interrupt
 *        state to clean up - it either powers the chip off (never returns)
 *        or fails safe back to Ev_Tag_Task, so this only ever needs to reap
 *        app.imuTask, not touch the accelerometer.
 * */
void sleep_terminate(void)
{
    TERMINATE_STD_TASK(app.imuTask);
}

/* @brief Phase 2: real System OFF sleep, motion is the only wake source.
 *        Called from main.c on Ev_Sleep_Task, after the normal mode-switch
 *        teardown has already torn down TAG ranging and reset the DW3000.
 *        No task is created - nrf_power_system_off() halts the CPU and never
 *        returns; waking happens via a full chip reset (see main.c, which
 *        detects the wake reason via RESETREAS and pulses HOST_WAKEUP_Pin
 *        immediately, then reboots normally back into TAG - which re-arms
 *        Phase 1 automatically via the hook in tag_helper()).
 * */
void sleep_helper(void const *argument)
{
    UNUSED(argument);

    imu_configure_wakeup_on_motion();

    if (!gImuFound)
    {
        /* No accelerometer -> no way to ever wake from real sleep. Fail safe:
         * don't power off with no way to wake, resume ranging immediately. */
        xEventGroupSetBits(app.xStartTaskEvent, Ev_Tag_Task);
        return;
    }

    imu_poll_motion(); /* flush any stale latch before arming - with GPIO
                         * SENSE an already-asserted pin wakes immediately,
                         * reproducing the false-wake bug seen earlier if
                         * skipped. */

    nrf_gpio_cfg_sense_input(LIS2DW12_INT1_Pin, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);

    deca_uart_close(); /* no UART while asleep */

    nrf_power_system_off(); /* never returns */
}
