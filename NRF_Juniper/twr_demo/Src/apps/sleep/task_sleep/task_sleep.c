/**
 * @file      task_sleep.c
 * @brief     Inactivity-triggered low-power sleep with motion-only wake-up.
 *
 *            Two phases, both using app.imuTask as the task handle (they never
 *            run at the same time - Phase 1 only while TAG is ranging, Phase 2
 *            only after TAG has been torn down):
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
 *              what gives the "auto re-arm after wake" behaviour for free.
 *
 *            Phase 2 - SLEEP (ImuTask, sleep_helper()):
 *              Entered via Ev_Sleep_Task, which runs through main.c's normal
 *              teardown path first (tag_terminate() etc., already resets the
 *              DW3000 into its low-current state). Arms the LIS2DW12's INT1
 *              wake-up-on-motion interrupt and blocks indefinitely - motion is
 *              the only way out, there is no timeout wake in this phase. On
 *              wake it pulses HOST_WAKEUP_Pin, reports over UART, and resumes
 *              TAG ranging by re-setting Ev_Tag_Task.
 *
 *            Both phases explicitly flush the LIS2DW12's wake-up interrupt
 *            (it is latched until WAKE_UP_SRC is read) before relying on it,
 *            otherwise a stale/latched condition from setup can look like an
 *            immediate false wake.
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
#include "usb_uart_tx.h"

/* How often Phase 1 polls the accelerometer for motion while TAG is ranging. */
#define WATCH_POLL_MS   1000

static stmdev_ctx_t gImuCtx;
static uint8_t      gImuConfigured;
static uint8_t      gImuFound;
static uint8_t      gInt1GpioteReady;

/* @brief Phase 2 only: LIS2DW12 INT1 GPIOTE ISR - motion detected while asleep. */
static void sleep_int1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    UNUSED(pin);
    UNUSED(action);

    if (app.imuTask.Handle)
    {
        osSignalSet(app.imuTask.Handle, app.imuTask.Signal);
    }
}

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

    if (!gInt1GpioteReady)
    {
        lis2dw12_int1_gpiote_init(sleep_int1_handler); /* registered once; only Phase 2 arms/uses it */
        gInt1GpioteReady = 1;
    }
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

/* @brief Phase 2: ImuTask - deep sleep, motion is the only wake source. */
static void ImuTask(void const *argument)
{
    UNUSED(argument);

    osMutexDef(imuMutex);
    app.imuTask.MutexId = osMutexCreate(osMutex(imuMutex));

    imu_configure_wakeup_on_motion();

    if (!gImuFound)
    {
        /* No accelerometer -> no way to ever wake from real sleep. Fail safe:
         * don't strand the device asleep, resume ranging immediately instead. */
        osMutexRelease(app.imuTask.MutexId);
        xEventGroupSetBits(app.xStartTaskEvent, Ev_Tag_Task);

        for (;;)
        {
            osSignalWait(app.imuTask.Signal, osWaitForever);
        }
    }

    imu_poll_motion(); /* flush any stale latch before arming - this is what
                         * fixed the immediate-false-wake bug seen on hardware */
    lis2dw12_int1_enable();

    osMutexRelease(app.imuTask.MutexId);

    /* Blocks here; FreeRTOS tickless idle sleeps the MCU while waiting. */
    osSignalWait(app.imuTask.Signal, osWaitForever);

    osMutexWait(app.imuTask.MutexId, 0);

    lis2dw12_int1_disable();

    host_wakeup_pulse();
    {
        char *wakeMsg = "wake:motion\r\n";
        port_tx_msg((uint8_t *)wakeMsg, (int)strlen(wakeMsg));
    }

    /* Resume TAG ranging. Via the auto-rearm hook in tag_helper(), this
     * automatically restarts Phase 1 watching too. */
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Tag_Task);

    osMutexRelease(app.imuTask.MutexId);

    for (;;)
    {
        osSignalWait(app.imuTask.Signal, osWaitForever);
    }
}

/* @brief Terminate whichever phase's task is currently using app.imuTask, if any.
 *        Safe to call even if neither phase was ever entered.
 * */
void sleep_terminate(void)
{
    if (gImuFound)
    {
        lis2dw12_int1_disable();
    }

    TERMINATE_STD_TASK(app.imuTask);
}

/* @brief Start Phase 2 (deep sleep). Called from main.c on Ev_Sleep_Task,
 *        after the normal mode-switch teardown has already torn down TAG
 *        ranging and reset the DW3000.
 * */
void sleep_helper(void const *argument)
{
    UNUSED(argument);

    osThreadDef(imuTask, ImuTask, PRIO_ImuTask, 0, 512);
    app.imuTask.Handle = osThreadCreate(osThread(imuTask), NULL);

    if (app.imuTask.Handle == NULL)
    {
        error_handler(1, _ERR_Create_Task_Bad);
    }
}
