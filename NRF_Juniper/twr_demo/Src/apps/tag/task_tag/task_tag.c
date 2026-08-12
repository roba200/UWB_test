/*
 * @file      task_tag.c
 * @brief     Decawave Application Layer
 *            RTOS tag implementation
 *
 *
 * @author    Decawave
 *
 * @attention Copyright 2017-2019 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */
#include "task_tag.h"
#include "util.h"
#include "app.h"
#include "port.h"
#include "tag.h"
#include "lis2dw12.h"
#include "nrf_gpio.h"
#include "nrf_power.h"
#include "nrf_delay.h"
//-----------------------------------------------------------------------------

#define BLINK_PERIOD_MS            (500)    /* range init phase - Blink send period, ms */

/* ---- IMU-driven sleep/wake ------------------------------------------------
 *
 * Motion/inactivity are detected entirely in the LIS2DW12's hardware (see
 * lis2dw12.c) - this file just reacts to the two events it can raise:
 *   - LIS2DW12_EVT_MOTION:     resume ranging, tell the host we're awake.
 *   - LIS2DW12_EVT_STATIONARY: the IMU's own inactivity timer elapsed -
 *                               pause ranging, sleep the DW3000, and put the
 *                               whole board into System OFF, arming the
 *                               LIS2DW12's INT1 (motion) as the wake source.
 *
 * imu_event_callback() runs in ISR context and only ever sets a pending-event
 * bitmask + wakes TagPollTask; all the actual work happens in
 * tag_handle_imu_events(), called from TagPollTask's own loop.
 */
#define IMU_EVT_SLEEP               (0x02U)
#define IMU_EVT_WAKE                (0x04U)

static volatile uint8_t g_imu_event_pending      = 0;
static volatile uint8_t g_uwb_ranging_paused     = 0;
static bool             g_imu_host_awake_reported = true;
static bool             g_ignore_next_motion_event = false;

static void signal_poll_task_from_imu(void)
{
    if(app.pollTask.Handle)
    {
        (void)osSignalSet(app.pollTask.Handle, app.pollTask.Signal);
    }
}

static uint8_t take_imu_events(void)
{
    uint8_t events;

    taskENTER_CRITICAL();
    events = g_imu_event_pending;
    g_imu_event_pending = 0;
    taskEXIT_CRITICAL();

    return events;
}

static void report_imu_wake_if_needed(void)
{
    if(!g_imu_host_awake_reported)
    {
        send_to_pc_imu_wake();
        g_imu_host_awake_reported = true;
    }
}

static bool tag_imu_wake_asserted(void)
{
    if(lis2dw12_motion_pending())
    {
        return true;
    }

    return nrf_gpio_pin_read(LIS2DW12_INT1_Pin) ? true : false;
}

static void tag_abort_sleep_for_motion(tag_info_t *p)
{
    if(p)
    {
        p->stationary_imu = 0;
        p->stationary = 0;
    }

    g_uwb_ranging_paused = 0;
    NVIC_ClearPendingIRQ(RTC0_IRQn);
    NVIC_EnableIRQ(RTC0_IRQn);
    lis2dw12_event_irq_enable(imu_event_callback);
}

/* @brief Consume any pending IMU event(s) and act on them. Called from
 *        TagPollTask. Returns true if this call just put the board into
 *        System OFF (caller should not touch anything else afterwards).
 * */
static bool tag_handle_imu_events(tag_info_t *p)
{
    uint8_t  events = take_imu_events();

    if(!p || events == 0)
    {
        return false;
    }

    if(events & IMU_EVT_WAKE)
    {
        report_imu_wake_if_needed();
        g_uwb_ranging_paused = 0;
        NVIC_ClearPendingIRQ(RTC0_IRQn);
        NVIC_EnableIRQ(RTC0_IRQn);
    }

#if (TAG_SLEEP_MODE_ENABLE == 1)
    if(events & IMU_EVT_SLEEP)
    {
        /* Do not enter System OFF if an INT1 wake source is already pending.
         * Otherwise the GPIO sense input wakes the nRF52 immediately after the
         * host receives IMU_SLEEP. */
        lis2dw12_event_irq_disable();
        lis2dw12_clear_events();
        if(tag_imu_wake_asserted())
        {
            tag_abort_sleep_for_motion(p);
            return false;
        }

        send_to_pc_imu_sleep();
        g_imu_host_awake_reported = false;
        g_ignore_next_motion_event = false;
        (void)flush_report_buf();
        nrf_delay_ms(100);

        NVIC_DisableIRQ(RTC0_IRQn);
        dwt_forcetrxoff();
#if (DW_TAG_NOT_SLEEPING == 0)
        taskENTER_CRITICAL();
        app.DwEnterSleep = DW_IS_SLEEPING_RX;
        dwt_entersleep(DWT_DW_IDLE_RC);
        app.DwSpiReady = DW_SPI_SLEEPING;
        taskEXIT_CRITICAL();
#endif
        g_uwb_ranging_paused = 1;
        lis2dw12_clear_events();
        if(tag_imu_wake_asserted())
        {
            tag_abort_sleep_for_motion(p);
            report_imu_wake_if_needed();
            return false;
        }

        nrf_gpio_cfg_sense_input(LIS2DW12_INT1_Pin,
                                 NRF_GPIO_PIN_NOPULL,
                                 NRF_GPIO_PIN_SENSE_HIGH);

        /* Only now, right as we're actually committing to it - not on either
         * of the aborted-sleep paths above, where the board stays awake.
         * Inside the TAG_SLEEP_MODE_ENABLE guard already, so the LEDs are
         * only ever touched here when sleep mode is actually active.
         *
         * Confirmed on hardware: this board's bsp_board_leds_on()/_off()
         * light the LEDs backwards from their names (see main.c) - calling
         * "_on()" here is what actually turns them off. */
        bsp_board_leds_on();

        nrf_power_system_off();
        return true;
    }
#endif /* TAG_SLEEP_MODE_ENABLE */

    return false;
}

void imu_event_callback(lis2dw12_event_t evt)
{
    tag_info_t *pTwrInfo = getTagInfoPtr();
    if (!pTwrInfo) return;

    switch (evt)
    {
    case LIS2DW12_EVT_STATIONARY:
        /* IMU built-in timer has elapsed - this interrupt IS the inactivity timeout. */
        if(g_uwb_ranging_paused) break;
        pTwrInfo->stationary_imu = 1;
        pTwrInfo->stationary = 1;
        g_imu_event_pending &= (uint8_t)~IMU_EVT_WAKE;
        g_imu_event_pending |= IMU_EVT_SLEEP;
        signal_poll_task_from_imu();
        break;

    case LIS2DW12_EVT_MOTION:
        if (g_ignore_next_motion_event)
        {
            g_ignore_next_motion_event = false;
            break;
        }
        pTwrInfo->stationary_imu = 0;
        pTwrInfo->stationary = 0;
        g_imu_event_pending &= (uint8_t)~IMU_EVT_SLEEP;
        g_imu_event_pending |= IMU_EVT_WAKE;
        signal_poll_task_from_imu();
        break;

    default:
        break;
    }
}

//-----------------------------------------------------------------------------

/*
 * @brief
 *     The thread is initiating the transmission of the blink
 *     on reception of app.blinkTask.Signal
 *
 * */
static void
BlinkTask(void const * arg)
{
    tag_info_t     *p;

    do{
        osDelay(100);
    }while(!(p = getTagInfoPtr()));    //wait for initialisation of psTagInfo


    do {
        osMutexRelease(app.blinkTask.MutexId);

        osSignalWait(app.blinkTask.Signal, osWaitForever);

        osMutexWait(app.blinkTask.MutexId, 0);    //we do not want the task can be deleted in the middle of operation

        if(app.DwSpiReady == DW_SPI_SLEEPING)
        {
            taskENTER_CRITICAL();
            tag_wakeup_dw3000_blink_poll(p);
            taskEXIT_CRITICAL();
        }
        else
        {
            taskENTER_CRITICAL();
            dwt_restoreconfig(); //restore configuration which has not been saved in AON
            app.DwEnterSleep = DW_NOT_SLEEPING;
            taskEXIT_CRITICAL();

            initiator_send_blink(p);

        }

    }while(1);

    UNUSED(arg);
}


/*
 * @brief
 *     The thread is initiating the TWR sequence
 *  on reception of .signal.twrTxPoll
 *
 * */
static void
TagPollTask(void const * arg)
{
    tag_info_t     *p = getTagInfoPtr();

    do{
        osDelay(100);
    }while(!(p = getTagInfoPtr()));    //wait for initialisation of pTwrInfo


    do {
        osMutexRelease(app.pollTask.MutexId);

        osSignalWait(app.pollTask.Signal, osWaitForever);

        osMutexWait(app.pollTask.MutexId, 0);

        if(tag_handle_imu_events(p))
        {
            osThreadYield();
            continue;
        }

        if(app.DwSpiReady == DW_SPI_SLEEPING)
        {
            taskENTER_CRITICAL();
            tag_wakeup_dw3000_blink_poll(p);
            taskEXIT_CRITICAL();

        }
        else
        {
            taskENTER_CRITICAL();
            dwt_restoreconfig(); //restore configuration which has not been saved in AON
            app.DwEnterSleep = DW_NOT_SLEEPING;
            taskEXIT_CRITICAL();

            if (p->faultyRangesCnt < app.pConfig->s.faultyRanges)
            {
                initiator_send_poll(p);
            }
            else
            {
                NVIC_DisableIRQ(RTC0_IRQn);
                /* This will restart Tag task completely from discovery phase */
                xEventGroupSetBits(app.xStartTaskEvent, Ev_Tag_Task);
            }
        }

    }while(1);

    UNUSED(arg);
}

/* @brief DW3000 RX : RTOS implementation
 *
 * */
static void
TagRxTask(void const * arg)
{
    error_e     ret;
    uint16_t    head, tail;

    tag_info_t  *ptwrInfo;

    do{
        osDelay(10);
    }while(!(ptwrInfo = getTagInfoPtr()));    //wait for initialisation of pTwrInfo

    int size = sizeof(ptwrInfo->rxPcktBuf.buf) / sizeof(ptwrInfo->rxPcktBuf.buf[0]);

    do {
        osMutexRelease(app.rxTask.MutexId);

        osSignalWait(app.rxTask.Signal, osWaitForever);

        osMutexWait(app.rxTask.MutexId, 0);

        taskENTER_CRITICAL();
        head = ptwrInfo->rxPcktBuf.head;
        tail = ptwrInfo->rxPcktBuf.tail;
        taskEXIT_CRITICAL();

        /* We are using circular buffer + Signal to safely deliver packets from ISR to APP */
        if(CIRC_CNT(head,tail,size) > 0)
        {
            rx_pckt_t_t       *prxPckt    = &ptwrInfo->rxPcktBuf.buf[tail];
            twr_res_ext_t   p;

            ret = twr_initiator_algorithm_rx(prxPckt, ptwrInfo); /**< Run bare twr_initiator_algorithm */

            switch (ret)
            {
            case _NO_Err_Response :
                ptwrInfo->faultyRangesCnt = 0;
                p.addr          = AR2U16(ptwrInfo->env.tagAddr);
                p.node_addr     = AR2U16(ptwrInfo->env.nodeAddr);
                p.rNum          = prxPckt->msg.respExtMsg.resp.rNum;
                p.x_cm          = (int16_t)AR2U16(prxPckt->msg.respExtMsg.resp.x_cm);
                p.y_cm          = (int16_t)AR2U16(prxPckt->msg.respExtMsg.resp.y_cm);
                p.clkOffset_pphm= (int16_t)AR2U16(prxPckt->msg.respExtMsg.resp.clkOffset_pphm); //Crystal Clock offset value reported back from the Node

                {//XTAL trimming will be performed after sending of Final.
                    /* Instead of using a clkOffset_pphm from Response, which calculated by Node based on distances measurements,
                     * the more precise and direct method of adjusting clock offset using
                     * carrier integrator counter value will be used.*/
                    float co_ppm;
                    co_ppm = dwt_readclockoffset();
                    /* save the offset value for future apply after Final message been sent */
                    ptwrInfo->clkOffset_pphm = (int)( co_ppm * (CLOCK_OFFSET_PPM_TO_RATIO * 1e6 * 100));
                }
                break;
            case _ERR_DelayedTX_Late:
            case _ERR_Not_Twr_Frame:
            case _NO_Err_Can_Sleep :
#if (DW_TAG_NOT_SLEEPING == 0)
                app.DwCanSleepInIRQ = DW_CAN_SLEEP_APP;
#endif
                break;
            default:
                break;
            }

#if (DW_TAG_NOT_SLEEPING == 0)
            if((app.DwCanSleepInIRQ == DW_CAN_SLEEP_APP) && (app.DwEnterSleep != DW_IS_SLEEPING_IRQ))
            {
                taskENTER_CRITICAL();
                app.DwEnterSleep = DW_IS_SLEEPING_RX;
                dwt_entersleep(DWT_DW_IDLE_RC);       //manual sleeping
                app.DwSpiReady = DW_SPI_SLEEPING;
                taskEXIT_CRITICAL();
            }
#endif

            taskENTER_CRITICAL();
            tail = (tail + 1) & (size-1);
            ptwrInfo->rxPcktBuf.tail = tail;
            taskEXIT_CRITICAL();

            /* ready to serve next raw reception */

            /* Report previous range/coordinates/offset back to UART/USB */
            if((app.pConfig->s.reportLevel) && (ret == _NO_Err_Response))
            {
               send_to_pc_tag_location(&p); //TODO: tag can report the distance to the PC from the previous location
                                             //TODO: also tag can calculate the location from SSTWR
            }

            UNUSED(p);
        }

        osThreadYield();
    }while(1);

    UNUSED(arg);
}


/* @brief Setup TWR tasks and timers for discovery phase.
 *         - blinking timer
 *         - blinking task
 *          - twr polling task
 *         - rx task
 * Only setup, do not start.
 * */
static void tag_setup_tasks(void)
{

// The RTC Timer is set as a part of tag_process_init()
// The RTC timer will produce Signals to activate either BlinkTask or TagPollTask 
// That will help with sending of first Poll on the correct slot.
// 
    /* Blinking thread for discovery phase of the Anchor until reception of Range Init */
    osThreadDef(blinkTask, BlinkTask, PRIO_BlinkTask, 0, 256);
    osMutexDef(blinkMutex);

    /* This will be the main poll thread for the Tag after discovery completed
     * Do not reduce the stack size for this thread.
     * */
    osThreadDef(pollTask, TagPollTask, PRIO_TagPollTask, 0, 256);
    osMutexDef(pollMutex);

    /* rxThread is passing signal from RX IRQ to an actual two-way ranging algorithm.
     * It awaiting of Rx Signal from RX IRQ ISR and decides what to do next in TWR exchange process.
     * Do not reduce the stack size for this thread.
     * */
    osThreadDef(rxTask, TagRxTask, PRIO_TagRxTask, 0, 384);
    osMutexDef(rxMutex);

    app.blinkTask.Handle    = osThreadCreate(osThread(blinkTask), NULL);
    app.blinkTask.MutexId   = osMutexCreate(osMutex(blinkMutex));


    app.pollTask.Handle     = osThreadCreate(osThread(pollTask), NULL);
    app.pollTask.MutexId    = osMutexCreate(osMutex(pollMutex));

    app.rxTask.Handle       = osThreadCreate(osThread(rxTask), NULL);
    app.rxTask.MutexId      = osMutexCreate(osMutex(rxMutex));

    if( (app.blinkTask.Handle == NULL)   ||\
        (app.pollTask. Handle == NULL)   ||\
        (app.rxTask.   Handle == NULL))
    {
        error_handler(1, _ERR_Create_Task_Bad);
    }
}


//-----------------------------------------------------------------------------

/* @brief
 *      Kill all task and timers related to tag/TWR if any
 *      DW3000's RX and IRQ shall be switched off before task termination,
 *      that IRQ will not produce unexpected Signal
 * */
void tag_terminate(void)
{
     
    NVIC_DisableIRQ(RTC0_IRQn);

    TERMINATE_STD_TASK(app.blinkTask);

    TERMINATE_STD_TASK(app.imuTask);

    TERMINATE_STD_TASK(app.rxTask);

    TERMINATE_STD_TASK(app.pollTask);

    tag_process_terminate();    //de-allocate Tag RAM Resources
}


/* @fn      tag_helper
 * @brief   this is a service function which starts the Tag
 *          top-level  application.
 *    Note: If the dynamic memory allocation is used, then the
 *          tag_process_init() will allocate the memory of sizeof(tag_info_t)
 *          from the <b>caller's</b> task stack, see _malloc_r() !
 * */
void tag_helper(void const *argument)
{
    error_e     tmp;
    bool        report_wake = false;

    port_disable_dw_irq_and_reset(1);

    taskENTER_CRITICAL();    /**< When the app will setup RTOS tasks, then if task has a higher priority,
                                 the kernel will start it immediately, thus we need to stop the scheduler.*/
    set_dw_spi_fast_rate();

    tag_setup_tasks();        /**< "RTOS-based" : setup all RTOS tasks. */

    /* "RTOS-independent" part : initialisation of two-way ranging process */
    tmp = tag_process_init();

    if( tmp != _NO_ERR)
    {
        error_handler(1, tmp);
    }

    if(g_systemoff_wakeup_pending)
    {
        tag_info_t *p = getTagInfoPtr();
        if(p)
        {
            /* The IMU's own INT1 was the wake source - suppress the
             * duplicate LIS2DW12_EVT_MOTION callback it will also raise.
             * This has to happen now, before ranging (and with it, the IMU
             * IRQ re-firing) starts - but the actual host notification is
             * deferred, see below. */
            g_ignore_next_motion_event = true;
            lis2dw12_clear_events();
            g_imu_host_awake_reported = false;
            report_wake = true;
        }
        g_systemoff_wakeup_pending = 0;
    }

    tag_process_start();

    taskEXIT_CRITICAL();    /**< all RTOS tasks can be scheduled */

    if(report_wake)
    {
        /* Deferred until ranging is actually running and the critical
         * section is released - report_imu_wake_if_needed() blocks for
         * ~200ms (the host-wake GPIO pulse's settle delay), and there's no
         * reason to hold up scheduling, or tell the host we're "awake",
         * before that's actually true. */
        report_imu_wake_if_needed();
    }
}


