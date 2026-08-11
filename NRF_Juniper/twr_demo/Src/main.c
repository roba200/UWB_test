/*! ----------------------------------------------------------------------------
 * @file    main.c
 * @brief   This is the a variant of implementation of the Slotted TWR PDoA Node 
 *          on Nordic nRF52840 platform with FreeRTOS
 *
 * @author  Decawave Applications
 *
 * @attention Copyright 2017-2019 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */
/* Includes ------------------------------------------------------------------*/

#include "port.h"
#include "deca_device_api.h"
#include "deca_dbg.h"
#include "deca_probe_interface.h"
#include "crc16.h"

#include "app.h"
#include "lis2dw12.h"

#include "nrf_drv_wdt.h"
#include "nrf_power.h"

#include <config.h>
#include <port.h>

#include <node.h>
#include <tag_list.h>
#include <task_node.h>
#include <task_usb2spi.h>
#include <task_tcfm.h>
#include <task_tcwm.h>
#include <task_flush.h>
#include <task_tag.h>

#define USB_DRV_UPDATE_MS   200

/* Private variables ---------------------------------------------------------*/

osThreadId        defaultTaskHandle;
gDiagPrintFStr_t  gDiagPrintFStr;
app_t             app;    /**< All global variables are in the "app" structure */

/* Set below, before anything else touches RESETREAS; consumed once by
 * tag_helper() on a motion-wake boot. See app.h. */
volatile uint8_t  g_systemoff_wakeup_pending;

//-----------------------------------------------------------------------------
//NRF port
uint32_t gRTC_SF_PERIOD = 3276;


extern nrf_drv_wdt_channel_id m_channel_id;
extern int gRangingStart;

extern void trilat_helper(void const *argument);
extern void trilat_terminate_tasks(void);

void StartDefaultTask(void const * argument);
void CtrlTask(void const * arg);
void deca_usb_init(void);

extern void trilat_terminate();
extern void listener_terminate();
extern void listener_helper();

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    int devID = 0, delayCnt = 0, status = 0;
    ret_code_t err_code;

    /* Read + clear RESETREAS before anything else can disturb it - this is
     * how we know whether this boot was a motion-triggered System OFF wake
     * (OFF bit set by the GPIO-SENSE DETECT signal) versus a normal
     * power-on/reset. Consumed once, later, by tag_helper(). */
    {
        uint32_t resetreas = NRF_POWER->RESETREAS;
        g_systemoff_wakeup_pending = ((resetreas & POWER_RESETREAS_OFF_Msk) != 0U) ? 1U : 0U;
        NRF_POWER->RESETREAS = resetreas;
    }

    gRangingStart = 0;

    init_crc16();

    bsp_board_init(BSP_INIT_LEDS|BSP_INIT_BUTTONS);
    uart_host_wake_gpio_init();
    peripherals_init();
    port_init_dw_chip();
    dw_irq_init();

    memset(&app,0,sizeof(app));
    memset(&gDiagPrintFStr, 0, sizeof(gDiagPrintFStr));

    load_bssConfig();                 /**< load the RAM Configuration parameters from NVM block */
    app.pConfig = get_pbssConfig();
    if (app.pConfig->s.uartEn)
    {
        deca_uart_init();
    }

    app.xStartTaskEvent = xEventGroupCreate(); /**< xStartTaskEvent indicates which tasks to be started */

    for(int i=0; i<6; i++)
    {
        bsp_board_led_invert(BSP_BOARD_LED_0);
        bsp_board_led_invert(BSP_BOARD_LED_1);
        nrf_delay_ms(250);
    }

#ifdef ENABLE_USB_PRINT
    deca_usb_init();
#endif

    nrf_delay_ms(1000);        /**< small pause to startup */

  reset_DW3000(); //this will reset DW device

  {
      /* Apply the magic-guarded default the first time this firmware runs
       * on a given device (or after a flash erase) - the field didn't exist
       * in older NVM images, so a blank/garbage value must not be trusted. */
      if (app.pConfig->s.imu_cfg_magic != IMU_CFG_MAGIC)
      {
          app.pConfig->s.imu_cfg_magic          = IMU_CFG_MAGIC;
          app.pConfig->s.imu_inactivity_seconds = DEFAULT_IMU_INACTIVITY_SEC;
      }

      if (lis2dw12_init(E_SPI_INTERFACE, LIS2DW12_MODE_MOTION_ONLY,
                        app.pConfig->s.imu_inactivity_seconds) == 0)
      {
          lis2dw12_event_irq_enable(imu_event_callback);
      }
      else
      {
          diag_printf("IMU init failed; ranging will proceed without motion/sleep.\r\n");
      }
  }

  if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf))
  {
    error_handler(1, _ERR_INIT);
  }
  /* This initialization is added to avoid crashing the board when calling APIs that writes inside local data
  like setxtaltrim */
  if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS)
  {
    error_handler(1, _ERR_INIT);
  }

  /* Cache the EUI here, the one point guaranteed to have the DW3000 awake
   * and responsive - reading it later (e.g. from the EUI command) can hit
   * the chip mid-sleep (dwt_entersleep(), used by the existing TagRxTask
   * fault-handling and by the IMU sleep path) and silently read back all
   * zero bytes over SPI. */
  dwt_geteui(app.eui64);

    
  /* initialize inter-task communication mail queue for Node :
   *
   * The RxTask need to send the rxPckt to the CalcTask.
   *
   * TODO: rxPcktPool_q_id should be a part of NodeInfo, but
   * FreeRTOS cannot free resources efficiently on task deletion.
   *
   * Current code has an implementation where NodeInfo is statically defined
   * and rxPcktPool_q is a part of FreeRtos Heap.
   *
   * Note, the debug accumulator & diagnostics readings are a part of
   * mail queue. Every rx_mail_t has a size of ~6kB.
   *
   * */
    osMailQDef(rxPcktPool_q, RX_MAIL_QUEUE_SIZE, rx_mail_t);
    app.rxPcktPool_q_id = osMailCreate(osMailQ(rxPcktPool_q), NULL);

    if(!app.rxPcktPool_q_id)
    {
        error_handler(1, _ERR_Cannot_Alloc_Mail);
    }

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    /* Note. The DefaultTask is responsible for starting & stopping of TOP Level applications. */
  osThreadDef(defaultTask, StartDefaultTask, PRIO_StartDefaultTask, 0, configMINIMAL_STACK_SIZE*2);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* FlushTask is always working and flushing the output buffer to uart/usb */
  osThreadDef(flushTask, FlushTask, PRIO_FlushTask, 0, configMINIMAL_STACK_SIZE);
  app.flushTask.Handle = osThreadCreate(osThread(flushTask), NULL);

  /* ctrlTask is always working serving rx from uart/usb */
  //2K for CTRL task: it needs a lot of memory: it uses mallocs(512), sscanf(212bytes)
  osThreadDef(ctrlTask, CtrlTask, PRIO_CtrlTask, 0, 512);
  app.ctrlTask.Handle = osThreadCreate(osThread(ctrlTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  
  if( !defaultTaskHandle | !app.flushTask.Handle | !app.ctrlTask.Handle )
  {
      error_handler(1, _ERR_Create_Task_Bad);
  }

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  while (1)
  {
  }
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
    const EventBits_t bitsWaitForAny = (Ev_Node_Task | Ev_Tag_Task | Ev_Trilat_N_Task |
                                        Ev_Usb2spi_Task | Ev_Tcwm_Task | Ev_Tcfm_Task | Ev_Listener_Task |
                                        Ev_Stop_All);

    EventBits_t    uxBits;

    uxBits = check_the_user_button();

    xEventGroupSetBits(app.xStartTaskEvent, uxBits);

    /* Infinite loop */
    while(1)
    {
        uxBits = xEventGroupWaitBits(app.xStartTaskEvent,   bitsWaitForAny,
                                                            pdTRUE, pdFALSE,
                                                            USB_DRV_UPDATE_MS/portTICK_PERIOD_MS );

        nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh

      uxBits &= bitsWaitForAny;

      if(uxBits)
      {
          app.lastErrorCode = _NO_ERR;

         /*   need to switch off DW chip's RX and IRQ before killing tasks */
          if(app.mode != mIDLE)
          {
              disable_dw3000_irq();
              reset_DW3000();
              app_apptimer_stop();                //cancel application slow timer
              dwt_setcallbacks(NULL, NULL, NULL, NULL, NULL, NULL, NULL);//DW_IRQ is disabled: safe to cancel all user call-backs
          }

          /* Event to start/stop task received */
          /* 1. free the resources: kill all user threads and timers */
          tag_terminate();
          node_terminate();
          usb2spi_terminate();
          tcfm_terminate();
          tcwm_terminate();
          trilat_terminate();
          listener_terminate();

          FlushTask_reset();

          app.lastErrorCode = _NO_ERR;

          //incoming Events are slow, and usually User-events, i.e. from a slow I/O,
          //however the Ev_Stop_All can be generated internally and may OR with other event,
          //because they can be received asynchronously.
          //Ev_Stop_All event should be tracked separately.
          app.mode = mIDLE;
          uxBits &= ~Ev_Stop_All;
      }

        nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh

        osThreadYield(); //force switch of context 

        taskENTER_CRITICAL();

      /* 2. Start appropriate RTOS top-level application or run a usb_vbus_driver() if a dummy loop */
      switch (uxBits)
      {
      case Ev_Listener_Task:
          app.mode = mLISTENER;
          listener_helper(NULL); /* call Listener helper function which will setup sub-tasks for Listener process */
          break;

#if (PDOA_TAG == 1)
      /* PDoA */
      case Ev_Tag_Task:
          app.mode = mPTAG;
          tag_helper(NULL); /* call Tag helper function which will setup sub-tasks for Tag process */
          break;
#endif

#if (PDOA_NODE == 1)
      case Ev_Node_Task:
          app.mode = mPNODE;
          node_helper(NULL); /* call Node helper function which will setup sub-tasks for Node process */
          break;
#endif

#if (CFG_LE_TRILAT == 1)
      case Ev_Trilat_N_Task:
          app.mode = mTRILAT_N;
          trilat_helper(NULL); /* call Trilat helper function which will setup sub-tasks for Node process & Trilat */
          break;
#endif

      /* Service apps */
      case Ev_Usb2spi_Task:
          /* Setup a Usb2Spi task : 8K of stack is required to this task */
          app.mode = mUSB2SPI;
          usb2spi_helper(NULL);
          break;

      case Ev_Tcfm_Task:
          /* Setup a TCFM task */
          app.mode = mTCFM;
          tcfm_helper(NULL);    /* call tcfm helper function which will setup sub-tasks for Power test */
          break;

      case Ev_Tcwm_Task:
          /* Setup a TCWM task */
          app.mode = mTCWM;
          tcwm_helper(NULL);    /* call tcwm helper function which will setup sub-tasks for Power test */
          break;

      case Ev_Stop_All:
          app.mode = mIDLE;
        break;

        default:
            //usb_vbus_driver();    
            /**< connection / disconnection of USB interface :
            on connection activate flush_task & usb_rx_process  */
            nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh
            break;
        }
        taskEXIT_CRITICAL();    //ready to switch to a created task

        osThreadYield();
    }
}

/** @} */