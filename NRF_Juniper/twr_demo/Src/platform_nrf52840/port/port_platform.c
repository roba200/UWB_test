/*! ----------------------------------------------------------------------------
 * @file    port_platform.c
 * @brief   HW specific definitions and functions for portability
 *
 * @author  Decawave 
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#include "port.h"
#include "deca_device_api.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_wdt.h"
#include "app.h"
#include "sdk_config.h"

/******************************************************************************
 *
 *                              APP global variables
 *
 ******************************************************************************/

extern nrf_drv_wdt_channel_id m_channel_id;

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

int gRangingStart = 0;

static
spi_handle_t spi_handler
=
{
/* below will be configured in the port_init_dw_chip() */
  .spi_inst       = 0,
  .frequency_slow = 0, 
  .frequency_fast = 0,
  .spi_config     = 0,

  .csPin          = DW3000_CS_Pin,
  .lock           = DW_HAL_NODE_UNLOCKED
};


const dw_t dw_chip
=
{
    .irqPin    = DW3000_IRQ_Pin,
    .irqN      = GPIOTE_IRQn,  /* NRF chip has only 1 IRQ for all GPIO */
    .rstPin    = DW3000_RST_Pin,
    .rstIrqN   = GPIOTE_IRQn,  /* NRF chip has only 1 IRQ for all GPIO */
    .wakeUpPin = DW3000_WUP_Pin,
    .pSpi      = &spi_handler,
};


const  dw_t         *pDwChip      = &dw_chip;
static spi_handle_t *pgSpiHandler = dw_chip.pSpi;
static volatile bool spi_xfer_done;

static volatile uint32_t signalResetDone;
uint32_t time32_incr = 0;
uint32_t timer_val = 0;
uint8_t spi_init_stat = 0; // use 1 for slow, use 2 for fast;

int readfromspi_uni(uint16_t headerLength,
                    const uint8_t *headerBuffer,
                    uint16_t readlength,
                    uint8_t *readBuffer,
                    spi_handle_t *pgSpiHandler);

int writetospi_uni(uint16_t headerLength,
                   const uint8_t *headerBuffer,
                   uint32_t bodylength,
                   const uint8_t *bodyBuffer,
                   spi_handle_t *pgSpiHandler);

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context);

static int port_init_device();


/******************************************************************************
 *
 *                              rtc operations
 *
 ******************************************************************************/

void juniper_configure_hal_rtc_callback(void (*cb)(void))
{
    app.HAL_RTCEx_WakeUpTimerEventCb = cb;

}

void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{

    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        if(app.HAL_RTCEx_WakeUpTimerEventCb)
        {
            app.HAL_RTCEx_WakeUpTimerEventCb();
        }
    }
}

/** @brief Initialization of the RTC driver instance
 */
uint32_t rtc_init(void)
{
    uint32_t err_code;
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;

    config.prescaler = RTC_WKUP_PRESCALER;   // WKUP_RESOLUTION_US counter period

    juniper_configure_hal_rtc_callback(NULL);

    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);

    return err_code;
}

/******************************************************************************
 *
 *                              Time section
 *
 ******************************************************************************/

/* @fn    portGetTickCnt
 * @brief wrapper for to read a SysTickTimer, which is incremented with
 *        CLOCKS_PER_SEC frequency.
 *        The resolution of time32_incr is usually 1/1000 sec.
 * */
__INLINE uint32_t
portGetTickCount(void)
{
    return time32_incr;
}

/* @brief     manually configuring of EXTI priority
 * */
void init_dw3000_irq(void)
{
}

void disable_dw3000_irq(void)
{
    /* Unfortunately NRF chip has only 1 IRQ for all GPIO pins */
    nrf_drv_gpiote_in_event_disable(pDwChip->irqPin);
}

void enable_dw3000_irq(void)
{
    nrf_drv_gpiote_in_event_enable(pDwChip->irqPin, true);
}

/* @fn      reset_DW1000
 * @brief   DW_RESET pin on DW1000 has 2 functions
 *          In general it is output, but it also can be used to reset the
 *          digital part of DW1000 by driving this pin low.
 *          Note, the DW_RESET pin should not be driven high externally.
 * */
void reset_DW3000(void)
{
    nrf_gpio_cfg_output(DW3000_RST_Pin);
    nrf_gpio_pin_clear(DW3000_RST_Pin);
    nrf_delay_ms(2);
    nrf_gpio_cfg_input(DW3000_RST_Pin, NRF_GPIO_PIN_NOPULL);
    nrf_delay_ms(2);
}

__attribute__((weak)) void wakeup_device_with_io(void)
{
    port_wakeup_dw3000_fast();
}

/* @fn      port_wakeup_dw3000_fast
 * @brief   waking up of DW3000 using DW_CS pin
 *
 *          the fast wakeup takes ~1ms:
 *          500us to hold the CS  - TODO: this time can be reduced
 *          500us to the crystal to startup
 *          + ~various time 100us...10ms
 * */
error_e port_wakeup_dw3000_fast(void)
{
    spi_handle_t *p = pDwChip->pSpi;
    nrf_gpio_cfg_output(p->csPin);
    nrf_gpio_pin_clear(p->csPin);
    nrf_delay_us(500);
    nrf_gpio_pin_set(p->csPin);

    return _NO_ERR;
}

/* @fn      port_wakeup_dw3000
 * @brief   waking up of DW3000 using DW_CS pin
 *
 *          the fast wakeup takes ~1ms:
 *          500us to hold the CS  - TODO: this time can be reduced
 *          500us to the crystal to startup
 *          + ~various time 100us...10ms
 * */
error_e port_wakeup_dw3000(void)
{
    error_e  ret;

    spi_handle_t *p = pDwChip->pSpi;

    nrf_gpio_cfg_output(p->csPin);
    nrf_gpio_pin_clear(p->csPin);
    nrf_delay_us(500);
    nrf_gpio_pin_set(p->csPin);

    //it takes ~500us for crystal startup total for the DW3000 to switch to RC_IDLE 120MHz
    nrf_delay_us(500);

    uint32_t cnt=0;
    // we are here on 120MHz RC
    /* Need to make sure DW IC is in IDLE_RC before proceeding */
    do
    {
        cnt++;
        if(cnt >= 5)
        {
            //FIXME: DW3000 B0 silicon bugfix
            if(cnt == 5)
            {
                nrf_gpio_pin_clear(p->csPin);
                nrf_delay_us(500);
                nrf_gpio_pin_set(p->csPin);
            }
            cnt++;
        }
        nrf_delay_us(20);

    }while (!dwt_checkidlerc() && cnt<10);

    if (cnt >= 10)
    {
        ret = _ERR_INIT;
    }
    else
    {
        ret = _NO_ERR;
    }
    return ret;
}

void port_disable_wake_init_dw(void)
{
    taskENTER_CRITICAL();         

    disable_dw3000_irq();             /**< disable NVIC IRQ until we configure the device */

    port_reinit_dw_chips();

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    port_wakeup_dw3000();  

    if(port_init_device() != 0x00)
    {
        error_handler(1,  _ERR_INIT);
    }

    taskEXIT_CRITICAL();
}

void port_reinit_dw_chips(void)
{
    nrf_gpio_pin_clear(DW3000_RST_Pin);
    nrf_delay_ms(1);
    nrf_gpio_cfg_input(DW3000_RST_Pin, NRF_GPIO_PIN_NOPULL);
    nrf_delay_ms(2);
}

void port_init_dw_chip()
{
    dwt_setlocaldataptr(0);

    nrf_drv_spi_t   *spi_inst;
    nrf_drv_spi_config_t  *spi_config;

    spi_inst   = &pgSpiHandler->spi_inst;
    spi_config = &pgSpiHandler->spi_config;
    spi_inst->inst_idx = SPI3_INSTANCE_INDEX;
    spi_inst->use_easy_dma = SPI3_USE_EASY_DMA;
    spi_inst->u.spim.p_reg = NRF_SPIM3;
    spi_inst->u.spim.drv_inst_idx = NRFX_SPIM3_INST_IDX;

    pgSpiHandler->frequency_slow = NRF_SPIM_FREQ_4M;
    pgSpiHandler->frequency_fast = NRF_SPIM_FREQ_32M;
    pgSpiHandler->lock = DW_HAL_NODE_UNLOCKED;

    spi_config->sck_pin = DW3000_CLK_Pin;
    spi_config->mosi_pin = DW3000_MOSI_Pin;
    spi_config->miso_pin = DW3000_MISO_Pin;
    spi_config->ss_pin = NRFX_SPIM_PIN_NOT_USED;  // not allowing Nordic driver to control the CSn pin
    spi_config->irq_priority = (APP_IRQ_PRIORITY_MID - 2);
    spi_config->orc = 0xFF;
    spi_config->frequency = pgSpiHandler->frequency_slow;
    spi_config->mode = NRF_DRV_SPI_MODE_0;
    spi_config->bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

    set_dw_spi_slow_rate();

    nrf_gpio_pin_set(pDwChip->pSpi->csPin);
    nrf_gpio_cfg_output(pDwChip->pSpi->csPin);
}

/* @fn      set_dw_spi_slow_rate
 * @brief   set 2MHz
 *          
 * */
void set_dw_spi_slow_rate(void)
{
    //set_SPI_master();
    if(spi_init_stat == 1)
    {
        return;
    }
    else 
    {
        if(spi_init_stat == 2)
        {

            nrf_drv_spi_uninit(&pgSpiHandler->spi_inst);
        }

        pgSpiHandler->spi_config.frequency = spi_handler.frequency_slow;

        APP_ERROR_CHECK( nrf_drv_spi_init(&pgSpiHandler->spi_inst, 
                                          &pgSpiHandler->spi_config, 
                                          spi_event_handler,
                                          NULL) );
        spi_init_stat = 1;

        nrf_delay_ms(2);

    }    
}

/* @fn      set_dw_spi_fast_rate
 * @brief   set 16MHz
 *          
 * */
void set_dw_spi_fast_rate(void)
{
    if(spi_init_stat == 2)
    {
        return;
    }
    else 
    {
        if(spi_init_stat == 1)
        {
            nrf_drv_spi_uninit(&pgSpiHandler->spi_inst);
        }
        pgSpiHandler->spi_config.frequency = spi_handler.frequency_fast;

        APP_ERROR_CHECK( nrf_drv_spi_init(&pgSpiHandler->spi_inst, 
                                          &pgSpiHandler->spi_config, 
                                          spi_event_handler,
                                          NULL) );
        nrf_gpio_cfg(pgSpiHandler->spi_config.sck_pin,
                     NRF_GPIO_PIN_DIR_OUTPUT,
                     NRF_GPIO_PIN_INPUT_CONNECT,
                     NRF_GPIO_PIN_NOPULL,
                     NRF_GPIO_PIN_H0H1,
                     NRF_GPIO_PIN_NOSENSE);
        nrf_gpio_cfg( pgSpiHandler->spi_config.mosi_pin,
                     NRF_GPIO_PIN_DIR_OUTPUT,
                     NRF_GPIO_PIN_INPUT_DISCONNECT,
                     NRF_GPIO_PIN_NOPULL,
                     NRF_GPIO_PIN_H0H1,
                     NRF_GPIO_PIN_NOSENSE);
        spi_init_stat = 2;
        nrf_delay_ms(2);
    }
}

/**
 *  @brief     Bare-metal level
 *          initialise master/slave DW1000 (check if can talk to device and wake up and reset)
 */
static int
port_init_device(void)
{

    set_dw_spi_slow_rate();

    if (dwt_check_dev_id()!=DWT_SUCCESS)
    {
        return DWT_ERROR;
    }

    //clear the sleep bit in case it is set - so that after the hard reset below the DW does not go into sleep
    dwt_softreset(0);

    return 0;
}

void port_stop_all_UWB(void)
{
    port_disable_dw_irq_and_reset(1);
    dwt_setcallbacks(NULL, NULL, NULL, NULL, NULL, NULL, NULL);
}


/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
}

//==============================================================================

void close_spi(nrf_drv_spi_t *p_instance)
{

    NRF_SPIM_Type * p_spi = p_instance->u.spim.p_reg;
    nrf_spim_disable(p_spi);
}

void open_spi(nrf_drv_spi_t *p_instance)
{

    NRF_SPIM_Type * p_spi = p_instance->u.spim.p_reg;
    nrf_spim_enable(p_spi);
}

int readfromspi(uint16_t headerLength,
                uint8_t *headerBuffer,
                uint16_t readlength,
                uint8_t *readBuffer)
{
    int ret;    

    ret = readfromspi_uni(headerLength, headerBuffer,
                            readlength, readBuffer, pgSpiHandler);
    return ret;
}

int writetospi( uint16_t headerLength,
                const uint8_t *headerBuffer,
                uint16_t bodylength,
                const uint8_t *bodyBuffer)
{
    int ret;    
    ret = writetospi_uni(headerLength, headerBuffer,
                          bodylength, bodyBuffer, pgSpiHandler);
    return ret;
}

int writetospiwithcrc(uint16_t  headerLength,
               const uint8_t    *headerBuffer,
               uint16_t         bodyLength,
               const uint8_t    *bodyBuffer,
                           uint8_t          crc8)
{
    return _ERR;
}

static inline void SPI_CSn_Assert(uint32_t CSn_pin)
{
    nrf_gpio_pin_clear(CSn_pin);
}

static inline void SPI_CSn_Deassert(uint32_t CSn_pin)
{
    nrf_gpio_pin_set(CSn_pin);
}

int readfromspi_uni(uint16_t headerLength,
                    const uint8_t *headerBuffer,
                    uint16_t readlength,
                    uint8_t *readBuffer,
                    spi_handle_t *pgSpiHandler)
{
    while(pgSpiHandler->lock);

    __HAL_LOCK(pgSpiHandler);

    open_spi(&pgSpiHandler->spi_inst);

    SPI_CSn_Assert(pDwChip->pSpi->csPin);
    
    spi_xfer_done = false;
    nrfx_spim_xfer_desc_t const spim_xfer_desc_header =
        {
        .p_tx_buffer = headerBuffer,
        .tx_length   = headerLength,
        .p_rx_buffer = NULL,
        .rx_length   = 0,
        };
    nrfx_spim_xfer(&pgSpiHandler->spi_inst.u.spim, &spim_xfer_desc_header, 0);
    while(!spi_xfer_done);

    spi_xfer_done = false;
    nrfx_spim_xfer_desc_t const spim_xfer_desc_body =
        {
        .p_tx_buffer = NULL,
        .tx_length   = 0,
        .p_rx_buffer = readBuffer,
        .rx_length   = readlength,
        };
    nrfx_spim_xfer(&pgSpiHandler->spi_inst.u.spim, &spim_xfer_desc_body, 0);
    while(!spi_xfer_done);

    SPI_CSn_Deassert(pDwChip->pSpi->csPin);

    close_spi(&pgSpiHandler->spi_inst);

    __HAL_UNLOCK(pgSpiHandler);

    return 0;
}

int writetospi_uni(uint16_t headerLength,
                   const uint8_t *headerBuffer,
                   uint32_t bodylength,
                   const uint8_t *bodyBuffer,
                   spi_handle_t *pgSpiHandler)
{
    while(pgSpiHandler->lock);

    __HAL_LOCK(pgSpiHandler);

    open_spi(&pgSpiHandler->spi_inst);

    SPI_CSn_Assert(pDwChip->pSpi->csPin);

    spi_xfer_done = false;
    nrfx_spim_xfer_desc_t const spim_xfer_desc_header =
        {
        .p_tx_buffer = headerBuffer,
        .tx_length   = headerLength,
        .p_rx_buffer = NULL,
        .rx_length   = 0,
        };
    nrfx_spim_xfer(&pgSpiHandler->spi_inst.u.spim, &spim_xfer_desc_header, 0);
    while(!spi_xfer_done);

    spi_xfer_done = false;
    nrfx_spim_xfer_desc_t const spim_xfer_desc_body =
        {
        .p_tx_buffer = bodyBuffer,
        .tx_length   = bodylength,
        .p_rx_buffer = NULL,
        .rx_length   = 0,
        };
    nrfx_spim_xfer(&pgSpiHandler->spi_inst.u.spim, &spim_xfer_desc_body, 0);
    while(!spi_xfer_done);

    SPI_CSn_Deassert(pDwChip->pSpi->csPin);

    close_spi(&pgSpiHandler->spi_inst);

     __HAL_UNLOCK(pgSpiHandler);

    return 0;
}

/**@brief Systick handler
 *
 * @param[in] void
 */
void SysTick_Handler (void) {
        time32_incr++;
}
/******************************************************************************
 *
 *                              END OF Time section
 *
 ******************************************************************************/

/******************************************************************************
 *
 *                          DW3000 port section
 *
 ******************************************************************************/

/**@brief timer_event_handler
 *
 * @param[in] void
 */
void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    timer_val++;
    // Enable SysTick Interrupt
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}
/******************************************************************************
 *
 *                          End APP port section
 *
 ******************************************************************************/



/******************************************************************************
 *
 *                              IRQ section
 *
 ******************************************************************************/
/*! ----------------------------------------------------------------------------
 * Function: decamutexon()
 *
 * Description: This function should disable interrupts.
 *
 *
 * input parameters: void
 *
 * output parameters: uint16
 * returns the state of the DW1000 interrupt
 */

decaIrqStatus_t decamutexon(void)
{
/* NRF chip has only 1 IRQ for all GPIO pins.
 * Disablin of the NVIC would not be of the best ideas.
 */
    uint32_t s = nrf_drv_gpiote_in_is_set(pDwChip->irqPin);
    if(s)
    {
        nrf_drv_gpiote_in_event_disable(pDwChip->irqPin);
    }
    return 0;
}
/*! ----------------------------------------------------------------------------
 * Function: decamutexoff()
 *
 * Description: This function should re-enable interrupts, or at least restore
 *              their state as returned(&saved) by decamutexon
 * This is called at the end of a critical section
 *
 * input parameters:
 * @param s - the state of the DW1000 interrupt as returned by decamutexon
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
void decamutexoff(decaIrqStatus_t s)
{
    if(s)
    {
        nrf_drv_gpiote_in_event_enable(pDwChip->irqPin, true);
    }
}

/* @fn      port_CheckEXT_IRQ
 * @brief   wrapper to read DW_IRQ input pin state
 * */
uint32_t port_CheckEXT_IRQ(void)
{
    return nrf_gpio_pin_read(pDwChip->irqPin);
}

/* @fn      process_deca_irq
 * @brief   main call-back for processing of DW3000 IRQ
 *          it re-enters the IRQ routing and processes all events.
 *          After processing of all events, DW3000 will clear the IRQ line.
 * */
void process_deca_irq(void)
{
// The

    while(port_CheckEXT_IRQ() != 0)
    {
         dwt_isr();

    } //while DW3000 IRQ line active
    if (app.DwCanSleepInIRQ == DW_CAN_SLEEP)
    {
        app.DwEnterSleep = DW_IS_SLEEPING_IRQ;
        dwt_entersleep(DWT_DW_IDLE_RC);   //manual sleep after successful Final TX or RX timeout
        app.DwSpiReady = DW_SPI_SLEEPING;
    }
}

/******************************************************************************
 *
 *                              END OF IRQ section
 *
 ******************************************************************************/

/******************************************************************************
 *
 *                              portDW
 *
 ******************************************************************************/



/*
 * @brief disable DW_IRQ, reset DW3000
 *        and set
 *        app.DwCanSleep = DW_CANNOT_SLEEP;
 *        app.DwEnterSleep = DW_NOT_SLEEPING;
 * */
error_e port_disable_dw_irq_and_reset(int reset)
{
    taskENTER_CRITICAL();

    disable_dw3000_irq(); /**< disable NVIC IRQ until we configure the device */

    //this is called to reset the DW device
    if (reset)
    {
        reset_DW3000();
    }

    app.DwCanSleepInIRQ = DW_CANNOT_SLEEP;
    app.DwEnterSleep = DW_NOT_SLEEPING;

    taskEXIT_CRITICAL();

    return _NO_ERR;
}

void error_handler(int block, error_e err)
{
    app.lastErrorCode = err;

    /* Flash Error Led*/
    while(block)
    {
        for(int i = err; i>0; i--)
        {
            for(int j = 3; j>0; j--)
            {
                nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh

                nrf_gpio_pin_write(LED_ERROR, 0);
                nrf_delay_ms(100);
                nrf_gpio_pin_write(LED_ERROR, 1);
                nrf_delay_ms(100);
            }
            nrf_delay_ms(1000);
        }
    }
}


/****************************************************************************//**
 *                timer
 */

const nrf_drv_timer_t timer1 = NRF_DRV_TIMER_INSTANCE(0);

/**
 * @brief Handler for timer events.
 */
void timer1_event_handler(nrf_timer_event_t event_type, void* p_context)
{

    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            SysTick_Handler();
            break;

        default:
            //Do nothing.
            break;
    }
}

/* @fn         init_timer(void)
 * @brief     initiate timestamp (in CLOCKS_PER_SEC)
 * @parm     p_timestamp pointer on current system timestamp
 */

uint32_t init_timer(void)
{
    uint32_t time_ms = 1; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_125kHz;
    err_code = nrf_drv_timer_init(&timer1, &timer_cfg, timer1_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&timer1, time_ms);

    nrf_drv_timer_extended_compare(&timer1, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&timer1);
    
    return err_code;
}

/* @fn         start_timer(uint32 *p_timestamp)
 * @brief     save system timestamp (in CLOCKS_PER_SEC)
 * @parm     p_timestamp pointer on current system timestamp
 */
void start_timer(volatile uint32_t * p_timestamp)
{
    *p_timestamp = HAL_GetTick();
}

/* @fn         check_timer(uint32 timestamp , uint32 time)
 * @brief     check if time from current timestamp over expectation
 * @param     [in] timestamp - current timestamp
 * @param     [in] time - time expectation (in CLOCKS_PER_SEC)
 * @return     true - time is over
 *             false - time is not over yet
 */
bool check_timer(uint32_t timestamp, uint32_t time)
{
    uint32_t time_passing;
    uint32_t temp_tick_time = HAL_GetTick();

    if (temp_tick_time >= timestamp)
    {
        time_passing = temp_tick_time - timestamp;
    }
    else
    {
        time_passing = 0xffffffffUL - timestamp + temp_tick_time;
    }

    if (time_passing >= time)
    {
        return (true);
    }

    return (false);
}


__STATIC_INLINE void nrf_gpio_cfg_input_sense_low(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config)
{
    nrf_gpio_cfg(
        pin_number,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        pull_config,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_SENSE_LOW);
}


__STATIC_INLINE void nrf_gpio_cfg_input_sense_high(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config)
{
    nrf_gpio_cfg(
        pin_number,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        pull_config,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_SENSE_HIGH);
}

__STATIC_INLINE void nrf_gpio_cfg_input_sense_none(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config)
{
    nrf_gpio_cfg(
        pin_number,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        pull_config,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);
}

/* @brief    Sleep
 *             -DDEBUG defined in Makefile prevents __WFI
 */
void Sleep(volatile uint32_t dwMs)
{
    uint32_t dwStart;
    start_timer(&dwStart);
    while (check_timer(dwStart, dwMs) == false)
    {
#ifndef DEBUG
//        __WFI();
#endif
    }
}

/***************************************************************************//*
 *
 *                               timers section
 *
 *****************************************************************************/
void
app_apptimer_delete(void)
{
    taskENTER_CRITICAL();

    if(app.appTimerHandle)
    {
        osTimerStop(app.appTimerHandle);
        app.appTimerHandle = 0;
        app.timer2Isr = NULL;
    }

    taskEXIT_CRITICAL();
}

void
app_apptimer_stop(void)
{
    taskENTER_CRITICAL();

    if(app.appTimerHandle)
    {
        osTimerStop(app.appTimerHandle);
    }

    taskEXIT_CRITICAL();
}

void*
app_apptimer_getIsr(void)
{
    return (void*)app.timer2Isr;
}

static void
appTmr_cb(void const *arg)
{
    if(app.timer2Isr)
    {
        app.timer2Isr();
    }
}

void
app_apptimer_config(uint32_t ms, void *isr)
{
    taskENTER_CRITICAL();

    if(app.appTimerHandle)
    {
        osTimerDelete(app.appTimerHandle);
    }

    app.appTimer_MS = ms;
    app.timer2Isr = isr;

    osTimerDef(appTmr, appTmr_cb);
    app.appTimerHandle  = osTimerCreate(osTimer(appTmr), osTimerOnce, NULL);

    taskEXIT_CRITICAL();
}

void
app_apptimer_start(void)
{
    if(osTimerStart(app.appTimerHandle, app.appTimer_MS/portTICK_PERIOD_MS) != osOK)
    {
        error_handler(1,_ERR_Timer_Start_Bad);
    }
}

#define BUTTON_TIME_TAG_MS (3000)

#define DEFAULT_EVENT      (Ev_Stop_All)
/*  The starting mode of the app can be chosen from:
 *  Ev_Node_Task
 *  Ev_Tag_Task
 *  Ev_Usb2spi_Task
 *  Ev_Stop_All
 */

/* @brief
 *      no press: "default app": app, which was running on "SAVE" command
 *      if not set, then default is DEFAULT_EVENT
 *      pressed not more than 3 seconds: Tag
 *      pressed longer than 3 seconds: Node
 * */
EventBits_t check_the_user_button(void)
{
    EventBits_t ret = DEFAULT_EVENT;

    if(nrf_gpio_pin_read(BUTTON_1) == GPIO_PIN_SET)
    {
        if(app.pConfig->s.default_event != 0)
        {
            ret = app.pConfig->s.default_event;
        }
    }
    else
    {
        uint32_t tmr;
        start_timer(&tmr);
        bool tmp = false;

        while((nrf_gpio_pin_read(BUTTON_1) == GPIO_PIN_RESET) && !(tmp = check_timer(tmr, BUTTON_TIME_TAG_MS)))
        {
          nrf_drv_wdt_channel_feed(m_channel_id);
        }

        ret = (tmp)?(Ev_Node_Task):(Ev_Tag_Task);
    }
    return ret;
}



/******************************************************************************
 *
 *                              end of portDW
 *
 ******************************************************************************/


/******************************************************************************
 *
 *                              start of malloc
 *
 ******************************************************************************/
//-----------------------------------------------------------------------------
/*
 * The standard syscall malloc/free used in sscanf/sprintf.
 * We want them to be replaced with FreeRTOS's implementation.
 *
 * This leads that the memory allocation will be managed by FreeRTOS heap4 memory.
 * */
void* _calloc_r(struct _reent *re, size_t num, size_t size)
{
    return pvPortMalloc(num*size);
}

void* _malloc_r(struct _reent *re, size_t size)
{
    return pvPortMalloc(size);
}

void _free_r(struct _reent *re, void* ptr)
{
    vPortFree(ptr);
    return;
}

/******************************************************************************
 *
 *                              end of malloc
 *
 ******************************************************************************/
 //

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    ASSERT(false);
}

/*
 * @fn      vPortSuppressTicksAndSleep
 * @brief   The function that is called by portSUPPRESS_TICKS_AND_SLEEP()
 * @param   [in] xExpectedIdleTime - time to sleep in FreeRTOS ticks
 */
void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
    /*
     * Implementation note:
     *
     * To help debugging the option configUSE_TICKLESS_IDLE_SIMPLE_DEBUG was presented.
     * This option would make sure that even if program execution was stopped inside
     * this function no more than expected number of ticks would be skipped.
     *
     * Normally RTC works all the time even if firmware execution was stopped
     * and that may lead to skipping too much of ticks.
     */

    TickType_t enterTime;

    /* Make sure the SysTick reload value does not overflow the counter. */
    if ( xExpectedIdleTime > portNRF_RTC_MAXTICKS - configEXPECTED_IDLE_TIME_BEFORE_SLEEP )
    {
        xExpectedIdleTime = portNRF_RTC_MAXTICKS - configEXPECTED_IDLE_TIME_BEFORE_SLEEP;
    }
    /* Block all the interrupts globally */
#ifdef SOFTDEVICE_PRESENT
    do{
        uint8_t dummy = 0;
        uint32_t err_code = sd_nvic_critical_region_enter(&dummy);
        APP_ERROR_CHECK(err_code);
    }while (0);
#else
    __disable_irq();
#endif

    enterTime = nrf_rtc_counter_get(portNRF_RTC_REG);

    if ( eTaskConfirmSleepModeStatus() != eAbortSleep )
    {
        TickType_t xModifiableIdleTime;
        TickType_t wakeupTime = (enterTime + xExpectedIdleTime) & portNRF_RTC_MAXTICKS;

        /* Stop tick events */
        nrf_rtc_int_disable(portNRF_RTC_REG, NRF_RTC_INT_TICK_MASK);

        /* Configure CTC interrupt */
        nrf_rtc_cc_set(portNRF_RTC_REG, 0, wakeupTime);
        nrf_rtc_event_clear(portNRF_RTC_REG, NRF_RTC_EVENT_COMPARE_0);
        nrf_rtc_int_enable(portNRF_RTC_REG, NRF_RTC_INT_COMPARE0_MASK);

        __DSB();

        /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
         * set its parameter to 0 to indicate that its implementation contains
         * its own wait for interrupt or wait for event instruction, and so wfi
         * should not be executed again.  However, the original expected idle
         * time variable must remain unmodified, so a copy is taken. */
        xModifiableIdleTime = xExpectedIdleTime;
        configPRE_SLEEP_PROCESSING( xModifiableIdleTime );
        if ( xModifiableIdleTime > 0 )
        {
#if 0  // With FreeRTOS sd_app_evt_wait increases power consumption with FreeRTOS compared to _WFE (NRFFOSDK-11174)
#ifdef SOFTDEVICE_PRESENT
            if (nrf_sdh_is_enabled())
            {
                uint32_t err_code = sd_app_evt_wait();
                APP_ERROR_CHECK(err_code);
            }
            else
#endif
#endif // (NRFFOSDK-11174)
        NVIC_ClearPendingIRQ(RTC0_IRQn);
        NVIC_ClearPendingIRQ(RTC1_IRQn);

// errata #87, 3.12 Errata_v1.6
#if (__FPU_USED == 1)
            __set_FPSCR(__get_FPSCR() & ~(0x0000009F));
            (void) __get_FPSCR();
            NVIC_ClearPendingIRQ(FPU_IRQn);
#endif // errata #87, 3.12 Errata_v1.6
            {
                /* No SD -  we would just block interrupts globally.
                * BASEPRI cannot be used for that because it would prevent WFE from wake up.
                */
                do{
                    __WFI();
                } while (0 == (NVIC->ISPR[0] | NVIC->ISPR[1]));
            }
        }

        configPOST_SLEEP_PROCESSING( xExpectedIdleTime );

        nrf_rtc_int_disable(portNRF_RTC_REG, NRF_RTC_INT_COMPARE0_MASK);
        nrf_rtc_event_clear(portNRF_RTC_REG, NRF_RTC_EVENT_COMPARE_0);

        /* Correct the system ticks */
        {
            TickType_t diff;
            TickType_t exitTime;

            nrf_rtc_event_clear(portNRF_RTC_REG, NRF_RTC_EVENT_TICK);
            nrf_rtc_int_enable (portNRF_RTC_REG, NRF_RTC_INT_TICK_MASK);

            exitTime = nrf_rtc_counter_get(portNRF_RTC_REG);
            diff =  (exitTime - enterTime) & portNRF_RTC_MAXTICKS;

            /* It is important that we clear pending here so that our corrections are latest and in sync with tick_interrupt handler */
            NVIC_ClearPendingIRQ(portNRF_RTC_IRQn);

            if ((configUSE_TICKLESS_IDLE_SIMPLE_DEBUG) && (diff > xExpectedIdleTime))
            {
                diff = xExpectedIdleTime;
            }

            if (diff > 0)
            {
                vTaskStepTick(diff);
            }
        }
    }
#ifdef SOFTDEVICE_PRESENT
    uint32_t err_code = sd_nvic_critical_region_exit(0);
    APP_ERROR_CHECK(err_code);
#else

  __enable_irq();
#endif
}
