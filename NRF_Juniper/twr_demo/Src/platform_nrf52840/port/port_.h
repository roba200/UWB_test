/**
 * @file      port.h
 *
 * @brief     port headers file to STM32F429
 *
 * @author    Decawave
 *
 * @attention Copyright 2017-2019 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __PORT__H__
#define __PORT__H__ 1

#ifdef __cplusplus
 extern "C" {
#endif

//#include <stdint.h>
#include <stdbool.h>
//#include "string.h"

#include "app.h"

//1 #include "usbd_cdc.h"

//#include "cmsis_os.h"

//#include "error.h"

//#include "circ_buf.h"
//#include "deca_spi.h"
//#include "deca_phy.h"
//#include "uwb_frames.h"
//1 #include "usbd_def.h"
//-----------------------------------------------------------------------------

#define assert_param(expr) ((void)0U)

#define RTLS_PORT   (3000)

//NRF
extern uint32_t gRTC_SF_PERIOD;


/* Definitions */

enum Priorities {
    /* Interrupts, which cannot use FreeRTOS API functions */
    PRIO_SPI1_IRQn          = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 2,
    PRIO_DMA2_Stream2_IRQn  = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1 ,
    PRIO_DMA2_Stream3_IRQn  = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1,

    /* Interrupt safe FreeRTOS API functions below is CMSIS IRQ's */
    PRIO_RTC_WKUP_IRQn      = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, //equivalent to the highest in the FreeRTOS
    PRIO_OTG_FS_IRQn        = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1,
    PRIO_USART3_IRQn        = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1,
    PRIO_DMA1_Stream1_IRQn  = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1,
    PRIO_DMA1_Stream3_IRQn  = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1,
    PRIO_ETH_IRQn           = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1,
    PRIO_TIM1_CC_IRQn       = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1,
    PRIO_TIM3_IRQn          = configLIBRARY_LOWEST_INTERRUPT_PRIORITY,

    /* Application-specific priorities : CMSIS-OS priorities +3..0..-3
     * osPriorityRealtime  is not used by Application level
     * osPriorityHigh       => configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + ?
     * osPriorityAboveNormal
     *
     * */
    PRIO_FlushTask          = osPriorityAboveNormal, /* FlushTask should have higher priority than CalckTask */
    PRIO_CtrlTask           = osPriorityNormal,
    PRIO_StartDefaultTask   = osPriorityLow,

    PRIO_RxTask             = osPriorityHigh,
    PRIO_CalcTask           = osPriorityNormal,
    PRIO_TrilatTask         = osPriorityBelowNormal,

    PRIO_TagPollTask        = osPriorityHigh,
    PRIO_TagRxTask          = osPriorityHigh,
    PRIO_BlinkTask          = osPriorityNormal,

    PRIO_TcfmTask           = osPriorityNormal,
    PRIO_TcwmTask           = osPriorityNormal,
    PRIO_Usb2SpiTask        = osPriorityNormal

};



typedef enum {
    Twr_Tx_Blink_Sent,          //tag sends blink
    Twr_Tx_Ranging_Config_Sent, //node sends range init
    Twr_Tx_Poll_Sent,           //tag sends poll
    Twr_Tx_Resp_Sent,           //node sends response
    Twr_Tx_Final_Sent,          //tag sends final
    Twr_Tx_Report_Sent          //not used
}tx_states_e;

//-----------------------------------------------------------------------------
// common macros

#ifndef SWAP
#define SWAP(a,b) {a^=b;b^=a;a^=b;}
#endif /* SWAP */

#ifndef MIN
#define MIN(a,b)    (((a) < (b)) ? (a) : (b))
#endif /* MIN */

#ifndef MAX
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#endif /* MAX */

#ifndef TRUE
#define TRUE  1
#endif /* TRUE */

#ifndef FALSE
#define FALSE  0
#endif /* FALSE */


#define MASK_40BIT            (0x00FFFFFFFFFFULL)  // DW1000 counter is 40 bits
#define MASK_TXDTS            (0x00FFFFFFFE00ULL)  //The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.


//-----------------------------------------------------------------------------
// experimental FREERtos macros: TODO:

#define TERMINATE_STD_TASK(x) \
do{\
    if(x .Handle) {\
        osMutexWait(x .MutexId, osWaitForever);\
        taskENTER_CRITICAL();\
        osMutexRelease(x .MutexId);\
        if(osThreadTerminate(x .Handle) == osOK)\
        {\
            osMutexDelete(x .MutexId);\
            x .Handle = NULL;\
            x .MutexId = NULL;\
        }\
        else\
        {\
            error_handler(1, _ERR_Cannot_Delete_Task);\
        }\
        taskEXIT_CRITICAL();\
    }\
} while(0)







//-----------------------------------------------------------------------------
//    DWxxx description

/* description of spi interface to DW1000 chip */
//struct spi_handle_s
//{
//    SPI_HandleTypeDef       *phspi;
//    uint32_t                prescaler_slow;
//    uint32_t                prescaler_fast;
//    DMA_TypeDef             *dmaRx;
//    uint32_t                streamRx;
//    uint32_t                channelRx;
//    DMA_TypeDef             *dmaTx;
//    uint32_t                streamTx;
//    uint32_t                channelTx;
//    uint32_t                csPin;
//    GPIO_TypeDef            *csPort;
//    __IO HAL_LockTypeDef    Lock;
//    __IO uint32_t           TxRxComplete;
//    uint8_t                 *pBuf;
//};
//
//typedef struct spi_handle_s spi_handle_t;

/* description of connection to the DW1000 chip */
//struct dw_s
//{
//    uint16_t        irqPin;
//    GPIO_TypeDef    *irqPort;
//    IRQn_Type       irqN;
//    uint16_t        rstPin;
//    GPIO_TypeDef    *rstPort;
//    uint16_t        wakeUpPin;
//    GPIO_TypeDef    *wakeUpPort;
//    IRQn_Type       rstIrqN;
//    spi_handle_t    *pSpi;
//};
//
//typedef struct dw_s dw_t;

/****************************************************************************//**
 *
 * */

extern app_t app;
//extern RTC_HandleTypeDef hrtc;
//extern SPI_HandleTypeDef hspi1;
//extern SPI_HandleTypeDef hspi2;
//extern SPI_HandleTypeDef hspi3;
//extern DMA_HandleTypeDef hdma_spi1_rx;
//extern DMA_HandleTypeDef hdma_spi1_tx;
//extern DMA_HandleTypeDef hdma_spi2_rx;
//extern DMA_HandleTypeDef hdma_spi2_tx;
//extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;
//extern UART_HandleTypeDef huart3;
//extern DMA_HandleTypeDef hdma_usart2_rx;
//extern DMA_HandleTypeDef hdma_usart2_tx;
//extern DMA_HandleTypeDef hdma_memtomem_dma1_channel1;
//extern USBD_HandleTypeDef hUsbDeviceFS;
//extern IWDG_HandleTypeDef hiwdg;
/****************************************************************************//**
 * port functions prototypes
 *
 * */
//void set_SPI_master(void);
//void set_SPI_slave(void);
//error_e port_wakeup_dw3000(void);

void init_dw3000_irq(void);
void enable_dw3000_irq(void);
void disable_dw3000_irq(void);
void reset_DW3000(void);

void port_stop_all_UWB(void);

error_e port_wakeup_dw3000(void);

/* mutex's */
decaIrqStatus_t decamutexon(void);
void decamutexoff(decaIrqStatus_t s);

/* Time section */
uint32_t init_timer(void);
void start_timer(volatile uint32_t * p_timestamp);
bool check_timer(uint32_t timestamp, uint32_t time);
void Sleep( volatile uint32_t );
void juniper_configure_hal_rtc_callback(void (*cb)(void));

/* button section */
EventBits_t check_the_user_button(void);

/* SYNC section */
//void port_set_syncenable(int enable);
//void port_set_sync(int enable);
//void port_set_syncclear(int enable);

error_e port_disable_dw_irq_and_reset(int reset);

void port_uart_rx_init(void);


/* application slow timer */
void app_apptimer_config(uint32_t Time_mS, void *isr);
void app_apptimer_start(void);
void app_apptimer_stop(void);
void app_apptimer_delete(void);
void* app_apptimer_getIsr(void);

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

#ifdef __cplusplus
}
#endif

#endif /* __PORT__H__ */
