/*! ----------------------------------------------------------------------------
 * @file    port_platform.h
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Deepa Gopinath
 */


#ifndef PORT_PLATFORM_H_
#define PORT_PLATFORM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "port_.h" //old include TODO: copy all relevant to here

#include "murata_evb_2ab_bsp.h"
#include "deca_types.h"
#include "deca_device_api.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
//#include "nrf_log.h"
#include "boards.h"
#include "app_error.h"
#include "app_uart.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_rtc.h"
#include "error.h"

/* calculates the ticks required for x us*/
#define RTC_US(x) RTC_US_TO_TICKS( x , RTC_DEFAULT_CONFIG_FREQUENCY )
#define TIMER_RTC_CC 0
extern const nrf_drv_rtc_t rtc;

uint32_t rtc_delay_interrupt_disable(void);
uint32_t rtc_delay_interrupt_us(uint32_t x);
uint32_t rtc_init(void);

void juniper_configure_hal_rtc_callback(void (*cb)(void));
void rtc_handler(nrf_drv_rtc_int_type_t int_type);

/*

Note: Antenna Delay Values
The sum of the values is the TX to RX antenna delay, this should be
experimentally determined by a calibration process. Here we use a hard coded
value (expected to be a little low so a positive error will be seen on the
resultant distance estimate. For a real production application, each
device should have its own antenna delay properly calibrated to get good
precision when performing range measurements.
*/

/**< max number of test bytes to be used for tx and rx. */
#define MAX_TEST_DATA_BYTES     (15U)

/* Default antenna delay values for 64 MHz PRF.*/
#define TX_ANT_DLY 16456
#define RX_ANT_DLY 16456

#define LED_ERROR BSP_LED_0


#define MASK_40BIT            (0x00FFFFFFFFFFULL)  // DW1000 counter is 40 bits
#define MASK_TXDTS            (0x00FFFFFFFE00ULL)  //The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.
#define UNUSED(x) ((void)(x))

//#define UART_0_RX_PIN  5
//#define UART_0_TX_PIN  31

typedef enum {
    DW_MASTER = 0,
    DW_SLAVE
} dw_name_e;

typedef enum
{
  DW_HAL_NODE_UNLOCKED  = NRF_SUCCESS,
  DW_HAL_NODE_LOCKED    = NRF_ERROR_BUSY
} dw_hal_lockTypeDef;


#define __HAL_LOCK(__HANDLE__)                                           \
                                do{                                        \
                                    if((__HANDLE__)->lock == DW_HAL_NODE_LOCKED)   \
                                    {                                      \
                                       return NRF_ERROR_BUSY;                    \
                                    }                                      \
                                    else                                   \
                                    {                                      \
                                       (__HANDLE__)->lock = DW_HAL_NODE_LOCKED;    \
                                    }                                      \
                                }while (0U)

 #define __HAL_UNLOCK(__HANDLE__)                                          \
                                  do{                                       \
                                    (__HANDLE__)->lock = DW_HAL_NODE_UNLOCKED;    \
                                  }while (0U)

/* description of spi interface to DW1000 chip */
typedef struct
{
  nrf_drv_spi_t         spi_inst;
  uint32_t              frequency_slow;
  uint32_t              frequency_fast;
  uint32_t              csPin;
  nrf_drv_spi_config_t  spi_config;
  dw_hal_lockTypeDef    lock;
}spi_handle_t;

/* description of connection to the DW3000 chip */
struct dw_s
{
    uint16_t        irqPin;
    IRQn_Type       irqN;
    uint16_t        rstPin;
    uint16_t        wakeUpPin;
    IRQn_Type       rstIrqN;
    spi_handle_t    *pSpi;
};

typedef struct dw_s dw_t;

extern const dw_t *pDwMaster;
extern const dw_t *pDwSlave;

extern dw_t dw_chip_A;
extern dw_t dw_chip_B;


int readfromspi(uint16_t headerLength,
                uint8_t *headerBuffer,
                uint16_t readlength,
                uint8_t *readBuffer);

int writetospi( uint16_t headerLength,
                const uint8_t *headerBuffer,
                uint16_t bodylength,
                const uint8_t *bodyBuffer);

int writetospiwithcrc(uint16_t  headerLength,
               const uint8_t    *headerBuffer,
               uint16_t         bodyLength,
               const uint8_t    *bodyBuffer,
               uint8_t          crc8);

void wakeup_device_with_io(void);
						   
decaIrqStatus_t decamutexon(void);
void decamutexoff(decaIrqStatus_t s);

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler_test(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context);

/******************************************************************************/
//-----------------------------------------------------------------------------
// common macros

#ifndef CIRC_CNT
#define CIRC_CNT(head,tail,size) (((head) - (tail)) & ((size)-1))
#endif/* Return count in buffer.  */

#ifndef CIRC_SPACE
#define CIRC_SPACE(head,tail,size) CIRC_CNT((tail),((head)+1),(size))
#endif/* Return space available, 0..size-1 */

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


 /*****************************************************************************
  *
  *                                 Types definitions
  *
  *****************************************************************************/


#ifndef FALSE
#define FALSE               0
#endif

#ifndef TRUE
#define TRUE                1
#endif


/******************************************************************************
 *
 *                              port function prototypes
 *
 ******************************************************************************/

void Sleep(uint32_t Delay);
void usleep(unsigned long usec);
uint32_t portGetTickCount(void);
#define HAL_GetTick portGetTickCount

void error_handler(int block, error_e err);

error_e port_wakeup_dw3000_fast(void);
error_e port_wakeup_dw3000(void);
void port_wakeup_dw1000_fast(void);

//void port_set_dw1000_slowrate(void);
void set_dw_spi_fast_rate(void);
void set_dw_spi_slow_rate(void);

void process_dwRSTn_irq(void);
void process_deca_irq(void);


void setup_DW1000RSTnIRQ(int enable);

void reset_DW3000(void);

int inittestapplication(void);
void interrupts_init(void);
void peripherals_init(void);
void dw_irq_init(void);
void deca_uart_init(void);
void deca_uart_close(void);
void deca_uart_error_handle(app_uart_evt_t * p_event);
int  deca_uart_transmit(char *tx_buffer, int size);
void deca_uart_receive(void);
void low_power(int);
void timer_event_handler(nrf_timer_event_t , void*);
int pp_usb_init(void);
int deca_usb_transmit(char *tx_buffer, int size);
void port_init_dw_chip(void);
void port_reinit_dw_chips(void);
void port_disable_wake_init_dw(void);
void disable_dw3000_irq(void);
//void set_SPI_master(void);
//void set_SPI_slave(void);
void enable_dw3000_irq(void);

void Restart_UART_timer();
void deca_discard_next_symbol(void);

void start_timer(volatile uint32_t * p_timestamp);
bool check_timer(uint32_t timestamp, uint32_t time);

void port_stop_all_UWB(void);

//void port_set_syncenable(int enable);
//void port_set_sync(int enable);
//void port_set_syncclear(int enable);

#ifdef __cplusplus
}
#endif

#endif /* PORT_PLATFORM_H_ */

