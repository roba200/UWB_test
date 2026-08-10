/*
 * @attention
 *
 * Copyright 2019 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#ifndef _MURATA_EVB_2AB_BSP_H
#define _MURATA_EVB_2AB_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* ENABLE_USB_PRINT Macro is uncommented then Segger RTT Print will be enabled*/
#define ENABLE_USB_PRINT

#if defined(BOARD_CUSTOM)
#include "nrf_gpio.h"
#endif

#define RX_PIN_NUMBER  NRF_GPIO_PIN_MAP(0, 8)
#define TX_PIN_NUMBER  NRF_GPIO_PIN_MAP(0, 7)
#define CTS_PIN_NUMBER UART_PIN_DISCONNECTED
#define RTS_PIN_NUMBER UART_PIN_DISCONNECTED

#define DW3000_RST_Pin      NRF_GPIO_PIN_MAP(0, 15)
#define DW3000_IRQ_Pin      NRF_GPIO_PIN_MAP(0, 25)
#define DW3000_WUP_Pin      NRF_GPIO_PIN_MAP(1, 2)

// SPI defs
#define DW3000_CS_Pin       NRF_GPIO_PIN_MAP(0, 20)
#define DW3000_CLK_Pin      NRF_GPIO_PIN_MAP(0, 16)
#define DW3000_MOSI_Pin     NRF_GPIO_PIN_MAP(0, 17)
#define DW3000_MISO_Pin     NRF_GPIO_PIN_MAP(0, 23)
#define DW3000_SPI_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW

// UART symbolic constants
#define UART_0_TX_PIN       TX_PIN_NUMBER
#define UART_0_RX_PIN       RX_PIN_NUMBER
#define DW3000_RTS_PIN_NUM      UART_PIN_DISCONNECTED
#define DW3000_CTS_PIN_NUM      UART_PIN_DISCONNECTED

// LIS2DW12 accelerometer (4-wire SPI, separate SPI instance from the DW3000)
#define LIS2DW12_SCL_Pin     NRF_GPIO_PIN_MAP(0, 19)   /* SPC */
#define LIS2DW12_CS_Pin      NRF_GPIO_PIN_MAP(1, 3)
#define LIS2DW12_SDO_Pin     NRF_GPIO_PIN_MAP(1, 0)     /* MISO / SA0 */
#define LIS2DW12_SDI_Pin     NRF_GPIO_PIN_MAP(0, 22)    /* MOSI */
#define LIS2DW12_INT1_Pin    NRF_GPIO_PIN_MAP(1, 1)     /* Wake-Up-on-motion interrupt */
#define LIS2DW12_INT2_Pin    NRF_GPIO_PIN_MAP(1, 4)     /* unused, reserved */

// GPIO pulsed to signal the host processor that the module has woken up
#define HOST_WAKEUP_Pin      NRF_GPIO_PIN_MAP(0, 27)

#ifdef __cplusplus
}
#endif
#endif // _MURATA_EVB_2AB_BSP_H
