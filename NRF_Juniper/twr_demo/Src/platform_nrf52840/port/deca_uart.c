/*! ----------------------------------------------------------------------------
 * @file    deca_uart.c
 * @brief   HW specific definitions and functions for UART Interface
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Deepa Gopinath
 */

#include "port.h"
#include "nrf_uart.h"
#include "app_uart.h"
#include "app.h"

/******************************************************************************
 *
 *                              Local variables
 *
 ******************************************************************************/

 static bool discard_next_symbol = false;

/******************************************************************************
 *
 *                              Uart Configuration
 *
 ******************************************************************************/

/* @fn  uart_error_handle
 *
 * @param[in] p_event Pointer to UART event.
 * */
void deca_uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        error_handler(0, _ERR_General_Error); 
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        error_handler(0, _ERR_General_Error); 
    }
    /* This event indicates that UART data has been received  */
    else if (p_event->evt_type == APP_UART_DATA_READY)
    {
        deca_uart_receive();
    }
}

/* @fn  deca_uart_init
 *
 * @brief Function for initializing the UART module.
 *
 * @param[in] void
 * */
void deca_uart_init(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
    {
        UART_0_RX_PIN,
        UART_0_TX_PIN,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    // Tx and Rx buffers have the same size
    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_RX_BUF_SIZE,
                       deca_uart_error_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    UNUSED_VARIABLE(err_code);
}

void deca_uart_close(void)
{
    app_uart_flush();
    app_uart_close();
}

/* @fn  deca_uart_transmit
 *
 * @brief Function for transmitting data on UART
 *
 * @param[in] tx_buffer char Pointer that contains the base address of the data buffer.
 * @param[in] size      int  The size of the data buffet to be sent.
 * @return    Returns NRF_SUCCESS or a related error code.
 * 
 * */
int deca_uart_transmit(char *tx_buffer, int size)
{
    ret_code_t ret;

    for (int i=0; i < size ; i++)
    {  
        if (app_uart_put(tx_buffer[i]) != NRF_SUCCESS)
        {
            error_handler(0, _ERR_UART_TX); /**< indicate UART transmit error */
            ret = _ERR_UART_TX;
            break;
        }
    }
    return ret;
}

/* @fn  deca_uart_receive
 *
 * @brief Function for receive data from UART and store into rx_buf
 *        (global array).
 *
 * @param[in] void
 * */
void deca_uart_receive(void)
{
    uint32_t err_code;
    uint8_t rx_data;
    int head, tail, size;

    head = app.uartRx.head;
    tail = app.uartRx.tail;
    size = sizeof(app.uartRx.buf);

    err_code = app_uart_get(&rx_data);
    if (discard_next_symbol) 
    {
        discard_next_symbol = false;
    }
    else
    {
        if (CIRC_SPACE(head, tail, size) > 0)
        {
            app.uartRx.buf[head] = rx_data;
            head = (head + 1) & (size - 1);
            app.uartRx.head = head;
        }
        else
        {
            /* USB RX packet can not fit free space in the buffer */
        }

        if(app.ctrlTask.Handle) //RTOS : ctrlTask could be not started yet
        {
            osSignalSet(app.ctrlTask.Handle, app.ctrlTask.Signal);    //signal to the ctrl thread : USB data ready
        }
    }
}

/****************************************************************************//**
 *
 *                          End of UART Configuration
 *
 *******************************************************************************/
