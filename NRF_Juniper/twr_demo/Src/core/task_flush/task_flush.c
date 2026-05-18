/*
 * @file      task_flush.c
 *
 * @brief     Flush task
  *
 * @author    Decawave
 *
 * @attention Copyright 2017-2019 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 */

#include "task_flush.h"

#include "usb_uart_tx.h"
#include "port.h"

#define USB_FLUSH_MS    5

/*
 * @brief this thread is
 *        flushing report buffer on demand or every USB_FLUSH_MS ms
 * */
void FlushTask(void const * argument)
{

    while(1)
    {
        osSignalWait(app.flushTask.Signal, USB_FLUSH_MS / portTICK_PERIOD_MS);

        flush_report_buf();
    }
}

void FlushTask_reset(void)
{
    if(app.flushTask.Handle)
    {
        taskENTER_CRITICAL();
        reset_report_buf();
        taskEXIT_CRITICAL();
    }
}
