/*!----------------------------------------------------------------------------
 * @file    header for task_tag.h
 *
 * @brief   Decawave Application Layer
 *          RTOS tag implementation
 *
 * @attention
 *
 * Copyright 2017-2019 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author
 *
 * Decawave
 */

#ifndef __TAG_TASK__H__
#define __TAG_TASK__H__ 1

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include "lis2dw12.h"
 typedef struct {
    uint16_t    addr;
    uint16_t    node_addr;
    uint8_t     rNum;
    int16_t     x_cm;
    int16_t     y_cm;
    int16_t     clkOffset_pphm;
 }twr_res_ext_t;

void tag_helper(void const *argument);
void tag_terminate(void);

/* IMU-driven sleep/wake - see task_tag.c for the state machine. */
void imu_event_callback(lis2dw12_event_t evt);

/* Host UART notifications for IMU sleep/wake and anchor-loss - implemented in json_2pc.c. */
void send_to_pc_imu_sleep(void);
void send_to_pc_imu_wake(void);
void send_to_pc_anchor_lost(void);
void uart_host_wake_gpio_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __TWR_TASK__H__ */
