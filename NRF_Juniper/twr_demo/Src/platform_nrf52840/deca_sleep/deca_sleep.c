/**-
 * @file    deca_sleep.c
 *
 * @brief   platform dependent sleep implementation
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include "deca_device_api.h"
#include "port.h"

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_sleep(unsigned int time_ms)
{
    nrf_delay_ms(time_ms);
}

/* @fn    usleep
 * @brief precise usleep() delay
 * */
void usleep(unsigned long time_us)
{
    nrf_delay_us(time_us);
}

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_usleep(unsigned long time_us)
{
    usleep(time_us);
}
