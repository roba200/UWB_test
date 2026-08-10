/**
 * @file      lis2dw12_platform.h
 *
 * @brief     nRF52840 platform glue (SPI + GPIOTE) for the ST LIS2DW12
 *            accelerometer register driver (drivers/lis2dw12_driver/driver/lis2dw12_reg.c).
 *
 * @attention Copyright 2019-2020 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 */

#ifndef LIS2DW12_PLATFORM_H_
#define LIS2DW12_PLATFORM_H_ 1

#include "lis2dw12_reg.h"
#include "nrf_drv_gpiote.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialises the SPI instance used to talk to the LIS2DW12 and fills in
 * ctx->write_reg / ctx->read_reg / ctx->handle. Safe to call more than once. */
void lis2dw12_platform_spi_init(stmdev_ctx_t *ctx);

/* One-time registration of the INT1 pin with the shared GPIOTE peripheral
 * (already initialised by dw_irq_init() at boot). The event is registered
 * disabled; use lis2dw12_int1_enable()/disable() to arm/disarm it. */
void lis2dw12_int1_gpiote_init(nrf_drv_gpiote_evt_handler_t handler);

void lis2dw12_int1_enable(void);
void lis2dw12_int1_disable(void);

/* Drives HOST_WAKEUP_Pin high for a few ms then low, to signal the host
 * processor that the module has woken up. */
void host_wakeup_pulse(void);

#ifdef __cplusplus
}
#endif

#endif /* LIS2DW12_PLATFORM_H_ */
