/**
 * @file      lis2dw12_platform.h
 *
 * @brief     nRF52840 platform glue (SPI) for the ST LIS2DW12 accelerometer
 *            register driver (drivers/lis2dw12_driver/driver/lis2dw12_reg.c).
 *            Used by Src/apps/tag/task_tag/lis2dw12.c, the interrupt-driven
 *            motion/inactivity driver - this file is purely the SPI
 *            transport, no interrupt/GPIO handling here.
 *
 * @attention Copyright 2019-2020 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 */

#ifndef LIS2DW12_PLATFORM_H_
#define LIS2DW12_PLATFORM_H_ 1

#include "lis2dw12_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialises the SPI instance used to talk to the LIS2DW12 and fills in
 * ctx->write_reg / ctx->read_reg / ctx->handle. Safe to call more than once. */
void lis2dw12_platform_spi_init(stmdev_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* LIS2DW12_PLATFORM_H_ */
