/**
 * @file      lis2dw12_platform.h
 *
 * @brief     nRF52840 platform glue (SPI) for the ST LIS2DW12 accelerometer
 *            register driver (drivers/lis2dw12_driver/driver/lis2dw12_reg.c).
 *            INT1 wake-source arming uses raw GPIO SENSE (System-OFF capable)
 *            configured directly in task_sleep.c via nrf_gpio_cfg_sense_input()
 *            - no GPIOTE/interrupt handling is needed here.
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

/* Configures HOST_WAKEUP_Pin as an output and drives it low. Call this
 * unconditionally, once, early at boot (regardless of wake reason) so the
 * pin has a well-defined LOW idle level - without this it's left floating
 * (nRF52 GPIO reset default) until the first real wake pulse, which reads as
 * a constant HIGH if the net has an external pull-up. */
void host_wakeup_pin_init(void);

/* Drives HOST_WAKEUP_Pin high for a few ms then low, to signal the host
 * processor that the module has woken up. */
void host_wakeup_pulse(void);

#ifdef __cplusplus
}
#endif

#endif /* LIS2DW12_PLATFORM_H_ */
