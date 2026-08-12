/**
 * @file      lis2dw12.h
 * @brief     Interrupt-driven LIS2DW12 motion/inactivity driver for TAG.
 *
 *            INT1 = wake-up-on-motion, INT2 = inactivity (sleep-state).
 *            Both conditions are detected entirely in hardware (the chip's
 *            own ACT_SLEEP_DUR register is the inactivity timer) - this
 *            driver only configures the sensor once and then relays its
 *            interrupts to a callback. No RTOS polling.
 *
 *            Ported from a reference "patient monitor" firmware variant,
 *            scoped down to motion/inactivity only - free-fall detection is
 *            intentionally not included.
 *
 * @attention Copyright 2019-2020 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 */

#ifndef __LIS2DW12__H__
#define __LIS2DW12__H__ 1

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Tunables ----------------------------------------------------------- */

/* Master switch for the whole inactivity -> System OFF sleep behaviour
 * (see tag_handle_imu_events()'s IMU_EVT_SLEEP branch in task_tag.c). Set
 * to 0 to keep motion detection/reporting (IMU_WAKE, LED, etc.) running as
 * normal but never actually enter System OFF - handy for development,
 * where powering all the way down on every idle period gets in the way. */
#define TAG_SLEEP_MODE_ENABLE               1

#define LIS2DW12_ODR_MOTION_ONLY_HZ        (100)

/* Wake-up (activity) threshold, 1 LSB = FS_XL/64 = 2g/64 = 31.25 mg at FS=2g. */
#define LIS2DW12_WAKE_THRESHOLD_LSB        (1)

/* Wake-up duration, in ODR samples - debounces short spikes. */
#define LIS2DW12_WAKE_DURATION_SAMPLES     (1)

/* Default inactivity period, in seconds (used on first boot / invalid NVM). */
#define LIS2DW12_DEFAULT_INACTIVITY_SECONDS (30U)

/* Bounds for host-configurable inactivity period (seconds). */
#define LIS2DW12_INACTIVITY_SECONDS_MIN    (1U)
#define LIS2DW12_INACTIVITY_SECONDS_MAX    (3600U)


/**
 * Lis2dw12 interface enum. This board only wires up SPI - E_TWI_INTERFACE
 * is kept for API-shape parity but is not implemented.
 */
typedef enum
{
    E_TWI_INTERFACE = 0,
    E_SPI_INTERFACE = 1
} lis2dw12_interface_t;

/**
 * Operating-mode enum. Kept as an enum (rather than dropped) so the API
 * shape matches the reference driver, but only MOTION_ONLY is implemented -
 * free-fall detection is out of scope for this port.
 */
typedef enum
{
    LIS2DW12_MODE_MOTION_ONLY = 0  /**< Activity/inactivity only. */
} lis2dw12_op_mode_t;

/**
 * Event types reported from the IMU ISR.
 */
typedef enum
{
    LIS2DW12_EVT_MOTION     = 0,  /**< Transition stationary -> moving */
    LIS2DW12_EVT_STATIONARY = 1   /**< Transition moving -> stationary (inactivity elapsed) */
} lis2dw12_event_t;

/**
 * Combined event callback from the IMU ISR.
 *
 * @note Runs in IRQ context. Keep handlers short; defer heavy work via RTOS signal.
 *
 * @param[in] evt Event that occurred.
 */
typedef void (*pfn_imu_event_callback_t)(lis2dw12_event_t evt);


/**
 * Initialise LIS2DW12 in motion-only mode.
 *
 * @param[in] interface         Bus interface - only E_SPI_INTERFACE is implemented.
 * @param[in] mode              Operating mode - only LIS2DW12_MODE_MOTION_ONLY exists.
 * @param[in] inactivity_seconds Host-configured inactivity period (1..3600 s).
 *                               The IMU's built-in ACT_SLEEP_DUR register is the
 *                               authoritative inactivity timer; no RTOS timer is used.
 * @return 0 on success, -1 on failure. On failure the caller should still operate
 *         without IMU-driven sleep - do not block primary ranging.
 */
int32_t lis2dw12_init(lis2dw12_interface_t interface,
                      lis2dw12_op_mode_t   mode,
                      uint32_t             inactivity_seconds);

/**
 * Set inactivity period in seconds.
 *
 * Clamped to [LIS2DW12_INACTIVITY_SECONDS_MIN .. LIS2DW12_INACTIVITY_SECONDS_MAX].
 * Updates the IMU's ACT_SLEEP_DUR register, which is the authoritative inactivity
 * timer. The new timeout takes effect from the next activity cycle.
 *
 * @param[in] seconds Desired inactivity period.
 * @return 0 on success, -1 on failure. The actual applied value can be read
 *         back with lis2dw12_get_inactivity_seconds().
 */
int32_t lis2dw12_set_inactivity_seconds(uint32_t seconds);

/**
 * Get the currently-applied inactivity period in seconds.
 */
uint32_t lis2dw12_get_inactivity_seconds(void);

/**
 * Clear latched LIS2DW12 interrupt source bits by reading the source registers.
 */
void lis2dw12_clear_events(void);

/**
 * Return true when a motion wake source is currently pending.
 *
 * The source registers are read as part of this call, so latched bits are
 * cleared after the check.
 */
bool lis2dw12_motion_pending(void);

/**
 * Register combined event callback and enable INT1/INT2 GPIO interrupts.
 */
void lis2dw12_event_irq_enable(pfn_imu_event_callback_t cb);

/**
 * Disable INT1/INT2 GPIO interrupts and clear the callback.
 */
void lis2dw12_event_irq_disable(void);

#ifdef __cplusplus
}
#endif

#endif /* __LIS2DW12__H__ */
