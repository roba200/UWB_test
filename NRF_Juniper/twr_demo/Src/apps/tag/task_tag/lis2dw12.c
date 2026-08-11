/**
 * @file      lis2dw12.c
 * @brief     Interrupt-driven LIS2DW12 motion/inactivity driver for TAG.
 *            See lis2dw12.h for the design summary.
 *
 * @attention Copyright 2019-2020 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 */

#include <string.h>
#include "port.h"
#include "lis2dw12.h"
#include "lis2dw12_platform.h"
#include "lis2dw12_reg.h"
#include "nrf_drv_gpiote.h"


/* ---- State --------------------------------------------------------------- */

static bool                      s_inited          = false;
static stmdev_ctx_t              s_ctx;
static uint32_t                  s_inactivity_s    = LIS2DW12_DEFAULT_INACTIVITY_SECONDS;
static pfn_imu_event_callback_t  s_event_cb        = NULL;
static bool                      s_irq_gpiote_init = false;


/* ---- Internal helpers ----------------------------------------------------- */

/* Convert inactivity seconds -> act_sleep_dur register value.
 * Register is 8 bits, 1 LSB = 512/ODR seconds. Clamp to 1..255. */
static uint8_t s_seconds_to_act_sleep_dur(uint32_t seconds, uint16_t odr_hz)
{
    uint32_t reg;

    if (seconds < LIS2DW12_INACTIVITY_SECONDS_MIN) seconds = LIS2DW12_INACTIVITY_SECONDS_MIN;
    if (seconds > LIS2DW12_INACTIVITY_SECONDS_MAX) seconds = LIS2DW12_INACTIVITY_SECONDS_MAX;

    reg = ((uint32_t)seconds * (uint32_t)odr_hz + 256U) / 512U;
    if (reg < 1U)   reg = 1U;
    if (reg > 255U) reg = 255U;
    return (uint8_t)reg;
}

/* INT1/INT2 ISR: decode which event fired by reading source registers. */
static void s_lis2dw12_int_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    lis2dw12_all_sources_t all_source;

    UNUSED(pin);
    UNUSED(action);

    if (!s_inited || s_event_cb == NULL) return;

    memset(&all_source, 0, sizeof(all_source));
    if (lis2dw12_all_sources_get(&s_ctx, &all_source) != 0)
    {
        /* bus error while reading source reg - nothing actionable in ISR */
        return;
    }

    /* If motion and sleep-state bits are both visible, treat it as wake.
     * Otherwise a stale sleep-state bit can immediately stop ranging again. */
    if (all_source.wake_up_src.wu_ia)
    {
        s_event_cb(LIS2DW12_EVT_MOTION);
    }
    else if (all_source.wake_up_src.sleep_state_ia ||
             all_source.status_dup.sleep_state_ia ||
             all_source.all_int_src.sleep_change_ia)
    {
        s_event_cb(LIS2DW12_EVT_STATIONARY);
    }
}

/* Apply the full register set. Assumes chip has been reset and bus is up. */
static int32_t s_apply_config(uint32_t inactivity_seconds)
{
    stmdev_ctx_t *ctx = &s_ctx;
    lis2dw12_ctrl4_int1_pad_ctrl_t int1_route;
    lis2dw12_ctrl5_int2_pad_ctrl_t int2_route;
    uint8_t act_sleep_reg;

    const uint16_t odr_hz = LIS2DW12_ODR_MOTION_ONLY_HZ;

    /* Block Data Update ON: keeps LSB/MSB coherent for any reads that still happen. */
    if (lis2dw12_block_data_update_set(ctx, PROPERTY_ENABLE) != 0) return -1;

    /* Full scale +/- 2g (threshold LSB = FS/64 = 31.25 mg). */
    if (lis2dw12_full_scale_set(ctx, LIS2DW12_2g) != 0) return -1;

    /* Filter: LP on output of wake-up path, ODR/2 bandwidth. */
    if (lis2dw12_filter_path_set(ctx, LIS2DW12_LPF_ON_OUT) != 0) return -1;
    if (lis2dw12_filter_bandwidth_set(ctx, LIS2DW12_ODR_DIV_2) != 0) return -1;

    /* Power mode: high-performance low-noise. Costs more current than a
     * low-power mode, but gives 14-bit resolution and the lowest noise
     * floor, which matters for reliably detecting small/gentle motion. */
    if (lis2dw12_power_mode_set(ctx, LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE) != 0) return -1;

    /* Wake-up duration (LSB = 1/ODR). Debounces short spikes. */
    if (lis2dw12_wkup_dur_set(ctx, LIS2DW12_WAKE_DURATION_SAMPLES) != 0) return -1;

    /* Activity (wake) threshold. */
    if (lis2dw12_wkup_threshold_set(ctx, LIS2DW12_WAKE_THRESHOLD_LSB) != 0) return -1;

    /* Feed HP-filtered data to the wake-up detector. */
    if (lis2dw12_wkup_feed_data_set(ctx, LIS2DW12_HP_FEED) != 0) return -1;

    /* Inactivity (sleep) duration, converted from seconds at the current ODR. */
    act_sleep_reg = s_seconds_to_act_sleep_dur(inactivity_seconds, odr_hz);
    if (lis2dw12_act_sleep_dur_set(ctx, act_sleep_reg) != 0) return -1;
    if (inactivity_seconds < LIS2DW12_INACTIVITY_SECONDS_MIN) inactivity_seconds = LIS2DW12_INACTIVITY_SECONDS_MIN;
    if (inactivity_seconds > LIS2DW12_INACTIVITY_SECONDS_MAX) inactivity_seconds = LIS2DW12_INACTIVITY_SECONDS_MAX;
    s_inactivity_s = inactivity_seconds;

    /* Activity/inactivity detection engine ON. */
    if (lis2dw12_act_mode_set(ctx, LIS2DW12_DETECT_ACT_INACT) != 0) return -1;

    /* Latch interrupt pins until the source register is read. This also makes
     * motion on INT1 reliable as a System OFF wake source. */
    if (lis2dw12_int_notification_set(ctx, LIS2DW12_INT_LATCHED) != 0) return -1;

    /* Route motion (wake-up) to INT1. */
    memset(&int1_route, 0, sizeof(int1_route));
    if (lis2dw12_pin_int1_route_get(ctx, &int1_route) != 0) return -1;
    int1_route.int1_wu = PROPERTY_ENABLE;
    if (lis2dw12_pin_int1_route_set(ctx, &int1_route) != 0) return -1;

    /* Route inactivity/sleep-state to INT2. INT1 does not expose these bits in
     * CTRL4, so relying on INT1 alone prevents the nRF52 from ever receiving
     * the "go to System OFF" trigger. */
    memset(&int2_route, 0, sizeof(int2_route));
    if (lis2dw12_pin_int2_route_get(ctx, &int2_route) != 0) return -1;
    int2_route.int2_sleep_chg   = PROPERTY_ENABLE;
    int2_route.int2_sleep_state = PROPERTY_ENABLE;
    if (lis2dw12_pin_int2_route_set(ctx, &int2_route) != 0) return -1;

    /* Finally, set ODR. ODR must be set last because threshold/duration
     * registers' physical meaning depends on ODR. */
    if (lis2dw12_data_rate_set(ctx, LIS2DW12_XL_ODR_100Hz) != 0) return -1;

    return 0;
}


/* ---- Public API ------------------------------------------------------- */

int32_t lis2dw12_init(lis2dw12_interface_t interface,
                      lis2dw12_op_mode_t   mode,
                      uint32_t             inactivity_seconds)
{
    uint8_t whoami = 0;
    uint8_t rst;
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);

    UNUSED(interface); /* this board is SPI-only */
    UNUSED(mode);       /* only LIS2DW12_MODE_MOTION_ONLY exists */

    s_inited = false;

    lis2dw12_platform_spi_init(&s_ctx);

    lis2dw12_device_id_get(&s_ctx, &whoami);
    if (whoami != LIS2DW12_ID)
    {
        return -1;
    }

    if (!s_irq_gpiote_init)
    {
        /* GPIOTE driver itself may already be init'd (DW3000 IRQ pin) -
         * nrf_drv_gpiote_init() no-ops in that case. */
        (void)nrf_drv_gpiote_init();
        if (nrf_drv_gpiote_in_init(LIS2DW12_INT1_Pin, &config, s_lis2dw12_int_event_handler) != NRF_SUCCESS)
        {
            return -1;
        }
        if (nrf_drv_gpiote_in_init(LIS2DW12_INT2_Pin, &config, s_lis2dw12_int_event_handler) != NRF_SUCCESS)
        {
            return -1;
        }
        s_irq_gpiote_init = true;
    }

    /* Software reset, wait for completion. */
    if (lis2dw12_reset_set(&s_ctx, PROPERTY_ENABLE) != 0) return -1;
    do
    {
        if (lis2dw12_reset_get(&s_ctx, &rst) != 0) return -1;
    } while (rst);

    if (s_apply_config(inactivity_seconds) != 0)
    {
        return -1;
    }

    s_inited = true;
    return 0;
}

int32_t lis2dw12_set_inactivity_seconds(uint32_t seconds)
{
    uint8_t reg;

    if (!s_inited) return -1;

    reg = s_seconds_to_act_sleep_dur(seconds, LIS2DW12_ODR_MOTION_ONLY_HZ);
    if (lis2dw12_act_sleep_dur_set(&s_ctx, reg) != 0) return -1;

    if (seconds < LIS2DW12_INACTIVITY_SECONDS_MIN) seconds = LIS2DW12_INACTIVITY_SECONDS_MIN;
    if (seconds > LIS2DW12_INACTIVITY_SECONDS_MAX) seconds = LIS2DW12_INACTIVITY_SECONDS_MAX;
    s_inactivity_s = seconds;
    return 0;
}

uint32_t lis2dw12_get_inactivity_seconds(void)
{
    return s_inactivity_s;
}

void lis2dw12_clear_events(void)
{
    lis2dw12_all_sources_t all_source;

    if (!s_inited) return;

    memset(&all_source, 0, sizeof(all_source));
    (void)lis2dw12_all_sources_get(&s_ctx, &all_source);
}

bool lis2dw12_motion_pending(void)
{
    lis2dw12_all_sources_t all_source;

    if (!s_inited) return false;

    memset(&all_source, 0, sizeof(all_source));
    if (lis2dw12_all_sources_get(&s_ctx, &all_source) != 0)
    {
        return false;
    }

    return (all_source.wake_up_src.wu_ia || all_source.all_int_src.wu_ia) ? true : false;
}

void lis2dw12_event_irq_enable(pfn_imu_event_callback_t cb)
{
    s_event_cb = cb;
    nrf_drv_gpiote_in_event_enable(LIS2DW12_INT1_Pin, true);
    nrf_drv_gpiote_in_event_enable(LIS2DW12_INT2_Pin, true);
}

void lis2dw12_event_irq_disable(void)
{
    nrf_drv_gpiote_in_event_disable(LIS2DW12_INT1_Pin);
    nrf_drv_gpiote_in_event_disable(LIS2DW12_INT2_Pin);
    s_event_cb = NULL;
}
