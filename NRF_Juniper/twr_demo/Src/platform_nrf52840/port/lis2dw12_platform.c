/**
 * @file      lis2dw12_platform.c
 *
 * @brief     nRF52840 platform glue (SPI) for the ST LIS2DW12 accelerometer
 *            register driver. Uses a dedicated SPI instance, separate from
 *            the DW3000's (SPIM3), so both chips can be accessed
 *            independently.
 *
 * @attention Copyright 2019-2020 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 */

#include <string.h>
#include "port.h"
#include "lis2dw12_platform.h"
#include "nrf_drv_spi.h"

/* Dedicated SPI instance for the LIS2DW12 - distinct from the DW3000's SPIM3. */
#define LIS2DW12_SPI_INSTANCE   2
static const nrf_drv_spi_t gLis2dw12Spi = NRF_DRV_SPI_INSTANCE(LIS2DW12_SPI_INSTANCE);
static int gLis2dw12SpiReady = 0;

/* ST SPI register-address convention: bit7 = read(1)/write(0). */
static int32_t lis2dw12_platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    uint8_t tx[1 + 32];

    if (len > sizeof(tx) - 1)
    {
        return -1;
    }

    tx[0] = reg & 0x7F;
    memcpy(&tx[1], bufp, len);

    UNUSED(handle);
    return (nrf_drv_spi_transfer(&gLis2dw12Spi, tx, (uint8_t)(1 + len), NULL, 0) == NRF_SUCCESS) ? 0 : -1;
}

static int32_t lis2dw12_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    uint8_t tx[1 + 32];
    uint8_t rx[1 + 32];

    if (len > sizeof(tx) - 1)
    {
        return -1;
    }

    memset(tx, 0xFF, sizeof(tx));
    tx[0] = reg | 0x80;

    UNUSED(handle);
    if (nrf_drv_spi_transfer(&gLis2dw12Spi, tx, (uint8_t)(1 + len), rx, (uint8_t)(1 + len)) != NRF_SUCCESS)
    {
        return -1;
    }

    memcpy(bufp, &rx[1], len);
    return 0;
}

void lis2dw12_platform_spi_init(stmdev_ctx_t *ctx)
{
    if (!gLis2dw12SpiReady)
    {
        nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

        spi_config.sck_pin      = LIS2DW12_SCL_Pin;
        spi_config.mosi_pin     = LIS2DW12_SDI_Pin;
        spi_config.miso_pin     = LIS2DW12_SDO_Pin;
        spi_config.ss_pin       = LIS2DW12_CS_Pin;   /* nrf_drv_spi drives CS automatically per transfer */
        spi_config.frequency    = NRF_DRV_SPI_FREQ_1M;
        spi_config.mode         = NRF_DRV_SPI_MODE_0;
        spi_config.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

        APP_ERROR_CHECK(nrf_drv_spi_init(&gLis2dw12Spi, &spi_config, NULL, NULL));
        gLis2dw12SpiReady = 1;
    }

    ctx->write_reg = lis2dw12_platform_write;
    ctx->read_reg  = lis2dw12_platform_read;
    ctx->handle    = NULL;
}
