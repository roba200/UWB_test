/*
 * Minimal, standalone System OFF current-draw test for nRF52840.
 *
 * No DW3000, no LIS2DW12, no FreeRTOS, no BSP - just this board's two LED
 * pins (from twr_demo's custom_board.h) and the raw SYSTEMOFF register
 * write. Purpose: find out whether THIS PHYSICAL BOARD is capable of
 * reaching genuine low-power System OFF at all, independent of every piece
 * of application firmware in twr_demo.
 *
 * Boot behaviour: light both LEDs for 3 seconds (visible proof the app is
 * running), turn them off, then enter System OFF. Nothing wakes it back up
 * on purpose - power-cycle the board to run it again.
 */
#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

/* Same pins as twr_demo's custom_board.h: LED_1 = P0.28, LED_2 = P0.21,
 * active-high. */
#define LED_1   NRF_GPIO_PIN_MAP(0, 28)
#define LED_2   NRF_GPIO_PIN_MAP(0, 21)

int main(void)
{
    nrf_gpio_cfg_output(LED_1);
    nrf_gpio_cfg_output(LED_2);

    nrf_gpio_pin_set(LED_1);
    nrf_gpio_pin_set(LED_2);
    nrf_delay_ms(3000);

    nrf_gpio_pin_clear(LED_1);
    nrf_gpio_pin_clear(LED_2);

    /* Give the pin writes time to settle before cutting power. */
    nrf_delay_ms(50);

    NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
    __DSB();

    /* Should never be reached on real hardware with no debugger attached -
     * if it is, System OFF did not actually take effect. */
    while (true)
    {
        __WFE();
    }
}
