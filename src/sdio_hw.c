/*
 * SDIO Hardware — GPIO pin init and HDD power control.
 *
 * All SDIO communication now uses PIO (sdio_pio.c).
 * This file only handles pin initialization and HDD power gating.
 *
 * The original bit-bang SDIO implementation is preserved in git history
 * (commit 5f52fd7 and earlier) if ever needed for debugging.
 */

#include "sdio_hw.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>

void sdio_init_pins(void) {
    gpio_init(SDIO_CLK_PIN);
    gpio_set_dir(SDIO_CLK_PIN, GPIO_OUT);
    gpio_put(SDIO_CLK_PIN, 0);

    gpio_init(SDIO_CMD_PIN);
    gpio_set_dir(SDIO_CMD_PIN, GPIO_OUT);
    gpio_put(SDIO_CMD_PIN, 1);
    gpio_pull_up(SDIO_CMD_PIN);

    for (int i = 0; i < 4; i++) {
        gpio_init(SDIO_DAT0_PIN + i);
        gpio_set_dir(SDIO_DAT0_PIN + i, GPIO_IN);
        gpio_pull_up(SDIO_DAT0_PIN + i);
    }

    gpio_init(HDD_EN_PIN);
    gpio_set_dir(HDD_EN_PIN, GPIO_OUT);
    gpio_put(HDD_EN_PIN, 0);  // Off until boot sequence starts
}

void hdd_power_on(void) {
    gpio_put(HDD_EN_PIN, 1);
    printf("[SDIO] HDD power ON\n");
}

void hdd_power_off(void) {
    gpio_put(HDD_EN_PIN, 0);
    printf("[SDIO] HDD power OFF\n");
}
