/*
 * LED indicators:
 *   GP16 = HDD Power (on when HDD powered)
 *   GP17 = HDD Healthy (on when drive init OK)
 *   GP18 = Read activity
 *   GP19 = Write activity
 * Active-low (GPIO high disables the LED)
 */
#ifndef LED_H
#define LED_H

#include "pico/stdlib.h"

#define LED_HDD_POWER_PIN  16
#define LED_HDD_HEALTH_PIN 17
#define LED_RX_PIN         18
#define LED_TX_PIN         19

static inline void led_init(void) {
    const uint pins[] = { LED_HDD_POWER_PIN, LED_HDD_HEALTH_PIN, LED_RX_PIN, LED_TX_PIN };
    for (int i = 0; i < 4; i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
        gpio_put(pins[i], 1);  // Default off.
    }
}

static inline void led_hdd_power_on(void)  { gpio_put(LED_HDD_POWER_PIN, 0); }
static inline void led_hdd_power_off(void) { gpio_put(LED_HDD_POWER_PIN, 1); }
static inline void led_hdd_healthy(void)   { gpio_put(LED_HDD_HEALTH_PIN, 0); }
static inline void led_hdd_unhealthy(void) { gpio_put(LED_HDD_HEALTH_PIN, 1); }
static inline void led_tx_on(void)  { gpio_put(LED_TX_PIN, 0); }
static inline void led_tx_off(void) { gpio_put(LED_TX_PIN, 1); }
static inline void led_rx_on(void)  { gpio_put(LED_RX_PIN, 0); }
static inline void led_rx_off(void) { gpio_put(LED_RX_PIN, 1); }

#endif
