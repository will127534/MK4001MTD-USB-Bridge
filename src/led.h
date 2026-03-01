/*
 * Activity LEDs: GP9 = TX (write), GP10 = RX (read)
 * Active-high (LED on when GPIO high)
 */
#ifndef LED_H
#define LED_H

#include "pico/stdlib.h"

#define LED_TX_PIN 9
#define LED_RX_PIN 10

static inline void led_init(void) {
    gpio_init(LED_TX_PIN);
    gpio_set_dir(LED_TX_PIN, GPIO_OUT);
    gpio_put(LED_TX_PIN, 0);

    gpio_init(LED_RX_PIN);
    gpio_set_dir(LED_RX_PIN, GPIO_OUT);
    gpio_put(LED_RX_PIN, 0);
}

static inline void led_tx_on(void)  { gpio_put(LED_TX_PIN, 1); }
static inline void led_tx_off(void) { gpio_put(LED_TX_PIN, 0); }
static inline void led_rx_on(void)  { gpio_put(LED_RX_PIN, 1); }
static inline void led_rx_off(void) { gpio_put(LED_RX_PIN, 0); }

#endif
