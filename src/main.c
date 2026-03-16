/*
 * MK4001MTD USB Bridge — Main
 * SDIO-ATA Microdrive → USB Mass Storage (all PIO, no bit-bang)
 *
 * Boot sequence:
 *   1. Startup pre-delay with LED wave animation
 *   2. Init USB + PIO
 *   3. Power cycle HDD, poll SDIO init via PIO
 *   4. ATA IDENTIFY + SMART diagnostics
 *   5. Main loop: tud_task() + idle standby
 *
 * UART debug on GP12(TX)/GP13(RX) via UART0 @ 115200
 */

#include "pico/stdlib.h"
#include "tusb.h"
#include "sdio_hw.h"
#include "sdio_pio.h"
#include "ata_sdio.h"
#include "msc_device.h"
#include "led.h"
#include <stdio.h>
#include <string.h>

// Timing
#define IDLE_STANDBY_MS     5000u
#define USB_PRE_DELAY_MS    5000u   // boot delay before presenting USB to the host
#define POWER_OFF_MS        500u    // initial power-off duration
#define SPINUP_MS           1500u   // initial spin-up wait
#define RETRY_POWER_OFF_MS  3000u   // retry: longer power-off
#define RETRY_SPINUP_MS     12000u  // retry: longer spin-up

// PIO clock dividers
#define PIO_SLOW_CLKDIV    62.5f   // ~500 KHz for SDIO card init
#define PIO_FAST_CLKDIV     3.125f // ~10 MHz for runtime

#define DRIVE_WARMUP_SECTORS 16u

static uint8_t identify_buf[512];

static uint16_t id_word(const uint8_t *buf, uint8_t w) {
    return (uint16_t)buf[w * 2] | ((uint16_t)buf[w * 2 + 1] << 8);
}

static void print_banner(void) {
    printf("\n\n========================================\n");
    printf("  MK4001MTD USB Bridge v0.11\n");
    printf("  SDIO-ATA → USB Mass Storage (PIO)\n");
    printf("========================================\n\n");
}

static void startup_wave_frame(uint32_t frame) {
    led_tx_off();
    led_rx_off();
    led_hdd_unhealthy();
    led_hdd_power_off();

    switch (frame % 7u) {
        case 0:
            led_tx_on();
            break;
        case 1:
            led_tx_on();
            led_rx_on();
            break;
        case 2:
            led_rx_on();
            break;
        case 3:
            led_rx_on();
            led_hdd_healthy();
            break;
        case 4:
            led_hdd_healthy();
            break;
        case 5:
            led_hdd_healthy();
            led_hdd_power_on();
            break;
        default:
            led_hdd_power_on();
            break;
    }
}

static void startup_predelay(uint32_t ms) {
    uint32_t frame = 0;
    uint32_t elapsed = 0;
    while (elapsed < ms) {
        uint32_t step_ms = ((ms - elapsed) >= 80u) ? 80u : (ms - elapsed);
        startup_wave_frame(frame++);
        sleep_ms(step_ms);
        elapsed += step_ms;
    }

    led_tx_off();
    led_rx_off();
    led_hdd_unhealthy();
    led_hdd_power_off();
}

// Sleep while servicing USB stack (avoids enumeration timeouts)
static void sleep_with_usb(uint32_t ms) {
    for (uint32_t i = 0; i < ms / 10; i++) {
        sleep_ms(10);
        tud_task();
    }
}

static void power_cycle_drive(uint32_t off_ms, uint32_t spinup_ms) {
    led_hdd_power_off();
    hdd_power_off();
    sleep_with_usb(off_ms);
    hdd_power_on();
    led_hdd_power_on();
    sleep_with_usb(spinup_ms);
}

// Poll PIO SDIO card init until it succeeds or timeout
static bool sdio_init_poll(uint32_t timeout_ms) {
    printf("[MAIN] SDIO init (PIO)...\n");
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start) < timeout_ms) {
        sdio_pio_acquire_pins();
        sdio_pio_set_clkdiv(PIO_SLOW_CLKDIV);
        if (sdio_pio_card_init()) return true;
        sleep_with_usb(100);
    }
    printf("[MAIN] SDIO init failed after %lums\n", timeout_ms);
    return false;
}

static void log_identify(const uint8_t *buf) {
    printf("IDENTIFY: W0=%04X W47=%04X W49=%04X W59=%04X\n",
           id_word(buf, 0), id_word(buf, 47), id_word(buf, 49), id_word(buf, 59));
    printf("  ATA W80=%04X  Cmd W82=%04X W83=%04X W84=%04X\n",
           id_word(buf, 80), id_word(buf, 82), id_word(buf, 83), id_word(buf, 84));
    printf("  En  W85=%04X W86=%04X W87=%04X  W89=%04X W128=%04X\n",
           id_word(buf, 85), id_word(buf, 86), id_word(buf, 87),
           id_word(buf, 89), id_word(buf, 128));
}

static bool init_drive(void) {
    // First attempt
    if (!sdio_init_poll(10000)) {
        printf("[MAIN] Retrying with long power cycle...\n");
        power_cycle_drive(RETRY_POWER_OFF_MS, RETRY_SPINUP_MS);
        if (!sdio_init_poll(15000)) return false;
    }

    // ATA IDENTIFY
    printf("[MAIN] ATA IDENTIFY...\n");
    memset(identify_buf, 0, sizeof(identify_buf));
    if (!ata_identify(identify_buf)) {
        printf("[MAIN] IDENTIFY failed!\n");
        return false;
    }
    ata_print_identify(identify_buf);
    log_identify(identify_buf);
    ata_smart_dump();

    // Warm up: MBR + sequential reads
    uint8_t sector[512];
    if (ata_read_sectors(0, 1, sector)) {
        printf("[MAIN] MBR: %s\n",
               (sector[510] == 0x55 && sector[511] == 0xAA) ? "valid 0x55AA" : "no signature");
    }
    printf("[MAIN] Warming up...\n");
    for (uint32_t i = 0; i < DRIVE_WARMUP_SECTORS; i++) {
        ata_read_sectors(i, 1, sector);
        tud_task();
    }

    return true;
}

static void service_idle_standby(void) {
    if (!msc_is_drive_spinning()) return;

    uint32_t now = to_ms_since_boot(get_absolute_time());
    uint32_t idle = now - msc_get_last_activity_ms();
    if (idle >= IDLE_STANDBY_MS) {
        printf("[PWR] Idle %lums → STANDBY + power gate\n", idle);
        msc_power_gate();
    }
}

static void __attribute__((noreturn)) halt_blink(void) {
    led_hdd_unhealthy();
    for (;;) {
        led_hdd_power_on();  sleep_ms(300);
        led_hdd_power_off(); sleep_ms(300);
    }
}

int main(void) {
    stdio_uart_init_full(uart0, 115200, 12, 13);
    sleep_ms(100);

    print_banner();
    led_init();
    sdio_init_pins();

    // Predelay before presenting USB, with a visible startup wave across
    // write -> read -> healthy -> power LEDs.
    printf("[MAIN] Pre-delay %dms...\n", USB_PRE_DELAY_MS);
    startup_predelay(USB_PRE_DELAY_MS);

    // Present USB only after the boot pre-delay.
    tusb_init();
    sdio_pio_set_clkdiv(PIO_FAST_CLKDIV);
    sdio_pio_init();

    // Power cycle + SDIO init (all PIO)
    printf("[MAIN] Power cycling HDD...\n");
    led_hdd_unhealthy();
    power_cycle_drive(POWER_OFF_MS, SPINUP_MS);

    if (!init_drive()) {
        printf("[MAIN] Drive init FAILED. Halting.\n");
        halt_blink();
    }
    led_hdd_healthy();

    // Verify PIO path
    uint8_t status = 0;
    if (!sdio_pio_cmd52_read(1, ATA_REG_STATUS, &status)) {
        printf("[MAIN] PIO verify failed!\n");
        halt_blink();
    }
    printf("[MAIN] PIO OK, STATUS=0x%02X\n", status);

    uint32_t sectors = ata_get_capacity(identify_buf);
    printf("[MAIN] Drive: %lu sectors (%lu MB)\n", sectors, sectors / 2048);
    msc_set_drive_params(sectors);

    printf("[MAIN] Ready.\n");

    for (;;) {
        tud_task();
        service_idle_standby();
    }
}

// USB suspend → power gate
void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    if (msc_is_drive_spinning() && !msc_is_power_gated()) {
        printf("[USB] Suspend → power gate\n");
        msc_power_gate();
    } else {
        printf("[USB] Suspend (drive off)\n");
    }
}

void tud_resume_cb(void) {
    printf("[USB] Resume\n");
}
