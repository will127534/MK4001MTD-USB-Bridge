/*
 * MK4001MTD USB Bridge - Main
 * SDIO-ATA Microdrive → USB Mass Storage
 *
 * UART debug on GP16/GP17 @ 115200
 */

#include "pico/stdlib.h"
#include "tusb.h"
#include "sdio_hw.h"
#include "sdio_pio.h"
#include "ata_sdio.h"
#include "led.h"
#include <stdio.h>
#include <string.h>

// Defined in msc_device.c
extern void msc_set_drive_params(uint32_t sectors);
extern void msc_touch_activity(void);
extern uint32_t msc_get_last_activity_ms(void);
extern bool msc_is_drive_spinning(void);
extern void msc_set_drive_stopped(void);

#define IDLE_STANDBY_MS 1000  // 1 second idle → STANDBY IMMEDIATE

static uint8_t identify_buf[512];

static bool init_drive(void) {
    printf("[MAIN] Starting SDIO init...\n");
    if (!sdio_card_init()) {
        printf("[MAIN] SDIO init failed, retrying after power cycle...\n");
        hdd_power_off();
        sleep_ms(3000);
        hdd_power_on();
        sleep_ms(12000);
        if (!sdio_card_init()) {
            printf("[MAIN] SDIO init failed again!\n");
            return false;
        }
    }

    printf("[MAIN] ATA IDENTIFY...\n");
    memset(identify_buf, 0, sizeof(identify_buf));
    if (!ata_identify(identify_buf)) {
        printf("[MAIN] IDENTIFY failed!\n");
        return false;
    }

    ata_print_identify(identify_buf);

    // Key IDENTIFY words
    {
        uint16_t w0  = identify_buf[0]   | (identify_buf[1]   << 8);
        uint16_t w47 = identify_buf[94]  | (identify_buf[95]  << 8);
        uint16_t w49 = identify_buf[98]  | (identify_buf[99]  << 8);
        uint16_t w59 = identify_buf[118] | (identify_buf[119] << 8);
        uint16_t w80 = identify_buf[160] | (identify_buf[161] << 8);
        uint16_t w82 = identify_buf[164] | (identify_buf[165] << 8);
        uint16_t w83 = identify_buf[166] | (identify_buf[167] << 8);
        uint16_t w84 = identify_buf[168] | (identify_buf[169] << 8);
        uint16_t w85 = identify_buf[170] | (identify_buf[171] << 8);
        uint16_t w86 = identify_buf[172] | (identify_buf[173] << 8);
        uint16_t w87 = identify_buf[174] | (identify_buf[175] << 8);
        uint16_t w89 = identify_buf[178] | (identify_buf[179] << 8);
        uint16_t w128 = identify_buf[256] | (identify_buf[257] << 8);
        printf("IDENTIFY: W0=%04X W47=%04X W49=%04X W59=%04X\n", w0, w47, w49, w59);
        printf("  ATA ver W80=%04X  Cmd set W82=%04X W83=%04X W84=%04X\n", w80, w82, w83, w84);
        printf("  Enabled W85=%04X W86=%04X W87=%04X  W89=%04X W128=%04X\n", w85, w86, w87, w89, w128);
    }

    // Dump SMART / vendor diagnostics
    ata_smart_dump();

    // Quick sector 0 test + warm-up reads
    uint8_t mbr[512];
    if (ata_read_sectors(0, 1, mbr)) {
        printf("[MAIN] MBR: %s\n",
               (mbr[510] == 0x55 && mbr[511] == 0xAA) ? "valid 0x55AA" : "no signature");
    }

    // Warm-up: read a few sectors to let heads settle
    printf("[MAIN] Warming up drive...\n");
    for (int i = 0; i < 16; i++) {
        ata_read_sectors(i, 1, mbr);
    }
    printf("[MAIN] Drive ready.\n");

    return true;
}

int main() {
    // UART on GP16(TX)/GP17(RX)
    stdio_uart_init_full(uart0, 115200, 16, 17);
    sleep_ms(100);

    printf("\n\n========================================\n");
    printf("  MK4001MTD USB Bridge v0.9\n");
    printf("  SDIO-ATA → USB Mass Storage\n");
    printf("========================================\n\n");

    led_init();
    sdio_init_pins();

    // Power cycle for clean start
    printf("[MAIN] Power cycling HDD...\n");
    hdd_power_off();
    sleep_ms(500);
    hdd_power_on();
    sleep_ms(200);  // 500ms spin-up

    if (!init_drive()) {
        printf("[MAIN] Drive init FAILED. Halting.\n");
        while (1) tight_loop_contents();
    }

    // Switch to PIO for fast SDIO
    printf("[MAIN] Switching to PIO SDIO...\n");
    sdio_pio_set_clkdiv(6.25f);  // ~5 MHz
    sdio_pio_init();

    // PIO verification
    uint8_t ptest = 0;
    if (sdio_pio_cmd52_read(1, ATA_REG_STATUS, &ptest))
        printf("[PIO] CMD52 OK: STATUS=0x%02X\n", ptest);
    else
        printf("[PIO] CMD52 FAILED — halting\n");

    // Get capacity and enable USB MSC
    uint32_t sectors = ata_get_capacity(identify_buf);
    printf("[MAIN] Drive has %lu sectors (%lu MB)\n", sectors, sectors / 2048);
    msc_set_drive_params(sectors);

    // Init USB
    printf("[MAIN] Starting USB MSC...\n");
    tusb_init();

    printf("[MAIN] USB MSC active. Ready for host.\n");

    // Main loop: service USB + idle standby
    while (1) {
        tud_task();

        // Check idle timeout for drive standby
        if (msc_is_drive_spinning()) {
            uint32_t now = to_ms_since_boot(get_absolute_time());
            uint32_t idle = now - msc_get_last_activity_ms();
            if (idle >= IDLE_STANDBY_MS) {
                printf("[PWR] Idle %lums → STANDBY\n", idle);
                ata_standby_immediate();
                msc_set_drive_stopped();
            }
        }
    }

    return 0;
}
