/*
 * USB MSC device callbacks for MK4001MTD Bridge
 * Bridges USB Mass Storage to ATA-over-SDIO
 */

#include "tusb.h"
#include "sdio_hw.h"
#include "ata_sdio.h"
#include "led.h"
#include <stdio.h>
#include <string.h>

// Drive parameters (set after IDENTIFY)
static uint32_t drive_sectors = 0;
static bool drive_ready = false;

// Activity tracking for idle standby
static volatile uint32_t last_activity_ms = 0;
static volatile bool drive_spinning = true;
static volatile bool idle_tracking_active = false;

void msc_touch_activity(void) {
    last_activity_ms = to_ms_since_boot(get_absolute_time());
    if (!drive_spinning) {
        printf("[PWR] Wake from standby\n");
    }
    drive_spinning = true;
    idle_tracking_active = true;
}

uint32_t msc_get_last_activity_ms(void) { return last_activity_ms; }
bool msc_is_drive_spinning(void) { return drive_spinning && idle_tracking_active; }
void msc_set_drive_stopped(void) { drive_spinning = false; }

void msc_set_drive_params(uint32_t sectors) {
    drive_sectors = sectors;
    drive_ready = true;
}

// Invoked when received SCSI_CMD_INQUIRY
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8],
                         uint8_t product_id[16], uint8_t product_rev[4]) {
    (void)lun;
    const char vid[] = "TOSHIBA ";
    const char pid[] = "MK4001MTD       ";
    const char rev[] = "0100";
    memcpy(vendor_id, vid, 8);
    memcpy(product_id, pid, 16);
    memcpy(product_rev, rev, 4);
}

// Invoked when received Test Unit Ready command
bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    (void)lun;
    return drive_ready;
}

// Invoked when received SCSI_CMD_READ_CAPACITY_10 and SCSI_CMD_READ_FORMAT_CAPACITY
void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size) {
    (void)lun;
    *block_count = drive_sectors;
    *block_size = 512;
}

// Invoked when received Start Stop Unit command
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition,
                            bool start, bool load_eject) {
    (void)lun;
    (void)power_condition;
    if (load_eject) {
        if (!start) {
            // Eject — could send STANDBY_IMMEDIATE
            printf("[MSC] Eject requested\n");
        }
    }
    return true;
}

// Invoked on read10 command
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                           void *buffer, uint32_t bufsize) {
    (void)lun;
    (void)offset;

    if (!drive_ready) return -1;
    if (lba >= drive_sectors) return -1;

    uint32_t count = bufsize / 512;
    if (count == 0) count = 1;
    if (count > 255) count = 255;

    msc_touch_activity();

    // Batch reads in chunks of up to 64 sectors
    // N91 traces show drive handles 128, but 64 (32KB) balances RAM on RP2040
    led_rx_on();
    uint32_t done = 0;
    while (done < count) {
        uint32_t chunk = count - done;
        if (chunk > 64) chunk = 64;
        if (!ata_read_sectors(lba + done, (uint8_t)chunk, (uint8_t *)buffer + (done * 512))) {
            printf("[MSC] Read failed at LBA %lu\n", lba + done);
            led_rx_off();
            return -1;
        }
        done += chunk;
    }
    led_rx_off();

    return (int32_t)(count * 512);
}

// Invoked on write10 command
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                            uint8_t *buffer, uint32_t bufsize) {
    (void)lun;
    (void)offset;

    if (!drive_ready) return -1;
    if (lba >= drive_sectors) return -1;

    uint32_t count = bufsize / 512;
    if (count == 0) count = 1;
    if (count > 255) count = 255;

    msc_touch_activity();

    // Batch writes in chunks of up to 64 sectors
    led_tx_on();
    uint32_t done = 0;
    while (done < count) {
        uint32_t chunk = count - done;
        if (chunk > 64) chunk = 64;
        if (!ata_write_sectors(lba + done, (uint8_t)chunk, buffer + (done * 512))) {
            printf("[MSC] Write failed at LBA %lu\n", lba + done);
            led_tx_off();
            return -1;
        }
        done += chunk;
    }
    led_tx_off();

    return (int32_t)(count * 512);
}

// Invoked when received SCSI command not handled above
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16],
                         void *buffer, uint16_t bufsize) {
    (void)lun;
    (void)buffer;
    (void)bufsize;

    printf("[MSC] Unhandled SCSI cmd: 0x%02X\n", scsi_cmd[0]);

    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    return -1;
}

// Required: is writable
bool tud_msc_is_writable_cb(uint8_t lun) {
    (void)lun;
    return true;
}
