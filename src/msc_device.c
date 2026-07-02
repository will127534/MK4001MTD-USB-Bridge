/*
 * USB MSC device callbacks for MK4001MTD Bridge
 * Bridges USB Mass Storage to ATA-over-SDIO
 */

#include "tusb.h"
#include "sdio_hw.h"    // hdd_power_on/off, ATA register defines
#include "sdio_pio.h"
#include "ata_sdio.h"
#include "msc_device.h"
#include "led.h"
#include <stdio.h>
#include <string.h>

#define MSC_BLOCK_SIZE 512u
#define MSC_MAX_ATA_SECTORS 255u
#define MSC_CHUNK_SECTORS 64u

// Drive parameters (set after IDENTIFY)
static uint32_t drive_sectors = 0;
static bool drive_ready = false;

// ---- Bad-sector cache ----
// Remembers LBA ranges that failed so repeat reads return zeros instantly
// instead of hammering the drive for 15+ seconds each time.
#define BAD_CACHE_SLOTS 16u

typedef struct {
    uint32_t lba_start;
    uint32_t lba_end;   // inclusive
    bool     active;
} bad_range_t;

static bad_range_t bad_cache[BAD_CACHE_SLOTS];
static uint32_t bad_cache_count = 0;

static void bad_cache_add(uint32_t lba) {
    // Try to extend an existing range
    for (uint32_t i = 0; i < bad_cache_count; i++) {
        if (!bad_cache[i].active) continue;
        if (lba >= bad_cache[i].lba_start && lba <= bad_cache[i].lba_end) return; // already cached
        if (lba == bad_cache[i].lba_end + 1) { bad_cache[i].lba_end = lba; return; }
        if (lba + 1 == bad_cache[i].lba_start) { bad_cache[i].lba_start = lba; return; }
    }
    // Reuse an inactive slot, else append
    for (uint32_t i = 0; i < bad_cache_count; i++) {
        if (!bad_cache[i].active) {
            bad_cache[i] = (bad_range_t){ lba, lba, true };
            return;
        }
    }
    if (bad_cache_count < BAD_CACHE_SLOTS) {
        bad_cache[bad_cache_count++] = (bad_range_t){ lba, lba, true };
    }
}

// A successful write to a cached-bad LBA clears it (the sector was repaired
// or rewritten cleanly) — like a drive clearing a pending-reallocation sector.
static void bad_cache_remove(uint32_t lba) {
    for (uint32_t i = 0; i < bad_cache_count; i++) {
        if (!bad_cache[i].active) continue;
        if (lba < bad_cache[i].lba_start || lba > bad_cache[i].lba_end) continue;

        if (bad_cache[i].lba_start == bad_cache[i].lba_end) {
            bad_cache[i].active = false;
        } else if (lba == bad_cache[i].lba_start) {
            bad_cache[i].lba_start++;
        } else if (lba == bad_cache[i].lba_end) {
            bad_cache[i].lba_end--;
        } else {
            // Split: keep the low half, put the high half in a free slot.
            // If no slot is free, leave the range intact (errs toward caution).
            for (uint32_t j = 0; j < BAD_CACHE_SLOTS; j++) {
                bool free_slot = (j >= bad_cache_count) || !bad_cache[j].active;
                if (free_slot) {
                    if (j >= bad_cache_count) bad_cache_count = j + 1;
                    bad_cache[j] = (bad_range_t){ lba + 1, bad_cache[i].lba_end, true };
                    bad_cache[i].lba_end = lba - 1;
                    break;
                }
            }
        }
        return;
    }
}

static bool bad_cache_hit(uint32_t lba) {
    for (uint32_t i = 0; i < bad_cache_count; i++) {
        if (bad_cache[i].active && lba >= bad_cache[i].lba_start && lba <= bad_cache[i].lba_end)
            return true;
    }
    return false;
}

// Check if ANY sector in [lba, lba+count) is cached as bad
static bool bad_cache_overlaps(uint32_t lba, uint32_t count) {
    for (uint32_t i = 0; i < bad_cache_count; i++) {
        if (!bad_cache[i].active) continue;
        if (lba + count > bad_cache[i].lba_start && lba <= bad_cache[i].lba_end)
            return true;
    }
    return false;
}

// ---- Sequential read prefetch ----
// TinyUSB MSC serializes "read chunk from drive" then "send chunk over USB",
// leaving the drive idle while USB transmits (the RP2040 USB controller
// streams a queued transfer from IRQ context without needing tud_task).
// When a sequential read stream is detected, the main loop prefetches the
// next chunk from the drive while the previous one is still going out over
// USB, overlapping the two instead of paying for them back-to-back.
#define LBA_NONE UINT32_MAX
static uint8_t prefetch_buf[MSC_CHUNK_SECTORS * MSC_BLOCK_SIZE] __attribute__((aligned(4)));
static uint32_t prefetch_lba = LBA_NONE;   // LBA of prefetch_buf[0]; LBA_NONE = empty
static uint32_t prefetch_want = LBA_NONE;  // armed prefetch target for the main loop
static uint32_t last_read_end = LBA_NONE;  // end LBA of the previous host read

// Writes are synchronous (write-through): the device reports no Caching mode
// page, so hosts assume write-through semantics — a write error must be a
// current error on the exact command, never deferred. That rules out
// write-behind staging on the write path.

// Activity tracking for idle standby + power gating
static volatile uint32_t last_activity_ms = 0;
static volatile bool drive_spinning = true;
static volatile bool idle_tracking_active = false;
static volatile bool power_gated = false;

#define WAKE_SLOW_CLKDIV   62.5f   // ~500 KHz for SDIO init

static void led_block_issue_flash(void) {
    led_hdd_unhealthy();
    sleep_ms(80);
    led_hdd_healthy();
}

// Synchronous wake — called from MSC callbacks, uses tud_task() to keep USB alive
static bool wake_from_power_gate(const char *reason) {
    printf("[PWR] Wake (%s): powering on HDD...\n", reason);
    hdd_power_on();
    led_hdd_power_on();

    // Brief settling delay for HDD power rail + SDIO controller
    sleep_ms(10);
    tud_task();

    // Poll SDIO init — reset PIO state each attempt for clean slate
    printf("[PWR] Wake: waiting for SDIO...\n");
    bool inited = false;
    sdio_pio_acquire_pins();
    sdio_pio_set_clkdiv(WAKE_SLOW_CLKDIV);
    if (sdio_pio_try_warm_probe()) {
        inited = true;
    }
    for (int attempt = 0; attempt < 100; attempt++) {  // up to ~10s
        if (inited) break;
        tud_task();
        sdio_pio_acquire_pins();
        sdio_pio_set_clkdiv(WAKE_SLOW_CLKDIV);
        if (sdio_pio_card_init()) { inited = true; break; }
        sleep_ms(100);
    }
    if (!inited) {
        printf("[PWR] Wake: SDIO init FAILED\n");
        led_hdd_unhealthy();
        return false;
    }

    // N91-style: poll ATA STATUS for 30ms until drive reports DRDY
    // (drive may need a moment after SDIO init to be ATA-ready)
    uint8_t status = 0;
    uint32_t poll_start = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - poll_start) < 30) {
        if (sdio_pio_cmd52_read(1, ATA_REG_STATUS, &status)) {
            if ((status & (ATA_STATUS_BSY | ATA_STATUS_DRDY)) == ATA_STATUS_DRDY) break;
        }
        sleep_ms(1);
    }

    printf("[PWR] Wake OK, STATUS=0x%02X (%lums)\n", status,
           to_ms_since_boot(get_absolute_time()) - poll_start);
    led_hdd_healthy();
    power_gated = false;
    drive_spinning = true;
    idle_tracking_active = true;
    last_activity_ms = to_ms_since_boot(get_absolute_time());
    return true;
}

void msc_touch_activity(const char *reason) {
    last_activity_ms = to_ms_since_boot(get_absolute_time());
    if (power_gated) {
        if (!wake_from_power_gate(reason)) {
            return;  // wake failed, caller checks power_gated
        }
    }
}

uint32_t msc_get_last_activity_ms(void) { return last_activity_ms; }
bool msc_is_drive_spinning(void) { return drive_spinning && idle_tracking_active; }
bool msc_is_power_gated(void) { return power_gated; }

void msc_power_gate(void) {
    printf("[PWR] STANDBY IMMEDIATE → power gate\n");
    ata_standby_immediate();
    sleep_ms(50);
    if (sdio_pio_deselect_card()) {
        printf("[PWR] SDIO card deselected\n");
    }
    sleep_ms(2);
    hdd_power_off();
    led_hdd_power_off();
    drive_spinning = false;
    power_gated = true;
}

void msc_set_drive_params(uint32_t sectors) {
    drive_sectors = sectors;
    drive_ready = true;
    drive_spinning = true;
    idle_tracking_active = true;
    last_activity_ms = to_ms_since_boot(get_absolute_time());
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

static bool msc_is_block_aligned(uint32_t offset, uint32_t bufsize) {
    return offset == 0 && (bufsize % MSC_BLOCK_SIZE) == 0;
}

static uint32_t msc_transfer_sector_count(uint32_t bufsize) {
    uint32_t count = bufsize / MSC_BLOCK_SIZE;
    if (count == 0) count = 1;
    if (count > MSC_MAX_ATA_SECTORS) count = MSC_MAX_ATA_SECTORS;
    return count;
}

static bool msc_transfer_in_range(uint32_t lba, uint32_t count) {
    return drive_ready && count > 0 && lba < drive_sectors && count <= (drive_sectors - lba);
}

/*
 * Sector-by-sector fallback for a failed chunk.
 * Uses fast single-sector I/O to locate the failing block, but fails the
 * SCSI command on the first bad sector instead of fabricating success.
 *
 * Reads of cached-bad LBAs fail fast (anti-hammer protection against host
 * retry storms on sectors that take seconds to time out). Writes always
 * touch the medium — per SBC a write must be attempted, and it may repair
 * the sector; success clears the LBA from the cache.
 *
 * Returns the number of sectors that failed (0 = all OK).
 */

static uint32_t msc_fallback_sectors(uint32_t lba, uint8_t *buf,
                                     uint32_t sector_count, bool is_write) {
    for (uint32_t i = 0; i < sector_count; i++) {
        uint8_t *sec_buf = buf + (i * MSC_BLOCK_SIZE);
        bool was_bad = bad_cache_hit(lba + i);

        if (!is_write && was_bad) {
            printf("[MSC] Cached bad sector read LBA=%lu\n", lba + i);
            return 1;
        }

        bool ok = is_write
            ? ata_write_sector_fast(lba + i, sec_buf)
            : ata_read_sector_fast(lba + i, sec_buf);

        if (!ok) {
            if (!was_bad) {
                printf("[MSC] BAD SECTOR %s LBA=%lu\n",
                       is_write ? "write" : "read", lba + i);
            }
            bad_cache_add(lba + i);
            ata_error_recovery();
            return 1;
        }

        if (is_write && was_bad) {
            printf("[MSC] Bad sector LBA=%lu repaired by write\n", lba + i);
            bad_cache_remove(lba + i);
        }
    }
    return 0;
}

static int32_t msc_transfer(uint32_t lba, void *buffer, uint32_t bufsize, bool is_write) {
    uint32_t count = msc_transfer_sector_count(bufsize);
    uint32_t done = 0;
    uint32_t total_bad = 0;

    if (!msc_transfer_in_range(lba, count)) {
        tud_msc_set_sense(0, SCSI_SENSE_ILLEGAL_REQUEST, 0x21, 0x00);
        return -1;
    }

    msc_touch_activity(is_write ? "WRITE" : "READ");
    if (power_gated) {
        // Wake failed — tell host to retry
        tud_msc_set_sense(0, SCSI_SENSE_NOT_READY, 0x04, 0x01);
        return -1;
    }
    if (is_write) {
        // Prefetched data may be about to go stale — drop it.
        prefetch_lba = LBA_NONE;
        prefetch_want = LBA_NONE;
        led_tx_on();
    } else {
        led_rx_on();
    }

    while (done < count) {
        uint32_t chunk = count - done;
        if (chunk > MSC_CHUNK_SECTORS) chunk = MSC_CHUNK_SECTORS;

        uint8_t *chunk_buf = (uint8_t *)buffer + (done * MSC_BLOCK_SIZE);

        // If this chunk overlaps known-bad sectors, skip straight to per-sector
        // to fail quickly with a precise medium error.
        if (bad_cache_overlaps(lba + done, chunk)) {
            uint32_t bad = msc_fallback_sectors(lba + done, chunk_buf, chunk, is_write);
            total_bad += bad;
            if (bad > 0) break;
        } else if (!is_write && prefetch_lba != LBA_NONE &&
                   lba + done >= prefetch_lba &&
                   lba + done + chunk <= prefetch_lba + MSC_CHUNK_SECTORS) {
            // Served from the prefetch buffer (already read + CRC-verified)
            memcpy(chunk_buf,
                   prefetch_buf + (lba + done - prefetch_lba) * MSC_BLOCK_SIZE,
                   chunk * MSC_BLOCK_SIZE);
        } else {
            bool ok = is_write
                ? ata_write_sectors(lba + done, (uint8_t)chunk, chunk_buf)
                : ata_read_sectors(lba + done, (uint8_t)chunk, chunk_buf);
            if (!ok) {
                printf("[MSC] Chunk %s failed at LBA %lu+%lu, falling back to per-sector\n",
                       is_write ? "write" : "read", lba + done, chunk);
                ata_error_recovery();
                uint32_t bad = msc_fallback_sectors(lba + done, chunk_buf, chunk, is_write);
                total_bad += bad;
                if (bad > 0) break;
            }
        }

        done += chunk;
    }

    if (is_write) {
        led_tx_off();
    } else {
        led_rx_off();
    }

    if (total_bad > 0) {
        printf("[MSC] Transfer LBA=%lu count=%lu failed with %lu bad sector(s)\n",
               lba, count, total_bad);
        led_block_issue_flash();
    }

    // Standards-style block device behavior: any unrecovered block error fails
    // the SCSI command, instead of returning synthetic zero-filled data.
    if (total_bad > 0) {
        if (is_write) {
            tud_msc_set_sense(0, 0x03, 0x0C, 0x00);  // MEDIUM ERROR, Write Error
        } else {
            tud_msc_set_sense(0, 0x03, 0x11, 0x00);  // MEDIUM ERROR, Unrecovered Read Error
        }
        return -1;
    }

    if (!is_write) {
        // Arm prefetch once a sequential read stream is detected: the main
        // loop reads the next chunk while this one streams out over USB.
        if (lba == last_read_end)
            prefetch_want = lba + count;
        last_read_end = lba + count;
    }
    return (int32_t)(count * MSC_BLOCK_SIZE);
}

// Called from the main loop: fetch the armed prefetch chunk, overlapping the
// drive read with the in-flight USB transfer of the previous chunk.
void msc_service_prefetch(void) {
    if (prefetch_want == LBA_NONE || !drive_ready || power_gated) return;

    uint32_t want = prefetch_want;
    prefetch_want = LBA_NONE;

    if (want == prefetch_lba) return;                              // already cached
    if (want + MSC_CHUNK_SECTORS > drive_sectors) return;          // partial chunks read direct
    if (bad_cache_overlaps(want, MSC_CHUNK_SECTORS)) return;       // strict path handles these

    prefetch_lba = LBA_NONE;
    if (ata_read_sectors(want, MSC_CHUNK_SECTORS, prefetch_buf)) {
        prefetch_lba = want;
    } else {
        // Leave error reporting to the direct read path when the host asks —
        // just put the drive back into a clean state.
        ata_error_recovery();
    }
}

// Invoked when received Test Unit Ready command
// Always return ready if drive was initialized — wake deferred to actual I/O.
// This prevents the kernel's periodic TUR polling from cycling the drive.
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
    if (load_eject && !start) {
        // Eject / safely-remove: flush + park + power down now instead of
        // waiting for the idle timeout. Next I/O wakes the drive as usual.
        printf("[MSC] Eject requested\n");
        if (drive_spinning && !power_gated) {
            msc_power_gate();
        }
    }
    return true;
}

// Invoked on read10 command
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                           void *buffer, uint32_t bufsize) {
    (void)lun;

    // With a 32 KB MSC endpoint buffer, TinyUSB keeps READ10 requests block-aligned.
    if (!msc_is_block_aligned(offset, bufsize)) {
        printf("[MSC] Unaligned read offset=%lu size=%lu\n", offset, bufsize);
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00);
        return -1;
    }
    return msc_transfer(lba, buffer, bufsize, false);
}

// Invoked on write10 command
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                            uint8_t *buffer, uint32_t bufsize) {
    (void)lun;

    // With a 32 KB MSC endpoint buffer, TinyUSB keeps WRITE10 requests block-aligned.
    if (!msc_is_block_aligned(offset, bufsize)) {
        printf("[MSC] Unaligned write offset=%lu size=%lu\n", offset, bufsize);
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00);
        return -1;
    }

    // Synchronous write-through: status reflects the medium, errors land on
    // this exact command.
    return msc_transfer(lba, buffer, bufsize, true);
}

// Invoked when received SCSI command not handled above
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16],
                         void *buffer, uint16_t bufsize) {
    (void)lun;
    (void)buffer;
    (void)bufsize;

    switch (scsi_cmd[0]) {
        case 0x35:   // SYNCHRONIZE CACHE (10) — issued by Linux on sync/unmount
        case 0x91: { // SYNCHRONIZE CACHE (16)
            // Writes are synchronous; only the drive's own cache may hold data.
            // If the drive is power gated, STANDBY IMMEDIATE already flushed it.
            if (drive_spinning && !power_gated && !ata_flush_cache()) {
                tud_msc_set_sense(lun, 0x03, 0x0C, 0x00);  // MEDIUM ERROR, Write Error
                return -1;
            }
            return 0;
        }
        default:
            break;
    }

    printf("[MSC] Unhandled SCSI cmd: 0x%02X\n", scsi_cmd[0]);

    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    return -1;
}

// Required: is writable
bool tud_msc_is_writable_cb(uint8_t lun) {
    (void)lun;
    return true;
}
