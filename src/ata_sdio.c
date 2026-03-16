/*
 * ATA-over-SDIO layer for MK4001MTD
 * CMD52 for register access, CMD53 for data transfer.
 * All SDIO operations use PIO (no bit-bang).
 */

#include "ata_sdio.h"
#include "sdio_hw.h"
#include "sdio_pio.h"
#include "tusb.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

#define ATA_RETRIES 2
#define ATA_DEVICE_HEAD_LBA 0xE0
#define ATA_READ_DRQ_TIMEOUT_MS       8000u
#define ATA_READ_COMPLETE_TIMEOUT_MS  2000u
#define ATA_FAST_READ_DRQ_TIMEOUT_MS  8000u
#define ATA_FAST_READ_COMPLETE_MS     2000u

static inline uint32_t ata_now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

// All SDIO goes through PIO
static inline bool cmd52_rd(uint8_t fn, uint32_t addr, uint8_t *data) {
    return sdio_pio_cmd52_read(fn, addr, data);
}

static inline bool cmd52_wr(uint8_t fn, uint32_t addr, uint8_t data, uint8_t *resp) {
    return sdio_pio_cmd52_write(fn, addr, data, resp);
}

static inline bool cmd53_rd(uint8_t fn, uint32_t addr, uint8_t *buf, uint16_t cnt) {
    return sdio_pio_cmd53_read_block(fn, addr, buf, cnt);
}

static inline bool cmd53_wr(uint8_t fn, uint32_t addr, const uint8_t *buf, uint16_t cnt) {
    return sdio_pio_cmd53_write_block(fn, addr, buf, cnt);
}

static bool ata_reg_read(uint32_t addr, uint8_t *data) {
    return cmd52_rd(1, addr, data);
}

static bool ata_reg_write(uint32_t addr, uint8_t data) {
    uint8_t ignored = 0;
    return cmd52_wr(1, addr, data, &ignored);
}

static bool ata_read_status_error(uint8_t *status_out, uint8_t *error_out) {
    uint8_t status = 0;
    uint8_t error = 0;

    if (!ata_reg_read(ATA_REG_STATUS, &status)) return false;
    if ((status & ATA_STATUS_ERR) && !ata_reg_read(ATA_REG_ERROR, &error)) return false;

    if (status_out) *status_out = status;
    if (error_out) *error_out = error;
    return true;
}

static void ata_log_status_error(const char *tag, uint32_t lba) {
    uint8_t status = 0;
    uint8_t error = 0;

    if (!ata_read_status_error(&status, &error)) {
        printf("[ATA] %s: status/error read fail LBA=%lu\n", tag, lba);
        return;
    }

    printf("[ATA] %s: ST=0x%02X%s%s", tag, status,
           (status & ATA_STATUS_DF) ? " DF" : "",
           (status & ATA_STATUS_ERR) ? " ERR" : "");

    if (status & ATA_STATUS_ERR) {
        printf(" ERR=0x%02X%s%s%s", error,
               (error & ATA_ERROR_ABRT) ? " ABRT" : "",
               (error & ATA_ERROR_IDNF) ? " IDNF" : "",
               (error & ATA_ERROR_UNC) ? " UNC" : "");
    }

    printf(" LBA=%lu\n", lba);
}

static bool ata_wait_command_result(uint32_t lba, const char *tag, uint32_t timeout_ms) {
    uint8_t status = 0;
    uint8_t error = 0;

    if (!ata_wait_not_busy(timeout_ms)) {
        printf("[ATA] %s: completion timeout LBA=%lu\n", tag, lba);
        ata_log_status_error(tag, lba);
        return false;
    }

    if (!ata_read_status_error(&status, &error)) {
        printf("[ATA] %s: final status read fail LBA=%lu\n", tag, lba);
        return false;
    }

    if (status & (ATA_STATUS_DF | ATA_STATUS_ERR)) {
        ata_log_status_error(tag, lba);
        return false;
    }

    return true;
}

static bool ata_issue_rw_command(uint32_t lba, uint8_t count, uint8_t command) {
    return ata_reg_write(ATA_REG_SECCOUNT, count) &&
           ata_reg_write(ATA_REG_LBA_LO, (lba >> 0) & 0xFF) &&
           ata_reg_write(ATA_REG_LBA_MID, (lba >> 8) & 0xFF) &&
           ata_reg_write(ATA_REG_LBA_HI, (lba >> 16) & 0xFF) &&
           ata_reg_write(ATA_REG_DEV_HEAD, ATA_DEVICE_HEAD_LBA | ((lba >> 24) & 0x0F)) &&
           ata_reg_write(ATA_REG_COMMAND, command);
}

static uint16_t ata_identify_word(const uint8_t *buf, uint8_t word_index) {
    size_t offset = (size_t)word_index * 2u;
    return (uint16_t)buf[offset] | ((uint16_t)buf[offset + 1] << 8);
}

static void ata_copy_identify_string(char *dst, size_t dst_size,
                                     const uint8_t *buf, uint8_t start_word, size_t word_count) {
    size_t src_offset = (size_t)start_word * 2u;
    size_t char_count = word_count * 2u;

    if (dst_size == 0) return;
    if (char_count > (dst_size - 1u)) char_count = dst_size - 1u;

    for (size_t i = 0; i < char_count; i += 2) {
        dst[i] = (char)(buf[src_offset + i + 1]);
        if ((i + 1u) < char_count) {
            dst[i + 1] = (char)(buf[src_offset + i]);
        }
    }
    dst[char_count] = '\0';

    while (char_count > 0 && dst[char_count - 1] == ' ') {
        dst[--char_count] = '\0';
    }
}

bool ata_wait_interrupt(uint32_t timeout_ms) {
    uint32_t start = ata_now_ms();
    uint8_t pending = 0;
    while ((ata_now_ms() - start) < timeout_ms) {
        if (!cmd52_rd(0, CCCR_INT_PENDING, &pending))
            return false;
        if (pending & 0x02) return true;
    }
    return false;
}

bool ata_wait_not_busy(uint32_t timeout_ms) {
    uint32_t start = ata_now_ms();
    uint8_t status = 0;
    while ((ata_now_ms() - start) < timeout_ms) {
        if (!ata_reg_read(ATA_REG_STATUS, &status))
            return false;
        if (!(status & ATA_STATUS_BSY)) return true;
    }
    return false;
}

static bool ata_wait_drq(uint32_t timeout_ms) {
    uint32_t start = ata_now_ms();
    while ((ata_now_ms() - start) < timeout_ms) {
        uint8_t status = 0;
        // Poll STATUS directly — works for both read (interrupt+DRQ) and write (DRQ only)
        if (!ata_reg_read(ATA_REG_STATUS, &status)) return false;
        if (status & ATA_STATUS_DRQ) return true;
        if (!(status & ATA_STATUS_BSY) && !(status & ATA_STATUS_DRQ)) {
            // Not busy, no DRQ — check if error
            if (status & ATA_STATUS_ERR) return false;
        }
    }
    return false;
}

bool ata_identify(uint8_t *buf) {
    for (int attempt = 0; attempt < ATA_RETRIES; attempt++) {
        if (attempt > 0) printf("[ATA] IDENTIFY retry %d\n", attempt);
        if (!ata_wait_not_busy(10000)) {
            printf("[ATA] IDENTIFY: BSY timeout\n");
            continue;
        }
        if (!ata_reg_write(ATA_REG_SECCOUNT, 0x00) ||
            !ata_reg_write(ATA_REG_COMMAND, ATA_CMD_IDENTIFY)) {
            printf("[ATA] IDENTIFY: taskfile write failed\n");
            continue;
        }
        if (!ata_wait_drq(15000)) { printf("[ATA] IDENTIFY: no DRQ\n"); continue; }
        if (cmd53_rd(1, ATA_REG_DATA, buf, 1)) {
            printf("[ATA] IDENTIFY complete\n");
            return true;
        }
    }
    printf("[ATA] IDENTIFY failed\n");
    return false;
}

bool ata_read_sectors(uint32_t lba, uint8_t count, uint8_t *buf) {
    for (int attempt = 0; attempt < ATA_RETRIES; attempt++) {
        if (attempt > 0) {
            printf("[ATA] READ retry %d LBA=%lu\n", attempt, lba);
            ata_wait_not_busy(200);
        }
        if (!ata_wait_not_busy(500)) { printf("[ATA] READ: BSY timeout LBA=%lu att=%d\n", lba, attempt); continue; }

        if (!ata_issue_rw_command(lba, count, ATA_CMD_READ_SEC)) {
            printf("[ATA] READ: taskfile write fail LBA=%lu\n", lba);
            continue;
        }

        // Wait for first DRQ (drive has data ready). Some UNC sectors do not
        // raise DRQ until just past 5s, so leave enough margin to capture
        // the final ATA result instead of reporting a generic timeout.
        if (!ata_wait_drq(ATA_READ_DRQ_TIMEOUT_MS)) {
            printf("[ATA] READ: DRQ timeout LBA=%lu\n", lba);
            ata_log_status_error("READ", lba);
            continue;
        }

        // Single CMD53 with block_count = sector count (PIO or bit-bang)
        if (!cmd53_rd(1, ATA_REG_DATA, buf, count)) {
            printf("[ATA] READ: CMD53 fail cnt=%d\n", count);
            ata_log_status_error("READ", lba);
            continue;
        }

        if (!ata_wait_command_result(lba, "READ", ATA_READ_COMPLETE_TIMEOUT_MS)) {
            continue;
        }
        return true;
    }
    return false;
}

bool ata_write_sectors(uint32_t lba, uint8_t count, const uint8_t *buf) {
    for (int attempt = 0; attempt < ATA_RETRIES; attempt++) {
        if (attempt > 0) {
            printf("[ATA] WRITE retry %d LBA=%lu\n", attempt, lba);
            ata_wait_not_busy(5000);
        }
        if (!ata_wait_not_busy(5000)) {
            printf("[ATA] WRITE: BSY timeout LBA=%lu att=%d\n", lba, attempt);
            continue;
        }

        if (!ata_issue_rw_command(lba, count, ATA_CMD_WRITE_SEC)) {
            printf("[ATA] WRITE: taskfile write fail LBA=%lu\n", lba);
            continue;
        }

        // Wait for DRQ — drive ready to accept data
        // N91 trace: drive reports 0xD8 (BSY+DRQ) as normal pre-transfer status
        if (!ata_wait_drq(5000)) {
            printf("[ATA] WRITE: DRQ timeout LBA=%lu\n", lba);
            ata_log_status_error("WRITE", lba);
            continue;
        }

        // Single multi-block CMD53 write (like N91: one CMD53 for all sectors)
        if (!cmd53_wr(1, ATA_REG_DATA, buf, count)) {
            printf("[ATA] WRITE: CMD53 fail cnt=%d LBA=%lu\n", count, lba);
            ata_log_status_error("WRITE", lba);
            continue;
        }

        if (ata_wait_command_result(lba, "WRITE", 10000)) {
            return true;
        }
    }
    return false;
}

uint32_t ata_get_capacity(const uint8_t *buf) {
    uint16_t w60 = ata_identify_word(buf, 60);
    uint16_t w61 = ata_identify_word(buf, 61);
    return ((uint32_t)w61 << 16) | w60;
}

void ata_print_identify(const uint8_t *buf) {
    char serial[21] = {0}, firmware[9] = {0}, model[41] = {0};
    ata_copy_identify_string(serial, sizeof(serial), buf, 10, 10);
    ata_copy_identify_string(firmware, sizeof(firmware), buf, 23, 4);
    ata_copy_identify_string(model, sizeof(model), buf, 27, 20);
    uint32_t sectors = ata_get_capacity(buf);
    printf("Model:    [%s]\nSerial:   [%s]\nFirmware: [%s]\nSectors:  %lu (%lu MB)\n",
           model, serial, firmware, sectors, sectors / 2048);

    // SMART support in IDENTIFY: word 82 bit 0, word 85 bit 0 (enabled)
    uint16_t w82 = ata_identify_word(buf, 82);
    uint16_t w85 = ata_identify_word(buf, 85);
    printf("SMART:    %s (supported=%d, enabled=%d)\n",
           (w82 & 1) ? "supported" : "not supported",
           (w82 & 1) ? 1 : 0, (w85 & 1) ? 1 : 0);
}

// ============================================================
// Error recovery
// ============================================================

void ata_error_recovery(void) {
    uint8_t status = 0, err = 0;

    // Step 1: IO_ABORT — tell the SDIO controller to cancel fn1's pending operation.
    // This is the SDIO-standard way to recover from a stuck I/O transfer.
    uint8_t abort_resp = 0;
    cmd52_wr(0, CCCR_IO_ABORT, 0x01, &abort_resp);  // Abort fn1

    // Step 2: Read STATUS to clear any pending interrupt / error latch
    if (ata_reg_read(ATA_REG_STATUS, &status)) {
        if (status & ATA_STATUS_ERR) {
            ata_reg_read(ATA_REG_ERROR, &err);
            printf("[ATA] Recovery: ST=0x%02X ERR=0x%02X\n", status, err);
        }
    }

    // Step 3: If still BSY after abort, quick wait then fn1 reset
    // Total budget: ~500ms max
    if (status & ATA_STATUS_BSY) {
        printf("[ATA] Recovery: BSY after IO_ABORT\n");
        bool cleared = ata_wait_not_busy(100);
        tud_task();
        if (!cleared) {
            printf("[ATA] Recovery: fn1 reset\n");
            cmd52_wr(0, CCCR_IO_ENABLE, 0x00, &abort_resp);
            sleep_ms(5); tud_task();
            cmd52_wr(0, CCCR_IO_ENABLE, 0x02, &abort_resp);
            uint32_t start = ata_now_ms();
            uint8_t rdy = 0;
            while ((ata_now_ms() - start) < 200) {
                if (cmd52_rd(0, CCCR_IO_READY, &rdy) && (rdy & 0x02)) break;
                sleep_ms(5); tud_task();
            }
            ata_wait_not_busy(200);
            tud_task();
        }
        // Final STATUS read to clear error latch
        ata_reg_read(ATA_REG_STATUS, &status);
    }
}

// ============================================================
// Fast single-sector read/write for bad-sector fallback
// Reduced retries (1). Read timeout still allows the drive to surface
// its final ATA error so slow UNC sectors are not misreported as DRQ timeouts.
// ============================================================

bool ata_read_sector_fast(uint32_t lba, uint8_t *buf) {
    if (!ata_wait_not_busy(2000)) {
        printf("[ATA] FAST-RD: BSY LBA=%lu\n", lba);
        return false;
    }
    if (!ata_issue_rw_command(lba, 1, ATA_CMD_READ_SEC)) return false;
    if (!ata_wait_drq(ATA_FAST_READ_DRQ_TIMEOUT_MS)) {
        printf("[ATA] FAST-RD: DRQ timeout LBA=%lu\n", lba);
        ata_log_status_error("FAST-RD", lba);
        return false;
    }
    if (!cmd53_rd(1, ATA_REG_DATA, buf, 1)) {
        printf("[ATA] FAST-RD: CMD53 fail LBA=%lu\n", lba);
        ata_log_status_error("FAST-RD", lba);
        return false;
    }
    if (!ata_wait_command_result(lba, "FAST-RD", ATA_FAST_READ_COMPLETE_MS)) {
        return false;
    }
    return true;
}

bool ata_write_sector_fast(uint32_t lba, const uint8_t *buf) {
    if (!ata_wait_not_busy(2000)) {
        printf("[ATA] FAST-WR: BSY LBA=%lu\n", lba);
        return false;
    }
    if (!ata_issue_rw_command(lba, 1, ATA_CMD_WRITE_SEC)) return false;
    if (!ata_wait_drq(2000)) {
        printf("[ATA] FAST-WR: DRQ timeout LBA=%lu\n", lba);
        ata_log_status_error("FAST-WR", lba);
        return false;
    }
    if (!cmd53_wr(1, ATA_REG_DATA, buf, 1)) {
        printf("[ATA] FAST-WR: CMD53 fail LBA=%lu\n", lba);
        ata_log_status_error("FAST-WR", lba);
        return false;
    }
    if (!ata_wait_command_result(lba, "FAST-WR", 5000)) {
        return false;
    }
    return true;
}

// ============================================================
// Power management
// ============================================================

bool ata_standby_immediate(void) {
    if (!ata_wait_not_busy(5000)) return false;
    if (!ata_reg_write(ATA_REG_COMMAND, ATA_CMD_STANDBY_IMM)) return false;
    return ata_wait_not_busy(5000);
}

// ============================================================
// SMART commands
// ============================================================

static bool ata_smart_cmd(uint8_t feature, uint8_t *buf) {
    for (int attempt = 0; attempt < ATA_RETRIES; attempt++) {
        if (attempt > 0) printf("[SMART] retry %d\n", attempt);
        if (!ata_wait_not_busy(5000)) continue;

        if (!ata_reg_write(ATA_REG_FEATURE, feature) ||
            !ata_reg_write(ATA_REG_SECCOUNT, 0x00) ||
            !ata_reg_write(ATA_REG_LBA_LO, 0x00) ||
            !ata_reg_write(ATA_REG_LBA_MID, SMART_LBA_MID) ||
            !ata_reg_write(ATA_REG_LBA_HI, SMART_LBA_HI) ||
            !ata_reg_write(ATA_REG_DEV_HEAD, ATA_DEVICE_HEAD_LBA) ||
            !ata_reg_write(ATA_REG_COMMAND, ATA_CMD_SMART)) {
            printf("[SMART] taskfile write failed\n");
            continue;
        }

        if (buf) {
            // Data-returning SMART command
            if (!ata_wait_drq(5000)) { printf("[SMART] no DRQ\n"); continue; }
            if (cmd53_rd(1, ATA_REG_DATA, buf, 1)) return true;
        } else {
            // Non-data SMART command (e.g., ENABLE)
            if (ata_wait_not_busy(5000)) {
                uint8_t status = 0;
                if (!ata_reg_read(ATA_REG_STATUS, &status)) continue;
                if (!(status & ATA_STATUS_ERR)) return true;
                uint8_t err = 0;
                ata_reg_read(ATA_REG_ERROR, &err);
                printf("[SMART] ERR=0x%02X\n", err);
            }
        }
    }
    return false;
}

bool ata_smart_enable(void) {
    printf("[SMART] Enabling SMART...\n");
    return ata_smart_cmd(SMART_ENABLE_OPS, NULL);
}

bool ata_smart_read_data(uint8_t *buf) {
    return ata_smart_cmd(SMART_READ_DATA, buf);
}

bool ata_smart_read_thresholds(uint8_t *buf) {
    return ata_smart_cmd(SMART_READ_THRESH, buf);
}

// Known SMART attribute names
static const char *smart_attr_name(uint8_t id) {
    switch (id) {
        case 1:   return "Raw Read Error Rate";
        case 2:   return "Throughput Performance";
        case 3:   return "Spin-Up Time";
        case 4:   return "Start/Stop Count";
        case 5:   return "Reallocated Sectors";
        case 7:   return "Seek Error Rate";
        case 8:   return "Seek Time Performance";
        case 9:   return "Power-On Hours";
        case 10:  return "Spin Retry Count";
        case 11:  return "Calibration Retry Count";
        case 12:  return "Power Cycle Count";
        case 100: return "Erase/Program Cycles";
        case 170: return "Reserve Block Count";
        case 171: return "Program Fail Count";
        case 172: return "Erase Fail Count";
        case 173: return "Wear Leveling Count";
        case 174: return "Unexpected Power Loss";
        case 175: return "Power Loss Protection";
        case 187: return "Reported Uncorrectable";
        case 188: return "Command Timeout";
        case 190: return "Airflow Temperature";
        case 191: return "G-Sense Error Rate";
        case 192: return "Power-Off Retract Count";
        case 193: return "Load Cycle Count";
        case 194: return "Temperature";
        case 195: return "Hardware ECC Recovered";
        case 196: return "Reallocation Event Count";
        case 197: return "Current Pending Sector";
        case 198: return "Offline Uncorrectable";
        case 199: return "UDMA CRC Error Count";
        case 200: return "Write Error Rate";
        case 220: return "Disk Shift";
        case 222: return "Loaded Hours";
        case 223: return "Load Retry Count";
        case 224: return "Load Friction";
        case 226: return "Load-In Time";
        case 240: return "Head Flying Hours";
        case 241: return "Total LBAs Written";
        case 242: return "Total LBAs Read";
        case 254: return "Free Fall Protection";
        default:  return NULL;
    }
}

// Vendor command 0xC2 (observed in N91 trace)
static bool ata_vendor_c2(uint8_t feature, uint8_t *out_regs) {
    if (!ata_wait_not_busy(5000)) return false;
    if (!ata_reg_write(ATA_REG_FEATURE, feature) ||
        !ata_reg_write(ATA_REG_COMMAND, 0xC2)) {
        return false;
    }
    if (!ata_wait_interrupt(5000)) return false;
    // Read back all registers
    return ata_reg_read(ATA_REG_ERROR, &out_regs[0]) &&
           ata_reg_read(ATA_REG_SECCOUNT, &out_regs[1]) &&
           ata_reg_read(ATA_REG_LBA_LO, &out_regs[2]) &&
           ata_reg_read(ATA_REG_LBA_MID, &out_regs[3]) &&
           ata_reg_read(ATA_REG_LBA_HI, &out_regs[4]) &&
           ata_reg_read(ATA_REG_STATUS, &out_regs[5]);
}

void ata_smart_dump(void) {
    static uint8_t smart_data[512];
    static uint8_t smart_thresh[512];

    printf("\n[DIAG] === Drive Diagnostics ===\n");
    printf("[DIAG] Standard SMART: not supported (IDENTIFY W82 bit0 = 0)\n");

    // Vendor command 0xC2 — Toshiba proprietary (observed in N91 trace)
    // Returns register values, no data transfer
    printf("[DIAG] Toshiba vendor CMD 0xC2:\n");
    uint8_t regs[6];

    // Features that the drive accepts (discovered by scanning 0x00-0xFF):
    // 0x01-0x04, 0x10-0x12, 0x20-0x21 — all others abort (ERR=0x04)
    static const struct { uint8_t feat; const char *label; } vc2[] = {
        {0x01, "unknown_01"},
        {0x02, "unknown_02"},
        {0x03, "unknown_03"},
        {0x04, "unknown_04"},
        {0x10, "diag_10 (LBA_LO varies)"},
        {0x11, "diag_11"},
        {0x12, "diag_12 (LBA_LO varies)"},
        {0x20, "query_20 (N91: SC=0xFF always)"},
        {0x21, "query_21 (N91: SC varies per boot)"},
    };

    for (int i = 0; i < (int)(sizeof(vc2)/sizeof(vc2[0])); i++) {
        printf("  FEAT=0x%02X %-32s → ", vc2[i].feat, vc2[i].label);
        if (ata_vendor_c2(vc2[i].feat, regs))
            printf("SC=%02X LBA=%02X/%02X/%02X ST=%02X%s\n",
                   regs[1], regs[2], regs[3], regs[4], regs[5],
                   (regs[5] & ATA_STATUS_ERR) ? " ERR" : "");
        else
            printf("no response\n");
    }
    printf("\n");
}
