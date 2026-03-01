/*
 * ATA-over-SDIO layer for MK4001MTD
 * CMD52 for register access, CMD53 for data transfer.
 * Uses PIO for all SDIO operations (bit-bang only for card init).
 */

#include "ata_sdio.h"
#include "sdio_hw.h"
#include "sdio_pio.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

#define ATA_RETRIES 3

// Auto-dispatch: PIO when ready, bit-bang otherwise (card init)
static bool cmd52_rd(uint8_t fn, uint32_t addr, uint8_t *data) {
    if (sdio_pio_is_ready()) return sdio_pio_cmd52_read(fn, addr, data);
    return sdio_cmd52_read(fn, addr, data);
}

static bool cmd52_wr(uint8_t fn, uint32_t addr, uint8_t data, uint8_t *resp) {
    if (sdio_pio_is_ready()) return sdio_pio_cmd52_write(fn, addr, data, resp);
    return sdio_cmd52_write(fn, addr, data, resp);
}

static bool cmd53_rd(uint8_t fn, uint32_t addr, uint8_t *buf, uint16_t cnt) {
    if (sdio_pio_is_ready()) return sdio_pio_cmd53_read_block(fn, addr, buf, cnt);
    return sdio_cmd53_read(fn, addr, buf, cnt, true, false);
}

static bool cmd53_wr(uint8_t fn, uint32_t addr, const uint8_t *buf, uint16_t cnt) {
    if (sdio_pio_is_ready()) return sdio_pio_cmd53_write_block(fn, addr, buf, cnt);
    return sdio_cmd53_write(fn, addr, buf, cnt, true, false);
}

bool ata_wait_interrupt(uint32_t timeout_ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    uint8_t pending;
    while ((to_ms_since_boot(get_absolute_time()) - start) < timeout_ms) {
        if (!cmd52_rd(0, CCCR_INT_PENDING, &pending))
            return false;
        if (pending & 0x02) return true;
    }
    return false;
}

bool ata_wait_not_busy(uint32_t timeout_ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    uint8_t status;
    while ((to_ms_since_boot(get_absolute_time()) - start) < timeout_ms) {
        if (!cmd52_rd(1, ATA_REG_STATUS, &status))
            return false;
        if (!(status & ATA_STATUS_BSY)) return true;
    }
    return false;
}

static bool ata_wait_drq(uint32_t timeout_ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start) < timeout_ms) {
        uint8_t status = 0;
        // Poll STATUS directly — works for both read (interrupt+DRQ) and write (DRQ only)
        if (!cmd52_rd(1, ATA_REG_STATUS, &status)) return false;
        if (status & ATA_STATUS_DRQ) return true;
        if (!(status & ATA_STATUS_BSY) && !(status & ATA_STATUS_DRQ)) {
            // Not busy, no DRQ — check if error
            if (status & ATA_STATUS_ERR) return false;
        }
    }
    return false;
}

bool ata_identify(uint8_t *buf) {
    uint8_t rd;
    for (int attempt = 0; attempt < ATA_RETRIES; attempt++) {
        if (attempt > 0) printf("[ATA] IDENTIFY retry %d\n", attempt);
        ata_wait_not_busy(10000);
        cmd52_wr(1, ATA_REG_SECCOUNT, 0x00, &rd);
        cmd52_wr(1, ATA_REG_COMMAND, ATA_CMD_IDENTIFY, &rd);
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
    uint8_t rd;
    for (int attempt = 0; attempt < ATA_RETRIES; attempt++) {
        if (attempt > 0) {
            printf("[ATA] READ retry %d LBA=%lu\n", attempt, lba);
            ata_wait_not_busy(5000);
        }
        if (!ata_wait_not_busy(5000)) { printf("[ATA] READ: BSY timeout LBA=%lu att=%d\n", lba, attempt); continue; }

        cmd52_wr(1, ATA_REG_SECCOUNT, count, &rd);
        cmd52_wr(1, ATA_REG_LBA_LO,  (lba >> 0)  & 0xFF, &rd);
        cmd52_wr(1, ATA_REG_LBA_MID, (lba >> 8)  & 0xFF, &rd);
        cmd52_wr(1, ATA_REG_LBA_HI,  (lba >> 16) & 0xFF, &rd);
        cmd52_wr(1, ATA_REG_DEV_HEAD, 0xE0 | ((lba >> 24) & 0x0F), &rd);
        cmd52_wr(1, ATA_REG_COMMAND, ATA_CMD_READ_SEC, &rd);

        // Wait for first DRQ (drive has data ready)
        if (!ata_wait_drq(5000)) { printf("[ATA] READ: DRQ timeout LBA=%lu\n", lba); continue; }

        // Single CMD53 with block_count = sector count (PIO or bit-bang)
        if (!cmd53_rd(1, ATA_REG_DATA, buf, count)) {
            printf("[ATA] READ: CMD53 fail cnt=%d\n", count);
            continue;
        }
        return true;
    }
    return false;
}

bool ata_write_sectors(uint32_t lba, uint8_t count, const uint8_t *buf) {
    uint8_t rd;

    for (int attempt = 0; attempt < ATA_RETRIES; attempt++) {
        if (attempt > 0) {
            printf("[ATA] WRITE retry %d LBA=%lu\n", attempt, lba);
            ata_wait_not_busy(5000);
        }
        if (!ata_wait_not_busy(5000)) {
            printf("[ATA] WRITE: BSY timeout LBA=%lu att=%d\n", lba, attempt);
            continue;
        }

        cmd52_wr(1, ATA_REG_SECCOUNT, count, &rd);
        cmd52_wr(1, ATA_REG_LBA_LO,  (lba >> 0)  & 0xFF, &rd);
        cmd52_wr(1, ATA_REG_LBA_MID, (lba >> 8)  & 0xFF, &rd);
        cmd52_wr(1, ATA_REG_LBA_HI,  (lba >> 16) & 0xFF, &rd);
        cmd52_wr(1, ATA_REG_DEV_HEAD, 0xE0 | ((lba >> 24) & 0x0F), &rd);
        cmd52_wr(1, ATA_REG_COMMAND, ATA_CMD_WRITE_SEC, &rd);

        // Wait for DRQ — drive ready to accept data
        // N91 trace: drive reports 0xD8 (BSY+DRQ) as normal pre-transfer status
        if (!ata_wait_drq(5000)) {
            printf("[ATA] WRITE: DRQ timeout LBA=%lu\n", lba);
            continue;
        }

        // Single multi-block CMD53 write (like N91: one CMD53 for all sectors)
        if (!cmd53_wr(1, ATA_REG_DATA, buf, count)) {
            printf("[ATA] WRITE: CMD53 fail cnt=%d LBA=%lu\n", count, lba);
            continue;
        }

        if (ata_wait_not_busy(10000)) {
            // Verify no error
            uint8_t status;
            cmd52_rd(1, ATA_REG_STATUS, &status);
            if (status & ATA_STATUS_ERR) {
                uint8_t err;
                cmd52_rd(1, ATA_REG_ERROR, &err);
                printf("[ATA] WRITE: post-write ERR=0x%02X ST=0x%02X LBA=%lu\n", err, status, lba);
                continue;
            }
            return true;
        }
    }
    return false;
}

uint32_t ata_get_capacity(const uint8_t *buf) {
    uint16_t w60 = buf[120] | (buf[121] << 8);
    uint16_t w61 = buf[122] | (buf[123] << 8);
    return ((uint32_t)w61 << 16) | w60;
}

void ata_print_identify(const uint8_t *buf) {
    char serial[21] = {0}, firmware[9] = {0}, model[41] = {0};
    for (int i = 0; i < 20; i += 2) { serial[i] = buf[20+i+1]; serial[i+1] = buf[20+i]; }
    for (int i = 0; i < 8; i += 2)  { firmware[i] = buf[46+i+1]; firmware[i+1] = buf[46+i]; }
    for (int i = 0; i < 40; i += 2) { model[i] = buf[54+i+1]; model[i+1] = buf[54+i]; }
    uint32_t sectors = ata_get_capacity(buf);
    printf("Model:    [%s]\nSerial:   [%s]\nFirmware: [%s]\nSectors:  %lu (%lu MB)\n",
           model, serial, firmware, sectors, sectors / 2048);

    // SMART support in IDENTIFY: word 82 bit 0, word 85 bit 0 (enabled)
    uint16_t w82 = buf[164] | (buf[165] << 8);
    uint16_t w85 = buf[170] | (buf[171] << 8);
    printf("SMART:    %s (supported=%d, enabled=%d)\n",
           (w82 & 1) ? "supported" : "not supported",
           (w82 & 1) ? 1 : 0, (w85 & 1) ? 1 : 0);
}

// ============================================================
// Power management
// ============================================================

bool ata_standby_immediate(void) {
    uint8_t rd;
    if (!ata_wait_not_busy(5000)) return false;
    cmd52_wr(1, ATA_REG_COMMAND, ATA_CMD_STANDBY_IMM, &rd);
    return ata_wait_not_busy(5000);
}

// ============================================================
// SMART commands
// ============================================================

static bool ata_smart_cmd(uint8_t feature, uint8_t *buf) {
    uint8_t rd;
    for (int attempt = 0; attempt < ATA_RETRIES; attempt++) {
        if (attempt > 0) printf("[SMART] retry %d\n", attempt);
        if (!ata_wait_not_busy(5000)) continue;

        cmd52_wr(1, ATA_REG_FEATURE,  feature, &rd);
        cmd52_wr(1, ATA_REG_SECCOUNT, 0x00, &rd);
        cmd52_wr(1, ATA_REG_LBA_LO,   0x00, &rd);
        cmd52_wr(1, ATA_REG_LBA_MID,  SMART_LBA_MID, &rd);
        cmd52_wr(1, ATA_REG_LBA_HI,   SMART_LBA_HI, &rd);
        cmd52_wr(1, ATA_REG_DEV_HEAD, 0xE0, &rd);
        cmd52_wr(1, ATA_REG_COMMAND,  ATA_CMD_SMART, &rd);

        if (buf) {
            // Data-returning SMART command
            if (!ata_wait_drq(5000)) { printf("[SMART] no DRQ\n"); continue; }
            if (cmd53_rd(1, ATA_REG_DATA, buf, 1)) return true;
        } else {
            // Non-data SMART command (e.g., ENABLE)
            if (ata_wait_not_busy(5000)) {
                uint8_t status;
                cmd52_rd(1, ATA_REG_STATUS, &status);
                if (!(status & ATA_STATUS_ERR)) return true;
                uint8_t err;
                cmd52_rd(1, ATA_REG_ERROR, &err);
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
    uint8_t rd;
    if (!ata_wait_not_busy(5000)) return false;
    cmd52_wr(1, ATA_REG_FEATURE, feature, &rd);
    cmd52_wr(1, ATA_REG_COMMAND, 0xC2, &rd);
    if (!ata_wait_interrupt(5000)) return false;
    // Read back all registers
    cmd52_rd(1, ATA_REG_ERROR,    &out_regs[0]);
    cmd52_rd(1, ATA_REG_SECCOUNT, &out_regs[1]);
    cmd52_rd(1, ATA_REG_LBA_LO,  &out_regs[2]);
    cmd52_rd(1, ATA_REG_LBA_MID, &out_regs[3]);
    cmd52_rd(1, ATA_REG_LBA_HI,  &out_regs[4]);
    cmd52_rd(1, ATA_REG_STATUS,   &out_regs[5]);
    return true;
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
