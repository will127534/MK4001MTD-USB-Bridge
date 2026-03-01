/*
 * SDIO Hardware Driver for MK4001MTD Microdrive
 * Bit-banged implementation with proper CRC.
 * Based on Nokia N91 logic trace analysis.
 */

#include "sdio_hw.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/structs/sio.h"
#include <stdio.h>
#include <string.h>

// Clock half-period in microseconds
static uint32_t clk_half_period_us = 2;  // ~250 KHz default (slow init)

// Current RCA
static uint16_t card_rca = 0;

// Retry count for commands
#define CMD_RETRIES 3
#define DATA_RETRIES 3

// ============================================================
// CRC7 for SDIO commands (polynomial x^7 + x^3 + 1)
// ============================================================
static uint8_t crc7(const uint8_t *data, int bits) {
    uint8_t crc = 0;
    for (int i = 0; i < bits; i++) {
        int byte_idx = i / 8;
        int bit_idx = 7 - (i % 8);
        uint8_t bit = (data[byte_idx] >> bit_idx) & 1;
        uint8_t fb = ((crc >> 6) ^ bit) & 1;
        crc = (crc << 1) & 0x7F;
        if (fb) crc ^= 0x09;
    }
    return crc;
}

// ============================================================
// CRC16-CCITT per data line (polynomial x^16 + x^12 + x^5 + 1)
// ============================================================
static uint16_t crc16_one_bit(uint16_t crc, uint8_t bit) {
    uint8_t fb = ((crc >> 15) ^ bit) & 1;
    crc = (crc << 1) & 0xFFFF;
    if (fb) crc ^= 0x1021;
    return crc;
}

// Compute CRC16 for one DAT line from a byte buffer
// dat_line: 0-3 (which bit of each nibble)
// nibbles: array of nibbles (DAT3..DAT0 per entry)
// count: number of nibbles
static uint16_t crc16_dat_line(const uint8_t *data, int total_bytes, int dat_line) {
    uint16_t crc = 0;
    for (int i = 0; i < total_bytes; i++) {
        // High nibble
        uint8_t hi_bit = (data[i] >> (4 + dat_line)) & 1;
        crc = crc16_one_bit(crc, hi_bit);
        // Low nibble
        uint8_t lo_bit = (data[i] >> dat_line) & 1;
        crc = crc16_one_bit(crc, lo_bit);
    }
    return crc;
}

// ============================================================
// Pin management
// ============================================================
static inline void clk_high(void) { gpio_put(SDIO_CLK_PIN, 1); }
static inline void clk_low(void)  { gpio_put(SDIO_CLK_PIN, 0); }
static inline void cmd_high(void) { gpio_put(SDIO_CMD_PIN, 1); }
static inline void cmd_low(void)  { gpio_put(SDIO_CMD_PIN, 0); }
static inline void cmd_output(void) { gpio_set_dir(SDIO_CMD_PIN, GPIO_OUT); }
static inline void cmd_input(void)  { gpio_set_dir(SDIO_CMD_PIN, GPIO_IN); }
static inline bool cmd_read(void)  { return gpio_get(SDIO_CMD_PIN); }

static inline void dat_input(void) {
    for (int i = 0; i < 4; i++)
        gpio_set_dir(SDIO_DAT0_PIN + i, GPIO_IN);
}

static inline void dat_output(void) {
    for (int i = 0; i < 4; i++)
        gpio_set_dir(SDIO_DAT0_PIN + i, GPIO_OUT);
}

static inline uint8_t dat_read4(void) {
    uint8_t val = 0;
    if (gpio_get(SDIO_DAT3_PIN)) val |= 0x08;
    if (gpio_get(SDIO_DAT2_PIN)) val |= 0x04;
    if (gpio_get(SDIO_DAT1_PIN)) val |= 0x02;
    if (gpio_get(SDIO_DAT0_PIN)) val |= 0x01;
    return val;
}

static inline void dat_write4(uint8_t nibble) {
    gpio_put(SDIO_DAT0_PIN, (nibble >> 0) & 1);
    gpio_put(SDIO_DAT1_PIN, (nibble >> 1) & 1);
    gpio_put(SDIO_DAT2_PIN, (nibble >> 2) & 1);
    gpio_put(SDIO_DAT3_PIN, (nibble >> 3) & 1);
}

static inline void clock_cycle(void) {
    clk_high();
    if (clk_half_period_us > 0) busy_wait_us_32(clk_half_period_us);
    clk_low();
    if (clk_half_period_us > 0) busy_wait_us_32(clk_half_period_us);
}

static inline void clock_half_high(void) {
    clk_high();
    if (clk_half_period_us > 0) busy_wait_us_32(clk_half_period_us);
}

static inline void clock_half_low(void) {
    clk_low();
    if (clk_half_period_us > 0) busy_wait_us_32(clk_half_period_us);
}

// ============================================================
// Init pins
// ============================================================
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

    gpio_init(HDD_POWER_PIN);
    gpio_set_dir(HDD_POWER_PIN, GPIO_OUT);
    gpio_put(HDD_POWER_PIN, 1);  // Power off
}

void sdio_set_clock_slow(void) {
    clk_half_period_us = 2;  // ~250 KHz
}

void sdio_set_clock_fast(void) {
    clk_half_period_us = 1;  // 500 KHz for CMD reliability
}

void sdio_set_clock_turbo(void) {
    clk_half_period_us = 0;  // Max GPIO speed (~10-15 MHz)
}

void hdd_power_on(void) {
    gpio_put(HDD_POWER_PIN, 0);
    printf("[SDIO] HDD power ON\n");
}

void hdd_power_off(void) {
    gpio_put(HDD_POWER_PIN, 1);
    printf("[SDIO] HDD power OFF\n");
}

// ============================================================
// Send 48-bit command, receive 48-bit response
// ============================================================
static bool sdio_send_cmd_once(uint8_t cmd_index, uint32_t arg, uint32_t *resp) {
    uint8_t cmd_buf[6];

    cmd_buf[0] = 0x40 | (cmd_index & 0x3F);
    cmd_buf[1] = (arg >> 24) & 0xFF;
    cmd_buf[2] = (arg >> 16) & 0xFF;
    cmd_buf[3] = (arg >> 8)  & 0xFF;
    cmd_buf[4] = arg & 0xFF;
    uint8_t crc = crc7(cmd_buf, 40);
    cmd_buf[5] = (crc << 1) | 0x01;

    cmd_output();
    for (int byte = 0; byte < 6; byte++) {
        for (int bit = 7; bit >= 0; bit--) {
            if ((cmd_buf[byte] >> bit) & 1)
                cmd_high();
            else
                cmd_low();
            clock_half_high();
            clock_half_low();
        }
    }

    cmd_high();
    cmd_input();

    bool got_start = false;
    for (int i = 0; i < 256; i++) {
        clock_half_high();
        if (!cmd_read()) {
            got_start = true;
            clock_half_low();
            break;
        }
        clock_half_low();
    }

    if (!got_start) return false;

    uint8_t resp_buf[6] = {0};
    for (int i = 1; i < 48; i++) {
        clock_half_high();
        int byte_idx = i / 8;
        int bit_idx = 7 - (i % 8);
        if (cmd_read())
            resp_buf[byte_idx] |= (1 << bit_idx);
        clock_half_low();
    }

    for (int i = 0; i < 8; i++) clock_cycle();

    uint32_t r = ((uint32_t)resp_buf[1] << 24) |
                 ((uint32_t)resp_buf[2] << 16) |
                 ((uint32_t)resp_buf[3] << 8)  |
                 (uint32_t)resp_buf[4];

    if (resp) *resp = r;
    return true;
}

bool sdio_send_cmd(uint8_t cmd_index, uint32_t arg, uint32_t *resp) {
    for (int retry = 0; retry < CMD_RETRIES; retry++) {
        if (sdio_send_cmd_once(cmd_index, arg, resp))
            return true;
        if (retry < CMD_RETRIES - 1) {
            for (int i = 0; i < 16; i++) clock_cycle();
        }
    }
    printf("[SDIO] CMD%d: no response after %d retries\n", cmd_index, CMD_RETRIES);
    return false;
}

// ============================================================
// CMD52: IO_RW_DIRECT
// ============================================================
bool sdio_cmd52_read(uint8_t fn, uint32_t addr, uint8_t *data) {
    uint32_t arg = ((uint32_t)(fn & 0x7) << 28) |
                   ((addr & 0x1FFFF) << 9);
    uint32_t resp = 0;
    if (!sdio_send_cmd(SDIO_CMD52, arg, &resp))
        return false;
    if (data) *data = resp & 0xFF;
    uint8_t flags = (resp >> 8) & 0xFF;
    if (flags & 0xC0) return false;
    return true;
}

bool sdio_cmd52_write(uint8_t fn, uint32_t addr, uint8_t data, uint8_t *resp_data) {
    uint32_t arg = (1UL << 31) |
                   ((uint32_t)(fn & 0x7) << 28) |
                   ((addr & 0x1FFFF) << 9) |
                   data;
    uint32_t resp = 0;
    if (!sdio_send_cmd(SDIO_CMD52, arg, &resp))
        return false;
    if (resp_data) *resp_data = resp & 0xFF;
    uint8_t flags = (resp >> 8) & 0xFF;
    if (flags & 0xC0) return false;
    return true;
}

// ============================================================
// CMD53: IO_RW_EXTENDED (read) with retry
// ============================================================
// Tight inline read loop — no function call overhead
static bool sdio_cmd53_read_once(uint8_t fn, uint32_t addr,
    uint8_t *buf, uint16_t count, bool block_mode, bool inc_addr) {
    uint32_t arg = ((uint32_t)(fn & 0x7) << 28) |
                   ((block_mode ? 1UL : 0UL) << 27) |
                   ((inc_addr ? 1UL : 0UL) << 26) |
                   ((addr & 0x1FFFF) << 9) |
                   (count & 0x1FF);

    uint32_t resp = 0;
    if (!sdio_send_cmd(SDIO_CMD53, arg, &resp)) {
        if (count > 1) printf("[SDIO] CMD53 cmd phase fail cnt=%d\n", count);
        return false;
    }

    uint8_t flags = (resp >> 8) & 0xFF;
    if (flags & 0xC0) {
        if (count > 1) printf("[SDIO] CMD53 flags=0x%02X cnt=%d\n", flags, count);
        return false;
    }

    dat_input();

    if (block_mode) {
        // Multi-block: each block has start + 512 bytes + CRC16(16 clk) + end
        for (uint16_t blk = 0; blk < count; blk++) {
            // Wait for start bit (all DAT lines low)
            bool got_start = false;
            for (int i = 0; i < 100000; i++) {
                clock_half_high();
                uint8_t d = dat_read4();
                clock_half_low();
                if (d == 0x00) { got_start = true; break; }
            }
            if (!got_start) {
                if (count > 1) printf("[SDIO] CMD53 no start blk=%d/%d\n", blk, count);
                return false;
            }

            // Read 512 bytes (1024 nibbles)
            uint8_t *dst = buf + (uint32_t)blk * 512;
            for (uint16_t i = 0; i < 512; i++) {
                clock_half_high();
                uint8_t hi = dat_read4();
                clock_half_low();
                clock_half_high();
                uint8_t lo = dat_read4();
                clock_half_low();
                dst[i] = (hi << 4) | lo;
            }

            // CRC16 (16 clocks) + end bit + gap
            for (int i = 0; i < 24; i++) clock_cycle();
        }
    } else {
        uint16_t total_bytes = (count == 0) ? 512 : count;

        // Wait for start bit
        bool got_start = false;
        for (int i = 0; i < 100000; i++) {
            clock_half_high();
            uint8_t d = dat_read4();
            clock_half_low();
            if (d == 0x00) { got_start = true; break; }
        }
        if (!got_start) return false;

        for (uint16_t i = 0; i < total_bytes; i++) {
            clock_half_high();
            uint8_t hi = dat_read4();
            clock_half_low();
            clock_half_high();
            uint8_t lo = dat_read4();
            clock_half_low();
            buf[i] = (hi << 4) | lo;
        }

        for (int i = 0; i < 24; i++) clock_cycle();
    }
    return true;
}

bool sdio_cmd53_read(uint8_t fn, uint32_t addr, uint8_t *buf, uint16_t count,
                     bool block_mode, bool inc_addr) {
    for (int retry = 0; retry < DATA_RETRIES; retry++) {
        if (sdio_cmd53_read_once(fn, addr, buf, count, block_mode, inc_addr))
            return true;
        if (retry < DATA_RETRIES - 1) {
            printf("[SDIO] CMD53 RD retry %d\n", retry + 1);
            for (int i = 0; i < 32; i++) clock_cycle();
        }
    }
    printf("[SDIO] CMD53 RD failed after %d retries\n", DATA_RETRIES);
    return false;
}

// ============================================================
// CMD53: IO_RW_EXTENDED (write) with CRC16
// ============================================================
static bool sdio_cmd53_write_once(uint8_t fn, uint32_t addr, const uint8_t *buf, uint16_t count,
                                   bool block_mode, bool inc_addr) {
    uint32_t arg = (1UL << 31) |
                   ((uint32_t)(fn & 0x7) << 28) |
                   ((block_mode ? 1UL : 0UL) << 27) |
                   ((inc_addr ? 1UL : 0UL) << 26) |
                   ((addr & 0x1FFFF) << 9) |
                   (count & 0x1FF);

    uint32_t resp = 0;
    if (!sdio_send_cmd(SDIO_CMD53, arg, &resp))
        return false;

    uint8_t flags = (resp >> 8) & 0xFF;
    if (flags & 0xC0) return false;

    uint16_t total_bytes = block_mode ? (count * 512) : ((count == 0) ? 512 : count);

    // Compute CRC16 for each DAT line before sending
    uint16_t crc_dat[4];
    for (int line = 0; line < 4; line++) {
        crc_dat[line] = crc16_dat_line(buf, total_bytes, line);
    }

    dat_output();

    // Start bit (all DAT lines low)
    dat_write4(0x00);
    clock_cycle();

    // Data nibbles
    for (uint16_t i = 0; i < total_bytes; i++) {
        dat_write4((buf[i] >> 4) & 0x0F);
        clock_cycle();
        dat_write4(buf[i] & 0x0F);
        clock_cycle();
    }

    // CRC16: 16 bits per line, MSB first
    for (int bit = 15; bit >= 0; bit--) {
        uint8_t nibble = 0;
        if ((crc_dat[3] >> bit) & 1) nibble |= 0x08;
        if ((crc_dat[2] >> bit) & 1) nibble |= 0x04;
        if ((crc_dat[1] >> bit) & 1) nibble |= 0x02;
        if ((crc_dat[0] >> bit) & 1) nibble |= 0x01;
        dat_write4(nibble);
        clock_cycle();
    }

    // Stop bit (all high)
    dat_write4(0x0F);
    clock_cycle();

    dat_input();

    // Wait for CRC status response from card (3 bits on DAT0: start + 2-bit status + end)
    // Then wait for busy (DAT0 low) to clear
    for (int i = 0; i < 1000000; i++) {
        clock_half_high();
        if (gpio_get(SDIO_DAT0_PIN)) {
            clock_half_low();
            break;
        }
        clock_half_low();
    }

    for (int i = 0; i < 8; i++) clock_cycle();
    return true;
}

bool sdio_cmd53_write(uint8_t fn, uint32_t addr, const uint8_t *buf, uint16_t count,
                      bool block_mode, bool inc_addr) {
    for (int retry = 0; retry < DATA_RETRIES; retry++) {
        if (sdio_cmd53_write_once(fn, addr, buf, count, block_mode, inc_addr))
            return true;
        if (retry < DATA_RETRIES - 1) {
            printf("[SDIO] CMD53 WR retry %d\n", retry + 1);
            for (int i = 0; i < 32; i++) clock_cycle();
        }
    }
    printf("[SDIO] CMD53 WR failed after %d retries\n", DATA_RETRIES);
    return false;
}

// ============================================================
// SDIO Card Init (from Nokia N91 trace)
// ============================================================
bool sdio_card_init(void) {
    uint32_t resp, dummy;

    printf("[SDIO] Starting card init...\n");
    sdio_set_clock_slow();

    // Cleanup prior state
    sdio_send_cmd(SDIO_CMD52, (1UL << 31) | (0x06 << 9) | 0x01, &dummy);
    sdio_send_cmd(SDIO_CMD7, 0x00000000, &dummy);
    for (int i = 0; i < 100; i++) clock_cycle();

    cmd_output();
    cmd_high();
    for (int i = 0; i < 80; i++) clock_cycle();

    // CMD5 probe
    printf("[SDIO] CMD5 probe...\n");
    if (!sdio_send_cmd(SDIO_CMD5, 0x00000000, &resp)) {
        printf("[SDIO] CMD5 probe failed\n");
        return false;
    }

    uint32_t card_ocr = resp & 0x00FFFFFF;
    uint32_t host_ocr = card_ocr ? (card_ocr & 0x00FF8000) : 0x001C0000;

    // CMD5 with OCR until ready
    bool ready = false;
    for (int retry = 0; retry < 100; retry++) {
        if (!sdio_send_cmd(SDIO_CMD5, host_ocr, &resp)) {
            sleep_ms(100);
            continue;
        }
        if (resp & 0x80000000) {
            ready = true;
            printf("[SDIO] Card ready (retry %d)\n", retry);
            break;
        }
        sleep_ms(500);
    }
    if (!ready) return false;

    // CMD3
    printf("[SDIO] CMD3...\n");
    if (!sdio_send_cmd(SDIO_CMD3, 0x00000000, &resp)) {
        sdio_send_cmd(SDIO_CMD7, 0x00000000, &dummy);
        sleep_ms(10);
        for (int i = 0; i < 80; i++) clock_cycle();
        if (!sdio_send_cmd(SDIO_CMD3, 0x00000000, &resp)) {
            printf("[SDIO] CMD3 failed, using RCA=0x0001\n");
            card_rca = 0x0001;
            goto skip_cmd3;
        }
    }
    card_rca = (resp >> 16) & 0xFFFF;
    printf("[SDIO] RCA=0x%04X\n", card_rca);

skip_cmd3:
    // CMD7 select
    if (!sdio_send_cmd(SDIO_CMD7, (uint32_t)card_rca << 16, &resp)) {
        printf("[SDIO] CMD7 failed\n");
        return false;
    }

    uint8_t rd;
    // Enable 4-bit bus
    sdio_cmd52_write(0, CCCR_BUS_IF_CTRL, 0x02, &rd);
    // Set fn1 block size = 512
    sdio_cmd52_write(0, FBR1_BLKSZ_LO, 0x00, &rd);
    sdio_cmd52_write(0, FBR1_BLKSZ_HI, 0x02, &rd);
    // Enable interrupts
    sdio_cmd52_write(0, CCCR_INT_ENABLE, 0x03, &rd);
    // Enable fn1
    sdio_cmd52_write(0, CCCR_IO_ENABLE, 0x02, &rd);

    // Wait for fn1 ready
    for (int i = 0; i < 50; i++) {
        sdio_cmd52_read(0, CCCR_IO_READY, &rd);
        if (rd & 0x02) break;
        sleep_ms(100);
    }

    printf("[SDIO] Switching to fast clock\n");
    sdio_set_clock_fast();
    printf("[SDIO] Card init complete!\n");
    return true;
}
