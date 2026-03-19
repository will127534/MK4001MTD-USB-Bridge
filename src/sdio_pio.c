/*
 * PIO-accelerated SDIO for MK4001MTD Bridge
 *
 * Single PIO0 SM0 — programs swapped on the fly:
 *   CMD tx/rx (26 instr) — loaded for CMD52/CMD53 command phase
 *   DAT read  (13 instr) — loaded for CMD53 read data phase
 *   DAT write (15 instr) — loaded for CMD53 write data phase
 *
 * Only one program loaded at a time → full 32 slots available.
 * Program swap cost is ~1µs (just bitmask + register writes).
 */

#include "sdio_pio.h"
#include "sdio_hw.h"
#include "sdio.pio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

static PIO pio = pio0;
static const uint SM = 0;  // single state machine
static int dma_chan = -1;
static bool pio_ready = false;
static float current_clkdiv = 6.25f;
static bool cached_rca_valid = false;
static uint16_t cached_rca = 0;

// Track which program is loaded
typedef enum { PGM_NONE, PGM_CMD, PGM_DAT_RD, PGM_DAT_WR } pgm_t;
static pgm_t loaded_pgm = PGM_NONE;
static const uint FIXED_OFFSET = 0;  // always load at offset 0

// ============================================================
// CRC helpers
// ============================================================
static uint8_t pio_crc7(const uint8_t *data, int bits) {
    uint8_t crc = 0;
    for (int i = 0; i < bits; i++) {
        uint8_t bit = (data[i / 8] >> (7 - (i % 8))) & 1;
        uint8_t fb = ((crc >> 6) ^ bit) & 1;
        crc = (crc << 1) & 0x7F;
        if (fb) crc ^= 0x09;
    }
    return crc & 0x7F;
}

// Byte-wise CRC16-CCITT lookup table (polynomial 0x1021)
static const uint16_t crc16_lut[256] = {
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
    0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
    0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
    0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
    0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,
    0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
    0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
    0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
    0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,
    0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
    0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,
    0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
    0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
    0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
    0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
    0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
    0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
    0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
    0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
    0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
    0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,
    0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
    0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
    0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
    0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
    0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,
    0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
    0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
    0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
    0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
    0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0,
};

static inline uint16_t crc16_byte(uint16_t crc, uint8_t byte) {
    return (crc << 8) ^ crc16_lut[((crc >> 8) ^ byte) & 0xFF];
}

// Compute CRC16 for all 4 DAT lines using byte-wise LUT
// Extracts bits for each line, packs into bytes, then LUT-driven CRC
static void crc16_all_lines(const uint8_t *data, int total_bytes, uint16_t crc_out[4]) {
    // Each DAT line gets 2 bits per data byte (1 from hi nibble, 1 from lo nibble)
    // 512 data bytes → 1024 bits per line → 128 packed bytes per line
    // Process 4 data bytes at a time → produces 1 packed byte per line (8 bits)
    uint16_t c0 = 0, c1 = 0, c2 = 0, c3 = 0;

    for (int i = 0; i < total_bytes; i += 4) {
        // Extract 8 bits for each DAT line from 4 consecutive data bytes
        // Each data byte contributes 2 bits: hi_nibble[line] then lo_nibble[line]
        uint8_t b0 = data[i], b1 = data[i+1], b2 = data[i+2], b3 = data[i+3];

        uint8_t p0 = ((b0 >> 4) & 1) << 7 | ((b0 >> 0) & 1) << 6 |
                     ((b1 >> 4) & 1) << 5 | ((b1 >> 0) & 1) << 4 |
                     ((b2 >> 4) & 1) << 3 | ((b2 >> 0) & 1) << 2 |
                     ((b3 >> 4) & 1) << 1 | ((b3 >> 0) & 1);
        uint8_t p1 = ((b0 >> 5) & 1) << 7 | ((b0 >> 1) & 1) << 6 |
                     ((b1 >> 5) & 1) << 5 | ((b1 >> 1) & 1) << 4 |
                     ((b2 >> 5) & 1) << 3 | ((b2 >> 1) & 1) << 2 |
                     ((b3 >> 5) & 1) << 1 | ((b3 >> 1) & 1);
        uint8_t p2 = ((b0 >> 6) & 1) << 7 | ((b0 >> 2) & 1) << 6 |
                     ((b1 >> 6) & 1) << 5 | ((b1 >> 2) & 1) << 4 |
                     ((b2 >> 6) & 1) << 3 | ((b2 >> 2) & 1) << 2 |
                     ((b3 >> 6) & 1) << 1 | ((b3 >> 2) & 1);
        uint8_t p3 = ((b0 >> 7) & 1) << 7 | ((b0 >> 3) & 1) << 6 |
                     ((b1 >> 7) & 1) << 5 | ((b1 >> 3) & 1) << 4 |
                     ((b2 >> 7) & 1) << 3 | ((b2 >> 3) & 1) << 2 |
                     ((b3 >> 7) & 1) << 1 | ((b3 >> 3) & 1);

        c0 = crc16_byte(c0, p0);
        c1 = crc16_byte(c1, p1);
        c2 = crc16_byte(c2, p2);
        c3 = crc16_byte(c3, p3);
    }
    crc_out[0] = c0; crc_out[1] = c1; crc_out[2] = c2; crc_out[3] = c3;
}

// ============================================================
// Program swap — direct instruction memory overwrite at offset 0
// ============================================================
static bool gpio_inited = false;

static void ensure_gpio_init(void) {
    if (gpio_inited) return;
    pio_gpio_init(pio, SDIO_CLK_PIN);
    pio_gpio_init(pio, SDIO_CMD_PIN);
    gpio_pull_up(SDIO_CMD_PIN);
    for (int i = 0; i < 4; i++) {
        pio_gpio_init(pio, SDIO_DAT0_PIN + i);
        gpio_pull_up(SDIO_DAT0_PIN + i);
    }
    gpio_inited = true;
}

// Blast program instructions directly into PIO instruction memory at offset 0
static void load_program_raw(const pio_program_t *program) {
    for (uint i = 0; i < program->length; i++)
        pio->instr_mem[FIXED_OFFSET + i] = program->instructions[i];
}

static void load_cmd(void) {
    if (loaded_pgm == PGM_CMD) return;
    pio_sm_set_enabled(pio, SM, false);
    ensure_gpio_init();
    load_program_raw(&sdio_cmd_tx_rx_program);
    sdio_cmd_tx_rx_program_init(pio, SM, FIXED_OFFSET,
                                 SDIO_CLK_PIN, SDIO_CMD_PIN, current_clkdiv);
    loaded_pgm = PGM_CMD;
}

static void load_dat_read(void) {
    if (loaded_pgm == PGM_DAT_RD) return;
    pio_sm_set_enabled(pio, SM, false);
    load_program_raw(&sdio_dat_read_program);
    sdio_dat_read_program_reinit(pio, SM, FIXED_OFFSET,
                                  SDIO_CLK_PIN, SDIO_DAT0_PIN, current_clkdiv);
    pio_sm_set_consecutive_pindirs(pio, SM, SDIO_DAT0_PIN, 4, false);
    pio_sm_set_consecutive_pindirs(pio, SM, SDIO_CLK_PIN, 1, true);
    loaded_pgm = PGM_DAT_RD;
}

static void load_dat_write(void) {
    if (loaded_pgm == PGM_DAT_WR) return;
    pio_sm_set_enabled(pio, SM, false);
    load_program_raw(&sdio_dat_write_program);
    sdio_dat_write_program_reinit(pio, SM, FIXED_OFFSET,
                                   SDIO_CLK_PIN, SDIO_DAT0_PIN, current_clkdiv);
    pio_sm_set_consecutive_pindirs(pio, SM, SDIO_DAT0_PIN, 4, true);
    pio_sm_set_consecutive_pindirs(pio, SM, SDIO_CLK_PIN, 1, true);
    loaded_pgm = PGM_DAT_WR;
}

// ============================================================
// Init
// ============================================================
void sdio_pio_init(void) {
    if (pio_ready) return;

    // Claim DMA channel and SM
    dma_chan = dma_claim_unused_channel(true);
    pio_sm_claim(pio, SM);

    // Load CMD program initially
    load_cmd();

    pio_ready = true;
    printf("[PIO] Init OK: clkdiv=%.2f (~%.1f MHz), CMD@%u\n",
           current_clkdiv, 125.0f / (current_clkdiv * 4.0f), FIXED_OFFSET);
}

bool sdio_pio_is_ready(void) { return pio_ready; }

void sdio_pio_set_clkdiv(float div) {
    current_clkdiv = div;
    if (pio_ready)
        pio_sm_set_clkdiv(pio, SM, div);
}

// ============================================================
// CMD: send command + receive response
// ============================================================
static void cmd_sm_reset(void) {
    load_cmd();
    pio_sm_set_enabled(pio, SM, false);
    pio_sm_clear_fifos(pio, SM);
    pio_sm_restart(pio, SM);
    pio_sm_exec(pio, SM, pio_encode_set(pio_pindirs, 1));
    pio_sm_exec(pio, SM, pio_encode_set(pio_pins, 1));
    pio_sm_exec(pio, SM, pio_encode_jmp(FIXED_OFFSET));
    pio_sm_set_enabled(pio, SM, true);
}

bool sdio_pio_send_cmd(uint8_t cmd_index, uint32_t arg, uint32_t *resp) {
    if (!pio_ready) return false;

    uint8_t cmd_buf[6];
    cmd_buf[0] = 0x40 | (cmd_index & 0x3F);
    cmd_buf[1] = (arg >> 24) & 0xFF;
    cmd_buf[2] = (arg >> 16) & 0xFF;
    cmd_buf[3] = (arg >> 8)  & 0xFF;
    cmd_buf[4] = arg & 0xFF;
    cmd_buf[5] = (pio_crc7(cmd_buf, 40) << 1) | 0x01;

    cmd_sm_reset();

    uint32_t config = (47u << 16) | 39u;
    pio_sm_put_blocking(pio, SM, config);

    uint32_t w1 = ((uint32_t)cmd_buf[0] << 24) | ((uint32_t)cmd_buf[1] << 16) |
                  ((uint32_t)cmd_buf[2] << 8)  | (uint32_t)cmd_buf[3];
    uint32_t w2 = ((uint32_t)cmd_buf[4] << 24) | ((uint32_t)cmd_buf[5] << 16);
    pio_sm_put_blocking(pio, SM, w1);
    pio_sm_put_blocking(pio, SM, w2);

    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    while (pio_sm_is_rx_fifo_empty(pio, SM)) {
        if ((to_ms_since_boot(get_absolute_time()) - t0) > 100) {
            printf("[PIO] CMD%d TIMEOUT\n", cmd_index);
            pio_sm_set_enabled(pio, SM, false);
            return false;
        }
    }

    uint32_t rx = pio_sm_get(pio, SM);
    if (resp) *resp = rx;
    return true;
}

// ============================================================
// CMD52 wrappers
// ============================================================
static bool pio_cmd52(bool is_write, uint8_t fn, uint32_t addr, uint8_t data,
                       uint8_t *out_data) {
    uint32_t arg = ((is_write ? 1UL : 0UL) << 31) |
                   ((uint32_t)(fn & 0x7) << 28) |
                   ((addr & 0x1FFFF) << 9) |
                   (is_write ? data : 0);

    for (int retry = 0; retry < 3; retry++) {
        uint32_t resp = 0;
        if (sdio_pio_send_cmd(52, arg, &resp)) {
            uint8_t flags = (resp >> 8) & 0xFF;
            if (!(flags & 0xC0)) {
                if (out_data) *out_data = resp & 0xFF;
                return true;
            }
        }
    }
    return false;
}

bool sdio_pio_cmd52_read(uint8_t fn, uint32_t addr, uint8_t *data) {
    return pio_cmd52(false, fn, addr, 0, data);
}

bool sdio_pio_cmd52_write(uint8_t fn, uint32_t addr, uint8_t data, uint8_t *resp_data) {
    return pio_cmd52(true, fn, addr, data, resp_data);
}

// Commands like deselect-CMD7 do not provide a useful response here, so we just
// clock them out and stop the state machine after the command phase completes.
static bool sdio_pio_send_cmd_no_resp(uint8_t cmd_index, uint32_t arg) {
    if (!pio_ready) return false;

    uint8_t cmd_buf[6];
    cmd_buf[0] = 0x40 | (cmd_index & 0x3F);
    cmd_buf[1] = (arg >> 24) & 0xFF;
    cmd_buf[2] = (arg >> 16) & 0xFF;
    cmd_buf[3] = (arg >> 8)  & 0xFF;
    cmd_buf[4] = arg & 0xFF;
    cmd_buf[5] = (pio_crc7(cmd_buf, 40) << 1) | 0x01;

    cmd_sm_reset();

    // The RX count is ignored here; we stop the state machine after the command
    // has had enough time to clock out on the wire.
    uint32_t config = (47u << 16) | 39u;
    pio_sm_put_blocking(pio, SM, config);

    uint32_t w1 = ((uint32_t)cmd_buf[0] << 24) | ((uint32_t)cmd_buf[1] << 16) |
                  ((uint32_t)cmd_buf[2] << 8)  | (uint32_t)cmd_buf[3];
    uint32_t w2 = ((uint32_t)cmd_buf[4] << 24) | ((uint32_t)cmd_buf[5] << 16);
    pio_sm_put_blocking(pio, SM, w1);
    pio_sm_put_blocking(pio, SM, w2);

    // 48 command bits + turnaround/guard clocks at the slow init clock fit well
    // within 1 ms. Stopping the SM avoids waiting forever for a response-less
    // command.
    sleep_ms(1);
    pio_sm_set_enabled(pio, SM, false);
    pio_sm_clear_fifos(pio, SM);
    return true;
}

bool sdio_pio_deselect_card(void) {
    return sdio_pio_send_cmd_no_resp(7, 0x00000000);
}

// ============================================================
// CMD53 read
// ============================================================
// Read a single block's data phase via PIO DAT read (assumes DAT program loaded)
// Returns true on success. buf must hold 512 bytes.
static bool pio_dat_read_one_block(uint8_t *buf) {
    uint32_t data_nibbles = 512 * 2;       // 1024
    uint32_t read_nibbles = data_nibbles + 16 + 1;  // +CRC+end
    uint32_t data_words = 512 / 4;         // 128

    // Reset SM for this block
    pio_sm_set_enabled(pio, SM, false);
    pio_sm_clear_fifos(pio, SM);
    pio_sm_restart(pio, SM);
    pio_sm_exec(pio, SM, pio_encode_jmp(FIXED_OFFSET));

    // DMA: PIO RX → buffer
    dma_channel_config dc = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_dreq(&dc, pio_get_dreq(pio, SM, false));
    dma_channel_configure(dma_chan, &dc, buf, &pio->rxf[SM], data_words, false);

    pio_sm_put_blocking(pio, SM, read_nibbles - 1);
    dma_channel_start(dma_chan);
    pio_sm_set_enabled(pio, SM, true);

    // Wait for DMA (data nibbles captured)
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    while (dma_channel_is_busy(dma_chan)) {
        if ((to_ms_since_boot(get_absolute_time()) - t0) > 5000) {
            dma_channel_abort(dma_chan);
            pio_sm_set_enabled(pio, SM, false);
            return false;
        }
        tight_loop_contents();
    }

    // SM is still running — clocking CRC+end nibbles into FIFO.
    // Wait for SM to finish all nibbles (stalls at `pull block` when done).
    // The SM pushes autopush words + explicit push. Once TX FIFO is empty
    // AND SM has stalled on pull, it's done. We detect this by waiting for
    // the SM to have consumed all clocks — just wait for FIFO to fill with
    // the remaining CRC words, then drain them.
    uint32_t t1 = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - t1) < 100) {
        // SM stalls on `pull block` when X reaches 0 (all nibbles read).
        // Check if SM PC is at offset+0 (pull instruction = first instruction)
        uint32_t pc = pio_sm_get_pc(pio, SM);
        if (pc == FIXED_OFFSET) break;  // SM finished, stalled at pull
        tight_loop_contents();
    }
    pio_sm_set_enabled(pio, SM, false);

    // Repack nibbles → bytes in-place
    uint32_t *words = (uint32_t *)buf;
    for (int w = data_words - 1; w >= 0; w--) {
        uint32_t word = words[w];
        uint8_t *dst = buf + w * 4;
        dst[0] = ((word >> 28) << 4) | ((word >> 24) & 0x0F);
        dst[1] = (((word >> 20) & 0x0F) << 4) | ((word >> 16) & 0x0F);
        dst[2] = (((word >> 12) & 0x0F) << 4) | ((word >> 8) & 0x0F);
        dst[3] = (((word >> 4) & 0x0F) << 4) | (word & 0x0F);
    }

    // Read CRC nibbles from FIFO (2 words = 16 CRC nibbles, then 1 partial with end bit)
    // Each CRC nibble is: bit3=DAT3, bit2=DAT2, bit1=DAT1, bit0=DAT0
    uint32_t crc_words[3] = {0, 0, 0};
    int crc_word_count = 0;
    while (!pio_sm_is_rx_fifo_empty(pio, SM) && crc_word_count < 3)
        crc_words[crc_word_count++] = pio_sm_get(pio, SM);

    // Drain any remaining
    while (!pio_sm_is_rx_fifo_empty(pio, SM))
        (void)pio_sm_get(pio, SM);

    // Verify CRC16 per DAT line if we got at least the 2 CRC words
    if (crc_word_count >= 2) {
        // Extract received CRC16 per line from the 16 nibbles (2 words × 8 nibbles)
        // Nibbles are packed MSB-first: word0 has CRC bits 15..8, word1 has bits 7..0
        // Compute expected CRC for all 4 lines in one pass (LUT-accelerated)
        uint16_t expected[4];
        crc16_all_lines(buf, 512, expected);

        for (int line = 0; line < 4; line++) {
            uint16_t received_crc = 0;
            for (int n = 0; n < 8; n++) {
                uint8_t nibble = (crc_words[0] >> (28 - n * 4)) & 0xF;
                received_crc = (received_crc << 1) | ((nibble >> line) & 1);
            }
            for (int n = 0; n < 8; n++) {
                uint8_t nibble = (crc_words[1] >> (28 - n * 4)) & 0xF;
                received_crc = (received_crc << 1) | ((nibble >> line) & 1);
            }
            if (received_crc != expected[line]) {
                printf("[PIO] CRC16 MISMATCH DAT%d: rx=0x%04X exp=0x%04X\n",
                       line, received_crc, expected[line]);
                return false;
            }
        }
    }

    return true;
}

bool sdio_pio_cmd53_read_block(uint8_t fn, uint32_t addr, uint8_t *buf,
                                uint16_t block_count) {
    if (!pio_ready) return false;

    // Send CMD53 via CMD program
    uint32_t arg = ((uint32_t)(fn & 0x7) << 28) |
                   (1UL << 27) |
                   ((addr & 0x1FFFF) << 9) |
                   (block_count & 0x1FF);

    uint32_t resp = 0;
    if (!sdio_pio_send_cmd(53, arg, &resp))
        return false;
    if (((resp >> 8) & 0xFF) & 0xC0) return false;

    // Stop CMD SM, swap to DAT read program
    pio_sm_set_enabled(pio, SM, false);
    load_dat_read();

    // Read each block individually — PIO wait_dat_start handles inter-block gaps
    for (uint16_t blk = 0; blk < block_count; blk++) {
        if (!pio_dat_read_one_block(buf + (uint32_t)blk * 512)) {
            printf("[PIO] DAT read fail blk=%d/%d\n", blk, block_count);
            load_cmd();
            return false;
        }
    }

    // Swap back to CMD
    load_cmd();
    return true;
}

// ============================================================
// CMD53 write (per-block CRC16, DMA to PIO)
// ============================================================
// Buffer for one block: start(1) + data(1024) + CRC(16) + stop(1) = 1042 nibbles
// = 131 words
static uint32_t write_stream_buf[((512 * 2 + 16 + 2 + 7) / 8)];

// Write a single block's data phase via PIO DAT write
// Returns true on success. data must point to 512 bytes.
static bool pio_dat_write_one_block(const uint8_t *data) {
    // Precompute CRC16 per DAT line for this block (LUT-accelerated)
    uint16_t crc_dat[4];
    crc16_all_lines(data, 512, crc_dat);

    // Build nibble stream: start(1) + data(1024) + CRC16(16) + stop(1)
    uint32_t nibble_count = 1 + 512 * 2 + 16 + 1;
    uint32_t word_count = (nibble_count + 7) / 8;
    memset(write_stream_buf, 0, word_count * 4);

    #define SET_NIBBLE(pos, val) do { \
        uint32_t wi = (pos) / 8; \
        uint32_t shift = 28 - ((pos) % 8) * 4; \
        write_stream_buf[wi] |= ((uint32_t)(val) & 0xF) << shift; \
    } while(0)

    uint32_t ni = 0;
    SET_NIBBLE(ni, 0x0); ni++;
    for (uint32_t i = 0; i < 512; i++) {
        SET_NIBBLE(ni, (data[i] >> 4) & 0x0F); ni++;
        SET_NIBBLE(ni, data[i] & 0x0F); ni++;
    }
    for (int bit = 15; bit >= 0; bit--) {
        uint8_t nibble = 0;
        if ((crc_dat[3] >> bit) & 1) nibble |= 0x08;
        if ((crc_dat[2] >> bit) & 1) nibble |= 0x04;
        if ((crc_dat[1] >> bit) & 1) nibble |= 0x02;
        if ((crc_dat[0] >> bit) & 1) nibble |= 0x01;
        SET_NIBBLE(ni, nibble); ni++;
    }
    SET_NIBBLE(ni, 0xF); ni++;
    #undef SET_NIBBLE

    // Reset SM for this block
    pio_sm_set_enabled(pio, SM, false);
    pio_sm_clear_fifos(pio, SM);
    pio_sm_restart(pio, SM);
    pio_sm_set_consecutive_pindirs(pio, SM, SDIO_DAT0_PIN, 4, true);
    pio_sm_exec(pio, SM, pio_encode_jmp(FIXED_OFFSET));

    // DMA: write_stream → PIO TX
    dma_channel_config dc = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, true);
    channel_config_set_write_increment(&dc, false);
    channel_config_set_dreq(&dc, pio_get_dreq(pio, SM, true));
    dma_channel_configure(dma_chan, &dc, &pio->txf[SM], write_stream_buf,
                          word_count, false);

    // Clear IRQ flag before starting
    pio->irq = (1u << 0);

    // Push nibble count, start DMA + SM
    pio_sm_put_blocking(pio, SM, nibble_count - 1);
    dma_channel_start(dma_chan);
    pio_sm_set_enabled(pio, SM, true);

    // Wait for IRQ 0 (write complete + card CRC status received + busy cleared)
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    while (!(pio->irq & (1u << 0))) {
        if ((to_ms_since_boot(get_absolute_time()) - t0) > 10000) {
            dma_channel_abort(dma_chan);
            pio_sm_set_enabled(pio, SM, false);
            printf("[PIO] Write blk timeout (DMA busy=%d, SM PC=%u)\n",
                   dma_channel_is_busy(dma_chan),
                   pio_sm_get_pc(pio, SM));
            return false;
        }
        tight_loop_contents();
    }

    pio_sm_set_enabled(pio, SM, false);
    return true;
}

bool sdio_pio_cmd53_write_block(uint8_t fn, uint32_t addr, const uint8_t *buf,
                                 uint16_t block_count) {
    if (!pio_ready) return false;

    // Send CMD53 write via CMD program
    uint32_t cmd_arg = (1UL << 31) |
                       ((uint32_t)(fn & 0x7) << 28) |
                       (1UL << 27) |
                       ((addr & 0x1FFFF) << 9) |
                       (block_count & 0x1FF);

    uint32_t resp = 0;
    if (!sdio_pio_send_cmd(53, cmd_arg, &resp))
        return false;
    uint8_t flags = (resp >> 8) & 0xFF;
    if (flags & 0xC0) return false;

    // Stop CMD, swap to DAT write program
    pio_sm_set_enabled(pio, SM, false);
    load_dat_write();

    // Write each block individually (each has its own CRC + busy wait)
    for (uint16_t blk = 0; blk < block_count; blk++) {
        if (!pio_dat_write_one_block(buf + (uint32_t)blk * 512)) {
            printf("[PIO] DAT write fail blk=%d/%d\n", blk, block_count);
            pio_sm_set_consecutive_pindirs(pio, SM, SDIO_DAT0_PIN, 4, false);
            load_cmd();
            return false;
        }
    }

    // Restore DAT pins to input
    pio_sm_set_consecutive_pindirs(pio, SM, SDIO_DAT0_PIN, 4, false);

    // Swap back to CMD
    load_cmd();

    return true;
}

// ============================================================
// Pin management — reclaim GPIO for PIO after power gate
// ============================================================
void sdio_pio_acquire_pins(void) {
    if (!pio_ready) return;
    pio_gpio_init(pio, SDIO_CLK_PIN);
    pio_gpio_init(pio, SDIO_CMD_PIN);
    for (int i = 0; i < 4; i++)
        pio_gpio_init(pio, SDIO_DAT0_PIN + i);
    pio_sm_set_consecutive_pindirs(pio, SM, SDIO_CLK_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(pio, SM, SDIO_CMD_PIN, 1, true);
    loaded_pgm = PGM_NONE;  // force program reload on next use
}

// ============================================================
// SDIO card init — CMD5/CMD3/CMD7 + CCCR config (all PIO)
// Used for both cold boot and power gate wake.
// Assumes slow clock already set; switches to fast clock for CCCR.
// ============================================================
#define INIT_FAST_CLKDIV  3.125f
#define CMD5_READY_TIMEOUT_MS 500u
#define CMD5_POLL_INTERVAL_MS 30u

static bool sdio_pio_wait_ready_ocr(uint32_t host_ocr, uint32_t *out_resp) {
    uint32_t resp = 0;
    bool saw_unsettled_ready = false;
    uint32_t start = to_ms_since_boot(get_absolute_time());

    while ((to_ms_since_boot(get_absolute_time()) - start) < CMD5_READY_TIMEOUT_MS) {
        if (sdio_pio_send_cmd(5, host_ocr, &resp) && (resp & 0x80000000)) {
            uint32_t ready_ocr = resp & 0x00FFFFFF;
            if (ready_ocr == host_ocr) {
                if (out_resp) *out_resp = resp;
                return true;
            }

            if (!saw_unsettled_ready) {
                printf("[SDIO] CMD5 ready but OCR not settled yet (resp=0x%08lX, want=0x%08lX)\n",
                       resp, 0x80000000u | host_ocr);
                saw_unsettled_ready = true;
            }

            sleep_ms(CMD5_POLL_INTERVAL_MS);
            continue;
        }

        sleep_ms(CMD5_POLL_INTERVAL_MS);
    }
    return false;
}

static bool sdio_pio_card_init_once(uint16_t *out_rca) {
    uint32_t resp;

    // CMD5 probe (no argument — discovers card OCR)
    if (!sdio_pio_send_cmd(5, 0x00000000, &resp)) return false;

    uint32_t card_ocr = resp & 0x00FFFFFF;
    // Follow the N91 traces here: probe may report 0x1F8000, but the working
    // host negotiation window is 0x001C0000 rather than mirroring the full
    // probed OCR mask.
    uint32_t host_ocr = (card_ocr & 0x001C0000) ? 0x001C0000 : 0x001C0000;

    // CMD5 with OCR until card signals ready (bit 31)
    if (!sdio_pio_wait_ready_ocr(host_ocr, &resp)) return false;
    printf("[SDIO] CMD5 ready (OCR=0x%08lX)\n", resp);

    // Cold boot can assert CMD5 ready slightly before the RCA assignment path is
    // stable. Retry CMD3 a few times with a fresh CMD5 ready poll before falling
    // back to the warm-restart deselect path.
    for (int cmd3_attempt = 0; cmd3_attempt < 4; cmd3_attempt++) {
        sleep_ms(cmd3_attempt == 0 ? 20 : 30);
        if (sdio_pio_send_cmd(3, 0x00000000, &resp)) {
            *out_rca = (resp >> 16) & 0xFFFF;
            if (cmd3_attempt == 0) {
                printf("[SDIO] RCA=0x%04X\n", *out_rca);
            } else {
                printf("[SDIO] RCA=0x%04X (after CMD5/CMD3 retry %d)\n", *out_rca, cmd3_attempt);
            }
            return true;
        }

        if (cmd3_attempt != 3) {
            printf("[SDIO] CMD3 failed, re-polling CMD5...\n");
            if (!sdio_pio_wait_ready_ocr(host_ocr, &resp)) break;
            printf("[SDIO] CMD5 ready (OCR=0x%08lX)\n", resp);
        }
    }

    // Warm MCU restarts on this board leave the drive powered, so the SDIO side
    // can still be sitting in a selected state from the previous host instance.
    printf("[SDIO] CMD3 failed, trying CMD7 deselect...\n");
    sdio_pio_deselect_card();
    sleep_ms(2);
    if (sdio_pio_send_cmd(3, 0x00000000, &resp)) {
        *out_rca = (resp >> 16) & 0xFFFF;
        printf("[SDIO] RCA=0x%04X (after deselect)\n", *out_rca);
        return true;
    }

    *out_rca = 0x0001;
    printf("[SDIO] CMD3 still failed, trying retained RCA=0x%04X\n", *out_rca);
    return true;
}

static bool sdio_pio_finish_init_with_rca(uint16_t rca) {
    uint32_t resp;

    // CMD7 — select card (enters Transfer state)
    if (!sdio_pio_send_cmd(7, (uint32_t)rca << 16, &resp)) {
        printf("[SDIO] CMD7 failed\n");
        return false;
    }

    // Switch to fast clock — CMD52 CCCR config requires full speed
    sdio_pio_set_clkdiv(INIT_FAST_CLKDIV);
    sleep_ms(10);

    // CCCR configuration
    uint8_t rd;
    sdio_pio_cmd52_write(0, CCCR_BUS_IF_CTRL, 0x02, &rd);  // 4-bit bus
    sdio_pio_cmd52_write(0, FBR1_BLKSZ_LO,    0x00, &rd);   // block size = 512
    sdio_pio_cmd52_write(0, FBR1_BLKSZ_HI,    0x02, &rd);
    sdio_pio_cmd52_write(0, CCCR_INT_ENABLE,   0x03, &rd);   // master + fn1 int
    sdio_pio_cmd52_write(0, CCCR_IO_ENABLE,    0x02, &rd);   // enable fn1

    // Wait for fn1 ready — required before CMD53 data transfers
    bool fn1_ok = false;
    for (int i = 0; i < 100; i++) {
        if (sdio_pio_cmd52_read(0, CCCR_IO_READY, &rd) && (rd & 0x02)) {
            printf("[SDIO] fn1 ready (attempt %d)\n", i);
            fn1_ok = true;
            break;
        }
        sleep_ms(100);
    }
    if (!fn1_ok) {
        printf("[SDIO] fn1 not ready!\n");
        return false;
    }

    cached_rca = rca;
    cached_rca_valid = true;
    return true;
}

bool sdio_pio_try_warm_probe(void) {
    if (!cached_rca_valid) return false;

    printf("[SDIO] Warm probe: trying cached RCA=0x%04X\n", cached_rca);
    if (sdio_pio_finish_init_with_rca(cached_rca)) {
        printf("[SDIO] Warm probe OK\n");
        return true;
    }

    printf("[SDIO] Warm probe failed, clearing cached RCA\n");
    cached_rca_valid = false;
    cached_rca = 0;
    return false;
}

bool sdio_pio_card_init(void) {
    uint16_t rca = 0;

    if (!sdio_pio_card_init_once(&rca)) return false;
    return sdio_pio_finish_init_with_rca(rca);
}
