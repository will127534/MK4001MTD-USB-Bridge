#ifndef SDIO_PIO_H
#define SDIO_PIO_H

#include <stdint.h>
#include <stdbool.h>

// Initialize PIO SDIO (claim SM0 + DMA channel, load CMD program)
void sdio_pio_init(void);
bool sdio_pio_is_ready(void);

// Set PIO clock divider (lower = faster). 62.5 = ~500 KHz, 3.125 = ~10 MHz
void sdio_pio_set_clkdiv(float div);

// Full SDIO card init: CMD5/CMD3/CMD7 + CCCR config.
// Assumes slow clock set by caller; switches to fast clock for CCCR.
// Used for both cold boot and power gate wake.
bool sdio_pio_card_init(void);

// Reclaim GPIO pins for PIO (call after power gate to reset pin state)
void sdio_pio_acquire_pins(void);

// PIO-based SDIO commands
bool sdio_pio_send_cmd(uint8_t cmd_index, uint32_t arg, uint32_t *resp);
bool sdio_pio_cmd52_read(uint8_t fn, uint32_t addr, uint8_t *data);
bool sdio_pio_cmd52_write(uint8_t fn, uint32_t addr, uint8_t data, uint8_t *resp_data);
bool sdio_pio_cmd53_read_block(uint8_t fn, uint32_t addr, uint8_t *buf, uint16_t block_count);
bool sdio_pio_cmd53_write_block(uint8_t fn, uint32_t addr, const uint8_t *buf, uint16_t block_count);

#endif
