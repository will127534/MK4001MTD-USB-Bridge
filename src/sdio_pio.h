#ifndef SDIO_PIO_H
#define SDIO_PIO_H

#include <stdint.h>
#include <stdbool.h>

// Enable PIO for fast SDIO after init (init still uses bit-bang at 400KHz)
void sdio_pio_init(void);
bool sdio_pio_is_ready(void);

// PIO-based command send+receive (replaces bit-bang sdio_send_cmd)
bool sdio_pio_send_cmd(uint8_t cmd_index, uint32_t arg, uint32_t *resp);

// PIO-based CMD52 wrappers
bool sdio_pio_cmd52_read(uint8_t fn, uint32_t addr, uint8_t *data);
bool sdio_pio_cmd52_write(uint8_t fn, uint32_t addr, uint8_t data, uint8_t *resp_data);

// PIO-based CMD53 block read (uses DAT read SM + DMA)
bool sdio_pio_cmd53_read_block(uint8_t fn, uint32_t addr, uint8_t *buf, uint16_t block_count);

// PIO-based CMD53 block write (CMD via PIO, data via bit-bang for now)
bool sdio_pio_cmd53_write_block(uint8_t fn, uint32_t addr, const uint8_t *buf, uint16_t block_count);

// Set PIO clock divider (lower = faster)
void sdio_pio_set_clkdiv(float div);

// Pin ownership: release for bit-bang, acquire for PIO
void sdio_pio_release_pins(void);
void sdio_pio_acquire_pins(void);

#endif
