#ifndef SDIO_HW_H
#define SDIO_HW_H

#include <stdint.h>
#include <stdbool.h>

// Pin assignments (GP0/GP1 dead on this Pico)
#define SDIO_CLK_PIN  2
#define SDIO_CMD_PIN  3
#define SDIO_DAT0_PIN 4
#define SDIO_DAT1_PIN 5
#define SDIO_DAT2_PIN 6
#define SDIO_DAT3_PIN 7

#define HDD_POWER_PIN 13  // GP13: HIGH=off, LOW=on

// SDIO command indices
#define SDIO_CMD5   5   // IO_SEND_OP_COND
#define SDIO_CMD3   3   // SEND_RELATIVE_ADDR
#define SDIO_CMD7   7   // SELECT_CARD
#define SDIO_CMD52  52  // IO_RW_DIRECT
#define SDIO_CMD53  53  // IO_RW_EXTENDED

// CCCR registers
#define CCCR_SDIO_REV       0x00
#define CCCR_SD_REV         0x01
#define CCCR_IO_ENABLE      0x02
#define CCCR_IO_READY       0x03
#define CCCR_INT_ENABLE     0x04
#define CCCR_INT_PENDING    0x05
#define CCCR_IO_ABORT       0x06
#define CCCR_BUS_IF_CTRL    0x07
#define CCCR_CARD_CAP       0x08
#define CCCR_FN0_BLKSZ_LO  0x10
#define CCCR_FN0_BLKSZ_HI  0x11
#define CCCR_POWER_CTRL     0x12
#define CCCR_HIGH_SPEED     0x13

// FBR (Function Basic Registers) for fn1
#define FBR1_BASE           0x100
#define FBR1_BLKSZ_LO      0x110
#define FBR1_BLKSZ_HI      0x111

// ATA registers (fn1)
#define ATA_REG_DATA        0x00
#define ATA_REG_ERROR       0x01  // read
#define ATA_REG_FEATURE     0x01  // write
#define ATA_REG_SECCOUNT    0x02
#define ATA_REG_LBA_LO     0x03
#define ATA_REG_LBA_MID    0x04
#define ATA_REG_LBA_HI     0x05
#define ATA_REG_DEV_HEAD   0x06
#define ATA_REG_STATUS     0x07  // read
#define ATA_REG_COMMAND    0x07  // write

// ATA commands
#define ATA_CMD_IDENTIFY    0xEC
#define ATA_CMD_READ_SEC    0x20
#define ATA_CMD_WRITE_SEC   0x30
#define ATA_CMD_SMART       0xB0
#define ATA_CMD_STANDBY_IMM 0xE0

// SMART sub-commands (FEATURES register)
#define SMART_READ_DATA     0xD0
#define SMART_READ_THRESH   0xD1
#define SMART_ENABLE_OPS    0xD8
// SMART signature (LBA_MID=0x4F, LBA_HI=0xC2)
#define SMART_LBA_MID       0x4F
#define SMART_LBA_HI        0xC2

// ATA status bits
#define ATA_STATUS_BSY      0x80
#define ATA_STATUS_DRDY     0x40
#define ATA_STATUS_DRQ      0x08
#define ATA_STATUS_ERR      0x01

// Function prototypes

// Low-level SDIO bus
void sdio_init_pins(void);
void sdio_set_clock_slow(void);   // ~250 KHz for init
void sdio_set_clock_fast(void);   // ~500 KHz for commands
void sdio_set_clock_turbo(void);  // Max speed (no delay) for data

// Send 48-bit command, receive 48-bit response
// Returns true on success
bool sdio_send_cmd(uint8_t cmd_index, uint32_t arg, uint32_t *resp);

// CMD52: single register read/write
bool sdio_cmd52_read(uint8_t fn, uint32_t addr, uint8_t *data);
bool sdio_cmd52_write(uint8_t fn, uint32_t addr, uint8_t data, uint8_t *resp_data);

// CMD53: multi-byte read/write
bool sdio_cmd53_read(uint8_t fn, uint32_t addr, uint8_t *buf, uint16_t len, bool block_mode, bool inc_addr);
bool sdio_cmd53_write(uint8_t fn, uint32_t addr, const uint8_t *buf, uint16_t len, bool block_mode, bool inc_addr);

// SDIO card init sequence (from trace analysis)
bool sdio_card_init(void);

// HDD power control
void hdd_power_on(void);
void hdd_power_off(void);

#endif // SDIO_HW_H
