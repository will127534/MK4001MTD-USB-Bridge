#ifndef SDIO_HW_H
#define SDIO_HW_H

#include <stdint.h>
#include <stdbool.h>

// ---- Pin assignments (GP0/GP1 dead on this Pico) ----
#define SDIO_CLK_PIN  2
#define SDIO_CMD_PIN  3
#define SDIO_DAT0_PIN 4   // DAT0-DAT3 are GP4-GP7
#define SDIO_DAT1_PIN 5
#define SDIO_DAT2_PIN 6
#define SDIO_DAT3_PIN 7
#define HDD_EN_PIN    9   // HDD power enable: HIGH=on, LOW=off

// ---- CCCR registers (SDIO common) ----
#define CCCR_IO_ENABLE      0x02
#define CCCR_IO_READY       0x03
#define CCCR_INT_ENABLE     0x04
#define CCCR_INT_PENDING    0x05
#define CCCR_IO_ABORT       0x06
#define CCCR_BUS_IF_CTRL    0x07

// FBR (Function Basic Registers) for fn1
#define FBR1_BLKSZ_LO      0x110
#define FBR1_BLKSZ_HI      0x111

// ---- ATA registers (mapped to SDIO fn1 addresses) ----
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

// ---- ATA commands ----
#define ATA_CMD_IDENTIFY    0xEC
#define ATA_CMD_READ_SEC    0x20
#define ATA_CMD_WRITE_SEC   0x30
#define ATA_CMD_SMART       0xB0
#define ATA_CMD_STANDBY_IMM 0xE0

// SMART sub-commands
#define SMART_READ_DATA     0xD0
#define SMART_READ_THRESH   0xD1
#define SMART_ENABLE_OPS    0xD8
#define SMART_LBA_MID       0x4F
#define SMART_LBA_HI        0xC2

// ---- ATA status bits ----
#define ATA_STATUS_BSY      0x80
#define ATA_STATUS_DRDY     0x40
#define ATA_STATUS_DF       0x20
#define ATA_STATUS_DRQ      0x08
#define ATA_STATUS_ERR      0x01

// ---- ATA error bits ----
#define ATA_ERROR_ABRT      0x04
#define ATA_ERROR_IDNF      0x10
#define ATA_ERROR_UNC       0x40

// ---- Functions ----
void sdio_init_pins(void);
void hdd_power_on(void);
void hdd_power_off(void);

#endif
