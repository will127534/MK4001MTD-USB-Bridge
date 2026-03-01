#ifndef ATA_SDIO_H
#define ATA_SDIO_H

#include <stdbool.h>
#include <stdint.h>

// Wait for ATA interrupt (fn1 INT_PENDING)
bool ata_wait_interrupt(uint32_t timeout_ms);

// Wait for ATA not busy
bool ata_wait_not_busy(uint32_t timeout_ms);

// ATA IDENTIFY DEVICE — fills 512-byte buffer
bool ata_identify(uint8_t *buf);

// ATA READ SECTORS — read 'count' sectors starting at LBA
bool ata_read_sectors(uint32_t lba, uint8_t count, uint8_t *buf);

// ATA WRITE SECTORS
bool ata_write_sectors(uint32_t lba, uint8_t count, const uint8_t *buf);

// Get drive capacity in sectors (from IDENTIFY data)
uint32_t ata_get_capacity(const uint8_t *identify_buf);

// Print drive info from IDENTIFY data
void ata_print_identify(const uint8_t *identify_buf);

// SMART: read 512-byte data page (FEATURES=0xD0)
bool ata_smart_read_data(uint8_t *buf);

// SMART: read 512-byte threshold page (FEATURES=0xD1)
bool ata_smart_read_thresholds(uint8_t *buf);

// SMART: enable operations (FEATURES=0xD8)
bool ata_smart_enable(void);

// Dump SMART attributes to UART
void ata_smart_dump(void);

// Power management
bool ata_standby_immediate(void);

#endif
