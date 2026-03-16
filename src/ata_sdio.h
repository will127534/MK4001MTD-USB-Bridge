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

// Dump drive diagnostics to UART (vendor 0xC2 commands)
void ata_smart_dump(void);

// Power management
bool ata_standby_immediate(void);

// Error recovery — clear drive error state after a failed command.
// Call after DRQ timeout / read/write failure before retrying a different LBA.
void ata_error_recovery(void);

// Single-sector read/write with reduced retries and shorter timeouts.
// Used by the bad-sector fallback path — faster failure for known-bad areas.
bool ata_read_sector_fast(uint32_t lba, uint8_t *buf);
bool ata_write_sector_fast(uint32_t lba, const uint8_t *buf);

#endif
