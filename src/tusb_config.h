#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#define CFG_TUSB_MCU OPT_MCU_RP2040
#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE

// Device configuration
#define CFG_TUD_ENDPOINT0_SIZE 64

// Class drivers
#define CFG_TUD_MSC 1
#define CFG_TUD_CDC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0

// MSC endpoint buffer size: 64 sectors × 512 bytes = 32 KB
#define CFG_TUD_MSC_EP_BUFSIZE 32768

#endif
