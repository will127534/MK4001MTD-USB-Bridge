#ifndef MSC_DEVICE_H
#define MSC_DEVICE_H

#include <stdbool.h>
#include <stdint.h>

// Drive geometry and activity state shared with the main loop.
void msc_set_drive_params(uint32_t sectors);
void msc_touch_activity(const char *reason);
uint32_t msc_get_last_activity_ms(void);
bool msc_is_drive_spinning(void);
bool msc_is_power_gated(void);
void msc_power_gate(void);

// Main-loop hook: prefetch the next sequential read chunk while the previous
// one streams to the host over USB.
void msc_service_prefetch(void);

#endif
