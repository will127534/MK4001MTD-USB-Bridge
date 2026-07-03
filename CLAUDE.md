# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

RP2040 (Pico) firmware bridging a Toshiba MK4001MTD 0.85" microdrive to USB Mass Storage. The drive is an SDIO card that tunnels ATA commands through CMD52/CMD53 (Nokia N91-era; not MMC, not CE-ATA, no existing driver) — this firmware implements the full stack from scratch. Development happens on this Raspberry Pi, which hosts the ARM toolchain, the SWD debug probe, and the bridge's USB port, so build/flash/test all run locally against real hardware. `hardware/` contains the KiCad PCB design for the reader board.

## Commands

```bash
# Build (SDK from $PICO_SDK_PATH — exported on this bench; without it, CMake
# fetches the pinned pico-sdk release from GitHub. Pin: MK4001_PICO_SDK_PIN
# in CMakeLists.txt, enforced at configure time.)
cd build && cmake .. && make -j4

# Flash via SWD debug probe (CMSIS-DAP, uses sudo openocd)
./testscript/flash.sh [elf]

# Flash + reset + capture UART boot log — the main dev loop
./testscript/flash_and_capture.sh [elf] [capture-seconds] [uart-device]
# (a stale older copy of flash_and_capture.sh sits at the repo root; prefer testscript/)

# Integration test suite (9 tests against the enumerated USB block device)
sudo ./testscript/test.sh [/dev/sdX]

# Filesystem-level write/remount/readback test with UART capture
./testscript/test_rw_uart.sh [size_mb]
```

- There is no lint step and no unit tests — all validation is hardware-in-the-loop. There is no single-test runner; tests are numbered sections inside `testscript/test.sh`.
- UART debug is 115200 baud, auto-detected from `/dev/serial/by-id/*Debug_Probe*if01`, `/dev/ttyACM0`, `/dev/ttyAMA0`. Every firmware subsystem logs with a `[MAIN]`/`[MSC]`/`[ATA]`/`[PWR]`/`[USB]` prefix — the UART log is the primary debugging signal.
- Ignore benign openocd errors: `"Invalid command argument"` and `"args[i] option value ('CX') is not valid"`.
- Power-cycle Pico + drive from the host: `sudo uhubctl -l 1 -p 2 -a off && sleep 4 && sudo uhubctl -l 1 -p 2 -a on`
- The `build/` directory is tracked in git, so every build dirties the tree; commits routinely include regenerated artifacts.

## Git

- `origin` is https://github.com/will127534/MK4001MTD-USB-Bridge — the published repo with a squashed history. The local branch `local-dev-history` preserves the fine-grained pre-publication dev history (per-version bug-fix commits, `v0.XX:` prefixed); it shares no ancestor with `origin/master`, so never merge the two.
- Where the README and the code disagree (e.g. the Status table's speed/idle numbers describe an older version), the code is the source of truth — the current firmware is v0.11 (power gating, warm boot, strict bad-sector reporting).

## Patched TinyUSB MSC driver (vendored in-repo)

The firmware compiles `lib/tinyusb_patched/msc_device.c` **instead of** the SDK TinyUSB's MSC class driver — `CMakeLists.txt` filters the stock file out of the `tinyusb_device_base` interface sources. Two deltas vs upstream (full diff in `lib/tinyusb_patched/msc_device.c.patch`): app-set sense data is preserved on READ10/WRITE10 errors (required for `MEDIUM ERROR` bad-sector reporting), and MODE SENSE(6) reports a Caching mode page with WCE=1 (hosts then flush via SYNCHRONIZE CACHE). **This repo needs zero SDK modifications and is pinned to pico-sdk 2.2.0** (`MK4001_PICO_SDK_PIN` in CMakeLists.txt, enforced at configure) because the vendored file must match the SDK's bundled TinyUSB (0.18.0). Never patch SDK files this firmware compiles; to move to a newer SDK, re-apply `msc_device.c.patch` to the new upstream, refresh the vendored copy, and bump the pin. The bench SDK's TinyUSB carries the user's own `hub.c` host-mode patch for *other* projects (also archived as `lib/tinyusb_patched/hub-host-fixes.patch`) — it is never compiled by this firmware; leave it alone.

## Architecture

Four layers, one file each:

```
USB Host ←→ msc_device.c (TinyUSB MSC) ←→ ata_sdio.c (ATA) ←→ sdio_pio.c + sdio.pio (PIO SDIO) ←→ sdio_hw.c (pins/power)
```

- **`msc_device.c`** — SCSI READ(10)/WRITE(10) → chunked ATA ops (64 sectors/chunk, 32 KB endpoint buffer). Drive I/O overlaps USB in both directions via main-loop hooks (`msc_service_write_behind`/`msc_service_prefetch`) while the RP2040 USB controller streams the previous chunk from IRQ context. Write caching is *advertised* (WCE=1 via the vendored TinyUSB MODE SENSE patch — hosts show "Write cache: enabled" and flush at fsync/unmount/suspend), which is what makes write-behind standards-legal; a failed background flush surfaces as MEDIUM ERROR on the next WRITE/SYNCHRONIZE CACHE. Bad-sector cache: repeat *reads* of known-bad LBAs fail fast (anti-hammer), but *writes* always reach the medium per SBC — via the strict synchronous path when overlapping the cache — and a successful write clears the LBA (write-repair). Sense mapping: read `UNC` → MEDIUM ERROR 03/11/00, write failure → 03/0C/00. `TEST_UNIT_READY` always answers "ready" without waking the drive, so kernel polling can't defeat power gating.
- **`ata_sdio.c`** — ATA commands issued as CMD52 writes to registers mapped into SDIO function-1 address space; sector data moves via CMD53. The full register map and all ATA/CCCR `#define`s live in `sdio_hw.h`. The read path samples final `STATUS/ERROR` after the data phase so slow-failing sectors surface real `UNC` instead of a generic DRQ timeout. `ata_error_recovery()` (IO_ABORT + fn1 reset, ~500ms) must run after any failed command before retrying.
- **`sdio_pio.c` / `sdio.pio`** — three PIO programs (CMD tx/rx, DAT read, DAT write) share PIO0 SM0, swapped at runtime by overwriting instruction memory at offset 0. The SM is fully reset (disable → clear FIFOs → restart → jump to 0) before every command; stale RX FIFO data causes phantom responses.
- **`main.c`** — boot sequence: 5s pre-delay with LED wave → `tusb_init()` → power-cycle drive → SDIO init → IDENTIFY → warm-up reads. Main loop: `tud_task()` + write-behind flush + read prefetch + temperature log (vendor 0xC2 FEAT=0x21, every 30s while active) + idle-standby check. All boot waits go through `sleep_with_usb()` so TinyUSB stays serviced — bare `sleep_ms()` during waits causes USB enumeration failures.

**Power gating:** 5s idle (`IDLE_STANDBY_MS` in main.c) or USB suspend → ATA STANDBY IMMEDIATE + cut drive power via the HDD_EN pin. Wake (~500ms) warm-probes the cached RCA, re-runs SDIO init if needed, then a 30ms DRDY poll.

**Two PIO clock domains:** card init (CMD5 → ~10ms settle → CMD3 → CMD7) must run at the slow clock (~500 kHz, clkdiv 62.5), but CMD52 CCCR config *silently fails* at the slow clock — `sdio_pio_card_init()` switches to fast (~10 MHz, clkdiv 3.125) before the CCCR writes. Both timing quirks were hard-won v0.11 fixes; don't reorder that sequence.

## Pins and Bench Constraints

- All pin assignments live in `sdio_hw.h` and `src/led.h`: SDIO on GP2–GP7, HDD power enable on GP9, UART console on GP12/GP13, LEDs on GP16–GP19. **GP0/GP1 are intentionally left free**: they were originally claimed by the stdio UART console in the build config, and a past Claude misdiagnosed SDIO failures on them as "dead pins". The SDIO GPIOs were deliberately moved off the UART pins — don't move anything back onto GP0/GP1 without checking stdio config, and don't reshuffle pins in general: the `hardware/` PCB is routed for the current assignment.
- The drive's long-standing bad sector at **LBA 1952** (UART signature `READ: ST=0x51 ERR ERR=0x40 UNC LBA=1952`) was **repaired on 2026-07-02** when v0.12 first allowed a write to reach it — it now reads clean, so there is currently no bad-sector fixture on this unit and the strict-error paths can't be exercised end-to-end (test.sh TEST 4 reports SKIP). `test.sh` expects exactly 7,862,400 total sectors.
- Cold spin-up can take ~9–12s; `init_drive()` retries with a longer power cycle before giving up.

## PIO Gotchas (learned the hard way)

- After `pull block` + `mov x, osr`, the OSR still holds the pulled value — use `out null, 32` to flush it and trigger autopull for real data.
- After read DMA completes, the SM is still clocking CRC + end nibbles; poll the SM program counter until it stalls at `pull block` before swapping programs.

## References

- `README.md` — full protocol documentation (SDIO-ATA register map, read/write paths, version history) with the author's inline "Human notes".
- `docs/N91_TRACE_ANALYSIS.md` — protocol ground truth decoded from Nokia N91 logic-analyzer captures. Raw traces are in `docs/RawLogicTraces.7z` (extracted working copies with the decoder also live at `/home/pi/logictrace/`), and the N91 service manual is `docs/nokia_n91_rm43_schematics.pdf`.
- Project blog post: https://www.willwhang.dev/Reading-MK4001MTD/
