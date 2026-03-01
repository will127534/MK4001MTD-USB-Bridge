# MK4001MTD USB Bridge

**RP2040 Pico firmware that bridges a Toshiba MK4001MTD 1.8" SDIO microdrive as a USB Mass Storage device.**
![_DSC1170](https://github.com/user-attachments/assets/8fb591be-40f6-4506-b9c9-b852034268aa)

The MK4001MTD is a 4 GB Microdrive originally used in the Nokia N91 music phone and some other devices, like MP3 players or USB drives, back when flash storage was still quite expensive.

You might have seen introductions claiming this drive uses the MMC protocol, but that’s actually incorrect. I’ve been investigating this for a while: I tried building an 8-bit MMCplus card reader and tested different SD/MMC readers without success. As a last resort, I bought a Nokia N91 to capture logic traces and confirm what protocol it actually uses.

Here is the photo when I was trying to use it with my 8bit-MMCPlus reader board, and it turns out it is not MMC :(
![_DSC0484](https://github.com/user-attachments/assets/759a07a8-eedc-4ffb-98e2-f37aaae8fe88)

So I end up getting N91 to collect traces:
![_DSC1093](https://github.com/user-attachments/assets/80ebd34a-8f5d-44cf-ab0b-073e908e863a)
![_DSC1131](https://github.com/user-attachments/assets/ff7113dc-a388-434d-98ef-c79e2a53c799)


Unlike standard ATA/CF Microdrives, it uses an SDIO interface with ATA commands tunneled through CMD52/CMD53. No existing driver supports this protocol, so this firmware implements the full stack from scratch.

This surprised me, because there is an SDIO-to-ATA standard called CE-ATA. But if you look closely at the release timeline, CE-ATA came later than this drive. As a result, this drive relies entirely on SDIO commands, and CE-ATA is not available.

The second hardware point to mention is that another piece of misinformation floating around—claiming it’s an 8-bit MMCPlus card—is not only untrue, but the pinout doesn’t follow the MMC standard either. You can find the Nokia N91 service manual with some documentation on the pinout: while the pin numbering follows the MMCPlus standard, the pin mapping does not. This is an important detail if you’re wiring it yourself: it uses the same MMC connector, but the pin mapping is different, more in the Hardware section.

Finally, note that this is co-developed with Claude/OpenClaw. I collected the logic traces manually and set up a closed-loop test station for OpenClaw to iterate on development—analyzing the traces and implementing features. The documentation will mostly be written by Claude; I’ll add my notes inline as well. I’ve also read and double-checked the documentation myself, and it should be reliable and easy to follow.

For insights into the analytics on N91 trace, it is under /docs/N91_TRACE_ANALYSIS.md, I've also put the N91 service manual there along with the raw logic traces.

## Status

Fully functional USB mass storage with PIO-accelerated reads/writes and idle power management.

| Metric | Value |
|--------|-------|
| Read speed | 550 kB/s |
| Write speed | 364 kB/s |
| Capacity | 3.75 GB (7,862,400 sectors) |
| Filesystem | FAT32 verified (mount/unmount/fsck clean) |
| Data integrity | Write+readback MD5 verified |
| Idle standby | 1 second timeout → STANDBY IMMEDIATE |

## How It Works

### Architecture

```
USB Host ←→ USB MSC (TinyUSB) ←→ ATA Layer ←→ SDIO Layer (PIO) ←→ MK4001MTD
```

The firmware has four layers:

1. **USB MSC** (`msc_device.c`) — TinyUSB Mass Storage Class. Translates SCSI READ(10)/WRITE(10) into ATA sector operations. EP buffer is 8192 bytes, batching up to 16 sectors per USB transfer. Activity LEDs on GP9 (TX/write) and GP10 (RX/read). Tracks last I/O timestamp for idle power management.

2. **ATA-over-SDIO** (`ata_sdio.c`) — Implements ATA commands (IDENTIFY, READ SECTORS, WRITE SECTORS) by writing to ATA registers mapped into SDIO function 1 address space via CMD52, and transferring sector data via CMD53. 3-tier retry logic at CMD, data, and ATA levels.

3. **PIO SDIO** (`sdio_pio.c`, `sdio.pio`) — Hardware-accelerated SDIO using RP2040's PIO peripheral. Three PIO programs share a single state machine via dynamic program swapping:
   - **CMD tx/rx** (26 instructions) — sends SDIO commands and receives responses
   - **DAT read** (13 instructions) — reads data blocks from 4-bit DAT bus via DMA
   - **DAT write** (16 instructions) — writes data blocks to 4-bit DAT bus via DMA, with built-in CRC status reception and busy-wait

4. **Bit-bang SDIO** (`sdio_hw.c`) — Software SDIO used only for card initialization at 400 KHz (before PIO is set up). All runtime I/O uses PIO.

Human notes: Interestingly, Claude was really reluctant to implement SDIO in PIO, and a lot of development cycles were wasted bouncing back and forth between PIO and bit-banging.

### The SDIO-ATA Protocol

The MK4001MTD presents itself as an SDIO card with one I/O function. Standard SDIO card initialization (CMD5/CMD3/CMD7) sets up the bus, then ATA registers are accessed through SDIO commands:

**Register access (CMD52):** Each ATA register is mapped to a function 1 address:

| Address | Register | Usage |
|---------|----------|-------|
| 0x00 | DATA | CMD53 target for sector data |
| 0x01 | ERR/FEAT | Error (read) / Feature (write) |
| 0x02 | SECCOUNT | Sector count |
| 0x03 | LBA_LO | LBA bits 0-7 |
| 0x04 | LBA_MID | LBA bits 8-15 |
| 0x05 | LBA_HI | LBA bits 16-23 |
| 0x06 | DEV/HEAD | Device/Head + LBA bits 24-27 |
| 0x07 | CMD/STATUS | Command (write) / Status (read) |

**Data transfer (CMD53):** Sector data is transferred by issuing CMD53 in block mode targeting the DATA register (address 0x00). For multi-sector reads, a single CMD53 with `block_count=N` transfers N × 512 bytes in one SDIO multi-block transaction.

**Interrupt signaling:** The drive signals sector readiness by asserting an SDIO interrupt (INT_PENDING bit 1 in CCCR register 0x05). Reading the ATA STATUS register clears the interrupt.

### Read Path (Multi-Block PIO)

For a 16-sector read:

```
1. Write ATA registers via PIO CMD52:
     SECCOUNT=16, LBA_LO/MID/HI, DEV/HEAD=0xE0, CMD=0x20

2. Poll STATUS via CMD52 until DRQ (bit 3) is set

3. Swap PIO to DAT read program
4. Send CMD53: block_mode=1, fn=1, addr=0x0000, block_count=16

5. PIO DAT read: for each of the 16 blocks:
   a. Wait for start bit (all DAT lines low)
   b. DMA 1024 nibbles (512 bytes) from PIO RX FIFO to buffer
   c. Wait for SM to finish clocking CRC+end nibbles (poll SM PC)
   d. Repack nibbles → bytes in-place

6. Swap PIO back to CMD program
```

### Write Path (Multi-Block PIO)

For a 16-sector write:

```
1. Write ATA registers via PIO CMD52:
     SECCOUNT=16, LBA, DEV/HEAD=0xE0, CMD=0x30

2. Poll STATUS via CMD52 until DRQ (bit 3) is set
   (STATUS 0xD8 = BSY+DRQ treated as DRQ-ready, per N91 trace)

3. Swap PIO to DAT write program
4. Send CMD53: block_mode=1, fn=1, addr=0x0000, block_count=16

5. PIO DAT write: for each of the 16 blocks:
   a. Precompute CRC16-CCITT per DAT line (4 independent CRCs)
   b. Build nibble stream: start(0x0) + data(1024 nibbles) + CRC(16) + end(0xF)
   c. DMA nibble stream to PIO TX FIFO
   d. PIO clocks out all nibbles, then:
      - Switches DAT to input
      - Clocks 16 cycles for CRC status from card
      - Polls DAT0 until card releases busy
      - Fires IRQ 0 to signal block completion

6. Swap PIO back to CMD program
```

### PIO Program Swapping

The RP2040 PIO has 32 instruction slots per block. Our three programs total 55 instructions, so they can't coexist. Instead, a single SM0 on PIO0 is used, and programs are swapped by writing directly to PIO instruction memory:

```c
static void load_program_raw(const pio_program_t *program) {
    for (uint i = 0; i < program->length; i++)
        pio->instr_mem[FIXED_OFFSET + i] = program->instructions[i];
}
```

This bypasses the SDK's `pio_add_program`/`pio_remove_program` allocator. Program swap takes ~1 µs. Each swap is followed by a program-specific reinit that sets pin mappings, shift direction, and clock divider.

### Power Management

Analysis of Nokia N91 logic traces reveals aggressive power management:

- **Idle mode:** STANDBY IMMEDIATE (0xE0) every **~7.5 seconds**, even with no I/O. Each standby triggers full SDIO re-initialization (CMD5 retry → CMD3 → CMD7 → CCCR setup). 28 standby cycles observed in one idle session.
- **Active mode:** Standby issued between bursts of I/O (24 cycles during USB drive file operations).
- No other power commands (IDLE, SLEEP, CHECK POWER MODE) or CCCR power register accesses observed.

The firmware replicates this behavior with a configurable idle timeout:

```c
#define IDLE_STANDBY_MS 1000  // in main.c
```

After 1 second with no USB read/write activity, the main loop sends STANDBY IMMEDIATE to spin down the drive platters. The drive automatically wakes on the next ATA command (READ/WRITE SECTORS), adding ~400ms of spin-up latency on the first access after standby.

The idle tracker is armed only after the first MSC read/write callback, so boot-time initialization and USB enumeration don't trigger premature standby.

## Building

### Prerequisites

- Raspberry Pi Pico SDK
- ARM toolchain (`arm-none-eabi-gcc`)
- CMake

### Build & Flash

```bash
cd /home/pi/mk4001_bridge/build
cmake ..
make -j4

sudo openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg \
  -c "adapter speed 1000" -c "init" -c "reset halt" -c "sleep 200" \
  -c "program /home/pi/mk4001_bridge/build/mk4001_bridge.elf verify" \
  -c "reset run" -c "exit"
```

### Hardware Wiring

| Pico GPIO | Function | Notes |
|-----------|----------|-------|
| GP2 | SDIO_CLK | Host clock output |
| GP3 | SDIO_CMD | Bidirectional command line |
| GP4 | SDIO_DAT0 | Data bit 0 |
| GP5 | SDIO_DAT1 | Data bit 1 |
| GP6 | SDIO_DAT2 | Data bit 2 |
| GP7 | SDIO_DAT3 | Data bit 3 |
| GP9 | LED_TX | Write activity LED (active high) |
| GP10 | LED_RX | Read activity LED (active high) |
| GP13 | HDD_PWR | Power switch (HIGH=off, LOW=on via transistor) |
| GP16 | UART TX | Debug output @ 115200 baud |
| GP17 | UART RX | Debug input |

**Note:** GP0 and GP1 are dead on this specific Pico unit. All SDIO pin assignments are shifted +2.

Human notes: Claude was wrong here because it didn’t realize that GP0 and GP1 were being used for the UART terminal in its build configuration. It kept forgetting this, to the point where I just moved the SDIO GPIOs off that UART.

HDD_PWR isn’t necessary. You don’t have to power-cycle the drive to use it; it’s more of a development convenience for resetting the HDD when a lot of things are hard-coded. That said, if you want power savings, you can use that signal, but it can handle warm reset without any issue.

You’ll see debug messages over UART. They’re not going through USB-CDC because it was easier for Claude to set up a separate UART-to-USB logging link that doesn’t disconnect or become unstable during early development.

Finally, here’s the wiring to the actual drive.  
![_DSC1176-2](https://github.com/user-attachments/assets/55c19b6a-03d5-4ee6-8c7b-da028ee14541)
Here is a crop from the N91 schematic, you can map the pin number also.
<img width="1188" height="1016" alt="image" src="https://github.com/user-attachments/assets/9d121aad-2bd2-4a44-9ba2-5efc2730b345" />

Side note that this is a 3V drive but I think 3.3V is fine, mostly it is to save some level shifting work.  

I have a USB reader board working in progress, will update to the hardware folder once it is done.
<img width="4796" height="2608" alt="image" src="https://github.com/user-attachments/assets/aec20183-9199-465c-9a98-3877b47dd1f7" />


## Source Files

| File | Purpose |
|------|---------|
| `main.c` | Initialization, power control, PIO verification, USB main loop |
| `sdio_hw.c/.h` | Bit-bang SDIO: CMD5/3/7/52/53, CRC7/16, card init |
| `sdio_pio.c/.h` | PIO SDIO: CMD52, CMD53 read/write, program swap, DMA |
| `sdio.pio` | PIO assembly: CMD tx/rx, DAT read, DAT write programs |
| `ata_sdio.c/.h` | ATA command layer: IDENTIFY, READ/WRITE SECTORS, SMART |
| `msc_device.c` | USB MSC callbacks: read10, write10, inquiry, capacity |
| `led.h` | Activity LED control (GP9 TX, GP10 RX) |
| `usb_descriptors.c` | USB device/config/string descriptors |
| `tusb_config.h` | TinyUSB configuration (MSC endpoint buffer = 8192) |

## Version History

| Version | Read | Write | Key Change |
|---------|------|-------|------------|
| v0.1–v0.3 | 105 kB/s | 93 kB/s | Bit-bang SDIO, CRC16, retry logic |
| v0.4 | 229 kB/s | 92 kB/s | PIO reads (two-SM approach) |
| v0.5 | 374 kB/s | 58 kB/s | Single-SM program swap, direct instr memory write |
| v0.6 | 583 kB/s | 93 kB/s | Multi-block CMD53 reads, CRC clock drain fix |
| v0.7 | 583 kB/s | 93 kB/s | Activity LEDs (GP9/GP10) |
| v0.8 | 588 kB/s | 274 kB/s | PIO writes, OSR flush fix (`out null, 32`) |
| **v0.9** | **550 kB/s** | **364 kB/s** | **Multi-block CMD53 writes (+33%), idle standby** |

## Testing

```bash
# Check device appeared
lsblk -dno NAME,MODEL | grep MK4001

# Filesystem test — mount, copy files, verify
sudo mount /dev/sdX1 /mnt/mk4001
cp /tmp/testfile /mnt/mk4001/
sync
md5sum /tmp/testfile /mnt/mk4001/testfile    # should match
sudo umount /mnt/mk4001

# Speed benchmarks (raw device, do NOT mount first — will corrupt filesystem)
# Use a safe offset past the filesystem or an unpartitioned drive
sudo dd if=/dev/sdX of=/dev/null bs=64k count=128 iflag=direct     # read
sudo dd if=/dev/zero of=/dev/sdX bs=64k count=64 oflag=direct seek=1024  # write (offset past FS)
```

## License

I don't care.
