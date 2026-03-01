# Nokia N91 ↔ MK4001MTD SDIO Trace Analysis

Complete protocol analysis from 5 logic analyzer captures of a Nokia N91 communicating with a Toshiba MK4001MTD microdrive over SDIO.

**Decoder:** `decode_sdio.py` — CRC FAIL flags are decoder artifacts (high-speed 4-bit mode sampling), not real protocol failures.
**Sample rate:** ~500 MS/s, SDIO CLK ≈ 3.76 MHz (133 samples/period)

**Trace files** (in `/home/pi/logictrace/`):

| Trace | File | Size | Transactions | Description |
|-------|------|------|-------------|-------------|
| Boot | `N91Boot_success_Fast6ch.csv` | 263 MB | 11,105 | Boot sequence with 6 init cycles |
| Idle | `N91_USBDrive_Idle_Fast6ch.csv` | 20 MB | 2,482 | USB drive mode, no host I/O |
| USBDrive | `N91_USBDrive_Fast6ch.csv` | 1.3 GB | 58,192 | File operations under Windows |
| FastFormat | `N91_FastFormat_INFO_success_Fast6ch.csv` | 730 MB | 24,712 | Quick format + info check |
| SlowFormat | `N91_SlowFormat_success_Fast6ch.csv` | 7.1 GB | 186,263 | Full zero-fill format (partial capture) |

CSV format: `Sample, DAT0, CLK, CMD, CD/DAT3, DAT2, DAT1` (state-change / transition-based)

Tools:
- `decode_sdio.py` — Python SDIO protocol decoder (CMD52/CMD53, ATA register tracking)
- `analyze_all.py` — batch analysis and statistics across all traces
- `decoded_transactions.log` — human-readable decoded output
- `analysis.md` — full machine-parsed analysis

---

## 1. SDIO Card Initialization

Every SDIO session follows the same sequence. The N91 performs this after every power-on AND after every STANDBY IMMEDIATE:

```
CMD5 arg=0x00000000                     → probe (get OCR)
CMD5 arg=0x001C0000 (repeated ~7×)      → set voltage, poll Ready bit
CMD3 arg=0x00000000                     → get RCA
CMD7 arg=0x00010000                     → select card (RCA=0x0001)
CMD52 writes (CCCR configuration)       → bus setup
```

### CMD5 (IO_SEND_OP_COND)

First CMD5 with arg=0x00000000 probes the card. Response: `0x101F8000`
- Ready=0, NumFn=1, Mem=0, OCR=0x1F8000

Subsequent CMD5s with arg=0x001C0000 set the voltage window. The card responds Ready=0 until ready, then Ready=1 (`0x901C0000`). Takes ~7 retries spaced ~2M samples apart (~4ms each).

### CMD3 (SEND_RELATIVE_ADDR)

Response: `0x00011E00` → RCA = 0x0001. Always the same across all cycles and traces.

### CMD7 (SELECT_CARD)

Arg: 0x00010000 (RCA=0x0001). Response: `0x00001E00`.

### CCCR Configuration (CMD52, function 0)

After CMD7, always the same register sequence:

| Step | Register | Address | Value | Meaning |
|------|----------|---------|-------|---------|
| 1 | BUS_INTERFACE_CONTROL | 0x0007 | 0x02 | 4-bit bus width |
| 2 | FN1_FBR_BLOCK_SIZE_0 | 0x0110 | 0x00 | Block size low byte |
| 3 | FN1_FBR_BLOCK_SIZE_1 | 0x0111 | 0x02 | Block size high (0x0200 = 512) |
| 4 | INT_ENABLE | 0x0004 | 0x03 | Master + fn1 interrupt enable |
| 5 | IO_ENABLE | 0x0002 | 0x02 | Enable function 1 |
| 6 | IO_READY (read) | 0x0003 | → 0x02 | Confirm fn1 ready (immediate) |

No high-speed mode (CCCR 0x13) observed — N91 runs at default speed (~3.76 MHz).

### Re-initialization Frequency

The N91 performs **full SDIO re-init** (CMD5→CMD3→CMD7→CCCR) after every STANDBY IMMEDIATE:

| Trace | Re-init Cycles | STANDBY_IMMEDIATE Count |
|-------|---------------|------------------------|
| Boot | 6 | 6 |
| Idle | 28 | 28 |
| USBDrive | 25 | 24 |
| FastFormat | 8 | 9 |
| SlowFormat (partial) | 1 | 0 |

This suggests the drive's SDIO controller loses state across standby.

---

## 2. ATA-over-SDIO Register Map (Function 1)

ATA registers are accessed via CMD52 to function 1 addresses:

| Fn1 Address | ATA Register | Direction |
|-------------|-------------|-----------|
| 0x0000 | DATA | CMD53 read/write (sector data) |
| 0x0001 | ERR / FEATURES | Read: error; Write: features |
| 0x0002 | SECTOR COUNT | Read/Write |
| 0x0003 | LBA LOW | Read/Write |
| 0x0004 | LBA MID | Read/Write |
| 0x0005 | LBA HIGH | Read/Write |
| 0x0006 | DEVICE / HEAD | Read/Write |
| 0x0007 | STATUS / COMMAND | Read: status; Write: command |

Interrupt signaling: CCCR INT_PENDING (fn0, address 0x0005), bit 1 = function 1 interrupt pending.

---

## 3. ATA Commands Used by N91

| Command | Code | Traces Present | Usage |
|---------|------|---------------|-------|
| READ SECTORS | 0x20 | All | Primary read — never uses READ MULTIPLE (0xC4) |
| WRITE SECTORS | 0x30 | USBDrive, FastFormat, SlowFormat | Primary write — never uses WRITE MULTIPLE (0xC5) |
| IDENTIFY DEVICE | 0xEC | Boot, FastFormat | Issued once per format, once at boot |
| STANDBY IMMEDIATE | 0xE0 | Boot, Idle, USBDrive, FastFormat | Power management between bursts |
| Vendor 0xC2 | 0xC2 | All | Toshiba multi-sector config |

**Key insight:** Multi-sector transfers are achieved entirely through CMD53 multi-block mode. The ATA SECCOUNT register determines how many sectors per command, and CMD53 block_count always matches SECCOUNT.

### IDENTIFY DEVICE (0xEC)

```
CMD52 WR fn1 SECCOUNT ← 0x00       (0 = 256 words = 512 bytes)
CMD52 WR fn1 COMMAND  ← 0xEC
Poll INT_PENDING until 0x02         (~85 polls, ~92ms in FastFormat)
CMD52 RD fn1 STATUS   → 0xD8       (BSY+DRQ — treated as DRQ-ready)
CMD53 RD block_mode=1, count=1     (512 bytes)
CMD52 RD fn1 STATUS   → 0x50       (DRDY+DSC — complete)
```

IDENTIFY first word: 0x0040 (non-removable, non-magnetic, non-ATAPI).
MBR OEM string: "EPOC" (Symbian OS).

### READ SECTORS (0x20)

Single-sector reads (boot, LBA 0 and 1):
```
SECCOUNT=0x01, LBA=0x000000, DEV/HEAD=0xE0, CMD=0x20
→ CMD53 RD block_mode=1, count=1
```

Multi-sector reads (all subsequent):
```
SECCOUNT=0x10, LBA=0x000020, DEV/HEAD=0xE0, CMD=0x20
→ CMD53 RD block_mode=1, count=16
```

Each followed by: INT_PENDING poll → CMD53 read → two STATUS reads (0xD8 then 0x50).

### WRITE SECTORS (0x30)

Only observed in USBDrive, FastFormat, and SlowFormat traces. Same pattern as reads but with CMD53 write direction.

### STANDBY IMMEDIATE (0xE0)

```
CMD52 RD fn1 STATUS → 0x50         (confirm ready)
CMD52 WR fn1 CMD    ← 0xE0         (standby)
CMD52 RD fn0 INT_PENDING → 0x00    (no interrupt expected)
→ Full SDIO re-init follows
```

---

## 4. Vendor Command 0xC2 — Toshiba Multi-Sector Config

Non-standard ATA command using FEATURES register as sub-command selector.

**FEAT=0x20 — Query max multi-sector capability:**
```
WR FEAT ← 0x20, WR CMD ← 0xC2
→ SECCOUNT=0xFF (255 max), LBA_MID=0xFF, LBA_LO=0x00, LBA_HI=0x00
```
Always returns 0xFF across all traces.

**FEAT=0x21 — Query current multi-sector count:**
```
WR FEAT ← 0x21, WR CMD ← 0xC2
→ SECCOUNT = varies (see below)
```

The current count is **not fixed** — the drive picks its own default after each re-init:

| Trace | FEAT=0x21 Values |
|-------|-----------------|
| Boot | 25 (0x19) |
| Idle | 35 (0x23) |
| USBDrive | 36, 36, 38 |
| FastFormat | 28, 30 |
| SlowFormat | 32 |

**Must query fresh after every init — do not cache.**

Functionally equivalent to ATA SET MULTIPLE MODE (0xC6), but Toshiba-proprietary. The bridge firmware can ignore this and set SECCOUNT directly.

---

## 5. CMD53 Data Transfer Patterns

All CMD53 transfers across all traces use block_mode=1 (never byte mode), block_size=512.

### Boot Trace

| Type | Count | block_count | Total bytes |
|------|-------|-------------|-------------|
| IDENTIFY data | 1 | 1 | 512 |
| Single-sector reads | 2 | 1 | 1,024 |
| Multi-sector reads | 200 | 16 | 1,638,400 |
| **Total** | **203** | | **1,639,936** |

No writes observed in boot trace.

### USBDrive Trace (983 CMD53 total)

| Block Count | Reads | Writes | Notes |
|-------------|-------|--------|-------|
| 1 | 6 | 21 | Single sector |
| 2 | 2 | 3 | |
| 3 | 4 | 1 | |
| 8 | 24 | **728** | **Dominant write size (4KB = Windows cluster)** |
| 13–15 | 2 | — | |
| 16 | 69 | 53 | Common for both |
| 24–48 | — | 4 | |
| 64 | 27 | — | |
| 80–96 | 2 | 2 | |
| 128 | 35 | — | Largest observed reads |

**Key insight:** Writes heavily favor 8-sector blocks (728/812 = 90%). Reads span a wide range up to 128 sectors.

### FastFormat Trace (460 CMD53 total)

- 389 reads: almost all 16-sector blocks (LBA 0–2768, metadata scan)
- 71 writes: 32/64-sector blocks (FAT/directory rewrite only)
- Does NOT touch data area — only clears allocation tables

### SlowFormat Trace (155 CMD53 in first 50M rows)

- 0 reads, 155 writes
- 153 writes use 128-sector blocks (64 KB each)
- Sequential zero-fill from LBA 0: `LBA=0, 128, 256, 384, ...`
- First write is LBA 1952 SEC=16 (FAT area), then LBA 0 SEC=1 (boot sector), then sequential
- Extrapolated: ~61K writes to zero entire 4 GB drive

### Multi-Block CMD53 Wire Format (4-bit bus)

Each block within a multi-block CMD53 transfer:

```
[start] [data ×1024 nibbles] [CRC16 ×16 clocks] [end]
   1          1024                  16              1    = 1042 clock cycles
```

- **Start bit**: all 4 DAT lines low (read: card drives; write: host drives)
- **Data**: 1024 clocks, 4 bits/clock = 512 bytes
- **CRC16**: 16 clocks, each DAT line carries independent CRC16-CCITT
- **End bit**: all DAT lines high
- **Write CRC status**: card drives 3-bit status on DAT0 (010=OK, 101=CRC error)
- **Write busy**: card holds DAT0 low until programming completes
- **Inter-block gap**: variable idle clocks

---

## 6. Power Management

### Idle Behavior (USB Drive Mode)

The N91 sends STANDBY IMMEDIATE (0xE0) every **~7.5 seconds** even with zero host I/O:

- 28 standby cycles in one idle session
- Each triggers full SDIO re-init (CMD5 retry → CMD3 → CMD7 → CCCR)
- Only 1 CMD53 in entire idle period (single 16-sector read at LBA 0)
- The drive never stays spun up more than ~7.5s without activity

### Active Behavior

- **USBDrive:** 24 STANDBY commands interspersed between I/O bursts
- **FastFormat:** 9 STANDBY commands
- **SlowFormat:** 0 STANDBY (heavy I/O keeps drive awake)

### Power Commands NOT Used

No IDLE (0xE1), IDLE IMMEDIATE (0xE3), SLEEP (0xE6), CHECK POWER MODE (0xE5), or CCCR Power Control register (0x12) accesses observed. The N91's approach: STANDBY IMMEDIATE + full SDIO re-init to wake.

---

## 7. Interrupt and Status Behavior

### INT_PENDING Polling

The N91 polls CCCR register 0x0005 via CMD52 at varying rates:

| Context | Poll Interval | Approx Time |
|---------|--------------|-------------|
| Active (after ATA cmd) | ~1,900 samples | ~4 µs |
| Background (between ops) | ~550K samples | ~1.1 ms |
| Idle (long waits) | ~10M samples | ~20 ms |

### STATUS Register Values

| Value | Bits | Meaning | Context |
|-------|------|---------|---------|
| 0x50 | DRDY+DSC | Ready, no data pending | After CMD53 completes |
| 0xD8 | BSY+DRDY+DSC+DRQ | Data ready to transfer | Before CMD53, after interrupt |

**STATUS 0xD8 (BSY+DRQ):** Per ATA spec, BSY=1 means other bits are invalid. However, this value is reported consistently and exclusively **before** CMD53 data transfers (never after). The drive's internal SDIO-ATA bridge appears to assert both BSY and DRQ simultaneously as its "data ready" indication. The N91 reads STATUS once before CMD53 (gets 0xD8), performs the transfer, then reads STATUS again after (gets 0x50).

**Verification:** INT_PENDING reads (same R5 response format, same bit positions) never show bit 7 corruption (only 0x00 and 0x02 observed across 178K reads), confirming 0xD8 is a real drive response, not a decoder artifact.

**Complete I/O sequence (every read/write follows this exactly):**
```
1. ATA command issued (CMD52 WR fn1 CMD/STATUS ← 0x20 or 0x30)
2. Poll INT_PENDING until 0x02
3. Read STATUS → 0xD8 (BSY+DRDY+DSC+DRQ)
4. CMD53 data transfer
5. Poll INT_PENDING until 0x02
6. Read STATUS → 0x50 (DRDY+DSC)
```

**Firmware implication:** When polling for DRQ, accept 0xD8 as "ready" — the BSY bit does not clear separately before DRQ on this drive.

---

## 8. SDIO Bus Configuration

Identical across all traces, after every re-init:
- 4-bit bus width (CCCR BUS_INTERFACE_CONTROL = 0x02)
- 512-byte block size for fn1 (FBR_BLOCK_SIZE = 0x0200)
- Interrupt enable for master + fn1
- IO enable for fn1
- No high-speed mode — default speed (~3.76 MHz)

---

## 9. Trace Statistics

### Boot Trace

| Item | Count |
|------|-------|
| Total SDIO commands | 11,105 |
| CMD5 | 54 |
| CMD3 / CMD7 | 6 / 6 |
| CMD52 | 10,836 |
| CMD53 | 203 |
| ATA IDENTIFY | 1 |
| ATA READ SECTORS | 202 |
| ATA STANDBY IMMEDIATE | 6 |
| Vendor 0xC2 | 4 |

### All Traces Summary

| Trace | Rows | CLK Edges | Transactions | CMD52 | CMD53 | Re-inits |
|-------|------|-----------|-------------|-------|-------|----------|
| Boot | 9.4M | 4.7M | 11,105 | 10,836 | 203 | 6 |
| Idle | 670K | 326K | 2,482 | 2,173 | 1 | 28 |
| USBDrive | 45.6M | 22.4M | 58,192 | 56,934 | 983 | 25 |
| FastFormat | 25.7M | 12.8M | 24,712 | 24,165 | 460 | 8 |
| SlowFormat | 256.9M | 127.8M | 186,263 | 184,672 | 790 | 1 |

### ATA Commands Per Trace

| Trace | READ | WRITE | IDENTIFY | STANDBY | 0xC2 |
|-------|------|-------|----------|---------|------|
| Boot | 202 | 0 | 1 | 6 | 4 |
| Idle | 1 | 0 | 0 | 28 | 2 |
| USBDrive | 171 | 812 | 0 | 24 | 6 |
| FastFormat | 388 | 71 | 1 | 9 | 4 |
| SlowFormat | 0 | 155+ | 0 | 0 | 2 |

---

## 10. Implications for MK4001 Bridge Firmware

1. **SECCOUNT must match CMD53 block_count** — confirmed in all traces
2. **Use READ/WRITE SECTORS (0x20/0x30)**, not READ/WRITE MULTIPLE — N91 never uses 0xC4/0xC5
3. **Vendor 0xC2 can be ignored** — bridge sets SECCOUNT directly
4. **Multi-block transfers up to 128 sectors** work reliably (SlowFormat does thousands)
5. **Full SDIO re-init after standby** is expected — drive SDIO controller loses state
6. **8-sector writes are the sweet spot** (4 KB = FAT32 cluster size on Windows)
7. **STATUS 0xD8 (BSY+DRQ) is the normal pre-transfer status** — drive never clears BSY before asserting DRQ; accept DRQ regardless of BSY
8. **CMD53 always block_mode=1** — no need for byte-mode CMD53 support
9. **Block size always 512** — no other sizes observed

---

*Generated from Nokia N91 logic analyzer traces, decoded via decode_sdio.py and analyze_all.py*
