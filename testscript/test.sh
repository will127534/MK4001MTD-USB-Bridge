#!/bin/bash
# MK4001MTD USB Bridge — integration test suite
# Usage: sudo ./test.sh [/dev/sdX]
# Requires: dd, cmp, openocd (for flash)
set -uo pipefail

DEV="${1:-/dev/sda}"
UART="/dev/ttyACM0"
UART_LOG="/tmp/mk4001_uart_test.txt"
PASS=0; FAIL=0; SKIP=0

# Drive geometry
TOTAL_SECTORS=7862400
BAD_LBA=1952
SAFE_LBA=$((TOTAL_SECTORS - 128))  # last 128 sectors for write tests

red()   { printf "\033[31m%s\033[0m\n" "$*"; }
green() { printf "\033[32m%s\033[0m\n" "$*"; }
yellow(){ printf "\033[33m%s\033[0m\n" "$*"; }
bold()  { printf "\033[1m%s\033[0m\n" "$*"; }

result() {
    if [ "$1" -eq 0 ]; then
        green "  PASS: $2"; ((PASS++))
    else
        red "  FAIL: $2"; ((FAIL++))
    fi
}

skip() { yellow "  SKIP: $1"; ((SKIP++)); }

# ── Setup ──────────────────────────────────────────────────
bold "=== MK4001MTD Bridge Test Suite ==="
echo "Device: $DEV"
echo ""

# Start UART capture if available
UART_PID=""
if [ -c "$UART" ]; then
    stty -F "$UART" 115200 raw -echo 2>/dev/null || true
    cat "$UART" > "$UART_LOG" &
    UART_PID=$!
    echo "UART capture: $UART → $UART_LOG (pid $UART_PID)"
else
    echo "UART not available ($UART), skipping UART capture"
fi

cleanup() {
    [ -n "$UART_PID" ] && kill "$UART_PID" 2>/dev/null || true
    rm -f /tmp/mk4001_test_write.bin /tmp/mk4001_test_read.bin
}
trap cleanup EXIT

# Wait for block device
if [ ! -b "$DEV" ]; then
    echo "Waiting for $DEV..."
    for i in $(seq 1 60); do
        [ -b "$DEV" ] && break
        sleep 1
    done
fi
if [ ! -b "$DEV" ]; then
    red "FATAL: $DEV not found after 60s"
    exit 1
fi
echo "Block device ready."
echo ""

# ── Test 1: MBR read ──────────────────────────────────────
bold "TEST 1: MBR read"
MBR_BIN=$(dd if="$DEV" bs=512 count=1 2>/dev/null | tail -c 2 | od -A n -t x1 | tr -d ' \n')
if [ "$MBR_BIN" = "55aa" ]; then
    result 0 "MBR signature 0x55AA"
else
    result 1 "MBR signature expected 55aa, got '$MBR_BIN'"
fi

# ── Test 2: Read capacity ─────────────────────────────────
bold "TEST 2: Disk size"
SIZE=$(blockdev --getsz "$DEV" 2>/dev/null || echo 0)
if [ "$SIZE" -eq "$TOTAL_SECTORS" ]; then
    result 0 "Sector count $SIZE matches expected $TOTAL_SECTORS"
else
    result 1 "Sector count $SIZE != expected $TOTAL_SECTORS"
fi

# ── Test 3: Sequential read (skip bad area) ───────────────
bold "TEST 3: Sequential read 1 MB from LBA 2048"
READ_OUT=$(dd if="$DEV" bs=32768 skip=32 count=32 of=/dev/null 2>&1)
READ_BYTES=$(echo "$READ_OUT" | grep -oP '^\d+ bytes' | grep -oP '^\d+')
if [ "${READ_BYTES:-0}" -eq 1048576 ]; then
    SPEED=$(echo "$READ_OUT" | grep -oP '[\d.]+ [kMG]B/s' || echo "?")
    result 0 "Read 1 MB OK ($SPEED)"
else
    result 1 "Expected 1048576 bytes, got ${READ_BYTES:-0}"
fi

# ── Test 4: Bad sector handling ───────────────────────────
bold "TEST 4: Bad sector read (LBA $BAD_LBA)"
# Should complete (bad sectors zero-filled) without hanging
if timeout 30 dd if="$DEV" bs=512 skip=$BAD_LBA count=1 of=/dev/null 2>/dev/null; then
    result 0 "Bad sector read completed (zero-filled)"
else
    result 1 "Bad sector read failed or timed out"
fi

# ── Test 5: Read sectors around bad area ──────────────────
bold "TEST 5: Read around bad sector (LBA $((BAD_LBA-2)) to $((BAD_LBA+2)))"
if timeout 30 dd if="$DEV" bs=512 skip=$((BAD_LBA-2)) count=5 of=/dev/null 2>/dev/null; then
    result 0 "Read 5 sectors around bad area OK"
else
    result 1 "Read around bad sector failed"
fi

# ── Test 6: Write + readback verify ───────────────────────
bold "TEST 6: Write + readback verify (64 sectors at LBA $SAFE_LBA)"
dd if=/dev/urandom bs=512 count=64 of=/tmp/mk4001_test_write.bin 2>/dev/null
if dd if=/tmp/mk4001_test_write.bin of="$DEV" bs=512 seek=$SAFE_LBA count=64 \
      conv=notrunc oflag=direct 2>/dev/null; then
    sync
    dd if="$DEV" bs=512 skip=$SAFE_LBA count=64 of=/tmp/mk4001_test_read.bin \
       iflag=direct 2>/dev/null
    if cmp -s /tmp/mk4001_test_write.bin /tmp/mk4001_test_read.bin; then
        result 0 "Write/readback 32 KB verified"
    else
        result 1 "Write/readback mismatch!"
    fi
else
    result 1 "Write failed"
fi

# ── Test 7: Large sequential write ────────────────────────
SAFE_LBA7=$((TOTAL_SECTORS - 512))  # need 512 sectors for 256KB
bold "TEST 7: Write 256 KB sequential (LBA $SAFE_LBA7)"
dd if=/dev/urandom bs=512 count=512 of=/tmp/mk4001_test_write.bin 2>/dev/null
WRITE_OUT=$(dd if=/tmp/mk4001_test_write.bin of="$DEV" bs=512 seek=$SAFE_LBA7 \
               count=512 conv=notrunc oflag=direct 2>&1)
WRITE_SPEED=$(echo "$WRITE_OUT" | grep -oP '[\d.]+ [kMG]B/s' || echo "?")
sync
dd if="$DEV" bs=512 skip=$SAFE_LBA7 count=512 of=/tmp/mk4001_test_read.bin \
   iflag=direct 2>/dev/null
if cmp -s /tmp/mk4001_test_write.bin /tmp/mk4001_test_read.bin; then
    result 0 "Write 256 KB + verify OK ($WRITE_SPEED)"
else
    result 1 "Large write/readback mismatch!"
fi

# ── Test 8: Idle power gate + wake ────────────────────────
bold "TEST 8: Idle power gate (5s) + wake"
echo "  Waiting 10s for idle power gate..."
sleep 10
if timeout 10 dd if="$DEV" bs=512 skip=100 count=1 of=/dev/null 2>/dev/null; then
    result 0 "Read after power gate wake OK"
else
    result 1 "Read after power gate wake failed"
fi

# ── Test 9: Repeated power gate cycles ────────────────────
bold "TEST 9: Power gate cycle x3"
CYCLE_OK=0
for c in 1 2 3; do
    echo "  Cycle $c: waiting 35s..."
    sleep 10
    if timeout 10 dd if="$DEV" bs=512 skip=$((c * 1000)) count=1 of=/dev/null 2>/dev/null; then
        ((CYCLE_OK++))
    fi
done
if [ "$CYCLE_OK" -eq 3 ]; then
    result 0 "All 3 power gate cycles OK"
else
    result 1 "Only $CYCLE_OK/3 power gate cycles succeeded"
fi

# ── Summary ───────────────────────────────────────────────
echo ""
bold "=== Results ==="
green "PASS: $PASS"
[ "$FAIL" -gt 0 ] && red "FAIL: $FAIL" || echo "FAIL: 0"
[ "$SKIP" -gt 0 ] && yellow "SKIP: $SKIP"
echo ""

if [ -n "$UART_PID" ]; then
    sleep 1
    kill "$UART_PID" 2>/dev/null || true
    wait "$UART_PID" 2>/dev/null || true
    bold "=== UART Log ==="
    cat "$UART_LOG"
fi

exit "$FAIL"
