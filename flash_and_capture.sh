#!/bin/bash
# Flash firmware, reset Pico via SWD, capture UART boot log
ELF="${1:-/home/pi/mk4001_bridge/build/mk4001_bridge.elf}"
UART="/dev/ttyAMA0"
CAPTURE_SEC="${2:-30}"
LOG="/tmp/uart_boot.txt"

set -e

stty -F "$UART" 115200 raw -echo

# Start UART capture BEFORE reset
cat "$UART" > "$LOG" &
CAT_PID=$!

# Flash + reset via OpenOCD (program includes reset run)
sudo openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg \
  -c "adapter speed 1000" -c "init" -c "reset halt" -c "sleep 200" \
  -c "program $ELF verify" \
  -c "reset run" -c "exit" 2>&1 | grep -E '(Programming|Verify|Error)'

echo "[*] Capturing UART for ${CAPTURE_SEC}s..."
sleep "$CAPTURE_SEC"
kill "$CAT_PID" 2>/dev/null
wait "$CAT_PID" 2>/dev/null

echo "=== UART Boot Log ==="
cat "$LOG"
