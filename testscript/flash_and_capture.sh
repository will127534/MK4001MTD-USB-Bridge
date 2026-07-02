#!/bin/bash
# Flash firmware, reset Pico via SWD, capture the probe UART boot log.

set -euo pipefail
shopt -s nullglob

ELF="${1:-/home/pi/mk4001_bridge/build/mk4001_bridge.elf}"
CAPTURE_SEC="${2:-30}"
UART_ARG="${3:-${UART:-}}"
LOG="${LOG:-/tmp/uart_boot.txt}"
OPENOCD_LOG="${OPENOCD_LOG:-/tmp/mk4001_openocd.log}"

detect_uart() {
  if [[ -n "$UART_ARG" ]]; then
    printf '%s\n' "$UART_ARG"
    return 0
  fi

  local candidates=(
    /dev/serial/by-id/*Debug_Probe*if01
    /dev/ttyACM0
    /dev/ttyAMA0
  )
  local uart
  for uart in "${candidates[@]}"; do
    if [[ -e "$uart" ]]; then
      printf '%s\n' "$uart"
      return 0
    fi
  done

  printf 'No UART device found. Pass one explicitly as: %s <elf> <seconds> <uart>\n' "$0" >&2
  return 1
}

cleanup() {
  if [[ -n "${CAT_PID:-}" ]]; then
    kill "$CAT_PID" 2>/dev/null || true
    wait "$CAT_PID" 2>/dev/null || true
  fi
}

UART="$(detect_uart)"
trap cleanup EXIT

stty -F "$UART" 115200 raw -echo
: > "$LOG"

# Start UART capture before reset so the earliest boot messages are not lost.
cat "$UART" > "$LOG" &
CAT_PID=$!

sudo openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg \
  -c "adapter speed 1000" -c "init" -c "reset halt" -c "sleep 200" \
  -c "program $ELF verify" \
  -c "reset run" -c "exit" > "$OPENOCD_LOG" 2>&1

echo "[*] UART: $UART"
grep -E '(Programming|Verify|Error|shutdown command invoked)' "$OPENOCD_LOG" || true

echo "[*] Capturing UART for ${CAPTURE_SEC}s..."
sleep "$CAPTURE_SEC"
cleanup
trap - EXIT

echo "=== UART Boot Log ($UART) ==="
cat "$LOG"
