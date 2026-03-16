#!/bin/bash
# Flash the current MK4001 bridge firmware to the Pico over SWD.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
ELF="${1:-${ROOT_DIR}/build/mk4001_bridge.elf}"
ADAPTER_SPEED="${ADAPTER_SPEED:-1000}"

if [[ ! -f "$ELF" ]]; then
  echo "ELF not found: $ELF" >&2
  exit 1
fi

sudo openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg \
  -c "adapter speed ${ADAPTER_SPEED}" \
  -c "init" \
  -c "reset halt" \
  -c "sleep 200" \
  -c "program ${ELF} verify" \
  -c "reset run" \
  -c "exit"
