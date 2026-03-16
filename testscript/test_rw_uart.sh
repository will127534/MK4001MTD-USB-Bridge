#!/bin/bash
# End-to-end filesystem write/read test for the MK4001 bridge while capturing UART logs.

set -euo pipefail
shopt -s nullglob

SIZE_MB="${1:-32}"
DEVICE_ARG="${2:-${DEVICE:-}}"
UART_ARG="${3:-${UART:-}}"
PARTITION_ARG="${4:-${PARTITION:-}}"
POST_IDLE_SEC="${POST_IDLE_SEC:-2}"
FSCK_STRICT="${FSCK_STRICT:-0}"
TEST_PREFIX="${TEST_PREFIX:-mk4001_rw_test}"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="$(mktemp -d "/tmp/${TEST_PREFIX}.${TIMESTAMP}.XXXXXX")"
MOUNTPOINT="${RUN_DIR}/mnt"
UART_LOG="${RUN_DIR}/uart.log"
SUMMARY_LOG="${RUN_DIR}/summary.txt"
SOURCE_FILE="${RUN_DIR}/source.bin"
READBACK_FILE="${RUN_DIR}/readback.bin"
TEST_NAME="${TEST_PREFIX}_${TIMESTAMP}.bin"

CAPTURE_PID=""
ACTIVE_MOUNTPOINT=""
MOUNTED_BY_SCRIPT=0
FSTYPE=""
FINAL_EXIT=0

log() {
  printf '%s\n' "$*" | tee -a "$SUMMARY_LOG"
}

cleanup() {
  if [[ -n "$CAPTURE_PID" ]]; then
    kill "$CAPTURE_PID" 2>/dev/null || true
    wait "$CAPTURE_PID" 2>/dev/null || true
  fi

  if [[ "$MOUNTED_BY_SCRIPT" -eq 1 ]] && [[ -n "$ACTIVE_MOUNTPOINT" ]] &&
     findmnt -rn "$ACTIVE_MOUNTPOINT" > /dev/null 2>&1; then
    sudo umount "$ACTIVE_MOUNTPOINT" || true
  fi
}

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

  printf 'No UART device found. Set UART=<device> or pass it as argv[3].\n' >&2
  return 1
}

detect_disk() {
  if [[ -n "$DEVICE_ARG" ]]; then
    realpath "$DEVICE_ARG"
    return 0
  fi

  local disk
  disk="$(lsblk -dpnr -o NAME,MODEL,TRAN | awk '$2 == "MK4001MTD" && $3 == "usb" { print $1; exit }')"
  if [[ -z "$disk" ]]; then
    printf 'Could not find the MK4001 USB disk. Set DEVICE=<device> or pass it as argv[2].\n' >&2
    return 1
  fi

  realpath "$disk"
}

detect_partition() {
  if [[ -n "$PARTITION_ARG" ]]; then
    realpath "$PARTITION_ARG"
    return 0
  fi

  local disk="$1"
  local part
  part="$(lsblk -lnpo NAME,TYPE "$disk" | awk '$2 == "part" { print $1; exit }')"
  if [[ -z "$part" ]]; then
    # The bridge is often formatted as a "superfloppy" with FAT directly on the disk node.
    printf '%s\n' "$disk"
    return 0
  fi

  realpath "$part"
}

measure_ns() {
  date +%s%N
}

format_rate() {
  local bytes="$1"
  local duration_ns="$2"
  awk -v bytes="$bytes" -v ns="$duration_ns" '
    BEGIN {
      sec = ns / 1000000000.0;
      if (sec <= 0) sec = 0.000001;
      mibps = (bytes / 1048576.0) / sec;
      kbps = (bytes / 1000.0) / sec;
      printf "%.2f MiB/s (%.0f kB/s)", mibps, kbps;
    }
  '
}

mount_test_volume() {
  if [[ "$FSTYPE" == "vfat" || "$FSTYPE" == "msdos" || "$FSTYPE" == "exfat" ]]; then
    sudo mount -o "uid=$(id -u),gid=$(id -g)" "$PARTITION" "$MOUNTPOINT"
  else
    sudo mount "$PARTITION" "$MOUNTPOINT"
  fi
  ACTIVE_MOUNTPOINT="$MOUNTPOINT"
  MOUNTED_BY_SCRIPT=1
}

trap cleanup EXIT

mkdir -p "$MOUNTPOINT"
: > "$SUMMARY_LOG"

UART="$(detect_uart)"
DISK="$(detect_disk)"
PARTITION="$(detect_partition "$DISK")"
FSTYPE="$(lsblk -dnro FSTYPE "$PARTITION" | head -n1)"
EXISTING_MOUNTPOINT="$(findmnt -rn -S "$PARTITION" -o TARGET || true)"

log "[*] Run dir: ${RUN_DIR}"
log "[*] Disk: ${DISK}"
log "[*] Filesystem node: ${PARTITION}"
log "[*] Filesystem type: ${FSTYPE:-unknown}"
log "[*] UART: ${UART}"
log "[*] Test file: ${TEST_NAME}"
log "[*] Payload size: ${SIZE_MB} MiB"

stty -F "$UART" 115200 raw -echo
: > "$UART_LOG"
cat "$UART" > "$UART_LOG" &
CAPTURE_PID="$!"

log "[*] Generating random payload..."
dd if=/dev/urandom of="$SOURCE_FILE" bs=1M count="$SIZE_MB" status=none
SOURCE_BYTES="$(stat -c%s "$SOURCE_FILE")"
SOURCE_SHA="$(sha256sum "$SOURCE_FILE" | awk '{print $1}')"
log "[*] Source SHA256: ${SOURCE_SHA}"

if [[ -n "$EXISTING_MOUNTPOINT" ]]; then
  ACTIVE_MOUNTPOINT="$EXISTING_MOUNTPOINT"
  log "[*] Reusing existing mount at ${ACTIVE_MOUNTPOINT}"
else
  log "[*] Mounting ${PARTITION}..."
  mount_test_volume
fi
DEST_FILE="${ACTIVE_MOUNTPOINT}/${TEST_NAME}"

log "[*] Writing payload to ${DEST_FILE}..."
WRITE_START_NS="$(measure_ns)"
dd if="$SOURCE_FILE" of="$DEST_FILE" bs=1M conv=fsync status=none
sync
WRITE_END_NS="$(measure_ns)"
WRITE_NS="$((WRITE_END_NS - WRITE_START_NS))"
WRITE_SHA="$(sha256sum "$DEST_FILE" | awk '{print $1}')"

if [[ "$WRITE_SHA" != "$SOURCE_SHA" ]]; then
  log "[ERROR] Checksum mismatch immediately after write."
  log "        source=${SOURCE_SHA}"
  log "        dest=${WRITE_SHA}"
  exit 1
fi

log "[*] Write rate: $(format_rate "$SOURCE_BYTES" "$WRITE_NS")"

log "[*] Remounting for persisted readback..."
sudo umount "$ACTIVE_MOUNTPOINT"
ACTIVE_MOUNTPOINT=""
MOUNTED_BY_SCRIPT=0
mount_test_volume
DEST_FILE="${ACTIVE_MOUNTPOINT}/${TEST_NAME}"

if [[ ! -f "$DEST_FILE" ]]; then
  log "[ERROR] ${DEST_FILE} is missing after remount."
  exit 1
fi

READ_START_NS="$(measure_ns)"
dd if="$DEST_FILE" of="$READBACK_FILE" bs=1M status=none
READ_END_NS="$(measure_ns)"
READ_NS="$((READ_END_NS - READ_START_NS))"
READ_SHA="$(sha256sum "$READBACK_FILE" | awk '{print $1}')"

if [[ "$READ_SHA" != "$SOURCE_SHA" ]]; then
  log "[ERROR] Readback checksum mismatch."
  log "        source=${SOURCE_SHA}"
  log "        readback=${READ_SHA}"
  exit 1
fi

log "[*] Read rate: $(format_rate "$SOURCE_BYTES" "$READ_NS")"
log "[*] Checksum verification passed after remount."

log "[*] Removing test file from the bridge..."
sudo rm -f "$DEST_FILE"
sync
sudo umount "$ACTIVE_MOUNTPOINT"
ACTIVE_MOUNTPOINT=""
MOUNTED_BY_SCRIPT=0

if command -v fsck.vfat > /dev/null 2>&1; then
  log "[*] Running fsck.vfat -n..."
  set +e
  sudo fsck.vfat -n "$PARTITION" | tee -a "$SUMMARY_LOG"
  FSCK_RC="${PIPESTATUS[0]}"
  set -e
  if [[ "$FSCK_RC" -ne 0 ]]; then
    log "[WARN] fsck.vfat -n returned ${FSCK_RC}; inspect the output above."
    if [[ "$FSCK_STRICT" == "1" ]]; then
      FINAL_EXIT="$FSCK_RC"
    fi
  fi
fi

log "[*] Waiting ${POST_IDLE_SEC}s for post-I/O UART logs..."
sleep "$POST_IDLE_SEC"

cleanup
trap - EXIT

log "[*] UART log: ${UART_LOG}"
log "[*] Summary: ${SUMMARY_LOG}"
exit "$FINAL_EXIT"
