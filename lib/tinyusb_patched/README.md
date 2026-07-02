# Patched TinyUSB MSC class driver

`msc_device.c` is a patched copy of TinyUSB's MSC device class driver
(`src/class/msc/msc_device.c`, upstream tag `0.18.0`, the version bundled
with the pico-sdk). The build compiles this copy instead of the stock one —
`CMakeLists.txt` filters the original out of the `tinyusb_device_base`
interface sources — so a stock pico-sdk checkout builds correct firmware
with no SDK modifications.

`msc_device.c.patch` is the full diff against upstream. Two changes:

1. **Preserve application sense data on READ10/WRITE10 errors.**
   Stock TinyUSB unconditionally overwrites the app-set sense with
   "medium not present" when a read/write callback fails, which breaks the
   bridge's `MEDIUM ERROR` bad-sector reporting. The patch only applies the
   default sense when the app hasn't set one.

2. **MODE SENSE(6) reports a Caching mode page with WCE=1.**
   The bridge (and the MK4001MTD itself) caches writes; advertising the
   cache tells the host to issue SYNCHRONIZE CACHE at integrity points
   (fsync, unmount, suspend), which the firmware honors — standard
   write-back cache semantics, like any real ATA-USB bridge.

If the pico-sdk (and its bundled TinyUSB) is ever updated, re-apply
`msc_device.c.patch` to the new upstream file and refresh this copy.

`hub-host-fixes.patch` is unrelated to this firmware (TinyUSB *host*-mode
hub robustness fixes from earlier local development — this project is
device-only and never compiles `hub.c`). It is preserved here for
reference only; the SDK itself is kept pristine.
