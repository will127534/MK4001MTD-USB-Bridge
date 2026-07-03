cmake_minimum_required(VERSION 3.13)

# This project is pinned to pico-sdk 2.2.0 (bundled TinyUSB 0.18.0): the
# vendored lib/tinyusb_patched/msc_device.c must match the SDK's TinyUSB.
set(MK4001_PICO_SDK_PIN "2.2.0")

# SDK resolution: an explicit PICO_SDK_PATH (CMake variable or environment)
# wins; without one, the pinned SDK release is fetched from GitHub at
# configure time (submodules included) for a reproducible build.
if (NOT PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_FETCH_FROM_GIT on)
    set(PICO_SDK_FETCH_FROM_GIT_TAG ${MK4001_PICO_SDK_PIN})
endif()
include(pico_sdk_import.cmake)

project(mk4001_bridge C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Enforce the SDK pin: the vendored TinyUSB MSC driver tracks the exact
# TinyUSB version bundled with this SDK release.
if (NOT PICO_SDK_VERSION_STRING VERSION_EQUAL MK4001_PICO_SDK_PIN)
    if (MK4001_ALLOW_SDK_MISMATCH)
        message(WARNING "pico-sdk ${PICO_SDK_VERSION_STRING} != pinned "
                "${MK4001_PICO_SDK_PIN}. Verify lib/tinyusb_patched/msc_device.c "
                "against the SDK's TinyUSB before trusting this build.")
    else()
        message(FATAL_ERROR "This project is pinned to pico-sdk "
                "${MK4001_PICO_SDK_PIN} (found ${PICO_SDK_VERSION_STRING}). "
                "The vendored TinyUSB MSC driver (lib/tinyusb_patched/) must "
                "match the SDK's bundled TinyUSB — re-apply msc_device.c.patch "
                "to the new upstream and refresh the copy, then update "
                "MK4001_PICO_SDK_PIN. Pass -DMK4001_ALLOW_SDK_MISMATCH=ON to "
                "build anyway.")
    endif()
endif()

# TinyUSB MSC class driver: compile the repo's patched copy instead of the
# stock one (app sense data preserved on read/write errors + Caching mode
# page with WCE=1). See lib/tinyusb_patched/ for the diff against upstream.
if (NOT PICO_TINYUSB_PATH)
    set(PICO_TINYUSB_PATH ${PICO_SDK_PATH}/lib/tinyusb)
endif()
get_target_property(TUD_BASE_SRCS tinyusb_device_base INTERFACE_SOURCES)
list(FILTER TUD_BASE_SRCS EXCLUDE REGEX "class/msc/msc_device\\.c$")
set_target_properties(tinyusb_device_base PROPERTIES INTERFACE_SOURCES "${TUD_BASE_SRCS}")

add_executable(mk4001_bridge
    src/main.c
    src/sdio_hw.c
    src/ata_sdio.c
    src/sdio_pio.c
    src/usb_descriptors.c
    src/msc_device.c
    lib/tinyusb_patched/msc_device.c
)

pico_generate_pio_header(mk4001_bridge ${CMAKE_CURRENT_LIST_DIR}/src/sdio.pio)

target_include_directories(mk4001_bridge PRIVATE src ${CMAKE_CURRENT_BINARY_DIR}
    ${PICO_TINYUSB_PATH}/src)

target_link_libraries(mk4001_bridge
    pico_stdlib
    hardware_gpio
    hardware_clocks
    hardware_pio
    hardware_dma
    tinyusb_device
    tinyusb_board
)

pico_enable_stdio_uart(mk4001_bridge 1)
pico_enable_stdio_usb(mk4001_bridge 0)

pico_add_extra_outputs(mk4001_bridge)
