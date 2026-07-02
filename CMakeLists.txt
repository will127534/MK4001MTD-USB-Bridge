cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH /home/pi/pico/pico-sdk)
include(pico_sdk_import.cmake)

project(mk4001_bridge C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

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
