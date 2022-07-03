cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(airquality)

pico_sdk_init()

add_subdirectory(bme688)
add_subdirectory(common)

add_executable(airquality
    main.c
    bme688/bme68x.c
    bme688/bme68x.h
    bme688/bme68x_defs.h
    common/common.c
    common/common.h
)

target_link_libraries(airquality
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(airquality 1)
pico_enable_stdio_uart(airquality 0)

pico_add_extra_outputs(airquality)