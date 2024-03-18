cmake_minimum_required(VERSION 3.12)

set(PICO_SDK_PATH "/pico-sdk")
set(PICO_BOARD "pico")
#set(PICO_CXX_ENABLE_EXCEPTIONS 1)
#set(PICO_CXX_ENABLE_RTTI 1)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import_optional.cmake)

set(PROJECT_NAME "redeye")

project(${PROJECT_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

add_executable(${PROJECT_NAME}
        main.cpp
        pi-pico-cpp/src/crc64.c
        pi-pico-cpp/src/Color.cpp
        pi-pico-cpp/src/Settings.cpp
)

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pi-pico-cpp/pio/ws2812b.pio)

target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/pi-pico-cpp/include
)

target_link_libraries(${PROJECT_NAME}
        pico_stdio 
        pico_stdlib
        hardware_flash
        hardware_pio
        pico_sync
        hardware_pwm
        hardware_i2c
)

# Configure USB for stdio (disables uart)
target_compile_definitions(${PROJECT_NAME} PRIVATE LOGGING_ENABLED)
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})
