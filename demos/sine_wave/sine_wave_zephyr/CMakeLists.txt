cmake_minimum_required(VERSION 3.20.0)

set(ZENBEDDED_SCHEMA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/interface_schema.yaml)

list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../zenbedded_transport
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../zenbedded_rcl
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(sine_wave)
target_sources(app PRIVATE src/main.cpp)
target_compile_definitions(app PRIVATE _GNU_SOURCE _USE_MATH_DEFINES)

if(CONFIG_USB_DEVICE_STACK_NEXT)
  include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)
endif()

target_include_directories(app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../zenbedded_rcl/include
)

add_custom_target(lib_dev_sync SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../zenbedded_rcl/src
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../zenbedded_rcl/include
)
