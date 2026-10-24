cmake_minimum_required(VERSION 3.20.0)

set(ZENBEDDED_SCHEMA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/interface_schema.yaml)

include(${CMAKE_CURRENT_LIST_DIR}/../../common/common.cmake)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(sine_wave)
target_sources(app PRIVATE src/main.cpp)
target_compile_definitions(app PRIVATE _GNU_SOURCE _USE_MATH_DEFINES)
