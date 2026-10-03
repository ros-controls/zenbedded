cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES
        "${CMAKE_CURRENT_LIST_DIR}/../../.."
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(inverted_pendulum_tier2)
target_sources(app PRIVATE src/main.cpp)
target_compile_definitions(app PRIVATE _GNU_SOURCE _USE_MATH_DEFINES)
