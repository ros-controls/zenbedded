cmake_minimum_required(VERSION 3.20.0)

include(${CMAKE_CURRENT_LIST_DIR}/../../../demos/common/common.cmake)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(inverted_pendulum_tier1)
target_sources(app PRIVATE src/main.cpp)
target_compile_definitions(app PRIVATE _GNU_SOURCE _USE_MATH_DEFINES)
