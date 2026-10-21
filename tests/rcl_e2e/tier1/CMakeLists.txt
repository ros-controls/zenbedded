cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES
    "${CMAKE_CURRENT_LIST_DIR}/../../../zenbedded_transport"
    "${CMAKE_CURRENT_LIST_DIR}/../../../zenbedded_rcl"
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(rcl_e2e_tier1)
target_sources(app PRIVATE src/main.cpp)
