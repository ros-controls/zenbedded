cmake_minimum_required(VERSION 3.20.0)

# Load the Transport Module
get_filename_component(WORKSPACE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../.. ABSOLUTE)
list(APPEND ZEPHYR_EXTRA_MODULES ${WORKSPACE_DIR}/zenbedded_transport)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zenbedded_tier2_test)

# The zenbedded_transport module now automatically generates interface_data.h
# in the background before compiling the app sources. (RCL can use it by including <interface_data.h>)

target_sources(app PRIVATE src/main.cpp)
