cmake_minimum_required(VERSION 3.20.0)

if(NOT DEFINED ENV{ZENBEDDED_MODULE_ROOT})
  message(FATAL_ERROR "Set ZENBEDDED_MODULE_ROOT to the Zenbedded repository root")
endif()
list(APPEND ZEPHYR_EXTRA_MODULES $ENV{ZENBEDDED_MODULE_ROOT})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zenbedded_external_consumer)

target_sources(app PRIVATE src/main.c)

if(CONFIG_ZENBEDDED_RCL)
  target_sources(app PRIVATE src/client_smoke.cpp)
endif()