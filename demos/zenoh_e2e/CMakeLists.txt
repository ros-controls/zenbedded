cmake_minimum_required(VERSION 3.28.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zenoh_e2e)

zephyr_compile_definitions(
  Z_CONFIG_SOCKET_TIMEOUT=1
  Z_FEATURE_UNSTABLE_API=1
  Z_TRANSPORT_LEASE=60000
)

target_sources(app PRIVATE src/main.c)
