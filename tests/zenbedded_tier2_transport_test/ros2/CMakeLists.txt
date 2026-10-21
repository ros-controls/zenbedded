cmake_minimum_required(VERSION 3.20.0)
project(zenbedded_tier2_test_ros)

find_package(ament_cmake REQUIRED)

zenbedded_generate_header(
  YAML config/interface_schema.yaml
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/zephyr/include/generated/interface_data.h
  TARGET generate_test_header
)

ament_package()
