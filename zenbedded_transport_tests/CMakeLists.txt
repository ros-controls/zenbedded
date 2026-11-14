cmake_minimum_required(VERSION 3.16)
project(zenbedded_transport_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)

  add_library(zcdr_under_test STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../zenbedded_transport/src/serialization.c
  )
  target_include_directories(zcdr_under_test PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../zenbedded_transport/include
  )
  target_compile_definitions(zcdr_under_test PUBLIC CONFIG_ZENBEDDED_TIER_1=1)

  ament_add_gtest(test_cdr_conformance
    test/test_cdr_conformance.cpp
  )
  target_link_libraries(test_cdr_conformance
    zcdr_under_test
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
  )

  ament_add_gtest(test_serialization_bounds
    test/test_serialization_bounds.cpp
  )
  target_include_directories(test_serialization_bounds PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../zenbedded_rcl/include
  )
  target_link_libraries(test_serialization_bounds zcdr_under_test)
endif()

ament_package()
