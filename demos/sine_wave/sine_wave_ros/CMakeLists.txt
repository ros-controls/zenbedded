cmake_minimum_required(VERSION 3.16)
project(zenbedded_sine_wave)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(control_msgs REQUIRED)

# 1. Tier 2 ros2_control controller plugin library
add_library(zenbedded_sine_wave SHARED src/sine_wave_tier2.cpp)
target_include_directories(zenbedded_sine_wave PRIVATE include)
target_link_libraries(zenbedded_sine_wave
        PUBLIC
        controller_interface::controller_interface
        pluginlib::pluginlib
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(controller_interface sine_wave_controller.xml)

# 2. Tier 1 standalone C++ node
add_executable(sine_wave_tier1 src/sine_wave_tier1.cpp)
target_include_directories(sine_wave_tier1 PRIVATE include)
target_link_libraries(sine_wave_tier1
        PRIVATE
        rclcpp::rclcpp
        ${control_msgs_TARGETS}
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS zenbedded_sine_wave
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)
install(TARGETS sine_wave_tier1
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(FILES sine_wave_controller.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf config launch DESTINATION share/${PROJECT_NAME})

ament_package()
