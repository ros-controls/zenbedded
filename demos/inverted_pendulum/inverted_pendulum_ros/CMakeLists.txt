cmake_minimum_required(VERSION 3.16)
project(zenbedded_inverted_pendulum)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()


find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


add_library(zenbedded_inverted_pendulum SHARED src/inverted_pendulum_controller.cpp)
target_include_directories(zenbedded_inverted_pendulum PRIVATE include)
target_link_libraries(zenbedded_inverted_pendulum
        PUBLIC
        controller_interface::controller_interface
        pluginlib::pluginlib
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(controller_interface inverted_pendulum_controller.xml)

add_library(mock_pendulum_hardware SHARED src/mock_pendulum_hardware.cpp)
target_include_directories(mock_pendulum_hardware PRIVATE include)
target_link_libraries(mock_pendulum_hardware
        PUBLIC
        hardware_interface::hardware_interface
        pluginlib::pluginlib
        rclcpp_lifecycle::rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(hardware_interface mock_pendulum_hardware.xml)


install(DIRECTORY include/ DESTINATION include)
install(TARGETS zenbedded_inverted_pendulum mock_pendulum_hardware
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(FILES inverted_pendulum_controller.xml mock_pendulum_hardware.xml
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_mock_pendulum test/test_mock_pendulum_launch.py)
endif()

ament_package()
