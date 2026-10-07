cmake_minimum_required(VERSION 3.16)
project(zenbedded_inverted_pendulum)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

option(WITH_RAYLIB "Build Raylib 3D visualizer for mock hardware" ON)

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

set(MOCK_PENDULUM_SOURCES src/mock_pendulum_hardware.cpp)
if(WITH_RAYLIB)
    find_package(raylib QUIET)
    if(raylib_FOUND)
        message(STATUS "Building mock hardware with Raylib 3D visualizer")
        list(APPEND MOCK_PENDULUM_SOURCES src/pendulum_visualizer.cpp)
    else()
        message(WARNING "WITH_RAYLIB=ON was requested, but raylib was not found. Building without visualizer.")
    endif()
endif()

add_library(mock_pendulum_hardware SHARED ${MOCK_PENDULUM_SOURCES})
target_include_directories(mock_pendulum_hardware PRIVATE include)
target_link_libraries(mock_pendulum_hardware
        PUBLIC
        hardware_interface::hardware_interface
        pluginlib::pluginlib
        rclcpp_lifecycle::rclcpp_lifecycle
)
if(WITH_RAYLIB AND raylib_FOUND)
    target_compile_definitions(mock_pendulum_hardware PUBLIC MOCK_PENDULUM_WITH_RAYLIB)
    target_link_libraries(mock_pendulum_hardware PUBLIC raylib)
endif()
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
