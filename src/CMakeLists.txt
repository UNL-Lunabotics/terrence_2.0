cmake_minimum_required(VERSION 3.5)
project(terrence_2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial REQUIRED)

add_library(diffdrive_arduino_with_scoop SHARED
  hardware/diffdrive_arduino_with_scoop.cpp
)

target_include_directories(diffdrive_arduino_with_scoop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(diffdrive_arduino_with_scoop
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  serial
)

pluginlib_export_plugin_description_file(
  hardware_interface plugins/diffdrive_arduino_with_scoop.xml
)

install(
  TARGETS diffdrive_arduino_with_scoop
  DESTINATION lib
)

install(
  FILES plugins/diffdrive_arduino_with_scoop.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
