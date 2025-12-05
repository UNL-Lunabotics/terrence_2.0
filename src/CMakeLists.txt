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

add_library(terrence_hw SHARED
  hardware/terrence.cpp
)

target_include_directories(terrence_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(terrence_hw
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# pament_target_dependencies(terrence_hw
#   rclcpp
#   rclcpp_lifecycle
#   hardware_interface
#   pluginlib
# )

pluginlib_export_plugin_description_file(
  hardware_interface plugins/terrence.xml
)

install(
  TARGETS terrence_hw
  DESTINATION lib
)

install(
  FILES plugins/terrence.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config description launch worlds hardware
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
