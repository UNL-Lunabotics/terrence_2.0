cmake_minimum_required(VERSION 3.16)
project(diffdrive_arduino LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  diffdrive_arduino
  SHARED
  src/diffdrive_system.cpp
)

target_compile_features(diffdrive_arduino PUBLIC cxx_std_17)

# IMPORTANT: include/ is now directly under this package root
target_include_directories(diffdrive_arduino PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  diffdrive_arduino PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Only keep this if you *actually* have a library named "serial" to link against.
# Otherwise comment it out or replace with whatever you really use.
target_link_libraries(diffdrive_arduino PUBLIC serial)

# Export visibility macro
target_compile_definitions(diffdrive_arduino PRIVATE "DIFFDRIVE_ARDUINO_BUILDING_DLL")

# Export hardware plugin description to pluginlib
pluginlib_export_plugin_description_file(hardware_interface diffdrive_arduino.xml)

## INSTALL

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the shared library
install(TARGETS diffdrive_arduino
  EXPORT export_diffdrive_arduino
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin XML so hardware_interface can find it
install(
  FILES diffdrive_arduino.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_diffdrive_arduino HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
