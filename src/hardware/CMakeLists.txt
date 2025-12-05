cmake_minimum_required(VERSION 3.16)
project(diffdrive_arduino LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# 1. Dependencies
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

# 2. Compile the Library
add_library(
  diffdrive_arduino
  SHARED
  src/diffbot_system.cpp  
)

target_compile_features(diffdrive_arduino PUBLIC cxx_std_17)

target_include_directories(diffdrive_arduino PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/diffdrive_arduino>
)

ament_target_dependencies(
  diffdrive_arduino PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Link serial library (assuming it's a system library or found package)
target_link_libraries(diffdrive_arduino PUBLIC serial)

# Visibility definitions (Required for Windows/Cross-platform, good practice generally)
target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFFDRIVE_ARDUINO_BUILDING_DLL")

# 3. Export the Plugin to Ament Index
pluginlib_export_plugin_description_file(hardware_interface diffdrive_arduino.xml)

# 4. INSTALL RULES

# A. Install the XML file (THE FIX)
# The error happens because this file is missing from share/diffdrive_arduino
install(
  FILES diffdrive_arduino.xml
  DESTINATION share/diffdrive_arduino
)

# B. Install Headers
install(
  DIRECTORY hardware/include/
  DESTINATION include/diffdrive_arduino
)

# C. Install the Shared Library (.so)
install(TARGETS diffdrive_arduino
  EXPORT export_diffdrive_arduino
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 5. Export Targets
ament_export_targets(export_diffdrive_arduino HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()