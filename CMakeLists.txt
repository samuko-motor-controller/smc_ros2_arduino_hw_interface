cmake_minimum_required(VERSION 3.16)
project(smc_ros2_arduino_hw_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  smc_ros2_arduino_hw_interface
  SHARED
  hardware/smc_system.cpp
)
target_compile_features(smc_ros2_arduino_hw_interface PUBLIC cxx_std_17)
target_include_directories(smc_ros2_arduino_hw_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/smc_ros2_arduino_hw_interface>
)
ament_target_dependencies(
  smc_ros2_arduino_hw_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(smc_ros2_arduino_hw_interface PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "SMC_ROS2_ARDUINO_HW_INTERFACE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface smc_ros2_arduino_hw_interface.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/smc_ros2_arduino_hw_interface
)

install(TARGETS smc_ros2_arduino_hw_interface
  EXPORT export_smc_ros2_arduino_hw_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_smc_ros2_arduino_hw_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
