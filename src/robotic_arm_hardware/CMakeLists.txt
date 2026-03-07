cmake_minimum_required(VERSION 3.8)
project(robotic_arm_hardware)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(robotic_arm_hardware SHARED
  src/arm_hardware_interface.cpp
)

target_include_directories(robotic_arm_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robotic_arm_hardware
  rclcpp
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(
  hardware_interface robotic_arm_hardware.xml
)

install(
  TARGETS robotic_arm_hardware
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES robotic_arm_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
