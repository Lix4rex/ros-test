cmake_minimum_required(VERSION 3.8)
project(simple_sim_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Dépendances ROS
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# ================= SIMPLE CONTROLLER =================
add_executable(simple_controller
  src/simple_controller.cpp
)

ament_target_dependencies(simple_controller
  rclcpp
  geometry_msgs
)

# ================= LIDAR RANGES NODE =================
add_executable(lidar_ranges_node
  src/lidar_ranges_node.cpp
)

ament_target_dependencies(lidar_ranges_node
  rclcpp
  sensor_msgs
  std_msgs
)

# ================= INSTALL =================
install(TARGETS
  simple_controller
  lidar_ranges_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch urdf world
  DESTINATION share/${PROJECT_NAME}
)

# ================= TESTS =================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
