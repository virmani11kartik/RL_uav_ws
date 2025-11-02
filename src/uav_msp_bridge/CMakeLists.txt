cmake_minimum_required(VERSION 3.8)
project(uav_msp_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Include directories
include_directories(include)

# Telemetry node
add_executable(telemetry_node 
  src/telemetry_node.cpp
  src/MSPClient.cpp
)

ament_target_dependencies(telemetry_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
)

# Motor control node
add_executable(arming_node 
  src/arming_node.cpp
  src/MSPClient.cpp
)

ament_target_dependencies(arming_node
  rclcpp
  std_msgs
)

add_executable(launch_node 
  src/launch_node.cpp
  src/MSPClient.cpp
)

ament_target_dependencies(launch_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
)

# Install executables
install(TARGETS
  telemetry_node
  arming_node
  launch_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()