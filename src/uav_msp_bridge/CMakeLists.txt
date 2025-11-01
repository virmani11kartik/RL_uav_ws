cmake_minimum_required(VERSION 3.16...3.28)

# Modern CMake behaviors
cmake_policy(SET CMP0057 NEW)  # IN_LIST in if()
cmake_policy(SET CMP0012 NEW)  # TRUE/FALSE recognized
if(POLICY CMP0130)
  cmake_policy(SET CMP0130 NEW)  # while() diagnosis
endif()

project(uav_msp_bridge)

# Require C++17 (or 20 if you prefer)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Node executable (includes MSPClient.cpp directly)
add_executable(telemetry_node
  src/telemetry_node.cpp
  src/MSPClient.cpp
)

# Include dirs for headers under include/uav_msp_bridge/...
target_include_directories(telemetry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Warnings (optional but recommended)
target_compile_options(telemetry_node PRIVATE -Wall -Wextra -Wpedantic)

# Link deps (single call)
ament_target_dependencies(telemetry_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
)

# Install the node
install(TARGETS telemetry_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install public headers (so others can include your client later)
install(DIRECTORY include/
  DESTINATION include
)

# Install launch & config
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
