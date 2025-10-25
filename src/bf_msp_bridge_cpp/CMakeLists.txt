cmake_minimum_required(VERSION 3.8)
project(bf_msp_bridge_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# Add executable FIRST
add_executable(bf_msp_bridge src/msp_bridge.cpp)

# Add include directories
target_include_directories(bf_msp_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Set C++ standard
target_compile_features(bf_msp_bridge PUBLIC c_std_99 cxx_std_17)

# THEN add dependencies (AFTER the target is created)
ament_target_dependencies(bf_msp_bridge
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
)

# Install targets
install(TARGETS bf_msp_bridge
  DESTINATION lib/${PROJECT_NAME})

# Install launch files (if you have any)
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()