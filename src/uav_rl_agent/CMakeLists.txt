cmake_minimum_required(VERSION 3.8)
project(uav_rl_agent)

# Default flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ---- C++ executable ----
add_executable(rl_agent_node_cpp src/rl_agent_node_cpp.cpp)
ament_target_dependencies(rl_agent_node_cpp rclcpp std_msgs geometry_msgs)

install(TARGETS rl_agent_node_cpp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ---- Python package + script (no setup.py needed) ----
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/rl_agent_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
