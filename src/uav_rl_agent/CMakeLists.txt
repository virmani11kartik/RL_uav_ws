cmake_minimum_required(VERSION 3.8)
project(uav_rl_agent)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ---- C++ node ----
add_executable(rl_agent_node_cpp src/rl_agent_node.cpp)
ament_target_dependencies(rl_agent_node_cpp rclcpp sensor_msgs std_msgs)
install(TARGETS rl_agent_node_cpp
  DESTINATION lib/${PROJECT_NAME})

# ---- Python package + script ----
ament_python_install_package(uav_rl_agent_py)

install(PROGRAMS
  scripts/rl_agent_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
