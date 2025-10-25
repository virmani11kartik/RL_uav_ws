cmake_minimum_required(VERSION 3.10)
project(bf_msp_bridge_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(msp_bridge src/msp_bridge.cpp)
target_include_directories(msp_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(msp_bridge rclcpp std_msgs sensor_msgs geometry_msgs)

install(TARGETS msp_bridge DESTINATION lib/${PROJECT_NAME})

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include)
  install(DIRECTORY include/ DESTINATION include)
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
