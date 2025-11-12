cmake_minimum_required(VERSION 3.8)
project(lab7_pkg_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

add_executable(rrt_node
  src/rrt_node.cpp
)

ament_target_dependencies(rrt_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  ackermann_msgs
  visualization_msgs
  tf2_ros
  ament_index_cpp
)

install(TARGETS rrt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

ament_package()
