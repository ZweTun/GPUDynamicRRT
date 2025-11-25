cmake_minimum_required(VERSION 3.5)
project(cpu_rrt)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Build the C++ RRT node
add_executable(rrt_node src/rrt_node.cpp src/rrt.cpp)
ament_target_dependencies(rrt_node 
  rclcpp geometry_msgs nav_msgs ackermann_msgs sensor_msgs std_msgs visualization_msgs tf2_ros
)

# Install C++ node executable
install(TARGETS rrt_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules and scripts (if any)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/rrt_node.py  # original Python node (if still needed)
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
