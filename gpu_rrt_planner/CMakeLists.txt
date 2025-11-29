cmake_minimum_required(VERSION 3.8)
project(gpu_rrt_planner LANGUAGES CXX CUDA)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Set CUDA architecture for Jetson Orin (compute capability 8.7)
set(CMAKE_CUDA_ARCHITECTURES 87)

# Include directories
include_directories(include)

# Build the GPU RRT node (C++ and CUDA source)
add_executable(gpu_rrt_node
    src/gpu_rrt_node.cpp
    src/gpu_rrt.cu
)
# Enable separable compilation for CUDA device code
set_target_properties(gpu_rrt_node PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

# Link against ROS2 packages
ament_target_dependencies(gpu_rrt_node rclcpp nav_msgs geometry_msgs visualization_msgs)

# Install targets and other files
install(TARGETS gpu_rrt_node
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
    DESTINATION include/
)
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)
install(PROGRAMS scripts/test_gpu_rrt.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
