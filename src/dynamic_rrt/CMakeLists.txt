cmake_minimum_required(VERSION 3.16) # CMake version on ROS 2 Foxy
project(dynamic_rrt)

option(DYNAMIC_RRT_BUILD_CUDA "Build the CUDA-accelerated RRT node" ON)

# Enable compiler warnings.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(ament_cmake REQUIRED)
set(core_dependencies
    ackermann_msgs
    ament_index_cpp
    geometry_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    tf2_ros
    visualization_msgs
)
foreach(dependency IN LISTS core_dependencies)
    find_package(${dependency} REQUIRED)
endforeach()

# Add the CPU executable.
add_executable(rrt_cpu src/rrt_base.cpp src/rrt_cpu.cpp)
target_compile_features(rrt_cpu PRIVATE cxx_std_17)
ament_target_dependencies(rrt_cpu ${core_dependencies})
install(TARGETS rrt_cpu DESTINATION lib/${PROJECT_NAME})

if(DYNAMIC_RRT_BUILD_CUDA)
    # Add the CUDA executable.

    # https://cliutils.gitlab.io/modern-cmake/chapters/packages/CUDA.html
    include(CheckLanguage)
    check_language(CUDA)
    if(NOT CMAKE_CUDA_COMPILER)
        message(FATAL_ERROR "DYNAMIC_RRT_BUILD_CUDA is ON but no CUDA compiler could be found.")
    endif()
    message(STATUS "CUDA compiler found: ${CMAKE_CUDA_COMPILER}")

    find_package(CUDAToolkit REQUIRED)

    add_executable(rrt_cuda src/rrt_base.cpp src/rrt_cuda.cpp src/rrt_kernels.cu)
    target_compile_features(rrt_cuda PRIVATE cxx_std_17 cuda_std_17)
    ament_target_dependencies(rrt_cuda ${core_dependencies})
    target_link_libraries(rrt_cuda CUDA::cudart)
    set_target_properties(rrt_cuda PROPERTIES
        CUDA_ARCHITECTURES "72;89" # Jetson Xavier NX and GeForce RTX 4090
        CUDA_SEPARABLE_COMPILATION ON
    )

    install(TARGETS rrt_cuda DESTINATION lib/${PROJECT_NAME})
endif()

# Install resource files.
install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME}/resource)

ament_package()
