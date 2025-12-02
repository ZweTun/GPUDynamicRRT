cmake_minimum_required(VERSION 3.16) # CMake version on ROS 2 Foxy
project(dynamic_rrt LANGUAGES CXX)

# Do not use cxx_std_17 because it can propagate to CUDA compilation.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable compiler warnings.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        $<$<COMPILE_LANGUAGE:CXX>:-Wall>
        $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
        $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
    )
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
ament_target_dependencies(rrt_cpu ${core_dependencies})
install(TARGETS rrt_cpu DESTINATION lib/${PROJECT_NAME})

option(DYNAMIC_RRT_BUILD_CUDA "Build the CUDA-accelerated RRT node" ON)
if(DYNAMIC_RRT_BUILD_CUDA)
    # Add the CUDA executable.

    # https://cliutils.gitlab.io/modern-cmake/chapters/packages/CUDA.html
    include(CheckLanguage)
    check_language(CUDA)
    if(NOT CMAKE_CUDA_COMPILER)
        message(FATAL_ERROR "DYNAMIC_RRT_BUILD_CUDA is ON but no CUDA compiler could be found.")
    endif()
    message(STATUS "CUDA compiler found: ${CMAKE_CUDA_COMPILER}")
    enable_language(CUDA)

    # CUDA C++17 is not supported until CMake 3.18.
    set(CMAKE_CUDA_STANDARD 14)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)

    find_package(CUDA REQUIRED) # Use the old FindCUDA module for compatibility with CMake 3.16.

    add_executable(rrt_cuda src/rrt_base.cpp src/rrt_cuda.cpp src/rrt_kernels.cu)
    ament_target_dependencies(rrt_cuda ${core_dependencies})
    target_include_directories(rrt_cuda SYSTEM PRIVATE ${CUDA_INCLUDE_DIRS})
    target_link_libraries(rrt_cuda ${CUDA_CUDART_LIBRARY} ${CUDA_curand_LIBRARY})
    set_target_properties(rrt_cuda PROPERTIES
        CUDA_ARCHITECTURES "72;89" # Jetson Xavier NX and GeForce RTX 4090
        CUDA_SEPARABLE_COMPILATION ON
    )

    install(TARGETS rrt_cuda DESTINATION lib/${PROJECT_NAME})
endif()

# Install resource files.
install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME}/resource)

ament_package()
