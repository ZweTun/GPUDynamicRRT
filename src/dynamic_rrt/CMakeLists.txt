cmake_minimum_required(VERSION 3.8)
project(dynamic_rrt)

option(DYNAMIC_RRT_BUILD_CUDA "Build the CUDA-accelerated RRT node" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(ackermann_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(rrt_node src/rrt_node.cpp)
ament_target_dependencies(rrt_node
  ackermann_msgs
  ament_index_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)

if(DYNAMIC_RRT_BUILD_CUDA)
  include(CheckLanguage)
  check_language(CUDA)

  set(dynamic_rrt_can_build_cuda FALSE)

  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    find_package(CUDAToolkit QUIET)
    if(CUDAToolkit_FOUND)
      set(dynamic_rrt_can_build_cuda TRUE)
    else()
      message(WARNING "CUDA Toolkit not found; building rrt_node_cuda with CPU fallback kernels.")
    endif()
  else()
    message(WARNING "CUDA compiler not available; building rrt_node_cuda with CPU fallback kernels.")
  endif()

  add_executable(rrt_node_cuda src/rrt_node_cuda.cpp)
  ament_target_dependencies(rrt_node_cuda
    ackermann_msgs
    ament_index_cpp
    geometry_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    tf2_ros
    visualization_msgs
  )
  target_compile_features(rrt_node_cuda PUBLIC cxx_std_14)

  if(dynamic_rrt_can_build_cuda)
    target_sources(rrt_node_cuda PRIVATE src/cuda/cuda_rrt_kernels.cu)
    set_target_properties(rrt_node_cuda PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
    target_link_libraries(rrt_node_cuda CUDA::cudart)
    target_compile_definitions(rrt_node_cuda PRIVATE DYNAMIC_RRT_HAS_CUDA=1)
  else()
    target_sources(rrt_node_cuda PRIVATE src/cuda/cuda_rrt_stub.cpp)
    target_compile_definitions(rrt_node_cuda PRIVATE DYNAMIC_RRT_HAS_CUDA=0)
  endif()
endif()

install(TARGETS rrt_node DESTINATION lib/${PROJECT_NAME})
if(TARGET rrt_node_cuda)
  install(TARGETS rrt_node_cuda DESTINATION lib/${PROJECT_NAME})
endif()
install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME}/resource)

ament_package()
