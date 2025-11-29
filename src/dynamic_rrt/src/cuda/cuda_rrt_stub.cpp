#include "cuda_rrt.hpp"

#include <stdexcept>

namespace dynamic_rrt {
namespace cuda {

CudaPlanResult runCudaRrt(const nav_msgs::msg::OccupancyGrid &,
                          const TreeNode &,
                          const std::array<double, 2> &,
                          int,
                          double,
                          double) {
    throw std::runtime_error("CUDA support disabled at compile time");
}

} // namespace cuda
} // namespace dynamic_rrt
