#ifndef DYNAMIC_RRT_RRT_KERNELS_HPP
#define DYNAMIC_RRT_RRT_KERNELS_HPP

#include <cstdint>
#include <vector>

#include "rrt_common.hpp"

namespace dynamic_rrt {

auto plan_rrt_cuda(
    const RRTStateBase& state, std::int32_t threads_per_block, unsigned long long seed
) -> std::vector<Point2D>;

}  // namespace dynamic_rrt

#endif  // DYNAMIC_RRT_RRT_KERNELS_HPP
