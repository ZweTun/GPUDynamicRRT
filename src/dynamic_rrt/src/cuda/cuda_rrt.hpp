#ifndef DYNAMIC_RRT_CUDA_RRT_HPP
#define DYNAMIC_RRT_CUDA_RRT_HPP

#include <array>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "../rrt_node.hpp"

namespace dynamic_rrt {
namespace cuda {

struct CudaPlanResult {
    std::vector<TreeNode> tree;
    std::vector<TreeNode> path;
};

CudaPlanResult runCudaRrt(const nav_msgs::msg::OccupancyGrid &map,
                          const TreeNode &start,
                          const std::array<double, 2> &goal,
                          int max_iterations,
                          double max_step,
                          double goal_threshold);

} // namespace cuda
} // namespace dynamic_rrt

#endif // DYNAMIC_RRT_CUDA_RRT_HPP
