#ifndef DYNAMIC_RRT_RRT_CUDA_HPP
#define DYNAMIC_RRT_RRT_CUDA_HPP

#include <cstdint>
#include <random>
#include <vector>

#include "rrt_base.hpp"

namespace dynamic_rrt {

class RRTCuda final : public RRTBase {
public:
    RRTCuda();

private:
    auto set_resolution(float resolution) -> void override;

    auto plan_rrt(
        const Pose2D& start,
        const Point2D& goal,
        std::int32_t map_width,
        std::int32_t map_height,
        const std::vector<std::int8_t>& map_data
    ) -> std::vector<Point2D> override;

    std::mt19937 random_engine_;

    // Algorithm parameters
    std::int32_t num_workers_ = 0;
    std::int32_t max_iterations_ = 0;
    std::int32_t max_nodes_per_tree_ = 0;
    std::int32_t max_sampling_attempts_ = 0;
    std::int32_t threads_per_block_ = 0;

    // Computed parameters
    float sample_forward_min_cells_ = 0.0f;
    float sample_forward_max_cells_ = 0.0f;
    float sample_lateral_range_cells_ = 0.0f;
    float sample_fallback_forward_min_cells_ = 0.0f;
    float sample_fallback_forward_max_cells_ = 0.0f;
    float steer_step_size_cells_ = 0.0f;
    float goal_tolerance_cells_ = 0.0f;
};

}  // namespace dynamic_rrt

#endif  // DYNAMIC_RRT_RRT_CUDA_HPP
