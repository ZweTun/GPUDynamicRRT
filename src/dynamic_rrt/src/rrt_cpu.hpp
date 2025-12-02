#ifndef DYNAMIC_RRT_RRT_CPU_HPP
#define DYNAMIC_RRT_RRT_CPU_HPP

#include <cstdint>
#include <random>
#include <vector>

#include "rrt_base.hpp"
#include "rrt_common.hpp"

namespace dynamic_rrt {

struct RRTStateCpu final : RRTStateBase {
    std::mt19937* random_engines;

    auto sample_uniform_real(float a, float b, unsigned int worker_index) const -> float;
};

class RRTCpu final : public RRTBase {
public:
    RRTCpu();

private:
    auto set_resolution(float resolution) -> void override;

    auto plan_rrt(
        const Pose2D& start,
        const Point2D& goal,
        std::int32_t map_width,
        std::int32_t map_height,
        const std::vector<std::int8_t>& map_data
    ) -> std::vector<Point2D> override;

    auto sample_point() -> Point2D;

    auto is_point_free(const Point2D& point) const -> bool;

    auto get_nearest_node_index(const Point2D& point) const -> std::int32_t;

    auto steer_towards(const Point2D& nearest_point, const Point2D& sampled_point) const -> Point2D;

    auto is_segment_collision_free(const Point2D& nearest_point, const Point2D& steered_point) const
        -> bool;

    auto is_goal_reached(const Point2D& point) const -> bool;

    auto construct_path(std::int32_t goal_index) -> std::vector<Point2D>;

    std::mt19937 random_engine_;

    // Algorithm parameters
    std::int32_t num_workers_ = 0;
    std::int32_t max_iterations_ = 0;
    std::int32_t max_nodes_per_tree_ = 0;
    std::int32_t max_sampling_attempts_ = 0;

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

#endif  // DYNAMIC_RRT_RRT_CPU_HPP
