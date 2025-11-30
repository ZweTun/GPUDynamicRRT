#ifndef DYNAMIC_RRT_RRT_CPU_HPP
#define DYNAMIC_RRT_RRT_CPU_HPP

#include <cstdint>
#include <random>
#include <vector>

#include "rrt_base.hpp"

namespace dynamic_rrt {

struct TreeNode {
    Point2D point;
    std::int32_t parent_index;
};

class RRTCpu final : public RRTBase {
public:
    RRTCpu();

private:
    auto set_resolution(double resolution) -> void override;

    auto plan_rrt(
        const Pose2D& start,
        const Point2D& goal,
        std::int32_t map_width,
        std::int32_t map_height,
        const std::vector<std::int8_t>& map_data,
        std::vector<Point2D>& local_waypoints
    ) -> void override;

    auto sample_point() -> Point2D;

    auto is_point_free(const Point2D& point) const -> bool;

    auto get_nearest_node_index(const Point2D& point) const -> std::int32_t;

    auto steer_towards(const Point2D& nearest_point, const Point2D& sampled_point) const -> Point2D;

    auto is_segment_collision_free(const Point2D& nearest_point, const Point2D& steered_point) const
        -> bool;

    auto is_goal_reached(const Point2D& point) const -> bool;

    auto construct_path(std::int32_t goal_index) -> void;

    std::mt19937 rng_;

    // Algorithm parameters
    std::int64_t max_iterations_ = 0;
    std::int64_t max_sampling_attempts_ = 0;

    // Computed parameters
    double sample_forward_min_cells_ = 0.0;
    double sample_forward_max_cells_ = 0.0;
    double sample_lateral_range_cells_ = 0.0;
    double fallback_forward_min_cells_ = 0.0;
    double fallback_forward_max_cells_ = 0.0;
    double steer_step_cells_ = 0.0;
    double goal_threshold_cells_ = 0.0;

    // Planning states
    Pose2D start_{};
    Point2D goal_{};
    std::int32_t map_width_ = 0;
    std::int32_t map_height_ = 0;
    const std::vector<std::int8_t>* map_data_ = nullptr;
    std::vector<Point2D>* local_waypoints_ = nullptr;
    std::vector<TreeNode> tree_;
};

}  // namespace dynamic_rrt

#endif  // DYNAMIC_RRT_RRT_CPU_HPP
