#include "rrt_cpu.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace dynamic_rrt {

RRTCpu::RRTCpu()
    : RRTBase("dynamic_rrt_cpu"),
      rng_(std::random_device()()) {
    // Declare parameters.
    this->declare_parameter<int>("max_iterations", 2500);
    max_iterations_ = this->get_parameter("max_iterations").as_int();
    this->declare_parameter<int>("max_sampling_attempts", 100);
    max_sampling_attempts_ = this->get_parameter("max_sampling_attempts").as_int();

    this->declare_parameter<double>("sample_forward_min_m", 0.7);
    this->declare_parameter<double>("sample_forward_max_m", 5.0);
    this->declare_parameter<double>("sample_lateral_range_m", 5.0);
    this->declare_parameter<double>("fallback_forward_min_m", 0.3);
    this->declare_parameter<double>("fallback_forward_max_m", 1.0);
    this->declare_parameter<double>("steer_step_m", 0.5);
    this->declare_parameter<double>("goal_threshold_m", 0.15);
}

auto RRTCpu::set_resolution(double resolution) -> void {
    sample_forward_min_cells_ =
        this->get_parameter("sample_forward_min_m").as_double() / resolution;
    sample_forward_max_cells_ =
        this->get_parameter("sample_forward_max_m").as_double() / resolution;
    sample_lateral_range_cells_ =
        this->get_parameter("sample_lateral_range_m").as_double() / resolution;
    fallback_forward_min_cells_ =
        this->get_parameter("fallback_forward_min_m").as_double() / resolution;
    fallback_forward_max_cells_ =
        this->get_parameter("fallback_forward_max_m").as_double() / resolution;
    steer_step_cells_ = this->get_parameter("steer_step_m").as_double() / resolution;
    goal_threshold_cells_ = this->get_parameter("goal_threshold_m").as_double() / resolution;
}

auto RRTCpu::plan_rrt(
    const Pose2D& start,
    const Point2D& goal,
    std::int32_t map_width,
    std::int32_t map_height,
    const std::vector<std::int8_t>& map_data
) -> std::vector<Point2D> {
    start_ = start;
    goal_ = goal;
    map_width_ = map_width;
    map_height_ = map_height;
    map_data_ = &map_data;

    tree_.clear();
    tree_.reserve(1 + max_iterations_);
    tree_.push_back(TreeNode{start_.position, -1});

    for (std::int64_t i = 0; i < max_iterations_; ++i) {
        const auto sampled_point = this->sample_point();
        const auto nearest_index = this->get_nearest_node_index(sampled_point);
        const auto& nearest_point = tree_[nearest_index].point;
        const auto steered_point = this->steer_towards(nearest_point, sampled_point);
        if (!this->is_segment_collision_free(nearest_point, steered_point)) {
            continue;
        }
        tree_.push_back(TreeNode{steered_point, nearest_index});
        if (!this->is_goal_reached(steered_point)) {
            continue;
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Path found after %ld iterations (tree size: %zu)",
            i + 1,
            tree_.size()
        );
        const auto goal_index = static_cast<std::int32_t>(tree_.size() - 1);
        return this->construct_path(goal_index);
    }

    RCLCPP_WARN(
        this->get_logger(),
        "Failed to find a path within %ld iterations (tree size: %zu)",
        max_iterations_,
        tree_.size()
    );
    return std::vector<Point2D>{};
}

auto RRTCpu::sample_point() -> Point2D {
    const auto cos_yaw = std::cos(start_.yaw);
    const auto sin_yaw = std::sin(start_.yaw);

    std::uniform_real_distribution forward_dist(
        sample_forward_min_cells_, sample_forward_max_cells_
    );
    std::uniform_real_distribution lateral_dist(
        -sample_lateral_range_cells_, sample_lateral_range_cells_
    );
    for (std::int64_t i = 0; i < max_sampling_attempts_; ++i) {
        const auto forward = forward_dist(rng_);
        const auto lateral = lateral_dist(rng_);
        const Point2D point{
            start_.position.x + forward * cos_yaw - lateral * sin_yaw,
            start_.position.y + forward * sin_yaw + lateral * cos_yaw
        };
        if (this->is_point_free(point)) {
            return point;
        }
    }
    // If we couldn't find a valid point, use a shorter segment straight ahead without checking for
    // obstacles.
    std::uniform_real_distribution fallback_dist(
        fallback_forward_min_cells_, fallback_forward_max_cells_
    );
    const auto forward = fallback_dist(rng_);
    return Point2D{start_.position.x + forward * cos_yaw, start_.position.y + forward * sin_yaw};
}

auto RRTCpu::is_point_free(const Point2D& point) const -> bool {
    const auto grid_x = static_cast<std::int32_t>(std::floor(point.x));
    const auto grid_y = static_cast<std::int32_t>(std::floor(point.y));
    if (grid_x < 0 || grid_x >= map_width_ || grid_y < 0 || grid_y >= map_height_) {
        return false;
    }
    const auto index = grid_y * map_width_ + grid_x;
    return (*map_data_)[index] == 0;
}

auto RRTCpu::get_nearest_node_index(const Point2D& point) const -> std::int32_t {
    std::int32_t nearest_index = -1;
    auto nearest_distance_squared = std::numeric_limits<double>::max();
    for (std::int32_t index = 0; index < static_cast<std::int32_t>(tree_.size()); ++index) {
        auto& node = tree_[index];
        const auto delta_x = point.x - node.point.x;
        const auto delta_y = point.y - node.point.y;
        const auto distance_squared = delta_x * delta_x + delta_y * delta_y;
        if (distance_squared < nearest_distance_squared) {
            nearest_distance_squared = distance_squared;
            nearest_index = index;
        }
    }
    return nearest_index;
}

auto RRTCpu::steer_towards(const Point2D& nearest_point, const Point2D& sampled_point) const
    -> Point2D {
    const auto delta_x = sampled_point.x - nearest_point.x;
    const auto delta_y = sampled_point.y - nearest_point.y;
    const auto distance = std::hypot(delta_x, delta_y);
    if (distance <= steer_step_cells_) {
        return sampled_point;
    }
    const auto scale = steer_step_cells_ / distance;
    return Point2D{nearest_point.x + delta_x * scale, nearest_point.y + delta_y * scale};
}

auto RRTCpu::is_segment_collision_free(const Point2D& nearest_point, const Point2D& steered_point)
    const -> bool {
    const auto num_steps = std::max(
        static_cast<std::int32_t>(std::ceil(std::abs(steered_point.x - nearest_point.x))),
        static_cast<std::int32_t>(std::ceil(std::abs(steered_point.y - nearest_point.y)))
    );
    if (num_steps == 0) {
        return this->is_point_free(steered_point);
    }
    for (std::int32_t i = 0; i <= num_steps; ++i) {
        const auto t = static_cast<double>(i) / static_cast<double>(num_steps);
        const auto point = Point2D{
            (1.0 - t) * nearest_point.x + t * steered_point.x,
            (1.0 - t) * nearest_point.y + t * steered_point.y
        };
        if (!this->is_point_free(point)) {
            return false;
        }
    }
    return true;
}

auto RRTCpu::is_goal_reached(const Point2D& point) const -> bool {
    const auto delta_x = point.x - goal_.x;
    const auto delta_y = point.y - goal_.y;
    const auto distance = std::hypot(delta_x, delta_y);
    return distance <= goal_threshold_cells_;
}

auto RRTCpu::construct_path(std::int32_t goal_index) -> std::vector<Point2D> {
    std::vector<Point2D> path;
    auto index = goal_index;
    while (index >= 0) {
        const auto& node = tree_[index];
        path.push_back(node.point);
        index = node.parent_index;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

}  // namespace dynamic_rrt

auto main(int argc, char* argv[]) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dynamic_rrt::RRTCpu>());
    rclcpp::shutdown();
    return 0;
}
