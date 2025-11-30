#include "rrt_base.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <utility>

#include "rclcpp/wait_for_message.hpp"

namespace dynamic_rrt {

using std::placeholders::_1;

RRTBase::RRTBase(const std::string& node_name)
    : Node(node_name) {
    // Declare parameters.
    this->declare_parameter<bool>("simulation", false);
    this->declare_parameter<int>("planning_interval_ms", 100);
    this->declare_parameter<int>("waypoint_publish_interval_ms", 1000);
    this->declare_parameter<double>("obstacle_margin", 0.15);

    rclcpp::QoS default_qos(10);

    // Create subscriptions.
    if (this->get_parameter("simulation").as_bool()) {
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom",
            default_qos,
            [this](const nav_msgs::msg::Odometry& msg) { this->update_pose(msg.pose.pose); }
        );
        RCLCPP_INFO(this->get_logger(), "Running in simulation mode");
    } else {
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pf/viz/inferred_pose",
            default_qos,
            [this](const geometry_msgs::msg::PoseStamped& msg) { this->update_pose(msg.pose); }
        );
        RCLCPP_INFO(this->get_logger(), "Running in real mode");
    }
    this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", default_qos, std::bind(&RRTBase::scan_callback, this, _1)
    );

    // Create publishers.
    this->waypoint_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/waypoints_markers", default_qos
    );

    // Create timers.
    this->planning_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(this->get_parameter("planning_interval_ms").as_int()),
        std::bind(&RRTBase::run_planning, this)
    );
    this->waypoint_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(this->get_parameter("waypoint_publish_interval_ms").as_int()),
        std::bind(&RRTBase::publish_waypoints, this)
    );

    // Initialize the occupancy grid.
    const auto map_topic = "/map";
    const auto success = rclcpp::wait_for_message<nav_msgs::msg::OccupancyGrid>(
        map_, this->shared_from_this(), map_topic
    );
    if (!success) {
        RCLCPP_FATAL(
            this->get_logger(), "Failed to receive initial occupancy grid from topic %s", map_topic
        );
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Received initial occupancy grid from topic %s", map_topic);
    map_height_ = static_cast<std::int32_t>(map_.info.height);
    map_width_ = static_cast<std::int32_t>(map_.info.width);
    RCLCPP_INFO(
        this->get_logger(),
        "Occupancy grid size: width=%d, height=%d, resolution=%.3f",
        map_width_,
        map_height_,
        map_.info.resolution
    );
    obstacle_inflation_radius_ = static_cast<std::int32_t>(
        std::ceil(this->get_parameter("obstacle_margin").as_double() / map_.info.resolution)
    );
    RCLCPP_INFO(
        this->get_logger(), "Obstacle inflation radius set to %d cells", obstacle_inflation_radius_
    );

    // Inflate obstacles in the initial occupancy grid.
    const auto original_map_data = map_.data;
    for (std::int32_t y = 0; y < map_height_; ++y) {
        for (std::int32_t x = 0; x < map_width_; ++x) {
            const auto index = y * map_width_ + x;
            if (original_map_data[index] == OCCUPANCY_GRID_OCCUPIED) {
                this->inflate_obstacle(x, y);
            }
        }
    }
}

auto RRTBase::scan_callback(const sensor_msgs::msg::LaserScan& msg) -> void {
    if (!pose_) {
        return;
    }
    for (std::size_t i = 0; i < msg.ranges.size(); ++i) {
        const auto range = msg.ranges[i];
        if (!std::isfinite(range)) {
            continue;
        }
        const auto angle = msg.angle_min + static_cast<double>(i) * msg.angle_increment;
        const auto obstacle_x = pose_->x + range * std::cos(pose_->yaw + angle);
        const auto obstacle_y = pose_->y + range * std::sin(pose_->yaw + angle);
        const auto grid_x = static_cast<std::int32_t>(
            (obstacle_x - map_.info.origin.position.x) / map_.info.resolution
        );
        const auto grid_y = static_cast<std::int32_t>(
            (obstacle_y - map_.info.origin.position.y) / map_.info.resolution
        );
        this->inflate_obstacle(grid_x, grid_y);
    }
}

auto RRTBase::run_planning() -> void {}

auto RRTBase::publish_waypoints() -> void {
    if (waypoints_.empty()) {
        return;
    }
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    marker_array->markers.reserve(waypoints_.size());
    const auto stamp = this->get_clock()->now();
    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
        const auto& waypoint = waypoints_[i];
        auto& marker = marker_array->markers.emplace_back();
        marker.header.frame_id = "map";
        marker.header.stamp = stamp;
        marker.ns = "waypoints";
        marker.id = static_cast<std::int32_t>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = waypoint.x;
        marker.pose.position.y = waypoint.y;
        marker.pose.position.z = 0.1;  // Slightly above the ground for visibility
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
    }
    waypoint_publisher_->publish(std::move(marker_array));
}

auto RRTBase::update_pose(const geometry_msgs::msg::Pose& pose) -> void {
    auto& position = pose.position;
    auto& orientation = pose.orientation;
    const auto sin_yaw_cos_pitch =
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
    const auto cos_yaw_cos_pitch =
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
    const auto yaw = std::atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch);
    pose_ = Pose2D{position.x, position.y, yaw};
}

auto RRTBase::inflate_obstacle(std::int32_t x, std::int32_t y) -> void {
    const auto y_min = std::clamp(y - obstacle_inflation_radius_, 0, map_height_ - 1);
    const auto y_max = std::clamp(y + obstacle_inflation_radius_, 0, map_height_ - 1);
    const auto x_min = std::clamp(x - obstacle_inflation_radius_, 0, map_width_ - 1);
    const auto x_max = std::clamp(x + obstacle_inflation_radius_, 0, map_width_ - 1);
    for (auto neighbor_y = y_min; neighbor_y <= y_max; ++neighbor_y) {
        for (auto neighbor_x = x_min; neighbor_x <= x_max; ++neighbor_x) {
            const auto delta_x = neighbor_x - x;
            const auto delta_y = neighbor_y - y;
            if (delta_x * delta_x + delta_y * delta_y >
                obstacle_inflation_radius_ * obstacle_inflation_radius_) {
                continue;
            }
            const auto neighbor_index = neighbor_y * map_width_ + neighbor_x;
            map_.data[neighbor_index] = OCCUPANCY_GRID_OCCUPIED;
        }
    }
}

}  // namespace dynamic_rrt
