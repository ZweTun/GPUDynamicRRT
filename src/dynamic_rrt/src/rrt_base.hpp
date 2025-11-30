#ifndef DYNAMIC_RRT_RRT_BASE_HPP
#define DYNAMIC_RRT_RRT_BASE_HPP

#include <cstdint>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace dynamic_rrt {

struct Pose2D {
    double x;
    double y;
    double yaw;
};

struct Waypoint {
    double x;
    double y;
};

class RRTBase : public rclcpp::Node {
public:
    RRTBase(const std::string& node_name);
    ~RRTBase() override = default;

    RRTBase(const RRTBase&) = delete;
    RRTBase(RRTBase&&) = delete;
    auto operator=(const RRTBase&) -> RRTBase& = delete;
    auto operator=(RRTBase&&) -> RRTBase& = delete;

private:
    auto scan_callback(const sensor_msgs::msg::LaserScan& msg) -> void;

    auto run_planning() -> void;
    auto publish_waypoints() -> void;

    auto update_pose(const geometry_msgs::msg::Pose& pose) -> void;
    auto inflate_obstacle(std::int32_t x, std::int32_t y) -> void;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_publisher_;

    // Timers
    rclcpp::TimerBase::SharedPtr planning_timer_;
    rclcpp::TimerBase::SharedPtr waypoint_timer_;

    // Constants
    static constexpr std::int8_t OCCUPANCY_GRID_OCCUPIED = 100;

    // Computed properties
    std::int32_t map_height_ = 0;
    std::int32_t map_width_ = 0;
    std::int32_t obstacle_inflation_radius_ = 0;

    // States
    nav_msgs::msg::OccupancyGrid map_;
    std::optional<Pose2D> pose_;
    std::vector<Waypoint> waypoints_;
};

}  // namespace dynamic_rrt

#endif  // DYNAMIC_RRT_RRT_BASE_HPP
