#ifndef DYNAMIC_RRT_RRT_BASE_HPP
#define DYNAMIC_RRT_RRT_BASE_HPP

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rrt_common.hpp"

namespace dynamic_rrt {

class RRTBase : public rclcpp::Node {
public:
    RRTBase(const std::string& node_name);
    ~RRTBase() override = default;

    RRTBase(const RRTBase&) = delete;
    RRTBase(RRTBase&&) = delete;
    auto operator=(const RRTBase&) -> RRTBase& = delete;
    auto operator=(RRTBase&&) -> RRTBase& = delete;

protected:
    virtual auto set_resolution(float resolution) -> void = 0;

    virtual auto plan_rrt(
        const Pose2D& start,
        const Point2D& goal,
        std::int32_t map_width,
        std::int32_t map_height,
        const std::vector<std::int8_t>& map_data
    ) -> std::vector<Point2D> = 0;

private:
    auto map_callback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) -> void;
    auto scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) -> void;

    auto run_planning() -> void;
    auto publish_local_waypoints() -> void;

    auto load_global_waypoints() -> bool;
    auto update_pose(const geometry_msgs::msg::Pose& pose) -> void;
    auto inflate_obstacle(std::int32_t x, std::int32_t y, rclcpp::Time time) -> void;
    auto publish_global_waypoints(std::int32_t selected_index) -> void;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_waypoint_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_waypoint_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

    // Timers
    rclcpp::TimerBase::SharedPtr planning_timer_;
    rclcpp::TimerBase::SharedPtr waypoint_timer_;

    // Constants
    static constexpr std::int8_t OCCUPANCY_GRID_OCCUPIED = 100;

    // Computed properties
    std::int32_t map_width_ = 0;
    std::int32_t map_height_ = 0;
    std::int32_t obstacle_inflation_radius_ = 0;
    std::vector<Point2D> global_waypoints_;

    // States
    nav_msgs::msg::OccupancyGrid map_;
    std::vector<rclcpp::Time> obstacle_timestamps_;
    std::optional<Pose2D> pose_;
    std::vector<Point2D> local_waypoints_;
};

}  // namespace dynamic_rrt

#endif  // DYNAMIC_RRT_RRT_BASE_HPP
