#include "rrt_base.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
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
    this->declare_parameter<std::string>("global_waypoint_csv", "");
    this->declare_parameter<double>("global_waypoint_max_distance", 5.0);

    rclcpp::QoS default_qos(10);

    // Create subscriptions.
    if (this->get_parameter("simulation").as_bool()) {
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom",
            default_qos,
            [this](const nav_msgs::msg::Odometry& msg) {
                RCLCPP_INFO_ONCE(
                    this->get_logger(),
                    "Received first Odometry message with frame ID: %s",
                    msg.header.frame_id.c_str()
                );
                this->update_pose(msg.pose.pose);
            }
        );
        RCLCPP_INFO(this->get_logger(), "Running in simulation mode");
    } else {
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pf/viz/inferred_pose",
            default_qos,
            [this](const geometry_msgs::msg::PoseStamped& msg) {
                RCLCPP_INFO_ONCE(
                    this->get_logger(),
                    "Received first PoseStamped message with frame ID: %s",
                    msg.header.frame_id.c_str()
                );
                this->update_pose(msg.pose);
            }
        );
        RCLCPP_INFO(this->get_logger(), "Running in real mode");
    }
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", default_qos, std::bind(&RRTBase::map_callback, this, _1)
    );
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
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
        std::bind(&RRTBase::publish_local_waypoints, this)
    );

    // Load global waypoints from CSV.
    if (!this->load_global_waypoints()) {
        rclcpp::shutdown();
        return;
    }
}

auto RRTBase::map_callback(const nav_msgs::msg::OccupancyGrid& msg) -> void {
    if (map_width_ > 0 && map_height_ > 0) {
        return;
    }

    map_ = msg;
    RCLCPP_INFO(this->get_logger(), "Received initial occupancy grid map");
    RCLCPP_INFO(this->get_logger(), "Map frame ID: %s", map_.header.frame_id.c_str());
    this->set_resolution(map_.info.resolution);
    RCLCPP_INFO(this->get_logger(), "Map resolution: %.3f [m/cell]", map_.info.resolution);
    map_width_ = static_cast<std::int32_t>(map_.info.width);
    map_height_ = static_cast<std::int32_t>(map_.info.height);
    RCLCPP_INFO(
        this->get_logger(), "Map size: width=%d, height=%d [cells]", map_width_, map_height_
    );
    const auto& position = map_.info.origin.position;
    RCLCPP_INFO(
        this->get_logger(),
        "Map origin position: x=%.3f, y=%.3f, z=%.3f [m]",
        position.x,
        position.y,
        position.z
    );
    const auto& orientation = map_.info.origin.orientation;
    RCLCPP_INFO(
        this->get_logger(),
        "Map origin orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    );

    obstacle_inflation_radius_ = static_cast<std::int32_t>(
        std::ceil(this->get_parameter("obstacle_margin").as_double() / map_.info.resolution)
    );
    RCLCPP_INFO(
        this->get_logger(),
        "Obstacle inflation radius set to %d [cells]",
        obstacle_inflation_radius_
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
    if (map_width_ == 0 || map_height_ == 0 || !pose_) {
        RCLCPP_WARN(
            this->get_logger(),
            "Cannot process LaserScan message before receiving initial map and pose"
        );
        return;
    }

    for (std::size_t i = 0; i < msg.ranges.size(); ++i) {
        const auto range = msg.ranges[i];
        if (!std::isfinite(range)) {
            continue;
        }
        const auto angle = msg.angle_min + static_cast<double>(i) * msg.angle_increment;
        const auto obstacle_x = pose_->position.x + range * std::cos(pose_->yaw + angle);
        const auto obstacle_y = pose_->position.y + range * std::sin(pose_->yaw + angle);
        const auto grid_x = static_cast<std::int32_t>(
            (obstacle_x - map_.info.origin.position.x) / map_.info.resolution
        );
        const auto grid_y = static_cast<std::int32_t>(
            (obstacle_y - map_.info.origin.position.y) / map_.info.resolution
        );
        this->inflate_obstacle(grid_x, grid_y);
    }
}

auto RRTBase::run_planning() -> void {
    if (map_width_ == 0 || map_height_ == 0 || !pose_) {
        RCLCPP_WARN(
            this->get_logger(), "Cannot run planning before receiving initial map and pose"
        );
        return;
    }
    local_waypoints_.clear();

    // Select a global waypoint as the local planning goal.
    const auto max_distance = this->get_parameter("global_waypoint_max_distance").as_double();
    std::optional<Point2D> best_waypoint;
    auto best_distance = 0.0;
    for (const auto& waypoint : global_waypoints_) {
        const auto delta_x = waypoint.x - pose_->position.x;
        const auto delta_y = waypoint.y - pose_->position.y;
        const auto distance = std::hypot(delta_x, delta_y);
        const auto x_car_frame = delta_x * std::cos(-pose_->yaw) - delta_y * std::sin(-pose_->yaw);
        if (distance <= max_distance && x_car_frame > 0.0 && distance > best_distance) {
            best_waypoint = waypoint;
            best_distance = distance;
        }
    }

    if (!best_waypoint) {
        RCLCPP_WARN(this->get_logger(), "No valid global waypoint found within range");
        return;
    }
    RCLCPP_INFO(
        this->get_logger(),
        "Selected global waypoint at (%.2f, %.2f) as local planning goal",
        best_waypoint->x,
        best_waypoint->y
    );

    // Convert start and goal positions to grid coordinates.
    auto start_pose = *pose_;
    auto goal_position = *best_waypoint;
    for (auto position : {&start_pose.position, &goal_position}) {
        position->x = (position->x - map_.info.origin.position.x) / map_.info.resolution;
        position->y = (position->y - map_.info.origin.position.y) / map_.info.resolution;
    }

    this->plan_rrt(start_pose, goal_position, map_width_, map_height_, map_.data, local_waypoints_);

    // Convert local waypoints back to world coordinates.
    for (auto& waypoint : local_waypoints_) {
        waypoint.x = waypoint.x * map_.info.resolution + map_.info.origin.position.x;
        waypoint.y = waypoint.y * map_.info.resolution + map_.info.origin.position.y;
    }
}

auto RRTBase::publish_local_waypoints() -> void {
    if (local_waypoints_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No local waypoints to publish");
        return;
    }

    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    marker_array->markers.reserve(local_waypoints_.size());
    const auto stamp = this->get_clock()->now();
    for (std::size_t i = 0; i < local_waypoints_.size(); ++i) {
        const auto& waypoint = local_waypoints_[i];
        auto& marker = marker_array->markers.emplace_back();
        marker.header.frame_id = map_.header.frame_id;
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
    RCLCPP_INFO(this->get_logger(), "Published %zu local waypoints", local_waypoints_.size());
}

auto RRTBase::load_global_waypoints() -> bool {
    auto csv_path = this->get_parameter("global_waypoint_csv").as_string();
    if (csv_path.empty()) {
        csv_path = ament_index_cpp::get_package_share_directory("dynamic_rrt");
        csv_path += "/resource/levine1.csv";
        RCLCPP_INFO(
            this->get_logger(),
            "No global waypoint CSV specified, using default path: %s",
            csv_path.c_str()
        );
    } else {
        RCLCPP_INFO(
            this->get_logger(),
            "Loading global waypoints from specified CSV path: %s",
            csv_path.c_str()
        );
    }

    std::ifstream stream(csv_path);
    if (!stream) {
        RCLCPP_FATAL(this->get_logger(), "Failed to open global waypoint CSV file");
        return false;
    }

    std::string line;
    while (std::getline(stream, line)) {
        if (line.empty()) {
            continue;
        }
        std::istringstream line_stream(line);
        std::string x_str, y_str;
        if (!std::getline(line_stream, x_str, ',') || !std::getline(line_stream, y_str, ',')) {
            RCLCPP_WARN(this->get_logger(), "Invalid line in CSV, skipping: %s", line.c_str());
            continue;
        }
        try {
            Point2D waypoint{std::stod(x_str), std::stod(y_str)};
            global_waypoints_.push_back(waypoint);
        } catch (const std::exception&) {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to parse coordinates from line, skipping: %s",
                line.c_str()
            );
        }
    }

    if (global_waypoints_.empty()) {
        RCLCPP_FATAL(this->get_logger(), "No valid global waypoints loaded from CSV");
        return false;
    }

    RCLCPP_INFO(
        this->get_logger(), "Loaded %zu global waypoints from CSV", global_waypoints_.size()
    );
    return true;
}

auto RRTBase::update_pose(const geometry_msgs::msg::Pose& pose) -> void {
    auto& position = pose.position;
    auto& orientation = pose.orientation;
    const auto sin_yaw_cos_pitch =
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
    const auto cos_yaw_cos_pitch =
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
    const auto yaw = std::atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch);
    pose_ = Pose2D{Point2D{position.x, position.y}, yaw};
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
