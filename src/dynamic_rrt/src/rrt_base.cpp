#include "rrt_base.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <unordered_map>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/wait_for_message.hpp"

namespace dynamic_rrt {

using std::placeholders::_1;

RRTBase::RRTBase(const std::string& node_name)
    : Node(node_name) {
    // Declare parameters.
    this->declare_parameter<bool>("simulation", false);
    this->declare_parameter<std::int64_t>("planning_interval_ms", 100);
    this->declare_parameter<std::int64_t>("waypoint_publish_interval_ms", 1000);
    this->declare_parameter<double>("obstacle_margin", 0.15);
    this->declare_parameter<std::string>("global_waypoint_csv", "");
    this->declare_parameter<double>("global_waypoint_max_distance", 5.0);

    rclcpp::QoS default_qos(10);

    // Create subscriptions.
    if (this->get_parameter("simulation").as_bool()) {
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom",
            default_qos,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                RCLCPP_INFO_ONCE(
                    this->get_logger(),
                    "Received first Odometry message with frame ID: %s",
                    msg->header.frame_id.c_str()
                );
                this->update_pose(msg->pose.pose);
            }
        );
        RCLCPP_INFO(this->get_logger(), "Running in simulation mode");
    } else {
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pf/viz/inferred_pose",
            default_qos,
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                RCLCPP_INFO_ONCE(
                    this->get_logger(),
                    "Received first PoseStamped message with frame ID: %s",
                    msg->header.frame_id.c_str()
                );
                this->update_pose(msg->pose);
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
    this->global_waypoint_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/global_waypoints_markers", default_qos
    );
    this->local_waypoint_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/waypoints_markers", default_qos
    );
    rclcpp::QoS map_qos(1);
    map_qos.transient_local();
    this->map_publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_updated", map_qos);

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

auto RRTBase::map_callback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) -> void {
    if (map_width_ > 0 && map_height_ > 0) {
        return;
    }

    map_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Received initial occupancy grid map");
    RCLCPP_INFO(this->get_logger(), "Map frame ID: %s", map_.header.frame_id.c_str());
    this->set_resolution(map_.info.resolution);
    RCLCPP_INFO(this->get_logger(), "Map resolution: %.3f [m/cell]", map_.info.resolution);
    map_width_ = static_cast<std::int32_t>(map_.info.width);
    map_height_ = static_cast<std::int32_t>(map_.info.height);
    RCLCPP_INFO(
        this->get_logger(),
        "Map size: width=" PRId32 ", height=" PRId32 " [cells]",
        map_width_,
        map_height_
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

    // Log occupancy grid value distribution.
    std::unordered_map<std::int8_t, std::int32_t> value_counts;
    for (const auto cell_value : map_.data) {
        ++value_counts[cell_value];
    }
    std::vector<std::int8_t> sorted_keys;
    sorted_keys.reserve(value_counts.size());
    for (const auto& pair : value_counts) {
        sorted_keys.push_back(pair.first);
    }
    std::sort(sorted_keys.begin(), sorted_keys.end());
    RCLCPP_INFO(this->get_logger(), "Occupancy grid value distribution:");
    for (const auto& key : sorted_keys) {
        const auto count = value_counts[key];
        const auto percentage = count * 100.0 / (map_width_ * map_height_);
        RCLCPP_INFO(
            this->get_logger(),
            "  Value " PRId8 ": count = " PRId32 ", percentage = %.1f%%",
            key,
            count,
            percentage
        );
    }

    obstacle_inflation_radius_ = static_cast<std::int32_t>(
        std::ceil(this->get_parameter("obstacle_margin").as_double() / map_.info.resolution)
    );
    RCLCPP_INFO(
        this->get_logger(),
        "Obstacle inflation radius set to " PRId32 " [cells]",
        obstacle_inflation_radius_
    );

    // Inflate obstacles in the initial occupancy grid.
    const auto original_map_data = map_.data;
    for (std::int32_t y = 0; y < map_height_; ++y) {
        for (std::int32_t x = 0; x < map_width_; ++x) {
            const auto index = y * map_width_ + x;
            if (original_map_data[index] != 0) {
                this->inflate_obstacle(x, y);
            }
        }
    }

    map_publisher_->publish(map_);
}

auto RRTBase::scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) -> void {
    if (map_width_ == 0 || map_height_ == 0 || !pose_) {
        RCLCPP_WARN(
            this->get_logger(),
            "Cannot process LaserScan message before receiving initial map and pose"
        );
        return;
    }

    // TODO: Enable dynamic obstacles once they can expire over time.
    for (std::size_t i = 0; false && i < msg->ranges.size(); ++i) {
        const auto range = msg->ranges[i];
        if (!std::isfinite(range)) {
            continue;
        }
        const auto angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
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

    map_publisher_->publish(map_);
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
    std::int32_t best_index = -1;
    auto best_distance = 0.0;
    for (std::size_t index = 0; index < global_waypoints_.size(); ++index) {
        const auto& waypoint = global_waypoints_[index];
        const auto delta_x = waypoint.x - pose_->position.x;
        const auto delta_y = waypoint.y - pose_->position.y;
        const auto distance = std::hypot(delta_x, delta_y);
        const auto x_car_frame = delta_x * std::cos(-pose_->yaw) - delta_y * std::sin(-pose_->yaw);
        if (distance <= max_distance && x_car_frame > 0.0 && distance > best_distance) {
            best_index = static_cast<std::int32_t>(index);
            best_distance = distance;
        }
    }
    this->publish_global_waypoints(best_index);
    if (best_index < 0) {
        RCLCPP_WARN(this->get_logger(), "No valid global waypoint found within range");
        return;
    }

    // Convert start and goal positions to grid coordinates.
    auto start_pose = *pose_;
    auto goal_position = global_waypoints_[best_index];
    RCLCPP_INFO(
        this->get_logger(),
        "Selected global waypoint at (%.3f, %.3f) as local planning goal",
        goal_position.x,
        goal_position.y
    );
    for (auto position : {&start_pose.position, &goal_position}) {
        position->x = (position->x - map_.info.origin.position.x) / map_.info.resolution;
        position->y = (position->y - map_.info.origin.position.y) / map_.info.resolution;
    }

    const auto grid_waypoints =
        this->plan_rrt(start_pose, goal_position, map_width_, map_height_, map_.data);
    if (grid_waypoints.empty()) {
        return;
    }

    // Convert local waypoints back to world coordinates.
    local_waypoints_.clear();
    local_waypoints_.reserve(grid_waypoints.size());
    for (const auto& waypoint : grid_waypoints) {
        local_waypoints_.push_back(Point2D{
            waypoint.x * map_.info.resolution + map_.info.origin.position.x,
            waypoint.y * map_.info.resolution + map_.info.origin.position.y
        });
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
    local_waypoint_publisher_->publish(std::move(marker_array));
    RCLCPP_INFO(this->get_logger(), "Published %zu local waypoints", local_waypoints_.size());
    local_waypoints_.clear();
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

auto RRTBase::publish_global_waypoints(std::int32_t selected_index) -> void {
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    marker_array->markers.reserve(global_waypoints_.size());
    const auto stamp = this->get_clock()->now();
    for (std::size_t i = 0; i < global_waypoints_.size(); ++i) {
        const auto& waypoint = global_waypoints_[i];
        auto& marker = marker_array->markers.emplace_back();
        marker.header.frame_id = map_.header.frame_id;
        marker.header.stamp = stamp;
        marker.ns = "global_waypoints";
        marker.id = static_cast<std::int32_t>(i);
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = waypoint.x;
        marker.pose.position.y = waypoint.y;
        marker.pose.position.z = 0.1;  // Slightly above the ground for visibility
        if (static_cast<std::int32_t>(i) == selected_index) {
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
        } else {
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
        }
        marker.color.a = 1.0f;
    }
    global_waypoint_publisher_->publish(std::move(marker_array));
    RCLCPP_INFO(this->get_logger(), "Published %zu global waypoints", global_waypoints_.size());
}

}  // namespace dynamic_rrt
