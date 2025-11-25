#include "lab7_pkg_cpp/rrt_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

namespace lab7_pkg_cpp {

RRTNode::RRTNode()
    : Node("rrt_node_cpp"),
      rng_(std::random_device{}()),
      forward_dist_(sample_forward_min_, sample_forward_max_),
      lateral_dist_(-sample_lateral_range_, sample_lateral_range_),
      fallback_forward_dist_(0.3, 1.0) {
  sim_mode_ = this->declare_parameter<bool>("sim", false);
  const std::string pose_topic_param = this->declare_parameter<std::string>(
    "pose_topic", "/pf/viz/inferred_pose");
  const std::string odom_topic_param = this->declare_parameter<std::string>(
    "odom_topic", "/ego_racecar/odom");
  const int declared_max_iterations = this->declare_parameter<int>(
    "max_iterations", max_iterations_);
  max_iterations_ = std::max(1, declared_max_iterations);
  goal_lookahead_distance_ = this->declare_parameter<double>(
    "goal_lookahead_distance", goal_lookahead_distance_);
  goal_bias_probability_ = this->declare_parameter<double>(
    "goal_bias_probability", goal_bias_probability_);
  goal_bias_probability_ = std::clamp(goal_bias_probability_, 0.0, 1.0);
  const int declared_goal_bias_start_iteration = this->declare_parameter<int>(
    "goal_bias_start_iteration", goal_bias_start_iteration_);
  goal_bias_start_iteration_ = std::max(0, declared_goal_bias_start_iteration);
  goal_bias_longitudinal_stddev_ = std::max(0.01, this->declare_parameter<double>(
    "goal_bias_longitudinal_stddev", goal_bias_longitudinal_stddev_));
  goal_bias_lateral_stddev_ = std::max(0.01, this->declare_parameter<double>(
    "goal_bias_lateral_stddev", goal_bias_lateral_stddev_));
  sample_forward_min_ = std::max(0.01, this->declare_parameter<double>(
    "sample_forward_min", sample_forward_min_));
  sample_forward_max_ = this->declare_parameter<double>(
    "sample_forward_max", sample_forward_max_);
  if (sample_forward_max_ <= sample_forward_min_ + 0.05) {
    sample_forward_max_ = sample_forward_min_ + 0.05;
  }
  sample_lateral_range_ = std::max(0.05, this->declare_parameter<double>(
    "sample_lateral_range", sample_lateral_range_));

  forward_dist_ = std::uniform_real_distribution<double>(sample_forward_min_, sample_forward_max_);
  lateral_dist_ = std::uniform_real_distribution<double>(-sample_lateral_range_, sample_lateral_range_);

  tree_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/rrt_tree", 10);
  path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/rrt_path", 10);
  waypoint_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints_markers", 10);
  sample_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/sample_points", 10);
  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos.durability(rclcpp::DurabilityPolicy::Volatile);

  if (sim_mode_) {
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_param, qos,
    std::bind(&RRTNode::odomCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Running in sim mode; subscribing to Odometry on %s",
        odom_topic_param.c_str());
  } else {
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_param, qos,
    std::bind(&RRTNode::poseCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Running in real mode; subscribing to PoseStamped on %s",
        pose_topic_param.c_str());
  }

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", qos,
      std::bind(&RRTNode::scanCallback, this, std::placeholders::_1));

  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  map_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  map_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos,
      std::bind(&RRTNode::mapCallback, this, std::placeholders::_1));

  planning_timer_ = this->create_wall_timer(
      100ms, std::bind(&RRTNode::planningTimerCallback, this));

  if (!loadWaypoints()) {
    RCLCPP_WARN(this->get_logger(), "Failed to load waypoints; planner will stay idle.");
  }

  publishWaypointsMarkers();
  publishSampleMarkers();
}

void RRTNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  const auto &position = msg->pose.position;
  const auto &orientation = msg->pose.orientation;
  Pose2D pose;
  pose.x = position.x;
  pose.y = position.y;
  pose.yaw = yawFromQuaternion(orientation.x, orientation.y, orientation.z, orientation.w);
  current_pose_ = pose;
}

void RRTNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto &position = msg->pose.pose.position;
  const auto &orientation = msg->pose.pose.orientation;
  Pose2D pose;
  pose.x = position.x;
  pose.y = position.y;
  pose.yaw = yawFromQuaternion(orientation.x, orientation.y, orientation.z, orientation.w);
  current_pose_ = pose;
}

void RRTNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!map_ready_ || !current_pose_) {
    return;
  }

  auto &data = map_inflated_.data;
  const auto &info = map_inflated_.info;
  if (data.empty() || info.resolution <= 0.0) {
    return;
  }

  const double car_x = current_pose_->x;
  const double car_y = current_pose_->y;
  const double car_yaw = current_pose_->yaw;

  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  const double resolution = info.resolution;
  const int width = static_cast<int>(info.width);
  const int height = static_cast<int>(info.height);

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const double range = msg->ranges[i];
    if (!std::isfinite(range)) {
      continue;
    }

    const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    const double world_x = car_x + range * std::cos(car_yaw + angle);
    const double world_y = car_y + range * std::sin(car_yaw + angle);

    const int grid_x = static_cast<int>((world_x - origin_x) / resolution);
    const int grid_y = static_cast<int>((world_y - origin_y) / resolution);

    if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
      continue;
    }

    const size_t index = static_cast<size_t>(grid_y) * width + static_cast<size_t>(grid_x);
    data[index] = 100;
  }
}

void RRTNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_original_ = *msg;
  map_inflated_ = *msg;
  map_frame_ = msg->header.frame_id.empty() ? std::string("map") : msg->header.frame_id;
  inflateMap(map_inflated_);
  map_ready_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "Occupancy grid received and inflated.");
}

void RRTNode::planningTimerCallback() {
  if (!map_ready_ || !current_pose_ || waypoints_.empty()) {
    return;
  }

  const auto goal_opt = selectGoal();
  if (!goal_opt) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "No forward waypoint available.");
    return;
  }

  const double start_x = current_pose_->x;
  const double start_y = current_pose_->y;
  const double goal_x = (*goal_opt)[0];
  const double goal_y = (*goal_opt)[1];

  std::vector<TreeNode> tree;
  tree.reserve(static_cast<size_t>(max_iterations_) + 1);
  tree.push_back(TreeNode{start_x, start_y, -1});

  std::vector<TreeNode> path;
  bool found = false;

  for (int i = 0; i < max_iterations_; ++i) {
  const auto sample_opt = sample(goal_opt, i);
    if (!sample_opt) {
      continue;
    }

    const int nearest_index = nearest(tree, *sample_opt);
    TreeNode new_node = steer(tree[nearest_index], *sample_opt);
    new_node.parent = nearest_index;

    if (checkCollision(tree[nearest_index], new_node)) {
      continue;
    }

    tree.push_back(new_node);
    if (isGoal(new_node, goal_x, goal_y)) {
      path = buildPath(tree, static_cast<int>(tree.size()) - 1);
      RCLCPP_INFO(this->get_logger(), "RRT path found after %d iterations.", i + 1);
      found = true;
      break;
    }
  }

  if (!found) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "RRT failed to find a path after %d iterations.", max_iterations_);
  } else {
    path_points_.clear();
    path_points_.reserve(path.size());
    for (const auto &node : path) {
      path_points_.push_back({node.x, node.y});
    }
  }

  visualize(tree, found ? &path : nullptr);
  publishWaypointsMarkers();
}

std::optional<std::array<double, 2>> RRTNode::sample(
    const std::optional<std::array<double, 2>> &goal, int iteration) {
  if (!current_pose_) {
    return std::nullopt;
  }

  const double car_x = current_pose_->x;
  const double car_y = current_pose_->y;
  const double car_yaw = current_pose_->yaw;

  constexpr int max_attempts = 100;
  std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
  std::normal_distribution<double> goal_longitudinal(0.0, goal_bias_longitudinal_stddev_);
  std::normal_distribution<double> goal_lateral(0.0, goal_bias_lateral_stddev_);

  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    const bool try_goal_bias = goal && iteration >= goal_bias_start_iteration_ &&
                               goal_bias_probability_ > 0.0 &&
                               unit_dist(rng_) < goal_bias_probability_;

    double x = 0.0;
    double y = 0.0;

    if (try_goal_bias) {
      const double dx = goal_longitudinal(rng_);
      const double dy = goal_lateral(rng_);
      x = (*goal)[0] + dx;
      y = (*goal)[1] + dy;
    } else {
      const double forward = forward_dist_(rng_);
      const double lateral = lateral_dist_(rng_);

      x = car_x + forward * std::cos(car_yaw) - lateral * std::sin(car_yaw);
      y = car_y + forward * std::sin(car_yaw) + lateral * std::cos(car_yaw);
    }

    if (isPointFree(x, y)) {
      addSamplePoint(x, y);
      return std::array<double, 2>{x, y};
    }
  }

  if (goal && isPointFree((*goal)[0], (*goal)[1])) {
    addSamplePoint((*goal)[0], (*goal)[1]);
    return *goal;
  }

  const double forward = fallback_forward_dist_(rng_);
  const double x = car_x + forward * std::cos(car_yaw);
  const double y = car_y + forward * std::sin(car_yaw);
  addSamplePoint(x, y);
  return std::array<double, 2>{x, y};
}

int RRTNode::nearest(const std::vector<TreeNode> &tree, const std::array<double, 2> &point) const {
  double best_distance = std::numeric_limits<double>::infinity();
  int best_index = 0;

  for (size_t i = 0; i < tree.size(); ++i) {
    const double dx = tree[i].x - point[0];
    const double dy = tree[i].y - point[1];
    const double dist = dx * dx + dy * dy;
    if (dist < best_distance) {
      best_distance = dist;
      best_index = static_cast<int>(i);
    }
  }

  return best_index;
}

TreeNode RRTNode::steer(const TreeNode &nearest_node, const std::array<double, 2> &point) const {
  TreeNode result;
  const double dx = point[0] - nearest_node.x;
  const double dy = point[1] - nearest_node.y;
  const double distance = std::hypot(dx, dy);

  if (distance <= steer_range_) {
    result.x = point[0];
    result.y = point[1];
  } else if (distance > 1e-6) {
    const double scale = steer_range_ / distance;
    result.x = nearest_node.x + dx * scale;
    result.y = nearest_node.y + dy * scale;
  } else {
    result.x = nearest_node.x;
    result.y = nearest_node.y;
  }

  result.parent = -1;
  return result;
}

bool RRTNode::checkCollision(const TreeNode &from, const TreeNode &to) const {
  if (!map_ready_ || map_inflated_.data.empty() || map_inflated_.info.resolution <= 0.0) {
    return true;
  }

  const auto &info = map_inflated_.info;
  const double resolution = info.resolution;
  const int width = static_cast<int>(info.width);
  const int height = static_cast<int>(info.height);
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;

  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double distance = std::hypot(dx, dy);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / resolution)));

  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const double world_x = from.x + t * dx;
    const double world_y = from.y + t * dy;

    const int grid_x = static_cast<int>((world_x - origin_x) / resolution);
    const int grid_y = static_cast<int>((world_y - origin_y) / resolution);

    if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
      return true;
    }

    const size_t index = static_cast<size_t>(grid_y) * width + static_cast<size_t>(grid_x);
    if (map_inflated_.data[index] != 0) {
      return true;
    }
  }

  return false;
}

bool RRTNode::isGoal(const TreeNode &node, double goal_x, double goal_y) const {
  const double dx = node.x - goal_x;
  const double dy = node.y - goal_y;
  return (dx * dx + dy * dy) <= (goal_threshold_ * goal_threshold_);
}

std::vector<TreeNode> RRTNode::buildPath(const std::vector<TreeNode> &tree, int goal_index) const {
  std::vector<TreeNode> path;
  int idx = goal_index;
  while (idx >= 0 && static_cast<size_t>(idx) < tree.size()) {
    path.push_back(tree[idx]);
    idx = tree[idx].parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::optional<std::array<double, 2>> RRTNode::selectGoal() const {
  if (!current_pose_) {
    return std::nullopt;
  }

  const Pose2D pose = *current_pose_;
  const std::array<double, 2> car_position{pose.x, pose.y};
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  std::optional<std::array<double, 2>> best_waypoint;
  double best_distance = std::numeric_limits<double>::infinity();

  constexpr double kMinLookahead = 0.3;

  for (const auto &wp : waypoints_) {
    const double dx = wp[0] - car_position[0];
    const double dy = wp[1] - car_position[1];
    const double distance = std::hypot(dx, dy);
    if (distance < kMinLookahead || distance > goal_lookahead_distance_) {
      continue;
    }

    const double x_car = dx * cos_yaw + dy * sin_yaw;
    if (x_car <= 0.0) {
      continue;
    }

    if (!best_waypoint || distance < best_distance) {
      best_waypoint = wp;
      best_distance = distance;
    }
  }

  return best_waypoint;
}

void RRTNode::visualize(const std::vector<TreeNode> &tree, const std::vector<TreeNode> *path) const {
  if (!tree_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray array_msg;
  visualization_msgs::msg::Marker nodes;
  nodes.header.frame_id = map_frame_;
  nodes.header.stamp = this->now();
  nodes.ns = "tree_nodes";
  nodes.id = 0;
  nodes.type = visualization_msgs::msg::Marker::POINTS;
  nodes.action = visualization_msgs::msg::Marker::ADD;
  nodes.scale.x = 0.05;
  nodes.scale.y = 0.05;
  nodes.color.r = 0.0f;
  nodes.color.g = 0.7f;
  nodes.color.b = 0.0f;
  nodes.color.a = 1.0f;

  visualization_msgs::msg::Marker branches;
  branches.header.frame_id = map_frame_;
  branches.header.stamp = nodes.header.stamp;
  branches.ns = "tree_branches";
  branches.id = 1;
  branches.type = visualization_msgs::msg::Marker::LINE_LIST;
  branches.action = visualization_msgs::msg::Marker::ADD;
  branches.scale.x = 0.02;
  branches.color.r = 0.0f;
  branches.color.g = 0.0f;
  branches.color.b = 0.7f;
  branches.color.a = 0.5f;

  for (size_t i = 0; i < tree.size(); ++i) {
    geometry_msgs::msg::Point p;
    p.x = tree[i].x;
    p.y = tree[i].y;
    p.z = 0.1;
    nodes.points.push_back(p);

    if (tree[i].parent >= 0 && static_cast<size_t>(tree[i].parent) < tree.size()) {
      geometry_msgs::msg::Point parent_point;
      parent_point.x = tree[tree[i].parent].x;
      parent_point.y = tree[tree[i].parent].y;
      parent_point.z = 0.1;
      branches.points.push_back(parent_point);
      branches.points.push_back(p);
    }
  }

  array_msg.markers.push_back(nodes);
  array_msg.markers.push_back(branches);
  tree_pub_->publish(array_msg);

  if (!path_pub_) {
    return;
  }

  visualization_msgs::msg::Marker path_marker;
  path_marker.header.frame_id = map_frame_;
  path_marker.header.stamp = nodes.header.stamp;
  path_marker.ns = "rrt_path";
  path_marker.id = 2;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.scale.x = 0.12;
  path_marker.color.r = 1.0f;
  path_marker.color.g = 0.0f;
  path_marker.color.b = 0.0f;
  path_marker.color.a = 1.0f;

  if (path) {
    for (const auto &node : *path) {
      geometry_msgs::msg::Point p;
      p.x = node.x;
      p.y = node.y;
      p.z = 0.15;
      path_marker.points.push_back(p);
    }
  } else {
    path_marker.action = visualization_msgs::msg::Marker::DELETE;
  }

  path_pub_->publish(path_marker);
  publishSampleMarkers();
}

bool RRTNode::loadWaypoints() {
  std::string package_share;
  try {
    package_share = ament_index_cpp::get_package_share_directory("lab7_pkg_cpp");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Unable to locate package share directory: %s", e.what());
    return false;
  }

  const std::string csv_path = package_share + "/resource/levine1.csv";
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint CSV: %s", csv_path.c_str());
    return false;
  }

  std::string line;
  waypoints_.clear();
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::stringstream ss(line);
    std::string x_str;
    std::string y_str;
    if (!std::getline(ss, x_str, ',')) {
      continue;
    }
    if (!std::getline(ss, y_str, ',')) {
      continue;
    }
    try {
      const double x = std::stod(x_str);
      const double y = std::stod(y_str);
      waypoints_.push_back({x, y});
    } catch (const std::exception &) {
      continue;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints.", waypoints_.size());
  return !waypoints_.empty();
}

void RRTNode::inflateMap(nav_msgs::msg::OccupancyGrid &map) const {
  if (map.data.empty() || map.info.resolution <= 0.0) {
    return;
  }

  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const int radius = static_cast<int>(std::ceil(obstacle_margin_ / map.info.resolution));
  if (radius <= 0) {
    return;
  }

  const auto data = map.data;
  std::vector<int8_t> inflated = data;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const size_t index = static_cast<size_t>(y) * width + static_cast<size_t>(x);
      if (data[index] != 100) {
        continue;
      }
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          if (dx * dx + dy * dy > radius * radius) {
            continue;
          }
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
            continue;
          }
          const size_t nindex = static_cast<size_t>(ny) * width + static_cast<size_t>(nx);
          inflated[nindex] = 100;
        }
      }
    }
  }

  map.data = std::move(inflated);
}

bool RRTNode::isPointFree(double x, double y) const {
  if (!map_ready_ || map_inflated_.data.empty() || map_inflated_.info.resolution <= 0.0) {
    return false;
  }

  const auto &info = map_inflated_.info;
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  const double resolution = info.resolution;
  const int width = static_cast<int>(info.width);
  const int height = static_cast<int>(info.height);

  const int grid_x = static_cast<int>((x - origin_x) / resolution);
  const int grid_y = static_cast<int>((y - origin_y) / resolution);

  if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
    return false;
  }

  const size_t index = static_cast<size_t>(grid_y) * width + static_cast<size_t>(grid_x);
  return map_inflated_.data[index] == 0;
}

void RRTNode::addSamplePoint(double x, double y) {
  sample_points_.push_back({x, y});
  if (sample_points_.size() > max_sample_points_) {
    sample_points_.pop_front();
  }
}

void RRTNode::publishWaypointsMarkers() const {
  if (!waypoint_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const auto stamp = this->now();

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = map_frame_;
  clear_marker.header.stamp = stamp;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  if (!path_points_.empty()) {
    int id = 0;
    for (const auto &point : path_points_) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = map_frame_;
      marker.header.stamp = stamp;
      marker.ns = "waypoints";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = point[0];
      marker.pose.position.y = point[1];
      marker.pose.position.z = 0.1;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.a = 1.0f;
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker_array.markers.push_back(marker);
    }
  }

  waypoint_pub_->publish(marker_array);
}

void RRTNode::publishSampleMarkers() const {
  if (!sample_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const auto stamp = this->now();

  visualization_msgs::msg::Marker samples;
  samples.header.frame_id = map_frame_;
  samples.header.stamp = stamp;
  samples.ns = "samples";
  samples.id = 0;
  samples.type = visualization_msgs::msg::Marker::POINTS;
  samples.action = visualization_msgs::msg::Marker::ADD;
  samples.pose.orientation.w = 1.0;
  samples.scale.x = 0.1;
  samples.scale.y = 0.1;
  samples.color.a = 1.0f;
  samples.color.r = 0.0f;
  samples.color.g = 1.0f;
  samples.color.b = 0.0f;

  for (const auto &point : sample_points_) {
    geometry_msgs::msg::Point pt;
    pt.x = point[0];
    pt.y = point[1];
    pt.z = 0.1;
    samples.points.push_back(pt);
  }

  marker_array.markers.push_back(samples);

  visualization_msgs::msg::Marker pose_marker;
  pose_marker.header.frame_id = map_frame_;
  pose_marker.header.stamp = stamp;
  pose_marker.ns = "current_pose";
  pose_marker.id = 1;
  pose_marker.type = visualization_msgs::msg::Marker::ARROW;

  if (current_pose_) {
    pose_marker.action = visualization_msgs::msg::Marker::ADD;
    pose_marker.pose.position.x = current_pose_->x;
    pose_marker.pose.position.y = current_pose_->y;
    pose_marker.pose.position.z = 0.1;
    pose_marker.scale.x = 0.5;
    pose_marker.scale.y = 0.1;
    pose_marker.scale.z = 0.1;
    pose_marker.color.a = 1.0f;
    pose_marker.color.r = 1.0f;
    pose_marker.color.g = 0.0f;
    pose_marker.color.b = 0.0f;
    const double half_yaw = current_pose_->yaw * 0.5;
    pose_marker.pose.orientation.x = 0.0;
    pose_marker.pose.orientation.y = 0.0;
    pose_marker.pose.orientation.z = std::sin(half_yaw);
    pose_marker.pose.orientation.w = std::cos(half_yaw);
  } else {
    pose_marker.action = visualization_msgs::msg::Marker::DELETE;
  }

  marker_array.markers.push_back(pose_marker);

  sample_pub_->publish(marker_array);
}

double RRTNode::yawFromQuaternion(double x, double y, double z, double w) {
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace lab7_pkg_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lab7_pkg_cpp::RRTNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
