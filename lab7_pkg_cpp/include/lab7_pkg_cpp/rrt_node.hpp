#pragma once

#include <array>
#include <deque>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace lab7_pkg_cpp {

struct TreeNode {
  double x;
  double y;
  int parent;
};

struct Pose2D {
  double x;
  double y;
  double yaw;
};

class RRTNode : public rclcpp::Node {
public:
  RRTNode();

private:
  // Callbacks
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void planningTimerCallback();

  // Core RRT helpers
  std::optional<std::array<double, 2>> sample(const std::optional<std::array<double, 2>> &goal,
                                              int iteration);
  int nearest(const std::vector<TreeNode> &tree, const std::array<double, 2> &point) const;
  TreeNode steer(const TreeNode &nearest_node, const std::array<double, 2> &point) const;
  bool checkCollision(const TreeNode &from, const TreeNode &to) const;
  bool isGoal(const TreeNode &node, double goal_x, double goal_y) const;
  std::vector<TreeNode> buildPath(const std::vector<TreeNode> &tree, int goal_index) const;
  std::optional<std::array<double, 2>> selectGoal() const;
  void visualize(const std::vector<TreeNode> &tree, const std::vector<TreeNode> *path) const;

  bool loadWaypoints();
  void inflateMap(nav_msgs::msg::OccupancyGrid &map) const;
  bool isPointFree(double x, double y) const;
  static double yawFromQuaternion(double x, double y, double z, double w);
  void publishWaypointsMarkers() const;
  void publishSampleMarkers() const;
  void addSamplePoint(double x, double y);

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sample_pub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr planning_timer_;

  // State
  std::optional<Pose2D> current_pose_;
  nav_msgs::msg::OccupancyGrid map_original_;
  nav_msgs::msg::OccupancyGrid map_inflated_;
  bool map_ready_ = false;
  std::string map_frame_ = "map";
  std::vector<std::array<double, 2>> waypoints_;
  std::vector<std::array<double, 2>> path_points_;
  std::deque<std::array<double, 2>> sample_points_;
  bool sim_mode_ = false;

  // Parameters
  int max_iterations_ = 6000;
  const double goal_threshold_ = 2.0;
  const double steer_range_ = 0.5;
  double sample_forward_min_ = 0.7;
  double sample_forward_max_ = 5.0;
  double sample_lateral_range_ = 5.0;
  const double obstacle_margin_ = 0.15;
  const std::size_t max_sample_points_ = 100;
  double goal_lookahead_distance_ = 12.0;
  double goal_bias_probability_ = 0.35;
  int goal_bias_start_iteration_ = 400;
  double goal_bias_longitudinal_stddev_ = 1.2;
  double goal_bias_lateral_stddev_ = 0.8;

  // Random sampling
  std::mt19937 rng_;
  std::uniform_real_distribution<double> forward_dist_;
  std::uniform_real_distribution<double> lateral_dist_;
  std::uniform_real_distribution<double> fallback_forward_dist_;
};

}  // namespace lab7_pkg_cpp
