#ifndef LAB7_PKG__RRT_H_
#define LAB7_PKG__RRT_H_

#include <cmath>
#include <vector>
#include <string>
#include <random>
#include <fstream>
#include <sstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

/// Struct defining a node in the RRT tree
struct RRT_Node {
    double x;
    double y;
    int parent;            // index of parent node in the tree (or -1 for root)
    double cost;           // (optional) cost for RRT* (not used in basic RRT)
    bool is_root;
    RRT_Node(double px=0.0, double py=0.0, int parent_idx=-1) 
        : x(px), y(py), parent(parent_idx), cost(0.0), is_root(parent_idx < 0) {}
};

class RRT : public rclcpp::Node {
public:
    RRT();
    ~RRT() override;

private:
    // Subscribers and Publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_viz_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_viz_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sample_viz_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_update_pub_;

    // Timers for periodic tasks
    rclcpp::TimerBase::SharedPtr planning_timer_;
    rclcpp::TimerBase::SharedPtr sample_timer_;
    rclcpp::TimerBase::SharedPtr waypoints_timer_;

    // Random number generation for sampling
    std::mt19937 gen_;  // Mersenne Twister random generator
    std::uniform_real_distribution<double> x_dist_;
    std::uniform_real_distribution<double> y_dist_;

    // Internal state
    bool sim_;                 // true if using simulation (Odometry), false if using PoseStamped
    bool map_received_;        // flag to indicate static map has been received
    bool pose_received_;       // flag to indicate current pose has been received
    nav_msgs::msg::OccupancyGrid map_updated_;  // occupancy grid (inflated + dynamic obstacles)
    double current_x_, current_y_, current_yaw_; // current vehicle pose (x, y, yaw)

    // RRT parameters (loaded from ROS parameters)
    int    max_iter_;
    double steer_range_;
    double goal_threshold_;
    double sample_range_x_;
    double sample_range_y_;
    double max_goal_distance_;
    double margin_;           // obstacle inflation margin
    int    max_samples_;      // max sample points to visualize

    // Waypoints and path
    std::vector<std::pair<double,double>> waypoints_;      // loaded waypoints from CSV (track waypoints)
    std::vector<std::pair<double,double>> path_points_;    // current RRT path waypoints (for publishing)

    // Sampled points for visualization
    std::vector<geometry_msgs::msg::Point> sample_points_;

    // Subscriber callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);

    // RRT methods
    std::pair<double, double> sample();  // sample a free point in front of the car
    bool is_point_free(double x, double y) const;  // check occupancy grid for collision
    int  nearest(const std::vector<RRT_Node> &tree, const std::pair<double,double> &pt);
    RRT_Node steer(const RRT_Node &nearest_node, const std::pair<double,double> &sampled_pt);
    bool check_collision(const RRT_Node &nearest_node, const RRT_Node &new_node);
    bool is_goal(const RRT_Node &node, double goal_x, double goal_y);
    std::vector<RRT_Node> find_path(const std::vector<RRT_Node> &tree, const RRT_Node &latest_node);

    // NOTE: now returns path AND fills 'tree' for visualization (same idea as Python self.tree)
    std::vector<RRT_Node> plan_rrt(double start_x, double start_y,
                                   double goal_x, double goal_y,
                                   std::vector<RRT_Node> &tree);

    // Helper methods
    double get_yaw_from_quaternion(double x, double y, double z, double w);
    void visualize_rrt(const std::vector<RRT_Node> &path, const std::vector<RRT_Node> &tree);
    void visualize_samples();
    void publish_waypoints_markers();

    // Timer callbacks
    void run_rrt_planning();   // periodically perform RRT planning
    void test_sampling();      // periodically sample points for visualization
};

#endif  // LAB7_PKG__RRT_H_
