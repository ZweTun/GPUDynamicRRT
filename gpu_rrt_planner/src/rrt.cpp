#include "rrt/rrt.h"

#include <limits>
#include <algorithm>

using std::placeholders::_1;

RRT::RRT()
: rclcpp::Node("rrt_node"), gen_(std::random_device{}())
{
    // Declare and get ROS parameters
    this->declare_parameter<bool>("sim", true);
    
    this->declare_parameter<std::string>("waypoints_csv", "");
    //this->declare_parameter<std::string>("waypoints_csv","/home/jeff/sim_ws/src/sampling-based-motion-planning-team10/lab7_pkg/scripts/levine.csv");
    
    this->declare_parameter<int>("max_iter", 2500);
    this->declare_parameter<double>("steer_range", 0.5);
    this->declare_parameter<double>("goal_threshold", 0.15);
    this->declare_parameter<double>("sample_range_x", 5.0);
    this->declare_parameter<double>("sample_range_y", 5.0);
    this->declare_parameter<double>("max_goal_distance", 4.0);
    this->declare_parameter<double>("margin", 0.15);
    this->declare_parameter<int>("max_samples", 100);

    sim_               = this->get_parameter("sim").get_value<bool>();
    std::string csv_path = this->get_parameter("waypoints_csv").get_value<std::string>();
    max_iter_          = this->get_parameter("max_iter").get_value<int>();
    steer_range_       = this->get_parameter("steer_range").get_value<double>();
    goal_threshold_    = this->get_parameter("goal_threshold").get_value<double>();
    sample_range_x_    = this->get_parameter("sample_range_x").get_value<double>();
    sample_range_y_    = this->get_parameter("sample_range_y").get_value<double>();
    max_goal_distance_ = this->get_parameter("max_goal_distance").get_value<double>();
    margin_            = this->get_parameter("margin").get_value<double>();
    max_samples_       = this->get_parameter("max_samples").get_value<int>();

    // Load waypoints from CSV file for the track (if provided)
    if (!csv_path.empty()) {
        std::ifstream infile(csv_path);
        if (!infile.is_open()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to open waypoints CSV file: %s",
                         csv_path.c_str());
        } else {
            std::string line;
            // Skip header line if present
            if (std::getline(infile, line)) {
                if (!line.empty() && line.at(0) == '#') {
                    // header starting with '#', skip
                } else {
                    // First line is data
                    std::stringstream ss(line);
                    std::string xs, ys;
                    if (std::getline(ss, xs, ',') && std::getline(ss, ys, ',')) {
                        try {
                            double x = std::stod(xs);
                            double y = std::stod(ys);
                            waypoints_.emplace_back(x, y);
                        } catch (...) {
                            // ignore malformed
                        }
                    }
                }
            }
            // Remaining lines
            while (std::getline(infile, line)) {
                if (line.empty() || line.at(0) == '#') {
                    continue;
                }
                std::stringstream ss(line);
                std::string xs, ys;
                if (std::getline(ss, xs, ',') && std::getline(ss, ys, ',')) {
                    try {
                        double x = std::stod(xs);
                        double y = std::stod(ys);
                        waypoints_.emplace_back(x, y);
                    } catch (...) {
                        // ignore malformed
                    }
                }
            }
            infile.close();
            RCLCPP_INFO(this->get_logger(),
                        "Loaded %zu waypoints from CSV.",
                        waypoints_.size());
        }
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "No waypoints CSV file specified. Using fallback goal selection.");
    }

    // Initialize occupancy grid flags
    map_received_  = false;
    pose_received_ = false;

    // Publishers for drive and visualisation
    drive_pub_    = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive", 10);
    tree_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/rrt_tree", 10);
    path_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/rrt_path", 10);
    waypoints_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/waypoints_markers", 10);
    sample_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/sample_points", 10);

    // Publisher for updated map (latched QoS to persist last message)
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();
    map_update_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map_updated", map_qos);

    // Create subscribers for pose (Odometry or PoseStamped based on simulation flag)
    std::string pose_topic;
    if (sim_) {
        pose_topic = "/ego_racecar/odom";
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            pose_topic, 10, std::bind(&RRT::odom_callback, this, _1));
    } else {
        pose_topic = "/pf/viz/inferred_pose";
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, 10, std::bind(&RRT::pose_callback, this, _1));
    }

    // LaserScan subscriber for dynamic obstacle updates
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&RRT::scan_callback, this, _1));

    // OccupancyGrid map subscriber (latched map)
    rclcpp::QoS map_sub_qos(rclcpp::KeepLast(1));
    map_sub_qos.reliable();
    map_sub_qos.transient_local();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_sub_qos, std::bind(&RRT::map_callback, this, _1));

    // Initialize random distribution for sampling (biased forward)
    x_dist_ = std::uniform_real_distribution<double>(0.7, sample_range_x_);
    y_dist_ = std::uniform_real_distribution<double>(-sample_range_y_, sample_range_y_);

    // Create timers for planning, sampling visualization, and waypoint marker publishing
    planning_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RRT::run_rrt_planning, this));

    sample_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&RRT::test_sampling, this));

    waypoints_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&RRT::publish_waypoints_markers, this));

    RCLCPP_INFO(this->get_logger(),
                "RRT node initialized (sim=%s)",
                sim_ ? "true" : "false");
}

RRT::~RRT()
{
    RCLCPP_INFO(this->get_logger(), "RRT node shutting down.");
}

void RRT::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {
    // Received the static map; inflate obstacles and store it
    if (map_received_) {
        return;  // Only process the first map message
    }
    RCLCPP_INFO(this->get_logger(), "Static map received (%d x %d)",
                map_msg->info.width, map_msg->info.height);
    map_received_ = true;
    // Copy the map message to map_updated_
    map_updated_ = *map_msg;
    // Inflate occupied cells by margin_
    double resolution = map_msg->info.resolution;
    int width = map_msg->info.width;
    int height = map_msg->info.height;
    int inflate_cells = static_cast<int>(std::floor(margin_ / resolution));
    std::vector<int8_t> data_in = map_msg->data;   // original map data
    std::vector<int8_t> data_out = data_in;        // copy to modify
    // Iterate through each cell and inflate obstacles
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = y * width + x;
            if (data_in[idx] == 100) {  // occupied cell
                // Mark neighboring cells within radius as occupied
                for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
                    for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
                        if (dx*dx + dy*dy <= inflate_cells * inflate_cells) {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                                int n_idx = ny * width + nx;
                                if (data_out[n_idx] != 100) {
                                    data_out[n_idx] = 100;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    map_updated_.data = data_out;
    // Publish the inflated map once
    map_update_pub_->publish(map_updated_);
    RCLCPP_INFO(this->get_logger(),
                "Published inflated occupancy grid (obstacle margin %.2fm).",
                margin_);
}

void RRT::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    // Extract pose from Odometry and update current pose
    const geometry_msgs::msg::Pose &pose = odom_msg->pose.pose;
    current_x_ = pose.position.x;
    current_y_ = pose.position.y;
    current_yaw_ = get_yaw_from_quaternion(pose.orientation.x,
                                           pose.orientation.y,
                                           pose.orientation.z,
                                           pose.orientation.w);
    pose_received_ = true;
}

void RRT::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
    // Extract pose from PoseStamped and update current pose
    const geometry_msgs::msg::Pose &pose = pose_msg->pose;
    current_x_ = pose.position.x;
    current_y_ = pose.position.y;
    current_yaw_ = get_yaw_from_quaternion(pose.orientation.x,
                                           pose.orientation.y,
                                           pose.orientation.z,
                                           pose.orientation.w);
    pose_received_ = true;
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    // Update the occupancy grid with obstacles from the LaserScan
    if (!map_received_) {
        return;  // wait until base map is received and inflated
    }
    if (!pose_received_) {
        RCLCPP_WARN(this->get_logger(),
                    "No pose available for LaserScan processing");
        return;
    }
    // Copy current map data for updating
    int width = map_updated_.info.width;
    int height = map_updated_.info.height;
    double res = map_updated_.info.resolution;
    double origin_x = map_updated_.info.origin.position.x;
    double origin_y = map_updated_.info.origin.position.y;
    std::vector<int8_t> new_map_data = map_updated_.data;  // start from existing map
    // Iterate through each laser ray
    size_t n_ranges = scan_msg->ranges.size();
    for (size_t i = 0; i < n_ranges; ++i) {
        float r = scan_msg->ranges[i];
        if (std::isnan(r) || std::isinf(r)) {
            continue;  // skip invalid range
        }
        // Compute angle of this ray in global frame
        float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        double obs_x = current_x_ + r * std::cos(current_yaw_ + angle);
        double obs_y = current_y_ + r * std::sin(current_yaw_ + angle);
        // Convert obstacle position to grid indices
        int grid_x = static_cast<int>((obs_x - origin_x) / res);
        int grid_y = static_cast<int>((obs_y - origin_y) / res);
        if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
            int idx = grid_y * width + grid_x;
            new_map_data[idx] = 100;  // mark cell as occupied
        }
    }
    // Update the stored occupancy grid (but do not override static inflation)
    map_updated_.data = new_map_data;
    // Optional: re-publish if desired
    // map_update_pub_->publish(map_updated_);
    RCLCPP_INFO(this->get_logger(),
                "LaserScan processed: updated occupancy grid with obstacles.");
}

double RRT::get_yaw_from_quaternion(double x, double y, double z, double w) {
    // Convert quaternion (x,y,z,w) to yaw (in radians)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

std::pair<double, double> RRT::sample() {
    // Sample a random point in free space, biased towards the area in front of the vehicle
    if (!pose_received_ || !map_received_) {
        return {NAN, NAN};
    }
    double car_x = current_x_;
    double car_y = current_y_;
    double car_theta = current_yaw_;
    // Try up to max_attempts to find a free point
    const int max_attempts = 100;
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        // Sample a forward distance (biased between 0.7m and sample_range_x_) and a lateral offset
        double forward_distance = x_dist_(gen_);
        double lateral_offset = y_dist_(gen_);
        // Compute sample coordinates in the global frame
        double sx = car_x + forward_distance * std::cos(car_theta)
                             - lateral_offset   * std::sin(car_theta);
        double sy = car_y + forward_distance * std::sin(car_theta)
                             + lateral_offset   * std::cos(car_theta);
        // Check if sampled point is in free space
        if (is_point_free(sx, sy)) {
            return {sx, sy};
        }
    }
    // If no valid point found, fallback: sample closer point directly in front of the car
    double fd = std::uniform_real_distribution<double>(0.3, 1.0)(gen_);
    double sx = car_x + fd * std::cos(car_theta);
    double sy = car_y + fd * std::sin(car_theta);
    return {sx, sy};
}

bool RRT::is_point_free(double x, double y) const {
    // Check if a given (x,y) lies in free space (not occupied) on the occupancy grid
    if (!map_received_) {
        return false;
    }
    // Convert world coordinates to map grid indices
    int grid_x = static_cast<int>((x - map_updated_.info.origin.position.x) /
                                  map_updated_.info.resolution);
    int grid_y = static_cast<int>((y - map_updated_.info.origin.position.y) /
                                  map_updated_.info.resolution);
    if (grid_x < 0 || grid_x >= static_cast<int>(map_updated_.info.width) ||
        grid_y < 0 || grid_y >= static_cast<int>(map_updated_.info.height)) {
        // Point is outside the map bounds
        return false;
    }
    int index = grid_y * map_updated_.info.width + grid_x;
    int8_t cell = map_updated_.data[index];
    // cell value: 0 = free, 100 = occupied, -1 = unknown. Only treat 0 as free.
    return (cell == 0);
}

int RRT::nearest(const std::vector<RRT_Node> &tree,
                 const std::pair<double,double> &pt) {
    // Find the index of the tree node closest to the given point (Euclidean distance)
    int nearest_index = 0;
    double min_dist_sq = std::numeric_limits<double>::infinity();
    double px = pt.first;
    double py = pt.second;
    for (size_t i = 0; i < tree.size(); ++i) {
        double dx = tree[i].x - px;
        double dy = tree[i].y - py;
        double dist_sq = dx*dx + dy*dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest_index = static_cast<int>(i);
        }
    }
    return nearest_index;
}

RRT_Node RRT::steer(const RRT_Node &nearest_node,
                    const std::pair<double,double> &sampled_pt) {
    // Steer from the nearest node towards the sampled point, but only up to steer_range_ distance
    double nx = nearest_node.x;
    double ny = nearest_node.y;
    double tx = sampled_pt.first;
    double ty = sampled_pt.second;
    // Compute distance and direction from nearest to sampled point
    double dx = tx - nx;
    double dy = ty - ny;
    double dist = std::sqrt(dx*dx + dy*dy);
    RRT_Node new_node;
    if (dist <= steer_range_) {
        // We can reach the sampled point directly
        new_node.x = tx;
        new_node.y = ty;
    } else {
        // Move from nearest node toward sampled point by steer_range_
        double theta = std::atan2(dy, dx);
        new_node.x = nx + steer_range_ * std::cos(theta);
        new_node.y = ny + steer_range_ * std::sin(theta);
    }
    new_node.parent = -1;  // parent will be set by caller
    new_node.is_root = false;
    return new_node;
}

bool RRT::check_collision(const RRT_Node &nearest_node,
                          const RRT_Node &new_node) {
    // Check if the straight line path from nearest_node to new_node is free of obstacles
    // Determine how many intermediate points to check based on grid resolution
    double dx = new_node.x - nearest_node.x;
    double dy = new_node.y - nearest_node.y;
    int steps_x = std::abs(static_cast<int>(
        std::ceil(dx / map_updated_.info.resolution)));
    int steps_y = std::abs(static_cast<int>(
        std::ceil(dy / map_updated_.info.resolution)));
    int num_steps = std::max(steps_x, steps_y);
    if (num_steps == 0) {
        // Nodes are essentially in the same cell
        return !is_point_free(new_node.x, new_node.y);  // true if collision
    }
    // Check intermediate points along the line
    double step_fraction = 1.0 / static_cast<double>(num_steps);
    for (int i = 0; i <= num_steps; ++i) {
        double t = i * step_fraction;
        double cx = nearest_node.x + t * dx;
        double cy = nearest_node.y + t * dy;
        if (!is_point_free(cx, cy)) {
            return true;  // collision detected
        }
    }
    return false;  // no collision along the path
}

bool RRT::is_goal(const RRT_Node &node, double goal_x, double goal_y) {
    // Check if the node is within goal_threshold_ distance of the goal
    double dx = node.x - goal_x;
    double dy = node.y - goal_y;
    double dist_sq = dx*dx + dy*dy;
    return (dist_sq <= goal_threshold_ * goal_threshold_);
}

std::vector<RRT_Node> RRT::find_path(const std::vector<RRT_Node> &tree,
                                     const RRT_Node &latest_node) {
    // Reconstruct path from start to goal by traversing parent links from latest_node back to root
    std::vector<RRT_Node> path;
    RRT_Node current = latest_node;
    path.push_back(current);
    int parent_index = current.parent;
    // Traverse back to root
    while (parent_index != -1) {
        current = tree[parent_index];
        path.push_back(current);
        parent_index = current.parent;
    }
    // Reverse the path to get start -> goal order
    std::reverse(path.begin(), path.end());
    // Store the path points for publishing (2D waypoints)
    path_points_.clear();
    path_points_.reserve(path.size());
    for (auto &node : path) {
        path_points_.emplace_back(node.x, node.y);
    }
    return path;
}

std::vector<RRT_Node> RRT::plan_rrt(double start_x, double start_y,
                                    double goal_x, double goal_y,
                                    std::vector<RRT_Node> &tree) {
    // Execute the RRT algorithm from start to goal. Returns the path if found (or empty vector if no path).
    tree.clear();
    tree.reserve(max_iter_ + 1);

    // Initialize tree with start node
    RRT_Node start_node(start_x, start_y, -1);
    start_node.is_root = true;
    tree.push_back(start_node);

    for (int i = 0; i < max_iter_; ++i) {
        // Sample a random free point
        std::pair<double,double> sampled_point = sample();
        if (std::isnan(sampled_point.first)) {
            // Sampling failed (no pose or map)
            continue;
        }
        // Find nearest existing tree node to this sample
        int nearest_idx = nearest(tree, sampled_point);
        RRT_Node nearest_node = tree[nearest_idx];
        // Steer from nearest node towards the sampled point
        RRT_Node new_node = steer(nearest_node, sampled_point);
        new_node.parent = nearest_idx;
        // Check if the new edge is collision-free
        if (!check_collision(nearest_node, new_node)) {
            // Add new node to the tree
            tree.push_back(new_node);
            // Check if new node is close enough to goal
            if (is_goal(new_node, goal_x, goal_y)) {
                RCLCPP_INFO(this->get_logger(),
                            "Path found after %d iterations.", i+1);
                // Construct path from start to goal
                std::vector<RRT_Node> path = find_path(tree, new_node);
                return path;
            }
        }
    }
    // If we reach here, no path was found within max_iter_
    RCLCPP_WARN(this->get_logger(),
                "No path found after %d iterations.", max_iter_);
    return std::vector<RRT_Node>();  // empty path
}

void RRT::visualize_rrt(const std::vector<RRT_Node> &path,
                        const std::vector<RRT_Node> &tree) {
    // Publish visualization markers for the RRT tree and the path (if any)
    if (tree.empty()) {
        return;
    }
    visualization_msgs::msg::MarkerArray marker_array;
    // Marker for tree nodes (POINTS)
    visualization_msgs::msg::Marker nodes_marker;
    nodes_marker.header.frame_id = "map";
    nodes_marker.header.stamp = this->now();
    nodes_marker.ns = "tree_nodes";
    nodes_marker.id = 0;
    nodes_marker.type = visualization_msgs::msg::Marker::POINTS;
    nodes_marker.action = visualization_msgs::msg::Marker::ADD;
    nodes_marker.scale.x = 0.05;
    nodes_marker.scale.y = 0.05;
    nodes_marker.color.r = 0.0f;
    nodes_marker.color.g = 0.7f;
    nodes_marker.color.b = 0.0f;
    nodes_marker.color.a = 1.0f;
    // Marker for tree branches (LINE_LIST)
    visualization_msgs::msg::Marker branches_marker;
    branches_marker.header.frame_id = "map";
    branches_marker.header.stamp = this->now();
    branches_marker.ns = "tree_branches";
    branches_marker.id = 1;
    branches_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    branches_marker.action = visualization_msgs::msg::Marker::ADD;
    branches_marker.scale.x = 0.02;
    branches_marker.color.r = 0.0f;
    branches_marker.color.g = 0.0f;
    branches_marker.color.b = 0.7f;
    branches_marker.color.a = 0.5f;
    // Populate tree nodes and branches
    for (size_t i = 0; i < tree.size(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = tree[i].x;
        p.y = tree[i].y;
        p.z = 0.1;
        // Add node point
        nodes_marker.points.push_back(p);
        // Add line from parent to this node (if not root)
        if (i > 0 && tree[i].parent != -1) {
            geometry_msgs::msg::Point p_parent;
            p_parent.x = tree[tree[i].parent].x;
            p_parent.y = tree[tree[i].parent].y;
            p_parent.z = 0.1;
            // Each line segment is a pair of points: parent and current node
            branches_marker.points.push_back(p_parent);
            branches_marker.points.push_back(p);
        }
    }
    marker_array.markers.push_back(nodes_marker);
    marker_array.markers.push_back(branches_marker);
    // Publish the tree visualization
    tree_viz_pub_->publish(marker_array);
    // If a path was found, visualize it
    if (!path.empty()) {
        // Marker for the RRT path (LINE_STRIP)
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = this->now();
        path_marker.ns = "path";
        path_marker.id = 2;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = 0.12;
        path_marker.color.r = 1.0f;
        path_marker.color.g = 0.0f;
        path_marker.color.b = 0.0f;
        path_marker.color.a = 1.0f;
        // Marker for path waypoints (SPHERE_LIST for visualization)
        visualization_msgs::msg::Marker waypoints_marker;
        waypoints_marker.header.frame_id = "map";
        waypoints_marker.header.stamp = this->now();
        waypoints_marker.ns = "path_waypoints";
        waypoints_marker.id = 3;
        waypoints_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        waypoints_marker.action = visualization_msgs::msg::Marker::ADD;
        waypoints_marker.scale.x = 0.15;
        waypoints_marker.scale.y = 0.15;
        waypoints_marker.scale.z = 0.15;
        waypoints_marker.color.r = 1.0f;
        waypoints_marker.color.g = 0.5f;
        waypoints_marker.color.b = 0.0f;
        waypoints_marker.color.a = 1.0f;
        // Fill path points in markers
        for (const RRT_Node &node : path) {
            geometry_msgs::msg::Point p;
            p.x = node.x;
            p.y = node.y;
            p.z = 0.15;
            path_marker.points.push_back(p);
            waypoints_marker.points.push_back(p);
        }
        // Publish the path line and waypoints markers
        path_viz_pub_->publish(path_marker);
        visualization_msgs::msg::MarkerArray path_array;
        path_array.markers.push_back(waypoints_marker);
        tree_viz_pub_->publish(path_array);
    }
}

void RRT::visualize_samples() {
    // Prepare markers for sampled points and current pose
    visualization_msgs::msg::MarkerArray marker_array;
    // Marker for all sampled points (POINTS)
    visualization_msgs::msg::Marker samples_marker;
    samples_marker.header.frame_id = "map";
    samples_marker.header.stamp = this->now();
    samples_marker.ns = "samples";
    samples_marker.id = 0;
    samples_marker.type = visualization_msgs::msg::Marker::POINTS;
    samples_marker.action = visualization_msgs::msg::Marker::ADD;
    samples_marker.scale.x = 0.1;
    samples_marker.scale.y = 0.1;
    samples_marker.color.r = 0.0f;
    samples_marker.color.g = 1.0f;
    samples_marker.color.b = 0.0f;
    samples_marker.color.a = 1.0f;
    // Add all sample points to the marker
    samples_marker.points = sample_points_;
    // Marker for current vehicle pose (ARROW)
    visualization_msgs::msg::Marker pose_marker;
    pose_marker.header.frame_id = "map";
    pose_marker.header.stamp = this->now();
    pose_marker.ns = "current_pose";
    pose_marker.id = 1;
    pose_marker.type = visualization_msgs::msg::Marker::ARROW;
    pose_marker.action = visualization_msgs::msg::Marker::ADD;
    pose_marker.scale.x = 0.5;  // arrow length
    pose_marker.scale.y = 0.1;  // arrow width
    pose_marker.scale.z = 0.1;  // arrow height (thickness)
    pose_marker.color.r = 1.0f;
    pose_marker.color.g = 0.0f;
    pose_marker.color.b = 0.0f;
    pose_marker.color.a = 1.0f;
    // Set arrow pose at current vehicle position and orientation
    pose_marker.pose.position.x = current_x_;
    pose_marker.pose.position.y = current_y_;
    pose_marker.pose.position.z = 0.1;
    // Orientation from current_yaw_
    pose_marker.pose.orientation.x = 0.0;
    pose_marker.pose.orientation.y = 0.0;
    pose_marker.pose.orientation.z = std::sin(current_yaw_ / 2.0);
    pose_marker.pose.orientation.w = std::cos(current_yaw_ / 2.0);
    // Add markers to array and publish
    marker_array.markers.push_back(samples_marker);
    marker_array.markers.push_back(pose_marker);
    sample_viz_pub_->publish(marker_array);
}

void RRT::publish_waypoints_markers() {
    // Publish the current RRT path as a set of waypoint markers (for pure pursuit)
    visualization_msgs::msg::MarkerArray marker_array;
    // Create a sphere marker for each waypoint in the path
    for (size_t i = 0; i < path_points_.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "waypoints";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = path_points_[i].first;
        marker.pose.position.y = path_points_[i].second;
        marker.pose.position.z = 0.1;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker_array.markers.push_back(marker);
    }
    waypoints_markers_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(),
                "Published %zu RRT waypoints for pure pursuit.",
                path_points_.size());
}

void RRT::run_rrt_planning() {
    // Timer callback for RRT path planning
    if (!map_received_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for map...");
        return;
    }
    if (!pose_received_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for pose...");
        return;
    }
    // Current start position
    double start_x = current_x_;
    double start_y = current_y_;
    double car_yaw = current_yaw_;
    // Determine goal: furthest waypoint ahead of the car within max_goal_distance_
    double goal_x, goal_y;
    if (!waypoints_.empty()) {
        double furthest_dist = -1.0;
        goal_x = start_x + max_goal_distance_ * std::cos(car_yaw);  // default goal (straight ahead)
        goal_y = start_y + max_goal_distance_ * std::sin(car_yaw);
        // Find waypoints that are within max_goal_distance_ and in front of the car
        for (auto &wp : waypoints_) {
            double dx = wp.first - start_x;
            double dy = wp.second - start_y;
            // Transform waypoint to car's coordinate frame (check if ahead)
            double x_car_frame = dx * std::cos(-car_yaw) - dy * std::sin(-car_yaw);
            if (x_car_frame > 0) {  // waypoint is in front of car
                double distance = std::sqrt(dx*dx + dy*dy);
                if (distance <= max_goal_distance_ && distance > furthest_dist) {
                    furthest_dist = distance;
                    goal_x = wp.first;
                    goal_y = wp.second;
                }
            }
        }
        if (furthest_dist < 0) {
            // No waypoint found ahead within range, fallback to straight ahead point
            goal_x = start_x + max_goal_distance_ * std::cos(car_yaw);
            goal_y = start_y + max_goal_distance_ * std::sin(car_yaw);
            RCLCPP_WARN(this->get_logger(),
                        "No valid waypoint ahead. Using default goal (%.2f, %.2f).",
                        goal_x, goal_y);
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Selected goal waypoint (%.2f, %.2f) at distance %.2f",
                        goal_x, goal_y, furthest_dist);
        }
    } else {
        // If no waypoints list, just set goal straight ahead at max_goal_distance_
        goal_x = start_x + max_goal_distance_ * std::cos(car_yaw);
        goal_y = start_y + max_goal_distance_ * std::sin(car_yaw);
        RCLCPP_INFO(this->get_logger(),
                    "No waypoints provided. Using default goal (%.2f, %.2f).",
                    goal_x, goal_y);
    }
    RCLCPP_INFO(this->get_logger(),
                "Planning path from (%.2f, %.2f) to (%.2f, %.2f)...",
                start_x, start_y, goal_x, goal_y);

    // Run RRT algorithm and get both path and full tree (for visualization)
    std::vector<RRT_Node> tree;
    std::vector<RRT_Node> path = plan_rrt(start_x, start_y, goal_x, goal_y, tree);

    // Visualize the RRT result (tree and path) — same idea as Python:
    // tree nodes + tree edges + path line + path waypoints.
    visualize_rrt(path, tree);
}

void RRT::test_sampling() {
    // Periodically sample points and publish them for visualization
    if (!map_received_ || !pose_received_) {
        return;
    }

    // Clear and reserve space for new samples
    sample_points_.clear();
    sample_points_.reserve(max_samples_);

    for (int i = 0; i < max_samples_; ++i) {
        auto p = sample();
        // Skip invalid samples
        if (std::isnan(p.first) || std::isnan(p.second)) {
            continue;
        }
        geometry_msgs::msg::Point pt;
        pt.x = p.first;
        pt.y = p.second;
        pt.z = 0.1;  // small height above the map
        sample_points_.push_back(pt);
    }

    // Publish markers for the sampled points and current pose
    visualize_samples();
}
