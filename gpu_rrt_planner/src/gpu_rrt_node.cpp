#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <cstdlib>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "gpu_rrt_planner.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class GpuRrtNode : public rclcpp::Node {
public:
    GpuRrtNode() : Node("gpu_rrt_node"), map_received_(false), odom_received_(false) {
        // Initialize random seed for sampling
        std::srand(static_cast<unsigned int>(std::time(nullptr)));

        // Subscribe to /map (OccupancyGrid)
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&GpuRrtNode::mapCallback, this, _1));
        // Subscribe to /odom (Odometry)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&GpuRrtNode::odomCallback, this, _1));
        // Subscribe to /goal (PoseStamped)
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal", 10, std::bind(&GpuRrtNode::goalCallback, this, _1));

        // Publisher for visualization of the planned path
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/waypoints_markers", 10);

        // Allocate GPU resources for RRT
        allocateRRTResources(MAX_NODES);
        RCLCPP_INFO(this->get_logger(), "GPU RRT planner node initialized.");
    }

    ~GpuRrtNode() {
        // Free GPU memory on shutdown
        freeRRTResources();
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // Convert occupancy grid data to binary (0 = free, 1 = occupied)
        map_width_ = msg->info.width;
        map_height_ = msg->info.height;
        map_resolution_ = msg->info.resolution;
        map_origin_x_ = msg->info.origin.position.x;
        map_origin_y_ = msg->info.origin.position.y;
        map_data_binary_.resize(map_width_ * map_height_);
        for (size_t i = 0; i < msg->data.size(); ++i) {
            int8_t val = msg->data[i];
            if (val < 0) {
                // Unknown values treated as occupied
                map_data_binary_[i] = 1;
            } else if (val >= OCC_THRESHOLD) {
                map_data_binary_[i] = 1;
            } else {
                map_data_binary_[i] = 0;
            }
        }
        // Copy map to GPU
        setMapData(map_data_binary_.data(), map_width_, map_height_, map_resolution_, map_origin_x_, map_origin_y_);
        map_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Map received (%dx%d)", map_width_, map_height_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Store current robot position from odometry
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        odom_received_ = true;
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Store goal position
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        RCLCPP_INFO(this->get_logger(), "Goal received at (%.2f, %.2f)", goal_x_, goal_y_);
        if (!map_received_ || !odom_received_) {
            RCLCPP_WARN(this->get_logger(), "Cannot plan path: waiting for map and odom.");
            return;
        }
        // Compute path using RRT
        computePath();
    }

    void computePath() {
        // RRT parameters
        const float STEP_SIZE = 1.0f;
        const float GOAL_THRESHOLD = 1.0f;  // distance within which we consider goal "reached"
        const float GOAL_BIAS = 0.05f;      // 5% chance to sample the goal directly
        const int MAX_ITERATIONS = MAX_NODES;

        // Tree data structures (on CPU side)
        std::vector<float> nodes_x;
        std::vector<float> nodes_y;
        std::vector<int> parent;
        nodes_x.reserve(MAX_NODES);
        nodes_y.reserve(MAX_NODES);
        parent.reserve(MAX_NODES);

        // Initialize tree with start position (current robot pose)
        nodes_x.push_back(current_x_);
        nodes_y.push_back(current_y_);
        parent.push_back(-1);
        int node_count = 1;
        // Add start node to device memory
        addNodeToDevice(current_x_, current_y_, 0);

        bool path_found = false;
        int goal_index = -1;

        for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
            // Sample a random point in the map (with goal bias)
            float sample_x, sample_y;
            if (((float) std::rand() / RAND_MAX) < GOAL_BIAS) {
                // Bias towards goal
                sample_x = goal_x_;
                sample_y = goal_y_;
            } else {
                // Uniform random sample within map bounds
                float min_x = map_origin_x_;
                float max_x = map_origin_x_ + map_width_ * map_resolution_;
                float min_y = map_origin_y_;
                float max_y = map_origin_y_ + map_height_ * map_resolution_;
                sample_x = min_x + ((float) std::rand() / RAND_MAX) * (max_x - min_x);
                sample_y = min_y + ((float) std::rand() / RAND_MAX) * (max_y - min_y);
            }

            // Find nearest node in the existing tree (GPU-accelerated)
            int nearest_idx = findNearestNode(sample_x, sample_y, node_count);
            if (nearest_idx < 0) {
                continue;
            }
            float nearest_x = nodes_x[nearest_idx];
            float nearest_y = nodes_y[nearest_idx];

            // Determine new node position in the direction of the sample (limit by STEP_SIZE)
            float dx = sample_x - nearest_x;
            float dy = sample_y - nearest_y;
            float dist = std::sqrt(dx*dx + dy*dy);
            float new_x = sample_x;
            float new_y = sample_y;
            if (dist > STEP_SIZE) {
                float scale = STEP_SIZE / dist;
                new_x = nearest_x + dx * scale;
                new_y = nearest_y + dy * scale;
            }

            // Collision check for edge from nearest node to new node (GPU-accelerated)
            if (checkCollision(nearest_x, nearest_y, new_x, new_y)) {
                continue;  // skip if this extension hits an obstacle
            }

            // Add new node to the tree
            nodes_x.push_back(new_x);
            nodes_y.push_back(new_y);
            parent.push_back(nearest_idx);
            int new_index = node_count;
            addNodeToDevice(new_x, new_y, new_index);
            node_count++;

            // Check if new node is close enough to goal
            float goal_dx = goal_x_ - new_x;
            float goal_dy = goal_y_ - new_y;
            float goal_dist = std::sqrt(goal_dx*goal_dx + goal_dy*goal_dy);
            if (goal_dist <= GOAL_THRESHOLD) {
                // Attempt to connect directly to goal if within threshold
                if (!checkCollision(new_x, new_y, goal_x_, goal_y_)) {
                    // Add goal as a final node
                    nodes_x.push_back(goal_x_);
                    nodes_y.push_back(goal_y_);
                    parent.push_back(new_index);
                    goal_index = node_count;
                    node_count++;
                    addNodeToDevice(goal_x_, goal_y_, goal_index);
                    path_found = true;
                    RCLCPP_INFO(this->get_logger(), "Path found in %d iterations, %d nodes.", iter+1, node_count);
                    break;
                }
            }
        }

        if (!path_found) {
            RCLCPP_WARN(this->get_logger(), "Path not found within the iteration limit.");
            return;
        }

        // Reconstruct path from goal to start using parent links
        std::vector<geometry_msgs::msg::Point> path_points;
        int idx = goal_index;
        while (idx != -1) {
            geometry_msgs::msg::Point p;
            p.x = nodes_x[idx];
            p.y = nodes_y[idx];
            p.z = 0.0;
            path_points.push_back(p);
            idx = parent[idx];
        }
        // Reverse the path to get start -> goal order
        std::reverse(path_points.begin(), path_points.end());

        // Create a visualization marker for the path (LINE_STRIP)
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = this->now();
        path_marker.ns = "gpu_rrt_path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.pose.orientation.w = 1.0;
        path_marker.pose.position.x = 0.0;
        path_marker.pose.position.y = 0.0;
        path_marker.pose.position.z = 0.0;
        // Line strip width
        path_marker.scale.x = 0.1;
        path_marker.scale.y = 0.1;
        path_marker.scale.z = 0.1;
        // Color (green)
        path_marker.color.a = 1.0;
        path_marker.color.r = 0.0;
        path_marker.color.g = 1.0;
        path_marker.color.b = 0.0;
        // Assign the computed path points
        path_marker.points = path_points;
        // Publish the marker
        marker_pub_->publish(path_marker);
    }

    // Subscribers and publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // Stored map and state data
    bool map_received_;
    bool odom_received_;
    int map_width_;
    int map_height_;
    float map_resolution_;
    float map_origin_x_;
    float map_origin_y_;
    std::vector<uint8_t> map_data_binary_;
    float current_x_;
    float current_y_;
    float goal_x_;
    float goal_y_;

    static const int MAX_NODES = 10000;
    static const int OCC_THRESHOLD = 50; // occupancy value (0-100) above which a cell is considered occupied
};

// Main entry point
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpuRrtNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
