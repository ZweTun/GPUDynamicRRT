#include "rrt_cuda.hpp"

#include <memory>

#include "cuda/rrt.h"
#include "rclcpp/rclcpp.hpp"
#include "rrt_kernels.cuh"

namespace dynamic_rrt {

RRTCuda::RRTCuda()
    : RRTBase("dynamic_rrt_cuda") {
    this->declare_parameter<int>("maxIter", 2500);
    this->declare_parameter<int>("maxNodes", 5000);
    this->declare_parameter<double>("maxStep", 0.5);
}

auto RRTCuda::set_resolution(double resolution) -> void {
    resolution_ = resolution;
}

auto RRTCuda::plan_rrt(
    const Pose2D& start,
    const Point2D& goal,
    std::int32_t map_width,
    std::int32_t map_height,
    const std::vector<std::int8_t>& map_data
) -> std::vector<Point2D> {
    OccupancyGrid grid{};
    grid.data = reinterpret_cast<uint8_t*>(const_cast<std::int8_t*>(map_data.data()));
    grid.width = static_cast<int>(map_width);
    grid.height = static_cast<int>(map_height);
    grid.resolution = 1.0f;  // Input map is already converted to cell units.
    grid.origin_x = 0.0f;
    grid.origin_y = 0.0f;
    const auto path = launchRRT(
        grid,
        static_cast<float>(start.position.x),
        static_cast<float>(start.position.y),
        static_cast<float>(start.yaw),
        static_cast<float>(goal.x),
        static_cast<float>(goal.y),
        static_cast<int>(this->get_parameter("maxIter").as_int()),
        static_cast<int>(this->get_parameter("maxNodes").as_int()),
        static_cast<float>(this->get_parameter("maxStep").as_double() / resolution_)
    );
    std::vector<Point2D> waypoints;
    waypoints.reserve(path.size());
    for (const auto& node : path) {
        waypoints.push_back(Point2D{static_cast<double>(node.x), static_cast<double>(node.y)});
    }
    if (!waypoints.empty()) {
        RCLCPP_INFO(this->get_logger(), "RRT found a path with %zu waypoints.", waypoints.size());
    } else {
        RCLCPP_WARN(this->get_logger(), "RRT failed to find a path.");
    }
    return waypoints;
}

}  // namespace dynamic_rrt

auto main(int argc, char* argv[]) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dynamic_rrt::RRTCuda>());
    rclcpp::shutdown();
    return 0;
}
