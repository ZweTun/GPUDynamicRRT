#include "rrt_cuda.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rrt_kernels.cuh"

namespace dynamic_rrt {

RRTCuda::RRTCuda()
    : RRTBase("dynamic_rrt_cuda") {}

auto RRTCuda::set_resolution(double resolution) -> void {
    // TODO
}

auto RRTCuda::plan_rrt(
    const Pose2D& start,
    const Point2D& goal,
    std::int32_t map_width,
    std::int32_t map_height,
    const std::vector<std::int8_t>& map_data,
    std::vector<Point2D>& local_waypoints
) -> void {
    // TODO
}

}  // namespace dynamic_rrt

auto main(int argc, char* argv[]) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dynamic_rrt::RRTCuda>());
    rclcpp::shutdown();
    return 0;
}
