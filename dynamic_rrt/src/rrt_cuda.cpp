#include "rrt_cuda.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rrt_kernels.hpp"

namespace dynamic_rrt {

RRTCuda::RRTCuda()
    : RRTBase("dynamic_rrt_cuda"),
      random_engine_(std::random_device()()) {
    // Declare parameters.
    this->declare_parameter<std::int64_t>("num_workers", 1);
    num_workers_ = static_cast<std::int32_t>(this->get_parameter("num_workers").as_int());
    this->declare_parameter<std::int64_t>("max_iterations", 2500);
    max_iterations_ = static_cast<std::int32_t>(this->get_parameter("max_iterations").as_int());
    this->declare_parameter<std::int64_t>("max_nodes_per_tree", 2500);
    max_nodes_per_tree_ =
        static_cast<std::int32_t>(this->get_parameter("max_nodes_per_tree").as_int());
    this->declare_parameter<std::int64_t>("max_sampling_attempts", 100);
    max_sampling_attempts_ =
        static_cast<std::int32_t>(this->get_parameter("max_sampling_attempts").as_int());
    this->declare_parameter<std::int64_t>("threads_per_block", 256);
    threads_per_block_ =
        static_cast<std::int32_t>(this->get_parameter("threads_per_block").as_int());

    this->declare_parameter<double>("sample_forward_min_m", 0.7);
    this->declare_parameter<double>("sample_forward_max_m", 5.0);
    this->declare_parameter<double>("sample_lateral_range_m", 5.0);
    this->declare_parameter<double>("sample_fallback_forward_min_m", 0.3);
    this->declare_parameter<double>("sample_fallback_forward_max_m", 1.0);
    this->declare_parameter<double>("steer_step_size_m", 0.5);
    this->declare_parameter<double>("goal_tolerance_m", 0.15);
}

auto RRTCuda::set_resolution(float resolution) -> void {
    sample_forward_min_cells_ =
        static_cast<float>(this->get_parameter("sample_forward_min_m").as_double() / resolution);
    sample_forward_max_cells_ =
        static_cast<float>(this->get_parameter("sample_forward_max_m").as_double() / resolution);
    sample_lateral_range_cells_ =
        static_cast<float>(this->get_parameter("sample_lateral_range_m").as_double() / resolution);
    sample_fallback_forward_min_cells_ = static_cast<float>(
        this->get_parameter("sample_fallback_forward_min_m").as_double() / resolution
    );
    sample_fallback_forward_max_cells_ = static_cast<float>(
        this->get_parameter("sample_fallback_forward_max_m").as_double() / resolution
    );
    steer_step_size_cells_ =
        static_cast<float>(this->get_parameter("steer_step_size_m").as_double() / resolution);
    goal_tolerance_cells_ =
        static_cast<float>(this->get_parameter("goal_tolerance_m").as_double() / resolution);
}

auto RRTCuda::plan_rrt(
    const Pose2D& start,
    const Point2D& goal,
    std::int32_t map_width,
    std::int32_t map_height,
    const std::vector<std::int8_t>& map_data
) -> std::vector<Point2D> {
    RRTStateBase state{};
    state.start = start;
    state.goal = goal;
    state.map = OccupancyGridView{map_data.data(), map_width, map_height};
    state.num_workers = num_workers_;
    state.max_iterations = max_iterations_;
    state.max_nodes_per_tree = max_nodes_per_tree_;
    state.max_sampling_attempts = max_sampling_attempts_;
    state.sample_forward_min = sample_forward_min_cells_;
    state.sample_forward_max = sample_forward_max_cells_;
    state.sample_lateral_range = sample_lateral_range_cells_;
    state.sample_fallback_forward_min = sample_fallback_forward_min_cells_;
    state.sample_fallback_forward_max = sample_fallback_forward_max_cells_;
    state.steer_step_size = steer_step_size_cells_;
    state.goal_tolerance = goal_tolerance_cells_;

    const auto seed = std::uniform_int_distribution<unsigned long long>()(random_engine_);
    return plan_rrt_cuda(state, threads_per_block_, seed, [this](const std::string& message) {
        RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    });
}

}  // namespace dynamic_rrt

auto main(int argc, char* argv[]) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dynamic_rrt::RRTCuda>());
    rclcpp::shutdown();
    return 0;
}
