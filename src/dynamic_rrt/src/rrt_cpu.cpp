#include "rrt_cpu.hpp"

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace dynamic_rrt {

auto RRTStateCpu::sample_uniform_real(float a, float b, unsigned int worker_index) const -> float {
    std::uniform_real_distribution distribution(a, b);
    return distribution(this->random_engines[worker_index]);
}

RRTCpu::RRTCpu()
    : RRTBase("dynamic_rrt_cpu"),
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

    this->declare_parameter<double>("sample_forward_min_m", 0.7);
    this->declare_parameter<double>("sample_forward_max_m", 5.0);
    this->declare_parameter<double>("sample_lateral_range_m", 5.0);
    this->declare_parameter<double>("sample_fallback_forward_min_m", 0.3);
    this->declare_parameter<double>("sample_fallback_forward_max_m", 1.0);
    this->declare_parameter<double>("steer_step_size_m", 0.5);
    this->declare_parameter<double>("goal_tolerance_m", 0.15);
}

auto RRTCpu::set_resolution(float resolution) -> void {
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

auto RRTCpu::plan_rrt(
    const Pose2D& start,
    const Point2D& goal,
    std::int32_t map_width,
    std::int32_t map_height,
    const std::vector<std::int8_t>& map_data
) -> std::vector<Point2D> {
    std::vector<TreeNode> tree_nodes(num_workers_ * max_nodes_per_tree_);
    std::vector<Tree> trees;
    trees.reserve(num_workers_);
    for (std::int32_t i = 0; i < num_workers_; ++i) {
        trees.push_back(Tree{&tree_nodes[i * max_nodes_per_tree_], 0, max_nodes_per_tree_});
    }

    std::vector<std::int32_t> goal_indices(num_workers_, -1);

    std::vector<std::mt19937> random_engines;
    random_engines.reserve(num_workers_);
    std::uniform_int_distribution<std::mt19937::result_type> seed_distribution;
    for (std::int32_t i = 0; i < num_workers_; ++i) {
        const auto seed = seed_distribution(random_engine_);
        random_engines.emplace_back(seed);
    }

    RRTStateCpu state{};
    state.start = start;
    state.goal = goal;
    state.grid = OccupancyGridView{map_data.data(), map_width, map_height};
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
    state.trees = trees.data();
    state.goal_indices = goal_indices.data();
    state.random_engines = random_engines.data();

    std::vector<std::thread> workers;
    workers.reserve(num_workers_);
    for (std::int32_t worker_index = 0; worker_index < num_workers_; ++worker_index) {
        workers.emplace_back([&state, worker_index]() {
            RRTStateCpu::search(state, static_cast<unsigned int>(worker_index));
        });
    }
    for (auto& worker : workers) {
        worker.join();
    }

    for (std::int32_t worker_index = 0; worker_index < num_workers_; ++worker_index) {
        const auto goal_index = goal_indices[worker_index];
        if (goal_index >= 0) {
            return trees[worker_index].construct_path(goal_index);
        }
    }
    return std::vector<Point2D>{};
}

}  // namespace dynamic_rrt

auto main(int argc, char* argv[]) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dynamic_rrt::RRTCpu>());
    rclcpp::shutdown();
    return 0;
}
