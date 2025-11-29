#include "rrt_cuda.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rrt_kernels.cuh"

namespace dynamic_rrt {

RRTCuda::RRTCuda()
    : RRTBase("dynamic_rrt_cuda") {}

} // namespace dynamic_rrt

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dynamic_rrt::RRTCuda>());
    rclcpp::shutdown();
    return 0;
}
