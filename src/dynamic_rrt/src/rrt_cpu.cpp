#include "rrt_cpu.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace dynamic_rrt {

RRTCpu::RRTCpu()
    : RRTBase("dynamic_rrt_cpu") {}

} // namespace dynamic_rrt

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dynamic_rrt::RRTCpu>());
    rclcpp::shutdown();
    return 0;
}
