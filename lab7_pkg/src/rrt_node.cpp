#include "rrt/rrt.h"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    // Create and spin the RRT node
    auto rrt_node = std::make_shared<RRT>();
    rclcpp::spin(rrt_node);
    rclcpp::shutdown();
    return 0;
}
