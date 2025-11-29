#ifndef DYNAMIC_RRT_RRT_BASE_HPP
#define DYNAMIC_RRT_RRT_BASE_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace dynamic_rrt {

class RRTBase : public rclcpp::Node {
public:
    RRTBase(const std::string &node_name);
    virtual ~RRTBase() = default;

    RRTBase(const RRTBase &) = delete;
    RRTBase(RRTBase &&) = delete;
    RRTBase &operator=(const RRTBase &) = delete;
    RRTBase &operator=(RRTBase &&) = delete;

private:
};

} // namespace dynamic_rrt

#endif // DYNAMIC_RRT_RRT_BASE_HPP
