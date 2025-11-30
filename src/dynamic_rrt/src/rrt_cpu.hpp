#ifndef DYNAMIC_RRT_RRT_CPU_HPP
#define DYNAMIC_RRT_RRT_CPU_HPP

#include <cstdint>
#include <vector>

#include "rrt_base.hpp"

namespace dynamic_rrt {

class RRTCpu final : public RRTBase {
public:
    RRTCpu();

private:
    auto plan_rrt(
        const Pose2D& start,
        const Point2D& goal,
        std::int32_t map_width,
        std::int32_t map_height,
        const std::vector<std::int8_t>& map_data,
        std::vector<Point2D>& local_waypoints
    ) -> void;
};

}  // namespace dynamic_rrt

#endif  // DYNAMIC_RRT_RRT_CPU_HPP
