#ifndef DYNAMIC_RRT_RRT_CUDA_HPP
#define DYNAMIC_RRT_RRT_CUDA_HPP

#include <cstdint>
#include <vector>

#include "rrt_base.hpp"

namespace dynamic_rrt {

class RRTCuda final : public RRTBase {
public:
    RRTCuda();

private:
    auto set_resolution(double resolution) -> void override;

    auto plan_rrt(
        const Pose2D& start,
        const Point2D& goal,
        std::int32_t map_width,
        std::int32_t map_height,
        const std::vector<std::int8_t>& map_data
    ) -> std::vector<Point2D> override;

    double resolution_;
};

}  // namespace dynamic_rrt

#endif  // DYNAMIC_RRT_RRT_CUDA_HPP
