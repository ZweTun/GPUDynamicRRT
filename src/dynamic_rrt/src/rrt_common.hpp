#ifndef DYNAMIC_RRT_RRT_COMMON_HPP
#define DYNAMIC_RRT_RRT_COMMON_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#ifdef __CUDACC__
    #define DYNAMIC_RRT_HOST_DEVICE __host__ __device__
#else
    #define DYNAMIC_RRT_HOST_DEVICE
#endif  // __CUDACC__

namespace dynamic_rrt {

struct Point2D {
    float x;
    float y;
};

struct Pose2D {
    Point2D position;
    float yaw;

    DYNAMIC_RRT_HOST_DEVICE auto forward_lateral(float forward, float lateral) const -> Point2D {
        const auto cos_yaw = ::cosf(this->yaw);
        const auto sin_yaw = ::sinf(this->yaw);
        return Point2D{
            this->position.x + forward * cos_yaw - lateral * sin_yaw,
            this->position.y + forward * sin_yaw + lateral * cos_yaw
        };
    }
};

struct OccupancyGridView {
    const std::int8_t* data;
    std::int32_t width;
    std::int32_t height;

    DYNAMIC_RRT_HOST_DEVICE auto is_point_free(const Point2D& point) const -> bool {
        const auto grid_x = static_cast<std::int32_t>(::floorf(point.x));
        const auto grid_y = static_cast<std::int32_t>(::floorf(point.y));
        if (grid_x < 0 || grid_x >= this->width || grid_y < 0 || grid_y >= this->height) {
            return false;
        }
        return this->data[grid_y * this->width + grid_x] == 0;
    }

    DYNAMIC_RRT_HOST_DEVICE auto is_segment_collision_free(const Point2D& start, const Point2D& end)
        const -> bool {
        const auto delta_x = static_cast<std::int32_t>(::ceilf(::fabsf(end.x - start.x)));
        const auto delta_y = static_cast<std::int32_t>(::ceilf(::fabsf(end.y - start.y)));
        const auto num_steps = delta_x > delta_y ? delta_x : delta_y;
        if (num_steps == 0) {
            return this->is_point_free(end);
        }
        for (std::int32_t i = 0; i <= num_steps; ++i) {
            const auto t = static_cast<float>(i) / static_cast<float>(num_steps);
            const auto point =
                Point2D{(1.0f - t) * start.x + t * end.x, (1.0f - t) * start.y + t * end.y};
            if (!this->is_point_free(point)) {
                return false;
            }
        }
        return true;
    }
};

struct TreeNode {
    Point2D position;
    std::int32_t parent_index;
};

struct Tree {
    TreeNode* nodes;
    std::int32_t size;
    std::int32_t capacity;

    DYNAMIC_RRT_HOST_DEVICE auto is_full() const -> bool {
        return this->size >= this->capacity;
    }

    DYNAMIC_RRT_HOST_DEVICE auto add_node(const Point2D& position, std::int32_t parent_index)
        -> void {
        this->nodes[this->size] = TreeNode{position, parent_index};
        ++this->size;
    }

    DYNAMIC_RRT_HOST_DEVICE auto get_nearest_node_index(const Point2D& point
    ) const -> std::int32_t {
        std::int32_t nearest_index = -1;
        auto nearest_distance_squared = std::numeric_limits<float>::max();
        for (std::int32_t index = 0; index < this->size; ++index) {
            auto& node = this->nodes[index];
            const auto delta_x = point.x - node.position.x;
            const auto delta_y = point.y - node.position.y;
            const auto distance_squared = delta_x * delta_x + delta_y * delta_y;
            if (distance_squared < nearest_distance_squared) {
                nearest_distance_squared = distance_squared;
                nearest_index = index;
            }
        }
        return nearest_index;
    }

    auto construct_path(std::int32_t goal_index) const -> std::vector<Point2D> {
        std::vector<Point2D> path;
        auto index = goal_index;
        while (index >= 0) {
            auto& node = this->nodes[index];
            path.push_back(node.position);
            index = node.parent_index;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};

struct RRTStateBase {
    Pose2D start;
    Point2D goal;
    OccupancyGridView grid;

    std::int32_t num_workers;
    std::int32_t max_iterations;
    std::int32_t max_nodes_per_tree;
    std::int32_t max_sampling_attempts;
    float sample_forward_min;
    float sample_forward_max;
    float sample_lateral_range;
    float sample_fallback_forward_min;
    float sample_fallback_forward_max;
    float steer_step_size;
    float goal_tolerance;

    Tree* trees;
    std::int32_t* goal_indices;

    template <typename Self>
    DYNAMIC_RRT_HOST_DEVICE static auto search(Self& self, unsigned int worker_index) -> void {
        auto& tree = self.trees[worker_index];
        tree.size = 0;
        tree.add_node(self.start.position, -1);
        for (std::int32_t i = 0; i < self.max_iterations && !tree.is_full(); ++i) {
            const auto sampled_point = Self::sample_point(self, worker_index);
            const auto nearest_index = tree.get_nearest_node_index(sampled_point);
            const auto& nearest_point = tree.nodes[nearest_index].position;
            const auto steered_point = self.steer_towards(nearest_point, sampled_point);
            if (!self.grid.is_segment_collision_free(nearest_point, steered_point)) {
                continue;
            }
            tree.add_node(steered_point, nearest_index);
            if (!self.is_goal_reached(steered_point)) {
                continue;
            }
            self.goal_indices[worker_index] = tree.size - 1;
            return;
        }
        self.goal_indices[worker_index] = -1;
    }

    template <typename Self>
    DYNAMIC_RRT_HOST_DEVICE static auto sample_point(Self& self, unsigned int worker_index)
        -> Point2D {
        for (std::int32_t i = 0; i < self.max_sampling_attempts; ++i) {
            const auto forward = self.sample_uniform_real(
                self.sample_forward_min, self.sample_forward_max, worker_index
            );
            const auto lateral = self.sample_uniform_real(
                -self.sample_lateral_range, self.sample_lateral_range, worker_index
            );
            const auto point = self.start.forward_lateral(forward, lateral);
            if (self.grid.is_point_free(point)) {
                return point;
            }
        }
        // If no valid point is found, use a shorter segment straight ahead without checking for
        // obstacles.
        const auto forward = self.sample_uniform_real(
            self.sample_fallback_forward_min, self.sample_fallback_forward_max, worker_index
        );
        return self.start.forward_lateral(forward, 0.0f);
    }

    DYNAMIC_RRT_HOST_DEVICE auto steer_towards(const Point2D& from, const Point2D& to) const
        -> Point2D {
        const auto delta_x = to.x - from.x;
        const auto delta_y = to.y - from.y;
        const auto distance = ::hypotf(delta_x, delta_y);
        if (distance <= this->steer_step_size) {
            return to;
        }
        const auto scale = this->steer_step_size / distance;
        return Point2D{from.x + delta_x * scale, from.y + delta_y * scale};
    }

    DYNAMIC_RRT_HOST_DEVICE auto is_goal_reached(const Point2D& point) const -> bool {
        const auto delta_x = point.x - this->goal.x;
        const auto delta_y = point.y - this->goal.y;
        const auto distance = ::hypotf(delta_x, delta_y);
        return distance <= this->goal_tolerance;
    }
};

}  // namespace dynamic_rrt

#endif  // DYNAMIC_RRT_RRT_COMMON_HPP
