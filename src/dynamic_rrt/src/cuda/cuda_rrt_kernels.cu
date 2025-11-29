#include "cuda_rrt.hpp"

#if !defined(DYNAMIC_RRT_HAS_CUDA) || (DYNAMIC_RRT_HAS_CUDA != 1)
// This translation unit should only be compiled when CUDA support is available.
// If it gets built without CUDA, force a compile-time error so the build setup can be fixed.
#error "cuda_rrt_kernels.cu compiled without CUDA support"
#endif

#include <cuda_runtime.h>

#include <thrust/random.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace dynamic_rrt {
namespace cuda {
namespace {

struct DeviceTreeNode {
    float x;
    float y;
    int parent;
};

struct DeviceOccupancyGrid {
    const uint8_t *data;
    int width;
    int height;
    float resolution;
    float origin_x;
    float origin_y;
};

__device__ float deviceDistanceSquared(float x1, float y1, float x2, float y2) {
    const float dx = x1 - x2;
    const float dy = y1 - y2;
    return dx * dx + dy * dy;
}

__device__ bool deviceIsPointFree(const DeviceOccupancyGrid &grid, float x, float y) {
    const int gx = static_cast<int>((x - grid.origin_x) / grid.resolution);
    const int gy = static_cast<int>((y - grid.origin_y) / grid.resolution);
    if (gx < 0 || gx >= grid.width || gy < 0 || gy >= grid.height) {
        return false;
    }
    const int idx = gy * grid.width + gx;
    return grid.data[idx] == 0;
}

__device__ bool deviceIsGoal(float x, float y, float gx, float gy, float goal_threshold_sq) {
    return deviceDistanceSquared(x, y, gx, gy) <= goal_threshold_sq;
}

__device__ int deviceMaxInt(int a, int b) {
    return (a > b) ? a : b;
}

__device__ DeviceTreeNode deviceSteer(const DeviceTreeNode &nearest,
                                      const DeviceTreeNode &target,
                                      float max_step) {
    const float dx = target.x - nearest.x;
    const float dy = target.y - nearest.y;
    const float dist = sqrtf(dx * dx + dy * dy);
    if (dist <= max_step) {
        return DeviceTreeNode{target.x, target.y, -1};
    }
    if (dist < 1e-6f) {
        return DeviceTreeNode{nearest.x, nearest.y, nearest.parent};
    }
    const float scale = max_step / dist;
    return DeviceTreeNode{nearest.x + dx * scale, nearest.y + dy * scale, nearest.parent};
}

__device__ int deviceNearest(const DeviceTreeNode *tree, int tree_size, float x, float y) {
    float best_dist = 1e20f;
    int best_index = 0;
    for (int i = 0; i < tree_size; ++i) {
        const float dist = deviceDistanceSquared(tree[i].x, tree[i].y, x, y);
        if (dist < best_dist) {
            best_dist = dist;
            best_index = i;
        }
    }
    return best_index;
}

__device__ bool deviceCollision(const DeviceOccupancyGrid &grid,
                                const DeviceTreeNode &from,
                                const DeviceTreeNode &to) {
    const float dx = to.x - from.x;
    const float dy = to.y - from.y;
    const float distance = sqrtf(dx * dx + dy * dy);
    const int steps = deviceMaxInt(1, static_cast<int>(ceilf(distance / grid.resolution)));

    for (int i = 0; i <= steps; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(steps);
        const float x = from.x + t * dx;
        const float y = from.y + t * dy;
        if (!deviceIsPointFree(grid, x, y)) {
            return true;
        }
    }
    return false;
}

__device__ DeviceTreeNode deviceSample(const DeviceOccupancyGrid &grid,
                                       int iteration,
                                       thrust::default_random_engine &rng) {
    thrust::uniform_real_distribution<float> dist01(0.0f, 1.0f);

    const int max_attempts = 1000;
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        (void)iteration;
        const float rx = dist01(rng);
        const float ry = dist01(rng);
        const float x = grid.origin_x + rx * static_cast<float>(grid.width) * grid.resolution;
        const float y = grid.origin_y + ry * static_cast<float>(grid.height) * grid.resolution;
        if (deviceIsPointFree(grid, x, y)) {
            return DeviceTreeNode{x, y, -1};
        }
    }

    const float cx = grid.origin_x + 0.5f * static_cast<float>(grid.width) * grid.resolution;
    const float cy = grid.origin_y + 0.5f * static_cast<float>(grid.height) * grid.resolution;
    return DeviceTreeNode{cx, cy, -1};
}

__global__ void kernRrt(int max_iterations,
                        int max_nodes,
                        int num_threads,
                        DeviceOccupancyGrid grid,
                        float start_x,
                        float start_y,
                        float goal_x,
                        float goal_y,
                        float goal_threshold_sq,
                        float max_step,
                        DeviceTreeNode *all_trees,
                        int *goal_indices,
                        int *tree_sizes) {
    const int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_threads) {
        return;
    }

    DeviceTreeNode *tree = &all_trees[tid * max_nodes];
    int size = 1;
    tree[0] = DeviceTreeNode{start_x, start_y, -1};
    goal_indices[tid] = -1;

    thrust::default_random_engine rng(tid * 2654435761u);

    for (int iter = 1; iter < max_iterations && size < max_nodes; ++iter) {
        DeviceTreeNode random_node = deviceSample(grid, iter, rng);
        const int nearest_index = deviceNearest(tree, size, random_node.x, random_node.y);
        DeviceTreeNode steered = deviceSteer(tree[nearest_index], random_node, max_step);
        steered.parent = nearest_index;

        if (deviceCollision(grid, tree[nearest_index], steered)) {
            continue;
        }

        const int slot = size++;
        tree[slot] = steered;

        if (deviceIsGoal(steered.x, steered.y, goal_x, goal_y, goal_threshold_sq)) {
            goal_indices[tid] = slot;
            break;
        }
    }

    tree_sizes[tid] = size;
}

template <typename T>
struct DeviceBufferDeleter {
    void operator()(T *ptr) const noexcept {
        if (ptr != nullptr) {
            cudaFree(ptr);
        }
    }
};

template <typename T>
using DevicePtr = std::unique_ptr<T, DeviceBufferDeleter<T>>;

inline void cudaCheck(cudaError_t error, const char *message) {
    if (error != cudaSuccess) {
        throw std::runtime_error(std::string(message) + ": " + cudaGetErrorString(error));
    }
}

} // namespace

CudaPlanResult runCudaRrt(const nav_msgs::msg::OccupancyGrid &map,
                          const TreeNode &start,
                          const std::array<double, 2> &goal,
                          int max_iterations,
                          double max_step,
                          double goal_threshold) {
    if (map.data.empty() || map.info.width == 0 || map.info.height == 0) {
        return {};
    }

    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    const std::size_t cell_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);

    std::vector<uint8_t> host_data(cell_count, 0u);
    for (std::size_t i = 0; i < cell_count; ++i) {
        const int8_t value = map.data[i];
        host_data[i] = (value > 0) ? 100u : 0u;
    }

    DevicePtr<uint8_t> d_grid_data;
    uint8_t *raw_grid_ptr = nullptr;
    cudaCheck(cudaMalloc(reinterpret_cast<void **>(&raw_grid_ptr), cell_count * sizeof(uint8_t)),
              "Failed to allocate device grid data");
    d_grid_data.reset(raw_grid_ptr);
    cudaCheck(cudaMemcpy(d_grid_data.get(), host_data.data(), cell_count * sizeof(uint8_t),
                         cudaMemcpyHostToDevice),
              "Failed to copy grid data to device");

    DeviceOccupancyGrid device_grid{};
    device_grid.data = static_cast<const uint8_t *>(d_grid_data.get());
    device_grid.width = width;
    device_grid.height = height;
    device_grid.resolution = static_cast<float>(map.info.resolution);
    device_grid.origin_x = static_cast<float>(map.info.origin.position.x);
    device_grid.origin_y = static_cast<float>(map.info.origin.position.y);

    const int num_threads = 512;
    const int max_nodes = std::max(2, std::min(max_iterations + 1, 8192));

    DevicePtr<DeviceTreeNode> d_all_trees;
    DeviceTreeNode *raw_all_trees = nullptr;
    cudaCheck(cudaMalloc(reinterpret_cast<void **>(&raw_all_trees),
                         static_cast<std::size_t>(num_threads) * max_nodes * sizeof(DeviceTreeNode)),
              "Failed to allocate device tree buffer");
    d_all_trees.reset(raw_all_trees);

    DevicePtr<int> d_goal_indices;
    int *raw_goal_indices = nullptr;
    cudaCheck(cudaMalloc(reinterpret_cast<void **>(&raw_goal_indices),
                         static_cast<std::size_t>(num_threads) * sizeof(int)),
              "Failed to allocate device goal indices");
    d_goal_indices.reset(raw_goal_indices);

    DevicePtr<int> d_tree_sizes;
    int *raw_tree_sizes = nullptr;
    cudaCheck(cudaMalloc(reinterpret_cast<void **>(&raw_tree_sizes),
                         static_cast<std::size_t>(num_threads) * sizeof(int)),
              "Failed to allocate device tree sizes");
    d_tree_sizes.reset(raw_tree_sizes);

    const dim3 block_dim(128);
    const dim3 grid_dim((num_threads + block_dim.x - 1) / block_dim.x);

    cudaCheck(cudaDeviceSynchronize(), "Pre-kernel device sync failed");

    kernRrt<<<grid_dim, block_dim>>>(max_iterations,
                                     max_nodes,
                                     num_threads,
                                     device_grid,
                                     static_cast<float>(start.x),
                                     static_cast<float>(start.y),
                                     static_cast<float>(goal[0]),
                                     static_cast<float>(goal[1]),
                                     static_cast<float>(goal_threshold * goal_threshold),
                                     static_cast<float>(std::max(max_step, 1e-3)),
                                     d_all_trees.get(),
                                     d_goal_indices.get(),
                                     d_tree_sizes.get());

    cudaCheck(cudaGetLastError(), "RRT kernel launch failed");
    cudaCheck(cudaDeviceSynchronize(), "RRT kernel execution failed");

    std::vector<DeviceTreeNode> host_all_trees(static_cast<std::size_t>(num_threads) * max_nodes);
    std::vector<int> host_goal_indices(num_threads, -1);
    std::vector<int> host_tree_sizes(num_threads, 0);

    cudaCheck(cudaMemcpy(host_all_trees.data(), d_all_trees.get(),
                         host_all_trees.size() * sizeof(DeviceTreeNode), cudaMemcpyDeviceToHost),
              "Failed to copy tree buffer to host");
    cudaCheck(cudaMemcpy(host_goal_indices.data(), d_goal_indices.get(),
                         host_goal_indices.size() * sizeof(int), cudaMemcpyDeviceToHost),
              "Failed to copy goal indices to host");
    cudaCheck(cudaMemcpy(host_tree_sizes.data(), d_tree_sizes.get(),
                         host_tree_sizes.size() * sizeof(int), cudaMemcpyDeviceToHost),
              "Failed to copy tree sizes to host");

    const int winning_tid = [&]() {
        for (int tid = 0; tid < num_threads; ++tid) {
            if (host_goal_indices[tid] >= 0) {
                return tid;
            }
        }
        return -1;
    }();

    CudaPlanResult result;

    if (winning_tid >= 0) {
        const int goal_index = host_goal_indices[winning_tid];
        int tree_size = host_tree_sizes[winning_tid];
        if (tree_size < 1) {
            tree_size = 1;
        } else if (tree_size > max_nodes) {
            tree_size = max_nodes;
        }

        const DeviceTreeNode *tree_base = host_all_trees.data() + winning_tid * max_nodes;
        result.tree.reserve(tree_size);
        for (int i = 0; i < tree_size; ++i) {
            const DeviceTreeNode &node = tree_base[i];
            result.tree.push_back(TreeNode{static_cast<double>(node.x),
                                           static_cast<double>(node.y),
                                           node.parent});
        }

        if (goal_index >= 0 && goal_index < tree_size) {
            std::vector<TreeNode> reversed_path;
            int current = goal_index;
            std::unordered_set<int> visited;
            reversed_path.reserve(tree_size);
            while (current >= 0 && current < tree_size && visited.find(current) == visited.end()) {
                visited.insert(current);
                reversed_path.push_back(result.tree[current]);
                current = result.tree[current].parent;
            }
            std::reverse(reversed_path.begin(), reversed_path.end());
            result.path = std::move(reversed_path);
        }
    }

    if (result.path.empty()) {
        result.tree.clear();
    }

    return result;
}

} // namespace cuda
} // namespace dynamic_rrt
