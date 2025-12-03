#include "rrt_kernels.hpp"

#include <stdexcept>
#include <string>

#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <thrust/copy.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

namespace dynamic_rrt {

struct RRTStateCuda final : RRTStateBase {
    unsigned long long seed;
    curandStatePhilox4_32_10_t* random_states;

    __device__ auto sample_uniform_real(float a, float b, unsigned int worker_index) -> float {
        const auto uniform = curand_uniform(&this->random_states[worker_index]);
        return a * (1.0f - uniform) + b * uniform;
    }
};

__global__ auto rrt_kernel_cuda(RRTStateCuda state) -> void {
    const auto worker_index = blockIdx.x * blockDim.x + threadIdx.x;
    if (worker_index >= static_cast<unsigned int>(state.num_workers)) {
        return;
    }
    curand_init(state.seed, worker_index, 0, &state.random_states[worker_index]);
    RRTStateBase::search(state, worker_index);
}

auto plan_rrt_cuda(
    const RRTStateBase& base_state, std::int32_t threads_per_block, unsigned long long seed
) -> std::vector<Point2D> {
    thrust::device_vector<std::int8_t> device_map_data(
        base_state.map.data, base_state.map.data + base_state.map.width * base_state.map.height
    );
    thrust::device_vector<TreeNode> device_tree_nodes(
        base_state.num_workers * base_state.max_nodes_per_tree
    );
    thrust::device_vector<Tree> device_trees(base_state.num_workers);
    thrust::device_vector<std::int32_t> device_goal_indices(base_state.num_workers);
    thrust::device_vector<curandStatePhilox4_32_10_t> device_random_states(base_state.num_workers);

    RRTStateCuda cuda_state{};
    static_cast<RRTStateBase&>(cuda_state) = base_state;
    cuda_state.map.data = device_map_data.data().get();
    cuda_state.tree_nodes = device_tree_nodes.data().get();
    cuda_state.trees = device_trees.data().get();
    cuda_state.goal_indices = device_goal_indices.data().get();
    cuda_state.seed = seed;
    cuda_state.random_states = device_random_states.data().get();

    const auto num_blocks = (base_state.num_workers + threads_per_block - 1) / threads_per_block;
    rrt_kernel_cuda<<<num_blocks, threads_per_block>>>(cuda_state);
    const auto status = cudaDeviceSynchronize();
    if (status != cudaSuccess) {
        throw std::runtime_error(
            std::string("CUDA kernel execution failed: ") + cudaGetErrorString(status)
        );
    }

    thrust::host_vector<std::int32_t> goal_indices = device_goal_indices;
    for (std::int32_t worker_index = 0; worker_index < cuda_state.num_workers; ++worker_index) {
        const auto goal_index = goal_indices[worker_index];
        if (goal_index < 0) {
            continue;
        }
        Tree tree = device_trees[worker_index];
        thrust::host_vector<TreeNode> tree_nodes(
            device_tree_nodes.begin() + worker_index * base_state.max_nodes_per_tree,
            device_tree_nodes.begin() + (worker_index + 1) * base_state.max_nodes_per_tree
        );
        tree.nodes = tree_nodes.data();
        return tree.construct_path(goal_index);
    }
    return std::vector<Point2D>{};
}

}  // namespace dynamic_rrt
