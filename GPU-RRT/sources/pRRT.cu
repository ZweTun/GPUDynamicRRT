#include "pRRT.h"

#include "rrt_types.h"
#include <vector>
#include <cmath>
#include "common.h"

#include <cuda_runtime.h>
#include "rrt.h"
#include <cuda.h>
#include <device_launch_parameters.h>
#include <thrust/version.h>
#include <thrust/random.h>
#include <unordered_set>

#include <algorithm>
#include <cstdint>
#include <device_functions.h>
#include <cuda_runtime_api.h>

// Returns true if the point (x,y) is free in the occupancy grid

__device__ bool isPointFreeGPU(const OccupancyGrid& grid, float x, float y) {
    int gx = (x - grid.origin_x) / grid.resolution;
    int gy = (y - grid.origin_y) / grid.resolution;
    if (gx < 0 || gx >= grid.width || gy < 0 || gy >= grid.height) {
        return false; // Out of bounds
    }
    int idx = gy * grid.width + gx;
    //0 means free space
    return grid.data[idx] == 0;
}


// Checks for collision along the segment [x1,y1]  [x2,y2]
__device__ bool checkCollisionGPU(const OccupancyGrid& grid, float x1, float y1, float x2, float y2) {
    int x_diff = abs(int(ceil((x2 - x1) / grid.resolution)));
    int y_diff = abs(int(ceil((y2 - y1) / grid.resolution)));

    int steps = -1;
    if (x_diff > y_diff) {
        steps = x_diff;
    }
    else {
        steps = y_diff;
    }
    if (steps == 0) {
        return !isPointFreeGPU(grid, x2, y2); // No movement
    }
    float dt = 1.0f / steps;
    float t = 0.0f;
    for (int i = 0; i <= steps; ++i) {
        float x = x1 + t * (x2 - x1);
        float y = y1 + t * (y2 - y1);
        if (!isPointFreeGPU(grid, x, y)) {
            return true; // Collision detected
        }
        t += dt;
    }
    return false; // No collision
}



// Steer by at most maxStep
__device__ TreeNode steerGPU(const TreeNode& from, const TreeNode& to, float maxStep) {
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist <= maxStep) {
        return { to.x, to.y, -1 };
    }
    else {
        float sx = from.x + (dx / dist) * maxStep;
        float sy = from.y + (dy / dist) * maxStep;
        return { sx, sy, -1 };
    }
}



__device__ TreeNode sampleFreeSpaceGPU(const OccupancyGrid& grid, int iter, float goalX, float goalY) {
    unsigned tid = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned seed = (tid * 1664525u) ^ (blockIdx.x * 1013904223u) ^ (unsigned)iter;

    thrust::minstd_rand rng(seed);
    thrust::uniform_real_distribution<float> dist01(0.0f, 1.0f);


    const int maxAttempts = 256;
    for (int attempt = 0; attempt < maxAttempts; ++attempt) {
        float rx = dist01(rng);
        float ry = dist01(rng);

        int cx = static_cast<int>(rx * grid.width);
        int cy = static_cast<int>(ry * grid.height);

        // clamp for safety
        if (cx < 0) cx = 0;
        if (cy < 0) cy = 0;
        if (cx >= grid.width) cx = grid.width - 1;
        if (cy >= grid.height) cy = grid.height - 1;

		// Convert to world coordinates
        float x = grid.origin_x + (cx + 0.5f) * grid.resolution;
        float y = grid.origin_y + (cy + 0.5f) * grid.resolution;

        if (isPointFreeGPU(grid, x, y)) {
            return { x, y, -1 };
        }
    }

    float cxw = grid.origin_x + 0.5f * grid.width * grid.resolution;
    float cyw = grid.origin_y + 0.5f * grid.height * grid.resolution;
    return { cxw, cyw, -1 };
}
// Goal check
__device__ bool isGoalGPU(float x, float y, float goalX, float goalY) {
    float dx = x - goalX;
    float dy = y - goalY;
	float thresh = 0.15f; // threshold
    return (sqrtf(dx * dx + dy * dy) < thresh);
}



// Shared memory per block
extern __shared__ unsigned char s_mem[]; // = buffer

__global__ void rrtSingleTreeKernel(
    OccupancyGrid grid,
    float goalX, float goalY,
    int maxIter, int maxNodes, float maxStep, 
    bool* d_goalReached,
    TreeNode* d_goal,
    TreeNode* d_tree,
    int* d_size, int* d_goalIdx
) {
    // Layout shared memory: [sampled][s_dist][s_idx]
    //Sampled new node 
    TreeNode* s_sampled = reinterpret_cast<TreeNode*>(s_mem);

	// Distance from sampled node to each node in the tree array
    float* s_dist = reinterpret_cast<float*>(s_sampled + 1);

	// Array of indices corresponding to local closest nodes in the tree
    int* s_idx = reinterpret_cast<int*>(s_dist + blockDim.x);

    for (int iter = 0; iter < maxIter; ++iter) {
        if (*d_goalReached) return;

        // One sample per block (thread 0 produces it)
        if (threadIdx.x == 0) {
            *s_sampled = sampleFreeSpaceGPU(grid, iter, goalX, goalY);
        }
        __syncthreads();

        int size = *d_size;
        if (size <= 0) continue;

        // Each thread scans a strided subset of the tree for local best
        float localBest = 1e30f;
        int localIdx = -1;
        TreeNode sampled = *s_sampled;
        for (int i = threadIdx.x; i < size; i += blockDim.x) {
            float dx = d_tree[i].x - sampled.x;
            float dy = d_tree[i].y - sampled.y;
            float dd = dx * dx + dy * dy;
            if (dd < localBest) { localBest = dd; localIdx = i; }
        }

        // Write locals to shared arrays
        s_dist[threadIdx.x] = localBest;
        s_idx[threadIdx.x] = localIdx;
        __syncthreads();

        // Reduce over threads to find the best 
        for (int offset = blockDim.x >> 1; offset > 0; offset >>= 1) {
            if (threadIdx.x < offset) {
                float d_other = s_dist[threadIdx.x + offset];
                int   i_other = s_idx[threadIdx.x + offset];
                if (d_other < s_dist[threadIdx.x]) {
                    s_dist[threadIdx.x] = d_other;
                    s_idx[threadIdx.x] = i_other;
                }
            }
            __syncthreads();
        }

        // Thread 0 handles insertion of new node
        if (threadIdx.x == 0) {
            int bestIdx = s_idx[0];
            if (bestIdx >= 0) {
                TreeNode nearest = d_tree[bestIdx];
                TreeNode proposed = steerGPU(nearest, sampled, maxStep);

				// 10% chance to sample goal directly
                unsigned seed = (threadIdx.x * 1337u) ^ (iter * 911u);
                thrust::default_random_engine rng(seed);
                thrust::uniform_real_distribution<float> dist01(0.0f, 1.0f);
                if (dist01(rng) < 0.1f) {
					proposed = { goalX, goalY, -1 };
                }

                proposed.parent = bestIdx;

				// Collision check
                bool collides = checkCollisionGPU(grid, nearest.x, nearest.y, proposed.x, proposed.y);
                if (!collides) {

					// Add the new node to the tree
                    int idx = atomicAdd(d_size, 1);
                    if (idx < maxNodes) {
                        d_tree[idx] = proposed;
                        if (isGoalGPU(proposed.x, proposed.y, goalX, goalY)) {
                            *d_goalReached = true;
                            *d_goal = proposed;
                            atomicExch(d_goalIdx, idx); // store goal index
                        }
                    }
                    else {
                        
                        atomicSub(d_size, 1);
                        return;
                    }
                }
            }
        }
        __syncthreads();
    }
}


using RRT::Common::PerformanceTimer;
PerformanceTimer& timerpRRT()
{
    static PerformanceTimer timer;
    return timer;
}


// Host starting point for GPU RRT
std::vector<TreeNode> gpuRRT(
    const OccupancyGrid& h_grid,
    float startX, float startY,
    float goalX, float goalY,
    int maxIter, int maxNodes, float maxStep
) {
    std::vector<TreeNode> tree(maxNodes);
    std::vector<TreeNode> path;

    // Initialize tree with start node
    tree[0] = { startX, startY, -1 };

    // Device grid
    OccupancyGrid d_grid;
    size_t gridBytes = static_cast<size_t>(h_grid.width) * h_grid.height * sizeof(uint8_t);
    cudaMalloc(&d_grid.data, gridBytes);
    cudaMemcpy(d_grid.data, h_grid.data, gridBytes, cudaMemcpyHostToDevice);
    d_grid.width = h_grid.width;
    d_grid.height = h_grid.height;
    d_grid.resolution = h_grid.resolution;
    d_grid.origin_x = h_grid.origin_x;
    d_grid.origin_y = h_grid.origin_y;

    // Device tree
    TreeNode* d_tree = nullptr;
    cudaMalloc(&d_tree, static_cast<size_t>(maxNodes) * sizeof(TreeNode));
    // copy first node to device
    cudaMemcpy(d_tree, tree.data(), sizeof(TreeNode), cudaMemcpyHostToDevice);

    // Device flags and counters
    bool* d_goalReached = nullptr;
    TreeNode* d_goal = nullptr;
    int* d_size = nullptr;
    int* d_goalIdx = nullptr; // goal node index on device
    cudaMalloc(&d_goalReached, sizeof(bool));
    cudaMalloc(&d_goal, sizeof(TreeNode));
    cudaMalloc(&d_size, sizeof(int));
    cudaMalloc(&d_goalIdx, sizeof(int));

    bool h_false = false;
    int h_size = 1;
    int h_neg1 = -1;
    cudaMemcpy(d_goalReached, &h_false, sizeof(bool), cudaMemcpyHostToDevice);
    cudaMemcpy(d_size, &h_size, sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_goalIdx, &h_neg1, sizeof(int), cudaMemcpyHostToDevice);

    // Launch
    int block = 128;          // power of two
    int gridSize = 64;
    size_t shmem =
        sizeof(TreeNode) +          // sampled
        block * sizeof(float) +     // s_dist
        block * sizeof(int);        // s_idx
    timerpRRT().startGpuTimer();
    rrtSingleTreeKernel << <gridSize, block, shmem >> > (
        d_grid, goalX, goalY,
        maxIter, maxNodes, maxStep,
        d_goalReached, d_goal, d_tree, d_size, d_goalIdx
        );
    cudaDeviceSynchronize();

    // Fetch results
    bool h_goalReached = false;
    cudaMemcpy(&h_goalReached, d_goalReached, sizeof(bool), cudaMemcpyDeviceToHost);
    cudaMemcpy(&h_size, d_size, sizeof(int), cudaMemcpyDeviceToHost);

    // Copy back the whole tree that was filled
    if (h_size > 0) {
        cudaMemcpy(tree.data(), d_tree, static_cast<size_t>(h_size) * sizeof(TreeNode), cudaMemcpyDeviceToHost);
    }

    if (h_goalReached) {
       
        int h_goalIdx = -1;
        cudaMemcpy(&h_goalIdx, d_goalIdx, sizeof(int), cudaMemcpyDeviceToHost);

		// Reconstruct path
        if (h_goalIdx >= 0 && h_goalIdx < h_size) {
            int cur = h_goalIdx;
            while (cur != -1) {
                path.push_back(tree[cur]);
                cur = tree[cur].parent;
            }
            std::reverse(path.begin(), path.end());
        }
       
    }

    timerpRRT().endGpuTimer();

    // Cleanup
    cudaFree(d_grid.data);
    cudaFree(d_tree);
    cudaFree(d_goalReached);
    cudaFree(d_goal);
    cudaFree(d_size);
    cudaFree(d_goalIdx);

    return path;
}


