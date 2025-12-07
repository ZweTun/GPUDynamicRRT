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
__device__ bool worldToGridGPU(const OccupancyGrid& grid,
    float x, float y,
    int& gx, int& gy) {
    gx = (int)floorf((x - grid.origin_x) / grid.resolution);
    gy = (int)floorf((y - grid.origin_y) / grid.resolution);

    if (gx < 0 || gx >= grid.width || gy < 0 || gy >= grid.height) {
        return false;
    }
    return true;
}

__device__ bool isCellFreeGPU(const OccupancyGrid& grid, int gx, int gy) {
    if (gx < 0 || gx >= grid.width || gy < 0 || gy >= grid.height) {
        return false; // treat out-of-bounds as collision
    }
    int idx = gy * grid.width + gx;
    return (grid.data[idx] == 0);
}

__device__ bool isPointFreeGPU(const OccupancyGrid& grid, float x, float y) {

    int gx = (int)floorf((x - grid.origin_x) / grid.resolution);
    int gy = (int)floorf((y - grid.origin_y) / grid.resolution);

    if (gx < 0 || gx >= grid.width || gy < 0 || gy >= grid.height) {
        return false; // Out of bounds
    }
    int idx = gy * grid.width + gx;
    //0 means free space
    return grid.data[idx] == 0;
}


//// Checks for collision along the segment [x1,y1]  [x2,y2]
__device__ bool checkCollisionGPU(const OccupancyGrid& grid,
    float x1, float y1,
    float x2, float y2) {
    int gx0, gy0, gx1, gy1;
    if (!worldToGridGPU(grid, x1, y1, gx0, gy0)) {
        return true;  // start outside map  collision
    }
    if (!worldToGridGPU(grid, x2, y2, gx1, gy1)) {
        return true;  // end outside map  collision
    }

    // Bresenham integer line from (gx0, gy0) to (gx1, gy1)
    int x = gx0;
    int y = gy0;
    int dx = abs(gx1 - gx0);
    int sx = (gx0 < gx1) ? 1 : -1;
    int dy = -abs(gy1 - gy0);
    int sy = (gy0 < gy1) ? 1 : -1;
    int err = dx + dy;

    while (true) {
        if (!isCellFreeGPU(grid, x, y)) {
            return true;  // hit obstacle cell
        }
        if (x == gx1 && y == gy1) break;

        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }

    return false;  // no collision along the discrete line
}

// Goal check
__device__ bool isGoalGPU(float x, float y, float goalX, float goalY) {
    float dx = x - goalX;
    float dy = y - goalY;
    float thresh = 0.15f; // threshold
    return (sqrtf(dx * dx + dy * dy) < thresh);
}



/// Steer by at most maxStep
__device__ TreeNode steerGPU(const TreeNode& from, const TreeNode& to, float maxStep) {
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist <= maxStep) {
        return to;
    }
    else {

        return { from.x + (dx / dist) * maxStep, from.y + (dy / dist) * maxStep, -1 };
    }
}




__device__ TreeNode sampleFreeSpaceGPU(const OccupancyGrid& grid, int iter, float goalX, float goalY) {
    unsigned tid = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned seed = (tid * 1664525u) ^ (blockIdx.x * 1013904223u) ^ (unsigned)iter;

    thrust::minstd_rand rng(seed);
    thrust::uniform_real_distribution<float> dist01(0.0f, 1.0f);


    const int maxAttempts = 1000;
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

	//Center fallback
    float cxw = (grid.origin_x + grid.width) * 0.5;
	float cyw = (grid.origin_y + grid.height) * 0.5;
    return { cxw, cyw, -1 };


}



__device__ void nearestNeighborGPU(
    const TreeNode& node,
    const TreeNode* d_tree,
    int size,
    int& outIdx,
    float& outDist
) {
    float localBest = 1e30f;
    int localIdx = -1;
    for (int i = threadIdx.x; i < size; i += blockDim.x) {
        float dx = d_tree[i].x - node.x;
        float dy = d_tree[i].y - node.y;
        float dd = dx * dx + dy * dy;
        if (dd < localBest) { localBest = dd; localIdx = i; }
    }

    // Write locals to shared arrays
    outDist = localBest;
    outIdx = localIdx;
}

__device__ void distReductionGPU(
    float* s_dist,
    int* s_idx
) {
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
}


// Compute vector between two tree nodes
__device__ void computeVectorGPU(
    const TreeNode& from,
    const TreeNode& to,
    float& out_dx,
    float& out_dy,
    float& out_dist
) {
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    float dist = sqrtf(dx * dx + dy * dy);
    out_dx = dx;
    out_dy = dy;
    out_dist = dist;
}



// Shared memory per block
extern __shared__ unsigned char s_mem[]; 


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

        // Thread 0 handles insertion of new nod
        if (threadIdx.x == 0) {
            int bestIdx = s_idx[0];
            if (bestIdx >= 0) {
                TreeNode nearest = d_tree[bestIdx];

                unsigned seed = (threadIdx.x * 1337u) ^ (iter * 911u);
                thrust::default_random_engine rng(seed);
                thrust::uniform_real_distribution<float> dist01(0.0f, 1.0f);

                TreeNode proposed;
                if (dist01(rng) < 0.1f) {
                    // 10% bias
                    TreeNode goalNode = { goalX, goalY, -1 };
                    proposed = goalNode;
                }
                else {
                    // normal RRT step toward random sample
                    proposed = steerGPU(nearest, sampled, maxStep);
                }

                proposed.parent = bestIdx;

                bool collides = checkCollisionGPU(grid, nearest.x, nearest.y,
                    proposed.x, proposed.y);

                if (!collides) {
                    int idx = atomicAdd(d_size, 1);
                    if (idx < maxNodes) {
                        d_tree[idx] = proposed;
                        if (isGoalGPU(proposed.x, proposed.y, goalX, goalY)) {
                            proposed.parent = bestIdx;     
                            *d_goalReached = true;
                            *d_goal = proposed;
                            atomicExch(d_goalIdx, idx);
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





// SINGLE TREE HOST 
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
 
	// Initialize device tree to zero
    cudaMemset(d_tree, 0, maxNodes * sizeof(TreeNode));
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

















//__global__ void rrtDoubleTreeKernel(
//    OccupancyGrid grid,
//    float goalX, float goalY,
//    int maxIter, int maxNodes, float maxStep,
//    bool* d_goalReached,
//    TreeNode* d_goal,
//    TreeNode* d_tree,
//    int* d_size, int* d_goalIdx, TreeNode* d_otree, int* d_osize, int* d_ogoalIdx
//) {
//    // Layout shared memory: [sampled][s_dist][s_idx]
//    TreeNode* s_sampled = reinterpret_cast<TreeNode*>(s_mem);
//    float* s_dist = reinterpret_cast<float*>(s_sampled + 1);
//    int* s_idx = reinterpret_cast<int*>(s_dist + blockDim.x);
//
//    // per-block control
//    //__shared__ int prevTotal;
//    //__shared__ int stagnCount;
//    //__shared__ int stopFlag;           // 0 = continue, 1 = stop requested
//    //const int STAGNATION_LIMIT = 512;
//
//    //if (threadIdx.x == 0) {
//    //    prevTotal = -1;
//    //    stagnCount = 0;
//    //    stopFlag = 0;
//    //}
//    __syncthreads();
//
//    for (int iter = 0; iter < maxIter; ++iter) {
//        // cooperative early exit
//        if (*d_goalReached) break;
//        /*      if (stopFlag) break;*/
//
//              // pick smaller / larger
//        TreeNode* smallerTree;
//        TreeNode* largerTree;
//        int* smallerSize;
//        int* largerSize;
//        int* smallerGoalIdx;
//        int* largerGoalIdx;
//
//        if (*d_size < *d_osize) {
//            smallerTree = d_tree; largerTree = d_otree;
//            smallerSize = d_size; largerSize = d_osize;
//            smallerGoalIdx = d_goalIdx; largerGoalIdx = d_ogoalIdx;
//        }
//        else {
//            smallerTree = d_otree; largerTree = d_tree;
//            smallerSize = d_osize; largerSize = d_size;
//            smallerGoalIdx = d_ogoalIdx; largerGoalIdx = d_goalIdx;
//        }
//
//        // one sample per block
//        if (threadIdx.x == 0) {
//            *s_sampled = sampleFreeSpaceGPU(grid, iter, goalX, goalY);
//        }
//        __syncthreads();
//
//        // nearest in smallerTree (all threads participate)
//        nearestNeighborGPU(*s_sampled, smallerTree, *smallerSize, s_idx[threadIdx.x], s_dist[threadIdx.x]);
//        __syncthreads();
//
//        // reduce to get best index/dist (writes into s_idx[0]/s_dist[0])
//        distReductionGPU(s_dist, s_idx);
//        __syncthreads();
//
//        // --- Thread 0 prepares insertion (steer + collision + allocate) ---
//        __shared__ int s_inserted_idx;
//        __shared__ int s_proposed_valid; // 0/1
//        if (threadIdx.x == 0) {
//            s_inserted_idx = -1;
//            s_proposed_valid = 0;
//            int bestIdx = s_idx[0];
//            if (bestIdx >= 0) {
//                TreeNode nearest = smallerTree[bestIdx];
//                TreeNode proposed = steerGPU(nearest, *s_sampled, maxStep);
//
//                // 10% goal-bias — device-callable RNG
//
//                unsigned seed = (unsigned)((unsigned)threadIdx.x * 1664525u ^ (unsigned)iter * 1013904223u);
//                thrust::minstd_rand rng(seed);
//                thrust::uniform_real_distribution<float> dist01(0.0f, 1.0f);
//                if (dist01(rng) < 0.1f) {
//                    // steer toward goal but keep maxStep clipping (do not assign raw goal)
//                    TreeNode goalNode = { goalX, goalY, -1 };
//                    proposed = steerGPU(nearest, goalNode, maxStep);
//                }
//
//                proposed.parent = bestIdx;
//
//                // collision check
//                if (!checkCollisionGPU(grid, nearest.x, nearest.y, proposed.x, proposed.y)) {
//                    // publish proposed into shared slot for NN step
//                    s_sampled[0] = proposed;
//
//                    // allocate index atomically and write node (thread 0 writes)
//                    int allocIdx = atomicAdd(smallerSize, 1);
//                    if (allocIdx < maxNodes) {
//                        smallerTree[allocIdx] = proposed;
//                        s_inserted_idx = allocIdx;
//                        s_proposed_valid = 1;
//
//                        // check goal reached
//                        if (isGoalGPU(proposed.x, proposed.y, goalX, goalY)) {
//                            *d_goalReached = true;
//                            *d_goal = proposed;
//                            atomicExch(smallerGoalIdx, allocIdx);
//                        }
//                    }
//                    else {
//                        // rollback increment already happened; decrement and mark invalid
//                        atomicSub(smallerSize, 1);
//                        s_inserted_idx = -1;
//                        s_proposed_valid = 0;
//                    }
//                }
//            }
//        }
//        // make shared state visible
//        __syncthreads();
//
//        // --- Collective nearest-neighbor on the larger tree using the shared proposed ---
//        if (s_proposed_valid) {
//            int otherSize = *largerSize; // racy but acceptable
//            nearestNeighborGPU(s_sampled[0], largerTree, otherSize, s_idx[threadIdx.x], s_dist[threadIdx.x]);
//        }
//        else {
//            // Initialize per-thread slots so reduction is safe
//            s_dist[threadIdx.x] = 1e30f;
//            s_idx[threadIdx.x] = -1;
//        }
//        __syncthreads();
//
//        // reduce to find nearest on larger tree
//        distReductionGPU(s_dist, s_idx);
//        __syncthreads();
//
//        // thread 0 handles the connection-extension step (if there is a valid proposed)
//        if (threadIdx.x == 0 && s_proposed_valid) {
//            int otherBestIdx = s_idx[0];
//            float otherBestDist = s_dist[0];
//            if (otherBestIdx >= 0) {
//                TreeNode otherNearest = largerTree[otherBestIdx];
//                TreeNode proposed = s_sampled[0]; // shared proposed
//                float vx = proposed.x - otherNearest.x;
//                float vy = proposed.y - otherNearest.y;
//                float dist = sqrtf(vx * vx + vy * vy);
//                int nSteps = static_cast<int>(ceilf(dist / maxStep));
//                if (nSteps > 0 && dist > 0.0f) { vx /= dist; vy /= dist; }
//
//                // extend step-by-step from the inserted node (s_inserted_idx)
//                int prevIdx = s_inserted_idx;
//                TreeNode curNode = proposed;
//                for (int step = 0; step < nSteps; ++step) {
//                    float nx = curNode.x + vx * maxStep;
//                    float ny = curNode.y + vy * maxStep;
//
//                    if (checkCollisionGPU(grid, curNode.x, curNode.y, nx, ny)) break;
//
//                    TreeNode stepNode;
//                    stepNode.x = nx;
//                    stepNode.y = ny;
//                    stepNode.parent = prevIdx;
//
//                    int newIdx = atomicAdd(smallerSize, 1);
//                    if (newIdx >= maxNodes) break;
//                    smallerTree[newIdx] = stepNode;
//                    prevIdx = newIdx;
//                    curNode.x = nx;
//                    curNode.y = ny;
//                }
//
//                float remaining = dist - nSteps * maxStep;
//                if (remaining <= maxStep + 1e-6f) {
//                    *d_goalReached = true;
//                    *d_goal = proposed;
//                    if (smallerTree == d_tree) {
//                        atomicExch(d_goalIdx, prevIdx);
//                        atomicExch(d_ogoalIdx, otherBestIdx);
//                    }
//                    else {
//                        atomicExch(d_ogoalIdx, prevIdx);
//                        atomicExch(d_goalIdx, otherBestIdx);
//                    }
//                }
//            }
//        }
//
//        // final sync before next iteration
//        //__syncthreads();
//
//        __syncthreads();
//
//        /*  if (stopFlag) break;*/
//    } // for iter
//}
//

// Host starting point for GPU RRT
//std::vector<TreeNode> gpuRRT(
//    const OccupancyGrid& h_grid,
//    float startX, float startY,
//    float goalX, float goalY,
//    int maxIter, int maxNodes, float maxStep
//) {
//    std::vector<TreeNode> tree(maxNodes);
//    std::vector<TreeNode> treeB(maxNodes);
//
//    std::vector<TreeNode> path;
//
//    // Initialize two tree with start node and end node
//    tree[0] = { startX, startY, -1 };
//	treeB[0] = { goalX, goalY, -1 };
//
//    // Device grid
//    OccupancyGrid d_grid;
//    size_t gridBytes = static_cast<size_t>(h_grid.width) * h_grid.height * sizeof(uint8_t);
//    cudaMalloc(&d_grid.data, gridBytes);
//    cudaMemcpy(d_grid.data, h_grid.data, gridBytes, cudaMemcpyHostToDevice);
//    d_grid.width = h_grid.width;
//    d_grid.height = h_grid.height;
//    d_grid.resolution = h_grid.resolution;
//    d_grid.origin_x = h_grid.origin_x;
//    d_grid.origin_y = h_grid.origin_y;
//
//    // Device trees
//    TreeNode* d_tree = nullptr;
//    cudaMalloc(&d_tree, static_cast<size_t>(maxNodes) * sizeof(TreeNode));
//
//	TreeNode* d_otree = nullptr;
//	cudaMalloc(&d_otree, static_cast<size_t>(maxNodes) * sizeof(TreeNode));
//
//    // copy first node to device trees
//    cudaMemcpy(d_tree, tree.data(), sizeof(TreeNode), cudaMemcpyHostToDevice);
//	cudaMemcpy(d_otree, treeB.data(), sizeof(TreeNode), cudaMemcpyHostToDevice);
//
//    // Device flags and counters
//    bool* d_goalReached = nullptr;
//    TreeNode* d_goal = nullptr;
//    int* d_size = nullptr;
//	int* d_osize = nullptr;
//    int* d_goalIdx = nullptr; // goal node index on device
//	int* d_ogoalIdx = nullptr; // goal node index on device for other tree
//    cudaMalloc(&d_goalReached, sizeof(bool));
//    cudaMalloc(&d_goal, sizeof(TreeNode));
//    cudaMalloc(&d_size, sizeof(int));
//	cudaMalloc(&d_osize, sizeof(int));
//    cudaMalloc(&d_goalIdx, sizeof(int));
//	cudaMalloc(&d_ogoalIdx, sizeof(int));
//
//    bool h_false = false;
//    int h_size = 1;
//	int h_osize = 1;
//    int h_neg1 = -1;
//    cudaMemcpy(d_goalReached, &h_false, sizeof(bool), cudaMemcpyHostToDevice);
//    cudaMemcpy(d_size, &h_size, sizeof(int), cudaMemcpyHostToDevice);
//	cudaMemcpy(d_osize, &h_osize, sizeof(int), cudaMemcpyHostToDevice);
//    cudaMemcpy(d_goalIdx, &h_neg1, sizeof(int), cudaMemcpyHostToDevice);
//	cudaMemcpy(d_ogoalIdx, &h_neg1, sizeof(int), cudaMemcpyHostToDevice);
//
//    // Launch
//    int block = 128;          // power of two
//    int gridSize = 64;
//    size_t shmem =
//        sizeof(TreeNode) +          // sampled
//        block * sizeof(float) +     // s_dist
//        block * sizeof(int);        // s_idx
//    timerpRRT().startGpuTimer();
//    rrtDoubleTreeKernel << <gridSize, block, shmem >> > (
//        d_grid, goalX, goalY,
//        maxIter, maxNodes, maxStep,
//		d_goalReached, d_goal, d_tree, d_size, d_goalIdx, d_otree, d_osize, d_ogoalIdx
//        );
//
//    cudaDeviceSynchronize();
//
//    // Fetch results
//    bool h_goalReached = false;
//    cudaMemcpy(&h_goalReached, d_goalReached, sizeof(bool), cudaMemcpyDeviceToHost);
//    cudaMemcpy(&h_size, d_size, sizeof(int), cudaMemcpyDeviceToHost);
//	cudaMemcpy(&h_osize, d_osize, sizeof(int), cudaMemcpyDeviceToHost);
//
//	int totalSize = h_size + h_osize;
//    // Copy back the whole tree that was filled
//    if (totalSize > 0) {
//        cudaMemcpy(tree.data(), d_tree, static_cast<size_t>(h_size) * sizeof(TreeNode), cudaMemcpyDeviceToHost);
//		cudaMemcpy(treeB.data(), d_otree, static_cast<size_t>(h_osize) * sizeof(TreeNode), cudaMemcpyDeviceToHost);
//    }
//
//    // Build both paths from each tree then append 
//    if (h_goalReached) {
//        int h_goalIdx = -1;
//        int h_ogoalIdx = -1;
//        cudaMemcpy(&h_goalIdx, d_goalIdx, sizeof(int), cudaMemcpyDeviceToHost);
//        cudaMemcpy(&h_ogoalIdx, d_ogoalIdx, sizeof(int), cudaMemcpyDeviceToHost);
//
//        // pathA: from start (d_tree) to meeting node (h_goalIdx) if valid
//        std::vector<TreeNode> pathA;
//        if (h_goalIdx >= 0 && h_goalIdx < h_size) {
//            int cur = h_goalIdx;
//            while (cur != -1) {
//                pathA.push_back(tree[cur]);
//                cur = tree[cur].parent;
//            }
//            std::reverse(pathA.begin(), pathA.end()); // now start->meet
//        }
//
//        // pathB: from goal (d_otree) to meeting node in other tree (h_ogoalIdx)
//        std::vector<TreeNode> pathB;
//        if (h_ogoalIdx >= 0 && h_ogoalIdx < h_osize) {
//            int cur = h_ogoalIdx;
//            while (cur != -1) {
//                pathB.push_back(treeB[cur]);
//                cur = treeB[cur].parent;
//            }
//           
//        }
//
//        //// Combine: pathA (start->meet) + pathB (meet->goal) but avoid duplicate meeting node
//        path = pathA;
//        if (!pathB.empty()) {
//            if (!path.empty() && path.back().x == pathB.front().x && path.back().y == pathB.front().y) {
//                // skip duplicate first element of pathB
//                path.insert(path.end(), pathB.begin() + 1, pathB.end());
//            }
//            else {
//                path.insert(path.end(), pathB.begin(), pathB.end());
//            }
//        }
//    }
//
//    timerpRRT().endGpuTimer();
//
//    // Cleanup
//      
//    cudaFree(d_grid.data);
//    cudaFree(d_tree);
//    cudaFree(d_otree);
//    cudaFree(d_goalReached);
//    cudaFree(d_goal);
//    cudaFree(d_size);
//    cudaFree(d_osize);
//    cudaFree(d_goalIdx);
//    cudaFree(d_ogoalIdx);
//
//    return path;
//}
//
//

