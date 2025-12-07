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


//// Checks for collision along the segment 
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

    // Bresenham integer line 
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



__device__ float distanceSquared2(float x1, float y1, float x2, float y2) {
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

// Goal check
__device__ bool isGoalGPU(float x, float y, float goalX, float goalY) {
    float threshold = 1.00f; //  threshold
    return distanceSquared2(x, y, goalX, goalY) < threshold * threshold;
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




__device__ TreeNode sampleFreeSpaceGPU(const OccupancyGrid& grid, int iter) {
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
 
    float cxw = grid.origin_x + (grid.width * grid.resolution) * 0.5f;
    float cyw = grid.origin_y + (grid.height * grid.resolution) * 0.5f;
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


// Many threads sample in parallel 
__global__ void sampleKernel(OccupancyGrid grid, int iter,
    float* sx, float* sy, int numSamples) {
    unsigned tid = threadIdx.x + blockIdx.x * blockDim.x;
	if (tid >= numSamples) return; 
	TreeNode sampled = sampleFreeSpaceGPU(grid, iter + tid);
	sx[tid] = sampled.x;
	sy[tid] = sampled.y;   
}

// 1024 blocks, each block handles one sample
__global__ void nearestNeighborBatch(
    OccupancyGrid grid,
    float* sx, float* sy, int numSamples,
    float* d_treeX, float* d_treeY, int* d_size, int* nnIdx)
{
    int sampleId = blockIdx.x;
    if (sampleId >= numSamples) return;

    // shared sample slot
    TreeNode* s_sampled = reinterpret_cast<TreeNode*>(s_mem);
    if (threadIdx.x == 0) {
        *s_sampled = { sx[sampleId], sy[sampleId], -1 };
    }
    __syncthreads();

    float* s_dist = reinterpret_cast<float*>(s_sampled + 1);
    int* s_idx = reinterpret_cast<int*>(s_dist + blockDim.x);

    int size = *d_size;
    if (size <= 0) { nnIdx[sampleId] = -1; return; }

    float localBest = 1e30f;
    int localIdx = -1;
    TreeNode sampled = *s_sampled;

    // Each thread examines a strided subset of  tree
    for (int i = threadIdx.x; i < size; i += blockDim.x) {
        float dx = d_treeX[i] - sampled.x;
        float dy = d_treeY[i] - sampled.y;
        float dd = dx * dx + dy * dy;
        if (dd < localBest) { localBest = dd; localIdx = i; }
    }

    s_dist[threadIdx.x] = localBest;
    s_idx[threadIdx.x] = localIdx;
    __syncthreads();

    // reduction
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

    __syncthreads();
    // thread 0 in the block writes the NN for this sample
     // thread 0 in the block writes the NN for this sample
    if (threadIdx.x == 0) {
              // one-block-per-sample design
        nnIdx[sampleId] = s_idx[0];
    }
}


// Each threads handles one sample to extend and insert
__global__ void extendAndInsertKernel(
    OccupancyGrid grid,
    const float* sx, const float* sy,
    const int* nnIdx,
    float* treeX, float* treeY,
    int* treeParent,
    int* d_size,
    int maxNodes,
    float maxStep,
    float goalX, float goalY,
    int* d_goalReached_int, 
    int* d_goalIdx, int numSamples, int iter
) {
	int sId = threadIdx.x + blockIdx.x * blockDim.x;

    if (sId >= numSamples) return;

    // fast exit if goal already found
    if (atomicAdd(d_goalReached_int, 0) > 0) return;

    int idx = nnIdx[sId];
    if (idx < 0) return;

    // validate nearest index against current size (avoid reading out-of-range)
    int curSize = *d_size;
    if (idx >= curSize) return;

    float nx = treeX[idx];
    float ny = treeY[idx];
    float qx = sx[sId];
    float qy = sy[sId];

    // steer toward sample 
    TreeNode proposed = steerGPU({ nx, ny, -1 }, { qx, qy, -1 }, maxStep);

    unsigned seed = (sId * 1337u) ^ (iter * 911u);
    thrust::default_random_engine rng(seed);
    thrust::uniform_real_distribution<float> dist01(0.0f, 1.0f);


    if (dist01(rng) < 0.05f) {
        // Bias increased to focus on goal sampling
        TreeNode goalNode = { goalX, goalY, -1 };
       // proposed = steerGPU({ nx, ny, -1 }, goalNode, maxStep); 
        proposed = goalNode;
    }
 
    float px = proposed.x;
    float py = proposed.y;

    // collision test 
    if (checkCollisionGPU(grid, nx, ny, px, py)) return;


    int newIdx = atomicAdd(d_size, 1);
    if (newIdx >= maxNodes) {
        atomicSub(d_size, 1);
        return;
    }



	// write SoA arrays (memory coalesced)
    treeX[newIdx] = px;
    treeY[newIdx] = py;
    treeParent[newIdx] = idx;

  
    if (isGoalGPU(px, py, goalX, goalY)) {
        atomicExch(d_goalReached_int, 1);
        atomicExch(d_goalIdx, newIdx);
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
    //printf("Grid metadata:\n");
    //printf("  width=%d height=%d\n", h_grid.width, h_grid.height);
    //printf("  origin=(%f, %f)\n", h_grid.origin_x, h_grid.origin_y);
    //printf("  resolution=%f\n", h_grid.resolution);


   
    // Device flags and counters
    int* d_size = nullptr;
    int* d_goalIdx = nullptr; // goal node index on device

    cudaMalloc(&d_size, sizeof(int));
    cudaMalloc(&d_goalIdx, sizeof(int));


    int h_size = 1;
    int h_neg1 = -1;

    cudaMemcpy(d_size, &h_size, sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_goalIdx, &h_neg1, sizeof(int), cudaMemcpyHostToDevice);


    // FULLY pRRT 
	// Setup sx and sy arrays on device
    int numSamples = 256;
	float* sx;
    float* sy;
	cudaMalloc(&sx, numSamples * sizeof(float));
	cudaMalloc(&sy, numSamples * sizeof(float));
    
    
    int* nnIdx;
    cudaMalloc(&nnIdx, numSamples * sizeof(int));


	// Tree storage in structure of arrays format for faster access
    float* d_treeX = nullptr;
    float* d_treeY = nullptr;
    int* d_treeParent = nullptr;
    int* d_goalReached_int = nullptr;
    cudaMalloc(&d_goalReached_int, sizeof(int));
    int h_goalFalse = 0;
    cudaMemcpy(d_goalReached_int, &h_goalFalse, sizeof(int), cudaMemcpyHostToDevice);

    cudaMalloc(&d_treeX, static_cast<size_t>(maxNodes) * sizeof(float));
    cudaMalloc(&d_treeY, static_cast<size_t>(maxNodes) * sizeof(float));
    cudaMalloc(&d_treeParent, static_cast<size_t>(maxNodes) * sizeof(int));
  
    // initialize root node in SoA
    float h0x = startX;
    float h0y = startY;
    int   h0parent = -1;
    cudaMemcpy(d_treeX, &h0x, sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_treeY, &h0y, sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_treeParent, &h0parent, sizeof(int), cudaMemcpyHostToDevice);

    // initialize size = 1 on device
    int h_one = 1;

    cudaMemcpy(d_size, &h_one, sizeof(int), cudaMemcpyHostToDevice);

    int iter = 0;

    timerpRRT().startGpuTimer();

	// Only copy every this iterations
    int copyIter = 1000;
    

    // Launch params to test with
  
    int block = 256;          // power of two

    size_t shmem =
        sizeof(TreeNode) +          // sampled
        block * sizeof(float) +     // s_dist
        block * sizeof(int);        // s_idx



    dim3 blockDim(block);
    dim3 gridDim((numSamples + block - 1) / block);

    while (iter < maxIter) {
      

		// Launch sampling kernel to fill sx and sy
		// Number of threads = numSamples * 256 roughly
        sampleKernel <<<gridDim, blockDim, shmem>>> (d_grid, iter, sx, sy, numSamples);

		//// Nearest neighbor per block 
        nearestNeighborBatch << <blockDim, block, shmem >> > (
            d_grid,
            sx, sy,
            numSamples,
            d_treeX, d_treeY,
            d_size,
            nnIdx);


		//// Extend and insert
        extendAndInsertKernel << <gridDim, blockDim, shmem >> > (
            d_grid,
            sx, sy,
            nnIdx,
            d_treeX, d_treeY,
            d_treeParent,
            d_size,
            maxNodes,
            maxStep,
            goalX, goalY,
            d_goalReached_int,
            d_goalIdx, numSamples, iter
			);


        int h_goal = 0;
        if (iter % copyIter == 0) {
        
            cudaMemcpy(&h_goal, d_goalReached_int, sizeof(int), cudaMemcpyDeviceToHost);

		}
   

        if (h_goal > 0) break;      
    
    
        iter++;
    }


    timerpRRT().endGpuTimer();

    int h_goalInt1 = 0;
    int h_size_int1 = 0;
    cudaMemcpy(&h_goalInt1, d_goalReached_int, sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(&h_size_int1, d_size, sizeof(int), cudaMemcpyDeviceToHost);


    // Wait and fetch SoA results
    cudaDeviceSynchronize();

    // Fetch device flags and counters
    int h_goalInt = 0;
    int h_size_int = 0;
    cudaMemcpy(&h_goalInt, d_goalReached_int, sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(&h_size_int, d_size, sizeof(int), cudaMemcpyDeviceToHost);


    if (h_goalInt) {
        int h_goalIdx = -1;
        cudaMemcpy(&h_goalIdx, d_goalIdx, sizeof(int), cudaMemcpyDeviceToHost);

        if (h_goalIdx >= 0 && h_goalIdx < h_size_int) {
            int cur = h_goalIdx;
            while (cur != -1) {
                float px = 0.0f, py = 0.0f;
                cudaMemcpy(&px, d_treeX + cur, sizeof(float), cudaMemcpyDeviceToHost);
                cudaMemcpy(&py, d_treeY + cur, sizeof(float), cudaMemcpyDeviceToHost);
                path.push_back({ px, py, -1 });
                int parentIdx = -1;
                cudaMemcpy(&parentIdx, d_treeParent + cur, sizeof(int), cudaMemcpyDeviceToHost);
                cur = parentIdx;
            }
            std::reverse(path.begin(), path.end());
        }
    }
    // Print size of tree
	printf("RRT Tree Size: %d nodes\n", h_size_int);



    // Cleanup
    cudaFree(d_grid.data);
    cudaFree(d_size);
    cudaFree(d_goalIdx);
    cudaFree(d_treeX);
    cudaFree(d_treeY);
    cudaFree(d_treeParent);
    cudaFree(sx);
    cudaFree(sy);
    cudaFree(nnIdx);
    cudaFree(d_goalReached_int);

    return path;
}











