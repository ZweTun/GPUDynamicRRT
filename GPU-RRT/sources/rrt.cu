
#include <cuda_runtime.h>
#include "rrt.h"
#include <cuda.h>\
#include "common.h"
#include "cmath"
#include <device_launch_parameters.h>
#include <vector>


// Devices

// Computes Euclidean distance between two points
__device__ float distance(float x1, float y1, float x2, float y2) {
    return sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Checks if the given point is in free space 
__device__ bool isPointFree(const OccupancyGrid* grid, float x, float y) {
    int gx = (x - grid->origin_x) / grid->resolution;
    int gy = (y - grid->origin_y) / grid->resolution;
    int idx = gy * grid->width + gx;
    return grid->data[idx] == 0;
}

// Checks if the given point is within a threshold distance to the goal
__device__ bool isGoal(float x, float y, float goalX, float goalY) {
    return distance(x, y, goalX, goalY) < 0.1;
}


// Samples a random point in free space
// Should call device_isPointFree to ensure sampled point is in free space
__device__ TreeNode sampleFreeSpace(OccupancyGrid* grid) {
	//TODO

}

// Returns index of nearest neighbor in the tree 
__device__ int nearestNeighbor(TreeNode* tree, int tree_size, float rand_x, float rand_y) {
    //TODO
}


//Checks for collision between two points
__device__ bool checkCollision(OccupancyGrid* grid, float x1, float y1, float x2, float y2) {
    //TODO
}




// Kernels
__global__ void kernInitTree(TreeNode* tree, int max_nodes, float start_x, float start_y) {
    int index = threadIdx.x + (blockIdx.x * blockDim.x);
    if (index == 0) {
        tree[0].x = start_x;
        tree[0].y = start_y;
        tree[0].parent = -1;
    }
}


// RRT main kernel
__global__ void kernRRT(
    int maxIter,
    int maxNodes,
    OccupancyGrid grid,
    float startX, float startY,
    float goalX, float goalY,
    TreeNode* allTrees,
    int* results    // size = numThreads
) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    // Each thread uses RRT tree region:
    TreeNode* tree = &allTrees[tid * maxNodes];

    int size = 1;
    tree[0] = { startX, startY, -1 };
    results[tid] = -1;  

    for (int iter = 1; iter < maxIter && size < maxNodes; ++iter) {
        TreeNode newNode = sampleFreeSpace(&grid);
        int nearestIdx = nearestNeighbor(tree, size, newNode.x, newNode.y);
        TreeNode nearest = tree[nearestIdx];

        if (!checkCollision(&grid, nearest.x, nearest.y, newNode.x, newNode.y)) {
            newNode.parent = nearestIdx;
            int idx = size++;
            tree[idx] = newNode;

            if (isGoal(newNode.x, newNode.y, goalX, goalY)) {

				// Path found
				// Store the index of the goal node
				// result[tid] will be used to trace back the path later
				// final node on host side will be tree[results[tid]]
                results[tid] = idx;  
                return;              
            }
        }
    }
    
	//No path found
}


void findFinalPathRecursive(const TreeNode* tree, int index, std::vector<TreeNode>& path) {
    if (index == -1) return;
    findFinalPathRecursive(tree, tree[index].parent, path);
    path.push_back(tree[index]);
}


std::vector<TreeNode> launchRRT(const OccupancyGrid& h_grid,
    float startX, float startY,
    float goalX, float goalY)
{   

	// parameters
    int numThreads = 1024;
    int maxNodes = 2048;
    int maxIter = 2000;

  
    OccupancyGrid d_grid;

    size_t gridBytes = h_grid.width * h_grid.height * sizeof(uint8_t);
    cudaMalloc(&d_grid.data, gridBytes);
    cudaMemcpy(d_grid.data, h_grid.data, gridBytes, cudaMemcpyHostToDevice);

    d_grid.width = h_grid.width;
    d_grid.height = h_grid.height;
    d_grid.resolution = h_grid.resolution;
    d_grid.origin_x = h_grid.origin_x;
    d_grid.origin_y = h_grid.origin_y;

	// allocations of GPU memory
    TreeNode* d_allTrees;
    cudaMalloc(&d_allTrees, numThreads * maxNodes * sizeof(TreeNode));

    int* d_results;
    cudaMalloc(&d_results, numThreads * sizeof(int));

    TreeNode* h_allTrees = new TreeNode[numThreads * maxNodes];
    int* h_results = new int[numThreads];

	// launch kernels
    dim3 block(128);
    dim3 gridDim((numThreads + block.x - 1) / block.x);

    kernRRT << <gridDim, block >> > (
        maxIter,
        maxNodes,
        d_grid,
        startX, startY,
        goalX, goalY,
        d_allTrees,
        d_results
        );

    cudaDeviceSynchronize();

   // Copy back to host
    cudaMemcpy(h_allTrees, d_allTrees,
        numThreads * maxNodes * sizeof(TreeNode),
        cudaMemcpyDeviceToHost);

    cudaMemcpy(h_results, d_results,
        numThreads * sizeof(int),
        cudaMemcpyDeviceToHost);

	// find a successful path
    for (int tid = 0; tid < numThreads; ++tid) {
        if (h_results[tid] != -1) {
            int goalIndex = h_results[tid];
            TreeNode* treeBase = &h_allTrees[tid * maxNodes];
			std::vector<TreeNode>& path = std::vector<TreeNode>();
			findFinalPathRecursive(treeBase, goalIndex, path);
			return path; // Return the found path
        }
    }

    // No solution
    return {};
}