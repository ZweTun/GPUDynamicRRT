
#include <cuda_runtime.h>
#include "rrt.h"
#include <cuda.h>\
#include "common.h"
#include "cmath"
#include <device_launch_parameters.h>



// Devices

// Computes Euclidean distance between two points
__device__ float distance(float x1, float y1, float x2, float y2) {
    return sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Checks if the given point is in free space 
__device__ bool isPointFree(const OccupancyGridGPU* grid, float x, float y) {
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
__device__ TreeNode sampleFreeSpace(OccupancyGridGPU* grid) {
	//TODO

}

// Returns index of nearest neighbor in the tree 
__device__ int nearestNeighbor(TreeNode* tree, int tree_size, float rand_x, float rand_y) {
    //TODO
}


//Checks for collision between two points
__device__ bool checkCollision(OccupancyGridGPU* grid, float x1, float y1, float x2, float y2) {
    //TODO
}


// Returns path as a array of TreeNode pointers from start to goal
__device__ TreeNode* findPath(TreeNode* tree, TreeNode lastNode) {
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
    OccupancyGridGPU grid,
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
                results[tid] = idx;  
                return;              
            }
        }
    }
    
	//No path found
}



void initOccupancyGridGPU(OccupancyGridGPU* grid, uint8_t* data, int width, int height, float resolution, float origin_x, float origin_y) {
    grid->data = data;
    grid->width = width;
    grid->height = height;
    grid->resolution = resolution;
    grid->origin_x = origin_x;
	grid->origin_y = origin_y;
}



