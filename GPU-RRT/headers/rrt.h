
#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <stdint.h>
#include "common.h"

struct TreeNode {
    float x, y;
    int parent;
};

struct OccupancyGrid {
	//2D array grid data 
    uint8_t* data;
    int width, height;
    float resolution;
    float origin_x, origin_y;
};




// device functions
__device__ int nearestNeighbor(TreeNode* tree, int tree_size, float rand_x, float rand_y);
__device__ TreeNode sampleFreeSpace(OccupancyGrid* grid);
__device__ bool isPointFree(const OccupancyGrid* grid, float x, float y);
__device__ bool checkCollision(OccupancyGrid* grid, float x1, float y1, float x2, float y2);
// kernels

__global__ void kernRRT(
    int maxIter,
    int maxNodes,
    OccupancyGrid grid,
    float startX, float startY,
    float goalX, float goalY,
    TreeNode* allTrees,
    int* results    
); 


void findPathRRT(
    int maxIter,
    int maxNodes,
    OccupancyGrid& grid,
    float startX, float startY,
    float goalX, float goalY,
    TreeNode* allTrees,
    int* results
);


