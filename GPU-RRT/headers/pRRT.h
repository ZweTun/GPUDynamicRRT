#pragma once

#include "cpu_rrt.h"

#include "rrt_types.h"
#include <vector>
#include <cmath>

#include <cuda.h>
#include <cuda_runtime.h>
#include <stdint.h>
#include "common.h"
#include <vector>
#include "rrt_types.h"


// ---- Device helpers ----

// Returns true if the point (x,y) is free in the occupancy grid
__device__ bool isPointFreeGPU(const OccupancyGrid& grid, float x, float y);

// Checks for collision along the segment [x1,y1]  [x2,y2]; true if any collision
__device__ bool checkCollisionGPU(const OccupancyGrid& grid, float x1, float y1, float x2, float y2);

// Steer from 'from' towards 'to' by at most maxStep
__device__ TreeNode steerGPU(const TreeNode& from, const TreeNode& to, float maxStep);

// Sampling in free space using per-thread RNG
__device__ TreeNode sampleFreeSpaceGPU(const OccupancyGrid& grid, int iter);

// Goal check
__device__ bool isGoalGPU(float x, float y, float goalX, float goalY);

// Nearest neighbor search in the tree
__device__ void nearestNeighborGPU(
    const TreeNode& node,
    const TreeNode* d_tree,
    int size,
    int& outIdx,
    float& outDist
);

// Device function to reduce distances from sample to tree nodes
__device__ void distReductionGPU(
    float* s_dist,
    int* s_idx
); 



// ---- Host launcher ----

std::vector<TreeNode> gpuRRT(
    const OccupancyGrid& h_grid,
    float startX, float startY,
    float goalX, float goalY,
    int maxIter, int maxNodes, float maxStep
);

