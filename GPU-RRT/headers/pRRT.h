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




//Checks for collision between two points
bool checkCollisionGPU(OccupancyGrid* grid, float x1, float y1, float x2, float y2);

bool isPointFreeGPU(const OccupancyGrid& grid, float x, float y);

__global__ void neighborSearchScan(int n, int* odata, const int* idata, int offset); 

int neighborSearch(int n, int* odata, const int* idata);

TreeNode steerGPU(const TreeNode& from, const TreeNode& to, float maxStep);

TreeNode sampleFreeSpaceGPU(const OccupancyGrid& grid);


// Checks if the given point is within a threshold distance to the goal
bool isGoalGPU(float x, float y, float goalX, float goalY);


// CPU RRT implementation
std::vector<TreeNode> gpuRRT(OccupancyGrid grid,
    float startX, float startY,
    float goalX, float goalY,
    int maxIter, int maxNodes, float maxStep);
