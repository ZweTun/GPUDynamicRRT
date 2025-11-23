#pragma once


#include "cpu_rrt.h"

#include "rrt_types.h"
#include <vector>
#include <cmath>




//Checks for collision between two points
bool checkCollisionCPU(OccupancyGrid* grid, float x1, float y1, float x2, float y2);

bool isPointFreeCPU(const OccupancyGrid& grid, float x, float y);

int nearestNeighborCPU(const std::vector<TreeNode>& tree, float x, float y);

TreeNode steerCPU(const TreeNode& from, const TreeNode& to, float maxStep);

TreeNode sampleFreeSpaceCPU(const OccupancyGrid& grid);


// Checks if the given point is within a threshold distance to the goal
bool isGoalCPU(float x, float y, float goalX, float goalY);


// CPU RRT implementation
std::vector<TreeNode> cpuRRT(OccupancyGrid grid,
    float startX, float startY,
    float goalX, float goalY,
    int maxIter, int maxNodes, float maxStep); 
