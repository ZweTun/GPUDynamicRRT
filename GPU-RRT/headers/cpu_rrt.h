#pragma once


#include "cpu_rrt.h"

#include "rrt_types.h"
#include <vector>
#include <cmath>




//Checks for collision between two points
bool checkCollisionCPU(OccupancyGrid* grid, float x1, float y1, float x2, float y2);

// Helper that returns true if the point (x,y) is free in the occupancy grid
bool isPointFreeCPU(const OccupancyGrid& grid, float x, float y);

// Nearest neighbor search in the tree
int nearestNeighborCPU(const std::vector<TreeNode>& tree, float x, float y, int size);

// Steer from 'from' towards 'to' by at most maxStep
TreeNode steerCPU(const TreeNode& from, const TreeNode& to, float maxStep);

// Sampling in free space
TreeNode sampleFreeSpaceCPU(const OccupancyGrid& grid);


// Checks if the given point is within a threshold distance to the goal
bool isGoalCPU(float x, float y, float goalX, float goalY);


// CPU RRT implementation
std::vector<TreeNode> cpuRRT(OccupancyGrid grid,
    float startX, float startY,
    float goalX, float goalY,
    int maxIter, int maxNodes, float maxStep); 
