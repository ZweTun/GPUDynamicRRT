#pragma once

#include "cpu_rrt.h"

#include "rrt_types.h"
#include <vector>
#include <cmath>

#include <vector>

#include <unordered_set>
#include <iostream>

#include "common.h"

//Checks for collision between two points
bool checkCollisionCPU(OccupancyGrid* grid, float x1, float y1, float x2, float y2) {
    //TODO
    int x_diff = abs(int(ceil((x2 - x1) / grid->resolution)));
    int y_diff = abs(int(ceil((y2 - y1) / grid->resolution)));
    int steps = -1;
    if (x_diff > y_diff) {
        steps = x_diff;
    }
    else {
        steps = y_diff;
    }
    if (steps == 0) {
        return !isPointFreeCPU(*grid, x2, y2); // No movement
    }

    float dt = 1.0 / steps;
    float t = 0.0;

    //Check points along the line
    for (int i = 0; i <= steps; ++i) {
        float x = x1 + t * (x2 - x1);
        float y = y1 + t * (y2 - y1);
        if (!isPointFreeCPU(*grid, x, y)) {
            return true; // Collision detected
        }
        t += dt;
    }
    return false; // No collision
}




bool isPointFreeCPU(const OccupancyGrid& grid, float x, float y) {
    int gx = (x - grid.origin_x) / grid.resolution;
    int gy = (y - grid.origin_y) / grid.resolution;
    if (gx < 0 || gx >= grid.width || gy < 0 || gy >= grid.height) return false;
    return grid.data[gy * grid.width + gx] == 0;
}

int nearestNeighborCPU(const std::vector<TreeNode>& tree, float x, float y, int size) {
    int best = 0;
    float bestDist = 1e9f;
    for (int i = 0; i < size; i++) {
        float dx = tree[i].x - x;
        float dy = tree[i].y - y;
        float d2 = dx * dx + dy * dy;
        if (d2 < bestDist) { bestDist = d2; best = i; }
    }
    return best;
}

TreeNode steerCPU(const TreeNode& from, const TreeNode& to, float maxStep) {
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist < maxStep) return to;
    return { from.x + dx / dist * maxStep, from.y + dy / dist * maxStep, -1 };
}


TreeNode sampleFreeSpaceCPU(const OccupancyGrid& grid) {
    float x, y;
	int maxAttempts = 1000;
	int attempts = 0;
    do {
        x = grid.origin_x + static_cast<float>(rand()) / RAND_MAX * grid.width * grid.resolution;
        y = grid.origin_y + static_cast<float>(rand()) / RAND_MAX * grid.height * grid.resolution;
        attempts++;
    } while (!isPointFreeCPU(grid, x, y) && attempts < maxAttempts);
    

    
    return { x, y, -1 };
}



// Checks if the given point is within a threshold distance to the goal
bool isGoalCPU(float x, float y, float goalX, float goalY) {
    float threshold = 0.15f; //  threshold
	return (sqrtf((x - goalX) * (x - goalX) + (y - goalY) * (y - goalY)) < threshold);
}

std::vector<TreeNode> findFinalPathCPU(std::vector<TreeNode> tree, int goalIdx) {
    std::vector<TreeNode> path;
    int idx = goalIdx;

    // protect against infinite loops
    std::unordered_set<int> visited;

    while (idx >= 0) {
 
        if (visited.count(idx)) {
            std::cout << "ERROR: cycle in parent pointers\n";
            break;
        }
	//	printf("Visiting node %d at (%.2f, %.2f)\n", idx, tree[idx].x, tree[idx].y);
        path.push_back(tree[idx]);
        visited.insert(idx);
        idx = tree[idx].parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}



// CPU RRT implementation
std::vector<TreeNode> cpuRRT(OccupancyGrid grid,
    float startX, float startY,
    float goalX, float goalY,
    int maxIter, int maxNodes, float maxStep) {
    
	std::vector<TreeNode> tree(maxNodes);
	std::vector<TreeNode> path;

    int size = 1;
    tree[0] = { startX, startY, -1 };

    for (int iter = 1; iter < maxIter && size < maxNodes; ++iter) {
        TreeNode newNode = sampleFreeSpaceCPU(grid);


		int nearestIdx = nearestNeighborCPU(tree, newNode.x, newNode.y, size);
        TreeNode nearest = tree[nearestIdx];
        newNode = steerCPU(nearest, newNode, maxStep); //Steer with max range 
        if (rand() % 100 < 5) {
            // With small probability, sample the goal directly
            newNode = { goalX, goalY, -1 };
        }
        if (!checkCollisionCPU(&grid, nearest.x, nearest.y, newNode.x, newNode.y)) {
            newNode.parent = nearestIdx;
            int idx = size++;
            tree[idx] = newNode;
            if (isGoalCPU(newNode.x, newNode.y, goalX, goalY)) {

                // Path found
        
                return findFinalPathCPU(tree, idx);
            }
        }
    }

    //No path found
    return {};
}
