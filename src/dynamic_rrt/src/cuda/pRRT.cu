#include "pRRT.h"

#include "rrt_types.h"
#include <vector>
#include <cmath>
#include "common.h"

//Checks for collision between two points
bool checkCollisionGPU(OccupancyGrid* grid, float x1, float y1, float x2, float y2) {
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
        return !isPointFreeGPU(*grid, x2, y2); // No movement
    }

    float dt = 1.0 / steps;
    float t = 0.0;

    //Check points along the line
    for (int i = 0; i <= steps; ++i) {
        float x = x1 + t * (x2 - x1);
        float y = y1 + t * (y2 - y1);
        if (!isPointFreeGPU(*grid, x, y)) {
            return true; // Collision detected
        }
        t += dt;
    }
    return false; // No collision
}




bool isPointFreeGPU(const OccupancyGrid& grid, float x, float y) {
    int gx = (x - grid.origin_x) / grid.resolution;
    int gy = (y - grid.origin_y) / grid.resolution;
    if (gx < 0 || gx >= grid.width || gy < 0 || gy >= grid.height) return false;
    return grid.data[gy * grid.width + gx] == 0;
}


__global__ void neighborSearchScan(int n, int* odata, const int* idata, int offset) {
    //TODO
}


// Lauch kernels that perform a neighborSearchScan
// Parellize nearest neighbor
// Returns idx of nearest neighbor
int neighborSearch(int n, int* odata, const int* idata) {
    //TODO
    return 0; 
}


TreeNode steerGPU(const TreeNode& from, const TreeNode& to, float maxStep) {
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist < maxStep) return to;
    return { from.x + dx / dist * maxStep, from.y + dy / dist * maxStep, -1 };
}


TreeNode sampleFreeSpaceGPU(const OccupancyGrid& grid) {
    float x, y;
    do {
        x = grid.origin_x + static_cast<float>(rand()) / RAND_MAX * grid.width * grid.resolution;
        y = grid.origin_y + static_cast<float>(rand()) / RAND_MAX * grid.height * grid.resolution;
    } while (!isPointFreeCPU(grid, x, y));
    return { x, y, -1 };
}



// Checks if the given point is within a threshold distance to the goal
bool isGoalGPU(float x, float y, float goalX, float goalY) {
    float threshold = 0.15f; //  threshold
    return (sqrtf((x - goalX) * (x - goalX) + (y - goalY) * (y - goalY)) < threshold);
}





// CPU RRT implementation
std::vector<TreeNode> gpuRRT(OccupancyGrid grid,
    float startX, float startY,
    float goalX, float goalY,
    int maxIter, int maxNodes, float maxStep) {

    std::vector<TreeNode>(tree) (maxNodes);
    std::vector<TreeNode> path;

    int size = 1;
    tree[0] = { startX, startY, -1 };

    for (int iter = 1; iter < maxIter && size < maxNodes; ++iter) {
        TreeNode newNode = sampleFreeSpaceGPU(grid);

        // Call neighborSearch
        int nearestIdx = 0;
        TreeNode nearest = tree[nearestIdx];
        newNode = steerGPU(nearest, newNode, 1.0f); //Steer with max range 0.5

        if (!checkCollisionGPU(&grid, nearest.x, nearest.y, newNode.x, newNode.y)) {
            newNode.parent = nearestIdx;
            int idx = size++;
            tree[idx] = newNode;
            path.push_back(newNode);
            if (isGoalGPU(newNode.x, newNode.y, goalX, goalY)) {

                // Path found
                path.push_back({ goalX, goalY, idx });


                return path;
            }
        }
    }

    //No path found
    return {};
}
