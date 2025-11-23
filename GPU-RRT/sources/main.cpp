#include <iostream>
#include <cuda.h>]
#include <cuda_runtime.h>
#include "rrt.h"

int width = 10;
int height = 10;
float resolution = 1.0f
;
// Define start and goal positions
float startX = 0.0f, startY = 0.0f;
float goalX = 9.0f, goalY = 9.0f;

// Visual representation of the occupancy grid 10 x 10

std::vector<int> visualMap = {
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,1,1,1,0,0,0,0,
    0,1,0,0,0,1,0,1,0,0,
    0,1,0,1,0,1,0,1,0,0,
    0,0,0,1,0,1,0,0,0,0,
    0,1,0,1,0,1,1,1,0,0,
    0,1,0,0,0,0,0,0,0,0,
     0,0,0,0,0,0,0,0,0,0,
     0,0,0,1,1,1,0,0,0,0,
     0,0,0,0,0,0,0,0,0,0
};



// All free space 
void initOccupancyGrid(OccupancyGrid& grid, int width, int height, float resolution) {
    grid.width = width;
    grid.height = height;
    grid.resolution = resolution;
    grid.origin_x = 0.0f;
    grid.origin_y = 0.0f;
    grid.data = new uint8_t[width * height];
    std::memset(grid.data, 0, width * height * sizeof(uint8_t));
}


// Initialize occupancy grid from a vector representation
void initOccupancyGridFromVector(OccupancyGrid& grid,
    int width, int height,
    float resolution,
    const std::vector<int>& src)
{
    grid.width = width;
    grid.height = height;
    grid.resolution = resolution;
    grid.origin_x = 0.0f;
    grid.origin_y = 0.0f;

    grid.data = new uint8_t[width * height];

    for (int i = 0; i < width * height; i++) {
        grid.data[i] = (src[i] == 1 ? 100 : 0);
        // 1 = obstacle, 0 = free
    }
}


int main() {
    std::cout << "Hello from GPU RRT!" << std::endl;
	OccupancyGrid* grid = new OccupancyGrid();
    //initOccupancyGrid(*grid, width, height, resolution);

	initOccupancyGridFromVector(*grid, width, height, resolution, visualMap);

    // Allocate memory for trees and results
	std::vector<TreeNode> path = launchRRT(*grid, startX, startY, goalX, goalY);
    if (!path.empty()) {
        std::cout << "Path found:" << std::endl;
        for (const auto& node : path) {
            std::cout << "(" << node.x << ", " << node.y << ")" << std::endl;
        }
    } else {
        std::cout << "No path found." << std::endl;
    }
    delete[] grid->data;
    delete grid;
	return 0;
}




