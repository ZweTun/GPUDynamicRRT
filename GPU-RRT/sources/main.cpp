#include <iostream>
#include <cuda.h>]
#include <cuda_runtime.h>
#include "rrt.h"


void initOccupancyGrid(OccupancyGrid& grid, int width, int height, float resolution) {
    grid.width = width;
    grid.height = height;
    grid.resolution = resolution;
    grid.origin_x = 0.0f;
    grid.origin_y = 0.0f;
    grid.data = new uint8_t[width * height];
    std::memset(grid.data, 0, width * height * sizeof(uint8_t));
}

int main() {
    std::cout << "Hello from GPU RRT!" << std::endl;
	OccupancyGrid* grid = new OccupancyGrid();
    initOccupancyGrid(*grid, 100, 100, 0.1f);
    // Define start and goal positions
    float startX = 1.0f, startY = 1.0f;
    float goalX = 8.0f, goalY = 8.0f;
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




