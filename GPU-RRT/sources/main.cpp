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
   
}




