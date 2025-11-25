#include <iostream>
#include <cuda.h>]
#include <cuda_runtime.h>
#include "rrt.h"
#include "cpu_rrt.h"
int width = 10;
int height = 10;
float resolution = 1.0f;
float maxStep = 0.5f;
int maxIter = 6000;
int maxNodes = 6000;

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


//int main() {
//    std::cout << "Hello RRT!" << std::endl;
//	OccupancyGrid* grid = new OccupancyGrid();
//    //initOccupancyGrid(*grid, width, height, resolution);
//
//	initOccupancyGridFromVector(*grid, width, height, resolution, visualMap);
//
//    // Allocate memory for trees and results
//    std::vector<TreeNode> path = launchRRT(*grid, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
//    //	std::vector<TreeNode> path = cpuRRT(*grid, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
//    if (!path.empty()) {
//        std::cout << "Path found:" << std::endl;
//        for (const auto& node : path) {
//            std::cout << "(" << node.x << ", " << node.y << ")" << std::endl;
//        }
//    } else {
//        std::cout << "No path found." << std::endl;
//    }
//    delete[] grid->data;
//    delete grid;
//	return 0;
//}



//For code testing purposes
// RRT_TESTS

#include <cstdio>
#include <vector>
#include <iostream>

#include "rrt_types.h"
#include "cpu_rrt.h"
#include "rrt.h"
#include "testing_helpers.hpp"

#include "common.h"
const int MAP_W = 10;
const int MAP_H = 10;

uint8_t gridDataCPU[MAP_W * MAP_H];
uint8_t gridDataGPU[MAP_W * MAP_H];

OccupancyGrid makeGrid(const std::vector<int>& visual) {
    OccupancyGrid g;
    g.width = MAP_W;
    g.height = MAP_H;
    g.resolution = 1.0f;
    g.origin_x = 0;
    g.origin_y = 0;

    for (int i = 0; i < MAP_W * MAP_H; i++) {
        gridDataCPU[i] = visual[i];
        gridDataGPU[i] = visual[i];
    }
    g.data = gridDataCPU;
    return g;
}


int main(int argc, char* argv[]) {

    printf("\n*****************\n");
    printf("** RRT TESTING **\n");
    printf("*****************\n");

    float startX = 0, startY = 0;
    float goalX = 8, goalY = 8;

    int maxIter = 4000;
    int maxNodes = 2048;
    float maxStep = 1.0f;

    //
    // Test 1: Empty map
    //
    printHeader("TEST 1: Empty map");

    std::vector<int> emptyMap(MAP_W * MAP_H, 0);
    OccupancyGrid gridEmpty = makeGrid(emptyMap);

    // CPU RRT
    printDesc("CPU RRT (empty map)");
    zeroTimerCPU();
    timerCPU().startCpuTimer();
    auto cpuPath = cpuRRT(gridEmpty, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
	timerCPU().endCpuTimer();
    printElapsedTimeCPU("(std::chrono measured)");
    printf("CPU path length: %zu\n", cpuPath.size());

    // GPU RRT
    printDesc("GPU RRT (empty map)");
    zeroTimerGPU();
    auto gpuPath = launchRRT(gridEmpty, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
    printElapsedTimeGPU("(CUDA measured)");
    printf("GPU path length: %zu\n", gpuPath.size());

    printCmpPath(cpuPath, gpuPath);

    //
    // Test 2: Maze
    //
    printHeader("TEST 2: Maze map");

    std::vector<int> mazeMap = {
        0,0,0,0,0,0,0,0,0,0,
        0,0,0,1,1,1,0,0,0,0,
        0,1,0,0,0,1,0,1,0,0,
        0,1,0,1,0,1,0,1,0,0,
        0,0,0,1,0,1,0,0,0,0,
        0,1,0,1,0,1,1,1,0,0,
        0,1,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0
    };

    OccupancyGrid gridMaze = makeGrid(mazeMap);

    printDesc("CPU RRT (maze)");
    zeroTimerCPU();
    timerCPU().startCpuTimer();
    auto cpuMaze = cpuRRT(gridMaze, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
	timerCPU().endCpuTimer();
    printElapsedTimeCPU("(std::chrono measured)");
    printf("CPU path length: %zu\n", cpuMaze.size());

    printDesc("GPU RRT (maze)");
    zeroTimerGPU();
    auto gpuMaze = launchRRT(gridMaze, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
    printElapsedTimeGPU("(CUDA measured)");
    printf("GPU path length: %zu\n", gpuMaze.size());

    printCmpPath(cpuMaze, gpuMaze);

    //
    // Test 3: Fully blocked
    //
    printHeader("TEST 3: Blocked map");

    std::vector<int> blocked(MAP_W * MAP_H, 1);
    blocked[0] = 0; // start free

    OccupancyGrid gridBlocked = makeGrid(blocked);

    printDesc("CPU RRT (blocked)");
    zeroTimerCPU();
    timerCPU().startCpuTimer();
    auto cpuBlocked = cpuRRT(gridBlocked, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
	timerCPU().endCpuTimer();
    printElapsedTimeCPU("(std::chrono measured)");
    printf("CPU path length: %zu\n", cpuBlocked.size());

    printDesc("GPU RRT (blocked)");
    zeroTimerGPU();
    auto gpuBlocked = launchRRT(gridBlocked, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
    printElapsedTimeGPU("(CUDA measured)");
    printf("GPU path length: %zu\n", gpuBlocked.size());

    printCmpPath(cpuBlocked, gpuBlocked);

    printf("\n*** RRT TESTING COMPLETE ***\n");

    return 0;
}

