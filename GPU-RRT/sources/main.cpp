#include <iostream>
#include <cuda.h>]
#include <cuda_runtime.h>
#include "rrt.h"
#include "cpu_rrt.h"
#include "pRRT.h"



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


void inflateObstaclesCPU(std::vector<int>& grid, int width, int height, int inflationRadius)
{
    std::vector<int> inflated = grid;  // start with copy

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {

            if (grid[y * width + x] == 1) {   // expand around OBSTACLE

                // Inflate neighbors
                for (int dy = -inflationRadius; dy <= inflationRadius; dy++) {
                    for (int dx = -inflationRadius; dx <= inflationRadius; dx++) {

                        int nx = x + dx;
                        int ny = y + dy;

                        if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                            continue;

                        // Only fill within circular radius
                        if (dx * dx + dy * dy <= inflationRadius * inflationRadius) {
                            inflated[ny * width + nx] = 1;
                        }
                    }
                }
            }

        }
    }

    grid = inflated;
}

OccupancyGrid makeGrid(const std::vector<int>& visual, int W, int H)
{
    OccupancyGrid g;
    g.width = W;
    g.height = H;
    g.resolution = 1.0f;
    g.origin_x = 0;
    g.origin_y = 0;

    // allocate exact size
    uint8_t* buf = new uint8_t[W * H];

    for (int i = 0; i < W * H; i++) {
        buf[i] = (uint8_t)visual[i];
    }

    g.data = buf; 
    return g;
}




int main(int argc, char* argv[]) {

    printf("\n*****************\n");
    printf("** RRT TESTING **\n");
    printf("*****************\n");

    //float startX = 0, startY = 0;
    //float goalX = 8, goalY = 8;

    //int maxIter = 4000;
    //int maxNodes = 2048;
    //float maxStep = 1.0f;

    float resolution = 0.05f;
    float maxStep = 1.0f;
    int maxIter = 10000;
    int maxNodes = 100000;


    // Define start and goal positions
    float startX = 0.0f, startY = 0.0f;
    float goalX = 9.0f, goalY = 9.0f;

    //
    // Test 1: Empty map
    //
    printHeader("TEST 1: Empty map (10x10)");

    std::vector<int> emptyMap(10 * 10, 0);
    OccupancyGrid gridEmpty = makeGrid(emptyMap, 10, 10);


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

    // GPU pRRT
	printDesc("pRRT (empty map)");
	zeroTimerpRRT();
	auto pRRTPath = gpuRRT(gridEmpty, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
    printElapsedTimepRRT("(CUDA measured)");
	printf("pRRT path length: %zu\n", pRRTPath.size());
	printCmpPath(cpuPath, pRRTPath);

    //
    // Test 2: Maze
    //
    printHeader("TEST 2: Maze map (10x10)");

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

    OccupancyGrid gridMaze = makeGrid(mazeMap, 10, 10);

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

    // GPU pRRT
    printDesc("pRRT (maze)");
    zeroTimerpRRT();
    auto pRRTMaze = gpuRRT(gridMaze, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
    printElapsedTimepRRT("(CUDA measured)");
    printf("pRRT path length: %zu\n", pRRTMaze.size());
    printCmpPath(cpuMaze, pRRTMaze);




    //
    // Test 3: Fully blocked
    //
 //   printHeader("TEST 3: Blocked map");

 //   std::vector<int> blocked(10 * 10, 1);
 //   blocked[0] = 0; // start free

 //   OccupancyGrid gridBlocked = makeGrid(blocked, 10, 10);

 //   printDesc("CPU RRT (blocked)");
 //   zeroTimerCPU();
 //   timerCPU().startCpuTimer();
 //   auto cpuBlocked = cpuRRT(gridBlocked, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
	//timerCPU().endCpuTimer();
 //   printElapsedTimeCPU("(std::chrono measured)");
 //   printf("CPU path length: %zu\n", cpuBlocked.size());

 //   printDesc("GPU RRT (blocked)");
 //   zeroTimerGPU();
 //   auto gpuBlocked = launchRRT(gridBlocked, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
 //   printElapsedTimeGPU("(CUDA measured)");
 //   printf("GPU path length: %zu\n", gpuBlocked.size());

 //   printCmpPath(cpuBlocked, gpuBlocked);


 //   // GPU pRRT
 //   printDesc("pRRT (blocked)");
 //   zeroTimerGPU();
 //   auto pRRTBlocked = gpuRRT(gridBlocked, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep, 0.15f);
 //   printElapsedTimeGPU("(CUDA measured)");
 //   printf("pRRT path length: %zu\n", pRRTBlocked.size());
 //   printCmpPath(cpuBlocked, pRRTBlocked);


    // ======================================
// TEST 4: Larger Corridor Map
// ======================================
    printHeader("TEST 4: Corridor map (20x10)");

    // A corridor-style map
    std::vector<int> bigCorridor = {
     0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,
     0,1,1,1,1,1,1,1,1,0,  0,1,1,1,1,1,1,1,1,0,
     0,1,0,0,0,0,0,0,1,0,  0,1,0,0,0,0,0,0,1,0,
     0,1,0,1,1,1,1,0,1,0,  0,1,0,1,1,1,1,0,1,0,
     0,1,0,1,0,0,1,0,1,0,  0,1,0,1,0,0,1,0,1,0,
     0,1,0,1,0,0,1,0,1,0,  0,1,0,1,0,0,1,0,1,0,
     0,1,0,1,1,1,1,0,1,0,  0,1,0,1,1,1,1,0,1,0,
     0,1,0,0,0,0,0,0,1,0,  0,1,0,0,0,0,0,0,1,0,
     0,0,0,0,0,0,0,0,0,0,  0,1,1,1,1,1,1,1,1,0,
     0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0
    };
    float margin = 0.15f;      // meters

    int radius = int(margin / resolution); // = 3



	inflateObstaclesCPU(bigCorridor, 20, 10, 0.99);

    OccupancyGrid gridCorridor = makeGrid(bigCorridor, 20, 10);
   
    printDesc("CPU RRT (corridor)");
    zeroTimerCPU();
    timerCPU().startCpuTimer();
    auto cpuCorr = cpuRRT(gridCorridor, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
    timerCPU().endCpuTimer();
    printElapsedTimeCPU("(std::chrono measured)");
    printf("CPU path length: %zu\n", cpuCorr.size());

    printDesc("GPU RRT (corridor)");
    zeroTimerGPU();
    auto gpuCorr = launchRRT(gridCorridor, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
    printElapsedTimeGPU("(CUDA measured)");
    printf("GPU path length: %zu\n", gpuCorr.size());
	//printPath(gpuCorr);

    printCmpPath(cpuCorr, gpuCorr);


    printDesc("pRRT (corridor)");
    zeroTimerpRRT();
    auto pRRTCorr = gpuRRT(gridCorridor, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
    printElapsedTimepRRT("(CUDA measured)");
    printf("pRRT path length: %zu\n", pRRTCorr.size());
    printCmpPath(cpuCorr, pRRTCorr);


    // ======================================
    // TEST 5: Random map (seeded)
    // ======================================
    printHeader("TEST 5: Random Map (1000x1000)");


    int RW = 1000;
    int RH = 1000;
	goalX = 600;
	goalY = 600;

    srand(12345);
    std::vector<int> randMap(RW* RH);

    for (int i = 0; i < RW * RH; i++) {
        randMap[i] = (rand() % 6 == 0) ? 1 : 0;   // ~16% obstacles
    }

    // Ensure start and goal are not obstacles
    int startIdx = (int)startY * RW + (int)startX;
    int goalIdx = (int)goalY * RW + (int)goalX;

    randMap[startIdx] = 0;
    randMap[goalIdx] = 0;

    OccupancyGrid gridRand = makeGrid(randMap, RW, RH);

    printDesc("CPU RRT (random)");
    zeroTimerCPU();
    timerCPU().startCpuTimer();
    auto cpuRand = cpuRRT(gridRand, startX, startY, goalX, goalY,
        maxIter, maxNodes, maxStep);
    timerCPU().endCpuTimer();
    printElapsedTimeCPU("(std::chrono measured)");
    printf("CPU path length: %zu\n", cpuRand.size());

    printDesc("GPU RRT (random)");
    zeroTimerGPU();
    auto gpuRand = launchRRT(gridRand, startX, startY, goalX, goalY,
        maxIter, maxNodes, maxStep);
    printElapsedTimeGPU("(CUDA measured)");
    printf("GPU path length: %zu\n", gpuRand.size());

    printCmpPath(cpuRand, gpuRand);


    printDesc("pRRT (random)");
    zeroTimerpRRT();
    auto pRRTRand= gpuRRT(gridRand, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
    printElapsedTimepRRT("(CUDA measured)");
    printf("pRRT path length: %zu\n", pRRTRand.size());
    printCmpPath(cpuRand, pRRTRand);





    printf("\n*** RRT TESTING COMPLETE ***\n");

    return 0;
}

