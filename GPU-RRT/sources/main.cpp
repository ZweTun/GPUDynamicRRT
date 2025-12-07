#include <iostream>
#include <cuda.h>]
#include <cuda_runtime.h>
#include "rrt.h"
#include "cpu_rrt.h"
#include "pRRT.h"

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

void inflateObstacles(
    std::vector<int>& grid,
    int width,
    int height,
    float inflation_meters,
    float resolution)
{
    int iterations = static_cast<int>(inflation_meters / resolution);
    if (iterations <= 0) return;

    std::vector<int> inflated = grid;

    for (int it = 0; it < iterations; it++) {
        std::vector<int> temp = inflated;

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {

                if (inflated[y * width + x] == 0) continue; // not obstacle

                // Expand into 8 neighbors 
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int nx = x + dx;
                        int ny = y + dy;

                        if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                            continue;

                        temp[ny * width + nx] = 100; // match ROS occupancy value
                    }
                }
            }
        }

        inflated.swap(temp);
    }

    grid = inflated;
}


//OccupancyGrid makeGrid(const std::vector<int>& visual, int W, int H, float resolution)
//{
//    OccupancyGrid g;
//    g.width = W;
//    g.height = H;
//    g.resolution = resolution;
//    g.origin_x = 0;
//    g.origin_y = 0;
//
//    // allocate exact size
//    uint8_t* buf = new uint8_t[W * H];
//
//    for (int i = 0; i < W * H; i++) {
//        buf[i] = (uint8_t)visual[i];
//    }
//
//    g.data = buf;
//    return g;
//}

OccupancyGrid makeGrid(const std::vector<int>& visual, int W, int H, float resolution, float inflation_m = 1.00)
{
    OccupancyGrid g;
    g.width = W;
    g.height = H;
    g.resolution = resolution;
    g.origin_x = 0.0f;
    g.origin_y = 0.0f;

    std::vector<int> inflated = visual;

    // Inflate obstacles like binary_dilation
    inflateObstacles(inflated, W, H, inflation_m, resolution);

    uint8_t* buf = new uint8_t[W * H];
    for (int i = 0; i < W* H; i++) {
        buf[i] = (inflated[i] > 0 ? 1 : 0);
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

    float resolution = 1.0f;
    float maxStep = 0.5f;
    int maxIter = 10000;
    int maxNodes = 100000;


    // Define start and goal positions
    float startX = 0.0f, startY = 0.0f;
    float goalX = 99.0f, goalY = 99.0f;
    float height = 100.0f, width = 100.0f;

    //
    // Test 1: Empty map
    //
    printHeader("TEST 1: Empty map (100x100)");
    startX = 0.0f, startY = 0.0f;
    goalX = 99.0f, goalY = 99.0f;
     height = 100.0f, width = 100.0f;

    std::vector<int> emptyMap(width * height, 0);
    OccupancyGrid gridEmpty = makeGrid(emptyMap, width, height, resolution);


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
 //   printHeader("TEST 2: Maze map (10x10)");

 //   std::vector<int> mazeMap = {
 //       0,0,0,0,0,0,0,0,0,0,
 //       0,0,0,1,1,1,0,0,0,0,
 //       0,1,0,0,0,1,0,1,0,0,
 //       0,1,0,1,0,1,0,1,0,0,
 //       0,0,0,1,0,1,0,0,0,0,
 //       0,1,0,1,0,1,1,1,0,0,
 //       0,1,0,0,0,0,0,0,0,0,
 //       0,0,0,0,0,0,0,0,0,0,
 //       0,0,0,0,0,0,0,0,0,0,
 //       0,0,0,0,0,0,0,0,0,0
 //   };

 //   OccupancyGrid gridMaze = makeGrid(mazeMap, 10, 10, resolution);

 //   printDesc("CPU RRT (maze)");
 //   zeroTimerCPU();
 //   timerCPU().startCpuTimer();
 //   auto cpuMaze = cpuRRT(gridMaze, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
	//timerCPU().endCpuTimer();
 //   printElapsedTimeCPU("(std::chrono measured)");
 //   printf("CPU path length: %zu\n", cpuMaze.size());

 //   printDesc("GPU RRT (maze)");
 //   zeroTimerGPU();
 //   auto gpuMaze = launchRRT(gridMaze, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
 //   printElapsedTimeGPU("(CUDA measured)");
 //   printf("GPU path length: %zu\n", gpuMaze.size());
 //   printCmpPath(cpuMaze, gpuMaze);

 //   // GPU pRRT
 //   printDesc("pRRT (maze)");
 //   zeroTimerpRRT();
 //   auto pRRTMaze = gpuRRT(gridMaze, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
 //   printElapsedTimepRRT("(CUDA measured)");
 //   printf("pRRT path length: %zu\n", pRRTMaze.size());
 //   printCmpPath(cpuMaze, pRRTMaze);
 //   printPath(pRRTMaze);




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
    printHeader("TEST 4: Corridor map (60x120)");

     height = 60;
	 width = 120;
	 startX = 5;
	 startY = 45;
	 goalX = 115;
	 goalY = 30;

	 std::vector<int> bigCorridor = std::vector<int>(width * height, 0);
	 // Add walls to create a corridor
     int corridor_top = 10;   // distance from top wall
     int corridor_bottom = height - 10;  // distance from bottom wall


	 int corridor_left = 10; // width of the corridor
	 int corridor_right = width - 10;
     for (int y = corridor_top; y < corridor_bottom; y++) {
         for (int x = corridor_left; x < corridor_right; x++) {
			 // Fill in obstacles 
			 bigCorridor[y * width + x] = 1;
		 }
     }

    OccupancyGrid gridCorridor = makeGrid(bigCorridor, width, height, resolution);
    
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


    printCmpPath(cpuCorr, gpuCorr);
    printPath(gpuCorr);


    printDesc("pRRT (corridor)");
    zeroTimerpRRT();
    auto pRRTCorr = gpuRRT(gridCorridor, startX, startY, goalX, goalY, maxIter, maxNodes, maxStep);
    printElapsedTimepRRT("(CUDA measured)");
    printf("pRRT path length: %zu\n", pRRTCorr.size());
    printCmpPath(cpuCorr, pRRTCorr);
    printPath(pRRTCorr);
    //printPath(gpuCorr);


    // ======================================
    // TEST 5: Random map (seeded)
    // ======================================
    printHeader("TEST 5: Random Map (1000x1000)");


    height = 1000;
    width = 1000;
    startX = 5;
    startY = 45;
    goalX = 500;
    goalY = 785;

    srand(12345);
    std::vector<int> randMap(width * height);

    for (int i = 0; i < width * height; i++) {
        randMap[i] = (rand() % 6 == 0) ? 1 : 0;   // ~16% obstacles
    }

    // Ensure start and goal are not obstacles
    int startIdx = (int)startY * width + (int)startX;
    int goalIdx = (int)goalY * width + (int)goalX;

    randMap[startIdx] = 0;
    randMap[goalIdx] = 0;

    OccupancyGrid gridRand = makeGrid(randMap, width, height, resolution, 0.0f);

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

