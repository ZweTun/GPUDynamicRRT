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



float computeCost(const std::vector<TreeNode>& path) {
    if (path.size() < 2) return 0.0f;

    float cost = 0.0f;
    for (int i = 0; i < path.size() - 1; i++) {
        float dx = path[i + 1].x - path[i].x;
        float dy = path[i + 1].y - path[i].y;
        cost += sqrtf(dx * dx + dy * dy);
    }
    return cost;
}


float median(std::vector<float> v) {
    if (v.empty()) return 0.0f;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

float percentile(std::vector<float> v, float p) {
    if (v.empty()) return 0.0f;
    std::sort(v.begin(), v.end());
    int idx = p * (v.size() - 1);
    return v[idx];
}

float mean(const std::vector<float>& v) {
    float s = 0;
    for (float x : v) s += x;
    return v.empty() ? 0.0f : s / v.size();
}


void profileMap(const char* name,
    const OccupancyGrid& grid,
    float startX, float startY,
    float goalX, float goalY,
    int maxIter, int maxNodes, float maxStep)
{
    const int TRIALS = 50;
    std::vector<float> times_cpu, costs_cpu;
    std::vector<float> times_gpu, costs_gpu;
    std::vector<float> times_prrt, costs_prrt;

    int solved_cpu = 0, solved_gpu = 0, solved_prrt = 0;

    for (int i = 0; i < TRIALS; i++) {
	   printf("\rTrial %d/%d...", i + 1, TRIALS);
        // CPU ------------------------------------------------------
        zeroTimerCPU();
        timerCPU().startCpuTimer();
        auto cpuPath = cpuRRT(grid, startX, startY, goalX, goalY,
            maxIter, maxNodes, maxStep);
        timerCPU().endCpuTimer();
        float t_cpu = getCPUTime();
        //printf("%.4f \n", t_cpu);
        if (!cpuPath.empty()) {
            solved_cpu++;
            times_cpu.push_back(t_cpu);
            costs_cpu.push_back(computeCost(cpuPath));
        }

        // GPU RRT ---------------------------------------------------
        zeroTimerGPU();
      
        auto gpuPath = launchRRT(grid, startX, startY, goalX, goalY,
            maxIter, maxNodes, maxStep);
		
        float t_gpu = getGPUTime();
        //printf("%.4f \n", t_gpu);

        if (!gpuPath.empty()) {
            solved_gpu++;
            times_gpu.push_back(t_gpu);
            costs_gpu.push_back(computeCost(gpuPath));
        }

        // pRRT ------------------------------------------------------
        zeroTimerpRRT();
        int pmaxNodes = 1000000;
       
        auto prrtPath = gpuRRT(grid, startX, startY, goalX, goalY,
            maxIter, pmaxNodes, maxStep);
       
        float t_prrt = getpRRTTime();
        //Print each t_pprt  
	   // printf("%.4f \n", t_prrt);
	
        if (!prrtPath.empty()) {
            solved_prrt++;
            times_prrt.push_back(t_prrt);
            costs_prrt.push_back(computeCost(prrtPath));
        }
    }

    // ===================== PRINT RESULTS =====================
    printf("\n===== PROFILE: %s =====\n", name);

    auto printStats = [&](const char* title,
        int solved, std::vector<float>& T, std::vector<float>& C)
        {
            printf("\n%s\n", title);
            printf("Solved: %d/%d (%.1f%%)\n", solved, TRIALS,
                100.0f * solved / TRIALS);

            if (!T.empty()) {
                printf("Mean time: %.4f ms\n", mean(T));
                printf("Median time: %.4f ms\n", median(T));
                printf("Q1: %.4f ms\n", percentile(T, 0.25f));
                printf("Q3: %.4f ms\n", percentile(T, 0.75f));
                printf("95th percentile: %.4f ms\n", percentile(T, 0.95f));
            }

            if (!C.empty()) {
                printf("Mean cost: %.4f\n", mean(C));
                printf("Median cost: %.4f\n", median(C));
            }
        };

    printStats("CPU RRT:", solved_cpu, times_cpu, costs_cpu);
    printStats("GPU RRT:", solved_gpu, times_gpu, costs_gpu);
    printStats("pRRT:", solved_prrt, times_prrt, costs_prrt);
}


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

    float resolution = 1.0f;
    float maxStep = 1.0f;
    int maxIter = 10000;
    int maxNodes = 100000;


    // Define start and goal positions
    float startX = 0.0f, startY = 0.0f;
    float goalX = 99.0f, goalY = 99.0f;
    float height = 100.0f, width = 100.0f;


    // Test 0: Empty ------------------------------------
    printHeader("TEST 0: Empty map (100x100)");
    startX = 0.0f, startY = 0.0f;
    goalX = 99.0f, goalY = 99.0f;
    height = 100.0f, width = 100.0f;

    std::vector<int> emptyMap(width * height, 0);
    OccupancyGrid gridEmpty = makeGrid(emptyMap, width, height, resolution);

    profileMap("EMPTY MAP", gridEmpty,
        startX, startY, goalX, goalY,
        maxIter, maxNodes, maxStep);


    // Test 1: Corridor ------------------------------------
    printHeader("TEST 1: Corridor map (60x120)");

    height = 60;
    width = 120;
    startX = 5;
    startY = 30;
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
    profileMap("Corridor MAP", gridCorridor,
        startX, startY, goalX, goalY,
        maxIter, maxNodes, maxStep);


	// Test 3: Random Obstacles ------------------------------------
	printHeader("TEST 3: Random Obstacles (1000x1000)");
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

    //Inflation turned off for large map test
    OccupancyGrid gridRand = makeGrid(randMap, width, height, resolution, 0.0f);


    profileMap("Random Obstacles MAP", gridRand,
        startX, startY, goalX, goalY,
		maxIter, maxNodes, maxStep);


    printf("\n*** RRT TESTING COMPLETE ***\n");

    return 0;
}

