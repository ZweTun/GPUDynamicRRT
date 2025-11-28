#pragma once

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <string>
#include <ctime>
#include <vector>
#include <cmath>
#include "rrt_types.h"   


template<typename T>
int cmpArrays(int n, T* a, T* b) {
    for (int i = 0; i < n; i++) {
        if (a[i] != b[i]) {
            printf("    a[%d] = %d, b[%d] = %d\n", i, a[i], i, b[i]);
            return 1;
        }
    }
    return 0;
}

void printDesc(const char* desc) {
    printf("==== %s ====\n", desc);
}

template<typename T>
void printCmpResult(int n, T* a, T* b) {
    printf("    %s\n",
        cmpArrays(n, a, b) ? "FAIL VALUE" : "passed");
}

template<typename T>
void printCmpLenResult(int n, int expN, T* a, T* b) {
    if (n != expN) {
        printf("    expected %d elements, got %d\n", expN, n);
    }
    printf("    %s\n",
        (n == -1 || n != expN) ? "FAIL COUNT" :
        cmpArrays(n, a, b) ? "FAIL VALUE" : "passed");
}

void zeroArray(int n, int* a) {
    for (int i = 0; i < n; i++) a[i] = 0;
}

void onesArray(int n, int* a) {
    for (int i = 0; i < n; i++) a[i] = 1;
}

void genArray(int n, int* a, int maxval) {
    srand(time(nullptr));
    for (int i = 0; i < n; i++) a[i] = rand() % maxval;
}

void printArray(int n, int* a, bool abridged = false) {
    printf("    [ ");
    for (int i = 0; i < n; i++) {
        if (abridged && i + 2 == 15 && n > 16) {
            i = n - 2;
            printf("... ");
        }
        printf("%3d ", a[i]);
    }
    printf("]\n");
}

template<typename T>
void printElapsedTime(T time, std::string note = "") {
    std::cout << "   elapsed time: " << time << " ms   " << note << std::endl;
}



// Big ASCII section headers
inline void printHeader(const std::string& title) {
    std::cout << "\n=====================================\n";
    std::cout << title << "\n";
    std::cout << "=====================================\n";
}

// Print a path
inline void printPath(const std::vector<TreeNode>& path) {
    for (const auto& n : path) {
        printf("   (%.3f, %.3f),\n", n.x, n.y);
    }
}

// Check if two paths reach the same goal (loosely)
inline bool pathsSimilar(const std::vector<TreeNode>& a,
    const std::vector<TreeNode>& b) {
    if (a.empty() && b.empty()) return true;
    if (a.empty() || b.empty()) return false;

    TreeNode endA = a.back();
    TreeNode endB = b.back();

    float dx = endA.x - endB.x;
    float dy = endA.y - endB.y;

    return (dx * dx + dy * dy) < 0.25f;   // within 0.5 distance
}

// Compare CPU and GPU RRT paths
inline void printCmpPath(const std::vector<TreeNode>& cpu,
    const std::vector<TreeNode>& gpu) {
    if (cpu.empty() && gpu.empty()) {
        printf("    BOTH FAILED  passed\n");
        return;
    }

    if (cpu.empty() && !gpu.empty()) {
        printf("    CPU failed, GPU succeeded\n");
        return;
    }
    if (!cpu.empty() && gpu.empty()) {
        printf("    GPU failed, CPU succeeded FAIL GPU\n");
        return;
    }

    if (pathsSimilar(cpu, gpu)) {
        printf("    Paths reach similar goal  passed\n");
    }
    else {
        printf("    WARNING: paths differ significantly\n");
    }
}


#include "common.h"  

using RRT::Common::PerformanceTimer;

using RRT::Common::timerCPU;
using RRT::Common::timerGPU;

// CPU timer helpers
inline void zeroTimerCPU() {
    // reset timer by starting & stopping immediately
    timerCPU().startCpuTimer();
    timerCPU().endCpuTimer();
}

inline float getCPUTime() {
    return timerCPU().getCpuElapsedTimeForPreviousOperation();
}

inline void printElapsedTimeCPU(const std::string& note) {
    printElapsedTime(getCPUTime(), note);
}


// GPU timer helpers
inline void zeroTimerGPU() {
    timerGPU().startGpuTimer();
    timerGPU().endGpuTimer();
}

inline float getGPUTime() {
    return timerGPU().getGpuElapsedTimeForPreviousOperation();
}

inline void printElapsedTimeGPU(const std::string& note) {
    printElapsedTime(getGPUTime(), note);
}
