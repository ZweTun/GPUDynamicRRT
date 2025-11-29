#include "gpu_rrt_planner.hpp"

#include <cuda_runtime.h>
#include <thrust/device_ptr.h>
#include <thrust/extrema.h>
#include <thrust/execution_policy.h>
#include <cmath>

/// Device pointers and map parameters (static internal state)
static float* d_nodes_x = nullptr;
static float* d_nodes_y = nullptr;
static float* d_distances = nullptr;
static uint8_t* d_map = nullptr;
static int* d_collisionFlag = nullptr;
static int map_width = 0;
static int map_height = 0;
static float map_resolution = 0.0f;
static float map_origin_x = 0.0f;
static float map_origin_y = 0.0f;
static int max_nodes_allocated = 0;

/// Kernel to compute distances from a query point (rx, ry) to all existing nodes.
__global__ void computeDistancesKernel(float rx, float ry, const float* nodes_x, const float* nodes_y, int n_nodes, float* distances) {
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n_nodes) {
        float dx = rx - nodes_x[i];
        float dy = ry - nodes_y[i];
        distances[i] = dx * dx + dy * dy;
    }
}

/// Kernel to check collision along a line segment by sampling points.
__global__ void collisionKernel(float x1, float y1, float x2, float y2,
                                const uint8_t* map, int width, int height,
                                float res, float orig_x, float orig_y,
                                int steps, int* collisionFlag) {
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i > steps) return;
    // If another thread already found a collision, exit early.
    if (*collisionFlag) return;
    // Parameter t from 0 to 1 for this sample point
    float t = (float) i / (float) steps;
    float x = x1 + t * (x2 - x1);
    float y = y1 + t * (y2 - y1);
    // Compute map cell indices
    int cx = (int) floorf((x - orig_x) / res);
    int cy = (int) floorf((y - orig_y) / res);
    if (cx < 0 || cx >= width || cy < 0 || cy >= height) {
        // Out of bounds is considered a collision
        atomicExch(collisionFlag, 1);
    } else {
        if (map[cx + cy * width] != 0) {
            // Occupied cell detected
            atomicExch(collisionFlag, 1);
        }
    }
}

void allocateRRTResources(int max_nodes) {
    max_nodes_allocated = max_nodes;
    // Allocate device memory for node coordinates and distance buffer
    cudaMalloc((void**)&d_nodes_x, max_nodes * sizeof(float));
    cudaMalloc((void**)&d_nodes_y, max_nodes * sizeof(float));
    cudaMalloc((void**)&d_distances, max_nodes * sizeof(float));
    cudaMalloc((void**)&d_collisionFlag, sizeof(int));
}

void freeRRTResources() {
    if (d_nodes_x) cudaFree(d_nodes_x);
    if (d_nodes_y) cudaFree(d_nodes_y);
    if (d_distances) cudaFree(d_distances);
    if (d_collisionFlag) cudaFree(d_collisionFlag);
    if (d_map) cudaFree(d_map);
    d_nodes_x = d_nodes_y = nullptr;
    d_distances = nullptr;
    d_collisionFlag = nullptr;
    d_map = nullptr;
}

void setMapData(const uint8_t* map_data, int width, int height, float resolution, float origin_x, float origin_y) {
    // If map size changed, reallocate device memory
    if (d_map && (width * height != map_width * map_height)) {
        cudaFree(d_map);
        d_map = nullptr;
    }
    map_width = width;
    map_height = height;
    map_resolution = resolution;
    map_origin_x = origin_x;
    map_origin_y = origin_y;
    if (!d_map) {
        cudaMalloc((void**)&d_map, width * height * sizeof(uint8_t));
    }
    // Copy map data (host to device)
    cudaMemcpy(d_map, map_data, width * height * sizeof(uint8_t), cudaMemcpyHostToDevice);
}

void addNodeToDevice(float x, float y, int index) {
    // Copy one node's coordinates to device arrays at the given index
    cudaMemcpy(d_nodes_x + index, &x, sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_nodes_y + index, &y, sizeof(float), cudaMemcpyHostToDevice);
}

int findNearestNode(float x, float y, int n_nodes) {
    if (n_nodes <= 0) return -1;
    // Launch kernel to compute distances to all nodes
    int threads = 256;
    int blocks = (n_nodes + threads - 1) / threads;
    computeDistancesKernel<<<blocks, threads>>>(x, y, d_nodes_x, d_nodes_y, n_nodes, d_distances);
    cudaDeviceSynchronize();  // ensure distances are computed
    // Use Thrust to find the index of minimum distance
    thrust::device_ptr<float> dist_ptr(d_distances);
    thrust::device_ptr<float> dist_ptr_end = dist_ptr + n_nodes;
    auto min_iter = thrust::min_element(thrust::device, dist_ptr, dist_ptr_end);
    // Compute index of minimum element
    int index = min_iter - dist_ptr;
    return index;
}

bool checkCollision(float x1, float y1, float x2, float y2) {
    if (!d_map) {
        // No map available, assume no collision
        return false;
    }
    // Compute number of steps for sampling along the line (at least 1)
    float dx = x2 - x1;
    float dy = y2 - y1;
    float distance = sqrtf(dx * dx + dy * dy);
    int steps = (int) ceilf(distance / map_resolution);
    if (steps < 1) steps = 1;
    // Initialize collision flag to 0
    int zero = 0;
    cudaMemcpy(d_collisionFlag, &zero, sizeof(int), cudaMemcpyHostToDevice);
    // Launch collision check kernel
    int threads = 256;
    int totalSamples = steps + 1; // number of sample points (including start and end)
    int blocks = (totalSamples + threads - 1) / threads;
    collisionKernel<<<blocks, threads>>>(x1, y1, x2, y2, d_map, map_width, map_height, map_resolution, map_origin_x, map_origin_y, steps, d_collisionFlag);
    // Copy collision flag back to host (this will synchronize the kernel)
    int flag_host = 0;
    cudaMemcpy(&flag_host, d_collisionFlag, sizeof(int), cudaMemcpyDeviceToHost);
    // If flag is set (1), a collision was detected
    return flag_host != 0;
}
