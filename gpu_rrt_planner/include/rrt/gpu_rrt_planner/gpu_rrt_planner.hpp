#pragma once

#include <cstdint>

/// Initialize GPU resources for RRT (allocate memory for nodes arrays etc.).
void allocateRRTResources(int max_nodes);

/// Free GPU resources (deallocate memory).
void freeRRTResources();

/// Copy occupancy grid map data to GPU (allocates memory if needed).
/// Map data should be width*height bytes, with 0 for free and 1 for occupied.
void setMapData(const uint8_t* map_data, int width, int height, float resolution, float origin_x, float origin_y);

/// Add a new RRT node (x, y) at the given index to GPU memory (so it can be used for nearest-neighbor computations).
void addNodeToDevice(float x, float y, int index);

/// Find index of the nearest existing node in the RRT to the point (x, y) using GPU parallel computation.
int findNearestNode(float x, float y, int n_nodes);

/// Check for collision along the line segment from (x1, y1) to (x2, y2) using GPU parallelization.
/// Returns true if a collision (obstacle) is detected, false if path is free.
bool checkCollision(float x1, float y1, float x2, float y2);
