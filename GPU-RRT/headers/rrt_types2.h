#pragma once
#include <cstdint>
struct TreeNode {
    float x, y;
    TreeNode* parent;
};

struct OccupancyGrid {
    //2D array grid data 
    uint8_t* data;
    int width, height;
    float resolution;
    float origin_x, origin_y;
};
