#pragma once
struct TreeNode {
    float x, y;
    int parent;
};

struct OccupancyGrid {
    //2D array grid data 
    uint8_t* data;
    int width, height;
    float resolution;
    float origin_x, origin_y;
};
