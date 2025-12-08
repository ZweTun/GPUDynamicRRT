
#include <cuda_runtime.h>
#include "rrt.h"
#include <cuda.h>\
#include "common.h"
#include "cmath"
#include <device_launch_parameters.h>
#include <vector>
#include <thrust/version.h>
#include <thrust/random.h>
#include <unordered_set>


// Devices

// Computes Euclidean distance between two points
__device__ float distanceSquared(float x1, float y1, float x2, float y2) {
	return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

// Checks if the given point is in free space 
__device__ bool isPointFree(const OccupancyGrid* grid, float x, float y) {
    int gx = (x - grid->origin_x) / grid->resolution;
    int gy = (y - grid->origin_y) / grid->resolution;
    if (gx < 0 || gx >= grid->width || gy < 0 || gy >= grid->height) {
        return false; // Out of bounds
	}
    int idx = gy * grid->width + gx;
	//0 means free space
    return grid->data[idx] == 0;
}

// Checks if the given point is within a threshold distance to the goal
__device__ bool isGoal(float x, float y, float goalX, float goalY) {
	float threshold = 0.15f; //  threshold
    return distanceSquared(x, y, goalX, goalY) < threshold * threshold;
}



// Samples a random point in free space
// Should call device_isPointFree to ensure sampled point is in free space
// Uses thrust random number generators
__device__ TreeNode sampleFreeSpace(OccupancyGrid* grid, int iter, float goalX, float goalY) {

	//Jittered random seed based on thread and iteration

    unsigned tid = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned seed = (tid * 1337u) ^ (iter * 911u);

    thrust::default_random_engine rng(seed);
    thrust::uniform_real_distribution<float> dist01(0.0f, 1.0f);

    int maxAttempts = 1000;

    for (int attempt = 0; attempt < maxAttempts; attempt++) {

        float rx = dist01(rng);
        float ry = dist01(rng);

        int cx = int(rx * grid->width);
        int cy = int(ry * grid->height);

        float x = grid->origin_x + (cx + 0.5f) * grid->resolution;
        float y = grid->origin_y + (cy + 0.5f) * grid->resolution;


        if (isPointFree(grid, x, y)) {
            return { x, y, -1 };
        }
    }

    // fallback: center sample
    float cx = grid->origin_x + 0.5f * grid->width * grid->resolution;
    float cy = grid->origin_y + 0.5f * grid->height * grid->resolution;

    return { cx, cy, -1 };
}

// Returns index of nearest neighbor in the tree 
__device__ int nearestNeighbor(TreeNode* tree, int tree_size, float rand_x, float rand_y) {
    //TODO
	int nearestIdx = -1;
    float minDist = 1e10f;
    for (int i = 0; i < tree_size; ++i) {
        float dist = distanceSquared(tree[i].x, tree[i].y, rand_x, rand_y);
        if (dist < minDist) {
            minDist = dist;
            nearestIdx = i;
        }
    }
	return nearestIdx;
}


// Steers from 'nearest' towards 'sampled' by at most maxStep distance
__device__ TreeNode steer(const TreeNode& from, const TreeNode& to, float maxStep) {
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist <= maxStep) {
        return to;
    }
    else {

        return { from.x + (dx/dist) * maxStep, from.y + (dy/dist) * maxStep, -1};
    }
}


//Checks for collision between two points
__device__ bool checkCollision(OccupancyGrid* grid, float x1, float y1, float x2, float y2) {
    //TODO
	int x_diff = abs(int(ceil((x2 - x1) / grid->resolution)));
    int y_diff = abs(int(ceil((y2 - y1) / grid->resolution)));

    int steps = -1;
    if (x_diff > y_diff) {
        steps = x_diff;
    }
    else {
        steps = y_diff;
	}
    if (steps == 0) {
        return !isPointFree(grid, x2, y2); // No movement
    }

    float dt = 1.0 / steps;
    float t = 0.0;

	//Check points along the line
    for (int i = 0; i <= steps; ++i) {
		float x = x1 + t * (x2 - x1);
        float y = y1 + t * (y2 - y1);
        if (!isPointFree(grid, x, y)) {
            return true; // Collision detected
        }
		t += dt;
    }
	return false; // No collision
}




// Kernels
__global__ void kernInitTree(TreeNode* tree, int max_nodes, float start_x, float start_y) {
    int index = threadIdx.x + (blockIdx.x * blockDim.x);
    if (index == 0) {
        tree[0].x = start_x;
        tree[0].y = start_y;
        tree[0].parent = -1;
    }
}


// RRT main kernel
__global__ void kernRRT(
    int maxIter,
    int maxNodes,
	int numThreads,
    OccupancyGrid grid,
    float startX, float startY,
    float goalX, float goalY,
    TreeNode* allTrees,
    int* results, float maxStep    // size = numThreads
) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid >= numThreads) return;
  //  printf("Thread %d running RRT\n", threadIdx.x);

    // Each thread uses RRT tree region:
    TreeNode* tree = &allTrees[tid * maxNodes];

    int size = 1;
    tree[0] = { startX, startY, -1 };
    results[tid] = -1;  

    for (int iter = 1; iter < maxIter && size < maxNodes; ++iter) {
        TreeNode newNode = sampleFreeSpace(&grid, iter, goalX, goalY);
        int nearestIdx = nearestNeighbor(tree, size, newNode.x, newNode.y);
        TreeNode nearest = tree[nearestIdx];
		newNode = steer(nearest, newNode, maxStep); //Steer with max range 0.5
       
        unsigned seed = (tid * 1337u) ^ (iter * 911u);
        thrust::default_random_engine rng(seed);
        thrust::uniform_real_distribution<float> dist01(0.0f, 1.0f);
        if (dist01(rng) < 0.1f) {
            newNode.x = goalX;
            newNode.y = goalY;
		}
        if (!checkCollision(&grid, nearest.x, nearest.y, newNode.x, newNode.y)) {
            newNode.parent = nearestIdx;
            int idx = size++;
            tree[idx] = newNode;
            
            if (isGoal(newNode.x, newNode.y, goalX, goalY)) {

				// Path found
				// Store the index of the goal node
				// result[tid] will be used to trace back the path later
				// final node on host side will be tree[results[tid]]
               // printf("Thread %d found RRT Path\n", threadIdx.x);

                results[tid] = idx;  
                return;              
            }
        }
    }
    
	//No path found
}



std::vector<TreeNode> findFinalPath(TreeNode* tree, int goalIdx) {
    std::vector<TreeNode> path;
    int idx = goalIdx;

    // protect against infinite loops
    std::unordered_set<int> visited;

    while (idx >= 0) {
        if (visited.count(idx)) {
            std::cout << "ERROR: cycle in parent pointers\n";
            break;
        }
        visited.insert(idx);

        path.push_back(tree[idx]);
        idx = tree[idx].parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}



using RRT::Common::PerformanceTimer;
PerformanceTimer& timerGPU()
{
    static PerformanceTimer timer;
    return timer;
}

std::vector<TreeNode> launchRRT(const OccupancyGrid& h_grid,
    float startX, float startY,
    float goalX, float goalY, int maxIter, int maxNodes, float maxStep) {
    timerGPU().startGpuTimer();

	// parameters
    int numThreads = 1024;
	// result path
    std::vector<TreeNode> path;
  
    OccupancyGrid d_grid;

    size_t gridBytes = h_grid.width * h_grid.height * sizeof(uint8_t);
    cudaMalloc(&d_grid.data, gridBytes);
    cudaMemcpy(d_grid.data, h_grid.data, gridBytes, cudaMemcpyHostToDevice);

    d_grid.width = h_grid.width;
    d_grid.height = h_grid.height;
    d_grid.resolution = h_grid.resolution;
    d_grid.origin_x = h_grid.origin_x;
    d_grid.origin_y = h_grid.origin_y;

	// allocations of GPU memory
    TreeNode* d_allTrees;
    cudaMalloc(&d_allTrees, numThreads * maxNodes * sizeof(TreeNode));

    int* d_results;
    cudaMalloc(&d_results, numThreads * sizeof(int));

    TreeNode* h_allTrees = new TreeNode[numThreads * maxNodes];
    int* h_results = new int[numThreads];

	// launch kernels
    dim3 block(128);
    dim3 gridDim((numThreads + block.x - 1) / block.x);
    //timerGPU().startGpuTimer();

    kernRRT <<<gridDim, block >>> (
        maxIter,
        maxNodes,
		numThreads,
        d_grid,
        startX, startY,
        goalX, goalY,
        d_allTrees,
        d_results, maxStep
        );
    cudaDeviceSynchronize();
    //timerGPU().endGpuTimer();
 



  
   // Copy back to host
    cudaMemcpy(h_allTrees, d_allTrees,
        numThreads * maxNodes * sizeof(TreeNode),
        cudaMemcpyDeviceToHost);

    cudaMemcpy(h_results, d_results,
        numThreads * sizeof(int),
        cudaMemcpyDeviceToHost);

	// find a successful path
    for (int tid = 0; tid < numThreads; ++tid) {
        if (h_results[tid] != -1) {
            int goalIndex = h_results[tid];
            TreeNode* treeBase = &h_allTrees[tid * maxNodes];
			
            // Print tree size
			///printf("Thread %d found RRT Path, tree size = %d\n", tid, goalIndex + 1);
            path = findFinalPath(treeBase, goalIndex);
            break;
			
        }
    } 


    // Clean up
    cudaFree(d_grid.data);
    cudaFree(d_allTrees);
    cudaFree(d_results);
    delete[] h_allTrees;
    delete[] h_results;

    timerGPU().endGpuTimer();
    return path; // may be empty if no solution






}
