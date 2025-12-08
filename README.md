# GPUDynamicRRT

CIS 5650 Final Project

* Zwe Tun
  * LinkedIn: <https://www.linkedin.com/in/zwe-tun-6b7191256/>
* Jefferson Koumba Moussadji Lu
* LinkedIn: <https://www.linkedin.com/in/-jeff-koumba-0b356721b/>
* Yunhao Qian
  * LinkedIn: <https://www.linkedin.com/in/yunhao-qian-026980170/>

Tested on: Intel(R) i7-14700HX, 2100 Mhz, RTX 5060 Laptop

## Project Overview

GPUDynamicRRT is a robotics project to develop a GPU-accelerated Rapidly-Exploring Random Tree (RRT) path planner for an autonomous F1Tenth racecar platform. The motivation comes from the demands of dynamic racing environments where obstacles, like other vehicles or moving pedestrians, appear or move unpredictably.

In such scenarios, a traditional CPU-based RRT planner might struggle to re-plan fast enough to avoid collisions, or might produce suboptimal paths due to limited sampling speed. The goal of GPUDynamicRRT was to leverage the parallel computing power of an on-board NVIDIA GPU to significantly accelerate RRT, enabling real-time path planning that can handle frequent re-planning and moving obstacles without slowing down the vehicle. 

By offloading the RRT computations to the GPU, the planner was intended to generate and evaluate far more samples in the same timeframe, greatly increasing the chances of finding a safe, feasible path quickly.

<p align="center">
  <img src="imgs/corridor.gif" />
  <p align="center">RRT for Dynamic Robotics: Fast, Reliable Path Planning</p>

<p align="center">
  <img src="imgs/Simulation_vs_Real_World.gif" />
  <p align="center">RRT on the F1tenth car vs RRT in simulation</p>

## Motivation

Originally, we expected the GPU-based planner to outperform a CPU-based RRT by a wide margin. In theory, the GPU could expand multiple branches of the search tree or run many RRT simulations in parallel, yielding a higher planning frequency and more reliable obstacle avoidance. The anticipated advantage was that the F1Tenth racecar could drive faster and more safely through dynamic courses, since a GPU-accelerated planner could continuously update the trajectory on the fly. The ultimate goal was to maintain real-time performance in cases where a single-threaded CPU planner might lag or even fail to find a path in time.

Despite the strong theoretical motivation for GPU acceleration, our latest benchmarks revealed a surprising result: in the current implementation, the CPU RRT actually outperforms the CUDA (GPU) RRT in both speed and reliability. This outcome is counter-intuitive given the GPU's potential, but it highlights the practical challenges and overheads in parallelizing RRT for the GPU. The sections below detail the system design, compare the CPU and GPU implementations, and present the benchmark results along with analysis of why the GPU version underperformed and how it can be improved.

## System Overview and Architecture

<p align="center">
  <img src="imgs/F1tenth_car.png" width="220"/> <img src="imgs/F1tenth_car2.png" width="220"/>
  <p align="center">F1tenth car hardware overview</p>
</p>

**Electrical Hardware Overview Walkthrough**

- 2D Lidar Scanner
- Nividia Nano Orin
- Lipo Battery

<p align="center">
  <img src="imgs/dev_environment.png" />
  <p align="center">Overview of the development and run-time architecture in GPUDynamicRRT</p>

GPUDynamicRRT is integrated into the F1Tenth autonomous racing framework using ROS 2. The planner exists as a ROS 2 node that can operate in either CPU or GPU mode, and it interfaces with other system components such as the simulator (or physical car sensors) and the vehicle control node. Figure 1 illustrates the high-level system architecture in the context of the F1Tenth platform (simulation and real car):

- The RRT Planner Node (our module) publishes planned paths as a series of waypoints. There are two variants of this node: rrt_cpu (a C++ CPU implementation) and rrt_cuda (a CUDA implementation). They are interchangeable via a configuration flag.

- The F1Tenth Simulator/Vehicle Interface provides the planner with necessary inputs. In simulation, a node from the f1tenth_gym_ros package publishes sensor data (e.g. LiDAR scans or occupancy grid) and odometry. On the real F1Tenth car, similar data comes from the car's sensors (2D LiDAR, IMU, etc.) and localization nodes (e.g. particle filter).

- The Waypoint Follower (Pure Pursuit or a similar controller) subscribes to the waypoints from the planner and drives the car along the path. We used an existing pure pursuit controller for following the RRT-generated waypoints.

- The Obstacle Tracker (if enabled) can provide dynamic obstacle information to the planner, which our RRT uses to avoid collisions. This may consist of a node that processes LiDAR scans to detect obstacles and feed them into the planning algorithm as moving obstacles with a limited lifespan.

- All components communicate via ROS topics, enabling modularity. For example, the RRT planner listens on a topic for the current goal or global waypoint (from a global planner or raceline) and the current occupancy grid (map with obstacles), and it outputs a local trajectory (waypoints) for the controller.

The entire software stack has been containerized for ease of deployment. We provide a Docker environment (see the 2025-11-13 branch in development) that includes ROS 2 (Humble), CUDA libraries, and all package dependencies. Using Docker Compose, we can launch all the necessary services, the simulator, the RRT planner node (CPU or GPU), the localization and control nodes, with a single command. This ensures consistency across different development machines and simplifies running the code on the physical F1Tenth car. The launch structure is configurable: a parameter use_gpu in the launch file toggles between starting the dynamic_rrt node in CPU mode (rrt_cpu) or GPU mode (rrt_cuda). For example, our ROS launch script includes both versions and chooses based on this flag (defaulting to CPU for now, given the current performance gap). The containerization and launch setup make it straightforward to switch implementations and integrate with the existing F1Tenth stack, whether in simulation or on the real car.


## GPU RRT Implementation Details

## CPU vs GPU RRT Implementation Comparison

**CPU RRT Implementation:** The CPU version of the planner (rrt_cpu) is written in C++ and is optimized for real-time performance on the car's multi-core ARM CPU. It improves upon an earlier Python implementation by using efficient data structures and algorithms for collision checking and tree expansion. The CPU RRT can optionally employ a few threads (in our tests we used 2 threads, set by num_workers: 2) to run multiple RRT searches in parallel on separate CPU cores. 
In practice, the CPU planner operates by iteratively growing a single search tree: it samples random points in the space, extends the tree toward those samples, and checks for a path to the goal. If dynamic obstacles are present, the planner re-plans at a fixed interval (e.g. every 300 ms) to update the path. The CPU implementation was expected to be the baseline for performance, something for the GPU to greatly surpass, and it indeed is quite efficient due to the simplicity of in-process memory access and low overhead per iteration.

**GPU RRT Implementation:** The GPU version (rrt_cuda) offloads the core of the RRT algorithm to CUDA kernels. Our initial GPU design follows a "brute-force parallelism" strategy: instead of one tree growing sequentially, we launch many RRT processes in parallel on the GPU. Specifically, each planning cycle spawns, say, 32 parallel RRT workers (configurable via num_workers, which we typically set to 32 for the GPU node). Each GPU thread (or thread block) independently grows its own RRT tree in the shared map, up to a maximum number of iterations (e.g. 2000 expansions per tree).
All these threads run concurrently on the GPU, exploring the space rapidly. The first thread that finds a valid path to the goal will signal success; the planner then copies that path back to the CPU and publishes it. If none of the GPU threads find a path within the allotted iterations, the planning cycle is considered a failure (no path found within the time bound).

This GPU approach was naively parallel but aimed to exploit raw parallelism quickly. We chose it as a stepping stone because it's conceptually simple: "launch N RRTs in parallel and hope one succeeds fast." This is in contrast to a more complex cooperative parallel RRT (our ultimate goal), where all threads work on one tree simultaneously (e.g. each thread handles part of the same tree's expansion in parallel). The cooperative approach can be more efficient (eliminating redundant work), but it is also more complicated to implement correctly, especially with dynamic obstacles and synchronization between threads. Our brute-force parallel approach, while potentially doing redundant work across independent trees, maximizes the use of GPU cores without needing fine-grained synchronization.

<p align="center">
  <img src="imgs/CUDA_approach.png" />
  <p align="center">CUDA RRT - Naive Approach vs. Final Goal</p>

**Integration and Data Handling:** Both implementations share some common logic for collision checking and path handling, to ensure they produce comparable paths. The map (occupancy grid) is provided to the GPU planner by copying it into GPU memory (currently on every planning cycle), and each GPU thread performs collision checks against this grid. The CPU planner accesses the map directly in memory. Both planners incorporate the same parameters for sampling (e.g. forward distance range, lateral sampling range, step size, goal tolerance) to ensure fairness in comparison. The planners also enforce a limit on planning time by capping iterations (here 2000) or stopping when the next control cycle is due (e.g. every 300 ms new plan). Notably, the GPU planner in its current form does fresh allocations of memory for each cycle, e.g., allocating arrays or thrust device vectors to store tree nodes for each thread , whereas the CPU planner reuses its data structures across cycles.

**Expectations vs Reality:** We anticipated that the GPU's massively parallel execution would allow 32 (or more) RRT searches to proceed simultaneously, likely outpacing the 2-thread CPU running sequentially or in limited parallel. The expectation was a dramatic reduction in planning time (potentially an order-of-magnitude speedup, as our earlier simple benchmarks suggested) and possibly a higher success rate in finding a path (since many random explorations increase the chance that at least one finds a viable route around obstacles). However, as the next section will detail, the reality in our experiments was that the CPU implementation often completed planning faster and with a higher success rate than the GPU implementation. We will look at quantitative benchmarks to understand this outcome and then analyze the technical reasons behind it.

## Performance Evaluation

A key objective of this project is to quantify the speed-up achieved through GPU acceleration. Even with our initial naive GPU implementation, the performance benefits can already been seen when compared to a single-threaded CPU RRT baseline.

<div align="center" style="display: flex; justify-content: center; gap: 20px;">
  <img src="imgs/EmptyP.png" width="30%" />
  <img src="imgs/CorridorP.png" width="30%" />
  <img src="imgs/Random2P.png" width="30%" />
</div>

<p align="center">
  <img src="imgs/corridorMap.png" />
  <p align="center">Different paths plotted by each RRT algorithm</p>

### Overview of Performance Metrics

To evaluate planning performance across different planners and environments, we use the following metrics:

1. Planning Time (ms)
The mean and medium planning time of the algorithm to produce a valid path form start to goal.

2. Cost of the Path
The total distance of the resulting path from start to goal.

3. Solve Percentage (Success Rate)
The percentage of trials in which a valid path is found within a time/iteration budget.

4. Percentile
The variability in runtime. Q1 is 25th percentile, Q3 is 75th percentile. The 95th percentile is of note to show that 95% of runs finished at or faster than this time, characterizing the “worst typical case."

5. CDF (Cumulative Distribution Function) of Planning Time
CDF curves allow comparison not just of average speed, but distributional speed characteristics:
Farther left = faster solve times.
Steeper slope = more consistent solve times.
Higher tail = greater variance. 
These plots mirror evaluation methods used in sampling-based planning research papers.

| Map Type (width, height) | Visualizations |
|----------|----------------|
| **Empty Map (100 × 100)** | <img src="imgs/Empty1.png" width="300"> <img src="imgs/Empty2.png" width="300"> |
| **Corridor Map (60 × 120)** | <img src="imgs/Corridor1.png" width="300"> <img src="imgs/Corridor2.png" width="300"> |
| **Random Map (1000 × 1000)** | <img src="imgs/Random1.png" width="300"> <img src="imgs/Random2.png" width="300"> |

*CPU RRT failed all trials of random map resulting in flat line*

## Data Collected from Deployment

To evaluate a naive GPU baseline, we implemented a multi-tree RRT planner that is logically identical to our multi-threaded CPU implementation. The common RRT logic is factored into a shared header and compiled separately for CPU and CUDA. In all experiments, we use the same RRT configuration; the CPU version runs with 2 worker threads, and the CUDA version launches 32 workers in parallel. We deploy both planners on the vehicle and collect statistics over two complete test loops.

In terms of planning latency, the CPU implementation is significantly faster. On average, the CPU planner completes in about 20 ms, whereas the CUDA planner requires roughly 40 ms per invocation. This is expected: for a single sequential workload, an individual CPU core is much more powerful than a single CUDA core, and our GPU implementation is essentially a direct port of the CPU code. Additionally, kernel launch overhead and host–device data movement further reduce the benefit of parallelizing independent RRT instances on the GPU.

![RRT Planning Time Distribution](imgs/RRT_Planning_Time_Distribution.png)

For successful planning attempts, the structure of the resulting paths is similar if not identical. The distribution of tree sizes is slightly larger for CUDA than for CPU, while the number of waypoints in the final path is very close between the two. This is consistent with both planners using the same sampling strategy, termination conditions, and per-tree node limits, with the GPU spending somewhat more effort per successful tree.

![RRT Tree Statistics](imgs/RRT_Tree_Statistics.png)

The main difference is in success rate. The CPU planner succeeds in approximately 82% of planning attempts, whereas the naive CUDA planner succeeds only about 68% of the time. Increasing the number of GPU workers from 2 to 32 does not significantly increase the probability of finding a path. At the worker level, we typically observe that in a given planning episode either almost all workers succeed or almost all fail, suggesting that each planning problem is effectively near-binary. Either easy enough that many independent trees succeed, or difficult enough that additional workers provide little help.

Several factors contribute to this behavior. First, planning latency in a dynamic environment matters: both planners are triggered at the same callback frequency, but the vehicle continues to move while planning is in progress. Because the CPU planner finishes in roughly 20 ms and the CUDA planner in roughly 40 ms, the CPU's RRT tree is built with environment information closer to the state at which its waypoints will be executed. The CUDA planner, by contrast, more often plans on slightly stale information, making it more likely to produce paths that are misaligned with the true current state and that lead to harder configurations in subsequent cycles.

Second, there is resource contention with the particle filter used for localization. On our platform, the GPU is shared between the CUDA RRT and a particle filter that estimates the vehicle's pose. Moving RRT from CPU to GPU reduces the resources available to the particle filter and may degrade the precision or freshness of the pose estimate, further amplifying the misalignment between planned paths and the true state. Due to operating system constraints on the vehicle, we cannot directly profile the GPU to quantify this effect, but it is a plausible contributor.

Third, per-tree node budget remains a key determinant of success. For a fixed planning time, more nodes in a single tree increase coverage and density, improving the chance of connecting start and goal in cluttered environments. In the naive multi-tree CUDA design, the computational budget is spread across many independent trees, each relatively small. Given the near-binary nature of success in our scenarios, simply running more small trees in parallel does little to change the underlying probability that any one tree will find a solution.

At the system level, the planner is allowed up to three retries before sending an updated waypoint sequence to the pure-pursuit controller. This yields an overall success rate of nearly 97% at the controller interface, even though the per-attempt success rates differ substantially between CPU and CUDA. Thus, the naive CUDA implementation is not constantly failing to provide a path. However, because its plans are more likely to be based on stale or less accurate information, they tend to steer the vehicle into more constrained and difficult situations. In practice, we observe more accidental crashes and near-collisions with the naive CUDA planner, even when it technically finds a path.

These observations indicate that a naive multi-tree CUDA implementation is poorly matched to our application. Although it performs more parallel computations than the CPU baseline, this does not translate into higher effective success or improved safety, because the dominant factors are planning latency, pose estimation quality, and per-tree node density, rather than the sheer number of independent trees. This motivates an algorithmic redesign: in pRRT, multiple CUDA threads cooperatively grow a single RRT tree (or a small set of trees), increasing the number of nodes per tree and reducing effective latency, thereby better exploiting the GPU's parallel capabilities.

## Performance Evaluation on F1tenth

We conducted head-to-head benchmarks of the CPU and GPU RRT planners in a realistic mapping of an indoor environment (the “Skirkanich Lobby” map, which includes multiple turns and obstacles). Both planners were tested under identical conditions: the car starts at a fixed location and repeatedly plans local paths toward successive waypoints on a pre-defined global route. Dynamic obstacles were not introduced in this particular test (to isolate planner performance), but the environment's static obstacles still require the planner to maneuver around walls and pillars. The planners ran in an iterative loop (at ~3.3 Hz planning interval) until the route was completed or a timeout occurred. We logged every planning cycle's result, including whether a path was found, how long the planning took, and how complex the resulting path was (number of waypoints). Below we present a comparison of key metrics between the CPU and GPU versions:


| Metric                                | **CPU RRT**                     | **GPU RRT**               |
| ------------------------------------- | ------------------------------- | ------------------------- |
| **Planning cycles (calls)**           | 487 cycles                      | 577 cycles                |
| **Successful plans**                  | 397 (out of 487)                | 395 (out of 577)          |
| **Success rate**                      | 81.5%                           | 68.5%                     |
| **Mean planning time (successful)**   | 13.9 ms                         | 56.3 ms                   |
| **Median planning time (successful)** | 11.3 ms                         | 39.4 ms                   |
| **90th percentile time (successful)** | 26.5 ms                         | 122.2 ms                  |
| **Mean planning time (failed)**       | 24.9 ms                         | 53.9 ms                   |
| **Median planning time (failed)**     | 19.0 ms                         | 32.5 ms                   |
| **Avg. path complexity (waypoints)**  | 34.7 waypoints per path         | 36.4 waypoints per path   |
| **Median path complexity**            | 32 waypoints                    | 33 waypoints              |
| **Resource usage per cycle**          | ~2 CPU threads (100% of 1 core) | GPU + 1 CPU core overhead |

Looking at the raw numbers, the CPU planner outshines the current GPU planner in both speed and reliability. The CPU version succeeded in ~81.5% of its planning attempts, while the GPU succeeded in only ~68.5%. In other words, the GPU planner failed to find a path in nearly one-third of the cycles (often due to hitting the iteration limit without connecting to the goal), whereas the CPU failed only ~18.5% of the time. When a path was found, the CPU's planning latency was much lower: a median of ~11 ms versus the GPU's median of ~39 ms for successful plans. Even the worst-case successful CPU plans (95th percentile around 37 ms) were faster than the GPU's median. The GPU had some runs that took notably longer (up to ~362 ms in rare cases for success, and some failure cases up to ~483 ms), indicating more variability and overhead. Path complexity (measured by number of waypoints in the resulting trajectory) was roughly comparable between the two, the GPU's paths had a similar median length (33 vs 32 waypoints) and only slightly higher mean, so at least in terms of path quality the GPU is on par with CPU.

## Several factors contribute to these outcomes:

- **Overhead per Cycle:** The GPU planner incurs substantial overhead each planning cycle. Memory management is a big factor. For instance, allocating and freeing CUDA device_vector buffers for tree nodes and other data on every cycle adds latency. We observed that even before doing meaningful work, the GPU kernel launches have a fixed cost. In our case, each cycle involves copying the latest map (occupancy grid) to the GPU, launching one or more kernels to run the RRT threads, and then copying back the result. For small problem sizes (a few thousand nodes per tree, per cycle), this overhead dominates the total time.

- **Parallel Redundancy:** In the brute-force parallel approach, many GPU threads may be doing redundant work. If one thread finds the goal early, the others might still be running or have done unnecessary computations up to that point. We stop the kernel as soon as a solution is found (using atomic flags or similar), but threads might not all abort instantaneously. Furthermore, if no thread finds the goal quickly, all 32 threads churn through the maximum iterations, which is a worst-case scenario that actually takes longer than the single threaded CPU running 2000 iterations. (The CPU in our test often found paths in far fewer than 2000 iterations, whereas the GPU sometimes had all threads explore 2000 each with nothing to show, essentially 64,000 total expansions wasted.)

- **Workload Granularity:** Each GPU thread's workload in our implementation is similar to the CPU's: growing a tree with up to 2000 nodes. While this can be parallelized across threads, the per-thread job is not very heavy. Modern GPUs excel when each thread block has a lot of work to do or when there are thousands of threads. In our case, 32 threads × 2000 iterations is relatively modest and does not fully utilize the GPU's potential compute throughput. It spends a larger fraction of time in overhead (kernel launch, memory transfer, context synchronization) relative to useful work. Essentially, the problem size per cycle is too small to amortize GPU overheads. The CPU, on the other hand, has negligible overhead to start processing those 2000 iterations and can sometimes terminate early when the goal is found.

**Thread Divergence and GPU Utilization:** The nature of RRT is somewhat sequential and irregular. In the GPU kernel, different threads may take different numbers of steps to reach a conclusion (some might find the goal quickly, others explore fruitlessly). This can cause divergence in GPU warps and under-utilization of some cores. By contrast, the CPU's few threads each run an independent RRT to completion or failure, and there's less of a performance penalty if one finishes earlier (it just idles or stops).

In summary, our detailed benchmarks make it clear that the current GPU implementation is slower per cycle and less reliable than the CPU implementation. The next section will delve into why this happened and how we plan to address it.

## Technical Analysis: Why is the GPU Slower?

It might be surprising that a GPU, capable of thousands of parallel operations, underperforms a CPU in this task. The reasons boil down to implementation overheads and the scale of the problem:

- **Memory Allocation and Transfer Overheads:** Each planning cycle, the GPU version uses CUDA Thrust vectors and allocates memory for node storage and other data structures. For example, it constructs thrust::device_vector buffers for the RRT tree nodes on the device. These allocations (and corresponding deallocations) are costly when done frequently. Additionally, the occupancy grid (map) is copied from host (CPU) to device (GPU) memory every cycle. If the map is large (e.g. a 2D grid covering an entire floor), this transfer can take a few milliseconds itself. On the CPU, the map is already in memory and reused; there is effectively zero setup cost each cycle beyond checking obstacle updates. Thus, the GPU starts each cycle with a disadvantage, it's doing extra work just to prepare for path planning.

- **Kernel Launch and Synchronization Overhead:** Launching a CUDA kernel has a fixed overhead on the order of microseconds to a millisecond, and using many small kernels or launching large numbers of threads for relatively short computations can be inefficient. Our planner launches at least one kernel per planning cycle to run the RRT workers, and possibly additional smaller kernels for things like copying results or checking termination conditions. These overheads add up, especially at a 300 ms planning interval. Moreover, the GPU kernel must synchronize back with the CPU when finished (to return the found path), which incurs additional latency. The CPU planner doesn't need to context-switch or synchronize with an external device, it just runs its algorithm in-line.

- **Underutilization of GPU for Small Problems:** As noted, each GPU thread in our planner may perform only a few thousand operations (e.g., up to 2000 loop iterations trying to extend the tree). Modern GPUs can easily handle millions of operations; doing only a few thousand per thread means the workload may not fully occupy the GPU's SMs (streaming multiprocessors). Additionally, with only 32 threads (one per RRT worker in our tests), we're not saturating the GPU's parallel resources. In fact, 32 threads is extremely low (GPUs often run thousands of threads concurrently). We chose 32 as a number of parallel RRTs thinking more might improve chance of success, but it's still not enough to leverage the hardware's capability, and increasing it further (with our current algorithm) would linearly increase memory usage and thread-launch overhead without a smarter strategy.

- **Algorithmic Inefficiencies on GPU:** The brute-force approach can waste a lot of work. If the goal is found early by one thread, we attempt to terminate others, but there's still some wasted computation. If the goal is not found early, the GPU essentially does 32× the work of the CPU (e.g., exploring up to 32 separate trees of 2000 nodes each) in an attempt to find a path. If none succeed, that's 64k node expansions done in parallel, which takes time, only to report failure. The CPU in that scenario would also report failure but having expanded just 2000 nodes in one tree (much less total work). In cases where the problem is hard (no easy path), the GPU is spending a lot more total effort per cycle (albeit in parallel) which can actually result in longer wall-clock time than the CPU's effort, because of the aforementioned overheads and because not all of the GPU's work translates to finding the path faster (it's redundant exploration).

- **Reliability Factors:** We also observed the GPU's success rate was lower. One reason could be the random seed usage: if all 32 GPU threads started with similar seeds or if there was a bug causing correlated sampling, they might explore similar areas and collectively fail where the CPU (with fewer but perhaps more carefully seeded tries) might succeed on a subsequent attempt. It's also possible that the CPU planner, running continuously, sometimes slightly exceeded the nominal 300 ms cycle to complete a path, whereas the GPU might have cut off strictly at the limit (we saw some GPU fails finishing in ~110-130 ms which might imply hitting iteration limits well before the 300 ms). These implementation details can affect success rate, the CPU's simpler single-tree approach might actually search more systematically, whereas the GPU's scattershot approach might miss the solution in all threads if none of the random explorations hit the right sequence.

In essence, the GPU's theoretical advantages were blunted by practical overheads and an approach that isn't well-aligned with the GPU's strengths. Our naive GPU RRT does a lot of small, parallel searches rather than one big parallel search, and it doesn't re-use work between cycles. The result is that the GPU spends a lot of time setting up and tearing down work, and relatively less time doing the useful computations compared to an optimized CPU code that runs light and continuous.

## Lessons Learned and Future Work

The development and benchmarking of GPUDynamicRRT have been invaluable in teaching us about parallel algorithm design in a robotics context. We learned that GPU acceleration is not a silver bullet. Careful attention must be paid to memory management, parallel workload distribution, and algorithm tuning. The fact that a well-optimized CPU implementation outperformed the GPU one reminds us that naively porting an algorithm to CUDA can sometimes make it faster in theory but slower in practice, especially for moderate-sized, real-time tasks.

Moving forward, we have several plans to bridge the gap and realize the potential of GPU acceleration for RRT:

- **Optimize Memory Management:** A top priority is to eliminate frequent memory allocations on the GPU. We intend to pre-allocate buffers (device memory) for tree nodes, random number states, etc., once at startup, and then reuse them every planning cycle. For example, if we allocate space for the maximum number of nodes a thread could generate (2000 nodes × 32 threads) in advance, we can simply reset indices and reuse that array each time instead of allocating new device_vectors. Similarly, we can keep a copy of the static map data in GPU memory persistently. The map rarely changes (unless the car moves to a new environment or the map is updated), so copying it every cycle is unnecessary – instead, we can upload it once and then just reference it on the device for collision checks each frame. These changes will cut down the overhead per cycle dramatically by trading memory capacity for speed.

- **Kernel Launch Consolidation:** We plan to restructure the GPU code to minimize the number of kernel launches and GPU-CPU synchronizations per cycle. This might involve merging what are currently separate steps into one kernel, or using CUDA streams to overlap data transfer with computation. By doing more in a single kernel (e.g., all RRT threads run and perhaps also check for termination within the kernel), we reduce the launch overhead and allow the GPU to work more asynchronously with the CPU.

- **Tuning Parallelization Parameters:** The current choice of 32 parallel RRT threads was somewhat arbitrary. We will experiment with the num_workers parameter to see if fewer threads (reducing overhead and redundancy) or more threads (increasing parallel search coverage) yield better performance. It's possible that an optimal number exists that balances the GPU utilization and overhead. Similarly, the threads_per_block (we used 16) and how threads are grouped could be tuned to match the GPU's architecture (for example, using warps effectively). We may find that, for instance, 8 parallel trees suffice in many cases and avoid the overhead of launching 32, or conversely that 64 threads give a small boost in success rate without too much extra cost.

- **Improving Randomization and Reliability:** We will ensure that each parallel worker uses a unique random seed and perhaps different sampling biases, to maximize the chance that at least one finds a difficult path. We might incorporate a strategy where if many threads fail quickly, the remaining threads get more iterations or expanded search space, rather than all doing identical amounts of work.

- **Transition to Cooperative Parallel RRT:** The ultimate evolution of this project is to implement the cooperative parallel RRT growth (illustrated as our “final goal” in earlier design discussions). In this approach, instead of many independent trees, all threads collaborate on one tree. For example, in each iteration, multiple sample points can be generated in parallel and multiple collision checks can happen in parallel for different branches of the tree. One could use parallel primitives to find the nearest neighbor in the tree faster, or extend several branches simultaneously. This would leverage the GPU to accelerate a single RRT's progress rather than brute-forcing many. It requires more complex synchronization (threads need to share the evolving tree data structure) and careful handling of memory (possibly using GPU dynamic parallelism or other features so threads can add nodes to a common tree). We believe that by avoiding redundant work, the cooperative approach will eventually outperform the CPU significantly, especially as the environment complexity grows. Early designs for this involve using structures like a parallel expand step where each GPU thread attempts to extend the tree from a different node concurrently and an atomic operation selects which extension is successful, etc. Implementing this will be a challenging but rewarding next step.

- **Hybrid Approaches:** Another idea for future exploration is a hybrid CPU-GPU planner. For example, the CPU could handle certain easy tasks or initial steps (like selecting random sample points or doing a quick check for direct path) and delegate heavy computations (like collision-checking many samples or expanding multiple nodes) to the GPU. Or run a CPU RRT and GPU RRT in parallel and take whichever finds a path first (“racing” them). This could potentially combine the reliability of the CPU approach with the occasional speed bursts of the GPU on easier problems.

- **Expanded Benchmarks:** We plan to test the improved GPU implementation in more scenarios, including with dynamic moving obstacles and in larger maps. It's possible that as the scenario scales up (either larger areas or the need for more frequent re-planning due to moving obstacles), the GPU's parallel advantage will start to show. For instance, if we required re-planning at 50 Hz instead of ~3 Hz, the CPU might not keep up, whereas an optimized GPU might. We will also profile the code to pinpoint exact bottlenecks (e.g., how much time is spent in memory operations vs. actual tree expansion) to quantitatively guide our optimizations.

In conclusion, while the current state of GPUDynamicRRT did not meet the initial performance expectations, it has provided a clear roadmap of improvements. We have preserved the original motivation – using GPU acceleration to enable faster and more frequent path planning in dynamic environments – and now we have a deeper understanding of how to achieve it. By addressing the technical issues (memory allocation, kernel overhead, parallel strategy) and iterating on our design, we are confident that the GPU-accelerated RRT will eventually surpass the CPU version. This process has been a valuable lesson in the difference between theoretical parallel speedup and practical implementation: to fully harness the GPU, one must design the algorithm with the GPU's strengths and weaknesses in mind. We remain committed to this goal and look forward to demonstrating a truly real-time, reliable GPU RRT planner on the F1Tenth platform in future iterations of the project.
