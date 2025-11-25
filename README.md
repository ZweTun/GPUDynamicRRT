# GPUDynamicRRT

CIS 5650 Final Project

## Project Structure

- `GPU-RRT`: CUDA implementation of the RRT algorithm.
- `f1tenth_gym_ros`: Copied from [this repository](https://github.com/f1tenth/f1tenth_gym_ros), which contains an ROS node that exposes an interface to the F1TENTH gymnasium simulator.
- `lab7_pkg`: ROS package that contains the Python CPU implementation of the RRT algorithm and a pure-pursuit waypoint tracker.
- `lab7_pkg_cpp`: ROS package that contains the C++ CPU implementation of the RRT algorithm.

Currently, some code are still in development and reside in other branches. For example, the `2025-11-13` branch contains a Docker Compose setup that stitches together different parts, allowing for easy simulation and testing.

**Meeting Notes 11/09/25**

Initial f1tenth gym setup on personal laptops.
<https://github.com/f1tenth/f1tenth_gym_ros/tree/dev-dynamics-temporary-documentation>

Electrical Hardware Overview Walkthrough

- 2D Lidar Scanner
- Nividia Orin Nano
- Lipo Battery
