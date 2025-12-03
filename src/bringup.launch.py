import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # rrt_node = Node(
    #     package="dynamic_rrt",
    #     executable="rrt_cpu",
    #     name="rrt_cpu",
    #     output="screen",
    #     parameters=[
    #         {
    #             "simulation": True,
    #             "planning_interval_ms": 100,
    #             "waypoint_publish_interval_ms": 1000,
    #             "obstacle_margin": 0.2,
    #             "global_waypoint_max_distance": 5.0,

    #             "num_workers": 2,
    #             "max_iterations": 20000,
    #             "max_nodes_per_tree": 20000,
    #             "sample_forward_min_m": 0.5,
    #             "sample_forward_max_m": 8.0,
    #             "sample_lateral_range_m": 5.0,
    #             "steer_step_size_m": 0.2,
    #             "goal_tolerance_m": 0.2,
    #         }
    #     ],
    # )
    rrt_node = Node(
        package="dynamic_rrt",
        executable="rrt_cuda",
        name="rrt_cuda",
        output="screen",
        parameters=[
            {
                "simulation": True,
                "planning_interval_ms": 100,
                "waypoint_publish_interval_ms": 1000,
                "obstacle_margin": 0.2,
                "global_waypoint_max_distance": 5.0,

                "num_workers": 512,
                "max_iterations": 5000,
                "max_nodes_per_tree": 5000,
                "threads_per_block": 64,
                "sample_forward_min_m": 0.5,
                "sample_forward_max_m": 8.0,
                "sample_lateral_range_m": 5.0,
                "steer_step_size_m": 0.2,
                "goal_tolerance_m": 0.2,
            }
        ],
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("f1tenth_gym_ros"),
                        "launch",
                        "gym_bridge_launch.py",
                    )
                ),
            ),
            rrt_node,
            Node(
                package="waypoint_tracker",
                executable="pure_pursuit",
                name="pure_pursuit",
                output="screen",
            ),
        ]
    )
