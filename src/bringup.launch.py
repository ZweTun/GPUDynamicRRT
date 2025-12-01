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
    #             "max_iterations": 20000,
    #             "global_waypoint_csv": "",
    #             "sample_forward_min_m": 0.5,
    #             "sample_forward_max_m": 8.0,
    #             "sample_lateral_min_m": -2.0,
    #             "sample_lateral_max_m": 2.0,
    #             "steer_step_m": 0.2,
    #             "goal_threshold_m": 0.2,
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
                "planning_interval_ms": 250,
                "waypoint_publish_interval_ms": 1000,
                "obstacle_margin": 0.2,
                "global_waypoint_max_distance": 5.0,
                "maxIter": 8000,
                "maxNodes": 8000,
                "maxStep": 0.4,
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
