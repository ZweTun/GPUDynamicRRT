import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
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
                # If gym_bridge_launch.py has launch arguments, pass them here:
                # launch_arguments={'arg_name': 'value'}.items(),
            ),
            Node(
                package="dynamic_rrt",
                executable="rrt_node",
                name="rrt_node",
                output="screen",
            ),
            Node(
                package="waypoint_tracker",
                executable="pure_pursuit",
                name="pure_pursuit",
                output="screen",
            ),
        ]
    )
