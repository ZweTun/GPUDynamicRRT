from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gpu_rrt_planner',
            executable='gpu_rrt_node',
            name='gpu_rrt_node',
            output='screen'
        ),
        Node(
            package='gpu_rrt_planner',
            executable='test_gpu_rrt.py',
            name='gpu_rrt_test',
            output='screen'
        )
    ])
