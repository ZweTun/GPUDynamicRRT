#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import time

class RRTTestNode(Node):
    def __init__(self):
        super().__init__('gpu_rrt_test')
        # Publishers for map, odom, and goal
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal', 10)
        # Timer to run test sequence once after a delay
        self.create_timer(2.0, self.run_test_once)
        self.test_executed = False

    def run_test_once(self):
        if self.test_executed:
            return
        self.get_logger().info('Publishing test map, odometry, and goal...')
        # Create a simple 10x10 grid map with an obstacle
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = 1.0
        map_msg.info.width = 10
        map_msg.info.height = 10
        map_msg.info.origin.position.x = 0.0
        map_msg.info.origin.position.y = 0.0
        map_msg.info.origin.orientation.w = 1.0
        # Fill map data: 0 = free, 100 = obstacle
        data = []
        for y in range(map_msg.info.height):
            for x in range(map_msg.info.width):
                if y == 5 and 2 <= x <= 7:
                    data.append(100)  # obstacle band across the map
                else:
                    data.append(0)    # free space
        map_msg.data = data

        # Create odometry message for robot start at (1,1)
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'map'
        odom_msg.pose.pose.position.x = 1.0
        odom_msg.pose.pose.position.y = 1.0
        odom_msg.pose.pose.orientation.w = 1.0  # no rotation (orientation irrelevant for planner)

        # Create goal message at (9,9)
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = 9.0
        goal_msg.pose.position.y = 9.0
        goal_msg.pose.orientation.w = 1.0

        # Publish map and odometry
        self.map_pub.publish(map_msg)
        self.odom_pub.publish(odom_msg)
        # Publish odometry twice to ensure the message is received
        self.odom_pub.publish(odom_msg)
        # Wait briefly before sending the goal to ensure map/odom are processed
        time.sleep(0.5)
        # Now publish the goal
        self.goal_pub.publish(goal_msg)

        self.test_executed = True

def main(args=None):
    rclpy.init(args=args)
    node = RRTTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
