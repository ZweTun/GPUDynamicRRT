import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial import KDTree, transform
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


class PurePursuit(Node):
    """
    Implement Pure Pursuit using RRT-generated waypoints.
    """

    def __init__(self):
        super().__init__("pure_pursuit_node")
        self.declare_parameter("simulation", False)
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('steering_gain', 0.5)
        self.declare_parameter('speed', 1.5)

        self.sim = self.get_parameter("simulation").get_parameter_value().bool_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value

        # Subscribe to the correct odometry topic
        odom_topic = "/ego_racecar/odom" if self.sim else "/pf/viz/inferred_pose"
        self.create_subscription(
            Odometry if self.sim else PoseStamped, odom_topic, self.pose_callback, 10
        )

        # Subscribe to waypoints from RRT
        self.create_subscription(
            MarkerArray, "/waypoints_markers", self.rrt_waypoints_callback, 10
        )

        # Publisher for driving commands
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

        # Parameters for lookahead distance and control gain
        self.L = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.P = self.get_parameter('steering_gain').get_parameter_value().double_value

        # Initialize empty waypoints
        self.waypoints = np.array([])
        self.kd_tree = None

    def rrt_waypoints_callback(self, msg):
        """Callback to update waypoints from RRT"""
        waypoints = []
        for marker in msg.markers:
            x, y = marker.pose.position.x, marker.pose.position.y
            waypoints.append([x, y])

        if waypoints:
            self.waypoints = np.array(waypoints)
            self.kd_tree = KDTree(self.waypoints)  # Update KDTree
            self.get_logger().info(
                f"Updated waypoints from RRT with {len(self.waypoints)} points"
            )

    def pose_callback(self, pose_msg):
        """Find the next waypoint to track and publish drive commands"""
        if self.waypoints.size == 0:
            self.get_logger().warn("No waypoints received yet.")
            return

        # Extract car position
        if self.sim:
            car_x = pose_msg.pose.pose.position.x
            car_y = pose_msg.pose.pose.position.y
            quat = pose_msg.pose.pose.orientation
        else:
            car_x = pose_msg.pose.position.x
            car_y = pose_msg.pose.position.y
            quat = pose_msg.pose.orientation

        # Convert quaternion to rotation matrix
        quat = [quat.x, quat.y, quat.z, quat.w]
        R = transform.Rotation.from_quat(quat)
        self.rot = R.as_matrix()

        # Find the closest waypoint
        _, idx = self.kd_tree.query([car_x, car_y])
        for i in range(idx, len(self.waypoints)):
            dist = np.linalg.norm(self.waypoints[i] - np.array([car_x, car_y]))
            if dist >= self.L:
                goal_x, goal_y = self.waypoints[i]
                break
        else:
            self.get_logger().warn("No valid lookahead point found.")
            return

        # Transform goal point to vehicle frame
        goal_y_vehicle = self.translatePoint(
            np.array([car_x, car_y]), np.array([goal_x, goal_y])
        )[1]

        # Compute curvature and steering angle
        curvature = 2 * goal_y_vehicle / (self.L**2)
        steering_angle = self.P * curvature
        steering_angle = np.clip(steering_angle, -0.35, 0.35)

        # Publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

    def translatePoint(self, currPoint, targetPoint):
        """Transform a point from global to vehicle frame"""
        H = np.zeros((4, 4))
        H[0:3, 0:3] = np.linalg.inv(self.rot)
        H[0, 3] = currPoint[0]
        H[1, 3] = currPoint[1]
        H[3, 3] = 1.0
        dir = targetPoint - currPoint
        translated_point = (H @ np.array((dir[0], dir[1], 0, 0))).reshape((4))
        return translated_point


def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
