#!/usr/bin/env python3
import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray


class TopicStats:
    def __init__(self):
        self.count = 0
        self.last_count = 0
        self.last_time = None
        self.rate = 0.0


class TopicRateLogger(Node):
    def __init__(self):
        super().__init__("topic_rate_logger")

        # Allow overriding log file via parameter if you want
        self.declare_parameter("log_file", "topic_rates.csv")
        log_file = self.get_parameter("log_file").get_parameter_value().string_value

        self.get_logger().info(f"Logging topic rates to {log_file}")

        # Prepare CSV file
        self._file = open(log_file, "w", newline="", buffering=1)
        self._writer = csv.writer(self._file)
        # time = seconds since node start
        self._writer.writerow(["time", "drive_hz", "rrt_tree_hz", "rrt_path_hz", "sample_points_hz"])

        self.start_wall_time = time.time()

        # Per-topic stats
        self.stats = {
            "drive": TopicStats(),
            "rrt_tree": TopicStats(),
            "rrt_path": TopicStats(),
            "sample_points": TopicStats(),
        }

        qos = QoSProfile(depth=10)

        # Subscriptions (types match your topics)
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            "/drive",
            self.drive_cb,
            qos,
        )
        self.rrt_tree_sub = self.create_subscription(
            MarkerArray,
            "/rrt_tree",
            self.rrt_tree_cb,
            qos,
        )
        self.rrt_path_sub = self.create_subscription(
            Marker,
            "/rrt_path",
            self.rrt_path_cb,
            qos,
        )
        self.sample_points_sub = self.create_subscription(
            MarkerArray,
            "/sample_points",
            self.sample_points_cb,
            qos,
        )

        # Timer to compute rates and log them (every 0.5 s)
        self.timer = self.create_timer(0.5, self.timer_cb)

        # Last time we updated rates
        self.last_update_time = self.get_clock().now()

    # --- Callbacks to count messages ---
    def drive_cb(self, msg):
        self.stats["drive"].count += 1

    def rrt_tree_cb(self, msg):
        self.stats["rrt_tree"].count += 1

    def rrt_path_cb(self, msg):
        self.stats["rrt_path"].count += 1

    def sample_points_cb(self, msg):
        self.stats["sample_points"].count += 1

    # --- Timer: compute Hz and log to CSV ---
    def timer_cb(self):
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        dt = (now - self.last_update_time).nanoseconds * 1e-9

        if dt <= 0.0:
            return

        self.last_update_time = now

        for key, stat in self.stats.items():
            if stat.last_time is None:
                # First measurement
                stat.last_time = now_sec
                stat.last_count = stat.count
                stat.rate = 0.0
            else:
                delta_n = stat.count - stat.last_count
                stat.rate = delta_n / dt if dt > 0.0 else 0.0
                stat.last_count = stat.count
                stat.last_time = now_sec

        # Time since node start for the x-axis
        t_rel = time.time() - self.start_wall_time

        row = [
            f"{t_rel:.3f}",
            f"{self.stats['drive'].rate:.3f}",
            f"{self.stats['rrt_tree'].rate:.3f}",
            f"{self.stats['rrt_path'].rate:.3f}",
            f"{self.stats['sample_points'].rate:.3f}",
        ]
        self._writer.writerow(row)

        # Optional console debug (throttled when rates are stable)
        self.get_logger().info(
            f"t={t_rel:5.1f}s | "
            f"/drive {self.stats['drive'].rate:5.2f} Hz | "
            f"/rrt_tree {self.stats['rrt_tree'].rate:5.2f} Hz | "
            f"/rrt_path {self.stats['rrt_path'].rate:5.2f} Hz | "
            f"/sample_points {self.stats['sample_points'].rate:5.2f} Hz"
        )

    def destroy_node(self):
        try:
            self._file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TopicRateLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
