#!/usr/bin/env python3
"""Record FAST-LIO /Odometry to CSV.

Why: `mat_pre.txt` (written by laserMapping) holds only RELATIVE time
(`lidar_beg_time - first_lidar_time`) plus Euler angles, so it cannot be
matched against GNSS fix stamps.  /Odometry carries the ABSOLUTE header.stamp
(LiDAR time base, same epoch as the bag's GNSS fixes) and the full quaternion.

Output: stamp_ns, stamp_sec, x, y, z, qx, qy, qz, qw   (frame: camera_init)
"""
import csv
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

QOS = 200  # FAST-LIO publishes /Odometry with KeepLast(20), reliable


class OdomRecorder(Node):
    def __init__(self, path: str):
        super().__init__("odom_recorder")
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["stamp_ns", "stamp_sec", "x", "y", "z",
                         "qx", "qy", "qz", "qw"])
        self.n = 0
        self.create_subscription(Odometry, "/Odometry", self.cb, QOS)
        self.get_logger().info(f"recording /Odometry -> {path}")

    def cb(self, m: Odometry) -> None:
        s = m.header.stamp
        ns = s.sec * 1_000_000_000 + s.nanosec
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.w.writerow([
            ns, f"{ns / 1e9:.9f}",
            f"{p.x:.6f}", f"{p.y:.6f}", f"{p.z:.6f}",
            f"{q.x:.6f}", f"{q.y:.6f}", f"{q.z:.6f}", f"{q.w:.6f}",
        ])
        self.n += 1
        if self.n % 200 == 0:
            self.f.flush()
            if self.n % 2000 == 0:
                self.get_logger().info(f"{self.n} poses written")

    def close(self) -> None:
        self.f.flush()
        self.f.close()
        print(f"RECORDER_DONE rows={self.n}", flush=True)


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: record_odom.py <out.csv>", file=sys.stderr)
        return 2
    path = sys.argv[1]
    rclpy.init()
    node = OdomRecorder(path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
