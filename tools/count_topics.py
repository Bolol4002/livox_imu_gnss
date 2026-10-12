#!/usr/bin/env python3
"""Count messages on topics over N seconds with RELIABLE QoS (exact rates)."""
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0


class Counter(Node):
    def __init__(self):
        super().__init__('counter')
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.n = {}
        self.last = {}
        for topic, t in [('/livox/points', PointCloud2), ('/livox/imu', Imu),
                         ('/Odometry', Odometry), ('/cloud_registered', PointCloud2)]:
            self.n[topic] = 0
            self.last[topic] = None
            self.create_subscription(t, topic, lambda msg, tp=topic: self.cb(tp, msg), qos)

    def cb(self, topic, msg):
        self.n[topic] += 1
        self.last[topic] = (msg.header.stamp.sec, msg.header.stamp.nanosec)


def main():
    rclpy.init()
    node = Counter()
    start = time.time()
    end = start + DUR
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    wall = time.time() - start
    print('window: %.1f s' % wall)
    for t, n in node.n.items():
        print('  %-20s %6d msgs  %6.3f Hz' % (t, n, n / wall))
        if node.last[t]:
            print('        last header stamp: %d.%09d' % node.last[t])
    rclpy.shutdown()


main()
