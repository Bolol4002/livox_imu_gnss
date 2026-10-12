import time, rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
class L(Node):
    def __init__(self):
        super().__init__('lag')
        self.p = None; self.o = None; self.pn = 0; self.on = 0
        q = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, '/livox/points', self.cp, q)
        self.create_subscription(Odometry, '/Odometry', self.co, q)
    def cp(self, m): self.p = m.header.stamp.sec + m.header.stamp.nanosec*1e-9; self.pn += 1
    def co(self, m): self.o = m.header.stamp.sec + m.header.stamp.nanosec*1e-9; self.on += 1
rclpy.init(); n = L(); t0 = time.time()
while time.time() - t0 < 8: rclpy.spin_once(n, timeout_sec=0.1)
print('points: %d  last=%.3f' % (n.pn, n.p or 0))
print('odom  : %d  last=%.3f' % (n.on, n.o or 0))
print('lag   : %.3f s' % ((n.p - n.o) if (n.p and n.o) else -1))
rclpy.shutdown()
