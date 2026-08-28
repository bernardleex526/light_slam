#!/usr/bin/env python3
"""Record the LIO trajectory (/lio_pose -> TUM) to stdout/file.

Usage: python3 record_lio.py [outfile] [duration_sec]
Lines: t x y z qx qy qz qw
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class LioRecorder(Node):
    def __init__(self, out, duration):
        super().__init__('lio_recorder')
        self.out = out
        self.duration = duration
        self.last_t = -1.0
        self.sub = self.create_subscription(PoseStamped, '/lio_pose', self.cb, 10)

    def cb(self, msg):
        t = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if t <= self.last_t:
            return  # 严格递增：/clock 1ms 量化会产生重复时间戳
        self.last_t = t
        p = msg.pose
        self.out.write(
            f'{t:.6f} {p.position.x:.6f} {p.position.y:.6f} {p.position.z:.6f} '
            f'{p.orientation.x:.6f} {p.orientation.y:.6f} '
            f'{p.orientation.z:.6f} {p.orientation.w:.6f}\n')
        self.out.flush()


def main():
    outfile = sys.argv[1] if len(sys.argv) > 1 else None
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    rclpy.init()
    out = open(outfile, 'w') if outfile else sys.stdout
    node = LioRecorder(out, duration)
    t0 = time.time()
    while time.time() - t0 < duration and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
