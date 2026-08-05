#!/usr/bin/env python3
"""Record /degeneracy_status (nullity + 6 eigenvalues) to a text file.

Usage: python3 record_degeneracy.py <outfile> [duration_sec]
Lines: t nullity ev0 ev1 ev2 ev3 ev4 ev5
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class DegeneracyRecorder(Node):
    def __init__(self, out):
        super().__init__('degeneracy_recorder')
        self.out = out
        self.sub = self.create_subscription(
            Float32MultiArray, '/degeneracy_status', self.cb, 10)

    def cb(self, msg):
        t = time.time()
        vals = [f'{v:.6f}' for v in msg.data]
        self.out.write(f'{t:.3f} ' + ' '.join(vals) + '\n')
        self.out.flush()


def main():
    outfile = sys.argv[1] if len(sys.argv) > 1 else None
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    rclpy.init()
    out = open(outfile, 'w') if outfile else sys.stdout
    node = DegeneracyRecorder(out)
    t0 = time.time()
    while time.time() - t0 < duration and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
