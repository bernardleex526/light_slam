#!/usr/bin/env python3
"""Record Gazebo ground truth (ModelStates + /clock -> TUM) to stdout/file.

Usage: python3 record_gt.py [outfile] [duration_sec]
Lines: t x y z qx qy qz qw  (pose of model 'm20', sim time from /clock)
"""
import sys
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock


class GtRecorder(Node):
    def __init__(self, out, duration):
        super().__init__('gt_recorder')
        self.out = out
        self.duration = duration
        self.sim_t = 0.0
        self.last_t = -1.0
        # /model_states is RELIABLE (gazebo_ros_state default); a best_effort
        # subscription silently matches nothing. /clock is BEST_EFFORT.
        ms_qos = rclpy.qos.QoSProfile(depth=10)
        clk_qos = rclpy.qos.QoSProfile(
            depth=2000, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(ModelStates, '/model_states',
                                            self.cb, ms_qos)
        self.clock_sub = self.create_subscription(Clock, '/clock',
                                                  self.clock_cb, clk_qos)

    def clock_cb(self, msg):
        self.sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9

    def cb(self, msg):
        if self.sim_t <= 0.0:
            return  # /clock not yet discovered: dropping keeps timestamps monotonic
        if self.sim_t <= self.last_t:
            return  # 严格递增：/clock 1ms 量化会产生重复时间戳
        self.last_t = self.sim_t
        if 'm20' not in msg.name:
            return
        i = list(msg.name).index('m20')
        p = msg.pose[i]
        self.out.write(
            f'{self.sim_t:.6f} {p.position.x:.6f} {p.position.y:.6f} '
            f'{p.position.z:.6f} {p.orientation.x:.6f} {p.orientation.y:.6f} '
            f'{p.orientation.z:.6f} {p.orientation.w:.6f}\n')
        self.out.flush()


def main():
    outfile = sys.argv[1] if len(sys.argv) > 1 else None
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    rclpy.init()
    out = open(outfile, 'w') if outfile else sys.stdout
    node = GtRecorder(out, duration)
    t0 = time.time()
    while time.time() - t0 < duration and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
