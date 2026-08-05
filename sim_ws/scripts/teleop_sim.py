#!/usr/bin/env python3
"""Teleop the M20 in simulation with a scene-dependent pattern.

Usage: python3 teleop_sim.py corridor|plaza|office
Publishes on /diff_drive_controller/cmd_vel_unstamped (the unstamped cmd_vel
topic of the diff_drive_controller inside the gzserver-hosted controller
manager).
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

PATTERNS = {
    # 走廊：直行往返 + 中途停驻（退化重点段）
    'corridor': [(0.6, 25), (0.0, 5), (-0.6, 25), (0.0, 5)],
    # 广场：直行 + 大弧线转弯
    'plaza': [(0.8, 20), (0.5, 15), (0.0, 5), (-0.8, 20), (0.0, 5)],
    # 办公室：走廊直行 + 进出门洞
    'office': [(0.5, 15), (0.0, 5), (-0.5, 15), (0.0, 5)],
}


def main():
    scene = sys.argv[1] if len(sys.argv) > 1 else 'corridor'
    pattern = PATTERNS.get(scene, PATTERNS['corridor'])
    rclpy.init()
    n = Node('teleop_sim')
    pub = n.create_publisher(Twist,
                             '/diff_drive_controller/cmd_vel_unstamped', 10)
    print(f'teleop {scene}: {pattern}')
    for vel, dur in pattern:
        t0 = time.time()
        while time.time() - t0 < dur and rclpy.ok():
            tw = Twist()
            tw.linear.x = vel
            pub.publish(tw)
            rclpy.spin_once(n, timeout_sec=0.1)
    pub.publish(Twist())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
