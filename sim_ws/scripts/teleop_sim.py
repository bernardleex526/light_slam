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
    # 走廊：直行往返 + 中途停驻（退化重点段）。
    # 实测（2026-08-05，轮向修复后，SIM 时间）：cmd_vel.x=0.5 → 实际速度 ~0.91 m/s
    # （轮速按碰撞体半径 0.09 折算；WSL 下 RTF≈0.83，墙钟测速会低估）。
    # 每腿 14s × 0.5 cmd ≈ 12.7m < 15m（半走廊长），全程不出走廊两端；
    # 往返总长 ~25m，留足安全边际（走廊全长 30m，可用 ~28m）。
    'corridor': [(0.5, 14), (0.0, 4), (-0.5, 14), (0.0, 4)],
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
