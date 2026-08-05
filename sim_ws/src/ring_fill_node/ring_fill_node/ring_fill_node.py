#!/usr/bin/env python3
"""Add a ring field to Gazebo point clouds (16-line -15..15 deg) -> /rs_points.

Gazebo's ray sensor PointCloud2 output (x,y,z,intensity) has no ring field;
lightning's Velodyne preprocessor needs point.ring to build scan timestamps.
Ring is computed from the vertical angle of each point.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
]
_POINT_STEP = 18


class RingFillNode(Node):
    def __init__(self):
        super().__init__('ring_fill_node')
        self.lines = self.declare_parameter('lines', 16).value
        self.in_topic = self.declare_parameter('in_topic', '/points_raw').value
        self.out_topic = self.declare_parameter('out_topic', '/rs_points').value
        self.pub = self.create_publisher(PointCloud2, self.out_topic, 10)
        self.sub = self.create_subscription(PointCloud2, self.in_topic, self.cb, 10)
        fov = 30.0 * math.pi / 180.0
        self.angles = np.linspace(-fov / 2, fov / 2, self.lines)
        self.dtype = np.dtype([
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
            ('intensity', '<f4'), ('ring', '<u2')])
        self.get_logger().info(
            f'ring_fill: {self.in_topic} -> {self.out_topic}, '
            f'{self.lines} lines')

    def cb(self, msg):
        pts = np.frombuffer(msg.data, dtype=np.float32)
        n = pts.size // 4
        pts = pts[:n * 4].reshape(-1, 4)
        ang = np.arctan2(pts[:, 2], np.hypot(pts[:, 0], pts[:, 1]))
        ring = np.argmin(np.abs(self.angles - ang[:, None]), axis=1)
        out = np.zeros(n, dtype=self.dtype)
        out['x'] = pts[:, 0]
        out['y'] = pts[:, 1]
        out['z'] = pts[:, 2]
        out['intensity'] = pts[:, 3]
        out['ring'] = ring
        out_msg = PointCloud2()
        out_msg.header = msg.header
        out_msg.height, out_msg.width = 1, n
        out_msg.fields = _FIELDS
        out_msg.point_step, out_msg.row_step = _POINT_STEP, _POINT_STEP * n
        out_msg.is_dense = True
        out_msg.data = out.tobytes()
        self.pub.publish(out_msg)


def main():
    rclpy.init()
    node = RingFillNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
