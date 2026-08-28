#!/usr/bin/env python3
# M20 一键启动就绪门禁：等待所有关键话题收到首帧数据后退出 0，超时退出 1。
# 纯 rclpy 实现，不依赖 ros2_engine。
# 用法: python3 topic_ready_check.py --topics /joint_states /odom_wheel ... --timeout 60.0
import argparse
import importlib
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

DEFAULT_TOPICS = ["/joint_states", "/odom_wheel", "/IMU", "/LIDAR/POINTS", "/lio_pose"]


def import_msg_type(type_str):
    """把 "sensor_msgs/msg/PointCloud2" 形式的类型字符串动态导入为消息类"""
    pkg, _, rest = type_str.partition("/")
    module_name, _, class_name = rest.partition("/")
    module = importlib.import_module(f"{pkg}.{module_name}")
    return getattr(module, class_name)


def main():
    parser = argparse.ArgumentParser(description="等待关键话题就绪的就绪门禁节点")
    parser.add_argument("--topics", nargs="+", default=DEFAULT_TOPICS, help="待检查话题列表")
    parser.add_argument("--timeout", type=float, default=60.0, help="超时秒数")
    args = parser.parse_args()

    rclpy.init()
    node = Node("topic_ready_check")

    # 传感器话题驱动端常为 best_effort，统一用 best_effort + depth 10 订阅：
    # best_effort 订阅端兼容 reliable/best_effort 两种发布端，保证都能收到数据
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

    received = {}   # topic -> 是否已收到首帧
    missing = set(args.topics)
    for topic in args.topics:
        received[topic] = False

    def make_callback(topic):
        def callback(msg):
            if not received[topic]:
                node.get_logger().info(f"已收到 {topic} 首帧")
                received[topic] = True
        return callback

    # 用墙钟计时，不受 use_sim_time 影响
    start = time.time()
    while time.time() - start < args.timeout:
        if all(received.values()):
            break
        # 话题类型从 ROS 图动态解析（发布者可能晚于本节点启动，故循环重试）
        if missing:
            try:
                # Humble 返回 [(topic, [type,...]), ...]，先转成 dict
                names_and_types = dict(node.get_topic_names_and_types())
            except Exception:
                names_and_types = {}
            for topic in list(missing):
                type_list = names_and_types.get(topic)
                if not type_list:
                    continue
                msg_type = import_msg_type(type_list[0])
                node.create_subscription(msg_type, topic, make_callback(topic), qos)
                missing.discard(topic)
        rclpy.spin_once(node, timeout_sec=0.1)

    if all(received.values()):
        print("READY: all topics ok")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    # 超时：列出未收到首帧的话题（含未在图谱中出现的话题）
    fail_topics = [t for t in args.topics if not received[t]]
    print(f"TIMEOUT after {args.timeout:.1f}s, missing topics: {fail_topics}")
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == "__main__":
    sys.exit(main())
