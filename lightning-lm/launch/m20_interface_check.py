#!/usr/bin/env python3
import argparse
import importlib
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

def _type(name):
    pkg, _, rest = name.partition("/")
    mod, _, cls = rest.partition("/")
    return getattr(importlib.import_module(f"{pkg}.{mod}"), cls)

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--lidar", default="/LIDAR/POINTS")
    p.add_argument("--imu", default="/IMU")
    p.add_argument("--timeout", type=float, default=10.0)
    a = p.parse_args(); rclpy.init(); n = Node("lightning_m20_interface_check")
    qos = QoSProfile(depth=64, reliability=ReliabilityPolicy.BEST_EFFORT)
    seen = {a.lidar: False, a.imu: False}; subs = {}; start = time.monotonic()
    while time.monotonic() - start < a.timeout and not all(seen.values()):
        for topic in list(seen):
            if topic in subs: continue
            types = dict(n.get_topic_names_and_types()).get(topic, [])
            if not types: continue
            expected = "sensor_msgs/msg/PointCloud2" if topic == a.lidar else "sensor_msgs/msg/Imu"
            if expected not in types: print(f"M20_INTERFACE=FAIL topic={topic} types={types}"); return 2
            cls = _type(expected)
            subs[topic] = n.create_subscription(cls, topic, lambda _m, t=topic: seen.__setitem__(t, True), qos)
        rclpy.spin_once(n, timeout_sec=0.1)
    print("M20_INTERFACE=PASS" if all(seen.values()) else "M20_INTERFACE=FAIL")
    return 0 if all(seen.values()) else 1

if __name__ == "__main__":
    try: raise SystemExit(main())
    finally:
        if rclpy.ok(): rclpy.shutdown()
