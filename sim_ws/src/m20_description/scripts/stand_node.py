#!/usr/bin/env python3
"""Send the standing-pose trajectory goal to the leg trajectory controller.

Pose (tunable via ROS params):
  hipx ~ 0, hipy ~ -0.7, knee ~ 1.4  (M20 wheeled-mode standing squat)
"""
import math
import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient

JOINTS = [
    "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint",
    "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint",
    "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint",
    "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint",
]


class StandNode(Node):
    def __init__(self):
        super().__init__("m20_stand_node")
        self.hipy = self.declare_parameter("hipy", -0.7).value
        self.knee = self.declare_parameter("knee", 1.4).value
        self.timeout = self.declare_parameter("timeout", 20.0).value
        self.client = ActionClient(
            self, FollowJointTrajectory,
            "/leg_standing_controller/follow_joint_trajectory")

    def run(self):
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("trajectory controller server not up")
            return 1
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = JOINTS
        pos = [0.0, self.hipy, self.knee] * 4
        goal.trajectory.points.append(JointTrajectoryPoint(
            positions=pos,
            time_from_start=rclpy.duration.Duration(seconds=4.0).to_msg()))
        self.get_logger().info(
            f"sending standing pose hipy={self.hipy} knee={self.knee}")
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        if future.done():
            gh = future.result()
            self.get_logger().info(f"goal accepted: {gh.accepted}")
            return 0 if gh.accepted else 1
        self.get_logger().error("timeout waiting for goal response")
        return 1


def main():
    rclpy.init()
    node = StandNode()
    rc = node.run()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
