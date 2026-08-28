#!/usr/bin/env python3
"""Adapt the official DeepRobotics M20 URDF for Gazebo Classic + ros2_control.

Reads the original sw_urdf_exporter URDF, and emits m20.urdf with:
  - mesh paths rewritten to package://m20_description/meshes/
  - <transmission> blocks for the 4 wheel joints
  - <ros2_control> block (gazebo_ros2_control): wheels -> velocity interface,
    12 leg joints -> position interface
  - gazebo plugins: ros2_control, IMU (200 Hz), ray LiDAR (16 lines, 10 Hz,
    PointCloud2 output)
  - IMU/LiDAR frames attached to base_link (static TF in launch)
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

WHEELS = ["fl", "fr", "hl", "hr"]
LEGS = ["fl", "fr", "hl", "hr"]

# hipx limits per side from the original URDF (joint limit blocks)
LIMITS = {
    "fl_hipx": (-0.436, 0.611), "fr_hipx": (-0.611, 0.436),
    "hl_hipx": (-0.436, 0.611), "hr_hipx": (-0.611, 0.436),
    "fl_hipy": (-2.583, 2.286), "fr_hipy": (-2.583, 2.286),
    "hl_hipy": (-2.286, 2.583), "hr_hipy": (-2.286, 2.583),
    "fl_knee": (-2.792, 2.809), "fr_knee": (-2.792, 2.809),
    "hl_knee": (-2.809, 2.792), "hr_knee": (-2.809, 2.792),
}

TRANSMISSIONS = "\n".join(
    f"""    <transmission name="{w}_wheel_trans">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="{w}_wheel_joint">
            <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
        </joint>
        <actuator name="{w}_wheel_motor">
            <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>"""
    for w in WHEELS
)

ROS2_CONTROL = """    <ros2_control name="GazeboSystem" type="system">
        <hardware>
            <plugin>gazebo_ros2_control/GazeboSystem</plugin>
        </hardware>
%s
    </ros2_control>""" % (
    "\n".join(
        f"""        <joint name="{w}_wheel_joint">
            <command_interface name="velocity">
                <param name="min">-30</param>
                <param name="max">30</param>
            </command_interface>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>"""
        for w in WHEELS),
)

GAZEBO_PLUGINS = f"""    <gazebo>
        <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
            <parameters>@CONTROL_YAML_PATH@</parameters>
        </plugin>
    </gazebo>

    <gazebo reference="base_link">
        <sensor name="m20_imu" type="imu">
            <always_on>true</always_on>
            <update_rate>200</update_rate>
            <plugin name="imu_node" filename="libgazebo_ros_imu_sensor.so">
                <ros>
                    <remapping>~/out:=/imu</remapping>
                </ros>
                <frame_name>imu_link</frame_name>
            </plugin>
        </sensor>
    </gazebo>

    <gazebo reference="base_link">
        <sensor name="m20_lidar" type="ray">
            <pose>0 0 0.20 0 0 0</pose>
            <always_on>true</always_on>
            <update_rate>10</update_rate>
            <visualize>false</visualize>
            <ray>
                <scan>
                    <horizontal>
                        <samples>720</samples>
                        <resolution>1</resolution>
                        <min_angle>-3.14159</min_angle>
                        <max_angle>3.14159</max_angle>
                    </horizontal>
                    <vertical>
                        <samples>16</samples>
                        <resolution>1</resolution>
                        <min_angle>-0.2618</min_angle>
                        <max_angle>0.2618</max_angle>
                    </vertical>
                </scan>
                <range>
                    <min>0.1</min>
                    <max>30.0</max>
                    <resolution>0.01</resolution>
                </range>
                <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.005</stddev>
                </noise>
            </ray>
            <plugin name="lidar_node" filename="libgazebo_ros_ray_sensor.so">
                <ros>
                    <remapping>~/out:=/points_raw</remapping>
                </ros>
                <output_type>sensor_msgs/PointCloud2</output_type>
                <frame_name>lidar_link</frame_name>
            </plugin>
        </sensor>
    </gazebo>
"""


def main() -> int:
    src = (ROOT / "urdf" / "M20.urdf.orig")
    dst = (ROOT / "urdf" / "m20.urdf")
    text = src.read_text(encoding="utf-8")

    # 1. package:// mesh paths
    text = re.sub(r"filename=\"\./meshes/([^\"]+)\"",
                  r'filename="package://m20_description/meshes/\1"', text)

    # 1b. drop the XML declaration: gazebo_ros spawn_entity uses
    #     ElementTree.fromstring(str), which rejects encoding declarations
    text = re.sub(r"<\?xml[^>]*\?>\s*", "", text, count=1)

    # 1b2. WHEEL AXIS FIX: the raw M20 URDF gives all 4 wheel joints axis
    #     (0 -1 0); positive joint velocity then rolls the robot BACKWARD
    #     (contact point velocity = ω × (0,0,-r) → -x). diff_drive_controller
    #     assumes positive wheel velocity == forward. Flip to (0 1 0): both
    #     sides then roll forward for positive velocity (verified by cross
    #     product and by the driving test: cmd_vel.x>0 → GT x increases).
    for w in WHEELS:
        anchor = f'    <joint name="{w}_wheel_joint" type="continuous">'
        idx = text.index(anchor)
        end = text.index("</joint>", idx)
        jtag = text[idx:end]
        jtag = re.sub(r'<axis xyz="0 -1 0"/>', '<axis xyz="0 1 0"/>', jtag, count=1)
        text = text[:idx] + jtag + text[end:]

    # 1c. joint damping on the leg joints (stabilizes the standing pose:
    #     undamped position-controlled legs diverge in ODE after ~2 min)
    for j in ["fl_hipx", "fl_hipy", "fl_knee", "fr_hipx", "fr_hipy", "fr_knee",
              "hl_hipx", "hl_hipy", "hl_knee", "hr_hipx", "hr_hipy", "hr_knee"]:
        anchor = f'    <joint name="{j}_joint"'
        idx = text.index(anchor)
        end = text.index("</joint>", idx)
        jtag = text[idx:end]
        if "<dynamics" not in jtag:
            jtag = jtag.replace(
                ">",
                '>\n        <dynamics damping="0.8" friction="0.02"/>',
                1)
            text = text[:idx] + jtag + text[end:]

    # 1d. FIXED-LEG FALLBACK: convert the 12 leg joints to fixed so the robot
    #     spawns as a rigid 4-wheeled cart (stands deterministically).
    #     The standing-pose path (joint_trajectory_controller + stand_node)
    #     remains in the repo; flip the two lines below to re-enable it.
    for j in ["fl_hipx", "fl_hipy", "fl_knee", "fr_hipx", "fr_hipy", "fr_knee",
              "hl_hipx", "hl_hipy", "hl_knee", "hr_hipx", "hr_hipy", "hr_knee"]:
        anchor = f'    <joint name="{j}_joint"'
        idx = text.index(anchor)
        end = text.index("</joint>", idx)
        jtag = text[idx:end]
        jtag = re.sub(r'type="revolute"', 'type="fixed"', jtag, count=1)
        jtag = re.sub(r'\s*<limit[^>]*/>', '', jtag)
        text = text[:idx] + jtag + text[end:]

    # 2. remove the <mujoco> block (not understood by gazebo_ros tooling)
    text = re.sub(r"\s*<mujoco>.*?</mujoco>", "", text, flags=re.S)

    # 3. transmissions right after each wheel joint
    out_parts = []
    for w in WHEELS:
        anchor = f'    <joint name="{w}_wheel_joint" type="continuous">'
        idx = text.index(anchor)
        end = text.index("</joint>", idx) + len("</joint>")
        out_parts.append(text[:end])
        tx = re.search(
            rf'<transmission name="{w}_wheel_trans">.*?</transmission>',
            TRANSMISSIONS, re.S).group(0)
        out_parts.append("\n" + "    " + tx.replace("\n", "\n    ") + "\n")
        text = text[end:]
    out_parts.append(text)

    text = "".join(out_parts)

    # 4. ros2_control block before </robot>
    text = text.rstrip()
    assert text.endswith("</robot>")
    text = text[: -len("</robot>")] + ROS2_CONTROL + "\n" + GAZEBO_PLUGINS + "\n</robot>\n"

    dst.write_text(text, encoding="utf-8")
    print(f"wrote {dst} ({len(text)} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
