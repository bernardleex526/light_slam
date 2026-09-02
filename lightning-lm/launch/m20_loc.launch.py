#!/usr/bin/env python3
# M20 定位链路一键启动（真机/仿真通用）
# 启动顺序：joints_adapter（可选）-> leg_wheel_odom（可选）-> run_loc_online -> 就绪门禁
# use_sim_time 不在此设置，由 config yaml 的 system.use_sim_time 控制
import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import FindExecutable, LaunchConfiguration

# stdbuf 逐行刷新日志（WSL 可用；若环境无 stdbuf 可去掉此前缀）
_STDBUF_PREFIX = "stdbuf -oL -eL"

# 就绪门禁必须等待的核心话题（与可选组件无关）；定位链路以 /ODOM 作为就绪标志。
# 话题名从 config yaml 读取（common.imu_topic / common.lidar_topic /
# system.odom_topic），支持隔离话题（如 /m20_slam/odom）；读取失败时回退默认名。
_DEFAULT_CORE_READY_TOPICS = ["/IMU", "/LIDAR/POINTS", "/ODOM"]


def _ready_topics_from_config(config_path):
    """从 config yaml 读取门禁需要的传感器/定位输出话题名。"""
    topics = list(_DEFAULT_CORE_READY_TOPICS)
    try:
        import yaml as _yaml

        with open(config_path, "r", encoding="utf-8") as f:
            cfg = _yaml.safe_load(f) or {}
        sys_cfg = cfg.get("system", {}) or {}
        common_cfg = cfg.get("common", {}) or {}
        topics[0] = str(common_cfg.get("imu_topic", topics[0]))
        topics[1] = str(common_cfg.get("lidar_topic", topics[1]))
        topics[2] = str(sys_cfg.get("odom_topic", topics[2]))
    except Exception:
        pass
    return topics


def _ready_check_script():
    """定位 topic_ready_check.py：优先取安装目录 lib/lightning，源码目录则取 launch/ 同目录"""
    try:
        from ament_index_python.packages import get_package_prefix

        installed = os.path.join(
            get_package_prefix("lightning"), "lib", "lightning", "topic_ready_check.py"
        )
        if os.path.exists(installed):
            return installed
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "topic_ready_check.py")


def _default_config_path():
    """config 默认值的绝对路径：优先取源码包根 config/，其次取安装前缀 lib/lightning/config/。
    避免相对路径依赖 ros2 launch 的当前工作目录，离开源码根目录执行也能找到配置。"""
    candidates = [
        os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "config", "default_m20.yaml"
        ),
    ]
    try:
        from ament_index_python.packages import get_package_prefix

        candidates.append(
            os.path.join(
                get_package_prefix("lightning"), "lib", "lightning", "config", "default_m20.yaml"
            )
        )
    except Exception:
        pass
    for path in candidates:
        if os.path.exists(path):
            return path
    return candidates[-1]


def _on_ready_check_exit(event, context):
    """就绪检查退出 0 表示全部话题就绪，launch 继续运行；
    退出非 0 表示超时/失败，此时才关闭整个 launch"""
    if event.returncode != 0:
        return Shutdown(reason=f"topic ready check 失败(exit={event.returncode})，缺少关键话题")
    return None


def _ready_check_action(context):
    """按 enable_joints_adapter/enable_leg_odom 动态组装就绪门禁话题。
    禁用任一可选组件时，不再等待其独有话题，避免门禁必然 60 秒超时并 Shutdown 整条链路。"""
    enable_joints = (
        context.launch_configurations.get("enable_joints_adapter", "true").lower() == "true"
    )
    enable_leg = context.launch_configurations.get("enable_leg_odom", "true").lower() == "true"

    config_path = context.launch_configurations.get("config", _default_config_path())
    topics = _ready_topics_from_config(config_path)
    if enable_joints:
        topics.append("/joint_states")
    if enable_leg:
        topics.append("/odom_wheel")

    ready_check = ExecuteProcess(
        cmd=[
            sys.executable, "-u", _ready_check_script(),
            "--topics", *topics,
            "--timeout", "60.0",
        ],
        output="log",
    )
    return [
        ready_check,
        RegisterEventHandler(
            OnProcessExit(target_action=ready_check, on_exit=_on_ready_check_exit)
        ),
    ]


def generate_launch_description():
    config = LaunchConfiguration("config")
    enable_joints_adapter = LaunchConfiguration("enable_joints_adapter")
    enable_leg_odom = LaunchConfiguration("enable_leg_odom")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=_default_config_path(),
            description="配置文件绝对路径（默认自动定位 default_m20.yaml，可显式覆盖）",
        ),
        DeclareLaunchArgument(
            "enable_joints_adapter",
            default_value="true",
            description="是否启动 m20_joints_adapter（开发机无 drdds 消息包时置 false）",
        ),
        DeclareLaunchArgument(
            "enable_leg_odom",
            default_value="true",
            description="是否启动 leg_wheel_odom 轮速里程计",
        ),

        # 1.（可选）JOINTS_DATA -> /joint_states 桥接，仅真机有 drdds 消息包时可用
        ExecuteProcess(
            cmd=[FindExecutable(name="joints_adapter_node")],
            output="log",
            prefix=_STDBUF_PREFIX,
            condition=IfCondition(enable_joints_adapter),
        ),
        # 2.（可选）轮速里程计，50Hz 发布 /odom_wheel
        ExecuteProcess(
            cmd=[FindExecutable(name="leg_wheel_odom_node")],
            output="log",
            prefix=_STDBUF_PREFIX,
            condition=IfCondition(enable_leg_odom),
        ),
        # 3. 在线定位主程序
        ExecuteProcess(
            cmd=[FindExecutable(name="run_loc_online"), "--config", config],
            output="log",
            prefix=_STDBUF_PREFIX,
        ),
        # 4. 就绪门禁（话题列表随可选组件动态生成；超时退出非 0 时关闭整条链路）
        OpaqueFunction(function=_ready_check_action),
    ])
