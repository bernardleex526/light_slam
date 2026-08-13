#!/bin/bash
# lightning-lm 依赖安装脚本
# 自动识别 ROS 发行版（支持 Foxy / Humble），适配 M20 真机 (Ubuntu 20.04 + Foxy) 与开发机 (Humble)
set -e

# 识别 ROS 发行版
if [ -n "$ROS_DISTRO" ]; then
    DISTRO="$ROS_DISTRO"
elif [ -f /opt/ros/foxy/setup.bash ]; then
    DISTRO="foxy"
elif [ -f /opt/ros/humble/setup.bash ]; then
    DISTRO="humble"
else
    echo "ERROR: 未找到 ROS2 安装，请先安装 ROS2 (Foxy 或 Humble)" >&2
    exit 1
fi
echo "Detected ROS distro: $DISTRO"

sudo apt update
sudo apt install -y \
    cmake \
    make \
    g++ \
    gcc \
    libssl-dev \
    libboost-all-dev \
    libeigen3-dev \
    libpcl-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    pcl-tools \
    libtbb-dev \
    ros-${DISTRO}-pcl-conversions \
    ros-${DISTRO}-pcl-ros

echo "依赖安装完成。"
echo "注意：Pangolin 需手动编译安装（见 docker/Dockerfile.foxy 或 README）。"