#!/bin/bash
set -e

# ROS 2 환경 설정
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# 실행할 설정 파일 확인 (기본값: config1.yaml)
CONFIG_FILE=${ROS_CONFIG_FILE:-config/config1.yaml}

echo "Using configuration file: $CONFIG_FILE"

# 실행할 ROS 2 런치 파일 실행
exec ros2 launch launch main_launch.py
