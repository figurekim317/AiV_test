#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Check for the specified configuration file (default: config/config1.yaml)
CONFIG_FILE=${ROS_CONFIG_FILE:-config/config1.yaml}

echo "Using configuration file: $CONFIG_FILE"

# Execute the ROS 2 launch file
exec ros2 launch launch main_launch.py
