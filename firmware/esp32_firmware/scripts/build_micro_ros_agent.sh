#!/usr/bin/env bash
set -e

echo "=== [1/3] Preparando workspace ~/ros2_ws para micro_ros_agent ==="
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

if [ ! -d "micro_ros_agent" ]; then
    echo "Clonando micro-ROS-Agent (branch humble)..."
    git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git micro_ros_agent
else
    echo "micro_ros_agent já presente em ~/ros2_ws/src."
fi

echo "=== [2/3] Compilando micro_ros_agent via colcon ==="
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# Compila apenas o micro_ros_agent com Release
colcon build --packages-select micro_ros_agent --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "=== [3/3] Sucesso! ==="
echo "Para carregar o micro_ros_agent no seu terminal, execute:"
echo "source ~/ros2_ws/install/setup.bash"
