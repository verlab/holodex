#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: /opt/ros/humble/setup.bash not found. Please install ROS 2 Humble."
    exit 1
fi

cd "${DIR}/orbbec_ws"
echo "[build] Building orbbec_ws with colcon..."
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "[build] Done! Source with: source ${DIR}/orbbec_ws/install/setup.bash"
