#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Source ROS 2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace
if [ -f "${DIR}/orbbec_ws/install/setup.bash" ]; then
    source "${DIR}/orbbec_ws/install/setup.bash"
else
    echo "Workspace not built yet! Run ${DIR}/scripts/build.sh first."
    exit 1
fi

# Clean stale SHM if any
ros2 run astra_camera clean_shm_node >/dev/null 2>&1 || true

echo "[start_camera] Launching Orbbec Astra Pro (ROS 2 Humble)..."
exec ros2 launch astra_pro_bringup astra_pro.launch.py
