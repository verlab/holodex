#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "${DIR}/orbbec_ws/install/setup.bash" ]; then
    source "${DIR}/orbbec_ws/install/setup.bash"
fi

ros2 run astra_camera clean_shm_node >/dev/null 2>&1 || true

echo "[test] Starting camera in background..."
ros2 launch astra_pro_bringup astra_pro.launch.py > /tmp/camera_test_launch.log 2>&1 &
CAM_PID=$!

trap "kill $CAM_PID 2>/dev/null || true" EXIT

echo "[test] Running stream verifier (timeout 15s)..."
timeout 15 python3 "${DIR}/scripts/verify_camera.py"

echo "[test] Finished successfully!"
