#!/usr/bin/env python3
"""
Pipeline Orchestrator and Real-time Monitor for Astra Pro + small_gicp Odometry.
Coordinates bringing up camera & odometry pipeline, tracks Hz / health metrics,
and provides an interface to trigger the 3D submap export service for the scientific paper.
"""

import os
import sys
import time
import signal
import subprocess
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

class TopicRateTracker:
    def __init__(self, window_size=20):
        self.window = deque(maxlen=window_size)
        self.count = 0
        self.last_time = None
        self.msg_info = ""

    def hit(self, info=""):
        now = time.time()
        if self.last_time is not None:
            dt = now - self.last_time
            if dt > 0:
                self.window.append(dt)
        self.last_time = now
        self.count += 1
        if info:
            self.msg_info = info

    def get_hz(self):
        if len(self.window) < 2:
            return 0.0
        avg_dt = sum(self.window) / len(self.window)
        return 1.0 / avg_dt if avg_dt > 0 else 0.0

class PipelineOrchestratorNode(Node):
    def __init__(self):
        super().__init__('pipeline_orchestrator')

        self.raw_tracker = TopicRateTracker()
        self.filtered_tracker = TopicRateTracker()
        self.odom_tracker = TopicRateTracker()

        self.latest_pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}

        # QoS configurada para SensorData (Best Effort)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub_raw = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.raw_cb,
            sensor_qos
        )

        self.sub_filtered = self.create_subscription(
            PointCloud2,
            '/camera/filtered_points',
            self.filtered_cb,
            sensor_qos
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            '/icp/odom',
            self.odom_cb,
            10
        )

        self.save_map_client = self.create_client(Trigger, '/icp/save_map')

    def raw_cb(self, msg):
        pts = msg.width * msg.height
        self.raw_tracker.hit(f"{msg.width}x{msg.height} ({pts} pts)")

    def filtered_cb(self, msg):
        pts = msg.width * msg.height
        self.filtered_tracker.hit(f"{pts} pts")

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        # Conversão rápida de quaternion para yaw (graus)
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        import math
        yaw = math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi

        self.latest_pose = {
            "x": p.x, "y": p.y, "z": p.z, "yaw_deg": yaw
        }
        self.odom_tracker.hit()

    def call_save_map(self):
        if not self.save_map_client.wait_for_service(timeout_sec=2.0):
            return False, "Servico /icp/save_map nao disponivel no momento!"
        req = Trigger.Request()
        future = self.save_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            res = future.result()
            return res.success, res.message
        return False, "Timeout ao chamar servico /icp/save_map"

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Pipeline Orchestrator & Health Monitor")
    parser.add_argument('--launch-all', action='store_true', help='Inicia automaticamente camera e pipeline')
    parser.add_argument('--headless', action='store_true', help='Modo nao-interativo')
    parser.add_argument('--save-map-now', action='store_true', help='Chama o servico de salvar mapa e sai')
    args = parser.parse_args()

    rclpy.init()
    node = PipelineOrchestratorNode()

    # Thread para girar o rclpy
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    if args.save_map_now:
        print("[INFO] Chamando servico de salvamento de mapa...")
        success, msg = node.call_save_map()
        print(f"[{'SUCESSO' if success else 'FALHA'}] {msg}")
        rclpy.shutdown()
        sys.exit(0 if success else 1)

    subprocesses = []

    def cleanup(sig=None, frame=None):
        print("\n[INFO] Finalizando orquestrador e subprocessos...")
        for p in subprocesses:
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    p.kill()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    if args.launch_all:
        print("[ORQUESTRAÇÃO] Iniciando pipeline de odometria ROS 2...")
        cmd_pipeline = ["ros2", "launch", "pointcloud_odometry", "pointcloud_odometry.launch.py"]
        p_pipe = subprocess.Popen(cmd_pipeline)
        subprocesses.append(p_pipe)
        time.sleep(2)

    print("\n================================================================")
    print("      ORBBEC ASTRA PRO + small_gicp ODOMETRY ORCHESTRATOR       ")
    print("================================================================")
    print(" Comandos: [s] Salvar Submapa (.ply) | [q] Sair")
    print("----------------------------------------------------------------")

    try:
        while rclpy.ok():
            raw_hz = node.raw_tracker.get_hz()
            filt_hz = node.filtered_tracker.get_hz()
            odom_hz = node.odom_tracker.get_hz()
            p = node.latest_pose

            status_line = (
                f"\r[RAW]: {raw_hz:4.1f} Hz ({node.raw_tracker.msg_info:<18}) | "
                f"[FILT]: {filt_hz:4.1f} Hz ({node.filtered_tracker.msg_info:<10}) | "
                f"[ODOM]: {odom_hz:4.1f} Hz | "
                f"Pose: X={p['x']:+5.2f} Y={p['y']:+5.2f} Yaw={p['yaw_deg']:+5.1f}°"
            )
            sys.stdout.write(status_line)
            sys.stdout.flush()

            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        cleanup()

if __name__ == '__main__':
    main()
