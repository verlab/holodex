#!/usr/bin/env python3
"""
Holodex Orange - Unified Robot Orchestrator & Telemetry Dashboard.
Features:
- Hardware connectivity checks (Camera USB + ESP32 micro-ROS link)
- Real-time Visual Odometry telemetry (Pose 6-DoF, Inliers, Submap Voxels, [TRACKING_LOST])
- Live System Benchmark (Current / Avg / Peak CPU, Big A76 vs Little A55, Temp °C, RAM)
- Layered process supervision with automatic environment sourcing & log capture
- Pre-flight conflict sanitizer & Emergency Stop
"""

import os
import sys
import re
import time
import math
import select
import signal
import termios
import tty
import subprocess
import threading
import json
from collections import deque
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2, Imu, Image
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int32MultiArray, Int32, String as StringMsg
    from std_srvs.srv import Trigger
    import tf2_ros
    HAS_RCLPY = True
    HAS_TF2 = True
except ImportError:
    HAS_RCLPY = False
    HAS_TF2 = False

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False


class RateTracker:
    """Sliding-window frequency and info tracker for ROS 2 topics."""
    def __init__(self, window_size=20):
        self.window = deque(maxlen=window_size)
        self.count = 0
        self.last_time = None
        self.info = ""

    def hit(self, info=""):
        now = time.time()
        if self.last_time is not None:
            dt = now - self.last_time
            if dt > 0:
                self.window.append(dt)
        self.last_time = now
        self.count += 1
        if info:
            self.info = info

    def get_hz(self):
        if len(self.window) < 2 or (time.time() - (self.last_time or 0)) > 2.0:
            return 0.0
        avg_dt = sum(self.window) / len(self.window)
        return 1.0 / avg_dt if avg_dt > 0 else 0.0


class ManagedProcess:
    """Subprocess supervisor with automatic ROS 2 environment sourcing and log capture."""
    def __init__(self, name, command, env=None, log_dir=None):
        self.name = name
        self.raw_command = command
        self.env = env or os.environ.copy()
        self.log_dir = Path(log_dir) if log_dir else (Path.home() / ".ros" / "log" / "holodex")
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log_file = self.log_dir / f"{name}.log"
        self.proc = None
        self.start_time = None

    def start(self):
        if self.is_running():
            return True

        # Prepara o comando com sourcing automático de todos os workspaces do robô
        home_str = str(Path.home())
        source_prefix = (
            "source /opt/ros/humble/setup.bash 2>/dev/null || true; "
            f"source {home_str}/orb/orbbec_ws/install/setup.bash 2>/dev/null || source /home/orangepi/orb/orbbec_ws/install/setup.bash 2>/dev/null || true; "
            f"source {home_str}/ros2_ws/install/setup.bash 2>/dev/null || source /home/orangepi/ros2_ws/install/setup.bash 2>/dev/null || true; "
        )
        full_command = f"bash -c '{source_prefix} {self.raw_command}'"

        log_fd = open(self.log_file, "a")
        log_fd.write(f"\n--- [START] {time.strftime('%Y-%m-%d %H:%M:%S')} ---\n")
        log_fd.flush()

        try:
            self.proc = subprocess.Popen(
                full_command,
                stdout=log_fd,
                stderr=subprocess.STDOUT,
                env=self.env,
                preexec_fn=os.setsid,
                shell=True
            )
            self.start_time = time.time()
            return True
        except Exception as e:
            print(f"[ERRO] Falha ao iniciar processo '{self.name}': {e}")
            return False

    def stop(self, timeout=3.0):
        if not self.is_running():
            return True

        try:
            pgid = os.getpgid(self.proc.pid)
            os.killpg(pgid, signal.SIGINT)
            start_wait = time.time()
            while time.time() - start_wait < timeout:
                if self.proc.poll() is not None:
                    return True
                time.sleep(0.1)

            os.killpg(pgid, signal.SIGTERM)
            time.sleep(0.5)
            if self.proc.poll() is None:
                os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        except Exception:
            pass

        return True

    def is_running(self):
        return self.proc is not None and self.proc.poll() is None

    def get_status_str(self):
        if self.is_running():
            uptime = int(time.time() - (self.start_time or time.time()))
            return f"\033[92m● ATIVO\033[0m (PID: {self.proc.pid}, {uptime}s)"
        elif self.proc is not None:
            return f"\033[91m■ MORTO\033[0m (Exit: {self.proc.returncode})"
        return "\033[90m○ INATIVO\033[0m"

    def tail_log(self, lines=8):
        if not self.log_file.exists():
            return []
        try:
            with open(self.log_file, "r") as f:
                all_lines = f.readlines()
                return [line.strip() for line in all_lines[-lines:]]
        except Exception:
            return []


class OrchestratorNode(Node if HAS_RCLPY else object):
    """ROS 2 Node aggregating telemetry, health metrics, and emergency controls."""
    def __init__(self):
        if not HAS_RCLPY:
            raise RuntimeError("rclpy não está disponível neste ambiente Python!")
        super().__init__('holodex_orchestrator')

        # Trackers de Frequência
        self.raw_cloud_tracker = RateTracker()
        self.filtered_cloud_tracker = RateTracker()
        self.submap_cloud_tracker = RateTracker()
        self.yolo_mask_tracker = RateTracker()
        self.ticks_tracker = RateTracker()
        self.imu_raw_tracker = RateTracker()
        self.imu_filt_tracker = RateTracker()
        self.ekf_odom_tracker = RateTracker()
        self.icp_odom_tracker = RateTracker()
        self.cmd_vel_tracker = RateTracker()
        self.cmd_nav_tracker = RateTracker()
        self.heartbeat_tracker = RateTracker()
        self.supervisor_status = {}

        # Telemetria do Robô & Estado
        self.ekf_pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "roll": 0.0, "pitch": 0.0, "vx": 0.0, "vy": 0.0, "wz": 0.0}
        self.icp_pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "roll": 0.0, "pitch": 0.0, "vx": 0.0, "vy": 0.0, "wz": 0.0}
        self.last_ticks = [0, 0, 0, 0]
        self.last_heartbeat = 0
        self.last_ticks_time = 0.0
        self.last_heartbeat_time = 0.0

        # Telemetria Avançada da Odometria Visual (small_gicp)
        self.icp_telemetry = {
            "state": "NO_DATA",
            "inliers": 0,
            "inlier_pct": 0.0,
            "points": 0,
            "submap_voxels": 0,
            "keyframes": 0,
            "icp_ms": 0.0,
            "throughput_hz": 0.0
        }

        # QoS
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Inscrições de Sensores
        self.create_subscription(PointCloud2, '/camera/depth/points', self._on_raw_cloud, best_effort_qos)
        self.create_subscription(PointCloud2, '/camera/filtered_points', self._on_filtered_cloud, best_effort_qos)
        self.create_subscription(PointCloud2, '/icp/submap_cloud', self._on_submap_cloud, 10)
        self.create_subscription(Image, '/yolo/dynamic_mask', self._on_yolo_mask, best_effort_qos)
        self.create_subscription(Int32MultiArray, '/wheel/ticks', self._on_wheel_ticks, best_effort_qos)
        self.create_subscription(Imu, '/imu/data_raw', self._on_imu_raw, best_effort_qos)
        self.create_subscription(Imu, '/imu/data', self._on_imu_data, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_ekf_odom, 10)
        self.create_subscription(Odometry, '/icp/odom', self._on_icp_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self._on_cmd_nav, 10)
        self.create_subscription(Int32, '/esp32/heartbeat', self._on_heartbeat, best_effort_qos)
        self.create_subscription(StringMsg, '/icp/telemetry', self._on_icp_telemetry, 10)
        self.create_subscription(StringMsg, '/motion_supervisor/status', self._on_supervisor_status, 10)

        # Publicador de Emergência e Serviços
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.save_map_client = self.create_client(Trigger, '/icp/save_submap')
        self.save_map_client_legacy = self.create_client(Trigger, '/icp/save_map')

        # Monitor de Transformações TF2
        if HAS_TF2:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_raw_cloud(self, msg):
        self.raw_cloud_tracker.hit(f"{msg.width}x{msg.height}")

    def _on_filtered_cloud(self, msg):
        pts = msg.width * msg.height
        self.filtered_cloud_tracker.hit(f"{pts} pts")

    def _on_submap_cloud(self, msg):
        pts = msg.width * msg.height
        self.submap_cloud_tracker.hit(f"{pts} voxels")

    def _on_yolo_mask(self, msg):
        self.yolo_mask_tracker.hit(f"{msg.width}x{msg.height}")

    def _on_wheel_ticks(self, msg):
        if len(msg.data) >= 4:
            self.last_ticks = list(msg.data[:4])
        self.last_ticks_time = time.time()
        self.ticks_tracker.hit()

    def _on_imu_raw(self, msg):
        self.imu_raw_tracker.hit()

    def _on_imu_data(self, msg):
        self.imu_filt_tracker.hit()

    def _on_heartbeat(self, msg):
        self.last_heartbeat = msg.data
        self.last_heartbeat_time = time.time()
        self.heartbeat_tracker.hit(f"#{msg.data}")

    def _on_cmd_vel(self, msg):
        self.cmd_vel_tracker.hit(f"vx={msg.linear.x:+.2f} vy={msg.linear.y:+.2f} wz={msg.angular.z:+.2f}")

    def _on_cmd_nav(self, msg):
        self.cmd_nav_tracker.hit(f"vx={msg.linear.x:+.2f} vy={msg.linear.y:+.2f} wz={msg.angular.z:+.2f}")

    def _on_supervisor_status(self, msg):
        try:
            self.supervisor_status = json.loads(msg.data)
        except Exception:
            pass

    def _on_icp_telemetry(self, msg):
        try:
            parts = msg.data.split(';')
            for p in parts:
                if ':' in p:
                    k, v = p.split(':', 1)
                    if k == "state":
                        self.icp_telemetry["state"] = v
                    elif k in ["inliers", "points", "submap_voxels", "keyframes"]:
                        self.icp_telemetry[k] = int(v)
                    elif k in ["inlier_pct", "icp_ms", "throughput_hz"]:
                        self.icp_telemetry[k] = float(v)
        except Exception:
            pass

    def _on_ekf_odom(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t = msg.twist.twist
        r, pt, y = self._quat_to_rpy_deg(o)
        self.ekf_pose = {"x": p.x, "y": p.y, "z": p.z, "roll": r, "pitch": pt, "yaw": y, "vx": t.linear.x, "vy": t.linear.y, "wz": t.angular.z}
        self.ekf_odom_tracker.hit()

    def _on_icp_odom(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t = msg.twist.twist
        r, pt, y = self._quat_to_rpy_deg(o)
        self.icp_pose = {"x": p.x, "y": p.y, "z": p.z, "roll": r, "pitch": pt, "yaw": y, "vx": t.linear.x, "vy": t.linear.y, "wz": t.angular.z}
        self.icp_odom_tracker.hit()

    def _quat_to_rpy_deg(self, o):
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (o.w * o.x + o.y * o.z)
        cosr_cosp = 1.0 - 2.0 * (o.x * o.x + o.y * o.y)
        roll = math.atan2(sinr_cosp, cosr_cosp) * 180.0 / math.pi

        # Pitch (y-axis rotation)
        sinp = 2.0 * (o.w * o.y - o.z * o.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(90.0, sinp)
        else:
            pitch = math.asin(sinp) * 180.0 / math.pi

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        yaw = math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi

        return roll, pitch, yaw

    def emergency_stop(self):
        msg = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.02)

    def check_tf(self, target_frame, source_frame):
        if not HAS_TF2 or not hasattr(self, 'tf_buffer'):
            return False
        try:
            return self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time())
        except Exception:
            return False

    def trigger_save_submap(self, timeout_sec=3.0):
        client = self.save_map_client
        if not client.wait_for_service(timeout_sec=0.5):
            client = self.save_map_client_legacy
            if not client.wait_for_service(timeout_sec=0.5):
                return False, "Serviço /icp/save_submap ou /icp/save_map indisponível"
        req = Trigger.Request()
        future = client.call_async(req)
        start = time.time()
        while time.time() - start < timeout_sec:
            if future.done():
                res = future.result()
                return res.success, res.message
            time.sleep(0.05)
        return False, "Timeout aguardando /icp/save_submap"


class HolodexOrchestrator:
    """Master manager orchestrating processes, safety, TUI, live benchmark, and telemetry."""
    def __init__(self, serial_port="/dev/ttyUSB0", camera_enabled=True, esp32_enabled=True, odom_enabled=True, supervisor_enabled=True, nav2_enabled=False, yolo_enabled=False):
        self.serial_port = serial_port
        self.camera_enabled = camera_enabled
        self.esp32_enabled = esp32_enabled
        self.odom_enabled = odom_enabled
        self.supervisor_enabled = supervisor_enabled
        self.nav2_enabled = nav2_enabled
        self.yolo_enabled = yolo_enabled

        self.node = None
        self.executor = None
        self.spin_thread = None
        self.running = True
        self.processes = {}
        self.status_message = "Inicializando orquestrador..."

        # Métricas de Benchmark Contínuo da Sessão
        self.bench_cpu_samples = deque(maxlen=600)  # Últimos 5 minutos (0.5s taxa)
        self.bench_temp_samples = deque(maxlen=600)
        self.bench_ram_peak = 0.0
        self.bench_cpu_peak = 0.0
        self.bench_temp_peak = 0.0

        self.env = os.environ.copy()
        self.log_dir = Path.home() / ".ros" / "log" / "holodex"

        self._setup_processes()

    def _setup_processes(self):
        # 1. Driver Orbbec Astra Pro
        cam_cmd = "ros2 launch astra_pro_bringup astra_pro.launch.py"
        self.processes["camera"] = ManagedProcess("camera", cam_cmd, env=self.env, log_dir=self.log_dir)

        # 2. micro-ROS Agent (ESP32)
        uros_cmd = f"ros2 run micro_ros_agent micro_ros_agent serial --dev {self.serial_port} -b 921600"
        self.processes["micro_ros"] = ManagedProcess("micro_ros", uros_cmd, env=self.env, log_dir=self.log_dir)

        # 3. Robot State Publisher (URDF: base_footprint -> base_link -> camera_link, imu_link, rodas)
        rsp_cmd = "ros2 launch holodex_bringup rsp.launch.py"
        self.processes["robot_state_pub"] = ManagedProcess("robot_state_pub", rsp_cmd, env=self.env, log_dir=self.log_dir)

        # 4. Odometria de Rodas + Filtro Madgwick + EKF
        base_cmd = "ros2 launch mecanum_wheel_odometry base_fusion.launch.py"
        self.processes["base_fusion"] = ManagedProcess("base_fusion", base_cmd, env=self.env, log_dir=self.log_dir)

        # 5. Filtro Striding + Odometria 3D small_gicp
        icp_cmd = "ros2 launch pointcloud_odometry pointcloud_odometry.launch.py"
        self.processes["pointcloud_odom"] = ManagedProcess("pointcloud_odom", icp_cmd, env=self.env, log_dir=self.log_dir)

        # 6. Supervisor de Movimento (Envelope de Segurança & Slip Guard)
        sup_cmd = "ros2 launch holodex_bringup supervisor.launch.py"
        self.processes["motion_supervisor"] = ManagedProcess("supervisor", sup_cmd, env=self.env, log_dir=self.log_dir)

        # 7. Stack Nav2 Holonômica (Planejamento Mecanum 4WD)
        nav2_cmd = "ros2 launch holodex_bringup navigation.launch.py"
        self.processes["nav2"] = ManagedProcess("nav2", nav2_cmd, env=self.env, log_dir=self.log_dir)

        # 8. Inferência NPU YOLOv8-seg (VIP9000 3 TOPS) para Máscara Dinâmica
        yolo_model = Path.home() / "yolov8n_seg_final.nb"
        if not yolo_model.exists():
            yolo_model = Path("/home/orangepi/yolov8n_seg_final.nb")
        yolo_cmd = f"ros2 run yolo_seg_ros2 yolo_seg_node --continuous --topic /camera/color/image_raw --mask-topic /yolo/dynamic_mask --model {yolo_model}"
        self.processes["yolo_seg"] = ManagedProcess("yolo_npu", yolo_cmd, env=self.env, log_dir=self.log_dir)

    @staticmethod
    def sanitize_environment():
        """Verifica e encerra processos concorrentes/órfãos para evitar sobreaquecimento da CPU."""
        conflicting_patterns = [
            "icp_odometry", "component_container", "astra_camera",
            "micro_ros_agent", "ekf_node", "mecanum_wheel_odometry",
            "motion_supervisor", "nav2", "controller_server", "planner_server", "bt_navigator",
            "yolo_seg_node"
        ]
        found_pids = []

        if HAS_PSUTIL:
            my_pid = os.getpid()
            for p in psutil.process_iter(['pid', 'name', 'cmdline']):
                if p.info['pid'] == my_pid:
                    continue
                cmdline = " ".join(p.info['cmdline'] or [])
                for pattern in conflicting_patterns:
                    if pattern in cmdline:
                        found_pids.append((p.info['pid'], p.info['name'], pattern))
                        break

        return found_pids

    @staticmethod
    def kill_conflicting_processes(found_pids):
        for pid, name, pattern in found_pids:
            try:
                os.kill(pid, signal.SIGTERM)
                time.sleep(0.1)
                os.kill(pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

    @staticmethod
    def check_devices():
        """Verifica presença de portas USB e sensores físicos."""
        devices = {
            "serial": False,
            "serial_path": "",
            "camera_usb": False,
            "camera_video": False
        }

        # Serial ESP32
        for p in ["/dev/esp32", "/dev/ttyUSB0", "/dev/ttyUSB1"]:
            if os.path.exists(p):
                devices["serial"] = True
                devices["serial_path"] = p
                break

        # USB Astra Pro (ID 2bc5:0403 ou 2bc5:0501)
        try:
            lsusb_out = subprocess.check_output("lsusb", shell=True, text=True, stderr=subprocess.DEVNULL)
            if "2bc5:0403" in lsusb_out or "2bc5:0501" in lsusb_out:
                devices["camera_usb"] = True
        except Exception:
            pass

        if os.path.exists("/dev/video0"):
            devices["camera_video"] = True

        return devices

    def start_all(self):
        """Inicializa processos em camadas ordenadas."""
        self.status_message = "Subindo processos do robô..."
        devs = self.check_devices()

        # Inicia câmera se conectada e habilitada
        if self.camera_enabled:
            if devs["camera_usb"]:
                subprocess.run("bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null; source /home/orangepi/orb/orbbec_ws/install/setup.bash 2>/dev/null; ros2 run astra_camera clean_shm_node 2>/dev/null || true'", shell=True)
                self.processes["camera"].start()
                time.sleep(1.5)
            else:
                self.status_message = "[AVISO] Câmera Astra não conectada via USB."

        # Inicia micro-ROS se porta serial existir
        if self.esp32_enabled:
            if devs["serial"]:
                self.serial_port = devs["serial_path"]
                self._setup_processes()
                self.processes["micro_ros"].start()
                time.sleep(1.0)
            else:
                self.status_message += " [AVISO] ESP32 não conectado."

        # Inicia URDF e Robot State Publisher
        self.processes["robot_state_pub"].start()
        time.sleep(0.5)

        # Inicia odometria de rodas / EKF
        if self.odom_enabled:
            self.processes["base_fusion"].start()
            time.sleep(1.0)

        # Inicia odometria visual se câmera estiver ativa
        if self.odom_enabled and self.camera_enabled and devs["camera_usb"]:
            self.processes["pointcloud_odom"].start()
            time.sleep(1.0)

        # Inicia supervisor de movimento
        if self.supervisor_enabled:
            self.processes["motion_supervisor"].start()
            time.sleep(0.5)

        # Inicia inferência YOLO na NPU (VIP9000) se habilitada
        if self.yolo_enabled and self.camera_enabled and devs["camera_usb"]:
            self.processes["yolo_seg"].start()
            time.sleep(1.0)

        # Inicia Nav2 se habilitado
        if self.nav2_enabled:
            self.processes["nav2"].start()

        self.status_message = "Nós supervisionados iniciados."

    def stop_all(self):
        """Encerramento seguro com parada de emergência e salvamento do mapa."""
        self.status_message = "Executando parada segura..."
        if self.node is not None:
            self.node.emergency_stop()
            try:
                self.node.trigger_save_submap(timeout_sec=2.0)
            except Exception:
                pass

        for name, p in reversed(list(self.processes.items())):
            p.stop(timeout=2.0)

        self.status_message = "Todos os processos finalizados."

    def get_system_telemetry(self):
        """Coleta métricas e alimenta o profiler de benchmark da sessão."""
        metrics = {
            "temp_c": 0.0,
            "temp_avg": 0.0,
            "temp_peak": 0.0,
            "cpu_total": 0.0,
            "cpu_avg": 0.0,
            "cpu_peak": 0.0,
            "cpus_little": [],
            "cpus_big": [],
            "ram_used_gb": 0.0,
            "ram_peak_gb": 0.0,
            "ram_total_gb": 0.0
        }

        # Temperatura Allwinner SoC
        for zone in ["/sys/class/thermal/thermal_zone0/temp", "/sys/class/thermal/thermal_zone1/temp"]:
            if os.path.exists(zone):
                try:
                    with open(zone, "r") as f:
                        metrics["temp_c"] = float(f.read().strip()) / 1000.0
                        break
                except Exception:
                    pass

        # CPU e Memória via psutil
        if HAS_PSUTIL:
            per_cpu = psutil.cpu_percent(percpu=True)
            metrics["cpu_total"] = psutil.cpu_percent()
            if len(per_cpu) >= 8:
                metrics["cpus_little"] = per_cpu[0:6]  # 6x Cortex-A55 (Little)
                metrics["cpus_big"] = per_cpu[6:8]     # 2x Cortex-A76 (Big)
            else:
                metrics["cpus_little"] = per_cpu

            mem = psutil.virtual_memory()
            metrics["ram_used_gb"] = (mem.total - mem.available) / (1024 ** 3)
            metrics["ram_total_gb"] = mem.total / (1024 ** 3)

        # Atualiza histórico de benchmark
        if metrics["temp_c"] > 0:
            self.bench_temp_samples.append(metrics["temp_c"])
            self.bench_temp_peak = max(self.bench_temp_peak, metrics["temp_c"])
            metrics["temp_avg"] = sum(self.bench_temp_samples) / len(self.bench_temp_samples)
            metrics["temp_peak"] = self.bench_temp_peak

        if metrics["cpu_total"] > 0:
            self.bench_cpu_samples.append(metrics["cpu_total"])
            self.bench_cpu_peak = max(self.bench_cpu_peak, metrics["cpu_total"])
            metrics["cpu_avg"] = sum(self.bench_cpu_samples) / len(self.bench_cpu_samples)
            metrics["cpu_peak"] = self.bench_cpu_peak

        self.bench_ram_peak = max(self.bench_ram_peak, metrics["ram_used_gb"])
        metrics["ram_peak_gb"] = self.bench_ram_peak

        return metrics

    def update_icp_metrics_from_log_fallback(self):
        """Se o tópico /icp/telemetry ainda não tiver dados, faz parse do log do processo."""
        if self.node is None or self.node.icp_telemetry["state"] != "NO_DATA":
            return

        lines = self.processes["pointcloud_odom"].tail_log(6)
        # Exemplo da linha:
        # [ICP Odometria] Throughput=29.8 msg/s | Pts=2145 | Inliers=1850 (86%) | ICP=6.8 ms | Pose=[0.12, -0.04, 0.01] RPY=[0.2, -0.8, 14.2] deg | Submap Voxels=482 | [TRACKING_LOST]
        for line in reversed(lines):
            if "[ICP Odometria]" in line:
                m_tp = re.search(r"Throughput=([\d\.]+)", line)
                m_pts = re.search(r"Pts=(\d+)", line)
                m_inliers = re.search(r"Inliers=(\d+)\s*\(([\d\.]+)%\)", line)
                m_icp = re.search(r"ICP=([\d\.]+)\s*ms", line)
                m_vox = re.search(r"Submap Voxels=(\d+)", line)
                is_lost = "[TRACKING_LOST]" in line

                if m_pts: self.node.icp_telemetry["points"] = int(m_pts.group(1))
                if m_inliers:
                    self.node.icp_telemetry["inliers"] = int(m_inliers.group(1))
                    self.node.icp_telemetry["inlier_pct"] = float(m_inliers.group(2))
                if m_icp: self.node.icp_telemetry["icp_ms"] = float(m_icp.group(1))
                if m_vox: self.node.icp_telemetry["submap_voxels"] = int(m_vox.group(1))
                if m_tp: self.node.icp_telemetry["throughput_hz"] = float(m_tp.group(1))
                self.node.icp_telemetry["state"] = "TRACKING_LOST" if is_lost else "TRACKING_OK"
                break

    def render_dashboard(self):
        """Renderiza dashboard unificado e rico com telemetria, benchmark e tracking."""
        t = self.get_system_telemetry()
        self.update_icp_metrics_from_log_fallback()
        n = self.node
        devs = self.check_devices()

        # Cores térmicas
        temp_color = "\033[92m" if t["temp_c"] < 65.0 else ("\033[93m" if t["temp_c"] < 80.0 else "\033[91m")

        # Formatação CPU
        little_str = " ".join([f"{c:4.1f}%" for c in t["cpus_little"]])
        big_str = " ".join([f"{c:4.1f}%" for c in t["cpus_big"]]) if t["cpus_big"] else "N/A"

        # Estado da Câmera
        if devs["camera_usb"]:
            cam_dev_str = "\033[92m● CONECTADA\033[0m (2bc5:0403 Astra Pro + /dev/video0)"
        else:
            cam_dev_str = "\033[91m○ DESCONECTADA\033[0m (Astra Pro USB não detectada)"

        # Estado da Conexão com ESP32
        now = time.time()
        last_esp_time = max(n.last_ticks_time, n.last_heartbeat_time) if n else 0.0
        esp_streaming = (now - last_esp_time) < 1.5 and last_esp_time > 0.0

        if esp_streaming:
            esp_status_str = f"\033[92m● CONECTADO & ATIVO\033[0m (Heartbeat #{n.last_heartbeat}, Ticks: {n.last_ticks})"
        elif devs["serial"]:
            esp_status_str = f"\033[93m▲ SEM TELEMETRIA\033[0m (Porta {devs['serial_path']} presente, aguardando micro-ROS)"
        else:
            esp_status_str = "\033[91m○ DESCONECTADO\033[0m (Porta /dev/ttyUSB0 ou /dev/esp32 ausente)"

        # Estado da Odometria Visual
        icp_st = n.icp_telemetry["state"] if n else "NO_DATA"
        if icp_st == "TRACKING_OK":
            track_badge = "\033[92m● TRACKING_OK\033[0m"
        elif icp_st == "TRACKING_LOST":
            track_badge = "\033[91m✖ TRACKING_LOST\033[0m"
        elif icp_st == "NO_DATA":
            track_badge = "\033[90m○ AGUARDANDO NUVEM\033[0m"
        else:
            track_badge = f"\033[93m▲ {icp_st}\033[0m"

        # Verificação da Árvore TF2
        tf_odom_base = n.check_tf('odom', 'base_link') if n else False
        tf_base_cam = n.check_tf('base_link', 'camera_link') if n else False
        tf_base_imu = n.check_tf('base_link', 'imu_link') if n else False

        tf_odom_str = "\033[92m● OK\033[0m" if tf_odom_base else "\033[93m○ PENDENTE\033[0m"
        tf_cam_str = "\033[92m● OK\033[0m" if tf_base_cam else "\033[91m✖ DESCONEXO\033[0m"
        tf_imu_str = "\033[92m● OK\033[0m" if tf_base_imu else "\033[91m✖ DESCONEXO\033[0m"

        out = []
        out.append("\033[H\033[J")  # Clear screen e move cursor para o topo
        out.append("================================================================================")
        out.append("              HOLODEX ORANGE - ORQUESTRADOR CENTRAL DO ROBÔ                    ")
        out.append("================================================================================")
        out.append(f" HARDWARE & CONEXÕES:")
        out.append(f"  • Câmera Orbbec Astra : {cam_dev_str}")
        out.append(f"  • Base ESP32 micro-ROS: {esp_status_str}")
        out.append(f"  • Árvore TF (TF2)     : odom->base_link [{tf_odom_str}] | base_link->camera_link [{tf_cam_str}] | base_link->imu_link [{tf_imu_str}]")
        out.append("--------------------------------------------------------------------------------")
        out.append(" BENCHMARK & DESEMPENHO DA SBC (Allwinner A733 big.LITTLE):")
        out.append(f"  • Temperatura CPU: {temp_color}{t['temp_c']:5.1f} °C\033[0m (Média: {t['temp_avg']:4.1f}°C | Pico: {t['temp_peak']:4.1f}°C)")
        out.append(f"  • CPU Total      : {t['cpu_total']:4.1f}% (Média: {t['cpu_avg']:4.1f}% | Pico: {t['cpu_peak']:4.1f}%) | RAM: {t['ram_used_gb']:.2f}/{t['ram_total_gb']:.1f} GB (Pico: {t['ram_peak_gb']:.2f}GB)")
        out.append(f"  • Núcleos Big   (2x A76):   [{big_str}] -> small_gicp & Visão")
        out.append(f"  • Núcleos Little(6x A55):   [{little_str}] -> I/O & EKF")
        out.append("--------------------------------------------------------------------------------")
        out.append(" PROCESSOS SUPERVISIONADOS:")
        for name, p in self.processes.items():
            out.append(f"  • {name:<16}: {p.get_status_str()}")
        out.append("--------------------------------------------------------------------------------")

        if n is not None:
            out.append(" FLUXO DE DADOS & TAXAS (Hz):")
            out.append(f"  • /camera/depth/points    : {n.raw_cloud_tracker.get_hz():4.1f} Hz ({n.raw_cloud_tracker.info})")
            out.append(f"  • /camera/filtered_points : {n.filtered_cloud_tracker.get_hz():4.1f} Hz ({n.filtered_cloud_tracker.info})")
            out.append(f"  • /icp/submap_cloud (3D)  : {n.submap_cloud_tracker.get_hz():4.1f} Hz ({n.submap_cloud_tracker.info})")
            out.append(f"  • /yolo/dynamic_mask(NPU) : {n.yolo_mask_tracker.get_hz():4.1f} Hz ({n.yolo_mask_tracker.info})")
            out.append(f"  • /wheel/ticks (Rodas)    : {n.ticks_tracker.get_hz():4.1f} Hz | /imu/data_raw: {n.imu_raw_tracker.get_hz():4.1f} Hz")
            out.append(f"  • /odometry/filtered(EKF) : {n.ekf_odom_tracker.get_hz():4.1f} Hz | /icp/odom: {n.icp_odom_tracker.get_hz():4.1f} Hz")
            out.append("--------------------------------------------------------------------------------")
            out.append(" ODOMETRIA VISUAL 3D (small_gicp):")
            tele = n.icp_telemetry
            ip = n.icp_pose
            out.append(f"  • Estado do Rastreio : {track_badge} | Latência Solver: {tele['icp_ms']:4.1f} ms | Taxa: {tele['throughput_hz']:4.1f} msg/s")
            out.append(f"  • Pontos Estruturais : {tele['inliers']} inliers de {tele['points']} pontos ({tele['inlier_pct']:4.1f}%)")
            out.append(f"  • Memória do Submapa : {tele['submap_voxels']} voxels gaussianos na RAM | Keyframes: #{tele['keyframes']}")
            out.append(f"  • Pose Estimada (ICP): X={ip['x']:+5.2f}m  Y={ip['y']:+5.2f}m  Z={ip['z']:+5.2f}m | Yaw={ip['yaw']:+5.1f}°  Pitch={ip['pitch']:+4.1f}°  Roll={ip['roll']:+4.1f}°")
            out.append(f"  • Velocidades (ICP)  : vx={ip['vx']:+4.2f} m/s  vy={ip['vy']:+4.2f} m/s  wz={ip['wz']:+4.2f} rad/s")

            # Supervisor de Movimento & Nav2
            sup = n.supervisor_status if hasattr(n, 'supervisor_status') and n.supervisor_status else None
            out.append("--------------------------------------------------------------------------------")
            out.append(" SUPERVISOR DE MOVIMENTO & NAVEGAÇÃO:")
            if sup:
                mode = sup.get('mode', 'IDLE')
                cin = sup.get('cmd_in', [0, 0, 0])
                cout = sup.get('cmd_out', [0, 0, 0])
                sc = sup.get('scales', {})
                md = sup.get('min_dist', {})
                
                mode_color = "\033[92m" if mode in ["NAV2", "TELEOP"] else "\033[90m"
                mode_str = f"{mode_color}● {mode}\033[0m"
                
                front_str = f"{md.get('front')}m" if md.get('front') is not None else ">1.8m"
                rear_str = f"\033[91m{md.get('rear')}m [BLIND STOP]\033[0m" if (md.get('rear') is not None and md.get('rear') < 0.25) else (f"{md.get('rear')}m" if md.get('rear') is not None else ">1.8m")
                left_str = f"{md.get('left')}m" if md.get('left') is not None else ">1.8m"
                right_str = f"{md.get('right')}m" if md.get('right') is not None else ">1.8m"

                out.append(f"  • Modo Operacional  : {mode_str} | Escalas: [Fwd: {sc.get('fwd', 1.0)} | Bwd: {sc.get('bwd', 1.0)} | Lft: {sc.get('lft', 1.0)} | Rgt: {sc.get('rgt', 1.0)}]")
                out.append(f"  • Envelope 360°     : Frente: {front_str} | Traseira: {rear_str} | Esq: {left_str} | Dir: {right_str}")
                out.append(f"  • Nav2 In (/cmd_nav): {n.cmd_nav_tracker.get_hz():4.1f} Hz (vx={cin[0]:+.2f} vy={cin[1]:+.2f} wz={cin[2]:+.2f})")
                out.append(f"  • ESP Out (/cmd_vel): {n.cmd_vel_tracker.get_hz():4.1f} Hz (vx={cout[0]:+.2f} vy={cout[1]:+.2f} wz={cout[2]:+.2f})")
            else:
                out.append(f"  • Status Supervisor : \033[90m○ AGUARDANDO DADOS (/motion_supervisor/status)\033[0m")
                out.append(f"  • Nav2 In (/cmd_nav): {n.cmd_nav_tracker.get_hz():4.1f} Hz | ESP Out (/cmd_vel): {n.cmd_vel_tracker.get_hz():4.1f} Hz")

        out.append("--------------------------------------------------------------------------------")
        out.append(f" MENSAGEM: {self.status_message}")
        out.append(" COMANDOS: [n] Toggle Nav2   | [y] Toggle YOLO NPU | [s] Salvar Submapa | [e] Parada Emergência")
        out.append("           [k] Matar Órfãos  | [r] Reiniciar Nós   | [l] Logs Recentes  | [q] Sair")
        out.append("================================================================================")

        sys.stdout.write("\n".join(out) + "\n")
        sys.stdout.flush()

    def run_interactive(self, duration=0):
        """Loop principal com captura não-bloqueante de teclado."""
        if not HAS_RCLPY:
            print("\033[91m[ERRO] ROS 2 (rclpy) não detectado no ambiente Python.\033[0m")
            print("Por favor, certifique-se de executar:")
            print("  source /opt/ros/humble/setup.bash")
            print("  source ~/ros2_ws/install/setup.bash")
            sys.exit(1)

        # Inicialização do nó ROS 2 em thread separada
        rclpy.init()
        self.node = OrchestratorNode()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        # Checagem pré-voo de processos conflitantes
        orphans = self.sanitize_environment()
        if orphans:
            self.status_message = f"\033[93mAviso: {len(orphans)} processos concorrentes detectados. Pressione [k] para limpá-los.\033[0m"

        # Inicia todos os subprocessos configurados
        self.start_all()

        old_settings = None
        start_loop = time.time()
        try:
            if sys.stdin.isatty():
                old_settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())

            last_render = 0
            while self.running:
                now = time.time()
                if duration > 0 and (now - start_loop) >= duration:
                    self.status_message = f"Tempo de teste concluído ({duration}s)."
                    break

                if now - last_render >= 0.5:
                    self.render_dashboard()
                    last_render = now

                if select.select([sys.stdin], [], [], 0.1)[0]:
                    char = sys.stdin.read(1).lower()
                    if char == 'q':
                        self.status_message = "Finalizando sistema..."
                        break
                    elif char == 'n':
                        if self.processes["nav2"].is_running():
                            self.processes["nav2"].stop()
                            self.status_message = "Nav2 desativado."
                        else:
                            self.processes["nav2"].start()
                            self.status_message = "Nav2 iniciado em segundo plano."
                    elif char == 'y':
                        if self.processes["yolo_seg"].is_running():
                            self.processes["yolo_seg"].stop()
                            self.status_message = "Inferência YOLO NPU desativada."
                        else:
                            self.processes["yolo_seg"].start()
                            self.status_message = "Inferência YOLO NPU ativada (VIP9000)."
                    elif char == 'e':
                        self.node.emergency_stop()
                        self.status_message = "\033[91mPARADA DE EMERGÊNCIA DISPARADA!\033[0m"
                    elif char == 's':
                        self.status_message = "Salvando submapa 3D consolidado..."
                        succ, msg = self.node.trigger_save_submap()
                        self.status_message = f"[{'OK' if succ else 'FALHA'}] {msg}"
                    elif char == 'k':
                        orphans = self.sanitize_environment()
                        self.kill_conflicting_processes(orphans)
                        self.status_message = f"Processos concorrentes limpos ({len(orphans)} eliminados)."
                    elif char == 'r':
                        self.start_all()
                    elif char == 'l':
                        logs = self.processes["pointcloud_odom"].tail_log(6)
                        self.status_message = "Log ICP: " + " | ".join(logs[-2:] if logs else ["Vazio"])

        except KeyboardInterrupt:
            pass
        finally:
            if old_settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self.stop_all()
            if self.node is not None and HAS_RCLPY:
                try:
                    if hasattr(self, 'executor') and self.executor:
                        self.executor.shutdown()
                    if hasattr(self, 'spin_thread') and self.spin_thread:
                        self.spin_thread.join(timeout=1.0)
                    self.node.destroy_node()
                    rclpy.shutdown()
                except Exception:
                    pass


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Holodex Orange - Robot Orchestrator & Monitor")
    parser.add_argument('--serial', default='/dev/ttyUSB0', help='Porta serial ESP32 (default: /dev/ttyUSB0)')
    parser.add_argument('--no-camera', action='store_true', help='Não inicializa driver da câmera Astra')
    parser.add_argument('--no-esp32', action='store_true', help='Não inicializa micro-ROS agent')
    parser.add_argument('--no-odom', action='store_true', help='Não inicializa nós de odometria')
    parser.add_argument('--no-supervisor', action='store_true', help='Não inicializa supervisor de movimento')
    parser.add_argument('--nav2', action='store_true', help='Inicia automaticamente a stack Nav2')
    parser.add_argument('--yolo', action='store_true', help='Inicia inferência de segmentação semântica acelerada na NPU (VIP9000)')
    parser.add_argument('--duration', type=int, default=0, help='Executa por N segundos e finaliza automaticamente')
    parser.add_argument('--check-only', action='store_true', help='Apenas checa integridade e processos órfãos e sai')
    parser.add_argument('--kill-orphans', action='store_true', help='Elimina processos ROS órfãos e sai')
    args = parser.parse_args()

    if args.check_only:
        orphans = HolodexOrchestrator.sanitize_environment()
        devs = HolodexOrchestrator.check_devices()
        print(f"[CHECK] Processos órfãos detectados: {len(orphans)}")
        for pid, name, pat in orphans:
            print(f"  PID {pid} ({name}) - padrão: {pat}")
        print(f"[CHECK] ESP32 Serial: {'DISPONÍVEL (' + devs['serial_path'] + ')' if devs['serial'] else 'NÃO ENCONTRADA'}")
        print(f"[CHECK] Orbbec Astra USB: {'CONECTADA (2bc5:0403/0501)' if devs['camera_usb'] else 'NÃO ENCONTRADA'}")
        print(f"[CHECK] Dispositivo V4L2: {'DETECTADO (/dev/video0)' if devs['camera_video'] else 'NÃO ENCONTRADO'}")
        sys.exit(0)

    if args.kill_orphans:
        orphans = HolodexOrchestrator.sanitize_environment()
        HolodexOrchestrator.kill_conflicting_processes(orphans)
        print(f"[OK] Eliminados {len(orphans)} processos concorrentes.")
        sys.exit(0)

    orchestrator = HolodexOrchestrator(
        serial_port=args.serial,
        camera_enabled=not args.no_camera,
        esp32_enabled=not args.no_esp32,
        odom_enabled=not args.no_odom,
        supervisor_enabled=not args.no_supervisor,
        nav2_enabled=args.nav2,
        yolo_enabled=args.yolo
    )
    orchestrator.run_interactive(duration=args.duration)


if __name__ == '__main__':
    main()
