#!/usr/bin/env python3
"""
Holodex Orange - Automated System Benchmark & Thermal Profiler.
Measures real CPU loads, big.LITTLE core distribution (Cortex-A76 vs A55),
temperatures, memory usage, and topic frequencies under load.
Outputs structured CSV and Markdown reports.
"""

import os
import sys
import time
import csv
import signal
from pathlib import Path
from collections import deque

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2, Imu
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Int32MultiArray
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False


class FreqTracker:
    def __init__(self, window_size=30):
        self.window = deque(maxlen=window_size)
        self.last_time = None

    def hit(self):
        now = time.time()
        if self.last_time is not None:
            dt = now - self.last_time
            if dt > 0:
                self.window.append(dt)
        self.last_time = now

    def get_hz(self):
        if len(self.window) < 2 or (time.time() - (self.last_time or 0)) > 2.0:
            return 0.0
        avg_dt = sum(self.window) / len(self.window)
        return 1.0 / avg_dt if avg_dt > 0 else 0.0


class BenchmarkRosNode(Node if HAS_RCLPY else object):
    def __init__(self):
        if not HAS_RCLPY:
            raise RuntimeError("rclpy indisponível")
        super().__init__('holodex_benchmark_node')

        self.trackers = {
            "raw_cloud": FreqTracker(),
            "filtered_cloud": FreqTracker(),
            "wheel_ticks": FreqTracker(),
            "imu_raw": FreqTracker(),
            "ekf_odom": FreqTracker(),
            "icp_odom": FreqTracker(),
        }

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(PointCloud2, '/camera/depth/points', lambda m: self.trackers["raw_cloud"].hit(), best_effort_qos)
        self.create_subscription(PointCloud2, '/camera/filtered_points', lambda m: self.trackers["filtered_cloud"].hit(), best_effort_qos)
        self.create_subscription(Int32MultiArray, '/wheel/ticks', lambda m: self.trackers["wheel_ticks"].hit(), best_effort_qos)
        self.create_subscription(Imu, '/imu/data_raw', lambda m: self.trackers["imu_raw"].hit(), best_effort_qos)
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: self.trackers["ekf_odom"].hit(), 10)
        self.create_subscription(Odometry, '/icp/odom', lambda m: self.trackers["icp_odom"].hit(), 10)

    def get_rates(self):
        return {k: v.get_hz() for k, v in self.trackers.items()}


class SystemProfiler:
    def __init__(self, out_dir="."):
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.records = []
        self.tracked_processes = [
            "icp_odometry", "astra_camera", "micro_ros_agent",
            "ekf_node", "mecanum_wheel_odometry"
        ]

    def get_cpu_temp(self):
        for zone in ["/sys/class/thermal/thermal_zone0/temp", "/sys/class/thermal/thermal_zone1/temp"]:
            if os.path.exists(zone):
                try:
                    with open(zone, "r") as f:
                        return float(f.read().strip()) / 1000.0
                except Exception:
                    pass
        return 0.0

    def sample_processes(self):
        proc_stats = {name: 0.0 for name in self.tracked_processes}
        if not HAS_PSUTIL:
            return proc_stats

        for p in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = " ".join(p.info['cmdline'] or [])
                for name in self.tracked_processes:
                    if name in cmdline:
                        proc_stats[name] += p.cpu_percent(interval=None)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        return proc_stats

    def sample(self, timestamp, rates=None):
        temp = self.get_cpu_temp()
        rates = rates or {}

        cpu_total = 0.0
        cpus_per = []
        ram_used_mb = 0.0

        if HAS_PSUTIL:
            cpus_per = psutil.cpu_percent(percpu=True)
            cpu_total = psutil.cpu_percent()
            mem = psutil.virtual_memory()
            ram_used_mb = (mem.total - mem.available) / (1024 * 1024)

        little_avg = sum(cpus_per[0:6]) / 6.0 if len(cpus_per) >= 6 else (sum(cpus_per) / len(cpus_per) if cpus_per else 0.0)
        big_avg = sum(cpus_per[6:8]) / 2.0 if len(cpus_per) >= 8 else 0.0

        proc_stats = self.sample_processes()

        entry = {
            "timestamp": timestamp,
            "temp_c": temp,
            "cpu_total_pct": cpu_total,
            "cpu_little_a55_pct": little_avg,
            "cpu_big_a76_pct": big_avg,
            "ram_used_mb": ram_used_mb,
            "hz_raw_cloud": rates.get("raw_cloud", 0.0),
            "hz_filt_cloud": rates.get("filtered_cloud", 0.0),
            "hz_wheel_ticks": rates.get("wheel_ticks", 0.0),
            "hz_ekf_odom": rates.get("ekf_odom", 0.0),
            "hz_icp_odom": rates.get("icp_odom", 0.0),
            **{f"cpu_{k}": v for k, v in proc_stats.items()}
        }
        self.records.append(entry)
        return entry

    def generate_reports(self, label="benchmark"):
        if not self.records:
            return None, None

        ts_str = time.strftime("%Y%m%d_%H%M%S")
        csv_file = self.out_dir / f"{label}_{ts_str}.csv"
        md_file = self.out_dir / f"{label}_{ts_str}.md"

        # 1. Salva CSV
        fieldnames = list(self.records[0].keys())
        with open(csv_file, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.records)

        # 2. Estatísticas Agregadas
        n = len(self.records)
        temps = [r["temp_c"] for r in self.records]
        cpu_totals = [r["cpu_total_pct"] for r in self.records]
        cpu_bigs = [r["cpu_big_a76_pct"] for r in self.records]
        cpu_littles = [r["cpu_little_a55_pct"] for r in self.records]
        rams = [r["ram_used_mb"] for r in self.records]

        md_content = f"""# Relatório de Benchmark de Desempenho e Térmico
**Data:** {time.strftime('%Y-%m-%d %H:%M:%S')}  
**Amostras coletadas:** {n} segundos  
**Arquivo de dados brutos:** `{csv_file.name}`

---

## 1. Resumo Executivo

| Métrica | Mínimo | Média | Máximo (Pico) |
| :--- | :---: | :---: | :---: |
| **Temperatura da CPU (°C)** | {min(temps):.1f} °C | {sum(temps)/n:.1f} °C | {max(temps):.1f} °C |
| **CPU Global Total (%)** | {min(cpu_totals):.1f}% | {sum(cpu_totals)/n:.1f}% | {max(cpu_totals):.1f}% |
| **Núcleos Big (2x Cortex-A76)** | {min(cpu_bigs):.1f}% | {sum(cpu_bigs)/n:.1f}% | {max(cpu_bigs):.1f}% |
| **Núcleos Little (6x Cortex-A55)** | {min(cpu_littles):.1f}% | {sum(cpu_littles)/n:.1f}% | {max(cpu_littles)/n:.1f}% |
| **Memória RAM Utilizada (MB)** | {min(rams):.0f} MB | {sum(rams)/n:.0f} MB | {max(rams):.0f} MB |

---

## 2. Diagnóstico de Headroom Térmico e Computacional
* **Margem Térmica:** O pico atingido foi de **{max(temps):.1f} °C** (limite de throttling: 85 °C).
* **Balanço big.LITTLE:** Os núcleos Cortex-A76 operaram com média de **{sum(cpu_bigs)/n:.1f}%**, enquanto os Cortex-A55 operaram com média de **{sum(cpu_littles)/n:.1f}%**.
"""
        with open(md_file, "w") as f:
            f.write(md_content)

        return csv_file, md_file


def main():
    import argparse
    import threading

    parser = argparse.ArgumentParser(description="Holodex Orange - System Benchmark")
    parser.add_argument('--duration', type=int, default=30, help='Duração do teste em segundos (default: 30)')
    parser.add_argument('--output-dir', default='/home/orangepi/benchmarks', help='Diretório de saída')
    parser.add_argument('--label', default='baseline', help='Rótulo do teste (ex: baseline, with_nav2)')
    args = parser.parse_args()

    profiler = SystemProfiler(out_dir=args.output_dir)
    node = None
    spin_thread = None

    if HAS_RCLPY:
        try:
            rclpy.init()
            node = BenchmarkRosNode()
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(node)
            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()
        except Exception as e:
            print(f"[WARN] Não foi possível iniciar nó ROS 2: {e}")

    print(f"\n=======================================================")
    print(f"      HOLODEX BENCHMARK: {args.label.upper()} ({args.duration}s)")
    print(f"=======================================================")
    print(" Segundo | Temp (°C) | CPU Total | A76 (Big) | A55 (Little)")
    print("-------------------------------------------------------")

    start_time = time.time()
    try:
        for sec in range(1, args.duration + 1):
            time.sleep(1.0)
            rates = node.get_rates() if node else {}
            s = profiler.sample(sec, rates)
            print(f"   {sec:02d}s   |  {s['temp_c']:5.1f} °C |   {s['cpu_total_pct']:4.1f}%   |   {s['cpu_big_a76_pct']:4.1f}%   |   {s['cpu_little_a55_pct']:4.1f}%")

    except KeyboardInterrupt:
        print("\n[INFO] Benchmark interrompido pelo usuário.")
    finally:
        csv_path, md_path = profiler.generate_reports(label=args.label)
        if csv_path:
            print(f"\n[SUCESSO] Relatórios gerados com sucesso:")
            print(f"  • Dados brutos: {csv_path}")
            print(f"  • Relatório:    {md_path}")
        if node and HAS_RCLPY:
            try:
                executor.shutdown()
                if spin_thread:
                    spin_thread.join(timeout=1.0)
                node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
