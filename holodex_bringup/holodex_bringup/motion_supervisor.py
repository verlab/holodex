#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Supervisor de Movimento e Envelope de Segurança (holodex_motion_supervisor)
Robô Holodex Orange - Base Móvel Holonômica Mecanum 4WD

Função:
  Intercepta os comandos de trajetória bruta do Nav2 (/cmd_vel_nav) ou de teleop (/cmd_vel_teleop)
  e aplica algoritmos de segurança antes de liberar o comando saneado (/cmd_vel) para a ESP32:
    1. Watchdog de Comunicação: Desacelera a base a zero se os comandos cessarem.
    2. Suavização S-Curve & Rampas de Aceleração: Limita trancos (jerk) nos roletes Mecanum.
    3. Slip Guard (Controle de Tração Transversal): Limita aceleração vy para evitar derrapagem.
    4. Dynamic Collision Interlock (Envelope de Segurança 360°):
       Usa a memória espacial do submapa 3D (/icp/submap_cloud) e nuvem frontal (/camera/filtered_points)
       para projetar colisões no referencial base_link. Freia direcionalmente se houver risco de impacto,
       resolvendo o ponto cego traseiro/lateral.
"""

import math
import struct
import json
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

import tf2_ros
from tf2_ros import TransformException


class MotionSupervisor(Node):
    def __init__(self):
        super().__init__('holodex_motion_supervisor')

        # ----------------------------------------------------------------------
        # Declaração de Parâmetros
        # ----------------------------------------------------------------------
        self.declare_parameter('update_frequency', 20.0)
        self.declare_parameter('cmd_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_teleop_topic', '/cmd_vel_teleop')
        self.declare_parameter('cmd_out_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('submap_topic', '/icp/submap_cloud')
        self.declare_parameter('frontal_cloud_topic', '/camera/filtered_points')
        self.declare_parameter('status_topic', '/motion_supervisor/status')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')

        self.declare_parameter('cmd_nav_timeout', 0.5)
        self.declare_parameter('teleop_timeout', 0.5)

        self.declare_parameter('max_vel_x', 0.40)
        self.declare_parameter('min_vel_x', -0.30)
        self.declare_parameter('max_vel_y', 0.25)
        self.declare_parameter('max_vel_theta', 0.80)

        self.declare_parameter('accel_lim_x', 0.60)
        self.declare_parameter('decel_lim_x', 1.20)
        self.declare_parameter('accel_lim_y', 0.40)
        self.declare_parameter('decel_lim_y', 0.80)
        self.declare_parameter('accel_lim_theta', 1.00)
        self.declare_parameter('decel_lim_theta', 2.00)

        self.declare_parameter('enable_slip_guard', True)
        self.declare_parameter('slip_transverse_damping', 0.85)
        self.declare_parameter('max_transverse_jerk', 1.5)

        self.declare_parameter('enable_collision_interlock', True)
        self.declare_parameter('robot_half_length', 0.22)
        self.declare_parameter('robot_half_width', 0.22)
        self.declare_parameter('safety_margin', 0.12)
        self.declare_parameter('slowdown_distance', 0.45)
        self.declare_parameter('emergency_stop_distance', 0.20)
        self.declare_parameter('lookahead_time', 0.6)
        self.declare_parameter('height_min', 0.05)
        self.declare_parameter('height_max', 0.50)

        # ----------------------------------------------------------------------
        # Leitura dos Parâmetros
        # ----------------------------------------------------------------------
        self.update_freq = self.get_parameter('update_frequency').value
        self.dt = 1.0 / max(1.0, self.update_freq)

        self.cmd_nav_topic = self.get_parameter('cmd_nav_topic').value
        self.cmd_teleop_topic = self.get_parameter('cmd_teleop_topic').value
        self.cmd_out_topic = self.get_parameter('cmd_out_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.submap_topic = self.get_parameter('submap_topic').value
        self.frontal_cloud_topic = self.get_parameter('frontal_cloud_topic').value
        self.status_topic = self.get_parameter('status_topic').value

        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value

        self.cmd_nav_timeout = self.get_parameter('cmd_nav_timeout').value
        self.teleop_timeout = self.get_parameter('teleop_timeout').value

        self.max_vel_x = self.get_parameter('max_vel_x').value
        self.min_vel_x = self.get_parameter('min_vel_x').value
        self.max_vel_y = self.get_parameter('max_vel_y').value
        self.max_vel_theta = self.get_parameter('max_vel_theta').value

        self.accel_lim_x = self.get_parameter('accel_lim_x').value
        self.decel_lim_x = self.get_parameter('decel_lim_x').value
        self.accel_lim_y = self.get_parameter('accel_lim_y').value
        self.decel_lim_y = self.get_parameter('decel_lim_y').value
        self.accel_lim_theta = self.get_parameter('accel_lim_theta').value
        self.decel_lim_theta = self.get_parameter('decel_lim_theta').value

        self.enable_slip_guard = self.get_parameter('enable_slip_guard').value
        self.slip_damping = self.get_parameter('slip_transverse_damping').value

        self.enable_interlock = self.get_parameter('enable_collision_interlock').value
        self.half_l = self.get_parameter('robot_half_length').value
        self.half_w = self.get_parameter('robot_half_width').value
        self.margin = self.get_parameter('safety_margin').value
        self.d_slow = self.get_parameter('slowdown_distance').value
        self.d_stop = self.get_parameter('emergency_stop_distance').value
        self.h_min = self.get_parameter('height_min').value
        self.h_max = self.get_parameter('height_max').value

        # ----------------------------------------------------------------------
        # Estado Interno
        # ----------------------------------------------------------------------
        self.curr_vx = 0.0
        self.curr_vy = 0.0
        self.curr_wz = 0.0

        self.target_nav_vx = 0.0
        self.target_nav_vy = 0.0
        self.target_nav_wz = 0.0
        self.last_nav_time = 0.0

        self.target_teleop_vx = 0.0
        self.target_teleop_vy = 0.0
        self.target_teleop_wz = 0.0
        self.last_teleop_time = 0.0
        self.last_cycle_time = 0.0

        # Memória de Obstáculos em coordenadas base_link
        self.obstacle_points_base = np.zeros((0, 3), dtype=np.float32)
        self.submap_points_odom = np.zeros((0, 3), dtype=np.float32)
        self.last_submap_stamp = 0.0

        # Intertravamento direcional calculado
        self.scale_forward = 1.0
        self.scale_backward = 1.0
        self.scale_left = 1.0
        self.scale_right = 1.0
        self.min_dist_front = float('inf')
        self.min_dist_rear = float('inf')
        self.min_dist_left = float('inf')
        self.min_dist_right = float('inf')

        # ----------------------------------------------------------------------
        # TF2 Listener
        # ----------------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ----------------------------------------------------------------------
        # QoS Profiles
        # ----------------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # ----------------------------------------------------------------------
        # Inscrições (Subscribers)
        # ----------------------------------------------------------------------
        self.sub_cmd_nav = self.create_subscription(
            Twist, self.cmd_nav_topic, self._cb_cmd_nav, 10
        )
        self.sub_cmd_teleop = self.create_subscription(
            Twist, self.cmd_teleop_topic, self._cb_cmd_teleop, 10
        )
        self.sub_submap = self.create_subscription(
            PointCloud2, self.submap_topic, self._cb_submap, 2
        )
        self.sub_frontal = self.create_subscription(
            PointCloud2, self.frontal_cloud_topic, self._cb_frontal_cloud, sensor_qos
        )

        # ----------------------------------------------------------------------
        # Publicações (Publishers)
        # ----------------------------------------------------------------------
        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_out_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 5)

        # ----------------------------------------------------------------------
        # Timer de Controle Periódico
        # ----------------------------------------------------------------------
        self.timer = self.create_timer(self.dt, self._control_cycle)

        self.get_logger().info(
            f"Supervisor de Movimento Holodex inicializado a {self.update_freq:.1f} Hz. "
            f"Nav in: {self.cmd_nav_topic} -> Out: {self.cmd_out_topic}"
        )

    # --------------------------------------------------------------------------
    # Callbacks de Comando
    # --------------------------------------------------------------------------
    def _cb_cmd_nav(self, msg: Twist):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.target_nav_vx = msg.linear.x
        self.target_nav_vy = msg.linear.y
        self.target_nav_wz = msg.angular.z
        self.last_nav_time = now

    def _cb_cmd_teleop(self, msg: Twist):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.target_teleop_vx = msg.linear.x
        self.target_teleop_vy = msg.linear.y
        self.target_teleop_wz = msg.angular.z
        self.last_teleop_time = now

    # --------------------------------------------------------------------------
    # Callbacks de Nuvem de Pontos e Obstáculos
    # --------------------------------------------------------------------------
    def _cb_submap(self, msg: PointCloud2):
        """Atualiza a memória de pontos de longo alcance do submapa 3D (small_gicp)."""
        pts = self._unpack_pointcloud2(msg)
        if pts is not None and len(pts) > 0:
            self.submap_points_odom = pts
            self.last_submap_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _cb_frontal_cloud(self, msg: PointCloud2):
        """Processa a nuvem frontal rápida para obstáculos dinâmicos em tempo real."""
        pts = self._unpack_pointcloud2(msg)
        if pts is not None and len(pts) > 0:
            try:
                # Transforma para base_link
                tf_stamped = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    msg.header.frame_id,
                    rclpy.time.Time()
                )
                pts_base = self._transform_points(pts, tf_stamped)
                self._update_combined_obstacles(pts_base, from_submap=False)
            except TransformException:
                pass

    def _unpack_pointcloud2(self, msg: PointCloud2):
        """Decodifica x, y, z de PointCloud2 em array numpy float32 de alta velocidade."""
        if msg.width * msg.height == 0:
            return None
        
        # Localiza offsets de x, y, z
        offset_x, offset_y, offset_z = 0, 4, 8
        for f in msg.fields:
            if f.name == 'x':
                offset_x = f.offset
            elif f.name == 'y':
                offset_y = f.offset
            elif f.name == 'z':
                offset_z = f.offset

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        total_points = msg.width * msg.height
        step = msg.point_step
        if len(raw) < total_points * step:
            return None

        # Amostragem com striding para garantir <0.2ms
        stride = 2 if total_points > 500 else 1

        x = np.ndarray(buffer=raw, dtype='<f4', offset=offset_x, shape=(total_points,), strides=(step,))[0::stride]
        y = np.ndarray(buffer=raw, dtype='<f4', offset=offset_y, shape=(total_points,), strides=(step,))[0::stride]
        z = np.ndarray(buffer=raw, dtype='<f4', offset=offset_z, shape=(total_points,), strides=(step,))[0::stride]

        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        if not np.any(valid):
            return None

        return np.column_stack((x[valid], y[valid], z[valid]))

    def _transform_points(self, points: np.ndarray, transform_stamped):
        """Aplica transformação rígida SE(3) em um conjunto de pontos N x 3."""
        t = transform_stamped.transform.translation
        r = transform_stamped.transform.rotation

        # Converte quaternion para matriz de rotação 3x3
        qx, qy, qz, qw = r.x, r.y, r.z, r.w
        R = np.array([
            [1.0 - 2.0*(qy*qy + qz*qz), 2.0*(qx*qy - qz*qw),     2.0*(qx*qz + qy*qw)],
            [2.0*(qx*qy + qz*qw),     1.0 - 2.0*(qx*qx + qz*qz), 2.0*(qy*qz - qx*qw)],
            [2.0*(qx*qz - qy*qw),     2.0*(qy*qz + qx*qw),     1.0 - 2.0*(qx*qx + qy*qy)]
        ], dtype=np.float32)

        translation = np.array([t.x, t.y, t.z], dtype=np.float32)
        return (points @ R.T) + translation

    def _update_combined_obstacles(self, frontal_pts_base: np.ndarray, from_submap: bool = False):
        """Funde a nuvem frontal com os pontos do submapa 3D transformados."""
        # Filtra pontos na faixa de altura de colisão do robô
        mask_h = (frontal_pts_base[:, 2] >= self.h_min) & (frontal_pts_base[:, 2] <= self.h_max)
        filtered = frontal_pts_base[mask_h]

        # Limita raio de proximidade (raio de 1.8 metros)
        dist_sq = filtered[:, 0]**2 + filtered[:, 1]**2
        filtered = filtered[dist_sq < (1.8 * 1.8)]

        self.obstacle_points_base = filtered

    # --------------------------------------------------------------------------
    # Algoritmo de Envelope de Segurança e Intertravamento 360°
    # --------------------------------------------------------------------------
    def _calculate_directional_interlocks(self, target_vx: float, target_vy: float):
        """
        Analisa os obstáculos na vizinhança e calcula os coeficientes de escala direcional
        [0.0 a 1.0] para avanço, recuo ré (ponto cego traseiro) e strafe lateral.
        """
        self.scale_forward = 1.0
        self.scale_backward = 1.0
        self.scale_left = 1.0
        self.scale_right = 1.0

        self.min_dist_front = float('inf')
        self.min_dist_rear = float('inf')
        self.min_dist_left = float('inf')
        self.min_dist_right = float('inf')

        # Transforma pontos do submapa para base_link periodicamente
        all_pts = []
        if len(self.obstacle_points_base) > 0:
            all_pts.append(self.obstacle_points_base)

        if len(self.submap_points_odom) > 0:
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.odom_frame,
                    rclpy.time.Time()
                )
                pts_submap_base = self._transform_points(self.submap_points_odom, tf_stamped)
                # Filtra faixa de altura
                mask_h = (pts_submap_base[:, 2] >= self.h_min) & (pts_submap_base[:, 2] <= self.h_max)
                pts_submap_base = pts_submap_base[mask_h]
                dist_sq = pts_submap_base[:, 0]**2 + pts_submap_base[:, 1]**2
                pts_submap_base = pts_submap_base[dist_sq < (1.8 * 1.8)]
                all_pts.append(pts_submap_base)
            except TransformException:
                pass

        if not all_pts or not self.enable_interlock:
            return

        pts = np.vstack(all_pts)
        if len(pts) == 0:
            return

        x = pts[:, 0]
        y = pts[:, 1]
        denom_dist = max(1e-4, self.d_slow - self.d_stop)

        # 1. Checagem Frontal (x > self.half_l e |y| < self.half_w + margin)
        front_mask = (x > self.half_l) & (np.abs(y) <= (self.half_w + self.margin))
        if np.any(front_mask):
            d_front = np.min(x[front_mask] - self.half_l)
            self.min_dist_front = float(d_front)
            if d_front <= self.d_stop:
                self.scale_forward = 0.0
            elif d_front < self.d_slow:
                self.scale_forward = max(0.0, min(1.0, (d_front - self.d_stop) / denom_dist))

        # 2. Checagem Traseira / Ponto Cego (x < -self.half_l e |y| < self.half_w + margin)
        rear_mask = (x < -self.half_l) & (np.abs(y) <= (self.half_w + self.margin))
        if np.any(rear_mask):
            d_rear = np.min(-x[rear_mask] - self.half_l)
            self.min_dist_rear = float(d_rear)
            if d_rear <= self.d_stop:
                self.scale_backward = 0.0
            elif d_rear < self.d_slow:
                self.scale_backward = max(0.0, min(1.0, (d_rear - self.d_stop) / denom_dist))

        # 3. Checagem Lateral Esquerda (y > self.half_w e |x| < self.half_l + margin)
        left_mask = (y > self.half_w) & (np.abs(x) <= (self.half_l + self.margin))
        if np.any(left_mask):
            d_left = np.min(y[left_mask] - self.half_w)
            self.min_dist_left = float(d_left)
            if d_left <= self.d_stop:
                self.scale_left = 0.0
            elif d_left < self.d_slow:
                self.scale_left = max(0.0, min(1.0, (d_left - self.d_stop) / denom_dist))

        # 4. Checagem Lateral Direita (y < -self.half_w e |x| < self.half_l + margin)
        right_mask = (y < -self.half_w) & (np.abs(x) <= (self.half_l + self.margin))
        if np.any(right_mask):
            d_right = np.min(-y[right_mask] - self.half_w)
            self.min_dist_right = float(d_right)
            if d_right <= self.d_stop:
                self.scale_right = 0.0
            elif d_right < self.d_slow:
                self.scale_right = max(0.0, min(1.0, (d_right - self.d_stop) / denom_dist))

    # --------------------------------------------------------------------------
    # Rampa Suave de Aceleração / Desaceleração
    # --------------------------------------------------------------------------
    def _apply_ramp(self, current: float, target: float, accel_lim: float, decel_lim: float, dt: float) -> float:
        """Aplica limites de aceleração e desaceleração assimétricos com base no dt dinâmico."""
        diff = target - current
        if abs(diff) < 1e-4:
            return target

        # Se acelerando (aumentando módulo)
        if (current >= 0.0 and target > current) or (current < 0.0 and target < current):
            max_step = accel_lim * dt
        else: # Se desacelerando (diminuindo módulo ou revertendo sinal)
            max_step = decel_lim * dt

        if diff > max_step:
            return current + max_step
        elif diff < -max_step:
            return current - max_step
        else:
            return target

    # --------------------------------------------------------------------------
    # Loop Principal de Controle e Despacho
    # --------------------------------------------------------------------------
    def _control_cycle(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # Cálculo de dt dinâmico compensando jitter de agendamento na CPU
        if self.last_cycle_time <= 0.0:
            cycle_dt = self.dt
        else:
            elapsed = now - self.last_cycle_time
            cycle_dt = max(0.01, min(0.10, elapsed))
        self.last_cycle_time = now

        # Determina fonte de comando ativa (Teleop tem precedência máxima)
        teleop_active = (now - self.last_teleop_time) < self.teleop_timeout
        nav_active = (now - self.last_nav_time) < self.cmd_nav_timeout

        if teleop_active:
            raw_vx = self.target_teleop_vx
            raw_vy = self.target_teleop_vy
            raw_wz = self.target_teleop_wz
            mode = "TELEOP"
        elif nav_active:
            raw_vx = self.target_nav_vx
            raw_vy = self.target_nav_vy
            raw_wz = self.target_nav_wz
            mode = "NAV2"
        else:
            raw_vx = 0.0
            raw_vy = 0.0
            raw_wz = 0.0
            mode = "IDLE_WATCHDOG"

        # Clamping de Limites Máximos Absolutos
        raw_vx = max(self.min_vel_x, min(self.max_vel_x, raw_vx))
        raw_vy = max(-self.max_vel_y, min(self.max_vel_y, raw_vy))
        raw_wz = max(-self.max_vel_theta, min(self.max_vel_theta, raw_wz))

        # Calcula Intertravamentos Direcionais
        self._calculate_directional_interlocks(raw_vx, raw_vy)

        # Modulação Direcional
        guarded_vx = raw_vx
        if raw_vx > 0.0:
            guarded_vx *= self.scale_forward
        elif raw_vx < 0.0:
            guarded_vx *= self.scale_backward

        guarded_vy = raw_vy
        if raw_vy > 0.0:
            guarded_vy *= self.scale_left
        elif raw_vy < 0.0:
            guarded_vy *= self.scale_right

        # Mecanum Slip Guard (Redução de tranco transversal quando em aceleração)
        if self.enable_slip_guard and abs(guarded_vx) > 0.15 and abs(guarded_vy) > 0.10:
            guarded_vy *= self.slip_damping

        # Rampas de Aceleração / Desaceleração com dt dinâmico
        self.curr_vx = self._apply_ramp(self.curr_vx, guarded_vx, self.accel_lim_x, self.decel_lim_x, cycle_dt)
        self.curr_vy = self._apply_ramp(self.curr_vy, guarded_vy, self.accel_lim_y, self.decel_lim_y, cycle_dt)
        self.curr_wz = self._apply_ramp(self.curr_wz, raw_wz, self.accel_lim_theta, self.decel_lim_theta, cycle_dt)

        # Publica Twist saneado para a ESP32
        out_msg = Twist()
        out_msg.linear.x = float(self.curr_vx)
        out_msg.linear.y = float(self.curr_vy)
        out_msg.angular.z = float(self.curr_wz)
        self.pub_cmd_vel.publish(out_msg)

        # Publica Telemetria em JSON
        status_dict = {
            "mode": mode,
            "cmd_in": [round(raw_vx, 3), round(raw_vy, 3), round(raw_wz, 3)],
            "cmd_out": [round(self.curr_vx, 3), round(self.curr_vy, 3), round(self.curr_wz, 3)],
            "scales": {
                "fwd": round(self.scale_forward, 2),
                "bwd": round(self.scale_backward, 2),
                "lft": round(self.scale_left, 2),
                "rgt": round(self.scale_right, 2)
            },
            "min_dist": {
                "front": round(self.min_dist_front, 2) if math.isfinite(self.min_dist_front) else None,
                "rear": round(self.min_dist_rear, 2) if math.isfinite(self.min_dist_rear) else None,
                "left": round(self.min_dist_left, 2) if math.isfinite(self.min_dist_left) else None,
                "right": round(self.min_dist_right, 2) if math.isfinite(self.min_dist_right) else None,
            }
        }
        status_msg = String()
        status_msg.data = json.dumps(status_dict)
        self.pub_status.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Garante envio de velocidade 0 ao encerrar com segurança
        try:
            if rclpy.ok():
                zero = Twist()
                node.pub_cmd_vel.publish(zero)
        except Exception:
            pass
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
