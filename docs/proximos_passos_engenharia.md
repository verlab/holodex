# Holodex Orange - Guia de Próximas Etapas e Implementações de Engenharia

Este documento descreve detalhadamente as **4 frentes prioritárias de desenvolvimento e validação** para o robô autônomo omnidirecional **Holodex Orange** (Mecanum 4WD, Orange Pi 4 Pro, Orbbec Astra Pro, ESP32 micro-ROS, small_gicp e Nav2).

---

## Índice

1. [Frente 1: Fusão Sensorial Completa no EKF (robot_localization)](#1-frente-1-fusão-sensorial-completa-no-ekf-robot_localization)
2. [Frente 2: Ativação da Percepção Dinâmica com Filtro Neural por NPU](#2-frente-2-ativação-da-percepção-dinâmica-com-filtro-neural-por-npu)
3. [Frente 3: Script de Teste e Envio de Metas Autônomas do Nav2](#3-frente-3-script-de-teste-e-envio-de-metas-autônomas-do-nav2)
4. [Frente 4: Conexão e Validação em Bancada com Hardware Completo](#4-frente-4-conexão-e-validação-em-bancada-com-hardware-completo)

---

## 1. Frente 1: Fusão Sensorial Completa no EKF (`robot_localization`)

### Contexto e Motivação
Atualmente, o filtro EKF ([`ekf.yaml`](file:///home/alisson/orange/mecanum_wheel_odometry/config/ekf.yaml)) funde apenas:
* **Odometria de Rodas (`/odom/wheel`):** Velocidades planares $v_x, v_y$ e velocidade angular $\omega_z$.
* **Sensor Inercial IMU (`/imu/data`):** Ângulo de guinada (Yaw) e taxa angular $\omega_z$ do filtro Madgwick.

A odometria visual 3D ([`/icp/odom`](file:///home/alisson/orange/pointcloud_odometry/src/icp_odometry_component.cpp)) roda paralelamente a ~30 Hz, mas **ainda não está injetada no EKF**. Em bases Mecanum 4WD, os roletes sofrem deslizamento transversal (*strafe slip*) significativo em manobras laterais e diagonais. Se o robô confiar exclusivamente nas rodas para posição, o referencial `odom -> base_link` (e a navegação do Nav2) acumulará deriva considerável.

### O que Injetar no EKF
O nó `small_gicp` calcula a odometria visual contra um submapa de células gaussianas na RAM e inverte a matriz Hessiana de Gauss-Newton ($H^{-1}$) para fornecer a covariância $6 \times 6$ real de pose e velocidade.

Podemos configurar o EKF para fundir `/icp/odom` como `odom1`:
* **Posição Planar:** $X, Y$ (em metros).
* **Orientação Planar:** $\text{Yaw}$ (em radianos).
* **Velocidades:** $v_x, v_y$ e $\omega_z$.

### Como Implementar

#### 1. Configurar [`mecanum_wheel_odometry/config/ekf.yaml`](file:///home/alisson/orange/mecanum_wheel_odometry/config/ekf.yaml)
Adicionar a entrada `odom1`:

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 1. Odometria das rodas Mecanum
    odom0: /odom/wheel
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    odom0_queue_size: 10
    odom0_differential: false
    odom0_relative: false

    # 2. Sensor inercial filtrado (Madgwick)
    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  false, false, false]
    imu0_differential: false
    imu0_relative: true
    imu0_queue_size: 10

    # 3. Odometria Visual 3D (small_gicp VGICP)
    odom1: /icp/odom
    odom1_config: [true,  true,  false,   # X, Y
                   false, false, true,    # Yaw
                   true,  true,  false,   # vx, vy
                   false, false, true,    # wz
                   false, false, false]
    odom1_queue_size: 10
    odom1_nodelay: true
    odom1_differential: false
    odom1_relative: false
    # Opcional: Rejeição de outliers por distância de Mahalanobis
    odom1_pose_rejection_threshold: 3.0
    odom1_twist_rejection_threshold: 2.0
```

#### 2. Proteção contra Perda de Rastreio (`TRACKING_LOST`)
No arquivo [`icp_odometry_component.cpp`](file:///home/alisson/orange/pointcloud_odometry/src/icp_odometry_component.cpp), quando o solver detectar `tracking_state_ == TrackingState::LOST`, deve-se:
1. Inflar a covariância das posições na mensagem `nav_msgs/msg/Odometry` para valores elevados (ex: $10^3$), fazendo o filtro de Kalman ignorar a medição visual momentaneamente; **OU**
2. Pausar a publicação de `/icp/odom` até que a relocalização bem-sucedida confirme o alinhamento com inliers $> 60\%$.

---

### Status (implementado, validado só em software)
`odom1: /icp/odom` (X, Y, Yaw) está no `ekf.yaml`; o ICP usa `/odom/wheel` como chute; a covariância é inflada em `TRACKING_LOST`; a IMU contribui só com `wz`. Falta validar com hardware e mover o submapa para o referencial do EKF.

---

## 2. Frente 2: Ativação da Percepção Dinâmica com Filtro Neural por NPU

### Contexto e Motivação
Quando pessoas, animais ou objetos móveis passam no campo de visão da câmera, a nuvem de pontos registra geometrias que não pertencem ao ambiente estático. Se esses pontos forem integrados ao `GaussianVoxelMap` do small_gicp:
1. O submapa fica corrompido com "fantasmas" no meio do caminho.
2. A odometria tenta se ancorar na pessoa em movimento, gerando desvio severo de pose.

O pacote [`yolo_seg_ros2`](file:///home/alisson/orange/yolo_seg_ros2/src/yolo_seg_node.cpp) foi adaptado para rodar contínuo na NPU **VeriSilicon VIP9000 (3 TOPS)** a ~25 FPS, publicando a máscara de segmentação das classes dinâmicas (pessoas, veículos e animais) no tópico `/yolo/dynamic_mask`.

### Como Implementar

#### 1. Ativar no Arquivo de Parâmetros [`pointcloud_odometry/config/params.yaml`](file:///home/alisson/orange/pointcloud_odometry/config/params.yaml)
Altere o parâmetro de filtro dinâmico:

```yaml
pointcloud_filter_component:
  ros__parameters:
    input_cloud_topic: "/camera/depth/points"
    output_cloud_topic: "/camera/filtered_points"
    mask_topic: "/yolo/dynamic_mask"
    stride_u: 8
    stride_v: 8
    min_range: 0.4
    max_range: 3.8
    enable_dynamic_filter: true    # <--- Mudar de false para true
    mask_timeout_sec: 0.35         # Descarte seguro se a inferência atrasar
```

#### 2. Funcionamento do Algoritmo em [`pointcloud_filter_component.cpp`](file:///home/alisson/orange/pointcloud_odometry/src/pointcloud_filter_component.cpp)
O nó já possui a rotina de rejeição por pixel implementada:
```cpp
if (enable_dynamic_filter_ && isDynamicPixel(v, u, msg->header.stamp)) {
    continue; // Ponto dinâmico descartado em < 0.05 microsegundos
}
```
Como a nuvem de profundidade da Astra Pro é matricial ($640 \times 480$), o índice $(v, u)$ do ponto é mapeado diretamente na máscara da NPU. Não há custo de projeção geométrica 3D, mantendo o pré-processamento abaixo de $0.20\text{ ms}$.

#### 3. Execução Integrada no Orquestrador
Lançar o sistema com a flag `--yolo`:
```bash
ros2 run holodex_bringup holodex_orchestrator --nav2 --yolo
```

---

## 3. Frente 3: Script de Teste e Envio de Metas Autônomas do Nav2

### Contexto e Motivação
Com a stack de navegação holonômica ([`navigation.launch.py`](file:///home/alisson/orange/holodex_bringup/launch/navigation.launch.py)) e o envelope de segurança ([`motion_supervisor.py`](file:///home/alisson/orange/holodex_bringup/holodex_bringup/motion_supervisor.py)) configurados, é fundamental testar o envio programático de trajetórias para a base Mecanum sem depender de uma estação gráfica externa rodando RViz2.

### Como Implementar

Criar um script de teste autônomo em Python: [`holodex_bringup/scripts/holodex_nav_test.py`](file:///home/alisson/orange/holodex_bringup/scripts).

#### Padrões de Teste Essenciais:
1. **Teste 1 - Avanço e Recuo Longitudinal:**
   * Avançar $+1.0\text{ m}$ em $X$, parar por 2 segundos e retornar para $0.0\text{ m}$.
   * Validação: Confirma suavização das rampas em $v_x$ e frenagem suave.
2. **Teste 2 - Strafe Lateral Puro (Holonômico):**
   * Deslocar $+0.6\text{ m}$ em $Y$ (lateral esquerda) com $\text{Yaw} = 0^\circ$ fixo.
   * Validação: Verifica se o DWB Local Planner comanda $v_y$ puro sem forçar rotação de proa.
3. **Teste 3 - Trajetória em Quadrado (Loop Closure):**
   * $(0, 0) \to (1, 0) \to (1, 1) \to (0, 1) \to (0, 0)$.
   * Validação: Mede o erro de deriva residual acumulado no retorno ao ponto inicial.

#### Exemplo de Estrutura do Script (Nav2 Action Client):
```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class HolodexNavTester(Node):
    def __init__(self):
        super().__init__('holodex_nav_tester')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_relative_goal(self, delta_x, delta_y, delta_yaw_rad):
        self._action_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'odom'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = delta_x
        goal_msg.pose.pose.position.y = delta_y
        goal_msg.pose.pose.orientation.z = math.sin(delta_yaw_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(delta_yaw_rad / 2.0)

        self.get_logger().info(f"Enviando meta: X={delta_x}m, Y={delta_y}m, Yaw={delta_yaw_rad}rad")
        return self._action_client.send_goal_async(goal_msg)
```

---

## 4. Frente 4: Conexão e Validação em Bancada com Hardware Completo

### Procedimento de Conexão Física
1. **Câmera Orbbec Astra Pro:**
   * Conectar o cabo USB na porta USB 3.0 (azul) da Orange Pi 4 Pro.
   * Validar reconhecimento dos dois barramentos internos:
     ```bash
     lsusb | grep -E "2bc5:0403|2bc5:0501"
     ```
     * `2bc5:0403`: Sensor OpenNI IR/Depth.
     * `2bc5:0501`: Sensor UVC RGB HD (`/dev/video0`).

2. **Base Móvel ESP32:**
   * Conectar o cabo Micro-USB / Type-C do DevKit na Orange Pi.
   * A porta serial será exposta em `/dev/ttyUSB0` (ou `/dev/esp32` se configurada regra udev).
   * Confirmar permissões:
     ```bash
     ls -la /dev/ttyUSB*
     ```

### Roteiro de Teste Passo a Passo

#### Passo 1: Checagem Rápida sem Subir Processos
```bash
ros2 run holodex_bringup holodex_orchestrator --check-only
```
* Deve acusar:
  * `ESP32 Serial: DISPONÍVEL (/dev/ttyUSB0)`
  * `Orbbec Astra USB: CONECTADA (2bc5:0403/0501)`
  * `Dispositivo V4L2: DETECTADO (/dev/video0)`

#### Passo 2: Execução Interativa do Robô
```bash
ros2 run holodex_bringup holodex_orchestrator --nav2
```
* O painel TUI exibirá:
  * **Taxa de Ticks das Rodas:** `/wheel/ticks` (~50 Hz)
  * **IMU Cru:** `/imu/data_raw` (~50 Hz)
  * **Odometria Visual:** `/icp/odom` (~30 Hz com status `● TRACKING_OK`)
  * **Submapa 3D:** `/icp/submap_cloud` (~1 Hz quando houver keyframes)
  * **Supervisor:** `/motion_supervisor/status` (~20 Hz)
  * **Temperatura e CPU:** Monitoramento em tempo real nos núcleos big.LITTLE Cortex-A76/A55.

#### Passo 3: Testes de Teclas no Dashboard TUI
* Pressione `[n]` para alternar ativação do Nav2.
* Pressione `[y]` para alternar inferência da NPU VIP9000.
* Pressione `[s]` para testar a exportação do submapa 3D consolidado (`.ply` e `.xyz`).
* Pressione `[e]` para validar a Parada de Emergência imediata.
* Pressione `[q]` para executar o encerramento gracioso e ordenado de todos os nós.
