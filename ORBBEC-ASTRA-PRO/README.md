# ORBBEC-ASTRA-PRO (ROS 2 Humble / ARM64 & x86)

Driver e bringup em **ROS 2 Humble** para a câmera **Orbbec Astra Pro** (Depth/IR OpenNI `2bc5:0403` + RGB UVC `2bc5:0501`).

Adaptado e testado com sucesso na **Orange Pi 4 Pro** (Allwinner/Rockchip aarch64, Ubuntu 22.04 Jammy LTS).

---

## Recursos e Tópicos Publicados

- **Nuvem de Pontos 3D**: `/camera/depth/points` (`sensor_msgs/msg/PointCloud2`, 640x480)
- **Imagem de Profundidade (Depth)**: `/camera/depth/image_raw` (`16UC1` a ~30 FPS)
- **Imagem Colorida (RGB)**: `/camera/color/image_raw` (`rgb8` via UVC HD Camera)
- **Imagem Infravermelha (IR)**: `/camera/ir/image_raw` (`16UC1` a ~30 FPS)
- **Calibração / Metadados**:
  - `/camera/depth/camera_info`
  - `/camera/color/camera_info`
  - `/camera/ir/camera_info`
- **Transforms TF**:
  - `/tf`, `/tf_static` (frames ópticos e de câmera)

---

## Dispositivos USB da Astra Pro

A Astra Pro combina dois dispositivos USB em uma única porta:
1. `2bc5:0403` - Sensor de Profundidade e Infravermelho (OpenNI2 com driver proprietário ARM64/x64).
2. `2bc5:0501` - Webcam HD RGB UVC padrão (V4L2 em `/dev/video0`).

> **Atenção:** Os IDs de UVC para a libuvc devem ser passados como inteiros decimais (`uvc_vendor_id:=11205`, `uvc_product_id:=1281`), já configurados por padrão nos arquivos de launch deste repositório.

---

## Como Rodar na Orange Pi 4 Pro

### Opção 1: Diretamente no Host (Recomendado na Orange Pi)

O ambiente da Orange Pi já foi configurado e compilado em `~/orb`.

#### 1. Iniciar a Câmera
```bash
cd ~/orb
./scripts/start_camera.sh
```
Ou manualmente:
```bash
source /opt/ros/humble/setup.bash
source ~/orb/orbbec_ws/install/setup.bash
ros2 launch astra_pro_bringup astra_pro.launch.py
```

#### 2. Testar e Validar os Streams
Em outro terminal (ou para verificação rápida):
```bash
cd ~/orb
./scripts/test_camera.sh
```
Esse script valida automaticamente a chegada de frames de Depth, RGB e PointCloud2.

#### 3. Recompilar o Workspace (se fizer alterações)
```bash
cd ~/orb
./scripts/build.sh
```

---

### Opção 2: Com Docker

O repositório inclui `Dockerfile` e `docker-compose.yml` baseados em ROS 2 Humble.

```bash
docker compose up --build
```

---

## Verificando tópicos e chamando serviços

Listar tópicos:
```bash
ros2 topic list
```

Medir taxa de atualização (FPS) da nuvem de pontos:
```bash
ros2 topic hz /camera/depth/points
```

Obter informações de hardware da câmera:
```bash
ros2 service call /camera/get_device_info astra_camera_msgs/srv/GetDeviceInfo "{}"
```

Ligar / Desligar Laser:
```bash
ros2 service call /camera/set_laser_enable std_srvs/srv/SetBool '{data: true}'
```

---

## Estrutura do Workspace

```
ORBBEC-ASTRA-PRO/
├── Dockerfile                  # Imagem ROS 2 Humble
├── docker-compose.yml          # Configuração Docker
├── config/
│   ├── camera_info/
│   └── udev/56-orbbec-usb.rules
├── orbbec_ws/
│   └── src/
│       ├── ros2_astra_camera/  # Driver ROS 2 oficial Orbbec (OpenNI2 + UVC)
│       └── astra_pro_bringup/  # Pacote de inicialização configurado para Astra Pro
└── scripts/
    ├── start_camera.sh         # Inicia driver e publicação de tópicos
    ├── test_camera.sh          # Teste end-to-end de recebimento de frames
    ├── verify_camera.py        # Subscriber Python de validação
    ├── build.sh                # Compilação com colcon
    ├── setup_host.sh           # Instalação de udev rules e dependências
    └── entrypoint.sh           # Entrypoint do Docker
```

---

## Integração com a Pipeline de Odometria (pointcloud_odometry)

A nuvem de pontos gerada em `/camera/depth/points` (640x480, 30 FPS) é consumida diretamente pelo pacote [`pointcloud_odometry`](../pointcloud_odometry):
1. **Decimação 2D**: Reduz os ~307k pontos para ~2.500 pontos via pixel striding direto em `< 0.5 ms`.
2. **Odometria 3D small_gicp**: Executa VGICP Frame-to-Submap com suporte a Nav2 e sem conflito de TF com o EKF das rodas Mecanum.

