#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KEEP_CAMERA=false
MODEL_PATH="/home/orangepi/yolov8n_seg_final.nb"
OUTPUT_IMG="/home/orangepi/resultado_yolov8_seg.jpg"
TOPIC="/camera/color/image_raw"

while [[ $# -gt 0 ]]; do
  case $1 in
    --keep-camera)
      KEEP_CAMERA=true
      shift
      ;;
    --model)
      MODEL_PATH="$2"
      shift 2
      ;;
    --output)
      OUTPUT_IMG="$2"
      shift 2
      ;;
    --topic)
      TOPIC="$2"
      shift 2
      ;;
    *)
      shift
      ;;
  esac
done

echo "=========================================================="
echo "    YOLOv8-seg NPU (Allwinner A733 / VIPLite) no ROS 2    "
echo "=========================================================="

# 1. Configurar ambiente ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[ERRO] /opt/ros/humble/setup.bash nao encontrado!"
    exit 1
fi

if [ -f "/home/orangepi/orb/orbbec_ws/install/setup.bash" ]; then
    source /home/orangepi/orb/orbbec_ws/install/setup.bash
fi

# 2. Localizar o executavel compilado
BIN=""
if [ -f "${DIR}/build/yolo_seg_node" ]; then
    BIN="${DIR}/build/yolo_seg_node"
elif [ -f "${DIR}/install/yolo_seg_ros2/lib/yolo_seg_ros2/yolo_seg_node" ]; then
    BIN="${DIR}/install/yolo_seg_ros2/lib/yolo_seg_ros2/yolo_seg_node"
elif which yolo_seg_node >/dev/null 2>&1; then
    BIN="$(which yolo_seg_node)"
elif [ -f "/home/orangepi/yolo_seg_ros2/build/yolo_seg_node" ]; then
    BIN="/home/orangepi/yolo_seg_ros2/build/yolo_seg_node"
else
    echo "[ERRO] Executavel yolo_seg_node nao encontrado. Compile antes de executar!"
    exit 1
fi

# 3. Verificar se o topico da camera ja esta ativo
CAM_PID=""
if ! ros2 topic list 2>/dev/null | grep -q "${TOPIC}"; then
    echo "[INFO] Topico ${TOPIC} nao detectado. Inicializando driver da camera Orbbec Astra Pro..."
    ros2 run astra_camera clean_shm_node >/dev/null 2>&1 || true
    ros2 launch astra_pro_bringup astra_pro.launch.py > /tmp/astra_camera_infer.log 2>&1 &
    CAM_PID=$!
    echo "[INFO] Processo da camera iniciado em background (PID: ${CAM_PID}). Aguardando publicacao do topico..."

    # Aguardar publicacao do topico (ate 20 segundos)
    READY=false
    for i in {1..20}; do
        if ros2 topic list 2>/dev/null | grep -q "${TOPIC}"; then
            echo "[INFO] Topico ${TOPIC} detectado!"
            READY=true
            break
        fi
        sleep 1
    done

    if [ "$READY" = false ]; then
        echo "[ERRO] Timeout aguardando topico da camera."
        if [ -n "$CAM_PID" ]; then kill $CAM_PID 2>/dev/null || true; fi
        exit 1
    fi
    # Pequeno tempo adicional para estabilizar o pipeline UVC
    sleep 2
else
    echo "[INFO] Topico ${TOPIC} ja esta ativo."
fi

# 4. Executar inferencia
echo "[INFO] Executando inferencia com o binario: ${BIN}"
"${BIN}" --model "${MODEL_PATH}" --topic "${TOPIC}" --output "${OUTPUT_IMG}"

# 5. Limpeza de processo se a camera foi aberta por este script
if [ -n "$CAM_PID" ]; then
    if [ "$KEEP_CAMERA" = true ]; then
        echo "[INFO] Camera mantida em execucao (PID: ${CAM_PID})."
    else
        echo "[INFO] Finalizando processo da camera (PID: ${CAM_PID})..."
        kill $CAM_PID 2>/dev/null || true
        wait $CAM_PID 2>/dev/null || true
    fi
fi

if [ -f "${OUTPUT_IMG}" ]; then
    echo "[SUCESSO] Imagem gerada com sucesso em: ${OUTPUT_IMG}"
    ls -lh "${OUTPUT_IMG}"
else
    echo "[AVISO] Arquivo de resultado nao encontrado."
fi
