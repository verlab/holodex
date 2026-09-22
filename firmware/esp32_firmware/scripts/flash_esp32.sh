#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Verificar se micro_ros_agent está segurando a porta serial
if pgrep -f 'micro_ros_agent' >/dev/null; then
    echo "[AVISO] micro_ros_agent está em execução! Interrompendo temporariamente para liberar a porta serial..."
    pkill -f 'micro_ros_agent' || true
    sleep 1
fi

# Checar ambiente ESP-IDF
if [ -z "$IDF_PATH" ]; then
    if [ -f "$HOME/esp/esp-idf/export.sh" ]; then
        echo "Carregando ambiente ESP-IDF de ~/esp/esp-idf/export.sh..."
        source "$HOME/esp/esp-idf/export.sh"
    else
        echo "[ERRO] ESP-IDF não encontrado em ~/esp/esp-idf. Execute setup_esp_idf.sh primeiro!"
        exit 1
    fi
fi

# Garantir que micro_ros_espidf_component está em components/
mkdir -p "${DIR}/components"
if [ ! -d "${DIR}/components/micro_ros_espidf_component" ]; then
    echo "Clonando micro_ros_espidf_component (branch humble)..."
    git clone --depth 1 -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git "${DIR}/components/micro_ros_espidf_component"
fi

cd "${DIR}"

# Detectar porta serial
PORT="/dev/esp32"
if [ ! -e "$PORT" ]; then
    PORT="/dev/ttyUSB0"
fi

if [ ! -e "$PORT" ]; then
    echo "[ERRO] Nenhuma porta serial ESP32 detectada (/dev/esp32 ou /dev/ttyUSB0)!"
    exit 1
fi

echo "=== Configurando e Compilando Firmware ESP32 ==="
if [ ! -d "build" ]; then
    idf.py set-target esp32
fi

idf.py build

echo "=== Gravando Firmware no ESP32 na porta ${PORT} ==="
idf.py -p "${PORT}" -b 921600 flash

echo "=== Gravação concluída com sucesso! ==="
echo "Você já pode iniciar o micro-ROS Agent com:"
echo "ros2 launch micro_ros_agent.launch.py serial_port:=${PORT}"
