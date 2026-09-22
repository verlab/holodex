#!/usr/bin/env bash
set -e

echo "=== [1/4] Instalando dependências de sistema do ESP-IDF no Ubuntu/ARM64 ==="
echo orangepi | sudo -S apt-get update
echo orangepi | sudo -S apt-get install -y \
    git wget flex bison gperf python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

mkdir -p ~/esp
cd ~/esp

echo "=== [2/4] Clonando ESP-IDF v5.2 (compatível com ESP32 WROOM e ESP32-P4) ==="
if [ ! -d "esp-idf" ]; then
    git clone --depth 1 -b release/v5.2 https://github.com/espressif/esp-idf.git
else
    echo "ESP-IDF já clonado em ~/esp/esp-idf. Prosseguindo..."
fi

echo "=== [3/4] Instalando ferramentas para ESP32 e ESP32-P4 ==="
cd ~/esp/esp-idf
./install.sh esp32,esp32p4

echo "=== [4/4] Instalando pacotes Python necessários para compilação do micro-ROS ==="
source ./export.sh
pip install catkin_pkg colcon-common-extensions lark pyyaml "empy==3.3.4"

echo "=== Instalação do ESP-IDF concluída com sucesso! ==="
echo "Para carregar o ambiente em um novo terminal, execute:"
echo "source ~/esp/esp-idf/export.sh"
