# ESP32 Firmware com micro-ROS (ESP-IDF)

Este módulo implementa o nó micro-ROS em C puro utilizando o framework **ESP-IDF** (versão 5.1/5.2) e a biblioteca oficial `micro_ros_espidf_component` para **ROS 2 Humble**.

O firmware foi desenhado especificamente para a base móvel do robô **Holodex Orange**, comunicando-se com a **Orange Pi 4 Pro** através de USB Serial (UART0 a 921,600 bps), pronto para futura transição direta para o **ESP32-P4**.

---

## 1. Arquitetura e Tópicos ROS 2

```
 +----------------------------------------+
 |          Orange Pi 4 Pro               |
 |                                        |
 |   +--------------------------------+   |
 |   |       micro_ros_agent          |   |
 |   +---------------+----------------+   |
 +-------------------|--------------------+
                     | USB Serial (/dev/ttyUSB0 ou /dev/esp32)
                     | 921,600 bps
 +-------------------|--------------------+
 |   +---------------+----------------+   |
 |   |    micro-ROS Client (ESP-IDF)  |   |
 |   |        /esp32_base_node        |   |
 |   +--------------------------------+   |
 |                                        |
 |          ESP32 DevKit v1               |
 +----------------------------------------+
```

### Tópicos Publicados e Assinados
* **Publicador:** `/esp32/heartbeat` (`std_msgs/msg/Int32`)
  * Incrementa e publica a cada 1 segundo (1 Hz) como contador heartbeat de integridade.
* **Assinante:** `/esp32/led` (`std_msgs/msg/Bool`)
  * Permite ligar (`true`) ou desligar (`false`) o LED de status onboard no GPIO 2 via ROS 2.

### Máquina de Estados e Reconexão
* O nó no ESP32 gerencia a reconexão automática:
  * **Piscando rápido (100ms ON / 400ms OFF):** Aguardando conexão com o `micro_ros_agent`.
  * **Duplo pulso rápido:** Conexão estabelecida com sucesso com o Agent.
  * **Pulso breve a cada segundo:** Heartbeat ativo.
  * Se o cabo for desconectado ou o agent reiniciar, o ESP32 destrói os contextos DDS com segurança e retorna para o estado de busca sem travar a CPU.

---

## 2. Estrutura do Projeto

```text
firmware/esp32_firmware/
├── CMakeLists.txt                 # CMake principal do ESP-IDF
├── sdkconfig.defaults             # Configurações do FreeRTOS e supressão de console
├── main/
│   ├── CMakeLists.txt
│   ├── main.c                    # Nó ROS 2 (Heartbeat + LED + Auto-reconexão)
│   ├── esp32_serial_transport.c  # Driver de transporte serial UART0 (921600 bps)
│   └── esp32_serial_transport.h
├── launch/
│   └── micro_ros_agent.launch.py # Launch file ROS 2 para a Orange Pi
├── scripts/
│   ├── 99-esp32.rules            # Regra udev para nome persistente /dev/esp32
│   ├── setup_esp_idf.sh          # Instalação automatizada do ESP-IDF v5.2 no ARM64
│   ├── build_micro_ros_agent.sh  # Compilação do micro_ros_agent no ~/ros2_ws
│   └── flash_esp32.sh            # Script para compilar e gravar o ESP32
└── README.md                     # Esta documentação
```

---

## 3. Passo a Passo de Instalação e Uso na Orange Pi 4 Pro

### Passo 1: Configurar a Regra Udev (Opcional, mas Recomendado)
Garante que a porta serial do ESP32 seja sempre reconhecida como `/dev/esp32` com permissões automáticas:
```bash
sudo cp scripts/99-esp32.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Passo 2: Instalar o ESP-IDF v5.2
Execute o script de automação:
```bash
chmod +x scripts/*.sh
./scripts/setup_esp_idf.sh
```

### Passo 3: Compilar o `micro_ros_agent` no ROS 2 Humble
Execute o script de build:
```bash
./scripts/build_micro_ros_agent.sh
```

### Passo 4: Compilar e Gravar o ESP32
```bash
./scripts/flash_esp32.sh
```

---

## 4. Como Executar e Testar a Comunicação

### Terminal 1: Iniciar o micro-ROS Agent
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch launch/micro_ros_agent.launch.py serial_port:=/dev/esp32 baudrate:=921600
```
*(Se você não configurou a regra udev, use `serial_port:=/dev/ttyUSB0`)*.

Assim que o Agent iniciar, ele negociará a sessão XRCE-DDS com o ESP32 e exibirá:
```text
[micro_ros_agent]: [publisher] Initialized
[micro_ros_agent]: [subscription] Initialized
```

### Terminal 2: Testar a Leitura do Heartbeat
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /esp32/heartbeat
```
Você verá o contador sendo incrementado em tempo real:
```yaml
data: 0
---
data: 1
---
data: 2
```

### Terminal 3: Testar o Acionamento do LED Onboard (GPIO 2)
Para ligar o LED:
```bash
ros2 topic pub --once /esp32/led std_msgs/msg/Bool "{data: true}"
```
Para desligar o LED:
```bash
ros2 topic pub --once /esp32/led std_msgs/msg/Bool "{data: false}"
```

---

## 5. Armadilhas Críticas de Engenharia Evitadas

1. **Corrupção de Pacotes Serial por Log do ESP-IDF:**
   * O ESP-IDF por padrão envia mensagens de boot e logs `ESP_LOGI` pela UART0. Se logs em texto forem impressos enquanto o micro-ROS opera na mesma UART0, o `micro_ros_agent` rejeita os quadros binários.
   * **Solução aplicada:** O `sdkconfig.defaults` desativa o console UART (`CONFIG_ESP_CONSOLE_NONE=y`), mantendo a linha serial 100% limpa.
2. **Conflito de Variáveis entre ROS 2 e ESP-IDF:**
   * Nunca faça `source /opt/ros/humble/setup.bash` e `source ~/esp/esp-idf/export.sh` no mesmo terminal para compilar o firmware. As variáveis de ambiente do Python e Cmake colidem. O script `flash_esp32.sh` isola a compilação do ESP-IDF.
3. **Concorrência de Acesso à Porta Serial:**
   * Não é possível gravar o ESP32 via `idf.py flash` se o `micro_ros_agent` estiver rodando e segurando a porta `/dev/ttyUSB0`. O script `flash_esp32.sh` detecta e encerra instâncias ativas do agente antes do flash.
