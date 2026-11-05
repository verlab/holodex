# GEMINI.md: Guia de Engenharia, Diagnóstico e Continuidade do Projeto

> **Atenção para Agentes de IA (Gemini, Claude, Antigravity) e Engenheiros de Robótica:**
> Este documento é a fonte canônica da verdade sobre a arquitetura, o histórico de depuração, armadilhas técnicas (*pitfalls*) e diretrizes de desenvolvimento do projeto **Holodex Orange**. 
> **Leia atentamente antes de propor ou aplicar qualquer modificação no código ou na placa.**

---

## 1. Visão Geral e Contexto do Projeto

* **Objetivo Geral:** Prover odometria 3D em tempo real por nuvem de pontos (~30 Hz) e segmentação semântica acelerada por hardware para uma base móvel omnidirecional **Mecanum 4WD**, operando de forma autônoma com **ROS 2 Humble** e **Nav2**.
* **Restrições de SWaP (Size, Weight, and Power):**
  * Toda a computação deve ocorrer localmente em uma **Single Board Computer (SBC)** de baixo custo e baixo consumo: **Orange Pi 4 Pro** (SoC Allwinner A733, 8 núcleos ARM Cortex-A55/A76, NPU VeriSilicon VIP9000 de 3 TOPS, 4GB/8GB RAM).
  * Sem GPU dedicada. Todo cálculo de visão e registro 3D depende exclusivamente de vetorização SIMD/NEON ARM e OpenMP na CPU, além da NPU embutida para redes neurais.
* **Sensor Primário:** Câmera RGB-D **Orbbec Astra Pro** (resolução de profundidade $640 \times 480$ a 30 FPS via luz estruturada infravermelha, combinada com câmera RGB UVC HD).
* **Repositório Git:** Localizado em `/home/alisson/orange` (branch `main`).
* **Ambiente na Orange Pi:**
  * Usuário padrão: `orangepi`
  * IP local típico: `192.168.15.5`
  * Workspaces:
    * `~/ros2_ws`: Contém os pacotes de odometria (`pointcloud_odometry`).
    * `~/orb`: Contém o driver da câmera e scripts de inicialização (`orbbec_ws`).
    * `~/yolo_seg_ros2`: Contém o nó de inferência NPU em C++ / VIPLite.

---

## 2. Arquitetura dos Módulos e Detalhes de Implementação

### 2.1. Pacote `pointcloud_odometry`

O pacote foi desenhado para contornar o maior gargalo de odometria 3D em ARM embarcado: **o custo de pré-processamento de nuvens densas**.

#### A. Componente de Filtro (`PointcloudFilterComponent`)
* **Decimação 2D por Pixel Striding (< 0.20 ms):**
  * Em vez de aplicar filtros 3D caros como `pcl::VoxelGrid` (que levam de 15 a 25 ms para calcular hashes espaciais sobre 307.200 pontos), aproveita a organização matricial da nuvem ($640 \times 480$).
  * Pula pixels nos eixos $u$ e $v$ usando passos configuráveis (`stride_u: 8`, `stride_v: 8`), reduzindo a nuvem para ~2.000 a 2.500 pontos em **0.16 ms**.
* **Pass-Through de Alcance:**
  * Filtra pontos fora de $0.4\text{ m} \le Z \le 3.8\text{ m}$. O limite superior foi fixado em **3.8 m** (não reduza para 2.5 m, pois o robô precisa enxergar paredes distantes para não sofrer deriva angular em ambientes abertos).
* **Filtro de Objetos Dinâmicos (`enable_dynamic_filter`):**
  * Assina `/yolo/dynamic_mask` (`sensor_msgs/msg/Image`, `mono8`).
  * Mapeia os índices $(u, v)$ da nuvem de pontos diretamente na máscara binária e descarta qualquer ponto associado a humanos ou obstáculos móveis detectados pela NPU.

#### B. Componente de Odometria (`IcpOdometryComponent`)
* **Algoritmo Base:** Utiliza o [small_gicp](https://github.com/koide3/small_gicp) configurado no modo **VGICP (Voxelized Generalized ICP)** com otimizador Levenberg-Marquardt.
* **Submapa na RAM (`GaussianVoxelMap`):**
  * Mantém um submapa local de células gaussianas diretamente na memória RAM.
  * O alinhamento de cada novo frame é feito **Frame-to-Submap** (e não Frame-to-Frame), eliminando deriva severa.
* **Validação Cinemática Pré-Keyframe:**
  * Antes de qualquer inserção no submapa ou atualização de pose, o nó valida se a translação e rotação são fisicamente plausíveis para um intervalo de ~33 ms ($\Delta d \le 0.12\text{ m}$, $\Delta \theta \le 15^\circ$).
* **Geração de Keyframes:**
  * Um frame aceito vira Keyframe e injeta pontos no submapa se houver deslocamento físico: $\Delta d > 0.15\text{ m}$ ou $\Delta \theta > 8.0^\circ$.
* **Covariância $6 \times 6$ Real:**
  * Inverte a matriz Hessiana de Gauss-Newton calculada pelo solver ($H^{-1}$) e preenche as matrizes de covariância de pose e velocidade na mensagem `nav_msgs/msg/Odometry`.
* **Zero-Collision TF (`publish_tf: false`):**
  * O robô possui um filtro EKF (`robot_localization`) fundindo encoders das rodas Mecanum e IMU. Para evitar conflito de publicadores no TF tree, a odometria visual **NUNCA** deve publicar a transformação `odom -> base_link` diretamente.
* **Chute Inicial (Initial Guess):**
  * Assina `/odom/wheel` (odometria pura das rodas Mecanum) como chute inicial. **Nunca** `/odometry/filtered`: o EKF funde `/icp/odom`, e isso criaria realimentação (dupla contagem).
  * Calcula o incremento de movimento relativo $\Delta T_{\text{wheel}}$ entre frames e alimenta o solver do ICP, garantindo convergência mesmo durante rotações bruscas.

---

### 2.2. Pacote `yolo_seg_ros2`

* **Hardware Target:** NPU VeriSilicon VIP9000 (3 TOPS) integrada ao SoC Allwinner A733.
* **Modelo Compilado:** [`Models/yolov8n_seg_final.nb`](file:///home/alisson/orange/Models/yolov8n_seg_final.nb) (rede YOLOv8n-seg quantizada em INT8 assimétrico via PTQ).
* **Runtime:** VIPLite API nativa em C++ (`libVIPhal.so`, `vip_lite.h`).
* **Performance:**
  * Inferência pura na NPU: **~4.8 ms**.
  * Pré-processamento e Pós-processamento (DFL + Sigmoide + NMS + Matmul de máscaras 104x104) em C++ otimizado com OpenCV.
* **Saída:** Gera caixas delimitadoras e uma máscara de segmentação (`/yolo/dynamic_mask`).

---

### 2.3. Pacote `ORBBEC-ASTRA-PRO`

* **Hardware:** Orbbec Astra Pro.
* **Topologia USB Única:** O mesmo cabo físico expõe dois dispositivos internos:
  * `2bc5:0403`: Sensor de profundidade infravermelho (OpenNI2 com regras udev proprietárias).
  * `2bc5:0501`: Câmera RGB HD (UVC padrão via V4L2 em `/dev/video0`).
* **Inicialização:** Realizada através do script padrão `cd ~/orb && ./scripts/start_camera.sh`.

---

### 2.4. Pacote `esp32_firmware` (micro-ROS para Controle de Base Mecanum)

* **Hardware Target:** ESP32-WROOM-32 (DevKit v1) via USB Serial (conversor Silicon Labs CP2102 onboard), preparado para migração futura ao **ESP32-P4**.
* **Framework:** **ESP-IDF v5.2** utilizando o componente `micro_ros_espidf_component` (branch `humble`).
* **Transporte:** Serial XRCE-DDS customizado (`esp32_serial_transport.c`) operando na UART0 a **921,600 bps** com MTU customizado de **1024 bytes** (necessário para pacotes grandes como `nav_msgs/msg/Odometry`).
* **Isolamento de Console:** Console de log do ESP-IDF desativado na UART0 (`CONFIG_ESP_CONSOLE_NONE=y`) para evitar contaminação do canal binário de micro-ROS com mensagens ASCII de boot/debug.
* **Arquitetura de Processos e Interfaces (C++ Desacoplado):**
  * **Interfaces Abstratas (`main/interfaces/`):**
    * [`IMotorController`](file:///home/alisson/orange/firmware/esp32_firmware/main/interfaces/i_motor_controller.hpp): Contrato puro para controle de motores, velocidades alvo, parada de emergência e telemetria.
    * [`IEncoderSensor`](file:///home/alisson/orange/firmware/esp32_firmware/main/interfaces/i_encoder_sensor.hpp): Contrato puro para leitura de encoders magnéticos/ópticos e contagem de pulsos.
    * [`IImuSensor`](file:///home/alisson/orange/firmware/esp32_firmware/main/interfaces/i_imu_sensor.hpp): Contrato puro para aquisição de aceleração, velocidade angular e atitude ($q$).
    * [`IYawFilter`](file:///home/alisson/orange/firmware/esp32_firmware/main/interfaces/i_yaw_filter.hpp): Contrato para estimadores de orientação e fusão de guinada (Yaw).
    * [`IRosBridge`](file:///home/alisson/orange/firmware/esp32_firmware/main/interfaces/i_ros_bridge.hpp): Contrato para a camada de comunicação micro-ROS.
  * **Processos Independentes com Afinidade de Núcleo (Dual-Core):**
    * [`ControlProcess`](file:///home/alisson/orange/firmware/esp32_firmware/main/control_process.hpp) (Core 1, 50 Hz): Loop de controle em tempo real determinístico. Amostra encoders, IMU, integra cinemática Mecanum 4WD, aplica o filtro de Kalman de 2 estados para Yaw, processa watchdog de segurança (500 ms) e comanda os motores. Gera snapshots atômicos (`StateSnapshot`) protegidos por mutex.
    * [`CommunicationProcess`](file:///home/alisson/orange/firmware/esp32_firmware/main/communication_process.hpp) (Core 0, micro-ROS task): Máquina de estados XRCE-DDS serial, sincronização de relógio epoch via `rmw_uros_sync_session`, subscrição com watchdog de `/cmd_vel` e `/esp32/led`, e publicação periódica assíncrona consumindo os snapshots do processo de controle.
  * **Drivers de Hardware e Filtros Concretos (Portados do Holodex):**
    * [`As5600MuxEncoder`](file:///home/alisson/orange/firmware/esp32_firmware/main/encoder_sensor.hpp): Mapeia o multiplexador I2C PCA9548A (`0x70`) e lê os 4 encoders magnéticos AS5600 de 12 bits (`0x36`) nos canais 0, 1, 4 e 5, com tratamento de descontinuidade de ângulo (unwrapping) e fallback seguro.
    * [`Bmi160ImuSensor`](file:///home/alisson/orange/firmware/esp32_firmware/main/imu_sensor.hpp): Driver do DFRobot BMI160 6-DoF no canal 6 do multiplexador (ou barramento I2C direto em `0x69`), configurado com escala de giro de $\pm 2000^\circ/\text{s}$, acelerômetro a $\pm 2\text{g}$, deadband angular de $0.02\text{ rad/s}$ e integração de quatérnion.
    * [`StepDirMotorController`](file:///home/alisson/orange/firmware/esp32_firmware/main/motor_controller.hpp): Controlador de 4 eixos de passo/direção compatível com FastAccelStepper (pinos FL: 19/18/17, FR: 27/14/13, RL: 23/22/32, RR: 33/25/26), 6400 passos/rev (32 micropassos) e chave de habilitação ativa em nível baixo.
    * [`KalmanYawFilter`](file:///home/alisson/orange/firmware/esp32_firmware/main/yaw_kalman_filter.hpp): Filtro de Kalman discreto de 2 estados ($x = [\theta, b_z]^T$) fundindo taxa angular do giroscópio com a estimativa cinemática das rodas Mecanum.
    * [`I2cBus`](file:///home/alisson/orange/firmware/esp32_firmware/main/i2c_bus.hpp): Driver mestre I2C thread-safe protegido por FreeRTOS mutex operando a 400 kHz em SDA=21 e SCL=5.
    * [`main.cpp`](file:///home/alisson/orange/firmware/esp32_firmware/main/main.cpp): Orquestrador ultra-limpo de alto nível que instancia os drivers de hardware, conecta as interfaces por injeção de dependência e inicializa as tarefas nos respectivos núcleos.
* **Topologia de Tópicos Ativos:**
  * **Publicadores (ESP32 $\to$ Orange Pi):**
    * `/odom/raw` (`nav_msgs/msg/Odometry`): Estimativa de pose e twist da base (~25 Hz).
    * `/imu/data_raw` (`sensor_msgs/msg/Imu`): Orientação, velocidade angular e aceleração linear com covariâncias (~25 Hz).
    * `/joint_states` (`sensor_msgs/msg/JointState`): Posição e velocidade angular das 4 rodas Mecanum (~17 Hz).
    * `/esp32/steppers/status` (`std_msgs/msg/Int32MultiArray`): Posição em passos e velocidades atuais dos 4 motores (~10 Hz).
    * `/esp32/heartbeat` (`std_msgs/msg/Int32`): Heartbeat de conectividade a 1.000 Hz.
  * **Inscritos (Orange Pi $\to$ ESP32):**
    * `/cmd_vel` (`geometry_msgs/msg/Twist`): Comandos de velocidade linear e rotação com watchdog de segurança (500 ms timeout).
    * `/esp32/led` (`std_msgs/msg/Bool`): Controle remoto do LED de status (GPIO 2).
* **Execução na Orange Pi:** Nó `micro_ros_agent` em C++ lançado via `micro_ros_agent.launch.py`. Porta configurada para `/dev/esp32` (via regra udev) ou `/dev/ttyUSB0`.

---

## 3. Catálogo de Problemas Críticos e Diagnósticos (Post-Mortems)

Esta seção documenta bugs reais encontrados durante os testes práticos na bancada e como foram definitivamente sanados. **Não reverta essas correções.**

### Post-Mortem 1: A "Ilusão" de 5 Leituras por Segundo vs. Throughput Real de 30 Hz
* **Sintoma:** O usuário observou que a medição visual no terminal parecia atualizar apenas ~5 vezes por segundo, gerando a suspeita de que a odometria estava processando em 5 Hz em vez de 30 Hz.
* **Causa Raiz:** O código possuía a macro `RCLCPP_INFO_THROTTLE(..., 200, ...)` no loop principal de odometria. Um intervalo de 200 ms limita o output no console a exatamente 5 linhas por segundo para evitar saturação do SSH. No entanto, o nó recebia, calculava e publicava odometria em **~29.8 Hz**.
* **Solução:**
  1. Realizamos uma amostragem direta de 10 segundos nos tópicos ROS 2, confirmando que `/camera/depth/points` publica a 29.4 Hz, `/camera/filtered_points` a 30.2 Hz e `/icp/odom` a 29.8 Hz.
  2. Implementamos um medidor de vazão com janela deslizante de 1 segundo (`real_throughput_hz_`) no arquivo [`icp_odometry_component.cpp`](file:///home/alisson/orange/pointcloud_odometry/src/icp_odometry_component.cpp).
  3. O log agora exibe explicitamente `Throughput=29.8 msg/s`, eliminando qualquer ambiguidade.

---

### Post-Mortem 2: Corrupção do Submapa por Jitter ("Geometria Fantasma")
* **Sintoma:** Em repouso com a câmera estática, ocorreu um salto anômalo na odometria ($\Delta d = 0.229\text{ m}$). A partir daquele instante, o ICP perdeu o rastreio e travou a pose indefinidamente, com inliers despencando para ~20%.
* **Causa Raiz:** O nó aceitou o salto anômalo como um novo Keyframe e inseriu os pontos saltados dentro do `GaussianVoxelMap`. O submapa passou a ter duas cópias idênticas da sala deslocadas no espaço ("geometria fantasma"). Quando o detector de anomalias atuou nos quadros seguintes, ele rejeitou qualquer atualização, gerando um loop infinito de travamento.
* **Solução:**
  * **Inversão da lógica:** A validação cinemática estrita ($\Delta d \le 0.12\text{ m}$, $\Delta \theta \le 15^\circ$ por frame de 33 ms) foi movida para **ANTES** de qualquer decisão de keyframe.
  * Se um frame sofrer jitter ou salto não físico, ele é categoricamente ignorado e **NUNCA** é injetado no submapa.

---

### Post-Mortem 3: Keyframe Starvation (O Paradoxo dos 75% de Inliers)
* **Sintoma:** Ao mover a câmera pela sala, o submapa parava de crescer (ficava estagnado em ~390 voxels) e o rastreio quebrava assim que o robô se afastava do ponto de origem.
* **Causa Raiz:** O código exigia que `inlier_ratio > 0.75` para salvar um novo keyframe. Esse requisito gerava um paradoxo: ao explorar novas áreas, a sobreposição visual com a pose inicial naturalmente diminui; como os inliers caíam para 60%–70%, o nó recusava novos keyframes, forçando o robô a rastrear todo o ambiente contra os 390 voxels da primeira foto.
* **Solução:**
  * Removemos a trava de 75% de inliers para criação de keyframe.
  * O critério de keyframe passou a ser puramente geométrico: solver convergiu, teste cinemático aprovado, e deslocamento em relação ao último keyframe satisfaz $\Delta d > 0.15\text{ m}$ OU $\Delta \theta > 8^\circ$.

---

### Post-Mortem 4: Aquecimento Extremo (95°C) e 100% de CPU na Orange Pi
* **Sintoma:** A CPU da placa bateu 95°C e o sistema sofreu reinicialização forçada (*thermal shutdown*).
* **Causa Raiz:** Ocorrência de múltiplos processos concorrentes rodando simultaneamente (ex: o usuário ou um script lançou `pointcloud_odometry.launch.py` enquanto o `pipeline_orchestrator.py` também estava ativo em segundo plano). Como o `small_gicp` compila com OpenMP e padrão de 8 threads, duas instâncias disputando CPU levaram todos os 8 núcleos a 100%.
* **Solução:**
  1. Fixamos `num_threads: 4` em [`params.yaml`](file:///home/alisson/orange/pointcloud_odometry/config/params.yaml) para deixar 4 núcleos livres para o sistema operacional, driver da câmera e rede neural.
  2. Implementamos a diretriz mandatória: **nunca rodar múltiplos nós ou orquestradores em paralelo sem antes verificar com `pgrep -fa 'icp_odometry'`**.

---

### Post-Mortem 5: Mapa 3D Invertido e Faces Triangulares Gigantes no `.ply`
* **Sintoma:** Ao inspecionar o arquivo `map_output.ply` gerado pelo serviço `/icp/save_submap` no Windows 3D Viewer, a cena aparecia de cabeça para baixo e repleta de triângulos brancos gigantes cruzando o ambiente.
* **Causa Raiz:**
  1. **Inversão Vertical:** O sistema de coordenadas ópticas da câmera (REP 103) adota $+Z$ para frente, $+X$ para a direita e $+Y$ para baixo (em direção ao piso). Visualizadores 3D convencionais adotam $+Y$ ou $+Z$ apontando para o teto.
  2. **Triângulos Gigantes:** O Windows 3D Viewer assume que arquivos `.ply` sem declaração de faces são malhas poligonais e tenta conectar vértices sequenciais em tiras (*triangle strips*). Como os pontos são organizados em varredura horizontal ($u=0\dots640$), o visualizador conectava o final da linha $i$ ao início da linha $i+1$, criando faces que cortavam a sala inteira.
* **Solução:**
  1. No exportador do mapa, invertemos o eixo Y: $Y_{\text{export}} = -Y_{\text{camera}}$.
  2. Inserimos no cabeçalho do arquivo `.ply` a linha explícita `element face 0` e `property list uchar int vertex_indices`, forçando qualquer software a renderizar estritamente como nuvem de pontos.
  3. Adicionamos a exportação paralela de um arquivo `.xyz` puro (coordenadas ASCII $X, Y, Z$) que não possui malha sob nenhuma hipótese.
  4. Adicionamos colorização por gradiente de altura (azul = piso, verde = meio, vermelho = teto).

---

### Post-Mortem 6: Falhas de Rastreio Induzidas por Filtros de Borda Agressivos
* **Sintoma:** Em uma tentativa anterior de limpar ruídos, foi implementado um filtro que calculava a derivada espacial do depth em 2D e removia pixels de descontinuidade. O resultado foi desastroso: a nuvem perdeu os cantos das paredes e quinas de móveis, e o ICP perdeu o tracking angular.
* **Causa Raiz:** Quinas e bordas são justamente as feições geométricas mais ricas que ancoram a rotação no ICP.
* **Solução:** O filtro agressivo foi removido de [`pointcloud_filter_component.cpp`](file:///home/alisson/orange/pointcloud_odometry/src/pointcloud_filter_component.cpp). A filtragem geométrica deve ser deixada para a resolução de voxel do `GaussianVoxelMap` e para o limitador de correspondência `max_corr_dist`.

---

### Post-Mortem 7: Queda Silenciosa de Mensagens Maiores que 512 Bytes (O Caso da Odometria 6x6)
* **Sintoma:** Ao habilitar a publicação de `/odom/raw`, tópicos como `/esp32/heartbeat`, `/imu/data_raw` e `/joint_states` publicavam normalmente, mas `/odom/raw` nunca entregava dados no `ros2 topic echo`, sem disparar erro no agente.
* **Causa Raiz:** O micro-XRCE-DDS Client vinha configurado por padrão com `UXR_CONFIG_CUSTOM_TRANSPORT_MTU = 512` bytes. Como a mensagem `nav_msgs/msg/Odometry` possui duas matrizes de covariância $6 \times 6$ em `double` (36 doubles para pose + 36 doubles para twist = 576 bytes só em números flutuantes) somadas a cabeçalhos, strings e poses, seu tamanho serializado atinge ~720 bytes. Streams `BEST_EFFORT` no XRCE-DDS não suportam fragmentação e descartam silenciosamente qualquer pacote que exceda o MTU do transporte.
* **Solução:**
  1. Adicionamos `-DUCLIENT_CUSTOM_TRANSPORT_MTU=1024` no arquivo [`app-colcon.meta`](file:///home/alisson/orange/firmware/esp32_firmware/app-colcon.meta).
  2. Desabilitamos o perfil de UDP no build do `microxrcedds_client` para permitir compilação limpa do transporte customizado.
  3. Reempacotamos a biblioteca `libmicroros.a` com MTU de 1024 bytes e buffer de saída de 4096 bytes. `/odom/raw` passou a publicar continuamente a 25 Hz.

---

### Post-Mortem 8: Pânico no Boot por Ponteiro Nulo no Transporte Serial Customizado
* **Sintoma:** O ESP32 reiniciava em loop infinito com erro de pânico `Guru Meditation Error: Core 0 panic'ed (LoadProhibited)` no endereço `0x00000000` assim que o `micro_ros_task` tentava inicializar a comunicação.
* **Causa Raiz:** A função `esp32_serial_open` faz cast de `transport->args` para `size_t*` e desreferencia `*uart_port`. No entanto, `initTransport` passava `(void*)uart_port` diretamente (isto é, o valor inteiro 0 foi convertido para ponteiro nulo), gerando falha de segmentação imediata ao acessar a memória 0.
* **Solução:**
  * Declaramos uma variável estática persistente `static size_t port = uart_port;` e passamos seu endereço `(void*)&port` para `rmw_uros_set_custom_transport`.

---

### Post-Mortem 9: Contaminação Binária na UART0 por Logs de Boot do ESP-IDF
* **Sintoma:** O `micro_ros_agent` na Orange Pi não conseguia estabelecer sessão XRCE-DDS com o ESP32 e acusava perda de sincronismo serial ou recebia caracteres corrompidos (`€€€ €€...`).
* **Causa Raiz:** A porta UART0 do ESP32 estava compartilhada com o console padrão do ESP-IDF (`CONFIG_ESP_CONSOLE_UART_DEFAULT=y` a 115.200 bps). Como o micro-ROS opera na mesma porta UART0 a 921.600 bps binário, cada caractere ASCII de log emitido pelo kernel corrompia as mensagens XRCE-DDS.
* **Solução:**
  * Forçamos `CONFIG_ESP_CONSOLE_NONE=y` e `CONFIG_ESP_CONSOLE_UART_DEFAULT=n` em `sdkconfig.defaults` e regeramos o `sdkconfig`. O micro-ROS agora possui exclusividade total sobre a UART0.

---

### Post-Mortem 10: Incompatibilidade de QoS em Tópicos Crus (Best Effort vs Reliable)
* **Sintoma:** O nó de odometria na Orange Pi recebia aviso do ROS 2 `New publisher discovered on topic '/wheel/ticks', offering incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY`.
* **Causa Raiz:** O micro-ROS publica tópicos de alta frequência em perfil `BEST_EFFORT` para economizar memória e evitar retransmissões que travariam o buffer do microcontrolador. Assinantes padrão em `rclcpp` usam perfil `RELIABLE`, que recusam conexão com publicadores Best Effort.
* **Solução:**
  * Configuramos explicitamente a política de QoS do subscriber na Orange Pi para `rclcpp::QoS(10).best_effort().durability_volatile()`.

---

### Post-Mortem 11: Tipagem Estrita de Dimensões em Inteiros no Nav2 Costmap (width e height)
* **Sintoma:** Ao ativar o Nav2, os nós `controller_server` e `planner_server` sofriam terminação imediata com código de saída `-6` (SIGABRT): `rclcpp::exceptions::InvalidParameterTypeException: parameter 'height' has invalid type: Wrong parameter type, parameter {height} is of type {integer}, setting it to {double} is not allowed.`
* **Causa Raiz:** No ROS 2 Humble, o plugin `nav2_costmap_2d` declara os parâmetros de dimensão geométrica `width` e `height` estritamente como inteiros em metros. Notações decimais no YAML (`width: 3.5` ou `15.0`) violam a tipagem estrita da biblioteca.
* **Solução:**
  * Ajustamos os parâmetros para inteiros explícitos no [`nav2_params.yaml`](file:///home/alisson/orange/holodex_bringup/config/nav2_params.yaml): `width: 4`, `height: 4` no `local_costmap` e `width: 15`, `height: 15` no `global_costmap`.

---

### Post-Mortem 12: Incompatibilidade do Tópico de Chute Inicial (/odom_combined vs /odometry/filtered)
* **Sintoma:** O solver do `small_gicp` não recebia chutes iniciais de odometria de rodas durante rotações rápidas, operando como tracking desacoplado.
* **Causa Raiz:** O componente de odometria ICP assinava `/odom_combined` por padrão, enquanto o filtro EKF do robô publica em `/odometry/filtered`.
* **Solução:**
  * (Superado pelo Post-Mortem 14) Havia sido alinhado para `/odometry/filtered`; agora o padrão é `/odom/wheel`. Mantém `use_tf_extrinsic: true` para compensação de braço de alavanca.

---

### Post-Mortem 14: Fusão do ICP no EKF (realimentação, covariância e yaw duplicado)
* **Problemas:** (1) o ICP usava a saída do EKF como chute inicial, e o EKF passou a fundir o ICP (laço de realimentação); (2) a covariância do ICP era `H⁻¹` limitada a 1.0 m², sem inflar em `TRACKING_LOST` nem em frames rejeitados (pose obsoleta republicada como medição); (3) a IMU entrava com yaw absoluto e `wz` do mesmo giroscópio; (4) TF `base_link→imu_link` publicado duas vezes (URDF e estático no `base_fusion.launch.py`).
* **Solução:** chute inicial em `/odom/wheel`; `publishOdometry` usa `σ²·H⁻¹` com pisos (`min_pos_variance`, `min_rot_variance`) e variância `lost_variance` (1e3) quando o frame é rejeitado ou o estado é LOST; `ekf.yaml` ganhou `odom1: /icp/odom` (só X, Y, Yaw, rejeição Mahalanobis 5.0) e a IMU contribui só com `wz`; TF estático removido (vale o do URDF).
* **Pendente:** o submapa (`/icp/submap_cloud`, frame `odom`) fica no referencial interno do ICP, não no do EKF; ver docs/proximos_passos_engenharia.md.

---

### Post-Mortem 15: Recuperação de `TRACKING_LOST` (dead-reckoning + porta adaptativa)
* **Problema:** com a pose congelada em frames rejeitados, o chute era `pose_congelada × incremento de 1 frame`. Se o robô andasse > ~0,5 m (ou girasse > ~25°) durante a perda, a relocalização era impossível. Os limites cinemáticos eram fixos e relativos à pose congelada.
* **Solução (`recovery_utils.hpp`, `icp_odometry_component.cpp`):** (1) no primeiro frame rejeitado arma-se uma âncora (pose ICP boa + pose da odometria de referência naquele instante); enquanto rejeitado, o chute é `T_icp_âncora · (T_ref_âncora⁻¹ · T_ref_agora)`; (2) a porta cinemática passa a medir o desvio do ICP em relação à **predição**: `max = base + slip_ratio × deslocamento_previsto` (base 0,08 m/5° normal; 0,30 m/25° em dead-reckoning/LOST). Sem predição, valem os limites absolutos antigos.
* **Referência:** `use_ekf_for_recovery` (padrão `false` = rodas). Com `true` usa `/odometry/filtered` só durante a rejeição, quando o ICP tem peso ~0 no EKF (sem realimentação). Ligar só depois de validar a IMU.
* **Status:** compilado e testado na placa só em lógica (`test_recovery_utils`). **Não validado com câmera/IMU reais.** A porta é adaptativa por deslocamento, não uma distância de Mahalanobis completa.
* **Ajustes a fazer em bancada:** `gate_*`, e se a relocalização precisa de raio de busca maior que 0,5 m (`relocalize_attempt_counter_ < 5` ainda limita a 5 tentativas com raio expandido).

---

### Post-Mortem 13: Namespace Mismatch de Parâmetros do Motion Supervisor
* **Sintoma:** O nó `holodex_motion_supervisor` iniciava sempre com parâmetros default (ex: limiares conservadores) ignorando as configurações do `supervisor.yaml`.
* **Causa Raiz:** O arquivo `supervisor.yaml` continha a chave raiz `motion_supervisor:`, enquanto o executável e os launch files nomeavam o nó como `holodex_motion_supervisor`. No ROS 2 Humble, nós não aninhados sob o mesmo nome ignoram o bloco de parâmetros.
* **Solução:**
  * Atualizamos a chave de configuração para `holodex_motion_supervisor: ros__parameters:`.

---

### Post-Mortem 14: Jitter Temporal e Descarte de Pacotes Fora de Ordem em Odometria de Rodas
* **Sintoma:** Ocorrência de avisos espúrios `Delta t elevado (4294.9s). Reiniciando base de tempo.` no nó de odometria de rodas quando ocorria jitter serial.
* **Causa Raiz:** A subtração do timestamp em microssegundos do ESP32 era realizada com tipos `uint32_t`. Um pacote que chegasse com timestamp minimamente inferior (fora de ordem em microssegundos) resultava em underflow unsigned (~$4.294\text{ s}$), disparando a reinicialização da base temporal.
* **Solução:**
  * Convertemos o cálculo da diferença para signed de 32 bits: `const int32_t dt_signed_us = static_cast<int32_t>(sample_us - last_sample_us_);`. Se `dt_signed_us <= 0`, o pacote é categorizado como duplicado/fora de ordem e descartado silenciosamente sem corromper o integrador de odometria.

---

## 4. Estado Atual do Sistema (Pronto para Uso)

| Funcionalidade | Status | Arquivos Relevantes | Observações |
| :--- | :--- | :--- | :--- |
| **Driver Astra Pro** | Funcionando (~30 Hz) | `~/orb/scripts/start_camera.sh` | Publica em `/camera/depth/points` |
| **Filtro 2D Striding** | Funcionando (<0.2 ms) | [`pointcloud_filter_component.cpp`](file:///home/alisson/orange/pointcloud_odometry/src/pointcloud_filter_component.cpp) | Reduz para ~2.100 pts |
| **Odometria VGICP** | Funcionando (~30 Hz) | [`icp_odometry_component.cpp`](file:///home/alisson/orange/pointcloud_odometry/src/icp_odometry_component.cpp) | Submapa estável na RAM |
| **Throughput Monitor** | Funcionando | [`icp_odometry_component.cpp`](file:///home/alisson/orange/pointcloud_odometry/src/icp_odometry_component.cpp#L363-L387) | Mede e loga vazão real em msg/s |
| **Exportação de Mapa** | Funcionando | [`icp_odometry_component.cpp`](file:///home/alisson/orange/pointcloud_odometry/src/icp_odometry_component.cpp#L500-L650) | Gera `.ply` (sem faces) e `.xyz` |
| **Inferência NPU INT8** | Operacional (~25 Hz Contínuo) | [`yolo_seg_node.cpp`](file:///home/alisson/orange/yolo_seg_ros2/src/yolo_seg_node.cpp) | Inferência a 4.8 ms na VIP9000, publica `/yolo/dynamic_mask` (mono8) |
| **Filtro Dinâmico** | Implementado e Integrado | [`pointcloud_filter_component.cpp`](file:///home/alisson/orange/pointcloud_odometry/src/pointcloud_filter_component.cpp) | Rejeita pontos mapeados na máscara de classes dinâmicas (humanos/veículos) |
| **Base micro-ROS ESP32** | Funcionando em Tempo Real (LEDC + PCNT + TWDT + Dado Cru) | [`main.cpp`](file:///home/alisson/orange/firmware/esp32_firmware/main/main.cpp) | 4 eixos LEDC pulsos reais, PCNT steps, IMU cru 50Hz, ticks 50Hz, watchdog 500ms |
| **Nó Odometria Mecanum (Orange Pi)** | Funcionando (~45 Hz) | [`mecanum_wheel_odometry_node.cpp`](file:///home/alisson/orange/mecanum_wheel_odometry/src/mecanum_wheel_odometry_node.cpp) | Cinemática direta 4WD, RK2, sem TF, publica `/odom/wheel` |
| **Fusão EKF + Madgwick (Orange Pi)** | Instalado e Operacional | `base_fusion.launch.py` | Gera `/odometry/filtered` e a única TF `odom -> base_link` |
| **Orquestrador Central & Benchmark** | Funcionando | [`holodex_orchestrator.py`](file:///home/alisson/orange/holodex_bringup/scripts/holodex_orchestrator.py) | Dashboard TUI, telemetria ao vivo, CPU big.LITTLE, shutdown seguro |
| **URDF e Árvore TF Unificada** | Funcionando | [`holodex.urdf`](file:///home/alisson/orange/holodex_bringup/urdf/holodex.urdf) | `base_footprint -> base_link -> camera_link`, `imu_link`, 4 rodas |
| **Submapa 3D como Tópico** | Funcionando (~1 Hz / Keyframe) | [`icp_odometry_component.cpp`](file:///home/alisson/orange/pointcloud_odometry/src/icp_odometry_component.cpp) | Publica centros gaussianos em `/icp/submap_cloud` (frame `odom`) |
| **Supervisor de Movimento** | Funcionando (20 Hz) | [`motion_supervisor.py`](file:///home/alisson/orange/holodex_bringup/holodex_bringup/motion_supervisor.py) | Intertravamento 360°, slip guard, jerk lim, /cmd_vel_nav -> /cmd_vel |
| **Nav2 Holonômico Mecanum** | Instalado e Validado | [`navigation.launch.py`](file:///home/alisson/orange/holodex_bringup/launch/navigation.launch.py) | DWB Mecanum holonômico + SmacPlanner + Costmap 360° com submapa |

---

## 5. Regras de Ouro para Novos Agentes de IA e Desenvolvedores

1. **JAMAIS lance processos duplicados na Orange Pi:**
   Antes de rodar qualquer nó de odometria, verifique se já não existe outro rodando:
   ```bash
   pgrep -fa 'icp_odometry|component_container'
   ```
2. **NUNCA altere `publish_tf: true` sem alinhamento com o EKF:**
   Se a odometria publicar TF diretamente, ela competirá com o `robot_localization` e fará o Nav2 oscilar violentamente.
3. **Mantenha `range_max: 3.8`:**
   Não reduza para 2.5 m. Paredes distantes fornecem restrições geométricas cruciais para o cálculo de rotação (Yaw).
4. **Compile SEMPRE em modo Release:**
   Compilar em modo Debug no ARM desacelera o `small_gicp` em mais de $10\times$, tornando impossível atingir 30 Hz.
5. **Cuidado com o USB da Astra Pro:**
   Se a câmera travar, desligar e religar pelo script `start_camera.sh` é o procedimento correto. Não mate os processos com `kill -9` sem necessidade, pois isso pode deixar o descritor USB em estado zumbi.

---

## 6. To-Do List e Roadmap Futuro (Backlog de Engenharia)

### Curto Prazo (Validação de Trajetória e Fusão Sensorial)
- [ ] **Teste de Circuito Fechado (Loop Closure Simples):**
  - Conduzir o robô em uma trajetória fechada (quadrado de $2\text{ m} \times 2\text{ m}$) retornando exatamente à posição de partida para quantificar a deriva ($x, y, \theta$).
- [ ] **Calibração Extrínseca Câmera-Robô (`tf_static`):**
  - Configurar a transformação exata entre `base_link` (centro do robô Mecanum) e `camera_link` no arquivo URDF ou publicador de TF estático.
  - Ativar `use_tf_extrinsic: true` para que as velocidades publicadas em `/icp/odom` reflitam o centro cinemático do robô.
- [ ] **Ajuste Fino do EKF (`robot_localization`):**
  - Configurar o arquivo `ekf.yaml` para fundir:
    - Encoders das rodas Mecanum: velocidades lineares ($v_x, v_y$).
    - IMU: velocidade angular ($\omega_z$) e orientação absoluta.
    - ICP Odometry: posição ($x, y$), orientação e velocidades refinadas.

### Médio Prazo (Ativação da Percepção Neural em Tempo Real)
- [ ] **Ativação Contínua do `yolo_seg_node` em Nó ROS 2:**
  - Garantir que o nó C++ rode em loop publicando continuamente a máscara em `/yolo/dynamic_mask` a ~20–30 FPS.
- [ ] **Ligar `enable_dynamic_filter: true`:**
  - Habilitar o parâmetro no [`params.yaml`](file:///home/alisson/orange/pointcloud_odometry/config/params.yaml) e testar a odometria com pessoas caminhando na frente do robô para validar a imunidade a interferências dinâmicas.

### Longo Prazo (Integração com SLAM e Nav2)
- [ ] **Costmap 3D / Obstacle Layer para Nav2:**
  - Integrar `/camera/filtered_points` ou o submapa local diretamente como camada de obstáculos tridimensionais no Nav2 Costmap 2D/3D (Spatio-Temporal Voxel Layer).
- [ ] **Exportação para SLAM Global:**
  - Avaliar o uso dos keyframes e poses gerados pelo `small_gicp` como frontend para um framework de grafos de poses (ex: `GTSAM` ou `Cartographer`).

---

## 7. Cheat Sheet de Comandos Úteis

### Iniciar Câmera na Orange Pi
```bash
cd ~/orb && ./scripts/start_camera.sh
```

### Iniciar Odometria 3D
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch pointcloud_odometry pointcloud_odometry.launch.py
```

### Inspecionar Throughput Real nos Tópicos
```bash
ros2 topic hz /icp/odom
ros2 topic hz /camera/filtered_points
```

### Salvar o Submapa 3D
```bash
ros2 service call /icp/save_submap std_srvs/srv/Trigger "{}"
```

### Checar Temperatura e Processos na Orange Pi
```bash
# Temperatura da CPU em graus Celsius
cat /sys/class/thermal/thermal_zone0/temp | awk '{print $1/1000 " °C"}'

# Checar processos de odometria em execução
pgrep -fa 'icp_odometry|component_container'
```
