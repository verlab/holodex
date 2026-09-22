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
  * Assina `/odom_combined` (odometria das rodas Mecanum + IMU).
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

### 2.4. Pacote `esp32_firmware` (micro-ROS para Controle de Base)

* **Hardware Target:** ESP32-WROOM-32 (DevKit v1) via USB Serial (conversor Silicon Labs CP2102 onboard), preparado para migração futura ao **ESP32-P4**.
* **Framework:** **ESP-IDF v5.1 / v5.2** utilizando o componente `micro_ros_espidf_component` (branch `humble`).
* **Transporte:** Serial XRCE-DDS customizado (`esp32_serial_transport.c`) operando na UART0 a **921,600 bps**.
* **Isolamento de Console:** Console de log do ESP-IDF desativado na UART0 (`CONFIG_ESP_CONSOLE_NONE=y`) para evitar contaminação do canal binário de micro-ROS com mensagens ASCII de boot/debug.
* **Topologia de Tópicos:**
  * `/esp32/heartbeat` (`std_msgs/msg/Int32`): Contador heartbeat a 1 Hz com auto-reconexão no client.
  * `/esp32/led` (`std_msgs/msg/Bool`): Controle remoto do LED de status (GPIO 2).
* **Execução na Orange Pi:** Nó `micro_ros_agent` em C++ lançado via `micro_ros_agent.launch.py` (ou `ros2 launch`). Porta configurada por padrão para `/dev/esp32` (via regra udev) ou `/dev/ttyUSB0`.

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

## 4. Estado Atual do Sistema (Pronto para Uso)

| Funcionalidade | Status | Arquivos Relevantes | Observações |
| :--- | :--- | :--- | :--- |
| **Driver Astra Pro** | Funcionando (~30 Hz) | `~/orb/scripts/start_camera.sh` | Publica em `/camera/depth/points` |
| **Filtro 2D Striding** | Funcionando (<0.2 ms) | [`pointcloud_filter_component.cpp`](file:///home/alisson/orange/pointcloud_odometry/src/pointcloud_filter_component.cpp) | Reduz para ~2.100 pts |
| **Odometria VGICP** | Funcionando (~30 Hz) | [`icp_odometry_component.cpp`](file:///home/alisson/orange/pointcloud_odometry/src/icp_odometry_component.cpp) | Submapa estável na RAM |
| **Throughput Monitor** | Funcionando | [`icp_odometry_component.cpp`](file:///home/alisson/orange/pointcloud_odometry/src/icp_odometry_component.cpp#L363-L387) | Mede e loga vazão real em msg/s |
| **Exportação de Mapa** | Funcionando | [`icp_odometry_component.cpp`](file:///home/alisson/orange/pointcloud_odometry/src/icp_odometry_component.cpp#L500-L650) | Gera `.ply` (sem faces) e `.xyz` |
| **Inferência NPU INT8** | Compilado e Validado | [`yolo_seg_node.cpp`](file:///home/alisson/orange/yolo_seg_ros2/src/yolo_seg_node.cpp) | Inferência a 4.8 ms na VIP9000 |
| **Filtro Dinâmico** | Implementado (Inativo) | `enable_dynamic_filter: false` | Pronto para ligar via parâmetro |

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
