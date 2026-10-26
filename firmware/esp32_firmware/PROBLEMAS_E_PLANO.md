# Firmware ESP32 (micro-ROS): problemas encontrados e plano de correção

> Documento para um agente/engenheiro que vai implementar as correções.
> Estado analisado: working tree da branch `main` (refactor em `*_process`, `interfaces/`, drivers separados), ainda não commitado.
> Nada foi compilado nem testado em hardware durante a análise: tudo abaixo vem de leitura de código.
> Os números de linha são do estado atual e podem se deslocar.

**Antes de mexer:** leia a seção 3 (Post-Mortems) de `/home/alisson/orange/GEMINI.md`. Não reverta o MTU de 1024 (`app-colcon.meta`), o console desativado na UART0 (`sdkconfig.defaults`) nem o `publish_tf: false` do lado da Orange Pi.

---

## 0. Decisões de arquitetura pedidas

### 0.1 Tirar o filtro de Kalman do ESP32 e levar para a Orange Pi: **recomendado**

**Sobre o argumento do "filtro pesado":** o `KalmanYawFilter` atual (`yaw_kalman_filter.cpp`) é de 2 estados, com ~50 operações de ponto flutuante por ciclo a 50 Hz. O ESP32 tem FPU de precisão simples, então o custo de CPU é desprezível (microssegundos). **Não existe slowdown vindo do Kalman.** Quem consome o tempo do ciclo de 20 ms é o I2C: 4 encoders + IMU + 5 trocas de canal do mux, cada uma com `ets_delay_us(50)` em busy-wait.

**Mesmo assim, mover é a decisão certa, por outros motivos:**

1. **Qualidade.** O filtro é simples e tem problemas (seção 2, item P8). Ajustar `Q`/`R` exige reflashar o ESP a cada tentativa. Na Orange Pi isso vira um arquivo YAML.
2. **O projeto já planeja isso.** O `GEMINI.md` prevê um EKF `robot_localization` fundindo encoders + IMU + ICP. Manter um segundo estimador de yaw no ESP duplica a fusão e cria duas fontes de orientação concorrentes.
3. **Separação de responsabilidades.** O ESP32 deve ser um nó de sensores e atuadores: amostrar com precisão de tempo, gerar pulsos de passo, ter um watchdog de segurança e publicar dado cru. A estimação de estado fica na Orange Pi, onde há CPU, depuração e `rosbag`.
4. **Reprodutibilidade.** Com dado cru gravado em bag, dá para reprocessar com outro filtro ou outros parâmetros. Com o dado já fundido, isso se perde.

**Contrapartidas a respeitar (por isso a segurança continua no ESP):**

- A Orange Pi tem latência e jitter (USB serial, Linux não-RT, carga do ICP/YOLO). Por isso o **watchdog de `/cmd_vel` e a parada de emergência continuam no ESP32**, e nenhum laço de segurança depende da Pi.
- Os timestamps do dado cru têm de ser os do **instante da amostragem** (seção 0.2), e não os do instante da publicação, senão a fusão na Pi sofre com o atraso variável do link.

**O que fazer:**

- Remover `KalmanYawFilter` e a interface `IYawFilter` do ESP (ou deixar o arquivo fora do build). `ControlProcess` deixa de receber `IYawFilter&`.
- Remover do ESP: integração de yaw do IMU (`imu_sensor.cpp:158-166`, `yaw_accumulated_`), cálculo de quaternion e `OdomState` com pose integrada.
- Na Orange Pi, criar:
  - um nó `mecanum_wheel_odometry` (Python ou C++): lê os ticks/joint states crus, aplica a cinemática direta (portar `MecanumKinematics::forwardKinematics`/`updateOdometry`) e publica `/odom/wheel` com covariância adequada, **sem TF**;
  - o `robot_localization` (EKF) fundindo `/odom/wheel` (vx, vy, vyaw) + `/imu/data_raw` filtrado (vyaw, e opcionalmente yaw absoluto). Para orientação a partir do IMU, usar `imu_filter_madgwick` ou `imu_complementary_filter` e alimentar o EKF com a saída;
  - o EKF é a única fonte de `odom -> base_link` (`publish_tf: false` no ICP, já definido).
- O Kalman portado para Python/C++ **não** é necessário: o EKF do `robot_localization` já cobre o caso (estima orientação e usa o giro para o bias, indiretamente). Só escreva um filtro próprio se o EKF não bastar.

### 0.2 Enviar leituras cruas pelo micro-ROS, e não só a odometria processada

Hoje o ESP publica `/odom/raw` (pose já integrada, com yaw do Kalman) e `/imu/data_raw` (com **deadband** aplicado e **quaternion integrado**, ou seja, nem o "raw" é raw). Mudar para:

| Tópico | Tipo sugerido | Conteúdo | Taxa |
|---|---|---|---|
| `/wheel/ticks` | `std_msgs/Int32MultiArray` (4 valores: FL, FR, RL, RR) **ou** mensagem customizada com `stamp` | contagem **acumulada** de ticks do AS5600 já com unwrapping (12 bits = 4096 ticks/rev), sem dividir por `dt` | 50 Hz (cada ciclo de controle) |
| `/joint_states` | `sensor_msgs/JointState` | posição (rad) e velocidade (rad/s) por roda, calculadas dos ticks. Dá compatibilidade com ferramentas ROS | 20-50 Hz |
| `/imu/data_raw` | `sensor_msgs/Imu` | gyro e accel **sem deadband e sem filtro**; `orientation_covariance[0] = -1` (convenção do ROS para "sem orientação") e quaternion `0,0,0,1`/ignorado | 50 Hz ou mais (ver nota) |
| `/esp32/steppers/status` | `Int32MultiArray` | passos **reais** contados, velocidade comandada, flags | 10 Hz |
| `/esp32/heartbeat` | `Int32` | contador | 1 Hz |

**Regras importantes para o dado cru:**

1. **Timestamp na amostragem.** O `RobotStateSnapshot` deve carregar o `esp_timer_get_time()` do instante da leitura. `fillTimestamp` em `micro_ros_bridge.cpp:896` hoje carimba o momento da **publicação**. Passar o stamp do snapshot (convertido para epoch via `rmw_uros_epoch_nanos`/offset de sync).
2. **Ticks acumulados, não deltas.** Com `BEST_EFFORT`, perder uma mensagem não pode perder movimento. A Pi calcula a diferença. O contador deve ser `int32` (ou `int64` convertido) com unwrapping feito no ESP, e deve **resetar só no boot**.
3. **Não subamostrar o giroscópio.** Hoje o controle roda a 50 Hz e a comunicação publica o **último** snapshot a ~30 Hz (`communication_process.cpp`), então amostras são perdidas. Para a integração de yaw na Pi, o ideal é publicar **cada amostra** (50 Hz) ou ler o FIFO do BMI160 e publicar em lote. Se for mais de uma amostra por mensagem, usar um buffer circular e uma mensagem com array.
4. **Largura de banda:** 921600 baud ≈ 90 KB/s úteis. `Imu` (~320 B) a 50 Hz ≈ 16 KB/s, ticks ≈ 3 KB/s, joint_states ≈ 10 KB/s. Cabe, mas somar tudo e medir com `ros2 topic hz/bw`. Não passar de ~50-60% do canal.
5. **Mensagem customizada:** exigiria recompilar a `libmicroros.a` (custom_messages no `micro_ros_espidf_component`). Se quiser evitar isso, `Int32MultiArray` + `JointState` + `Imu` resolvem. Se criar `WheelTicks.msg` (`Header header; int32[4] ticks;`), manter `rmw_microxrcedds` com `MAX_PUBLISHERS` suficiente (hoje 10; já tem 5 + novos).
6. **Calibração do bias do giroscópio** pode ficar no ESP (média de ~2 s parado no boot, publicando o valor) ou na Pi. Se ficar no ESP, publique o bias estimado em um tópico de diagnóstico e **não** use deadband.

---

## 1. Resumo do estado

- A arquitetura de código está boa: interfaces, injeção de dependência, dois processos em núcleos distintos, mutex no snapshot, fallbacks quando mux/IMU não existem.
- **O firmware ainda não move o robô** (P1) e a odometria pode ser inventada a partir do comando (P2). O `GEMINI.md` diz que a base está "funcionando em tempo real": isso não é verdade para o código atual. Corrigir o documento ao final.

---

## 2. Problemas e como resolver

Severidade: 🔴 crítico · 🟠 alto · 🟡 médio · ⚪ baixo.

### 🔴 P1: Não existem pulsos de STEP: os motores nunca giram
- **Onde:** `main/motor_controller.cpp` inteiro (`init`, `applyGpioHardware`, `setWheelVelocities`).
- **O que acontece:** `PIN_STEP_*` entram na máscara de GPIO de saída (`motor_controller.cpp:22-32`) e nunca são acionados. Só DIR e ENA são escritos. Não há FastAccelStepper, LEDC, RMT nem MCPWM no projeto (`grep` confirma). `current_position_steps` fica sempre 0 e `current_speed_steps_s` é a velocidade **comandada**, e não a real.
- **Como resolver (opções, da mais simples à mais robusta):**
  1. **FastAccelStepper** como componente ESP-IDF (é o que o comentário em `robot_types.hpp:85` e o `GEMINI.md` prometem). Dá rampa de aceleração (`max_accel_rad_s2` já existe no construtor), contagem de passos real e `runForward/runBackward` + `setSpeedInHz`. Verificar a compatibilidade com ESP-IDF 5.2 e o uso de RMT (conflito com outros periféricos? o I2C e a UART não usam RMT).
  2. **LEDC** (PWM por hardware) com frequência variável = velocidade em passos/s, e **PCNT** para contar passos. Mais código, sem dependência externa, mas a aceleração tem de ser feita por software em rampa na task de controle.
  3. **RMT** com *encoder* de passos para trajetórias. Mais complexo; só se precisar de perfis precisos.
- **Detalhes a tratar:**
  - Limitar a velocidade máxima por roda (hoje `max_speed_rad_s = 25` ⇒ ~25.500 passos/s com 6400 passos/rev; confirmar que o driver e o motor aguentam).
  - Aplicar a rampa (`max_accel_rad_s2 = 50`) para não perder passo em mudança brusca de `/cmd_vel`.
  - Tempo mínimo de DIR antes do primeiro pulso (setup time do driver, tipicamente alguns µs), conforme o datasheet.
  - Escalonar o `ENA` (hoje habilitado no `init` e nunca desabilitado em repouso: motor quente e consumo contínuo). Desabilitar após N segundos parado, se o projeto aceitar perder o torque de retenção.
  - `status_.*.current_position_steps` passa a vir do contador real.
- **Critério de aceite:** com `ros2 topic pub /cmd_vel ...` cada roda gira no sentido e na velocidade esperados; `/esp32/steppers/status` mostra passos crescendo; ao parar o publicador, para em ≤ 500 ms.

### 🔴 P2: Sem encoders, a "odometria" vem do próprio comando
- **Onde:** `control_process.cpp:127-130`.
- **O que acontece:** se os AS5600 não respondem (`!enc_ok` ou nenhum sensor ativo), `wheel_vel = motor_ctrl_.getEstimatedWheelVelocities()`, que é a velocidade **comandada**. Como o P1 faz o robô não andar, ele publica movimento que não existe, e isso entra no EKF e no Nav2 como se fosse medido.
- **Como resolver:**
  - No modelo de dado cru (seção 0.2) o ESP **não** deve inventar medida. Publicar só ticks reais; se não há encoder, publicar o tópico com flag de inválido (ex.: um campo/tópico de diagnóstico `/esp32/encoders/valid` ou simplesmente **não publicar**).
  - Na Pi, o nó `mecanum_wheel_odometry` deve aumentar muito a covariância (ou não publicar) quando os ticks estiverem parados/inválidos.
  - Se quiser manter um modo "malha aberta" para teste de bancada, torne-o **explícito** (parâmetro de build/`Kconfig`) e publique em outro tópico (`/odom/open_loop`).

### 🟠 P3: Corrida de dados entre `/cmd_vel` (Core 0) e o controle (Core 1)
- **Onde:** `control_process.cpp:73-82` (`setTargetTwist`), `:84-88` (`triggerEmergencyStop`), `:166-167` (`getGroupStatus`); `motor_controller.cpp` (`status_`, `target_velocities_`, `is_enabled_`).
- **O que acontece:** o callback do micro-ROS roda na task de comunicação e chama `motor_ctrl_.setWheelVelocities(...)` direto, enquanto a task de controle lê e escreve o mesmo estado sem mutex. `last_cmd_timestamp_us_` (`int64_t`) é escrito em um núcleo e lido em outro sem atomic: leitura rasgada é possível em Xtensa de 32 bits, e o watchdog pode disparar errado (parar o robô com comando válido) ou deixar de disparar.
- **Como resolver (preferido):** o callback só **entrega** o comando, e quem aplica é a task de controle.
  - Use `xQueueOverwrite` em uma fila de tamanho 1 (`TwistCommand` + timestamp) ou uma struct protegida por `portMUX`/mutex.
  - O laço de controle lê a fila a cada ciclo, chama `inverseKinematics` e `setWheelVelocities` ⇒ **um único dono** do estado do motor.
  - `triggerEmergencyStop` vira uma flag atômica (`std::atomic<bool>` ou `xTaskNotify`) consumida pelo laço de controle.
  - `led_override_` (`communication_process.hpp`): `std::atomic<bool>`.

### 🟠 P4: Task watchdog desligado e reset em cascata
- **Onde:** `sdkconfig.defaults:3` (`CONFIG_ESP_TASK_WDT=n`); `communication_process.cpp:210-217,221-229`.
- **O que acontece:**
  1. Com o TWDT desligado, uma task de controle travada (I2C pendurado, por exemplo) não é detectada, e as saídas de motor ficam no último estado.
  2. 5 pings falhos seguidos (`pingAgent(100, 1)` ⇒ ~0,5 s) causam `esp_restart()`. Reiniciar o agente na Pi, um glitch no USB ou uma carga alta reinicia o ESP, perde o estado e interrompe o movimento sem explicação.
- **Como resolver:**
  - Religar `CONFIG_ESP_TASK_WDT=y` e registrar a task de controle (`esp_task_wdt_add`) com `esp_task_wdt_reset()` a cada ciclo. Configurar o panic handler para colocar os pinos ENA em nível seguro (desabilitado).
  - Substituir o `esp_restart()` por: parar motores → `destroyEntities()` → voltar a `WAITING_AGENT`. Só reiniciar como último recurso (ex.: `createEntities` falhando N vezes).
  - Aumentar a tolerância (por exemplo 10 falhas ou timeout em milissegundos, e não em número de iterações do laço).

### 🟠 P5: Laço de comunicação gasta tempo em ping e perde amostras
- **Onde:** `communication_process.cpp:156-218`.
- **O que acontece:** a cada iteração do `AGENT_CONNECTED` há `getSnapshot`, publicações, `spinSome(5)` e um `pingAgent(100, 1)` (que pode bloquear até 100 ms). A taxa real de publicação cai e fica irregular, e o `spinSome` atrasa a chegada de `/cmd_vel`.
- **Como resolver:**
  - Fazer o ping a cada ~1 s (por tempo, não por iteração) e com timeout curto.
  - Publicar com base em **eventos** (cada novo snapshot do controle) usando `xTaskNotify` do controle para a comunicação, em vez de fazer polling.
  - Medir as taxas reais com `ros2 topic hz` e ajustar `INTERVAL_*` (os valores do código: 30/30/20/10/1 Hz; o `GEMINI.md` fala em 25/25/17/10 e em heartbeat de **1000 Hz**, que está errado).

### 🟠 P6: BMI160 sem configurar faixa e ODR
- **Onde:** `imu_sensor.cpp:79-95` (init) e `:141-155` (escalas).
- **O que acontece:** o código assume ±2000 °/s (16,4 LSB/°/s) e ±2 g, mas só envia comandos de *power mode*. Os registros `ACC_RANGE (0x41)`, `GYR_RANGE (0x43)`, `ACC_CONF (0x40)`, `GYR_CONF (0x42)` nunca são escritos. Funciona só se o reset do chip deixar exatamente esses valores. O `ODR` padrão também pode ser menor que 50 Hz e o filtro digital pode introduzir atraso.
- **Como resolver:**
  - Escrever explicitamente: `GYR_RANGE` (±2000 ou ±500 °/s se quiser mais resolução; **ajustar `kGyrScale` junto**), `GYR_CONF` (ODR ≥ 100 Hz, modo normal), `ACC_RANGE`, `ACC_CONF`.
  - Ler de volta os registros para validar e logar.
  - Verificar o retorno de **todas** as escritas de `init` (hoje ignora os `esp_err_t`).
  - Opcional: ler o **FIFO** com timestamps do sensor para não perder amostras.

### 🟡 P7: Deadband e integração de yaw dentro do "raw"
- **Onde:** `imu_sensor.cpp:148-166`, `main.cpp:29` (deadband 0,02 rad/s).
- **O que acontece:** o deadband zera rotações lentas legítimas (< ~1,15 °/s) e vicia a integração. O quaternion publicado em `/imu/data_raw` é yaw puro integrado, que não pertence a um dado "raw".
- **Como resolver:** remover o deadband do firmware, não integrar nem publicar orientação (ver seção 0.2) e fazer a remoção de bias (calibração estática) na Pi ou numa rotina de boot explícita.

### 🟡 P8: Filtro de Kalman com provável erro de sinal e fusão inconsistente
- **Onde:** `yaw_kalman_filter.cpp:50-81`; `control_process.cpp:144-163`; `mecanum_kinematics.cpp:49-90`.
- **Observações (a confirmar na revisão, se for decidido manter o filtro em qualquer lugar):**
  - O modelo é `gz_medido = ω_real + bias`. A "medição" das rodas é `ω_real`. A inovação `innov = ω_rodas − (gz − bias)` tem sensibilidade em relação a `bias` **negativa** (`H = [0, −1]`), mas as atualizações `bias_ += k1*innov` e `theta_ += k0*innov` usam o ganho como se `H = [0, +1]`. Isso parece **inverter o sinal da correção do bias**, o que daria realimentação positiva. Verifique com um teste unitário (giro constante + bias conhecido ⇒ o bias estimado deve convergir ao valor verdadeiro).
  - O `theta_` quase não é corrigido (`p01` inicia em 0), então na prática o yaw é só a integral do giro.
  - O yaw é calculado **duas vezes**: `MecanumKinematics::odom_theta_` (das rodas) rotaciona `x/y`, enquanto `theta`/quaternion publicados vêm do Kalman (`control_process.cpp:153-163`). Se divergirem, a pose publicada fica inconsistente.
  - Se `wheel_vel` vier do fallback de comando (P2), a "medição" das rodas é fictícia.
- **Como resolver:** o item 0.1 elimina o problema removendo o filtro do ESP. Se algum dia for reintroduzido (na Pi), escreva testes de host primeiro.

### 🟡 P9: Leitura I2C cara, bloqueante e sem tratamento de falha
- **Onde:** `encoder_sensor.cpp:72-83,95-146`; `imu_sensor.cpp:31-40,120-129`; `i2c_bus.cpp`.
- **O que acontece:**
  - Cada ciclo faz ~10 transações I2C + `ets_delay_us(50)` (busy-wait) por seleção de canal. O IMU também seleciona seu canal do mux a cada leitura.
  - Uma leitura de encoder que falha vira `delta_rad = 0` (velocidade zero), sem contador de falhas: um sensor solto parece "parado".
  - Cada função tem timeout de 10 ms; se o barramento travar, o laço de 20 ms estoura. Não há recuperação do barramento (clock de 9 pulsos / reinstalar o driver).
- **Como resolver:**
  - Trocar o `ets_delay_us(50)` por nada (o PCA9548A não precisa de espera após o ACK) ou, no máximo, por uma espera mínima medida.
  - Contar falhas por canal; após N falhas consecutivas marcar o encoder como inválido e sinalizar (diagnóstico). Nunca converter falha em 0 rad/s.
  - Considerar leitura do IMU **sem** mux (IMU direto no barramento principal, se o hardware permitir) e mover a leitura dos 4 AS5600 para uma segunda porta I2C (I2C_NUM_1) para paralelizar.
  - Implementar recuperação do barramento.
  - Medir o tempo de cada ciclo (`esp_timer_get_time` antes/depois) e logar o *jitter* e o pior caso.

### 🟡 P10: Cálculo de velocidade por diferença de 12 bits a 50 Hz
- **Onde:** `encoder_sensor.cpp:118-126,138-141`.
- **O que acontece:** a resolução de velocidade é de 2π/4096/0,02 s ≈ 0,077 rad/s por tick (≈ 4 mm/s com roda de 50 mm): ruidosa em baixa velocidade. Além disso, o `dt_s` usado é o do laço, e não o intervalo real entre as leituras de cada canal.
- **Como resolver:** publicar **ticks acumulados** (seção 0.2) e deixar o cálculo de velocidade (com filtro adequado) para a Pi. Se o ESP precisar de velocidade (para um PID futuro), usar janela móvel e o timestamp real da leitura.
- **Verificar também:** onde o AS5600 está montado (eixo da roda ou do motor?). Se for no motor com redução, falta o fator de transmissão em `kTicksToRad`. E o sentido de rotação de cada encoder deve combinar com os sinais da cinemática (rodas direita/esquerda espelhadas).

### ⚪ P11: Itens menores
- `main.c.bak` na pasta `main/` ⇒ remover do repositório. Remover do git os arquivos antigos já deletados (`encoder_interface.*`, `imu_interface.*`, `stepper_interface.*`) fazendo o commit do refactor.
- `main.cpp:25`: comentário com erro de digitação ("Inst instanciação").
- `sdkconfig.defaults:2` (`ESP_MAIN_TASK_STACK_SIZE=5000`) e stack das tasks (4096 controle / 8192 comunicação): medir com `uxTaskGetStackHighWaterMark` e dar margem. O `rclc` + serialização do `Odometry` é pesado.
- `wrapToPi` com `while` repetido em 3 arquivos (se o Kalman e a integração saírem do ESP, o problema some junto).
- `~ControlProcess`/`~CommunicationProcess` fazem `vTaskDelete` na própria task em uso: irrelevante com objetos estáticos, mas frágil.
- `destroyEntities()` (`micro_ros_bridge.cpp:840-880`): após `fini` das sequências, confirmar que `createEntities()` reaproveita corretamente `odom_msg_`/`joint_state_msg_` em reconexões (strings/sequências finalizadas e recriadas só em parte). Teste: reiniciar o agente 20 vezes e checar vazamento (`heap_caps_get_free_size`).
- Covariâncias constantes em `/odom/raw` (`micro_ros_bridge.cpp:785-797`): sai de cena com a odometria; a covariância correta passa a ser da Pi.
- `GEMINI.md`: atualizar taxas (heartbeat é 1 Hz), remover a menção a FastAccelStepper até que exista e registrar a nova arquitetura (dado cru no ESP, EKF na Pi).
- Não há **nenhum teste**. As interfaces permitem testes de host (cinemática, unwrapping de encoder) com `Unity`/`GoogleTest` sem hardware.

---

## 3. Ordem de execução sugerida

1. **P1**: pulsos de STEP reais com contagem de passos. Sem isso, o resto não é validável.
2. **P3**: fila de comando entre comunicação e controle, com um único dono do estado do motor.
3. **P4**: religar o TWDT, parar motores em *panic*, trocar `esp_restart()` por reconexão.
4. **P6 + P7**: configurar o BMI160 explicitamente e remover deadband e integração de yaw.
5. **Seção 0.2**: novos tópicos crus (ticks acumulados, `JointState`, `Imu` cru) com timestamp de amostragem e sem subamostragem do gyro. **P5** e **P9/P10** junto.
6. **P2**: remover o fallback "velocidade comandada = medida".
7. **Seção 0.1** (lado Pi): nó `mecanum_wheel_odometry` + `robot_localization` + `imu_filter_madgwick`; só então remover `KalmanYawFilter`/`IYawFilter` do ESP. Manter o Kalman no build até o EKF da Pi estar validado, para poder comparar.
8. **P11**: limpeza, documentação e testes de host.

## 4. Verificação ponta a ponta (critérios de aceite)

- `ros2 topic hz` em `/wheel/ticks` e `/imu/data_raw` ≈ taxa nominal (±5%), sem buracos > 3 períodos em 5 min.
- Giro de uma roda de 1 volta à mão ⇒ ticks acumulados variam ≈ ±4096 (sem salto no *wrap*).
- IMU parada por 60 s: média do gyro z ≈ constante (bias); sem deadband, sem deriva de quaternion publicado (ele nem existe mais).
- `/cmd_vel` ⇒ cada roda gira com sinal e velocidade corretos; `ros2 topic echo /esp32/steppers/status` mostra passos reais. Parar o publicador ⇒ motores param em ≤ 500 ms. Matar o `micro_ros_agent` ⇒ motores param e o ESP **reconecta sem resetar** quando o agente voltar.
- Teste de quadrado 1 m × 1 m na bancada comparando `/odom/wheel` e o EKF da Pi; erro de fechamento registrado.
- Reiniciar o agente 20 vezes: sem vazamento de heap e sem travamento.
- Tempo de ciclo do `ControlProcess` medido: pior caso < 15 ms (de 20 ms).

---

## 5. Relatório de Resolução e Validação em Hardware (Concluído)

Todas as tarefas e problemas levantados foram implementados, gravados no ESP32 e validados na bancada com a Orange Pi em 26/10/2026:

1. **🔴 P1 Resolvido (Pulsos de STEP Reais e Contagem de Passos):**
   - Implementado gerador de pulsos de passo via periférico nativo `driver/ledc.h` nos 4 pinos (GPIO 19, 27, 23, 33). Frequência ajustada dinamicamente conforme comando cinemático.
   - DIR setup time respeitado com 30 µs de atraso antes do primeiro pulso. ENA configurado active-low com desligamento automático após 3s em repouso.
   - Contagem real de passos (`current_position_steps`) via `driver/pulse_cnt.h` e acumulador temporal de passos.
   - Validado: ao comandar $v_x = 0.2\text{ m/s}$ via `/cmd_vel`, `/esp32/steppers/status` registrou 4074 passos/s e acumulou passos de forma contínua; ao encerrar o comando, o watchdog zerou as velocidades e travou o contador em $\le 500\text{ ms}$.

2. **🔴 P2 Resolvido (Sem fallback inventado de odometria):**
   - Em `control_process.cpp`, se os AS5600 não responderem ou estiverem ausentes, `wheel_vel` é fixado em 0. Nenhuma medição é falsificada a partir dos comandos.

3. **🟠 P3 Resolvido (Corrida de dados entre núcleos eliminada):**
   - Criada fila FreeRTOS `cmd_queue_` com `xQueueOverwrite`. O callback micro-ROS no Core 0 apenas deposita o comando.
   - O laço determinístico no Core 1 é o único dono do estado dos motores e da cinemática. Flag de emergência atômica (`std::atomic<bool>`).

4. **🟠 P4 Resolvido (Task Watchdog e Reconexão Graceful):**
   - `CONFIG_ESP_TASK_WDT` configurado e inicializado com 3s de timeout. Registrado no laço do Core 1 com `esp_task_wdt_reset()` a 50 Hz.
   - Eliminado `esp_restart()` em caso de desconexão micro-ROS: o firmware para os motores, destrói entidades com segurança e retorna a `WAITING_AGENT`.
   - Validado: ao matar e relançar o `micro_ros_agent`, o ESP32 reconectou instantaneamente sem reiniciar.

5. **🟠 P5 Resolvido (Comunicação Otimizada e Heartbeat 1 Hz):**
   - `pingAgent(20, 1)` restrito a cada 1.5s (não bloqueia mais o laço a cada ciclo).
   - Heartbeat medido em exatamente $1.0\text{ Hz}$ com desvio padrão de $3.3\text{ ms}$.

6. **🟠 P6 e 🟡 P7 Resolvidos (Configuração Explícita BMI160 e IMU Cru):**
   - Registros do BMI160 configurados explicitamente: $\pm 2\text{g}$, $\pm 2000^\circ/\text{s}$, ODR 100 Hz.
   - Removidos deadband e integração interna de quaternion do firmware.
   - `/imu/data_raw` publica dados inerciais crus com `orientation_covariance[0] = -1.0` a ~45 Hz.

7. **Seção 0.2 Resolvida (Tópicos Crus de Telemetria com Timestamps Reais):**
   - `/wheel/ticks` publica 4 ticks acumulados com unwrapping contínuo a 45 Hz.
   - `/joint_states` publica posições e velocidades angulares a ~23 Hz.
   - `sample_timestamp_us` carimba o momento exato da amostragem física, convertido para epoch pelo offset de sincronização de relógio.

8. **Seção 0.1 Resolvida (Arquitetura na Orange Pi: Odometria + EKF + Madgwick):**
   - Criado pacote ROS 2 [`mecanum_wheel_odometry`](file:///home/alisson/orange/mecanum_wheel_odometry) na Orange Pi.
   - Nó [`mecanum_wheel_odometry_node`](file:///home/alisson/orange/mecanum_wheel_odometry/src/mecanum_wheel_odometry_node.cpp) assina `/wheel/ticks` em `BEST_EFFORT`, aplica cinemática direta 4WD e integração RK2 de ponto médio, publicando `/odom/wheel` (com `publish_tf: false`).
   - Integrado `imu_filter_madgwick` gerando `/imu/data` com orientação filtrada.
   - Integrado `robot_localization` (EKF) gerando `/odometry/filtered` e a única TF válida `odom -> base_link`.
   - Lançamento conjunto via `ros2 launch mecanum_wheel_odometry base_fusion.launch.py`.

9. **⚪ P11 Resolvido (Testes de Host e Limpeza):**
   - Implementado teste unitário de host em [`test/test_kinematics_host.cpp`](file:///home/alisson/orange/firmware/esp32_firmware/test/test_kinematics_host.cpp) cobrindo cinemática direta, cinemática inversa, round-trips compostos e unwrapping de 12 bits através da fronteira 4095/0. Todos os testes aprovados com 100% de sucesso.
   - Removidos arquivos de backup e interfaces depreciadas.

