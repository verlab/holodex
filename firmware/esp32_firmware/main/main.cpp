#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "robot_types.hpp"
#include "mecanum_kinematics.hpp"
#include "stepper_interface.hpp"
#include "encoder_interface.hpp"
#include "imu_interface.hpp"
#include "micro_ros_bridge.hpp"

static const char* TAG = "MainApp";

#define LED_PIN GPIO_NUM_2
#define CMD_VEL_TIMEOUT_US (500000ULL) // 500 ms watchdog de segurança

// Instâncias globais dos subsistemas
static holodex::MecanumKinematics kinematics(0.048f, 0.220f, 0.180f);
static holodex::StepperInterface stepper_interface(3200.0f, 25.0f, 50.0f);
static holodex::EncoderInterface encoder_interface(1440.0f);
static holodex::ImuInterface imu_interface;

// Dados compartilhados entre as tarefas (protegidos por mutex)
static SemaphoreHandle_t data_mutex = NULL;
static holodex::OdomState shared_odom;
static holodex::ImuState shared_imu;
static holodex::WheelPositions shared_positions;
static holodex::WheelVelocities shared_velocities;
static holodex::StepperGroupStatus shared_stepper_status;

// Variáveis de controle de comandos
static int64_t last_cmd_vel_timestamp = 0;
static bool led_override = false;
static bool led_state = false;

// Callbacks de recepção do micro-ROS
static void onTwistCommand(const holodex::TwistCommand& cmd)
{
    last_cmd_vel_timestamp = esp_timer_get_time();

    // Cinemática inversa Mecanum -> velocidades angulares das 4 rodas
    holodex::WheelVelocities target_wheels = kinematics.inverseKinematics(cmd);

    // Envio aos motores de passo
    stepper_interface.setWheelVelocities(target_wheels);
}

static void onLedCommand(bool state)
{
    led_override = true;
    led_state = state;
    gpio_set_level(LED_PIN, state ? 1 : 0);
}

// Tarefa de controle de malha fechada e integração cinemática (50 Hz)
void control_loop_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Iniciando tarefa de controle em tempo real (50 Hz)...");

    const TickType_t period_ticks = pdMS_TO_TICKS(20); // 20 ms = 50 Hz
    TickType_t last_wake_time = xTaskGetTickCount();
    int64_t last_time_us = esp_timer_get_time();

    while (1) {
        vTaskDelayUntil(&last_wake_time, period_ticks);

        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_time_us) / 1000000.0f;
        last_time_us = now_us;
        if (dt <= 0.0f || dt > 0.1f) dt = 0.02f;

        // 1. Watchdog de segurança para /cmd_vel
        if (last_cmd_vel_timestamp > 0 && (now_us - last_cmd_vel_timestamp) > CMD_VEL_TIMEOUT_US) {
            stepper_interface.emergencyStop();
            last_cmd_vel_timestamp = 0; // Evita paradas repetitivas
        }

        // 2. Leitura e atualização dos encoders das rodas
        holodex::WheelPositions wheel_pos;
        holodex::WheelVelocities wheel_vel;
        encoder_interface.update(dt, wheel_pos, wheel_vel);

        // Se os encoders estiverem zerados (bancada sem encoders físicos conectados),
        // utiliza a estimativa de velocidade gerada pelo perfil de aceleração dos steppers
        if (wheel_vel.fl == 0.0f && wheel_vel.fr == 0.0f &&
            wheel_vel.rl == 0.0f && wheel_vel.rr == 0.0f) {
            wheel_vel = stepper_interface.getEstimatedWheelVelocities();
        }

        // 3. Leitura e filtro da IMU
        holodex::ImuState imu_st;
        imu_interface.update(dt, imu_st);

        // 4. Integração da odometria cinemática
        holodex::OdomState odom_st;
        kinematics.updateOdometry(wheel_vel, dt, odom_st);

        // 5. Coleta do status dos motores de passo
        holodex::StepperGroupStatus step_st;
        stepper_interface.getGroupStatus(step_st);

        // 6. Atualização atômica das estruturas compartilhadas
        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            shared_odom = odom_st;
            shared_imu = imu_st;
            shared_positions = wheel_pos;
            shared_velocities = wheel_vel;
            shared_stepper_status = step_st;
            xSemaphoreGive(data_mutex);
        }
    }
}

// Tarefa de comunicação micro-ROS (XRCE-DDS)
void micro_ros_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Iniciando tarefa micro-ROS XRCE-DDS...");

    holodex::MicroRosBridge& bridge = holodex::MicroRosBridge::getInstance();
    bridge.setTwistCallback(onTwistCommand);
    bridge.setLedCallback(onLedCommand);

    typedef enum {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    } agent_state_t;

    agent_state_t state = WAITING_AGENT;
    int failed_pings = 0;
    int32_t heartbeat_counter = 0;

    int64_t last_odom_pub_us = 0;
    int64_t last_imu_pub_us = 0;
    int64_t last_joint_pub_us = 0;
    int64_t last_stepper_pub_us = 0;
    int64_t last_heartbeat_pub_us = 0;
    int64_t last_time_sync_us = 0;

    const int64_t INTERVAL_ODOM_US    = 33333;  // ~30 Hz
    const int64_t INTERVAL_IMU_US     = 33333;  // ~30 Hz
    const int64_t INTERVAL_JOINT_US   = 50000;  // ~20 Hz
    const int64_t INTERVAL_STEPPER_US = 100000; // ~10 Hz
    const int64_t INTERVAL_HEARTBEAT_US = 1000000; // 1 Hz
    const int64_t INTERVAL_SYNC_US    = 5000000; // 5 segundos

    while (1) {
        switch (state) {
            case WAITING_AGENT:
                // Pisca rápido indicando aguardo do Agente
                if (!led_override) {
                    gpio_set_level(LED_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(80));
                    gpio_set_level(LED_PIN, 0);
                    vTaskDelay(pdMS_TO_TICKS(420));
                } else {
                    vTaskDelay(pdMS_TO_TICKS(500));
                }

                if (bridge.pingAgent(100, 2)) {
                    state = AGENT_AVAILABLE;
                }
                break;

            case AGENT_AVAILABLE:
                if (bridge.createEntities()) {
                    state = AGENT_CONNECTED;
                    failed_pings = 0;
                    heartbeat_counter = 0;

                    // Sincronização inicial de tempo
                    bridge.syncTime(200);

                    // Pulso duplo indicando conexão bem-sucedida
                    if (!led_override) {
                        gpio_set_level(LED_PIN, 1);
                        vTaskDelay(pdMS_TO_TICKS(150));
                        gpio_set_level(LED_PIN, 0);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        gpio_set_level(LED_PIN, 1);
                        vTaskDelay(pdMS_TO_TICKS(150));
                        gpio_set_level(LED_PIN, 0);
                    }
                } else {
                    bridge.destroyEntities();
                    state = WAITING_AGENT;
                }
                break;

            case AGENT_CONNECTED: {
                int64_t now_us = esp_timer_get_time();

                // Cópia local dos dados para publicação
                holodex::OdomState local_odom;
                holodex::ImuState local_imu;
                holodex::WheelPositions local_positions;
                holodex::WheelVelocities local_velocities;
                holodex::StepperGroupStatus local_steppers;

                if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                    local_odom = shared_odom;
                    local_imu = shared_imu;
                    local_positions = shared_positions;
                    local_velocities = shared_velocities;
                    local_steppers = shared_stepper_status;
                    xSemaphoreGive(data_mutex);
                }

                // Publicação de Odometria (~30 Hz)
                if (now_us - last_odom_pub_us >= INTERVAL_ODOM_US) {
                    bridge.publishOdometry(local_odom);
                    last_odom_pub_us = now_us;
                }

                // Publicação de IMU (~30 Hz)
                if (now_us - last_imu_pub_us >= INTERVAL_IMU_US) {
                    bridge.publishImu(local_imu);
                    last_imu_pub_us = now_us;
                }

                // Publicação de Joint States (~20 Hz)
                if (now_us - last_joint_pub_us >= INTERVAL_JOINT_US) {
                    bridge.publishJointStates(local_positions, local_velocities);
                    last_joint_pub_us = now_us;
                }

                // Publicação de Status dos Steppers (~10 Hz)
                if (now_us - last_stepper_pub_us >= INTERVAL_STEPPER_US) {
                    bridge.publishStepperStatus(local_steppers);
                    last_stepper_pub_us = now_us;
                }

                // Publicação de Heartbeat (1 Hz)
                if (now_us - last_heartbeat_pub_us >= INTERVAL_HEARTBEAT_US) {
                    bridge.publishHeartbeat(heartbeat_counter++);
                    last_heartbeat_pub_us = now_us;

                    if (!led_override) {
                        gpio_set_level(LED_PIN, 1);
                        vTaskDelay(pdMS_TO_TICKS(30));
                        gpio_set_level(LED_PIN, 0);
                    }
                }

                // Sincronização periódica de relógio (a cada 5 s)
                if (now_us - last_time_sync_us >= INTERVAL_SYNC_US) {
                    bridge.syncTime(50);
                    last_time_sync_us = now_us;
                }

                // Processa mensagens de entrada (/cmd_vel, /esp32/led)
                bridge.spinSome(5);

                // Verificação de conectividade do agente
                if (!bridge.pingAgent(100, 1)) {
                    failed_pings++;
                    if (failed_pings >= 5) {
                        state = AGENT_DISCONNECTED;
                    }
                } else {
                    failed_pings = 0;
                }
                break;
            }

            case AGENT_DISCONNECTED:
                ESP_LOGW(TAG, "Agente micro-ROS desconectado! Reiniciando conexao...");
                stepper_interface.emergencyStop();
                led_override = false;
                gpio_set_level(LED_PIN, 0);
                bridge.destroyEntities();
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_restart();
                break;
        }
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando Holodex Mecanum Base Firmware...");

    // Inicialização do mutex de dados
    data_mutex = xSemaphoreCreateMutex();

    // Configuração do pino de LED onboard
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_PIN, 0);

    // Inicialização dos subsistemas de hardware
    stepper_interface.init();
    encoder_interface.init();
    imu_interface.init();

    // Inicialização da camada de transporte serial XRCE-DDS (UART0)
    holodex::MicroRosBridge& bridge = holodex::MicroRosBridge::getInstance();
    bridge.initTransport(0);

    // Tarefa de controle de malha fechada no Core 1 (prioridade 10, alta)
    xTaskCreatePinnedToCore(
        control_loop_task,
        "ctrl_loop_task",
        4096,
        NULL,
        10,
        NULL,
        1
    );

    // Tarefa micro-ROS no Core 0 (prioridade 5, 8KB stack)
    xTaskCreatePinnedToCore(
        micro_ros_task,
        "uros_task",
        8192,
        NULL,
        5,
        NULL,
        0
    );
}
