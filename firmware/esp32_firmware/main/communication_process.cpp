#include "communication_process.hpp"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_system.h"

static const char* TAG = "CommunicationProcess";

namespace holodex {

CommunicationProcess* CommunicationProcess::s_instance_ = nullptr;

CommunicationProcess::CommunicationProcess(IRosBridge& ros_bridge, ControlProcess& control_proc)
    : ros_bridge_(ros_bridge),
      control_proc_(control_proc),
      task_handle_(nullptr),
      led_override_(false)
{
    s_instance_ = this;
}

CommunicationProcess::~CommunicationProcess()
{
    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }
}

bool CommunicationProcess::init()
{
    ESP_LOGI(TAG, "Configurando GPIO do LED de status...");
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar GPIO do LED: %s", esp_err_to_name(err));
        return false;
    }

    setLed(false);

    // Registra callbacks estáticos da ponte micro-ROS
    ros_bridge_.setTwistCallback(&CommunicationProcess::onTwistReceived);
    ros_bridge_.setLedCallback(&CommunicationProcess::onLedReceived);

    return true;
}

bool CommunicationProcess::start(BaseType_t core_id, UBaseType_t priority, uint32_t stack_size)
{
    ESP_LOGI(TAG, "Iniciando task de comunicacao micro-ROS no Core %d (Prioridade %d)...",
             (int)core_id, (int)priority);

    BaseType_t ret = xTaskCreatePinnedToCore(
        &CommunicationProcess::taskEntry,
        "comm_task",
        stack_size,
        this,
        priority,
        &task_handle_,
        core_id
    );
    return (ret == pdPASS);
}

void CommunicationProcess::setLed(bool on)
{
    gpio_set_level(LED_PIN, on ? 1 : 0);
}

void CommunicationProcess::onTwistReceived(const TwistCommand& cmd)
{
    if (s_instance_) {
        // Entrega assíncrona na fila do Core 1 (P3: zero race conditions)
        s_instance_->control_proc_.setTargetTwist(cmd);
    }
}

void CommunicationProcess::onLedReceived(bool state)
{
    if (s_instance_) {
        s_instance_->led_override_.store(true);
        s_instance_->setLed(state);
    }
}

void CommunicationProcess::taskEntry(void* arg)
{
    CommunicationProcess* self = static_cast<CommunicationProcess*>(arg);
    self->run();
}

void CommunicationProcess::run()
{
    ESP_LOGI(TAG, "Maquina de estados XRCE-DDS serial iniciada.");

    enum AgentState {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    } state = WAITING_AGENT;

    int32_t heartbeat_counter = 0;
    int failed_pings = 0;

    // Ticks e IMU são publicados uma vez por amostra do controle (50 Hz): republicar a mesma
    // amostra geraria dt=0 / picos de velocidade no receptor, e pular uma perderia dado do giroscópio.
    int64_t last_ticks_sample_us = 0;
    int64_t last_imu_sample_us = 0;
    int64_t last_joint_pub_us = 0;
    int64_t last_odom_pub_us = 0;
    int64_t last_stepper_pub_us = 0;
    int64_t last_heartbeat_pub_us = 0;
    int64_t last_time_sync_us = 0;
    int64_t last_ping_time_us = 0;

    // Intervalos Nominais (Seção 0.2)
    const int64_t INTERVAL_JOINT_US     = 40000ULL;  // 25 Hz
    const int64_t INTERVAL_ODOM_US      = 40000ULL;  // 25 Hz
    const int64_t INTERVAL_STEPPER_US   = 100000ULL; // 10 Hz
    const int64_t INTERVAL_HEARTBEAT_US = 1000000ULL;// 1 Hz
    const int64_t INTERVAL_SYNC_US      = 5000000ULL;// a cada 5 s
    const int64_t INTERVAL_PING_US      = 1500000ULL;// ping a cada 1.5 s (P5)

    while (1) {
        switch (state) {
            case WAITING_AGENT:
                // Pisca suavemente indicando busca de conexão
                if (!led_override_.load()) {
                    setLed(true);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    setLed(false);
                }

                if (ros_bridge_.pingAgent(50, 1)) {
                    ESP_LOGI(TAG, "Agente micro-ROS detectado! Conectando...");
                    state = AGENT_AVAILABLE;
                } else {
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                break;

            case AGENT_AVAILABLE:
                if (ros_bridge_.createEntities()) {
                    ESP_LOGI(TAG, "Entidades e tópicos inicializados. Pronto.");
                    state = AGENT_CONNECTED;
                    failed_pings = 0;
                    heartbeat_counter = 0;

                    ros_bridge_.syncTime(200);

                    if (!led_override_.load()) {
                        setLed(true);  vTaskDelay(pdMS_TO_TICKS(150));
                        setLed(false); vTaskDelay(pdMS_TO_TICKS(100));
                        setLed(true);  vTaskDelay(pdMS_TO_TICKS(150));
                        setLed(false);
                    }
                } else {
                    ros_bridge_.destroyEntities();
                    state = WAITING_AGENT;
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
                break;

            case AGENT_CONNECTED: {
                int64_t now_us = esp_timer_get_time();

                RobotStateSnapshot snapshot;
                bool have_snapshot = control_proc_.getSnapshot(snapshot, 0);

                if (have_snapshot) {
                    // 1. Ticks Acumulados Crus (50 Hz - Seção 0.2)
                    if (snapshot.sample_timestamp_us != last_ticks_sample_us) {
                        ros_bridge_.publishWheelTicks(snapshot.accumulated_ticks, snapshot.sample_timestamp_us);
                        last_ticks_sample_us = snapshot.sample_timestamp_us;
                    }

                    // 2. IMU Cru (50 Hz - Seção 0.2)
                    if (snapshot.sample_timestamp_us != last_imu_sample_us) {
                        ros_bridge_.publishImu(snapshot.imu, snapshot.sample_timestamp_us);
                        last_imu_sample_us = snapshot.sample_timestamp_us;
                    }

                    // 3. Joint States (25 Hz)
                    if (now_us - last_joint_pub_us >= INTERVAL_JOINT_US) {
                        ros_bridge_.publishJointStates(snapshot.wheel_positions, snapshot.wheel_velocities, snapshot.sample_timestamp_us);
                        last_joint_pub_us = now_us;
                    }

                    // 4. Odometria (~25 Hz)
                    if (now_us - last_odom_pub_us >= INTERVAL_ODOM_US) {
                        ros_bridge_.publishOdometry(snapshot.odom, snapshot.sample_timestamp_us);
                        last_odom_pub_us = now_us;
                    }

                    // 5. Steppers Status (10 Hz)
                    if (now_us - last_stepper_pub_us >= INTERVAL_STEPPER_US) {
                        ros_bridge_.publishStepperStatus(snapshot.stepper_status);
                        last_stepper_pub_us = now_us;
                    }
                }

                // 6. Heartbeat (1 Hz - P5)
                if (now_us - last_heartbeat_pub_us >= INTERVAL_HEARTBEAT_US) {
                    ros_bridge_.publishHeartbeat(heartbeat_counter++);
                    last_heartbeat_pub_us = now_us;

                    if (!led_override_.load()) {
                        setLed(true);
                        vTaskDelay(pdMS_TO_TICKS(30));
                        setLed(false);
                    }
                }

                // 7. Sincronização periódica de relógio epoch (a cada 5 s)
                if (now_us - last_time_sync_us >= INTERVAL_SYNC_US) {
                    ros_bridge_.syncTime(50);
                    last_time_sync_us = now_us;
                }

                // 8. Processa assinaturas (/cmd_vel, /esp32/led)
                ros_bridge_.spinSome(5);

                // 9. Monitor não-bloqueante de conexão com o agente (a cada 1.5 s - P5)
                if (now_us - last_ping_time_us >= INTERVAL_PING_US) {
                    last_ping_time_us = now_us;
                    if (!ros_bridge_.pingAgent(20, 1)) {
                        failed_pings++;
                        if (failed_pings >= 10) { // 15 segundos sem resposta
                            state = AGENT_DISCONNECTED;
                        }
                    } else {
                        failed_pings = 0;
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(5));
                break;
            }

            case AGENT_DISCONNECTED:
                ESP_LOGW(TAG, "Agente desconectado! Parando motores e aguardando reconexao (sem reiniciar)...");
                control_proc_.triggerEmergencyStop();
                led_override_.store(false);
                setLed(false);
                ros_bridge_.destroyEntities();
                vTaskDelay(pdMS_TO_TICKS(500));
                state = WAITING_AGENT; // Reconecta limpo sem esp_restart! (P4)
                break;
        }
    }
}

} // namespace holodex
