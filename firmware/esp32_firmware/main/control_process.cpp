#include "control_process.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

static const char* TAG = "ControlProcess";
static const int64_t CMD_TIMEOUT_US = 500000ULL; // 500 ms watchdog

namespace holodex {

ControlProcess::ControlProcess(IMotorController& motor_ctrl,
                               IEncoderSensor& encoder_sensor,
                               IImuSensor& imu_sensor,
                               MecanumKinematics& kinematics,
                               IYawFilter& yaw_filter,
                               float max_wheel_accel_rad_s2)
    : motor_ctrl_(motor_ctrl),
      encoder_sensor_(encoder_sensor),
      imu_sensor_(imu_sensor),
      kinematics_(kinematics),
      yaw_filter_(yaw_filter),
      max_wheel_accel_rad_s2_(max_wheel_accel_rad_s2),
      task_handle_(nullptr),
      mutex_(nullptr),
      cmd_queue_(nullptr),
      emergency_stop_requested_(false),
      last_cmd_timestamp_us_(0)
{
    memset(&state_snapshot_, 0, sizeof(state_snapshot_));
}

ControlProcess::~ControlProcess()
{
    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }
    if (mutex_ != nullptr) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
    if (cmd_queue_ != nullptr) {
        vQueueDelete(cmd_queue_);
        cmd_queue_ = nullptr;
    }
}

bool ControlProcess::init()
{
    ESP_LOGI(TAG, "Inicializando ControlProcess...");
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == nullptr) {
        ESP_LOGE(TAG, "Falha ao criar mutex do ControlProcess");
        return false;
    }

    cmd_queue_ = xQueueCreate(1, sizeof(TwistCommand));
    if (cmd_queue_ == nullptr) {
        ESP_LOGE(TAG, "Falha ao criar fila de comandos /cmd_vel");
        return false;
    }

    // O TWDT é inicializado pelo sistema (CONFIG_ESP_TASK_WDT_INIT/PANIC no sdkconfig.defaults);
    // aqui a task de controle só se registra nele em run().
    emergency_stop_requested_.store(false);
    return true;
}

bool ControlProcess::start(BaseType_t core_id, UBaseType_t priority, uint32_t stack_size)
{
    ESP_LOGI(TAG, "Iniciando task de controle deterministico no Core %d (Prioridade %d)...",
             (int)core_id, (int)priority);

    BaseType_t ret = xTaskCreatePinnedToCore(
        &ControlProcess::taskEntry,
        "control_task",
        stack_size,
        this,
        priority,
        &task_handle_,
        core_id
    );
    return (ret == pdPASS);
}

void ControlProcess::taskEntry(void* arg)
{
    ControlProcess* self = static_cast<ControlProcess*>(arg);
    self->run();
}

void ControlProcess::setTargetTwist(const TwistCommand& cmd)
{
    // Entrega não-bloqueante na fila (P3: único dono no Core 1)
    if (cmd_queue_ != nullptr) {
        xQueueOverwrite(cmd_queue_, &cmd);
    }
}

void ControlProcess::triggerEmergencyStop()
{
    emergency_stop_requested_.store(true);
}

bool ControlProcess::getSnapshot(RobotStateSnapshot& out_snapshot, TickType_t wait_ticks)
{
    if (mutex_ == nullptr) return false;
    if (xSemaphoreTake(mutex_, wait_ticks) == pdTRUE) {
        out_snapshot = state_snapshot_;
        xSemaphoreGive(mutex_);
        return true;
    }
    return false;
}

void ControlProcess::run()
{
    ESP_LOGI(TAG, "Loop do ControlProcess em execucao a 50 Hz.");

    // Registra task no Task Watchdog do ESP-IDF (P4)
    esp_task_wdt_add(NULL);

    const TickType_t period_ticks = pdMS_TO_TICKS(20); // 20 ms = 50 Hz
    TickType_t last_wake_time = xTaskGetTickCount();
    int64_t last_time_us = esp_timer_get_time();

    while (1) {
        vTaskDelayUntil(&last_wake_time, period_ticks);

        // Alimenta o Task Watchdog (P4)
        esp_task_wdt_reset();

        int64_t cycle_start_us = esp_timer_get_time();
        int64_t now_us = cycle_start_us;
        float dt_s = (float)(now_us - last_time_us) * 1e-6f;
        last_time_us = now_us;
        if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.02f;

        // 1. Processamento de parada de emergência
        if (emergency_stop_requested_.exchange(false)) {
            target_twist_ = TwistCommand(0.0f, 0.0f, 0.0f);
            ramped_wheel_vel_ = WheelVelocities(0.0f, 0.0f, 0.0f, 0.0f);
            motor_ctrl_.emergencyStop();
            last_cmd_timestamp_us_ = 0;
        }

        // 2. Recepção de novos comandos de velocidade da fila
        TwistCommand new_cmd;
        if (cmd_queue_ != nullptr && xQueueReceive(cmd_queue_, &new_cmd, 0) == pdTRUE) {
            target_twist_ = new_cmd;
            last_cmd_timestamp_us_ = now_us;
        }

        // 3. Watchdog de segurança de comandos (500 ms)
        if (last_cmd_timestamp_us_ > 0 && (now_us - last_cmd_timestamp_us_) > CMD_TIMEOUT_US) {
            target_twist_ = TwistCommand(0.0f, 0.0f, 0.0f);
        }

        // 4. Cinemática inversa e rampa de aceleração (P1)
        WheelVelocities target_wheel_vel = kinematics_.inverseKinematics(target_twist_);
        float max_delta = max_wheel_accel_rad_s2_ * dt_s;

        ramped_wheel_vel_.fl += std::clamp(target_wheel_vel.fl - ramped_wheel_vel_.fl, -max_delta, max_delta);
        ramped_wheel_vel_.fr += std::clamp(target_wheel_vel.fr - ramped_wheel_vel_.fr, -max_delta, max_delta);
        ramped_wheel_vel_.rl += std::clamp(target_wheel_vel.rl - ramped_wheel_vel_.rl, -max_delta, max_delta);
        ramped_wheel_vel_.rr += std::clamp(target_wheel_vel.rr - ramped_wheel_vel_.rr, -max_delta, max_delta);

        motor_ctrl_.setWheelVelocities(ramped_wheel_vel_);

        // 5. Timestamp exato da amostragem física (Seção 0.2)
        int64_t sample_timestamp_us = esp_timer_get_time();

        // 6. Leitura dos encoders das rodas
        WheelPositions wheel_pos;
        WheelVelocities wheel_vel;
        bool enc_ok = encoder_sensor_.update(dt_s, wheel_pos, wheel_vel);

        // P2: Se os encoders não responderem, NÃO inventa medida da odometria!
        if (!enc_ok || encoder_sensor_.getActiveSensorsCount() == 0) {
            wheel_vel = WheelVelocities(0.0f, 0.0f, 0.0f, 0.0f);
        }

        // 7. Amostragem do sensor inercial (IMU)
        ImuState imu_st;
        bool imu_ok = imu_sensor_.update(dt_s, imu_st);

        // 8. Odometria dead-reckoning (a fusão de yaw fica no EKF da Orange Pi)
        OdomState odom_st;
        kinematics_.updateOdometry(wheel_vel, dt_s, odom_st);

        // 9. Leitura do status dos motores de passo
        StepperGroupStatus stepper_st;
        motor_ctrl_.getGroupStatus(stepper_st);

        // 10. Publicação atômica do snapshot
        if (mutex_ != nullptr && xSemaphoreTake(mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
            state_snapshot_.sample_timestamp_us = sample_timestamp_us;
            state_snapshot_.odom = odom_st;
            state_snapshot_.imu = imu_st;
            state_snapshot_.accumulated_ticks = encoder_sensor_.readTicks();
            state_snapshot_.wheel_positions = wheel_pos;
            state_snapshot_.wheel_velocities = wheel_vel;
            state_snapshot_.stepper_status = stepper_st;
            state_snapshot_.encoders_valid = enc_ok && (encoder_sensor_.getActiveSensorsCount() > 0);
            state_snapshot_.imu_valid = imu_ok;
            xSemaphoreGive(mutex_);
        }

        int64_t cycle_duration_us = esp_timer_get_time() - cycle_start_us;
        if (cycle_duration_us > 18000) {
            ESP_LOGW(TAG, "Jitter alto no ciclo de controle: %lld us", (long long)cycle_duration_us);
        }
    }
}

} // namespace holodex
