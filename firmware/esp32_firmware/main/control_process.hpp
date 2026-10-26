#ifndef CONTROL_PROCESS_HPP
#define CONTROL_PROCESS_HPP

#include <stdint.h>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "robot_types.hpp"
#include "interfaces/i_motor_controller.hpp"
#include "interfaces/i_encoder_sensor.hpp"
#include "interfaces/i_imu_sensor.hpp"
#include "interfaces/i_yaw_filter.hpp"
#include "mecanum_kinematics.hpp"

namespace holodex {

struct RobotStateSnapshot {
    int64_t sample_timestamp_us;
    OdomState odom;
    ImuState imu;
    EncoderTicks accumulated_ticks;
    WheelPositions wheel_positions;
    WheelVelocities wheel_velocities;
    StepperGroupStatus stepper_status;
    bool encoders_valid;
    bool imu_valid;
};

class ControlProcess {
public:
    ControlProcess(IMotorController& motor_ctrl,
                   IEncoderSensor& encoder_sensor,
                   IImuSensor& imu_sensor,
                   MecanumKinematics& kinematics,
                   IYawFilter& yaw_filter,
                   float max_wheel_accel_rad_s2 = 50.0f);
    ~ControlProcess();

    // Inicialização do processo e dos recursos internos
    bool init();

    // Dispara a tarefa em tempo real no núcleo especificado (Core 1)
    bool start(BaseType_t core_id = 1, UBaseType_t priority = 5, uint32_t stack_size = 4096);

    // Método não-bloqueante thread-safe chamado pelo processo de comunicação quando chega novo /cmd_vel
    void setTargetTwist(const TwistCommand& cmd);

    // Método thread-safe para obtenção do snapshot mais recente de telemetria
    bool getSnapshot(RobotStateSnapshot& out_snapshot, TickType_t wait_ticks = 2);

    // Solicitação atômica de parada imediata
    void triggerEmergencyStop();

private:
    static void taskEntry(void* arg);
    void run();

    IMotorController& motor_ctrl_;
    IEncoderSensor& encoder_sensor_;
    IImuSensor& imu_sensor_;
    MecanumKinematics& kinematics_;
    IYawFilter& yaw_filter_;
    float max_wheel_accel_rad_s2_;

    TaskHandle_t task_handle_;
    SemaphoreHandle_t mutex_;
    QueueHandle_t cmd_queue_;
    std::atomic<bool> emergency_stop_requested_;

    int64_t last_cmd_timestamp_us_;
    TwistCommand target_twist_;
    WheelVelocities ramped_wheel_vel_;
    RobotStateSnapshot state_snapshot_;
};

} // namespace holodex

#endif // CONTROL_PROCESS_HPP
