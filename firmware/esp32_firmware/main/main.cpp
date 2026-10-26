#include "esp_log.h"

// Interfaces
#include "interfaces/i_motor_controller.hpp"
#include "interfaces/i_encoder_sensor.hpp"
#include "interfaces/i_imu_sensor.hpp"
#include "interfaces/i_yaw_filter.hpp"
#include "interfaces/i_ros_bridge.hpp"

// Implementações de hardware e algoritmos
#include "i2c_bus.hpp"
#include "motor_controller.hpp"
#include "encoder_sensor.hpp"
#include "imu_sensor.hpp"
#include "yaw_kalman_filter.hpp"
#include "mecanum_kinematics.hpp"
#include "micro_ros_bridge.hpp"

// Processos dedicados
#include "control_process.hpp"
#include "communication_process.hpp"

static const char* TAG = "MainOrchestrator";

// 1. Instanciação das implementações de hardware e sensores
static holodex::I2cBus& i2c_bus = holodex::I2cBus::getInstance();
static holodex::StepDirMotorController motor_controller(6400.0f, 25.0f, 50.0f);
static holodex::As5600MuxEncoder encoder_sensor(i2c_bus);
static holodex::Bmi160ImuSensor imu_sensor(i2c_bus);
// Geometria medida do chassi Holodex: Raio=0.05m (50mm), Track=0.39m (2*ly), Wheelbase=0.38m (2*lx)
static holodex::MecanumKinematics kinematics(0.050f, 0.390f, 0.380f);
static holodex::KalmanYawFilter yaw_filter;
static holodex::MicroRosBridge& ros_bridge = holodex::MicroRosBridge::getInstance();

// 2. Instanciação dos processos com injeção de dependência via interfaces
static holodex::ControlProcess control_process(
    motor_controller,
    encoder_sensor,
    imu_sensor,
    kinematics,
    yaw_filter
);

static holodex::CommunicationProcess communication_process(
    ros_bridge,
    control_process
);

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== HoloDex Robot Base Firmware Orchestrator ===");

    // 1. Inicialização do barramento I2C compartilhado (SDA=21, SCL=5, 400 kHz)
    i2c_bus.init(GPIO_NUM_21, GPIO_NUM_5, 400000);

    // 2. Inicialização dos atuadores, sensores, filtros e transporte serial
    motor_controller.init();
    encoder_sensor.init();
    imu_sensor.init();
    yaw_filter.reset(0.0f);
    ros_bridge.initTransport(0);

    // 3. Inicialização dos processos de controle e comunicação
    control_process.init();
    communication_process.init();

    // 4. Disparo dos processos em núcleos separados
    // Core 1: Processo de controle de malha fechada (50 Hz, prioridade 10)
    control_process.start(/*core_id=*/1, /*priority=*/10, /*stack_size=*/4096);

    // Core 0: Processo de comunicação micro-ROS XRCE-DDS (prioridade 5)
    communication_process.start(/*core_id=*/0, /*priority=*/5, /*stack_size=*/8192);

    ESP_LOGI(TAG, "Orquestracao concluida. Threads ativas nos Cores 0 e 1.");
}
