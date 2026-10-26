#ifndef MICRO_ROS_BRIDGE_HPP
#define MICRO_ROS_BRIDGE_HPP

#include <stdint.h>
#include <stdbool.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

#include "robot_types.hpp"

namespace holodex {

// Assinaturas para callbacks de comando
typedef void (*TwistCommandCallback)(const TwistCommand& cmd);
typedef void (*LedCommandCallback)(bool state);

class MicroRosBridge {
public:
    static MicroRosBridge& getInstance();

    // Inicialização da camada de transporte serial XRCE-DDS
    bool initTransport(size_t uart_port = 0);

    // Criação dos nós, publishers, subscribers e alocação estática dos buffers ROS 2
    bool createEntities();

    // Liberação controlada de recursos e buffers
    void destroyEntities();

    // Processamento de mensagens recebidas pelo executor (sem bloqueio prolongado)
    void spinSome(uint32_t timeout_ms = 10);

    // Sincronização temporal de época com o agente ROS 2
    bool syncTime(uint32_t timeout_ms = 100);

    // Teste de conexão ativa com o agente
    bool pingAgent(uint32_t timeout_ms = 100, uint8_t attempts = 1);

    // Publicadores de Telemetria
    bool publishOdometry(const OdomState& state);
    bool publishImu(const ImuState& state);
    bool publishJointStates(const WheelPositions& positions, const WheelVelocities& velocities);
    bool publishStepperStatus(const StepperGroupStatus& status);
    bool publishHeartbeat(int32_t counter);

    // Registro de callbacks
    void setTwistCallback(TwistCommandCallback cb) { twist_cb_ = cb; }
    void setLedCallback(LedCommandCallback cb) { led_cb_ = cb; }

    bool isEntitiesCreated() const { return entities_created_; }
    bool isTimeSynchronized() const { return time_synchronized_; }

private:
    MicroRosBridge();
    ~MicroRosBridge();

    MicroRosBridge(const MicroRosBridge&) = delete;
    MicroRosBridge& operator=(const MicroRosBridge&) = delete;

    // Callbacks estáticos invocados pelo rclc_executor
    static void cmdVelCallback(const void* msgin);
    static void ledCallback(const void* msgin);

    // Preenchimento de timestamp ROS 2 (header.stamp)
    void fillTimestamp(builtin_interfaces__msg__Time& stamp);

    // micro-ROS rcl & rclc handles
    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;
    rclc_executor_t executor_;

    // Publishers
    rcl_publisher_t odom_pub_;
    rcl_publisher_t imu_pub_;
    rcl_publisher_t joint_state_pub_;
    rcl_publisher_t stepper_status_pub_;
    rcl_publisher_t heartbeat_pub_;

    // Subscribers
    rcl_subscription_t cmd_vel_sub_;
    rcl_subscription_t led_sub_;

    // Mensagens estáticas pré-alocadas
    nav_msgs__msg__Odometry odom_msg_;
    sensor_msgs__msg__Imu imu_msg_;
    sensor_msgs__msg__JointState joint_state_msg_;
    std_msgs__msg__Int32MultiArray stepper_status_msg_;
    std_msgs__msg__Int32 heartbeat_msg_;
    geometry_msgs__msg__Twist cmd_vel_msg_;
    std_msgs__msg__Bool led_msg_;

    // Callbacks de usuário
    TwistCommandCallback twist_cb_;
    LedCommandCallback led_cb_;

    bool entities_created_;
    bool time_synchronized_;
};

} // namespace holodex

#endif // MICRO_ROS_BRIDGE_HPP
