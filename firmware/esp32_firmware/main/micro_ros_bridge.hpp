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

#include "interfaces/i_ros_bridge.hpp"

namespace holodex {

class MicroRosBridge : public IRosBridge {
public:
    static MicroRosBridge& getInstance();

    // Inicialização da camada de transporte serial XRCE-DDS
    bool initTransport(size_t uart_port = 0) override;

    // Criação dos nós, publishers, subscribers e alocação estática dos buffers ROS 2
    bool createEntities() override;

    // Liberação controlada de recursos e buffers
    void destroyEntities() override;

    // Processamento de mensagens recebidas pelo executor (sem bloqueio prolongado)
    void spinSome(uint32_t timeout_ms = 10) override;

    // Sincronização temporal de época com o agente ROS 2
    bool syncTime(uint32_t timeout_ms = 100) override;

    // Teste de conexão ativa com o agente
    bool pingAgent(uint32_t timeout_ms = 100, uint8_t attempts = 1) override;

    // Publicadores de Telemetria
    bool publishOdometry(const OdomState& state, int64_t sample_timestamp_us = 0) override;
    bool publishImu(const ImuState& state, int64_t sample_timestamp_us = 0) override;
    bool publishJointStates(const WheelPositions& positions, const WheelVelocities& velocities, int64_t sample_timestamp_us = 0) override;
    bool publishWheelTicks(const EncoderTicks& ticks, int64_t sample_timestamp_us) override;
    bool publishStepperStatus(const StepperGroupStatus& status) override;
    bool publishHeartbeat(int32_t counter) override;

    // Registro de callbacks
    void setTwistCallback(TwistCommandCallback cb) override { twist_cb_ = cb; }
    void setLedCallback(LedCommandCallback cb) override { led_cb_ = cb; }

    bool isEntitiesCreated() const override { return entities_created_; }
    bool isTimeSynchronized() const override { return time_synchronized_; }

    MicroRosBridge();
    virtual ~MicroRosBridge();

private:
    void fillTimestamp(builtin_interfaces__msg__Time& stamp, int64_t sample_timestamp_us = 0);

    // Callbacks estáticos para integração com rclc
    static void cmdVelCallback(const void* msgin);
    static void ledCallback(const void* msgin);

    static MicroRosBridge* s_instance_;

    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;
    rclc_executor_t executor_;

    // Publishers
    rcl_publisher_t odom_pub_;
    rcl_publisher_t imu_pub_;
    rcl_publisher_t joint_state_pub_;
    rcl_publisher_t wheel_ticks_pub_;
    rcl_publisher_t stepper_status_pub_;
    rcl_publisher_t heartbeat_pub_;

    // Subscribers
    rcl_subscription_t cmd_vel_sub_;
    rcl_subscription_t led_sub_;

    // Mensagens pré-alocadas
    nav_msgs__msg__Odometry odom_msg_;
    sensor_msgs__msg__Imu imu_msg_;
    sensor_msgs__msg__JointState joint_state_msg_;
    std_msgs__msg__Int32MultiArray wheel_ticks_msg_;
    std_msgs__msg__Int32MultiArray stepper_status_msg_;
    std_msgs__msg__Int32 heartbeat_msg_;
    geometry_msgs__msg__Twist cmd_vel_msg_;
    std_msgs__msg__Bool led_msg_;

    TwistCommandCallback twist_cb_;
    LedCommandCallback led_cb_;

    bool entities_created_;
    bool time_synchronized_;
};

} // namespace holodex

#endif // MICRO_ROS_BRIDGE_HPP
