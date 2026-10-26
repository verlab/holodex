#include "micro_ros_bridge.hpp"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"

#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include "esp32_serial_transport.h"

static const char* TAG = "MicroRosBridge";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Falha micro-ROS na linha %d: %d", __LINE__, (int)temp_rc); return false;}}

namespace holodex {

MicroRosBridge* MicroRosBridge::s_instance_ = nullptr;

MicroRosBridge& MicroRosBridge::getInstance()
{
    static MicroRosBridge instance;
    return instance;
}

MicroRosBridge::MicroRosBridge()
    : twist_cb_(nullptr),
      led_cb_(nullptr),
      entities_created_(false),
      time_synchronized_(false)
{
    s_instance_ = this;

    memset(&allocator_, 0, sizeof(allocator_));
    memset(&support_, 0, sizeof(support_));
    memset(&node_, 0, sizeof(node_));
    memset(&executor_, 0, sizeof(executor_));

    memset(&odom_pub_, 0, sizeof(odom_pub_));
    memset(&imu_pub_, 0, sizeof(imu_pub_));
    memset(&joint_state_pub_, 0, sizeof(joint_state_pub_));
    memset(&wheel_ticks_pub_, 0, sizeof(wheel_ticks_pub_));
    memset(&stepper_status_pub_, 0, sizeof(stepper_status_pub_));
    memset(&heartbeat_pub_, 0, sizeof(heartbeat_pub_));

    memset(&cmd_vel_sub_, 0, sizeof(cmd_vel_sub_));
    memset(&led_sub_, 0, sizeof(led_sub_));

    memset(&odom_msg_, 0, sizeof(odom_msg_));
    memset(&imu_msg_, 0, sizeof(imu_msg_));
    memset(&joint_state_msg_, 0, sizeof(joint_state_msg_));
    memset(&wheel_ticks_msg_, 0, sizeof(wheel_ticks_msg_));
    memset(&stepper_status_msg_, 0, sizeof(stepper_status_msg_));
    memset(&heartbeat_msg_, 0, sizeof(heartbeat_msg_));
    memset(&cmd_vel_msg_, 0, sizeof(cmd_vel_msg_));
    memset(&led_msg_, 0, sizeof(led_msg_));
}

MicroRosBridge::~MicroRosBridge()
{
    destroyEntities();
}

bool MicroRosBridge::initTransport(size_t uart_port)
{
    ESP_LOGI(TAG, "Configurando transporte serial micro-ROS customizado (UART %u)...", (unsigned int)uart_port);

    static size_t port = uart_port;
    port = uart_port;
    rmw_uros_set_custom_transport(
        true,
        (void*)&port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read
    );

    ESP_LOGI(TAG, "Transporte serial micro-XRCE-DDS configurado.");
    return true;
}

bool MicroRosBridge::createEntities()
{
    if (entities_created_) {
        return true;
    }

    ESP_LOGI(TAG, "Criando entidades micro-ROS...");

    allocator_ = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator_));
    RCCHECK(rclc_support_init_with_options(&support_, 0, NULL, &init_options, &allocator_));

    // Nó ROS 2 principal do chassi
    RCCHECK(rclc_node_init_default(&node_, "esp32_base_node", "", &support_));

    // 1. Publisher: /odom/raw (nav_msgs/msg/Odometry)
    RCCHECK(rclc_publisher_init_best_effort(
        &odom_pub_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/raw"
    ));

    // 2. Publisher: /imu/data_raw (sensor_msgs/msg/Imu)
    RCCHECK(rclc_publisher_init_best_effort(
        &imu_pub_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data_raw"
    ));

    // 3. Publisher: /joint_states (sensor_msgs/msg/JointState)
    RCCHECK(rclc_publisher_init_best_effort(
        &joint_state_pub_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states"
    ));

    // 4. Publisher: /wheel/ticks (std_msgs/msg/Int32MultiArray) (Seção 0.2)
    RCCHECK(rclc_publisher_init_best_effort(
        &wheel_ticks_pub_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "wheel/ticks"
    ));

    // 5. Publisher: /esp32/steppers/status (std_msgs/msg/Int32MultiArray)
    RCCHECK(rclc_publisher_init_best_effort(
        &stepper_status_pub_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "esp32/steppers/status"
    ));

    // 6. Publisher: /esp32/heartbeat (std_msgs/msg/Int32)
    RCCHECK(rclc_publisher_init_best_effort(
        &heartbeat_pub_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "esp32/heartbeat"
    ));

    // 7. Subscriber: /cmd_vel (geometry_msgs/msg/Twist)
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));

    // 8. Subscriber: /esp32/led (std_msgs/msg/Bool)
    RCCHECK(rclc_subscription_init_default(
        &led_sub_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "esp32/led"
    ));

    // Inicialização do executor com 2 handles (subscriptions)
    RCCHECK(rclc_executor_init(&executor_, &support_.context, 2, &allocator_));
    RCCHECK(rclc_executor_add_subscription(
        &executor_,
        &cmd_vel_sub_,
        &cmd_vel_msg_,
        cmdVelCallback,
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_subscription(
        &executor_,
        &led_sub_,
        &led_msg_,
        ledCallback,
        ON_NEW_DATA
    ));

    // Pré-alocação e configuração estática de buffers de mensagens ROS 2
    // A. Odometria
    rosidl_runtime_c__String__assign(&odom_msg_.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&odom_msg_.child_frame_id, "base_link");
    for (int i = 0; i < 36; ++i) {
        odom_msg_.pose.covariance[i] = 0.0;
        odom_msg_.twist.covariance[i] = 0.0;
    }
    odom_msg_.pose.covariance[0] = 0.01;
    odom_msg_.pose.covariance[7] = 0.01;
    odom_msg_.pose.covariance[14] = 1e6;
    odom_msg_.pose.covariance[21] = 1e6;
    odom_msg_.pose.covariance[28] = 1e6;
    odom_msg_.pose.covariance[35] = 0.05;

    odom_msg_.twist.covariance[0] = 0.01;
    odom_msg_.twist.covariance[7] = 0.01;
    odom_msg_.twist.covariance[14] = 1e6;
    odom_msg_.twist.covariance[21] = 1e6;
    odom_msg_.twist.covariance[28] = 1e6;
    odom_msg_.twist.covariance[35] = 0.05;

    // B. IMU
    rosidl_runtime_c__String__assign(&imu_msg_.header.frame_id, "imu_link");
    for (int i = 0; i < 9; ++i) {
        imu_msg_.orientation_covariance[i] = 0.0;
        imu_msg_.angular_velocity_covariance[i] = 0.0;
        imu_msg_.linear_acceleration_covariance[i] = 0.0;
    }
    // Convenção padrão ROS: orientation_covariance[0] = -1.0 indica que o sensor não estima orientação
    imu_msg_.orientation_covariance[0] = -1.0;

    imu_msg_.angular_velocity_covariance[0] = 0.001;
    imu_msg_.angular_velocity_covariance[4] = 0.001;
    imu_msg_.angular_velocity_covariance[8] = 0.001;

    imu_msg_.linear_acceleration_covariance[0] = 0.01;
    imu_msg_.linear_acceleration_covariance[4] = 0.01;
    imu_msg_.linear_acceleration_covariance[8] = 0.01;

    // C. Joint States (4 rodas: FL, FR, RL, RR)
    rosidl_runtime_c__String__assign(&joint_state_msg_.header.frame_id, "base_link");
    rosidl_runtime_c__String__Sequence__init(&joint_state_msg_.name, 4);
    rosidl_runtime_c__String__assign(&joint_state_msg_.name.data[0], "fl_wheel_joint");
    rosidl_runtime_c__String__assign(&joint_state_msg_.name.data[1], "fr_wheel_joint");
    rosidl_runtime_c__String__assign(&joint_state_msg_.name.data[2], "rl_wheel_joint");
    rosidl_runtime_c__String__assign(&joint_state_msg_.name.data[3], "rr_wheel_joint");

    rosidl_runtime_c__double__Sequence__init(&joint_state_msg_.position, 4);
    rosidl_runtime_c__double__Sequence__init(&joint_state_msg_.velocity, 4);
    joint_state_msg_.effort.size = 0;
    joint_state_msg_.effort.capacity = 0;
    joint_state_msg_.effort.data = nullptr;

    // D. Wheel Ticks (4 contadores inteiros acumulados + timestamp da amostra em us, 32 bits)
    rosidl_runtime_c__int32__Sequence__init(&wheel_ticks_msg_.data, 5);
    wheel_ticks_msg_.data.size = 5;

    // E. Stepper Status (8 canais: 4 posições + 4 velocidades)
    rosidl_runtime_c__int32__Sequence__init(&stepper_status_msg_.data, 8);
    stepper_status_msg_.data.size = 8;

    entities_created_ = true;
    ESP_LOGI(TAG, "Todas as entidades e topicos micro-ROS foram criados com sucesso!");
    return true;
}

void MicroRosBridge::destroyEntities()
{
    if (!entities_created_) {
        return;
    }

    ESP_LOGW(TAG, "Destruindo entidades micro-ROS...");

    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support_.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // Finalização dos buffers dinâmicos ROS 2
    rosidl_runtime_c__String__fini(&odom_msg_.header.frame_id);
    rosidl_runtime_c__String__fini(&odom_msg_.child_frame_id);

    rosidl_runtime_c__String__fini(&imu_msg_.header.frame_id);

    rosidl_runtime_c__String__fini(&joint_state_msg_.header.frame_id);
    rosidl_runtime_c__String__Sequence__fini(&joint_state_msg_.name);
    rosidl_runtime_c__double__Sequence__fini(&joint_state_msg_.position);
    rosidl_runtime_c__double__Sequence__fini(&joint_state_msg_.velocity);

    rosidl_runtime_c__int32__Sequence__fini(&wheel_ticks_msg_.data);
    rosidl_runtime_c__int32__Sequence__fini(&stepper_status_msg_.data);

    // Finalização das entidades rcl
    rcl_ret_t rc_fini = RCL_RET_OK;
    rc_fini = rcl_publisher_fini(&odom_pub_, &node_); (void)rc_fini;
    rc_fini = rcl_publisher_fini(&imu_pub_, &node_); (void)rc_fini;
    rc_fini = rcl_publisher_fini(&joint_state_pub_, &node_); (void)rc_fini;
    rc_fini = rcl_publisher_fini(&wheel_ticks_pub_, &node_); (void)rc_fini;
    rc_fini = rcl_publisher_fini(&stepper_status_pub_, &node_); (void)rc_fini;
    rc_fini = rcl_publisher_fini(&heartbeat_pub_, &node_); (void)rc_fini;

    rc_fini = rcl_subscription_fini(&cmd_vel_sub_, &node_); (void)rc_fini;
    rc_fini = rcl_subscription_fini(&led_sub_, &node_); (void)rc_fini;

    rc_fini = rclc_executor_fini(&executor_); (void)rc_fini;
    rc_fini = rcl_node_fini(&node_); (void)rc_fini;
    rc_fini = rclc_support_fini(&support_); (void)rc_fini;

    memset(&odom_msg_, 0, sizeof(odom_msg_));
    memset(&imu_msg_, 0, sizeof(imu_msg_));
    memset(&joint_state_msg_, 0, sizeof(joint_state_msg_));
    memset(&wheel_ticks_msg_, 0, sizeof(wheel_ticks_msg_));
    memset(&stepper_status_msg_, 0, sizeof(stepper_status_msg_));

    entities_created_ = false;
    time_synchronized_ = false;
}

void MicroRosBridge::spinSome(uint32_t timeout_ms)
{
    if (!entities_created_) return;
    rclc_executor_spin_some(&executor_, timeout_ms * 1000000ULL);
}

bool MicroRosBridge::pingAgent(uint32_t timeout_ms, uint8_t attempts)
{
    rmw_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    return (ret == RMW_RET_OK);
}

bool MicroRosBridge::syncTime(uint32_t timeout_ms)
{
    if (!entities_created_) return false;
    rmw_ret_t ret = rmw_uros_sync_session(timeout_ms);
    time_synchronized_ = (ret == RMW_RET_OK);
    return time_synchronized_;
}

void MicroRosBridge::fillTimestamp(builtin_interfaces__msg__Time& stamp, int64_t sample_timestamp_us)
{
    int64_t now_us = esp_timer_get_time();
    int64_t time_ns = 0;

    if (time_synchronized_ && rmw_uros_epoch_synchronized()) {
        int64_t epoch_now_ns = rmw_uros_epoch_nanos();
        if (sample_timestamp_us > 0 && sample_timestamp_us <= now_us) {
            int64_t age_ns = (now_us - sample_timestamp_us) * 1000LL;
            time_ns = epoch_now_ns - age_ns;
        } else {
            time_ns = epoch_now_ns;
        }
    } else {
        int64_t target_us = (sample_timestamp_us > 0) ? sample_timestamp_us : now_us;
        time_ns = target_us * 1000LL;
    }

    stamp.sec = (int32_t)(time_ns / 1000000000LL);
    stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);
}

bool MicroRosBridge::publishOdometry(const OdomState& state, int64_t sample_timestamp_us)
{
    if (!entities_created_) return false;

    fillTimestamp(odom_msg_.header.stamp, sample_timestamp_us);

    odom_msg_.pose.pose.position.x = state.x;
    odom_msg_.pose.pose.position.y = state.y;
    odom_msg_.pose.pose.position.z = 0.0;

    odom_msg_.pose.pose.orientation.x = state.qx;
    odom_msg_.pose.pose.orientation.y = state.qy;
    odom_msg_.pose.pose.orientation.z = state.qz;
    odom_msg_.pose.pose.orientation.w = state.qw;

    odom_msg_.twist.twist.linear.x = state.vx;
    odom_msg_.twist.twist.linear.y = state.vy;
    odom_msg_.twist.twist.linear.z = 0.0;
    odom_msg_.twist.twist.angular.x = 0.0;
    odom_msg_.twist.twist.angular.y = 0.0;
    odom_msg_.twist.twist.angular.z = state.omega;

    return (rcl_publish(&odom_pub_, &odom_msg_, NULL) == RCL_RET_OK);
}

bool MicroRosBridge::publishImu(const ImuState& state, int64_t sample_timestamp_us)
{
    if (!entities_created_) return false;

    fillTimestamp(imu_msg_.header.stamp, sample_timestamp_us);

    imu_msg_.orientation.x = state.qx;
    imu_msg_.orientation.y = state.qy;
    imu_msg_.orientation.z = state.qz;
    imu_msg_.orientation.w = state.qw;

    imu_msg_.angular_velocity.x = state.gx;
    imu_msg_.angular_velocity.y = state.gy;
    imu_msg_.angular_velocity.z = state.gz;

    imu_msg_.linear_acceleration.x = state.ax;
    imu_msg_.linear_acceleration.y = state.ay;
    imu_msg_.linear_acceleration.z = state.az;

    return (rcl_publish(&imu_pub_, &imu_msg_, NULL) == RCL_RET_OK);
}

bool MicroRosBridge::publishJointStates(const WheelPositions& positions, const WheelVelocities& velocities, int64_t sample_timestamp_us)
{
    if (!entities_created_) return false;

    fillTimestamp(joint_state_msg_.header.stamp, sample_timestamp_us);

    joint_state_msg_.position.data[0] = positions.fl;
    joint_state_msg_.position.data[1] = positions.fr;
    joint_state_msg_.position.data[2] = positions.rl;
    joint_state_msg_.position.data[3] = positions.rr;

    joint_state_msg_.velocity.data[0] = velocities.fl;
    joint_state_msg_.velocity.data[1] = velocities.fr;
    joint_state_msg_.velocity.data[2] = velocities.rl;
    joint_state_msg_.velocity.data[3] = velocities.rr;

    return (rcl_publish(&joint_state_pub_, &joint_state_msg_, NULL) == RCL_RET_OK);
}

bool MicroRosBridge::publishWheelTicks(const EncoderTicks& ticks, int64_t sample_timestamp_us)
{
    if (!entities_created_) return false;

    wheel_ticks_msg_.data.data[0] = ticks.fl;
    wheel_ticks_msg_.data.data[1] = ticks.fr;
    wheel_ticks_msg_.data.data[2] = ticks.rl;
    wheel_ticks_msg_.data.data[3] = ticks.rr;
    wheel_ticks_msg_.data.data[4] = (int32_t)(uint32_t)(sample_timestamp_us & 0xFFFFFFFFLL);

    return (rcl_publish(&wheel_ticks_pub_, &wheel_ticks_msg_, NULL) == RCL_RET_OK);
}

bool MicroRosBridge::publishStepperStatus(const StepperGroupStatus& status)
{
    if (!entities_created_) return false;

    stepper_status_msg_.data.data[0] = status.fl.current_position_steps;
    stepper_status_msg_.data.data[1] = status.fr.current_position_steps;
    stepper_status_msg_.data.data[2] = status.rl.current_position_steps;
    stepper_status_msg_.data.data[3] = status.rr.current_position_steps;

    stepper_status_msg_.data.data[4] = status.fl.current_speed_steps_s;
    stepper_status_msg_.data.data[5] = status.fr.current_speed_steps_s;
    stepper_status_msg_.data.data[6] = status.rl.current_speed_steps_s;
    stepper_status_msg_.data.data[7] = status.rr.current_speed_steps_s;

    return (rcl_publish(&stepper_status_pub_, &stepper_status_msg_, NULL) == RCL_RET_OK);
}

bool MicroRosBridge::publishHeartbeat(int32_t counter)
{
    if (!entities_created_) return false;
    heartbeat_msg_.data = counter;
    return (rcl_publish(&heartbeat_pub_, &heartbeat_msg_, NULL) == RCL_RET_OK);
}

void MicroRosBridge::cmdVelCallback(const void* msgin)
{
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
    if (s_instance_ && s_instance_->twist_cb_) {
        TwistCommand cmd(msg->linear.x, msg->linear.y, msg->angular.z);
        s_instance_->twist_cb_(cmd);
    }
}

void MicroRosBridge::ledCallback(const void* msgin)
{
    const std_msgs__msg__Bool* msg = (const std_msgs__msg__Bool*)msgin;
    if (s_instance_ && s_instance_->led_cb_) {
        s_instance_->led_cb_(msg->data);
    }
}

} // namespace holodex
