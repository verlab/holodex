#pragma once

#include "RobotTypes.hpp"

#include <Arduino.h>

extern "C" {
#include <zenoh-pico.h>
}

struct MotorDTO {
    int32_t motorA = 0;
    int32_t motorB = 0;
    int32_t motorC = 0;
    int32_t motorD = 0;
} __attribute__((packed));

/**
 * Zenoh bridge — four comparable odometry streams:
 *   holodex/cmd_vel              (sub) JSON twist
 *   holodex/odom                 (pub) wheels+enc+IMU  (primary)
 *   holodex/odom/wheels          (pub) stepper rates, yaw from wheels
 *   holodex/odom/wheels_enc      (pub) encoder blend, yaw from wheels
 *   holodex/odom/imu             (pub) inertial only, no wheel data
 *   demo/pc_to_esp               (sub) raw motor Hz
 *
 * Legacy aliases also published:
 *   holodex/odom/stepper  → same as wheels
 *   holodex/odom/encoder  → same as wheels_enc
 */
class ZenohStateManager {
public:
    using CmdCallback = void (*)(const Twist2D& twist, void* ctx);
    using MotorCallback = void (*)(const MotorDTO& motors, void* ctx);

    explicit ZenohStateManager(const char* router_endpoint);
    ~ZenohStateManager();

    bool isReady() const { return zenoh_ready_; }

    void setCommandCallback(CmdCallback cb, void* ctx) {
        cmd_cb_ = cb;
        cmd_ctx_ = ctx;
    }

    void setMotorCallback(MotorCallback cb, void* ctx) {
        motor_cb_ = cb;
        motor_ctx_ = ctx;
    }

    void publishOdometry(const OdometryState& state);             // full
    void publishOdometryWheels(const OdometryState& state);       // wheels only
    void publishOdometryWheelsEnc(const OdometryState& state);    // wheels+enc
    void publishOdometryImu(const OdometryState& state);          // IMU only
    // Back-compat names
    void publishOdometryStepper(const OdometryState& state) { publishOdometryWheels(state); }
    void publishOdometryEncoder(const OdometryState& state) { publishOdometryWheelsEnc(state); }
    void loop();

private:
    static void staticCmdHandler(z_loaned_sample_t* sample, void* arg);
    static void staticMotorHandler(z_loaned_sample_t* sample, void* arg);
    void onCmdVel(const z_loaned_sample_t* sample);
    void onMotorDto(const z_loaned_sample_t* sample);
    void publishNamed(z_owned_publisher_t& pub, const OdometryState& state, const char* source);
    bool declarePub(z_owned_publisher_t& pub, const char* ke, const char* label);

    z_owned_session_t session_{};
    z_owned_publisher_t odom_pub_{};
    z_owned_publisher_t odom_wheels_pub_{};
    z_owned_publisher_t odom_wenc_pub_{};
    z_owned_publisher_t odom_imu_pub_{};
    z_owned_subscriber_t cmd_sub_{};
    z_owned_subscriber_t motor_sub_{};
    bool zenoh_ready_ = false;
    bool session_opened_ = false;
    CmdCallback cmd_cb_ = nullptr;
    void* cmd_ctx_ = nullptr;
    MotorCallback motor_cb_ = nullptr;
    void* motor_ctx_ = nullptr;
};
