#pragma once

#include "HolonomicKinematics.hpp"
#include "RobotConfig.hpp"
#include "RobotTypes.hpp"
#include "YawKalman.hpp"
#include <cmath>

/**
 * Dead-reckoning from wheel step rates (SI).
 *
 * Yaw modes (for fair article comparisons):
 *   use_imu=false → integrate wheel FK ω only
 *   use_imu=true  → gyro (+ optional 2-state Kalman bias)
 */
class Odometry {
public:
    Odometry(HolonomicKinematics& kin, RobotConfig& cfg) : kin_(kin), cfg_(cfg) {}

    void reset(float x = 0.0f, float y = 0.0f, float theta = 0.0f) {
        state_.pose.x = x;
        state_.pose.y = y;
        state_.pose.theta = theta;
        state_.twist = {};
        for (int i = 0; i < 4; ++i) {
            state_.wheel_hz[i] = 0.0f;
        }
        have_gyro_ = false;
        last_gyro_z_ = 0.0f;
        filt_ = {};
        yaw_kf_.reset(theta);
        fusion_ = "wheels";
    }

    /**
     * @param step_hz   wheel rates in logical IK frame (step/s)
     * @param use_imu   if false, ignore gyro entirely (wheels-only yaw)
     * @param label     short tag stored in fusionMode() for UI/telemetry
     */
    const OdometryState& update(const float step_hz[4], float dt_s, float gyro_z,
                                bool gyro_valid, uint32_t stamp_ms, bool use_imu,
                                const char* label) {
        if (dt_s <= 0.0f) {
            return state_;
        }

        for (int i = 0; i < 4; ++i) {
            state_.wheel_hz[i] = step_hz[i];
        }

        const Twist2D wheel = kin_.forwardSi(step_hz);

        constexpr float a_xy = 0.35f;
        filt_.vx += a_xy * (wheel.vx - filt_.vx);
        filt_.vy += a_xy * (wheel.vy - filt_.vy);
        filt_.omega += a_xy * (wheel.omega - filt_.omega);

        Twist2D body;
        body.vx = filt_.vx;
        body.vy = filt_.vy;

        const bool imu_ok = use_imu && gyro_valid && !isnan(gyro_z);
        if (imu_ok) {
            // Below the deadband the gyro is reporting bias, not rotation.
            // Integrating it makes the heading crawl while the robot sits still.
            gyro_z = deadband(gyro_z, cfg_.gyro_deadband);
            last_gyro_z_ = gyro_z;
            have_gyro_ = true;
            if (cfg_.yaw_kalman) {
                if (!yaw_kf_.ready()) {
                    yaw_kf_.reset(state_.pose.theta);
                }
                yaw_kf_.predict(gyro_z, dt_s);
                yaw_kf_.updateWheelOmega(wheel.omega, gyro_z);
                body.omega = gyro_z - yaw_kf_.bias();
                state_.pose.theta = yaw_kf_.theta();
            } else {
                body.omega = gyro_z;
                state_.pose.theta = wrapPi(state_.pose.theta + body.omega * dt_s);
            }
            fusion_ = label ? label : "imu";
        } else {
            body.omega = filt_.omega;
            state_.pose.theta = wrapPi(state_.pose.theta + body.omega * dt_s);
            if (yaw_kf_.ready()) {
                yaw_kf_.setTheta(state_.pose.theta);
            }
            fusion_ = label ? label : "wheels";
        }

        state_.twist = body;

        const float th = state_.pose.theta;
        const float c = cosf(th);
        const float s = sinf(th);
        state_.pose.x += (c * body.vx - s * body.vy) * dt_s;
        state_.pose.y += (s * body.vx + c * body.vy) * dt_s;
        state_.stamp_ms = stamp_ms;
        return state_;
    }

    const OdometryState& state() const { return state_; }
    const char* fusionMode() const { return fusion_; }
    float gyroBias() const { return yaw_kf_.bias(); }

    void clearGyro() { have_gyro_ = false; }

private:
    HolonomicKinematics& kin_;
    RobotConfig& cfg_;
    OdometryState state_{};
    Twist2D filt_{};
    YawKalman yaw_kf_{};
    bool have_gyro_ = false;
    float last_gyro_z_ = 0.0f;
    const char* fusion_ = "wheels";

    static float wrapPi(float a) {
        while (a > static_cast<float>(M_PI)) {
            a -= 2.0f * static_cast<float>(M_PI);
        }
        while (a < -static_cast<float>(M_PI)) {
            a += 2.0f * static_cast<float>(M_PI);
        }
        return a;
    }

    /** Zero inside ±band, continuous outside it. */
    static float deadband(float v, float band) {
        if (band <= 0.0f) {
            return v;
        }
        if (fabsf(v) <= band) {
            return 0.0f;
        }
        return v - (v > 0.0f ? band : -band);
    }
};
