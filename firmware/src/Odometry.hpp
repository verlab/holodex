#pragma once

#include "HolonomicKinematics.hpp"
#include "RobotConfig.hpp"
#include "RobotTypes.hpp"
#include "YawKalman.hpp"
#include <cmath>

/**
 * Dead-reckoning from measured wheel step rates (SI) + IMU yaw fusion.
 *
 *   - translation (vx, vy) from wheel FK (encoder-preferred rates upstream)
 *   - yaw: optional 2-state Kalman [theta, gyro_bias] when gyro present;
 *     otherwise integrate wheel omega
 *   - light EMA on body twist to reject encoder quantization noise
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
     * @param step_hz  measured wheel rates in logical IK frame (step/s)
     * @param gyro_z   rad/s about +z
     * @param gyro_valid true if gyro_z is usable
     */
    const OdometryState& update(const float step_hz[4], float dt_s, float gyro_z,
                                bool gyro_valid, uint32_t stamp_ms) {
        if (dt_s <= 0.0f) {
            return state_;
        }

        for (int i = 0; i < 4; ++i) {
            state_.wheel_hz[i] = step_hz[i];
        }

        const Twist2D wheel = kin_.forwardSi(step_hz);

        // EMA on translation (encoder quantization / I2C jitter)
        constexpr float a_xy = 0.35f;
        filt_.vx += a_xy * (wheel.vx - filt_.vx);
        filt_.vy += a_xy * (wheel.vy - filt_.vy);

        Twist2D body;
        body.vx = filt_.vx;
        body.vy = filt_.vy;

        if (gyro_valid && !isnan(gyro_z)) {
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
                fusion_ = "enc+gyroKF";
            } else {
                body.omega = gyro_z;
                state_.pose.theta = wrapPi(state_.pose.theta + body.omega * dt_s);
                fusion_ = "enc+gyro";
            }
        } else {
            body.omega = wheel.omega;
            filt_.omega += a_xy * (wheel.omega - filt_.omega);
            body.omega = filt_.omega;
            state_.pose.theta = wrapPi(state_.pose.theta + body.omega * dt_s);
            if (yaw_kf_.ready()) {
                yaw_kf_.setTheta(state_.pose.theta);
            }
            fusion_ = have_gyro_ ? "wheels(gyro stale)" : "wheels";
        }

        state_.twist = body;

        // Integrate translation in world frame using (possibly KF) heading
        const float th = state_.pose.theta;
        const float c = cosf(th);
        const float s = sinf(th);
        // When KF owns theta, translation still uses body twist × dt at current heading
        // Mid-point: use heading before omega integration for non-KF path already applied;
        // for KF path theta is post-predict — use previous heading for xy:
        // (already advanced theta — use body rates with current th; small error at 50 Hz)
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
};
