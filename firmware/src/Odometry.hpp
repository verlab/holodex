#pragma once

#include "HolonomicKinematics.hpp"
#include "RobotTypes.hpp"
#include <cmath>

/**
 * Dead-reckoning from measured wheel step rates (SI).
 *
 * Fusion (simple, robust when IMU is missing):
 *   - translation (vx, vy) always from wheel forward kinematics
 *   - yaw rate from gyro when fresh; otherwise from wheels
 */
class Odometry {
public:
    explicit Odometry(HolonomicKinematics& kin) : kin_(kin) {}

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
    }

    /**
     * @param step_hz  measured wheel rates in logical IK frame (step/s)
     * @param gyro_z   rad/s about +z, or NAN if no fresh sample this tick
     * @param gyro_valid true if gyro_z is usable (held or just read)
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
        Twist2D body;
        body.vx = wheel.vx;
        body.vy = wheel.vy;

        if (gyro_valid && !isnan(gyro_z)) {
            last_gyro_z_ = gyro_z;
            have_gyro_ = true;
            body.omega = gyro_z;  // rad/s — IMU is authoritative for yaw
        } else if (have_gyro_) {
            // Stale gyro: fall back to wheels (do not keep integrating old gyro forever)
            body.omega = wheel.omega;
        } else {
            body.omega = wheel.omega;
        }

        state_.twist = body;

        const float th = state_.pose.theta;
        const float c = cosf(th);
        const float s = sinf(th);
        state_.pose.x += (c * body.vx - s * body.vy) * dt_s;
        state_.pose.y += (s * body.vx + c * body.vy) * dt_s;
        state_.pose.theta = wrapPi(state_.pose.theta + body.omega * dt_s);
        state_.stamp_ms = stamp_ms;
        return state_;
    }

    const OdometryState& state() const { return state_; }

    void clearGyro() { have_gyro_ = false; }

private:
    HolonomicKinematics& kin_;
    OdometryState state_{};
    bool have_gyro_ = false;
    float last_gyro_z_ = 0.0f;

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
