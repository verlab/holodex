#pragma once

#include "HolonomicKinematics.hpp"
#include "RobotTypes.hpp"
#include <cmath>

/**
 * Dead-reckoning from commanded wheel step rates.
 * Uses SI twist estimate from kinematics (scaled maxima).
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
    }

    const OdometryState& update(const float step_hz[4], float dt_s, float gyro_z,
                                uint32_t stamp_ms) {
        if (dt_s <= 0.0f) {
            return state_;
        }

        for (int i = 0; i < 4; ++i) {
            state_.wheel_hz[i] = step_hz[i];
        }

        Twist2D body = kin_.forwardSi(step_hz);
        if (!isnan(gyro_z)) {
            constexpr float alpha = 0.98f;
            body.omega = alpha * gyro_z + (1.0f - alpha) * body.omega;
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

private:
    HolonomicKinematics& kin_;
    OdometryState state_{};

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
