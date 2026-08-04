#pragma once

#include <cmath>

/**
 * Lightweight 2-state Kalman for heading: x = [theta, gyro_bias].
 *
 * Predict with IMU gyro; update with wheel-FK yaw rate when available.
 * Cheap on ESP32 (no matrices beyond 2×2 scalar algebra).
 */
class YawKalman {
public:
    void reset(float theta = 0.0f) {
        theta_ = theta;
        bias_ = 0.0f;
        p00_ = 0.05f;
        p01_ = 0.0f;
        p11_ = 0.01f;
        init_ = true;
    }

    bool ready() const { return init_; }
    float theta() const { return theta_; }
    float bias() const { return bias_; }

    /** Predict with gyro (rad/s). */
    void predict(float gyro_z, float dt_s) {
        if (!init_ || dt_s <= 0.0f) {
            return;
        }
        const float om = gyro_z - bias_;
        theta_ = wrapPi(theta_ + om * dt_s);

        // P = F P F' + Q ; F = [[1, -dt],[0, 1]]
        const float dt = dt_s;
        const float p00 = p00_ - dt * (p01_ + p01_) + dt * dt * p11_ + q_theta_;
        const float p01 = p01_ - dt * p11_;
        const float p11 = p11_ + q_bias_;
        p00_ = p00;
        p01_ = p01;
        p11_ = p11;
    }

    /** Soft update from wheel omega (rad/s) — mainly observes bias. */
    void updateWheelOmega(float wheel_omega, float gyro_z) {
        if (!init_) {
            return;
        }
        // z = gyro - bias  ≈  wheel_omega  →  innov on bias
        const float pred = gyro_z - bias_;
        const float innov = wheel_omega - pred;
        const float s = p11_ + r_wheel_;
        if (s < 1e-9f) {
            return;
        }
        const float k0 = p01_ / s;
        const float k1 = p11_ / s;
        theta_ = wrapPi(theta_ + k0 * innov);
        bias_ += k1 * innov;
        // Joseph-lite covariance update on bias column
        p00_ -= k0 * p01_;
        p01_ -= k0 * p11_;
        p11_ -= k1 * p11_;
        if (p00_ < 1e-6f) {
            p00_ = 1e-6f;
        }
        if (p11_ < 1e-6f) {
            p11_ = 1e-6f;
        }
    }

    void setTheta(float theta) { theta_ = wrapPi(theta); }

private:
    bool init_ = false;
    float theta_ = 0.0f;
    float bias_ = 0.0f;
    float p00_ = 0.05f;
    float p01_ = 0.0f;
    float p11_ = 0.01f;
    // Tuned for ~50 Hz, BMI160 noise vs wheel FK
    static constexpr float q_theta_ = 1e-5f;
    static constexpr float q_bias_ = 1e-7f;
    static constexpr float r_wheel_ = 0.05f;

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
