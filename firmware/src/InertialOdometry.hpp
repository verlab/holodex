#pragma once

#include "RobotConfig.hpp"
#include "RobotTypes.hpp"
#include <cmath>

/**
 * Strapdown dead reckoning from the IMU alone.
 *
 * Heading comes from integrating gyro Z; position from double-integrating the
 * accelerometer. No wheel rate, encoder or command ever enters this class — it
 * is the inertial-only stream, useful both as a comparison baseline and as the
 * absolute heading reference for the controller.
 *
 * Double integration means bias is everything: 0.05 m/s² of uncorrected offset
 * is 2.5 cm of position error after one second and 2.5 m after ten. Two things
 * keep that in check:
 *
 *   1. Bias calibration at rest. Gyro and horizontal accelerometer offsets are
 *      averaged while the robot is still and then refined every time it stops.
 *      On a level floor this also absorbs the gravity that leaks into X/Y from
 *      a slightly tilted mount (a real ramp would not be compensated).
 *   2. Zero-velocity updates. Once the IMU has been quiet for kRestHoldMs the
 *      velocity is snapped back to zero, which stops the random walk from
 *      accumulating across the stops that dominate a teleop session.
 *
 * Rest detection is inertial only: residual acceleration small, vibration
 * small, gyro quiet. The vibration test is what separates a real stop from
 * cruising at constant velocity (where residual acceleration is also ~0) — a
 * stepper base shakes plenty while rolling. imu_rest_acc tunes that split.
 *
 * Integration uses the midpoint heading and trapezoidal velocity, which costs
 * nothing here and roughly squares the accuracy of the double integral versus
 * plain Euler.
 */
class InertialOdometry {
public:
    explicit InertialOdometry(RobotConfig& cfg) : cfg_(cfg) {}

    void reset(float x = 0.0f, float y = 0.0f, float theta = 0.0f) {
        state_ = {};
        state_.pose.x = x;
        state_.pose.y = y;
        state_.pose.theta = theta;
        vwx_ = 0.0f;
        vwy_ = 0.0f;
        at_rest_ = false;
        rest_ms_ = 0;
        yaw_rate_ = 0.0f;
        yaw_rate_raw_ = 0.0f;
        recalibrate();
    }

    /** Drop the learned offsets and re-average them (robot must be still). */
    void recalibrate() {
        calib_n_ = 0;
        calib_gz_ = 0.0f;
        calib_ax_ = 0.0f;
        calib_ay_ = 0.0f;
        calibrated_ = false;
        lp_valid_ = false;
    }

    /**
     * Fold in one IMU sample. Uses the sample's own dt, so a dropped read or a
     * jittery control loop cannot mis-scale the integration.
     */
    void update(const ImuSample& s, uint32_t stamp_ms) {
        if (!s.valid) {
            return;
        }
        state_.stamp_ms = stamp_ms;
        yaw_rate_raw_ = s.gyro_z - bias_gz_;

        // Slow average of the raw accelerometer: the reference the vibration
        // test measures against.
        if (!lp_valid_) {
            lp_ax_ = s.ax;
            lp_ay_ = s.ay;
            lp_az_ = s.az;
            lp_valid_ = true;
        } else {
            constexpr float a = 0.1f;
            lp_ax_ += a * (s.ax - lp_ax_);
            lp_ay_ += a * (s.ay - lp_ay_);
            lp_az_ += a * (s.az - lp_az_);
        }
        const float vib = fabsf(s.ax - lp_ax_) + fabsf(s.ay - lp_ay_) + fabsf(s.az - lp_az_);

        const float quiet_acc = restAccelThreshold();
        const bool quiet_vib = vib < quiet_acc;
        const bool quiet_abs = fabsf(s.ax - bias_ax_) < quiet_acc &&
                               fabsf(s.ay - bias_ay_) < quiet_acc &&
                               fabsf(s.gyro_z - bias_gz_) < restGyroThreshold();
        // Until the offsets are known only the bias-free test can be trusted:
        // a mount tilted by a couple of degrees, or a typical gyro offset, would
        // fail the absolute check forever and calibration would never finish.
        const bool quiet = calibrated_ ? (quiet_vib && quiet_abs) : quiet_vib;

        const uint32_t dt_ms =
            static_cast<uint32_t>(lroundf(s.dt_s * 1000.0f));
        if (quiet) {
            rest_ms_ += dt_ms;
        } else {
            rest_ms_ = 0;
        }
        at_rest_ = quiet && rest_ms_ >= kRestHoldMs;

        if (!calibrated_) {
            collectCalibration(s, quiet);
            return;
        }

        if (at_rest_) {
            // Standing still: kill the accumulated velocity before it walks the
            // pose away, and use the free samples to refine the offsets.
            vwx_ = 0.0f;
            vwy_ = 0.0f;
            bias_gz_ += kBiasAlphaGyro * (s.gyro_z - bias_gz_);
            bias_ax_ += kBiasAlphaAccel * (s.ax - bias_ax_);
            bias_ay_ += kBiasAlphaAccel * (s.ay - bias_ay_);
            yaw_rate_ = 0.0f;
            state_.twist = {};
            return;
        }

        if (s.dt_s <= 0.0f) {
            return;
        }

        // Bias-corrected rate, then the drift deadband: below it the gyro is
        // reporting offset, not rotation, and integrating it would creep.
        const float w = deadband(s.gyro_z - bias_gz_, cfg_.gyro_deadband);
        yaw_rate_ = w;

        const float dt = s.dt_s;
        const float th0 = state_.pose.theta;
        const float th_mid = th0 + 0.5f * w * dt;
        const float th1 = wrapPi(th0 + w * dt);

        const float ax = s.ax - bias_ax_;
        const float ay = s.ay - bias_ay_;

        // Rotate body acceleration into the world frame at the midpoint
        // heading. Integrating velocity in the world frame (rather than in the
        // body frame) is what removes the Coriolis term entirely.
        const float c = cosf(th_mid);
        const float sn = sinf(th_mid);
        const float awx = c * ax - sn * ay;
        const float awy = sn * ax + c * ay;

        const float vwx1 = vwx_ + awx * dt;
        const float vwy1 = vwy_ + awy * dt;
        state_.pose.x += 0.5f * (vwx_ + vwx1) * dt;
        state_.pose.y += 0.5f * (vwy_ + vwy1) * dt;
        vwx_ = vwx1;
        vwy_ = vwy1;
        state_.pose.theta = th1;

        // Report the twist in the body frame, like every other stream
        const float ct = cosf(th1);
        const float st = sinf(th1);
        state_.twist.vx = ct * vwx_ + st * vwy_;
        state_.twist.vy = -st * vwx_ + ct * vwy_;
        state_.twist.omega = w;
    }

    const OdometryState& state() const { return state_; }
    /** Absolute heading (rad) — the controller's yaw reference. */
    float heading() const { return state_.pose.theta; }
    /** Bias-corrected, deadbanded yaw rate (rad/s). */
    float yawRate() const { return yaw_rate_; }
    /** Bias-corrected yaw rate before the deadband, for downstream filters. */
    float yawRateRaw() const { return yaw_rate_raw_; }
    bool calibrated() const { return calibrated_; }
    bool atRest() const { return at_rest_; }
    float gyroBias() const { return bias_gz_; }
    float speed() const { return hypotf(vwx_, vwy_); }
    const char* mode() const {
        return calibrated_ ? (at_rest_ ? "imu·rest" : "imu") : "imu·calib";
    }

private:
    static constexpr uint32_t kRestHoldMs = 150;
    static constexpr int kCalibSamples = 80;  // ≈1.6 s at 50 Hz
    static constexpr float kBiasAlphaGyro = 0.02f;
    static constexpr float kBiasAlphaAccel = 0.01f;

    RobotConfig& cfg_;
    OdometryState state_{};
    float vwx_ = 0.0f;  // world-frame velocity (m/s)
    float vwy_ = 0.0f;
    float yaw_rate_ = 0.0f;
    float yaw_rate_raw_ = 0.0f;
    float bias_gz_ = 0.0f;
    float bias_ax_ = 0.0f;
    float bias_ay_ = 0.0f;
    bool calibrated_ = false;
    bool at_rest_ = false;
    uint32_t rest_ms_ = 0;
    int calib_n_ = 0;
    float calib_gz_ = 0.0f;
    float calib_ax_ = 0.0f;
    float calib_ay_ = 0.0f;
    float lp_ax_ = 0.0f;
    float lp_ay_ = 0.0f;
    float lp_az_ = 0.0f;
    bool lp_valid_ = false;

    float restAccelThreshold() const {
        return cfg_.imu_rest_acc > 0.02f ? cfg_.imu_rest_acc : 0.02f;
    }
    float restGyroThreshold() const {
        return cfg_.gyro_deadband > 0.015f ? cfg_.gyro_deadband : 0.015f;
    }

    /**
     * Average the offsets over the first still seconds. Any movement restarts
     * the average — a bias learned while rolling would be baked into every
     * later estimate.
     */
    void collectCalibration(const ImuSample& s, bool quiet) {
        if (!quiet) {
            calib_n_ = 0;
            calib_gz_ = 0.0f;
            calib_ax_ = 0.0f;
            calib_ay_ = 0.0f;
            return;
        }
        calib_gz_ += s.gyro_z;
        calib_ax_ += s.ax;
        calib_ay_ += s.ay;
        if (++calib_n_ < kCalibSamples) {
            return;
        }
        const float n = static_cast<float>(calib_n_);
        bias_gz_ = calib_gz_ / n;
        bias_ax_ = calib_ax_ / n;
        bias_ay_ = calib_ay_ / n;
        calibrated_ = true;
        vwx_ = 0.0f;
        vwy_ = 0.0f;
        Serial.printf("[IMU] bias gz=%.4f rad/s  ax=%.3f  ay=%.3f m/s²\n", bias_gz_, bias_ax_,
                      bias_ay_);
    }

    static float deadband(float v, float band) {
        if (band <= 0.0f) {
            return v;
        }
        if (fabsf(v) <= band) {
            return 0.0f;
        }
        return v - (v > 0.0f ? band : -band);
    }

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
