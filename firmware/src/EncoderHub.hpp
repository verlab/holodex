#pragma once

#include "As5600.hpp"
#include "Pca9548a.hpp"
#include "RobotConfig.hpp"
#include <cmath>

/**
 * 4× AS5600 on PCA9548A channels 0..3.
 *
 * Converts magnet angle rate → equivalent wheel step Hz for odometry
 * (same sign convention as logical IK / motor_sign).
 */
class EncoderHub {
public:
    static constexpr uint8_t kMuxCh[4] = {0, 1, 2, 3};

    EncoderHub(Pca9548a& mux, RobotConfig& cfg, TwoWire& wire = Wire)
        : mux_(mux),
          cfg_(cfg),
          wire_(wire),
          enc_{As5600(wire), As5600(wire), As5600(wire), As5600(wire)} {}

    bool begin() {
        ok_count_ = 0;
        for (int i = 0; i < 4; ++i) {
            enc_ok_[i] = false;
            if (!mux_.select(kMuxCh[i])) {
                Serial.printf("[ENC] M%d mux select failed\n", i);
                continue;
            }
            // Fresh channel: AS5600 is always 0x36 — mux avoids address clash.
            if (!enc_[i].begin()) {
                Serial.printf("[ENC] M%d AS5600 not found on mux ch%u\n", i, kMuxCh[i]);
                continue;
            }
            enc_[i].syncPrev();
            enc_ok_[i] = true;
            ok_count_++;
            Serial.printf("[ENC] M%d AS5600 OK (ch%u)\n", i, kMuxCh[i]);
        }
        mux_.disableAll();
        last_ms_ = millis();
        return ok_count_ > 0;
    }

    int okCount() const { return ok_count_; }
    bool motorOk(int i) const { return i >= 0 && i < 4 && enc_ok_[i]; }

    /**
     * Read all encoders; fill step-Hz equivalents (logical frame).
     * Returns true if at least one encoder produced a sample.
     */
    bool update(float step_hz_out[4]) {
        const uint32_t now = millis();
        float dt = (now - last_ms_) * 0.001f;
        if (dt < 1e-4f) {
            dt = 1e-4f;
        }
        last_ms_ = now;

        bool any = false;
        for (int i = 0; i < 4; ++i) {
            step_hz_out[i] = 0.0f;
            angle_rad_[i] = NAN;
            if (!enc_ok_[i]) {
                continue;
            }
            if (!mux_.select(kMuxCh[i])) {
                continue;
            }
            float ang = 0.0f;
            if (!enc_[i].readAngleRad(ang)) {
                continue;
            }
            angle_rad_[i] = ang;
            const float dtheta = enc_[i].consumeDeltaRad();
            // Wheel shaft ω (rad/s) → step Hz
            const float omega = dtheta / dt;
            float hz = (omega / (2.0f * static_cast<float>(M_PI))) * cfg_.steps_per_rev;
            if (cfg_.motor_sign[i] < 0) {
                hz = -hz;
            }
            // Encoder may be mounted opposite to motor DIR — invert via NVS later if needed
            step_hz_out[i] = hz;
            enc_hz_[i] = hz;
            any = true;
        }
        mux_.disableAll();
        return any;
    }

    float angleRad(int i) const {
        return (i >= 0 && i < 4) ? angle_rad_[i] : NAN;
    }
    float lastHz(int i) const { return (i >= 0 && i < 4) ? enc_hz_[i] : 0.0f; }

private:
    Pca9548a& mux_;
    RobotConfig& cfg_;
    TwoWire& wire_;
    As5600 enc_[4];
    bool enc_ok_[4] = {};
    int ok_count_ = 0;
    float angle_rad_[4] = {NAN, NAN, NAN, NAN};
    float enc_hz_[4] = {};
    uint32_t last_ms_ = 0;
};
