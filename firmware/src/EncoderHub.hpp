#pragma once

#include "As5600.hpp"
#include "Pca9548a.hpp"
#include "RobotConfig.hpp"
#include <cmath>

/**
 * 4× AS5600 on PCA9548A channels 0..3.
 *
 * Call sample() once per control tick; odometry/UI read cached values.
 * Leaves the mux on the last encoder channel (IMU selects ch4 next).
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
            if (!enc_[i].begin()) {
                Serial.printf("[ENC] M%d AS5600 not found on mux ch%u\n", i, kMuxCh[i]);
                continue;
            }
            enc_[i].syncPrev();
            enc_ok_[i] = true;
            ok_count_++;
            Serial.printf("[ENC] M%d AS5600 OK (ch%u)\n", i, kMuxCh[i]);
        }
        last_us_ = micros();
        return ok_count_ > 0;
    }

    int okCount() const { return ok_count_; }
    bool motorOk(int i) const { return i >= 0 && i < 4 && enc_ok_[i]; }
    bool fresh() const { return fresh_; }
    uint32_t lastSampleUs() const { return last_us_; }

    /**
     * Read all live encoders once. Updates angle_rad_ / enc_hz_ cache.
     * @return true if at least one encoder produced a sample.
     */
    bool sample() {
        const uint32_t now = micros();
        float dt = (now - last_us_) * 1e-6f;
        if (dt < 1e-4f) {
            dt = 1e-4f;
        } else if (dt > 0.1f) {
            // First sample / stall — sync unwrap, skip rate this tick
            for (int i = 0; i < 4; ++i) {
                if (enc_ok_[i] && mux_.select(kMuxCh[i])) {
                    float ang = 0.0f;
                    if (enc_[i].readAngleRad(ang)) {
                        angle_rad_[i] = ang;
                        enc_[i].syncPrev();
                    }
                }
            }
            last_us_ = now;
            fresh_ = false;
            return false;
        }
        last_us_ = now;

        bool any = false;
        for (int i = 0; i < 4; ++i) {
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
            const float omega = dtheta / dt;
            float hz = (omega / (2.0f * static_cast<float>(M_PI))) * cfg_.steps_per_rev;
            if (cfg_.motor_sign[i] < 0) {
                hz = -hz;
            }
            enc_hz_[i] = hz;
            any = true;
        }
        fresh_ = any;
        return any;
    }

    float angleRad(int i) const {
        return (i >= 0 && i < 4) ? angle_rad_[i] : NAN;
    }
    float angleDeg(int i) const {
        const float r = angleRad(i);
        return isnan(r) ? NAN : (r * 180.0f / static_cast<float>(M_PI));
    }
    float lastHz(int i) const { return (i >= 0 && i < 4) ? enc_hz_[i] : 0.0f; }

private:
    Pca9548a& mux_;
    RobotConfig& cfg_;
    TwoWire& wire_;
    As5600 enc_[4];
    bool enc_ok_[4] = {};
    int ok_count_ = 0;
    bool fresh_ = false;
    float angle_rad_[4] = {NAN, NAN, NAN, NAN};
    float enc_hz_[4] = {};
    uint32_t last_us_ = 0;
};
