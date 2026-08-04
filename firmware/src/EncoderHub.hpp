#pragma once

#include "As5600.hpp"
#include "Pca9548a.hpp"
#include "RobotConfig.hpp"
#include <cmath>

/**
 * 4× AS5600 on PCA9548A channels 0..3.
 *
 * Config maps mux channel → motor (enc_ch[Mi]) and shaft polarity (enc_sign).
 * Call sample() once per control tick; odometry/UI read motor-indexed cache.
 */
class EncoderHub {
public:
    EncoderHub(Pca9548a& mux, RobotConfig& cfg, TwoWire& wire = Wire)
        : mux_(mux),
          cfg_(cfg),
          wire_(wire),
          enc_{As5600(wire), As5600(wire), As5600(wire), As5600(wire)} {}

    bool begin() {
        ch_ok_count_ = 0;
        for (int ch = 0; ch < 4; ++ch) {
            ch_ok_[ch] = false;
            if (!mux_.select(static_cast<uint8_t>(ch))) {
                Serial.printf("[ENC] mux ch%u select failed\n", ch);
                continue;
            }
            if (!enc_[ch].begin()) {
                Serial.printf("[ENC] AS5600 not found on mux ch%u\n", ch);
                continue;
            }
            enc_[ch].syncPrev();
            ch_ok_[ch] = true;
            ch_ok_count_++;
            Serial.printf("[ENC] AS5600 OK on mux ch%u\n", ch);
        }
        last_us_ = micros();
        logMap();
        return ch_ok_count_ > 0;
    }

    void logMap() const {
        Serial.printf("[ENC] map M0←ch%u%s  M1←ch%u%s  M2←ch%u%s  M3←ch%u%s\n",
                      static_cast<unsigned>(cfg_.enc_ch[0]), cfg_.enc_sign[0] < 0 ? "-" : "+",
                      static_cast<unsigned>(cfg_.enc_ch[1]), cfg_.enc_sign[1] < 0 ? "-" : "+",
                      static_cast<unsigned>(cfg_.enc_ch[2]), cfg_.enc_sign[2] < 0 ? "-" : "+",
                      static_cast<unsigned>(cfg_.enc_ch[3]), cfg_.enc_sign[3] < 0 ? "-" : "+");
    }

    int okCount() const {
        int n = 0;
        for (int m = 0; m < 4; ++m) {
            if (motorOk(m)) {
                n++;
            }
        }
        return n;
    }
    int channelsOk() const { return ch_ok_count_; }
    bool channelOk(int ch) const { return ch >= 0 && ch < 4 && ch_ok_[ch]; }

    /** True if motor Mi has a live AS5600 on its configured mux channel. */
    bool motorOk(int motor) const {
        if (motor < 0 || motor > 3) {
            return false;
        }
        const int ch = RobotConfig::clampEncCh(cfg_.enc_ch[motor]);
        return ch_ok_[ch];
    }

    bool fresh() const { return fresh_; }
    uint32_t lastSampleUs() const { return last_us_; }

    /**
     * Read all mux channels that are mapped to at least one motor.
     * Fills motor-indexed angle_rad_ / enc_hz_ (logical IK frame).
     */
    bool sample() {
        const uint32_t now = micros();
        float dt = (now - last_us_) * 1e-6f;
        if (dt < 1e-4f) {
            dt = 1e-4f;
        } else if (dt > 0.1f) {
            for (int ch = 0; ch < 4; ++ch) {
                if (!ch_ok_[ch]) {
                    continue;
                }
                if (!mux_.select(static_cast<uint8_t>(ch))) {
                    continue;
                }
                float ang = 0.0f;
                if (enc_[ch].readAngleRad(ang)) {
                    ch_angle_[ch] = ang;
                    enc_[ch].syncPrev();
                }
            }
            float z[4] = {};
            publishMotorsFromChannels(z, /*rates=*/false);
            last_us_ = now;
            fresh_ = false;
            return false;
        }
        last_us_ = now;

        bool any_ch = false;
        float ch_hz[4] = {};
        for (int ch = 0; ch < 4; ++ch) {
            if (!ch_ok_[ch]) {
                continue;
            }
            // Skip unused channels to save I2C time
            if (!channelMapped(ch)) {
                continue;
            }
            if (!mux_.select(static_cast<uint8_t>(ch))) {
                continue;
            }
            float ang = 0.0f;
            if (!enc_[ch].readAngleRad(ang)) {
                continue;
            }
            ch_angle_[ch] = ang;
            const float dtheta = enc_[ch].consumeDeltaRad();
            ch_hz[ch] = (dtheta / dt) / (2.0f * static_cast<float>(M_PI)) * cfg_.steps_per_rev;
            any_ch = true;
        }

        publishMotorsFromChannels(ch_hz, /*rates=*/true);
        fresh_ = any_ch && okCount() > 0;
        return fresh_;
    }

    float angleRad(int motor) const {
        return (motor >= 0 && motor < 4) ? angle_rad_[motor] : NAN;
    }
    float angleDeg(int motor) const {
        const float r = angleRad(motor);
        return isnan(r) ? NAN : (r * 180.0f / static_cast<float>(M_PI));
    }
    float lastHz(int motor) const {
        return (motor >= 0 && motor < 4) ? enc_hz_[motor] : 0.0f;
    }
    int muxChannel(int motor) const {
        return (motor >= 0 && motor < 4) ? RobotConfig::clampEncCh(cfg_.enc_ch[motor]) : -1;
    }

private:
    Pca9548a& mux_;
    RobotConfig& cfg_;
    TwoWire& wire_;
    As5600 enc_[4];
    bool ch_ok_[4] = {};
    int ch_ok_count_ = 0;
    bool fresh_ = false;
    float ch_angle_[4] = {NAN, NAN, NAN, NAN};
    float angle_rad_[4] = {NAN, NAN, NAN, NAN};
    float enc_hz_[4] = {};
    uint32_t last_us_ = 0;

    bool channelMapped(int ch) const {
        for (int m = 0; m < 4; ++m) {
            if (RobotConfig::clampEncCh(cfg_.enc_ch[m]) == ch) {
                return true;
            }
        }
        return false;
    }

    void publishMotorsFromChannels(const float ch_hz[4], bool rates) {
        for (int m = 0; m < 4; ++m) {
            const int ch = RobotConfig::clampEncCh(cfg_.enc_ch[m]);
            if (!ch_ok_[ch]) {
                angle_rad_[m] = NAN;
                enc_hz_[m] = 0.0f;
                continue;
            }
            angle_rad_[m] = ch_angle_[ch];
            if (rates) {
                float hz = ch_hz[ch];
                if (cfg_.enc_sign[m] < 0) {
                    hz = -hz;
                }
                enc_hz_[m] = hz;
            }
        }
    }
};
