#pragma once

#include "HolonomicKinematics.hpp"
#include "Odometry.hpp"
#include "RobotConfig.hpp"
#include "RobotTypes.hpp"
#include "stepper.hpp"
#include <cmath>

/**
 * Applies motor commands immediately on receive (WiFi callbacks), so teleop
 * still works when the main loop is busy with SoftAP / WebSockets.
 * FastAccelStepper runs in hardware — ESP32 is not CPU-bound by 4 steppers.
 */
class MotionController {
public:
    MotionController(RobotConfig& cfg, Stepper* motors[4], HolonomicKinematics& kin,
                     Odometry& odom)
        : cfg_(cfg), kin_(kin), odom_(odom) {
        for (int i = 0; i < 4; ++i) {
            motors_[i] = motors[i];
        }
    }

    void setWebCommand(const Twist2D& ncmd, uint32_t now_ms) {
        if (estop_) {
            return;
        }
        web_cmd_.twist = clampNorm(ncmd);
        web_cmd_.source = CmdSource::Web;
        web_cmd_.stamp_ms = now_ms;
        mode_ = Mode::Cinematic;

        float hz[4] = {};
        kin_.inverseNormalized(web_cmd_.twist, hz);
        applyWheelHz(hz);
        active_source_ = CmdSource::Web;
        copyHz(hz);
    }

    void setZenohCommand(const Twist2D& si, uint32_t now_ms) {
        if (estop_) {
            return;
        }
        Twist2D n;
        n.vx = (cfg_.max_vx > 1e-6f) ? (si.vx / cfg_.max_vx) : 0.0f;
        n.vy = (cfg_.max_vy > 1e-6f) ? (si.vy / cfg_.max_vy) : 0.0f;
        n.omega = (cfg_.max_omega > 1e-6f) ? (si.omega / cfg_.max_omega) : 0.0f;
        zenoh_cmd_.twist = clampNorm(n);
        zenoh_cmd_.source = CmdSource::Zenoh;
        zenoh_cmd_.stamp_ms = now_ms;
        mode_ = Mode::Cinematic;

        float hz[4] = {};
        kin_.inverseNormalized(zenoh_cmd_.twist, hz);
        applyWheelHz(hz);
        active_source_ = CmdSource::Zenoh;
        copyHz(hz);
    }

    /**
     * Raw step-Hz per motor (Qt demo/pc_to_esp).
     * test_mode=true → idle motors freewheel (disable holding) so each axis
     * can be spun alone without fighting the other three locked steppers.
     */
    void setRawMotorHz(float m0, float m1, float m2, float m3, uint32_t now_ms,
                       bool test_mode = false) {
        if (estop_) {
            return;
        }
        raw_hz_[0] = m0;
        raw_hz_[1] = m1;
        raw_hz_[2] = m2;
        raw_hz_[3] = m3;
        raw_stamp_ms_ = now_ms;
        raw_test_mode_ = test_mode;
        mode_ = Mode::RawMotors;
        web_cmd_.source = CmdSource::Web;
        web_cmd_.stamp_ms = now_ms;

        applyWheelHz(raw_hz_, raw_test_mode_);
        active_source_ = CmdSource::Web;
        copyHz(raw_hz_);
    }

    void emergencyStop(uint32_t now_ms) {
        estop_ = true;
        web_cmd_ = {};
        zenoh_cmd_ = {};
        mode_ = Mode::Idle;
        web_cmd_.stamp_ms = now_ms;
        for (int i = 0; i < 4; ++i) {
            if (motors_[i]) {
                motors_[i]->hardStop();
            }
            last_hz_[i] = 0.0f;
            applied_hz_[i] = 0.0f;
        }
        active_source_ = CmdSource::None;
        Serial.println("[ESTOP] latched");
    }

    void clearEstop() {
        estop_ = false;
        web_cmd_ = {};
        zenoh_cmd_ = {};
        mode_ = Mode::Idle;
        Serial.println("[ESTOP] cleared");
    }

    bool estop() const { return estop_; }
    bool driving() const {
        return !estop_ && active_source_ != CmdSource::None &&
               (fabsf(applied_hz_[0]) + fabsf(applied_hz_[1]) + fabsf(applied_hz_[2]) +
                fabsf(applied_hz_[3])) > 1.0f;
    }
    CmdSource activeSource() const { return active_source_; }
    bool motorOk(int i) const {
        return i >= 0 && i < 4 && motors_[i] && motors_[i]->ok();
    }
    const float* appliedHz() const { return applied_hz_; }

    void update(float dt_s, float gyro_z, uint32_t now_ms) {
        float hz[4] = {};

        if (estop_) {
            active_source_ = CmdSource::None;
            odom_.update(hz, dt_s, gyro_z, now_ms);
            return;
        }

        const bool web_fresh = web_cmd_.source == CmdSource::Web &&
                               (now_ms - web_cmd_.stamp_ms) <= cfg_.cmd_timeout_ms;
        const bool zenoh_fresh = zenoh_cmd_.source == CmdSource::Zenoh &&
                                 (now_ms - zenoh_cmd_.stamp_ms) <= cfg_.cmd_timeout_ms;
        const bool raw_fresh =
            mode_ == Mode::RawMotors && (now_ms - raw_stamp_ms_) <= cfg_.cmd_timeout_ms;

        bool freewheel = false;
        if (web_fresh && mode_ == Mode::RawMotors && raw_fresh) {
            for (int i = 0; i < 4; ++i) {
                hz[i] = raw_hz_[i];
            }
            active_source_ = CmdSource::Web;
            freewheel = raw_test_mode_;
        } else if (web_fresh && mode_ == Mode::Cinematic) {
            kin_.inverseNormalized(web_cmd_.twist, hz);
            active_source_ = CmdSource::Web;
        } else if (zenoh_fresh) {
            kin_.inverseNormalized(zenoh_cmd_.twist, hz);
            active_source_ = CmdSource::Zenoh;
        } else {
            // Timeout → stop + re-hold (exit freewheel test)
            active_source_ = CmdSource::None;
            mode_ = Mode::Idle;
            raw_test_mode_ = false;
        }

        applyWheelHz(hz, freewheel);
        copyHz(hz);
        odom_.update(hz, dt_s, gyro_z, now_ms);
    }

private:
    enum class Mode : uint8_t { Idle, Cinematic, RawMotors };

    RobotConfig& cfg_;
    HolonomicKinematics& kin_;
    Odometry& odom_;
    Stepper* motors_[4] = {};
    VelocityCommand web_cmd_{};
    VelocityCommand zenoh_cmd_{};
    CmdSource active_source_ = CmdSource::None;
    Mode mode_ = Mode::Idle;
    bool estop_ = false;
    bool raw_test_mode_ = false;
    float last_hz_[4] = {};
    float applied_hz_[4] = {};
    float raw_hz_[4] = {};
    uint32_t raw_stamp_ms_ = 0;

    void copyHz(const float hz[4]) {
        for (int i = 0; i < 4; ++i) {
            applied_hz_[i] = hz[i];
        }
    }

    static float clampf(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    Twist2D clampNorm(Twist2D t) const {
        t.vx = clampf(t.vx, -1.0f, 1.0f);
        t.vy = clampf(t.vy, -1.0f, 1.0f);
        t.omega = clampf(t.omega, -1.0f, 1.0f);
        return t;
    }

    void applyWheelHz(const float hz[4], bool freewheel_idle = false) {
        int32_t targets[4];
        int32_t deltas[4];
        int32_t max_delta = 1;
        for (int i = 0; i < 4; ++i) {
            float h = hz[i];
            const float lim = static_cast<float>(cfg_.max_step_hz);
            if (h > lim) {
                h = lim;
            } else if (h < -lim) {
                h = -lim;
            }
            targets[i] = static_cast<int32_t>(lroundf(h));
            deltas[i] = abs(targets[i] - static_cast<int32_t>(lroundf(last_hz_[i])));
            if (deltas[i] > max_delta) {
                max_delta = deltas[i];
            }
        }

        for (int i = 0; i < 4; ++i) {
            if (!motors_[i]) {
                continue;
            }
            // Same proportional accel sync as the original main.cpp loop.
            int32_t accel = static_cast<int32_t>(
                (static_cast<int64_t>(cfg_.max_accel_hz) * deltas[i]) / max_delta);
            // Non-zero targets need real accel (avoid ~0 from idle siblings).
            if (targets[i] != 0 && accel < static_cast<int32_t>(cfg_.max_accel_hz / 4)) {
                accel = static_cast<int32_t>(cfg_.max_accel_hz / 4);
            }
            if (accel < 1) {
                accel = 1;
            }
            motors_[i]->setSpeed(targets[i], accel, freewheel_idle);
            last_hz_[i] = static_cast<float>(targets[i]);
        }
    }
};
