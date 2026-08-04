#pragma once

#include "EncoderHub.hpp"
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

    void attachEncoders(EncoderHub* enc) { enc_ = enc; }
    EncoderHub* encoders() const { return enc_; }

    void setWebCommand(const Twist2D& ncmd, uint32_t now_ms) {
        if (estop_) {
            return;
        }
        cancelMove();
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
        cancelMove();
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
     * Closed-loop body-frame translation using odometry (wheels + optional gyro yaw).
     * dx/dy in meters in the pose at command time; omega held at 0.
     */
    bool startMoveBody(float dx_m, float dy_m, float speed_mps, uint32_t now_ms) {
        if (estop_) {
            return false;
        }
        const float dist = hypotf(dx_m, dy_m);
        if (dist < 0.005f) {
            return false;
        }
        const auto& p = odom_.state().pose;
        const float c = cosf(p.theta);
        const float s = sinf(p.theta);
        move_tx_ = p.x + c * dx_m - s * dy_m;
        move_ty_ = p.y + s * dx_m + c * dy_m;
        float spd = fabsf(speed_mps);
        if (spd < 0.05f) {
            spd = 0.05f;
        }
        if (spd > cfg_.max_vx) {
            spd = cfg_.max_vx;
        }
        move_speed_ = spd;
        mode_ = Mode::MoveTo;
        web_cmd_ = {};
        zenoh_cmd_ = {};
        raw_test_mode_ = false;
        active_source_ = CmdSource::Web;
        web_cmd_.stamp_ms = now_ms;
        Serial.printf("[MOVE] body dx=%.3f dy=%.3f → (%.3f,%.3f) @ %.2f m/s\n", dx_m, dy_m,
                      move_tx_, move_ty_, move_speed_);
        return true;
    }

    bool moveActive() const { return mode_ == Mode::MoveTo; }

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
        cancelMove();
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
        cancelMove();
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
        cancelMove();
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

    void update(float dt_s, float gyro_z, bool gyro_valid, uint32_t now_ms) {
        float cmd_hz[4] = {};

        if (estop_) {
            active_source_ = CmdSource::None;
            float measured[4] = {};
            readMeasuredLogicalHz(measured);
            odom_.update(measured, dt_s, gyro_z, gyro_valid, now_ms);
            return;
        }

        const bool web_fresh = web_cmd_.source == CmdSource::Web &&
                               (now_ms - web_cmd_.stamp_ms) <= cfg_.cmd_timeout_ms;
        const bool zenoh_fresh = zenoh_cmd_.source == CmdSource::Zenoh &&
                                 (now_ms - zenoh_cmd_.stamp_ms) <= cfg_.cmd_timeout_ms;
        const bool raw_fresh =
            mode_ == Mode::RawMotors && (now_ms - raw_stamp_ms_) <= cfg_.cmd_timeout_ms;

        bool freewheel = false;
        if (mode_ == Mode::MoveTo) {
            if (!serviceMoveTo(cmd_hz)) {
                mode_ = Mode::Idle;
                active_source_ = CmdSource::None;
            } else {
                active_source_ = CmdSource::Web;
            }
        } else if (web_fresh && mode_ == Mode::RawMotors && raw_fresh) {
            for (int i = 0; i < 4; ++i) {
                cmd_hz[i] = raw_hz_[i];
            }
            active_source_ = CmdSource::Web;
            freewheel = raw_test_mode_;
        } else if (web_fresh && mode_ == Mode::Cinematic) {
            kin_.inverseNormalized(web_cmd_.twist, cmd_hz);
            active_source_ = CmdSource::Web;
        } else if (zenoh_fresh) {
            kin_.inverseNormalized(zenoh_cmd_.twist, cmd_hz);
            active_source_ = CmdSource::Zenoh;
        } else {
            // Timeout → stop + re-hold (exit freewheel test)
            active_source_ = CmdSource::None;
            mode_ = Mode::Idle;
            raw_test_mode_ = false;
        }

        applyWheelHz(cmd_hz, freewheel);
        copyHz(cmd_hz);

        // Prefer AS5600 wheel rates; fall back to FastAccelStepper ramp speed
        float measured[4] = {};
        readMeasuredLogicalHz(measured);
        odom_.update(measured, dt_s, gyro_z, gyro_valid, now_ms);
    }

private:
    enum class Mode : uint8_t { Idle, Cinematic, RawMotors, MoveTo };

    RobotConfig& cfg_;
    HolonomicKinematics& kin_;
    Odometry& odom_;
    EncoderHub* enc_ = nullptr;
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
    float move_tx_ = 0.0f;
    float move_ty_ = 0.0f;
    float move_speed_ = 0.2f;

    void cancelMove() {
        if (mode_ == Mode::MoveTo) {
            mode_ = Mode::Idle;
        }
    }

    /** @return false when goal reached / aborted (cmd_hz zeroed). */
    bool serviceMoveTo(float cmd_hz[4]) {
        const auto& p = odom_.state().pose;
        const float ex = move_tx_ - p.x;
        const float ey = move_ty_ - p.y;
        const float dist = hypotf(ex, ey);
        constexpr float kArriveM = 0.025f;
        if (dist < kArriveM) {
            for (int i = 0; i < 4; ++i) {
                cmd_hz[i] = 0.0f;
            }
            Serial.println("[MOVE] arrived");
            return false;
        }

        const float c = cosf(p.theta);
        const float s = sinf(p.theta);
        const float ebx = c * ex + s * ey;
        const float eby = -s * ex + c * ey;

        float speed = move_speed_;
        constexpr float kSlowM = 0.15f;
        if (dist < kSlowM) {
            speed *= dist / kSlowM;
        }

        Twist2D n{};
        n.vx = (cfg_.max_vx > 1e-6f) ? ((ebx / dist) * speed / cfg_.max_vx) : 0.0f;
        n.vy = (cfg_.max_vy > 1e-6f) ? ((eby / dist) * speed / cfg_.max_vy) : 0.0f;
        n.omega = 0.0f;
        kin_.inverseNormalized(clampNorm(n), cmd_hz);
        return true;
    }

    void copyHz(const float hz[4]) {
        for (int i = 0; i < 4; ++i) {
            applied_hz_[i] = hz[i];
        }
    }

    /**
     * Wheel rates for odometry: AS5600 when present, else FAS current speed.
     * Output is logical IK frame (motor_sign undone for FAS).
     */
    void readMeasuredLogicalHz(float out[4]) {
        float enc_hz[4] = {};
        const bool have_enc = enc_ && enc_->okCount() > 0 && enc_->update(enc_hz);
        for (int i = 0; i < 4; ++i) {
            if (have_enc && enc_->motorOk(i)) {
                out[i] = enc_hz[i];
                continue;
            }
            float h = 0.0f;
            if (motors_[i] && motors_[i]->ok()) {
                h = motors_[i]->getCurrentSpeedHz();
                if (cfg_.motor_sign[i] < 0) {
                    h = -h;
                }
            }
            out[i] = h;
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
            // motor_sign from Config Flip checkboxes (teleop + zenoh). Motor-test
            // bypasses this so sliders match electrical STEP/DIR sense.
            if (!freewheel_idle && cfg_.motor_sign[i] < 0) {
                h = -h;
            }
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
            // Proportional accel so wheels arrive together (original main.cpp).
            // Floor keeps small-delta wheels from lagging visibly behind.
            int32_t accel = static_cast<int32_t>(
                (static_cast<int64_t>(cfg_.max_accel_hz) * deltas[i]) / max_delta);
            const int32_t floor_a = static_cast<int32_t>(cfg_.max_accel_hz / 2);
            if (targets[i] != 0 && accel < floor_a) {
                accel = floor_a;
            }
            if (accel < 1) {
                accel = 1;
            }
            motors_[i]->setSpeed(targets[i], accel, freewheel_idle);
            last_hz_[i] = static_cast<float>(targets[i]);
        }
    }
};
