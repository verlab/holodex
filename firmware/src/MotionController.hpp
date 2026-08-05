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
                     Odometry& odom_fused, Odometry& odom_stepper, Odometry& odom_encoder)
        : cfg_(cfg),
          kin_(kin),
          odom_(odom_fused),
          odom_step_(odom_stepper),
          odom_enc_(odom_encoder) {
        for (int i = 0; i < 4; ++i) {
            motors_[i] = motors[i];
        }
    }

    void attachEncoders(EncoderHub* enc) { enc_ = enc; }
    EncoderHub* encoders() const { return enc_; }

    /** Primary: wheels+enc+IMU. Compare: wheels-only, wheels+enc (no IMU). */
    Odometry& odomFull() { return odom_; }
    Odometry& odomWheels() { return odom_step_; }
    Odometry& odomWheelsEnc() { return odom_enc_; }
    // Aliases kept for older call sites
    Odometry& odomFused() { return odom_; }
    Odometry& odomStepper() { return odom_step_; }
    Odometry& odomEncoder() { return odom_enc_; }

    void resetAllOdom() {
        odom_.reset();
        odom_step_.reset();
        odom_enc_.reset();
    }

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
        // Keep normalized copy for timeout/refresh path; drive with SI IK
        Twist2D n;
        n.vx = (cfg_.max_vx > 1e-6f) ? (si.vx / cfg_.max_vx) : 0.0f;
        n.vy = (cfg_.max_vy > 1e-6f) ? (si.vy / cfg_.max_vy) : 0.0f;
        n.omega = (cfg_.max_omega > 1e-6f) ? (si.omega / cfg_.max_omega) : 0.0f;
        zenoh_cmd_.twist = clampNorm(n);
        zenoh_si_ = si;
        zenoh_cmd_.source = CmdSource::Zenoh;
        zenoh_cmd_.stamp_ms = now_ms;
        mode_ = Mode::Cinematic;

        float hz[4] = {};
        kin_.inverseSi(si, hz);
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
        resetSpeedIntegrators();
        active_source_ = CmdSource::None;
        Serial.println("[ESTOP] latched");
    }

    void clearEstop() {
        estop_ = false;
        cancelMove();
        web_cmd_ = {};
        zenoh_cmd_ = {};
        mode_ = Mode::Idle;
        resetSpeedIntegrators();
        Serial.println("[ESTOP] cleared");
    }

    /** Odom wheel-rate source hint for UI: enc / fas / mix. */
    const char* wheelMeasureSrc() const { return measure_src_; }

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
            updateOdometryStreams(dt_s, gyro_z, gyro_valid, now_ms);
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
            kin_.inverseSi(zenoh_si_, cmd_hz);
            active_source_ = CmdSource::Zenoh;
        } else {
            // Timeout → stop + re-hold (exit freewheel test)
            active_source_ = CmdSource::None;
            mode_ = Mode::Idle;
            raw_test_mode_ = false;
            resetSpeedIntegrators();
        }

        // Body-level speed assist (preserves mecanum coupling / ICR at center)
        if (!freewheel && cfg_.enc_speed_fb && enc_ && enc_->fresh() &&
            active_source_ != CmdSource::None) {
            applySpeedFeedback(cmd_hz, dt_s);
        } else if (active_source_ == CmdSource::None) {
            resetSpeedIntegrators();
        }

        applyWheelHz(cmd_hz, freewheel);
        copyHz(cmd_hz);
        updateOdometryStreams(dt_s, gyro_z, gyro_valid, now_ms);
    }

private:
    enum class Mode : uint8_t { Idle, Cinematic, RawMotors, MoveTo };

    RobotConfig& cfg_;
    HolonomicKinematics& kin_;
    Odometry& odom_;       // wheels + enc blend + IMU yaw (primary / ROS)
    Odometry& odom_step_;  // FAS wheel rates only, yaw from wheels (no IMU)
    Odometry& odom_enc_;   // FAS+AS5600 blend, yaw from wheels (no IMU)
    EncoderHub* enc_ = nullptr;
    Stepper* motors_[4] = {};
    VelocityCommand web_cmd_{};
    VelocityCommand zenoh_cmd_{};
    Twist2D zenoh_si_{};  // last SI cmd_vel (for accurate mecanum IK on refresh)
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
    // Body-twist integrators for speed assist (not per-wheel — preserves ICR)
    float speed_i_vx_ = 0.0f;
    float speed_i_vy_ = 0.0f;
    float speed_i_w_ = 0.0f;
    const char* measure_src_ = "fas";

    void cancelMove() {
        if (mode_ == Mode::MoveTo) {
            mode_ = Mode::Idle;
        }
    }

    void resetSpeedIntegrators() {
        speed_i_vx_ = 0.0f;
        speed_i_vy_ = 0.0f;
        speed_i_w_ = 0.0f;
    }

    /**
     * Body-level PI speed assist: compare commanded vs encoder twist, correct in
     * SE(2), then re-IK. Per-wheel PI was wrong — it broke mecanum coupling and
     * could zero one wheel while others spun (ICR at a corner).
     */
    void applySpeedFeedback(float cmd_hz[4], float dt_s) {
        for (int i = 0; i < 4; ++i) {
            if (!enc_->motorOk(i)) {
                resetSpeedIntegrators();
                return;
            }
        }

        float meas_hz[4] = {};
        for (int i = 0; i < 4; ++i) {
            meas_hz[i] = enc_->lastHz(i);
        }

        const Twist2D cmd_tw = kin_.forwardSi(cmd_hz);
        const Twist2D meas_tw = kin_.forwardSi(meas_hz);

        const float cmd_mag =
            fabsf(cmd_tw.vx) + fabsf(cmd_tw.vy) + fabsf(cmd_tw.omega) * kin_.leverArm();
        if (cmd_mag < 0.02f) {
            resetSpeedIntegrators();
            return;
        }

        constexpr float kP = 0.35f;
        constexpr float kI = 0.6f;
        const float i_lim_v = 0.15f * cfg_.max_vx;
        const float i_lim_w = 0.15f * cfg_.max_omega;

        const float evx = cmd_tw.vx - meas_tw.vx;
        const float evy = cmd_tw.vy - meas_tw.vy;
        const float ew = cmd_tw.omega - meas_tw.omega;

        speed_i_vx_ = clampf(speed_i_vx_ + evx * dt_s, -i_lim_v, i_lim_v);
        speed_i_vy_ = clampf(speed_i_vy_ + evy * dt_s, -i_lim_v, i_lim_v);
        speed_i_w_ = clampf(speed_i_w_ + ew * dt_s, -i_lim_w, i_lim_w);

        Twist2D out;
        out.vx = cmd_tw.vx + kP * evx + kI * speed_i_vx_;
        out.vy = cmd_tw.vy + kP * evy + kI * speed_i_vy_;
        out.omega = cmd_tw.omega + kP * ew + kI * speed_i_w_;

        // Cap assist so we never more than ~40% above the original command
        const float out_mag =
            fabsf(out.vx) + fabsf(out.vy) + fabsf(out.omega) * kin_.leverArm();
        if (out_mag > 1.4f * cmd_mag && out_mag > 1e-6f) {
            const float s = (1.4f * cmd_mag) / out_mag;
            out.vx *= s;
            out.vy *= s;
            out.omega *= s;
        }

        // wheelsFromBody: cmd_hz already embeds invert_*; do not invert again
        kin_.wheelsFromBody(out, cmd_hz);
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

    void readFasLogicalHz(float out[4]) const {
        for (int i = 0; i < 4; ++i) {
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

    void readEncLogicalHz(float out[4]) const {
        const bool enc_fresh = enc_ && enc_->fresh();
        for (int i = 0; i < 4; ++i) {
            if (enc_fresh && enc_->motorOk(i)) {
                out[i] = enc_->lastHz(i);
            } else {
                out[i] = 0.0f;
            }
        }
    }

    void updateOdometryStreams(float dt_s, float gyro_z, bool gyro_valid, uint32_t now_ms) {
        float fas[4] = {};
        float enc[4] = {};
        float blend[4] = {};
        readFasLogicalHz(fas);
        readEncLogicalHz(enc);

        int n_enc = 0;
        const float a = cfg_.enc_blend;
        for (int i = 0; i < 4; ++i) {
            if (cfg_.enc_odom && enc_ && enc_->fresh() && enc_->motorOk(i)) {
                blend[i] = a * enc[i] + (1.0f - a) * fas[i];
                n_enc++;
            } else {
                blend[i] = fas[i];
            }
        }
        if (n_enc == 4) {
            measure_src_ = (a > 0.99f) ? "enc" : "mix";
        } else if (n_enc > 0) {
            measure_src_ = "mix";
        } else {
            measure_src_ = "fas";
        }

        // 1) wheels only (stepper ramp) — no IMU
        odom_step_.update(fas, dt_s, gyro_z, gyro_valid, now_ms, /*use_imu=*/false, "wheels");
        // 2) wheels + encoders (blend) — no IMU
        odom_enc_.update(blend, dt_s, gyro_z, gyro_valid, now_ms, /*use_imu=*/false, "wheels+enc");
        // 3) wheels + encoders + IMU yaw
        odom_.update(blend, dt_s, gyro_z, gyro_valid, now_ms, /*use_imu=*/true, "wheels+enc+imu");
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
