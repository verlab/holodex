#pragma once

#include "EncoderHub.hpp"
#include "HolonomicKinematics.hpp"
#include "InertialOdometry.hpp"
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
                     Odometry& odom_fused, Odometry& odom_stepper, Odometry& odom_encoder,
                     InertialOdometry& odom_imu)
        : cfg_(cfg),
          kin_(kin),
          odom_(odom_fused),
          odom_step_(odom_stepper),
          odom_enc_(odom_encoder),
          imu_(odom_imu) {
        for (int i = 0; i < 4; ++i) {
            motors_[i] = motors[i];
        }
    }

    void attachEncoders(EncoderHub* enc) { enc_ = enc; }
    EncoderHub* encoders() const { return enc_; }

    /** Primary: wheels+enc+IMU. Compare: wheels-only, wheels+enc, IMU-only. */
    Odometry& odomFull() { return odom_; }
    Odometry& odomWheels() { return odom_step_; }
    Odometry& odomWheelsEnc() { return odom_enc_; }
    InertialOdometry& odomImu() { return imu_; }
    // Aliases kept for older call sites
    Odometry& odomFused() { return odom_; }
    Odometry& odomStepper() { return odom_step_; }
    Odometry& odomEncoder() { return odom_enc_; }

    void resetAllOdom() {
        odom_.reset();
        odom_step_.reset();
        odom_enc_.reset();
        // Also re-learns the IMU offsets, so the robot must sit still for a
        // moment after a reset before the inertial stream comes back.
        imu_.reset();
        syncHeadingRef();
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
        holding_ = false;

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
        holding_ = false;

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
        holding_ = false;
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
        holding_ = false;
        syncHeadingRef();
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
        holding_ = false;
        syncHeadingRef();
        resetSpeedIntegrators();
        Serial.println("[ESTOP] cleared");
    }

    /** Odom wheel-rate source hint for UI: enc / fas / mix. */
    const char* wheelMeasureSrc() const { return measure_src_; }

    bool estop() const { return estop_; }
    /**
     * Wheels are turning. Deliberately not gated on an active command: the
     * heading hold moves the robot with no command present, and the LED / radio
     * guards must treat that as motion.
     */
    bool driving() const {
        return !estop_ && (fabsf(applied_hz_[0]) + fabsf(applied_hz_[1]) +
                           fabsf(applied_hz_[2]) + fabsf(applied_hz_[3])) > 1.0f;
    }
    /** True while the yaw servo is actively turning the robot back on heading. */
    bool headingHolding() const { return hold_engaged_; }
    float headingRef() const { return heading_ref_; }
    CmdSource activeSource() const { return active_source_; }
    bool motorOk(int i) const {
        return i >= 0 && i < 4 && motors_[i] && motors_[i]->ok();
    }
    const float* appliedHz() const { return applied_hz_; }

    /**
     * One control tick.
     *
     * @param dt_s interval since the previous tick, measured at sampling time
     * @param imu  latest IMU sample, carrying its own dt for the integrators
     */
    void update(float dt_s, const ImuSample& imu, uint32_t now_ms) {
        // Inertial state first: the heading servo below reads it this same tick,
        // and it integrates on the sample's own clock, not on dt_s.
        imu_.update(imu, now_ms);
        const bool gyro_valid = imu.valid && imu_.calibrated();
        // Wheel streams get the bias-corrected rate; each applies its own
        // deadband, so do not hand them the already-deadbanded value.
        const float gyro_z = gyro_valid ? imu_.yawRateRaw() : NAN;

        float cmd_hz[4] = {};

        if (estop_) {
            active_source_ = CmdSource::None;
            syncHeadingRef();
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
        bool coasting = false;
        if (mode_ == Mode::MoveTo) {
            holding_ = false;
            if (!serviceMoveTo(cmd_hz)) {
                mode_ = Mode::Idle;
                active_source_ = CmdSource::None;
            } else {
                active_source_ = CmdSource::Web;
            }
        } else if (mode_ == Mode::RawMotors && raw_fresh) {
            holding_ = false;
            for (int i = 0; i < 4; ++i) {
                cmd_hz[i] = raw_hz_[i];
            }
            active_source_ = CmdSource::Web;
            freewheel = raw_test_mode_;
        } else if (web_fresh && mode_ == Mode::Cinematic) {
            holding_ = false;
            kin_.inverseNormalized(web_cmd_.twist, cmd_hz);
            active_source_ = CmdSource::Web;
        } else if (zenoh_fresh) {
            holding_ = false;
            kin_.inverseSi(zenoh_si_, cmd_hz);
            active_source_ = CmdSource::Zenoh;
        } else {
            coasting = serviceCommandTimeout(cmd_hz, now_ms);
        }

        // Rotation the operator actually asked for this tick, captured before any
        // assist touches it: this is what the heading reference must integrate.
        const float omega_cmd = kin_.forwardSi(cmd_hz).omega;

        // Body-level speed assist (preserves mecanum coupling / ICR at center).
        // Stands down while coasting or ramping — in both cases the measured
        // twist lags the command by design, and correcting for that only winds up
        // the integrator and overshoots once the wheels arrive.
        if (!freewheel && !coasting && !wheelsRamping() && cfg_.enc_speed_fb && enc_ &&
            enc_->fresh() && active_source_ != CmdSource::None) {
            applySpeedFeedback(cmd_hz, dt_s);
        } else if (active_source_ == CmdSource::None) {
            resetSpeedIntegrators();
        }

        // Absolute heading hold — last, so it also corrects whatever the speed
        // assist just did. Runs with or without an active command: if the IMU
        // says the robot is not pointing where it should, it gets turned back.
        // Skipped only for the motor test bench, where the operator drives each
        // axis by hand and an injected rotation would corrupt the measurement.
        if (!freewheel && mode_ != Mode::RawMotors && cfg_.yaw_assist && gyro_valid) {
            if (!hold_ready_) {
                syncHeadingRef();  // measurement just came back — start from here
                hold_ready_ = true;
            }
            applyHeadingHold(cmd_hz, dt_s, omega_cmd);
        } else {
            // Leave the reference alone while blind. Pinning it to a heading that
            // is itself frozen would silently drop whatever rotation happened
            // during the gap, and the servo would pay that back as a spurious
            // spin the moment samples return.
            hold_ready_ = false;
            hold_engaged_ = false;
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
    InertialOdometry& imu_;  // IMU only — also the absolute heading reference
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
    // Absolute heading hold against the IMU angle
    static constexpr float kYawErrLimRad = 0.6f;      // cap on the debt we chase
    static constexpr float kYawHoldP = 2.5f;          // 1/s
    static constexpr float kYawHoldD = 0.35f;         // dimensionless
    static constexpr float kYawMaxCorrFrac = 0.5f;    // of max_omega
    static constexpr float kYawExitFrac = 0.35f;      // of yaw_tol — hysteresis
    static constexpr float kYawTolMinRad = 0.002f;    // floor so the servo can settle
    float heading_ref_ = 0.0f;                        // rad, where we should point
    bool hold_engaged_ = false;
    bool hold_ready_ = false;                         // reference synced to the IMU

    // Wheel is "arrived" within a couple of Hz of its target
    static constexpr float kAtTargetHz = 2.0f;
    // Hard accel ceiling of the step generator, applied before the split
    static constexpr uint32_t kAccelCeilingHz = 30000;
    // Watchdog coast-down: hold the last command and fade it out
    static constexpr uint32_t kCoastMs = 250;
    bool holding_ = false;
    uint32_t hold_start_ms_ = 0;
    float hold_hz_[4] = {};
    bool last_freewheel_ = false;
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

    /** True while any wheel is still travelling to its setpoint. */
    bool wheelsRamping() const {
        float band = 0.02f * static_cast<float>(cfg_.max_step_hz);
        if (band < 50.0f) {
            band = 50.0f;
        }
        for (int i = 0; i < 4; ++i) {
            if (motors_[i] && fabsf(last_hz_[i] - motors_[i]->getCurrentSpeedHz()) > band) {
                return true;
            }
        }
        return false;
    }

    void resetSpeedIntegrators() {
        speed_i_vx_ = 0.0f;
        speed_i_vy_ = 0.0f;
        speed_i_w_ = 0.0f;
    }

    /**
     * Command watchdog. A stale command coasts to zero over kCoastMs instead of
     * dropping there on the first late tick, so one lost packet (or one loop
     * iteration blocked by the HTTP server) costs a small dip rather than a full
     * decelerate/re-accelerate cycle. A real link loss still stops the robot.
     * @return true while coasting, so the speed assist stays out of the way.
     */
    bool serviceCommandTimeout(float cmd_hz[4], uint32_t now_ms) {
        if (!holding_) {
            if (active_source_ == CmdSource::None) {
                mode_ = Mode::Idle;
                raw_test_mode_ = false;
                resetSpeedIntegrators();
                return false;
            }
            holding_ = true;
            hold_start_ms_ = now_ms;
            for (int i = 0; i < 4; ++i) {
                hold_hz_[i] = applied_hz_[i];
            }
        }

        const uint32_t held = now_ms - hold_start_ms_;
        if (held < kCoastMs) {
            const float k = 1.0f - static_cast<float>(held) / static_cast<float>(kCoastMs);
            for (int i = 0; i < 4; ++i) {
                cmd_hz[i] = hold_hz_[i] * k;
            }
            return true;
        }

        holding_ = false;
        active_source_ = CmdSource::None;
        mode_ = Mode::Idle;
        raw_test_mode_ = false;
        resetSpeedIntegrators();
        return false;
    }

    /**
     * IMU heading / rate expressed in the same frame as the commands.
     *
     * cmd_hz already carries invert_omega, so the twist recovered from it lives
     * in the inverted frame while the gyro always reports the physical one.
     * Comparing the two directly would turn the servo into positive feedback the
     * moment someone ticks that box on the config page.
     */
    float imuHeadingCmd() const {
        return cfg_.invert_omega ? -imu_.heading() : imu_.heading();
    }
    float imuRateCmd() const {
        return cfg_.invert_omega ? -imu_.yawRate() : imu_.yawRate();
    }

    /** Adopt the current IMU heading as the target — nothing to correct. */
    void syncHeadingRef() {
        heading_ref_ = imuHeadingCmd();
        hold_engaged_ = false;
        hold_ready_ = false;
    }

    /**
     * Servo the chassis onto the heading the commands add up to, using the IMU
     * angle as ground truth.
     *
     * The reference integrates only what the operator asked for; the correction
     * is deliberately kept out of it. Feeding the correction back — or using the
     * wheel-integrated angle as the reference — turns the loop into a tail chase,
     * because rotating the robot moves reference and measurement by the same
     * amount and the error never closes.
     *
     * Runs whether or not a command is active, so an unmodelled yaw (mecanum
     * roller slip while strafing, a nudge from a person, a wheel that lost a few
     * steps) is turned back out even while the robot is nominally parked.
     *
     * yaw_tol sets how much error is worth chasing. Below it the reading is
     * indistinguishable from gyro drift, and the hysteresis on the way out stops
     * the robot from twitching around the threshold.
     */
    void applyHeadingHold(float cmd_hz[4], float dt_s, float omega_cmd) {
        if (dt_s > 0.0f) {
            heading_ref_ = wrapPi(heading_ref_ + omega_cmd * dt_s);
        }

        float err = wrapPi(heading_ref_ - imuHeadingCmd());
        // Forgive anything past the cap. A long spin that slipped would
        // otherwise bank a debt the robot pays back as a surprise rotation the
        // moment the operator lets go.
        if (fabsf(err) > kYawErrLimRad) {
            err = err > 0.0f ? kYawErrLimRad : -kYawErrLimRad;
            heading_ref_ = wrapPi(imuHeadingCmd() + err);
        }

        // Floor the tolerance: at exactly zero the engage test always passes and
        // the hysteresis exit never can, so the servo would latch on forever.
        const float tol = cfg_.yaw_tol > kYawTolMinRad ? cfg_.yaw_tol : kYawTolMinRad;
        if (hold_engaged_) {
            if (fabsf(err) < kYawExitFrac * tol) {
                hold_engaged_ = false;
            }
        } else if (fabsf(err) > tol) {
            hold_engaged_ = true;
        }
        if (!hold_engaged_) {
            return;
        }

        // P on the angle, D on the *rate error* — damping against the raw rate
        // would fight every commanded rotation instead of just the unwanted ones.
        float corr = kYawHoldP * err + kYawHoldD * (omega_cmd - imuRateCmd());
        const float lim = kYawMaxCorrFrac * cfg_.max_omega;
        corr = clampf(corr, -lim, lim);

        Twist2D tw = kin_.forwardSi(cmd_hz);
        tw.omega = clampf(tw.omega + corr, -cfg_.max_omega, cfg_.max_omega);
        kin_.wheelsFromBody(tw, cmd_hz);
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

    static float wrapPi(float a) {
        while (a > static_cast<float>(M_PI)) {
            a -= 2.0f * static_cast<float>(M_PI);
        }
        while (a < -static_cast<float>(M_PI)) {
            a += 2.0f * static_cast<float>(M_PI);
        }
        return a;
    }

    Twist2D clampNorm(Twist2D t) const {
        t.vx = clampf(t.vx, -1.0f, 1.0f);
        t.vy = clampf(t.vy, -1.0f, 1.0f);
        t.omega = clampf(t.omega, -1.0f, 1.0f);
        return t;
    }

    /**
     * Program the four steppers so they all reach their targets at the same
     * instant. Acceleration is shared out in proportion to how far each wheel
     * still has to travel: with a_i = a_max·Δ_i/Δ_max every wheel needs the same
     * ramp time T = Δ_max/a_max. A wheel that arrives early is not harmless —
     * the mecanum mix turns the mismatch into yaw or sideways drift that nobody
     * commanded, for as long as the ramp lasts.
     *
     * Δ is measured against each motor's *actual* rate rather than its last
     * setpoint, and the split is recomputed every tick while any wheel is still
     * ramping. Teleop refreshes mid-ramp all the time, and a wheel that falls
     * behind gets a larger share until it is back in step.
     */
    void applyWheelHz(const float hz[4], bool freewheel_idle = false) {
        // Logical → electrical. Saturation uses one common scale: clamping wheels
        // individually would tilt the commanded twist instead of just slowing it.
        float el[4];
        float peak = 0.0f;
        for (int i = 0; i < 4; ++i) {
            float h = hz[i];
            // motor_sign from Config Flip checkboxes (teleop + zenoh). Motor-test
            // bypasses this so sliders match electrical STEP/DIR sense.
            if (!freewheel_idle && cfg_.motor_sign[i] < 0) {
                h = -h;
            }
            el[i] = h;
            const float a = fabsf(h);
            if (a > peak) {
                peak = a;
            }
        }
        const float lim = static_cast<float>(cfg_.max_step_hz);
        if (peak > lim && peak > 1e-6f) {
            const float s = lim / peak;
            for (int i = 0; i < 4; ++i) {
                el[i] *= s;
            }
        }

        int32_t targets[4];
        float deltas[4];
        float max_delta = 0.0f;
        bool setpoint_changed = false;
        bool ramping = false;
        for (int i = 0; i < 4; ++i) {
            targets[i] = static_cast<int32_t>(lroundf(el[i]));
            if (targets[i] != static_cast<int32_t>(lroundf(last_hz_[i]))) {
                setpoint_changed = true;
            }
            const float cur = motors_[i] ? motors_[i]->getCurrentSpeedHz() : 0.0f;
            deltas[i] = fabsf(static_cast<float>(targets[i]) - cur);
            if (deltas[i] > max_delta) {
                max_delta = deltas[i];
            }
            if (deltas[i] > kAtTargetHz) {
                ramping = true;
            }
        }

        const bool mode_changed = freewheel_idle != last_freewheel_;
        last_freewheel_ = freewheel_idle;

        // Steady state: re-issuing an unchanged setpoint costs a critical section
        // per motor in FastAccelStepper for nothing.
        if (!mode_changed && !setpoint_changed && !ramping) {
            return;
        }

        // Cap before splitting, never after: clamping a single wheel's share is
        // what would desynchronise the ramp.
        const uint32_t a_cap = cfg_.max_accel_hz < kAccelCeilingHz ? cfg_.max_accel_hz
                                                                   : kAccelCeilingHz;
        const float a_max = static_cast<float>(a_cap > 0 ? a_cap : 1);
        const float ramp_s = (max_delta > 1e-3f) ? (max_delta / a_max) : 0.0f;

        for (int i = 0; i < 4; ++i) {
            if (!motors_[i]) {
                continue;
            }
            int32_t accel = static_cast<int32_t>(lroundf(a_max));
            if (ramp_s > 1e-6f) {
                accel = static_cast<int32_t>(lroundf(deltas[i] / ramp_s));
                if (accel < 1) {
                    accel = 1;
                }
            }
            motors_[i]->setSpeed(targets[i], accel, freewheel_idle);
            last_hz_[i] = static_cast<float>(targets[i]);
        }
    }
};
