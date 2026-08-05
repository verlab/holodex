#pragma once

#include "RobotConfig.hpp"
#include "RobotTypes.hpp"
#include <cmath>

/**
 * Mecanum 4WD inverse / forward kinematics (body frame, spin about chassis center).
 *
 * Motor index / corner (HoloDex physical wiring):
 *   M0 front-left   (+lx, +ly)
 *   M1 rear-left    (−lx, +ly)
 *   M2 front-right  (+lx, −ly)
 *   M3 rear-right   (−lx, −ly)
 *
 * Standard roller layout (ROS / OpenCR style), L = lx + ly:
 *   v_FL =  vx − vy − L·ω
 *   v_RL =  vx + vy − L·ω
 *   v_FR =  vx + vy + L·ω
 *   v_RR =  vx − vy + L·ω
 *
 * Wheel linear speeds (m/s) → step Hz via RobotConfig::mpsToStepHz.
 * Left motors are mounted mirrored: motor_sign[M0]=motor_sign[M1]=−1
 * (applied in MotionController, not here).
 */
class HolonomicKinematics {
public:
    explicit HolonomicKinematics(const RobotConfig& cfg) : cfg_(cfg) { rebuild(); }

    void setConfig(const RobotConfig& cfg) {
        cfg_ = cfg;
        rebuild();
    }

    float leverArm() const { return L_; }

    /** Body twist (SI) → wheel step Hz. */
    void inverseSi(const Twist2D& si_in, float step_hz_out[4]) const {
        Twist2D si = si_in;
        if (cfg_.invert_vx) {
            si.vx = -si.vx;
        }
        if (cfg_.invert_vy) {
            si.vy = -si.vy;
        }
        if (cfg_.invert_omega) {
            si.omega = -si.omega;
        }

        const float Lw = L_ * si.omega;
        const float v[4] = {
            si.vx - si.vy - Lw,  // M0 FL
            si.vx + si.vy - Lw,  // M1 RL
            si.vx + si.vy + Lw,  // M2 FR
            si.vx - si.vy + Lw,  // M3 RR
        };
        for (int i = 0; i < 4; ++i) {
            step_hz_out[i] = cfg_.mpsToStepHz(v[i]);
        }
        saturatePeak(step_hz_out);
    }

    /**
     * Normalized sticks (−1..1) → step Hz.
     * Maps to SI via max_vx / max_vy / max_omega, then inverseSi.
     */
    void inverseNormalized(const Twist2D& ncmd, float step_hz_out[4]) const {
        Twist2D si;
        si.vx = ncmd.vx * cfg_.max_vx;
        si.vy = ncmd.vy * cfg_.max_vy;
        si.omega = ncmd.omega * cfg_.max_omega;
        inverseSi(si, step_hz_out);
    }

    /** @deprecated use inverseNormalized(Twist2D). Kept for call sites. */
    void inverseNormalized(float dir_x, float yaw_n, float strafe_n,
                           float step_hz_out[4]) const {
        Twist2D n;
        n.vx = dir_x;
        n.vy = strafe_n;
        n.omega = yaw_n;
        inverseNormalized(n, step_hz_out);
    }

    /**
     * Body twist from wheel step rates (SI). Exact left-inverse of inverseSi.
     */
    Twist2D forwardSi(const float step_hz[4]) const {
        const float v0 = cfg_.stepHzToMps(step_hz[0]);  // FL
        const float v1 = cfg_.stepHzToMps(step_hz[1]);  // RL
        const float v2 = cfg_.stepHzToMps(step_hz[2]);  // FR
        const float v3 = cfg_.stepHzToMps(step_hz[3]);  // RR

        Twist2D si;
        si.vx = 0.25f * (v0 + v1 + v2 + v3);
        // Standard: vy = ¼(v_FR − v_RR − v_FL + v_RL)
        si.vy = 0.25f * (v2 - v3 - v0 + v1);
        if (L_ > 1e-6f) {
            // ω = ¼(v_FR + v_RR − v_FL − v_RL) / L
            si.omega = 0.25f * (v2 + v3 - v0 - v1) / L_;
        } else {
            si.omega = 0.0f;
        }
        return si;
    }

private:
    RobotConfig cfg_;
    float L_ = 0.385f;  // lx + ly

    void rebuild() {
        const float lx = cfg_.lx_m > 1e-4f ? cfg_.lx_m : 0.19f;
        const float ly = cfg_.ly_m > 1e-4f ? cfg_.ly_m : 0.195f;
        L_ = lx + ly;
    }

    void saturatePeak(float hz[4]) const {
        const float lim = static_cast<float>(cfg_.max_step_hz);
        float peak = 0.0f;
        for (int i = 0; i < 4; ++i) {
            const float a = fabsf(hz[i]);
            if (a > peak) {
                peak = a;
            }
        }
        if (peak > lim && peak > 1e-6f) {
            const float s = lim / peak;
            for (int i = 0; i < 4; ++i) {
                hz[i] *= s;
            }
        }
    }
};
