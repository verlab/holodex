#pragma once

#include "RobotConfig.hpp"
#include "RobotTypes.hpp"
#include <cmath>

/**
 * Holonomic IK for 4× 45° omni wheels (port of cinematic.py + measured lx/ly).
 *
 * getDirection(dir_x, dir_y, rotation) convention (same as Qt):
 *   Web/Qt stick: getDirection(sy, sx, rot) with sy=forward, sx=right.
 *
 * Motor index / corner:
 *   M0 front-right  (+lx, −ly)  û = (−1,−1)/√2
 *   M1 rear-right   (−lx, −ly)  û = (−1,+1)/√2
 *   M2 front-left   (+lx, +ly)  û = (+1,−1)/√2
 *   M3 rear-left    (−lx, +ly)  û = (+1,+1)/√2
 *
 *   wheel_i = û · v + ω_n * (x û_y − y û_x) / R
 *
 * Pure yaw (ω only): all four wheels same sign & |Hz|.
 * Pure forward: M0=M1=−a, M2=M3=+a.
 * motor_sign is applied in MotionController (not here).
 */
class HolonomicKinematics {
public:
    explicit HolonomicKinematics(const RobotConfig& cfg) : cfg_(cfg) { rebuild(); }

    void setConfig(const RobotConfig& cfg) {
        cfg_ = cfg;
        rebuild();
    }

    void inverseNormalized(float dir_x, float dir_y, float rotation, float step_hz_out[4]) const {
        float raw[4];
        float peak = 1e-9f;
        for (int i = 0; i < 4; ++i) {
            const float transl = dir_x * ux_[i] + dir_y * uy_[i];
            const float rot = rotation * (x_[i] * uy_[i] - y_[i] * ux_[i]) * inv_r_;
            raw[i] = transl + rot;
            const float a = fabsf(raw[i]);
            if (a > peak) {
                peak = a;
            }
        }
        const float n = (peak > 1.0f) ? peak : 1.0f;
        const float scale = static_cast<float>(cfg_.max_step_hz);
        for (int i = 0; i < 4; ++i) {
            step_hz_out[i] = (raw[i] / n) * scale;
        }
    }

    void inverseNormalized(const Twist2D& ncmd, float step_hz_out[4]) const {
        float dx = ncmd.vx;
        float dy = ncmd.vy;
        float w = ncmd.omega;
        if (cfg_.invert_vx) {
            dx = -dx;
        }
        if (cfg_.invert_vy) {
            dy = -dy;
        }
        if (cfg_.invert_omega) {
            w = -w;
        }
        inverseNormalized(dx, dy, w, step_hz_out);
    }

    Twist2D forwardSi(const float step_hz[4]) const {
        const float scale = static_cast<float>(cfg_.max_step_hz);
        if (scale < 1.0f) {
            return {};
        }
        float ata[3][3] = {};
        float atb[3] = {};
        for (int i = 0; i < 4; ++i) {
            const float yi = step_hz[i] / scale;
            const float a0 = ux_[i];
            const float a1 = uy_[i];
            const float a2 = (x_[i] * uy_[i] - y_[i] * ux_[i]) * inv_r_;
            const float row[3] = {a0, a1, a2};
            for (int r = 0; r < 3; ++r) {
                atb[r] += row[r] * yi;
                for (int c = 0; c < 3; ++c) {
                    ata[r][c] += row[r] * row[c];
                }
            }
        }
        float x[3] = {};
        solve3x3(ata, atb, x);
        Twist2D si;
        si.vx = x[0] * cfg_.max_vx;
        si.vy = x[1] * cfg_.max_vy;
        si.omega = x[2] * cfg_.max_omega;
        return si;
    }

private:
    RobotConfig cfg_;
    float ux_[4] = {};
    float uy_[4] = {};
    float x_[4] = {};
    float y_[4] = {};
    float inv_r_ = 1.0f;

    void rebuild() {
        const float lx = cfg_.lx_m > 1e-4f ? cfg_.lx_m : 0.19f;
        const float ly = cfg_.ly_m > 1e-4f ? cfg_.ly_m : 0.195f;
        const float px[4] = {+lx, -lx, +lx, -lx};
        const float py[4] = {-ly, -ly, +ly, +ly};
        const float rdx[4] = {-1.0f, -1.0f, +1.0f, +1.0f};
        const float rdy[4] = {-1.0f, +1.0f, -1.0f, +1.0f};
        float rmax = 1e-6f;
        for (int i = 0; i < 4; ++i) {
            x_[i] = px[i];
            y_[i] = py[i];
            const float dn = sqrtf(rdx[i] * rdx[i] + rdy[i] * rdy[i]);
            ux_[i] = rdx[i] / dn;
            uy_[i] = rdy[i] / dn;
            const float ri = sqrtf(px[i] * px[i] + py[i] * py[i]);
            if (ri > rmax) {
                rmax = ri;
            }
        }
        inv_r_ = 1.0f / rmax;
    }

    static bool solve3x3(float A[3][3], const float b[3], float x[3]) {
        float M[3][4] = {
            {A[0][0], A[0][1], A[0][2], b[0]},
            {A[1][0], A[1][1], A[1][2], b[1]},
            {A[2][0], A[2][1], A[2][2], b[2]},
        };
        for (int col = 0; col < 3; ++col) {
            int pivot = col;
            float best = fabsf(M[col][col]);
            for (int row = col + 1; row < 3; ++row) {
                const float v = fabsf(M[row][col]);
                if (v > best) {
                    best = v;
                    pivot = row;
                }
            }
            if (best < 1e-8f) {
                x[0] = x[1] = x[2] = 0.0f;
                return false;
            }
            if (pivot != col) {
                for (int k = 0; k < 4; ++k) {
                    const float tmp = M[col][k];
                    M[col][k] = M[pivot][k];
                    M[pivot][k] = tmp;
                }
            }
            const float diag = M[col][col];
            for (int k = col; k < 4; ++k) {
                M[col][k] /= diag;
            }
            for (int row = 0; row < 3; ++row) {
                if (row == col) {
                    continue;
                }
                const float f = M[row][col];
                for (int k = col; k < 4; ++k) {
                    M[row][k] -= f * M[col][k];
                }
            }
        }
        x[0] = M[0][3];
        x[1] = M[1][3];
        x[2] = M[2][3];
        return true;
    }
};
