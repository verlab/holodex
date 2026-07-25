#pragma once

#include "RobotConfig.hpp"
#include "RobotTypes.hpp"
#include <cmath>

/**
 * Exact port of src/cinematic.py + motor layout from src/main.py.
 *
 * Qt path:
 *   set_value(joy_x, -joy_y)
 *   getDirection(y, x, rot)   // note the swap
 *
 * Web stick (sx=right, sy=up) matches pygame forward via:
 *   getDirection(sy, sx, rot)
 */
class HolonomicKinematics {
public:
    explicit HolonomicKinematics(const RobotConfig& cfg) : cfg_(cfg) { rebuild(); }

    void setConfig(const RobotConfig& cfg) {
        cfg_ = cfg;
        rebuild();
    }

    /**
     * Same math as Cinematics.getDirection(direction_x, direction_y, rotation)
     * then × max_step_hz (Qt used MAX_SPEED=25000).
     */
    void inverseNormalized(float direction_x, float direction_y, float rotation,
                           float step_hz_out[4]) const {
        const float scale = static_cast<float>(cfg_.max_step_hz);
        for (int i = 0; i < 4; ++i) {
            const float transl = direction_x * dx_[i] + direction_y * dy_[i];
            const float rot = rotation * (px_[i] * dy_[i] - py_[i] * dx_[i]);
            float hz = (transl + rot) * scale;
            const float sign = (cfg_.motor_sign[i] < 0) ? -1.0f : 1.0f;
            hz *= sign;
            if (hz > scale) {
                hz = scale;
            } else if (hz < -scale) {
                hz = -scale;
            }
            step_hz_out[i] = hz;
        }
    }

    /** Convenience: Twist2D as (direction_x, direction_y, omega) in cinematic space. */
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
        // Approximate SI for odom display only
        const float scale = static_cast<float>(cfg_.max_step_hz);
        if (scale < 1.0f) {
            return {};
        }
        float ata[3][3] = {};
        float atb[3] = {};
        for (int i = 0; i < 4; ++i) {
            const float sign = (cfg_.motor_sign[i] < 0) ? -1.0f : 1.0f;
            const float yi = (step_hz[i] * sign) / scale;
            const float a0 = dx_[i];
            const float a1 = dy_[i];
            const float a2 = px_[i] * dy_[i] - py_[i] * dx_[i];
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
    float dx_[4] = {};
    float dy_[4] = {};
    float px_[4] = {};
    float py_[4] = {};

    void rebuild() {
        // Identical to src/main.py addMotor(...) calls
        const float raw[4][4] = {
            {-1.0f, -1.0f, 23.5f, -14.7f},
            {-1.0f, 1.0f, -23.5f, -14.7f},
            {1.0f, -1.0f, 23.5f, 14.7f},
            {1.0f, 1.0f, -23.5f, 14.7f},
        };
        for (int i = 0; i < 4; ++i) {
            const float dn = sqrtf(raw[i][0] * raw[i][0] + raw[i][1] * raw[i][1]);
            dx_[i] = raw[i][0] / dn;
            dy_[i] = raw[i][1] / dn;
            const float pn = sqrtf(raw[i][2] * raw[i][2] + raw[i][3] * raw[i][3]);
            px_[i] = raw[i][2] / pn;
            py_[i] = raw[i][3] / pn;
        }
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
