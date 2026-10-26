#include "yaw_kalman_filter.hpp"
#include <cmath>

namespace holodex {

KalmanYawFilter::KalmanYawFilter(float q_theta, float q_bias, float r_wheel)
    : is_initialized_(false),
      theta_(0.0f),
      bias_(0.0f),
      p00_(0.05f),
      p01_(0.0f),
      p11_(0.01f),
      q_theta_(q_theta),
      q_bias_(q_bias),
      r_wheel_(r_wheel)
{
}

void KalmanYawFilter::reset(float initial_theta)
{
    theta_ = wrapToPi(initial_theta);
    bias_ = 0.0f;
    p00_ = 0.05f;
    p01_ = 0.0f;
    p11_ = 0.01f;
    is_initialized_ = true;
}

void KalmanYawFilter::predict(float gyro_z, float dt_s)
{
    if (!is_initialized_ || dt_s <= 0.0f) {
        return;
    }

    // Predição do estado: theta = theta + (gyro_z - bias) * dt
    const float omega_corrected = gyro_z - bias_;
    theta_ = wrapToPi(theta_ + omega_corrected * dt_s);

    // Propagação da covariância: P = F * P * F^T + Q
    // F = [[1, -dt], [0, 1]]
    const float p00 = p00_ - dt_s * (p01_ + p01_) + (dt_s * dt_s) * p11_ + q_theta_;
    const float p01 = p01_ - dt_s * p11_;
    const float p11 = p11_ + q_bias_;

    p00_ = p00;
    p01_ = p01;
    p11_ = p11;
}

void KalmanYawFilter::updateWheelOmega(float wheel_omega, float gyro_z)
{
    if (!is_initialized_) {
        return;
    }

    // Inovação do filtro (medição esperada a partir das rodas)
    const float predicted_omega = gyro_z - bias_;
    const float innov = wheel_omega - predicted_omega;

    // Covariância da inovação S = H * P * H^T + R
    const float s = p11_ + r_wheel_;
    if (s < 1e-9f) {
        return;
    }

    // Ganho de Kalman K = P * H^T / S
    const float k0 = p01_ / s;
    const float k1 = p11_ / s;

    // Atualização de estado
    theta_ = wrapToPi(theta_ + k0 * innov);
    bias_ += k1 * innov;

    // Atualização da covariância Joseph-lite
    p00_ -= k0 * p01_;
    p01_ -= k0 * p11_;
    p11_ -= k1 * p11_;

    if (p00_ < 1e-6f) p00_ = 1e-6f;
    if (p11_ < 1e-6f) p11_ = 1e-6f;
}

float KalmanYawFilter::wrapToPi(float angle)
{
    while (angle > (float)M_PI)  angle -= 2.0f * (float)M_PI;
    while (angle < -(float)M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}

} // namespace holodex
