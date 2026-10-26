#ifndef YAW_KALMAN_FILTER_HPP
#define YAW_KALMAN_FILTER_HPP

#include "interfaces/i_yaw_filter.hpp"

namespace holodex {

class KalmanYawFilter : public IYawFilter {
public:
    KalmanYawFilter(float q_theta = 1e-5f, float q_bias = 1e-7f, float r_wheel = 0.05f);
    virtual ~KalmanYawFilter() = default;

    void reset(float initial_theta = 0.0f) override;
    void predict(float gyro_z, float dt_s) override;
    void updateWheelOmega(float wheel_omega, float gyro_z) override;

    float getTheta() const override { return theta_; }
    float getBias() const override { return bias_; }
    bool isReady() const override { return is_initialized_; }

private:
    static float wrapToPi(float angle);

    bool is_initialized_;
    float theta_;
    float bias_;

    // Matriz de covariância P (2x2)
    float p00_;
    float p01_;
    float p11_;

    // Covariâncias de processo (Q) e medição (R)
    float q_theta_;
    float q_bias_;
    float r_wheel_;
};

} // namespace holodex

#endif // YAW_KALMAN_FILTER_HPP
