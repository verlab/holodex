#include "mecanum_kinematics.hpp"
#include <cmath>

namespace holodex {

MecanumKinematics::MecanumKinematics(float wheel_radius, float track_width, float wheel_base)
    : odom_x_(0.0f), odom_y_(0.0f), odom_theta_(0.0f) {
    setGeometry(wheel_radius, track_width, wheel_base);
}

void MecanumKinematics::setGeometry(float wheel_radius, float track_width, float wheel_base) {
    wheel_radius_ = (wheel_radius > 0.001f) ? wheel_radius : 0.048f;
    track_width_ = (track_width > 0.001f) ? track_width : 0.220f;
    wheel_base_ = (wheel_base > 0.001f) ? wheel_base : 0.180f;
    k_geom_ = (track_width_ + wheel_base_) / 2.0f;
}

WheelVelocities MecanumKinematics::inverseKinematics(const TwistCommand& cmd) const {
    const float inv_r = 1.0f / wheel_radius_;
    const float k_wz = k_geom_ * cmd.angular_z;

    // Fórmulas analíticas para base 4WD Mecanum:
    // FL = (vx - vy - K * wz) / R
    // FR = (vx + vy + K * wz) / R
    // RL = (vx + vy - K * wz) / R
    // RR = (vx - vy + K * wz) / R
    return WheelVelocities(
        (cmd.linear_x - cmd.linear_y - k_wz) * inv_r,
        (cmd.linear_x + cmd.linear_y + k_wz) * inv_r,
        (cmd.linear_x + cmd.linear_y - k_wz) * inv_r,
        (cmd.linear_x - cmd.linear_y + k_wz) * inv_r
    );
}

TwistCommand MecanumKinematics::forwardKinematics(const WheelVelocities& wheels) const {
    const float r_over_4 = wheel_radius_ * 0.25f;

    // Fórmulas analíticas inversas:
    // vx = (R / 4) * (w_fl + w_fr + w_rl + w_rr)
    // vy = (R / 4) * (-w_fl + w_fr + w_rl - w_rr)
    // wz = (R / (4 * K)) * (-w_fl + w_fr - w_rl + w_rr)
    const float vx = r_over_4 * (wheels.fl + wheels.fr + wheels.rl + wheels.rr);
    const float vy = r_over_4 * (-wheels.fl + wheels.fr + wheels.rl - wheels.rr);
    const float wz = (r_over_4 / k_geom_) * (-wheels.fl + wheels.fr - wheels.rl + wheels.rr);

    return TwistCommand(vx, vy, wz);
}

void MecanumKinematics::updateOdometry(const WheelVelocities& wheels, float dt, OdomState& out_odom) {
    if (dt <= 0.0f) {
        return;
    }

    // Calcula velocidades da base no referencial do robô
    TwistCommand body_twist = forwardKinematics(wheels);

    // Integração no referencial global (odom) com rotação pelo ângulo médio
    const float delta_theta = body_twist.angular_z * dt;
    const float mid_theta = odom_theta_ + (delta_theta * 0.5f);

    const float cos_theta = cosf(mid_theta);
    const float sin_theta = sinf(mid_theta);

    const float delta_x = (body_twist.linear_x * cos_theta - body_twist.linear_y * sin_theta) * dt;
    const float delta_y = (body_twist.linear_x * sin_theta + body_twist.linear_y * cos_theta) * dt;

    odom_x_ += delta_x;
    odom_y_ += delta_y;
    odom_theta_ += delta_theta;

    // Normaliza theta para [-pi, pi]
    while (odom_theta_ > M_PI)  odom_theta_ -= 2.0f * M_PI;
    while (odom_theta_ < -M_PI) odom_theta_ += 2.0f * M_PI;

    // Preenche struct de odometria
    out_odom.x = odom_x_;
    out_odom.y = odom_y_;
    out_odom.theta = odom_theta_;

    out_odom.vx = body_twist.linear_x;
    out_odom.vy = body_twist.linear_y;
    out_odom.omega = body_twist.angular_z;

    // Quatérnion a partir do ângulo Yaw (rotação pura em Z)
    const float half_yaw = odom_theta_ * 0.5f;
    out_odom.qx = 0.0f;
    out_odom.qy = 0.0f;
    out_odom.qz = sinf(half_yaw);
    out_odom.qw = cosf(half_yaw);
}

void MecanumKinematics::resetOdometry(float x, float y, float theta) {
    odom_x_ = x;
    odom_y_ = y;
    odom_theta_ = theta;
}

} // namespace holodex
