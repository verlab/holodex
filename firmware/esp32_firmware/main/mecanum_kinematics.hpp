#ifndef MECANUM_KINEMATICS_HPP
#define MECANUM_KINEMATICS_HPP

#include "robot_types.hpp"

namespace holodex {

class MecanumKinematics {
public:
    MecanumKinematics(float wheel_radius = 0.048f,
                      float track_width = 0.220f,
                      float wheel_base = 0.180f);

    // Cinemática Inversa: converte Twist (vx, vy, wz) para velocidade angular das 4 rodas (rad/s)
    WheelVelocities inverseKinematics(const TwistCommand& cmd) const;

    // Cinemática Direta: converte velocidades angulares das 4 rodas (rad/s) para Twist (vx, vy, wz)
    TwistCommand forwardKinematics(const WheelVelocities& wheels) const;

    // Atualiza a odometria integrando a cinemática direta no tempo
    void updateOdometry(const WheelVelocities& wheels, float dt, OdomState& out_odom);

    // Reinicia a pose acumulada da odometria
    void resetOdometry(float x = 0.0f, float y = 0.0f, float theta = 0.0f);

    // Getters e Setters de geometria
    void setGeometry(float wheel_radius, float track_width, float wheel_base);
    float getWheelRadius() const { return wheel_radius_; }
    float getTrackWidth() const { return track_width_; }
    float getWheelBase() const { return wheel_base_; }

private:
    float wheel_radius_; // Raio da roda Mecanum em metros (R)
    float track_width_;  // Distância entre rodas esquerda e direita em metros (W)
    float wheel_base_;   // Distância entre eixos dianteiro e traseiro em metros (L)
    float k_geom_;       // (L + W) / 2

    float odom_x_;
    float odom_y_;
    float odom_theta_;
};

} // namespace holodex

#endif // MECANUM_KINEMATICS_HPP
