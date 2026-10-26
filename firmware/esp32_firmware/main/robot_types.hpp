#ifndef ROBOT_TYPES_HPP
#define ROBOT_TYPES_HPP

#include <stdint.h>
#include <stddef.h>

namespace holodex {

// Comandos de velocidade linear e angular (padrão ROS 2 geometry_msgs/Twist)
struct TwistCommand {
    float linear_x;   // m/s (para frente/trás)
    float linear_y;   // m/s (strafe lateral Mecanum)
    float angular_z;  // rad/s (rotação em torno do eixo Z)

    TwistCommand() : linear_x(0.0f), linear_y(0.0f), angular_z(0.0f) {}
    TwistCommand(float lx, float ly, float az) : linear_x(lx), linear_y(ly), angular_z(az) {}
};

// Velocidades de cada uma das 4 rodas Mecanum (em rad/s)
struct WheelVelocities {
    float fl;  // Front-Left
    float fr;  // Front-Right
    float rl;  // Rear-Left
    float rr;  // Rear-Right

    WheelVelocities() : fl(0.0f), fr(0.0f), rl(0.0f), rr(0.0f) {}
    WheelVelocities(float v_fl, float v_fr, float v_rl, float v_rr)
        : fl(v_fl), fr(v_fr), rl(v_rl), rr(v_rr) {}
};

// Posições angulares de cada uma das 4 rodas Mecanum (em radianos)
struct WheelPositions {
    float fl;
    float fr;
    float rl;
    float rr;

    WheelPositions() : fl(0.0f), fr(0.0f), rl(0.0f), rr(0.0f) {}
};

// Estado de Odometria (Pose e Twist 2D/3D no plano)
struct OdomState {
    float x;      // metros no referencial odom
    float y;      // metros no referencial odom
    float theta;  // radianos (orientação Yaw)

    float vx;     // m/s linear no referencial base_link
    float vy;     // m/s lateral no referencial base_link
    float omega;  // rad/s angular no referencial base_link

    // Quatérnion normalizado (Z-up)
    float qx;
    float qy;
    float qz;
    float qw;

    OdomState() : x(0.0f), y(0.0f), theta(0.0f),
                  vx(0.0f), vy(0.0f), omega(0.0f),
                  qx(0.0f), qy(0.0f), qz(0.0f), qw(1.0f) {}
};

// Dados inerciais da IMU
struct ImuState {
    // Quatérnion de orientação estimada
    float qx;
    float qy;
    float qz;
    float qw;

    // Velocidades angulares (rad/s)
    float gx;
    float gy;
    float gz;

    // Acelerações lineares (m/s^2)
    float ax;
    float ay;
    float az;

    ImuState() : qx(0.0f), qy(0.0f), qz(0.0f), qw(1.0f),
                 gx(0.0f), gy(0.0f), gz(0.0f),
                 ax(0.0f), ay(0.0f), az(9.81f) {}
};

// Status de controle de cada motor de passo (FastAccelStepper)
struct StepperChannelStatus {
    int32_t current_position_steps; // Posição absoluta em passos
    int32_t target_position_steps;  // Posição alvo
    int32_t current_speed_steps_s;  // Velocidade atual (passos/s)
    bool is_running;
    bool is_enabled;

    StepperChannelStatus() : current_position_steps(0), target_position_steps(0),
                             current_speed_steps_s(0), is_running(false), is_enabled(false) {}
};

// Status dos 4 motores de passo agrupados
struct StepperGroupStatus {
    StepperChannelStatus fl;
    StepperChannelStatus fr;
    StepperChannelStatus rl;
    StepperChannelStatus rr;
};

// Leituras dos 4 encoders em contagens de pulso (ticks)
struct EncoderTicks {
    int32_t fl;
    int32_t fr;
    int32_t rl;
    int32_t rr;

    EncoderTicks() : fl(0), fr(0), rl(0), rr(0) {}
};

} // namespace holodex

#endif // ROBOT_TYPES_HPP
