#ifndef STEPPER_INTERFACE_HPP
#define STEPPER_INTERFACE_HPP

#include "robot_types.hpp"

namespace holodex {

class StepperInterface {
public:
    StepperInterface(float steps_per_rev = 3200.0f,
                     float max_speed_rad_s = 25.0f,
                     float max_accel_rad_s2 = 50.0f);

    // Inicialização dos pinos e timers dos motores de passo (preparado para FastAccelStepper)
    bool init();

    // Comanda velocidades angulares para as 4 rodas (rad/s)
    void setWheelVelocities(const WheelVelocities& velocities);

    // Retorna o status completo dos 4 motores para telemetria ROS 2
    void getGroupStatus(StepperGroupStatus& out_status) const;

    // Converte a velocidade atual de passos para velocidades angulares das rodas (rad/s)
    WheelVelocities getEstimatedWheelVelocities() const;

    // Habilita ou desabilita os drivers dos motores (pinos Enable)
    void enableMotors(bool enable);

    // Parada de emergência imediata (desaceleração máxima)
    void emergencyStop();

    // Configuração de microstepping e passos
    void setStepsPerRev(float steps_per_rev) { steps_per_rev_ = steps_per_rev; }
    float getStepsPerRev() const { return steps_per_rev_; }

private:
    float steps_per_rev_;
    float max_speed_rad_s_;
    float max_accel_rad_s2_;

    bool is_enabled_;

    // Estruturas de status interno dos 4 canais
    StepperGroupStatus status_;

    // Velocidades comandadas atualmente (rad/s)
    WheelVelocities target_velocities_;
};

} // namespace holodex

#endif // STEPPER_INTERFACE_HPP
