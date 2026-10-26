#ifndef I_MOTOR_CONTROLLER_HPP
#define I_MOTOR_CONTROLLER_HPP

#include "robot_types.hpp"

namespace holodex {

class IMotorController {
public:
    virtual ~IMotorController() = default;

    // Inicialização dos pinos e periféricos dos motores
    virtual bool init() = 0;

    // Aplicação de velocidades angulares para as 4 rodas (rad/s)
    virtual void setWheelVelocities(const WheelVelocities& velocities) = 0;

    // Parada imediata de emergência dos 4 motores
    virtual void emergencyStop() = 0;

    // Habilitação / desabilitação das saídas dos drivers (pino Enable)
    virtual void setEnabled(bool enabled) = 0;

    // Leitura do estado atual dos motores (passos acumulados e velocidade em passos/s)
    virtual void getGroupStatus(StepperGroupStatus& out_status) const = 0;

    // Estimativa de velocidades angulares baseada nos passos atuais (rad/s)
    virtual WheelVelocities getEstimatedWheelVelocities() const = 0;
};

} // namespace holodex

#endif // I_MOTOR_CONTROLLER_HPP
