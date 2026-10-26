#ifndef I_ENCODER_SENSOR_HPP
#define I_ENCODER_SENSOR_HPP

#include "robot_types.hpp"

namespace holodex {

class IEncoderSensor {
public:
    virtual ~IEncoderSensor() = default;

    // Inicialização do barramento e dos sensores de ângulo/encoders
    virtual bool init() = 0;

    // Amostragem e cálculo de posições (rad) e velocidades angulares (rad/s)
    virtual bool update(float dt_s, WheelPositions& out_positions, WheelVelocities& out_velocities) = 0;

    // Retorna a contagem bruta de pulsos/ticks acumulada
    virtual EncoderTicks readTicks() = 0;

    // Retorna a quantidade de sensores ativos e operacionais
    virtual int getActiveSensorsCount() const = 0;

    // Reinicia contadores de pulsos e referências
    virtual void reset() = 0;
};

} // namespace holodex

#endif // I_ENCODER_SENSOR_HPP
