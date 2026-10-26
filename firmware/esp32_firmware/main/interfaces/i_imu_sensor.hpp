#ifndef I_IMU_SENSOR_HPP
#define I_IMU_SENSOR_HPP

#include "robot_types.hpp"

namespace holodex {

class IImuSensor {
public:
    virtual ~IImuSensor() = default;

    // Inicialização do barramento e do chip IMU (ex: BMI160)
    virtual bool init() = 0;

    // Leitura dos dados inerciais, aplicação de filtros e atualização do quatérnion
    virtual bool update(float dt_s, ImuState& out_state) = 0;

    // Indica se a IMU física foi encontrada e está respondendo
    virtual bool isReady() const = 0;

    // Retorna a velocidade angular no eixo Z (rad/s)
    virtual float getGyroZ() const = 0;
};

} // namespace holodex

#endif // I_IMU_SENSOR_HPP
