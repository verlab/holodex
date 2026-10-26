#ifndef IMU_INTERFACE_HPP
#define IMU_INTERFACE_HPP

#include "robot_types.hpp"

namespace holodex {

class ImuInterface {
public:
    ImuInterface();

    // Inicialização do barramento I2C/SPI e calibração dos offsets da IMU
    bool init();

    // Atualiza as leituras dos sensores e executa filtro de orientação
    bool update(float dt, ImuState& out_state);

    // Retorna a leitura mais recente em cache
    const ImuState& getState() const { return state_; }

private:
    ImuState state_;
    float yaw_accumulated_;
};

} // namespace holodex

#endif // IMU_INTERFACE_HPP
