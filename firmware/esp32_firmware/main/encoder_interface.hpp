#ifndef ENCODER_INTERFACE_HPP
#define ENCODER_INTERFACE_HPP

#include "robot_types.hpp"

namespace holodex {

class EncoderInterface {
public:
    EncoderInterface(float ticks_per_rev = 1440.0f);

    // Inicialização do hardware de contagem de pulsos (ex: ESP32 PCNT)
    bool init();

    // Lê os pulsos brutos atuais dos 4 canais
    EncoderTicks readTicks();

    // Atualiza leituras de posição angular acumulada e calcula velocidades (rad/s)
    void update(float dt, WheelPositions& out_positions, WheelVelocities& out_velocities);

    // Reinicia contadores de pulsos
    void reset();

    void setTicksPerRev(float ticks_per_rev) { ticks_per_rev_ = ticks_per_rev; }
    float getTicksPerRev() const { return ticks_per_rev_; }

private:
    float ticks_per_rev_;

    EncoderTicks last_ticks_;
    WheelPositions current_positions_;
    WheelVelocities current_velocities_;
};

} // namespace holodex

#endif // ENCODER_INTERFACE_HPP
