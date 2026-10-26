#include "encoder_interface.hpp"
#include <cmath>

namespace holodex {

EncoderInterface::EncoderInterface(float ticks_per_rev)
    : ticks_per_rev_(ticks_per_rev) {
}

bool EncoderInterface::init() {
    // Ponto de integração para o driver PCNT (Pulse Counter) do ESP-IDF:
    // Configura os 4 canais de encoder em quadratura (fase A e fase B)
    reset();
    return true;
}

EncoderTicks EncoderInterface::readTicks() {
    // [Hardware PCNT Hook]
    // Lê os registradores dos 4 contadores de hardware
    return last_ticks_;
}

void EncoderInterface::update(float dt, WheelPositions& out_positions, WheelVelocities& out_velocities) {
    if (dt <= 0.0f) {
        out_positions = current_positions_;
        out_velocities = current_velocities_;
        return;
    }

    EncoderTicks new_ticks = readTicks();

    const float ticks_to_rad = (2.0f * M_PI) / ticks_per_rev_;
    const float inv_dt = 1.0f / dt;

    // Deltas de contagem
    const float delta_rad_fl = static_cast<float>(new_ticks.fl - last_ticks_.fl) * ticks_to_rad;
    const float delta_rad_fr = static_cast<float>(new_ticks.fr - last_ticks_.fr) * ticks_to_rad;
    const float delta_rad_rl = static_cast<float>(new_ticks.rl - last_ticks_.rl) * ticks_to_rad;
    const float delta_rad_rr = static_cast<float>(new_ticks.rr - last_ticks_.rr) * ticks_to_rad;

    // Atualiza posições acumuladas
    current_positions_.fl += delta_rad_fl;
    current_positions_.fr += delta_rad_fr;
    current_positions_.rl += delta_rad_rl;
    current_positions_.rr += delta_rad_rr;

    // Velocidades instantâneas
    current_velocities_.fl = delta_rad_fl * inv_dt;
    current_velocities_.fr = delta_rad_fr * inv_dt;
    current_velocities_.rl = delta_rad_rl * inv_dt;
    current_velocities_.rr = delta_rad_rr * inv_dt;

    last_ticks_ = new_ticks;

    out_positions = current_positions_;
    out_velocities = current_velocities_;
}

void EncoderInterface::reset() {
    last_ticks_ = EncoderTicks();
    current_positions_ = WheelPositions();
    current_velocities_ = WheelVelocities();
}

} // namespace holodex
