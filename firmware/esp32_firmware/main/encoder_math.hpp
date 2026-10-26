#ifndef ENCODER_MATH_HPP
#define ENCODER_MATH_HPP

#include <stdint.h>

namespace holodex {

// Diferença entre duas leituras de 12 bits do AS5600 tratando a passagem por 4095 <-> 0.
inline int32_t unwrapEncoderDelta12(uint16_t current_raw, uint16_t previous_raw)
{
    int32_t delta = (int32_t)current_raw - (int32_t)previous_raw;
    if (delta > 2048)       delta -= 4096;
    else if (delta < -2048) delta += 4096;
    return delta;
}

} // namespace holodex

#endif // ENCODER_MATH_HPP
