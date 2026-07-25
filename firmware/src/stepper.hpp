#pragma once

#include "FastAccelStepper.h"
#include <cstdint>

/** Thin wrapper matching the original Holodex stepper behavior (Qt era). */
class Stepper {
public:
    Stepper(int pulse_pin, int dir_pin, int enable_pin);

    bool ok() const { return p_stepper != nullptr; }
    float getSpeedHz() const { return commanded_hz_; }

    /**
     * Signed speed in step Hz.
     * 0 → stopMove; if freewheel_if_zero, also disableOutputs (no holding).
     * Non-zero → enableOutputs + runForward/runBackward (same as old main loop).
     */
    void setSpeed(int32_t speed_hz, int32_t accel_hz, bool freewheel_if_zero = false);

    void setEnabled(bool on);

    /** Immediate halt (E-STOP only). */
    void hardStop();

private:
    FastAccelStepper* p_stepper = nullptr;
    float commanded_hz_ = 0.0f;
};
