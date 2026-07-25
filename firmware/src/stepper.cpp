#include "stepper.hpp"

FastAccelStepperEngine fasEngine = FastAccelStepperEngine();
bool g_fasEngine_initialized = false;

Stepper::Stepper(int pulse_pin, int dir_pin, int enable_pin) {
    if (!g_fasEngine_initialized) {
        fasEngine.init();
        g_fasEngine_initialized = true;
    }

    p_stepper = fasEngine.stepperConnectToPin(pulse_pin);
    if (!p_stepper) {
        Serial.printf("[Stepper] FAILED pulse=%d (pin not usable as STEP)\n", pulse_pin);
        return;
    }

    // Enable active-LOW (TB6600 / A4988 / DRV8825).
    // DIR setup time 300 µs — needed when yaw reverses M2/M3 vs forward.
    p_stepper->setDirectionPin(dir_pin, true /* dir high = forward */, 300);
    p_stepper->setEnablePin(enable_pin, true /* active low */);
    p_stepper->setAutoEnable(false);
    p_stepper->setSpeedInHz(1000);
    p_stepper->setAcceleration(10000);
    p_stepper->enableOutputs();
    Serial.printf("[Stepper] OK pulse=%d dir=%d ena=%d\n", pulse_pin, dir_pin, enable_pin);
}

void Stepper::hardStop() {
    if (!p_stepper) {
        return;
    }
    p_stepper->forceStop();
    commanded_hz_ = 0.0f;
}

void Stepper::setEnabled(bool on) {
    if (!p_stepper) {
        return;
    }
    if (on) {
        p_stepper->enableOutputs();
    } else {
        p_stepper->disableOutputs();
    }
}

void Stepper::setSpeed(int32_t speed_hz, int32_t accel_hz, bool freewheel_if_zero) {
    if (!p_stepper) {
        return;
    }

    if (speed_hz == 0) {
        p_stepper->stopMove();
        commanded_hz_ = 0.0f;
        if (freewheel_if_zero) {
            p_stepper->disableOutputs();
        }
        return;
    }

    const bool forward = speed_hz > 0;
    int32_t mag = speed_hz < 0 ? -speed_hz : speed_hz;

    if (accel_hz < 500) {
        accel_hz = 500;
    }
    if (accel_hz > 30000) {
        accel_hz = 30000;
    }

    // Direction change (critical for yaw: M2/M3 reverse vs forward) — stop first
    // so DIR is stable before pulses resume.
    const bool was_running = commanded_hz_ != 0.0f;
    const bool was_forward = commanded_hz_ > 0.0f;
    if (was_running && was_forward != forward) {
        p_stepper->stopMove();
    }

    p_stepper->enableOutputs();
    p_stepper->setAcceleration(static_cast<uint32_t>(accel_hz));
    p_stepper->setSpeedInHz(static_cast<uint32_t>(mag));
    if (forward) {
        p_stepper->runForward();
    } else {
        p_stepper->runBackward();
    }
    p_stepper->applySpeedAcceleration();
    commanded_hz_ = forward ? static_cast<float>(mag) : -static_cast<float>(mag);
}
