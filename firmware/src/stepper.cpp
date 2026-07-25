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
        Serial.printf("[Stepper] FAILED pulse=%d\n", pulse_pin);
        return;
    }

    p_stepper->setDirectionPin(dir_pin);
    p_stepper->setEnablePin(enable_pin);
    p_stepper->setAutoEnable(false);
    p_stepper->setSpeedInHz(1000);
    p_stepper->setAcceleration(6000);
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

void Stepper::setSpeed(int32_t speed_hz, int32_t accel_hz) {
    if (!p_stepper) {
        return;
    }

    if (speed_hz == 0) {
        if (commanded_hz_ != 0.0f) {
            p_stepper->stopMove();
            commanded_hz_ = 0.0f;
        }
        return;
    }

    bool forward = true;
    int32_t mag = speed_hz;
    if (mag < 0) {
        mag = -mag;
        forward = false;
    }

    // Avoid re-planning every packet (main source of acoustic noise)
    const int32_t prev = static_cast<int32_t>(lroundf(commanded_hz_));
    const bool same_dir = (prev == 0) || ((prev > 0) == forward);
    if (same_dir && prev != 0 && abs(abs(prev) - mag) < 150) {
        return;
    }

    if (accel_hz < 500) {
        accel_hz = 500;
    }
    // Cap accel for smoother omni motion
    if (accel_hz > 20000) {
        accel_hz = 20000;
    }

    p_stepper->setAcceleration(static_cast<uint32_t>(accel_hz));
    p_stepper->setSpeedInHz(static_cast<uint32_t>(mag));

    const bool need_restart =
        (prev == 0) || ((prev > 0) != forward) || !p_stepper->isRunning();
    if (need_restart) {
        if (forward) {
            p_stepper->runForward();
        } else {
            p_stepper->runBackward();
        }
    }
    p_stepper->applySpeedAcceleration();
    commanded_hz_ = forward ? static_cast<float>(mag) : -static_cast<float>(mag);
}
