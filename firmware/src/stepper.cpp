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

float Stepper::getCurrentSpeedHz() const {
    if (!p_stepper) {
        return 0.0f;
    }
    // realtime=false reads the ramp generator instead of the step queue. The
    // realtime variant is documented to jump between the two sources during
    // acceleration, which would both add noise to odometry and scramble the
    // per-wheel acceleration split that keeps the ramps synchronised.
    // milliHz → Hz (signed: direction of motion)
    return static_cast<float>(p_stepper->getCurrentSpeedInMilliHz(/*realtime=*/false)) * 0.001f;
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

    // MotionController apportions accel so all four wheels finish their ramp at
    // the same instant. Any clamping here breaks that split — the old floor of
    // 500 let small-delta wheels arrive early and skewed the robot mid-ramp.
    if (accel_hz < 1) {
        accel_hz = 1;
    }

    if (speed_hz == 0) {
        // setAcceleration() is documented as *not* picked up by stopMove(), so
        // the deceleration ramp must be applied before asking it to stop.
        // Otherwise every wheel brakes with whatever acceleration the previous
        // move left behind and they stop out of step.
        const int32_t cur_mhz = p_stepper->getCurrentSpeedInMilliHz(/*realtime=*/false);
        const int32_t cur_hz = (cur_mhz < 0 ? -cur_mhz : cur_mhz) / 1000;
        p_stepper->setAcceleration(accel_hz);
        if (cur_hz > 0) {
            // Hold the ceiling at the present rate: applying must not ramp up
            p_stepper->setSpeedInHz(static_cast<uint32_t>(cur_hz));
            p_stepper->applySpeedAcceleration();
        }
        p_stepper->stopMove();
        commanded_hz_ = 0.0f;
        if (freewheel_if_zero) {
            p_stepper->disableOutputs();
        }
        return;
    }

    const bool forward = speed_hz > 0;
    const int32_t mag = speed_hz < 0 ? -speed_hz : speed_hz;

    // runForward()/runBackward() reverse a running motor on their own and honour
    // the 300 µs DIR setup time. The manual stopMove() that used to precede a
    // reversal only cost those wheels time the others did not pay.
    p_stepper->enableOutputs();
    p_stepper->setAcceleration(accel_hz);
    p_stepper->setSpeedInHz(static_cast<uint32_t>(mag));
    if (forward) {
        p_stepper->runForward();
    } else {
        p_stepper->runBackward();
    }
    p_stepper->applySpeedAcceleration();
    commanded_hz_ = forward ? static_cast<float>(mag) : -static_cast<float>(mag);
}
