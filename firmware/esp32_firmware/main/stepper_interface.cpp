#include "stepper_interface.hpp"
#include <cmath>
#include <algorithm>

namespace holodex {

StepperInterface::StepperInterface(float steps_per_rev, float max_speed_rad_s, float max_accel_rad_s2)
    : steps_per_rev_(steps_per_rev),
      max_speed_rad_s_(max_speed_rad_s),
      max_accel_rad_s2_(max_accel_rad_s2),
      is_enabled_(false) {
}

bool StepperInterface::init() {
    // Ponto de integração para FastAccelStepperEngine::init()
    // e mapeamento dos pinos STEP, DIR e EN dos 4 drivers (ex: TMC2209 / A4988)
    is_enabled_ = true;
    status_.fl.is_enabled = true;
    status_.fr.is_enabled = true;
    status_.rl.is_enabled = true;
    status_.rr.is_enabled = true;
    return true;
}

void StepperInterface::setWheelVelocities(const WheelVelocities& velocities) {
    if (!is_enabled_) {
        return;
    }

    // Limita as velocidades à velocidade máxima permitida
    target_velocities_.fl = std::clamp(velocities.fl, -max_speed_rad_s_, max_speed_rad_s_);
    target_velocities_.fr = std::clamp(velocities.fr, -max_speed_rad_s_, max_speed_rad_s_);
    target_velocities_.rl = std::clamp(velocities.rl, -max_speed_rad_s_, max_speed_rad_s_);
    target_velocities_.rr = std::clamp(velocities.rr, -max_speed_rad_s_, max_speed_rad_s_);

    // Converte rad/s para frequência de passos (passos/s)
    const float rad_to_steps = steps_per_rev_ / (2.0f * M_PI);

    status_.fl.current_speed_steps_s = static_cast<int32_t>(target_velocities_.fl * rad_to_steps);
    status_.fr.current_speed_steps_s = static_cast<int32_t>(target_velocities_.fr * rad_to_steps);
    status_.rl.current_speed_steps_s = static_cast<int32_t>(target_velocities_.rl * rad_to_steps);
    status_.rr.current_speed_steps_s = static_cast<int32_t>(target_velocities_.rr * rad_to_steps);

    status_.fl.is_running = (status_.fl.current_speed_steps_s != 0);
    status_.fr.is_running = (status_.fr.current_speed_steps_s != 0);
    status_.rl.is_running = (status_.rl.current_speed_steps_s != 0);
    status_.rr.is_running = (status_.rr.current_speed_steps_s != 0);

    // [FastAccelStepper Hook]
    // stepper_fl->setSpeedInHz(abs(status_.fl.current_speed_steps_s));
    // if (status_.fl.current_speed_steps_s > 0) stepper_fl->runForward();
    // else if (status_.fl.current_speed_steps_s < 0) stepper_fl->runBackward();
    // else stepper_fl->stopMove();
}

void StepperInterface::getGroupStatus(StepperGroupStatus& out_status) const {
    out_status = status_;
}

WheelVelocities StepperInterface::getEstimatedWheelVelocities() const {
    const float steps_to_rad = (2.0f * M_PI) / steps_per_rev_;
    return WheelVelocities(
        static_cast<float>(status_.fl.current_speed_steps_s) * steps_to_rad,
        static_cast<float>(status_.fr.current_speed_steps_s) * steps_to_rad,
        static_cast<float>(status_.rl.current_speed_steps_s) * steps_to_rad,
        static_cast<float>(status_.rr.current_speed_steps_s) * steps_to_rad
    );
}

void StepperInterface::enableMotors(bool enable) {
    is_enabled_ = enable;
    status_.fl.is_enabled = enable;
    status_.fr.is_enabled = enable;
    status_.rl.is_enabled = enable;
    status_.rr.is_enabled = enable;
    if (!enable) {
        emergencyStop();
    }
}

void StepperInterface::emergencyStop() {
    target_velocities_ = WheelVelocities(0.0f, 0.0f, 0.0f, 0.0f);
    status_.fl.current_speed_steps_s = 0;
    status_.fr.current_speed_steps_s = 0;
    status_.rl.current_speed_steps_s = 0;
    status_.rr.current_speed_steps_s = 0;
    status_.fl.is_running = false;
    status_.fr.is_running = false;
    status_.rl.is_running = false;
    status_.rr.is_running = false;

    // [FastAccelStepper Hook]
    // stepper_fl->forceStop(); stepper_fr->forceStop(); ...
}

} // namespace holodex
