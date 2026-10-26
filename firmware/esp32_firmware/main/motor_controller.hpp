#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include "interfaces/i_motor_controller.hpp"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"

namespace holodex {

class StepDirMotorController : public IMotorController {
public:
    // Mapeamento dos pinos conforme definido no hardware Holodex
    static constexpr gpio_num_t PIN_STEP_FL = GPIO_NUM_19;
    static constexpr gpio_num_t PIN_DIR_FL  = GPIO_NUM_18;
    static constexpr gpio_num_t PIN_ENA_FL  = GPIO_NUM_17;

    static constexpr gpio_num_t PIN_STEP_FR = GPIO_NUM_27;
    static constexpr gpio_num_t PIN_DIR_FR  = GPIO_NUM_14;
    static constexpr gpio_num_t PIN_ENA_FR  = GPIO_NUM_13;

    static constexpr gpio_num_t PIN_STEP_RL = GPIO_NUM_23;
    static constexpr gpio_num_t PIN_DIR_RL  = GPIO_NUM_22;
    static constexpr gpio_num_t PIN_ENA_RL  = GPIO_NUM_32;

    static constexpr gpio_num_t PIN_STEP_RR = GPIO_NUM_33;
    static constexpr gpio_num_t PIN_DIR_RR  = GPIO_NUM_25;
    static constexpr gpio_num_t PIN_ENA_RR  = GPIO_NUM_26;

    StepDirMotorController(float steps_per_rev = 6400.0f,
                           float max_speed_rad_s = 25.0f,
                           float max_accel_rad_s2 = 50.0f);
    virtual ~StepDirMotorController();

    bool init() override;
    void setWheelVelocities(const WheelVelocities& velocities) override;
    void emergencyStop() override;
    void setEnabled(bool enabled) override;
    void getGroupStatus(StepperGroupStatus& out_status) const override;
    WheelVelocities getEstimatedWheelVelocities() const override;

private:
    void applyChannelHardware(int channel, float speed_rad_s);
    void updateStepCounters() const;

    float steps_per_rev_;
    float max_speed_rad_s_;
    float max_accel_rad_s2_;
    bool is_enabled_;

    mutable StepperGroupStatus status_;
    WheelVelocities target_velocities_;

    // Periféricos de hardware para geração de passos e contagem
    ledc_timer_t ledc_timers_[4];
    ledc_channel_t ledc_channels_[4];
    pcnt_unit_handle_t pcnt_units_[4];
    pcnt_channel_handle_t pcnt_chans_[4];

    mutable int last_pcnt_count_[4];
    mutable int32_t current_position_steps_[4];
    mutable int no_pulse_cycles_[4];
    bool last_dir_[4];
    uint32_t last_freq_hz_[4];
    bool pulsing_[4];
    int64_t all_stopped_since_us_;
    bool pcnt_initialized_;
};

} // namespace holodex

#endif // MOTOR_CONTROLLER_HPP
