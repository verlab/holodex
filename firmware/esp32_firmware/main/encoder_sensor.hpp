#ifndef ENCODER_SENSOR_HPP
#define ENCODER_SENSOR_HPP

#include "interfaces/i_encoder_sensor.hpp"
#include "i2c_bus.hpp"

namespace holodex {

class As5600MuxEncoder : public IEncoderSensor {
public:
    static constexpr uint8_t MUX_ADDR = 0x70;
    static constexpr uint8_t AS5600_ADDR = 0x36;
    static constexpr uint8_t AS5600_REG_ANGLE = 0x0C;

    // Mapeamento idêntico ao firmware original Holodex:
    // M0 (FL) -> Ch 0, M1 (FR) -> Ch 1, M2 (RL) -> Ch 4, M3 (RR) -> Ch 5
    static constexpr uint8_t CH_FL = 0;
    static constexpr uint8_t CH_FR = 1;
    static constexpr uint8_t CH_RL = 4;
    static constexpr uint8_t CH_RR = 5;

    As5600MuxEncoder(I2cBus& i2c = I2cBus::getInstance());
    virtual ~As5600MuxEncoder() = default;

    bool init() override;
    bool update(float dt_s, WheelPositions& out_positions, WheelVelocities& out_velocities) override;
    EncoderTicks readTicks() override;
    int getActiveSensorsCount() const override { return active_sensors_count_; }
    void reset() override;

private:
    bool selectMuxChannel(uint8_t channel);
    bool readRawAngle(uint8_t channel, uint16_t& out_raw);

    I2cBus& i2c_;
    bool mux_ready_;
    int active_sensors_count_;

    bool channel_active_[4]; // FL, FR, RL, RR
    uint8_t consecutive_errors_[4];
    uint16_t last_raw_[4];
    uint16_t prev_raw_[4];
    int32_t accumulated_ticks_[4];

    WheelPositions current_positions_;
    WheelVelocities current_velocities_;
};

} // namespace holodex

#endif // ENCODER_SENSOR_HPP
