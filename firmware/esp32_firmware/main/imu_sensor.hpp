#ifndef IMU_SENSOR_HPP
#define IMU_SENSOR_HPP

#include "interfaces/i_imu_sensor.hpp"
#include "i2c_bus.hpp"

namespace holodex {

class Bmi160ImuSensor : public IImuSensor {
public:
    static constexpr uint8_t BMI160_ADDR = 0x69;
    static constexpr uint8_t MUX_ADDR = 0x70;
    static constexpr uint8_t MUX_CH_IMU = 6;

    // Registradores do chip BMI160
    static constexpr uint8_t REG_CHIP_ID = 0x00;
    static constexpr uint8_t REG_DATA_START = 0x0C; // 12 bytes: Gyro (6) + Accel (6)
    static constexpr uint8_t REG_ACC_CONF = 0x40;
    static constexpr uint8_t REG_ACC_RANGE = 0x41;
    static constexpr uint8_t REG_GYR_CONF = 0x42;
    static constexpr uint8_t REG_GYR_RANGE = 0x43;
    static constexpr uint8_t REG_CMD = 0x7E;

    static constexpr uint8_t CMD_SOFT_RESET = 0xB6;
    static constexpr uint8_t CMD_ACC_NORMAL = 0x11;
    static constexpr uint8_t CMD_GYR_NORMAL = 0x15;

    Bmi160ImuSensor(I2cBus& i2c = I2cBus::getInstance());
    virtual ~Bmi160ImuSensor() = default;

    bool init() override;
    bool update(float dt_s, ImuState& out_state) override;
    bool isReady() const override { return ready_; }
    float getGyroZ() const override { return state_.gz; }

private:
    bool selectMuxImuChannel();

    I2cBus& i2c_;
    bool ready_;
    bool on_mux_;
    ImuState state_;
};

} // namespace holodex

#endif // IMU_SENSOR_HPP
