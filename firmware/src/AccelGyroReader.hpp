#pragma once

#include "RobotTypes.hpp"

#include <Arduino.h>
#include <DFRobot_BMI160.h>
#include <cmath>

class Pca9548a;

class AccelGyroReader {
public:
    AccelGyroReader();

    /** Optional: IMU behind PCA9548A (prefer ch6; avoid encoder channels). Pass nullptr for main bus. */
    void attachMux(Pca9548a* mux, uint8_t channel);

    bool begin();
    bool ready() const { return ready_; }

    /** Read IMU; returns true on success. */
    bool read();

    /**
     * Last reading with its own timestamp and interval. dt_s spans from the
     * previous *successful* read, so a dropped sample hands the whole elapsed
     * time to the next one instead of losing it.
     */
    const ImuSample& lastSample() const { return sample_; }

    float accelX() const { return accel_mps2_[0]; }
    float accelY() const { return accel_mps2_[1]; }
    float accelZ() const { return accel_mps2_[2]; }

    /** Gyro in rad/s. */
    float gyroX() const { return gyro_rps_[0]; }
    float gyroY() const { return gyro_rps_[1]; }
    float gyroZ() const { return gyro_rps_[2]; }

private:
    bool selectBus();
    static float accelToMps2(int16_t val);
    static float gyroToRps(int16_t val);

    DFRobot_BMI160 bmi160_;
    Pca9548a* mux_ = nullptr;
    uint8_t mux_ch_ = 0;
    bool ready_ = false;
    float accel_mps2_[3] = {};
    float gyro_rps_[3] = {};
    ImuSample sample_{};
    uint32_t last_us_ = 0;
    bool have_prev_ = false;
};
