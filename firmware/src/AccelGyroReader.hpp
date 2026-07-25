#pragma once

#include <Arduino.h>
#include <DFRobot_BMI160.h>
#include <cmath>

class AccelGyroReader {
public:
    AccelGyroReader();

    bool begin();
    bool ready() const { return ready_; }

    /** Read IMU; returns true on success. */
    bool read();

    float accelX() const { return accel_mps2_[0]; }
    float accelY() const { return accel_mps2_[1]; }
    float accelZ() const { return accel_mps2_[2]; }

    /** Gyro in rad/s. */
    float gyroX() const { return gyro_rps_[0]; }
    float gyroY() const { return gyro_rps_[1]; }
    float gyroZ() const { return gyro_rps_[2]; }

private:
    static float accelToMps2(int16_t val);
    static float gyroToRps(int16_t val);

    DFRobot_BMI160 bmi160_;
    bool ready_ = false;
    float accel_mps2_[3] = {};
    float gyro_rps_[3] = {};
};
