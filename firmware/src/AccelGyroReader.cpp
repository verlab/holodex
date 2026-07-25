#include "AccelGyroReader.hpp"

AccelGyroReader::AccelGyroReader() = default;

bool AccelGyroReader::begin() {
    if (bmi160_.I2cInit(0x69) != BMI160_OK) {
        Serial.println("[IMU] BMI160 init failed — check wiring/address");
        ready_ = false;
        return false;
    }
    delay(20);
    ready_ = true;
    Serial.println("[IMU] BMI160 ready");
    return true;
}

bool AccelGyroReader::read() {
    if (!ready_) {
        return false;
    }

    int16_t raw[6] = {};
    // Library layout: gyro[3] then accel[3]
    bmi160_.getAccelGyroData(raw);

    for (int i = 0; i < 3; ++i) {
        gyro_rps_[i] = gyroToRps(raw[i]);
        accel_mps2_[i] = accelToMps2(raw[i + 3]);
    }
    return true;
}

float AccelGyroReader::accelToMps2(int16_t val) {
    return (static_cast<float>(val) / 16384.0f) * 9.80665f;
}

float AccelGyroReader::gyroToRps(int16_t val) {
    // BMI160 default ±2000 dps, 16.4 LSB/(°/s)
    constexpr float deg2rad = static_cast<float>(M_PI) / 180.0f;
    return (static_cast<float>(val) / 16.4f) * deg2rad;
}
