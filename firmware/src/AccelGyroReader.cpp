#include "AccelGyroReader.hpp"
#include "Pca9548a.hpp"

AccelGyroReader::AccelGyroReader() = default;

void AccelGyroReader::attachMux(Pca9548a* mux, uint8_t channel) {
    mux_ = mux;
    mux_ch_ = channel;
}

bool AccelGyroReader::selectBus() {
    if (!mux_) {
        return true;
    }
    return mux_->select(mux_ch_);
}

bool AccelGyroReader::begin() {
    if (!selectBus()) {
        Serial.println("[IMU] mux select failed before init");
        ready_ = false;
        return false;
    }
    if (bmi160_.I2cInit(0x69) != BMI160_OK) {
        // try alternate address
        if (!selectBus() || bmi160_.I2cInit(0x68) != BMI160_OK) {
            Serial.println("[IMU] BMI160 init failed — check wiring/address/mux");
            ready_ = false;
            return false;
        }
    }
    delay(20);
    ready_ = true;
    Serial.printf("[IMU] BMI160 ready%s\n", mux_ ? " (via PCA9548A)" : "");
    return true;
}

bool AccelGyroReader::read() {
    sample_.valid = false;
    if (!ready_) {
        return false;
    }
    if (!selectBus()) {
        return false;
    }

    int16_t raw[6] = {};
    // Library layout: gyro[3] then accel[3]
    bmi160_.getAccelGyroData(raw);

    // Stamp as close to the transfer as possible: this is the instant the
    // integrators must date the sample to, not the top of the control tick.
    const uint32_t now_us = micros();

    for (int i = 0; i < 3; ++i) {
        gyro_rps_[i] = gyroToRps(raw[i]);
        accel_mps2_[i] = accelToMps2(raw[i + 3]);
    }

    sample_.ax = accel_mps2_[0];
    sample_.ay = accel_mps2_[1];
    sample_.az = accel_mps2_[2];
    sample_.gyro_z = gyro_rps_[2];
    sample_.stamp_us = now_us;
    // micros() overflow is harmless: unsigned wrap gives the right delta
    sample_.dt_s = have_prev_ ? (now_us - last_us_) * 1e-6f : 0.0f;
    // First sample, or a gap so long the integration would be meaningless
    // (OTA, blocked loop): report dt=0 so integrators skip instead of jumping.
    if (sample_.dt_s > 0.25f) {
        sample_.dt_s = 0.0f;
    }
    sample_.valid = true;
    last_us_ = now_us;
    have_prev_ = true;
    return true;
}

float AccelGyroReader::accelToMps2(int16_t val) {
    return (static_cast<float>(val) / 16384.0f) * 9.80665f;
}

float AccelGyroReader::gyroToRps(int16_t val) {
    // BMI160 default ±2000 dps, 16.4 LSB/(°/s) → convert to rad/s
    constexpr float deg2rad = static_cast<float>(M_PI) / 180.0f;
    return (static_cast<float>(val) / 16.4f) * deg2rad;
}
