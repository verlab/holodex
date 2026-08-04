#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <cmath>

/**
 * AMS AS5600 12-bit magnetic angle sensor (I2C 0x36).
 * Raw angle registers: 0x0C (high) / 0x0D (low), 0..4095 → 0..2π.
 */
class As5600 {
public:
    explicit As5600(TwoWire& wire = Wire, uint8_t addr = 0x36)
        : wire_(wire), addr_(addr) {}

    bool begin() {
        uint16_t raw = 0;
        if (!readRaw(raw)) {
            return false;
        }
        ready_ = true;
        last_raw_ = raw;
        last_angle_rad_ = rawToRad(raw);
        return true;
    }

    bool ready() const { return ready_; }

    /** Absolute angle in radians [0, 2π). */
    bool readAngleRad(float& angle_rad) {
        uint16_t raw = 0;
        if (!readRaw(raw)) {
            return false;
        }
        last_raw_ = raw;
        last_angle_rad_ = rawToRad(raw);
        angle_rad = last_angle_rad_;
        return true;
    }

    float lastAngleRad() const { return last_angle_rad_; }
    uint16_t lastRaw() const { return last_raw_; }

    /**
     * Unwrap delta from previous sample (radians, shortest path).
     * Call after a successful readAngleRad / begin.
     */
    float consumeDeltaRad() {
        int32_t d = static_cast<int32_t>(last_raw_) - static_cast<int32_t>(prev_raw_);
        if (d > 2048) {
            d -= 4096;
        } else if (d < -2048) {
            d += 4096;
        }
        prev_raw_ = last_raw_;
        return static_cast<float>(d) * (2.0f * static_cast<float>(M_PI) / 4096.0f);
    }

    void syncPrev() { prev_raw_ = last_raw_; }

private:
    TwoWire& wire_;
    uint8_t addr_;
    bool ready_ = false;
    uint16_t last_raw_ = 0;
    uint16_t prev_raw_ = 0;
    float last_angle_rad_ = 0.0f;

    static float rawToRad(uint16_t raw) {
        return static_cast<float>(raw & 0x0FFF) * (2.0f * static_cast<float>(M_PI) / 4096.0f);
    }

    bool readRaw(uint16_t& out) {
        wire_.beginTransmission(addr_);
        wire_.write(static_cast<uint8_t>(0x0C));
        if (wire_.endTransmission(false) != 0) {
            return false;
        }
        if (wire_.requestFrom(addr_, static_cast<uint8_t>(2)) != 2) {
            return false;
        }
        const uint8_t hi = wire_.read();
        const uint8_t lo = wire_.read();
        out = static_cast<uint16_t>(((hi & 0x0F) << 8) | lo);
        return true;
    }
};
