#pragma once

#include <Arduino.h>
#include <Wire.h>

/** NXP PCA9548A 8-channel I2C mux (default address 0x70). */
class Pca9548a {
public:
    explicit Pca9548a(TwoWire& wire = Wire, uint8_t addr = 0x70)
        : wire_(wire), addr_(addr) {}

    bool begin() {
        wire_.beginTransmission(addr_);
        const uint8_t err = wire_.endTransmission();
        if (err != 0) {
            Serial.printf("[MUX] PCA9548A not found at 0x%02X (err=%u)\n", addr_, err);
            return false;
        }
        disableAll();
        ready_ = true;
        Serial.printf("[MUX] PCA9548A ready @ 0x%02X\n", addr_);
        return true;
    }

    bool ready() const { return ready_; }
    uint8_t activeChannel() const { return active_; }

    /** Select one channel 0..7. Skips I2C write if already selected. */
    bool select(uint8_t channel) {
        if (!ready_ || channel > 7) {
            return false;
        }
        if (channel == active_) {
            return true;
        }
        wire_.beginTransmission(addr_);
        wire_.write(static_cast<uint8_t>(1u << channel));
        if (wire_.endTransmission() != 0) {
            active_ = 0xFF;
            return false;
        }
        active_ = channel;
        // PCA9548A switch < 1 µs; short settle is enough at 400 kHz
        delayMicroseconds(50);
        return true;
    }

    bool disableAll() {
        wire_.beginTransmission(addr_);
        wire_.write(static_cast<uint8_t>(0));
        const bool ok = wire_.endTransmission() == 0;
        active_ = 0xFF;
        return ok;
    }

private:
    TwoWire& wire_;
    uint8_t addr_;
    bool ready_ = false;
    uint8_t active_ = 0xFF;
};
