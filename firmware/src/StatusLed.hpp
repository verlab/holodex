#pragma once

#include <Arduino.h>

/**
 * Onboard LED status (GPIO2 on DOIT ESP32 DevKit).
 *
 * Patterns:
 *   Boot     — three quick flashes
 *   Idle     — slow blink (~1 Hz) SoftAP alive
 *   Driving  — fast blink (~5 Hz) teleop command active
 *   Estop    — double-blink pause
 *   Ota      — very fast flicker
 *   Error    — solid ON
 */
class StatusLed {
public:
    enum class Mode : uint8_t { Idle, Driving, Estop, Ota, Error };

    explicit StatusLed(uint8_t pin = 2) : pin_(pin) {}

    void begin() {
        pinMode(pin_, OUTPUT);
        digitalWrite(pin_, LOW);
        // Boot signature
        for (int i = 0; i < 3; ++i) {
            digitalWrite(pin_, HIGH);
            delay(80);
            digitalWrite(pin_, LOW);
            delay(80);
        }
    }

    void setMode(Mode m) { mode_ = m; }
    Mode mode() const { return mode_; }

    void loop() {
        const uint32_t now = millis();
        switch (mode_) {
            case Mode::Driving:
                blink(now, 100);
                break;
            case Mode::Ota:
                blink(now, 40);
                break;
            case Mode::Error:
                digitalWrite(pin_, HIGH);
                break;
            case Mode::Estop:
                // on 80, off 80, on 80, off 400
                {
                    const uint32_t phase = now % 640;
                    const bool on = (phase < 80) || (phase >= 160 && phase < 240);
                    digitalWrite(pin_, on ? HIGH : LOW);
                }
                break;
            case Mode::Idle:
            default:
                blink(now, 500);
                break;
        }
    }

private:
    uint8_t pin_;
    Mode mode_ = Mode::Idle;
    bool lit_ = false;
    uint32_t last_toggle_ms_ = 0;

    void blink(uint32_t now, uint32_t half_period_ms) {
        if (now - last_toggle_ms_ >= half_period_ms) {
            last_toggle_ms_ = now;
            lit_ = !lit_;
            digitalWrite(pin_, lit_ ? HIGH : LOW);
        }
    }
};
