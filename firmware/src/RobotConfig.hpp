#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <cmath>
#include <cstring>

/**
 * Persistent robot / network configuration.
 *
 * Measured chassis (wheel centers):
 *   wheelbase  38 cm → lx = 0.19 m
 *   track      39 cm → ly = 0.195 m
 *   wheel radius 50 mm, 200-step motor @ 32 µsteps → 6400 steps/rev
 * Driver current limit (2 A) is set on the hardware (Vref), not in firmware.
 */
struct RobotConfig {
    // SoftAP (always on for local web teleop)
    char ap_ssid[32] = "HoloDex";
    char ap_password[64] = "holodex123";  // min 8 chars for WPA2

    // Optional STA (for Zenoh / lab network)
    char sta_ssid[32] = "";
    char sta_password[64] = "";
    char zenoh_endpoint[96] = "tcp/192.168.1.100:7447";
    bool zenoh_enabled = true;

    // Holonomic geometry (SI) — half wheelbase / half track
    float wheel_radius_m = 0.05f;
    float lx_m = 0.19f;    // |x| of wheel centers (38 cm axle spacing / 2)
    float ly_m = 0.195f;   // |y| of wheel centers (39 cm wheel spacing / 2)
    float steps_per_rev = 6400.0f;  // 200 * 32 microsteps
    float max_vx = 0.35f;   // m/s
    float max_vy = 0.35f;   // m/s
    float max_omega = 1.5f; // rad/s
    // Quieter defaults @ 32 µsteps (raise in web if you need more speed)
    uint32_t max_step_hz = 12000;
    uint32_t max_accel_hz = 6000;  // gentler ramp → less vibration / noise
    uint32_t cmd_timeout_ms = 600;

    // Axis / motor polarity (toggle on web if a direction is mirrored)
    bool invert_vx = false;
    bool invert_vy = false;
    bool invert_omega = false;
    int8_t motor_sign[4] = {1, 1, 1, 1};  // set -1 to flip an individual wheel

    bool load() {
        Preferences prefs;
        // RW open creates the NVS namespace on first boot (avoids NOT_FOUND spam)
        if (!prefs.begin("holodex", false)) {
            Serial.println("[CFG] NVS unavailable — using compile-time defaults");
            return false;
        }
        prefs.getString("ap_ssid", ap_ssid, sizeof(ap_ssid));
        prefs.getString("ap_pass", ap_password, sizeof(ap_password));
        prefs.getString("sta_ssid", sta_ssid, sizeof(sta_ssid));
        prefs.getString("sta_pass", sta_password, sizeof(sta_password));
        prefs.getString("zenoh_ep", zenoh_endpoint, sizeof(zenoh_endpoint));
        zenoh_enabled = prefs.getBool("zenoh_en", zenoh_enabled);
        wheel_radius_m = prefs.getFloat("r_m", wheel_radius_m);
        lx_m = prefs.getFloat("lx_m", lx_m);
        ly_m = prefs.getFloat("ly_m", ly_m);
        steps_per_rev = prefs.getFloat("spr", steps_per_rev);
        max_vx = prefs.getFloat("max_vx", max_vx);
        max_vy = prefs.getFloat("max_vy", max_vy);
        max_omega = prefs.getFloat("max_w", max_omega);
        max_step_hz = prefs.getUInt("max_hz", max_step_hz);
        max_accel_hz = prefs.getUInt("max_acc", max_accel_hz);
        cmd_timeout_ms = prefs.getUInt("cmd_to", cmd_timeout_ms);
        invert_vx = prefs.getBool("inv_vx", invert_vx);
        invert_vy = prefs.getBool("inv_vy", invert_vy);
        invert_omega = prefs.getBool("inv_w", invert_omega);
        motor_sign[0] = static_cast<int8_t>(prefs.getInt("ms0", motor_sign[0]));
        motor_sign[1] = static_cast<int8_t>(prefs.getInt("ms1", motor_sign[1]));
        motor_sign[2] = static_cast<int8_t>(prefs.getInt("ms2", motor_sign[2]));
        motor_sign[3] = static_cast<int8_t>(prefs.getInt("ms3", motor_sign[3]));
        // v4: quieter teleop defaults + clear bad invert flags
        const int ver = prefs.getInt("cfg_ver", 0);
        if (ver < 4) {
            invert_vx = false;
            invert_vy = false;
            invert_omega = false;
            max_step_hz = 12000;
            max_accel_hz = 6000;
            prefs.putBool("inv_vx", false);
            prefs.putBool("inv_vy", false);
            prefs.putBool("inv_w", false);
            prefs.putUInt("max_hz", max_step_hz);
            prefs.putUInt("max_acc", max_accel_hz);
            prefs.putInt("cfg_ver", 4);
        }
        prefs.end();
        return true;
    }

    bool save() const {
        Preferences prefs;
        if (!prefs.begin("holodex", false)) {
            return false;
        }
        prefs.putString("ap_ssid", ap_ssid);
        prefs.putString("ap_pass", ap_password);
        prefs.putString("sta_ssid", sta_ssid);
        prefs.putString("sta_pass", sta_password);
        prefs.putString("zenoh_ep", zenoh_endpoint);
        prefs.putBool("zenoh_en", zenoh_enabled);
        prefs.putFloat("r_m", wheel_radius_m);
        prefs.putFloat("lx_m", lx_m);
        prefs.putFloat("ly_m", ly_m);
        prefs.putFloat("spr", steps_per_rev);
        prefs.putFloat("max_vx", max_vx);
        prefs.putFloat("max_vy", max_vy);
        prefs.putFloat("max_w", max_omega);
        prefs.putUInt("max_hz", max_step_hz);
        prefs.putUInt("max_acc", max_accel_hz);
        prefs.putUInt("cmd_to", cmd_timeout_ms);
        prefs.putBool("inv_vx", invert_vx);
        prefs.putBool("inv_vy", invert_vy);
        prefs.putBool("inv_w", invert_omega);
        prefs.putInt("ms0", motor_sign[0]);
        prefs.putInt("ms1", motor_sign[1]);
        prefs.putInt("ms2", motor_sign[2]);
        prefs.putInt("ms3", motor_sign[3]);
        prefs.end();
        return true;
    }

    float wheelCircumference() const {
        return 2.0f * static_cast<float>(M_PI) * wheel_radius_m;
    }

    /** Convert linear wheel speed (m/s) to signed step rate (Hz). */
    float mpsToStepHz(float v_mps) const {
        if (wheel_radius_m <= 1e-6f || steps_per_rev <= 1e-6f) {
            return 0.0f;
        }
        return (v_mps / wheelCircumference()) * steps_per_rev;
    }

    float stepHzToMps(float hz) const {
        if (steps_per_rev <= 1e-6f) {
            return 0.0f;
        }
        return (hz / steps_per_rev) * wheelCircumference();
    }
};
