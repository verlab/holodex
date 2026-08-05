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
 * TB6600 current / µstep are set by DIP switches (≤4 A), not in firmware.
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
    // M0 FL, M1 RL face opposite DIR sense vs right side → default flip left
    int8_t motor_sign[4] = {-1, -1, 1, 1};

    // AS5600 on PCA9548A: which mux channel feeds motor Mi, and shaft polarity
    // Physical wiring: M0←ch0, M1←ch1, M2←ch4, M3←ch5 (IMU on ch6)
    int8_t enc_ch[4] = {0, 1, 4, 5};
    int8_t enc_sign[4] = {-1, 1, 1, -1};  // M0/M3: raw fwd angle sense vs DIR
    bool enc_odom = true;              // prefer encoder rates for odometry
    bool enc_speed_fb = true;          // PI speed assist from encoder vs command
    float enc_blend = 0.85f;           // weight of encoder vs FAS when both exist
    bool yaw_kalman = true;            // fuse gyro + wheel omega for heading

    /** Clamp encoder mux channel to 0..7 (PCA9548A). */
    static int8_t clampEncCh(int v) {
        if (v < 0) {
            return 0;
        }
        if (v > 7) {
            return 7;
        }
        return static_cast<int8_t>(v);
    }

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
        enc_ch[0] = clampEncCh(prefs.getInt("ech0", enc_ch[0]));
        enc_ch[1] = clampEncCh(prefs.getInt("ech1", enc_ch[1]));
        enc_ch[2] = clampEncCh(prefs.getInt("ech2", enc_ch[2]));
        enc_ch[3] = clampEncCh(prefs.getInt("ech3", enc_ch[3]));
        enc_sign[0] = static_cast<int8_t>(prefs.getInt("es0", enc_sign[0]) < 0 ? -1 : 1);
        enc_sign[1] = static_cast<int8_t>(prefs.getInt("es1", enc_sign[1]) < 0 ? -1 : 1);
        enc_sign[2] = static_cast<int8_t>(prefs.getInt("es2", enc_sign[2]) < 0 ? -1 : 1);
        enc_sign[3] = static_cast<int8_t>(prefs.getInt("es3", enc_sign[3]) < 0 ? -1 : 1);
        enc_odom = prefs.getBool("enc_odom", enc_odom);
        enc_speed_fb = prefs.getBool("enc_fb", enc_speed_fb);
        enc_blend = prefs.getFloat("enc_bl", enc_blend);
        if (enc_blend < 0.0f) {
            enc_blend = 0.0f;
        } else if (enc_blend > 1.0f) {
            enc_blend = 1.0f;
        }
        yaw_kalman = prefs.getBool("yaw_kf", yaw_kalman);

        // v4: quieter teleop defaults
        // v7: clear polarity experiments
        // v8: encoder map / fusion defaults (ch0..3)
        // v9: physical motor order FL/RL/FR/RR, left motor_sign−1
        // v10: encoder mux M0←0 M1←1 M2←4 M3←5 (IMU ch6)
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
        if (prefs.getInt("cfg_ver", 0) < 7) {
            motor_sign[0] = 1;
            motor_sign[1] = 1;
            motor_sign[2] = 1;
            motor_sign[3] = 1;
            invert_omega = false;
            prefs.putInt("ms0", 1);
            prefs.putInt("ms1", 1);
            prefs.putInt("ms2", 1);
            prefs.putInt("ms3", 1);
            prefs.putBool("inv_w", false);
            prefs.putInt("cfg_ver", 7);
        }
        if (prefs.getInt("cfg_ver", 0) < 8) {
            enc_ch[0] = 0;
            enc_ch[1] = 1;
            enc_ch[2] = 2;
            enc_ch[3] = 3;
            enc_sign[0] = enc_sign[1] = enc_sign[2] = enc_sign[3] = 1;
            enc_odom = true;
            enc_speed_fb = true;
            enc_blend = 0.85f;
            yaw_kalman = true;
            prefs.putInt("ech0", 0);
            prefs.putInt("ech1", 1);
            prefs.putInt("ech2", 2);
            prefs.putInt("ech3", 3);
            prefs.putInt("es0", 1);
            prefs.putInt("es1", 1);
            prefs.putInt("es2", 1);
            prefs.putInt("es3", 1);
            prefs.putBool("enc_odom", true);
            prefs.putBool("enc_fb", true);
            prefs.putFloat("enc_bl", 0.85f);
            prefs.putBool("yaw_kf", true);
            prefs.putInt("cfg_ver", 8);
        }
        if (prefs.getInt("cfg_ver", 0) < 9) {
            // Match physical wiring: M0 FL, M1 RL, M2 FR, M3 RR; left DIR flipped
            motor_sign[0] = -1;
            motor_sign[1] = -1;
            motor_sign[2] = 1;
            motor_sign[3] = 1;
            invert_vx = false;
            invert_vy = false;
            invert_omega = false;
            prefs.putInt("ms0", -1);
            prefs.putInt("ms1", -1);
            prefs.putInt("ms2", 1);
            prefs.putInt("ms3", 1);
            prefs.putBool("inv_vx", false);
            prefs.putBool("inv_vy", false);
            prefs.putBool("inv_w", false);
            prefs.putInt("cfg_ver", 9);
            Serial.println("[CFG] migrated to v9: FL/RL/FR/RR + left flip");
        }
        if (prefs.getInt("cfg_ver", 0) < 10) {
            enc_ch[0] = 0;
            enc_ch[1] = 1;
            enc_ch[2] = 4;
            enc_ch[3] = 5;
            prefs.putInt("ech0", 0);
            prefs.putInt("ech1", 1);
            prefs.putInt("ech2", 4);
            prefs.putInt("ech3", 5);
            prefs.putInt("cfg_ver", 10);
            Serial.println("[CFG] migrated to v10: enc M←ch 0,1,4,5 (IMU ch6)");
        }
        if (prefs.getInt("cfg_ver", 0) < 11) {
            // Body-twist speed FB + encoder logical frame (motor_sign). Keep FB on
            // but reset enc_sign to +1 so magnet polarity is independent of DIR flip.
            enc_sign[0] = enc_sign[1] = enc_sign[2] = enc_sign[3] = 1;
            prefs.putInt("es0", 1);
            prefs.putInt("es1", 1);
            prefs.putInt("es2", 1);
            prefs.putInt("es3", 1);
            prefs.putInt("cfg_ver", 11);
            Serial.println("[CFG] migrated to v11: enc logical frame via motor_sign");
        }
        if (prefs.getInt("cfg_ver", 0) < 12) {
            // Bench: forward → raw angle ↑ M0/M2, ↓ M1/M3. Map to electrical then logical.
            enc_sign[0] = -1;
            enc_sign[1] = 1;
            enc_sign[2] = 1;
            enc_sign[3] = -1;
            prefs.putInt("es0", -1);
            prefs.putInt("es1", 1);
            prefs.putInt("es2", 1);
            prefs.putInt("es3", -1);
            prefs.putInt("cfg_ver", 12);
            Serial.println("[CFG] migrated to v12: enc_sign {-1,+1,+1,-1} (M0/M3 flip)");
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
        prefs.putInt("ech0", enc_ch[0]);
        prefs.putInt("ech1", enc_ch[1]);
        prefs.putInt("ech2", enc_ch[2]);
        prefs.putInt("ech3", enc_ch[3]);
        prefs.putInt("es0", enc_sign[0]);
        prefs.putInt("es1", enc_sign[1]);
        prefs.putInt("es2", enc_sign[2]);
        prefs.putInt("es3", enc_sign[3]);
        prefs.putBool("enc_odom", enc_odom);
        prefs.putBool("enc_fb", enc_speed_fb);
        prefs.putFloat("enc_bl", enc_blend);
        prefs.putBool("yaw_kf", yaw_kalman);
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
