#include <Arduino.h>
#include <Wire.h>
#include <cmath>

#include "AccelGyroReader.hpp"
#include "EncoderHub.hpp"
#include "FirmwareVersion.hpp"
#include "HolonomicKinematics.hpp"
#include "MotionController.hpp"
#include "Odometry.hpp"
#include "OtaService.hpp"
#include "Pca9548a.hpp"
#include "RobotConfig.hpp"
#include "StatusLed.hpp"
#include "WebInterface.hpp"
#include "WifiPortal.hpp"
#include "ZenohStateManager.hpp"
#include "stepper.hpp"

#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

// I2C
#define I2C_SDA 21
#define I2C_SCL 5
// PCA9548A: AS5600 on ch0,1,4,5 (M0..M3); BMI160 on ch6
#define MUX_I2C_ADDR 0x70
#define MUX_CH_IMU 6

// Built-in LED on DOIT ESP32 DevKit
#define STATUS_LED_PIN 2

// Stepper pins (pulse / dir / enable)
#define PULSE_PIN1 19
#define DIR_PIN1 18
#define ENA_PIN1 17

#define PULSE_PIN2 27
#define DIR_PIN2 14
#define ENA_PIN2 13

#define PULSE_PIN3 23
#define DIR_PIN3 22
#define ENA_PIN3 32

#define PULSE_PIN4 33
#define DIR_PIN4 25
#define ENA_PIN4 26

static RobotConfig g_cfg;
static Stepper* g_motors[4] = {};
static Pca9548a* g_mux = nullptr;
static EncoderHub* g_enc = nullptr;
static AccelGyroReader* g_imu = nullptr;
static HolonomicKinematics* g_kin = nullptr;
static Odometry* g_odom = nullptr;         // wheels+enc+IMU (primary)
static Odometry* g_odom_step = nullptr;    // wheels only (FAS, no IMU)
static Odometry* g_odom_enc = nullptr;     // wheels+enc blend (no IMU)
static MotionController* g_motion = nullptr;
static WifiPortal* g_wifi = nullptr;
static WebInterface* g_web = nullptr;
static ZenohStateManager* g_zenoh = nullptr;
static StatusLed g_led(STATUS_LED_PIN);
static OtaService g_ota;

// Odometry snapshot handed to ZenohTask (owner: main loop, reader: ZenohTask)
static OdometryState g_state_main;
static OdometryState g_state_step;
static OdometryState g_state_enc;
static SemaphoreHandle_t g_odom_mutex = nullptr;
static volatile bool g_zenoh_ready = false;
static volatile bool g_zenoh_restart_req = false;

static uint32_t g_last_ctrl_ms = 0;
static float g_gyro_z = NAN;
static bool g_gyro_valid = false;
static uint32_t g_gyro_stamp_ms = 0;

static void onOtaBusy(bool busy, void* /*ctx*/) {
    g_led.setMode(busy ? StatusLed::Mode::Ota : StatusLed::Mode::Idle);
    if (busy && g_motion) {
        g_motion->emergencyStop(millis());
    }
}

// Zenoh subscriber callbacks run on the zenoh-pico read task. Driving
// MotionController from there races the control loop and reaches
// FastAccelStepper from two threads at once, so commands are parked in this
// slot and applied by loop(). Only the newest one matters.
static portMUX_TYPE g_zcmd_mux = portMUX_INITIALIZER_UNLOCKED;
static Twist2D g_zcmd_twist{};
static MotorDTO g_zcmd_motors{};
static bool g_zcmd_twist_pending = false;
static bool g_zcmd_motors_pending = false;

static void onZenohCmd(const Twist2D& twist, void* /*ctx*/) {
    portENTER_CRITICAL(&g_zcmd_mux);
    g_zcmd_twist = twist;
    g_zcmd_twist_pending = true;
    portEXIT_CRITICAL(&g_zcmd_mux);
}

static void onZenohMotors(const MotorDTO& m, void* /*ctx*/) {
    portENTER_CRITICAL(&g_zcmd_mux);
    g_zcmd_motors = m;
    g_zcmd_motors_pending = true;
    portEXIT_CRITICAL(&g_zcmd_mux);
}

static void drainZenohCommands(uint32_t now) {
    Twist2D twist{};
    MotorDTO motors{};
    bool have_twist = false;
    bool have_motors = false;

    portENTER_CRITICAL(&g_zcmd_mux);
    if (g_zcmd_twist_pending) {
        twist = g_zcmd_twist;
        g_zcmd_twist_pending = false;
        have_twist = true;
    }
    if (g_zcmd_motors_pending) {
        motors = g_zcmd_motors;
        g_zcmd_motors_pending = false;
        have_motors = true;
    }
    portEXIT_CRITICAL(&g_zcmd_mux);

    if (!g_motion) {
        return;
    }
    if (have_motors) {
        g_motion->setRawMotorHz(static_cast<float>(motors.motorA),
                                static_cast<float>(motors.motorB),
                                static_cast<float>(motors.motorC),
                                static_cast<float>(motors.motorD), now);
    } else if (have_twist) {
        g_motion->setZenohCommand(twist, now);
    }
}

/**
 * Sole owner of g_zenoh: session open, publishing and teardown all happen here.
 * The blocking calls stay off the Arduino loop (z_open against an unreachable
 * router costs seconds) and no other thread can free the session while a
 * publish is in flight.
 */
static void ZenohTask(void* /*param*/) {
    uint32_t last_pub_ms = 0;
    uint32_t last_try_ms = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));
        const uint32_t now = millis();

        if (g_zenoh_restart_req) {
            g_zenoh_restart_req = false;
            delete g_zenoh;
            g_zenoh = nullptr;
            g_zenoh_ready = false;
        }

        if (!g_zenoh) {
            if (!g_cfg.zenoh_enabled || g_cfg.zenoh_endpoint[0] == '\0') {
                continue;
            }
            if (!g_wifi || !g_wifi->staConnected()) {
                continue;
            }
            if (now - last_try_ms < 8000) {
                continue;
            }
            last_try_ms = now;

            auto* z = new ZenohStateManager(g_cfg.zenoh_endpoint);
            if (!z->isReady()) {
                delete z;
                Serial.println("[Zenoh] Not ready — will retry");
                continue;
            }
            z->setCommandCallback(onZenohCmd, nullptr);
            z->setMotorCallback(onZenohMotors, nullptr);
            g_zenoh = z;
            g_zenoh_ready = true;
            continue;
        }

        if (now - last_pub_ms < 50) {
            continue;
        }
        last_pub_ms = now;

        OdometryState local_main;
        OdometryState local_step;
        OdometryState local_enc;
        if (!g_odom_mutex || xSemaphoreTake(g_odom_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
            continue;
        }
        local_main = g_state_main;
        local_step = g_state_step;
        local_enc = g_state_enc;
        xSemaphoreGive(g_odom_mutex);

        g_zenoh->publishOdometry(local_main);
        g_zenoh->publishOdometryWheels(local_step);
        g_zenoh->publishOdometryWheelsEnc(local_enc);
    }
}

static void onConfigChanged(void* /*ctx*/) {
    if (g_kin) {
        g_kin->setConfig(g_cfg);
    }
    if (g_enc) {
        g_enc->logMap();
    }
    // Teardown is ZenohTask's job — freeing the session from the web handler
    // would pull it out from under an in-flight publish.
    g_zenoh_restart_req = true;
    if (g_web) {
        g_web->setZenohReady(false);
    }
    if (g_wifi) {
        g_wifi->begin();
    }
}

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== HoloDex firmware ===");
    Serial.printf("[Boot] Version %s (%s)\n", HOLODEX_FW_VERSION, HOLODEX_FW_BUILD);

    g_led.begin();
    g_cfg.load();

    g_wifi = new WifiPortal(g_cfg);
    g_wifi->begin();

    g_ota.begin(onOtaBusy, nullptr);

    g_motors[0] = new Stepper(PULSE_PIN1, DIR_PIN1, ENA_PIN1);
    g_motors[1] = new Stepper(PULSE_PIN2, DIR_PIN2, ENA_PIN2);
    g_motors[2] = new Stepper(PULSE_PIN3, DIR_PIN3, ENA_PIN3);
    g_motors[3] = new Stepper(PULSE_PIN4, DIR_PIN4, ENA_PIN4);

    g_kin = new HolonomicKinematics(g_cfg);
    g_odom = new Odometry(*g_kin, g_cfg);
    g_odom_step = new Odometry(*g_kin, g_cfg);
    g_odom_enc = new Odometry(*g_kin, g_cfg);
    g_motion = new MotionController(g_cfg, g_motors, *g_kin, *g_odom, *g_odom_step, *g_odom_enc);

    g_web = new WebInterface(g_cfg, *g_motion, *g_odom, *g_wifi);
    g_web->begin(&g_ota);
    g_web->onConfigChanged(onConfigChanged, nullptr);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);  // PCA9548A / AS5600 / BMI160 all support fast-mode
    Wire.setTimeOut(8);     // fail fast on NACK — never stall the 50 Hz loop

    g_imu = new AccelGyroReader();
    g_mux = new Pca9548a(Wire, MUX_I2C_ADDR);
    bool mux_ok = false;
    if (g_mux->begin()) {
        mux_ok = true;
        g_enc = new EncoderHub(*g_mux, g_cfg);
        // Probe only mapped encoder channels — never poke IMU mux ch6 as AS5600
        if (g_enc->begin(/*also_scan_all=*/false)) {
            g_motion->attachEncoders(g_enc);
            Serial.printf("[Boot] Encoders: %d/4 AS5600 (M←ch 0,1,4,5)\n", g_enc->okCount());
        } else {
            Serial.println("[Boot] Mux OK but no AS5600 — odom uses stepper speed");
        }
        g_mux->disableAll();
        g_imu->attachMux(g_mux, MUX_CH_IMU);
        if (g_imu->begin()) {
            Serial.printf("[Boot] BMI160 on PCA9548A ch%u\n", MUX_CH_IMU);
        } else {
            Serial.printf("[IMU] Not on mux ch%u — retry on main bus\n", MUX_CH_IMU);
            g_imu->attachMux(nullptr, 0);
            if (!g_imu->begin()) {
                Serial.println("[Boot] IMU optional — continuing without gyro fusion");
            }
        }
    } else {
        delete g_mux;
        g_mux = nullptr;
        Serial.println("[Boot] No PCA9548A — IMU/encoders on main bus if present");
        if (!g_imu->begin()) {
            Serial.println("[Boot] IMU optional — continuing without gyro fusion");
        }
    }
    g_web->attachSensors(g_imu, mux_ok);

    g_last_ctrl_ms = millis();
    Serial.println("[Boot] LED: slow=idle  fast=driving  double=ESTOP  flicker=OTA");
    Serial.println("[Boot] http://192.168.4.1/  OTA password=holodex");
    Serial.println("[Boot] Move: GET /api/move?dx=0.3&dy=0&v=0.2");

    g_odom_mutex = xSemaphoreCreateMutex();
    // Core 0 alongside the WiFi stack, below the Arduino loop priority: motor
    // control must never wait on the network. Stack covers z_open + JSON.
    xTaskCreatePinnedToCore(ZenohTask, "Zenoh", 16384, nullptr, 1, nullptr, 0);
    Serial.println("[Boot] Zenoh bridge task started (core 0)");
}

void loop() {
    const uint32_t now = millis();

    g_led.loop();
    g_ota.loop();

    if (g_ota.busy()) {
        return;  // don't teleop / zenoh during flash
    }

    if (g_wifi) {
        g_wifi->loop();
    }
    if (g_web) {
        g_web->loop();
    }

    // Keep the radio on-channel while the operator is driving: an STA
    // (re)association pulls it away from the SoftAP and freezes the web UI.
    if (g_wifi && g_motion) {
        g_wifi->setRadioBusy(g_motion->activeSource() != CmdSource::None ||
                             g_motion->driving());
    }

    // LED mode from robot state
    if (g_motion) {
        if (g_motion->estop()) {
            g_led.setMode(StatusLed::Mode::Estop);
        } else if (g_motion->driving()) {
            g_led.setMode(StatusLed::Mode::Driving);
        } else {
            g_led.setMode(StatusLed::Mode::Idle);
        }
    }

    if (!g_motion || !g_odom) {
        return;
    }

    drainZenohCommands(now);

    // Control / sensors / odom @ 50 Hz.
    // One mux sweep per tick: mapped AS5600 channels then BMI160 (no disableAll).
    const uint32_t dt_ms = now - g_last_ctrl_ms;
    if (dt_ms >= 20) {
        const float dt_s = dt_ms * 0.001f;
        g_last_ctrl_ms = now;

        if (g_enc) {
            g_enc->sample();
        }
        if (g_imu && g_imu->ready()) {
            if (g_imu->read()) {
                g_gyro_z = g_imu->gyroZ();  // rad/s
                g_gyro_valid = true;
                g_gyro_stamp_ms = now;
            }
        }
        // Drop gyro if last good sample is too old (IMU unplugged mid-run)
        if (g_gyro_valid && (now - g_gyro_stamp_ms) > 100) {
            g_gyro_valid = false;
            g_gyro_z = NAN;
            if (g_odom) {
                g_odom->clearGyro();
            }
            if (g_odom_step) {
                g_odom_step->clearGyro();
            }
            if (g_odom_enc) {
                g_odom_enc->clearGyro();
            }
        }

        g_motion->update(dt_s, g_gyro_z, g_gyro_valid, now);
    }

    // Refresh the snapshot ZenohTask publishes from. Zero timeout: telemetry is
    // never worth stalling the control loop for.
    if (g_odom_mutex && dt_ms >= 20 && xSemaphoreTake(g_odom_mutex, 0) == pdTRUE) {
        g_state_main = g_odom->state();
        g_state_step = g_odom_step->state();
        g_state_enc = g_odom_enc->state();
        xSemaphoreGive(g_odom_mutex);
    }

    if (g_web) {
        g_web->setZenohReady(g_zenoh_ready);
    }
}
