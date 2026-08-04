#include <Arduino.h>
#include <Wire.h>
#include <cmath>

#include "AccelGyroReader.hpp"
#include "FirmwareVersion.hpp"
#include "HolonomicKinematics.hpp"
#include "MotionController.hpp"
#include "Odometry.hpp"
#include "OtaService.hpp"
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
static AccelGyroReader* g_imu = nullptr;
static HolonomicKinematics* g_kin = nullptr;
static Odometry* g_odom = nullptr;
static MotionController* g_motion = nullptr;
static WifiPortal* g_wifi = nullptr;
static WebInterface* g_web = nullptr;
static ZenohStateManager* g_zenoh = nullptr;
static StatusLed g_led(STATUS_LED_PIN);
static OtaService g_ota;

static uint32_t g_last_ctrl_ms = 0;
static uint32_t g_last_zenoh_odom_ms = 0;
static uint32_t g_last_zenoh_try_ms = 0;
static float g_gyro_z = NAN;
static bool g_gyro_valid = false;
static uint32_t g_gyro_stamp_ms = 0;

static void onOtaBusy(bool busy, void* /*ctx*/) {
    g_led.setMode(busy ? StatusLed::Mode::Ota : StatusLed::Mode::Idle);
    if (busy && g_motion) {
        g_motion->emergencyStop(millis());
    }
}

static void onZenohCmd(const Twist2D& twist, void* /*ctx*/) {
    if (g_motion) {
        g_motion->setZenohCommand(twist, millis());
    }
}

static void onZenohMotors(const MotorDTO& m, void* /*ctx*/) {
    if (g_motion) {
        g_motion->setRawMotorHz(static_cast<float>(m.motorA), static_cast<float>(m.motorB),
                                static_cast<float>(m.motorC), static_cast<float>(m.motorD),
                                millis());
    }
}

static void tryStartZenoh() {
    if (!g_cfg.zenoh_enabled || g_cfg.zenoh_endpoint[0] == '\0') {
        return;
    }
    if (!g_wifi || !g_wifi->staConnected()) {
        return;
    }
    if (g_zenoh && g_zenoh->isReady()) {
        return;
    }

    if (g_zenoh) {
        delete g_zenoh;
        g_zenoh = nullptr;
    }

    Serial.println("[Zenoh] Starting bridge...");
    g_zenoh = new ZenohStateManager(g_cfg.zenoh_endpoint);
    if (!g_zenoh->isReady()) {
        delete g_zenoh;
        g_zenoh = nullptr;
        Serial.println("[Zenoh] Not ready — will retry");
    } else {
        g_zenoh->setCommandCallback(onZenohCmd, nullptr);
        g_zenoh->setMotorCallback(onZenohMotors, nullptr);
    }

    if (g_web) {
        g_web->setZenohReady(g_zenoh && g_zenoh->isReady());
    }
}

static void onConfigChanged(void* /*ctx*/) {
    if (g_kin) {
        g_kin->setConfig(g_cfg);
    }
    if (g_zenoh) {
        delete g_zenoh;
        g_zenoh = nullptr;
    }
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
    g_odom = new Odometry(*g_kin);
    g_motion = new MotionController(g_cfg, g_motors, *g_kin, *g_odom);

    g_web = new WebInterface(g_cfg, *g_motion, *g_odom, *g_wifi);
    g_web->begin(&g_ota);
    g_web->onConfigChanged(onConfigChanged, nullptr);

    Wire.begin(I2C_SDA, I2C_SCL);
    g_imu = new AccelGyroReader();
    if (!g_imu->begin()) {
        Serial.println("[Boot] IMU optional — continuing without gyro fusion");
    }

    g_last_ctrl_ms = millis();
    Serial.println("[Boot] LED: slow=idle  fast=driving  double=ESTOP  flicker=OTA");
    Serial.println("[Boot] http://192.168.4.1/  OTA password=holodex");
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

    // Control / odom @ 50 Hz. IMU read at the same rate (was 10 Hz with 20 ms dt).
    const uint32_t dt_ms = now - g_last_ctrl_ms;
    if (dt_ms >= 20) {
        const float dt_s = dt_ms * 0.001f;
        g_last_ctrl_ms = now;

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
        }

        g_motion->update(dt_s, g_gyro_z, g_gyro_valid, now);
    }

    if ((!g_zenoh || !g_zenoh->isReady()) && now - g_last_zenoh_try_ms > 8000) {
        g_last_zenoh_try_ms = now;
        tryStartZenoh();
    }

    if (g_zenoh && g_zenoh->isReady() && now - g_last_zenoh_odom_ms >= 200) {
        g_last_zenoh_odom_ms = now;
        g_zenoh->publishOdometry(g_odom->state());
    }
}
