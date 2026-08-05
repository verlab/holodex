#pragma once

#include "AccelGyroReader.hpp"
#include "EncoderHub.hpp"
#include "MotionController.hpp"
#include "Odometry.hpp"
#include "RobotConfig.hpp"
#include "WifiPortal.hpp"
#include "web_ui_html.hpp"

#include "FirmwareVersion.hpp"
#include "OtaService.hpp"

#include <ArduinoJson.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <cmath>
#include <cstring>

class WebInterface {
public:
    WebInterface(RobotConfig& cfg, MotionController& motion, Odometry& odom, WifiPortal& wifi)
        : cfg_(cfg),
          motion_(motion),
          odom_(odom),
          wifi_(wifi),
          http_(80),
          ws_(81) {}

    void begin(OtaService* ota = nullptr) {
        http_.on("/", HTTP_GET, [this]() { handleRoot(); });
        http_.on("/api/status", HTTP_GET, [this]() { sendOdom(); });
        http_.on("/api/odom", HTTP_GET, [this]() { sendOdom(); });
        http_.on("/api/cfg", HTTP_GET, [this]() { sendCfg(); });

        http_.on("/api/cmd", HTTP_GET, [this]() { handleCmdGet(); });
        http_.on("/api/cmd", HTTP_POST, [this]() { handleCmdPost(); });
        http_.on("/api/estop", HTTP_GET, [this]() { handleEstop(); });
        http_.on("/api/clear_estop", HTTP_GET, [this]() { handleClearEstop(); });
        http_.on("/api/reset_odom", HTTP_GET, [this]() { handleResetOdom(); });
        http_.on("/api/move", HTTP_GET, [this]() { handleMove(); });
        http_.on("/api/move", HTTP_POST, [this]() { handleMove(); });
        http_.on("/api/cfg", HTTP_POST, [this]() { handleCfgPost(); });

        if (ota) {
            ota->attachWeb(http_);
        }

        http_.onNotFound([this]() {
            if (http_.uri().startsWith("/api/")) {
                http_.send(404, "text/plain", "not found");
                return;
            }
            handleRoot();
        });
        http_.begin();

        ws_.begin();
        ws_.onEvent([this](uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
            onWsEvent(num, type, payload, len);
        });
        Serial.println("[Web] Teleop m0..m3  OTA POST /api/update");
    }

    void loop() {
        http_.handleClient();
        ws_.loop();

        // 10 Hz odom push — keep SoftAP responsive for teleop
        const uint32_t now = millis();
        if (now - last_broadcast_ms_ >= 100) {
            last_broadcast_ms_ = now;
            broadcastOdom();
        }
    }

    WebServer& http() { return http_; }

    void setZenohReady(bool ready) { zenoh_ready_ = ready; }

    /** Optional sensors for /api/odom + WS telemetry (mux: enc 0,1,4,5 · IMU ch6). */
    void attachSensors(AccelGyroReader* imu, bool mux_ok) {
        imu_ = imu;
        mux_ok_ = mux_ok;
    }

    using ConfigChangedFn = void (*)(void*);
    void onConfigChanged(ConfigChangedFn fn, void* ctx) {
        cfg_cb_ = fn;
        cfg_ctx_ = ctx;
    }

private:
    RobotConfig& cfg_;
    MotionController& motion_;
    Odometry& odom_;
    WifiPortal& wifi_;
    AccelGyroReader* imu_ = nullptr;
    bool mux_ok_ = false;
    WebServer http_;
    WebSocketsServer ws_;
    uint32_t last_broadcast_ms_ = 0;
    bool zenoh_ready_ = false;
    ConfigChangedFn cfg_cb_ = nullptr;
    void* cfg_ctx_ = nullptr;
    float last_cmd_vx_ = 0;
    float last_cmd_vy_ = 0;
    float last_cmd_w_ = 0;
    uint32_t cmd_count_ = 0;

    void handleRoot() { http_.send_P(200, "text/html", WEB_UI_HTML); }

    void cors() {
        http_.sendHeader("Access-Control-Allow-Origin", "*");
        http_.sendHeader("Cache-Control", "no-store");
    }

    void acceptStick(float dir_x, float dir_y, float omega) {
        last_cmd_vx_ = dir_x;
        last_cmd_vy_ = dir_y;
        last_cmd_w_ = omega;
        cmd_count_++;
        if (motion_.estop()) {
            return;
        }
        Twist2D n;
        n.vx = dir_x;
        n.vy = dir_y;
        n.omega = omega;
        motion_.setWebCommand(n, millis());
    }

    void acceptMotors(float m0, float m1, float m2, float m3, bool test_mode = false) {
        last_cmd_vx_ = m0;
        last_cmd_vy_ = m1;
        last_cmd_w_ = m2;
        cmd_count_++;
        if (motion_.estop()) {
            return;
        }
        motion_.setRawMotorHz(m0, m1, m2, m3, millis(), test_mode);
    }

    void handleCmdGet() {
        // Preferred: raw motor Hz (same payload the Qt app sent over Zenoh)
        if (http_.hasArg("m0") || http_.hasArg("m1") || http_.hasArg("m2") ||
            http_.hasArg("m3")) {
            const bool test = http_.hasArg("test") && http_.arg("test").toInt() != 0;
            acceptMotors(http_.arg("m0").toFloat(), http_.arg("m1").toFloat(),
                         http_.arg("m2").toFloat(), http_.arg("m3").toFloat(), test);
            sendOdom();
            return;
        }
        // Fallback: cinematic stick (dir_x, dir_y, omega) == getDirection args
        acceptStick(http_.hasArg("vx") ? http_.arg("vx").toFloat() : 0.0f,
                    http_.hasArg("vy") ? http_.arg("vy").toFloat() : 0.0f,
                    http_.hasArg("omega") ? http_.arg("omega").toFloat() : 0.0f);
        sendOdom();
    }

    void handleCmdPost() {
        if (http_.hasArg("m0") || http_.hasArg("vx") || http_.hasArg("plain")) {
            if (http_.hasArg("plain")) {
                JsonDocument doc;
                if (!deserializeJson(doc, http_.arg("plain"))) {
                    if (!doc["m0"].isNull() || !doc["motors"].isNull()) {
                        const bool test = doc["test"] | false;
                        if (doc["motors"].is<JsonArray>()) {
                            JsonArray a = doc["motors"].as<JsonArray>();
                            acceptMotors(a[0] | 0.0f, a[1] | 0.0f, a[2] | 0.0f, a[3] | 0.0f,
                                         test);
                        } else {
                            acceptMotors(doc["m0"] | 0.0f, doc["m1"] | 0.0f, doc["m2"] | 0.0f,
                                         doc["m3"] | 0.0f, test);
                        }
                    } else {
                        acceptStick(doc["vx"] | 0.0f, doc["vy"] | 0.0f, doc["omega"] | 0.0f);
                    }
                    sendOdom();
                    return;
                }
            }
            handleCmdGet();
            return;
        }
        http_.send(400, "text/plain", "use m0..m3 or vx,vy,omega");
    }

    void handleEstop() {
        motion_.emergencyStop(millis());
        sendOdom();
    }

    void handleClearEstop() {
        motion_.clearEstop();
        sendOdom();
    }

    void handleResetOdom() {
        motion_.resetAllOdom();
        sendOdom();
    }

    void handleMove() {
        float dx = 0.0f;
        float dy = 0.0f;
        float v = 0.2f;
        if (http_.hasArg("plain")) {
            JsonDocument doc;
            if (!deserializeJson(doc, http_.arg("plain"))) {
                dx = doc["dx"] | 0.0f;
                dy = doc["dy"] | 0.0f;
                v = doc["v"] | doc["speed"] | 0.2f;
            }
        } else {
            dx = http_.hasArg("dx") ? http_.arg("dx").toFloat() : 0.0f;
            dy = http_.hasArg("dy") ? http_.arg("dy").toFloat() : 0.0f;
            if (http_.hasArg("v")) {
                v = http_.arg("v").toFloat();
            } else if (http_.hasArg("speed")) {
                v = http_.arg("speed").toFloat();
            }
        }
        if (!motion_.startMoveBody(dx, dy, v, millis())) {
            cors();
            http_.send(400, "application/json",
                       "{\"ok\":false,\"err\":\"move rejected (estop or zero distance)\"}");
            return;
        }
        sendOdom();
    }

    void handleCfgPost() {
        JsonDocument doc;
        if (http_.hasArg("plain") && !deserializeJson(doc, http_.arg("plain"))) {
            applyConfig(doc);
            cors();
            http_.send(200, "application/json", "{\"ok\":true}");
            return;
        }
        // Also accept query/form fields
        if (http_.hasArg("sta_ssid")) {
            strlcpy(cfg_.sta_ssid, http_.arg("sta_ssid").c_str(), sizeof(cfg_.sta_ssid));
        }
        if (http_.hasArg("zenoh_endpoint")) {
            strlcpy(cfg_.zenoh_endpoint, http_.arg("zenoh_endpoint").c_str(),
                    sizeof(cfg_.zenoh_endpoint));
        }
        cfg_.save();
        if (cfg_cb_) {
            cfg_cb_(cfg_ctx_);
        }
        cors();
        http_.send(200, "application/json", "{\"ok\":true}");
    }

    void sendOdom() {
        cors();
        String out;
        serializeJson(makeOdomDoc(), out);
        http_.send(200, "application/json", out);
    }

    void sendCfg() {
        cors();
        String out;
        serializeJson(makeCfgDoc(), out);
        http_.send(200, "application/json", out);
    }

    void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
        if (type == WStype_CONNECTED) {
            String out;
            serializeJson(makeCfgDoc(), out);
            ws_.sendTXT(num, out);
            return;
        }
        if (type != WStype_TEXT || payload == nullptr || len == 0) {
            return;
        }

        JsonDocument doc;
        if (deserializeJson(doc, payload, len)) {
            return;
        }

        const char* t = doc["type"] | "";
        if (strcmp(t, "cmd") == 0 || strcmp(t, "motors") == 0) {
            if (!doc["m0"].isNull() || doc["motors"].is<JsonArray>()) {
                const bool test = doc["test"] | false;
                if (doc["motors"].is<JsonArray>()) {
                    JsonArray a = doc["motors"].as<JsonArray>();
                    acceptMotors(a[0] | 0.0f, a[1] | 0.0f, a[2] | 0.0f, a[3] | 0.0f, test);
                } else {
                    acceptMotors(doc["m0"] | 0.0f, doc["m1"] | 0.0f, doc["m2"] | 0.0f,
                                 doc["m3"] | 0.0f, test);
                }
            } else {
                acceptStick(doc["vx"] | 0.0f, doc["vy"] | 0.0f, doc["omega"] | 0.0f);
            }
        } else if (strcmp(t, "estop") == 0) {
            motion_.emergencyStop(millis());
        } else if (strcmp(t, "clear_estop") == 0) {
            motion_.clearEstop();
        } else if (strcmp(t, "reset_odom") == 0) {
            motion_.resetAllOdom();
        } else if (strcmp(t, "move") == 0) {
            motion_.startMoveBody(doc["dx"] | 0.0f, doc["dy"] | 0.0f,
                                  doc["v"] | doc["speed"] | 0.2f, millis());
        } else if (strcmp(t, "save_cfg") == 0) {
            applyConfig(doc);
        }
    }

    void applyConfig(JsonDocument& doc) {
        if (doc["sta_ssid"].is<const char*>()) {
            strlcpy(cfg_.sta_ssid, doc["sta_ssid"] | "", sizeof(cfg_.sta_ssid));
        }
        if (doc["sta_password"].is<const char*>()) {
            const char* pw = doc["sta_password"] | "";
            if (pw[0] != '\0') {
                strlcpy(cfg_.sta_password, pw, sizeof(cfg_.sta_password));
            }
        }
        if (doc["zenoh_endpoint"].is<const char*>()) {
            strlcpy(cfg_.zenoh_endpoint, doc["zenoh_endpoint"] | "", sizeof(cfg_.zenoh_endpoint));
        }
        if (!doc["wheel_radius_m"].isNull()) {
            cfg_.wheel_radius_m = doc["wheel_radius_m"].as<float>();
        }
        if (!doc["lx_m"].isNull()) {
            cfg_.lx_m = doc["lx_m"].as<float>();
        }
        if (!doc["ly_m"].isNull()) {
            cfg_.ly_m = doc["ly_m"].as<float>();
        }
        if (!doc["steps_per_rev"].isNull()) {
            cfg_.steps_per_rev = doc["steps_per_rev"].as<float>();
        }
        if (!doc["max_vx"].isNull()) {
            cfg_.max_vx = doc["max_vx"].as<float>();
        }
        if (!doc["max_vy"].isNull()) {
            cfg_.max_vy = doc["max_vy"].as<float>();
        }
        if (!doc["max_omega"].isNull()) {
            cfg_.max_omega = doc["max_omega"].as<float>();
        }
        if (!doc["max_step_hz"].isNull()) {
            cfg_.max_step_hz = doc["max_step_hz"].as<uint32_t>();
        }
        if (!doc["max_accel_hz"].isNull()) {
            cfg_.max_accel_hz = doc["max_accel_hz"].as<uint32_t>();
        }
        if (!doc["invert_vx"].isNull()) {
            cfg_.invert_vx = doc["invert_vx"].as<bool>();
        }
        if (!doc["invert_vy"].isNull()) {
            cfg_.invert_vy = doc["invert_vy"].as<bool>();
        }
        if (!doc["invert_omega"].isNull()) {
            cfg_.invert_omega = doc["invert_omega"].as<bool>();
        }
        if (doc["motor_sign"].is<JsonArray>()) {
            JsonArray arr = doc["motor_sign"].as<JsonArray>();
            for (int i = 0; i < 4 && i < static_cast<int>(arr.size()); ++i) {
                const int v = arr[i].as<int>();
                cfg_.motor_sign[i] = (v < 0) ? -1 : 1;
            }
        }
        if (doc["enc_ch"].is<JsonArray>()) {
            JsonArray arr = doc["enc_ch"].as<JsonArray>();
            for (int i = 0; i < 4 && i < static_cast<int>(arr.size()); ++i) {
                cfg_.enc_ch[i] = RobotConfig::clampEncCh(arr[i].as<int>());
            }
        }
        if (doc["enc_sign"].is<JsonArray>()) {
            JsonArray arr = doc["enc_sign"].as<JsonArray>();
            for (int i = 0; i < 4 && i < static_cast<int>(arr.size()); ++i) {
                cfg_.enc_sign[i] = (arr[i].as<int>() < 0) ? -1 : 1;
            }
        }
        if (!doc["enc_odom"].isNull()) {
            cfg_.enc_odom = doc["enc_odom"].as<bool>();
        }
        if (!doc["enc_speed_fb"].isNull()) {
            cfg_.enc_speed_fb = doc["enc_speed_fb"].as<bool>();
        }
        if (!doc["enc_blend"].isNull()) {
            float b = doc["enc_blend"].as<float>();
            if (b < 0.0f) {
                b = 0.0f;
            } else if (b > 1.0f) {
                b = 1.0f;
            }
            cfg_.enc_blend = b;
        }
        if (!doc["yaw_kalman"].isNull()) {
            cfg_.yaw_kalman = doc["yaw_kalman"].as<bool>();
        }
        cfg_.save();
        if (cfg_cb_) {
            cfg_cb_(cfg_ctx_);
        }
    }

    void fillApFields(JsonDocument& doc) const {
        doc["ap_ssid"] = wifi_.apSsid();
        doc["ap_password"] = wifi_.apPassword();
        char url[48];
        snprintf(url, sizeof(url), "http://%s/", wifi_.apIp().toString().c_str());
        doc["ap_url"] = url;
        doc["ap_up"] = wifi_.apUp();
    }

    static const char* sourceName(CmdSource s) {
        switch (s) {
            case CmdSource::Web:
                return "web";
            case CmdSource::Zenoh:
                return "zenoh";
            default:
                return "none";
        }
    }

    JsonDocument makeCfgDoc() const {
        JsonDocument doc;
        doc["type"] = "cfg";
        doc["version"] = HOLODEX_FW_VERSION;
        doc["build"] = HOLODEX_FW_BUILD;
        doc["sta_ssid"] = cfg_.sta_ssid;
        doc["zenoh_endpoint"] = cfg_.zenoh_endpoint;
        doc["wheel_radius_m"] = cfg_.wheel_radius_m;
        doc["lx_m"] = cfg_.lx_m;
        doc["ly_m"] = cfg_.ly_m;
        doc["steps_per_rev"] = cfg_.steps_per_rev;
        doc["max_vx"] = cfg_.max_vx;
        doc["max_vy"] = cfg_.max_vy;
        doc["max_omega"] = cfg_.max_omega;
        doc["max_step_hz"] = cfg_.max_step_hz;
        doc["max_accel_hz"] = cfg_.max_accel_hz;
        doc["invert_vx"] = cfg_.invert_vx;
        doc["invert_vy"] = cfg_.invert_vy;
        doc["invert_omega"] = cfg_.invert_omega;
        JsonArray ms = doc["motor_sign"].to<JsonArray>();
        JsonArray ech = doc["enc_ch"].to<JsonArray>();
        JsonArray es = doc["enc_sign"].to<JsonArray>();
        for (int i = 0; i < 4; ++i) {
            ms.add(cfg_.motor_sign[i]);
            ech.add(cfg_.enc_ch[i]);
            es.add(cfg_.enc_sign[i]);
        }
        doc["enc_odom"] = cfg_.enc_odom;
        doc["enc_speed_fb"] = cfg_.enc_speed_fb;
        doc["enc_blend"] = cfg_.enc_blend;
        doc["yaw_kalman"] = cfg_.yaw_kalman;
        fillApFields(doc);
        return doc;
    }

    JsonDocument makeOdomDoc() const {
        const auto& s = odom_.state();
        JsonDocument doc;
        doc["type"] = "odom";
        doc["version"] = HOLODEX_FW_VERSION;
        doc["build"] = HOLODEX_FW_BUILD;
        doc["max_step_hz"] = cfg_.max_step_hz;
        // Primary (full) — also mirrored under *_full for UI clarity
        doc["x"] = s.pose.x;
        doc["y"] = s.pose.y;
        doc["theta"] = s.pose.theta;
        doc["vx"] = s.twist.vx;
        doc["vy"] = s.twist.vy;
        doc["omega"] = s.twist.omega;
        doc["x_full"] = s.pose.x;
        doc["y_full"] = s.pose.y;
        doc["theta_full"] = s.pose.theta;
        doc["vx_full"] = s.twist.vx;
        doc["vy_full"] = s.twist.vy;
        doc["omega_full"] = s.twist.omega;
        {
            const auto& w = motion_.odomWheels().state();
            doc["x_wheels"] = w.pose.x;
            doc["y_wheels"] = w.pose.y;
            doc["theta_wheels"] = w.pose.theta;
            doc["vx_wheels"] = w.twist.vx;
            doc["vy_wheels"] = w.twist.vy;
            doc["omega_wheels"] = w.twist.omega;
            const auto& we = motion_.odomWheelsEnc().state();
            doc["x_wheels_enc"] = we.pose.x;
            doc["y_wheels_enc"] = we.pose.y;
            doc["theta_wheels_enc"] = we.pose.theta;
            doc["vx_wheels_enc"] = we.twist.vx;
            doc["vy_wheels_enc"] = we.twist.vy;
            doc["omega_wheels_enc"] = we.twist.omega;
            // legacy keys
            doc["x_step"] = w.pose.x;
            doc["y_step"] = w.pose.y;
            doc["theta_step"] = w.pose.theta;
            doc["x_enc"] = we.pose.x;
            doc["y_enc"] = we.pose.y;
            doc["theta_enc"] = we.pose.theta;
        }
        doc["odom_wheels_fusion"] = motion_.odomWheels().fusionMode();
        doc["odom_wheels_enc_fusion"] = motion_.odomWheelsEnc().fusionMode();
        doc["source"] = sourceName(motion_.activeSource());
        doc["sta"] = wifi_.staConnected() ? wifi_.staIp().toString().c_str() : "down";
        doc["zenoh"] = zenoh_ready_ ? "ready" : "down";
        doc["estop"] = motion_.estop();
        doc["moving"] = motion_.moveActive();
        doc["cmd_vx"] = last_cmd_vx_;
        doc["cmd_vy"] = last_cmd_vy_;
        doc["cmd_w"] = last_cmd_w_;
        doc["cmd_n"] = cmd_count_;
        fillApFields(doc);
        doc["driving"] = motion_.driving();
        doc["mux"] = mux_ok_;

        JsonArray wheels = doc["wheels"].to<JsonArray>();
        JsonArray mok = doc["motor_ok"].to<JsonArray>();
        JsonArray eok = doc["enc_ok"].to<JsonArray>();
        JsonArray eang = doc["enc_deg"].to<JsonArray>();
        JsonArray ehz = doc["enc_hz"].to<JsonArray>();
        const float* applied = motion_.appliedHz();
        EncoderHub* enc = motion_.encoders();
        for (int i = 0; i < 4; ++i) {
            wheels.add(applied ? applied[i] : s.wheel_hz[i]);
            mok.add(motion_.motorOk(i));
            const bool e = enc && enc->motorOk(i);
            eok.add(e);
            if (e) {
                const float deg = enc->angleDeg(i);
                if (isnan(deg)) {
                    eang.add(nullptr);
                } else {
                    eang.add(deg);
                }
                ehz.add(enc->lastHz(i));
            } else {
                eang.add(nullptr);
                ehz.add(0.0f);
            }
        }
        doc["enc_n"] = enc ? enc->okCount() : 0;
        doc["enc_fresh"] = enc && enc->fresh();
        doc["wheel_src"] = motion_.wheelMeasureSrc();
        doc["odom_fusion"] = odom_.fusionMode();
        doc["gyro_bias"] = odom_.gyroBias();
        JsonArray ech = doc["enc_ch"].to<JsonArray>();
        for (int i = 0; i < 4; ++i) {
            ech.add(cfg_.enc_ch[i]);
        }

        const bool imu_ok = imu_ && imu_->ready();
        doc["imu_ok"] = imu_ok;
        if (imu_ok) {
            constexpr float r2d = 180.0f / static_cast<float>(M_PI);
            doc["gx"] = imu_->gyroX();
            doc["gy"] = imu_->gyroY();
            doc["gz"] = imu_->gyroZ();
            doc["gx_dps"] = imu_->gyroX() * r2d;
            doc["gy_dps"] = imu_->gyroY() * r2d;
            doc["gz_dps"] = imu_->gyroZ() * r2d;
            doc["ax"] = imu_->accelX();
            doc["ay"] = imu_->accelY();
            doc["az"] = imu_->accelZ();
        }
        return doc;
    }

    void broadcastOdom() {
        String out;
        serializeJson(makeOdomDoc(), out);
        ws_.broadcastTXT(out);
    }
};
