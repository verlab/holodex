#include "ZenohStateManager.hpp"

#include <ArduinoJson.h>
#include <cstring>

bool ZenohStateManager::declarePub(z_owned_publisher_t& pub, const char* ke, const char* label) {
    z_view_keyexpr_t view;
    z_view_keyexpr_from_str_unchecked(&view, ke);
    if (z_declare_publisher(z_session_loan(&session_), &pub, z_view_keyexpr_loan(&view), NULL) <
        0) {
        Serial.printf("[Zenoh] Failed to declare %s\n", label);
        return false;
    }
    return true;
}

ZenohStateManager::ZenohStateManager(const char* router_endpoint) {
    z_owned_config_t config;
    z_config_default(&config);

    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, "client");
    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, router_endpoint);

    Serial.printf("[Zenoh] Opening session to %s ...\n", router_endpoint);
    if (z_open(&session_, z_config_move(&config), NULL) < 0) {
        Serial.println("[Zenoh] Failed to open session");
        return;
    }
    session_opened_ = true;

    if (zp_start_read_task(z_session_loan_mut(&session_), NULL) < 0 ||
        zp_start_lease_task(z_session_loan_mut(&session_), NULL) < 0) {
        Serial.println("[Zenoh] Failed to start background tasks");
        return;
    }

    // Three streams only (avoid extra flash for legacy alias pubs)
    if (!declarePub(odom_pub_, "holodex/odom", "holodex/odom") ||
        !declarePub(odom_wheels_pub_, "holodex/odom/wheels", "holodex/odom/wheels") ||
        !declarePub(odom_wenc_pub_, "holodex/odom/wheels_enc", "holodex/odom/wheels_enc")) {
        return;
    }

    z_view_keyexpr_t cmd_ke;
    z_view_keyexpr_from_str_unchecked(&cmd_ke, "holodex/cmd_vel");
    z_owned_closure_sample_t cmd_closure;
    z_closure_sample(&cmd_closure, ZenohStateManager::staticCmdHandler, NULL, this);
    if (z_declare_subscriber(z_session_loan(&session_), &cmd_sub_, z_view_keyexpr_loan(&cmd_ke),
                             z_closure_sample_move(&cmd_closure), NULL) < 0) {
        Serial.println("[Zenoh] Failed to declare holodex/cmd_vel subscriber");
        return;
    }

    z_view_keyexpr_t motor_ke;
    z_view_keyexpr_from_str_unchecked(&motor_ke, "demo/pc_to_esp");
    z_owned_closure_sample_t motor_closure;
    z_closure_sample(&motor_closure, ZenohStateManager::staticMotorHandler, NULL, this);
    if (z_declare_subscriber(z_session_loan(&session_), &motor_sub_,
                             z_view_keyexpr_loan(&motor_ke), z_closure_sample_move(&motor_closure),
                             NULL) < 0) {
        Serial.println("[Zenoh] Failed to declare demo/pc_to_esp subscriber");
        return;
    }

    zenoh_ready_ = true;
    Serial.println("[Zenoh] Ready  pub=odom | odom/wheels | odom/wheels_enc");
}

ZenohStateManager::~ZenohStateManager() {
    (void)session_opened_;
}

void ZenohStateManager::staticCmdHandler(z_loaned_sample_t* sample, void* arg) {
    auto* self = static_cast<ZenohStateManager*>(arg);
    if (self) {
        self->onCmdVel(sample);
    }
}

void ZenohStateManager::staticMotorHandler(z_loaned_sample_t* sample, void* arg) {
    auto* self = static_cast<ZenohStateManager*>(arg);
    if (self) {
        self->onMotorDto(sample);
    }
}

void ZenohStateManager::onCmdVel(const z_loaned_sample_t* sample) {
    const z_loaned_bytes_t* payload = z_sample_payload(sample);
    const size_t n = z_bytes_len(payload);
    if (n == 0 || n > 256) {
        return;
    }

    char buf[257];
    z_bytes_reader_t reader = z_bytes_get_reader(payload);
    const size_t got = z_bytes_reader_read(&reader, reinterpret_cast<uint8_t*>(buf), n);
    buf[got < sizeof(buf) ? got : sizeof(buf) - 1] = '\0';

    JsonDocument doc;
    if (deserializeJson(doc, buf)) {
        return;
    }

    Twist2D twist;
    twist.vx = doc["vx"] | 0.0f;
    twist.vy = doc["vy"] | 0.0f;
    if (!doc["omega"].isNull()) {
        twist.omega = doc["omega"].as<float>();
    } else {
        twist.omega = doc["wz"] | 0.0f;
    }

    if (cmd_cb_) {
        cmd_cb_(twist, cmd_ctx_);
    }
}

void ZenohStateManager::onMotorDto(const z_loaned_sample_t* sample) {
    const z_loaned_bytes_t* payload = z_sample_payload(sample);
    if (z_bytes_len(payload) != sizeof(MotorDTO)) {
        return;
    }
    MotorDTO data{};
    z_bytes_reader_t reader = z_bytes_get_reader(payload);
    z_bytes_reader_read(&reader, reinterpret_cast<uint8_t*>(&data), sizeof(MotorDTO));
    if (motor_cb_) {
        motor_cb_(data, motor_ctx_);
    }
}

void ZenohStateManager::publishNamed(z_owned_publisher_t& pub, const OdometryState& state,
                                     const char* source) {
    if (!zenoh_ready_) {
        return;
    }

    JsonDocument doc;
    doc["x"] = state.pose.x;
    doc["y"] = state.pose.y;
    doc["theta"] = state.pose.theta;
    doc["vx"] = state.twist.vx;
    doc["vy"] = state.twist.vy;
    doc["omega"] = state.twist.omega;
    doc["stamp_ms"] = state.stamp_ms;
    doc["source"] = source;
    JsonArray wh = doc["wheels"].to<JsonArray>();
    for (int i = 0; i < 4; ++i) {
        wh.add(state.wheel_hz[i]);
    }

    char buf[320];
    const size_t n = serializeJson(doc, buf, sizeof(buf));
    if (n == 0 || n >= sizeof(buf)) {
        return;
    }

    z_owned_bytes_t data;
    z_bytes_copy_from_str(&data, buf);
    z_publisher_put(z_publisher_loan(&pub), z_bytes_move(&data), NULL);
}

void ZenohStateManager::publishOdometry(const OdometryState& state) {
    publishNamed(odom_pub_, state, "wheels+enc+imu");
}

void ZenohStateManager::publishOdometryWheels(const OdometryState& state) {
    publishNamed(odom_wheels_pub_, state, "wheels");
}

void ZenohStateManager::publishOdometryWheelsEnc(const OdometryState& state) {
    publishNamed(odom_wenc_pub_, state, "wheels+enc");
}

void ZenohStateManager::loop() {}
