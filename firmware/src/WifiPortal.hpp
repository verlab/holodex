#pragma once

#include "RobotConfig.hpp"
#include <WiFi.h>

/**
 * SoftAP for web teleop/config + optional STA for Zenoh/lab network.
 * SoftAP is brought up with reduced TX power to limit brownout on USB power.
 */
class WifiPortal {
public:
    explicit WifiPortal(RobotConfig& cfg) : cfg_(cfg) {}

    void begin() {
        WiFi.persistent(false);
        WiFi.disconnect(false, false);
        delay(50);

        // AP-only until STA is configured — less peak current than AP+STA
        const bool want_sta = cfg_.sta_ssid[0] != '\0';
        WiFi.mode(want_sta ? WIFI_AP_STA : WIFI_AP);
        delay(100);

        const IPAddress ap_ip(192, 168, 4, 1);
        const IPAddress gateway(192, 168, 4, 1);
        const IPAddress netmask(255, 255, 255, 0);
        WiFi.softAPConfig(ap_ip, gateway, netmask);

        // Lower TX power reduces brownouts on weak USB cables / hubs
        WiFi.setTxPower(WIFI_POWER_8_5dBm);

        const bool ap_ok =
            WiFi.softAP(cfg_.ap_ssid, cfg_.ap_password, 1 /*channel*/, 0 /*hidden*/, 4);
        delay(200);

        Serial.println("========================================");
        Serial.printf("[WiFi] SoftAP %s\n", ap_ok ? "STARTED" : "FAILED");
        Serial.printf("[WiFi]   SSID     : %s\n", cfg_.ap_ssid);
        Serial.printf("[WiFi]   Password : %s\n", cfg_.ap_password);
        Serial.printf("[WiFi]   URL      : http://%s/\n", WiFi.softAPIP().toString().c_str());
        Serial.printf("[WiFi]   Mode     : %s\n", want_sta ? "AP+STA" : "AP-only");
        Serial.printf("[WiFi]   TX power : 8.5 dBm (brownout-safe)\n");
        Serial.println("========================================");

        if (want_sta) {
            WiFi.setAutoReconnect(true);
            WiFi.begin(cfg_.sta_ssid, cfg_.sta_password);
            Serial.printf("[WiFi] STA connecting to '%s'...\n", cfg_.sta_ssid);
        } else {
            Serial.println("[WiFi] STA not configured (AP-only mode)");
        }
    }

    void loop() {
        if (WiFi.getMode() == WIFI_OFF || !(WiFi.getMode() & WIFI_AP)) {
            if (millis() - last_ap_repair_ms_ > 3000) {
                last_ap_repair_ms_ = millis();
                Serial.println("[WiFi] SoftAP missing — restarting");
                begin();
            }
            return;
        }

        if (cfg_.sta_ssid[0] == '\0') {
            return;
        }

        const wl_status_t st = WiFi.status();
        if (st == WL_CONNECTED) {
            if (!sta_annunciated_) {
                Serial.printf("[WiFi] STA connected  ip=%s\n", WiFi.localIP().toString().c_str());
                sta_annunciated_ = true;
            }
            return;
        }

        sta_annunciated_ = false;
        if (millis() - last_reconnect_ms_ > 15000) {
            last_reconnect_ms_ = millis();
            Serial.println("[WiFi] STA reconnect (SoftAP kept up)");
            WiFi.begin(cfg_.sta_ssid, cfg_.sta_password);
        }
    }

    bool staConnected() const { return WiFi.status() == WL_CONNECTED; }
    bool apUp() const { return (WiFi.getMode() & WIFI_AP) != 0; }
    IPAddress apIp() const { return WiFi.softAPIP(); }
    IPAddress staIp() const { return WiFi.localIP(); }
    const char* apSsid() const { return cfg_.ap_ssid; }
    const char* apPassword() const { return cfg_.ap_password; }
    uint8_t apStations() const { return WiFi.softAPgetStationNum(); }

private:
    RobotConfig& cfg_;
    uint32_t last_reconnect_ms_ = 0;
    uint32_t last_ap_repair_ms_ = 0;
    bool sta_annunciated_ = false;
};
