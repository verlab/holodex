#pragma once

#include "RobotConfig.hpp"
#include <WiFi.h>
#include <cstring>

/**
 * SoftAP for web teleop/config + optional STA for Zenoh/lab network.
 * SoftAP is brought up with reduced TX power to limit brownout on USB power.
 *
 * The ESP32 has a single radio shared by AP and STA. Any STA association that
 * needs a scan sweeps every channel, and while it does the SoftAP is off-channel
 * and the web UI simply stops answering for seconds. So once the lab network is
 * found we pin its channel/BSSID and reconnect directly, never by scan, and the
 * SoftAP is opened on that same channel so the radio never has to hop.
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

        // Follow the STA channel when we know it: the driver would otherwise
        // drag the AP over on association and drop every web client.
        const uint8_t ap_channel = sta_channel_ ? sta_channel_ : 1;
        const bool ap_ok =
            WiFi.softAP(cfg_.ap_ssid, cfg_.ap_password, ap_channel, 0 /*hidden*/, 4);
        delay(200);

        Serial.println("========================================");
        Serial.printf("[WiFi] SoftAP %s\n", ap_ok ? "STARTED" : "FAILED");
        Serial.printf("[WiFi]   SSID     : %s\n", cfg_.ap_ssid);
        Serial.printf("[WiFi]   Password : %s\n", cfg_.ap_password);
        Serial.printf("[WiFi]   URL      : http://%s/\n", WiFi.softAPIP().toString().c_str());
        Serial.printf("[WiFi]   Mode     : %s\n", want_sta ? "AP+STA" : "AP-only");
        Serial.printf("[WiFi]   Channel  : %u\n", ap_channel);
        Serial.printf("[WiFi]   TX power : 8.5 dBm (brownout-safe)\n");
        Serial.println("========================================");

        if (want_sta) {
            hookEvents();
            // Driver-managed reconnect scans at unpredictable moments, which is
            // exactly what makes the web UI freeze — we retry on our own terms.
            WiFi.setAutoReconnect(false);
            staConnect();
        } else {
            Serial.println("[WiFi] STA not configured (AP-only mode)");
        }
    }

    /** Teleop in progress — hold off anything that takes the radio off-channel. */
    void setRadioBusy(bool busy) { radio_busy_ = busy; }

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

        // Driver events, not WiFi.status(): in AP+STA the status enum reports a
        // stopped STA even while the link is up and carrying traffic.
        if (sta_up_) {
            if (!sta_annunciated_) {
                Serial.printf("[WiFi] STA connected  ip=%s  ch=%u\n",
                              WiFi.localIP().toString().c_str(), sta_channel_);
                sta_annunciated_ = true;
                retry_delay_ms_ = kRetryMinMs;
            }
            return;
        }

        sta_annunciated_ = false;
        // A reconnect attempt costs the SoftAP a few hundred ms even when it is
        // scan-free, so never start one under the operator's fingers.
        if (radio_busy_) {
            return;
        }
        if (millis() - last_reconnect_ms_ > retry_delay_ms_) {
            last_reconnect_ms_ = millis();
            Serial.printf("[WiFi] STA reconnect%s in %lus (SoftAP kept up)\n",
                          sta_bssid_valid_ ? " direct" : " scan",
                          static_cast<unsigned long>(retry_delay_ms_ / 1000));
            staConnect();
            // Back off when the network stays absent: a scan-based retry mutes
            // the SoftAP for seconds, and hammering it every 15 s is worse than
            // simply not having Zenoh.
            if (!sta_bssid_valid_ && retry_delay_ms_ < kRetryMaxMs) {
                retry_delay_ms_ *= 2;
            }
        }
    }

    bool staConnected() const { return sta_up_; }
    uint8_t staChannel() const { return sta_channel_; }
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
    volatile bool radio_busy_ = false;
    volatile bool sta_up_ = false;
    volatile uint8_t sta_channel_ = 0;
    uint8_t sta_bssid_[6] = {};
    volatile bool sta_bssid_valid_ = false;
    bool events_hooked_ = false;
    static constexpr uint32_t kRetryMinMs = 15000;
    static constexpr uint32_t kRetryMaxMs = 240000;
    uint32_t retry_delay_ms_ = kRetryMinMs;

    static WifiPortal* s_self_;

    /**
     * Association details come from the driver event, which is also the only
     * place the BSSID/channel are reliable. Caching them is what lets later
     * reconnects skip the all-channel scan that mutes the SoftAP.
     */
    static void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
        WifiPortal* self = s_self_;
        if (!self) {
            return;
        }
        if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
            self->sta_channel_ = info.wifi_sta_connected.channel;
            memcpy(self->sta_bssid_, info.wifi_sta_connected.bssid, sizeof(self->sta_bssid_));
            self->sta_bssid_valid_ = true;
        } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
            self->sta_up_ = true;
        } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
            self->sta_up_ = false;
        }
    }

    void hookEvents() {
        if (events_hooked_) {
            return;
        }
        s_self_ = this;
        WiFi.onEvent(onWifiEvent);
        events_hooked_ = true;
    }

    /**
     * Associate to the lab network. With a known channel+BSSID the driver goes
     * straight there; without one it must sweep every channel, which mutes the
     * SoftAP for seconds.
     */
    void staConnect() {
        if (sta_bssid_valid_ && sta_channel_) {
            WiFi.begin(cfg_.sta_ssid, cfg_.sta_password, sta_channel_, sta_bssid_);
        } else {
            WiFi.begin(cfg_.sta_ssid, cfg_.sta_password);
        }
        Serial.printf("[WiFi] STA connecting to '%s'...\n", cfg_.sta_ssid);
    }
};

inline WifiPortal* WifiPortal::s_self_ = nullptr;
