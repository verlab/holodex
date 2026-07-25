#pragma once

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>

/**
 * Dual OTA:
 *   1) ArduinoOTA / PlatformIO espota (hostname holodex)
 *   2) Browser upload POST /api/update (multipart firmware.bin)
 */
class OtaService {
public:
    using BusyFn = void (*)(bool busy, void* ctx);

    void begin(BusyFn on_busy = nullptr, void* ctx = nullptr) {
        busy_cb_ = on_busy;
        busy_ctx_ = ctx;

        ArduinoOTA.setHostname("holodex");
        ArduinoOTA.setPassword("holodex");
        ArduinoOTA.onStart([this]() {
            setBusy(true);
            Serial.println("[OTA] ArduinoOTA start");
        });
        ArduinoOTA.onEnd([this]() {
            setBusy(false);
            Serial.println("\n[OTA] ArduinoOTA done — rebooting");
        });
        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            static uint32_t last = 0;
            if (millis() - last > 500) {
                last = millis();
                Serial.printf("[OTA] %u%%\n", progress * 100 / total);
            }
        });
        ArduinoOTA.onError([this](ota_error_t error) {
            setBusy(false);
            Serial.printf("[OTA] Error %u\n", error);
        });
        ArduinoOTA.begin();
        Serial.println("[OTA] Ready  hostname=holodex  password=holodex");
        Serial.println("[OTA] pio: upload_protocol=espota  upload_port=192.168.4.1");
    }

    void attachWeb(WebServer& http) {
        http.on(
            "/api/update", HTTP_POST,
            [this, &http]() { handleUpdateFinish(http); },
            [this, &http]() { handleUpdateUpload(http); });
        http.on("/api/update", HTTP_GET, [&http]() {
            http.send(200, "text/plain",
                      "POST multipart firmware.bin to /api/update\n"
                      "Or: pio run -t upload --upload-port 192.168.4.1 (espota)\n");
        });
    }

    void loop() { ArduinoOTA.handle(); }

    bool busy() const { return busy_; }

private:
    bool busy_ = false;
    BusyFn busy_cb_ = nullptr;
    void* busy_ctx_ = nullptr;
    bool update_error_ = false;

    void setBusy(bool b) {
        busy_ = b;
        if (busy_cb_) {
            busy_cb_(b, busy_ctx_);
        }
    }

    void handleUpdateFinish(WebServer& http) {
        if (Update.hasError() || update_error_) {
            setBusy(false);
            http.send(500, "text/plain", "Update failed");
            return;
        }
        http.sendHeader("Connection", "close");
        http.send(200, "text/plain", "OK — rebooting");
        delay(500);
        ESP.restart();
    }

    void handleUpdateUpload(WebServer& http) {
        HTTPUpload& upload = http.upload();
        if (upload.status == UPLOAD_FILE_START) {
            update_error_ = false;
            setBusy(true);
            Serial.printf("[OTA] Web upload: %s\n", upload.filename.c_str());
            // Sketch size for current partition
            const uint32_t max_size = UPDATE_SIZE_UNKNOWN;
            if (!Update.begin(max_size)) {
                Update.printError(Serial);
                update_error_ = true;
            }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (!update_error_) {
                if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                    Update.printError(Serial);
                    update_error_ = true;
                }
            }
        } else if (upload.status == UPLOAD_FILE_END) {
            if (!update_error_ && Update.end(true)) {
                Serial.printf("[OTA] Success %u bytes\n", upload.totalSize);
            } else {
                Update.printError(Serial);
                update_error_ = true;
                setBusy(false);
            }
        } else if (upload.status == UPLOAD_FILE_ABORTED) {
            Update.end();
            update_error_ = true;
            setBusy(false);
            Serial.println("[OTA] Aborted");
        }
    }
};
