#pragma once
#include <Arduino.h>
#include <vector>

extern "C" {
#include <zenoh-pico.h>
}
#include "ZenohListener.hpp"


struct MotorDTO
{
    int32_t motorA = 0;
    int32_t motorB = 0;
    int32_t motorC = 0;
    int32_t motorD = 0;
} __packed;

class ZenohStateManager
{
public:
    ZenohStateManager(const char* zenoh_router_endpoint);
    void sendMessage(const char* message);

    bool isReady();

    int get_motorA() { return lastPacket.motorA; }
    int get_motorB() { return lastPacket.motorB; }
    int get_motorC() { return lastPacket.motorC; }
    int get_motorD() { return lastPacket.motorD; }

public:
    void addListener(IZenohListener* listener);

private:
    // O Zenoh agora passa referências emprestadas (loaned) para os callbacks
    static void staticMessageHandler(z_loaned_sample_t* sample, void* arg);
    void messageHandler(const z_loaned_sample_t* sample);

private:
    std::vector<IZenohListener*> listeners;
    z_owned_session_t session;
    z_owned_publisher_t pub;
    z_owned_subscriber_t sub;
    MotorDTO lastPacket;
    bool zenoh_ready = false;
};