#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <algorithm>

extern "C" {
#include <zenoh-pico.h>
}

#include "ZenohStateManager.hpp"
#include "stepper.hpp"
#include "AccelGyroReader.hpp"


// I2C pins
#define I2C_SDA 21
#define I2C_SCL 5

// Pinos
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


// Zenoh config
const char* SSID = "";
const char* PASSWORD = "";
const char* zenoh_router_endpoint = "tcp/150.164.212.252:7447"; // Altere para o IP do seu roteador/agent Zenoh
ZenohStateManager* zenoh = nullptr;


Stepper* stepper1 = nullptr;
Stepper* stepper2 = nullptr;
Stepper* stepper3 = nullptr;
Stepper* stepper4 = nullptr;
uint32_t MAX_ACCEL = 10000;

// Accel sensor
AccelGyroReader* accelSensor = nullptr;


void setup() {
    Serial.begin(115200);

    // Init i2c
    Wire.begin(I2C_SDA, I2C_SCL);

    // Driver config
    stepper1 = new Stepper(
        PULSE_PIN1,
        DIR_PIN1,
        ENA_PIN1
    );
    stepper2 = new Stepper(
        PULSE_PIN2,
        DIR_PIN2,
        ENA_PIN2
    );
    stepper3 = new Stepper(
        PULSE_PIN3,
        DIR_PIN3,
        ENA_PIN3
    );
    stepper4 = new Stepper(
        PULSE_PIN4,
        DIR_PIN4,
        ENA_PIN4
    );

    accelSensor = new AccelGyroReader();
    
    
    // Starting WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);
    Serial.print("Conectando ao WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Conectado!");


    // Starting zenoh
    zenoh = nullptr;//new ZenohStateManager(zenoh_router_endpoint);

    if(zenoh && zenoh->isReady() == false) {
        delete zenoh;
        zenoh = nullptr;
    }

}

void loop() {
    static int msg_id = 0;
    char payload[64];
    sprintf(payload, "Mensagem %d do ESP32", msg_id++);

    if(zenoh){
        static int n = 0;
        if(n>20)
        {
            zenoh->sendMessage(payload);
            n = 0;
        }
        n++;

        int32_t ma = zenoh->get_motorA();
        int32_t mb = zenoh->get_motorB();
        int32_t mc = zenoh->get_motorC();
        int32_t md = zenoh->get_motorD();

        int32_t current_ma = stepper1->getSpeedHz();
        int32_t current_mb = stepper2->getSpeedHz();
        int32_t current_mc = stepper3->getSpeedHz();
        int32_t current_md = stepper4->getSpeedHz();

        int32_t d1 = abs(ma - current_ma);
        int32_t d2 = abs(mb - current_mb);
        int32_t d3 = abs(mc - current_mc);
        int32_t d4 = abs(md - current_md);

        int32_t max_delta = max(max(d1, d2), max(d3, d4));

        if(max_delta == 0) max_delta = 1;

        int32_t a1 = (MAX_ACCEL * d1) / max_delta;
        int32_t a2 = (MAX_ACCEL * d2) / max_delta;
        int32_t a3 = (MAX_ACCEL * d3) / max_delta;
        int32_t a4 = (MAX_ACCEL * d4) / max_delta;

        stepper1->setSpeed(ma, a1, ma>0);
        stepper2->setSpeed(mb, a2, mb>0);
        stepper3->setSpeed(mc, a3, mc>0);
        stepper4->setSpeed(md, a4, md>0);
        
        delay(100); // Envia a cada 0.1 segundos
    }
    else {
        accelSensor->read();
        delay(50);
    }
    


}