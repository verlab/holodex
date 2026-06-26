#include <Arduino.h>
#include <WiFi.h>
#include <algorithm>

extern "C" {
#include <zenoh-pico.h>
}

#include "ZenohStateManager.hpp"
#include "FastAccelStepper.h"

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
const char* SSID = "Verlab";
const char* PASSWORD = "Verlab$Router";
const char* zenoh_router_endpoint = "tcp/150.164.212.252:7447"; // Altere para o IP do seu roteador/agent Zenoh
ZenohStateManager* zenoh = nullptr;




FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;
FastAccelStepper *stepper3 = NULL;
FastAccelStepper *stepper4 = NULL;
uint32_t MAX_ACCEL = 10000;



void setup() {
    Serial.begin(115200);

    // FastAccelStepper engine init
    engine.init();

    // Driver config
    stepper1 = engine.stepperConnectToPin(PULSE_PIN1);
    if (stepper1) {
        stepper1->setDirectionPin(DIR_PIN1);
        stepper1->setEnablePin(ENA_PIN1);
        stepper1->setAutoEnable(true); // Opcional: habilita/desabilita automaticamente
        stepper1->setSpeedInHz(30000);
        stepper1->setAcceleration(10000);
        stepper1->enableOutputs();
    }

    stepper2 = engine.stepperConnectToPin(PULSE_PIN2);
    if (stepper2) {
        stepper2->setDirectionPin(DIR_PIN2);
        stepper2->setEnablePin(ENA_PIN2);
        stepper2->setAutoEnable(true); // Opcional: habilita/desabilita automaticamente
        stepper2->setSpeedInHz(30000);
        stepper2->setAcceleration(10000);
        stepper2->enableOutputs();
    }

    stepper3 = engine.stepperConnectToPin(PULSE_PIN3);
    if (stepper3) {
        stepper3->setDirectionPin(DIR_PIN3);
        stepper3->setEnablePin(ENA_PIN3);
        stepper3->setAutoEnable(true); // Opcional: habilita/desabilita automaticamente
        stepper3->setSpeedInHz(30000);
        stepper3->setAcceleration(10000);
        stepper3->enableOutputs();
    }

    stepper4 = engine.stepperConnectToPin(PULSE_PIN4);
    if (stepper4) {
        stepper4->setDirectionPin(DIR_PIN4);
        stepper4->setEnablePin(ENA_PIN4);
        stepper4->setAutoEnable(true); // Opcional: habilita/desabilita automaticamente
        stepper4->setSpeedInHz(30000);
        stepper4->setAcceleration(10000);
        stepper4->enableOutputs();
    }

    // Starting motors    
    stepper1->runForward();
    stepper2->runBackward();
    stepper3->runForward();
    stepper4->runBackward();
    
    // Starting WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);
    Serial.print("Conectando ao WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Conectado!");

    zenoh = new ZenohStateManager(zenoh_router_endpoint);



    

}


// Função auxiliar para atualizar a velocidade de forma segura
void atualizarMotor(FastAccelStepper* stepper, int32_t velocidadeAlvo, bool andarParaFrente, int32_t accel) {
    if (!stepper) return;

    if (velocidadeAlvo == 0) {
        // Se a velocidade for 0 ou negativa (caso não use sinal para direção), para o motor
        stepper->stopMove();
        return;
    } 
    if(velocidadeAlvo < 0){
         velocidadeAlvo = -velocidadeAlvo;
    }



    // Define a nova velocidade máxima regulamentar
    stepper->setSpeedInHz((uint32_t)velocidadeAlvo);
    stepper->setAcceleration(accel);

    
   
    if (andarParaFrente) {
        stepper->runForward();
    } else {
        stepper->runBackward();
    }
    stepper->applySpeedAcceleration();


}

void loop() {
    static int msg_id = 0;
    char payload[64];
    sprintf(payload, "Mensagem %d do ESP32", msg_id++);

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

    int32_t current_ma = stepper1->getCurrentSpeedInMilliHz() / 1000.0f;
    int32_t current_mb = stepper2->getCurrentSpeedInMilliHz() / 1000.0f;
    int32_t current_mc = stepper3->getCurrentSpeedInMilliHz() / 1000.0f;
    int32_t current_md = stepper4->getCurrentSpeedInMilliHz() / 1000.0f;

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
    
    

    atualizarMotor(stepper1, (int32_t)zenoh->get_motorA(), zenoh->get_motorA()>0, a1);
    atualizarMotor(stepper2, (int32_t)zenoh->get_motorB(), zenoh->get_motorB()>0, a2);
    atualizarMotor(stepper3, (int32_t)zenoh->get_motorC(), zenoh->get_motorC()>0, a3);
    atualizarMotor(stepper4, (int32_t)zenoh->get_motorD(), zenoh->get_motorD()>0, a4);
    


    delay(100); // Envia a cada 2 segundos
}