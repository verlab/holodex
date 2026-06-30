#include "stepper.hpp"


FastAccelStepperEngine fasEngine = FastAccelStepperEngine();
bool g_fasEngine_initialized = false;


Stepper::Stepper(int pulse_pin, int dir_pin, int enable_pin)
{
    // Initializes engine if needed
    if(g_fasEngine_initialized == false)
    {
        fasEngine.init();
        g_fasEngine_initialized = true;
    }


    // Initializes stepper
    p_stepper = fasEngine.stepperConnectToPin(pulse_pin);

    p_stepper->setDirectionPin(dir_pin);
    p_stepper->setEnablePin(enable_pin);
    //p_stepper->setAutoEnable(true);
    p_stepper->setSpeedInHz(0);
    p_stepper->setAcceleration(0);
    p_stepper->enableOutputs();

}

float Stepper::getSpeedHz()
{
    return 0.0f;
}

void Stepper::setSpeed(int32_t speed, int32_t accel, bool forward)
{
    if (!p_stepper) return;

    if (speed == 0) {
        // Se a velocidade for 0 ou negativa (caso não use sinal para direção), para o motor
        p_stepper->stopMove();
        return;
    } 
    if(speed < 0){
         speed = -speed;
    }



    // Define a nova velocidade máxima regulamentar
    p_stepper->setSpeedInHz((uint32_t)speed);
    p_stepper->setAcceleration(accel);

    
   
    if (forward) {
        p_stepper->runForward();
    } else {
        p_stepper->runBackward();
    }
    p_stepper->applySpeedAcceleration();
}
