#pragma once
#include "FastAccelStepper.h"



class Stepper
{
public:
    Stepper(int pulse_pin, int dir_pin, int enable_pin);

    float getSpeedHz();

    void setSpeed(int32_t speed, int32_t accel, bool forward);

private:
    FastAccelStepper* p_stepper;

};