#pragma once
#include <Arduino.h>
#include <DFRobot_BMI160.h>

class AccelGyroReader
{
public:
    AccelGyroReader();

    void read();
    
private:
    float convert(int16_t val);

    struct dataStruct {
        int16_t gyro[3];
        int16_t accel[3];
    } __attribute__((packed)); 

    DFRobot_BMI160 bmi160;
    dataStruct lastRead;
};