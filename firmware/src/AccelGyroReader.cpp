#include "AccelGyroReader.hpp"

AccelGyroReader::AccelGyroReader()
{
    // 1. Inicializa o I2C. O método I2cInit da biblioteca da DFRobot 
    // já faz o soft reset e acorda o sensor (Modo Normal) internamente.
    if (bmi160.I2cInit(0x69) != BMI160_OK) {
        Serial.println("I2C Init failed. Check wiring/address.");
        return;
    }

    // 2. A chamada de softReset() foi REMOVIDA. 
    // Se você fizer um soft reset aqui, o sensor voltará a dormir (Suspend Mode) e lerá 0.

    delay(20); // Pequeno atraso para estabilização
    Serial.println("BMI initialized successfully.");
}

void AccelGyroReader::read()
{
    // Lê os 12 bytes e preenche a struct (6 bytes para Gyro, 6 para Accel)
    bmi160.getAccelGyroData(reinterpret_cast<int16_t*>(&lastRead));

    Serial.print("Accel read:");
    Serial.print(" X ");
    Serial.print(convert(lastRead.accel[0]));
    Serial.print(" Y ");
    Serial.print(convert(lastRead.accel[1]));
    Serial.print(" Z ");
    Serial.println(convert(lastRead.accel[2]));
}

float AccelGyroReader::convert(int16_t val)
{
    return (float(val)/16384.0f) * 9.80665f;
}
