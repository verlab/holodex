#include "imu_sensor.hpp"
#include <cmath>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "Bmi160ImuSensor";

namespace holodex {

Bmi160ImuSensor::Bmi160ImuSensor(I2cBus& i2c)
    : i2c_(i2c),
      ready_(false),
      on_mux_(false)
{
    state_.qx = 0.0f;
    state_.qy = 0.0f;
    state_.qz = 0.0f;
    state_.qw = 1.0f;
    state_.gx = 0.0f;
    state_.gy = 0.0f;
    state_.gz = 0.0f;
    state_.ax = 0.0f;
    state_.ay = 0.0f;
    state_.az = 9.80665f;
}

bool Bmi160ImuSensor::selectMuxImuChannel()
{
    uint8_t mask = (1 << MUX_CH_IMU);
    esp_err_t err = i2c_.write(MUX_ADDR, &mask, 1, 10);
    return (err == ESP_OK);
}

bool Bmi160ImuSensor::init()
{
    ESP_LOGI(TAG, "Inicializando sensor inercial BMI160...");
    ready_ = false;
    on_mux_ = false;

    if (!i2c_.isInitialized()) {
        i2c_.init();
    }

    // 1. Tenta comunicar via canal 6 do multiplexador PCA9548A
    if (selectMuxImuChannel()) {
        uint8_t chip_id = 0;
        if (i2c_.writeRead(BMI160_ADDR, REG_CHIP_ID, &chip_id, 1, 10) == ESP_OK) {
            if (chip_id == 0xD1) {
                on_mux_ = true;
                ESP_LOGI(TAG, "BMI160 detectado no canal %d do PCA9548A (Chip ID: 0x%02X).", MUX_CH_IMU, chip_id);
            }
        }
    }

    // 2. Se não detectou no mux, tenta diretamente no barramento I2C principal
    if (!on_mux_) {
        uint8_t disable_mux = 0;
        i2c_.write(MUX_ADDR, &disable_mux, 1, 10);

        uint8_t chip_id = 0;
        if (i2c_.writeRead(BMI160_ADDR, REG_CHIP_ID, &chip_id, 1, 10) == ESP_OK && chip_id == 0xD1) {
            ESP_LOGI(TAG, "BMI160 detectado diretamente no barramento principal (Chip ID: 0x%02X).", chip_id);
        } else {
            ESP_LOGW(TAG, "BMI160 nao encontrado (ID != 0xD1). Operando em modo de emulacao inercial.");
            ready_ = false;
            return false;
        }
    }

    // 3. Soft reset
    uint8_t cmd_reset[2] = {REG_CMD, CMD_SOFT_RESET};
    i2c_.write(BMI160_ADDR, cmd_reset, 2, 10);
    vTaskDelay(pdMS_TO_TICKS(50));

    if (on_mux_) selectMuxImuChannel();

    // 4. Ativação dos modos Normal de potência PRIMEIRO: em suspend o BMI160 não aceita
    // escritas de configuração de forma confiável.
    uint8_t cmd_acc[2] = {REG_CMD, CMD_ACC_NORMAL};
    i2c_.write(BMI160_ADDR, cmd_acc, 2, 10);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t cmd_gyr[2] = {REG_CMD, CMD_GYR_NORMAL};
    i2c_.write(BMI160_ADDR, cmd_gyr, 2, 10);
    vTaskDelay(pdMS_TO_TICKS(80));

    // 5. Configuração explícita de Faixa e ODR (P6), com pausa entre escritas
    struct RegWrite { uint8_t reg; uint8_t value; const char* name; };
    const RegWrite cfg[] = {
        {REG_ACC_RANGE, 0x03, "ACC_RANGE"}, // ±2g
        {REG_ACC_CONF,  0x28, "ACC_CONF"},  // ODR 100 Hz, modo normal
        {REG_GYR_RANGE, 0x00, "GYR_RANGE"}, // ±2000 °/s
        {REG_GYR_CONF,  0x28, "GYR_CONF"},  // ODR 100 Hz, modo normal
    };

    bool config_ok = true;
    for (const RegWrite& w : cfg) {
        if (on_mux_) selectMuxImuChannel();
        uint8_t buf[2] = {w.reg, w.value};
        esp_err_t werr = i2c_.write(BMI160_ADDR, buf, 2, 10);
        vTaskDelay(pdMS_TO_TICKS(2));

        // Readback: só os 4 bits baixos de ACC_RANGE e os 3 de GYR_RANGE são significativos
        uint8_t rb = 0;
        esp_err_t rerr = i2c_.writeRead(BMI160_ADDR, w.reg, &rb, 1, 10);
        const uint8_t mask = (w.reg == REG_ACC_RANGE) ? 0x0F : (w.reg == REG_GYR_RANGE) ? 0x07 : 0xFF;
        if (werr != ESP_OK || rerr != ESP_OK || (rb & mask) != (w.value & mask)) {
            ESP_LOGE(TAG, "BMI160 %s: escreveu 0x%02X, leu 0x%02X (werr=%s, rerr=%s)",
                     w.name, w.value, rb, esp_err_to_name(werr), esp_err_to_name(rerr));
            config_ok = false;
        }
    }

    if (!config_ok) {
        ESP_LOGE(TAG, "Configuracao do BMI160 nao confere: escalas SI poderiam estar erradas. IMU desativada.");
        ready_ = false;
        return false;
    }
    ESP_LOGI(TAG, "BMI160 configurado e validado (±2g, ±2000 dps, 100 Hz).");

    ready_ = true;
    return true;
}

bool Bmi160ImuSensor::update(float dt_s, ImuState& out_state)
{
    if (!ready_) {
        out_state = state_;
        return false;
    }

    if (on_mux_) {
        selectMuxImuChannel();
    }

    uint8_t raw[12] = {0};
    esp_err_t err = i2c_.writeRead(BMI160_ADDR, REG_DATA_START, raw, 12, 10);
    if (err != ESP_OK) {
        out_state = state_;
        return false;
    }

    // Leitura dos registros brutos de 16 bits com sinal
    int16_t gx_raw = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t gy_raw = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t gz_raw = (int16_t)((raw[5] << 8) | raw[4]);

    int16_t ax_raw = (int16_t)((raw[7] << 8) | raw[6]);
    int16_t ay_raw = (int16_t)((raw[9] << 8) | raw[8]);
    int16_t az_raw = (int16_t)((raw[11] << 8) | raw[10]);

    // Conversão pura para unidades SI (sem deadband e sem integração) (P7)
    // Giroscópio: ±2000 dps (16.4 LSB por deg/s) -> rad/s
    constexpr float kDegToRad = (float)M_PI / 180.0f;
    constexpr float kGyrScale = (1.0f / 16.4f) * kDegToRad;
    state_.gx = (float)gx_raw * kGyrScale;
    state_.gy = (float)gy_raw * kGyrScale;
    state_.gz = (float)gz_raw * kGyrScale;

    // Acelerômetro: ±2g (16384 LSB por g) -> m/s²
    constexpr float kAccScale = (1.0f / 16384.0f) * 9.80665f;
    state_.ax = (float)ax_raw * kAccScale;
    state_.ay = (float)ay_raw * kAccScale;
    state_.az = (float)az_raw * kAccScale;

    // Orientação: padrão identidade (quaternion cru / não integrado no firmware)
    state_.qx = 0.0f;
    state_.qy = 0.0f;
    state_.qz = 0.0f;
    state_.qw = 1.0f;

    out_state = state_;
    return true;
}

} // namespace holodex
