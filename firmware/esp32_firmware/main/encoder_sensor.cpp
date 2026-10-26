#include "encoder_sensor.hpp"
#include "encoder_math.hpp"
#include <cmath>
#include <string.h>
#include "esp_log.h"

static const char* TAG = "As5600MuxEncoder";

namespace holodex {

As5600MuxEncoder::As5600MuxEncoder(I2cBus& i2c)
    : i2c_(i2c),
      mux_ready_(false),
      active_sensors_count_(0)
{
    memset(channel_active_, 0, sizeof(channel_active_));
    memset(consecutive_errors_, 0, sizeof(consecutive_errors_));
    memset(last_raw_, 0, sizeof(last_raw_));
    memset(prev_raw_, 0, sizeof(prev_raw_));
    memset(accumulated_ticks_, 0, sizeof(accumulated_ticks_));
}

bool As5600MuxEncoder::init()
{
    ESP_LOGI(TAG, "Inicializando Hub de Encoders (PCA9548A + 4x AS5600)...");
    active_sensors_count_ = 0;

    if (!i2c_.isInitialized()) {
        i2c_.init();
    }

    // Testa comunicação com o multiplexador PCA9548A em 0x70
    uint8_t disable_cmd = 0;
    esp_err_t err = i2c_.write(MUX_ADDR, &disable_cmd, 1, 10);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "PCA9548A nao encontrado em 0x%02X (%s). Encoders fisicos inativos.",
                 MUX_ADDR, esp_err_to_name(err));
        mux_ready_ = false;
        return false;
    }

    mux_ready_ = true;
    ESP_LOGI(TAG, "PCA9548A multiplexer detectado em 0x%02X.", MUX_ADDR);

    const uint8_t channels[4] = {CH_FL, CH_FR, CH_RL, CH_RR};
    const char* names[4] = {"FL (Ch 0)", "FR (Ch 1)", "RL (Ch 4)", "RR (Ch 5)"};

    for (int i = 0; i < 4; ++i) {
        channel_active_[i] = false;
        consecutive_errors_[i] = 0;
        if (!selectMuxChannel(channels[i])) {
            continue;
        }

        uint16_t raw = 0;
        if (readRawAngle(channels[i], raw)) {
            channel_active_[i] = true;
            last_raw_[i] = raw;
            prev_raw_[i] = raw;
            active_sensors_count_++;
            ESP_LOGI(TAG, "AS5600 OK na roda %s | Angulo inicial: %u", names[i], raw);
        } else {
            ESP_LOGW(TAG, "AS5600 ausente na roda %s.", names[i]);
        }
    }

    selectMuxChannel(0xFF);
    ESP_LOGI(TAG, "Total de encoders AS5600 ativos: %d/4", active_sensors_count_);
    return (active_sensors_count_ > 0);
}

bool As5600MuxEncoder::selectMuxChannel(uint8_t channel)
{
    if (!mux_ready_) return false;
    uint8_t mask = (channel <= 7) ? (1 << channel) : 0;
    esp_err_t err = i2c_.write(MUX_ADDR, &mask, 1, 10);
    return (err == ESP_OK);
}

bool As5600MuxEncoder::readRawAngle(uint8_t channel, uint16_t& out_raw)
{
    uint8_t data[2] = {0};
    esp_err_t err = i2c_.writeRead(AS5600_ADDR, AS5600_REG_ANGLE, data, 2, 10);
    if (err == ESP_OK) {
        out_raw = (uint16_t)(((data[0] & 0x0F) << 8) | data[1]);
        return true;
    }
    return false;
}

bool As5600MuxEncoder::update(float dt_s, WheelPositions& out_positions, WheelVelocities& out_velocities)
{
    if (!mux_ready_ || active_sensors_count_ == 0 || dt_s <= 0.0f) {
        out_positions = current_positions_;
        out_velocities = current_velocities_;
        return false;
    }

    const uint8_t channels[4] = {CH_FL, CH_FR, CH_RL, CH_RR};
    constexpr float kTicksToRad = (2.0f * (float)M_PI) / 4096.0f;

    float delta_rad[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int read_success_count = 0;

    for (int i = 0; i < 4; ++i) {
        if (!channel_active_[i] && consecutive_errors_[i] >= 5) {
            continue;
        }

        if (selectMuxChannel(channels[i])) {
            uint16_t raw = 0;
            if (readRawAngle(channels[i], raw)) {
                last_raw_[i] = raw;

                int32_t delta = unwrapEncoderDelta12(last_raw_[i], prev_raw_[i]);

                prev_raw_[i] = last_raw_[i];
                accumulated_ticks_[i] += delta;
                delta_rad[i] = (float)delta * kTicksToRad;

                consecutive_errors_[i] = 0;
                channel_active_[i] = true;
                read_success_count++;
            } else {
                consecutive_errors_[i]++;
                if (consecutive_errors_[i] >= 5) {
                    channel_active_[i] = false;
                    ESP_LOGW(TAG, "Encoder canal %d falhou 5 vezes consecutivas. Desativando.", i);
                }
            }
        }
    }

    selectMuxChannel(0xFF);

    current_positions_.fl += delta_rad[0];
    current_positions_.fr += delta_rad[1];
    current_positions_.rl += delta_rad[2];
    current_positions_.rr += delta_rad[3];

    current_velocities_.fl = delta_rad[0] / dt_s;
    current_velocities_.fr = delta_rad[1] / dt_s;
    current_velocities_.rl = delta_rad[2] / dt_s;
    current_velocities_.rr = delta_rad[3] / dt_s;

    out_positions = current_positions_;
    out_velocities = current_velocities_;
    return (read_success_count > 0);
}

EncoderTicks As5600MuxEncoder::readTicks()
{
    EncoderTicks ticks;
    ticks.fl = accumulated_ticks_[0];
    ticks.fr = accumulated_ticks_[1];
    ticks.rl = accumulated_ticks_[2];
    ticks.rr = accumulated_ticks_[3];
    return ticks;
}

void As5600MuxEncoder::reset()
{
    memset(accumulated_ticks_, 0, sizeof(accumulated_ticks_));
    current_positions_ = WheelPositions();
    current_velocities_ = WheelVelocities();
}

} // namespace holodex
