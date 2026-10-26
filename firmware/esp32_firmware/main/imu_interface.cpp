#include "imu_interface.hpp"
#include <cmath>
#include "esp_log.h"

static const char* TAG = "ImuInterface";

namespace holodex {

ImuInterface::ImuInterface()
    : yaw_accumulated_(0.0f)
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

bool ImuInterface::init()
{
    ESP_LOGI(TAG, "Inicializando interface IMU...");
    yaw_accumulated_ = 0.0f;
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
    ESP_LOGI(TAG, "Interface IMU inicializada com sucesso.");
    return true;
}

bool ImuInterface::update(float dt, ImuState& out_state)
{
    if (dt <= 0.0f) {
        dt = 0.02f; // Fallback 50 Hz
    }

    // Integração simples de orientação Yaw a partir da velocidade angular Z
    yaw_accumulated_ += state_.gz * dt;

    // Normalização no intervalo [-pi, pi]
    while (yaw_accumulated_ > M_PI)  yaw_accumulated_ -= 2.0f * M_PI;
    while (yaw_accumulated_ < -M_PI) yaw_accumulated_ += 2.0f * M_PI;

    // Conversão de Yaw para Quatérnion (Roll=0, Pitch=0)
    float half_yaw = yaw_accumulated_ * 0.5f;
    state_.qx = 0.0f;
    state_.qy = 0.0f;
    state_.qz = sinf(half_yaw);
    state_.qw = cosf(half_yaw);

    out_state = state_;
    return true;
}

} // namespace holodex
