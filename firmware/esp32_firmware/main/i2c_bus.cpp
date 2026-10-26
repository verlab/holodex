#include "i2c_bus.hpp"
#include "esp_log.h"

static const char* TAG = "I2cBus";

namespace holodex {

I2cBus& I2cBus::getInstance()
{
    static I2cBus instance;
    return instance;
}

I2cBus::I2cBus()
    : port_(I2C_NUM_0),
      initialized_(false),
      mutex_(nullptr)
{
    mutex_ = xSemaphoreCreateMutex();
}

I2cBus::~I2cBus()
{
    if (initialized_) {
        i2c_driver_delete(port_);
        initialized_ = false;
    }
    if (mutex_) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
}

bool I2cBus::init(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed_hz, i2c_port_t port)
{
    if (initialized_) {
        return true;
    }

    port_ = port;
    ESP_LOGI(TAG, "Configurando barramento I2C (SDA=%d, SCL=%d, %lu Hz)...",
             (int)sda_pin, (int)scl_pin, (unsigned long)clk_speed_hz);

    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.scl_io_num = scl_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = clk_speed_hz;
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config(port_, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar parametros do I2C: %s", esp_err_to_name(err));
        return false;
    }

    err = i2c_driver_install(port_, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao instalar driver I2C: %s", esp_err_to_name(err));
        return false;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "Barramento I2C inicializado com sucesso.");
    return true;
}

esp_err_t I2cBus::write(uint8_t dev_addr, const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = i2c_master_write_to_device(port_, dev_addr, data, len, pdMS_TO_TICKS(timeout_ms));
    xSemaphoreGive(mutex_);
    return ret;
}

esp_err_t I2cBus::read(uint8_t dev_addr, uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = i2c_master_read_from_device(port_, dev_addr, data, len, pdMS_TO_TICKS(timeout_ms));
    xSemaphoreGive(mutex_);
    return ret;
}

esp_err_t I2cBus::writeRead(uint8_t dev_addr, uint8_t reg, uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = i2c_master_write_read_device(port_, dev_addr, &reg, 1, data, len, pdMS_TO_TICKS(timeout_ms));
    xSemaphoreGive(mutex_);
    return ret;
}

} // namespace holodex
