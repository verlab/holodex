#ifndef I2C_BUS_HPP
#define I2C_BUS_HPP

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace holodex {

class I2cBus {
public:
    static I2cBus& getInstance();

    bool init(gpio_num_t sda_pin = GPIO_NUM_21,
              gpio_num_t scl_pin = GPIO_NUM_5,
              uint32_t clk_speed_hz = 400000,
              i2c_port_t port = I2C_NUM_0);

    esp_err_t write(uint8_t dev_addr, const uint8_t* data, size_t len, uint32_t timeout_ms = 10);
    esp_err_t read(uint8_t dev_addr, uint8_t* data, size_t len, uint32_t timeout_ms = 10);
    esp_err_t writeRead(uint8_t dev_addr, uint8_t reg, uint8_t* data, size_t len, uint32_t timeout_ms = 10);

    bool isInitialized() const { return initialized_; }

private:
    I2cBus();
    ~I2cBus();

    i2c_port_t port_;
    bool initialized_;
    SemaphoreHandle_t mutex_;
};

} // namespace holodex

#endif // I2C_BUS_HPP
