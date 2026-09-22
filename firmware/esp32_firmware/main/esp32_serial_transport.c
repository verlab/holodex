#include "esp32_serial_transport.h"

#include <driver/uart.h>
#include <driver/gpio.h>
#include "esp_err.h"

#ifndef CONFIG_MICROROS_UART_TXD
#define CONFIG_MICROROS_UART_TXD UART_PIN_NO_CHANGE
#endif

#ifndef CONFIG_MICROROS_UART_RXD
#define CONFIG_MICROROS_UART_RXD UART_PIN_NO_CHANGE
#endif

#ifndef CONFIG_MICROROS_UART_RTS
#define CONFIG_MICROROS_UART_RTS UART_PIN_NO_CHANGE
#endif

#ifndef CONFIG_MICROROS_UART_CTS
#define CONFIG_MICROROS_UART_CTS UART_PIN_NO_CHANGE
#endif

#ifndef CONFIG_MICROROS_UART_BAUDRATE
#define CONFIG_MICROROS_UART_BAUDRATE 921600
#endif

#define UART_BUFFER_SIZE 2048

bool esp32_serial_open(struct uxrCustomTransport * transport) {
    size_t * uart_port = (size_t*) transport->args;

    uart_config_t uart_config = {
        .baud_rate = CONFIG_MICROROS_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_param_config(*uart_port, &uart_config) != ESP_OK) {
        return false;
    }

    int tx_pin = (CONFIG_MICROROS_UART_TXD < 0) ? UART_PIN_NO_CHANGE : CONFIG_MICROROS_UART_TXD;
    int rx_pin = (CONFIG_MICROROS_UART_RXD < 0) ? UART_PIN_NO_CHANGE : CONFIG_MICROROS_UART_RXD;
    int rts_pin = (CONFIG_MICROROS_UART_RTS < 0) ? UART_PIN_NO_CHANGE : CONFIG_MICROROS_UART_RTS;
    int cts_pin = (CONFIG_MICROROS_UART_CTS < 0) ? UART_PIN_NO_CHANGE : CONFIG_MICROROS_UART_CTS;

    if (uart_set_pin(*uart_port, tx_pin, rx_pin, rts_pin, cts_pin) != ESP_OK) {
        return false;
    }

    if (uart_driver_install(*uart_port, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 0, NULL, 0) != ESP_OK) {
        return false;
    }

    return true;
}

bool esp32_serial_close(struct uxrCustomTransport * transport) {
    size_t * uart_port = (size_t*) transport->args;
    return (uart_driver_delete(*uart_port) == ESP_OK);
}

size_t esp32_serial_write(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err) {
    (void) err;
    size_t * uart_port = (size_t*) transport->args;
    const int txBytes = uart_write_bytes(*uart_port, (const char*) buf, len);
    return (txBytes < 0) ? 0 : (size_t) txBytes;
}

size_t esp32_serial_read(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err) {
    (void) err;
    size_t * uart_port = (size_t*) transport->args;
    TickType_t ticks = (timeout <= 0) ? 0 : pdMS_TO_TICKS(timeout);
    const int rxBytes = uart_read_bytes(*uart_port, buf, len, ticks);
    return (rxBytes < 0) ? 0 : (size_t) rxBytes;
}
