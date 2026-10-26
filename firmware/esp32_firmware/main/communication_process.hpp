#ifndef COMMUNICATION_PROCESS_HPP
#define COMMUNICATION_PROCESS_HPP

#include <stdint.h>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "interfaces/i_ros_bridge.hpp"
#include "control_process.hpp"

namespace holodex {

class CommunicationProcess {
public:
    static constexpr gpio_num_t LED_PIN = GPIO_NUM_2;

    CommunicationProcess(IRosBridge& ros_bridge, ControlProcess& control_proc);
    ~CommunicationProcess();

    bool init();
    bool start(BaseType_t core_id = 0, UBaseType_t priority = 5, uint32_t stack_size = 8192);

    void setLed(bool on);

private:
    static void taskEntry(void* arg);
    void run();

    static void onTwistReceived(const TwistCommand& cmd);
    static void onLedReceived(bool state);

    static CommunicationProcess* s_instance_;

    IRosBridge& ros_bridge_;
    ControlProcess& control_proc_;
    TaskHandle_t task_handle_;

    std::atomic<bool> led_override_;
};

} // namespace holodex

#endif // COMMUNICATION_PROCESS_HPP
