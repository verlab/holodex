#ifndef I_ROS_BRIDGE_HPP
#define I_ROS_BRIDGE_HPP

#include <stdint.h>
#include <stddef.h>
#include "robot_types.hpp"

namespace holodex {

typedef void (*TwistCommandCallback)(const TwistCommand& cmd);
typedef void (*LedCommandCallback)(bool state);

class IRosBridge {
public:
    virtual ~IRosBridge() = default;

    virtual bool initTransport(size_t uart_port = 0) = 0;
    virtual bool createEntities() = 0;
    virtual void destroyEntities() = 0;

    virtual void spinSome(uint32_t timeout_ms = 10) = 0;
    virtual bool syncTime(uint32_t timeout_ms = 100) = 0;
    virtual bool pingAgent(uint32_t timeout_ms = 100, uint8_t attempts = 1) = 0;

    virtual bool publishOdometry(const OdomState& state, int64_t sample_timestamp_us = 0) = 0;
    virtual bool publishImu(const ImuState& state, int64_t sample_timestamp_us = 0) = 0;
    virtual bool publishJointStates(const WheelPositions& positions, const WheelVelocities& velocities, int64_t sample_timestamp_us = 0) = 0;
    // data = {fl, fr, rl, rr, sample_us_low32}: o último campo é esp_timer (us) da amostragem,
    // truncado em 32 bits; o receptor deve calcular dt com aritmética uint32 (wrap-safe).
    virtual bool publishWheelTicks(const EncoderTicks& ticks, int64_t sample_timestamp_us) = 0;
    virtual bool publishStepperStatus(const StepperGroupStatus& status) = 0;
    virtual bool publishHeartbeat(int32_t counter) = 0;

    virtual void setTwistCallback(TwistCommandCallback cb) = 0;
    virtual void setLedCallback(LedCommandCallback cb) = 0;

    virtual bool isEntitiesCreated() const = 0;
    virtual bool isTimeSynchronized() const = 0;
};

} // namespace holodex

#endif // I_ROS_BRIDGE_HPP
