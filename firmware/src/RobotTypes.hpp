#pragma once

#include <cstdint>

/** Body-frame velocity command (SI units). */
struct Twist2D {
    float vx = 0.0f;     // m/s  (+X forward)
    float vy = 0.0f;     // m/s  (+Y left)
    float omega = 0.0f;  // rad/s (+CCW)
};

/** Planar pose in the odometry frame. */
struct Pose2D {
    float x = 0.0f;      // m
    float y = 0.0f;      // m
    float theta = 0.0f;  // rad
};

/** Full odometry snapshot for UI / ROS bridge. */
struct OdometryState {
    Pose2D pose;
    Twist2D twist;
    float wheel_hz[4] = {0, 0, 0, 0};
    uint32_t stamp_ms = 0;
};

enum class CmdSource : uint8_t {
    None = 0,
    Web,
    Zenoh
};

struct VelocityCommand {
    Twist2D twist;
    CmdSource source = CmdSource::None;
    uint32_t stamp_ms = 0;
};
