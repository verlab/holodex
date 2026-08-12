#pragma once

#include <cstdint>

/** Body-frame velocity command (SI units). */
struct Twist2D {
    float vx = 0.0f;     // m/s  (+X forward)
    float vy = 0.0f;     // m/s  (+Y left)
    float omega = 0.0f;  // rad/s (+CCW)
};

/**
 * One IMU reading with the interval it actually covers.
 *
 * dt_s is measured between consecutive successful reads, not between control
 * ticks: the loop jitters (SoftAP / HTTP work) and a read can fail on a mux
 * NACK, so integrating with the tick period would scale every sample by the
 * wrong time and bleed straight into heading drift.
 */
struct ImuSample {
    float ax = 0.0f;      // m/s², body +X forward
    float ay = 0.0f;      // m/s², body +Y left
    float az = 0.0f;      // m/s², body +Z up
    float gyro_z = 0.0f;  // rad/s, +CCW
    float dt_s = 0.0f;    // interval since the previous valid sample
    uint32_t stamp_us = 0;
    bool valid = false;
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
