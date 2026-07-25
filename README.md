# HoloDex

VerLab **omnidirectional** robot: NEMA23 steppers and 3D-printed omni wheels.

## Overview

| Area | Description |
|------|-------------|
| **Actuation** | NEMA23 steppers + drivers |
| **Mechanics** | Omnidirectional wheels (3D-printable) |
| **Onboard control** | Holonomic IK, odometry, SoftAP web teleop (ESP32) |
| **ROS path** | Zenoh bridge (`holodex/cmd_vel`, `holodex/odom`) |

## Repository layout

| Path | Role |
|------|------|
| `firmware/` | ESP32 firmware (PlatformIO) — primary control stack |
| `docker/` | Zenoh router for lab / ROS integration |
| `src/` | Legacy desktop UI (optional) |

## Quick start (web teleop)

1. Build & flash firmware:
   ```bash
   cd firmware
   pio run -t upload
   ```
2. Join SoftAP **`HoloDex`** / password **`holodex123`**
3. Open **http://192.168.4.1/**
4. Use the on-screen joystick; odometry streams live over WebSocket

Optional: in the web **Configuration** panel, set lab Wi‑Fi (STA) and Zenoh endpoint (`tcp/<router-ip>:7447`), then start the router:

```bash
cd docker && docker compose up -d
```

Publish body velocity (m/s, rad/s):

```json
{"vx": 0.1, "vy": 0.0, "omega": 0.0}
```

Topic: `holodex/cmd_vel` — odometry is published on `holodex/odom`.

### Hardware parameters (defaults in firmware)

| Parameter | Value |
|-----------|--------|
| Wheel radius | 50 mm |
| Microstepping | 32 → **6400** steps/rev |
| Driver current | 2 A (set on driver Vref, not in firmware) |
| Axle spacing (wheelbase) | 38 cm → `lx = 0.19 m` |
| Wheel spacing (track) | 39 cm → `ly = 0.195 m` |

## Contribution

Branching and commits: [CONTRIBUTING.md](CONTRIBUTING.md).

## VerLab

Internal [VerLab](https://github.com/verlab) project — private repository.
