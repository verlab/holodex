# HoloDex

VerLab **mecanum** robot: 4× NEMA23 steppers, AS5600 encoders (PCA9548A), BMI160 IMU, ESP32 control.

## Overview

| Area | Description |
|------|-------------|
| **Actuation** | NEMA23 steppers + drivers (2 A via Vref) |
| **Mechanics** | 4WD mecanum wheels — spin about chassis center |
| **Onboard control** | Mecanum IK/FK, SoftAP web teleop, optional enc speed PI |
| **Sensing** | 4× AS5600 + BMI160 on PCA9548A |
| **Odometry** | Three parallel streams for comparison (wheels / wheels+enc / full+IMU) |
| **ROS path** | Zenoh `cmd_vel` + three `odom*` topics |

## Architecture (high level)

```mermaid
flowchart LR
  subgraph Host["Host / lab PC"]
    WEB["SoftAP web UI"]
    EXP["Experiment script"]
    RTR["Zenoh router\n:7447"]
  end

  subgraph ESP["ESP32"]
    MC["MotionController"]
    KIN["Mecanum IK"]
    FAS["Steppers ×4"]
    ENC["AS5600 ×4"]
    IMU["BMI160"]
    OW["odom wheels"]
    OWE["odom wheels+enc"]
    OF["odom full"]
  end

  WEB -->|teleop| MC
  EXP --> RTR -->|cmd_vel| MC
  MC --> KIN --> FAS
  FAS -->|Hz| OW
  FAS -->|Hz| OWE
  ENC -->|Hz| OWE
  FAS -->|Hz| OF
  ENC -->|Hz| OF
  IMU -->|gyro Z| OF
  OW --> RTR
  OWE --> RTR
  OF --> RTR
  OF --> WEB
```

## Architecture (detailed)

```mermaid
flowchart TB
  subgraph Inputs
    SoftAP["SoftAP\n192.168.4.1 :80 / :81"]
    STA["STA lab Wi‑Fi"]
    Zcmd["Zenoh sub\nholodex/cmd_vel"]
  end

  subgraph Tick["Control @ 50 Hz"]
    MUX["PCA9548A\nch0,1,4,5 AS5600 · ch6 IMU"]
    SAMP["sample encoders + IMU"]
    MODE{"command mode"}
    IK["IK: v = vx ± vy ± L·ω\nL = lx + ly"]
    PI["enc speed PI optional"]
    OUT["applyWheelHz → FAS"]
  end

  subgraph Rates["Wheel rate sources"]
    FASHz["FAS measured Hz"]
    ENCHz["AS5600 → Hz"]
    BLEND["blend α·enc + 1−α·fas"]
  end

  subgraph Odom["Three odometry streams"]
    OW["holodex/odom/wheels\nrates: FAS\nyaw: wheel FK · no IMU"]
    OWE["holodex/odom/wheels_enc\nrates: blend\nyaw: wheel FK · no IMU"]
    OF["holodex/odom\nrates: blend\nyaw: gyro + Kalman"]
  end

  subgraph Pub["Outputs"]
    WS["WebSocket /api/odom"]
    ZP["Zenoh publishers"]
    MOT["NEMA23 ×4"]
  end

  SoftAP --> MODE
  STA --> Zcmd --> MODE
  MUX --> SAMP
  SAMP --> MODE
  MODE -->|twist / cinematic| IK --> PI --> OUT --> MOT
  MODE -->|raw m0..m3| OUT
  OUT --> FASHz
  SAMP --> ENCHz
  FASHz --> OW
  FASHz --> BLEND
  ENCHz --> BLEND
  BLEND --> OWE
  BLEND --> OF
  SAMP -->|gyro Z only here| OF
  OW --> ZP
  OWE --> ZP
  OF --> ZP
  OF --> WS
```

## Odometry comparison (article)

```mermaid
flowchart LR
  subgraph Sources
    FAS["Stepper ramp Hz"]
    ENC["AS5600 Hz"]
    GYRO["BMI160 ωz"]
  end

  subgraph Streams
    W["wheels\nFAS + yaw wheels"]
    WE["wheels_enc\nblend + yaw wheels"]
    F["full\nblend + yaw IMU"]
  end

  FAS --> W
  FAS --> WE
  ENC --> WE
  FAS --> F
  ENC --> F
  GYRO --> F

  W --> CMP["experiment script\nnpz · plots · RMSE"]
  WE --> CMP
  F --> CMP
```

| Stream | Topic | Wheel rates | Yaw |
|--------|-------|-------------|-----|
| **wheels** | `holodex/odom/wheels` | FastAccelStepper only | wheel FK (no IMU) |
| **wheels_enc** | `holodex/odom/wheels_enc` | FAS + AS5600 blend | wheel FK (no IMU) |
| **full** | `holodex/odom` | FAS + AS5600 blend | gyro + Kalman |

## Mecanum kinematics

Early firmware (`5ed7a02`) only accepted **raw motor Hz** over Zenoh (no onboard IK). A later 45° omni û-model with swapped yaw/strafe columns did **not** spin about the chassis center.

Firmware **0.4.x** uses standard mecanum ([reference](https://github.com/Shimraz/Kinematics-of-Mobile-Robot)):

\[
\begin{aligned}
v_{FL} &= v_x - v_y - L\omega \\
v_{RL} &= v_x + v_y - L\omega \\
v_{FR} &= v_x + v_y + L\omega \\
v_{RR} &= v_x - v_y + L\omega
\end{aligned}
\quad L = l_x + l_y
\]

Motor order: **M0=FL, M1=RL, M2=FR, M3=RR**. Left motors use `motor_sign = −1` (DIR mirrored). Encoders: mux **ch0,1,4,5**; IMU: **ch6**.

## Repository layout

| Path | Role |
|------|------|
| `firmware/` | ESP32 PlatformIO stack (primary) |
| `docker/` | Zenoh router (+ experiment profile) |
| `experiments/` | Triple-odom capture (npz / plots / MSE) |
| `src/` | Legacy desktop UI (optional) |

## Quick start (web teleop)

1. Build & flash:
   ```bash
   cd firmware && pio run -t upload
   ```
2. Join SoftAP **`HoloDex`** / **`holodex123`**
3. Open **http://192.168.4.1/**
4. Drive tab: joysticks; Config: encoder mux map, STA, Zenoh endpoint

### Hardware defaults

| Parameter | Value |
|-----------|--------|
| Wheel radius | 50 mm |
| Microstepping | 32 → **6400** steps/rev |
| Wheelbase / 2 (`lx`) | 0.19 m |
| Track / 2 (`ly`) | 0.195 m |
| I2C | SDA 21, SCL 5 · mux `0x70` |

## Zenoh + odometry experiment

```bash
cd docker && docker compose up -d zenoh
# On robot web Config: STA + tcp/<your-PC-LAN-IP>:7447
docker compose --profile experiment run --rm experiment
```

Details: [experiments/README.md](experiments/README.md).

| Topic | Direction | Content |
|-------|-----------|---------|
| `holodex/cmd_vel` | → robot | `{"vx","vy","omega"}` SI |
| `holodex/odom` | ← robot | wheels+enc+IMU (full) |
| `holodex/odom/wheels` | ← robot | stepper rates, yaw from wheels |
| `holodex/odom/wheels_enc` | ← robot | encoder blend, yaw from wheels |

## Contribution

Branch from `dev`; PRs → `dev` → `main`. See [CONTRIBUTING.md](CONTRIBUTING.md).

## VerLab

Internal [VerLab](https://github.com/verlab) project — private repository.
