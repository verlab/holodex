# HoloDex odometry experiments

Capture three **independent** odometry streams over Zenoh for article plots:

| Stream | Topic | Wheel rates | Yaw |
|--------|-------|-------------|-----|
| **wheels** | `holodex/odom/wheels` | FastAccelStepper only | wheel FK (no IMU) |
| **wheels_enc** | `holodex/odom/wheels_enc` | FAS + AS5600 blend | wheel FK (no IMU) |
| **full** | `holodex/odom` | FAS + AS5600 blend | gyro + Kalman |

## Prerequisites

1. Flash firmware ≥ **0.4.1**.
2. Robot SoftAP config: STA + Zenoh endpoint `tcp/<PC-LAN-IP>:7447`.
3. Start the router:

```bash
cd docker && docker compose up -d zenoh
```

## Run (Docker)

```bash
cd docker
docker compose --profile experiment run --rm experiment
```

Outputs in `experiments/out/<UTC-timestamp>/`:

| File | Content |
|------|---------|
| `odom_compare.npz` | Time series for all three streams |
| `metrics.json` | MSE / RMSE / MAE (xy + θ) |
| `trajectory.png` | XY paths |
| `theta.png` | Heading vs time |
| `err.png` | Error vs full |
| `metrics.png` | RMSE bars |

## Run (host)

```bash
pip install -r experiments/requirements.txt
python experiments/record_odom_compare.py --endpoint tcp/127.0.0.1:7447
```

Manual drive:

```bash
python experiments/record_odom_compare.py --record-only --endpoint tcp/127.0.0.1:7447
```
