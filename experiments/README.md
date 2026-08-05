# HoloDex odometry experiments

Capture **stepper-only**, **encoder-only**, and **fused** odometry over Zenoh for article plots.

## Prerequisites

1. Flash firmware ≥ **0.4.0** (mecanum IK + dual odom streams).
2. Robot SoftAP config: STA joined to lab Wi‑Fi, Zenoh endpoint `tcp/<PC-LAN-IP>:7447`, Zenoh enabled.
3. Start the router:

```bash
cd docker && docker compose up -d zenoh
```

## Run (Docker)

```bash
cd docker
docker compose --profile experiment run --rm experiment
```

Outputs land in `experiments/out/<UTC-timestamp>/`:

| File | Content |
|------|---------|
| `odom_compare.npz` | Time series for fused / stepper / encoder |
| `metrics.json` | MSE / RMSE / MAE (xy + θ) vs fused |
| `trajectory.png` | XY paths |
| `theta.png` | Heading vs time |
| `err.png` | Error vs fused |
| `metrics.png` | RMSE bar chart |

## Run (host)

```bash
pip install -r experiments/requirements.txt
python experiments/record_odom_compare.py --endpoint tcp/127.0.0.1:7447
```

Manual drive (no auto trajectory):

```bash
python experiments/record_odom_compare.py --record-only --endpoint tcp/127.0.0.1:7447
```

## Zenoh topics

| Topic | Source |
|-------|--------|
| `holodex/cmd_vel` | Script → robot (JSON `{vx,vy,omega}`) |
| `holodex/odom` | Fused (encoder blend + yaw Kalman) |
| `holodex/odom/stepper` | FastAccelStepper rates only |
| `holodex/odom/encoder` | AS5600 rates only |
