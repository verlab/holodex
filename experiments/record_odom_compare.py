#!/usr/bin/env python3
"""
Record HoloDex dual odometry (stepper vs encoder/fused) over Zenoh.

Run inside the docker compose network (or host with router on :7447):

  cd docker && docker compose --profile experiment run --rm experiment

Or locally (router already up):

  pip install -r experiments/requirements.txt
  python experiments/record_odom_compare.py --endpoint tcp/127.0.0.1:7447

Publishes a yaw + translate trajectory on holodex/cmd_vel and records:
  holodex/odom/wheels       — stepper rates, yaw from wheels (no IMU)
  holodex/odom/wheels_enc   — encoder blend, yaw from wheels (no IMU)
  holodex/odom              — wheels+enc+IMU (primary / full)

Outputs under experiments/out/<timestamp>/ :
  odom_compare.npz, metrics.json, trajectory.png, theta.png, err.png, metrics.png
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

try:
    import zenoh
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "eclipse-zenoh is required. pip install -r experiments/requirements.txt"
    ) from exc


TOPICS = {
    "full": "holodex/odom",                 # wheels + enc + IMU
    "wheels": "holodex/odom/wheels",        # stepper / FAS only
    "wheels_enc": "holodex/odom/wheels_enc",  # FAS + AS5600 blend
}
CMD_TOPIC = "holodex/cmd_vel"
REF_STREAM = "full"


@dataclass
class StreamBuf:
    t: list[float] = field(default_factory=list)
    x: list[float] = field(default_factory=list)
    y: list[float] = field(default_factory=list)
    theta: list[float] = field(default_factory=list)
    vx: list[float] = field(default_factory=list)
    vy: list[float] = field(default_factory=list)
    omega: list[float] = field(default_factory=list)
    stamp_ms: list[int] = field(default_factory=list)

    def append(self, msg: dict, t0: float) -> None:
        self.t.append(time.time() - t0)
        self.x.append(float(msg.get("x", 0.0)))
        self.y.append(float(msg.get("y", 0.0)))
        self.theta.append(float(msg.get("theta", 0.0)))
        self.vx.append(float(msg.get("vx", 0.0)))
        self.vy.append(float(msg.get("vy", 0.0)))
        self.omega.append(float(msg.get("omega", 0.0)))
        self.stamp_ms.append(int(msg.get("stamp_ms", 0)))

    def as_arrays(self) -> dict[str, np.ndarray]:
        return {
            "t": np.asarray(self.t, dtype=np.float64),
            "x": np.asarray(self.x, dtype=np.float64),
            "y": np.asarray(self.y, dtype=np.float64),
            "theta": np.asarray(self.theta, dtype=np.float64),
            "vx": np.asarray(self.vx, dtype=np.float64),
            "vy": np.asarray(self.vy, dtype=np.float64),
            "omega": np.asarray(self.omega, dtype=np.float64),
            "stamp_ms": np.asarray(self.stamp_ms, dtype=np.int64),
        }


def wrap_pi(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


def interp_pose(src: dict[str, np.ndarray], t_query: np.ndarray) -> dict[str, np.ndarray]:
    if src["t"].size < 2:
        z = np.zeros_like(t_query)
        return {"x": z, "y": z, "theta": z, "vx": z, "vy": z, "omega": z}
    out = {}
    for k in ("x", "y", "theta", "vx", "vy", "omega"):
        out[k] = np.interp(t_query, src["t"], src[k])
    out["theta"] = wrap_pi(out["theta"])
    return out


def metrics_vs(ref: dict[str, np.ndarray], other: dict[str, np.ndarray], name: str) -> dict:
    dx = other["x"] - ref["x"]
    dy = other["y"] - ref["y"]
    dth = wrap_pi(other["theta"] - ref["theta"])
    dist = np.hypot(dx, dy)
    return {
        "vs": name,
        "n": int(ref["x"].size),
        "mse_xy": float(np.mean(dist**2)),
        "rmse_xy": float(np.sqrt(np.mean(dist**2))),
        "mae_xy": float(np.mean(np.abs(dist))),
        "max_xy": float(np.max(dist) if dist.size else 0.0),
        "mse_theta": float(np.mean(dth**2)),
        "rmse_theta": float(np.sqrt(np.mean(dth**2))),
        "mae_theta": float(np.mean(np.abs(dth))),
        "max_theta": float(np.max(np.abs(dth)) if dth.size else 0.0),
        "final_xy_err": float(dist[-1]) if dist.size else 0.0,
        "final_theta_err": float(dth[-1]) if dth.size else 0.0,
        "path_len_ref": float(np.sum(np.hypot(np.diff(ref["x"], prepend=ref["x"][:1]),
                                              np.diff(ref["y"], prepend=ref["y"][:1])))),
        "path_len_other": float(np.sum(np.hypot(np.diff(other["x"], prepend=other["x"][:1]),
                                                np.diff(other["y"], prepend=other["y"][:1])))),
    }


def open_session(endpoint: str):
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", json.dumps([endpoint]))
    return zenoh.open(conf)


def publish_cmd(pub, vx: float, vy: float, omega: float) -> None:
    payload = json.dumps({"vx": vx, "vy": vy, "omega": omega})
    pub.put(payload)


def run_trajectory(pub, args) -> list[dict]:
    """Open-loop body twist schedule for the paper experiment."""
    log = []
    phases = [
        ("yaw_ccw", args.yaw_duration, 0.0, 0.0, args.yaw_rate),
        ("pause", 0.5, 0.0, 0.0, 0.0),
        ("yaw_cw", args.yaw_duration, 0.0, 0.0, -args.yaw_rate),
        ("pause", 0.5, 0.0, 0.0, 0.0),
        ("forward", args.trans_duration, args.trans_speed, 0.0, 0.0),
        ("pause", 0.5, 0.0, 0.0, 0.0),
        ("strafe", args.trans_duration, 0.0, args.trans_speed, 0.0),
        ("stop", 1.0, 0.0, 0.0, 0.0),
    ]
    for name, dur, vx, vy, w in phases:
        t_end = time.time() + dur
        log.append({"phase": name, "t": time.time(), "vx": vx, "vy": vy, "omega": w, "dur": dur})
        while time.time() < t_end:
            publish_cmd(pub, vx, vy, w)
            time.sleep(args.cmd_period)
    publish_cmd(pub, 0.0, 0.0, 0.0)
    return log


def save_plots(out_dir: Path, arrays: dict[str, dict[str, np.ndarray]], metrics: list[dict]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 6))
    for name, a in arrays.items():
        if a["t"].size == 0:
            continue
        ax.plot(a["x"], a["y"], label=name, linewidth=1.5)
        ax.scatter(a["x"][:1], a["y"][:1], s=30)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("HoloDex odometry trajectories")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "trajectory.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    for name, a in arrays.items():
        if a["t"].size == 0:
            continue
        ax.plot(a["t"], np.degrees(a["theta"]), label=name)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("theta [deg]")
    ax.set_title("Heading vs time")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "theta.png", dpi=160)
    plt.close(fig)

    # Error of wheels / wheels_enc vs full (IMU) on common time base
    if arrays[REF_STREAM]["t"].size >= 2:
        tq = arrays[REF_STREAM]["t"]
        ref = interp_pose(arrays[REF_STREAM], tq)
        fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        for name in ("wheels", "wheels_enc"):
            if arrays[name]["t"].size < 2:
                continue
            o = interp_pose(arrays[name], tq)
            dist = np.hypot(o["x"] - ref["x"], o["y"] - ref["y"])
            dth = np.degrees(np.abs(wrap_pi(o["theta"] - ref["theta"])))
            axes[0].plot(tq, dist, label=name)
            axes[1].plot(tq, dth, label=name)
        axes[0].set_ylabel("|Δxy| [m]")
        axes[1].set_ylabel("|Δθ| [deg]")
        axes[1].set_xlabel("t [s]")
        axes[0].set_title("Error vs full (wheels+enc+IMU)")
        for ax in axes:
            ax.grid(True, alpha=0.3)
            ax.legend()
        fig.tight_layout()
        fig.savefig(out_dir / "err.png", dpi=160)
        plt.close(fig)

    # Metrics bar chart
    if metrics:
        labels = [m["vs"] for m in metrics]
        fig, axes = plt.subplots(1, 2, figsize=(8, 3.5))
        axes[0].bar(labels, [m["rmse_xy"] for m in metrics], color="#3dd6c6")
        axes[0].set_ylabel("RMSE xy [m]")
        axes[0].set_title("Position RMSE vs fused")
        axes[1].bar(labels, [math.degrees(m["rmse_theta"]) for m in metrics], color="#7ec8ff")
        axes[1].set_ylabel("RMSE θ [deg]")
        axes[1].set_title("Heading RMSE vs fused")
        fig.tight_layout()
        fig.savefig(out_dir / "metrics.png", dpi=160)
        plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--endpoint", default="tcp/zenoh:7447", help="Zenoh router endpoint")
    ap.add_argument("--out", type=Path, default=None, help="Output directory")
    ap.add_argument("--record-only", action="store_true", help="Do not publish cmd_vel")
    ap.add_argument("--warmup", type=float, default=2.0, help="Seconds to wait for first samples")
    ap.add_argument("--yaw-rate", type=float, default=0.6, help="Yaw rate rad/s")
    ap.add_argument("--yaw-duration", type=float, default=3.0)
    ap.add_argument("--trans-speed", type=float, default=0.12, help="m/s")
    ap.add_argument("--trans-duration", type=float, default=2.5)
    ap.add_argument("--cmd-period", type=float, default=0.05)
    ap.add_argument("--tail", type=float, default=1.0, help="Extra record time after trajectory")
    args = ap.parse_args()

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = args.out or (Path(__file__).resolve().parent / "out" / stamp)
    out_dir.mkdir(parents=True, exist_ok=True)

    bufs = {k: StreamBuf() for k in TOPICS}
    lock = threading.Lock()
    t0 = time.time()
    counts = defaultdict(int)

    print(f"[zenoh] connecting to {args.endpoint}")
    session = open_session(args.endpoint)

    def make_cb(name: str):
        def _cb(sample):
            try:
                payload = sample.payload.to_string()
                msg = json.loads(payload)
            except Exception:
                return
            with lock:
                bufs[name].append(msg, t0)
                counts[name] += 1

        return _cb

    subs = []
    for name, ke in TOPICS.items():
        subs.append(session.declare_subscriber(ke, make_cb(name)))
        print(f"[zenoh] subscribed {ke}")

    pub = session.declare_publisher(CMD_TOPIC)

    print(f"[rec] warmup {args.warmup:.1f}s …")
    time.sleep(args.warmup)
    with lock:
        print(f"[rec] samples so far: {dict(counts)}")

    traj_log = []
    if not args.record_only:
        print("[cmd] running trajectory (yaw ±, forward, strafe)")
        traj_log = run_trajectory(pub, args)
        time.sleep(args.tail)
    else:
        print(f"[rec] record-only for {args.yaw_duration + args.trans_duration + 5:.1f}s — drive the robot manually")
        time.sleep(args.yaw_duration + args.trans_duration + 5.0)

    publish_cmd(pub, 0.0, 0.0, 0.0)
    time.sleep(0.2)

    with lock:
        arrays = {k: v.as_arrays() for k, v in bufs.items()}
        n_counts = dict(counts)

    # Align on full (IMU) time base; also compare wheels vs wheels_enc
    metrics = []
    if arrays[REF_STREAM]["t"].size >= 2:
        tq = arrays[REF_STREAM]["t"]
        ref = interp_pose(arrays[REF_STREAM], tq)
        for name in ("wheels", "wheels_enc"):
            if arrays[name]["t"].size < 2:
                continue
            other = interp_pose(arrays[name], tq)
            metrics.append(metrics_vs(ref, other, f"{name}_vs_full"))
        if arrays["wheels"]["t"].size >= 2 and arrays["wheels_enc"]["t"].size >= 2:
            w = interp_pose(arrays["wheels"], tq)
            we = interp_pose(arrays["wheels_enc"], tq)
            metrics.append(metrics_vs(w, we, "wheels_enc_vs_wheels"))

    meta = {
        "endpoint": args.endpoint,
        "topics": TOPICS,
        "counts": n_counts,
        "trajectory": traj_log,
        "yaw_rate": args.yaw_rate,
        "yaw_duration": args.yaw_duration,
        "trans_speed": args.trans_speed,
        "trans_duration": args.trans_duration,
        "record_only": args.record_only,
        "out": str(out_dir),
    }
    npz_path = out_dir / "odom_compare.npz"
    np.savez_compressed(
        npz_path,
        meta_json=np.array(json.dumps(meta)),
        **{f"{name}_{k}": arr for name, data in arrays.items() for k, arr in data.items()},
    )

    metrics_path = out_dir / "metrics.json"
    metrics_path.write_text(json.dumps({"counts": n_counts, "metrics": metrics}, indent=2), encoding="utf-8")

    save_plots(out_dir, arrays, metrics)

    print(f"[done] samples={n_counts}")
    for m in metrics:
        print(
            f"  {m['vs']}: RMSE_xy={m['rmse_xy']:.4f} m  "
            f"RMSE_θ={math.degrees(m['rmse_theta']):.2f} deg  "
            f"final_xy={m['final_xy_err']:.4f} m"
        )
    print(f"[done] wrote {npz_path}")
    print(f"[done] wrote {metrics_path}")
    print(f"[done] plots in {out_dir}")

    session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
