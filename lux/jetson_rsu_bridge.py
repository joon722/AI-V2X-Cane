#!/usr/bin/env python3
"""
Jetson <-> RSU ESP32 bridge.

RSU ESP32 sends one JSON object per line over USB/UART. This bridge keeps the
latest cane state, evaluates vehicle messages with risk_engine, and sends one
JSON reply per vehicle back to the RSU ESP32.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from risk_engine import RiskInput, evaluate_risk, haversine_m, load_zones

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        "pyserial is required on Jetson. Install with: python3 -m pip install pyserial"
    ) from exc


@dataclass
class CaneState:
    node_id: int
    x: float
    y: float
    speed_mps: float
    updated_monotonic: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Jetson RSU risk bridge.")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Jetson serial port")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    parser.add_argument("--zone-file", default="zone_definition.csv", help="Zone CSV path")
    parser.add_argument("--cane-stale-sec", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--no-reply", action="store_true")
    return parser.parse_args()


def read_xy(msg: dict) -> tuple[float, float] | None:
    if "x" in msg and "y" in msg:
        return float(msg["x"]), float(msg["y"])
    if "lat" in msg and "lng" in msg:
        # Fallback for GPS-only live tests. A real deployment should use a
        # shared local coordinate transform before SUMO/AI integration.
        return float(msg["lat"]), float(msg["lng"])
    return None


def update_cane_state(msg: dict) -> CaneState | None:
    xy = read_xy(msg)
    if xy is None:
        return None
    return CaneState(
        node_id=int(msg.get("node_id", 0)),
        x=xy[0],
        y=xy[1],
        speed_mps=float(msg.get("speed_mps", 0.0)),
        updated_monotonic=time.monotonic(),
    )


def make_risk_input(cane: CaneState, vehicle_msg: dict) -> RiskInput | None:
    vehicle_xy = read_xy(vehicle_msg)
    if vehicle_xy is None:
        return None

    ped_x, ped_y = cane.x, cane.y
    veh_x, veh_y = vehicle_xy
    distance_m = vehicle_msg.get("distance_m")
    if distance_m is None and "lat" in vehicle_msg and "lng" in vehicle_msg:
        if abs(ped_x) <= 90.0 and abs(ped_y) <= 180.0:
            distance_m = haversine_m(ped_x, ped_y, float(vehicle_msg["lat"]), float(vehicle_msg["lng"]))

    return RiskInput(
        ts_ms=int(vehicle_msg.get("ts_ms", time.time() * 1000)),
        ped_x=ped_x,
        ped_y=ped_y,
        veh_x=veh_x,
        veh_y=veh_y,
        ped_speed_mps=cane.speed_mps,
        veh_speed_mps=float(vehicle_msg.get("speed_mps", 0.0)),
        distance_m=None if distance_m is None else float(distance_m),
        rel_speed_mps=(
            None
            if vehicle_msg.get("rel_speed_mps") is None
            else float(vehicle_msg["rel_speed_mps"])
        ),
    )


def main() -> int:
    args = parse_args()
    zone_path = Path(args.zone_file)
    zones = load_zones(zone_path)
    cane_state: CaneState | None = None

    print(f"[JETSON] opening {args.port} baud={args.baud} zones={len(zones)}")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout, write_timeout=args.timeout)
    except serial.SerialException as exc:
        print(f"[JETSON] failed to open serial port: {exc}", file=sys.stderr)
        return 2

    with ser:
        print("[JETSON] ready")
        while True:
            raw = ser.readline()
            if not raw:
                continue

            text = raw.decode("utf-8", errors="replace").strip()
            if not text:
                continue

            try:
                msg = json.loads(text)
            except json.JSONDecodeError:
                print(f"[BAD JSON] {text}")
                continue

            msg_type = msg.get("type")
            if msg_type == "cane":
                updated = update_cane_state(msg)
                if updated is None:
                    print(f"[CANE] ignored invalid position: {msg}")
                    continue
                cane_state = updated
                print(f"[CANE] id={updated.node_id} x={updated.x:.6f} y={updated.y:.6f}")
                continue

            if msg_type != "vehicle":
                print(f"[IGNORE] {msg}")
                continue

            if cane_state is None or time.monotonic() - cane_state.updated_monotonic > args.cane_stale_sec:
                print("[VEHICLE] skipped: cane state missing/stale")
                continue

            risk_input = make_risk_input(cane_state, msg)
            if risk_input is None:
                print(f"[VEHICLE] skipped: invalid vehicle position: {msg}")
                continue

            result = evaluate_risk(risk_input, zones)
            ttc_text = "inf" if result.ttc_s is None else f"{result.ttc_s:.2f}"
            vehicle_id = int(msg.get("node_id", 0))
            print(
                "[RISK] "
                f"veh={vehicle_id} cane={cane_state.node_id} "
                f"dist={result.distance_m:.2f}m rel={result.rel_speed_mps:.2f}mps "
                f"ttc={ttc_text}s final={result.final_risk} {result.reason}"
            )

            if args.no_reply:
                continue

            reply = {
                "type": "risk",
                "target_id": cane_state.node_id,
                "src_id": vehicle_id,
                "risk": result.final_risk,
                "distance_m": round(result.distance_m, 2),
                "ttc_s": None if result.ttc_s is None else round(result.ttc_s, 2),
                "reason": result.reason,
                "jetson_ms": int(time.time() * 1000),
            }
            ser.write((json.dumps(reply, separators=(",", ":")) + "\n").encode("utf-8"))
            ser.flush()


if __name__ == "__main__":
    raise SystemExit(main())

