#!/usr/bin/env python3
"""
Jetson UART probe for the V2X RSU bridge.

Reads JSON lines from the RSU bridge, keeps the latest cane position, computes
vehicle distance/TTC against that position, and sends a risk reply back:

    {"target_id":456,"src_id":123,"risk":2}

Default Jetson Nano UART candidates:
    /dev/ttyTHS1  (GPIO UART on many Jetson Nano setups)
    /dev/ttyUSB0  (USB-UART adapter)
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        "pyserial is required. Install on Jetson with: python3 -m pip install pyserial"
    ) from exc


EARTH_RADIUS_M = 6_371_000.0


@dataclass
class RiskResult:
    distance_m: float
    ttc_s: float | None
    risk: int
    reason: str


@dataclass
class CaneState:
    node_id: int
    lat: float
    lng: float
    seq: int | None
    updated_monotonic: float


def haversine_m(lat1: float, lng1: float, lat2: float, lng2: float) -> float:
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lam = math.radians(lng2 - lng1)

    a = (
        math.sin(d_phi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(d_lam / 2.0) ** 2
    )
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return EARTH_RADIUS_M * c


def estimate_ttc(distance_m: float, speed_mps: float) -> float | None:
    if speed_mps <= 0.2:
        return None
    return distance_m / speed_mps


def classify_risk(distance_m: float, ttc_s: float | None) -> tuple[int, str]:
    # Match the cane-side local risk thresholds.
    if distance_m < 3.0:
        return 3, "distance<3m"
    if ttc_s is not None and ttc_s < 1.5:
        return 3, "ttc<1.5s"
    if distance_m < 6.0:
        return 2, "distance<6m"
    if ttc_s is not None and ttc_s < 3.0:
        return 2, "ttc<3s"
    if distance_m < 10.0:
        return 1, "distance<10m"
    if ttc_s is not None and ttc_s < 5.0:
        return 1, "ttc<5s"
    return 0, "safe"


def evaluate_vehicle(msg: dict, cane_lat: float, cane_lng: float) -> RiskResult:
    if not int(msg.get("gps_valid", 0)):
        return RiskResult(float("nan"), None, 0, "gps_invalid")

    lat = float(msg["lat"])
    lng = float(msg["lng"])
    speed = float(msg.get("speed_mps", msg.get("speed", 0.0)))

    distance_m = haversine_m(cane_lat, cane_lng, lat, lng)
    ttc_s = estimate_ttc(distance_m, speed)
    risk, reason = classify_risk(distance_m, ttc_s)
    return RiskResult(distance_m, ttc_s, risk, reason)


def open_serial(port: str, baud: int, timeout_s: float) -> serial.Serial:
    return serial.Serial(
        port=port,
        baudrate=baud,
        timeout=timeout_s,
        write_timeout=timeout_s,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read RSU bridge JSON and reply with risk.")
    parser.add_argument("--port", default="/dev/ttyTHS1", help="Jetson serial port")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    parser.add_argument("--cane-lat", type=float, default=37.000000, help="Fallback cane/reference latitude")
    parser.add_argument("--cane-lng", type=float, default=127.000000, help="Fallback cane/reference longitude")
    parser.add_argument(
        "--cane-stale-sec",
        type=float,
        default=2.0,
        help="Use fallback position if the last cane packet is older than this many seconds",
    )
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial timeout seconds")
    parser.add_argument("--no-reply", action="store_true", help="Do not write risk replies back to ESP32")
    return parser.parse_args()


def valid_position(msg: dict) -> bool:
    return bool(int(msg.get("gps_valid", 0))) and "lat" in msg and "lng" in msg


def update_cane_state(msg: dict) -> CaneState | None:
    if not valid_position(msg):
        return None
    return CaneState(
        node_id=int(msg.get("node_id", 0)),
        lat=float(msg["lat"]),
        lng=float(msg["lng"]),
        seq=msg.get("seq"),
        updated_monotonic=time.monotonic(),
    )


def choose_cane_reference(
    cane_state: CaneState | None,
    fallback_lat: float,
    fallback_lng: float,
    stale_sec: float,
) -> tuple[float, float, int, str]:
    if cane_state is not None and time.monotonic() - cane_state.updated_monotonic <= stale_sec:
        return cane_state.lat, cane_state.lng, cane_state.node_id, "live_cane"
    return fallback_lat, fallback_lng, 0, "fallback_cane"


def main() -> int:
    args = parse_args()
    cane_state: CaneState | None = None

    print(
        f"[JETSON] opening {args.port} baud={args.baud} "
        f"fallback_cane=({args.cane_lat:.6f},{args.cane_lng:.6f})"
    )

    try:
        ser = open_serial(args.port, args.baud, args.timeout)
    except serial.SerialException as exc:
        print(f"[JETSON] failed to open serial port: {exc}", file=sys.stderr)
        return 2

    with ser:
        print("[JETSON] ready. Waiting for ESP32 JSON lines...")
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
                    print(f"[CANE] ignored invalid position id={msg.get('node_id')} seq={msg.get('seq')}")
                    continue
                cane_state = updated
                print(
                    "[CANE] "
                    f"id={cane_state.node_id} seq={cane_state.seq} "
                    f"lat={cane_state.lat:.6f} lng={cane_state.lng:.6f}"
                )
                continue

            if msg_type != "vehicle":
                print(f"[IGNORE] {msg}")
                continue

            cane_lat, cane_lng, cane_id, cane_source = choose_cane_reference(
                cane_state,
                args.cane_lat,
                args.cane_lng,
                args.cane_stale_sec,
            )
            result = evaluate_vehicle(msg, cane_lat, cane_lng)
            ttc_text = "inf" if result.ttc_s is None else f"{result.ttc_s:.2f}"
            vehicle_id = int(msg.get("node_id", 0))

            print(
                "[VEHICLE] "
                f"id={vehicle_id} seq={msg.get('seq')} cane={cane_id} source={cane_source} "
                f"dist={result.distance_m:.2f}m ttc={ttc_text}s "
                f"risk={result.risk} reason={result.reason}"
            )

            if not args.no_reply:
                reply = {
                    "target_id": cane_id,
                    "src_id": vehicle_id,
                    "risk": result.risk,
                    "distance_m": None if math.isnan(result.distance_m) else round(result.distance_m, 2),
                    "reason": result.reason,
                    "jetson_ms": int(time.time() * 1000),
                }
                ser.write((json.dumps(reply, separators=(",", ":")) + "\n").encode("utf-8"))
                ser.flush()


if __name__ == "__main__":
    raise SystemExit(main())
