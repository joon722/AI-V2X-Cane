#!/usr/bin/env python3
"""
Jetson UART probe for the V2X vehicle ESP32.

Reads JSON lines from the vehicle ESP32, computes distance/TTC against a cane
reference position, prints the result, and sends a simple risk reply back:

    {"risk":2}

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
    # Conservative demo thresholds. Tune after outdoor tests.
    if distance_m <= 8.0:
        return 3, "distance<=8m"
    if distance_m <= 18.0:
        return 2, "distance<=18m"
    if ttc_s is not None and ttc_s <= 4.0 and distance_m <= 35.0:
        return 2, "ttc<=4s_and_distance<=35m"
    if distance_m <= 30.0:
        return 1, "distance<=30m"
    if ttc_s is not None and ttc_s <= 8.0 and distance_m <= 50.0:
        return 1, "ttc<=8s_and_distance<=50m"
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
    parser = argparse.ArgumentParser(description="Read ESP32 vehicle JSON and reply with risk.")
    parser.add_argument("--port", default="/dev/ttyTHS1", help="Jetson serial port")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    parser.add_argument("--cane-lat", type=float, default=37.000000, help="Cane/reference latitude")
    parser.add_argument("--cane-lng", type=float, default=127.000000, help="Cane/reference longitude")
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial timeout seconds")
    parser.add_argument("--no-reply", action="store_true", help="Do not write risk replies back to ESP32")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    print(
        f"[JETSON] opening {args.port} baud={args.baud} "
        f"cane=({args.cane_lat:.6f},{args.cane_lng:.6f})"
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

            if msg.get("type") != "vehicle":
                print(f"[IGNORE] {msg}")
                continue

            result = evaluate_vehicle(msg, args.cane_lat, args.cane_lng)
            ttc_text = "inf" if result.ttc_s is None else f"{result.ttc_s:.2f}"

            print(
                "[VEHICLE] "
                f"id={msg.get('node_id')} seq={msg.get('seq')} "
                f"dist={result.distance_m:.2f}m ttc={ttc_text}s "
                f"risk={result.risk} reason={result.reason}"
            )

            if not args.no_reply:
                reply = {
                    "risk": result.risk,
                    "distance_m": None if math.isnan(result.distance_m) else round(result.distance_m, 2),
                    "reason": result.reason,
                    "jetson_ms": int(time.time() * 1000),
                }
                ser.write((json.dumps(reply, separators=(",", ":")) + "\n").encode("utf-8"))
                ser.flush()


if __name__ == "__main__":
    raise SystemExit(main())
