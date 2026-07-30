#!/usr/bin/env python3
"""Keep the latest cane/vehicle record and report whether they can be paired."""

import argparse
import json
import sys
import time

from step3_parse_v2x import SOURCE_MODES, normalize_record, serial_lines
from step5_test_vehicle import SPEED_MPS, START_DISTANCE_M, TestVehicle


FRESH_WINDOW_S = 0.5
TRACKED_TYPES = ("cane", "vehicle")


def has_position(row):
    """True when the record carries coordinates usable for distance math.

    gps_valid only reports the fix quality. The cane keeps sending its fallback
    coordinates while gps_valid=0, and those are still what step 5+ measures
    against, so trust source_mode for provenance and check the numbers here.
    """
    try:
        lat = float(row.get("lat", ""))
        lng = float(row.get("lng", ""))
    except (TypeError, ValueError):
        return False
    # (0, 0) is the usual "no fix" sentinel rather than a real location.
    return not (lat == 0.0 and lng == 0.0)


class StateStore:
    """Latest state per node type, with freshness judged against wall time."""

    def __init__(self, fresh_window_s=FRESH_WINDOW_S):
        self.fresh_window_s = fresh_window_s
        self.latest = {}

    def update(self, row):
        node_type = row["type"]
        if node_type not in TRACKED_TYPES:
            return False
        self.latest[node_type] = row
        return True

    def status(self, node_type, now):
        row = self.latest.get(node_type)
        if row is None:
            return "MISSING"
        if now - row["pc_time"] > self.fresh_window_s:
            return "STALE"
        return "READY"

    def snapshot(self, now=None):
        now = time.time() if now is None else now
        statuses = {node: self.status(node, now) for node in TRACKED_TYPES}
        pair_valid = all(status == "READY" for status in statuses.values())
        # Risk needs both positions, so a fresh pair without coordinates is not enough.
        risk_valid = pair_valid and all(
            has_position(self.latest[node]) for node in TRACKED_TYPES
        )
        return statuses, pair_valid, risk_valid


def format_snapshot(statuses, pair_valid, risk_valid):
    lines = [f"{node}={statuses[node]}" for node in TRACKED_TYPES]
    lines.append(f"pair_valid={pair_valid}")
    lines.append(f"risk_valid={risk_valid}")
    return "\n".join(lines)


def process_line(raw_line, source_mode, store):
    line = raw_line.strip()
    if not line:
        return False
    try:
        payload = json.loads(line)
        if not isinstance(payload, dict):
            raise ValueError("top-level JSON must be an object")
        row = normalize_record(payload, source_mode)
    except (json.JSONDecodeError, ValueError) as exc:
        print(f"[WARN] parse_failed error={exc} source={source_mode}", file=sys.stderr)
        return False

    if not store.update(row):
        print(f"[WARN] ignored_type type={row['type']!r}", file=sys.stderr)
        return False

    print(
        f"[STATE] type={row['type']} seq={row['seq']} "
        f"gps_valid={row['gps_valid']} source={row['source_mode']}",
        flush=True,
    )
    statuses, pair_valid, risk_valid = store.snapshot(now=row["pc_time"])
    print(format_snapshot(statuses, pair_valid, risk_valid), flush=True)
    return True


def inject_vehicle(vehicle, store, source_mode, now):
    """Feed one simulated vehicle record aimed at the cane's latest position.

    Targeting the live cane record instead of fixed coordinates keeps the
    simulated approach meaningful whether the cane has a GPS fix or is running
    on its indoor fallback position.
    """
    cane = store.latest.get("cane")
    if cane is None or not has_position(cane):
        return False
    if not vehicle.is_due(now):
        return False

    payload, distance_m = vehicle.record(float(cane["lat"]), float(cane["lng"]), now)
    print(f"[TESTVEH] seq={payload['seq']} distance_m={distance_m:.1f}", flush=True)
    return process_line(json.dumps(payload), source_mode, store)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Track latest cane/vehicle state and report pairing validity."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial port")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate")
    parser.add_argument(
        "--source-mode",
        choices=SOURCE_MODES,
        default="test",
        help="origin of this input stream (default: test)",
    )
    parser.add_argument(
        "--fresh-window-ms",
        type=int,
        default=int(FRESH_WINDOW_S * 1000),
        help="how recent a record must be to count as READY (default: 500)",
    )
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="read JSON lines from stdin instead of a serial port",
    )
    parser.add_argument(
        "--test-vehicle",
        action="store_true",
        help="inject a simulated vehicle closing in on the cane (step 5)",
    )
    parser.add_argument(
        "--vehicle-speed",
        type=float,
        default=SPEED_MPS,
        help=f"simulated vehicle speed in m/s (default: {SPEED_MPS})",
    )
    parser.add_argument(
        "--vehicle-start-m",
        type=float,
        default=START_DISTANCE_M,
        help=f"how far the simulated vehicle starts out (default: {START_DISTANCE_M})",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    store = StateStore(fresh_window_s=args.fresh_window_ms / 1000)
    print(
        f"[INFO] source_mode={args.source_mode} fresh_window_ms={args.fresh_window_ms}",
        file=sys.stderr,
    )
    vehicle = None
    if args.test_vehicle:
        vehicle = TestVehicle(
            start_distance_m=args.vehicle_start_m, speed_mps=args.vehicle_speed
        )
        print(
            f"[INFO] test_vehicle start_m={args.vehicle_start_m} "
            f"speed={args.vehicle_speed}",
            file=sys.stderr,
        )

    lines = sys.stdin if args.stdin else serial_lines(args.port, args.baud)
    try:
        for line in lines:
            process_line(line, args.source_mode, store)
            if vehicle is not None:
                inject_vehicle(vehicle, store, args.source_mode, time.time())
    except KeyboardInterrupt:
        print("\n[INFO] stopped", file=sys.stderr)


if __name__ == "__main__":
    main()
