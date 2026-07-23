#!/usr/bin/env python3
"""Simulate a vehicle closing in on a target position.

Step 9 replaces this producer with a real vehicle ESP32. Everything downstream
reads the same record shape, so only the producer changes.
"""

import argparse
import json
import math
import sys
import time


NODE_ID = 900000001
START_DISTANCE_M = 50.0
SPEED_MPS = 5.0
# 0 = the vehicle starts due north of the target and drives south into it.
APPROACH_BEARING_DEG = 0.0
PERIOD_S = 0.2
METERS_PER_DEGREE_LAT = 111320.0


def offset_position(lat, lng, bearing_deg, distance_m):
    """Point `distance_m` away from (lat, lng) along `bearing_deg` (0=N, 90=E).

    Flat-earth approximation. Over the few hundred metres this simulation covers
    its error stays far below the GPS noise it stands in for.
    """
    bearing = math.radians(bearing_deg)
    d_lat = distance_m * math.cos(bearing) / METERS_PER_DEGREE_LAT
    d_lng = distance_m * math.sin(bearing) / (
        METERS_PER_DEGREE_LAT * math.cos(math.radians(lat))
    )
    return lat + d_lat, lng + d_lng


class TestVehicle:
    """Emits a record every `period_s`, each one closer to the target."""

    def __init__(
        self,
        start_distance_m=START_DISTANCE_M,
        speed_mps=SPEED_MPS,
        bearing_deg=APPROACH_BEARING_DEG,
        period_s=PERIOD_S,
    ):
        self.start_distance_m = start_distance_m
        self.speed_mps = speed_mps
        self.bearing_deg = bearing_deg
        self.period_s = period_s
        self.seq = 0
        self.started_at = None
        self.last_emit_at = None

    def is_due(self, now):
        if self.last_emit_at is None:
            return True
        return now - self.last_emit_at >= self.period_s

    def distance_at(self, elapsed_s):
        # Stops on top of the target rather than driving through it, so the
        # distance readout stays monotonic for the whole run.
        return max(0.0, self.start_distance_m - self.speed_mps * elapsed_s)

    def record(self, target_lat, target_lng, now):
        """Return (payload, distance_m) for this tick and advance the clock."""
        if self.started_at is None:
            self.started_at = now
        self.last_emit_at = now
        self.seq += 1

        distance_m = self.distance_at(now - self.started_at)
        lat, lng = offset_position(
            target_lat, target_lng, self.bearing_deg, distance_m
        )
        payload = {
            "type": "vehicle",
            "node_id": NODE_ID,
            "seq": self.seq,
            "gps_valid": 1,
            "lat": lat,
            "lng": lng,
            "speed_mps": self.speed_mps,
            # Heading is the direction of travel: the reverse of where it started.
            "heading_deg": (self.bearing_deg + 180.0) % 360.0,
            "node_risk": 0,
            "source_mode": "simulation",
        }
        return payload, distance_m


def parse_args():
    parser = argparse.ArgumentParser(
        description="Stream simulated vehicle JSON approaching a fixed target."
    )
    parser.add_argument("--target-lat", type=float, required=True)
    parser.add_argument("--target-lng", type=float, required=True)
    parser.add_argument("--speed", type=float, default=SPEED_MPS)
    parser.add_argument("--start-m", type=float, default=START_DISTANCE_M)
    parser.add_argument("--bearing", type=float, default=APPROACH_BEARING_DEG)
    parser.add_argument("--period", type=float, default=PERIOD_S)
    parser.add_argument("--duration", type=float, default=20.0)
    return parser.parse_args()


def main():
    args = parse_args()
    vehicle = TestVehicle(
        start_distance_m=args.start_m,
        speed_mps=args.speed,
        bearing_deg=args.bearing,
        period_s=args.period,
    )
    print(
        f"[INFO] start_m={args.start_m} speed={args.speed} bearing={args.bearing}",
        file=sys.stderr,
    )
    deadline = time.time() + args.duration
    try:
        while time.time() < deadline:
            now = time.time()
            if vehicle.is_due(now):
                payload, distance_m = vehicle.record(
                    args.target_lat, args.target_lng, now
                )
                print(json.dumps(payload), flush=True)
                print(f"[TESTVEH] distance_m={distance_m:.1f}", file=sys.stderr)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[INFO] stopped", file=sys.stderr)


if __name__ == "__main__":
    main()
