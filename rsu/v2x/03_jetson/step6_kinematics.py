#!/usr/bin/env python3
"""Distance, closing speed and time-to-collision between the cane and a vehicle.

Two tracks are computed from the same input and reported side by side:

    raw       measured coordinates, speed/heading straight out of the JSON
    filtered  constant-velocity Kalman estimate of position and velocity

The raw track must agree with the step 5 generator's intended distance to the
last decimal, which is what proves the formulas. The filtered track is what
step 7 scores, and lags by design.
"""

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from step3_parse_v2x import SOURCE_MODES, normalize_record, serial_lines
from step4_state_store import FRESH_WINDOW_S, StateStore, has_position
from step5_test_vehicle import SPEED_MPS, START_DISTANCE_M, TestVehicle


# Same constant step 5 uses to place the simulated vehicle. Sharing it is
# deliberate: it makes "distance we generated" and "distance we measured"
# comparable exactly, so any mismatch is a bug rather than a model difference.
METERS_PER_DEGREE_LAT = 111320.0

# Horizontal accuracy of the GPS class used here (~2.5 m CEP).
GPS_SIGMA_M = 2.5
# Acceleration a tracked node can plausibly produce. Sized for a car rather than
# a pedestrian so the filter never lags behind the faster of the two.
ACCEL_SIGMA_MPS2 = 3.0
# A new node's speed is unknown, so the filter starts wide and lets the first
# few fixes pull it in.
INITIAL_SPEED_SIGMA_MPS = 10.0

# The GPS produces one fix per second while the radio repeats it every 100 ms,
# and transport adds its own delay. The fix timestamp (gps_time_ms, UTC
# milliseconds into the day, shared by every receiver) is therefore the honest
# time axis; arrival time is the fallback for rows that lack it.
GPS_TIME_INVALID_MS = 0xFFFFFFFF
MS_PER_DAY = 86_400_000

# A node silent for this long restarts its filter: extrapolating a stale
# velocity across a gap fabricates motion that was never measured.
MAX_TRACK_GAP_S = 10.0

# The receiver measures speed from the Doppler shift of the satellite signals,
# so it does not inherit position error and is available the moment the fix
# arrives - unlike a velocity recovered from differentiating noisy positions,
# which needs several fixes of lag. 0.5 m/s is conservative for this receiver
# class in motion (vibration, low speed course jitter).
DOPPLER_SPEED_SIGMA_MPS = 0.5

# Below this reported speed the cane is treated as standing still and its
# filter velocity is observed as zero. 2026-08-12 field data: a cane lying
# still reported speed noise up to 0.38 m/s while the position-only filter
# fabricated up to 6.85 m/s of phantom velocity - the zero observation kills
# the phantom, and walking (about 1 m/s) stays above the gate.
CANE_STILL_SPEED_MPS = 0.45
ZUPT_SIGMA_MPS = 0.2

# The vehicle's heading field is its actual motion direction (GPS course,
# reverse-corrected by the firmware), so the full Doppler vector is usable -
# but only while the node vouches for it (heading_valid) and moves fast enough
# for course to mean anything. The cane's heading is IMU pointing direction,
# not walking direction, so the cane never gets a vector observation.
VEHICLE_DOPPLER_MIN_SPEED_MPS = 0.4


def measurement_time(row):
    """(seconds, is_gps): when the row's fix was measured, not when it arrived.

    Rows without a usable GPS timestamp (older firmware, replayed logs, the
    0xFFFFFFFF no-fix sentinel) fall back to arrival time, which then carries
    the transport delay and fix age as error.
    """
    try:
        ms = int(float(row.get("gps_time_ms")))
    except (TypeError, ValueError):
        return float(row["pc_time"]), False
    if not 0 <= ms < MS_PER_DAY:
        return float(row["pc_time"]), False
    return ms / 1000.0, True

CSV_FIELDS = (
    "pc_time",
    "cane_seq",
    "vehicle_seq",
    "distance_raw",
    "closing_los_raw",
    "closing_diff_raw",
    "ttc_simple_raw",
    "tcpa_raw",
    "dcpa_raw",
    "distance_kf",
    "closing_los_kf",
    "ttc_simple_kf",
    "tcpa_kf",
    "dcpa_kf",
)


class LocalFrame:
    """Flat-earth east/north metres about a fixed origin.

    Bearings and dot products need a metric plane; latitude and longitude do
    not provide one. Over the few hundred metres this system covers the flat
    approximation stays far inside GPS noise.
    """

    def __init__(self, lat0, lng0):
        self.lat0 = lat0
        self.lng0 = lng0
        self._lng_scale = METERS_PER_DEGREE_LAT * math.cos(math.radians(lat0))

    def to_enu(self, lat, lng):
        return (lng - self.lng0) * self._lng_scale, (lat - self.lat0) * METERS_PER_DEGREE_LAT


def velocity_from_heading(speed_mps, heading_deg):
    """Split a speed and compass heading into east/north components."""
    heading = math.radians(heading_deg)
    return speed_mps * math.sin(heading), speed_mps * math.cos(heading)


@dataclass(frozen=True)
class Kinematics:
    distance_m: float
    closing_los: float
    closing_diff: float | None
    ttc_simple: float | None
    tcpa: float | None
    dcpa: float | None
    # Where the vehicle is, not just how far away. A scalar distance cannot say
    # whether the car is straight ahead or off to one side, which is the one
    # thing a display has to know to draw it. Same metric plane as LocalFrame:
    # east, north, in metres, relative to the cane.
    rel_east: float
    rel_north: float


def relative_kinematics(
    cane_pos,
    cane_vel,
    veh_pos,
    veh_vel,
    prev_distance_m=None,
    dt_s=None,
):
    """Relative motion of the vehicle with respect to the cane.

    `closing_los` projects the relative velocity onto the line of sight, so it
    answers "how fast is the gap shrinking" rather than "how fast are they
    going". `tcpa`/`dcpa` add the part a scalar TTC cannot express: a vehicle
    passing to one side has a short TTC but a `dcpa` that shows it misses.
    """
    rx = veh_pos[0] - cane_pos[0]
    ry = veh_pos[1] - cane_pos[1]
    vx = veh_vel[0] - cane_vel[0]
    vy = veh_vel[1] - cane_vel[1]

    distance_m = math.hypot(rx, ry)
    r_dot_v = rx * vx + ry * vy
    v_squared = vx * vx + vy * vy

    closing_los = 0.0 if distance_m == 0.0 else -r_dot_v / distance_m
    ttc_simple = distance_m / closing_los if closing_los > 0.0 else None

    tcpa = None
    dcpa = None
    if v_squared > 0.0:
        candidate = -r_dot_v / v_squared
        # A negative time means the closest approach already happened; the pair
        # is separating and there is nothing ahead to warn about.
        if candidate >= 0.0:
            tcpa = candidate
            dcpa = math.hypot(rx + vx * tcpa, ry + vy * tcpa)

    closing_diff = None
    if prev_distance_m is not None and dt_s is not None and dt_s > 0.0:
        closing_diff = (prev_distance_m - distance_m) / dt_s

    return Kinematics(
        distance_m=distance_m,
        closing_los=closing_los,
        closing_diff=closing_diff,
        ttc_simple=ttc_simple,
        tcpa=tcpa,
        dcpa=dcpa,
        rel_east=rx,
        rel_north=ry,
    )


class KalmanCV1D:
    """Constant-velocity Kalman filter on a single axis.

    East and north are uncoupled under this model, so two of these are exactly
    equivalent to one 4x4 filter while staying small enough to read, and
    without pulling numpy onto the Jetson.
    """

    def __init__(
        self,
        sigma_pos=GPS_SIGMA_M,
        sigma_accel=ACCEL_SIGMA_MPS2,
        initial_speed_sigma=INITIAL_SPEED_SIGMA_MPS,
    ):
        self.measurement_var = sigma_pos**2
        self.accel_var = sigma_accel**2
        self.initial_speed_var = initial_speed_sigma**2
        self.pos = None
        self.vel = 0.0
        self.cov = [[0.0, 0.0], [0.0, 0.0]]

    def observe(self, measurement, dt):
        if self.pos is None:
            self.pos = measurement
            self.vel = 0.0
            self.cov = [
                [self.measurement_var, 0.0],
                [0.0, self.initial_speed_var],
            ]
            return
        self._predict(dt)
        self._update(measurement)

    def predict_to(self, dt):
        """Position and velocity `dt` ahead, leaving the filter untouched."""
        return self.pos + self.vel * dt, self.vel

    def observe_velocity(self, measurement, var):
        """Direct velocity measurement update (H = [0, 1]).

        Applied at the same instant as the position update it follows, so
        there is no predict step here: time has not advanced.
        """
        if self.pos is None:
            return
        (p00, p01), (p10, p11) = self.cov
        innovation_var = p11 + var
        gain_pos = p01 / innovation_var
        gain_vel = p11 / innovation_var

        residual = measurement - self.vel
        self.pos += gain_pos * residual
        self.vel += gain_vel * residual

        self.cov = [
            [p00 - gain_pos * p10, p01 - gain_pos * p11],
            [(1.0 - gain_vel) * p10, (1.0 - gain_vel) * p11],
        ]

    def _predict(self, dt):
        self.pos += self.vel * dt
        (p00, p01), (p10, p11) = self.cov

        # F P F^T with F = [[1, dt], [0, 1]]
        n00 = p00 + dt * (p01 + p10) + dt * dt * p11
        n01 = p01 + dt * p11
        n10 = p10 + dt * p11
        n11 = p11

        # Q for white-noise acceleration of variance accel_var.
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        self.cov = [
            [n00 + self.accel_var * dt4 / 4.0, n01 + self.accel_var * dt3 / 2.0],
            [n10 + self.accel_var * dt3 / 2.0, n11 + self.accel_var * dt2],
        ]

    def _update(self, measurement):
        (p00, p01), (p10, p11) = self.cov
        innovation_var = p00 + self.measurement_var
        gain_pos = p00 / innovation_var
        gain_vel = p10 / innovation_var

        residual = measurement - self.pos
        self.pos += gain_pos * residual
        self.vel += gain_vel * residual

        self.cov = [
            [(1.0 - gain_pos) * p00, (1.0 - gain_pos) * p01],
            [p10 - gain_vel * p00, p11 - gain_vel * p01],
        ]


class NodeTracker:
    """Kalman-filtered position and velocity for one node."""

    def __init__(self, sigma_pos=GPS_SIGMA_M, sigma_accel=ACCEL_SIGMA_MPS2):
        self.sigma_pos = sigma_pos
        self.sigma_accel = sigma_accel
        self.east = KalmanCV1D(sigma_pos, sigma_accel)
        self.north = KalmanCV1D(sigma_pos, sigma_accel)
        self.last_time = None
        self.time_is_gps = None
        self.day_offset = 0.0

    @property
    def ready(self):
        return self.last_time is not None

    def _reset(self):
        self.east = KalmanCV1D(self.sigma_pos, self.sigma_accel)
        self.north = KalmanCV1D(self.sigma_pos, self.sigma_accel)
        self.last_time = None
        self.day_offset = 0.0

    def observe(self, east, north, timestamp, is_gps=False):
        """Feed one fix. Returns True when it advanced the filter.

        The return value lets the caller attach same-instant follow-up
        measurements (Doppler velocity) only to fixes that were actually new.
        """
        # Switching between the GPS clock and the arrival clock mid-track would
        # make dt meaningless, so a base change starts the track over.
        if self.time_is_gps is not None and is_gps != self.time_is_gps:
            self._reset()
        self.time_is_gps = is_gps

        timestamp += self.day_offset
        if self.last_time is not None:
            if is_gps and timestamp < self.last_time - 43200.0:
                # GPS time-of-day wrapped past midnight.
                self.day_offset += 86400.0
                timestamp += 86400.0
            if timestamp <= self.last_time:
                # The same fix arriving again through the faster send loop
                # carries no new information; feeding it again would only
                # shrink the covariance without cause.
                return False
            if timestamp - self.last_time > MAX_TRACK_GAP_S:
                self._reset()

        dt = 0.0 if self.last_time is None else timestamp - self.last_time
        self.east.observe(east, dt)
        self.north.observe(north, dt)
        self.last_time = timestamp
        return True

    def observe_velocity(self, vel_east, vel_north, sigma):
        """Doppler velocity for the fix just accepted by observe()."""
        var = sigma * sigma
        self.east.observe_velocity(vel_east, var)
        self.north.observe_velocity(vel_north, var)

    def state_ahead(self, dt):
        """Position and velocity `dt` seconds past the last accepted fix."""
        dt = max(0.0, dt)
        pos_e, vel_e = self.east.predict_to(dt)
        pos_n, vel_n = self.north.predict_to(dt)
        return pos_e, pos_n, vel_e, vel_n

    def state_at(self, timestamp):
        """Extrapolate to a shared clock so both nodes are compared at one instant."""
        return self.state_ahead(timestamp - self.last_time)


def to_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


@dataclass(frozen=True)
class _DistanceAnchor:
    """The sample the raw closing speed is differentiated against."""

    distance_m: float
    raw_time: float
    cane_time: float
    vehicle_time: float

    def is_superseded_by(self, cane, vehicle):
        return (
            cane["pc_time"] > self.cane_time
            and vehicle["pc_time"] > self.vehicle_time
        )


class KinematicsPipeline:
    """Turns the step 4 state store into per-update relative kinematics."""

    def __init__(self, store, sigma_pos=GPS_SIGMA_M, sigma_accel=ACCEL_SIGMA_MPS2):
        self.store = store
        self.frame = None
        self.trackers = {
            "cane": NodeTracker(sigma_pos, sigma_accel),
            "vehicle": NodeTracker(sigma_pos, sigma_accel),
        }
        self.anchor = None
        # Filtered (east, north, v_east, v_north) for each node as of the last
        # successful compute(). None until then.
        self.last_states = None

    def observe(self, row):
        if not has_position(row):
            return
        lat = to_float(row["lat"])
        lng = to_float(row["lng"])
        if self.frame is None:
            # The cane is the thing being protected, so it anchors the frame and
            # its own coordinates stay at the origin.
            if row["type"] != "cane":
                return
            self.frame = LocalFrame(lat, lng)
        east, north = self.frame.to_enu(lat, lng)
        t_meas, is_gps = measurement_time(row)
        tracker = self.trackers[row["type"]]
        if not tracker.observe(east, north, t_meas, is_gps):
            return
        self._observe_doppler(row, tracker)

    def _observe_doppler(self, row, tracker):
        """Fold the node's own Doppler speed into the freshly updated filter.

        Positions alone recover velocity only after several fixes of lag, and
        position noise on a stationary node fabricates velocity outright. The
        receiver already measured the real thing; use it, per node:

        - cane: zero-velocity observation while its reported speed says it is
          standing still. Its heading is IMU pointing, not walking direction,
          so a moving cane gets no vector observation.
        - vehicle: full velocity vector, but only while the firmware vouches
          for the motion heading (heading_valid) and the speed is above the
          course-jitter floor. No zero-velocity clamp for the vehicle: when
          its GPS misreads motion as standstill, clamping would suppress a
          real approach, and misses cost more than false alarms here.
        """
        speed = to_float(row.get("speed_mps"))
        if row["type"] == "cane":
            if speed < CANE_STILL_SPEED_MPS:
                tracker.observe_velocity(0.0, 0.0, ZUPT_SIGMA_MPS)
            return
        heading_valid = to_float(row.get("heading_valid"), 0.0) == 1.0
        if heading_valid and speed >= VEHICLE_DOPPLER_MIN_SPEED_MPS:
            vel_east, vel_north = velocity_from_heading(
                speed, to_float(row.get("heading_deg"))
            )
            tracker.observe_velocity(vel_east, vel_north, DOPPLER_SPEED_SIGMA_MPS)

    def compute(self):
        """Kinematics for the current pair, or None when it is not computable."""
        _, _, risk_valid = self.store.snapshot(now=self._latest_time())
        if not risk_valid or self.frame is None:
            return None
        if not all(tracker.ready for tracker in self.trackers.values()):
            return None

        cane = self.store.latest["cane"]
        vehicle = self.store.latest["vehicle"]

        # Compare both nodes on one clock: the GPS fix times when both sides
        # carry them, arrival times otherwise. A mixed pair shares no clock
        # except arrival, so it falls back wholesale.
        cane_t, cane_gps = measurement_time(cane)
        veh_t, veh_gps = measurement_time(vehicle)
        if cane_gps != veh_gps:
            cane_t = float(cane["pc_time"])
            veh_t = float(vehicle["pc_time"])

        # The filter can extrapolate, so it reports at the newest timestamp.
        now = max(cane_t, veh_t)
        # The raw distance cannot. It is only as current as its stalest input,
        # and dating it any later makes the derivative below read a time gap
        # across which the distance never actually moved.
        raw_time = min(cane_t, veh_t)

        cane_pos = self.frame.to_enu(to_float(cane["lat"]), to_float(cane["lng"]))
        veh_pos = self.frame.to_enu(to_float(vehicle["lat"]), to_float(vehicle["lng"]))
        cane_vel = velocity_from_heading(
            to_float(cane["speed_mps"]), to_float(cane["heading_deg"])
        )
        veh_vel = velocity_from_heading(
            to_float(vehicle["speed_mps"]), to_float(vehicle["heading_deg"])
        )

        # Differentiating the distance is only meaningful across an interval in
        # which both nodes actually reported again. Measuring across a shorter
        # span reads whatever happened to arrive first: a few milliseconds of no
        # movement (closing speed 0) or a whole update in no time at all.
        prev_distance = None
        dt_s = None
        if self.anchor is not None and self.anchor.is_superseded_by(cane, vehicle):
            prev_distance = self.anchor.distance_m
            dt_s = raw_time - self.anchor.raw_time
            # A clock-base change (GPS time <-> arrival time) makes the two
            # anchor times incomparable; drop the derivative for that round.
            if not 0.0 < dt_s <= MAX_TRACK_GAP_S:
                prev_distance = None
                dt_s = None

        raw = relative_kinematics(
            cane_pos,
            cane_vel,
            veh_pos,
            veh_vel,
            prev_distance_m=prev_distance,
            dt_s=dt_s,
        )
        if self.anchor is None or raw.closing_diff is not None:
            self.anchor = _DistanceAnchor(
                distance_m=raw.distance_m,
                raw_time=raw_time,
                cane_time=cane["pc_time"],
                vehicle_time=vehicle["pc_time"],
            )

        # Each tracker extrapolates by how far the shared instant sits past its
        # own newest fix, so a node whose GPS clock differs from its arrival
        # clock still projects the right amount.
        cane_e, cane_n, cane_ve, cane_vn = self.trackers["cane"].state_ahead(now - cane_t)
        veh_e, veh_n, veh_ve, veh_vn = self.trackers["vehicle"].state_ahead(now - veh_t)
        # The model needs the same filtered states this comparison is built on.
        # Recomputing them downstream would duplicate the clock reconciliation
        # above and drift from it the moment either side changes.
        self.last_states = (
            (cane_e, cane_n, cane_ve, cane_vn),
            (veh_e, veh_n, veh_ve, veh_vn),
        )
        filtered = relative_kinematics(
            (cane_e, cane_n),
            (cane_ve, cane_vn),
            (veh_e, veh_n),
            (veh_ve, veh_vn),
        )
        return now, raw, filtered

    def _latest_time(self):
        times = [row["pc_time"] for row in self.store.latest.values()]
        return max(times) if times else time.time()


def fmt(value, digits=2):
    return "-" if value is None else f"{value:.{digits}f}"


def format_kinematics(raw, filtered):
    return (
        f"[KIN] d={raw.distance_m:.2f}m "
        f"closing los/diff={fmt(raw.closing_los)}/{fmt(raw.closing_diff)} "
        f"kf={fmt(filtered.closing_los)} "
        f"ttc={fmt(raw.ttc_simple)}s "
        f"tcpa={fmt(raw.tcpa)}s dcpa={fmt(raw.dcpa)}m"
    )


def csv_row(now, store, raw, filtered):
    return {
        "pc_time": round(now, 3),
        "cane_seq": store.latest["cane"]["seq"],
        "vehicle_seq": store.latest["vehicle"]["seq"],
        "distance_raw": round(raw.distance_m, 3),
        "closing_los_raw": round(raw.closing_los, 3),
        "closing_diff_raw": None if raw.closing_diff is None else round(raw.closing_diff, 3),
        "ttc_simple_raw": None if raw.ttc_simple is None else round(raw.ttc_simple, 3),
        "tcpa_raw": None if raw.tcpa is None else round(raw.tcpa, 3),
        "dcpa_raw": None if raw.dcpa is None else round(raw.dcpa, 3),
        "distance_kf": round(filtered.distance_m, 3),
        "closing_los_kf": round(filtered.closing_los, 3),
        "ttc_simple_kf": None if filtered.ttc_simple is None else round(filtered.ttc_simple, 3),
        "tcpa_kf": None if filtered.tcpa is None else round(filtered.tcpa, 3),
        "dcpa_kf": None if filtered.dcpa is None else round(filtered.dcpa, 3),
    }


def append_row(csv_path, row):
    path = Path(csv_path)
    needs_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        if needs_header:
            writer.writeheader()
        writer.writerow(row)


def process_line(raw_line, source_mode, pipeline, csv_path):
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

    if not pipeline.store.update(row):
        print(f"[WARN] ignored_type type={row['type']!r}", file=sys.stderr)
        return False

    pipeline.observe(row)
    result = pipeline.compute()
    if result is None:
        return True

    now, raw, filtered = result
    print(format_kinematics(raw, filtered), flush=True)
    append_row(csv_path, csv_row(now, pipeline.store, raw, filtered))
    return True


def inject_vehicle(vehicle, pipeline, source_mode, csv_path, now):
    cane = pipeline.store.latest.get("cane")
    if cane is None or not has_position(cane) or not vehicle.is_due(now):
        return False
    payload, distance_m = vehicle.record(
        to_float(cane["lat"]), to_float(cane["lng"]), now
    )
    print(f"[TESTVEH] seq={payload['seq']} distance_m={distance_m:.1f}", flush=True)
    return process_line(json.dumps(payload), source_mode, pipeline, csv_path)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compute distance, closing speed, TTC and CPA for the cane/vehicle pair."
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
        "--csv", default="step6_kinematics_log.csv", help="output CSV path"
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
    parser.add_argument("--vehicle-speed", type=float, default=SPEED_MPS)
    parser.add_argument("--vehicle-start-m", type=float, default=START_DISTANCE_M)
    parser.add_argument(
        "--gps-sigma-m",
        type=float,
        default=GPS_SIGMA_M,
        help=f"GPS position noise in metres (default: {GPS_SIGMA_M})",
    )
    parser.add_argument(
        "--accel-sigma",
        type=float,
        default=ACCEL_SIGMA_MPS2,
        help=f"process noise as acceleration in m/s^2 (default: {ACCEL_SIGMA_MPS2})",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    store = StateStore(fresh_window_s=args.fresh_window_ms / 1000)
    pipeline = KinematicsPipeline(
        store, sigma_pos=args.gps_sigma_m, sigma_accel=args.accel_sigma
    )
    print(
        f"[INFO] source_mode={args.source_mode} csv={args.csv} "
        f"gps_sigma_m={args.gps_sigma_m} accel_sigma={args.accel_sigma}",
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
            process_line(line, args.source_mode, pipeline, args.csv)
            if vehicle is not None:
                inject_vehicle(vehicle, pipeline, args.source_mode, args.csv, time.time())
    except KeyboardInterrupt:
        print("\n[INFO] stopped", file=sys.stderr)


if __name__ == "__main__":
    main()
