#!/usr/bin/env python3
"""Turn step 6 kinematics into a risk score and a 0-3 risk level.

The scoring table is the team's, copied verbatim (see below). The only thing
step 7 adds on top is a DCPA suppression gate: the team table has no term for
closest-approach distance, so a car grazing past the kerb and a car on a
head-on course score the same. Step 6 computes dcpa precisely to tell those
apart; this gate is where that distinction is finally used.

step2-6 are not modified; this file imports the pipeline and scores the
filtered (Kalman) track, which is the one step 7 was always meant to consume.
"""

import argparse
import csv
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from step3_parse_v2x import SOURCE_MODES, normalize_record, serial_lines
from step4_state_store import FRESH_WINDOW_S, StateStore, has_position
from step5_test_vehicle import SPEED_MPS, START_DISTANCE_M, TestVehicle
from step6_kinematics import GPS_SIGMA_M, KinematicsPipeline, to_float


# --- Team scoring table -----------------------------------------------------
# Verbatim copy of tmp/AI-V2X-Cane-audit/scripts/risk_calculator.py. The audit
# folder is throwaway, so importing from it is fragile; the numbers are frozen
# here instead and mirrored by test_step7_risk.py. If the team retunes the
# table, re-sync these three functions and nothing else.

def calculate_ttc(distance_m, relative_speed_mps):
    """TTC in seconds. A non-approaching pair returns a large sentinel."""
    if relative_speed_mps <= 0:
        return 9999.0
    return distance_m / relative_speed_mps


def calculate_risk_score(
    distance_m,
    relative_speed_mps,
    vehicle_speed_mps,
    ttc,
    zone_base_risk=0,
):
    """0-100 risk from distance, TTC, relative speed, vehicle speed and zone."""
    score = 0.0

    # 1. Distance: up to 30
    if distance_m <= 10:
        score += 30
    elif distance_m <= 20:
        score += 25
    elif distance_m <= 40:
        score += 18
    elif distance_m <= 60:
        score += 10
    elif distance_m <= 100:
        score += 5

    # 2. TTC: up to 35
    if ttc <= 1:
        score += 35
    elif ttc <= 2:
        score += 30
    elif ttc <= 3:
        score += 25
    elif ttc <= 5:
        score += 20
    elif ttc <= 8:
        score += 15
    elif ttc <= 12:
        score += 8

    # 3. Relative speed: up to 20
    if relative_speed_mps >= 20:
        score += 20
    elif relative_speed_mps >= 15:
        score += 17
    elif relative_speed_mps >= 10:
        score += 13
    elif relative_speed_mps >= 5:
        score += 8
    elif relative_speed_mps > 0:
        score += 3

    # 4. Vehicle speed: up to 10
    if vehicle_speed_mps >= 20:
        score += 10
    elif vehicle_speed_mps >= 15:
        score += 8
    elif vehicle_speed_mps >= 10:
        score += 6
    elif vehicle_speed_mps >= 5:
        score += 3

    # 5. Hazard-zone correction: up to 5
    score += min(max(zone_base_risk, 0), 5)

    return round(min(score, 100), 2)


def classify_risk_level(risk_score):
    """0 safe, 1 caution, 2 warning, 3 danger."""
    if risk_score >= 70:
        return 3
    if risk_score >= 45:
        return 2
    if risk_score >= 20:
        return 1
    return 0

# --- End team scoring table -------------------------------------------------


# DCPA gate margins, tied to the GPS noise the filtered position carries. A dcpa
# inside 1 sigma could well be a real hit given measurement error, so it is not
# suppressed; only a miss clearly outside the noise (>3 sigma) is pulled down,
# and never to zero, because dcpa is itself an estimate that can be wrong.
DCPA_NEAR_M = GPS_SIGMA_M          # 2.5 m, ~1 sigma: no suppression inside this
DCPA_FAR_M = 3.0 * GPS_SIGMA_M     # 7.5 m, ~3 sigma: full suppression beyond
DCPA_FLOOR = 0.2                   # a clear miss still keeps a fifth of the score


def dcpa_gate(dcpa, near_m=DCPA_NEAR_M, far_m=DCPA_FAR_M, floor=DCPA_FLOOR):
    """Multiplier in [floor, 1] that fades risk out as the miss distance grows.

    dcpa is None when there is no closest approach ahead (receding or static);
    there is nothing to suppress, so the gate is transparent.
    """
    if dcpa is None or dcpa <= near_m:
        return 1.0
    if dcpa >= far_m:
        return floor
    frac = (dcpa - near_m) / (far_m - near_m)
    return 1.0 + frac * (floor - 1.0)


@dataclass(frozen=True)
class RiskAssessment:
    distance_m: float
    closing_los: float
    vehicle_speed_mps: float
    ttc: float
    dcpa: float | None
    base_score: float
    gate: float
    final_score: float
    risk_level: int


def assess_risk(
    kinematics,
    vehicle_speed_mps,
    zone_base_risk=0,
    near_m=DCPA_NEAR_M,
    far_m=DCPA_FAR_M,
    floor=DCPA_FLOOR,
):
    """Score one filtered-track Kinematics sample.

    `closing_los` is the line-of-sight closing speed, which is what the team
    table means by relative speed. TTC is recomputed through the team's own
    function so the score stays exactly consistent with that table.
    """
    closing = kinematics.closing_los
    ttc = calculate_ttc(kinematics.distance_m, closing)
    base = calculate_risk_score(
        kinematics.distance_m, closing, vehicle_speed_mps, ttc, zone_base_risk
    )
    gate = dcpa_gate(kinematics.dcpa, near_m, far_m, floor)
    final = round(base * gate, 2)
    return RiskAssessment(
        distance_m=kinematics.distance_m,
        closing_los=closing,
        vehicle_speed_mps=vehicle_speed_mps,
        ttc=ttc,
        dcpa=kinematics.dcpa,
        base_score=base,
        gate=round(gate, 3),
        final_score=final,
        risk_level=classify_risk_level(final),
    )


CSV_FIELDS = (
    "pc_time",
    "cane_seq",
    "vehicle_seq",
    "distance_m",
    "closing_los",
    "ttc",
    "vehicle_speed",
    "dcpa",
    "base_score",
    "gate",
    "final_score",
    "risk_level",
)


def fmt(value, digits=2):
    return "-" if value is None else f"{value:.{digits}f}"


def format_risk(assessment):
    return (
        f"[RISK] score={assessment.final_score:.2f} level={assessment.risk_level} "
        f"(base={assessment.base_score:.2f} "
        f"dcpa={fmt(assessment.dcpa)}m gate={assessment.gate:.2f})"
    )


def csv_row(now, store, assessment):
    return {
        "pc_time": round(now, 3),
        "cane_seq": store.latest["cane"]["seq"],
        "vehicle_seq": store.latest["vehicle"]["seq"],
        "distance_m": round(assessment.distance_m, 3),
        "closing_los": round(assessment.closing_los, 3),
        "ttc": round(assessment.ttc, 3),
        "vehicle_speed": round(assessment.vehicle_speed_mps, 3),
        "dcpa": None if assessment.dcpa is None else round(assessment.dcpa, 3),
        "base_score": assessment.base_score,
        "gate": assessment.gate,
        "final_score": assessment.final_score,
        "risk_level": assessment.risk_level,
    }


def append_row(csv_path, row):
    path = Path(csv_path)
    needs_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        if needs_header:
            writer.writeheader()
        writer.writerow(row)


def process_line(raw_line, source_mode, pipeline, csv_path, gate_params):
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

    now, _raw, filtered = result
    vehicle_speed = to_float(pipeline.store.latest["vehicle"]["speed_mps"])
    assessment = assess_risk(filtered, vehicle_speed, **gate_params)
    print(format_risk(assessment), flush=True)
    append_row(csv_path, csv_row(now, pipeline.store, assessment))
    return True


def inject_vehicle(vehicle, pipeline, source_mode, csv_path, gate_params, now):
    cane = pipeline.store.latest.get("cane")
    if cane is None or not has_position(cane) or not vehicle.is_due(now):
        return False
    payload, distance_m = vehicle.record(
        to_float(cane["lat"]), to_float(cane["lng"]), now
    )
    print(f"[TESTVEH] seq={payload['seq']} distance_m={distance_m:.1f}", flush=True)
    return process_line(json.dumps(payload), source_mode, pipeline, csv_path, gate_params)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Score the cane/vehicle pair into a risk level (0-3)."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial port")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate")
    parser.add_argument(
        "--source-mode",
        choices=SOURCE_MODES,
        default="test",
        help="origin of this input stream (default: test)",
    )
    parser.add_argument("--csv", default="step7_risk_log.csv", help="output CSV path")
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
        "--dcpa-near-m",
        type=float,
        default=DCPA_NEAR_M,
        help=f"dcpa below this is never suppressed (default: {DCPA_NEAR_M})",
    )
    parser.add_argument(
        "--dcpa-far-m",
        type=float,
        default=DCPA_FAR_M,
        help=f"dcpa above this is fully suppressed to the floor (default: {DCPA_FAR_M})",
    )
    parser.add_argument(
        "--dcpa-floor",
        type=float,
        default=DCPA_FLOOR,
        help=f"score multiplier for a clear miss (default: {DCPA_FLOOR})",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    store = StateStore(fresh_window_s=args.fresh_window_ms / 1000)
    pipeline = KinematicsPipeline(store)
    gate_params = {
        "near_m": args.dcpa_near_m,
        "far_m": args.dcpa_far_m,
        "floor": args.dcpa_floor,
    }
    print(
        f"[INFO] source_mode={args.source_mode} csv={args.csv} "
        f"dcpa_near_m={args.dcpa_near_m} dcpa_far_m={args.dcpa_far_m} "
        f"dcpa_floor={args.dcpa_floor}",
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
            process_line(line, args.source_mode, pipeline, args.csv, gate_params)
            if vehicle is not None:
                inject_vehicle(
                    vehicle, pipeline, args.source_mode, args.csv, gate_params, time.time()
                )
    except KeyboardInterrupt:
        print("\n[INFO] stopped", file=sys.stderr)


if __name__ == "__main__":
    main()
