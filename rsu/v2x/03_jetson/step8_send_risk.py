#!/usr/bin/env python3
"""Send step 7's risk_level down to the RSU, which broadcasts it to the cane.

The downlink shape ({"target_id":0,"risk":N}) and the fact that the RSU echoes
it onto the cane's node_risk were both verified by probe_risk_downlink.py. This
step wires step 7's live risk_level into that path.

Two policies sit in front of the transmit, both in RiskTransmitter so they can
be tested without a serial port:

  trust gating   a fallback (gps_valid=0) position makes the risk spatially
                 meaningless, so a nonzero value is held back and only a 0 (which
                 clears a stale alarm) goes out, unless --tx-untrusted is set.
  rate limiting  the cane reports at ~10 Hz; sending every record floods the RSU,
                 so we send on change and otherwise only on a heartbeat.

step2-7 are not modified; this file imports the pipeline and scores the filtered
track itself, the same small duplication step 7 carries from step 6.
"""

import argparse
import csv
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from step3_parse_v2x import SOURCE_MODES, normalize_record
from step4_state_store import FRESH_WINDOW_S, StateStore, has_position
from step5_test_vehicle import SPEED_MPS, START_DISTANCE_M, TestVehicle
from step6_kinematics import KinematicsPipeline, to_float
from step7_risk import DCPA_FAR_M, DCPA_FLOOR, DCPA_NEAR_M, assess_risk
from step8_stability import HOLD_S, LevelStabilizer


HEARTBEAT_S = 1.0

# After the RSU accepts a downlink it echoes these two record types back on the
# same serial line (documented in the session summary). They confirm the command
# landed; they are not pipeline input, so step 8 consumes them without warning.
RSU_ACK_TYPES = ("risk_tx", "risk_broadcast_to_seen")


def _gps_trusted(cane_gps_valid):
    """gps_valid is 1 for a real fix, 0 for the indoor fallback coordinate."""
    return to_float(cane_gps_valid) >= 0.5


@dataclass(frozen=True)
class TxDecision:
    should_send: bool
    computed_level: int
    effective_level: int
    trusted: bool
    reason: str  # "change" | "heartbeat" | "hold"


class RiskTransmitter:
    """Decides whether a risk_level should be transmitted right now.

    Pure state machine: no serial, no clock of its own. `now` is passed in so the
    heartbeat is testable and the whole thing stays deterministic.
    """

    def __init__(self, target_id=0, heartbeat_s=HEARTBEAT_S, allow_untrusted=False):
        self.target_id = target_id
        self.heartbeat_s = heartbeat_s
        self.allow_untrusted = allow_untrusted
        self.last_level = None
        self.last_send_time = None

    def consider(self, computed_level, cane_gps_valid, now):
        trusted = self.allow_untrusted or _gps_trusted(cane_gps_valid)
        # An untrusted fix can still clear an alarm (send 0); it just cannot raise one.
        effective = computed_level if trusted else 0

        if self.last_level is None or effective != self.last_level:
            reason, should_send = "change", True
        elif self.last_send_time is None or now - self.last_send_time >= self.heartbeat_s:
            reason, should_send = "heartbeat", True
        else:
            reason, should_send = "hold", False

        if should_send:
            self.last_level = effective
            self.last_send_time = now

        return TxDecision(should_send, computed_level, effective, trusted, reason)

    def command(self, effective_level):
        """The verified downlink shape, compact and newline-free."""
        return json.dumps(
            {"target_id": self.target_id, "risk": effective_level},
            separators=(",", ":"),
        )


CSV_FIELDS = (
    "pc_time",
    "cane_seq",
    "computed_level",
    "effective_level",
    "trusted",
    "reason",
    "target_id",
    "distance_m",
    "closing_mps",
    "ttc_s",
    "risk_score",
    "cane_gps_valid",
    "cane_speed_mps",
    "cane_heading_deg",
    "veh_gps_valid",
    "veh_speed_mps",
    "veh_heading_deg",
)


def csv_row(now, store, transmitter, decision, assessment):
    cane = store.latest["cane"]
    vehicle = store.latest["vehicle"]
    return {
        "pc_time": round(now, 3),
        "cane_seq": store.latest["cane"]["seq"],
        "computed_level": decision.computed_level,
        "effective_level": decision.effective_level,
        "trusted": int(decision.trusted),
        "reason": decision.reason,
        "target_id": transmitter.target_id,
        # closing_mps: 양수 = 접근 중, 음수 = 멀어지는 중.
        "distance_m": round(assessment.distance_m, 2),
        "closing_mps": round(assessment.closing_los, 2),
        "ttc_s": round(assessment.ttc, 2),
        "risk_score": assessment.final_score,
        "cane_gps_valid": cane["gps_valid"],
        "cane_speed_mps": cane["speed_mps"],
        "cane_heading_deg": cane["heading_deg"],
        "veh_gps_valid": vehicle["gps_valid"],
        "veh_speed_mps": vehicle["speed_mps"],
        "veh_heading_deg": vehicle["heading_deg"],
    }


def append_row(csv_path, row):
    path = Path(csv_path)
    needs_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        if needs_header:
            writer.writeheader()
        writer.writerow(row)


def format_tx(decision):
    suppressed = (
        f" computed={decision.computed_level}->{decision.effective_level}"
        if decision.computed_level != decision.effective_level
        else ""
    )
    return (
        f"[TX] risk={decision.effective_level} reason={decision.reason}{suppressed}"
    )


class RiskSender:
    """Scores each record and pushes the resulting level through the transmitter."""

    def __init__(self, pipeline, transmitter, transport, csv_path, gate_params,
                 stabilizer=None):
        self.pipeline = pipeline
        self.transmitter = transmitter
        self.transport = transport
        self.csv_path = csv_path
        self.gate_params = gate_params
        self.stabilizer = stabilizer

    def process_line(self, raw_line, source_mode):
        line = raw_line.strip()
        if not line:
            return
        try:
            payload = json.loads(line)
            if not isinstance(payload, dict):
                raise ValueError("top-level JSON must be an object")
            row = normalize_record(payload, source_mode)
        except (json.JSONDecodeError, ValueError) as exc:
            print(f"[WARN] parse_failed error={exc} source={source_mode}", file=sys.stderr)
            return

        if row["type"] in RSU_ACK_TYPES:
            return

        if not self.pipeline.store.update(row):
            print(f"[WARN] ignored_type type={row['type']!r}", file=sys.stderr)
            return

        self.pipeline.observe(row)
        result = self.pipeline.compute()
        if result is None:
            return

        now, _raw, filtered = result
        store = self.pipeline.store
        vehicle_speed = to_float(store.latest["vehicle"]["speed_mps"])
        assessment = assess_risk(filtered, vehicle_speed, **self.gate_params)

        cane_gps_valid = store.latest["cane"]["gps_valid"]
        # Hysteresis (step8_stability): rises pass through instantly, drops must
        # persist hold_s. Applied before trust gating so an untrusted fix still
        # clears to 0 immediately inside the transmitter.
        level = assessment.risk_level
        if self.stabilizer is not None:
            level = self.stabilizer.stabilize(level, now)
        decision = self.transmitter.consider(level, cane_gps_valid, now)
        if not decision.should_send:
            return

        self.transport(self.transmitter.command(decision.effective_level))
        print(format_tx(decision), flush=True)
        append_row(
            self.csv_path,
            csv_row(now, store, self.transmitter, decision, assessment),
        )


def inject_vehicle(vehicle, sender, source_mode, now):
    cane = sender.pipeline.store.latest.get("cane")
    if cane is None or not has_position(cane) or not vehicle.is_due(now):
        return
    payload, distance_m = vehicle.record(
        to_float(cane["lat"]), to_float(cane["lng"]), now
    )
    print(f"[TESTVEH] seq={payload['seq']} distance_m={distance_m:.1f}", flush=True)
    sender.process_line(json.dumps(payload), source_mode)


def serial_transport(connection):
    def send(command):
        connection.write((command + "\n").encode("utf-8"))
        connection.flush()
    return send


def stdout_transport(command):
    # No hardware: show what would have gone on the wire.
    print(f"[WIRE] {command}", file=sys.stderr, flush=True)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Transmit step 7 risk_level to the RSU downlink."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial port")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate")
    parser.add_argument(
        "--source-mode",
        choices=SOURCE_MODES,
        default="test",
        help="origin of this input stream (default: test)",
    )
    parser.add_argument("--csv", default="step8_risk_tx_log.csv", help="output CSV path")
    parser.add_argument(
        "--fresh-window-ms",
        type=int,
        default=int(FRESH_WINDOW_S * 1000),
        help="how recent a record must be to count as READY (default: 500)",
    )
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="read JSON lines from stdin and print commands instead of using serial",
    )
    parser.add_argument(
        "--test-vehicle",
        action="store_true",
        help="inject a simulated vehicle closing in on the cane (step 5)",
    )
    parser.add_argument("--vehicle-speed", type=float, default=SPEED_MPS)
    parser.add_argument("--vehicle-start-m", type=float, default=START_DISTANCE_M)
    parser.add_argument("--target-id", type=int, default=0, help="downlink target (0=broadcast)")
    parser.add_argument(
        "--tx-heartbeat-s",
        type=float,
        default=HEARTBEAT_S,
        help=f"resend the same level at least this often (default: {HEARTBEAT_S})",
    )
    parser.add_argument(
        "--tx-untrusted",
        action="store_true",
        help="transmit risk even when the cane fix is a fallback (gps_valid=0)",
    )
    parser.add_argument(
        "--level-hold-s",
        type=float,
        default=HOLD_S,
        help=f"a lower level must persist this long before the alarm drops; "
        f"0 disables the hysteresis (default: {HOLD_S})",
    )
    parser.add_argument("--dcpa-near-m", type=float, default=DCPA_NEAR_M)
    parser.add_argument("--dcpa-far-m", type=float, default=DCPA_FAR_M)
    parser.add_argument("--dcpa-floor", type=float, default=DCPA_FLOOR)
    return parser.parse_args()


def main():
    args = parse_args()
    store = StateStore(fresh_window_s=args.fresh_window_ms / 1000)
    pipeline = KinematicsPipeline(store)
    transmitter = RiskTransmitter(
        target_id=args.target_id,
        heartbeat_s=args.tx_heartbeat_s,
        allow_untrusted=args.tx_untrusted,
    )
    gate_params = {
        "near_m": args.dcpa_near_m,
        "far_m": args.dcpa_far_m,
        "floor": args.dcpa_floor,
    }
    stabilizer = LevelStabilizer(hold_s=args.level_hold_s) if args.level_hold_s > 0 else None
    print(
        f"[INFO] source_mode={args.source_mode} csv={args.csv} target_id={args.target_id} "
        f"heartbeat_s={args.tx_heartbeat_s} tx_untrusted={args.tx_untrusted} "
        f"level_hold_s={args.level_hold_s}",
        file=sys.stderr,
    )

    vehicle = None
    if args.test_vehicle:
        vehicle = TestVehicle(
            start_distance_m=args.vehicle_start_m, speed_mps=args.vehicle_speed
        )
        print(
            f"[INFO] test_vehicle start_m={args.vehicle_start_m} speed={args.vehicle_speed}",
            file=sys.stderr,
        )

    if args.stdin:
        sender = RiskSender(
            pipeline, transmitter, stdout_transport, args.csv, gate_params,
            stabilizer=stabilizer,
        )
        _run(sender, sys.stdin, args.source_mode, vehicle)
        return

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required for serial mode: pip3 install pyserial") from exc

    with serial.Serial(args.port, args.baud, timeout=1) as connection:
        connection.reset_input_buffer()
        print(f"[INFO] port={args.port} baud={args.baud}", file=sys.stderr)
        sender = RiskSender(
            pipeline, transmitter, serial_transport(connection), args.csv, gate_params,
            stabilizer=stabilizer,
        )
        _run(sender, _serial_lines(connection), args.source_mode, vehicle)


def _serial_lines(connection):
    while True:
        raw = connection.readline()
        if raw:
            yield raw.decode("utf-8", errors="replace")


def _run(sender, lines, source_mode, vehicle):
    try:
        for line in lines:
            sender.process_line(line, source_mode)
            if vehicle is not None:
                inject_vehicle(vehicle, sender, source_mode, time.time())
    except KeyboardInterrupt:
        print("\n[INFO] stopped", file=sys.stderr)


if __name__ == "__main__":
    main()
