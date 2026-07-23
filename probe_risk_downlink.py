#!/usr/bin/env python3
"""Check whether the RSU accepts a risk downlink and the cane reflects it.

This answers one question before step 5 is worth building: can the Jetson push a
risk value back down to the cane at all? It alternates a risk value on the serial
port and reports whether the cane's node_risk follows.

Nothing else in the pipeline depends on this file. It is a one-off experiment.
"""

import argparse
import json
import sys
import time

from step3_parse_v2x import normalize_record


# Start at 2. The cane rests at 0, so sending 0 first proves nothing and only
# delays the answer by one period.
RISK_CYCLE = (2, 0)


def build_command(target_id, risk):
    """The downlink shape named in v2x_session_summary_next_steps.md."""
    return json.dumps({"target_id": target_id, "risk": risk}, separators=(",", ":"))


def verdict(sent_values, observed_values, raw_lines=0):
    """Decide what the observed node_risk values say about the downlink."""
    observed = set(observed_values)
    if not observed:
        if raw_lines:
            return "NO_PARSE", f"{raw_lines}줄을 받았지만 지팡이 레코드로 못 읽었습니다."
        return "NO_DATA", "시리얼에서 바이트가 한 개도 안 왔습니다."
    if len(observed) == 1:
        return "NO_CHANGE", f"node_risk가 {observed_values[0]} 에서 전혀 변하지 않았습니다."
    # 0 is the resting value, so only a non-zero echo proves the command landed.
    if observed & (set(sent_values) - {0}):
        return "RESPONDS", "보낸 risk 값이 node_risk에 나타났습니다."
    return "CHANGED_OTHER", "node_risk가 변하긴 했는데 보낸 값과 다릅니다."


def parse_args():
    parser = argparse.ArgumentParser(
        description="Probe the risk downlink path from Jetson to the cane."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial port")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate")
    parser.add_argument(
        "--target-id",
        type=int,
        default=0,
        help="target node id in the command (default: 0)",
    )
    parser.add_argument(
        "--period", type=float, default=3.0, help="seconds between commands"
    )
    parser.add_argument(
        "--duration", type=float, default=20.0, help="total run time in seconds"
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="print a record only when node_risk changes",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        import serial
    except ImportError:
        raise SystemExit("pyserial is required: pip3 install pyserial")

    sent_values = []
    observed_values = []
    unparsed = []
    raw_lines = 0
    started = time.monotonic()
    next_send = started
    cycle_index = 0

    # timeout=1 matches step3. A shorter timeout truncates lines mid-frame and
    # every fragment then fails to parse, which looks identical to no data.
    with serial.Serial(args.port, args.baud, timeout=1) as connection:
        connection.reset_input_buffer()
        print(f"[INFO] port={args.port} target_id={args.target_id}", file=sys.stderr)
        print(f"[INFO] {args.duration}초 동안 관찰합니다.", file=sys.stderr)

        try:
            while time.monotonic() - started < args.duration:
                now = time.monotonic()
                if now >= next_send:
                    risk = RISK_CYCLE[cycle_index % len(RISK_CYCLE)]
                    command = build_command(args.target_id, risk)
                    connection.write((command + "\n").encode("utf-8"))
                    connection.flush()
                    sent_values.append(risk)
                    print(f"[SEND] {command}", flush=True)
                    cycle_index += 1
                    next_send = now + args.period

                raw = connection.readline()
                if not raw:
                    continue
                raw_lines += 1
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                try:
                    payload = json.loads(line)
                    if not isinstance(payload, dict):
                        raise ValueError("top-level JSON must be an object")
                    row = normalize_record(payload, "fallback")
                except (json.JSONDecodeError, ValueError) as exc:
                    if len(unparsed) < 5:
                        unparsed.append(f"{exc} | {line[:80]!r}")
                    continue
                if row["type"] != "cane":
                    if len(unparsed) < 5:
                        unparsed.append(f"type={row['type']!r} (cane 아님)")
                    continue
                previous = observed_values[-1] if observed_values else None
                observed_values.append(row["node_risk"])
                if not args.quiet or row["node_risk"] != previous:
                    print(
                        f"[RECV] seq={row['seq']} node_risk={row['node_risk']}",
                        flush=True,
                    )
        except KeyboardInterrupt:
            # Still report what was collected; an interrupted run is often the
            # only run the user has patience for.
            print("\n[INFO] 중단됨", file=sys.stderr)

    code, message = verdict(sent_values, observed_values, raw_lines)
    print("", file=sys.stderr)
    print(f"[RESULT] {code}: {message}", file=sys.stderr)
    print(f"[RESULT] 시리얼에서 읽은 줄 수: {raw_lines}", file=sys.stderr)
    print(f"[RESULT] 지팡이 레코드 수: {len(observed_values)}", file=sys.stderr)
    print(f"[RESULT] 보낸 risk 값: {sorted(set(sent_values))}", file=sys.stderr)
    print(
        f"[RESULT] 관찰된 node_risk 값: {sorted(set(observed_values))}",
        file=sys.stderr,
    )
    for sample in unparsed:
        print(f"[RESULT] 못 읽은 줄 예: {sample}", file=sys.stderr)


if __name__ == "__main__":
    main()
