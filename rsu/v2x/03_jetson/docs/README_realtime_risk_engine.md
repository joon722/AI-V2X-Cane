#!/usr/bin/env python3
import argparse
import csv
import json
import math
import time
from pathlib import Path

import serial


DISTANCE_KEYS = ("distance_m", "distance", "dist_m", "dist", "range_m")
TTC_KEYS = ("ttc_s", "ttc", "TTC")
RISK_KEYS = ("risk", "node_risk", "risk_level")


def as_float(value):
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def as_int(value):
    number = as_float(value)
    if number is None:
        return None
    return int(number)


def first_number(data, keys):
    for key in keys:
        if key in data:
            number = as_float(data.get(key))
            if number is not None:
                return number
    return None


def compute_risk(distance_m, ttc_s):
    if (
        (distance_m is not None and distance_m < 3.0)
        or (ttc_s is not None and ttc_s < 2.0)
    ):
        return 3, "distance_lt_3m_or_ttc_lt_2s"
    if (
        (distance_m is not None and distance_m < 6.0)
        or (ttc_s is not None and ttc_s < 4.0)
    ):
        return 2, "distance_lt_6m_or_ttc_lt_4s"
    if (
        (distance_m is not None and distance_m < 12.0)
        or (ttc_s is not None and ttc_s < 6.0)
    ):
        return 1, "distance_lt_12m_or_ttc_lt_6s"
    return 0, "safe_distance_and_ttc"


def parse_json_line(raw):
    text = raw.strip()
    if not text.startswith("{"):
        return None, "text"
    try:
        return json.loads(text), None
    except json.JSONDecodeError:
        return None, "bad_json"


def extract_serial_messages(buffer):
    messages = []
    buffer = buffer.replace("\r", "\n")

    while buffer:
        first_brace = buffer.find("{")
        first_newline = buffer.find("\n")

        if first_newline != -1 and (first_brace == -1 or first_newline < first_brace):
            line = buffer[:first_newline].strip()
            buffer = buffer[first_newline + 1 :]
            if line:
                messages.append(line)
            continue

        if first_brace > 0:
            buffer = buffer[first_brace:]
            continue

        if not buffer.startswith("{"):
            if len(buffer) > 1024:
                buffer = buffer[-128:]
            break

        depth = 0
        in_string = False
        escaped = False
        end_index = None

        for index, char in enumerate(buffer):
            if escaped:
                escaped = False
                continue
            if char == "\\":
                escaped = True
                continue
            if char == '"':
                in_string = not in_string
                continue
            if in_string:
                continue
            if char == "{":
                depth += 1
            elif char == "}":
                depth -= 1
                if depth == 0:
                    end_index = index + 1
                    break

        if end_index is None:
            if len(buffer) > 4096:
                buffer = buffer[-1024:]
            break

        messages.append(buffer[:end_index].strip())
        buffer = buffer[end_index:]

    return messages, buffer


def normalize_message(data):
    msg_type = str(data.get("type", "json"))
    distance_m = first_number(data, DISTANCE_KEYS)
    ttc_s = first_number(data, TTC_KEYS)
    node_risk = as_int(data.get("node_risk"))
    if node_risk is None:
        node_risk = as_int(data.get("risk_level"))
    if node_risk is None:
        node_risk = as_int(data.get("risk"))

    computed_risk, reason = compute_risk(distance_m, ttc_s)
    seq = data.get("seq", data.get("sequence", ""))
    rssi = data.get("rssi", data.get("RSSI", ""))

    return {
        "type": msg_type,
        "seq": seq,
        "rssi": rssi,
        "distance_m": distance_m,
        "ttc_s": ttc_s,
        "node_risk": node_risk,
        "computed_risk": computed_risk,
        "reason": reason,
        "raw": json.dumps(data, ensure_ascii=False),
    }


def fmt_optional(value, digits=2):
    if value is None:
        return "-"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def main():
    parser = argparse.ArgumentParser(
        description="Read V2X serial JSON, compute distance/TTC risk, and save CSV."
    )
    parser.add_argument("port", help="Serial port, for example /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.2)
    parser.add_argument("--ok-timeout", type=float, default=3.0)
    parser.add_argument("--log", default="final_risk_log.csv")
    args = parser.parse_args()

    log_path = Path(args.log)
    last_rx = None
    latest_vehicle = None
    latest_cane = None

    print(f"[RISK_ENGINE] port={args.port}, baud={args.baud}")
    print(f"[LOG] {log_path.resolve()}")
    print("[RULE] risk3: distance<3m or TTC<2s")
    print("[RULE] risk2: distance<6m or TTC<4s")
    print("[RULE] risk1: distance<12m or TTC<6s")
    print("[RULE] risk0: otherwise")
    print("Stop: Ctrl + C")

    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)

    try:
        with log_path.open("w", newline="", encoding="utf-8") as f:
            fieldnames = [
                "wall_time",
                "elapsed_s",
                "gap_s",
                "type",
                "seq",
                "rssi",
                "distance_m",
                "ttc_s",
                "node_risk",
                "computed_risk",
                "vehicle_risk",
                "cane_risk",
                "final_risk",
                "reason",
                "raw",
            ]
            writer = csv.DictWriter(
                f,
                fieldnames=fieldnames,
                quoting=csv.QUOTE_MINIMAL,
                escapechar="\\",
                doublequote=True,
            )
            writer.writeheader()
            start = time.time()
            serial_buffer = ""

            while True:
                now = time.time()
                waiting = ser.in_waiting if ser.in_waiting > 0 else 1
                chunk = ser.read(waiting).decode("utf-8", errors="replace")

                if not chunk:
                    if last_rx is not None and now - last_rx > args.ok_timeout:
                        print(f"[WAIT] no data for {now - last_rx:.1f}s")
                        last_rx = now
                    continue

                serial_buffer += chunk
                messages, serial_buffer = extract_serial_messages(serial_buffer)
                if not messages:
                    continue

                for raw in messages:
                    gap = "" if last_rx is None else f"{now - last_rx:.3f}"
                    last_rx = now

                    data, parse_error = parse_json_line(raw)
                    if parse_error is not None:
                        writer.writerow(
                            {
                                "wall_time": f"{now:.3f}",
                                "elapsed_s": f"{now - start:.3f}",
                                "gap_s": gap,
                                "type": parse_error,
                                "raw": raw,
                            }
                        )
                        f.flush()
                        print(
                            f"[RX] type={parse_error:8s} gap={gap or '-':>6s} raw={raw}"
                        )
                        continue

                    msg = normalize_message(data)
                    if msg["type"] == "vehicle":
                        latest_vehicle = msg
                    elif msg["type"] == "cane":
                        latest_cane = msg

                    vehicle_risk = (
                        latest_vehicle["computed_risk"]
                        if latest_vehicle is not None
                        else None
                    )
                    cane_risk = (
                        latest_cane["computed_risk"] if latest_cane is not None else None
                    )
                    known_risks = [
                        risk
                        for risk in (vehicle_risk, cane_risk, msg["computed_risk"])
                        if risk is not None
                    ]
                    final_risk = max(known_risks) if known_risks else msg["computed_risk"]

                    writer.writerow(
                        {
                            "wall_time": f"{now:.3f}",
                            "elapsed_s": f"{now - start:.3f}",
                            "gap_s": gap,
                            "type": msg["type"],
                            "seq": msg["seq"],
                            "rssi": msg["rssi"],
                            "distance_m": fmt_optional(msg["distance_m"]),
                            "ttc_s": fmt_optional(msg["ttc_s"]),
                            "node_risk": fmt_optional(msg["node_risk"], 0),
                            "computed_risk": msg["computed_risk"],
                            "vehicle_risk": fmt_optional(vehicle_risk, 0),
                            "cane_risk": fmt_optional(cane_risk, 0),
                            "final_risk": final_risk,
                            "reason": msg["reason"],
                            "raw": msg["raw"],
                        }
                    )
                    f.flush()

                    print(
                        "[RX] "
                        f"type={msg['type']:8s} seq={fmt_optional(msg['seq'], 0):>4s} "
                        f"dist={fmt_optional(msg['distance_m']):>5s}m "
                        f"ttc={fmt_optional(msg['ttc_s']):>5s}s "
                        f"risk={msg['computed_risk']} rssi={fmt_optional(msg['rssi'], 0)}"
                    )
                    print(
                        "[RISK] "
                        f"vehicle={fmt_optional(vehicle_risk, 0)} "
                        f"cane={fmt_optional(cane_risk, 0)} "
                        f"final={final_risk} reason={msg['reason']}"
                    )

    except KeyboardInterrupt:
        print("\n[STOP]")
        print(f"[SAVED] {log_path.resolve()}")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
