#!/usr/bin/env python3
import argparse
import csv
import json
import time
from pathlib import Path

import serial


FIELDS = [
    "wall_time",
    "elapsed_s",
    "type",
    "node_id",
    "risk_level",
    "lat",
    "lng",
    "speed_mps",
    "heading_deg",
    "seq",
    "raw",
]


def parse_message(raw):
    raw = raw.replace("\x00", "").strip()
    row = {field: "" for field in FIELDS}
    row["raw"] = raw

    if not raw:
        row["type"] = "empty"
        return row

    if raw.startswith("{"):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            row["type"] = "bad_json"
            return row

        row["type"] = data.get("type", "json")
        row["node_id"] = data.get("node_id", data.get("src_id", ""))
        row["risk_level"] = data.get("risk_level", data.get("risk", ""))
        row["lat"] = data.get("lat", data.get("latitude", ""))
        row["lng"] = data.get("lng", data.get("longitude", ""))
        row["speed_mps"] = data.get("speed_mps", "")
        row["heading_deg"] = data.get("heading_deg", "")
        row["seq"] = data.get("seq", data.get("seq_num", ""))
        return row

    if raw.startswith("RISK,"):
        parts = raw.split(",")
        row["type"] = "risk_text"
        if len(parts) > 1:
            row["risk_level"] = parts[1]
        if len(parts) > 2:
            row["seq"] = parts[2]
        return row

    row["type"] = "text"
    return row


def main():
    parser = argparse.ArgumentParser(description="Log V2X serial messages to CSV.")
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyUSB0 or COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--out", default="v2x_rx_log.csv")
    parser.add_argument("--timeout", type=float, default=0.2)
    args = parser.parse_args()

    out_path = Path(args.out)
    start = time.time()
    counts = {}

    print(f"[LOGGER] {args.port} @ {args.baud}")
    print(f"[CSV] {out_path.resolve()}")
    print("Stop: Ctrl + C")

    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)

    try:
        with out_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=FIELDS,
                quoting=csv.QUOTE_ALL,
                escapechar="\\",
                doublequote=True,
            )
            writer.writeheader()

            while True:
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
                if not raw:
                    continue

                now = time.time()
                row = parse_message(raw)
                row["wall_time"] = f"{now:.3f}"
                row["elapsed_s"] = f"{now - start:.3f}"
                writer.writerow(row)
                f.flush()

                msg_type = row["type"]
                counts[msg_type] = counts.get(msg_type, 0) + 1

                if msg_type in ("vehicle", "cane"):
                    print(
                        f"[LOG] type={msg_type:7s} node={row['node_id']} "
                        f"risk={row['risk_level']} lat={row['lat']} lng={row['lng']} seq={row['seq']}"
                    )
                else:
                    print(f"[LOG] type={msg_type:9s} raw={raw!r}")

    except KeyboardInterrupt:
        print("\n[STOP]")
        print(f"[SAVED] {out_path.resolve()}")
        print(f"[COUNT] {counts}")
    finally:
        ser.close()


if __name__ == "__main__":
    main()