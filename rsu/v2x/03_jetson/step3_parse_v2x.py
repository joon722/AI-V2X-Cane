#!/usr/bin/env python3
"""Parse newline-delimited V2X JSON, label its source, and append it to CSV."""

import argparse
import csv
import json
import sys
import time
from pathlib import Path


SOURCE_MODES = ("real", "test", "fallback", "simulation")
CSV_FIELDS = (
    "pc_time",
    "type",
    "node_id",
    "seq",
    "gps_valid",
    "lat",
    "lng",
    "speed_mps",
    "heading_deg",
    "heading_valid",
    "hdop",
    "gps_time_ms",
    "node_risk",
    "tx_ms",
    "rx_ms",
    "recv_count",
    "lost_count",
    "rssi",
    # 차량이 지팡이 신호를 직접 들은 RSSI로 추정한 거리(m). 프로토콜 v4에서 상태
    # 패킷에 추가되면 브리지가 실어 보낸다. 구 펌웨어면 빈 값 → 방향 융합 자동 OFF.
    "rssi_dist",
    # DWM3001 UWB 측거. 차량이 MSG_UWB_RANGE(38B)로 브로드캐스트한 것을 브리지가
    # type="uwb" 행으로 전달한다. rssi_dist와 채널을 분리한 이유: RSSI 융합 게이트는
    # RSSI 튜닝이라 UWB가 같은 필드로 오면 출처 구분이 안 된다. cane/vehicle 행에선 빈 값.
    "uwb_dist",
    "uwb_raw",
    "uwb_closing",
    "uwb_calib",
    "src_mac",
    "source_mode",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Parse V2X JSON and retain its source_mode in logs and CSV."
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
        "--csv", default="step3_v2x_parsed_log.csv", help="output CSV path"
    )
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="read JSON lines from stdin instead of a serial port",
    )
    return parser.parse_args()


def normalize_record(payload, cli_source_mode, now=None):
    """Return a stable CSV row and reject unknown embedded source modes."""
    embedded_mode = payload.get("source_mode")
    if embedded_mode is not None and embedded_mode not in SOURCE_MODES:
        raise ValueError(f"invalid source_mode: {embedded_mode!r}")

    # An explicit mode attached by the producer is more specific than the
    # receiver's stream-level default.
    source_mode = embedded_mode or cli_source_mode
    row = {field: payload.get(field, "") for field in CSV_FIELDS}
    row["pc_time"] = time.time() if now is None else now
    row["source_mode"] = source_mode
    return row


def append_row(csv_path, row):
    path = Path(csv_path)
    needs_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        if needs_header:
            writer.writeheader()
        writer.writerow(row)


def process_line(raw_line, source_mode, csv_path):
    line = raw_line.strip()
    if not line:
        return False
    try:
        payload = json.loads(line)
        if not isinstance(payload, dict):
            raise ValueError("top-level JSON must be an object")
        row = normalize_record(payload, source_mode)
        append_row(csv_path, row)
    except (json.JSONDecodeError, ValueError) as exc:
        print(f"[WARN] parse_failed error={exc} source={source_mode}", file=sys.stderr)
        return False

    print(
        f"[STATE] type={row['type']} seq={row['seq']} "
        f"node_risk={row['node_risk']} gps_valid={row['gps_valid']} "
        f"source={row['source_mode']}",
        flush=True,
    )
    return True


def serial_lines(port, baud):
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required for serial input: pip install pyserial") from exc

    with serial.Serial(port, baud, timeout=1) as connection:
        # Bytes buffered while nothing was reading the port are usually a partial
        # frame or boot noise, and decode into replacement chars that fail to parse.
        connection.reset_input_buffer()
        print(f"[INFO] port={port} baud={baud}", file=sys.stderr)
        while True:
            raw = connection.readline()
            if raw:
                yield raw.decode("utf-8", errors="replace")


def main():
    args = parse_args()
    print(f"[INFO] source_mode={args.source_mode} csv={args.csv}", file=sys.stderr)
    lines = sys.stdin if args.stdin else serial_lines(args.port, args.baud)
    try:
        for line in lines:
            process_line(line, args.source_mode, args.csv)
    except KeyboardInterrupt:
        print("\n[INFO] stopped", file=sys.stderr)


if __name__ == "__main__":
    main()
