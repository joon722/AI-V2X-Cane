#!/usr/bin/env python3
import csv
import json
import time
import serial

PORT = "/dev/ttyUSB0"
BAUD = 115200
OUT_CSV = "step2_cane_parsed_log.csv"

FIELDS = [
    "pc_time",
    "type",
    "node_id",
    "seq",
    "gps_valid",
    "lat",
    "lng",
    "speed_mps",
    "heading_deg",
    "node_risk",
    "tx_ms",
    "rx_ms",
    "recv_count",
    "lost_count",
    "rssi",
    "src_mac",
    "source_mode",
]

def get_int(data, *names, default=0):
    for name in names:
        if name in data and data[name] != "":
            return int(data[name])
    return default

def get_float(data, *names, default=0.0):
    for name in names:
        if name in data and data[name] != "":
            return float(data[name])
    return default

def decide_source_mode(data):
    gps_valid = get_int(data, "gps_valid", default=0)
    lat = get_float(data, "lat", "latitude", default=0.0)
    lng = get_float(data, "lng", "longitude", default=0.0)

    # 지금 37.0 / 127.0은 테스트용 고정 좌표로 보고 표시
    if lat == 37.0 and lng == 127.0:
        return "test"

    if gps_valid == 1:
        return "real"

    return "fallback"

def parse_packet(line):
    data = json.loads(line)

    node_type = data.get("type", "unknown")
    node_risk = get_int(data, "node_risk", "risk", "risk_level", default=0)

    row = {
        "pc_time": f"{time.time():.3f}",
        "type": node_type,
        "node_id": get_int(data, "node_id", default=0),
        "seq": get_int(data, "seq", "seq_num", default=0),
        "gps_valid": get_int(data, "gps_valid", default=0),
        "lat": get_float(data, "lat", "latitude", default=0.0),
        "lng": get_float(data, "lng", "longitude", default=0.0),
        "speed_mps": get_float(data, "speed_mps", default=0.0),
        "heading_deg": get_float(data, "heading_deg", default=0.0),
        "node_risk": node_risk,
        "tx_ms": get_int(data, "tx_ms", default=0),
        "rx_ms": get_int(data, "rx_ms", default=0),
        "recv_count": get_int(data, "recv_count", default=0),
        "lost_count": get_int(data, "lost_count", default=0),
        "rssi": get_int(data, "rssi", default=0),
        "src_mac": data.get("src_mac", ""),
        "source_mode": decide_source_mode(data),
    }

    return row

def main():
    print(f"[STEP2] Open serial {PORT} @ {BAUD}")
    print(f"[CSV] {OUT_CSV}")
    print("Stop: Ctrl + C")

    ser = serial.Serial(PORT, BAUD, timeout=0.2)

    with open(OUT_CSV, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()

        try:
            while True:
                line = ser.readline().decode("utf-8", errors="ignore").strip()

                if not line:
                    continue

                if not line.startswith("{") or not line.endswith("}"):
                    print("[DROP] incomplete:", line[:120])
                    continue

                try:
                    row = parse_packet(line)
                except Exception as e:
                    print("[JSON FAIL]", e, line[:120])
                    continue

                writer.writerow(row)
                f.flush()

                print(
                    f"[RX] type={row['type']} "
                    f"seq={row['seq']} "
                    f"node_risk={row['node_risk']} "
                    f"gps_valid={row['gps_valid']} "
                    f"source={row['source_mode']}"
                )

        except KeyboardInterrupt:
            print("\n[STOP]")
            print(f"[SAVED] {OUT_CSV}")

    ser.close()

if __name__ == "__main__":
    main()