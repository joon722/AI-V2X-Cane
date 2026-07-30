#!/usr/bin/env python3
import argparse
import csv
import json
import time
from pathlib import Path

import serial


def parse_line(line):
    line = line.strip()
    if not line:
        return "empty", {}

    if line.startswith("{"):
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            return "bad_json", {"raw": line}
        return data.get("type", "json"), data

    if line.startswith("RISK,"):
        parts = line.split(",")
        data = {"raw": line}
        if len(parts) >= 2:
            data["risk"] = parts[1]
        if len(parts) >= 3:
            data["time"] = parts[2]
        if len(parts) >= 4:
            data["distance"] = parts[3]
        if len(parts) >= 5:
            data["ttc"] = parts[4]
        return "risk_text", data

    return "text", {"raw": line}


def main():
    parser = argparse.ArgumentParser(description="Monitor ESP32/RSU UART serial link.")
    parser.add_argument("port", help="Serial port, for example COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout-sec", type=float, default=2.0)
    parser.add_argument("--log", default="serial_link_monitor_log.csv")
    args = parser.parse_args()

    log_path = Path(args.log)
    print(f"[MONITOR] port={args.port}, baud={args.baud}")
    print(f"[LOG] {log_path.resolve()}")
    print("[OK 기준] vehicle/cane/json/RISK 메시지가 timeout 안에 계속 들어오면 연결 정상")
    print("Stop: Ctrl + C")

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    last_rx = None
    counts = {}

    with log_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["wall_time", "age_sec", "kind", "risk", "node_id", "raw_or_json"])

        try:
            while True:
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
                now = time.time()

                if raw:
                    last_rx = now
                    kind, data = parse_line(raw)
                    counts[kind] = counts.get(kind, 0) + 1

                    risk = data.get("risk", data.get("risk_level", ""))
                    node_id = data.get("node_id", data.get("src_id", ""))
                    payload = json.dumps(data, ensure_ascii=False) if kind != "text" else raw
                    writer.writerow([f"{now:.3f}", "0.000", kind, risk, node_id, payload])
                    f.flush()

                    if kind in ("vehicle", "cane", "risk_text", "json"):
                        print(f"[RX OK] kind={kind:9s} risk={str(risk):>3s} node={str(node_id):>4s} raw={raw}")
                    else:
                        print(f"[RX] kind={kind:9s} raw={raw}")
                    continue

                if last_rx is None:
                    print("[WAIT] 아직 수신 없음")
                    time.sleep(0.8)
                    continue

                age = now - last_rx
                if age > args.timeout_sec:
                    print(f"[TIMEOUT] {age:.1f}s 동안 수신 없음. 포트/baud/ESP32 전원/펌웨어 출력 확인")
                    writer.writerow([f"{now:.3f}", f"{age:.3f}", "timeout", "", "", ""])
                    f.flush()
                    time.sleep(0.8)

        except KeyboardInterrupt:
            print("\n[STOP]")
            print("[COUNT]", counts)
        finally:
            ser.close()


if __name__ == "__main__":
    main()
