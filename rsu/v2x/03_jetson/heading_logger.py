#!/usr/bin/env python3
"""방향성 실험용 기록기.

브리지 원본 JSON에서 지팡이/차량의 위치·속도·방향(heading)을
위험 계산 조건 없이 그대로 CSV에 기록하고, 1초마다 화면에도 보여준다.
(step8 엔진과 포트를 공유할 수 없으므로 실행 전에 엔진을 stop 할 것.)

사용:
    sudo systemctl stop v2x-risk-engine
    python3 heading_logger.py --csv logs/exp_방향성.csv
    (실험 후 Ctrl+C)
    sudo systemctl start v2x-risk-engine
"""
import argparse
import csv
import json
import time

import serial

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
    "rssi",
]


def main():
    parser = argparse.ArgumentParser(description="heading/speed raw logger")
    parser.add_argument("--port", default="/dev/v2x-rsu")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--csv", default="heading_log.csv")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)
    ser.reset_input_buffer()

    counts = {"cane": 0, "vehicle": 0}
    latest = {}
    last_print = 0.0

    with open(args.csv, "a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS, extrasaction="ignore")
        if handle.tell() == 0:
            writer.writeheader()

        print(f"[기록 시작] {args.csv}")
        print("[안내] 방향 기준: 북=0도, 동=90도, 남=180도, 서=270도. 종료는 Ctrl+C")

        prev_counts = dict(counts)

        try:
            while True:
                raw = ser.readline().decode("utf-8", errors="replace").strip()

                # 패킷이 없어도 1초마다 반드시 상태를 출력한다.
                now = time.time()
                if now - last_print >= 1.0:
                    last_print = now
                    handle.flush()
                    parts = []
                    for name in ("cane", "vehicle"):
                        d = latest.get(name)
                        rate = counts[name] - prev_counts[name]
                        if d:
                            parts.append(
                                f"{name}: 방향={d.get('heading_deg')}도 "
                                f"속도={d.get('speed_mps')}m/s "
                                f"gps={d.get('gps_valid')} "
                                f"수신={rate}건/초"
                            )
                        else:
                            parts.append(f"{name}: 수신 없음")
                    prev_counts = dict(counts)
                    print(" | ".join(parts))

                if not raw.startswith("{"):
                    continue
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    continue

                node_type = data.get("type")
                if node_type not in counts:
                    continue

                data["pc_time"] = f"{time.time():.3f}"
                writer.writerow({k: data.get(k, "") for k in FIELDS})
                counts[node_type] += 1
                latest[node_type] = data

        except KeyboardInterrupt:
            print(
                f"\n[종료] cane {counts['cane']}건, "
                f"vehicle {counts['vehicle']}건 -> {args.csv}"
            )
        finally:
            handle.flush()
            ser.close()


if __name__ == "__main__":
    main()
