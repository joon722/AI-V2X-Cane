#!/usr/bin/env python3
"""V2X UDP 텔레메트리 수집기.

지팡이(4210)와 차량(4211)이 V2X-LOG Wi-Fi로 1초마다 방송하는
"이름:값" 텔레메트리(가속도, 자이로, RSSI, 원시 GPS 등)를 CSV로 저장한다.

사용법 (V2X-LOG Wi-Fi에 접속한 PC에서):
    python udp_telemetry_logger.py --dir 저장폴더

종료: Ctrl+C
결과: udp_cane_날짜시간.csv / udp_vehicle_날짜시간.csv
(컬럼은 첫 패킷의 항목으로 자동 결정되므로 펌웨어가 항목을 늘려도 코드는 그대로.)
"""
import argparse
import csv
import select
import socket
import time
from pathlib import Path

PORTS = {4210: "cane", 4211: "vehicle"}


def parse_packet(text):
    data = {}
    for line in text.splitlines():
        if ":" in line:
            key, _, value = line.partition(":")
            key = key.strip()
            if key:
                data[key] = value.strip()
    return data


def main():
    parser = argparse.ArgumentParser(description="V2X UDP telemetry -> CSV")
    parser.add_argument("--dir", default=".", help="CSV 저장 폴더")
    args = parser.parse_args()

    out_dir = Path(args.dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")

    sockets = {}
    for port in PORTS:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("", port))
        sockets[s] = port

    print("[대기] 지팡이=4210, 차량=4211 포트에서 수신 대기 중...")
    print("[주의] 이 PC가 V2X-LOG Wi-Fi에 접속되어 있어야 합니다. 종료는 Ctrl+C")

    writers = {}  # port -> (file, DictWriter, path)
    counts = {port: 0 for port in PORTS}
    last_status = time.time()

    try:
        while True:
            ready, _, _ = select.select(list(sockets), [], [], 1.0)
            now = time.time()

            for s in ready:
                payload, addr = s.recvfrom(4096)
                port = sockets[s]
                data = parse_packet(payload.decode("utf-8", errors="replace"))
                if not data:
                    continue

                row = {"pc_time": f"{now:.3f}", "src_ip": addr[0]}
                row.update(data)

                if port not in writers:
                    path = out_dir / f"udp_{PORTS[port]}_{stamp}.csv"
                    # utf-8-sig: 엑셀에서 한글 컬럼명이 바로 열리게 함.
                    handle = path.open("w", newline="", encoding="utf-8-sig")
                    writer = csv.DictWriter(
                        handle,
                        fieldnames=list(row.keys()),
                        extrasaction="ignore",
                    )
                    writer.writeheader()
                    writers[port] = (handle, writer, path)
                    print(f"[시작] {PORTS[port]} -> {path}")

                handle, writer, _ = writers[port]
                writer.writerow({k: row.get(k, "") for k in writer.fieldnames})
                handle.flush()
                counts[port] += 1

            if now - last_status >= 5.0:
                last_status = now
                summary = ", ".join(
                    f"{name} {counts[port]}건" for port, name in PORTS.items()
                )
                print(f"[수신 현황] {summary}")

    except KeyboardInterrupt:
        print("\n[종료]")
    finally:
        for handle, _, path in writers.values():
            handle.close()
            print(f"[저장됨] {path}")


if __name__ == "__main__":
    main()
