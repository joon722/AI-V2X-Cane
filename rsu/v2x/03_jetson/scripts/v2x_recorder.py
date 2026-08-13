# -*- coding: utf-8 -*-
"""V2X 노드 UDP 텔레메트리 기록기 (중선 듀얼시리얼기록 호환 포맷).

지팡이(4210)/차량(4211)이 V2X-LOG Wi-Fi로 뿌리는 "이름:값" UDP 패킷을 받아
런 폴더에 지팡이_raw.log / 지팡이_값기록.csv / 차량_raw.log / 차량_값기록.csv 저장.

사용법 (노트북을 V2X-LOG Wi-Fi에 연결한 뒤):
    python v2x_recorder.py 평행1p5m_젯슨연동
    (Ctrl+C 로 종료 = 런 1개 끝. 다음 런은 다시 실행)

첫 실행 때 Windows 방화벽 허용 창이 뜨면 반드시 "액세스 허용".
"""
import csv
import os
import selectors
import socket
import sys
from datetime import datetime

PORTS = {4210: "지팡이", 4211: "차량"}
BASE_DIR = os.path.join(
    os.path.expanduser("~"),
    "OneDrive", "바탕 화면", "기록들",
    "records_" + datetime.now().strftime("%y%m%d"),
)


class NodeWriter:
    def __init__(self, run_dir, name):
        self.name = name
        self.count = 0
        self.first_ts = None
        self.header = None
        self.raw = open(
            os.path.join(run_dir, f"{name}_raw.log"),
            "w", encoding="utf-8-sig", newline="",
        )
        self.csv_file = open(
            os.path.join(run_dir, f"{name}_값기록.csv"),
            "w", encoding="utf-8-sig", newline="",
        )
        self.csv = csv.writer(self.csv_file)

    def handle(self, payload: bytes):
        now = datetime.now()
        hms = now.strftime("%H:%M:%S")
        if self.first_ts is None:
            self.first_ts = now
        elapsed = (now - self.first_ts).total_seconds()

        text = payload.decode("utf-8", errors="replace")
        fields = {}
        for line in text.splitlines():
            line = line.strip()
            if not line:
                continue
            self.raw.write(f"{hms}  {line}\n")
            if ":" in line:
                key, value = line.split(":", 1)
                fields[key] = value

        if not fields:
            return
        if self.header is None:
            self.header = list(fields.keys())
            self.csv.writerow(["시각", "경과초"] + self.header)
        self.csv.writerow(
            [hms, f"{elapsed:.3f}"] +
            [fields.get(k, "") for k in self.header]
        )
        self.count += 1
        self.raw.flush()
        self.csv_file.flush()

    def close(self):
        self.raw.close()
        self.csv_file.close()


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else "run"
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    run_dir = os.path.join(BASE_DIR, f"{label}_{stamp}")
    os.makedirs(run_dir, exist_ok=True)
    print(f"[기록기] 저장 폴더: {run_dir}")
    print("[기록기] Ctrl+C 로 종료 (런 1개 끝)")

    sel = selectors.DefaultSelector()
    writers = {}
    for port, name in PORTS.items():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", port))
        sock.setblocking(False)
        sel.register(sock, selectors.EVENT_READ, name)
        writers[name] = NodeWriter(run_dir, name)

    last_status = datetime.now()
    try:
        while True:
            for key, _ in sel.select(timeout=1.0):
                try:
                    payload, _addr = key.fileobj.recvfrom(4096)
                except OSError:
                    continue
                writers[key.data].handle(payload)

            now = datetime.now()
            if (now - last_status).total_seconds() >= 2.0:
                last_status = now
                status = "  ".join(
                    f"{name} {w.count}건" for name, w in writers.items()
                )
                print(f"\r[수신] {status}   ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        for name, w in writers.items():
            w.close()
            print(f"[종료] {name}: {w.count}건 저장")
        print(f"[종료] 폴더: {run_dir}")


if __name__ == "__main__":
    main()
