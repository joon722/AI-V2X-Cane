# -*- coding: utf-8 -*-
"""젯슨 상주형 V2X UDP 텔레메트리 기록기.

지팡이(4210)/차량(4211)의 "이름:값" UDP 방송을 받아
~/telemetry_records/<시작시각>/ 아래에
지팡이_raw.log / 지팡이_값기록.csv / 차량_raw.log / 차량_값기록.csv 저장.
(logs/는 root 소유라 홈 아래를 쓴다)
(중선 듀얼시리얼기록 호환 포맷)

- 젯슨 Wi-Fi가 V2X-LOG에 붙어 있을 때만 패킷이 들어온다.
  안 붙어 있으면 조용히 대기하므로 항상 켜두면 된다.
- 파일은 첫 패킷이 올 때 만든다(빈 폴더/0바이트 방지).
- 두 번째 인스턴스는 포트 바인드 실패로 스스로 종료(중복 방지).
"""
import csv
import os
import selectors
import socket
import sys
from datetime import datetime

PORTS = {4210: "지팡이", 4211: "차량"}
BASE = os.path.expanduser("~/telemetry_records")


class NodeWriter:
    def __init__(self, name):
        self.name = name
        self.count = 0
        self.first_ts = None
        self.header = None
        self.raw = None
        self.csv_file = None
        self.csv = None

    def _open(self):
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        run_dir = os.path.join(BASE, RUN_STAMP)
        os.makedirs(run_dir, exist_ok=True)
        self.raw = open(
            os.path.join(run_dir, f"{self.name}_raw.log"),
            "a", encoding="utf-8-sig", newline="",
        )
        self.csv_file = open(
            os.path.join(run_dir, f"{self.name}_값기록.csv"),
            "a", encoding="utf-8-sig", newline="",
        )
        self.csv = csv.writer(self.csv_file)

    def handle(self, payload):
        now = datetime.now()
        if self.raw is None:
            self._open()
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
        if self.count % 10 == 0:
            self.raw.flush()
            self.csv_file.flush()


RUN_STAMP = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

def main():
    sel = selectors.DefaultSelector()
    writers = {}
    for port, name in PORTS.items():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # SO_REUSEADDR 없음: 두 번째 인스턴스는 여기서 죽는다(중복 방지).
        sock.bind(("", port))
        sock.setblocking(False)
        sel.register(sock, selectors.EVENT_READ, name)
        writers[name] = NodeWriter(name)

    print(f"[recorder] start {RUN_STAMP}, waiting on 4210/4211", flush=True)
    while True:
        for key, _ in sel.select(timeout=5.0):
            try:
                payload, _addr = key.fileobj.recvfrom(4096)
            except OSError:
                continue
            writers[key.data].handle(payload)


if __name__ == "__main__":
    main()
