#!/usr/bin/env python3
"""raw 로그를 요약해 링크 상태를 몇 초마다 한 줄로 보여준다.

raw 로그는 초당 20줄씩 흘러서 tail -f 로는 "지금 실험할 수 있는 상태인가"를
판단하기 어렵다. 이 도구는 그 판단에 필요한 것만 본다:
지팡이와 차량이 둘 다 들어오는가, GPS는 잡혔는가, 신호 세기는 쓸 만한가.

사용법 (젯슨에서):
    python3 ~/v2x/03_jetson/scripts/watch_link.py

종료: Ctrl+C
"""

import argparse
import json
import time
from pathlib import Path


# 창이 길수록 건수는 안정적이지만 화면이 그만큼 늦게 바뀐다. 이 도구는
# "지금 노드가 살아있나"를 보는 용도라 반응 속도를 택했다.
WINDOW_S = 2.0
INTERVAL_S = 1.0
# ESP-NOW는 -90 아래로 내려가면 유실이 급격히 늘어난다.
RSSI_WEAK = -85


def latest_log(log_dir):
    files = sorted(Path(log_dir).glob("raw_*.log"), key=lambda p: p.stat().st_mtime)
    return files[-1] if files else None


def tail_lines(path, count):
    """파일 끝부분만 읽는다. 실험이 길어지면 로그가 수십 MB가 되기 때문."""
    with path.open("rb") as handle:
        handle.seek(0, 2)
        size = handle.tell()
        block = min(size, count * 300)
        handle.seek(size - block)
        text = handle.read().decode("utf-8", errors="replace")
    return text.splitlines()[-count:]


def summarize(lines, now, window_s):
    """최근 window_s 초 안의 줄만 종류별로 모은다."""
    nodes = {"cane": [], "vehicle": []}
    tx = []
    for line in lines:
        parts = line.split(" ", 2)
        if len(parts) < 3:
            continue
        try:
            stamp = float(parts[0])
        except ValueError:
            continue
        if now - stamp > window_s:
            continue
        if parts[1] == "TX":
            tx.append(parts[2])
            continue
        try:
            record = json.loads(parts[2])
        except json.JSONDecodeError:
            continue
        if record.get("type") in nodes:
            nodes[record["type"]].append(record)
    return nodes, tx


def describe(label, records):
    if not records:
        return f"{label} 없음"
    last = records[-1]
    rssi = sum(int(r.get("rssi", 0)) for r in records) / len(records)
    warn = " 약함!" if rssi < RSSI_WEAK else ""
    return f"{label} {len(records):3d}건 gps={last.get('gps_valid')} rssi={rssi:.0f}{warn}"


def verdict(nodes):
    """실험을 시작해도 되는 상태인지 한 단어로."""
    if not nodes["cane"] and not nodes["vehicle"]:
        return "신호없음"
    if not nodes["cane"]:
        return "지팡이없음"
    if not nodes["vehicle"]:
        return "차량없음"
    if not all(
        str(records[-1].get("gps_valid")) == "1" for records in nodes.values()
    ):
        return "GPS대기"
    return "READY"


def status_line(path, now, window_s):
    nodes, tx = summarize(tail_lines(path, 400), now, window_s)
    clock = time.strftime("%H:%M:%S", time.localtime(now))
    tx_part = f"TX {len(tx)}건"
    if tx:
        tx_part += f" {tx[-1]}"
    return (
        f"{clock} [{verdict(nodes)}] "
        f"{describe('지팡이', nodes['cane'])} | "
        f"{describe('차량', nodes['vehicle'])} | {tx_part}"
    )


def parse_args():
    parser = argparse.ArgumentParser(description="V2X 링크 상태를 요약해서 보여준다.")
    parser.add_argument(
        "--log-dir",
        default=str(Path.home() / "v2x/03_jetson/logs"),
        help="raw_*.log 가 쌓이는 폴더",
    )
    parser.add_argument("--window-s", type=float, default=WINDOW_S,
                        help=f"최근 몇 초를 집계할지 (기본 {WINDOW_S})")
    parser.add_argument("--interval-s", type=float, default=INTERVAL_S,
                        help=f"몇 초마다 갱신할지 (기본 {INTERVAL_S})")
    parser.add_argument("--once", action="store_true", help="한 번만 출력하고 끝낸다")
    return parser.parse_args()


def main():
    args = parse_args()
    path = latest_log(args.log_dir)
    if path is None:
        raise SystemExit(f"raw 로그가 없습니다: {args.log_dir}")

    print(f"[감시] {path.name} (최근 {args.window_s:.0f}초 기준, 종료는 Ctrl+C)")
    try:
        while True:
            print(status_line(path, time.time(), args.window_s), flush=True)
            if args.once:
                return
            time.sleep(args.interval_s)
    except KeyboardInterrupt:
        print("\n[종료]")


if __name__ == "__main__":
    main()
