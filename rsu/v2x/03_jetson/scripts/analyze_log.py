#!/usr/bin/env python3
"""실험 로그 한 세트를 요약한다.

실험이 끝날 때마다 확인하는 지표들을 한 번에 낸다:
GPS 품질, 좌표 튐, 패킷 도달, 다운링크 반영, 접근속도 추정 오차.

접근속도 오차는 실제 움직인 구간만 따로 본다. 지팡이가 대부분 정지해 있어서
전체 평균을 내면 정지 구간이 오차를 0으로 희석해 버리기 때문이다.

사용법 (젯슨에서, 인자 없으면 가장 최근 실험):
    python3 ~/v2x/03_jetson/scripts/analyze_log.py
    python3 ~/v2x/03_jetson/scripts/analyze_log.py exp_8.5_5hz
"""

import argparse
import csv
import json
import math
import statistics as st
from pathlib import Path


LOG_DIR = Path.home() / "v2x/03_jetson/logs"

# RC카가 낼 수 없는 속도. 이보다 빠른 좌표 이동은 GPS 튐으로 본다.
IMPOSSIBLE_MPS = 8.0
BLATANT_MPS = 20.0
# 이 속도 이상으로 실제 거리가 변한 구간만 접근속도 정확도를 따진다.
MOVING_MPS = 1.0


def meters(a, b):
    scale = 111320 * math.cos(math.radians((a[0] + b[0]) / 2))
    return math.hypot((b[1] - a[1]) * scale, (b[0] - a[0]) * 111132)


def find_pair(log_dir, name):
    """이름을 주면 그 짝을, 안 주면 가장 최근 실험을 찾는다."""
    if name:
        raws = sorted(log_dir.glob(f"*{name}*.log"))
        csvs = sorted(log_dir.glob(f"*{name}*.csv"))
    else:
        raws = sorted(log_dir.glob("raw_*.log"), key=lambda p: p.stat().st_mtime)
        csvs = sorted(log_dir.glob("risk_tx_*.csv"), key=lambda p: p.stat().st_mtime)
    return (raws[-1] if raws else None), (csvs[-1] if csvs else None)


def read_raw(path):
    nodes = {"cane": [], "vehicle": []}
    for line in path.open(encoding="utf-8", errors="replace"):
        parts = line.split(" ", 2)
        if len(parts) < 3 or parts[1] != "RX":
            continue
        try:
            stamp = float(parts[0])
            record = json.loads(parts[2])
        except ValueError:
            continue
        if record.get("type") in nodes:
            nodes[record["type"]].append((stamp, record))
    return nodes


def report_raw(nodes):
    print("=== 노드별 수신 ===")
    for name, records in nodes.items():
        if not records:
            print(f"  {name}: 없음")
            continue
        invalid = sum(1 for _, r in records if not r.get("gps_valid"))
        rssi = [int(r["rssi"]) for _, r in records if r.get("rssi") not in (None, "")]
        label = "지팡이" if name == "cane" else "차량"
        print(
            f"  {label}: {len(records)}건, "
            f"GPS무효 {invalid/len(records)*100:.1f}%, "
            f"rssi 중앙값 {st.median(rssi):.0f}" if rssi else ""
        )

    print()
    print("=== GPS 좌표 튐 ===")
    for name, records in nodes.items():
        jumps, updates, prev = [], [], None
        for stamp, r in records:
            if not r.get("gps_valid"):
                continue
            pos = (r["lat"], r["lng"])
            if pos == (0, 0):
                continue
            if prev is not None:
                gap = stamp - prev[0]
                if pos != prev[1]:
                    if 0 < gap < 10:
                        updates.append(gap)
                    if 0 < gap < 2:
                        jumps.append(meters(prev[1], pos) / gap)
                    prev = (stamp, pos)
            else:
                prev = (stamp, pos)
        if not jumps:
            continue
        label = "지팡이" if name == "cane" else "차량"
        bad = sum(1 for v in jumps if v > IMPOSSIBLE_MPS)
        worst = sum(1 for v in jumps if v > BLATANT_MPS)
        print(
            f"  {label}: 불가능한 이동 {bad/len(jumps)*100:.2f}%, "
            f"명백한 튐 {worst/len(jumps)*100:.2f}%, 최대 {max(jumps):.0f} m/s"
        )
        if updates:
            hz = 1 / st.median(updates)
            print(f"         GPS 갱신 {st.median(updates):.2f}초 간격 -> 약 {hz:.1f} Hz")


def report_csv(path):
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    if not rows:
        print("  (판단 기록 없음 - 위험 계산이 한 번도 안 돌았습니다)")
        return
    num = lambda r, k: float(r[k])

    print(f"=== 위험 판단 ({len(rows)}회 송신) ===")
    levels = {}
    for r in rows:
        levels[r["effective_level"]] = levels.get(r["effective_level"], 0) + 1
    print("  보낸 등급:", dict(sorted(levels.items())))

    lags, missed = [], 0
    for k in range(1, len(rows)):
        if rows[k]["effective_level"] == rows[k - 1]["effective_level"]:
            continue
        target = int(rows[k]["effective_level"])
        start = num(rows[k], "pc_time")
        for m in range(k, len(rows)):
            if num(rows[m], "pc_time") - start > 6:
                break
            if int(rows[m]["cane_node_risk"]) == target:
                lags.append(num(rows[m], "pc_time") - start)
                break
        else:
            missed += 1
    total = len(lags) + missed
    if total:
        print(f"  다운링크: 등급변경 {total}회 중 {len(lags)}회 반영 "
              f"({len(lags)/total*100:.0f}%)")
        if lags:
            print(f"            반영까지 중앙값 {st.median(lags):.2f}초, 최대 {max(lags):.2f}초")

    print()
    print("=== 접근속도 추정 (실제 거리변화와 비교) ===")
    pairs = []
    for a, b in zip(rows, rows[1:]):
        gap = num(b, "pc_time") - num(a, "pc_time")
        if not 0.5 < gap < 2:
            continue
        real = (num(a, "distance_m") - num(b, "distance_m")) / gap
        if abs(real) < IMPOSSIBLE_MPS:
            pairs.append((real, num(a, "closing_mps")))
    if not pairs:
        print("  비교할 구간이 없습니다")
        return
    moving = [(r, e) for r, e in pairs if abs(r) >= MOVING_MPS]
    still = len(pairs) - len(moving)
    print(f"  비교구간 {len(pairs)}개 (거의 정지 {still}개는 제외)")
    if moving:
        err = [r - e for r, e in moving]
        print(f"  움직인 {len(moving)}개 구간: 오차 중앙값 {st.median(err):+.2f} m/s")
        print(f"    (+는 실제보다 낮게 봤다는 뜻 = 위험을 늦게 감지)")
        print(f"    과소평가 {sum(1 for d in err if d > 1)}회 / "
              f"과대평가 {sum(1 for d in err if d < -1)}회")
    else:
        print("  실제로 움직인 구간이 없어 판단할 수 없습니다")


def parse_args():
    parser = argparse.ArgumentParser(description="V2X 실험 로그 요약")
    parser.add_argument("name", nargs="?", help="실험 이름 (생략하면 가장 최근)")
    parser.add_argument("--log-dir", default=str(LOG_DIR))
    return parser.parse_args()


def main():
    args = parse_args()
    raw_path, csv_path = find_pair(Path(args.log_dir), args.name)

    if raw_path is None and csv_path is None:
        raise SystemExit(f"로그를 찾지 못했습니다: {args.log_dir}")

    if raw_path is not None:
        print(f"[통신 로그] {raw_path.name}")
        report_raw(read_raw(raw_path))
        print()
    if csv_path is not None:
        print(f"[판단 로그] {csv_path.name}")
        report_csv(csv_path)


if __name__ == "__main__":
    main()
