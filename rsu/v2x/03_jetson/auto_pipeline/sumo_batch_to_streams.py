#!/usr/bin/env python3
"""SUMO 생성기 v3 시나리오 폴더 여러 개 → sim_to_rsu_stream 일괄 실행 (쌍 선택 포함).

sim_to_rsu_stream 은 시나리오당 (차량 1, 보행자 1) 쌍 하나를 변환하고 기본값은 각 파일의
첫 id 다. 첫 쌍은 대개 멀리 떨어져 있어 위험 표본(hit)이 거의 없다. 여기서는 시나리오마다
참값 거리 기준으로 "가장 가까이 접근하는 쌍" 상위 K개를 골라 변환한다 (K개 중 일부는
안 위험한 쌍도 섞이도록 하한 없음 → 안전/위험 균형).

사용
    python sumo_batch_to_streams.py <scenarios_root> --out sumo_streams --pairs 2 --limit 700 --randomize
"""
import argparse
import csv
import math
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent


def closest_pairs(scenario_dir, k):
    """(min_distance, vehicle_id, person_id) 오름차순 상위 k."""
    veh = list(csv.DictReader((scenario_dir / "feature.csv").open(encoding="utf-8-sig"), delimiter=";"))
    ped = list(csv.DictReader((scenario_dir / "pedestrian.csv").open(encoding="utf-8-sig"), delimiter=";"))
    ped_by_t = defaultdict(list)
    for r in ped:
        ped_by_t[float(r["timestep_time"])].append((r["person_id"], float(r["person_x"]), float(r["person_y"])))
    best = {}
    for r in veh:
        t = float(r["timestep_time"])
        if t not in ped_by_t:
            continue
        vx, vy = float(r["vehicle_x"]), float(r["vehicle_y"])
        for pid, px, py in ped_by_t[t]:
            d = math.hypot(vx - px, vy - py)
            key = (r["vehicle_id"], pid)
            if d < best.get(key, math.inf):
                best[key] = d
    ranked = sorted(((d, v, p) for (v, p), d in best.items()))
    return ranked[:k]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root")
    ap.add_argument("--out", default="sumo_streams")
    ap.add_argument("--pairs", type=int, default=2, help="시나리오당 최근접 쌍 수")
    ap.add_argument("--limit", type=int, default=0, help="처리할 시나리오 수 (0=전부)")
    ap.add_argument("--randomize", action="store_true")
    ap.add_argument("--gps-jump", action="store_true",
                    help="run_with_gps_jump.py 를 거쳐 실행 (8/17 실측 GPS 튐 강조 프리셋)")
    args = ap.parse_args()

    dirs = sorted(d for d in Path(args.root).glob("scenario_*")
                  if (d / "feature.csv").exists() and (d / "pedestrian.csv").exists())
    if args.limit:
        dirs = dirs[:args.limit]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    done = 0
    for i, sd in enumerate(dirs, 1):
        try:
            pairs = closest_pairs(sd, args.pairs)
        except Exception as e:
            print(f"[SKIP] {sd.name}: {e}", file=sys.stderr)
            continue
        for d, v, p in pairs:
            if args.gps_jump:
                cmd = [sys.executable, str(HERE / "run_with_gps_jump.py"), "stream", str(sd)]
            else:
                cmd = [sys.executable, str(HERE / "sim_to_rsu_stream.py"), str(sd)]
            cmd += ["--vehicle-id", v, "--person-id", p, "--out", str(out)]
            if args.randomize:
                cmd.append("--randomize")
            r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
            if r.returncode == 0:
                done += 1
            else:
                print(f"[FAIL] {sd.name} {v}/{p}: {(r.stderr or '').strip()[-200:]}", file=sys.stderr)
        if i % 50 == 0:
            print(f"{i}/{len(dirs)} scenarios, {done} streams", flush=True)
    print(f"[OK] {done} streams -> {out}")


if __name__ == "__main__":
    main()
