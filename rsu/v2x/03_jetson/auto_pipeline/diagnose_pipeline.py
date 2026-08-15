#!/usr/bin/env python3
"""젯슨 추론 파이프라인 진단.

두 가지를 수치로 확인한다. 읽기만 하며 어떤 파일도 수정하지 않는다.

1. 규칙 점수와 모델 예측의 일치율
   risk_score 를 팀 임계값(70/45/20)으로 자른 등급과 모델이 낸 등급을 비교한다.
   일치율이 1에 가까우면 모델이 risk_score 만 보고 판정한다는 뜻이며, 나머지
   입력(거리·속도·방향 벡터 등)이 학습에 기여하지 않았음을 시사한다.

2. 프레임 부족으로 추론에서 제외된 차량
   process_scenarios.py 는 시퀀스 길이(10)보다 관측 구간이 짧은 차량을 통째로
   건너뛴다. 빠르게 지나가는 차일수록 관측 구간이 짧으므로 위험한 차가 우선
   누락될 수 있다. 제외 차량의 속도 분포와 보행자 최근접 거리를 확인한다.
   최근접 거리가 핵심이다 - 제외된 차가 보행자 가까이 지나갔다면 놓친 위험이다.

사용:
    python3 diagnose_pipeline.py
    python3 diagnose_pipeline.py --limit 50
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

RULE_THRESHOLDS = (70, 45, 20)  # step7_risk.classify_risk_level 과 동일
SEQ_LEN = 10                    # scaler.json 의 sequence_length
NEAR_M = (5.0, 10.0)            # 제외 차량이 보행자에 얼마나 접근했는지 볼 기준


def rule_level(score):
    hi, mid, low = RULE_THRESHOLDS
    if score >= hi:
        return 3
    if score >= mid:
        return 2
    if score >= low:
        return 1
    return 0


def newest(paths, limit):
    return sorted(paths, key=lambda p: p.stat().st_mtime, reverse=True)[:limit]


# ---------------------------------------------------------
# 1. 규칙 점수 대비 모델 예측
# ---------------------------------------------------------
def check_leakage(results_dir, limit):
    files = newest(list(Path(results_dir).glob("*_result.csv")), limit)
    if not files:
        print(f"[건너뜀] 결과 파일이 없습니다: {results_dir}")
        return

    rows = matched = 0
    confusion = {}
    used_column = None
    skipped_files = 0
    for path in files:
        df = pd.read_csv(path)
        column = ("onnx_risk_level_raw" if "onnx_risk_level_raw" in df.columns
                  else "onnx_risk_level")
        if column not in df.columns or "risk_score" not in df.columns:
            skipped_files += 1
            continue
        used_column = used_column or column
        rule = df["risk_score"].map(rule_level)
        model = df[column].astype(int)
        rows += len(df)
        matched += int((rule == model).sum())
        for r, m in zip(rule, model):
            confusion[(r, m)] = confusion.get((r, m), 0) + 1

    if rows == 0:
        print("[건너뜀] 비교 가능한 결과 파일이 없습니다")
        return

    print("=" * 62)
    print("1. 규칙 점수 대비 모델 예측 일치율")
    print("=" * 62)
    print(f"대상 파일 {len(files) - skipped_files}개, {rows:,}행")
    print(f"비교 컬럼: {used_column}"
          + ("  (게이트 적용 후 값)" if used_column == "onnx_risk_level" else "  (게이트 전 원본)"))
    print()
    print(f"  일치율  {matched / rows * 100:6.2f}%")
    print()
    print("  규칙\\모델      0       1       2       3")
    for r in range(4):
        cells = "".join(f"{confusion.get((r, m), 0):8,}" for m in range(4))
        print(f"     {r}      {cells}")
    print()
    if matched / rows > 0.95:
        print("  판정: 모델이 risk_score 로 등급을 재현하고 있다.")
        print("        나머지 입력이 판정에 기여하지 않을 가능성이 높다.")
    elif matched / rows > 0.8:
        print("  판정: 상당 부분 일치한다. risk_score 의존도가 높다.")
    else:
        print("  판정: 규칙과 다르게 판정하고 있다. risk_score 외 입력도 쓰고 있다.")
    print()


# ---------------------------------------------------------
# 2. 프레임 부족으로 제외된 차량
# ---------------------------------------------------------
def load_scenario(scenario_dir):
    """process_scenarios.build_features 와 같은 방식으로 병합한 표를 반환."""
    veh = pd.read_csv(scenario_dir / "feature.csv", sep=";")
    ped = pd.read_csv(scenario_dir / "pedestrian.csv", sep=";")
    ped = (ped.sort_values("timestep_time")
              .groupby("timestep_time", as_index=False).first())
    ped = ped[["timestep_time", "person_x", "person_y"]]
    ped.columns = ["timestep_time", "ped_x", "ped_y"]
    df = veh.merge(ped, on="timestep_time", how="inner")
    if df.empty:
        return None
    df["distance_m"] = np.hypot(df["vehicle_x"] - df["ped_x"],
                                df["vehicle_y"] - df["ped_y"])
    return df


def check_skipped_vehicles(incoming_dir, limit, seq_len):
    dirs = [d for d in Path(incoming_dir).glob("scenario_*")
            if (d / "feature.csv").exists() and (d / "pedestrian.csv").exists()]
    dirs = newest(dirs, limit)
    if not dirs:
        print(f"[건너뜀] 시나리오 원본이 없습니다: {incoming_dir}")
        return

    kept_speed, skip_speed, skip_min_dist, kept_min_dist = [], [], [], []
    total = skipped = 0
    used = 0
    for scenario_dir in dirs:
        try:
            df = load_scenario(scenario_dir)
        except (KeyError, ValueError, pd.errors.ParserError):
            continue
        if df is None:
            continue
        used += 1
        for _, group in df.groupby("vehicle_id", sort=False):
            total += 1
            speed = float(group["vehicle_speed"].max())
            closest = float(group["distance_m"].min())
            if len(group) < seq_len:
                skipped += 1
                skip_speed.append(speed)
                skip_min_dist.append(closest)
            else:
                kept_speed.append(speed)
                kept_min_dist.append(closest)

    if total == 0:
        print("[건너뜀] 분석할 차량이 없습니다")
        return

    print("=" * 62)
    print(f"2. 관측 구간 {seq_len}프레임 미만으로 추론에서 제외된 차량")
    print("=" * 62)
    print(f"대상 시나리오 {used}개")
    print(f"  전체 차량   {total:,}대")
    print(f"  제외 차량   {skipped:,}대  ({skipped / total * 100:.1f}%)")
    print()

    if not skip_speed:
        print("  제외된 차량이 없다.")
        print()
        return

    def line(label, values):
        arr = np.asarray(values)
        return (f"  {label:10s} 평균 {arr.mean():6.2f}  "
                f"중앙 {np.median(arr):6.2f}  최대 {arr.max():6.2f}")

    print("  최고 속도 (m/s)")
    print(line("제외 차량", skip_speed))
    if kept_speed:
        print(line("유지 차량", kept_speed))
    print()

    print("  보행자 최근접 거리 (m)")
    print(line("제외 차량", skip_min_dist))
    if kept_min_dist:
        print(line("유지 차량", kept_min_dist))
    print()

    arr = np.asarray(skip_min_dist)
    print("  제외 차량 중 보행자에게 접근한 대수")
    for margin in NEAR_M:
        n = int((arr <= margin).sum())
        print(f"    {margin:4.0f} m 이내   {n:,}대  ({n / len(arr) * 100:.1f}%)")
    print()

    near = int((arr <= NEAR_M[0]).sum())
    if near > 0:
        print(f"  판정: 보행자 {NEAR_M[0]:.0f} m 이내로 접근한 차량 {near:,}대가")
        print("        추론 자체를 거치지 않았다. 놓친 위험에 해당한다.")
    elif skip_speed and kept_speed and np.mean(skip_speed) > np.mean(kept_speed):
        print("  판정: 제외 차량이 유지 차량보다 빠르다. 빠른 차가 우선 누락된다.")
    else:
        print("  판정: 제외 차량이 보행자 근처로 오지 않았다. 위험도는 낮다.")
    print()


def main():
    parser = argparse.ArgumentParser(description="젯슨 추론 파이프라인 진단")
    parser.add_argument("--results-dir", default=str(Path.home() / "inference_results"))
    parser.add_argument("--incoming-dir", default=str(Path.home() / "incoming_data"))
    parser.add_argument("--limit", type=int, default=20,
                        help="분석할 시나리오 수 (최신순, 기본 20)")
    parser.add_argument("--seq-len", type=int, default=SEQ_LEN)
    args = parser.parse_args()

    check_leakage(args.results_dir, args.limit)
    check_skipped_vehicles(args.incoming_dir, args.limit, args.seq_len)
    return 0


if __name__ == "__main__":
    sys.exit(main())
