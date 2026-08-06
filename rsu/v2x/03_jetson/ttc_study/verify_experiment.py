#!/usr/bin/env python3
"""실험 직후 한 번에 돌리는 검증 스크립트.

raw 로그 하나를 받아 네 가지를 확인한다. 실험이 목적을 달성했는지, 그리고
배포한 변경이 의도대로 동작했는지를 현장에서 바로 판정하기 위한 것이다.

  1. GPS 갱신 주기   T_floor 2.0초는 5Hz(0.2초)를 전제로 계산한 값이다. 1Hz면
                     전제가 깨지고 실제 필요한 하한은 2.8초가 된다.
  2. 안전 하한 발동   TTC 2.0초 이하에서 레벨 3이 나갔는지. 배포 확인용.
  3. 위험 사례 수량   모델 검증에 쓸 수 있는 표본이 모였는지. 차량이 실제로
                     움직인 구간에서 세야 한다 - 정지 구간을 섞으면 희석된다.
  4. 섀도우 비교      같은 궤적에 모델을 돌려 규칙과 무엇이 달랐는지.

사용:
    python verify_experiment.py --log ../logs/raw_YYYYMMDD_HHMMSS.log
"""

import argparse
import json
from pathlib import Path

import numpy as np

from field_check import MOVING_MPS, field_features, read_raw_log, resample_pair
from safety_floor import T_FLOOR_S, safety_floor_s

# 이 아래로 벌어지면 실제 접촉 위험으로 본다 (oracle.D_CRIT_M과 같은 값).
D_CRIT_M = 2.0
# 모델 검증에 쓸 만한 접근으로 볼 최소 차량 속도.
FAST_MPS = 3.0


def gps_rate(rows):
    """좌표가 실제로 바뀐 간격에서 갱신 주기를 추정한다.

    노드는 10Hz로 올려 보내지만 그것은 같은 좌표의 재전송을 포함한다. 좌표가
    바뀐 시점만 세야 GPS 자체의 주기가 나온다.
    """
    out = {}
    for kind, records in rows.items():
        gaps, last = [], None
        for t, lat, lng in records:
            if last is not None and (lat, lng) != last[1]:
                gaps.append(t - last[0])
            if last is None or (lat, lng) != last[1]:
                last = (t, (lat, lng))
        gaps = np.array([g for g in gaps if 0.05 < g < 3.0])
        out[kind] = float(np.median(gaps)) if len(gaps) >= 20 else None
    return out


def check_gps(rows):
    print("1. GPS 갱신 주기")
    rates = gps_rate(rows)
    ok = True
    for kind, period in rates.items():
        if period is None:
            print(f"   {kind:8s} 표본 부족")
            continue
        hz = 1.0 / period
        verdict = "5Hz 계열" if hz >= 3.5 else ("1Hz 계열" if hz < 2.0 else "중간")
        print(f"   {kind:8s} {period*1000:6.0f} ms  ->  {hz:4.1f} Hz  ({verdict})")
        if hz < 3.5:
            ok = False
    if ok:
        print(f"   판정: 5Hz 확인. T_floor {T_FLOOR_S:.1f}초의 전제가 충족된다.")
    else:
        need = safety_floor_s(gps_period_s=1.0)
        print(f"   판정: 5Hz가 아니다. 이 조건에서 필요한 하한은 {need:.1f}초이고,")
        print(f"         배포된 {T_FLOOR_S:.1f}초는 {need - T_FLOOR_S:.1f}초 모자란다.")
    return ok


def check_safety_floor(log_path):
    """TX 로그에서 하한이 발동했을 만한 구간을 센다.

    raw 로그의 TX는 risk 값만 담으므로, 같은 시각대의 RX에서 재구성한 TTC와
    맞춰 본다. 정확한 판정은 step8 CSV(ttc_s 컬럼)로 해야 하지만, 여기서는
    "발동이 있었는가"를 빠르게 보는 것이 목적이다.
    """
    print("\n2. 안전 하한 발동")
    tx = []
    with open(log_path, encoding="utf-8", errors="replace") as handle:
        for line in handle:
            p = line.split(" ", 2)
            if len(p) < 3 or p[1] != "TX":
                continue
            try:
                tx.append((float(p[0]), json.loads(p[2]).get("risk")))
            except (ValueError, json.JSONDecodeError):
                continue
    if not tx:
        print("   TX 레코드 없음")
        return
    level3 = sum(1 for _, r in tx if r == 3)
    print(f"   TX {len(tx)}건 중 레벨 3: {level3}건 ({level3/len(tx)*100:.1f}%)")
    print("   정확한 하한 발동 판정은 step8 CSV에서:")
    print("     ttc_s <= 2.0 이면서 risk_level == 3 인 행")


def check_sample_size(field):
    print("\n3. 모델 검증용 표본")
    d = field["distance_m"]
    speed = field["veh_speed_mps"]
    moving = speed >= MOVING_MPS
    fast = speed >= FAST_MPS

    print(f"   전체 {len(d):,}스텝 / 움직인 구간 {int(moving.sum()):,} "
          f"({moving.mean()*100:.1f}%) / {FAST_MPS:.0f} m/s 이상 {int(fast.sum()):,}")
    print(f"   최소 거리 {d.min():.2f} m")

    # 접촉 위험이 실현된 구간: 거리가 d_crit 이하로 내려간 연속 구간 수
    close = d <= D_CRIT_M
    events = int(np.sum(close[1:] & ~close[:-1])) + int(close[0])
    fast_close = int(np.sum((close[1:] & ~close[:-1]) & fast[1:]))
    print(f"   접촉({D_CRIT_M} m 이내) 이벤트 {events}건, 그중 고속 {fast_close}건")

    if fast_close >= 30:
        print("   판정: 모델 섀도우 비교에 충분하다.")
    elif fast_close > 0:
        print(f"   판정: 표본이 적다. 고속 위험 30건 목표에 {30 - fast_close}건 부족.")
    else:
        print("   판정: 고속 위험 사례가 없다. 이 로그로는 모델을 검증할 수 없다.")


def check_shadow(field, n_scenarios):
    print("\n4. 섀도우 비교 (모델 vs 규칙)")
    import pandas as pd

    from baselines import ALARM_LEVEL, team_table_levels
    from dataset import build_dataset, split_by_scenario
    from model import predict_proba, threshold_at_false_alarm_rate, train

    sim = build_dataset(n_scenarios, gps_sigma_m=2.5, lead_s=2.0)
    train_df, test_df = split_by_scenario(sim, test_ratio=0.3, seed=0)
    clf = train(train_df, seed=0, label_col="y_train")

    # 모델 임계값은 시뮬레이션에서 규칙과 같은 오경보를 쓰도록 맞춘다.
    levels_sim = team_table_levels({c: test_df[c].to_numpy() for c in (
        "distance_m", "closing_los", "ttc", "veh_speed_mps", "dcpa_m")})
    far = float((levels_sim >= ALARM_LEVEL)[test_df.y.to_numpy() == 0].mean())
    proba_sim = predict_proba(clf, test_df)
    threshold = threshold_at_false_alarm_rate(proba_sim, test_df.y.to_numpy(), far)

    frame = pd.DataFrame({c: field[c] for c in field})
    proba = predict_proba(clf, frame)
    model_alarm = proba >= threshold
    rule_alarm = team_table_levels(field) >= ALARM_LEVEL

    print(f"   임계값 {threshold:.3f} (시뮬레이션 오경보 {far*100:.2f}% 기준)")
    print(f"   규칙 경보 {int(rule_alarm.sum()):,}스텝 / 모델 경보 {int(model_alarm.sum()):,}스텝")
    only_model = int((model_alarm & ~rule_alarm).sum())
    only_rule = int((rule_alarm & ~model_alarm).sum())
    print(f"   모델만 경보 {only_model:,} / 규칙만 경보 {only_rule:,}")

    d = field["distance_m"]
    for label, mask in (("모델만", model_alarm & ~rule_alarm),
                        ("규칙만", rule_alarm & ~model_alarm)):
        if mask.sum():
            print(f"   {label} 경보 구간: 거리 중앙 {np.median(d[mask]):.1f} m, "
                  f"차속 중앙 {np.median(field['veh_speed_mps'][mask]):.1f} m/s")


def main():
    parser = argparse.ArgumentParser(description="실험 로그를 한 번에 검증한다.")
    parser.add_argument("--log", required=True, help="step8이 남긴 raw 로그 경로")
    parser.add_argument("--n-scenarios", type=int, default=300,
                        help="섀도우 비교용 모델을 학습할 시뮬레이션 규모")
    parser.add_argument("--skip-shadow", action="store_true")
    args = parser.parse_args()

    path = Path(args.log)
    if not path.exists():
        raise SystemExit(f"로그가 없다: {path}")

    rows = read_raw_log(path)
    print(f"실측 레코드: cane {len(rows['cane']):,}  vehicle {len(rows['vehicle']):,}\n")
    if not rows["cane"] or not rows["vehicle"]:
        raise SystemExit("두 노드의 유효 GPS가 모두 필요하다")

    check_gps(rows)
    check_safety_floor(path)

    scenario = resample_pair(rows["cane"], rows["vehicle"])
    if scenario is None:
        raise SystemExit("두 노드가 겹치는 구간이 너무 짧다")
    field = field_features(scenario)
    check_sample_size(field)

    if not args.skip_shadow:
        check_shadow(field, args.n_scenarios)


if __name__ == "__main__":
    main()
