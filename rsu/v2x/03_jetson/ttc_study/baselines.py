#!/usr/bin/env python3
"""규칙 쪽 두 가지를 오라클 정답으로 채점한다.

  A  현 점수표      step7_risk를 그대로 불러 쓴다. 동결 사본을 두지 않는 이유는
                    비교 대상이 실제 배포 코드와 어긋나면 "현 시스템이 이만큼"이라는
                    말이 성립하지 않기 때문이다.
  B  고정 TTC 임계값  TTC <= theta 면 경보. 규칙으로 낼 수 있는 최선을 스윕으로 찾는다.

B가 중요한 이유는 이것이 모델이 넘어야 할 선이기 때문이다. 모델이 A만 이기는 것은
"점수표를 잘 조정했다"는 뜻이지 AI가 필요하다는 증거가 아니다. B까지 넘어야
"고정 임계값으로는 여기가 한계"라고 말할 수 있다.

지표를 셋으로 보는 이유:
  재현율만 보면 늘 울리는 판정기가 이긴다.
  오경보율만 보면 침묵하는 판정기가 이긴다.
  경보 여유시간은 "맞히긴 했는데 너무 늦게"를 잡아낸다. 실측 기준 이 시스템의
  전달 지연이 0.1초이고 사람이 멈추는 데 시간이 더 걸리므로, 늦은 정답은
  정답이 아니다.
"""

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_JETSON = Path(__file__).resolve().parent.parent
if str(_JETSON) not in sys.path:
    sys.path.insert(0, str(_JETSON))

from step7_risk import calculate_risk_score, classify_risk_level, dcpa_gate  # noqa: E402

# 경보로 칠 최소 레벨. step7 기준 2는 '경보', 3은 '위험'이다.
ALARM_LEVEL = 2


@dataclass(frozen=True)
class Score:
    recall: float             # 위험 시점 중 경보한 비율
    false_alarm_rate: float   # 안전 시점 중 경보한 비율
    lead_median: float        # 첫 경보 -> 위험 실현까지 (초), 중앙값
    lead_p5: float            # 같은 값의 5퍼센타일 = 최악에 가까운 경우
    detected_scenarios: int   # 위험 시나리오 중 하나라도 경보한 수
    total_dangerous: int


@dataclass(frozen=True)
class SweepRow:
    threshold_s: float
    recall: float
    false_alarm_rate: float
    lead_median: float
    detected_scenarios: int


def team_table_levels(features, zone_base_risk=0):
    """현 점수표를 피처 표에 적용해 0~3 레벨을 낸다.

    step7_risk.assess_risk는 Kinematics 객체를 받으므로, 배열을 다루는 여기서는
    같은 모듈의 함수 셋을 같은 순서로 직접 부른다(점수 -> 게이트 -> 레벨).
    """
    distance = np.asarray(features["distance_m"])
    closing = np.asarray(features["closing_los"])
    ttc = np.asarray(features["ttc"])
    veh_speed = np.asarray(features["veh_speed_mps"])
    dcpa = np.asarray(features["dcpa_m"])

    levels = np.zeros(len(distance), dtype=np.int8)
    for i in range(len(distance)):
        score = calculate_risk_score(
            float(distance[i]), float(closing[i]), float(veh_speed[i]),
            float(ttc[i]), zone_base_risk,
        )
        levels[i] = classify_risk_level(score * dcpa_gate(float(dcpa[i])))
    return levels


def score_alarms(df, alarm):
    """경보 배열 하나를 오라클 정답으로 채점한다."""
    y = df.y.to_numpy().astype(bool)
    alarm = np.asarray(alarm, dtype=bool)

    recall = float(alarm[y].mean()) if y.any() else 0.0
    false_alarm_rate = float(alarm[~y].mean()) if (~y).any() else 0.0

    leads = []
    detected = 0
    dangerous_ids = df.loc[y, "scenario_id"].unique()
    for sid in dangerous_ids:
        rows = df[df.scenario_id == sid]
        mask = alarm[rows.index.to_numpy()]
        if not mask.any():
            continue
        detected += 1
        first_alarm_t = float(rows.t.to_numpy()[mask][0])
        # 그 시나리오에서 위험이 처음 실현되는 시각
        hits = rows.t_hit.to_numpy()
        hits = hits[~np.isnan(hits)]
        if len(hits):
            leads.append(float(hits.min()) - first_alarm_t)

    return Score(
        recall=recall,
        false_alarm_rate=false_alarm_rate,
        lead_median=float(np.median(leads)) if leads else float("nan"),
        lead_p5=float(np.percentile(leads, 5)) if leads else float("nan"),
        detected_scenarios=detected,
        total_dangerous=len(dangerous_ids),
    )


def sweep_ttc_threshold(df, thresholds):
    """TTC <= theta 를 경보로 보는 단순 규칙을 임계값별로 채점한다."""
    ttc = df.ttc.to_numpy()
    rows = []
    for theta in thresholds:
        s = score_alarms(df, alarm=ttc <= theta)
        rows.append(SweepRow(
            threshold_s=float(theta),
            recall=s.recall,
            false_alarm_rate=s.false_alarm_rate,
            lead_median=s.lead_median,
            detected_scenarios=s.detected_scenarios,
        ))
    return rows


def main():
    import argparse

    from dataset import build_dataset

    parser = argparse.ArgumentParser(description="규칙 베이스라인 두 가지를 채점한다.")
    parser.add_argument("--n-scenarios", type=int, default=300)
    parser.add_argument("--gps-sigma-m", type=float, default=2.5)
    args = parser.parse_args()

    ds = build_dataset(args.n_scenarios, gps_sigma_m=args.gps_sigma_m)
    print(f"시나리오 {ds.scenario_id.nunique()}개 / {len(ds):,}행 / "
          f"위험 {ds.y.mean()*100:.2f}%\n")

    levels = team_table_levels({c: ds[c].to_numpy() for c in ds.columns if c in (
        "distance_m", "closing_los", "ttc", "veh_speed_mps", "dcpa_m")})
    a = score_alarms(ds, alarm=levels >= ALARM_LEVEL)
    print("A. 현 점수표 (step7_risk, 레벨2+)")
    print(f"   재현율 {a.recall*100:5.1f}%  오경보율 {a.false_alarm_rate*100:5.1f}%  "
          f"검출 {a.detected_scenarios}/{a.total_dangerous}  여유 {a.lead_median:.2f}s\n")

    print("B. 고정 TTC 임계값")
    print(f"   {'임계값':>6}  {'재현율':>7}  {'오경보율':>8}  {'여유(중앙)':>10}  검출")
    for row in sweep_ttc_threshold(ds, np.arange(0.5, 8.1, 0.5)):
        print(f"   {row.threshold_s:5.1f}s  {row.recall*100:6.1f}%  "
              f"{row.false_alarm_rate*100:7.1f}%  {row.lead_median:9.2f}s  "
              f"{row.detected_scenarios}")


if __name__ == "__main__":
    main()
