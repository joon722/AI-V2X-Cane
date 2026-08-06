"""채점기 검증.

지표가 틀리면 세 방법의 비교가 통째로 무의미해지므로, 답을 아는 인공 예측기로
먼저 검증한다.
"""

import numpy as np
import pandas as pd
import pytest

from baselines import score_alarms, sweep_ttc_threshold, team_table_levels


def toy_frame():
    """위험 3시점, 안전 3시점짜리 손계산용 표.

    시나리오 1은 t=0,1,2에서 위험하고 t=2.0에 위험이 실현된다.
    시나리오 2는 내내 안전하다.
    """
    return pd.DataFrame({
        "y":           [1, 1, 1, 0, 0, 0],
        "t":           [0.0, 1.0, 2.0, 0.0, 1.0, 2.0],
        "t_hit":       [2.0, 2.0, 2.0, np.nan, np.nan, np.nan],
        "scenario_id": [1, 1, 1, 2, 2, 2],
    })


def test_perfect_predictor_scores_perfectly():
    df = toy_frame()

    s = score_alarms(df, alarm=df.y.to_numpy().astype(bool))

    assert s.recall == pytest.approx(1.0)
    assert s.false_alarm_rate == pytest.approx(0.0)


def test_always_alarming_predictor_has_full_false_alarm_rate():
    """다 울리면 재현율은 1이지만 오경보율도 1이다 - 재현율만 보면 안 되는 이유."""
    df = toy_frame()

    s = score_alarms(df, alarm=np.ones(len(df), dtype=bool))

    assert s.recall == pytest.approx(1.0)
    assert s.false_alarm_rate == pytest.approx(1.0)


def test_silent_predictor_detects_nothing():
    df = toy_frame()

    s = score_alarms(df, alarm=np.zeros(len(df), dtype=bool))

    assert s.recall == pytest.approx(0.0)
    assert s.detected_scenarios == 0


def test_lead_time_is_measured_from_the_first_alarm():
    """경보 여유시간은 첫 경보와 위험 실현 시각의 차다."""
    df = toy_frame()
    # t=1.0에서 처음 울린다. 위험 실현은 t=2.0이므로 여유는 1.0초.
    alarm = np.array([False, True, True, False, False, False])

    s = score_alarms(df, alarm=alarm)

    assert s.lead_median == pytest.approx(1.0)


def test_scenario_is_detected_if_any_timestep_alarms():
    """한 시점이라도 울리면 그 시나리오는 잡은 것으로 센다."""
    df = toy_frame()
    alarm = np.array([False, False, True, False, False, False])

    s = score_alarms(df, alarm=alarm)

    assert s.detected_scenarios == 1
    assert s.total_dangerous == 1


def test_team_table_reproduces_step7_levels():
    """현 점수표 채점이 step7_risk와 같은 값을 낸다.

    비교 대상이 실제 배포 코드와 달라지면 "현 시스템이 이만큼"이라는 말이
    성립하지 않으므로, 동결 사본이 아니라 실제 모듈을 불러 쓴다.
    """
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from step7_risk import calculate_risk_score, classify_risk_level, dcpa_gate

    f = {
        "distance_m": np.array([8.0]),
        "closing_los": np.array([12.0]),
        "ttc": np.array([0.67]),
        "veh_speed_mps": np.array([12.0]),
        "dcpa_m": np.array([1.0]),
    }
    expected = classify_risk_level(
        calculate_risk_score(8.0, 12.0, 12.0, 0.67) * dcpa_gate(1.0)
    )

    assert team_table_levels(f)[0] == expected


def test_raising_the_threshold_never_lowers_recall():
    """TTC 임계값을 올리면 더 일찍 울리므로 재현율이 떨어지지 않는다.

    스윕이 뒤집혀 있으면 최적점이 반대로 잡히므로 단조성을 확인한다.
    """
    from dataset import build_dataset

    ds = build_dataset(n_scenarios=30, gps_sigma_m=2.5)

    table = sweep_ttc_threshold(ds, thresholds=(1.0, 2.0, 4.0, 8.0))

    recalls = [row.recall for row in table]
    assert recalls == sorted(recalls)


def test_sweep_reports_one_row_per_threshold():
    from dataset import build_dataset

    ds = build_dataset(n_scenarios=10, gps_sigma_m=2.5)

    table = sweep_ttc_threshold(ds, thresholds=(1.0, 2.0, 3.0))

    assert len(table) == 3
    assert [r.threshold_s for r in table] == [1.0, 2.0, 3.0]
