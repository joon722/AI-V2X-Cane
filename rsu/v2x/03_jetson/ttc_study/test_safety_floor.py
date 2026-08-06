"""안전 하한과 적시 경보 지표 검증.

T_floor는 학습으로 정하지 않고 물리로 정한다. 그래서 값 자체보다 "무엇을 더해서
나온 값인가"가 검증 대상이다 - 항이 빠지거나 순서가 틀리면 근거가 무너진다.
"""

import numpy as np
import pandas as pd
import pytest

from safety_floor import (
    DEFAULT_BUDGET,
    safety_floor_s,
    timely_alarm_rate,
)


def test_floor_is_the_sum_of_its_physical_parts():
    """하한은 각 항의 합이다. 임의의 상수가 아니다."""
    got = safety_floor_s(
        gps_period_s=0.2,
        transport_s=0.1,
        perception_s=0.3,
        stopping_s=1.2,
        margin_s=0.2,
    )

    assert got == pytest.approx(2.0)


def test_slower_gps_raises_the_floor():
    """GPS가 느리면 하한이 올라간다 - 위치가 묵은 만큼 더 일찍 울려야 한다."""
    fast = safety_floor_s(gps_period_s=0.2)
    slow = safety_floor_s(gps_period_s=1.0)

    assert slow > fast
    assert slow - fast == pytest.approx(0.8)


def test_default_budget_matches_the_published_thresholds():
    """기본 예산의 합이 공인 기준(GB/T 33577 최소 2초)과 같은 자리에 온다."""
    assert safety_floor_s(**DEFAULT_BUDGET) == pytest.approx(2.0, abs=0.05)


def test_timely_alarm_needs_enough_lead_time():
    """위험 실현보다 T_floor 이상 앞서 울려야 적시 경보로 센다.

    맞히긴 했는데 사람이 반응할 수 없는 시점이면 그 정답은 쓸모가 없다. 재현율만
    보면 이 구분이 사라진다.
    """
    df = pd.DataFrame({
        "y":           [1, 1, 1, 1],
        "t":           [0.0, 1.0, 2.0, 3.0],
        "t_hit":       [3.0, 3.0, 3.0, 3.0],
        "scenario_id": [1, 1, 1, 1],
    })

    early = np.array([True, False, False, False])   # t=0, 여유 3.0초
    late = np.array([False, False, True, False])    # t=2.0, 여유 1.0초

    assert timely_alarm_rate(df, early, floor_s=2.0) == pytest.approx(1.0)
    assert timely_alarm_rate(df, late, floor_s=2.0) == pytest.approx(0.0)


def test_silence_is_not_timely():
    """한 번도 울리지 않은 시나리오는 적시가 아니다."""
    df = pd.DataFrame({
        "y": [1, 1], "t": [0.0, 1.0], "t_hit": [2.0, 2.0], "scenario_id": [1, 1],
    })

    assert timely_alarm_rate(df, np.zeros(2, dtype=bool), floor_s=2.0) == pytest.approx(0.0)


def test_only_the_first_alarm_counts():
    """늦게 다시 울려도 첫 경보가 이미 적시였으면 적시다."""
    df = pd.DataFrame({
        "y": [1, 1, 1], "t": [0.0, 1.0, 2.0], "t_hit": [3.0, 3.0, 3.0],
        "scenario_id": [1, 1, 1],
    })

    assert timely_alarm_rate(df, np.array([True, False, True]), floor_s=2.0) == pytest.approx(1.0)


def test_rate_is_measured_over_dangerous_scenarios():
    """분모는 위험 시나리오 수다. 안전한 시나리오는 이 지표에 끼지 않는다."""
    df = pd.DataFrame({
        "y":           [1, 1, 0, 0],
        "t":           [0.0, 1.0, 0.0, 1.0],
        "t_hit":       [3.0, 3.0, np.nan, np.nan],
        "scenario_id": [1, 1, 2, 2],
    })

    alarm = np.array([True, False, True, True])  # 시나리오 2의 경보는 분모 밖

    assert timely_alarm_rate(df, alarm, floor_s=2.0) == pytest.approx(1.0)
