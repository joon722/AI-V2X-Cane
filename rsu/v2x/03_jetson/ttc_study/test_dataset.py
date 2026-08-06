"""데이터셋 조립 검증.

가장 중요한 것은 시나리오 경계다. 같은 시나리오의 앞부분이 학습에, 뒷부분이
평가에 들어가면 모델이 정답을 미리 본 셈이 되어 성능이 부풀려진다. 시간
연속 데이터에서 가장 흔한 누수이므로 테스트로 못 박는다.
"""

import numpy as np
import pytest

from dataset import build_dataset, split_by_scenario
from features import FEATURE_COLUMNS


def test_dataset_carries_features_labels_and_scenario_id():
    ds = build_dataset(n_scenarios=5, gps_sigma_m=2.5)

    for name in FEATURE_COLUMNS:
        assert name in ds.columns
    assert "y" in ds.columns
    assert "scenario_id" in ds.columns


def test_rows_come_from_the_requested_number_of_scenarios():
    ds = build_dataset(n_scenarios=7, gps_sigma_m=2.5)

    assert ds.scenario_id.nunique() == 7


def test_same_seed_reproduces_the_dataset():
    a = build_dataset(n_scenarios=4, gps_sigma_m=2.5, seed_offset=100)
    b = build_dataset(n_scenarios=4, gps_sigma_m=2.5, seed_offset=100)

    assert np.allclose(a.distance_m.to_numpy(), b.distance_m.to_numpy())
    assert (a.y.to_numpy() == b.y.to_numpy()).all()


def test_split_never_puts_one_scenario_on_both_sides():
    """분할이 시나리오 단위로 이뤄진다 - 시계열 데이터의 대표적 누수 방지."""
    ds = build_dataset(n_scenarios=20, gps_sigma_m=2.5)

    train, test = split_by_scenario(ds, test_ratio=0.3, seed=0)

    assert set(train.scenario_id) & set(test.scenario_id) == set()
    assert len(train) + len(test) == len(ds)


def test_split_puts_dangerous_scenarios_on_both_sides():
    """위험 시나리오가 학습과 평가 양쪽에 들어간다.

    위험 시나리오는 전체의 6% 남짓이라, 그냥 섞으면 한쪽으로 쏠린다. 실제로
    60개를 나눴을 때 위험 5개 중 4개가 평가로 가서 학습 세트에 1개만 남았고,
    모델이 아무것도 배우지 못해 AUC가 0.49(무작위)로 나왔다.
    """
    ds = build_dataset(n_scenarios=80, gps_sigma_m=2.5)

    train, test = split_by_scenario(ds, test_ratio=0.3, seed=0)

    assert train.y.sum() > 0, "학습 세트에 위험 사례가 없다"
    assert test.y.sum() > 0, "평가 세트에 위험 사례가 없다"


def test_split_keeps_the_danger_ratio_on_both_sides():
    """양쪽의 위험 시나리오 비율이 전체 비율과 크게 다르지 않다."""
    ds = build_dataset(n_scenarios=120, gps_sigma_m=2.5)
    train, test = split_by_scenario(ds, test_ratio=0.25, seed=0)

    def danger_ratio(d):
        per_scenario = d.groupby("scenario_id").y.max() > 0
        return per_scenario.mean()

    assert abs(danger_ratio(train) - danger_ratio(test)) < 0.10


def test_split_is_reproducible():
    ds = build_dataset(n_scenarios=20, gps_sigma_m=2.5)

    a, _ = split_by_scenario(ds, test_ratio=0.3, seed=7)
    b, _ = split_by_scenario(ds, test_ratio=0.3, seed=7)

    assert set(a.scenario_id) == set(b.scenario_id)


def test_dataset_contains_both_classes():
    """위험과 안전이 모두 들어 있어야 채점이 가능하다."""
    ds = build_dataset(n_scenarios=60, gps_sigma_m=2.5)

    assert ds.y.sum() > 0, "위험 사례가 없으면 재현율을 잴 수 없다"
    assert (ds.y == 0).sum() > 0, "안전 사례가 없으면 오경보율을 잴 수 없다"


def test_labels_are_built_from_truth_not_noisy_observation():
    """정답은 참값 궤적에서, 피처는 노이즈 낀 관측에서 만든다.

    라벨까지 노이즈에서 뽑으면 "GPS가 흔들려서 위험해 보인 순간"을 진짜 위험으로
    가르치게 된다. 젯슨이 풀어야 할 문제는 그 반대다.
    """
    clean = build_dataset(n_scenarios=6, gps_sigma_m=0.0, seed_offset=0)
    noisy = build_dataset(n_scenarios=6, gps_sigma_m=2.5, seed_offset=0)

    assert (clean.y.to_numpy() == noisy.y.to_numpy()).all()
    assert not np.allclose(clean.distance_m.to_numpy(), noisy.distance_m.to_numpy())


def test_time_and_hit_time_are_kept_for_lead_time_evaluation():
    """경보 여유시간을 재려면 시각과 위험 실현 시각이 필요하다."""
    ds = build_dataset(n_scenarios=5, gps_sigma_m=2.5)

    assert "t" in ds.columns
    assert "t_hit" in ds.columns
    dangerous = ds[ds.y == 1]
    if len(dangerous):
        assert (dangerous.t_hit >= dangerous.t).all()
