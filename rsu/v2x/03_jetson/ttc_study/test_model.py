"""모델 검증.

성능보다 먼저 확인할 것은 "정답을 몰래 보고 있지 않은가"다. 현재 파이프라인이
risk_score를 입력에 넣어 라벨 누수를 만든 전례가 있으므로, 같은 실수를 구조로
막는다.
"""

import numpy as np
import pytest

from dataset import build_dataset, split_by_scenario
from features import FEATURE_COLUMNS
from model import MODEL_INPUTS, predict_proba, threshold_at_false_alarm_rate, train


def small_split():
    """학습에 위험 사례가 충분히 들어갈 만큼은 만든다.

    위험 시나리오가 전체의 6% 남짓이라 60개로는 학습 쪽에 한두 개밖에 안 남는다.
    층화 분할을 넣었어도 표본 자체가 적으면 결과가 시드에 휘둘린다.
    """
    ds = build_dataset(n_scenarios=150, gps_sigma_m=2.5)
    return split_by_scenario(ds, test_ratio=0.3, seed=0)


def test_model_inputs_are_features_only():
    """라벨과 정답 유래 컬럼은 입력에서 제외된다."""
    banned = {"y", "t_hit", "d_min_future", "scenario_id", "t"}

    assert set(MODEL_INPUTS) == set(FEATURE_COLUMNS)
    assert banned.isdisjoint(MODEL_INPUTS)


def test_predictions_are_probabilities():
    train_df, test_df = small_split()

    proba = predict_proba(train(train_df, seed=0), test_df)

    assert proba.shape == (len(test_df),)
    assert proba.min() >= 0.0 and proba.max() <= 1.0


def test_model_learns_something_beyond_chance():
    """무작위보다 나아야 한다. 아니면 피처나 라벨 쪽이 잘못된 것이다."""
    from sklearn.metrics import roc_auc_score

    train_df, test_df = small_split()

    proba = predict_proba(train(train_df, seed=0), test_df)

    assert roc_auc_score(test_df.y.to_numpy(), proba) > 0.7


def test_training_is_reproducible():
    train_df, test_df = small_split()

    a = predict_proba(train(train_df, seed=1), test_df)
    b = predict_proba(train(train_df, seed=1), test_df)

    assert np.allclose(a, b)


def test_threshold_hits_the_requested_false_alarm_rate():
    """임계값을 오경보율로 지정할 수 있어야 규칙과 같은 조건에서 비교된다.

    모델과 점수표를 각자 편한 지점에서 비교하면 아무 말이나 할 수 있다. 오경보를
    같은 양만 쓰게 묶어놓고 적시 경보를 비교해야 공정하다.
    """
    y = np.array([0] * 90 + [1] * 10)
    proba = np.linspace(0.0, 1.0, 100)

    th = threshold_at_false_alarm_rate(proba, y, target=0.10)
    far = float((proba[y == 0] >= th).mean())

    assert far == pytest.approx(0.10, abs=0.02)


def test_threshold_is_strict_when_no_false_alarms_allowed():
    """오경보 예산이 0이면 안전 시점 중 어느 것도 넘지 못하는 임계값을 준다."""
    y = np.array([0] * 90 + [1] * 10)
    proba = np.linspace(0.0, 1.0, 100)

    th = threshold_at_false_alarm_rate(proba, y, target=0.0)

    assert (proba[y == 0] >= th).sum() == 0
