#!/usr/bin/env python3
"""모델 판정을 규칙 위에 얹는 게이트 검증.

구조는 max(안전 하한, 규칙 점수, 모델)이다. 세 층 중 어느 하나가 위험이라고
하면 위험으로 나간다. 검증할 것은 층이 서로를 덮어쓰지 않는가다.

  안전 하한이 모델보다 세다   모델이 안전하다고 해도 TTC 2초면 레벨 3이 나간다.
                              모델은 학습 분포 밖에서 조용히 틀릴 수 있고,
                              하한은 물리로 계산한 값이라 그럴 일이 없다.
  모델은 규칙을 낮추지 못한다  모델이 안전하다고 해서 규칙이 올린 레벨을 내리면
                              안 된다. 모델은 더하기만 한다.
  모델이 없어도 돌아간다      파일이 없거나 껐을 때 기존 동작 그대로여야 한다.
"""

import unittest

from model_gate import ModelGate, MODEL_ALARM_LEVEL


class StubModel:
    """확률을 고정으로 돌려주는 가짜 모델."""

    def __init__(self, proba, threshold=0.5):
        self._proba = proba
        self.threshold = threshold
        self.features = ("distance_m",)

    def predict_proba(self, feature_map):
        return self._proba


class StubStreamer:
    def update(self, now, cane_state, veh_state):
        return {"distance_m": 10.0}


def gate(proba=None, threshold=0.5):
    model = None if proba is None else StubModel(proba, threshold)
    return ModelGate(model=model, streamer=StubStreamer())


STATES = ((0.0, 0.0, 0.0, 0.0), (0.0, 20.0, 0.0, -10.0))


class ModelGateTest(unittest.TestCase):

    def test_model_raises_a_quiet_rule(self):
        """규칙이 조용할 때 모델이 경보하면 레벨이 올라간다."""
        result = gate(proba=0.9).apply(rule_level=0, now=1.0, states=STATES)

        self.assertEqual(result.level, MODEL_ALARM_LEVEL)
        self.assertEqual(result.source, "model")

    def test_model_never_lowers_the_rule(self):
        """모델이 안전하다고 해도 규칙이 올린 레벨은 내려가지 않는다."""
        result = gate(proba=0.01).apply(rule_level=2, now=1.0, states=STATES)

        self.assertEqual(result.level, 2)
        self.assertEqual(result.source, "table")

    def test_safety_floor_outranks_the_model(self):
        """하한이 레벨 3을 냈으면 모델이 뭐라 하든 3이다."""
        result = gate(proba=0.0).apply(rule_level=3, now=1.0, states=STATES)

        self.assertEqual(result.level, 3)

    def test_without_a_model_nothing_changes(self):
        """모델이 없으면 규칙 레벨이 그대로 나간다."""
        result = gate(proba=None).apply(rule_level=1, now=1.0, states=STATES)

        self.assertEqual(result.level, 1)
        self.assertIsNone(result.proba)
        self.assertEqual(result.source, "table")

    def test_probability_is_reported_for_logging(self):
        """판정과 무관하게 확률은 기록된다 - 나중에 규칙과 비교해야 한다."""
        result = gate(proba=0.42).apply(rule_level=0, now=1.0, states=STATES)

        self.assertAlmostEqual(result.proba, 0.42)

    def test_threshold_decides_the_alarm(self):
        """임계값 경계에서 판정이 갈린다."""
        below = gate(proba=0.49, threshold=0.5).apply(rule_level=0, now=1.0, states=STATES)
        at = gate(proba=0.50, threshold=0.5).apply(rule_level=0, now=1.0, states=STATES)

        self.assertEqual(below.level, 0)
        self.assertEqual(at.level, MODEL_ALARM_LEVEL)

    def test_missing_states_skip_the_model(self):
        """필터가 아직 준비되지 않았으면 모델을 건너뛴다."""
        result = gate(proba=0.9).apply(rule_level=1, now=1.0, states=None)

        self.assertEqual(result.level, 1)
        self.assertIsNone(result.proba)

    def test_model_failure_does_not_break_the_pipeline(self):
        """모델이 예외를 던져도 규칙 판정은 나간다.

        경보 경로에 있는 코드라, 모델의 문제로 시스템 전체가 멈추면 안 된다.
        """
        class Broken(StubModel):
            def predict_proba(self, feature_map):
                raise ValueError("boom")

        broken = ModelGate(model=Broken(0.9), streamer=StubStreamer())

        result = broken.apply(rule_level=1, now=1.0, states=STATES)

        self.assertEqual(result.level, 1)
        self.assertIsNone(result.proba)


if __name__ == "__main__":
    unittest.main()
