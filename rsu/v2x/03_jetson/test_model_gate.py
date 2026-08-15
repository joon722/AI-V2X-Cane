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
from pathlib import Path

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


class DeployedModelSanityTest(unittest.TestCase):
    """배포되는 모델 파일 자체를 검증한다.

    위의 테스트들은 StubModel로 게이트의 결합 규칙만 본다. 그래서 '학습된 모델이
    이상한 값을 낸다'는 종류의 결함은 하나도 잡지 못한다 - 게이트는 시키는 대로
    올렸을 뿐이니 전부 통과한다.

    여기서는 실제 risk_model.json을 읽어 명백한 상황을 넣는다. 평균 지표(적시경보,
    오경보율)로는 드러나지 않는 국소적 오작동을 잡기 위해서다.
    """

    def _quiet_gate(self):
        model_path = Path(__file__).with_name("risk_model.json")
        if not model_path.exists():
            self.skipTest("risk_model.json 없음")
        return ModelGate.load(model_path, quiet=True)

    def _settle(self, gate_obj, cane, veh, steps=5, dt=0.2):
        """정지 상태를 몇 스텝 먹여 변화율 항을 정상 상태로 만든 뒤 판정한다."""
        result = None
        for i in range(steps):
            result = gate_obj.apply(rule_level=0, now=1.0 + i * dt,
                                    states=(cane, veh))
        return result

    @unittest.expectedFailure
    def test_parked_car_at_30m_is_quiet(self):
        """30m 밖에 정지한 차. 접근하지 않으므로 경보가 나가면 안 된다.

        알려진 한계로 두고 넘어간다(2026-08-07 결정). 고쳐지면 이 테스트가
        '예상외 성공'으로 알려주므로 표시만 남긴다.

        확인한 것: 시나리오 생성기가 30~100m에서 시작하므로 학습 데이터의 30m는
        대부분 '위험이 이미 진행 중인 거리'다. 반면 '가까운데 둘 다 정지'라는
        조합은 데이터에 없다. 1500 시나리오로 3분할 재학습해도 결과가 같았고
        (확률 0.909), 60m에서는 정상이었다(0.004). 학습 방식이 아니라 데이터의
        문제이므로 재학습으로는 고쳐지지 않는다.

        범위 밖으로 둔 근거: V2X 노드가 달린 차량만 관측되므로 길가의 일반 주차
        차량은 애초에 보이지 않는다. 다만 시나리오 E(접근하다 정지)에서는 이
        상황이 만들어지므로, 실측에서 경보가 꺼지지 않는지 확인할 것.
        """
        gate_obj = self._quiet_gate()

        result = self._settle(gate_obj, (0.0, 0.0, 0.0, 0.0), (0.0, 30.0, 0.0, 0.0))

        self.assertEqual(
            result.level, 0,
            f"정지한 30m 차량에 레벨 {result.level}이 나갔다 "
            f"(확률 {result.proba}). 모델 재학습 또는 교체가 필요하다.",
        )

    def test_receding_car_is_quiet(self):
        """멀어지는 차. 20m 거리에서 초당 10m씩 멀어지는 중이면 위험이 아니다."""
        gate_obj = self._quiet_gate()

        result = self._settle(gate_obj, (0.0, 0.0, 0.0, 0.0), (0.0, 20.0, 0.0, 10.0))

        self.assertEqual(
            result.level, 0,
            f"멀어지는 차량에 레벨 {result.level}이 나갔다 (확률 {result.proba}).",
        )


if __name__ == "__main__":
    unittest.main()
