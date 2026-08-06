#!/usr/bin/env python3
"""스트리밍 피처가 학습 때와 같은 값을 내는지 검증.

모델은 ttc_study/features.py가 배열로 만든 피처로 학습했다. 젯슨은 한 시점씩
흘러오므로 같은 값을 다른 방식으로 계산해야 하는데, 하나라도 어긋나면 학습한
적 없는 입력을 넣는 셈이 되어 시뮬레이션에서 측정한 성능이 무의미해진다.
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "ttc_study"))

from model_features import FEATURE_ORDER, StreamingFeatures  # noqa: E402  (03_jetson)


class StreamingMatchesBatchTest(unittest.TestCase):
    """같은 궤적을 배치와 스트리밍으로 계산해 값이 일치하는지 본다."""

    def _batch(self, scenario):
        from features import build_features
        return build_features(scenario, gps_sigma_m=0.0)

    def _stream(self, scenario):
        """참값 궤적을 한 시점씩 먹인다.

        노이즈를 끄면 features.py도 참값 속도를 그대로 쓰므로, 칼만을 거치지 않은
        같은 입력으로 두 경로를 비교할 수 있다.
        """
        streamer = StreamingFeatures()
        rows = []
        for i, t in enumerate(scenario.t):
            rows.append(streamer.update(
                float(t),
                cane_state=(float(scenario.ped.x[i]), float(scenario.ped.y[i]),
                            float(scenario.ped.vx[i]), float(scenario.ped.vy[i])),
                veh_state=(float(scenario.veh.x[i]), float(scenario.veh.y[i]),
                           float(scenario.veh.vx[i]), float(scenario.veh.vy[i])),
            ))
        return rows

    def _scenario(self, **overrides):
        from scenario_sim import ScenarioParams, simulate
        base = dict(
            approach_deg=30.0, start_distance_m=60.0, veh_speed_mps=10.0,
            miss_offset_m=2.0, ped_speed_mps=1.0, ped_heading_deg=100.0,
            veh_accel_mps2=-1.0, turn_rate_dps=8.0,
        )
        base.update(overrides)
        return simulate(ScenarioParams(**base), dt=0.1, duration_s=8.0)

    def test_all_features_match_the_batch_path(self):
        scenario = self._scenario()
        batch = self._batch(scenario)
        stream = self._stream(scenario)

        for name in FEATURE_ORDER:
            for i, row in enumerate(stream):
                self.assertAlmostEqual(
                    row[name], float(batch[name][i]), places=6,
                    msg=f"{name} 가 {i}번째 시점에서 어긋난다",
                )

    def test_matches_for_a_head_on_approach(self):
        """통과 순간의 특이점까지 같게 처리되는지 확인한다."""
        scenario = self._scenario(miss_offset_m=0.0, turn_rate_dps=0.0,
                                  veh_accel_mps2=0.0, ped_speed_mps=0.0)
        batch = self._batch(scenario)
        stream = self._stream(scenario)

        for name in ("d_closing_dt", "d_heading_dt", "ttc", "dcpa_m"):
            for i, row in enumerate(stream):
                self.assertAlmostEqual(row[name], float(batch[name][i]), places=6,
                                       msg=f"{name} @ {i}")

    def test_matches_for_a_stationary_vehicle(self):
        """정지 차량처럼 경계에 놓인 경우에도 같다."""
        scenario = self._scenario(veh_speed_mps=0.0, veh_accel_mps2=0.0,
                                  turn_rate_dps=0.0)
        batch = self._batch(scenario)
        stream = self._stream(scenario)

        for name in FEATURE_ORDER:
            for i, row in enumerate(stream):
                self.assertAlmostEqual(row[name], float(batch[name][i]), places=6,
                                       msg=f"{name} @ {i}")


class StreamingBehaviourTest(unittest.TestCase):

    def test_first_sample_has_zero_rates(self):
        """첫 시점은 이전 값이 없으므로 변화율이 0이다."""
        streamer = StreamingFeatures()

        row = streamer.update(0.0, (0.0, 0.0, 0.0, 0.0), (0.0, 50.0, 0.0, -10.0))

        self.assertEqual(row["d_closing_dt"], 0.0)
        self.assertEqual(row["d_heading_dt"], 0.0)

    def test_every_declared_feature_is_produced(self):
        streamer = StreamingFeatures()

        row = streamer.update(0.0, (0.0, 0.0, 0.0, 0.0), (0.0, 50.0, 0.0, -10.0))

        self.assertEqual(set(row), set(FEATURE_ORDER))

    def test_values_stay_finite(self):
        """겹친 위치처럼 극단적 입력에서도 NaN/inf가 나오지 않는다."""
        streamer = StreamingFeatures()

        row = streamer.update(0.0, (0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))

        for name, value in row.items():
            self.assertTrue(math.isfinite(value), f"{name} = {value}")

    def test_uses_no_third_party_import(self):
        """젯슨에서 돌아야 하므로 표준 라이브러리와 기존 파이프라인만 쓴다.

        step6_kinematics는 젯슨에 이미 있고 그 자체가 numpy를 쓰지 않으므로
        (필터를 순수 파이썬으로 구현해 둔 이유가 그것이다) 의존해도 된다.
        """
        import ast

        src = (Path(__file__).resolve().parent / "model_features.py"
               ).read_text(encoding="utf-8")
        imported = set()
        for node in ast.parse(src).body:
            if isinstance(node, ast.Import):
                imported.update(a.name.split(".")[0] for a in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module.split(".")[0])

        allowed = {"math", "json", "pathlib", "typing", "step6_kinematics"}
        self.assertLessEqual(imported, allowed)


if __name__ == "__main__":
    unittest.main()
