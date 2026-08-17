#!/usr/bin/env python3
"""Tests for step 7 risk scoring: vendored team table + DCPA suppression gate."""

import unittest

from step6_kinematics import relative_kinematics
from step7_risk import (
    DCPA_FAR_M,
    DCPA_FLOOR,
    DCPA_NEAR_M,
    MIN_CLOSING_MPS,
    T_ALARM_MAX_TTC_S,
    T_FLOOR_TTC_S,
    assess_risk,
    calculate_risk_score,
    calculate_ttc,
    classify_risk_level,
    dcpa_gate,
)


class VendoredTeamTableTest(unittest.TestCase):
    """The three scoring functions are a verbatim copy of the team table.

    These are frozen regression values computed from that table, so a change to
    the copied numbers is caught here rather than silently diverging.
    """

    def test_close_head_on_scores_level_3(self):
        ttc = calculate_ttc(8.0, 5.0)  # 1.6 s
        score = calculate_risk_score(8.0, 5.0, 5.0, ttc)
        self.assertEqual(score, 71.0)  # 30 + 30 + 8 + 3
        self.assertEqual(classify_risk_level(score), 3)

    def test_distant_slow_approach_scores_level_1(self):
        ttc = calculate_ttc(50.0, 5.0)  # 10 s
        score = calculate_risk_score(50.0, 5.0, 5.0, ttc)
        self.assertEqual(score, 29.0)  # 10 + 8 + 8 + 3
        self.assertEqual(classify_risk_level(score), 1)

    def test_fast_close_approach_saturates_high(self):
        ttc = calculate_ttc(8.0, 25.0)  # 0.32 s
        score = calculate_risk_score(8.0, 25.0, 25.0, ttc)
        self.assertEqual(score, 95.0)  # 30 + 35 + 20 + 10
        self.assertEqual(classify_risk_level(score), 3)

    def test_stationary_vehicle_scores_only_distance(self):
        ttc = calculate_ttc(8.0, 0.0)  # 9999, no approach
        score = calculate_risk_score(8.0, 0.0, 0.0, ttc)
        self.assertEqual(score, 30.0)  # distance only
        self.assertEqual(classify_risk_level(score), 1)

    def test_ttc_is_infinite_when_not_approaching(self):
        self.assertEqual(calculate_ttc(10.0, 0.0), 9999.0)
        self.assertEqual(calculate_ttc(10.0, -3.0), 9999.0)


class DcpaGateTest(unittest.TestCase):
    def test_missing_dcpa_does_not_suppress(self):
        # A receding or stationary pair has no closest approach ahead; there is
        # nothing to gate, so the team table's own low score stands.
        self.assertEqual(dcpa_gate(None), 1.0)

    def test_dead_centre_is_not_suppressed(self):
        self.assertEqual(dcpa_gate(0.0), 1.0)

    def test_inside_the_near_margin_is_not_suppressed(self):
        self.assertEqual(dcpa_gate(DCPA_NEAR_M), 1.0)

    def test_beyond_the_far_margin_is_floored(self):
        self.assertEqual(dcpa_gate(DCPA_FAR_M), DCPA_FLOOR)
        self.assertEqual(dcpa_gate(100.0), DCPA_FLOOR)

    def test_midpoint_interpolates_linearly(self):
        midpoint = (DCPA_NEAR_M + DCPA_FAR_M) / 2
        expected = 1.0 + 0.5 * (DCPA_FLOOR - 1.0)
        self.assertAlmostEqual(dcpa_gate(midpoint), expected, places=9)

    def test_gate_is_monotonically_non_increasing(self):
        values = [dcpa_gate(d) for d in [0.0, 1.0, 2.5, 4.0, 5.0, 6.0, 7.5, 20.0]]
        for earlier, later in zip(values, values[1:]):
            self.assertGreaterEqual(earlier, later)


class AssessRiskTest(unittest.TestCase):
    def test_head_on_collision_course_is_not_gated_and_stays_high(self):
        """Vehicle 8 m due north, driving straight in. dcpa=0 -> gate is a no-op."""
        kin = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 8.0),
            veh_vel=(0.0, -5.0),
        )
        result = assess_risk(kin, vehicle_speed_mps=5.0)
        self.assertEqual(result.gate, 1.0)
        self.assertEqual(result.final_score, result.base_score)
        self.assertEqual(result.risk_level, 3)

    def test_wide_glancing_pass_is_suppressed_below_the_head_on_case(self):
        """Same speed and similar range, but the car misses by 9 m.

        Without the gate this scores level 2 on the team table; the gate must
        pull it down because the vehicle never actually reaches the cane.
        """
        kin = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(9.0, 8.0),
            veh_vel=(0.0, -5.0),
        )
        self.assertGreaterEqual(kin.dcpa, DCPA_FAR_M)  # a clear miss

        result = assess_risk(kin, vehicle_speed_mps=5.0)
        self.assertEqual(result.gate, DCPA_FLOOR)
        self.assertEqual(classify_risk_level(result.base_score), 2)  # ungated
        self.assertLess(result.risk_level, 2)  # gated down
        self.assertAlmostEqual(result.final_score, result.base_score * DCPA_FLOOR, places=2)

    def test_receding_vehicle_has_no_gate_and_is_not_an_alarm(self):
        kin = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 8.0),
            veh_vel=(0.0, 5.0),
        )
        self.assertIsNone(kin.dcpa)
        result = assess_risk(kin, vehicle_speed_mps=5.0)
        self.assertEqual(result.gate, 1.0)
        # 8/12 이전에는 거리 항(8 m -> 30점)만으로 레벨 1이 나갔다. 그 규칙이
        # 그날 경보 1004건 중 506건(접근하지 않는 쌍)을 만들었다. 지금은 TTC
        # 상한이 걸러낸다 - 점수는 기록용으로 남고 레벨만 0이 된다.
        self.assertEqual(result.risk_level, 0)
        self.assertEqual(result.reason, "ttc_capped")
        self.assertEqual(result.final_score, 33.0)  # 거리 30 + 차량속도 3


class TtcCapTest(unittest.TestCase):
    """접근이 무의미하게 먼 쌍은 경보하지 않는다.

    8/12 실측: 경보 1004건 중 접근 안 함(TTC 9999) 50.4% + TTC 30초 초과
    23.6% = 74%가 위험이 아닌데 울렸다. 반대로 실제 접근 구간의 TTC는
    3.7~9.1초로 전부 30초 안이었다 - 상한 30초는 진짜 위험을 하나도 걸지
    않으면서 그 74%를 걸러낸다.

    상한은 규칙 레벨에만 걸린다. 모델은 여전히 위로 올릴 수 있다 - TTC가
    무한대인 측면 위험을 잡는 것이 모델의 존재 이유라서, 모델까지 막으면
    모델을 두는 의미가 없다. 안전 하한(TTC<=T_FLOOR)은 상한보다 먼저
    판정되므로 서로 겹칠 일이 없다.
    """

    def test_stationary_pair_nearby_is_silent(self):
        # 8/12에 가장 흔했던 오경보: 둘 다 정지, 7 m -> 레벨 1 진동.
        kin = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 7.0),
            veh_vel=(0.0, 0.0),
        )
        result = assess_risk(kin, vehicle_speed_mps=0.0)
        self.assertEqual(result.risk_level, 0)
        self.assertEqual(result.reason, "ttc_capped")

    def test_very_slow_convergence_beyond_the_cap_is_silent(self):
        # 50 m 를 1 m/s 로 접근 -> TTC 50초. 어제 지도에 51초짜리가 올라갔었다.
        kin = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 50.0),
            veh_vel=(0.0, -1.0),
        )
        self.assertGreater(kin.ttc_simple, T_ALARM_MAX_TTC_S)
        result = assess_risk(kin, vehicle_speed_mps=1.0)
        self.assertEqual(result.risk_level, 0)
        self.assertEqual(result.reason, "ttc_capped")

    def test_real_approach_inside_the_cap_is_untouched(self):
        # 50 m 를 10 m/s 로 접근 -> TTC 5초. 기존 채점 그대로.
        kin = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 50.0),
            veh_vel=(0.0, -10.0),
        )
        self.assertLess(kin.ttc_simple, T_ALARM_MAX_TTC_S)
        result = assess_risk(kin, vehicle_speed_mps=10.0)
        self.assertEqual(result.reason, "table")
        self.assertEqual(result.risk_level, classify_risk_level(result.final_score))
        self.assertGreaterEqual(result.risk_level, 1)

    def test_safety_floor_wins_over_the_cap(self):
        # 하한 안쪽이면 상한 판정 자체가 일어나지 않는다.
        kin = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 10.0),
            veh_vel=(0.0, -10.0),
        )
        self.assertLessEqual(kin.ttc_simple, T_FLOOR_TTC_S)
        result = assess_risk(kin, vehicle_speed_mps=10.0)
        self.assertEqual(result.risk_level, 3)
        self.assertEqual(result.reason, "safety_floor")

    def test_cap_can_be_disabled_for_analysis(self):
        """상한을 끈 채로 채점할 수 있어야 8/12 이전 데이터와 비교가 된다."""
        kin = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 8.0),
            veh_vel=(0.0, 5.0),
        )
        result = assess_risk(kin, vehicle_speed_mps=5.0, alarm_max_ttc_s=0.0)
        self.assertEqual(result.risk_level, 1)
        self.assertEqual(result.reason, "table")

    def test_cap_value_matches_the_pipeline_bound(self):
        """지도 업로드(upload_events.MAX_TTC_S)와 같은 30초여야 한다.

        판단에 의미가 있는 TTC 구간이 30초까지라는 결정(model_features.
        TTC_MAX_S)을 세 곳이 공유한다. 갈라지면 화면·지도·경보가 서로
        다른 기준으로 움직인다.
        """
        self.assertEqual(T_ALARM_MAX_TTC_S, 30.0)


class ClosingDeadbandTest(unittest.TestCase):
    """GPS 잡음 크기의 접근속도는 접근이 아니다.

    8/17 오후 실측: 두 노드가 모두 서 있는데 KF 접근속도가 |중앙 0.04~0.19,
    p90 0.22~0.7| m/s로 흔들렸다(패킷 좌표 float32 격자 0.42/0.53 m + GPS σ).
    10 m 안에서는 그 잡음이 TTC 30초 경계(0.33 m/s)와 8초 경계(1.25 m/s)를
    넘나들며 레벨 0↔1↔2를 깜빡이게 했다. 0.5 m/s 아래 접근속도는 잡음이므로
    TTC를 내지 않는다. 진짜 접근(RC 1 m/s+, 실차 3 m/s+)은 그대로 통과하고,
    코앞(near_floor)은 거리 규칙이 따로 지킨다.
    """

    def _still_pair_with_noise_closing(self, distance_m, closing_mps):
        # 접근속도만 잡음으로 주어진 정지 쌍.
        return relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, distance_m),
            veh_vel=(0.0, -closing_mps),
        )

    def test_noise_level_closing_speed_gives_no_ttc_and_no_alarm(self):
        # 8 m, closing 0.3 m/s -> TTC 26.7초로 상한 안이라 레벨 1이 나던 경우.
        kin = self._still_pair_with_noise_closing(8.0, 0.3)
        result = assess_risk(kin, vehicle_speed_mps=0.1)
        self.assertEqual(result.ttc, 9999.0)
        self.assertEqual(result.risk_level, 0)
        self.assertEqual(result.reason, "ttc_capped")

    def test_the_measured_closing_speed_is_still_logged(self):
        kin = self._still_pair_with_noise_closing(8.0, 0.3)
        result = assess_risk(kin, vehicle_speed_mps=0.1)
        self.assertAlmostEqual(result.closing_los, 0.3, places=6)

    def test_closing_at_the_deadband_passes(self):
        # 8 m, closing 0.5 m/s -> TTC 16초, 상한 안 -> 점수표대로 레벨 1.
        kin = self._still_pair_with_noise_closing(8.0, MIN_CLOSING_MPS)
        result = assess_risk(kin, vehicle_speed_mps=0.5)
        self.assertAlmostEqual(result.ttc, 16.0, places=6)
        self.assertEqual(result.reason, "table")
        self.assertGreaterEqual(result.risk_level, 1)

    def test_near_floor_is_unaffected_by_the_deadband(self):
        kin = self._still_pair_with_noise_closing(2.5, 0.3)
        result = assess_risk(kin, vehicle_speed_mps=0.1)
        self.assertEqual(result.risk_level, 2)
        self.assertEqual(result.reason, "near_floor")

    def test_deadband_can_be_disabled_for_analysis(self):
        kin = self._still_pair_with_noise_closing(8.0, 0.3)
        result = assess_risk(kin, vehicle_speed_mps=0.1, min_closing_mps=0.0)
        self.assertAlmostEqual(result.ttc, 8.0 / 0.3, places=6)
        self.assertEqual(result.reason, "table")


class SafetyFloorTest(unittest.TestCase):
    """TTC가 반응 가능 시간 아래로 내려가면 점수와 무관하게 최고 레벨이 나간다.

    근거는 ttc_study/SPEC.md에 정리되어 있다. T_FLOOR_TTC_S는 GPS 주기 + 전송
    0.1(8/5 실측) + 인지·판단 0.3 + 정지 동작 1.2 + 마진 0.2의 합이다.

    2026-08-08 실측으로 GPS 주기를 1.0초(차량 1045 ms)로 잡으면서 하한이
    2.0 -> 2.8초가 되었다. 아래 테스트가 상수를 직접 참조하는 것은 이 때문이다 -
    값을 적어두면 GPS 주기가 바뀔 때마다 테스트가 함께 틀린다.

    시뮬레이션 검증(1200 x 3회, 평가 기준 2.8초): 하한 2.0은 하한 없음과 적시경보가
    52.2%로 같아 오경보만 늘린다. 2.8초에서 57.7%로 오르고 오경보는 3.36 ->
    4.75%가 된다. 놓치지 않는 쪽에 비용을 쓰는 것이 이 규칙의 목적이다.
    """

    def test_imminent_collision_forces_top_level(self):
        # 10 m 앞에서 10 m/s로 정면 접근 -> TTC 1.0초
        kin = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 10.0),
            veh_vel=(0.0, -10.0),
        )
        self.assertLessEqual(kin.ttc_simple, T_FLOOR_TTC_S)

        result = assess_risk(kin, vehicle_speed_mps=10.0)

        self.assertEqual(result.risk_level, 3)

    def test_floor_overrides_the_dcpa_gate(self):
        """게이트가 점수를 깎아도 하한은 무시하고 발동한다.

        게이트는 '스쳐 갈 차'를 걸러내는 장치인데, 그 판단 자체가 추정이다.
        접촉까지 2초도 남지 않은 상황에서는 추정이 틀렸을 때의 대가가 너무 크다.
        """
        kin = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(8.0, 6.0),
            veh_vel=(-4.0, -3.0),
        )
        self.assertLessEqual(kin.ttc_simple, T_FLOOR_TTC_S)

        result = assess_risk(kin, vehicle_speed_mps=5.0)

        self.assertEqual(result.risk_level, 3)
        self.assertEqual(result.reason, "safety_floor")

    def test_above_the_floor_nothing_changes(self):
        """하한 위에서는 기존 채점이 그대로다."""
        kin = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 50.0),
            veh_vel=(0.0, -10.0),
        )
        self.assertGreater(kin.ttc_simple, T_FLOOR_TTC_S)

        result = assess_risk(kin, vehicle_speed_mps=10.0)

        self.assertEqual(result.risk_level, classify_risk_level(result.final_score))
        self.assertEqual(result.reason, "table")

    def test_receding_vehicle_never_triggers_the_floor(self):
        """멀어지는 차는 TTC가 없으므로 하한이 걸리지 않는다."""
        kin = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 3.0),
            veh_vel=(0.0, 5.0),
        )
        self.assertIsNone(kin.ttc_simple)

        result = assess_risk(kin, vehicle_speed_mps=5.0)

        self.assertNotEqual(result.reason, "safety_floor")

    def test_floor_can_be_disabled_for_analysis(self):
        """하한을 끈 채로도 채점할 수 있어야 전후 비교가 가능하다."""
        kin = relative_kinematics(
            cane_pos=(0.0, 0.0),
            cane_vel=(0.0, 0.0),
            veh_pos=(0.0, 10.0),
            veh_vel=(0.0, -10.0),
        )

        result = assess_risk(kin, vehicle_speed_mps=10.0, floor_ttc_s=0.0)

        self.assertEqual(result.reason, "table")


if __name__ == "__main__":
    unittest.main()
