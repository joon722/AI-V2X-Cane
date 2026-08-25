#!/usr/bin/env python3
"""step8 UWB 통합: 전용 type="uwb" 행 → 거리축 교체(실외) + UWB 단독(실내).

스펙(설명서/UWB통합_스펙_20260822.md §5) 요지:
- UWB는 rssi_dist가 아니라 전용 행으로 온다(§0.5 채널 분리). step8의 RSSI
  융합 게이트(12 m 상한 등)는 RSSI 튜닝이라 UWB가 같은 필드로 섞이면 출처
  구분이 안 돼 오적용 사고가 난다.
- 실외(GPS 유효): 채점 거리·접근속도만 UWB로 교체, 방향·지도는 GPS 유지.
  stale이면 자동으로 GPS 폴백(기존 v4 그대로 = 후퇴선).
- 실내(양쪽 GPS 무효): pair 신선(위치 무관) + UWB 신선·보정 완료일 때
  거리/TTC 밴드만 판정하고, consider(uwb_trusted=True)로 무효 fix에서도
  경보를 올린다. 모델 게이트는 미적용(레벨 = 규칙).
- --no-uwb: 비상 차단. 융합·실내 모두 무동작.

템플릿: test_step8_send_risk.py의 DistanceTrendRssiFusionTest (실제 파이프라인).
"""

import contextlib
import io
import json
import sys
import unittest
from unittest import mock

import step8_send_risk as s8
from step3_parse_v2x import normalize_record
from step4_state_store import FRESH_WINDOW_S, StateStore
from step6_kinematics import KinematicsPipeline
from step7_risk import DCPA_FAR_M, DCPA_FLOOR, DCPA_NEAR_M, T_FLOOR_TTC_S
from step8_send_risk import RiskTransmitter, UWB_FRESH_S, parse_args
from step8_stability import LevelStabilizer

CANE_LAT, CANE_LNG = 37.496330, 126.958313
DEG_PER_M = 1.0 / 111320.0


def uwb_line(dist, closing=0.0, calib=1, seq=0):
    """브리지 v5가 MSG_UWB_RANGE(38B)에서 만드는 JSON 모양(스펙 §4)."""
    return json.dumps({
        "type": "uwb", "node_id": 305419896, "seq": seq,
        "uwb_dist": round(dist, 3), "uwb_raw": round(dist, 3),
        "uwb_closing": round(closing, 3), "uwb_calib": calib,
        "gps_valid": 0, "tx_ms": 0, "rx_ms": 0, "rssi": -60,
    })


def cane_line(seq, gps_valid=1, lat=CANE_LAT, lng=CANE_LNG):
    return json.dumps({
        "type": "cane", "node_id": 1, "seq": seq, "gps_valid": gps_valid,
        "heading_valid": 0, "lat": lat, "lng": lng,
        "speed_mps": 0.02, "heading_deg": 0.0, "node_risk": 0,
    })


def vehicle_line(seq, north_m, gps_valid=1, speed_mps=0.03):
    lat = CANE_LAT + north_m * DEG_PER_M if gps_valid else 0.0
    lng = CANE_LNG if gps_valid else 0.0
    return json.dumps({
        "type": "vehicle", "node_id": 2, "seq": seq, "gps_valid": gps_valid,
        "heading_valid": 0, "lat": lat, "lng": lng,
        "speed_mps": speed_mps, "heading_deg": 180.0, "node_risk": 0,
    })


class UwbPipelineCase(unittest.TestCase):
    """실제 파이프라인(store→KF→assess)으로 도는 공통 뼈대."""

    def setUp(self):
        self.rows = []
        self.sent = []
        self.clock = {"now": 1000.0}
        self._orig_norm, self._orig_append = s8.normalize_record, s8.append_row
        s8.normalize_record = lambda p, m: normalize_record(p, m, now=self.clock["now"])
        s8.append_row = lambda path, row: self.rows.append(row)
        self.addCleanup(self._restore)
        quiet = contextlib.redirect_stdout(io.StringIO())
        quiet.__enter__()
        self.addCleanup(quiet.__exit__, None, None, None)

    def _restore(self):
        s8.normalize_record, s8.append_row = self._orig_norm, self._orig_append

    def make_sender(self, **kwargs):
        return s8.RiskSender(
            KinematicsPipeline(StateStore(fresh_window_s=FRESH_WINDOW_S)),
            RiskTransmitter(target_id=0, heartbeat_s=0.2),
            transport=self.sent.append, csv_path=None,
            gate_params={"near_m": DCPA_NEAR_M, "far_m": DCPA_FAR_M,
                         "floor": DCPA_FLOOR, "floor_ttc_s": T_FLOOR_TTC_S},
            stabilizer=LevelStabilizer(),
            **kwargs,
        )

    def run_outdoor(self, sender, uwb_mode, cycles=25):
        """GPS 유효, 차량은 10 m 북쪽에 정지. uwb_mode: 'fresh'(매 사이클) |
        'once'(첫 사이클만 → 이후 stale) | None(전무)."""
        for i in range(cycles):
            self.clock["now"] = 1000.0 + i * 0.2
            sender.process_line(cane_line(i), "real")
            self.clock["now"] += 0.05
            if uwb_mode == "fresh" or (uwb_mode == "once" and i == 0):
                sender.process_line(uwb_line(6.0, 0.0, seq=i), "real")
            self.clock["now"] += 0.05
            sender.process_line(vehicle_line(i, north_m=10.0), "real")

    def run_indoor(self, sender, start_m, step_m, closing, cycles=35):
        """양쪽 GPS 무효(lat=lng=0) + 5 Hz UWB 시퀀스."""
        for i in range(cycles):
            self.clock["now"] = 1000.0 + i * 0.2
            dist = max(2.0, start_m + i * step_m)
            sender.process_line(uwb_line(dist, closing, seq=i), "real")
            self.clock["now"] += 0.05
            sender.process_line(cane_line(i, gps_valid=0, lat=0.0, lng=0.0), "real")
            self.clock["now"] += 0.05
            sender.process_line(vehicle_line(i, 0.0, gps_valid=0), "real")

    @staticmethod
    def risk(cmd):
        return json.loads(cmd)["risk"]


class UwbRecordFreshnessTest(UwbPipelineCase):
    """케이스 1: uwb 행 기록·신선도·calib=0/stale 무시, store 미등록(경고 없음)."""

    def test_uwb_row_is_recorded_without_ignored_type_warning(self):
        sender = self.make_sender()
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            sender.process_line(uwb_line(5.0, 0.4), "real")
        self.assertNotIn("ignored_type", err.getvalue())
        self.assertEqual(sender._uwb_fresh(1000.5), (5.0, 0.4))

    def test_stale_uwb_is_not_used(self):
        sender = self.make_sender()
        sender.process_line(uwb_line(5.0, 0.4), "real")
        self.assertIsNone(sender._uwb_fresh(1000.0 + UWB_FRESH_S + 0.1))

    def test_uncalibrated_uwb_is_not_used(self):
        sender = self.make_sender()
        sender.process_line(uwb_line(5.0, 0.4, calib=0), "real")
        self.assertIsNone(sender._uwb_fresh(1000.1))

    def test_nonpositive_distance_is_ignored(self):
        sender = self.make_sender()
        sender.process_line(uwb_line(0.0, 0.4), "real")
        self.assertIsNone(sender._uwb_fresh(1000.1))


class OutdoorFusionTest(UwbPipelineCase):
    """케이스 2: GPS 유효 + uwb fresh → 채점 거리 = UWB / stale → GPS 폴백."""

    def test_distance_axis_is_replaced_while_uwb_fresh(self):
        self.run_outdoor(self.make_sender(), "fresh")
        fused = [r for r in self.rows if r["distance_source"] == "uwb"]
        self.assertTrue(fused, "uwb fresh인데 distance_source=uwb 행이 없다")
        self.assertTrue(all(abs(float(r["distance_m"]) - 6.0) < 0.01 for r in fused),
                        [(r["distance_m"], r["distance_source"]) for r in fused[:5]])

    def test_stale_uwb_falls_back_to_gps_distance(self):
        self.run_outdoor(self.make_sender(), "once")
        late = self.rows[-5:]
        self.assertTrue(late)
        self.assertTrue(all(r["distance_source"] == "gps" for r in late),
                        [(r["distance_m"], r["distance_source"]) for r in late])
        self.assertTrue(all(float(r["distance_m"]) > 8.0 for r in late),
                        [r["distance_m"] for r in late])


class IndoorUwbOnlyTest(UwbPipelineCase):
    """케이스 3·4: 실내 — UWB 단독 밴드 판정. 접근이면 경보, 멀어지면 무음."""

    def test_approach_raises_alarm_trusted_and_transmitted(self):
        self.run_indoor(self.make_sender(), start_m=8.0, step_m=-0.2, closing=1.0)
        self.assertTrue(any(self.risk(c) >= 1 for c in self.sent),
                        "실내 UWB 접근인데 경보 송신이 없다")
        self.assertTrue(any(self.risk(c) >= 3 for c in self.sent),
                        "TTC 하한(safety_floor) 구간인데 레벨 3이 없다")
        alarm_rows = [r for r in self.rows if int(r["effective_level"]) >= 1]
        self.assertTrue(alarm_rows, "경보 행이 CSV에 안 남았다")
        self.assertTrue(all(r["distance_source"] == "uwb_only" for r in alarm_rows))
        self.assertTrue(all(int(r["trusted"]) == 1 for r in alarm_rows),
                        "uwb_trusted가 신뢰 게이트를 못 넘었다")
        # 무효 fix(gps_valid=0)에서 신뢰가 UWB로부터 왔음이 행에 그대로 남는다.
        self.assertTrue(all(int(r["cane_gps_valid"]) == 0 for r in alarm_rows))
        # 모델 게이트 미적용: 레벨 = 규칙 그대로.
        self.assertTrue(all(r["level_source"] == "table" for r in alarm_rows))

    def test_receding_uwb_stays_silent(self):
        self.run_indoor(self.make_sender(), start_m=2.0, step_m=0.2, closing=-1.0)
        self.assertTrue(all(self.risk(c) == 0 for c in self.sent),
                        [c for c in self.sent if self.risk(c) != 0][:5])


class NoUwbRowsGapHoldTest(UwbPipelineCase):
    """케이스 5: uwb 행 전무 → 기존 gap-hold 그대로(상세 회귀는 test_step8_gap_hold)."""

    def test_gap_hold_still_carries_the_level(self):
        sender = self.make_sender()
        t = [1000.0]

        def feed(line):
            self.clock["now"] = t[0]
            sender.process_line(line, "test")
            t[0] += 0.5

        # head-on 접근(8 m/s)으로 등급을 올린다 — test_step8_gap_hold와 같은 장면.
        for i, north in enumerate([40, 32, 24, 16, 8, 4, 3, 2]):
            feed(cane_line(100 + i))
            feed(vehicle_line(200 + i, north, speed_mps=8.0))
        pre = self.risk(self.sent[-1])
        self.assertGreaterEqual(pre, 1, "접근에서 경보 등급이 안 올라갔다")
        n = len(self.sent)
        # 차량 무음 2.0 s(< cap 3.0) — uwb가 없으니 gap-hold가 그대로 이어받아야 한다.
        for i in range(4):
            feed(cane_line(130 + i))
        gap = [self.risk(c) for c in self.sent[n:]]
        self.assertGreaterEqual(len(gap), 2, "공백에서 하트비트가 안 나갔다")
        self.assertTrue(all(r == pre for r in gap), (pre, gap))


class ConsiderUwbTrustedTest(unittest.TestCase):
    """케이스 6: consider(uwb_trusted=True) 단독 — 기본값 미전달이면 동작 불변."""

    def test_uwb_trusted_lets_an_untrusted_fix_raise(self):
        tx = RiskTransmitter(heartbeat_s=1.0)
        decision = tx.consider(2, cane_gps_valid=0, now=0.0, uwb_trusted=True)
        self.assertTrue(decision.trusted)
        self.assertEqual(decision.effective_level, 2)

    def test_default_kwarg_keeps_the_old_suppression(self):
        tx = RiskTransmitter(heartbeat_s=1.0)
        decision = tx.consider(2, cane_gps_valid=0, now=0.0)
        self.assertFalse(decision.trusted)
        self.assertEqual(decision.effective_level, 0)


class NoUwbFlagTest(UwbPipelineCase):
    """케이스 7: --no-uwb 비상 차단 — 융합·실내 모두 무동작."""

    def test_uwb_on_by_default(self):
        with mock.patch.object(sys, "argv", ["step8"]):
            self.assertFalse(parse_args().no_uwb)

    def test_no_uwb_flag_is_parsed(self):
        with mock.patch.object(sys, "argv", ["step8", "--no-uwb"]):
            self.assertTrue(parse_args().no_uwb)

    def test_no_uwb_disables_outdoor_fusion(self):
        self.run_outdoor(self.make_sender(uwb_enabled=False), "fresh")
        self.assertTrue(self.rows)
        self.assertTrue(all(r["distance_source"] == "gps" for r in self.rows))
        self.assertTrue(all(float(r["distance_m"]) > 8.0 for r in self.rows),
                        [r["distance_m"] for r in self.rows[:5]])

    def test_no_uwb_disables_indoor_path(self):
        self.run_indoor(self.make_sender(uwb_enabled=False),
                        start_m=8.0, step_m=-0.2, closing=1.0)
        self.assertEqual([c for c in self.sent if self.risk(c) >= 1], [])


if __name__ == "__main__":
    unittest.main()
