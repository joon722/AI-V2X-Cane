#!/usr/bin/env python3
"""Tests for step 8 risk downlink policy: trust gating + on-change/heartbeat."""

import unittest
from pathlib import Path
from types import SimpleNamespace

from step8_send_risk import (
    CSV_FIELDS,
    RawLog,
    RiskTransmitter,
    TxDecision,
    csv_row,
    format_tx,
    load_vehicle_bias,
    vehicle_bias_messages,
    _run,
)


class HotLoopSurvivesRowErrorsTest(unittest.TestCase):
    """한 줄 처리가 실패해도 경보 루프가 계속 돌아야 한다(루프 사망 = 미탐)."""

    def test_a_failing_line_does_not_kill_the_loop(self):
        seen = []

        class FlakySender:
            def process_line(self, line, source_mode):
                seen.append(line)
                if line == "boom":
                    raise RuntimeError("serial write failed")

        _run(FlakySender(), ["a", "boom", "b"], "real", vehicle=None)
        # boom 에서 죽지 않고 그 뒤 "b" 까지 처리했어야 한다.
        self.assertEqual(seen, ["a", "boom", "b"])

    def test_keyboard_interrupt_still_stops(self):
        class StopSender:
            def process_line(self, line, source_mode):
                raise KeyboardInterrupt

        # KeyboardInterrupt 는 삼키지 않고 정상 종료로 빠져나와야 한다(예외 전파 X).
        _run(StopSender(), ["x"], "real", vehicle=None)


class OnChangeAndHeartbeatTest(unittest.TestCase):
    def setUp(self):
        # gps_valid=1 so trust never suppresses; this class is about timing.
        self.tx = RiskTransmitter(heartbeat_s=1.0)

    def test_first_value_is_always_sent(self):
        decision = self.tx.consider(2, cane_gps_valid=1, now=0.0)
        self.assertTrue(decision.should_send)
        self.assertEqual(decision.reason, "change")
        self.assertEqual(decision.effective_level, 2)

    def test_same_level_inside_the_heartbeat_window_is_held(self):
        self.tx.consider(2, cane_gps_valid=1, now=0.0)
        decision = self.tx.consider(2, cane_gps_valid=1, now=0.5)
        self.assertFalse(decision.should_send)
        self.assertEqual(decision.reason, "hold")

    def test_same_level_is_resent_once_the_heartbeat_elapses(self):
        self.tx.consider(2, cane_gps_valid=1, now=0.0)
        decision = self.tx.consider(2, cane_gps_valid=1, now=1.0)
        self.assertTrue(decision.should_send)
        self.assertEqual(decision.reason, "heartbeat")

    def test_a_level_change_is_sent_immediately(self):
        self.tx.consider(2, cane_gps_valid=1, now=0.0)
        decision = self.tx.consider(3, cane_gps_valid=1, now=0.1)
        self.assertTrue(decision.should_send)
        self.assertEqual(decision.reason, "change")
        self.assertEqual(decision.effective_level, 3)


class TrustGatingTest(unittest.TestCase):
    def test_untrusted_position_suppresses_a_nonzero_risk_to_zero(self):
        tx = RiskTransmitter(heartbeat_s=1.0)  # allow_untrusted defaults to False
        decision = tx.consider(2, cane_gps_valid=0, now=0.0)
        self.assertEqual(decision.computed_level, 2)
        self.assertEqual(decision.effective_level, 0)
        self.assertFalse(decision.trusted)

    def test_a_trusted_alarm_is_cleared_when_the_fix_is_lost(self):
        """Sends 2 with a valid fix, then loses the fix: a 0 must go out to stop
        the cane vibrating on a now-untrustworthy alarm."""
        tx = RiskTransmitter(heartbeat_s=1.0)
        first = tx.consider(2, cane_gps_valid=1, now=0.0)
        self.assertEqual(first.effective_level, 2)

        cleared = tx.consider(2, cane_gps_valid=0, now=0.1)
        self.assertTrue(cleared.should_send)
        self.assertEqual(cleared.reason, "change")
        self.assertEqual(cleared.effective_level, 0)

    def test_allow_untrusted_passes_the_computed_level_through(self):
        tx = RiskTransmitter(heartbeat_s=1.0, allow_untrusted=True)
        decision = tx.consider(2, cane_gps_valid=0, now=0.0)
        self.assertTrue(decision.trusted)
        self.assertEqual(decision.effective_level, 2)


class CommandFormatTest(unittest.TestCase):
    def test_command_matches_the_verified_downlink_shape(self):
        tx = RiskTransmitter(target_id=0)
        self.assertEqual(tx.command(2), '{"target_id":0,"risk":2}')

    def test_target_id_is_carried_into_the_command(self):
        tx = RiskTransmitter(target_id=4125577512)
        self.assertEqual(tx.command(1), '{"target_id":4125577512,"risk":1}')


def _store(cane_node_risk, veh_node_risk):
    cane = {
        "seq": 7,
        "node_risk": cane_node_risk,
        "gps_valid": 1,
        "lat": 37.496655,
        "lng": 126.957886,
        "speed_mps": 0.5,
        "heading_deg": 180.0,
    }
    vehicle = {
        "node_risk": veh_node_risk,
        "gps_valid": 1,
        "speed_mps": 5.0,
        "heading_deg": 90.0,
    }
    return SimpleNamespace(latest={"cane": cane, "vehicle": vehicle})


class NodeRiskLoggingTest(unittest.TestCase):
    """The downlink is only verifiable if what the nodes report is logged next to it."""

    def setUp(self):
        self.decision = TxDecision(True, 2, 2, True, "change")
        # Mirrors RiskAssessment's shape, including the fields the model gate
        # logs next to the rule's own verdict.
        self.assessment = SimpleNamespace(
            distance_m=5.0, closing_los=1.2, ttc=4.1, final_score=0.6,
            risk_level=2, reason="table",
        )

    def test_both_node_risks_land_in_the_row(self):
        row = csv_row(
            1.0, _store(2, 1), RiskTransmitter(), self.decision, self.assessment
        )
        self.assertEqual(row["cane_node_risk"], 2)
        self.assertEqual(row["veh_node_risk"], 1)

    def test_row_keys_match_the_header(self):
        """DictWriter raises on any mismatch, so the CSV would break at runtime."""
        row = csv_row(
            1.0, _store(0, 0), RiskTransmitter(), self.decision, self.assessment
        )
        self.assertEqual(set(row), set(CSV_FIELDS))

    def test_tx_line_shows_what_the_nodes_reported(self):
        line = format_tx(self.decision, 2, 0)
        self.assertIn("cane_node_risk=2", line)
        self.assertIn("veh_node_risk=0", line)


class RawLogTest(unittest.TestCase):
    """통신이 의심스러울 때 볼 수 있는 유일한 기록이라, 방향과 순서가 남아야 한다."""

    def test_rx_and_tx_are_interleaved_in_arrival_order(self):
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "raw.log"
            log = RawLog(path)
            log.write("RX", '{"type":"cane","node_risk":0}')
            log.write("TX", '{"target_id":0,"risk":2}')
            log.write("RX", '{"type":"cane","node_risk":2}')
            log.close()

            lines = path.read_text(encoding="utf-8").splitlines()

        self.assertEqual([line.split(" ", 2)[1] for line in lines], ["RX", "TX", "RX"])
        self.assertIn('"risk":2', lines[1])
        # 앞의 시각이 float로 읽혀야 나중에 지연을 계산할 수 있다.
        self.assertLessEqual(
            float(lines[0].split(" ", 1)[0]), float(lines[2].split(" ", 1)[0])
        )

    def test_unparseable_lines_are_kept(self):
        """깨진 줄이야말로 남겨야 원인을 찾을 수 있다."""
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "raw.log"
            log = RawLog(path)
            log.write("RX", "{broken json")
            log.close()
            self.assertIn("{broken json", path.read_text(encoding="utf-8"))


class VehicleBiasLoadTest(unittest.TestCase):
    """calibrate_bias.py 가 남긴 파일을 읽되, 없거나 깨져도 엔진은 떠야 한다."""

    def test_missing_file_means_no_bias(self):
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(
                load_vehicle_bias(Path(tmp) / "none.json"), (0.0, 0.0, None)
            )

    def test_reads_the_calibration_file(self):
        import json
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "vehicle_bias.json"
            path.write_text(json.dumps(
                {"bias_east_m": 6.98, "bias_north_m": -1.35, "created_at": 1000.0}
            ))
            east, north, meta = load_vehicle_bias(path)
            self.assertEqual((east, north), (6.98, -1.35))
            self.assertEqual(meta["created_at"], 1000.0)

    def test_corrupt_file_degrades_to_zero_without_raising(self):
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "vehicle_bias.json"
            path.write_text("{{{ not json")
            self.assertEqual(load_vehicle_bias(path), (0.0, 0.0, None))

    def test_messages_warn_when_the_bias_is_older_than_30_minutes(self):
        fresh = vehicle_bias_messages(6.98, -1.35, {"created_at": 1000.0}, now=1000.0 + 600)
        stale = vehicle_bias_messages(6.98, -1.35, {"created_at": 1000.0}, now=1000.0 + 31 * 60)
        self.assertFalse(any(line.startswith("[WARN]") for line in fresh))
        self.assertTrue(any(line.startswith("[WARN]") for line in stale))
        self.assertTrue(any("+6.98" in line for line in fresh))

    def test_messages_say_so_when_not_calibrated(self):
        lines = vehicle_bias_messages(0.0, 0.0, None, now=0.0)
        self.assertEqual(len(lines), 1)
        self.assertIn("없음", lines[0])


class DopplerBoundWiringTest(unittest.TestCase):
    """step8이 두 노드의 도플러 속도를 채점(step7 클램프)에 넘기는지 — 실제 파이프라인으로.

    8/18 16:44:15 실측 모양: 차량 GPS 위치가 1 m/s로 지팡이 쪽으로 흘러가는데(드리프트)
    차량 도플러는 0.01 m/s. 이전 코드는 KF 접근속도 ≈1 → TTC 6~7 s → 경고. 클램프가
    연결되면 채점 접근속도가 0.33으로 잘려 데드밴드 아래 → 조용해야 한다. 같은 흐름에서
    차량이 1 m/s를 보고하면(진짜 접근) 경보가 나야 하고, 차량 GPS가 무효면 클램프를
    걸지 않아야 한다.
    """

    def _run(self, veh_reported_speed, veh_gps_valid=1, speed_at=None):
        import step8_send_risk as s8
        from step3_parse_v2x import normalize_record
        from step4_state_store import FRESH_WINDOW_S, StateStore
        from step6_kinematics import KinematicsPipeline
        from step7_risk import DCPA_FAR_M, DCPA_FLOOR, DCPA_NEAR_M, T_FLOOR_TTC_S
        from step8_stability import LevelStabilizer
        import json

        rows = []
        clock = {"now": 1000.0}
        orig_norm, orig_append = s8.normalize_record, s8.append_row
        s8.normalize_record = lambda p, m: normalize_record(p, m, now=clock["now"])
        s8.append_row = lambda path, row: rows.append(row)
        try:
            store = StateStore(fresh_window_s=FRESH_WINDOW_S)
            sender = s8.RiskSender(
                KinematicsPipeline(store),
                RiskTransmitter(target_id=0, heartbeat_s=0.2),
                transport=lambda cmd: None, csv_path=None,
                gate_params={"near_m": DCPA_NEAR_M, "far_m": DCPA_FAR_M,
                             "floor": DCPA_FLOOR, "floor_ttc_s": T_FLOOR_TTC_S},
                stabilizer=LevelStabilizer(hold_s=2.0),
                # 이 클래스는 클램프/ZUPT 를 격리 검증한다. 거리추세는 '위치가 꾸준히
                # 준다'를 접근으로 보므로 켜면 이 합성 드리프트를 잡아 반대로 작동한다.
                dist_trend_min_mps=0.0,
            )
            cane_lat, cane_lng = 37.496330, 126.958313
            deg_per_m = 1.0 / 111320.0
            # 차량: 12 m 북쪽에서 시작, 0.2 s마다 0.2 m 남하(=1 m/s 위치 드리프트), 4 m에서 멈춤
            for i in range(60):
                clock["now"] = 1000.0 + i * 0.2
                dist = max(4.0, 12.0 - i * 0.2)
                if speed_at is not None:
                    veh_reported_speed = speed_at(i)
                sender.process_line(json.dumps({
                    "type": "cane", "node_id": 1, "seq": i, "gps_valid": 1, "heading_valid": 0,
                    "lat": cane_lat, "lng": cane_lng, "speed_mps": 0.02, "heading_deg": 0.0,
                    "node_risk": 0}), "real")
                clock["now"] += 0.05
                sender.process_line(json.dumps({
                    "type": "vehicle", "node_id": 2, "seq": i, "gps_valid": veh_gps_valid,
                    "heading_valid": 0,
                    "lat": cane_lat + dist * deg_per_m if veh_gps_valid else 0.0,
                    "lng": cane_lng if veh_gps_valid else 0.0,
                    "speed_mps": veh_reported_speed, "heading_deg": 180.0,
                    "node_risk": 0}), "real")
        finally:
            s8.normalize_record, s8.append_row = orig_norm, orig_append
        return rows

    def test_position_drift_with_a_still_doppler_is_silent(self):
        rows = self._run(veh_reported_speed=0.01)
        far = [r for r in rows if float(r["distance_m"]) > 3.0]
        self.assertGreater(len(far), 20)
        self.assertTrue(all(int(r["effective_level"]) == 0 for r in far),
                        [(r["distance_m"], r["closing_mps"], r["effective_level"]) for r in far
                         if int(r["effective_level"]) > 0][:5])

    def test_the_same_drift_with_a_moving_doppler_alarms(self):
        rows = self._run(veh_reported_speed=1.0)
        self.assertTrue(any(int(r["effective_level"]) >= 1 for r in rows
                            if float(r["distance_m"]) > 3.0))

    def test_an_invalid_vehicle_fix_disables_the_clamp(self):
        # 차량 GPS 무효면 위치가 안 들어와 판정 자체가 드물지만, 나온 판정에서
        # doppler_speeds_mps=None 이 넘어가는지 → assess_risk 호출 인자로 본다.
        import step8_send_risk as s8
        seen = []
        orig = s8.assess_risk
        s8.assess_risk = lambda *a, **k: (seen.append(k), orig(*a, **k))[1]
        try:
            self._run(veh_reported_speed=0.01, veh_gps_valid=1)
            self.assertTrue(seen and all(k.get("doppler_speeds_mps") is not None for k in seen))
            seen.clear()
            self._run(veh_reported_speed=0.01, veh_gps_valid=0)
            self.assertTrue(all(k.get("doppler_speeds_mps") is None for k in seen))
        finally:
            s8.assess_risk = orig

    def test_a_vehicle_that_just_stopped_keeps_its_lagged_approach_for_a_few_seconds(self):
        # 8/18 16:31:44·16:37:47: 위치 기반 접근속도는 도플러보다 2~3 s 늦다. 차가
        # 1.5 m/s로 오다가 도플러가 0.05로 떨어진 직후에도 closing은 아직 1 m/s라
        # 경보가 이어져야 한다(진짜 접근의 꼬리). 순간 도플러로 자르면 이 꼬리가 잘린다.
        rows = self._run(veh_reported_speed=None,
                         speed_at=lambda i: 1.5 if i < 30 else 0.05)
        t_stop = 1000.0 + 30 * 0.2
        tail = [r for r in rows if t_stop <= float(r["pc_time"]) < t_stop + 2.0
                and float(r["distance_m"]) > 3.0]
        self.assertGreater(len(tail), 5)
        self.assertTrue(any(int(r["effective_level"]) >= 1 for r in tail),
                        [(r["distance_m"], r["closing_mps"], r["veh_speed_mps"]) for r in tail][:6])

    def test_the_peak_hold_expires_so_a_long_stopped_vehicle_is_clamped_again(self):
        # 도플러가 0.05로 떨어진 뒤 창(3 s)이 지나면 다시 상한이 걸린다: 위치가 계속
        # 흘러도(1 m/s 드리프트) 조용해야 한다. 등급 홀드(2 s)가 내려가는 시간까지 더해
        # 5.5 s 뒤부터 본다.
        rows = self._run(veh_reported_speed=None,
                         speed_at=lambda i: 1.5 if i < 10 else 0.05)
        t_stop = 1000.0 + 10 * 0.2
        late = [r for r in rows if float(r["pc_time"]) >= t_stop + 3.0 + 2.0 + 0.5
                and float(r["distance_m"]) > 3.0]
        self.assertGreater(len(late), 5)
        self.assertTrue(all(int(r["effective_level"]) == 0 for r in late),
                        [(r["distance_m"], r["closing_mps"], r["effective_level"]) for r in late
                         if int(r["effective_level"]) > 0][:6])


class DistanceTrendRssiFusionTest(unittest.TestCase):
    """저속 실접근(순간 접근속도 데드밴드 아래)을 GPS 거리추세 AND RSSI로 잡는지 — 실제 파이프라인.

    8/19 실기: 차를 손에 들고 ~0.7 m/s로 걸어가 다가갔는데 도플러가 0이라 데드밴드에
    걸려 미탐. GPS 거리추세만으론 점프 가짜접근으로 오탐 51% → 차량이 지팡이를 직접
    들은 RSSI 거리(rssi_dist)도 다가올 때만 인정한다. rssi_dist 없으면(구 펌웨어) 융합
    OFF → 기존 동작(미탐 유지, 안전).
    """

    def _run(self, rssi_mode):
        """rssi_mode: 'approach'(rssi_dist 감소), 'flat'(그대로), None(rssi_dist 없음)."""
        import step8_send_risk as s8
        from step3_parse_v2x import normalize_record
        from step4_state_store import FRESH_WINDOW_S, StateStore
        from step6_kinematics import KinematicsPipeline
        from step7_risk import DCPA_FAR_M, DCPA_FLOOR, DCPA_NEAR_M, T_FLOOR_TTC_S
        from step8_stability import LevelStabilizer
        import json
        rows = []
        clock = {"now": 1000.0}
        orig_norm, orig_append = s8.normalize_record, s8.append_row
        s8.normalize_record = lambda p, m: normalize_record(p, m, now=clock["now"])
        s8.append_row = lambda path, row: rows.append(row)
        try:
            store = StateStore(fresh_window_s=FRESH_WINDOW_S)
            sender = s8.RiskSender(
                KinematicsPipeline(store),
                RiskTransmitter(target_id=0, heartbeat_s=0.2),
                transport=lambda cmd: None, csv_path=None,
                gate_params={"near_m": DCPA_NEAR_M, "far_m": DCPA_FAR_M,
                             "floor": DCPA_FLOOR, "floor_ttc_s": T_FLOOR_TTC_S},
                stabilizer=LevelStabilizer(),
            )
            cane_lat, cane_lng = 37.496330, 126.958313
            deg_per_m = 1.0 / 111320.0
            # 손에 든 차가 11 m 북쪽에서 0.7 m/s(0.14 m/step)로 남하(도플러 0.03, 데드밴드↓).
            for i in range(80):
                clock["now"] = 1000.0 + i * 0.2
                dist = max(3.0, 11.0 - i * 0.14)   # 11 m 시작(RSSI 범위 12 m 안)
                sender.process_line(json.dumps({
                    "type": "cane", "node_id": 1, "seq": i, "gps_valid": 1, "heading_valid": 0,
                    "lat": cane_lat, "lng": cane_lng, "speed_mps": 0.02, "heading_deg": 0.0,
                    "node_risk": 0}), "real")
                clock["now"] += 0.05
                veh = {"type": "vehicle", "node_id": 2, "seq": i, "gps_valid": 1, "heading_valid": 0,
                       "lat": cane_lat + dist * deg_per_m, "lng": cane_lng,
                       "speed_mps": 0.03, "heading_deg": 180.0, "node_risk": 0}
                if rssi_mode == "approach":
                    veh["rssi_dist"] = round(dist, 2)      # RSSI 추정거리도 실제 따라 감소
                elif rssi_mode == "flat":
                    veh["rssi_dist"] = 9.0                 # 안 줄어듦(정지로 봄)
                # None: rssi_dist 키 없음(구 펌웨어)
                sender.process_line(json.dumps(veh), "real")
        finally:
            s8.normalize_record, s8.append_row = orig_norm, orig_append
        return rows

    def test_alarms_when_gps_trend_and_rssi_both_say_approaching(self):
        rows = self._run(rssi_mode="approach")
        self.assertTrue(any(int(r["effective_level"]) >= 1 for r in rows),
                        "GPS추세+RSSI 둘 다 접근인데 저속 접근 미탐 "
                        + str([(r["distance_m"], r["ttc_s"]) for r in rows[-6:]]))

    def test_missed_when_rssi_is_absent_old_firmware(self):
        # rssi_dist 없으면 융합 OFF → 저속 접근은 데드밴드에 눌려 미탐(안전 기본).
        rows = self._run(rssi_mode=None)
        self.assertTrue(all(int(r["effective_level"]) == 0 for r in rows
                            if float(r["distance_m"]) > 2.5))

    def test_missed_when_rssi_says_not_approaching(self):
        # RSSI 거리가 안 줄면(가짜 GPS 접근 방어) 억제 → 미탐.
        rows = self._run(rssi_mode="flat")
        self.assertTrue(all(int(r["effective_level"]) == 0 for r in rows
                            if float(r["distance_m"]) > 2.5))


if __name__ == "__main__":
    unittest.main()
