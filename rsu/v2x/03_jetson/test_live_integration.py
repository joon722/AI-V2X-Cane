"""step8 판정 루프와 화면 상태의 결합.

live_state 단위 테스트는 가짜 입력으로 모양만 본다. 여기서는 실제 노드 JSON을
파이프라인에 흘려 step4~step8 전 경로를 태운다 - 상대좌표가 step6에서
화면까지 실제로 도달하는지는 이 방법으로만 확인된다.
"""
import contextlib
import io
import itertools
import json
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from live_state import LiveState
from model_gate import ModelGate
from step4_state_store import StateStore
from step6_kinematics import KinematicsPipeline
from step7_risk import DCPA_FAR_M, DCPA_FLOOR, DCPA_NEAR_M, T_FLOOR_TTC_S
from step8_send_risk import RiskSender, RiskTransmitter

CANE_LAT, CANE_LNG = 37.4963, 126.9575


def cane_line(seq, gps_valid=1):
    return json.dumps({
        "type": "cane", "node_id": 4125577512, "seq": seq, "gps_valid": gps_valid,
        "lat": CANE_LAT, "lng": CANE_LNG, "speed_mps": 0.0, "heading_deg": 0.0,
        "node_risk": 0,
    })


def vehicle_line(seq, north_m):
    """지팡이 정북 north_m 지점의 차량. seq가 늘수록 가까워진다."""
    return json.dumps({
        "type": "vehicle", "node_id": 111, "seq": seq, "gps_valid": 1,
        "lat": CANE_LAT + north_m / 111_320.0, "lng": CANE_LNG,
        "speed_mps": 8.0, "heading_deg": 180.0, "node_risk": 0,
    })


class SenderCase(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)
        # 기록의 pc_time은 normalize_record 안의 time.time()에서 온다. 테스트는
        # 순식간에 돌아 두 기록의 dt가 0이 되고, 그러면 칼만 필터가 갱신을
        # 건너뛰어 거리가 첫 값에 굳는다. 실측 판정 간격(1.03초)에 맞춰
        # 노드 하나당 0.5초씩 흐르게 한다.
        ticks = itertools.count(1000.0, 0.5)
        patcher = mock.patch("step3_parse_v2x.time.time", side_effect=ticks)
        patcher.start()
        self.addCleanup(patcher.stop)
        # step8은 전송마다 stdout에 [TX] 줄을 찍는다. 테스트 출력이 그것으로
        # 덮이지 않게 삼킨다. 실패 보고는 stderr로 나가므로 영향받지 않는다.
        quiet = contextlib.redirect_stdout(io.StringIO())
        quiet.__enter__()
        self.addCleanup(quiet.__exit__, None, None, None)
        self.state = LiveState()
        self.sent = []
        self.sender = self._make_sender(self.state)

    def _make_sender(self, live_state, heartbeat_s=1.0):
        store = StateStore()
        return RiskSender(
            KinematicsPipeline(store),
            RiskTransmitter(target_id=0, heartbeat_s=heartbeat_s),
            self.sent.append,
            self.tmp / "tx.csv",
            {"near_m": DCPA_NEAR_M, "far_m": DCPA_FAR_M,
             "floor": DCPA_FLOOR, "floor_ttc_s": T_FLOOR_TTC_S},
            model_gate=ModelGate(),
            live_state=live_state,
        )

    def approach(self, sender=None, steps=4):
        """차량이 다가오는 장면을 흘려보낸다."""
        sender = sender or self.sender
        for i in range(steps):
            sender.process_line(cane_line(seq=100 + i), "test")
            sender.process_line(vehicle_line(seq=200 + i, north_m=40.0 - i * 8.0), "test")


class TestPipelineReachesLiveState(SenderCase):
    def test_measurement_arrives(self):
        self.approach()
        snap = self.state.snapshot(now=0.0)
        self.assertIsNotNone(snap["t"])
        self.assertIsNotNone(snap["distance_m"])
        self.assertGreater(snap["distance_m"], 0.0)

    def test_relative_position_survives_the_whole_path(self):
        """step6가 계산한 상대좌표가 화면까지 온다. 차는 정북에 있었다."""
        self.approach()
        snap = self.state.snapshot(now=0.0)
        self.assertIsNotNone(snap["rel_north_m"])
        self.assertGreater(snap["rel_north_m"], 0.0)
        self.assertAlmostEqual(snap["rel_east_m"], 0.0, delta=1.0)

    def test_distance_agrees_with_the_relative_position(self):
        """숫자와 그림이 어긋나면 화면이 거짓말을 한다. 같은 소스여야 한다."""
        self.approach()
        snap = self.state.snapshot(now=0.0)
        hypot = (snap["rel_east_m"] ** 2 + snap["rel_north_m"] ** 2) ** 0.5
        self.assertAlmostEqual(snap["distance_m"], hypot, delta=0.05)


class TestUpdatesWhileHolding(SenderCase):
    def test_screen_updates_even_when_nothing_is_transmitted(self):
        """다운링크는 등급이 같으면 참는다. 화면까지 같이 멈추면 안 된다.

        heartbeat를 사실상 끄고, 등급이 바뀌지 않는 먼 거리에서 차를 조금씩
        움직인다. 그 구간에서 전송은 한 건도 없어야 하고 화면은 계속 변해야 한다.
        """
        sender = self._make_sender(self.state, heartbeat_s=10_000.0)
        # 60m 밖에 세워 등급을 0으로 안정시킨다.
        for i in range(8):
            sender.process_line(cane_line(seq=100 + i), "test")
            sender.process_line(vehicle_line(seq=200 + i, north_m=60.0), "test")

        before = self.state.snapshot(now=0.0)
        sent_before = len(self.sent)

        # 같은 등급을 유지한 채 거리만 바꾼다.
        for i in range(3):
            sender.process_line(cane_line(seq=120 + i), "test")
            sender.process_line(vehicle_line(seq=220 + i, north_m=58.0 - i), "test")

        after = self.state.snapshot(now=0.0)
        self.assertEqual(len(self.sent), sent_before, "이 구간에 전송이 있었다")
        self.assertNotEqual(after["distance_m"], before["distance_m"])
        self.assertNotEqual(after["t"], before["t"])


class TestIsolationFromTheAlarm(SenderCase):
    def test_a_broken_live_state_does_not_stop_the_downlink(self):
        """화면이 터져도 지팡이는 경보를 받아야 한다."""
        class Exploding:
            def update(self, *args, **kwargs):
                raise RuntimeError("boom")

        sender = self._make_sender(Exploding())
        self.approach(sender=sender)
        self.assertTrue(self.sent, "화면 예외가 다운링크까지 막았다")

    def test_without_live_state_nothing_changes(self):
        sender = self._make_sender(None)
        self.approach(sender=sender)
        self.assertTrue(self.sent)


if __name__ == "__main__":
    unittest.main()
