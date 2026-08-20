"""차량 GPS 순간 무효(compute None) 동안 다운링크 유지 + 상한 후 SAFE 해제.

8/19 코너 1차에서 차량 GPS가 ~3.9 s 무효가 되자 compute()가 None을 냈고, 그
사이 다운링크가 끊겨 노드가 RSU 타임아웃으로 위험을 조기 해제했다. 여기서는
차량 패킷을 끊어 vehicle을 stale 시켜(=compute None) 그 공백을 재현하고,
_heartbeat_through_gap 이 마지막 레벨을 하트비트로 유지하다가 상한을 넘으면
SAFE(0)로 정리하는지 확인한다. cap=0 이면 예전 동작(즉시 무음)인지도 본다.
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

from model_gate import ModelGate
from step4_state_store import StateStore
from step6_kinematics import KinematicsPipeline
from step7_risk import DCPA_FAR_M, DCPA_FLOOR, DCPA_NEAR_M, T_FLOOR_TTC_S
from step8_send_risk import RiskSender, RiskTransmitter

CANE_LAT, CANE_LNG = 37.4963, 126.9575


def cane_line(seq):
    return json.dumps({
        "type": "cane", "node_id": 4125577512, "seq": seq, "gps_valid": 1,
        "lat": CANE_LAT, "lng": CANE_LNG, "speed_mps": 0.0, "heading_deg": 0.0,
        "node_risk": 0,
    })


def vehicle_line(seq, north_m):
    """지팡이 정북 north_m 지점에서 남향으로 다가오는 차량(head-on)."""
    return json.dumps({
        "type": "vehicle", "node_id": 111, "seq": seq, "gps_valid": 1,
        "lat": CANE_LAT + north_m / 111_320.0, "lng": CANE_LNG,
        "speed_mps": 8.0, "heading_deg": 180.0, "node_risk": 0,
    })


class GapHoldCase(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)
        ticks = itertools.count(1000.0, 0.5)  # process_line 1회 = +0.5 s
        patcher = mock.patch("step3_parse_v2x.time.time", side_effect=ticks)
        patcher.start()
        self.addCleanup(patcher.stop)
        quiet = contextlib.redirect_stdout(io.StringIO())
        quiet.__enter__()
        self.addCleanup(quiet.__exit__, None, None, None)
        self.sent = []

    def make_sender(self, gap_hold_cap_s, heartbeat_s=0.5):
        return RiskSender(
            KinematicsPipeline(StateStore()),
            RiskTransmitter(target_id=0, heartbeat_s=heartbeat_s),
            self.sent.append,
            self.tmp / "tx.csv",
            {"near_m": DCPA_NEAR_M, "far_m": DCPA_FAR_M,
             "floor": DCPA_FLOOR, "floor_ttc_s": T_FLOOR_TTC_S},
            model_gate=ModelGate(),
            gap_hold_cap_s=gap_hold_cap_s,
        )

    def approach(self, sender):
        """head-on 접근으로 경보 등급을 올린다."""
        for i, north in enumerate([40, 32, 24, 16, 8, 4, 3, 2]):
            sender.process_line(cane_line(100 + i), "test")
            sender.process_line(vehicle_line(200 + i, north), "test")

    @staticmethod
    def risk(cmd):
        return json.loads(cmd)["risk"]

    def test_downlink_stays_alive_and_holds_level_during_gap(self):
        sender = self.make_sender(gap_hold_cap_s=10.0)  # 상한 넉넉 → 계속 유지
        self.approach(sender)
        pre_level = self.risk(self.sent[-1])
        self.assertGreaterEqual(pre_level, 1, "접근에서 경보 등급이 안 올라갔다")
        n_before = len(self.sent)
        # 차량 패킷을 끊는다 → vehicle stale → compute None 구간
        for i in range(8):
            sender.process_line(cane_line(130 + i), "test")
        # 예전엔 이 구간이 무음이었다. 이제는 하트비트가 계속 나가야 한다.
        self.assertGreaterEqual(len(self.sent) - n_before, 3,
                                "GPS 공백에서 다운링크가 끊겼다(하트비트 없음)")
        gap_sends = [self.risk(c) for c in self.sent[n_before:]]
        # 상한 안이므로 전부 마지막 레벨을 유지해야 한다(0으로 안 떨어짐).
        self.assertTrue(all(r == pre_level for r in gap_sends),
                        f"공백 중 레벨 유지 실패: {gap_sends} (기대 {pre_level})")

    def test_releases_to_safe_after_cap(self):
        sender = self.make_sender(gap_hold_cap_s=1.0)  # 상한 짧게
        self.approach(sender)
        self.assertGreaterEqual(self.risk(self.sent[-1]), 1)
        # 상한(1.0 s)을 훨씬 넘는 공백
        for i in range(12):
            sender.process_line(cane_line(130 + i), "test")
        self.assertEqual(self.risk(self.sent[-1]), 0,
                         "상한 초과인데 SAFE(0)로 해제되지 않았다")

    def test_cap_zero_keeps_old_silent_behavior(self):
        sender = self.make_sender(gap_hold_cap_s=0.0)  # 기능 OFF = 예전 동작
        self.approach(sender)
        # vehicle을 확실히 stale 시킨 뒤(2줄) 카운트를 고정하고
        sender.process_line(cane_line(130), "test")
        sender.process_line(cane_line(131), "test")
        n_settled = len(self.sent)
        # 이후 공백 구간엔 전송이 한 건도 없어야 한다(즉시 무음).
        for i in range(6):
            sender.process_line(cane_line(132 + i), "test")
        self.assertEqual(len(self.sent), n_settled,
                         "cap=0인데 공백에서 전송이 있었다(예전 동작 아님)")


if __name__ == "__main__":
    unittest.main()
