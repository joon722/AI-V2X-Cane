#!/usr/bin/env python3
"""step8_multi_vehicle: 차량별 차선 분리, 지팡이 최댓값 다운링크, 단일 차량 동등성."""

import json
import unittest
from unittest import mock

import step8_send_risk as s8
import step8_multi_vehicle as mv
from step3_parse_v2x import normalize_record
from step4_state_store import StateStore
from step5_test_vehicle import offset_position
from step6_kinematics import KinematicsPipeline
from step7_risk import DCPA_FAR_M, DCPA_FLOOR, DCPA_NEAR_M, T_FLOOR_TTC_S
from step8_stability import LevelStabilizer
from model_gate import ModelGate

CANE_LAT, CANE_LNG = 37.4977, 126.9528
CANE_ID = 4125577512
VEH_A, VEH_B = 2882630492, 2882630493
GATE_PARAMS = {"near_m": DCPA_NEAR_M, "far_m": DCPA_FAR_M, "floor": DCPA_FLOOR,
               "floor_ttc_s": T_FLOOR_TTC_S}
T0 = 1_760_000_000.0


def cane_packet(seq):
    return {"type": "cane", "node_id": CANE_ID, "seq": seq, "gps_valid": 1,
            "lat": CANE_LAT, "lng": CANE_LNG, "speed_mps": 0.0, "heading_deg": 0.0,
            "heading_valid": 0, "node_risk": 0}


def vehicle_packet(node_id, seq, bearing_from_cane, along_m, speed, lateral_m=1.0):
    """지팡이에서 bearing 방향 along_m 지점(+ 옆으로 lateral_m)에 있고 지팡이 쪽으로 달리는 차.

    along_m이 음수면 지팡이를 지나친 뒤다. heading은 지팡이를 향하는 방향.
    """
    lat, lng = offset_position(CANE_LAT, CANE_LNG, bearing_from_cane, along_m)
    lat, lng = offset_position(lat, lng, (bearing_from_cane + 90.0) % 360.0, lateral_m)
    return {"type": "vehicle", "node_id": node_id, "seq": seq, "gps_valid": 1,
            "lat": lat, "lng": lng, "speed_mps": speed,
            "heading_deg": (bearing_from_cane + 180.0) % 360.0, "heading_valid": 1,
            "node_risk": 0}


def two_car_stream(include_b=True, duration_s=16.0):
    """[(pc_time, packet)] 10 Hz.

    A: 북쪽 40 m에서 8 m/s로 남하, t=5 s에 지팡이 옆 1 m를 지난다.
    B: t=3 s부터 동쪽 60 m에서 8 m/s로 서진, t=10.5 s에 지팡이 옆 1 m를 지난다.
    두 차가 3~10 s 동안 동시에 송신한다 - 한 칸 저장이면 서로 덮어쓰는 구간.
    """
    out = []
    seq = {"cane": 0, VEH_A: 0, VEH_B: 0}
    n = int(duration_s * 10)
    for k in range(n):
        t = k * 0.1
        out.append((T0 + t, cane_packet(seq["cane"])))
        seq["cane"] += 1
        ta = t + 0.03
        if ta <= 10.0:
            out.append((T0 + ta, vehicle_packet(VEH_A, seq[VEH_A], 0.0, 40.0 - 8.0 * ta, 8.0)))
            seq[VEH_A] += 1
        tb = t + 0.06
        if include_b and 3.0 <= tb <= 13.0:
            out.append((T0 + tb, vehicle_packet(VEH_B, seq[VEH_B], 90.0,
                                                60.0 - 8.0 * (tb - 3.0), 8.0)))
            seq[VEH_B] += 1
    out.sort(key=lambda item: item[0])
    return out


class Clock:
    """재생 도구(replay_0817)와 같은 방식: 원본 시각을 normalize_record에 주입한다."""

    def __init__(self):
        self.now = None

    def normalize(self, payload, mode):
        return normalize_record(payload, mode, now=self.now)


def run_multi(stream, downlink="max", lane_ttl_s=mv.LANE_LEVEL_TTL_S, lane_expire_s=mv.LANE_EXPIRE_S):
    """MultiVehicleSender로 재생. (보낸 명령 [(t, dict)], 차선 기록 [(t, {차: 등급})], sender)."""
    clock = Clock()
    sent, lane_rows = [], []
    config = mv.LaneConfig(gate_params=GATE_PARAMS, hold_s=1.0)
    sender = mv.MultiVehicleSender(
        config, transport=lambda cmd: sent.append((clock.now, json.loads(cmd))),
        downlink=downlink, lane_ttl_s=lane_ttl_s, lane_expire_s=lane_expire_s)
    with mock.patch.object(s8, "normalize_record", clock.normalize), \
            mock.patch.object(s8, "append_row", lambda path, row: None), \
            mock.patch("builtins.print"):
        for t, packet in stream:
            clock.now = t
            sender.process_line(json.dumps(packet), "simulation")
            lane_rows.append((t, sender.lane_levels(t)))
    return sent, lane_rows, sender


def run_single(stream):
    """기존 step8 RiskSender(차량 칸 하나)로 재생. 보낸 명령 [(t, dict)]."""
    clock = Clock()
    sent = []
    store = StateStore()
    sender = s8.RiskSender(
        KinematicsPipeline(store), s8.RiskTransmitter(target_id=0),
        transport=lambda cmd: sent.append((clock.now, json.loads(cmd))),
        csv_path=None, gate_params=GATE_PARAMS, stabilizer=LevelStabilizer(hold_s=1.0),
        model_gate=ModelGate())
    with mock.patch.object(s8, "normalize_record", clock.normalize), \
            mock.patch.object(s8, "append_row", lambda path, row: None), \
            mock.patch("builtins.print"):
        for t, packet in stream:
            clock.now = t
            sender.process_line(json.dumps(packet), "simulation")
    return sent


def max_sent(sent, start, end):
    """[start, end] 동안 지팡이에 나간 최고 등급(보낸 것이 없으면 직전 값)."""
    level = 0
    for t, cmd in sent:
        if t < start:
            level = cmd["risk"]          # 구간 시작 시점에 이미 울리고 있던 등급
        elif t <= end:
            level = max(level, cmd["risk"])
    return level


class SingleVehicleMatchesStep8Test(unittest.TestCase):
    """차가 한 대면 차선도 하나 - 지팡이로 나가는 등급 변화가 기존 step8과 같아야 한다."""

    def test_same_level_changes_as_step8(self):
        stream = two_car_stream(include_b=False)
        single = run_single(stream)
        multi, _, _ = run_multi(stream)

        def changes(sent):
            out, last = [], None
            for t, cmd in sent:
                if cmd["risk"] != last:
                    out.append((round(t, 3), cmd["risk"]))
                    last = cmd["risk"]
            return out

        self.assertTrue(changes(single), "기준 step8이 아무것도 보내지 않았다")
        self.assertEqual(changes(multi), changes(single))
        self.assertTrue(all(cmd["target_id"] == 0 for _, cmd in multi))


class TwoVehiclesAreTrackedSeparatelyTest(unittest.TestCase):
    def setUp(self):
        self.sent, self.lane_rows, self.sender = run_multi(two_car_stream())

    def lane_max(self, vehicle, start, end):
        levels = [lv.get(str(vehicle), 0) for t, lv in self.lane_rows if start <= t - T0 <= end]
        return max(levels) if levels else 0

    def test_each_car_gets_its_own_lane(self):
        keys = {key for _, levels in self.lane_rows for key in levels}
        self.assertEqual(keys, {str(VEH_A), str(VEH_B)})

    def test_both_cars_raise_their_own_alarm(self):
        # A는 5 s, B는 10.5 s에 지팡이 옆을 지난다. 그 직전 3초에 각자 경보 이상.
        self.assertGreaterEqual(self.lane_max(VEH_A, 2.0, 5.0), 2)
        self.assertGreaterEqual(self.lane_max(VEH_B, 7.5, 10.5), 2)

    def test_lanes_do_not_borrow_each_others_danger(self):
        # A가 지팡이 옆을 지날 때 B는 아직 30 m 넘게 떨어져 있다(TTC 4 s 이상).
        self.assertLessEqual(self.lane_max(VEH_B, 4.0, 5.0), 1)

    def test_cane_hears_both_alarms(self):
        self.assertGreaterEqual(max_sent(self.sent, T0 + 2.0, T0 + 5.0), 2)
        self.assertGreaterEqual(max_sent(self.sent, T0 + 7.5, T0 + 10.5), 2)

    def test_max_mode_broadcasts_only(self):
        self.assertTrue(self.sent)
        self.assertTrue(all(cmd["target_id"] == 0 for _, cmd in self.sent))


class PerTargetDownlinkTest(unittest.TestCase):
    def test_vehicles_and_cane_get_their_own_target(self):
        sent, _, _ = run_multi(two_car_stream(), downlink="per-target")
        targets = {cmd["target_id"] for _, cmd in sent}
        self.assertEqual(targets, {VEH_A, VEH_B, CANE_ID})
        # 지팡이 몫은 두 차 중 높은 쪽: B가 지나갈 때도 경보.
        cane = [(t, cmd) for t, cmd in sent if cmd["target_id"] == CANE_ID]
        self.assertGreaterEqual(max_sent(cane, T0 + 7.5, T0 + 10.5), 2)
        # A 몫은 B 때문에 올라가지 않는다: A가 지나간 뒤(7.5~10.5 s) A에게 간 등급은 낮다.
        to_a = [cmd["risk"] for t, cmd in sent if cmd["target_id"] == VEH_A and t >= T0 + 7.5]
        self.assertTrue(all(level <= 1 for level in to_a), to_a)


class RoutingAndLifecycleTest(unittest.TestCase):
    def test_uwb_goes_only_to_its_vehicle(self):
        stream = two_car_stream()
        _, _, sender = run_multi(stream[:200])
        lanes = sender.lanes
        uwb = {"type": "uwb", "node_id": VEH_B, "seq": 1, "uwb_dist": 3.0,
               "uwb_raw": 3.1, "uwb_closing": 1.0, "uwb_calib": 1, "gps_valid": 0}
        clock = Clock()
        clock.now = stream[199][0] + 0.01
        with mock.patch.object(s8, "normalize_record", clock.normalize), \
                mock.patch("builtins.print"):
            sender.process_line(json.dumps(uwb), "simulation")
        self.assertIsNotNone(lanes[str(VEH_B)].sender._uwb)
        self.assertIsNone(lanes[str(VEH_A)].sender._uwb)

    def test_quiet_lane_closes(self):
        # A만 있는 스트림을 끝까지(A는 10 s에 끊김) + 지팡이만 2 s 더. 만료 1 s로 짧게.
        stream = two_car_stream(include_b=False, duration_s=14.0)
        _, lane_rows, sender = run_multi(stream, lane_expire_s=1.0)
        self.assertIn(str(VEH_A), lane_rows[50][1])
        self.assertNotIn(str(VEH_A), sender.lanes)

    def test_stale_lane_does_not_hold_the_maximum(self):
        sender = mv.MultiVehicleSender(mv.LaneConfig(gate_params=GATE_PARAMS),
                                       transport=lambda cmd: None, lane_ttl_s=4.0)
        lane = mv.VehicleLane("1", sender.config, transport=lambda cmd: None)
        lane.transmitter.last_level = 3
        lane.last_judged_at = 100.0
        sender.lanes["1"] = lane
        self.assertEqual(sender.lane_levels(103.0), {"1": 3})
        self.assertEqual(sender.lane_levels(104.5), {"1": 0})

    def test_lane_error_after_judging_keeps_the_alarm(self):
        # 차선 CSV 기록이 매번 터져도(디스크 꽉 참 등) 그 차선이 낸 등급은 살아 있어야 한다.
        clock = Clock()
        sent = []
        sender = mv.MultiVehicleSender(mv.LaneConfig(gate_params=GATE_PARAMS),
                                       transport=lambda cmd: sent.append((clock.now, json.loads(cmd))))

        def broken_append(path, row):
            raise OSError("disk full")

        with mock.patch.object(s8, "normalize_record", clock.normalize), \
                mock.patch.object(s8, "append_row", broken_append), \
                mock.patch("builtins.print"):
            for t, packet in two_car_stream(include_b=False):
                clock.now = t
                sender.process_line(json.dumps(packet), "simulation")
        self.assertGreaterEqual(max_sent(sent, T0 + 2.0, T0 + 5.0), 3)

    def test_vehicle_key_falls_back_to_mac(self):
        self.assertEqual(mv.vehicle_key({"node_id": 42}), "42")
        self.assertEqual(mv.vehicle_key({"node_id": "", "src_mac": "AA:BB"}), "AA:BB")
        self.assertEqual(mv.vehicle_key({}), "unknown")
        self.assertEqual(mv.target_id_of("AA:BB"), 0)


class CliTest(unittest.TestCase):
    def test_step8_options_are_inherited(self):
        args = mv.parse_args(["--stdin", "--no-fusion", "--downlink", "per-target",
                              "--test-vehicles", "2"])
        self.assertTrue(args.stdin and args.no_fusion)
        self.assertEqual(args.downlink, "per-target")
        self.assertEqual(args.test_vehicles, 2)

    def test_test_vehicles_arrive_at_different_times(self):
        vehicles = mv.make_test_vehicles(2, 50.0, 5.0)
        self.assertEqual([node for node, _ in vehicles], [900000001, 900000002])
        self.assertNotEqual(vehicles[0][1].start_distance_m, vehicles[1][1].start_distance_m)


if __name__ == "__main__":
    unittest.main()
