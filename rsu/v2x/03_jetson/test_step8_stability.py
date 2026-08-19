#!/usr/bin/env python3
"""Tests for the level hysteresis: rise instantly, fall only after the hold."""

import unittest

from step8_stability import LevelStabilizer


class RiseAndFirstValueTest(unittest.TestCase):
    def test_first_value_is_adopted_immediately(self):
        st = LevelStabilizer(hold_s=2.0)
        self.assertEqual(st.stabilize(1, now=0.0), 1)

    def test_a_rise_is_never_delayed(self):
        st = LevelStabilizer(hold_s=2.0)
        st.stabilize(0, now=0.0)
        self.assertEqual(st.stabilize(2, now=0.1), 2)

    def test_a_rise_during_a_pending_drop_cancels_the_drop(self):
        # 1 -> dip to 0 -> back to 1 before the hold expires: the dip must not
        # survive as a pending drop that later fires. A NEW dip restarts the clock.
        st = LevelStabilizer(hold_s=2.0)
        st.stabilize(1, now=0.0)
        st.stabilize(0, now=0.5)
        st.stabilize(1, now=1.0)                       # cancels the 0.5 dip
        self.assertEqual(st.stabilize(0, now=2.6), 1)  # new dip, clock restarts at 2.6
        self.assertEqual(st.stabilize(0, now=3.1), 1)  # only 0.5s into the new dip
        self.assertEqual(st.stabilize(0, now=4.7), 0)  # 2.1s sustained: adopted


class FallTest(unittest.TestCase):
    def test_a_brief_dip_is_held_at_the_previous_level(self):
        st = LevelStabilizer(hold_s=2.0)
        st.stabilize(1, now=0.0)
        self.assertEqual(st.stabilize(0, now=0.5), 1)
        self.assertEqual(st.stabilize(0, now=1.5), 1)

    def test_a_sustained_drop_is_adopted_after_the_hold(self):
        st = LevelStabilizer(hold_s=2.0)
        st.stabilize(1, now=0.0)
        st.stabilize(0, now=0.5)
        self.assertEqual(st.stabilize(0, now=2.5), 0)

    def test_field_log_flapping_burst_is_flattened(self):
        # The worst 2s burst from risk_tx_20260801_145449.csv: 0/1 toggling on
        # nearly every fix. The output must stay at 1 throughout.
        st = LevelStabilizer(hold_s=2.0)
        burst = [
            (1, 0.00), (0, 0.08), (1, 0.16), (0, 0.24), (1, 0.31),
            (0, 0.55), (1, 0.62), (0, 0.66), (1, 0.73), (0, 0.81),
            (1, 0.87), (0, 0.91), (1, 0.98), (0, 1.10),
        ]
        outputs = {st.stabilize(level, now) for level, now in burst}
        self.assertEqual(outputs, {1})

    def test_zero_hold_disables_the_hysteresis(self):
        st = LevelStabilizer(hold_s=0.0)
        st.stabilize(1, now=0.0)
        self.assertEqual(st.stabilize(0, now=0.001), 0)


class SilenceExpiresTheHoldTest(unittest.TestCase):
    """판정이 홀드보다 오래 끊겼다 돌아오면 이전 레벨은 시효가 지난 것이다.

    8/18 16:09:38 실측: 지팡이가 44초 무음이었다 복귀했는데, 무음 직전의 경고(L2)가
    첫 판정에서 홀드 시계를 새로 시작해 2초 더 나갔다. 홀드는 "짧은 요동"을 누르는
    장치지 "긴 공백 너머의 기억"이 아니다 — 공백이 hold_s 이상이면 새로 시작한다.
    """

    def test_a_lower_level_after_a_long_silence_is_adopted_at_once(self):
        st = LevelStabilizer(hold_s=2.0)
        st.stabilize(2, now=0.0)
        self.assertEqual(st.stabilize(0, now=44.0), 0)

    def test_a_silence_shorter_than_the_hold_still_holds(self):
        st = LevelStabilizer(hold_s=2.0)
        st.stabilize(2, now=0.0)
        self.assertEqual(st.stabilize(0, now=1.5), 2)

    def test_a_silence_exactly_the_hold_long_expires_it(self):
        # 판정이 계속됐다면 그 시점엔 이미 내려갔을 길이 → 같은 결론.
        st = LevelStabilizer(hold_s=2.0)
        st.stabilize(2, now=0.0)
        self.assertEqual(st.stabilize(0, now=2.0), 0)

    def test_a_rise_after_a_long_silence_is_still_immediate(self):
        st = LevelStabilizer(hold_s=2.0)
        st.stabilize(1, now=0.0)
        self.assertEqual(st.stabilize(3, now=44.0), 3)


class RecedingClearsAfterShortHoldTest(unittest.TestCase):
    """멀어지는 중(지나간 차)이라도 즉시 끄지 않고 짧게(receding_hold_s, 0.6 s) 유지한다.

    2026-08-19 실기: 사용자가 차를 손에 들고 지팡이로 걸어갔는데, GPS 점프가 거리를
    6.6→12.8 m로 튕겨 한두 샘플 "멀어짐"으로 보였다(09:11). 즉시 해제하면 그 순간
    경보가 꺼진다. 0.6 s 유지하면 그 사이 fix가 다시 잡혀 "아직 다가온다"가 확인될 수
    있어 오클리어를 막는다. 진짜 지나간 차면 0.6 s 더 울릴 뿐(옛 2 s 홀드·3 s 잔상보다
    훨씬 짧다). 아직 다가오는데 등급만 잠깐 내려간 것(잡음)은 전체 홀드(1 s)로 누른다.
    """

    def test_a_receding_drop_is_held_for_the_receding_window_then_clears(self):
        # 멀어짐 시작(0.1)부터 receding_hold_s(0.6) 뒤에 해제.
        st = LevelStabilizer(hold_s=1.0, receding_hold_s=0.6)
        st.stabilize(2, now=0.0)
        self.assertEqual(st.stabilize(0, now=0.1, receding=True), 2)   # 유지 시작
        self.assertEqual(st.stabilize(0, now=0.5, receding=True), 2)   # 0.4 s, 아직 유지
        self.assertEqual(st.stabilize(0, now=0.8, receding=True), 0)   # 0.7 s ≥ 0.6, 해제

    def test_a_reapproach_within_the_receding_window_keeps_the_alarm(self):
        # 점프였고 그 사이 다시 다가옴이 확인되면 경보가 살아있어야 한다.
        st = LevelStabilizer(hold_s=1.0, receding_hold_s=0.6)
        st.stabilize(2, now=0.0)
        self.assertEqual(st.stabilize(0, now=0.1, receding=True), 2)   # 멀어짐 잠깐
        self.assertEqual(st.stabilize(2, now=0.3, receding=False), 2)  # 다시 다가옴 → 시계 리셋
        self.assertEqual(st.stabilize(0, now=0.5, receding=True), 2)   # 새 멀어짐, 다시 유지
        self.assertEqual(st.stabilize(0, now=1.2, receding=True), 0)   # 0.5부터 0.7 s 뒤 해제

    def test_a_receding_drop_is_not_cleared_instantly(self):
        st = LevelStabilizer(hold_s=1.0, receding_hold_s=0.6)
        st.stabilize(2, now=0.0)
        self.assertEqual(st.stabilize(0, now=0.1, receding=True), 2)   # 즉시 해제 아님

    def test_a_drop_while_still_approaching_holds_the_full_hold(self):
        st = LevelStabilizer(hold_s=1.0, receding_hold_s=0.6)
        st.stabilize(2, now=0.0)
        self.assertEqual(st.stabilize(0, now=0.1, receding=False), 2)  # 드롭 시작
        self.assertEqual(st.stabilize(0, now=0.9, receding=False), 2)  # 0.8 s < 1.0, 유지
        self.assertEqual(st.stabilize(0, now=1.2, receding=False), 0)  # 1.1 s ≥ 1.0, 해제

    def test_receding_clears_faster_than_an_approaching_flicker(self):
        rec = LevelStabilizer(hold_s=1.0, receding_hold_s=0.6)
        rec.stabilize(2, now=0.0); rec.stabilize(0, now=0.1, receding=True)
        app = LevelStabilizer(hold_s=1.0, receding_hold_s=0.6)
        app.stabilize(2, now=0.0); app.stabilize(0, now=0.1, receding=False)
        # 드롭 0.1 시작 → t=0.8: 멀어짐(0.7≥0.6) 꺼지고, 접근 잡음(0.7<1.0) 유지
        self.assertEqual(rec.stabilize(0, now=0.8, receding=True), 0)
        self.assertEqual(app.stabilize(0, now=0.8, receding=False), 2)

    def test_defaults(self):
        from step8_stability import HOLD_S, RECEDING_HOLD_S
        self.assertEqual(HOLD_S, 1.0)
        self.assertEqual(RECEDING_HOLD_S, 0.6)


if __name__ == "__main__":
    unittest.main()
