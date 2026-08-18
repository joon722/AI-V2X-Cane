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


class RecedingClearsAtOnceTest(unittest.TestCase):
    """홀드는 경계에서 떨리는 잡음을 누르는 장치지, 지나간 차의 경보를 붙들어 두는 장치가 아니다.

    8/18: 차가 지나가 멀어지는 중(도플러로 확인된 접근속도 ≤ −0.5 m/s)이면 위험이 끝난 게
    확실하므로 낮은 등급을 즉시 채택한다. 아직 다가오는데(또는 정지·잡음) 등급만 잠깐
    내려간 것은 지금처럼 홀드한다. 기본 홀드는 2 → 1 s (오늘 잡음을 눌러 감당 가능).
    """

    def test_a_drop_while_receding_is_adopted_immediately(self):
        st = LevelStabilizer(hold_s=2.0)
        st.stabilize(2, now=0.0)
        self.assertEqual(st.stabilize(0, now=0.2, receding=True), 0)

    def test_a_drop_while_still_approaching_is_held(self):
        st = LevelStabilizer(hold_s=2.0)
        st.stabilize(2, now=0.0)
        self.assertEqual(st.stabilize(0, now=0.2, receding=False), 2)

    def test_receding_is_the_default_off(self):
        st = LevelStabilizer(hold_s=2.0)
        st.stabilize(2, now=0.0)
        self.assertEqual(st.stabilize(0, now=0.2), 2)

    def test_default_hold_is_one_second(self):
        from step8_stability import HOLD_S
        self.assertEqual(HOLD_S, 1.0)


if __name__ == "__main__":
    unittest.main()
