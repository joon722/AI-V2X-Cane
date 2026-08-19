#!/usr/bin/env python3
"""Hysteresis for the transmitted risk level: rise instantly, fall only after a hold.

Field log 2026-08-01 (risk_tx_20260801_145449.csv) showed the level flapping
0<->1 up to 15 transitions in 2 seconds while the vehicle sat nearly still.
Cause: with closing speed hovering around zero, the TTC sentinel (9999) and the
DCPA miss gate switch on and off together, so the score jumps between ~30 and
~6.6 across the level-1 boundary on almost every fix.

Smoothing the score would blunt real alarms, so the fix sits one layer up: the
level itself gets asymmetric hysteresis.

  rise   a higher level is adopted immediately - never delay a warning.
  fall   a lower level must persist for hold_s before it is adopted; until
         then the previous level is held. A brief dip is noise, a sustained
         drop is a real all-clear (worst case it vibrates hold_s longer).

Pure state machine like RiskTransmitter: `now` is passed in, no clock of its
own, so the behaviour is testable and deterministic. Trust gating is NOT this
module's job - step 8 still forces 0 when the fix is untrusted, and that path
bypasses the hold on purpose (a fake position must not keep an alarm alive).
"""

# 2.0 → 1.0 (2026-08-18): the deadband, Doppler bound, ZUPT and frozen-fix
# handling deployed cut boundary flapping (level changes 96 → 67 on the day's
# log), so a shorter hold now buys the same calm. A drop while approaching (a
# level flicker that is not a real all-clear) waits this long.
HOLD_S = 1.0

# A drop while clearly receding (step 7's Doppler-bounded closing at or below
# the deadband, i.e. a car that has passed) waits only this long - shorter than
# HOLD_S, but NOT instant.
#
# 2026-08-19 실기: 사용자가 차를 손에 들고 지팡이로 걸어갔는데, GPS 점프가 거리를
# 6.6→12.8 m로 튕겨(09:11) 한두 샘플 "멀어짐"으로 위장했다. 즉시 해제하면 그 순간
# 경보가 꺼지지만, 0.6 s 유지하면 그 사이 fix가 다시 잡혀 "아직 다가온다"가 확인될
# 수 있어 오클리어를 막는다(그때 rise 가 시계를 리셋한다). 진짜 지나간 차면 0.6 s
# 더 울릴 뿐 - 옛 2 s 홀드·3 s 잔상보다 훨씬 짧다. step6 점프 게이트가 큰 점프에서
# 트랙을 리셋해 새 fix로 다시 잡는 동안, 이 창이 경보를 붙들어 준다.
RECEDING_HOLD_S = 0.6


class LevelStabilizer:
    """Feed every computed level through stabilize(); transmit what it returns."""

    def __init__(self, hold_s=HOLD_S, receding_hold_s=RECEDING_HOLD_S):
        self.hold_s = hold_s
        self.receding_hold_s = receding_hold_s
        self.level = None
        self.lower_since = None  # when the current run of lower candidates began
        self.last_now = None     # when stabilize() was last called

    def stabilize(self, candidate, now, receding=False):
        """receding: the pair is clearly moving apart (step 7's Doppler-bounded
        closing at or below the deadband, i.e. a car that has passed). Then the
        danger is over and a lower level is adopted at once - the hold exists to
        flatten noise at a boundary, not to keep an alarm alive behind a car
        that has gone (2026-08-18: 2 s of buzzing after every pass)."""
        # 판정이 홀드보다 오래 끊겼다 돌아오면(노드 무음·재부팅) 이전 레벨은 시효가
        # 지난 것 — 판정이 계속됐다면 그 사이 이미 내려갔을 길이다. 8/18 16:09:38:
        # 지팡이 44 s 무음 뒤 첫 판정에서 무음 직전 L2가 2 s 더 나갔다.
        if (self.last_now is not None and self.hold_s > 0
                and now - self.last_now >= self.hold_s):
            self.level = None
            self.lower_since = None
        self.last_now = now

        if self.level is None or candidate >= self.level:
            # First value, a rise, or confirmation of the held level: adopt and
            # forget any pending drop - the danger is (still) real. A re-approach
            # within the receding window lands here and keeps the alarm alive.
            self.level = candidate
            self.lower_since = None
            return self.level

        # candidate < held level: don't drop yet, start (or continue) the clock.
        # Receding (a car that passed) clears faster than an approaching
        # flicker, but neither clears instantly.
        hold = self.receding_hold_s if receding else self.hold_s
        if self.lower_since is None:
            self.lower_since = now
        if now - self.lower_since >= hold:
            self.level = candidate
            self.lower_since = None
        return self.level
