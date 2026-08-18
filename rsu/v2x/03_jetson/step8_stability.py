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
# handling deployed today cut boundary flapping (level changes 96 → 67 on the
# day's log), so a shorter hold now buys the same calm; and a drop while the
# pair is clearly receding is adopted at once (see stabilize()).
HOLD_S = 1.0


class LevelStabilizer:
    """Feed every computed level through stabilize(); transmit what it returns."""

    def __init__(self, hold_s=HOLD_S):
        self.hold_s = hold_s
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

        if self.level is None or candidate >= self.level or receding:
            # First value, a rise, confirmation of the held level, or a drop
            # while receding: adopt and forget any pending drop.
            self.level = candidate
            self.lower_since = None
            return self.level

        # candidate < held level: don't drop yet, start (or continue) the clock.
        if self.lower_since is None:
            self.lower_since = now
        if now - self.lower_since >= self.hold_s:
            self.level = candidate
            self.lower_since = None
        return self.level
