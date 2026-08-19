#!/usr/bin/env python3
"""Send step 7's risk_level down to the RSU, which broadcasts it to the cane.

The downlink shape ({"target_id":0,"risk":N}) and the fact that the RSU echoes
it onto the cane's node_risk were both verified by probe_risk_downlink.py. This
step wires step 7's live risk_level into that path.

Two policies sit in front of the transmit, both in RiskTransmitter so they can
be tested without a serial port:

  trust gating   a fallback (gps_valid=0) position makes the risk spatially
                 meaningless, so a nonzero value is held back and only a 0 (which
                 clears a stale alarm) goes out, unless --tx-untrusted is set.
  rate limiting  the cane reports at ~10 Hz; sending every record floods the RSU,
                 so we send on change and otherwise only on a heartbeat.

step2-7 are not modified; this file imports the pipeline and scores the filtered
track itself, the same small duplication step 7 carries from step 6.
"""

import argparse
import csv
import json
import sys
import time
from dataclasses import dataclass, replace
from pathlib import Path

import live_server
from step3_parse_v2x import SOURCE_MODES, normalize_record
from step4_state_store import FRESH_WINDOW_S, StateStore, has_position
from step5_test_vehicle import SPEED_MPS, START_DISTANCE_M, TestVehicle
from step6_kinematics import KinematicsPipeline, to_float
from model_gate import ModelGate
from step7_risk import (
    DCPA_FAR_M,
    DCPA_FLOOR,
    DCPA_NEAR_M,
    T_FLOOR_TTC_S,
    TREND_MIN_MPS,
    assess_risk,
)
from step8_stability import HOLD_S, LevelStabilizer


HEARTBEAT_S = 1.0

# 접근속도(closing_los) 평활 계수. 저속·GPS 노이즈로 접근속도가 프레임마다
# 0↔음수로 튀고, TTC=거리/접근속도라 그 노이즈가 TTC를 심하게 출렁이게 한다.
# 짧은 EMA로 접근속도를 평활하면 TTC·레벨이 안정된다(≈0.3~0.5s 지연, 근접
# 거리 하한이 그 지연을 커버). 도플러 융합이 저속(heading_valid=0)에서 안
# 켜지는 것을 보완한다. 1.0이면 평활 없음(원값). 낮을수록 더 부드럽고 더 느리다.
CLOSING_EMA_ALPHA = 0.2

# 도플러 물리 상한(step7 CLOSING_DOPPLER_MARGIN_MPS)에 넣는 노드 속도의 창. 위치
# 기반 접근속도(노드 GPS 필터 → KF → EMA)는 도플러보다 2~3 s 늦게 따라온다 —
# 8/18 16:31:44·16:37:47: 차가 이미 0.1 m/s로 감속했는데 closing은 그제야 1 m/s.
# 순간 도플러로 자르면 진짜 접근의 꼬리가 잘리므로, 최근 이 창 안의 최대 도플러를
# 쓴다. 서 있는 노드는 창 내내 ≈0이라 유령은 그대로 잘린다.
DOPPLER_PEAK_WINDOW_S = 3.0

# 거리추세를 재는 창(초). 이 시간 동안의 거리 감소를 속도로 환산해 저속 실접근을
# 잡는다. 길수록 잡음에 강하지만 반응이 느리고, 짧으면 반대. 4 s는 8/19 실기의
# 놓친 저속 접근(거리 4초에 1.5~2.9 m 감소)과 정지 잡음(p90 1.2 m)을 가르는 값.
DIST_TREND_WINDOW_S = 4.0

# RSSI 방향 융합. GPS 거리추세만으로 저속 접근을 잡으면 GPS 점프가 만든 가짜 접근으로
# 오탐 51%(8/19 재생). 그래서 차량이 지팡이를 직접 들은 RSSI(rssi_dist)도 "다가오는
# 중"일 때만 GPS 추세를 인정한다 — 둘은 독립이라 GPS 점프가 RSSI를 같이 안 흔든다.
# 버킷 08-08 무지향성 안테나: RSSI거리 vs GPS거리 상관 −0.95. rssi_dist가 패킷에
# 없으면(구 펌웨어) 융합은 자동 OFF → 기존 동작.
RSSI_TREND_WINDOW_S = 4.0     # RSSI 거리추세 창
RSSI_RANGE_M = 12.0          # RSSI 추정거리 유효범위(0.8~12 m). 밖은 포화라 융합 안 함
RSSI_APPROACH_MIN_M = 1.0    # 창 동안 RSSI 거리가 이만큼 줄면 "다가오는 중"

# calibrate_bias.py 가 저장하는 차량 GPS 상대 바이어스. 없으면 보정 없이 동작.
VEHICLE_BIAS_FILE = Path(__file__).resolve().with_name("vehicle_bias.json")
# 바이어스는 시간에 따라 변한다. 이보다 오래된 값이면 재보정을 권한다.
VEHICLE_BIAS_STALE_S = 30 * 60


def load_vehicle_bias(path):
    """(bias_east, bias_north, meta) from calibrate_bias.py's file.

    A missing file means "not calibrated" → zero bias. A corrupt file must not
    keep the alarm engine from starting, so it degrades to zero as well, loudly.
    """
    path = Path(path)
    if not path.exists():
        return 0.0, 0.0, None
    try:
        meta = json.loads(path.read_text(encoding="utf-8"))
        east = float(meta["bias_east_m"])
        north = float(meta["bias_north_m"])
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"[WARN] vehicle_bias_load_failed path={path} error={exc!r} -> 보정 없이 동작",
              file=sys.stderr)
        return 0.0, 0.0, None
    return east, north, meta


def vehicle_bias_messages(bias_east, bias_north, meta, now):
    """시작 로그 줄들: 무엇이 적용됐고 얼마나 오래된 값인지."""
    if meta is None:
        return ["[INFO] vehicle_bias 없음 → 보정 없이 동작 (calibrate_bias.py 로 측정)"]
    age_min = (now - to_float(meta.get("created_at"))) / 60.0
    lines = [f"[INFO] vehicle_bias east={bias_east:+.2f} north={bias_north:+.2f} m "
             f"age={age_min:.0f}min"]
    if age_min * 60.0 > VEHICLE_BIAS_STALE_S:
        lines.append(f"[WARN] vehicle_bias 가 {age_min:.0f}분 전 값 — 바이어스는 시간에 따라 "
                     f"변하므로 재보정 권장 (calibrate_bias.py → 재시작)")
    return lines

# After the RSU accepts a downlink it echoes these two record types back on the
# same serial line (documented in the session summary). They confirm the command
# landed; they are not pipeline input, so step 8 consumes them without warning.
RSU_ACK_TYPES = ("risk_tx", "risk_broadcast_to_seen")


def _gps_trusted(cane_gps_valid):
    """gps_valid is 1 for a real fix, 0 for the indoor fallback coordinate."""
    return to_float(cane_gps_valid) >= 0.5


@dataclass(frozen=True)
class TxDecision:
    should_send: bool
    computed_level: int
    effective_level: int
    trusted: bool
    reason: str  # "change" | "heartbeat" | "hold"


class RiskTransmitter:
    """Decides whether a risk_level should be transmitted right now.

    Pure state machine: no serial, no clock of its own. `now` is passed in so the
    heartbeat is testable and the whole thing stays deterministic.
    """

    def __init__(self, target_id=0, heartbeat_s=HEARTBEAT_S, allow_untrusted=False):
        self.target_id = target_id
        self.heartbeat_s = heartbeat_s
        self.allow_untrusted = allow_untrusted
        self.last_level = None
        self.last_send_time = None

    def consider(self, computed_level, cane_gps_valid, now):
        trusted = self.allow_untrusted or _gps_trusted(cane_gps_valid)
        # An untrusted fix can still clear an alarm (send 0); it just cannot raise one.
        effective = computed_level if trusted else 0

        if self.last_level is None or effective != self.last_level:
            reason, should_send = "change", True
        elif self.last_send_time is None or now - self.last_send_time >= self.heartbeat_s:
            reason, should_send = "heartbeat", True
        else:
            reason, should_send = "hold", False

        if should_send:
            self.last_level = effective
            self.last_send_time = now

        return TxDecision(should_send, computed_level, effective, trusted, reason)

    def command(self, effective_level):
        """The verified downlink shape, compact and newline-free."""
        return json.dumps(
            {"target_id": self.target_id, "risk": effective_level},
            separators=(",", ":"),
        )


# cane_node_risk / veh_node_risk: 노드가 스스로 브로드캐스트한 risk_level.
# 지팡이는 max(자체판단, 내려받은 RSU risk)를 실어 보내므로, effective_level과
# 나란히 놓으면 다운링크가 실제로 수신·반영됐는지 한 파일에서 읽을 수 있다.
# 단 같은 행의 두 값은 같은 시점이 아니다 - node_risk는 직전에 받은 값이라
# 방금 보낸 risk의 반영은 다음 행부터 나타난다.
CSV_FIELDS = (
    "pc_time",
    "cane_seq",
    "computed_level",
    "effective_level",
    "cane_node_risk",
    "veh_node_risk",
    "trusted",
    "reason",
    "target_id",
    "distance_m",
    "closing_mps",
    "ttc_s",
    "risk_score",
    "rule_level",
    "rule_reason",
    "model_proba",
    "level_source",
    "cane_gps_valid",
    "cane_lat",
    "cane_lng",
    "cane_speed_mps",
    "cane_heading_deg",
    "veh_gps_valid",
    "veh_lat",
    "veh_lng",
    "veh_speed_mps",
    "veh_heading_deg",
    # 차량의 지팡이 기준 상대위치(로컬 ENU). 차량 1인칭 화면이 차량 GPS(veh_lat/lng
    # 는 자주 무효)가 아니라 이 상대좌표를 heading 으로 회전해 보행자를 배치한다.
    "rel_east_m",
    "rel_north_m",
)


def csv_row(now, store, transmitter, decision, assessment, gate=None,
            kinematics=None):
    cane = store.latest["cane"]
    vehicle = store.latest["vehicle"]
    return {
        "pc_time": round(now, 3),
        "cane_seq": store.latest["cane"]["seq"],
        "computed_level": decision.computed_level,
        "effective_level": decision.effective_level,
        "cane_node_risk": cane["node_risk"],
        "veh_node_risk": vehicle["node_risk"],
        "trusted": int(decision.trusted),
        "reason": decision.reason,
        "target_id": transmitter.target_id,
        # closing_mps: 양수 = 접근 중, 음수 = 멀어지는 중.
        "distance_m": round(assessment.distance_m, 2),
        "closing_mps": round(assessment.closing_los, 2),
        "ttc_s": round(assessment.ttc, 2),
        "risk_score": assessment.final_score,
        # 규칙 자체의 판정과 모델이 얹은 결과를 나란히 남긴다. 실험 후 "모델이
        # 없었다면 어땠을까"를 이 두 열만으로 재구성할 수 있다.
        "rule_level": assessment.risk_level,
        "rule_reason": assessment.reason,
        "model_proba": None if gate is None or gate.proba is None else round(gate.proba, 4),
        "level_source": "table" if gate is None else gate.source,
        "cane_gps_valid": cane["gps_valid"],
        "cane_lat": cane["lat"],
        "cane_lng": cane["lng"],
        "cane_speed_mps": cane["speed_mps"],
        "cane_heading_deg": cane["heading_deg"],
        "veh_gps_valid": vehicle["gps_valid"],
        # 차량은 GPS 픽스가 없을 때가 잦아 없으면 빈 값으로 둔다(화면이 상대좌표를 씀).
        "veh_lat": vehicle.get("lat"),
        "veh_lng": vehicle.get("lng"),
        "veh_speed_mps": vehicle["speed_mps"],
        "veh_heading_deg": vehicle["heading_deg"],
        "rel_east_m": None if kinematics is None else round(kinematics.rel_east, 2),
        "rel_north_m": None if kinematics is None else round(kinematics.rel_north, 2),
    }


def append_row(csv_path, row):
    path = Path(csv_path)
    needs_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        if needs_header:
            writer.writeheader()
        writer.writerow(row)


def format_tx(decision, cane_node_risk, veh_node_risk):
    suppressed = (
        f" computed={decision.computed_level}->{decision.effective_level}"
        if decision.computed_level != decision.effective_level
        else ""
    )
    # node_risk는 직전에 받은 값이다. 방금 보낸 risk의 에코는 다음 [TX] 줄에 나온다.
    return (
        f"[TX] risk={decision.effective_level} reason={decision.reason}{suppressed}"
        f" cane_node_risk={cane_node_risk} veh_node_risk={veh_node_risk}"
    )


class RawLog:
    """시리얼로 오간 줄을 방향 표시와 함께 시간순 그대로 남긴다.

    CSV는 판단 결과만 담고, 노드가 실제로 무엇을 보냈는지는 어디에도 안 남는다.
    통신 자체가 의심스러울 때 볼 곳이 이 파일이다. 파싱에 실패한 줄도 그대로
    남겨야 원인을 알 수 있으므로 기록은 파싱보다 먼저 한다.
    """

    def __init__(self, path):
        self.handle = Path(path).open("a", encoding="utf-8")

    def write(self, direction, line):
        # 줄마다 flush: 실험 중 전원이 끊겨도 직전까지는 남는다.
        self.handle.write(f"{time.time():.3f} {direction} {line}\n")
        self.handle.flush()

    def close(self):
        self.handle.close()


class RiskSender:
    """Scores each record and pushes the resulting level through the transmitter."""

    def __init__(self, pipeline, transmitter, transport, csv_path, gate_params,
                 stabilizer=None, raw_log=None, model_gate=None, live_state=None,
                 dist_trend_min_mps=TREND_MIN_MPS):
        self.pipeline = pipeline
        self.transmitter = transmitter
        self.transport = transport
        self.csv_path = csv_path
        self.gate_params = gate_params
        self.stabilizer = stabilizer
        self.raw_log = raw_log
        # 화면용 최신 상태. None이면 화면 경로 자체가 없는 것과 같다.
        self.live_state = live_state
        # 모델 판정은 규칙 위에 얹힌다(max). 게이트가 없거나 모델 파일이 없으면
        # 규칙만으로 이전과 똑같이 돈다.
        self.model_gate = model_gate or ModelGate()
        # 접근속도 EMA 상태. None이면 아직 첫 샘플(원값 그대로).
        self.closing_ema = None
        # 노드별 (pc_time, 도플러 속도) 최근 이력 — 유효 fix만. 도플러 상한용.
        self._doppler_hist = {"cane": [], "vehicle": []}
        # 거리추세 융합 켜기/끄기. 기본 켜짐(TREND_MIN_MPS)이지만 실제 발동은 RSSI
        # 이중확인(rssi_dist "다가옴" + 12 m 안)이 있어야 한다 — GPS 추세만으론 점프
        # 가짜접근으로 오탐 51%(8/19)라서. rssi_dist 없으면(구 펌웨어) 자동 OFF = 기존
        # 동작. 0을 주면 융합 자체를 끈다(클램프/도플러만 검증할 때).
        self.dist_trend_min_mps = dist_trend_min_mps
        self._dist_hist = []  # (pc_time, raw distance_m)
        # RSSI 방향 융합용 (pc_time, rssi_dist) — 차량 rssi_dist 유효 패킷만.
        self._rssi_hist = []

    def _record_doppler(self, row):
        if row["type"] not in self._doppler_hist or not _gps_trusted(row["gps_valid"]):
            return
        hist = self._doppler_hist[row["type"]]
        hist.append((to_float(row["pc_time"]), abs(to_float(row["speed_mps"]))))
        cutoff = hist[-1][0] - DOPPLER_PEAK_WINDOW_S
        while hist and hist[0][0] < cutoff:
            hist.pop(0)

    def _peak_doppler(self, node_type):
        hist = self._doppler_hist[node_type]
        return max(v for _, v in hist) if hist else None

    def _trend_closing(self, now, distance_m):
        """최근 DIST_TREND_WINDOW_S 동안 거리가 준 속도(m/s). 이력이 짧으면 0.

        (pc_time, 거리)를 쌓고, 창 시작 시점의 거리와 지금 거리의 차를 창 길이로
        나눈다. 정지 잡음은 몇 초 꾸준히 줄지 않아 값이 작고, 느린 실접근은 크다.
        사용 여부(RSSI 이중확인·범위)는 호출자가 정한다.
        """
        self._dist_hist.append((now, distance_m))
        cutoff = now - DIST_TREND_WINDOW_S - 1.0
        while self._dist_hist and self._dist_hist[0][0] < cutoff:
            self._dist_hist.pop(0)
        target = now - DIST_TREND_WINDOW_S
        old = None
        for t, d in self._dist_hist:
            if t <= target:
                old = d
            else:
                break
        if old is None:
            return 0.0
        return (old - distance_m) / DIST_TREND_WINDOW_S

    def _record_rssi(self, row):
        """차량 패킷의 rssi_dist(RSSI 추정거리)를 이력에 쌓는다. 유효값(>0)만."""
        if row["type"] != "vehicle":
            return
        rd = row.get("rssi_dist")
        try:
            rd = float(rd)
        except (TypeError, ValueError):
            return
        if rd <= 0:
            return
        now = to_float(row["pc_time"])
        self._rssi_hist.append((now, rd))
        cutoff = now - RSSI_TREND_WINDOW_S - 1.0
        while self._rssi_hist and self._rssi_hist[0][0] < cutoff:
            self._rssi_hist.pop(0)

    def _rssi_approaching(self, now):
        """RSSI 추정거리가 최근 창 동안 줄었나. True(다가옴)/False(멀어짐·정지)/None(데이터 없음).

        rssi_dist가 패킷에 없으면(구 펌웨어) 이력이 비어 None → 융합 자동 OFF.
        """
        target = now - RSSI_TREND_WINDOW_S
        old = None
        for t, d in self._rssi_hist:
            if t <= target:
                old = d
            else:
                break
        if old is None or not self._rssi_hist:
            return None
        return (old - self._rssi_hist[-1][1]) >= RSSI_APPROACH_MIN_M

    def process_line(self, raw_line, source_mode):
        line = raw_line.strip()
        if not line:
            return
        if self.raw_log is not None:
            self.raw_log.write("RX", line)
        try:
            payload = json.loads(line)
            if not isinstance(payload, dict):
                raise ValueError("top-level JSON must be an object")
            row = normalize_record(payload, source_mode)
        except (json.JSONDecodeError, ValueError) as exc:
            print(f"[WARN] parse_failed error={exc} source={source_mode}", file=sys.stderr)
            return

        if row["type"] in RSU_ACK_TYPES:
            return

        if not self.pipeline.store.update(row):
            print(f"[WARN] ignored_type type={row['type']!r}", file=sys.stderr)
            return
        self._record_doppler(row)
        self._record_rssi(row)

        self.pipeline.observe(row)
        result = self.pipeline.compute()
        if result is None:
            return

        now, raw, filtered = result
        # 접근속도 평활(EMA): 노이즈로 튀는 closing_los를 부드럽게 해 TTC를 안정화.
        # 첫 샘플은 원값 그대로라 기존 동작과 동일하게 시작한다.
        if CLOSING_EMA_ALPHA < 1.0:
            if self.closing_ema is None:
                self.closing_ema = filtered.closing_los
            else:
                self.closing_ema = (
                    CLOSING_EMA_ALPHA * filtered.closing_los
                    + (1.0 - CLOSING_EMA_ALPHA) * self.closing_ema
                )
            filtered = replace(filtered, closing_los=self.closing_ema)
        store = self.pipeline.store
        vehicle_speed = to_float(store.latest["vehicle"]["speed_mps"])
        cane_gps_valid = store.latest["cane"]["gps_valid"]
        # 도플러 물리 상한(step7): 두 노드 GPS가 모두 유효할 때만, 최근 창의 최대
        # 도플러 쌍을 넘겨 채점 접근속도를 그 합+여유로 자른다. 무효 fix의 속도는
        # 0이라 믿을 수 없고, 순간값은 위치 추정의 지연 때문에 진짜 접근을 자른다.
        doppler_speeds = None
        if (_gps_trusted(cane_gps_valid)
                and _gps_trusted(store.latest["vehicle"]["gps_valid"])):
            veh_peak, cane_peak = self._peak_doppler("vehicle"), self._peak_doppler("cane")
            if veh_peak is not None and cane_peak is not None:
                doppler_speeds = (veh_peak, cane_peak)
        # 거리추세: 저속 실접근(순간 접근속도가 데드밴드 아래)을 거리 감소로 잡는다.
        # 필터 거리는 차량 ZUPT 로 강하게 눌려 뒤처지므로, 두 노드의 실제 좌표로 잰
        # raw 거리로 추세를 계산한다(4 s 창이 그 잡음을 평균한다).
        trend_closing = self._trend_closing(now, raw.distance_m)
        # RSSI 이중확인: GPS 추세만으론 점프가 만든 가짜 접근으로 오탐 51%(8/19). 융합이
        # 켜져 있고(dist_trend_min_mps>0), 차량이 지팡이를 직접 들은 RSSI 거리도 "다가오는
        # 중"이며, RSSI 유효범위(12 m) 안일 때만 추세를 인정한다. rssi_dist 없으면
        # rssi_approaching=None → 추세 무효 → 기존 동작(안전).
        if not (self.dist_trend_min_mps > 0
                and self._rssi_approaching(now) is True
                and raw.distance_m <= RSSI_RANGE_M):
            trend_closing = 0.0
        assessment = assess_risk(filtered, vehicle_speed, doppler_speeds_mps=doppler_speeds,
                                 trend_closing_mps=trend_closing, **self.gate_params)

        # 모델은 규칙 위에 얹힌다. 안전 하한(step7)이 낸 레벨 3은 모델이 낮추지
        # 못하고, 모델이 안전하다고 해도 규칙 레벨은 그대로 간다 - 검증되지 않은
        # 쪽이 검증된 쪽을 무르게 만들지 않기 위해서다.
        gate = self.model_gate.apply(
            assessment.risk_level, now, self.pipeline.last_states
        )
        # Hysteresis (step8_stability): rises pass through instantly, drops must
        # persist hold_s. Applied before trust gating so an untrusted fix still
        # clears to 0 immediately inside the transmitter.
        level = gate.level
        if self.stabilizer is not None:
            # 지나간 차(확실히 멀어지는 중)면 낮은 등급을 홀드 없이 즉시 채택한다.
            level = self.stabilizer.stabilize(level, now, receding=assessment.receding)
        decision = self.transmitter.consider(level, cane_gps_valid, now)
        # 화면은 전송 여부와 무관하게 매 판정마다 갱신한다. 다운링크는 등급이
        # 바뀌거나 heartbeat일 때만 나가지만 거리와 TTC는 그 사이에도 계속
        # 변하고, 화면이 보여줘야 하는 것이 바로 그 변화다.
        #
        # 감싸는 이유: 화면은 곁가지고 경보가 본체다. 여기서 무슨 일이 나도
        # 아래 다운링크까지 같이 죽으면 안 된다.
        if self.live_state is not None:
            try:
                self.live_state.update(
                    now, store, decision, assessment, filtered,
                    gate=gate, target_id=self.transmitter.target_id,
                )
            except Exception as exc:  # noqa: BLE001 - 경보를 지키는 쪽이 우선
                print(f"[WARN] live_state_failed error={exc}", file=sys.stderr)
        if not decision.should_send:
            return

        command = self.transmitter.command(decision.effective_level)
        if self.raw_log is not None:
            self.raw_log.write("TX", command)
        self.transport(command)
        print(
            format_tx(
                decision,
                store.latest["cane"]["node_risk"],
                store.latest["vehicle"]["node_risk"],
            ),
            flush=True,
        )
        append_row(
            self.csv_path,
            csv_row(now, store, self.transmitter, decision, assessment, gate,
                    kinematics=filtered),
        )


def inject_vehicle(vehicle, sender, source_mode, now):
    cane = sender.pipeline.store.latest.get("cane")
    if cane is None or not has_position(cane) or not vehicle.is_due(now):
        return
    payload, distance_m = vehicle.record(
        to_float(cane["lat"]), to_float(cane["lng"]), now
    )
    print(f"[TESTVEH] seq={payload['seq']} distance_m={distance_m:.1f}", flush=True)
    sender.process_line(json.dumps(payload), source_mode)


def serial_transport(connection):
    def send(command):
        connection.write((command + "\n").encode("utf-8"))
        connection.flush()
    return send


def stdout_transport(command):
    # No hardware: show what would have gone on the wire.
    print(f"[WIRE] {command}", file=sys.stderr, flush=True)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Transmit step 7 risk_level to the RSU downlink."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial port")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate")
    parser.add_argument(
        "--source-mode",
        choices=SOURCE_MODES,
        default="test",
        help="origin of this input stream (default: test)",
    )
    parser.add_argument("--csv", default="step8_risk_tx_log.csv", help="output CSV path")
    parser.add_argument(
        "--raw-log",
        default=None,
        help="시리얼로 오간 모든 줄(RX/TX)을 이 파일에 그대로 기록",
    )
    parser.add_argument(
        "--fresh-window-ms",
        type=int,
        default=int(FRESH_WINDOW_S * 1000),
        help="how recent a record must be to count as READY (default: 500)",
    )
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="read JSON lines from stdin and print commands instead of using serial",
    )
    parser.add_argument(
        "--test-vehicle",
        action="store_true",
        help="inject a simulated vehicle closing in on the cane (step 5)",
    )
    parser.add_argument("--vehicle-speed", type=float, default=SPEED_MPS)
    parser.add_argument("--vehicle-start-m", type=float, default=START_DISTANCE_M)
    parser.add_argument("--target-id", type=int, default=0, help="downlink target (0=broadcast)")
    parser.add_argument(
        "--model",
        default=None,
        help="모델 파일 경로 (기본: 이 파일 옆의 risk_model.json)",
    )
    parser.add_argument(
        "--no-model",
        action="store_true",
        help="모델을 쓰지 않고 규칙만으로 동작한다",
    )
    parser.add_argument(
        "--tx-heartbeat-s",
        type=float,
        default=HEARTBEAT_S,
        help=f"resend the same level at least this often (default: {HEARTBEAT_S})",
    )
    parser.add_argument(
        "--tx-untrusted",
        action="store_true",
        help="transmit risk even when the cane fix is a fallback (gps_valid=0)",
    )
    parser.add_argument(
        "--level-hold-s",
        type=float,
        default=HOLD_S,
        help=f"a lower level must persist this long before the alarm drops; "
        f"0 disables the hysteresis (default: {HOLD_S})",
    )
    parser.add_argument(
        "--live-port",
        type=int,
        default=live_server.DEFAULT_PORT,
        help=f"차량뷰 화면을 내주는 포트. 0이면 화면을 끈다 "
        f"(기본: {live_server.DEFAULT_PORT})",
    )
    parser.add_argument("--dcpa-near-m", type=float, default=DCPA_NEAR_M)
    parser.add_argument("--dcpa-far-m", type=float, default=DCPA_FAR_M)
    parser.add_argument("--dcpa-floor", type=float, default=DCPA_FLOOR)
    parser.add_argument(
        "--floor-ttc-s",
        type=float,
        default=T_FLOOR_TTC_S,
        help=f"TTC가 이 값 아래면 무조건 레벨 3. 0이면 하한을 끈다 "
        f"(기본: {T_FLOOR_TTC_S}). 실험 중 같은 시나리오를 값만 바꿔 찍으면 "
        f"하한이 실제로 몇 번 울리는지 비교할 수 있다",
    )
    parser.add_argument(
        "--vehicle-bias-file",
        default=str(VEHICLE_BIAS_FILE),
        help="calibrate_bias.py 가 저장한 차량 GPS 상대 바이어스 파일. 없으면 보정 없이 동작",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    store = StateStore(fresh_window_s=args.fresh_window_ms / 1000)
    bias_east, bias_north, bias_meta = load_vehicle_bias(args.vehicle_bias_file)
    pipeline = KinematicsPipeline(store, vehicle_bias=(bias_east, bias_north))
    transmitter = RiskTransmitter(
        target_id=args.target_id,
        heartbeat_s=args.tx_heartbeat_s,
        allow_untrusted=args.tx_untrusted,
    )
    gate_params = {
        "near_m": args.dcpa_near_m,
        "far_m": args.dcpa_far_m,
        "floor": args.dcpa_floor,
        "floor_ttc_s": args.floor_ttc_s,
    }
    stabilizer = LevelStabilizer(hold_s=args.level_hold_s) if args.level_hold_s > 0 else None
    raw_log = RawLog(args.raw_log) if args.raw_log else None
    print(
        f"[INFO] source_mode={args.source_mode} csv={args.csv} target_id={args.target_id} "
        f"heartbeat_s={args.tx_heartbeat_s} tx_untrusted={args.tx_untrusted} "
        f"level_hold_s={args.level_hold_s} floor_ttc_s={args.floor_ttc_s} "
        f"raw_log={args.raw_log} live_port={args.live_port}",
        file=sys.stderr,
    )
    for line in vehicle_bias_messages(bias_east, bias_north, bias_meta, time.time()):
        print(line, file=sys.stderr)

    # 화면 서버. 못 띄워도 None이 돌아올 뿐이고 경보는 그대로 나간다.
    live_state = live_server.start_or_none(
        args.live_port, on_message=lambda text: print(text, file=sys.stderr)
    )

    # 모델 파일이 손상돼도 엔진은 떠야 한다. 로드 실패 시 규칙만으로 동작한다
    # (파일 부재는 load 안에서 이미 규칙 폴백; 여기선 파싱·형식 오류를 잡는다).
    try:
        model_gate = (ModelGate() if args.no_model
                      else ModelGate.load(args.model))
    except Exception as exc:  # noqa: BLE001 - 손상 모델이 경보 엔진을 못 죽이게
        print(f"[WARN] model_load_failed error={exc!r} -> 규칙만으로 동작",
              file=sys.stderr)
        model_gate = ModelGate()

    vehicle = None
    if args.test_vehicle:
        vehicle = TestVehicle(
            start_distance_m=args.vehicle_start_m, speed_mps=args.vehicle_speed
        )
        print(
            f"[INFO] test_vehicle start_m={args.vehicle_start_m} speed={args.vehicle_speed}",
            file=sys.stderr,
        )

    if args.stdin:
        sender = RiskSender(
            pipeline, transmitter, stdout_transport, args.csv, gate_params,
            stabilizer=stabilizer, raw_log=raw_log, model_gate=model_gate,
            live_state=live_state,
        )
        _run(sender, sys.stdin, args.source_mode, vehicle)
        if raw_log is not None:
            raw_log.close()
        return

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required for serial mode: pip3 install pyserial") from exc

    with serial.Serial(args.port, args.baud, timeout=1) as connection:
        connection.reset_input_buffer()
        print(f"[INFO] port={args.port} baud={args.baud}", file=sys.stderr)
        sender = RiskSender(
            pipeline, transmitter, serial_transport(connection), args.csv, gate_params,
            stabilizer=stabilizer, raw_log=raw_log, model_gate=model_gate,
            live_state=live_state,
        )
        _run(sender, _serial_lines(connection), args.source_mode, vehicle)

    if raw_log is not None:
        raw_log.close()


def _serial_lines(connection):
    while True:
        raw = connection.readline()
        if raw:
            yield raw.decode("utf-8", errors="replace")


def _run(sender, lines, source_mode, vehicle):
    try:
        for line in lines:
            # 한 줄 처리가 실패해도(시리얼 쓰기 오류·CSV append·계산 예외) 경보
            # 루프는 계속 돌아야 한다. 여기서 루프가 죽으면 그 자체가 미탐이다.
            # 그래서 한 줄의 예외는 로그만 남기고 다음 줄로 넘어간다.
            try:
                sender.process_line(line, source_mode)
                if vehicle is not None:
                    inject_vehicle(vehicle, sender, source_mode, time.time())
            except (KeyboardInterrupt, SystemExit):
                raise
            except Exception as exc:  # noqa: BLE001 - 루프 생존이 우선
                print(f"[WARN] process_failed error={exc!r}", file=sys.stderr)
    except KeyboardInterrupt:
        print("\n[INFO] stopped", file=sys.stderr)


if __name__ == "__main__":
    main()
