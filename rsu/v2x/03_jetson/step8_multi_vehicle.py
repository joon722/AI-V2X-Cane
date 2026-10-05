#!/usr/bin/env python3
"""차량 여러 대를 한 대씩 따로 추적·판정하고, 위험한 차가 있으면 모두 울린다.

왜 필요한가
    step4 StateStore는 노드 종류(cane / vehicle)마다 최신 한 줄만 들고 있다. 차가
    두 대면 두 차의 패킷이 같은 "vehicle" 칸을 번갈아 덮어쓰고, step6의 차량 칼만
    추적기 하나가 A 위치와 B 위치 사이를 오간다. 거리 미분도 두 차를 섞는다 -
    가짜 접근속도·TTC가 나오고, 결국 (섞인) 한 대만 추적된다. v4.1에서 고친
    "최근접 보행자가 바뀔 때 다른 사람과의 거리를 미분" 버그와 같은 종류다.

어떻게 고쳤나 - step4~8은 판정 로직을 한 줄도 고치지 않는다
    차량 node_id마다 "차선(lane)" 하나를 둔다. 차선 = 기존 step8 한 벌
    (StateStore + KinematicsPipeline + RiskSender: 칼만, 접근속도 EMA, 도플러 상한,
    거리추세·RSSI 융합, UWB, 홀드, gap-hold 전부)이다.

        지팡이 행  -> 모든 차선에 (각 차선이 "자기 차 vs 지팡이"를 판정)
        차량 행    -> 그 node_id의 차선에만 (없으면 새로 연다)
        uwb 행     -> 측거를 보낸 차(node_id)의 차선에만

    차선마다 자기 등급이 나오고(차선별 CSV에 따로 기록), 지팡이에는 그중 가장 높은
    등급을 보낸다. 차가 한 대뿐이면 차선도 하나라 기존 step8과 같은 판정이 나온다
    (test_step8_multi_vehicle.py가 확인한다).

다운링크 (--downlink)
    max (기본)  차선 등급의 최댓값 하나를 target_id=0으로 방송한다. 지금 RSU 브리지
                (rsu/v2x/v2x_bridge/v2x_bridge.ino)는 target_id를 무시하고
                MSG_RSU_REPLY를 모두에게 방송하므로 이 방식만 안전하다. 어느 차든
                위험하면 지팡이와 차량 스피커가 모두 울린다.
    per-target  차량에는 자기 등급을 target_id=<차 node_id>로, 지팡이에는 최댓값을
                target_id=<지팡이 node_id>로 따로 보낸다. 지팡이·차량 V5 펌웨어에는
                target_id 필터가 이미 있지만, 브리지가 target_id대로 보내도록 바뀌기
                전에는 켜면 안 된다 - 노드가 남의 등급까지 받아 경보가 깜빡인다.

실행 (step8_send_risk.py와 옵션이 같고 아래만 더 있다)
    python3 step8_multi_vehicle.py --stdin --test-vehicles 2   # 하드웨어 없이 2대 접근
    python3 step8_multi_vehicle.py --port /dev/ttyUSB0 --source-mode real
"""

import contextlib
import csv
import io
import json
from collections import deque
import sys
import time
from dataclasses import dataclass, replace
from pathlib import Path

import live_server
import step8_send_risk as s8
from model_features import StreamingFeatures
from model_gate import ModelGate
from step4_state_store import StateStore, has_position
from step5_test_vehicle import NODE_ID as TEST_NODE_ID
from step5_test_vehicle import TestVehicle
from step6_kinematics import KinematicsPipeline, to_float
from step7_risk import TREND_MIN_MPS
from step8_stability import HOLD_S, LevelStabilizer


DOWNLINK_MODES = ("max", "per-target")

# 차선의 등급을 믿는 시간. 차선은 지팡이 행을 받을 때마다 판정(또는 gap-hold)을
# 하므로 보통은 계속 새 등급이 나온다. 지팡이까지 끊겨 차선에 아무 행도 안 오면
# 등급이 그대로 굳는데, 그 굳은 등급이 최댓값을 영원히 붙들면 안 된다. gap-hold
# 상한(3 s)과 홀드(1 s)를 다 쓰고도 새 판정이 없으면 0으로 본다.
LANE_LEVEL_TTL_S = s8.GAP_HOLD_CAP_S + HOLD_S

# 차가 이만큼 조용하고 등급도 0이면 차선을 닫는다. 지나간 차의 칼만·이력을 계속
# 들고 있을 이유가 없다. 다시 나타나면 새 차선으로 처음부터 추적한다(어차피 step6이
# 2 s 넘는 공백이면 트랙을 새로 시작한다).
LANE_EXPIRE_S = 30.0

# 새 차선에 미리 넣는 지팡이 이력의 길이. 차선마다 지팡이 칼만 추적기가 따로라서,
# 차가 나타난 순간 지팡이 추적을 처음부터 시작하면 걷는 보행자의 속도 추정이 다시
# 수렴하는 동안 접근속도·TTC가 "그 차만 있었을 때"와 달라진다(SUMO 재생에서 경보
# 구간 42개 중 1개가 어긋남). 칼만의 기억은 몇 초면 사라지므로 10 s면 충분하고,
# 그보다 앞은 step6이 2 s 공백에서 트랙을 새로 시작하는 것과 같은 이유로 무의미하다.
CANE_PRIME_S = 10.0

# 집계 CSV에 step8 열 뒤로 덧붙이는 열. 행 내용(거리·TTC 등)은 지금 가장 위험한
# 차(primary)의 것이다. 열 이름으로 읽는 도구(stream_live 등)는 그대로 동작한다.
MULTI_CSV_FIELDS = s8.CSV_FIELDS + ("vehicle_id", "n_vehicles", "vehicle_levels")


def vehicle_key(row):
    """차선을 가르는 차량 식별자. node_id, 없으면 src_mac, 둘 다 없으면 "unknown"."""
    node = str(row.get("node_id", "")).strip()
    if node and node != "0":
        return node
    mac = str(row.get("src_mac", "")).strip()
    return mac or "unknown"


def target_id_of(key):
    """다운링크 target_id는 숫자다. 숫자가 아닌 식별자(MAC 등)는 0(방송)."""
    try:
        return int(key)
    except (TypeError, ValueError):
        return 0


@dataclass(frozen=True)
class Judgement:
    """차선 RiskSender가 한 번 판정한 결과 한 벌 (live_state.update 인자 그대로)."""

    now: float
    store: StateStore
    decision: s8.TxDecision
    assessment: object
    kinematics: object
    gate: object


class LaneScreen:
    """차선의 live_state 자리. 화면 대신 마지막 판정을 받아 둔다.

    RiskSender는 판정할 때마다 live_state.update()를 부른다. 그 인자가 곧 판정
    결과라서, step8을 고치지 않고도 차선의 거리·TTC·등급을 꺼내 올 수 있다.
    """

    def __init__(self):
        self.last = None

    def update(self, now, store, decision, assessment, kinematics, gate=None, target_id=0):
        self.last = Judgement(now, store, decision, assessment, kinematics, gate)


class LaneTransmitter(s8.RiskTransmitter):
    """consider() 호출 수를 센다. 이번 줄에서 차선이 등급을 냈는지 알기 위해서다."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.considered = 0

    def consider(self, *args, **kwargs):
        self.considered += 1
        return super().consider(*args, **kwargs)


@dataclass(frozen=True)
class LaneConfig:
    """차선마다 똑같이 쓰는 step8 설정. step8 main()이 RiskSender에 넘기는 값과 같다."""

    gate_params: dict
    fresh_window_s: float = s8.FRESH_WINDOW_S
    heartbeat_s: float = s8.HEARTBEAT_S
    allow_untrusted: bool = False
    hold_s: float = HOLD_S
    gap_hold_cap_s: float = s8.GAP_HOLD_CAP_S
    dist_trend_min_mps: float = TREND_MIN_MPS
    uwb_enabled: bool = True
    vehicle_bias: tuple = (0.0, 0.0)
    model: object = None          # TreeEnsemble 하나를 차선들이 같이 쓴다(읽기 전용)
    # 차선별 CSV 폴더. None이면 차선 RiskSender에 csv_path=None이 그대로 넘어가므로,
    # 재생 도구처럼 s8.append_row를 바꿔 끼울 때만 None을 쓴다(main은 항상 준다).
    lanes_dir: Path | None = None


class VehicleLane:
    """차량 한 대 몫의 step8 한 벌."""

    def __init__(self, key, config, transport, echo=False):
        self.key = key
        # 차선 RiskSender가 stdout에 찍는 [TX] 줄을 보여 줄지. max 모드에선 차선이 실제로
        # 보내지 않으므로 숨긴다(진짜 송신 줄과 섞이면 현장에서 헷갈린다). per-target에선
        # 그 줄이 곧 그 차로 간 송신이라 차량 id를 붙여 보여 준다.
        self.echo = echo
        store = StateStore(fresh_window_s=config.fresh_window_s)
        pipeline = KinematicsPipeline(store, vehicle_bias=config.vehicle_bias)
        self.transmitter = LaneTransmitter(
            target_id=target_id_of(key),
            heartbeat_s=config.heartbeat_s,
            allow_untrusted=config.allow_untrusted,
        )
        self.screen = LaneScreen()
        # 모델 피처는 후방차분이라 상태가 있다. 차마다 따로 둬야 차끼리 섞이지 않는다.
        model_gate = (ModelGate(model=config.model, streamer=StreamingFeatures())
                      if config.model is not None else ModelGate())
        stabilizer = LevelStabilizer(hold_s=config.hold_s) if config.hold_s > 0 else None
        self.csv_path = None
        if config.lanes_dir is not None:
            safe = "".join(ch if ch.isalnum() else "_" for ch in key)
            self.csv_path = Path(config.lanes_dir) / f"veh_{safe}.csv"
        self.sender = s8.RiskSender(
            pipeline, self.transmitter, transport, self.csv_path, config.gate_params,
            stabilizer=stabilizer, raw_log=None, model_gate=model_gate,
            live_state=self.screen,
            dist_trend_min_mps=config.dist_trend_min_mps,
            gap_hold_cap_s=config.gap_hold_cap_s,
            uwb_enabled=config.uwb_enabled,
        )
        self.opened_at = None
        self.last_vehicle_at = None   # 이 차의 마지막 패킷 시각(pc_time)
        self.last_judged_at = None    # 마지막으로 등급을 낸 줄의 시각(pc_time)
        self.distance_source = "gps"

    @property
    def level(self):
        """지금 이 차선이 내보내는 등급(신뢰 게이트·홀드 적용 후). 아직 없으면 None."""
        return self.transmitter.last_level

    def level_at(self, now, ttl_s=LANE_LEVEL_TTL_S):
        """집계에 쓸 등급. 판정이 ttl_s보다 오래 없었으면 0, 판정 전이면 None."""
        if self.level is None:
            return None
        if self.last_judged_at is None or now - self.last_judged_at > ttl_s:
            return 0
        return self.level

    def prime(self, cane_row):
        """새 차선에 지나간 지팡이 행을 원래 시각 그대로 먼저 넣는다(CANE_PRIME_S 참조).

        안 넣으면 차선은 다음 지팡이 행(0.1~0.2 s 뒤)까지 기준 좌표(LocalFrame)가
        없어 첫 차량 행을 버리고, 지팡이 속도 추정도 처음부터 다시 배운다 - 단일
        차량에서도 기존 step8과 판정이 어긋난다. RiskSender.process_line이 지팡이 행에 하는 기록 단계
        (store 갱신, 도플러 이력, 칼만 관측)만 그대로 밟는다 - 차량이 아직 없어
        판정·송신 단계는 어차피 아무 일도 하지 않는다.
        """
        try:
            row = dict(cane_row)
            self.sender.pipeline.store.update(row)
            self.sender._record_doppler(row)
            self.sender.pipeline.observe(row)
        except Exception as exc:  # noqa: BLE001 - 준비 실패는 다음 지팡이 행이 메운다
            print(f"[WARN] lane_prime_failed vehicle={self.key} error={exc!r}", file=sys.stderr)

    def feed(self, text, source_mode, now):
        """한 줄을 이 차선의 step8에 넣는다. 예외는 이 차선 안에서 끝낸다.

        한 차의 계산이 터져도 다른 차의 경보와 지팡이 다운링크는 계속 나가야 한다.
        """
        before = self.transmitter.considered
        captured = io.StringIO()
        try:
            with contextlib.redirect_stdout(captured):
                self.sender.process_line(text, source_mode)
        except Exception as exc:  # noqa: BLE001 - 차선 하나가 전체 경보를 죽이면 안 된다
            # 예외가 판정 뒤(송신·CSV 기록 중)에 났을 수도 있으므로 아래 판정 기록은
            # 그대로 진행한다 - 이미 낸 등급까지 "오래됨"으로 버리지 않게.
            print(f"[WARN] lane_failed vehicle={self.key} error={exc!r}", file=sys.stderr)
        finally:
            if self.echo:
                for printed in captured.getvalue().splitlines():
                    print(f"[veh={self.key}] {printed}", flush=True)
        if self.transmitter.considered != before:
            self.last_judged_at = now
            judged = self.screen.last
            if judged is not None and judged.gate is None:
                self.distance_source = "uwb_only"
            elif self.sender._uwb_fresh(now) is not None:
                self.distance_source = "uwb"
            else:
                self.distance_source = "gps"


class MultiVehicleSender:
    """줄을 차선에 나눠 넣고, 차선 등급을 모아 지팡이 다운링크를 낸다."""

    def __init__(self, config, transport, csv_path=None, downlink="max",
                 raw_log=None, live_state=None, lane_ttl_s=LANE_LEVEL_TTL_S,
                 lane_expire_s=LANE_EXPIRE_S):
        if downlink not in DOWNLINK_MODES:
            raise ValueError(f"downlink must be one of {DOWNLINK_MODES}: {downlink!r}")
        self.config = config
        self.transport = transport
        self.csv_path = csv_path
        self.downlink = downlink
        self.raw_log = raw_log
        self.live_state = live_state
        self.lane_ttl_s = lane_ttl_s
        self.lane_expire_s = lane_expire_s
        self.lanes = {}
        self.cane_row = None
        self._cane_history = deque()  # 최근 CANE_PRIME_S 동안의 지팡이 행(새 차선 준비용)
        # 지팡이 몫의 송신기. 신뢰 게이트는 차선마다 이미 적용됐으므로 여기선 열어 둔다
        # (신뢰 못 할 fix로 낸 등급은 차선에서 이미 0이 되어 올라온다).
        self.cane_transmitter = s8.RiskTransmitter(
            target_id=0, heartbeat_s=config.heartbeat_s, allow_untrusted=True)
        self._bias_warned = False

    # ------------------------------------------------------------ 입력
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
            # 모듈 경유로 부른다 - 재생 도구가 s8.normalize_record를 바꿔 원본
            # 시각을 주입하면 차선과 집계가 같은 시계를 쓴다.
            row = s8.normalize_record(payload, source_mode)
        except (json.JSONDecodeError, ValueError) as exc:
            print(f"[WARN] parse_failed error={exc} source={source_mode}", file=sys.stderr)
            return

        kind = row["type"]
        now = to_float(row["pc_time"])
        if kind in s8.RSU_ACK_TYPES:
            return
        if kind == "cane":
            self.cane_row = row
            self._cane_history.append(row)
            while self._cane_history and now - to_float(self._cane_history[0]["pc_time"]) > CANE_PRIME_S:
                self._cane_history.popleft()
            if self.downlink == "per-target":
                self.cane_transmitter.target_id = target_id_of(row.get("node_id"))
            for lane in list(self.lanes.values()):
                lane.feed(line, source_mode, now)
        elif kind == "vehicle":
            lane = self._lane_for(vehicle_key(row), now)
            lane.last_vehicle_at = now
            lane.feed(line, source_mode, now)
        elif kind == "uwb":
            lane = self._uwb_lane(row)
            if lane is not None:
                lane.feed(line, source_mode, now)
            # UWB 행은 판정을 만들지 않는다(다음 지팡이/차량 행이 읽어 간다).
            return
        else:
            print(f"[WARN] ignored_type type={kind!r}", file=sys.stderr)
            return

        self._close_quiet_lanes(now)
        self._send_cane(now)

    def _lane_for(self, key, now):
        lane = self.lanes.get(key)
        if lane is not None:
            return lane
        transport = self._lane_transport()
        if self.config.lanes_dir is not None:
            Path(self.config.lanes_dir).mkdir(parents=True, exist_ok=True)
        lane = VehicleLane(key, self.config, transport, echo=self.downlink == "per-target")
        lane.opened_at = now
        for cane_row in self._cane_history:
            lane.prime(cane_row)
        self.lanes[key] = lane
        print(f"[LANE] open vehicle={key} lanes={len(self.lanes)}", file=sys.stderr, flush=True)
        if len(self.lanes) > 1 and not self._bias_warned and any(self.config.vehicle_bias):
            # calibrate_bias.py는 차 한 대와 지팡이 사이를 잰 값이다. 다른 차에도 같은
            # 값을 빼고 있다는 것을 남겨 둔다(차별 보정은 아직 없다).
            print("[WARN] vehicle_bias는 차 한 대 기준으로 잰 값인데 모든 차에 적용 중",
                  file=sys.stderr)
            self._bias_warned = True
        return lane

    def _lane_transport(self):
        """max 모드: 차선은 등급만 내고 보내지 않는다. per-target: 자기 차에 직접 보낸다."""
        if self.downlink == "max":
            return lambda command: None

        def send(command):
            if self.raw_log is not None:
                self.raw_log.write("TX", command)
            self.transport(command)
        return send

    def _uwb_lane(self, row):
        key = vehicle_key(row)
        lane = self.lanes.get(key)
        if lane is None and key == "unknown" and len(self.lanes) == 1:
            # 보낸 차를 모르는 옛 형식이라도 차가 한 대뿐이면 그 차의 것이다.
            lane = next(iter(self.lanes.values()))
        return lane

    def _close_quiet_lanes(self, now):
        for key, lane in list(self.lanes.items()):
            quiet = lane.last_vehicle_at is None or now - lane.last_vehicle_at > self.lane_expire_s
            if quiet and not lane.level_at(now, self.lane_ttl_s):
                del self.lanes[key]
                print(f"[LANE] close vehicle={key} lanes={len(self.lanes)}",
                      file=sys.stderr, flush=True)

    # ------------------------------------------------------------ 출력
    def lane_levels(self, now):
        """{차량: 집계용 등급}. 아직 판정이 없는 차선은 빠진다."""
        levels = {}
        for key, lane in self.lanes.items():
            level = lane.level_at(now, self.lane_ttl_s)
            if level is not None:
                levels[key] = level
        return levels

    def _primary(self, levels):
        """가장 위험한 차선. 등급이 같으면 더 가까운 차."""
        best = None
        for key, level in levels.items():
            judged = self.lanes[key].screen.last
            if judged is None:
                continue
            rank = (level, -judged.assessment.distance_m)
            if best is None or rank > best[0]:
                best = (rank, self.lanes[key])
        return None if best is None else best[1]

    def _send_cane(self, now):
        levels = self.lane_levels(now)
        if not levels:
            # 아직 어느 차선도 판정하지 않았다 - step8도 첫 판정 전엔 아무것도 안 보낸다.
            return
        top = max(levels.values())
        cane_gps = self.cane_row["gps_valid"] if self.cane_row is not None else 1
        decision = self.cane_transmitter.consider(top, cane_gps, now)
        primary = self._primary(levels)
        judged = None if primary is None else primary.screen.last
        if judged is not None:
            # 화면·CSV의 trusted는 실제로 그 등급을 낸 차선의 신뢰 판단을 따른다.
            decision = replace(decision, trusted=judged.decision.trusted)

        if self.live_state is not None and judged is not None:
            try:
                self.live_state.update(
                    judged.now, judged.store, decision, judged.assessment,
                    judged.kinematics, gate=judged.gate,
                    target_id=self.cane_transmitter.target_id,
                )
            except Exception as exc:  # noqa: BLE001 - 경보를 지키는 쪽이 우선
                print(f"[WARN] live_state_failed error={exc}", file=sys.stderr)

        if not decision.should_send:
            return
        command = self.cane_transmitter.command(decision.effective_level)
        if self.raw_log is not None:
            self.raw_log.write("TX", command)
        self.transport(command)
        summary = ",".join(f"{key}:{level}" for key, level in sorted(levels.items()))
        print(f"[TX] risk={decision.effective_level} reason={decision.reason} "
              f"vehicles={summary} primary={None if primary is None else primary.key}",
              flush=True)
        if self.csv_path is not None and judged is not None:
            row = s8.csv_row(now, judged.store, self.cane_transmitter, decision,
                             judged.assessment, judged.gate, kinematics=judged.kinematics,
                             distance_source=primary.distance_source)
            row["vehicle_id"] = primary.key
            row["n_vehicles"] = len(levels)
            row["vehicle_levels"] = summary.replace(",", ";")
            append_row(self.csv_path, row)

    @property
    def latest_cane(self):
        return self.cane_row


def append_row(csv_path, row):
    path = Path(csv_path)
    needs_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=MULTI_CSV_FIELDS)
        if needs_header:
            writer.writeheader()
        writer.writerow(row)


# ---------------------------------------------------------------- 시험 차량
def make_test_vehicles(count, start_distance_m, speed_mps):
    """지팡이를 향해 서로 다른 방향에서 다가오는 시험 차량 count대. [(node_id, TestVehicle)]

    방향은 360/count 간격, node_id는 step5 NODE_ID부터 1씩. 두 번째 차부터는 출발
    거리를 15 m씩 늘려 도착 시각을 엇갈리게 한다 - 동시에 오면 한 대만 추적돼도
    경보가 울려서 차선 분리를 확인할 수 없다.
    """
    vehicles = []
    for i in range(count):
        vehicles.append((TEST_NODE_ID + i, TestVehicle(
            start_distance_m=start_distance_m + 15.0 * i,
            speed_mps=speed_mps,
            bearing_deg=(360.0 / count) * i,
        )))
    return vehicles


def inject_test_vehicles(vehicles, sender, source_mode, now):
    cane = sender.latest_cane
    if cane is None or not has_position(cane):
        return
    for node_id, vehicle in vehicles:
        if not vehicle.is_due(now):
            continue
        payload, distance_m = vehicle.record(to_float(cane["lat"]), to_float(cane["lng"]), now)
        payload["node_id"] = node_id
        print(f"[TESTVEH] vehicle={node_id} seq={payload['seq']} distance_m={distance_m:.1f}",
              flush=True)
        sender.process_line(json.dumps(payload), source_mode)


# ---------------------------------------------------------------- CLI
def parse_args(argv=None):
    parser = s8.build_parser()
    parser.description = "차량 여러 대를 따로 판정해 RSU 다운링크로 보낸다 (step8 다중 차량판)."
    parser.add_argument(
        "--downlink", choices=DOWNLINK_MODES, default="max",
        help="max: 차선 최댓값 하나를 방송(지금 브리지용, 기본). per-target: 차량마다 "
        "자기 등급을 target_id로 - 브리지가 target_id를 지원할 때만",
    )
    parser.add_argument(
        "--test-vehicles", type=int, default=0,
        help="하드웨어 없이 지팡이로 다가오는 시험 차량 N대를 넣는다 (--test-vehicle 대신)",
    )
    parser.add_argument(
        "--lane-expire-s", type=float, default=LANE_EXPIRE_S,
        help=f"차가 이만큼 조용하고 등급이 0이면 차선을 닫는다 (기본: {LANE_EXPIRE_S})",
    )
    return parser.parse_args(argv)


def lanes_dir_for(csv_path):
    """집계 CSV 옆의 차선 CSV 폴더. stream_live는 logs/*.csv만 보므로 하위 폴더에 둔다."""
    path = Path(csv_path)
    return path.with_name(f"{path.stem}_vehicles")


def main():
    args = parse_args()
    bias_east, bias_north, bias_meta = s8.load_vehicle_bias(args.vehicle_bias_file)
    try:
        gate = ModelGate() if args.no_model else ModelGate.load(args.model)
    except Exception as exc:  # noqa: BLE001 - 손상 모델이 경보 엔진을 못 죽이게
        print(f"[WARN] model_load_failed error={exc!r} -> 규칙만으로 동작", file=sys.stderr)
        gate = ModelGate()
    config = LaneConfig(
        gate_params={
            "near_m": args.dcpa_near_m,
            "far_m": args.dcpa_far_m,
            "floor": args.dcpa_floor,
            "floor_ttc_s": args.floor_ttc_s,
        },
        fresh_window_s=args.fresh_window_ms / 1000,
        heartbeat_s=args.tx_heartbeat_s,
        allow_untrusted=args.tx_untrusted,
        hold_s=args.level_hold_s,
        gap_hold_cap_s=args.gap_hold_cap_s,
        dist_trend_min_mps=0.0 if args.no_fusion else TREND_MIN_MPS,
        uwb_enabled=not args.no_uwb,
        vehicle_bias=(bias_east, bias_north),
        model=gate.model,
        lanes_dir=lanes_dir_for(args.csv),
    )
    raw_log = s8.RawLog(args.raw_log) if args.raw_log else None
    print(
        f"[INFO] multi_vehicle downlink={args.downlink} source_mode={args.source_mode} "
        f"csv={args.csv} lanes_dir={config.lanes_dir} heartbeat_s={args.tx_heartbeat_s} "
        f"tx_untrusted={args.tx_untrusted} level_hold_s={args.level_hold_s} "
        f"floor_ttc_s={args.floor_ttc_s} gap_hold_cap_s={args.gap_hold_cap_s} "
        f"lane_expire_s={args.lane_expire_s} live_port={args.live_port}",
        file=sys.stderr,
    )
    if args.downlink == "per-target":
        print("[WARN] per-target: 브리지가 target_id대로 보내는 펌웨어여야 한다. 지금 "
              "v2x_bridge.ino는 target_id를 무시하고 방송한다", file=sys.stderr)
    for line in s8.vehicle_bias_messages(bias_east, bias_north, bias_meta, time.time()):
        print(line, file=sys.stderr)
    live_state = live_server.start_or_none(
        args.live_port, on_message=lambda text: print(text, file=sys.stderr)
    )

    test_count = args.test_vehicles or (1 if args.test_vehicle else 0)
    vehicles = make_test_vehicles(test_count, args.vehicle_start_m, args.vehicle_speed)

    def build(transport):
        return MultiVehicleSender(
            config, transport, csv_path=args.csv, downlink=args.downlink,
            raw_log=raw_log, live_state=live_state, lane_expire_s=args.lane_expire_s,
        )

    if args.stdin:
        _run(build(s8.stdout_transport), sys.stdin, args.source_mode, vehicles)
    else:
        connection = s8.open_bridge_serial(args.port, args.baud)
        try:
            connection.dtr = False
            connection.rts = False
            connection.reset_input_buffer()
            print(f"[INFO] port={args.port} baud={args.baud}", file=sys.stderr)
            _run(build(s8.serial_transport(connection)), s8._serial_lines(connection),
                 args.source_mode, vehicles)
        finally:
            connection.close()
    if raw_log is not None:
        raw_log.close()


def _run(sender, lines, source_mode, vehicles):
    """step8 _run과 같다: 한 줄의 예외로 경보 루프가 죽지 않는다."""
    try:
        for line in lines:
            try:
                sender.process_line(line, source_mode)
                if vehicles:
                    inject_test_vehicles(vehicles, sender, source_mode, time.time())
            except (KeyboardInterrupt, SystemExit):
                raise
            except Exception as exc:  # noqa: BLE001 - 루프 생존이 우선
                print(f"[WARN] process_failed error={exc!r}", file=sys.stderr)
    except KeyboardInterrupt:
        print("\n[INFO] stopped", file=sys.stderr)


if __name__ == "__main__":
    main()
