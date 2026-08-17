# -*- coding: utf-8 -*-
"""8/17 오후 raw 로그를 step8 조립 그대로 재생 — 코드 디렉터리 두 개(전/후) 비교.

replay_fusion.py와 같은 방식(원본 pc_time을 normalize_record(now=)로 주입)이되,
어느 코드로 돌릴지 --code-dir 로 받는다. 전/후를 각각 다른 프로세스로 돌려
같은 지표를 찍고 나란히 본다.

    python replay_0817.py --code-dir <구코드 폴더> --label before
    python replay_0817.py --code-dir <03_jetson>    --label after

정책은 8/17 젯슨 배포와 동일: heartbeat 0.2s, hold 2s, floor 2.0, 모델 OFF, 화면 없음.
"""
import argparse
import contextlib
import io
import json
import re
import statistics as st
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--code-dir", required=True)
parser.add_argument("--label", default="")
parser.add_argument("--logs-dir", default=r"C:\Users\user\OneDrive\바탕 화면\v2x(lux)\03_jetson\logs")
parser.add_argument("sessions", nargs="*", default=["174332", "180708", "181320", "182456"])
args = parser.parse_args()

CODE = str(Path(args.code_dir).resolve())
sys.path.insert(0, CODE)
sys.path.insert(1, str(Path(CODE) / "deploy"))

import step8_send_risk as s8  # noqa: E402
from step3_parse_v2x import normalize_record  # noqa: E402
from step4_state_store import FRESH_WINDOW_S, StateStore  # noqa: E402
from step6_kinematics import KinematicsPipeline  # noqa: E402
from step7_risk import DCPA_FAR_M, DCPA_FLOOR, DCPA_NEAR_M, T_FLOOR_TTC_S  # noqa: E402
from step8_stability import LevelStabilizer  # noqa: E402
from model_gate import ModelGate  # noqa: E402

LINE_RE = re.compile(r"^(\d+\.\d+) RX (\{.*\})\s*$")


class Replay:
    def __init__(self):
        store = StateStore(fresh_window_s=FRESH_WINDOW_S)
        pipeline = KinematicsPipeline(store)
        transmitter = s8.RiskTransmitter(target_id=0, heartbeat_s=0.2, allow_untrusted=False)
        gate_params = {"near_m": DCPA_NEAR_M, "far_m": DCPA_FAR_M, "floor": DCPA_FLOOR,
                       "floor_ttc_s": T_FLOOR_TTC_S}
        self.rows = []
        self.sender = s8.RiskSender(pipeline, transmitter, transport=lambda cmd: None,
                                    csv_path=None, gate_params=gate_params,
                                    stabilizer=LevelStabilizer(hold_s=2.0), raw_log=None,
                                    model_gate=ModelGate())
        self.now = None


def run_session(name):
    rp = Replay()
    s8.normalize_record = lambda payload, mode: normalize_record(payload, mode, now=rp.now)
    s8.append_row = lambda path, row: rp.rows.append(row)
    veh_gap_ends = []          # 차량이 >=2s 무음 뒤 다시 나타난 시각(재획득)
    last_veh = None
    with open(Path(args.logs_dir) / f"raw_{name}.log", encoding="utf-8", errors="replace") as fh, \
            contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        for line in fh:
            m = LINE_RE.match(line)
            if not m:
                continue
            ts = float(m.group(1))
            text = m.group(2)
            try:
                rec = json.loads(text)
            except ValueError:
                continue
            if rec.get("type") == "vehicle" and rec.get("gps_valid"):
                if last_veh is not None and ts - last_veh >= 2.0:
                    veh_gap_ends.append(ts)
                last_veh = ts
            rp.now = ts
            rp.sender.process_line(text, "real")
    return rp.rows, veh_gap_ends


def f(r, k):
    try:
        return float(r[k])
    except (TypeError, ValueError, KeyError):
        return float("nan")


def metrics(rows, veh_gap_ends):
    n = len(rows)
    if not n:
        return {"rows": 0}
    eff = [int(r["effective_level"]) for r in rows]
    t = [f(r, "pc_time") for r in rows]
    still = [i for i, r in enumerate(rows)
             if f(r, "veh_speed_mps") < 0.3 and f(r, "cane_speed_mps") < 0.3]
    still_alarm = sum(1 for i in still if eff[i] >= 1)
    still_l3 = sum(1 for i in still if eff[i] >= 3)
    still_ttc10 = sum(1 for i in still if f(rows[i], "ttc_s") < 10)
    # 재획득 뒤 1.5초 안의 L3(유령 접근)
    reacq_l3 = 0
    for g in veh_gap_ends:
        if any(eff[i] >= 3 and g <= t[i] <= g + 1.5 for i in range(n)):
            reacq_l3 += 1
    # 등급 변경 횟수(플래핑 지표)
    changes = sum(1 for a, b in zip(eff, eff[1:]) if a != b)
    # 실제 접근 에피소드: 차량이 움직이며(속도>=0.4) 8 m 안으로 들어온 구간, 3초 끊기면 새 에피소드
    eps = []
    cur = None
    for i, r in enumerate(rows):
        hit = f(r, "distance_m") < 8.0 and f(r, "veh_speed_mps") >= 0.4
        if hit:
            if cur is None or t[i] - cur["end"] > 3.0:
                cur = {"start": t[i], "end": t[i], "max_eff": eff[i], "min_d": f(r, "distance_m")}
                eps.append(cur)
            cur["end"] = t[i]
            cur["max_eff"] = max(cur["max_eff"], eff[i])
            cur["min_d"] = min(cur["min_d"], f(r, "distance_m"))
    # 에피소드 안 또는 앞뒤 1초에 L2+가 있었나
    detected = 0
    for e in eps:
        if any(eff[i] >= 2 and e["start"] - 1.0 <= t[i] <= e["end"] + 1.0 for i in range(n)):
            detected += 1
    ttc_fin = [f(r, "ttc_s") for r in rows if f(r, "ttc_s") < 9999]
    return {
        "rows": n,
        "alarm%": round(sum(1 for e in eff if e >= 1) / n * 100, 1),
        "L2+%": round(sum(1 for e in eff if e >= 2) / n * 100, 1),
        "L3rows": sum(1 for e in eff if e >= 3),
        "level_changes": changes,
        "still_rows": len(still),
        "still_alarm": still_alarm,
        "still_L3": still_l3,
        "still_ttc<10": still_ttc10,
        "veh_reacq": len(veh_gap_ends),
        "reacq_L3(<=1.5s)": reacq_l3,
        "approach_eps(d<8,v>=0.4)": len(eps),
        "approach_detected(L2+)": detected,
        "ttc_finite_rows": len(ttc_fin),
        "ttc_med": round(st.median(ttc_fin), 1) if ttc_fin else None,
    }


for s in args.sessions:
    name = s if "_" in s else f"20260817_{s}"
    rows, gaps = run_session(name)
    print(f"[{args.label}] {name}", json.dumps(metrics(rows, gaps), ensure_ascii=False))
