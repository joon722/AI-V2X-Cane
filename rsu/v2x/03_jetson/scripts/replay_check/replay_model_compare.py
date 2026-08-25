# -*- coding: utf-8 -*-
"""실기 raw 로그 3-way 재생 비교: 규칙만 vs 구모델(1200) vs 신모델(대량 학습).

replay_fusion.py(8/15 재생검증 본체)의 재생 기법을 그대로 쓰되 모델 파일을
파라미터로 받는다. 판정 경로는 배포 기본값(융합 ON, heartbeat/hold/floor 기본).
출력: 세션별 전송행 수 / L1+·L2+·L3 경보행 / 모델이 개입한 행(level_source=model).

사용:  PYTHONUTF8=1 python replay_model_compare.py [--models name=path ...]
"""
import argparse
import contextlib
import io
import json
import pickle
import re
import sys
from pathlib import Path

JETSON = r"C:\Users\user\OneDrive\바탕 화면\v2x(lux)\03_jetson"
sys.path.insert(0, JETSON)
sys.path.insert(0, str(Path(JETSON) / "deploy"))

import step8_send_risk as s8  # noqa: E402
from step3_parse_v2x import normalize_record  # noqa: E402
from step4_state_store import FRESH_WINDOW_S, StateStore  # noqa: E402
from step6_kinematics import KinematicsPipeline  # noqa: E402
from step7_risk import DCPA_FAR_M, DCPA_FLOOR, DCPA_NEAR_M, T_FLOOR_TTC_S  # noqa: E402
from step8_stability import HOLD_S, LevelStabilizer  # noqa: E402
from model_gate import ModelGate  # noqa: E402

LOGDIR = Path(JETSON) / "logs"
HERE = Path(__file__).resolve().parent
LINE_RE = re.compile(r"^(\d+\.\d+) RX (\{.*\})\s*$")

# replay_fusion.SESSIONS 와 동일 — 8/15 재생검증과 같은 무대라 결과 비교 가능.
SESSIONS = [
    "raw_20260812_193237.log", "raw_20260812_195704.log", "raw_20260812_201035.log",
    "raw_20260812_201948.log", "raw_20260812_203250.log", "raw_20260812_203823.log",
    "raw_20260812_213246.log", "raw_20260812_220647.log",
    "raw_20260813_081033.log", "raw_20260813_083827.log",
    "raw_20260814_142647.log", "raw_20260814_155051.log",
]


class Replay:
    def __init__(self, model_path):
        store = StateStore(fresh_window_s=FRESH_WINDOW_S)
        pipeline = KinematicsPipeline(store)
        transmitter = s8.RiskTransmitter(target_id=0, heartbeat_s=s8.HEARTBEAT_S,
                                         allow_untrusted=False)
        gate_params = {"near_m": DCPA_NEAR_M, "far_m": DCPA_FAR_M,
                       "floor": DCPA_FLOOR, "floor_ttc_s": T_FLOOR_TTC_S}
        stabilizer = LevelStabilizer(hold_s=HOLD_S)
        gate = ModelGate.load(model_path, quiet=True) if model_path else ModelGate()
        self.rows = []
        self.sender = s8.RiskSender(pipeline, transmitter, transport=lambda cmd: None,
                                    csv_path=None, gate_params=gate_params,
                                    stabilizer=stabilizer, raw_log=None, model_gate=gate)
        self.now = None

    def feed(self, ts, text):
        self.now = ts
        self.sender.process_line(text, "real")


def run_session(name, model_path):
    rp = Replay(model_path)
    s8.normalize_record = lambda payload, mode: normalize_record(payload, mode, now=rp.now)
    s8.append_row = lambda path, row: rp.rows.append(row)
    n_lines = 0
    with open(LOGDIR / name, encoding="utf-8", errors="replace") as fh, \
            contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        for line in fh:
            m = LINE_RE.match(line)
            if not m:
                continue
            ts, text = float(m.group(1)), m.group(2)
            if '"type"' not in text:
                continue
            n_lines += 1
            try:
                rp.feed(ts, text)
            except Exception as exc:  # noqa: BLE001
                print(f"[ERR] {name} t={ts} {exc!r}", file=sys.__stderr__)
    return rp.rows, n_lines


def summarize(rows):
    def lvl(r):
        try:
            return int(r.get("effective_level") or 0)
        except (TypeError, ValueError):
            return 0
    n = len(rows)
    return {
        "tx": n,
        "l1p": sum(1 for r in rows if lvl(r) >= 1),
        "l2p": sum(1 for r in rows if lvl(r) >= 2),
        "l3": sum(1 for r in rows if lvl(r) == 3),
        "model_rows": sum(1 for r in rows if r.get("level_source") == "model"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--models", nargs="+",
                    default=[f"rule=", f"old={Path(JETSON)/'risk_model.json'}"],
                    help="name=path (path 비우면 규칙만)")
    ap.add_argument("--out", default=str(HERE / "replay_compare.pkl"))
    ap.add_argument("--sessions", nargs="+", default=None,
                    help="재생할 raw 로그 이름들 (기본: 8/12~8/14 12세션)")
    args = ap.parse_args()
    global SESSIONS
    if args.sessions:
        SESSIONS = args.sessions

    configs = []
    for spec in args.models:
        name, _, path = spec.partition("=")
        configs.append((name, path or None))

    missing = [n for n in SESSIONS if not (LOGDIR / n).exists()]
    for n in missing:
        print(f"[SKIP] 로그 없음: {n}")
    sessions = [n for n in SESSIONS if n not in missing]

    results = {}
    totals = {name: {"tx": 0, "l1p": 0, "l2p": 0, "l3": 0, "model_rows": 0}
              for name, _ in configs}
    header = f"{'session':32s}" + "".join(
        f" | {name}: L1+/L2+/L3 (model행)" for name, _ in configs)
    print(header, flush=True)
    for sess in sessions:
        line = f"{sess:32s}"
        for name, path in configs:
            rows, _ = run_session(sess, path)
            s = summarize(rows)
            results[(sess, name)] = s
            for k in totals[name]:
                totals[name][k] += s[k]
            line += f" | {s['l1p']:4d}/{s['l2p']:4d}/{s['l3']:3d} ({s['model_rows']:4d})"
        print(line, flush=True)

    print("\n=== 합계 (세션 " + str(len(sessions)) + "개) ===")
    for name, _ in configs:
        t = totals[name]
        print(f"{name:8s}: 전송 {t['tx']:6d}행, L1+ {t['l1p']:5d}, L2+ {t['l2p']:5d}, "
              f"L3 {t['l3']:4d}, 모델개입 {t['model_rows']:5d}행")

    with open(args.out, "wb") as fh:
        pickle.dump(results, fh)
    print("saved", args.out)


if __name__ == "__main__":
    main()
