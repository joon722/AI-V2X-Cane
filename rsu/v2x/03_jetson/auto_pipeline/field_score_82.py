#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""8/17 실측 raw 로그에서 82 모델 vs 규칙 — 현장 채점 (현준 지시 ①).

방법: 실측 raw 로그(pc_time RX {json})를 build_dataset_from_streams.stream_to_features 로 흘린다.
     이 함수는 젯슨 실행 코드(step3 파서 → step4 store → step6 KF → StreamingFeatures) 그대로라
     학습 특징 = 실행 특징. 그 15열 위에서 규칙(팀 점수표)과 트리 모델을 나란히 돌린다.

라벨: 저장소에 대본 라벨 파일이 없다(2026-08-18 기준). 그래서 두 층으로 채점한다.
  (A) 라벨 불필요 지표 — 정지 오경보, 재획득 직후 유령 경보, replay_0817 방식 접근 에피소드 검출
  (B) 대본 라벨이 오면: --labels CSV (session,t_start,t_end,event) 로 event=approach 창을
      위험 시나리오로 삼아 timely_alarm_rate 를 계산한다 (창 시작 = 접촉 시각 대용은 아니므로
      "창 안 첫 경보가 창 시작 후 몇 초"를 리드로 보고, 창 끝(최근접) - 첫 경보 >= T_floor 를 적시로 센다).

사용
    python field_score_82.py --logs-dir "<...>/data/젯슨로그_20260817" --model risk_model_streams_82.json
    python field_score_82.py ... --labels 대본_20260818.csv
"""
import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
JETSON = HERE.parent
for p in (HERE, JETSON, JETSON / "ttc_study"):
    sys.path.insert(0, str(p))
sys.stdout.reconfigure(encoding="utf-8")

from baselines import ALARM_LEVEL, team_table_levels  # noqa: E402
from build_dataset_from_streams import stream_to_features  # noqa: E402
from model_runtime import TreeEnsemble  # noqa: E402
from safety_floor import T_FLOOR_S  # noqa: E402

LINE_RE = re.compile(r"^(\d+\.\d+) RX (\{.*\})\s*$")
DEFAULT_SESSIONS = ["111216", "122247", "141209", "145152", "172101", "174332", "180708", "181320", "182456"]


def read_field_log(path):
    """실측 raw 로그 → 패킷 목록 (잘린 줄·JSON 아닌 줄은 건너뜀). 재획득 시각도 같이 낸다."""
    packets, reacq, last_veh = [], [], None
    for line in Path(path).open(encoding="utf-8", errors="replace"):
        m = LINE_RE.match(line)
        if not m:
            continue
        try:
            pk = json.loads(m.group(2))
        except ValueError:
            continue
        if pk.get("type") not in ("cane", "vehicle"):
            continue
        ts = float(m.group(1))
        pk["pc_time"] = ts
        if pk["type"] == "vehicle" and pk.get("gps_valid"):
            if last_veh is not None and ts - last_veh >= 2.0:
                reacq.append(ts)
            last_veh = ts
        packets.append(pk)
    return packets, reacq


def approach_episodes(df):
    """replay_0817 과 같은 정의: 차량 속도>=0.4 이고 거리<8 m 인 구간, 3 s 끊기면 새 에피소드."""
    eps, cur = [], None
    t = df.t.to_numpy(); d = df.distance_m.to_numpy(); v = df.veh_speed_mps.to_numpy()
    for i in range(len(df)):
        if d[i] < 8.0 and v[i] >= 0.4:
            if cur is None or t[i] - cur["end"] > 3.0:
                cur = {"start": t[i], "end": t[i], "min_d": d[i], "t_min": t[i]}
                eps.append(cur)
            cur["end"] = t[i]
            if d[i] < cur["min_d"]:
                cur["min_d"], cur["t_min"] = d[i], t[i]
    return eps


def score_alarm(df, alarm, reacq, eps, label):
    t = df.t.to_numpy()
    n = len(df)
    still = (df.veh_speed_mps.to_numpy() < 0.3) & (df.ped_speed_mps.to_numpy() < 0.3)
    still_fa = float(alarm[still].mean() * 100) if still.any() else float("nan")
    ghost = sum(1 for g in reacq if np.any(alarm & (t >= g) & (t <= g + 1.5)))
    det = timely = 0
    for e in eps:
        m = alarm & (t >= e["start"] - 1.0) & (t <= e["end"] + 1.0)
        if m.any():
            det += 1
            if e["t_min"] - t[m][0] >= T_FLOOR_S:   # 최근접 시각보다 T_floor 앞서 첫 경보
                timely += 1
    return {"label": label, "alarm%": round(float(alarm.mean() * 100), 2),
            "still_alarm%": round(still_fa, 2), "reacq_ghost": f"{ghost}/{len(reacq)}",
            "approach_detected": f"{det}/{len(eps)}", "approach_timely(T_floor)": f"{timely}/{len(eps)}"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs-dir", required=True)
    ap.add_argument("--model", default="risk_model_streams_82.json")
    ap.add_argument("--models", nargs="*", default=None, help="여러 모델 비교 (label=path ...)")
    ap.add_argument("--sessions", nargs="*", default=DEFAULT_SESSIONS)
    ap.add_argument("--labels", default=None, help="대본 라벨 CSV (session,t_start,t_end,event) — 있으면 (B)")
    ap.add_argument("--out", default="results/field_score_0817.csv")
    args = ap.parse_args()

    models = {}
    for spec in (args.models or [f"82={args.model}"]):
        label, path = spec.split("=", 1)
        rt = TreeEnsemble.load(path)
        thr = json.loads(Path(path).read_text(encoding="utf-8")).get("threshold", 0.5)
        models[label] = (rt, thr)
        print(f"[model {label}] {path} threshold {thr:.4f}")

    labels = None
    if args.labels:
        labels = pd.read_csv(args.labels)
        print(f"[labels] {len(labels)} windows from {args.labels}")

    all_rows, summary = [], []
    for s in args.sessions:
        path = Path(args.logs_dir) / f"raw_20260817_{s}.log"
        if not path.exists():
            print(f"[SKIP] {path.name} 없음"); continue
        packets, reacq = read_field_log(path)
        rows = stream_to_features(packets)
        if not rows:
            print(f"[{s}] 특징 0행 (쌍 미성립)"); continue
        df = pd.DataFrame(rows)
        df["session"] = s
        levels = team_table_levels({c: df[c].to_numpy() for c in
                                    ("distance_m", "closing_los", "ttc", "veh_speed_mps", "dcpa_m")})
        rule_alarm = levels >= ALARM_LEVEL
        if labels is not None:
            win = labels[(labels.session.astype(str) == s) & (labels.event == "approach")]
            eps = [{"start": r.t_start, "end": r.t_end, "min_d": np.nan, "t_min": r.t_end} for r in win.itertuples()]
            eps_src = "대본"
        else:
            eps = approach_episodes(df); eps_src = "휴리스틱(d<8,v>=0.4)"
        print(f"\n[{s}] {len(df):,}행, {df.t.max()-df.t.min():.0f}s, 재획득 {len(reacq)}, "
              f"접근 에피소드 {len(eps)} ({eps_src}), 정지 {int(((df.veh_speed_mps<0.3)&(df.ped_speed_mps<0.3)).sum())}행")
        res = [score_alarm(df, rule_alarm, reacq, eps, "규칙")]
        for label, (rt, thr) in models.items():
            feats = df[list(rt.features)].to_numpy(dtype=float)
            proba = np.array([rt.predict_proba(dict(zip(rt.features, r))) for r in feats])
            res.append(score_alarm(df, proba >= thr, reacq, eps, f"모델 {label}"))
            df[f"proba_{label}"] = proba
        for r in res:
            print("   " + "  ".join(f"{k} {v}" for k, v in r.items()))
            summary.append({"session": s, **r})
        df["rule_alarm"] = rule_alarm
        all_rows.append(df)

    Path(args.out).parent.mkdir(exist_ok=True)
    pd.DataFrame(summary).to_csv(args.out, index=False, encoding="utf-8-sig")
    if all_rows:
        big = pd.concat(all_rows, ignore_index=True)
        # 세션 합산
        print("\n===== 전체 합산 =====")
        still = (big.veh_speed_mps < 0.3) & (big.ped_speed_mps < 0.3)
        print(f"행 {len(big):,}, 정지 {int(still.sum()):,}행")
        print(f"   규칙     alarm {big.rule_alarm.mean()*100:.2f}%  정지 오경보 {big.rule_alarm[still].mean()*100:.2f}%")
        for label, (rt, thr) in models.items():
            a = big[f"proba_{label}"] >= thr
            print(f"   모델 {label:6s} alarm {a.mean()*100:.2f}%  정지 오경보 {a[still].mean()*100:.2f}%")
        big.to_csv(Path(args.out).with_name("field_features_0817.csv"), index=False)
    print(f"\n저장: {args.out}")


if __name__ == "__main__":
    main()
