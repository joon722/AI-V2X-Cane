#!/usr/bin/env python3
"""잡음 낀 패킷 스트림 → 젯슨 실행 특징(15개) + 참값 라벨 → 경보 모델 학습셋.

무엇을 고치나
    실시간 경보 게이트(model_gate → risk_model.json)는 학습 때 ttc_study/features.py
    (배치)로 특징을 만들었고, 실행 때는 step6 KF 상태에서 model_features.StreamingFeatures
    로 만든다. 값은 맞춰 놨지만 입력이 다르다: 학습은 시뮬 좌표에 σ만 얹은 것, 실행은
    양자화·무효·재획득 점프·heading 규칙이 섞인 진짜 패킷. 여기서는 잡음층(gps_noise)
    이 만든 패킷을 **젯슨 실행 코드 그대로**(step3 파서 → step4 store → step6 KF →
    StreamingFeatures) 흘려 특징을 뽑는다. 학습 특징 = 실행 특징. 라벨은 잡음 없는
    참값 궤적에서.

입력 두 갈래
    (a) --streams DIR : sim_to_rsu_stream.py 가 만든 raw_sim_*.log + truth_sim_*.csv 쌍
                       (민서 SUMO 생성기 시나리오, 실차 스케일)
    (b) --from-scenario-sim N : ttc_study/scenario_sim 시나리오 N개를 여기서 만들어
                       잡음층에 통과 (RC·보행자 스케일, 기존 risk_model.json 학습 원천)
    (a)(b) 섞어도 된다.

출력
    CSV: features(15) + y, y_train, t, scenario_id, d_min_future, source
      y        = 앞으로 horizon(3 s) 안에 d_crit(2 m) 이내로 들어오면 1  (평가 기준)
      y_train  = 창을 lead(2 s = T_floor) 만큼 민 것 = "지금 울려야 피할 수 있는 위험" (학습 기준)
    --train 을 주면 model_runtime.py 와 같은 절차로 risk_model.json 까지 만든다
    (시나리오 단위 분할, y_train 으로 학습, 규칙과 같은 오경보율에서 임계값).

사용
    python build_dataset_from_streams.py --from-scenario-sim 1200 --randomize --out ds.csv
    python build_dataset_from_streams.py --streams out/ --from-scenario-sim 600 --out ds.csv --train --model risk_model_streams.json
"""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
JETSON = HERE.parent
for p in (HERE, JETSON, JETSON / "ttc_study"):
    sys.path.insert(0, str(p))

import gps_noise  # noqa: E402
from features import FEATURE_COLUMNS  # noqa: E402  (ttc_study - 학습 쪽 특징 이름 = 실행 쪽과 동일)
from model_features import StreamingFeatures  # noqa: E402
from step3_parse_v2x import normalize_record  # noqa: E402
from step4_state_store import FRESH_WINDOW_S, StateStore  # noqa: E402
from step6_kinematics import KinematicsPipeline  # noqa: E402

D_CRIT_M = 2.0
HORIZON_S = 3.0
LEAD_S = 2.0          # model_runtime.main 과 동일 (T_floor)
ORIGIN = (37.4977, 126.9528)
BASE_EPOCH = 1_760_000_000.0


# ---------------------------------------------------------------- 스트림 → 특징
def stream_to_features(packets):
    """패킷(dict, pc_time 포함) 목록 → 판정마다 실행 특징 한 벌.

    젯슨 step8 이 하는 것과 같은 순서: normalize_record(now=pc_time) → store.update →
    pipeline.observe → pipeline.compute → StreamingFeatures.update(now, *last_states).
    model_gate 는 compute 가 결과를 줄 때마다 특징을 만들고 reset 하지 않는다 - 그대로.
    """
    store = StateStore(fresh_window_s=FRESH_WINDOW_S)
    pipeline = KinematicsPipeline(store)
    streamer = StreamingFeatures()
    rows = []
    for pk in packets:
        payload = {k: v for k, v in pk.items() if k != "pc_time"}
        row = normalize_record(payload, "real", now=float(pk["pc_time"]))
        if not store.update(row):
            continue
        pipeline.observe(row)
        result = pipeline.compute()
        if result is None or pipeline.last_states is None:
            continue
        now = result[0]
        feats = streamer.update(now, pipeline.last_states[0], pipeline.last_states[1])
        feats = {k: float(feats[k]) for k in FEATURE_COLUMNS}
        feats["t"] = float(now)
        rows.append(feats)
    return rows


def read_stream(path):
    """raw 로그(pc_time RX {json}) → 패킷 dict 목록."""
    packets = []
    for line in Path(path).open(encoding="utf-8", errors="replace"):
        parts = line.rstrip("\n").split(" ", 2)
        if len(parts) < 3 or parts[1] != "RX":
            continue
        try:
            pk = json.loads(parts[2])
            pk["pc_time"] = float(parts[0])
        except ValueError:
            continue
        packets.append(pk)
    return packets


# ---------------------------------------------------------------- 라벨
def _future_min(ts, dist, i, lo_s, hi_s):
    """ts[i] 기준 [lo_s, hi_s] 창 안의 최소 거리(창 밖이면 inf)."""
    m = math.inf
    for q in range(i, len(ts)):
        dt = ts[q] - ts[i]
        if dt > hi_s:
            break
        if dt >= lo_s:
            m = min(m, dist[q])
    return m


def attach_labels(rows, truth, base_epoch=BASE_EPOCH, d_crit_m=D_CRIT_M, horizon_s=HORIZON_S,
                  lead_s=LEAD_S, tol_s=0.11):
    """특징 행에 참값 라벨을 시각으로 붙인다.

    truth: sim_to_rsu_stream.truth_rows 형식(t 는 base_epoch 기준 상대초, distance_m 참값).
    y      : [t, t+horizon] 안 최소 거리 <= d_crit
    y_train: [t+lead, t+lead+horizon] 안 최소 거리 <= d_crit
    d_min_future: y 창의 최소 거리
    """
    ts = [float(r["t"]) for r in truth]
    dist = [float(r["distance_m"]) for r in truth]
    # 시나리오의 첫 접촉 시각(참값이 d_crit 이내로 처음 들어온 순간). 없으면 NaN.
    # safety_floor.timely_alarm_rate 가 "첫 경보가 접촉보다 T_floor 이상 앞섰나"를 볼 때 쓴다.
    t_hit = next((base_epoch + t for t, d in zip(ts, dist) if d <= d_crit_m), math.nan)
    out = []
    j = 0
    for r in rows:
        t_rel = r["t"] - base_epoch
        while j + 1 < len(ts) and abs(ts[j + 1] - t_rel) <= abs(ts[j] - t_rel):
            j += 1
        if abs(ts[j] - t_rel) > tol_s:
            continue  # 참값 없는 시각(파일 끝 등)은 버린다
        dmin = _future_min(ts, dist, j, 0.0, horizon_s)
        dmin_lead = _future_min(ts, dist, j, lead_s, lead_s + horizon_s)
        rec = dict(r)
        rec["y"] = int(dmin <= d_crit_m)
        rec["y_train"] = int(dmin_lead <= d_crit_m)
        rec["d_min_future"] = dmin if math.isfinite(dmin) else dist[j]
        rec["t_hit"] = t_hit
        out.append(rec)
    return out


# ---------------------------------------------------------------- 원천 (a): 스트림 폴더
def build_from_streams_dir(streams_dir, base_epoch=BASE_EPOCH):
    import pandas as pd
    frames = []
    for raw in sorted(Path(streams_dir).glob("raw_sim_*.log")):
        tag = raw.name[len("raw_sim_"):-len(".log")]
        truth_path = raw.with_name(f"truth_sim_{tag}.csv")
        if not truth_path.exists():
            print(f"[SKIP] 참값 없음: {truth_path.name}", file=sys.stderr)
            continue
        truth = list(csv.DictReader(truth_path.open(encoding="utf-8")))
        rows = attach_labels(stream_to_features(read_stream(raw)), truth, base_epoch=base_epoch)
        if not rows:
            continue
        df = pd.DataFrame(rows)
        df["scenario_id"] = tag
        df["source"] = "sumo"
        frames.append(df)
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


# ---------------------------------------------------------------- 원천 (b): scenario_sim
def scenario_to_samples(scenario, hz=5.0):
    """ttc_study Scenario(0.1 s 격자) → 두 노드의 (t, east, north, speed, heading) 표본 + 참값 거리."""
    step = max(1, int(round((1.0 / hz) / (scenario.t[1] - scenario.t[0]))))
    idx = range(0, len(scenario.t), step)

    def track(tr):
        out = []
        last_hd = 0.0
        for i in idx:
            vx, vy = float(tr.vx[i]), float(tr.vy[i])
            spd = math.hypot(vx, vy)
            if spd > 0.05:
                last_hd = math.degrees(math.atan2(vx, vy)) % 360.0
            out.append((float(scenario.t[i]), float(tr.x[i]), float(tr.y[i]), spd, last_hd))
        return out

    ped, veh = track(scenario.ped), track(scenario.veh)
    truth = [{"t": p[0], "distance_m": math.hypot(v[1] - p[1], v[2] - p[2])} for p, v in zip(ped, veh)]
    return ped, veh, truth


def build_from_scenario_sim(n_scenarios, seed=0, params=None, randomize=False, hz=5.0,
                            base_epoch=BASE_EPOCH):
    import pandas as pd
    from scenario_sim import sample_params, simulate

    frames = []
    for i in range(n_scenarios):
        sid = seed + i
        scenario = simulate(sample_params(sid), duration_s=None)
        ped, veh, truth = scenario_to_samples(scenario, hz=hz)
        rng = np.random.default_rng(np.random.SeedSequence([sid, 7]))
        if params is not None:
            p_ped = p_veh = params
        elif randomize:
            p_ped, p_veh = (gps_noise.FieldNoiseParams.randomized(rng),
                            gps_noise.FieldNoiseParams.randomized(rng))
        else:
            p_ped, p_veh = gps_noise.FieldNoiseParams(), gps_noise.FieldNoiseParams()
        cane = gps_noise.noisify_track(ped, p_ped, rng, ORIGIN, "cane")
        car = gps_noise.noisify_track(veh, p_veh, rng, ORIGIN, "vehicle")
        packets = cane + car
        for pk in packets:
            pk["pc_time"] = round(base_epoch + pk["pc_time"], 3)
        packets.sort(key=lambda p: (p["pc_time"], 0 if p["type"] == "cane" else 1, p["seq"]))
        rows = attach_labels(stream_to_features(packets), truth, base_epoch=base_epoch)
        if not rows:
            continue
        df = pd.DataFrame(rows)
        df["scenario_id"] = sid
        df["source"] = "scenario_sim"
        frames.append(df)
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


# ---------------------------------------------------------------- 학습 (model_runtime.main 과 동일 절차)
def train_and_export(df, out_path, seed=0):
    from baselines import ALARM_LEVEL, team_table_levels
    from dataset import split_by_scenario
    from model import predict_proba, threshold_at_false_alarm_rate, train
    from model_runtime import TreeEnsemble, save_model
    from safety_floor import T_FLOOR_S, timely_alarm_rate

    train_df, test_df = split_by_scenario(df, test_ratio=0.3, seed=seed)
    clf = train(train_df, seed=seed, label_col="y_train")
    levels = team_table_levels({c: test_df[c].to_numpy() for c in (
        "distance_m", "closing_los", "ttc", "veh_speed_mps", "dcpa_m")})
    rule_alarm = levels >= ALARM_LEVEL
    target_far = float(rule_alarm[test_df.y.to_numpy() == 0].mean())
    proba = predict_proba(clf, test_df)
    threshold = threshold_at_false_alarm_rate(proba, test_df.y.to_numpy(), target_far)
    y = test_df.y.to_numpy()
    model_alarm = proba >= threshold
    recall_model = float(model_alarm[y == 1].mean()) if (y == 1).any() else float("nan")
    recall_rule = float(rule_alarm[y == 1].mean()) if (y == 1).any() else float("nan")
    # 시나리오 단위 '반응할 수 있을 만큼 미리 울렸나' — model_runtime.main 과 같은 본 지표
    timely_rule = timely_alarm_rate(test_df, rule_alarm)
    timely_model = timely_alarm_rate(test_df, model_alarm)
    meta = {
        "trained_rows": int(len(train_df)),
        "trained_scenarios": int(train_df.scenario_id.nunique()),
        "sources": sorted(df.source.unique().tolist()),
        "seed": seed,
        "target_false_alarm_rate": target_far,
        "recall_rule": recall_rule,
        "recall_model": recall_model,
        "timely_rule": timely_rule,
        "timely_model": timely_model,
        "t_floor_s": T_FLOOR_S,
        "trained_on": "field-noise streams (gps_noise.FieldNoiseParams) through runtime pipeline",
    }
    path = save_model(clf, out_path, threshold=threshold, meta=meta)
    runtime = TreeEnsemble.load(path)
    rows = test_df[list(runtime.features)].to_numpy(dtype=float)
    check = np.array([runtime.predict_proba(dict(zip(runtime.features, r))) for r in rows[:200]])
    gap = float(np.max(np.abs(check - proba[:200]))) if len(rows) else 0.0
    print(f"  임계값 {threshold:.4f} (규칙 오경보 {target_far*100:.2f}% 기준)")
    print(f"  적시경보(시나리오, T_floor 앞)  규칙 {timely_rule*100:.1f}% -> 모델 {timely_model*100:.1f}%")
    print(f"  위험시점 재현율(행)            규칙 {recall_rule*100:.1f}% -> 모델 {recall_model*100:.1f}%")
    print(f"  저장 {path}  sklearn 대비 최대 오차 {gap:.2e}")
    return path


# ---------------------------------------------------------------- CLI
def parse_args():
    ap = argparse.ArgumentParser(description="잡음 스트림 → 실행 특징 + 참값 라벨 학습셋")
    ap.add_argument("--streams", default=None, help="raw_sim_*.log + truth_sim_*.csv 폴더")
    ap.add_argument("--from-scenario-sim", type=int, default=0, help="scenario_sim 시나리오 수")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--randomize", action="store_true", help="환경 의존 잡음을 시나리오마다 무작위")
    ap.add_argument("--out", default="training_dataset_streams.csv")
    ap.add_argument("--train", action="store_true", help="학습까지 하고 risk_model JSON 저장")
    ap.add_argument("--model", default="risk_model_streams.json")
    return ap.parse_args()


def main():
    import pandas as pd
    args = parse_args()
    parts = []
    if args.streams:
        d = build_from_streams_dir(args.streams)
        print(f"[streams] {len(d)} rows, {d.scenario_id.nunique() if len(d) else 0} scenarios")
        parts.append(d)
    if args.from_scenario_sim:
        d = build_from_scenario_sim(args.from_scenario_sim, seed=args.seed, randomize=args.randomize)
        print(f"[scenario_sim] {len(d)} rows, {d.scenario_id.nunique() if len(d) else 0} scenarios")
        parts.append(d)
    if not parts:
        raise SystemExit("--streams 또는 --from-scenario-sim 중 하나는 필요")
    df = pd.concat([p for p in parts if len(p)], ignore_index=True)
    df.to_csv(args.out, index=False)
    print(f"[OK] {args.out}: {len(df)} rows, y=1 {df.y.mean()*100:.1f}%, y_train=1 {df.y_train.mean()*100:.1f}%")
    if args.train:
        train_and_export(df, args.model, seed=args.seed)


if __name__ == "__main__":
    main()
