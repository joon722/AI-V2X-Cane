# -*- coding: utf-8 -*-
"""
실측 주행 검증 파이프라인 — 실차 GPS 로그로 AI 3초 예측의 실환경 정확도 측정

입력: step3 형식 로그 CSV (pc_time,type,node_id,seq,gps_valid,lat,lng,speed_mps,...)
      - 하드웨어 팀원이 주행 테스트 때 기존 로깅 그대로 남긴 파일
처리: GPS 위경도 -> SUMO 좌표 변환 -> 1초 단위 정리 -> 20개 특징 계산
      -> v4 ONNX 예측 -> 실제 3초 뒤와 대조
출력: 정확도·선행시간 리포트 + real_result.csv/real_pedestrian.csv (뷰어·지도용)

사용:
  python real_validate.py 주행로그.csv                # gps_valid=1 행만 사용
  python real_validate.py 주행로그.csv --allow-invalid  # 테스트 로그(더미 GPS) 허용
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import onnxruntime as ort
from pyproj import Proj

sys.stdout.reconfigure(encoding="utf-8")
HERE = Path(__file__).parent
from build_dataset_local import (  # noqa: E402
    calculate_ttc, calculate_risk_score, classify_risk_level, dcpa_gate,
    TTC_CLAMP_S, ZONE_BASE_RISK)

MODEL = HERE / "models_v4" / "risk_transformer_v3.onnx"
SCALER = HERE / "models_v4" / "scaler_v3.json"
_UTM = Proj(proj="utm", zone=52, ellps="WGS84")
NET_OFF = (-315516.76, -4150401.46)
SEQ = 10


def to_sumo(lat, lng):
    e, n = _UTM(np.asarray(lng, float), np.asarray(lat, float))
    return e + NET_OFF[0], n + NET_OFF[1]


def load_log(path, allow_invalid):
    df = pd.read_csv(path)
    need = {"pc_time", "type", "node_id", "gps_valid", "lat", "lng",
            "speed_mps"}
    missing = need - set(df.columns)
    if missing:
        raise SystemExit(f"로그에 필요한 컬럼이 없습니다: {missing}")
    if not allow_invalid:
        df = df[df["gps_valid"] == 1]
        if df.empty:
            raise SystemExit("gps_valid=1 행이 없습니다. 테스트 로그라면 "
                             "--allow-invalid 옵션을 쓰세요.")
    df["sec"] = df["pc_time"].astype(float).round(0).astype(np.int64)
    # 1초 단위 평균 위치 (노드별)
    g = (df.groupby(["type", "node_id", "sec"], as_index=False)
           .agg(lat=("lat", "mean"), lng=("lng", "mean"),
                speed=("speed_mps", "mean")))
    x, y = to_sumo(g["lat"], g["lng"])
    g["x"], g["y"] = x, y
    cane = g[g["type"] == "cane"]
    vehs = g[g["type"] == "vehicle"]
    if cane.empty or vehs.empty:
        raise SystemExit(f"cane {len(cane)}행 / vehicle {len(vehs)}행 — "
                         "둘 다 필요합니다 (두 노드 전원·GPS 확인)")
    # 지팡이는 1대 가정: 초당 1행
    cane = cane.groupby("sec", as_index=False).first()
    return cane, vehs


def build_rows(cane, vehs):
    cane_idx = cane.set_index("sec")
    rows = []
    for vid, g in vehs.groupby("node_id"):
        g = g.sort_values("sec").reset_index(drop=True)
        common = g["sec"].isin(cane_idx.index)
        g = g[common].reset_index(drop=True)
        if len(g) < 2:
            continue
        c = cane_idx.loc[g["sec"]]
        rx = g["x"].to_numpy() - c["x"].to_numpy()
        ry = g["y"].to_numpy() - c["y"].to_numpy()
        dist = np.hypot(rx, ry)
        dt = np.diff(g["sec"].to_numpy(), prepend=g["sec"].iloc[0] - 1)
        rel = np.concatenate([[0.0], -(np.diff(dist) / np.diff(g["sec"]))])
        dvx = np.concatenate([[0.0], np.diff(rx) / np.diff(g["sec"])])
        dvy = np.concatenate([[0.0], np.diff(ry) / np.diff(g["sec"])])
        ttc = np.minimum([calculate_ttc(d, r) for d, r in zip(dist, rel)],
                         TTC_CLAMP_S)
        score = [calculate_risk_score(d, r, v, t, ZONE_BASE_RISK)
                 for d, r, v, t in zip(dist, rel, g["speed"], ttc)]
        v2 = dvx**2 + dvy**2
        with np.errstate(divide="ignore", invalid="ignore"):
            t_cpa = np.where(v2 > 1e-6, -(rx*dvx + ry*dvy)/v2, -1.0)
        dcpa = np.where(t_cpa > 0,
                        np.hypot(rx + dvx*t_cpa, ry + dvy*t_cpa), dist)
        t_hit = np.clip(t_cpa, 0.0, 3.0)
        pdist = np.hypot(rx + dvx*t_hit, ry + dvy*t_hit)
        pscore = [calculate_risk_score(d, r, v, t if r > 0.1 else 9999.0)
                  for d, r, v, t in zip(pdist, rel, g["speed"],
                                        np.where(t_hit > 0.05, t_hit, 0.05))]
        rows.append(pd.DataFrame({
            "vehicle_id": str(vid), "timestep_time": g["sec"],
            "ped_x": c["x"].to_numpy(), "ped_y": c["y"].to_numpy(),
            "veh_x": g["x"], "veh_y": g["y"],
            "ped_speed_mps": c["speed"].to_numpy(),
            "veh_speed_mps": g["speed"],
            "distance_m": dist, "rel_speed_mps": rel,
            "ttc": ttc, "ttc_valid": (rel > 0.1).astype(float),
            "risk_score": score, "zone_base_risk": ZONE_BASE_RISK,
            "dx": rx, "dy": ry, "dvx": dvx, "dvy": dvy, "dcpa_m": dcpa,
            "phys_min_dist_3s": pdist,
            "phys_t_cpa": np.maximum(t_hit, 0.05), "phys_score_3s": pscore,
        }))
    if not rows:
        raise SystemExit("지팡이와 시간이 겹치는 차량 데이터가 없습니다")
    return pd.concat(rows, ignore_index=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--allow-invalid", action="store_true")
    a = ap.parse_args()

    cane, vehs = load_log(a.log, a.allow_invalid)
    print(f"지팡이 {len(cane)}초 / 차량 노드 {vehs['node_id'].nunique()}대")
    df = build_rows(cane, vehs)

    s = json.loads(SCALER.read_text(encoding="utf-8"))
    cols = s["feature_columns"]
    mean = np.array(s["mean"], np.float32)
    scale = np.array(s["scale"], np.float32)
    sess = ort.InferenceSession(str(MODEL))

    out = []
    for vid, g in df.groupby("vehicle_id"):
        g = g.sort_values("timestep_time").reset_index(drop=True)
        f = ((g[cols].to_numpy(np.float32) - mean) / scale)
        win = [f[max(0, i-SEQ+1):i+1] for i in range(len(g))]
        win = [np.vstack([np.repeat(w[:1], SEQ-len(w), 0), w]) for w in win]
        logits = sess.run(None, {"input": np.stack(win).astype(np.float32)})[0]
        g["onnx_risk_level"] = logits.argmax(1).astype(int)
        e = np.exp(logits - logits.max(1, keepdims=True))
        g["onnx_confidence"] = (e / e.sum(1, keepdims=True)).max(1)
        out.append(g)
    res = pd.concat(out, ignore_index=True)
    res["actual_level"] = [classify_risk_level(sc * dcpa_gate(d))
                           for sc, d in zip(res["risk_score"], res["dcpa_m"])]

    # 리포트: 예측 vs 실제 3초 뒤
    tot = hit = fd = fh = 0
    leads = []
    for vid, g in res.groupby("vehicle_id"):
        a_ = g["actual_level"].to_numpy()
        p_ = g["onnx_risk_level"].to_numpy()
        fut = np.array([a_[i+1:i+4].max() if i < len(a_)-1 else -1
                        for i in range(len(a_))])
        v = fut >= 0
        tot += v.sum(); hit += (p_[v] == fut[v]).sum()
        d = v & (fut >= 2)
        fd += d.sum(); fh += (p_[d] >= 2).sum()
        ai = np.where(a_ >= 2)[0]; pi = np.where(p_ >= 2)[0]
        if len(ai) and len(pi) and pi[0] <= ai[0]:
            t = g["timestep_time"].to_numpy()
            leads.append(int(t[ai[0]] - t[pi[0]]))
    print("\n===== 실측 검증 리포트 =====")
    print(f"예측 ↔ 3초 뒤 실제 일치율: {hit/tot*100:.1f}% ({hit}/{tot})")
    if fd:
        print(f"미래 위험(L2+) 사전 검출률: {fh/fd*100:.1f}% ({fh}/{fd})")
    if leads:
        print(f"선행 경고 시간: 평균 {np.mean(leads):.1f}초 "
              f"(위험 차량 {len(leads)}건)")

    res.to_csv(HERE / "real_result.csv", index=False)
    # 뷰어용 보행자 파일 (지팡이 궤적)
    ped_out = cane.rename(columns={"sec": "timestep_time", "x": "person_x",
                                   "y": "person_y", "speed": "person_speed"})
    ped_out["person_id"] = "cane"
    ped_out[["timestep_time", "person_id", "person_x", "person_y",
             "person_speed"]].to_csv(HERE / "real_pedestrian.csv",
                                     sep=";", index=False)
    print("저장: real_result.csv, real_pedestrian.csv (뷰어/지도용)")


if __name__ == "__main__":
    main()
