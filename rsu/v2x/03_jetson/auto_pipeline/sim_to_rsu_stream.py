#!/usr/bin/env python3
"""생성기 v3 시나리오(SUMO 1 Hz 참값) → 지팡이/차량 RSU 패킷 스트림 + 참값 CSV.

왜 이 층이 필요한가
    지금까지 모델은 시뮬레이터의 완벽한 좌표에서 바로 계산한 특징으로 학습했고,
    젯슨은 잡음 낀 패킷을 KF(step6)로 거른 상태에서 특징(model_features)을 만든다.
    학습 특징 ≠ 실행 특징. 8/16 실기 AUC 0.48 의 구조적 원인이다.

    이 스크립트는 참값 궤적을 gps_noise.noisify_track 으로 "RSU 가 받는 패킷"
    (raw 로그의 RX 줄과 같은 형식)으로 바꾼다. 그 파일을 젯슨 파이프라인에 그대로
    흘리면(scripts/replay_check/replay_0817.py 방식) 학습 특징 = 실행 특징이 된다.
    라벨은 잡음이 없는 참값 궤적에서 뽑는다(truth CSV).

입력  scenario_dir/feature.csv (;구분, timestep_time, vehicle_id, vehicle_x/y/speed/angle)
      scenario_dir/pedestrian.csv (;구분, timestep_time, person_id, person_x/y/speed/angle)
      -> 차량 하나 + 보행자 하나 쌍. (같은 시나리오의 다른 쌍은 --vehicle-id/--person-id)
출력  <out>/raw_sim_<scenario>_<veh>_<ped>.log   RSU raw 로그 형식 (pc_time RX {json})
      <out>/truth_sim_<scenario>_<veh>_<ped>.csv 5 Hz 참값: 거리·접근속도·TTC·3초 내 최근접·라벨

사용
    python sim_to_rsu_stream.py <scenario_dir> [--out DIR] [--seed N] [--randomize]
        --randomize : 환경 의존 잡음을 FieldNoiseParams.randomized() 로 시나리오마다 흔든다
        --stall     : 차량 60초마다 10.8초 정지 재현(기본 꺼짐 - P0 로 고칠 결함)
"""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import gps_noise  # noqa: E402
from step7_risk import calculate_risk_score, calculate_ttc, classify_risk_level  # noqa: E402

ORIGIN = (37.4977, 126.9528)   # 캠퍼스 중심 (생성기 CAMPUS_CENTER_LONLAT 과 동일)
DEFAULT_HZ = 5.0
BASE_EPOCH = 1_760_000_000.0   # 스트림 pc_time 기준(임의의 최근 epoch)


# ---------------------------------------------------------------- 입력
def load_v3_pair(scenario_dir, vehicle_id=None, person_id=None):
    """feature.csv + pedestrian.csv 에서 (차량, 보행자) 한 쌍의 1 Hz 프레임."""
    scenario_dir = Path(scenario_dir)
    veh = list(csv.DictReader((scenario_dir / "feature.csv").open(encoding="utf-8-sig"), delimiter=";"))
    ped = list(csv.DictReader((scenario_dir / "pedestrian.csv").open(encoding="utf-8-sig"), delimiter=";"))
    if vehicle_id is None:
        vehicle_id = veh[0]["vehicle_id"]
    if person_id is None:
        person_id = ped[0]["person_id"]
    veh_by_t = {float(r["timestep_time"]): r for r in veh if r["vehicle_id"] == vehicle_id}
    ped_by_t = {float(r["timestep_time"]): r for r in ped if r["person_id"] == person_id}
    frames = []
    for t in sorted(set(veh_by_t) & set(ped_by_t)):
        v, p = veh_by_t[t], ped_by_t[t]
        frames.append({
            "t": t,
            "ped_x": float(p["person_x"]), "ped_y": float(p["person_y"]),
            "ped_speed": float(p.get("person_speed") or 0.0), "ped_angle": float(p.get("person_angle") or 0.0),
            "veh_x": float(v["vehicle_x"]), "veh_y": float(v["vehicle_y"]),
            "veh_speed": float(v.get("vehicle_speed") or 0.0), "veh_angle": float(v.get("vehicle_angle") or 0.0),
        })
    return frames


# ---------------------------------------------------------------- 보간
def _heading_from(dx, dy, fallback):
    if math.hypot(dx, dy) < 0.05:
        return fallback
    return math.degrees(math.atan2(dx, dy)) % 360.0


def interpolate(frames, hz=DEFAULT_HZ):
    """1 Hz 프레임 → 두 노드의 (t, east, north, speed, heading) 표본 목록.

    east/north 는 보행자 첫 위치 기준(지팡이가 원점 근처에서 시작). 방향은 그 1초
    동안의 변위 방향(SUMO angle 과 같은 나침반 각), 정지면 SUMO angle 을 쓴다.
    """
    if len(frames) < 2:
        raise ValueError("프레임이 2개 이상 필요")
    x0, y0 = frames[0]["ped_x"], frames[0]["ped_y"]
    steps = max(1, int(round(hz)))
    ped, veh = [], []
    for a, b in zip(frames, frames[1:]):
        dt = b["t"] - a["t"]
        ped_hd = _heading_from(b["ped_x"] - a["ped_x"], b["ped_y"] - a["ped_y"], a["ped_angle"])
        veh_hd = _heading_from(b["veh_x"] - a["veh_x"], b["veh_y"] - a["veh_y"], a["veh_angle"])
        for k in range(steps):
            f = k / steps
            t = a["t"] + f * dt
            ped.append((t, a["ped_x"] + f * (b["ped_x"] - a["ped_x"]) - x0,
                        a["ped_y"] + f * (b["ped_y"] - a["ped_y"]) - y0,
                        a["ped_speed"] + f * (b["ped_speed"] - a["ped_speed"]), ped_hd))
            veh.append((t, a["veh_x"] + f * (b["veh_x"] - a["veh_x"]) - x0,
                        a["veh_y"] + f * (b["veh_y"] - a["veh_y"]) - y0,
                        a["veh_speed"] + f * (b["veh_speed"] - a["veh_speed"]), veh_hd))
    last = frames[-1]
    ped.append((last["t"], last["ped_x"] - x0, last["ped_y"] - y0, last["ped_speed"], ped[-1][4]))
    veh.append((last["t"], last["veh_x"] - x0, last["veh_y"] - y0, last["veh_speed"], veh[-1][4]))
    return ped, veh


# ---------------------------------------------------------------- 참값
def truth_rows(ped, veh, horizon_s=3.0, hit_m=2.0):
    """표본마다 잡음 없는 참값: 거리, 접근속도(LOS), TTC, 앞으로 horizon 안 최근접, 라벨.

    label_hit_3s = horizon 안에 hit_m 이내로 들어오면 1 (모델 v5 라벨과 같은 뜻, 단
    참값 좌표로 계산). rule_level_clean = 참값으로 돌린 팀 점수표 등급(참고용).
    """
    n = len(ped)
    ts = [p[0] for p in ped]
    dist = [math.hypot(v[1] - p[1], v[2] - p[2]) for p, v in zip(ped, veh)]
    rows = []
    for i in range(n):
        p, v = ped[i], veh[i]
        # 참값 속도벡터: 이웃 표본 차분
        j = min(i + 1, n - 1); k = max(i - 1, 0)
        dt = ts[j] - ts[k]
        if dt <= 0:
            pv = vv = (0.0, 0.0)
        else:
            pv = ((ped[j][1] - ped[k][1]) / dt, (ped[j][2] - ped[k][2]) / dt)
            vv = ((veh[j][1] - veh[k][1]) / dt, (veh[j][2] - veh[k][2]) / dt)
        rx, ry = v[1] - p[1], v[2] - p[2]
        vx, vy = vv[0] - pv[0], vv[1] - pv[1]
        d = dist[i]
        closing = 0.0 if d == 0 else -(rx * vx + ry * vy) / d
        ttc = calculate_ttc(d, closing)
        # 앞으로 horizon 초 안의 최근접 거리
        m = d
        for q in range(i, n):
            if ts[q] - ts[i] > horizon_s:
                break
            m = min(m, dist[q])
        score = calculate_risk_score(d, closing, v[3], ttc)
        rows.append({
            "t": round(ts[i], 3), "ped_e": round(p[1], 3), "ped_n": round(p[2], 3),
            "veh_e": round(v[1], 3), "veh_n": round(v[2], 3),
            "distance_m": round(d, 3), "closing_mps": round(closing, 3),
            "ttc_s": round(ttc, 3), "min_dist_3s": round(m, 3),
            "label_hit_3s": int(m <= hit_m), "rule_level_clean": classify_risk_level(score),
        })
    return rows


# ---------------------------------------------------------------- 스트림
def make_packets(ped, veh, params_ped, params_veh, rng, base_epoch=BASE_EPOCH, origin=ORIGIN):
    """두 노드의 잡음 낀 패킷을 시간순으로 합친다. pc_time 은 base_epoch + t."""
    cane = gps_noise.noisify_track(ped, params_ped, rng, origin, "cane")
    car = gps_noise.noisify_track(veh, params_veh, rng, origin, "vehicle")
    merged = cane + car
    for p in merged:
        p["pc_time"] = round(base_epoch + p["pc_time"], 3)
    # 같은 시각이면 지팡이 먼저(실기에서 지팡이 패킷이 근소하게 앞섰던 것과 무관, 결정적 순서용)
    merged.sort(key=lambda p: (p["pc_time"], 0 if p["type"] == "cane" else 1, p["seq"]))
    return merged


def write_stream(packets, path):
    """raw 로그 형식: '<pc_time> RX <json>' 한 줄씩."""
    with Path(path).open("w", encoding="utf-8", newline="\n") as fh:
        for p in packets:
            body = {k: v for k, v in p.items() if k != "pc_time"}
            fh.write(f"{p['pc_time']:.3f} RX {json.dumps(body, separators=(',', ':'))}\n")


def write_truth(rows, path):
    with Path(path).open("w", encoding="utf-8", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


# ---------------------------------------------------------------- CLI
def parse_args():
    ap = argparse.ArgumentParser(description="생성기 v3 시나리오 → RSU 패킷 스트림 + 참값")
    ap.add_argument("scenario_dir")
    ap.add_argument("--vehicle-id", default=None)
    ap.add_argument("--person-id", default=None)
    ap.add_argument("--out", default=None, help="출력 폴더(기본: 시나리오 폴더)")
    ap.add_argument("--seed", type=int, default=None, help="기본: 시나리오 이름 해시")
    ap.add_argument("--randomize", action="store_true", help="환경 의존 잡음을 범위 안에서 무작위")
    ap.add_argument("--stall", action="store_true", help="차량 60초마다 10.8초 정지 재현")
    ap.add_argument("--hz", type=float, default=DEFAULT_HZ)
    return ap.parse_args()


def main():
    args = parse_args()
    sd = Path(args.scenario_dir)
    frames = load_v3_pair(sd, args.vehicle_id, args.person_id)
    seed = args.seed if args.seed is not None else gps_noise.scenario_seed(sd.name)
    rng = np.random.default_rng(seed)
    if args.randomize:
        p_ped, p_veh = gps_noise.FieldNoiseParams.randomized(rng), gps_noise.FieldNoiseParams.randomized(rng)
    else:
        p_ped, p_veh = gps_noise.FieldNoiseParams(), gps_noise.FieldNoiseParams()
    if args.stall:
        p_veh.stall_period_s = 60.0
    ped, veh = interpolate(frames, args.hz)
    packets = make_packets(ped, veh, p_ped, p_veh, rng)
    truth = truth_rows(ped, veh)
    out = Path(args.out) if args.out else sd
    out.mkdir(parents=True, exist_ok=True)
    tag = f"{sd.name}_{args.vehicle_id or 'v0'}_{args.person_id or 'p0'}"
    write_stream(packets, out / f"raw_sim_{tag}.log")
    write_truth(truth, out / f"truth_sim_{tag}.csv")
    hits = sum(r["label_hit_3s"] for r in truth)
    print(f"[OK] {tag}: frames={len(frames)} packets={len(packets)} truth_rows={len(truth)} "
          f"hit_rows={hits} seed={seed} sigma_ped={p_ped.sigma_m:.2f} sigma_veh={p_veh.sigma_m:.2f}")


if __name__ == "__main__":
    main()
