#!/usr/bin/env python3
"""SUMO 생성기 v3 시나리오 → "보행자 1명 + 차량 여러 대" RSU 패킷 스트림 + 차량별 참값.

왜 따로 있나
    sim_to_rsu_stream.py는 (차 1대, 보행자 1명) 쌍만 스트림으로 만든다. 그래서
    SUMO로 만든 시험에는 "차 두 대가 같은 보행자에게 동시에 다가오는" 상황이 아예
    없었고, 젯슨 step4가 차량을 한 칸에만 저장해 두 차가 서로 덮어쓰는 문제가 드러나지
    않았다. 이 스크립트는 그 상황을 SUMO에서 찾아 스트림으로 만든다. 잡음 모델·참값
    계산은 sim_to_rsu_stream / gps_noise를 그대로 쓴다(새로 만든 것은 다중 차량 배치뿐).

찾는 것 (find_multi_approaches)
    한 보행자에게 near_m 안까지 오는 차가, 최근접 시각이 window_s 안에 모여 2대 이상.

출력 (CLI)
    <out>/raw_multi_<시나리오>_p<보행자>_<N>v.log      RSU raw 로그 형식(pc_time RX {json})
    <out>/truth_multi_<시나리오>_p<보행자>_<차>.csv     차량별 5 Hz 참값(sim_to_rsu_stream과 같은 열)
    <out>/label_multi_<시나리오>_p<보행자>_<차>.csv     차량별 1 Hz SUMO 위험 라벨(채점표 x DCPA 게이트,
                                                        v2~v5 학습 라벨과 같은 식 - sumo_labels)

사용
    python sim_multi_vehicle_stream.py <scenario_dir> [--out DIR] [--near-m 15] [--randomize]
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import gps_noise  # noqa: E402
from sim_to_rsu_stream import (  # noqa: E402
    BASE_EPOCH, DEFAULT_HZ, ORIGIN, _heading_from, truth_rows, write_stream, write_truth,
)

CANE_NODE_ID = 4125577512       # gps_noise 기본값과 같은 실기 지팡이 id
VEHICLE_NODE_BASE = 2882630492  # 실기 차량 id부터 1씩 - 차마다 다른 node_id
WARMUP_S = 20.0                 # 첫 최근접보다 이만큼 앞부터 재생(칼만·이력 준비)
TAIL_S = 5.0                    # 마지막 최근접 뒤로 이만큼 더


# ---------------------------------------------------------------- 찾기
def _read(scenario_dir):
    import pandas as pd

    scenario_dir = Path(scenario_dir)
    veh = pd.read_csv(scenario_dir / "feature.csv", sep=";")
    ped = pd.read_csv(scenario_dir / "pedestrian.csv", sep=";")
    veh["vehicle_id"] = veh["vehicle_id"].astype(str)
    ped["person_id"] = ped["person_id"].astype(str)
    return veh, ped


def find_multi_approaches(scenario_dir, near_m=15.0, window_s=10.0, min_vehicles=2):
    """한 보행자에게 near_m 안까지 오는 차가 window_s 안에 min_vehicles대 이상 모인 묶음.

    반환: [{"person_id", "vehicles": [(차 id, 최근접 시각, 최근접 거리)]}] - 보행자마다
    가장 많은 차가 모인 묶음 하나.
    """
    veh, ped = _read(scenario_dir)
    df = veh[["timestep_time", "vehicle_id", "vehicle_x", "vehicle_y"]].merge(
        ped[["timestep_time", "person_id", "person_x", "person_y"]], on="timestep_time")
    if df.empty:
        return []
    df["d"] = np.hypot(df["vehicle_x"] - df["person_x"], df["vehicle_y"] - df["person_y"])
    closest = df.loc[df.groupby(["person_id", "vehicle_id"])["d"].idxmin()]
    closest = closest[closest["d"] <= near_m]
    found = []
    for pid, group in closest.groupby("person_id"):
        group = group.sort_values("timestep_time")
        times = group["timestep_time"].to_numpy()
        best = None
        for i in range(len(group)):
            j = np.searchsorted(times, times[i] + window_s, side="right")
            if j - i >= min_vehicles and (best is None or j - i > best[1] - best[0]):
                best = (i, j)
        if best is None:
            continue
        rows = group.iloc[best[0]:best[1]]
        found.append({
            "person_id": pid,
            "vehicles": [(r.vehicle_id, float(r.timestep_time), round(float(r.d), 2))
                         for r in rows.itertuples()],
        })
    return found


# ---------------------------------------------------------------- 궤적
def _track(points, hz, x0, y0):
    """1 Hz [(t, x, y, speed, angle)] → hz 표본 [(t, east, north, speed, heading)].

    sim_to_rsu_stream.interpolate의 한 노드 몫과 같은 규칙: 선형 보간, 방향은 그
    1초의 변위 방향(정지면 SUMO angle).
    """
    steps = max(1, int(round(hz)))
    out = []
    for a, b in zip(points, points[1:]):
        dt = b[0] - a[0]
        if dt <= 0 or dt > 1.5:
            # SUMO 출력이 끊긴 구간(차가 사라졌다 다시 나타남)은 이어 붙이지 않는다.
            continue
        heading = _heading_from(b[1] - a[1], b[2] - a[2], a[4])
        for k in range(steps):
            f = k / steps
            out.append((a[0] + f * dt, a[1] + f * (b[1] - a[1]) - x0,
                        a[2] + f * (b[2] - a[2]) - y0, a[3] + f * (b[3] - a[3]), heading))
    if points and out:
        last = points[-1]
        out.append((last[0], last[1] - x0, last[2] - y0, last[3], out[-1][4]))
    return out


def load_episode(scenario_dir, person_id, vehicle_ids, warmup_s=WARMUP_S, tail_s=TAIL_S,
                 t_focus=None, hz=DEFAULT_HZ):
    """(보행자 표본, {차 id: 표본}) - 모두 같은 원점(재생 구간 첫 보행자 위치) 기준.

    t_focus=(첫 최근접, 마지막 최근접)을 주면 그 앞 warmup_s ~ 뒤 tail_s만 자른다.
    """
    veh, ped = _read(scenario_dir)
    p = ped[ped["person_id"] == str(person_id)].sort_values("timestep_time")
    if t_focus is not None:
        t0, t1 = t_focus[0] - warmup_s, t_focus[1] + tail_s
        p = p[(p["timestep_time"] >= t0) & (p["timestep_time"] <= t1)]
    if len(p) < 2:
        raise ValueError("보행자 표본이 2개 미만")
    x0, y0 = float(p["person_x"].iloc[0]), float(p["person_y"].iloc[0])
    t_lo, t_hi = float(p["timestep_time"].iloc[0]), float(p["timestep_time"].iloc[-1])
    ped_pts = list(zip(p["timestep_time"].astype(float), p["person_x"], p["person_y"],
                       p["person_speed"].fillna(0.0), p["person_angle"].fillna(0.0)))
    tracks = {}
    for vid in vehicle_ids:
        v = veh[(veh["vehicle_id"] == str(vid)) & (veh["timestep_time"] >= t_lo)
                & (veh["timestep_time"] <= t_hi)].sort_values("timestep_time")
        pts = list(zip(v["timestep_time"].astype(float), v["vehicle_x"], v["vehicle_y"],
                       v["vehicle_speed"].fillna(0.0), v["vehicle_angle"].fillna(0.0)))
        track = _track(pts, hz, x0, y0)
        if len(track) >= 2:
            tracks[str(vid)] = track
    return _track(ped_pts, hz, x0, y0), tracks


def aligned_truth(ped, track):
    """차 표본 시각에 맞춘 보행자 표본으로 sim_to_rsu_stream.truth_rows를 계산한다."""
    ped_by_t = {round(s[0], 3): s for s in ped}
    pairs = [(ped_by_t[round(s[0], 3)], s) for s in track if round(s[0], 3) in ped_by_t]
    if len(pairs) < 2:
        return []
    return truth_rows([a for a, _ in pairs], [b for _, b in pairs])


# ---------------------------------------------------------------- SUMO 위험 라벨
LABEL_HORIZON_S = 3   # risk_level_future3 = 향후 1~3초 최대 (build_v3_and_baseline과 같음)


def _labeler():
    """학습 라벨을 만든 build_dataset_local(팀 채점표·DCPA 게이트 동결 사본)을 불러온다."""
    transformer = HERE.parents[3] / "AI_Model" / "transformer"
    if str(transformer) not in sys.path:
        sys.path.insert(0, str(transformer))
    import build_dataset_local
    return build_dataset_local


def sumo_labels(scenario_dir, person_id, vehicle_ids):
    """{차 id: 1 Hz 라벨 DataFrame} - v2~v5 학습 라벨과 같은 식을 이 (차, 보행자) 쌍에 적용.

    build_dataset_local.build_scenario의 쌍 계산을 그대로 옮겼다: 참값 좌표(잡음 없음)로
    거리, 거리 미분 접근속도, TTC(30 s 클램프), 팀 채점표, 상대속도 벡터 DCPA,
    dcpa_gate(2.5/7.5 m, 바닥 0.2) -> risk_level. 미래 라벨 risk_level_future3는
    build_v3_and_baseline처럼 향후 1~3초 최대(끝 3초는 있는 만큼만, future_full=False).
    build_scenario는 차마다 "최근접 보행자" 한 명을 고르지만 여기서는 지팡이 보행자로
    고정한다 - 묻는 것이 "이 차가 이 보행자에게 위험했나"이기 때문이다.
    """
    import pandas as pd

    bdl = _labeler()
    veh, ped = _read(scenario_dir)
    p = ped[ped["person_id"] == str(person_id)][["timestep_time", "person_x", "person_y"]]
    out = {}
    for vid in vehicle_ids:
        v = veh[veh["vehicle_id"] == str(vid)][
            ["timestep_time", "vehicle_x", "vehicle_y", "vehicle_speed"]]
        df = v.merge(p, on="timestep_time").sort_values("timestep_time").reset_index(drop=True)
        if len(df) < 2:
            continue
        dt = df["timestep_time"].diff()
        rx = df["vehicle_x"] - df["person_x"]
        ry = df["vehicle_y"] - df["person_y"]
        dist = np.sqrt(rx ** 2 + ry ** 2)
        rel = (-dist.diff() / dt).fillna(0.0)
        ttc = np.minimum([bdl.calculate_ttc(d, r) for d, r in zip(dist, rel)], bdl.TTC_CLAMP_S)
        score = [bdl.calculate_risk_score(d, r, s, t, bdl.ZONE_BASE_RISK)
                 for d, r, s, t in zip(dist, rel, df["vehicle_speed"], ttc)]
        dvx, dvy = rx.diff() / dt, ry.diff() / dt
        v2 = dvx ** 2 + dvy ** 2
        t_cpa = -(rx * dvx + ry * dvy) / v2.where(v2 > 1e-6)
        dcpa = np.sqrt((rx + dvx * t_cpa) ** 2 + (ry + dvy * t_cpa) ** 2)
        dcpa = dcpa.where((t_cpa > 0) & v2.notna(), dist).fillna(dist)
        gate = [bdl.dcpa_gate(d) for d in dcpa]
        level = pd.Series([bdl.classify_risk_level(s * g) for s, g in zip(score, gate)])
        fut = pd.concat([level.shift(-k) for k in range(1, LABEL_HORIZON_S + 1)], axis=1)
        out[str(vid)] = pd.DataFrame({
            "t": df["timestep_time"].astype(float),
            "distance_m": dist.round(3),
            "risk_score": score,
            "dcpa_m": dcpa.round(3),
            "gate": np.round(gate, 3),
            "risk_level": level,
            "risk_level_future3": fut.max(axis=1).fillna(level).astype(int),
            "future_full": fut.notna().all(axis=1),
        })
    return out


# ---------------------------------------------------------------- 패킷
def node_packets(ped, tracks, seed, randomize=False, origin=ORIGIN):
    """노드별 잡음 패킷 {"cane": [...], 차 id: [...]}. pc_time = BASE_EPOCH + t.

    노드마다 난수열을 따로 둔다. 그래야 차 한 대만 뺀 스트림(기준 재생)을 만들어도
    남은 노드의 잡음이 전체 스트림 때와 똑같다 - 비교가 공정해진다.
    """
    def params(rng):
        return gps_noise.FieldNoiseParams.randomized(rng) if randomize else gps_noise.FieldNoiseParams()

    out = {}
    rng = np.random.default_rng(seed)
    out["cane"] = gps_noise.noisify_track(ped, params(rng), rng, origin, "cane",
                                          node_id=CANE_NODE_ID)
    for i, (vid, track) in enumerate(sorted(tracks.items())):
        rng = np.random.default_rng(seed + 1000 * (i + 1))
        out[vid] = gps_noise.noisify_track(track, params(rng), rng, origin, "vehicle",
                                           node_id=VEHICLE_NODE_BASE + i)
    for packets in out.values():
        for p in packets:
            p["pc_time"] = round(BASE_EPOCH + p["pc_time"], 3)
    return out


def merge_packets(per_node, keep=None):
    """노드별 패킷을 시간순 하나로. keep을 주면 그 노드들만("cane"은 항상)."""
    merged = []
    for node, packets in per_node.items():
        if node == "cane" or keep is None or node in keep:
            merged.extend(packets)
    merged.sort(key=lambda p: (p["pc_time"], 0 if p["type"] == "cane" else 1,
                               p["node_id"], p["seq"]))
    return merged


def node_id_map(tracks):
    """차 id → 패킷 node_id (node_packets와 같은 순서 규칙)."""
    return {vid: VEHICLE_NODE_BASE + i for i, vid in enumerate(sorted(tracks))}


# ---------------------------------------------------------------- CLI
def parse_args():
    ap = argparse.ArgumentParser(description="SUMO 시나리오 → 다중 차량 RSU 패킷 스트림 + 참값")
    ap.add_argument("scenario_dir")
    ap.add_argument("--out", default=None, help="출력 폴더(기본: 시나리오 폴더)")
    ap.add_argument("--near-m", type=float, default=15.0)
    ap.add_argument("--window-s", type=float, default=10.0)
    ap.add_argument("--seed", type=int, default=None, help="기본: 시나리오 이름 해시")
    ap.add_argument("--randomize", action="store_true", help="환경 의존 잡음을 노드마다 무작위")
    ap.add_argument("--hz", type=float, default=DEFAULT_HZ)
    return ap.parse_args()


def main():
    args = parse_args()
    sd = Path(args.scenario_dir)
    found = find_multi_approaches(sd, args.near_m, args.window_s)
    if not found:
        print(f"[SKIP] {sd.name}: {args.near_m} m 안으로 {args.window_s} s 안에 2대 이상 오는 경우 없음")
        return
    out = Path(args.out) if args.out else sd
    out.mkdir(parents=True, exist_ok=True)
    seed = args.seed if args.seed is not None else gps_noise.scenario_seed(sd.name)
    for item in found:
        vids = [v for v, _, _ in item["vehicles"]]
        t_min = [t for _, t, _ in item["vehicles"]]
        ped, tracks = load_episode(sd, item["person_id"], vids, t_focus=(min(t_min), max(t_min)),
                                   hz=args.hz)
        if len(tracks) < 2:
            continue
        per_node = node_packets(ped, tracks, seed, args.randomize)
        packets = merge_packets(per_node)
        tag = f"{sd.name}_p{item['person_id']}"
        write_stream(packets, out / f"raw_multi_{tag}_{len(tracks)}v.log")
        ids = node_id_map(tracks)
        labels = sumo_labels(sd, item["person_id"], list(tracks))
        for vid, track in tracks.items():
            rows = aligned_truth(ped, track)
            for r in rows:
                r["node_id"] = ids[vid]
            if rows:
                write_truth(rows, out / f"truth_multi_{tag}_{vid}.csv")
            if vid in labels:
                lab = labels[vid]
                lab.insert(0, "node_id", ids[vid])
                lab.to_csv(out / f"label_multi_{tag}_{vid}.csv", index=False)
        print(f"[OK] {tag}: vehicles={item['vehicles']} packets={len(packets)}")


if __name__ == "__main__":
    main()
