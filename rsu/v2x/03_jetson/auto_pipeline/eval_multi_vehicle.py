#!/usr/bin/env python3
"""SUMO 다중 차량 접근 재생: 기존 step8(차량 칸 1개) vs step8_multi_vehicle(차선 분리).

무엇을 재나
    같은 잡음 패킷을 세 가지로 재생한다.
      기준(차별)  차 한 대 + 지팡이만 남긴 스트림을 기존 step8로 - "그 차만 있었다면
                  울렸을 경보". 차량 칸 덮어쓰기가 없으므로 단일 차량 판정의 정답지다.
      기존        모든 차 + 지팡이를 기존 step8로 (두 차가 한 칸을 번갈아 덮어씀).
      다중        모든 차 + 지팡이를 step8_multi_vehicle(max 다운링크)로.

    1) 경보 유지율: 기준에서 경보(L2+)가 울린 구간마다, 같은 구간에 지팡이 다운링크가
       L2+였나. 놓친 구간 = 그 차만 있었으면 울렸을 경보가 다른 차 때문에 안 울림.
    2) 근접 통과 적시 경보(참값): SUMO 참값으로 near_hit_m 안까지 온 차마다, 최근접
       3초 전~최근접 사이에 L2+가 있었나. 잡음 없는 좌표 기준이라 기준과 독립이다.
    3) 가짜 경보: 지팡이가 L2+인데 어느 차의 기준도 L2+가 아닌 시간(초).

    모델은 끈다(규칙만) - 추적 분리 효과만 보기 위해서. 그 밖의 정책은 step8 기본값.

사용
    python eval_multi_vehicle.py <v5data 폴더> [--limit 200] [--out multi_vehicle_eval]
"""
import argparse
import contextlib
import csv
import io
import json
import sys
import time
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import gps_noise  # noqa: E402
import sim_multi_vehicle_stream as sm  # noqa: E402
import step8_multi_vehicle as mv  # noqa: E402
import step8_send_risk as s8  # noqa: E402
from model_gate import ModelGate  # noqa: E402
from sim_to_rsu_stream import BASE_EPOCH  # noqa: E402
from step3_parse_v2x import normalize_record  # noqa: E402
from step4_state_store import StateStore  # noqa: E402
from step6_kinematics import KinematicsPipeline  # noqa: E402
from step7_risk import DCPA_FAR_M, DCPA_FLOOR, DCPA_NEAR_M, T_FLOOR_TTC_S  # noqa: E402
from step8_stability import HOLD_S, LevelStabilizer  # noqa: E402

ALARM = 2
GATE_PARAMS = {"near_m": DCPA_NEAR_M, "far_m": DCPA_FAR_M, "floor": DCPA_FLOOR,
               "floor_ttc_s": T_FLOOR_TTC_S}
LEAD_S = 3.0


# ---------------------------------------------------------------- 재생
def _replay(packets, build):
    """packets를 원본 시각 그대로 흘린다. 반환: (보낸 [(t, risk)], sender)."""
    state = {"now": None}
    sent = []

    def transport(command):
        sent.append((state["now"], json.loads(command)["risk"]))

    sender = build(transport)
    with mock.patch.object(s8, "normalize_record",
                           lambda payload, mode: normalize_record(payload, mode, now=state["now"])), \
            mock.patch.object(s8, "append_row", lambda path, row: None), \
            contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        for p in packets:
            state["now"] = p["pc_time"]
            body = {k: v for k, v in p.items() if k != "pc_time"}
            sender.process_line(json.dumps(body, separators=(",", ":")), "simulation")
    return sent, sender


def replay_single(packets):
    def build(transport):
        return s8.RiskSender(
            KinematicsPipeline(StateStore()), s8.RiskTransmitter(target_id=0),
            transport=transport, csv_path=None, gate_params=GATE_PARAMS,
            stabilizer=LevelStabilizer(hold_s=HOLD_S), model_gate=ModelGate())
    return _replay(packets, build)[0]


def replay_multi(packets):
    def build(transport):
        return mv.MultiVehicleSender(mv.LaneConfig(gate_params=GATE_PARAMS), transport)
    return _replay(packets, build)[0]


# ---------------------------------------------------------------- 지표
def level_at(sent, t):
    level = 0
    for ts, risk in sent:
        if ts > t:
            break
        level = risk
    return level


def alarm_intervals(sent, end_t):
    """경보(L2+)가 켜져 있던 [시작, 끝) 구간들."""
    out, start = [], None
    for ts, risk in sent:
        if risk >= ALARM and start is None:
            start = ts
        elif risk < ALARM and start is not None:
            out.append((start, ts))
            start = None
    if start is not None:
        out.append((start, end_t))
    return out


def alarm_during(sent, start, end):
    """[start, end] 동안 한 번이라도 L2+였나, 처음 L2+가 된 시각(없으면 None)."""
    if level_at(sent, start) >= ALARM:
        return True, start
    for ts, risk in sent:
        if start < ts <= end and risk >= ALARM:
            return True, ts
    return False, None


def covered_time(intervals_a, intervals_b):
    """a 구간 중 b 어느 구간에도 안 걸리는 시간(초)."""
    extra = 0.0
    for s, e in intervals_a:
        cuts = sorted((max(s, bs), min(e, be)) for bs, be in intervals_b if be > s and bs < e)
        covered, cursor = 0.0, s
        for cs, ce in cuts:
            if ce > cursor:
                covered += ce - max(cs, cursor)
                cursor = ce
        extra += (e - s) - covered
    return extra


def evaluate_episode(scenario_dir, item, near_hit_m, randomize):
    vids = [v for v, _, _ in item["vehicles"]]
    t_min = [t for _, t, _ in item["vehicles"]]
    ped, tracks = sm.load_episode(scenario_dir, item["person_id"], vids,
                                  t_focus=(min(t_min), max(t_min)))
    if len(tracks) < 2:
        return None
    seed = gps_noise.scenario_seed(f"{scenario_dir.name}_p{item['person_id']}")
    per_node = sm.node_packets(ped, tracks, seed, randomize)
    full = sm.merge_packets(per_node)
    end_t = full[-1]["pc_time"]

    old = replay_single(full)
    new = replay_multi(full)
    refs = {vid: replay_single(sm.merge_packets(per_node, keep={vid})) for vid in tracks}
    ref_intervals = {vid: alarm_intervals(sent, end_t) for vid, sent in refs.items()}
    all_ref = [iv for ivs in ref_intervals.values() for iv in ivs]

    rows = []
    for vid, track in tracks.items():
        truth = sm.aligned_truth(ped, track)
        d_min, t_cpa = min((r["distance_m"], r["t"]) for r in truth) if truth else (None, None)
        row = {"scenario": scenario_dir.name, "person_id": item["person_id"], "vehicle_id": vid,
               "n_vehicles": len(tracks), "truth_min_dist_m": d_min,
               "ref_alarm_episodes": len(ref_intervals[vid])}
        for name, sent in (("old", old), ("new", new)):
            hit, delays = 0, []
            for s, e in ref_intervals[vid]:
                ok, first = alarm_during(sent, s, e)
                if ok:
                    hit += 1
                    delays.append(first - s)
            row[f"{name}_kept"] = hit
            row[f"{name}_delay_s"] = round(max(delays), 2) if delays else None
        if d_min is not None and d_min <= near_hit_m:
            cpa = BASE_EPOCH + t_cpa
            for name, sent in (("ref", refs[vid]), ("old", old), ("new", new)):
                row[f"{name}_timely"] = int(alarm_during(sent, cpa - LEAD_S, cpa)[0])
        rows.append(row)
    episode = {
        "duration_s": end_t - full[0]["pc_time"],
        "old_extra_s": covered_time(alarm_intervals(old, end_t), all_ref),
        "new_extra_s": covered_time(alarm_intervals(new, end_t), all_ref),
    }
    return rows, episode


# ---------------------------------------------------------------- 실행
def summarize(rows, episodes, near_hit_m):
    eps = sum(r["ref_alarm_episodes"] for r in rows)
    lines = []
    lines.append(f"차량 {len(rows)}대 / 묶음 {len(episodes)}개 / 재생 {sum(e['duration_s'] for e in episodes) / 60:.1f}분")
    lines.append(f"차 한 대만 있었다면 울렸을 경보 구간: {eps}개 (차 {sum(1 for r in rows if r['ref_alarm_episodes'])}대)")
    for name, label in (("old", "기존(칸 1개)"), ("new", "다중(차선)")):
        kept = sum(r[f"{name}_kept"] for r in rows)
        missed_cars = sum(1 for r in rows if r[f"{name}_kept"] < r["ref_alarm_episodes"])
        lines.append(f"  {label}: 유지 {kept}/{eps} ({kept / max(1, eps) * 100:.1f}%), "
                     f"경보를 하나라도 놓친 차 {missed_cars}대")
    near = [r for r in rows if "ref_timely" in r]
    if near:
        lines.append(f"참값 {near_hit_m} m 안 근접 통과 차 {len(near)}대 - 최근접 {LEAD_S:.0f}초 전 안에 L2+:")
        for name, label in (("ref", "기준(차별 단독)"), ("old", "기존(칸 1개)"), ("new", "다중(차선)")):
            ok = sum(r[f"{name}_timely"] for r in near)
            lines.append(f"  {label}: {ok}/{len(near)} ({ok / len(near) * 100:.1f}%)")
    total = sum(e["duration_s"] for e in episodes)
    for name, label in (("old", "기존(칸 1개)"), ("new", "다중(차선)")):
        extra = sum(e[f"{name}_extra_s"] for e in episodes)
        lines.append(f"가짜 경보(어느 차 기준도 L2+ 아님) {label}: {extra:.1f}초 ({extra / max(1e-9, total) * 100:.2f}% 시간)")
    return lines


def main():
    ap = argparse.ArgumentParser(description="SUMO 다중 차량 접근: 기존 step8 vs 다중 차선")
    ap.add_argument("data_dir")
    ap.add_argument("--limit", type=int, default=200, help="묶음(보행자+차 여러 대) 개수 상한")
    ap.add_argument("--near-m", type=float, default=15.0)
    ap.add_argument("--window-s", type=float, default=10.0)
    ap.add_argument("--near-hit-m", type=float, default=3.0)
    ap.add_argument("--randomize", action="store_true")
    ap.add_argument("--out", default=str(HERE / "multi_vehicle_eval"))
    args = ap.parse_args()

    dirs = sorted(d for d in Path(args.data_dir).glob("scenario_*") if (d / "DONE").exists())
    rows, episodes = [], []
    started = time.time()
    for d in dirs:
        if len(episodes) >= args.limit:
            break
        for item in sm.find_multi_approaches(d, args.near_m, args.window_s):
            if len(episodes) >= args.limit:
                break
            try:
                result = evaluate_episode(d, item, args.near_hit_m, args.randomize)
            except Exception as exc:  # noqa: BLE001 - 한 묶음 실패로 전체를 멈추지 않는다
                print(f"[WARN] {d.name} p{item['person_id']}: {exc!r}", file=sys.stderr)
                continue
            if result is None:
                continue
            rows.extend(result[0])
            episodes.append(result[1])
            if len(episodes) % 20 == 0:
                print(f"{len(episodes)}/{args.limit} 묶음 ({time.time() - started:.0f}s)", flush=True)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    fields = []
    for r in rows:
        fields.extend(k for k in r if k not in fields)
    with (out / "per_vehicle.csv").open("w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    summary = summarize(rows, episodes, args.near_hit_m)
    (out / "summary.txt").write_text("\n".join(summary) + "\n", encoding="utf-8")
    print("\n".join(summary))


if __name__ == "__main__":
    main()
