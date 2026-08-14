#!/usr/bin/env python3
"""
SUMO 시나리오 자동 생성기 v3 (Google Cloud = 데이터 생성 서버)

역할: SUMO 실행 -> fcd.xml -> fcd_trajectory.csv -> feature.csv 생성까지만.
TTC / risk_level / AI 추론은 여기서 하지 않는다 (Jetson Nano 담당).

v3 변경점:
  - 교내 차량(campus_*)은 캠퍼스 안 highway.service 도로만 랜덤워크로 순회한다.
    (기존 v2.1은 randomTrips 전 지역 무작위라 "campus_" 이름과 달리 교외를 돌았음)
  - 교외 차량(시내 도로)은 기존 route.rou.xml에서 시나리오마다 랜덤 샘플링.
  - 교내:교외 비율을 시나리오마다 랜덤(약 1:2 ~ 1:20)으로 배정, META에 기록.

생성 구조:
    generated_data/
        scenario_0001/feature.csv + DONE
        ...

사용 예:
    python generate_scenarios.py --count 5      # 5개만 생성
    python generate_scenarios.py                # 무한 반복 (기본)
"""
import argparse
import logging
import os
import random
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pandas as pd

sys.path.append("/usr/share/sumo/tools")
import sumolib

PROJECT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = PROJECT_DIR / "generated_data"
SUMO_CONFIG = PROJECT_DIR / "config.sumocfg"
XML2CSV = Path("/usr/share/sumo/tools/xml/xml2csv.py")
RANDOM_TRIPS = Path("/usr/share/sumo/tools/randomTrips.py")

NET_FILE = PROJECT_DIR / "net_v2.net.xml"
ROUTE_SRC = PROJECT_DIR / "route.rou.xml"
VTYPES_FILE = PROJECT_DIR / "vtypes_mix.add.xml"
DRIVER_TYPES = ["cautious_car", "normal_car", "aggressive_car"]

SIM_END = 200.0            # config.sumocfg의 end와 동일
CAMPUS_CENTER_LONLAT = (126.9528, 37.4977)   # 캠퍼스 중심
CAMPUS_RADIUS_M = 350.0

# 조심/보통 운전 성향 정의 (aggressive_car는 route.rou.xml에 기존 정의됨)
VTYPES_XML = """<additional>
    <vType id="cautious_car" accel="1.5" decel="6.0" sigma="0.3"
           length="5" minGap="3.0" tau="1.6" maxSpeed="25"
           speedFactor="normc(0.85,0.05,0.6,1.0)"/>
    <vType id="normal_car" accel="2.6" decel="4.5" sigma="0.5"
           length="5" minGap="2.5" tau="1.0" maxSpeed="25"
           speedFactor="normc(1.0,0.1,0.7,1.3)"/>
</additional>
"""

log = logging.getLogger("generator")

_city_pool = None      # [(depart, vehicle_xml_block)]
_campus_graph = None   # {edge_id: [캠퍼스 안 후속 edge id들]}


def city_pool() -> list:
    """route.rou.xml에서 시뮬레이션 시간 안에 출발하는 교외 차량 블록 목록."""
    global _city_pool
    if _city_pool is None:
        text = ROUTE_SRC.read_text(encoding="utf-8")
        pool = []
        for block in re.findall(r"<vehicle\b.*?</vehicle>", text, re.DOTALL):
            m = re.search(r'depart="([\d.]+)"', block)
            if m and float(m.group(1)) < SIM_END:
                pool.append((float(m.group(1)), block))
        if not pool:
            raise RuntimeError("route.rou.xml에서 교외 차량을 찾지 못했습니다")
        _city_pool = pool
    return _city_pool


def campus_graph() -> dict:
    """캠퍼스 안 service 도로(승용차 통행 가능)의 연결 그래프."""
    global _campus_graph
    if _campus_graph is None:
        net = sumolib.net.readNet(str(NET_FILE))
        cx, cy = net.convertLonLat2XY(*CAMPUS_CENTER_LONLAT)
        r2 = CAMPUS_RADIUS_M ** 2
        ids = set()
        for e in net.getEdges():
            if e.getFunction() == "internal" or e.getType() != "highway.service":
                continue
            sh = e.getShape()
            ex, ey = sh[len(sh) // 2]
            if (ex - cx) ** 2 + (ey - cy) ** 2 > r2:
                continue
            if any(l.allows("passenger") for l in e.getLanes()):
                ids.add(e.getID())
        if not ids:
            raise RuntimeError(
                "캠퍼스 service 도로가 없습니다 — net_v2.net.xml에 "
                "passenger 허용 패치가 적용됐는지 확인하세요")
        graph = {}
        for i in ids:
            graph[i] = [n.getID() for n in net.getEdge(i).getOutgoing()
                        if n.getID() in ids]
        lengths = {i: net.getEdge(i).getLength() for i in ids}
        _campus_graph = (graph, lengths)
    return _campus_graph


def campus_walk(rng, target_len: float) -> list:
    """캠퍼스 그래프 위 랜덤워크. 캠퍼스 밖 edge는 절대 포함되지 않는다."""
    graph, lengths = campus_graph()
    starts = [i for i in graph if graph[i]]
    cur = rng.choice(starts)
    route, total = [cur], lengths[cur]
    while total < target_len:
        nxt = graph.get(cur) or []
        if not nxt:
            break
        prev = route[-2] if len(route) >= 2 else None
        cand = [n for n in nxt if n != prev] or nxt   # 가능하면 즉시 유턴 회피
        cur = rng.choice(cand)
        route.append(cur)
        total += lengths[cur]
    return route


def pick_vtype(rng, weights) -> str:
    return rng.choices(DRIVER_TYPES, weights=weights)[0]


def make_mixed_route(rng, out_file: Path, n_city: int, weights) -> int:
    """교외 차량을 랜덤 샘플링하고 운전 성향을 배합해 경로 파일 생성."""
    pool = city_pool()
    chosen = sorted(rng.sample(pool, min(n_city, len(pool))), key=lambda p: p[0])

    def assign(match):
        tag = match.group(0)
        tag = re.sub(r'\s+type="[^"]*"', "", tag)
        return tag.replace("<vehicle ", f'<vehicle type="{pick_vtype(rng, weights)}" ', 1)

    blocks = [re.sub(r"<vehicle [^>]*>", assign, b) for _, b in chosen]
    out_file.write_text(
        '<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
        'xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">\n'
        '    <vType id="aggressive_car" accel="3.5" decel="7.0" sigma="1.0"\n'
        '           length="5" minGap="0.5" maxSpeed="25"/>\n\n    '
        + "\n    ".join(blocks) + "\n</routes>\n",
        encoding="utf-8")
    return len(chosen)


def make_campus_routes(rng, out_file: Path, n_campus: int, weights) -> int:
    """캠퍼스 안 service 도로만 도는 교내 차량 경로 파일 생성."""
    gap = 180.0 / max(n_campus, 1)
    lines = []
    for i in range(n_campus):
        route = campus_walk(rng, target_len=rng.uniform(700, 1500))
        depart = i * gap + rng.uniform(0, gap * 0.5)
        lines.append(
            '    <vehicle id="campus_%d" type="%s" depart="%.2f">\n'
            '        <route edges="%s"/>\n'
            "    </vehicle>" % (i, pick_vtype(rng, weights), depart, " ".join(route)))
    out_file.write_text(
        '<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
        'xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">\n'
        + "\n".join(lines) + "\n</routes>\n",
        encoding="utf-8")
    return n_campus


def make_random_pedestrians(num: int, out_file: Path) -> None:
    """시나리오별 랜덤 보행자 생성 (수·경로·출발 시각 모두 다름)."""
    rng = random.Random(num * 7919)
    period = rng.choice([15, 20, 30, 40])  # 작을수록 보행자 많음 (약 3~10명)
    run_cmd([sys.executable, str(RANDOM_TRIPS),
             "-n", str(NET_FILE), "--pedestrians",
             "-o", str(out_file), "--seed", str(num),
             "-p", str(period), "-b", "0", "-e", "150"],
            timeout=120)


def run_cmd(cmd, timeout):
    """명령 실행. 실패 시 stderr 끝부분을 포함한 에러를 던진다."""
    env = dict(os.environ)
    env["PYTHONPATH"] = "/usr/share/sumo/tools" + (
        os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else ""
    )
    result = subprocess.run(
        cmd, cwd=PROJECT_DIR, capture_output=True, text=True,
        timeout=timeout, env=env,
    )
    if result.returncode != 0:
        tail = (result.stderr or result.stdout or "").strip()[-500:]
        raise RuntimeError(f"명령 실패 {cmd[0]} (exit {result.returncode}): {tail}")


def next_scenario_number() -> int:
    """폴더 스캔 + 영속 카운터의 최댓값 사용.

    cleanup이 폴더를 지워도 번호가 뒤로 돌아가지 않도록 카운터 파일에
    마지막 번호를 기억한다 (문자열 정렬 keep-30 때문에 10xxx가 전부
    지워지고 9xxx만 남으면 번호가 재사용되던 버그 방지).
    """
    OUTPUT_DIR.mkdir(exist_ok=True)
    counter_file = PROJECT_DIR / ".scenario_counter"
    nums = []
    for d in OUTPUT_DIR.glob("scenario_*"):
        try:
            nums.append(int(d.name.split("_")[1]))
        except (IndexError, ValueError):
            pass
    try:
        nums.append(int(counter_file.read_text().strip()))
    except (OSError, ValueError):
        pass
    n = max(nums, default=0) + 1
    counter_file.write_text(str(n))
    return n


def generate_one(num: int, keep_intermediate: bool) -> Path:
    scenario_dir = OUTPUT_DIR / f"scenario_{num:04d}"
    scenario_dir.mkdir(parents=True, exist_ok=True)

    if not VTYPES_FILE.exists():
        VTYPES_FILE.write_text(VTYPES_XML, encoding="utf-8")

    rng = random.Random(num * 31337)

    # 운전 성향 배합 (시나리오마다 랜덤)
    w = [rng.uniform(0.2, 0.5), rng.uniform(0.3, 0.6), rng.uniform(0.1, 0.4)]
    total = sum(w)
    weights = [round(x / total, 2) for x in w]

    # 교내:교외 비율 랜덤 (약 1:2 ~ 1:20)
    ratio = rng.uniform(0.05, 0.5)
    n_city = rng.randint(60, 180)
    n_campus = max(2, int(n_city * ratio))

    route_file = scenario_dir / "route_mixed.rou.xml"
    campus_file = scenario_dir / "veh_campus.rou.xml"
    ped_file = scenario_dir / "ped_random.rou.xml"
    n_city = make_mixed_route(rng, route_file, n_city, weights)
    n_campus = make_campus_routes(rng, campus_file, n_campus, weights)
    make_random_pedestrians(num, ped_file)

    # SUMO 실행 (횡단보도 있는 net_v2 + 혼합 성향 + 랜덤 보행자)
    fcd_xml = scenario_dir / "fcd.xml"
    run_cmd(
        [
            "sumo",
            "-c", str(SUMO_CONFIG),
            "-n", str(NET_FILE),
            "-r", f"{route_file},{campus_file},{ped_file}",
            "--additional-files", str(VTYPES_FILE),
            "--seed", str(num),
            "--fcd-output", str(fcd_xml),
            "--no-step-log", "true",
        ],
        timeout=600,
    )
    (scenario_dir / "META").write_text(
        f"generator=v3 campus_random_walk "
        f"n_campus={n_campus} n_city={n_city} ratio={n_campus / n_city:.3f} "
        f"driver_mix(cautious/normal/aggressive)={weights}\n")
    if not fcd_xml.exists() or fcd_xml.stat().st_size == 0:
        raise RuntimeError("fcd.xml이 생성되지 않았습니다")

    # fcd.xml -> fcd_trajectory.csv (SUMO 기본 도구 사용)
    trajectory_csv = scenario_dir / "fcd_trajectory.csv"
    run_cmd(
        [sys.executable, str(XML2CSV), str(fcd_xml), "-o", str(trajectory_csv)],
        timeout=300,
    )

    # 전처리 -> feature.csv / pedestrian.csv
    df = pd.read_csv(trajectory_csv, sep=";")

    veh_cols = ["timestep_time"] + [c for c in df.columns
                                    if c.startswith("vehicle_")]
    veh = df[veh_cols].dropna(subset=["vehicle_id"])
    if veh.empty:
        raise RuntimeError("차량 데이터가 비어 있습니다")
    feature_csv = scenario_dir / "feature.csv"
    veh.to_csv(feature_csv, sep=";", index=False)

    if "person_id" in df.columns:
        ped_cols = ["timestep_time"] + [c for c in df.columns
                                        if c.startswith("person_")]
        ped = df[ped_cols].dropna(subset=["person_id"])
        if not ped.empty:
            ped.to_csv(scenario_dir / "pedestrian.csv", sep=";", index=False)

    # 중간 파일 정리 (디스크 절약)
    if not keep_intermediate:
        fcd_xml.unlink(missing_ok=True)
        trajectory_csv.unlink(missing_ok=True)
        route_file.unlink(missing_ok=True)
        ped_file.unlink(missing_ok=True)
        campus_file.unlink(missing_ok=True)

    # 완료 표시 (Jetson은 이 파일이 있는 폴더만 가져간다)
    (scenario_dir / "DONE").write_text(time.strftime("%Y-%m-%dT%H:%M:%S\n"))
    return feature_csv


def disk_free_gb() -> float:
    return shutil.disk_usage(PROJECT_DIR).free / 1e9


def main() -> None:
    p = argparse.ArgumentParser(description="SUMO 시나리오 자동 생성기")
    p.add_argument("--count", type=int, default=0,
                   help="생성할 시나리오 수 (0 = 무한 반복)")
    p.add_argument("--interval", type=float, default=30.0,
                   help="시나리오 간 대기 시간(초)")
    p.add_argument("--min-free-gb", type=float, default=2.0,
                   help="디스크 여유 공간이 이보다 작으면 생성을 멈추고 대기")
    p.add_argument("--keep-intermediate", action="store_true",
                   help="fcd.xml / fcd_trajectory.csv를 삭제하지 않고 보존")
    args = p.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        handlers=[
            logging.FileHandler(PROJECT_DIR / "generator.log"),
            logging.StreamHandler(),
        ],
    )
    log.info("생성기 시작 v3 (count=%s, interval=%ss)",
             args.count or "무한", args.interval)

    made = 0
    consecutive_failures = 0
    while args.count == 0 or made < args.count:
        if disk_free_gb() < args.min_free_gb:
            log.warning("디스크 여유 공간 부족(%.1fGB) — 10분 후 재확인",
                        disk_free_gb())
            time.sleep(600)
            continue

        num = next_scenario_number()
        try:
            feature = generate_one(num, args.keep_intermediate)
            with open(feature) as f:
                rows = sum(1 for _ in f) - 1
            log.info("scenario_%04d 완료 (%d행)", num, rows)
            made += 1
            consecutive_failures = 0
        except Exception:
            consecutive_failures += 1
            log.exception("scenario_%04d 실패 (연속 %d회)",
                          num, consecutive_failures)
            shutil.rmtree(OUTPUT_DIR / f"scenario_{num:04d}",
                          ignore_errors=True)
            time.sleep(min(60 * consecutive_failures, 600))
            continue

        time.sleep(args.interval)

    log.info("총 %d개 시나리오 생성 완료", made)


if __name__ == "__main__":
    main()
