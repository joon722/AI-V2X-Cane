#!/usr/bin/env python3
"""
SUMO 시나리오 자동 생성기 (Google Cloud = 데이터 생성 서버)

역할: SUMO 실행 -> fcd.xml -> fcd_trajectory.csv -> feature.csv 생성까지만.
TTC / risk_level / AI 추론은 여기서 하지 않는다 (Jetson Nano 담당).

생성 구조:
    generated_data/
        scenario_0001/feature.csv + DONE
        scenario_0002/feature.csv + DONE
        ...

DONE 파일은 시나리오 생성이 완전히 끝났다는 표시로,
Jetson은 DONE이 있는 폴더만 가져가면 된다.

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

PROJECT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = PROJECT_DIR / "generated_data"
SUMO_CONFIG = PROJECT_DIR / "config.sumocfg"
XML2CSV = Path("/usr/share/sumo/tools/xml/xml2csv.py")
RANDOM_TRIPS = Path("/usr/share/sumo/tools/randomTrips.py")

# ── 시나리오 다양화(v2) 설정 ─────────────────────────────
# net_v2: 인도 자동 생성(448개) + 횡단보도 추가 — 보행자가 인도를 걷고
# 횡단보도에서 차량이 실제로 양보/감속한다.
NET_FILE = PROJECT_DIR / "net_v2.net.xml"
ROUTE_SRC = PROJECT_DIR / "route.rou.xml"
VTYPES_FILE = PROJECT_DIR / "vtypes_mix.add.xml"
DRIVER_TYPES = ["cautious_car", "normal_car", "aggressive_car"]

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

_route_cache = None


def load_route_template() -> str:
    global _route_cache
    if _route_cache is None:
        _route_cache = ROUTE_SRC.read_text(encoding="utf-8")
    return _route_cache


def make_mixed_route(num: int, out_file: Path) -> list:
    """시나리오별로 차량 운전 성향을 랜덤 비율로 배합한 경로 파일 생성."""
    rng = random.Random(num)
    w = [rng.uniform(0.2, 0.5), rng.uniform(0.3, 0.6), rng.uniform(0.1, 0.4)]
    total = sum(w)
    weights = [x / total for x in w]

    def assign(match):
        vtype = rng.choices(DRIVER_TYPES, weights=weights)[0]
        tag = match.group(0)
        tag = re.sub(r'\s+type="[^"]*"', "", tag)  # 기존 type 제거
        return tag.replace("<vehicle ", f'<vehicle type="{vtype}" ', 1)

    text = re.sub(r"<vehicle [^>]*>", assign, load_route_template())
    out_file.write_text(text, encoding="utf-8")
    return [round(x, 2) for x in weights]


def make_random_pedestrians(num: int, out_file: Path) -> None:
    """시나리오별 랜덤 보행자 생성 (수·경로·출발 시각 모두 다름)."""
    rng = random.Random(num * 7919)
    period = rng.choice([15, 20, 30, 40])  # 작을수록 보행자 많음 (약 3~10명)
    run_cmd([sys.executable, str(RANDOM_TRIPS),
             "-n", str(NET_FILE), "--pedestrians",
             "-o", str(out_file), "--seed", str(num),
             "-p", str(period), "-b", "0", "-e", "150"],
            timeout=120)

log = logging.getLogger("generator")


def run_cmd(cmd, timeout):
    """명령 실행. 실패 시 stderr 끝부분을 포함한 에러를 던진다."""
    # xml2csv.py가 sumolib을 찾을 수 있도록 SUMO tools 경로를 추가
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
    OUTPUT_DIR.mkdir(exist_ok=True)
    nums = []
    for d in OUTPUT_DIR.glob("scenario_*"):
        try:
            nums.append(int(d.name.split("_")[1]))
        except (IndexError, ValueError):
            pass
    return max(nums, default=0) + 1


def generate_one(num: int, keep_intermediate: bool) -> Path:
    scenario_dir = OUTPUT_DIR / f"scenario_{num:04d}"
    scenario_dir.mkdir(parents=True, exist_ok=True)

    if not VTYPES_FILE.exists():
        VTYPES_FILE.write_text(VTYPES_XML, encoding="utf-8")

    # 1. 시나리오별 다양화 입력 생성 (운전 성향 배합 + 랜덤 보행자)
    route_file = scenario_dir / "route_mixed.rou.xml"
    ped_file = scenario_dir / "ped_random.rou.xml"
    weights = make_mixed_route(num, route_file)
    make_random_pedestrians(num, ped_file)

    # 2. SUMO 실행 (횡단보도 있는 net_v2 + 혼합 성향 + 랜덤 보행자)
    fcd_xml = scenario_dir / "fcd.xml"
    run_cmd(
        [
            "sumo",
            "-c", str(SUMO_CONFIG),
            "-n", str(NET_FILE),
            "-r", f"{route_file},{ped_file}",
            "--additional-files", str(VTYPES_FILE),
            "--seed", str(num),
            "--fcd-output", str(fcd_xml),
            "--no-step-log", "true",
        ],
        timeout=600,
    )
    (scenario_dir / "META").write_text(
        f"generator=v2 crossings sidewalk random_peds "
        f"driver_mix(cautious/normal/aggressive)={weights}\n")
    if not fcd_xml.exists() or fcd_xml.stat().st_size == 0:
        raise RuntimeError("fcd.xml이 생성되지 않았습니다")

    # 2. fcd.xml -> fcd_trajectory.csv (SUMO 기본 도구 사용)
    trajectory_csv = scenario_dir / "fcd_trajectory.csv"
    run_cmd(
        [sys.executable, str(XML2CSV), str(fcd_xml), "-o", str(trajectory_csv)],
        timeout=300,
    )

    # 3. 전처리 -> feature.csv / pedestrian.csv
    # fcd 출력에는 차량 행과 보행자 행이 섞여 있다 (각 행은 한쪽 컬럼만 채워짐).
    # feature.csv는 기존 fcd_trajectory.csv와 같은 형식(차량 행, ';' 구분)으로
    # 만들어서 Jetson의 후속 처리 코드가 그대로 읽을 수 있게 한다.
    df = pd.read_csv(trajectory_csv, sep=";")

    veh_cols = ["timestep_time"] + [c for c in df.columns
                                    if c.startswith("vehicle_")]
    veh = df[veh_cols].dropna(subset=["vehicle_id"])
    if veh.empty:
        raise RuntimeError("차량 데이터가 비어 있습니다")
    feature_csv = scenario_dir / "feature.csv"
    veh.to_csv(feature_csv, sep=";", index=False)

    # 보행자 위치도 함께 저장 (Jetson에서 실제 보행자 좌표를 쓸 수 있도록)
    if "person_id" in df.columns:
        ped_cols = ["timestep_time"] + [c for c in df.columns
                                        if c.startswith("person_")]
        ped = df[ped_cols].dropna(subset=["person_id"])
        if not ped.empty:
            ped.to_csv(scenario_dir / "pedestrian.csv", sep=";", index=False)

    # 4. 중간 파일 정리 (디스크 절약)
    if not keep_intermediate:
        fcd_xml.unlink(missing_ok=True)
        trajectory_csv.unlink(missing_ok=True)
        route_file.unlink(missing_ok=True)
        ped_file.unlink(missing_ok=True)

    # 5. 완료 표시 (Jetson은 이 파일이 있는 폴더만 가져간다)
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
    log.info("생성기 시작 (count=%s, interval=%ss)",
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
            # 실패한 폴더는 삭제해서 같은 번호로 재시도되게 한다
            shutil.rmtree(OUTPUT_DIR / f"scenario_{num:04d}",
                          ignore_errors=True)
            # 연속 실패 시 점점 오래 대기 (최대 10분)
            time.sleep(min(60 * consecutive_failures, 600))
            continue

        time.sleep(args.interval)

    log.info("총 %d개 시나리오 생성 완료", made)


if __name__ == "__main__":
    main()
