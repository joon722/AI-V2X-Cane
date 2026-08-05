#!/usr/bin/env python3
"""
위험지도용 데이터 추출 (서버에서 실행)

Jetson이 업로드한 추론 결과(가상, results/)와 실측 데이터(선택)를
격자 단위로 집계해서 지도(Leaflet)에 바로 올릴 수 있는 JSON/CSV를 만든다.

- SUMO x/y 좌표를 실제 위경도(WGS84)로 변환 (net_clean.net.xml의 UTM 투영 사용)
- 가상 데이터: 가중치 1, 실측 데이터: 가중치 3 (기본값, 옵션으로 조정)
- 격자별 위험 점수 = Σ(risk_level × 데이터 출처 가중치)

사용:
    python3 build_map_data.py                       # 가상 데이터만
    python3 build_map_data.py --real-csv 실측.csv   # 실측 혼합 (lat,lon,risk_level 컬럼 필요)

출력: map_data/risk_map_data.json, risk_map_data.csv
cron이 30분마다 자동 갱신한다.
"""
import argparse
import glob
import json
import sys
import time
from pathlib import Path

import pandas as pd

sys.path.insert(0, "/usr/share/sumo/tools")
import sumolib  # noqa: E402

BASE = Path.home() / "SUMO_project"
NET_FILE = BASE / "net_clean.net.xml"
RESULTS_DIR = BASE / "results"
OUT_DIR = BASE / "map_data"

GRID_M = 10.0          # 격자 크기 (m)
VIRTUAL_WEIGHT = 1.0   # 가상(SUMO) 데이터 가중치
REAL_WEIGHT = 3.0      # 실측 데이터 가중치


def load_virtual() -> pd.DataFrame:
    """Jetson 추론 결과에서 위험 이벤트(레벨 1 이상)만 모은다."""
    files = sorted(glob.glob(str(RESULTS_DIR / "*_result.csv"))) + \
            sorted(glob.glob(str(RESULTS_DIR / "*_result.csv.gz")))
    frames = []
    for f in files:
        df = pd.read_csv(f, usecols=["veh_x", "veh_y", "ttc",
                                     "onnx_risk_level"])
        df = df[df["onnx_risk_level"] >= 1]
        if not df.empty:
            frames.append(df)
    if not frames:
        return pd.DataFrame(), len(files)
    out = pd.concat(frames, ignore_index=True)
    out = out.rename(columns={"onnx_risk_level": "risk_level"})
    out["weight"] = VIRTUAL_WEIGHT
    out["source"] = "virtual"
    return out, len(files)


def load_real(path: str) -> pd.DataFrame:
    """실측 데이터(lat, lon, risk_level 컬럼)를 읽어 SUMO 좌표로 변환한다."""
    net = sumolib.net.readNet(str(NET_FILE))
    df = pd.read_csv(path)
    need = {"lat", "lon", "risk_level"}
    if not need.issubset(df.columns):
        raise SystemExit(f"실측 CSV에는 {need} 컬럼이 필요합니다. "
                         f"현재: {list(df.columns)}")
    xy = [net.convertLonLat2XY(lo, la)
          for lo, la in zip(df["lon"], df["lat"])]
    df["veh_x"] = [p[0] for p in xy]
    df["veh_y"] = [p[1] for p in xy]
    if "ttc" not in df.columns:
        df["ttc"] = float("nan")
    df = df[df["risk_level"] >= 1][["veh_x", "veh_y", "ttc", "risk_level"]]
    df["weight"] = REAL_WEIGHT
    df["source"] = "real"
    return df


def main():
    p = argparse.ArgumentParser(description="위험지도 데이터 추출")
    p.add_argument("--real-csv", help="실측 데이터 CSV (lat,lon,risk_level)")
    p.add_argument("--real-weight", type=float, default=REAL_WEIGHT)
    args = p.parse_args()

    virtual, n_files = load_virtual()
    parts = [virtual] if len(virtual) else []
    if args.real_csv:
        real = load_real(args.real_csv)
        real["weight"] = args.real_weight
        parts.append(real)
    if not parts:
        raise SystemExit("집계할 위험 이벤트가 없습니다")
    df = pd.concat(parts, ignore_index=True)

    # 격자 집계 (10m 단위)
    df["gx"] = (df["veh_x"] // GRID_M).astype(int)
    df["gy"] = (df["veh_y"] // GRID_M).astype(int)
    grid = df.groupby(["gx", "gy"]).agg(
        risk_score=("risk_level",
                    lambda s: float((s * df.loc[s.index, "weight"]).sum())),
        event_count=("risk_level", "size"),
        max_level=("risk_level", "max"),
        avg_ttc=("ttc", "mean"),
        real_count=("source", lambda s: int((s == "real").sum())),
    ).reset_index()

    # 격자 중심 좌표를 위경도로 변환
    net = sumolib.net.readNet(str(NET_FILE))
    center_x = (grid["gx"] + 0.5) * GRID_M
    center_y = (grid["gy"] + 0.5) * GRID_M
    lonlat = [net.convertXY2LonLat(x, y)
              for x, y in zip(center_x, center_y)]
    grid["lon"] = [round(p[0], 6) for p in lonlat]
    grid["lat"] = [round(p[1], 6) for p in lonlat]

    # 점수 정규화 (0~100)
    top = grid["risk_score"].max()
    grid["risk_score_norm"] = (grid["risk_score"] / top * 100).round(1)
    grid["avg_ttc"] = grid["avg_ttc"].round(2)
    grid = grid.sort_values("risk_score", ascending=False)

    OUT_DIR.mkdir(exist_ok=True)
    cols = ["lat", "lon", "risk_score_norm", "event_count",
            "max_level", "avg_ttc", "real_count"]
    grid[cols].to_csv(OUT_DIR / "risk_map_data.csv", index=False)
    payload = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "source_files": n_files,
        "grid_size_m": GRID_M,
        "virtual_weight": VIRTUAL_WEIGHT,
        "real_weight": args.real_weight if args.real_csv else None,
        "cells": grid[cols].to_dict(orient="records"),
    }
    (OUT_DIR / "risk_map_data.json").write_text(
        json.dumps(payload, ensure_ascii=False), encoding="utf-8")

    print(f"격자 {len(grid)}개 생성 (이벤트 {len(df)}건, "
          f"결과 파일 {n_files}개, 실측 {int(grid['real_count'].sum())}건)")
    print("상위 위험 격자 5개:")
    print(grid[cols].head(5).to_string(index=False))


if __name__ == "__main__":
    main()
