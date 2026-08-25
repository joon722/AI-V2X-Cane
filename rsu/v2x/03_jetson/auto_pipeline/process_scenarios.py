#!/usr/bin/env python3
"""
Jetson Nano 자동 추론 파이프라인 (Jetson = AI 추론 장치)

서버(Google Cloud)에서 생성된 시나리오를 받아서:
    1. 서버에서 새 시나리오 수신 (rsync)
    2. feature.csv + pedestrian.csv -> 모델 입력 11개 feature 계산
    3. scaler 적용 + Transformer ONNX 추론  (기존 risk_inference_onnx.py 재사용)
    4. 결과 CSV를 서버(~/SUMO_project/results/)로 업로드

risk_score 공식은 step7_risk.py의 팀 공식(calculate_risk_score 등)을 그대로
import해서 사용한다. zone_base_risk는 SUMO 시뮬레이션에 구역 정보가 없으므로
학습 데이터 기본값과 같은 0을 사용한다.

사용 예:
    python process_scenarios.py --once              # 한 번만 실행
    python process_scenarios.py --once --no-sync --no-upload   # 로컬 테스트
    python process_scenarios.py                     # 무한 반복 (cron이 관리)
"""
import argparse
import logging
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd

# 기존 코드 재사용 (수정하지 않고 import만)
JETSON_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(JETSON_DIR))
from risk_inference_onnx import (  # noqa: E402
    create_sequences,
    normalize_features,
    softmax,
)
from step7_risk import calculate_risk_score, calculate_ttc  # noqa: E402
from retention import cleanup_old_scenarios, purged_exclude_args  # noqa: E402

# ---------------------------------------------------------
# 설정
# ---------------------------------------------------------
SERVER = "ssukpc347@8.230.1.67"
SERVER_KEY = str(Path.home() / ".ssh" / "risk_server_key")
SERVER_DATA_DIR = "SUMO_project/generated_data"
SERVER_RESULT_DIR = "SUMO_project/results"

INCOMING_DIR = Path.home() / "incoming_data"
RESULT_DIR = Path.home() / "inference_results"

ZONE_BASE_RISK = 0.0  # SUMO 시나리오에는 구역 정보가 없음
BATCH_SIZE = 256

# v3 모델: 3초 선행 예측 — 출력(onnx_risk_level)은 "현재 위험"이 아니라
# "향후 3초 내 도달할 최대 위험 레벨"이다 (실제 위험보다 중앙값 4초 먼저 경고).
# 입력 19개 = v2의 16개 + 물리 외삽 3개(등속 가정 3초 내 최소 거리/시점/점수).
MODEL_PATH = Path(__file__).resolve().parent / "risk_transformer_v3.onnx"
SCALER_PATH = Path(__file__).resolve().parent / "scaler_v3.json"
PREDICT_HORIZON_S = 3.0

log = logging.getLogger("pipeline")

SSH_OPTS = ["-i", SERVER_KEY, "-o", "ConnectTimeout=15",
            "-o", "StrictHostKeyChecking=accept-new"]


# ---------------------------------------------------------
# 1. 서버에서 시나리오 수신
# ---------------------------------------------------------
def sync_from_server() -> bool:
    INCOMING_DIR.mkdir(exist_ok=True)
    cmd = [
        "rsync", "-a", "-e", "ssh " + " ".join(SSH_OPTS),
        # 보존 정리로 삭제한 시나리오는 다시 받지 않는다 (include보다 먼저 와야 함)
        *purged_exclude_args(INCOMING_DIR),
        "--include=scenario_*/",
        "--include=scenario_*/feature.csv",
        "--include=scenario_*/pedestrian.csv",
        "--include=scenario_*/DONE",
        "--exclude=*",
        f"{SERVER}:{SERVER_DATA_DIR}/",
        str(INCOMING_DIR) + "/",
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    if result.returncode != 0:
        log.warning("서버 수신 실패 (오프라인?): %s",
                    (result.stderr or "").strip()[-200:])
        return False
    return True


# ---------------------------------------------------------
# 2. 모델 입력 feature 계산
# ---------------------------------------------------------
def build_features(scenario_dir: Path) -> pd.DataFrame:
    veh = pd.read_csv(scenario_dir / "feature.csv", sep=";")

    ped_csv = scenario_dir / "pedestrian.csv"
    if not ped_csv.exists():
        raise RuntimeError("pedestrian.csv가 없습니다")
    ped = pd.read_csv(ped_csv, sep=";")

    # [팀 리뷰 반영] 모든 (차량, 보행자) 쌍을 만들고, 미분(접근속도·상대속도)은
    # 반드시 "같은 보행자" 쌍 안에서만 계산한다. 최근접 보행자가 바뀔 때
    # 다른 사람과의 거리를 미분하면 가짜 접근속도 -> TTC 폭주가 생긴다.
    ped = ped[["timestep_time", "person_id",
               "person_x", "person_y", "person_speed"]]
    ped.columns = ["timestep_time", "person_id",
                   "ped_x", "ped_y", "ped_speed_mps"]

    df = veh.merge(ped, on="timestep_time", how="inner")
    if df.empty:
        raise RuntimeError("차량-보행자 시간대가 겹치지 않습니다")
    df = df.rename(columns={
        "vehicle_x": "veh_x",
        "vehicle_y": "veh_y",
        "vehicle_speed": "veh_speed_mps",
    })
    df = df.sort_values(["vehicle_id", "person_id",
                         "timestep_time"]).reset_index(drop=True)

    # 거리
    df["distance_m"] = np.sqrt(
        (df["veh_x"] - df["ped_x"]) ** 2 + (df["veh_y"] - df["ped_y"]) ** 2
    )

    # 접근 속도 — 같은 (차량, 보행자) 쌍 안에서만 미분
    pair = df.groupby(["vehicle_id", "person_id"], sort=False)
    pair_keys = [df["vehicle_id"], df["person_id"]]
    dt = pair["timestep_time"].diff()
    dd = -pair["distance_m"].diff()
    df["rel_speed_mps"] = (dd / dt).fillna(0.0)

    # TTC: 팀 공식 + 30초 클램프 + 유효 플래그 (팀 리뷰 반영)
    raw_ttc = np.array([
        calculate_ttc(d, r)
        for d, r in zip(df["distance_m"], df["rel_speed_mps"])
    ])
    df["ttc"] = np.minimum(raw_ttc, 30.0)
    df["ttc_valid"] = (df["rel_speed_mps"] > 0.1).astype(float)
    df["risk_score"] = [
        calculate_risk_score(d, r, v, t, ZONE_BASE_RISK)
        for d, r, v, t in zip(df["distance_m"], df["rel_speed_mps"],
                              df["veh_speed_mps"], df["ttc"])
    ]
    df["zone_base_risk"] = ZONE_BASE_RISK
    df["ts_ms"] = (df["timestep_time"] * 1000).astype(int)

    # 벡터 특징: 상대 위치/속도 + 최근접 예상 거리(DCPA) — 같은 쌍 안에서만 미분
    rx = df["veh_x"] - df["ped_x"]
    ry = df["veh_y"] - df["ped_y"]
    dvx = rx.groupby(pair_keys, sort=False).diff() / dt
    dvy = ry.groupby(pair_keys, sort=False).diff() / dt
    df["dx"], df["dy"] = rx, ry
    df["dvx"] = dvx.fillna(0.0)
    df["dvy"] = dvy.fillna(0.0)
    v2 = dvx ** 2 + dvy ** 2
    t_cpa = -(rx * dvx + ry * dvy) / v2.where(v2 > 1e-6)
    cpa_x = rx + dvx * t_cpa
    cpa_y = ry + dvy * t_cpa
    dcpa = np.sqrt(cpa_x ** 2 + cpa_y ** 2)
    # 멀어지는 중(t_cpa<0)이거나 속도 정보가 없으면 현재 거리를 그대로 사용
    df["dcpa_m"] = dcpa.where((t_cpa > 0) & v2.notna(),
                              df["distance_m"]).fillna(df["distance_m"])

    # 물리 외삽 특징 (v3 학습 데이터 생성 코드와 동일한 계산):
    # 등속 가정으로 3초 안에 도달할 최소 거리·시점과 그때의 채점표 점수
    vx = df["dvx"].to_numpy()
    vy = df["dvy"].to_numpy()
    rxa, rya = rx.to_numpy(), ry.to_numpy()
    v2f = vx ** 2 + vy ** 2
    with np.errstate(divide="ignore", invalid="ignore"):
        t_cpa_p = np.where(v2f > 1e-6, -(rxa * vx + rya * vy) / v2f, -1.0)
    t_hit = np.clip(t_cpa_p, 0.0, PREDICT_HORIZON_S)
    px, py = rxa + vx * t_hit, rya + vy * t_hit
    df["phys_min_dist_3s"] = np.sqrt(px ** 2 + py ** 2)
    df["phys_t_cpa"] = t_hit
    ttc_arg = np.where(t_hit > 0.05, t_hit, 0.05)
    df["phys_score_3s"] = [
        calculate_risk_score(d, r, v, t if r > 0.1 else 9999.0)
        for d, r, v, t in zip(df["phys_min_dist_3s"], df["rel_speed_mps"],
                              df["veh_speed_mps"], ttc_arg)]

    # 모든 파생값 계산이 끝난 뒤에야 (차량, 시점)별 최근접 보행자 선택
    idx = df.groupby(["vehicle_id", "timestep_time"])["distance_m"].idxmin()
    df = (df.loc[idx]
            .sort_values(["vehicle_id", "timestep_time"])
            .reset_index(drop=True))
    return df


# ---------------------------------------------------------
# 3. 차량별 시퀀스 추론
# ---------------------------------------------------------
def run_inference(session, df, feature_columns, mean, scale, seq_len):
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name

    results = []
    skipped = 0
    for vehicle_id, group in df.groupby("vehicle_id", sort=False):
        if len(group) < seq_len:
            skipped += 1
            continue
        features = group[feature_columns].to_numpy(dtype=np.float32)
        normalized = normalize_features(features, mean, scale)
        sequences = create_sequences(normalized, seq_len)

        logits_list = []
        for start in range(0, len(sequences), BATCH_SIZE):
            batch = sequences[start:start + BATCH_SIZE]
            out = session.run([output_name], {input_name: batch})
            logits_list.append(np.asarray(out[0], dtype=np.float32))
        logits = np.concatenate(logits_list, axis=0)
        probs = softmax(logits)

        result = group.iloc[seq_len - 1:].copy()
        result["onnx_risk_level"] = np.argmax(probs, axis=1).astype(np.int64)
        result["onnx_confidence"] = np.max(probs, axis=1)
        results.append(result)

    if skipped:
        log.info("데이터가 %d행 미만인 차량 %d대는 건너뜀", seq_len, skipped)
    if not results:
        raise RuntimeError("추론 가능한 차량이 없습니다")
    return pd.concat(results, ignore_index=True)


# ---------------------------------------------------------
# 4. 결과 업로드
# ---------------------------------------------------------
def upload_result(result_csv: Path) -> bool:
    mkdir = subprocess.run(
        ["ssh"] + SSH_OPTS + [SERVER, f"mkdir -p {SERVER_RESULT_DIR}"],
        capture_output=True, text=True, timeout=60)
    if mkdir.returncode != 0:
        log.warning("서버 결과 폴더 생성 실패: %s",
                    (mkdir.stderr or "").strip()[-200:])
        return False
    scp = subprocess.run(
        ["scp"] + SSH_OPTS + [str(result_csv),
                              f"{SERVER}:{SERVER_RESULT_DIR}/"],
        capture_output=True, text=True, timeout=300)
    if scp.returncode != 0:
        log.warning("결과 업로드 실패: %s", (scp.stderr or "").strip()[-200:])
        return False
    return True


# ---------------------------------------------------------
# 시나리오 하나 처리
# ---------------------------------------------------------
def process_scenario(scenario_dir, session, feature_columns, mean, scale,
                     seq_len, do_upload):
    df = build_features(scenario_dir)
    result = run_inference(session, df, feature_columns, mean, scale, seq_len)

    RESULT_DIR.mkdir(exist_ok=True)
    result_csv = RESULT_DIR / f"{scenario_dir.name}_result.csv"
    out_cols = ["ts_ms", "timestep_time", "vehicle_id"] + feature_columns + \
               ["onnx_risk_level", "onnx_confidence"]
    result[out_cols].to_csv(result_csv, index=False)

    counts = result["onnx_risk_level"].value_counts().sort_index()
    summary = ", ".join(f"L{k}:{v}" for k, v in counts.items())
    log.info("%s 추론 완료 (%d행, %s)", scenario_dir.name, len(result), summary)

    uploaded = upload_result(result_csv) if do_upload else False
    if uploaded:
        (scenario_dir / ".uploaded").touch()
    (scenario_dir / ".processed").touch()


def pending_scenarios():
    if not INCOMING_DIR.exists():
        return []
    dirs = sorted(d for d in INCOMING_DIR.glob("scenario_*") if d.is_dir())
    return [d for d in dirs
            if (d / "DONE").exists() and not (d / ".processed").exists()]


def retry_uploads():
    """추론은 끝났지만 업로드에 실패했던 결과를 재시도."""
    if not INCOMING_DIR.exists():
        return
    for d in INCOMING_DIR.glob("scenario_*"):
        if (d / ".processed").exists() and not (d / ".uploaded").exists():
            result_csv = RESULT_DIR / f"{d.name}_result.csv"
            if result_csv.exists() and upload_result(result_csv):
                (d / ".uploaded").touch()
                log.info("%s 결과 업로드 재시도 성공", d.name)


def main():
    p = argparse.ArgumentParser(description="Jetson 자동 추론 파이프라인")
    p.add_argument("--once", action="store_true", help="한 번만 실행하고 종료")
    p.add_argument("--no-sync", action="store_true", help="서버 수신 생략")
    p.add_argument("--no-upload", action="store_true", help="결과 업로드 생략")
    p.add_argument("--interval", type=float, default=60.0,
                   help="반복 실행 간격(초)")
    args = p.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        handlers=[
            logging.FileHandler(Path.home() / "pipeline.log"),
            logging.StreamHandler(),
        ],
    )

    # v2 scaler/모델 로드 (auto_pipeline 폴더의 v2 파일 사용)
    import json as _json
    import onnxruntime as ort
    scaler = _json.loads(SCALER_PATH.read_text(encoding="utf-8"))
    feature_columns = scaler["feature_columns"]
    mean = np.array(scaler["mean"], dtype=np.float32)
    scale = np.array(scaler["scale"], dtype=np.float32)
    scale = np.where(scale == 0, 1.0, scale).astype(np.float32)
    seq_len = int(scaler["sequence_length"])
    session = ort.InferenceSession(str(MODEL_PATH),
                                   providers=["CPUExecutionProvider"])
    log.info("파이프라인 시작 — v2 모델 (feature %d개, 시퀀스 길이 %d)",
             len(feature_columns), seq_len)

    while True:
        if not args.no_sync:
            sync_from_server()
        if not args.no_upload:
            retry_uploads()

        # 백로그 추론에 앞서 업로드 완료된 옛 시나리오부터 정리해 디스크를 확보한다
        cleanup_old_scenarios(INCOMING_DIR, RESULT_DIR, log=log)

        for scenario_dir in pending_scenarios():
            try:
                process_scenario(scenario_dir, session, feature_columns,
                                 mean, scale, seq_len,
                                 do_upload=not args.no_upload)
                # 전원/발열 보호: 연속 추론 사이 짧은 휴식
                # (5V 4A 어댑터 교체 후 2.0 -> 0.5초로 완화)
                time.sleep(0.5)
            except Exception:
                log.exception("%s 처리 실패", scenario_dir.name)
                # 실패 표시를 남겨 같은 시나리오를 무한 재시도하지 않게 함
                (scenario_dir / ".processed").touch()
                (scenario_dir / ".failed").touch()

        if args.once:
            break
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
