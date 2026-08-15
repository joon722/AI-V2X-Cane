#!/usr/bin/env python3
"""SUMO → 젯슨 데이터 계약 v1 검증기.

시나리오 폴더(tracks.csv + meta.json)가 계약을 지키는지 확인한다.
- 민서(VM): 시나리오 생성 직후 실행, 통과해야 DONE 파일을 만든다.
- 현준(젯슨): process_scenarios.py가 처리 전에 실행, 불합격이면 .failed 처리.

의존성 없음(표준 라이브러리만) - VM/젯슨 어디서든 python3 하나로 돈다.

사용:
    python3 validate_csv.py <시나리오_폴더>
통과하면 종료코드 0, 위반이 하나라도 있으면 1과 함께 위반 목록 출력.
"""
import argparse
import csv
import json
import sys
from pathlib import Path

REQUIRED_COLUMNS = [
    "t_s", "obj_type", "obj_id", "x_m", "y_m",
    "lat", "lon", "speed_mps", "angle_deg",
]
OPTIONAL_COLUMNS = ["sumo_risk_label"]
OBJ_TYPES = {"vehicle", "pedestrian"}
META_REQUIRED_KEYS = ["scenario_id", "kind", "config", "model_version"]
META_KINDS = {"current", "future"}
# 한반도 범위 밖이면 좌표 변환 누락(SUMO 로컬 좌표가 그대로 온 것)으로 판정
LAT_RANGE = (33.0, 39.5)
LON_RANGE = (124.0, 132.0)


def _to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _check_meta(scenario_dir, errors):
    meta_path = scenario_dir / "meta.json"
    if not meta_path.is_file():
        errors.append("meta.json 파일이 없습니다")
        return
    try:
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as ex:
        errors.append(f"meta.json 파싱 실패: {ex}")
        return
    for key in META_REQUIRED_KEYS:
        if key not in meta:
            errors.append(f"meta.json에 {key} 항목이 없습니다")
    if meta.get("kind") not in META_KINDS and "kind" in meta:
        errors.append(
            f"meta.json kind는 current/future 중 하나여야 합니다 (값: {meta.get('kind')})"
        )


def _check_row(fields, row, line_no, errors):
    def err(msg):
        errors.append(f"tracks.csv {line_no}행: {msg}")

    if len(row) != len(fields):
        err(f"컬럼 수가 헤더와 다릅니다 (헤더 {len(fields)}개, 행 {len(row)}개)")
        return None
    values = dict(zip(fields, row))

    if _to_float(values["t_s"]) is None:
        err(f"t_s가 숫자가 아닙니다 (값: {values['t_s']})")

    obj_type = values["obj_type"]
    if obj_type not in OBJ_TYPES:
        err(f"obj_type은 vehicle/pedestrian만 허용합니다 (값: {obj_type})")

    if not values["obj_id"].strip():
        err("obj_id가 비어 있습니다")

    for col in ("x_m", "y_m"):
        if _to_float(values[col]) is None:
            err(f"{col}이 숫자가 아닙니다 (값: {values[col]})")

    lat = _to_float(values["lat"])
    lon = _to_float(values["lon"])
    if lat is None or not (LAT_RANGE[0] <= lat <= LAT_RANGE[1]):
        err(f"lat이 한반도 범위({LAT_RANGE[0]}~{LAT_RANGE[1]}) 밖입니다 "
            f"(값: {values['lat']}) - 좌표 변환 누락 의심")
    if lon is None or not (LON_RANGE[0] <= lon <= LON_RANGE[1]):
        err(f"lon이 한반도 범위({LON_RANGE[0]}~{LON_RANGE[1]}) 밖입니다 "
            f"(값: {values['lon']}) - 좌표 변환 누락 의심")

    speed = _to_float(values["speed_mps"])
    if speed is None or speed < 0:
        err(f"speed_mps는 0 이상 숫자여야 합니다 (값: {values['speed_mps']})")

    angle_raw = values["angle_deg"].strip()
    if obj_type == "vehicle":
        angle = _to_float(angle_raw)
        if angle is None or not (0.0 <= angle <= 360.0):
            err(f"vehicle의 angle_deg는 0~360 필수입니다 (값: {angle_raw!r})")
    elif angle_raw and _to_float(angle_raw) is None:
        err(f"angle_deg가 숫자가 아닙니다 (값: {angle_raw})")

    label_raw = values.get("sumo_risk_label", "").strip()
    if label_raw:
        label = _to_float(label_raw)
        if label is None or label != int(label) or not (0 <= label <= 3):
            err(f"sumo_risk_label은 0~3 정수 또는 빈칸이어야 합니다 (값: {label_raw})")
    return obj_type


def _check_tracks(scenario_dir, errors):
    tracks_path = scenario_dir / "tracks.csv"
    if not tracks_path.is_file():
        errors.append("tracks.csv 파일이 없습니다")
        return
    try:
        text = tracks_path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        errors.append("tracks.csv가 UTF-8이 아닙니다")
        return
    lines = text.splitlines()
    if not lines:
        errors.append("tracks.csv가 비어 있습니다")
        return

    header_line = lines[0]
    if ";" in header_line and "," not in header_line:
        errors.append("tracks.csv 구분자가 세미콜론(;)입니다 - 계약은 쉼표(,)")
        return
    fields = next(csv.reader([header_line]))

    for col in REQUIRED_COLUMNS:
        if col not in fields:
            errors.append(f"tracks.csv 필수 컬럼 {col}이 없습니다")
    known = set(REQUIRED_COLUMNS) | set(OPTIONAL_COLUMNS)
    for col in fields:
        if col not in known:
            errors.append(
                f"tracks.csv에 계약에 없는 컬럼 {col}이 있습니다 - 계약 버전을 올려 협의할 것"
            )
    if errors:
        return

    seen_types = set()
    for line_no, row in enumerate(csv.reader(lines[1:]), start=2):
        if not row:
            continue
        obj_type = _check_row(fields, row, line_no, errors)
        if obj_type:
            seen_types.add(obj_type)

    if "vehicle" not in seen_types:
        errors.append("tracks.csv에 vehicle 행이 하나도 없습니다")
    if "pedestrian" not in seen_types:
        errors.append("tracks.csv에 pedestrian 행이 하나도 없습니다")


def validate_scenario(scenario_dir):
    """시나리오 폴더 검증. 위반 목록(list[str]) 반환 - 비어 있으면 통과."""
    scenario_dir = Path(scenario_dir)
    errors = []
    if not scenario_dir.is_dir():
        return [f"시나리오 폴더가 없습니다: {scenario_dir}"]
    _check_meta(scenario_dir, errors)
    _check_tracks(scenario_dir, errors)
    return errors


def main():
    parser = argparse.ArgumentParser(description="SUMO → 젯슨 데이터 계약 v1 검증")
    parser.add_argument("scenario_dir", help="scenario_* 폴더 경로")
    parser.add_argument("--max-errors", type=int, default=20,
                        help="출력할 최대 위반 수 (기본 20)")
    args = parser.parse_args()

    errors = validate_scenario(args.scenario_dir)
    if not errors:
        print(f"[OK] 계약 v1 통과: {args.scenario_dir}")
        return 0
    for line in errors[: args.max_errors]:
        print(f"[FAIL] {line}")
    hidden = len(errors) - args.max_errors
    if hidden > 0:
        print(f"[FAIL] ... 외 {hidden}건")
    print(f"[FAIL] 총 {len(errors)}건 위반 - DONE 파일을 만들지 마세요")
    return 1


if __name__ == "__main__":
    sys.exit(main())
