"""validate_csv.py 단위 테스트.

SUMO → 젯슨 데이터 계약 v1을 시나리오 폴더가 지키는지 검증하는 로직을 확인한다.
계약: tracks.csv(쉼표, UTF-8) + meta.json. DONE은 생산자가 검증 통과 후 만들므로
검증 대상이 아니다.
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "auto_pipeline"))

import validate_csv  # noqa: E402

HEADER = "t_s,obj_type,obj_id,x_m,y_m,lat,lon,speed_mps,angle_deg,sumo_risk_label\n"

VEHICLE_ROW = "0.0,vehicle,car_1,3686.5,1345.5,37.4963,126.9575,3.6,41.0,0\n"
PED_ROW = "0.5,pedestrian,ped_1,3600.0,1400.0,37.4960,126.9570,1.2,,\n"


def make_scenario(tmp_path, header=HEADER, rows=(VEHICLE_ROW, PED_ROW), meta=None,
                  write_tracks=True, write_meta=True):
    scen = tmp_path / "scenario_current_20260805_1400"
    scen.mkdir()
    if write_tracks:
        (scen / "tracks.csv").write_text(header + "".join(rows), encoding="utf-8")
    if write_meta:
        if meta is None:
            meta = {"scenario_id": "current_20260805_1400", "kind": "current",
                    "config": "base", "model_version": "v1"}
        (scen / "meta.json").write_text(json.dumps(meta), encoding="utf-8")
    return scen


def test_valid_scenario_passes(tmp_path):
    scen = make_scenario(tmp_path)
    assert validate_csv.validate_scenario(scen) == []


def test_missing_tracks_csv_fails(tmp_path):
    scen = make_scenario(tmp_path, write_tracks=False)
    errors = validate_csv.validate_scenario(scen)
    assert any("tracks.csv" in e for e in errors)


def test_missing_meta_json_fails(tmp_path):
    scen = make_scenario(tmp_path, write_meta=False)
    errors = validate_csv.validate_scenario(scen)
    assert any("meta.json" in e for e in errors)


def test_semicolon_delimiter_fails(tmp_path):
    scen = make_scenario(
        tmp_path,
        header=HEADER.replace(",", ";"),
        rows=(VEHICLE_ROW.replace(",", ";"), PED_ROW.replace(",", ";")),
    )
    errors = validate_csv.validate_scenario(scen)
    assert any("세미콜론" in e for e in errors)


def test_missing_required_column_fails(tmp_path):
    header = HEADER.replace(",angle_deg", "")
    rows = ("0.0,vehicle,car_1,3686.5,1345.5,37.4963,126.9575,3.6,0\n",)
    scen = make_scenario(tmp_path, header=header, rows=rows)
    errors = validate_csv.validate_scenario(scen)
    assert any("angle_deg" in e for e in errors)


def test_unknown_extra_column_fails(tmp_path):
    header = HEADER.rstrip("\n") + ",mystery\n"
    rows = (VEHICLE_ROW.rstrip("\n") + ",1\n", PED_ROW.rstrip("\n") + ",1\n")
    scen = make_scenario(tmp_path, header=header, rows=rows)
    errors = validate_csv.validate_scenario(scen)
    assert any("mystery" in e for e in errors)


def test_sumo_risk_label_column_is_optional(tmp_path):
    header = HEADER.replace(",sumo_risk_label", "")
    rows = (
        "0.0,vehicle,car_1,3686.5,1345.5,37.4963,126.9575,3.6,41.0\n",
        "0.5,pedestrian,ped_1,3600.0,1400.0,37.4960,126.9570,1.2,\n",
    )
    scen = make_scenario(tmp_path, header=header, rows=rows)
    assert validate_csv.validate_scenario(scen) == []


def test_no_pedestrian_rows_fails(tmp_path):
    scen = make_scenario(tmp_path, rows=(VEHICLE_ROW,))
    errors = validate_csv.validate_scenario(scen)
    assert any("pedestrian" in e for e in errors)


def test_no_vehicle_rows_fails(tmp_path):
    scen = make_scenario(tmp_path, rows=(PED_ROW,))
    errors = validate_csv.validate_scenario(scen)
    assert any("vehicle" in e for e in errors)


def test_local_xy_in_latlon_fails(tmp_path):
    # 과거 실사고: SUMO 로컬 좌표(3600/1400)가 위경도 자리에 그대로 들어옴
    bad = "0.0,vehicle,car_1,3686.5,1345.5,3600.0,1400.0,3.6,41.0,0\n"
    scen = make_scenario(tmp_path, rows=(bad, PED_ROW))
    errors = validate_csv.validate_scenario(scen)
    assert any("lat" in e or "lon" in e for e in errors)


def test_vehicle_empty_angle_fails(tmp_path):
    bad = "0.0,vehicle,car_1,3686.5,1345.5,37.4963,126.9575,3.6,,0\n"
    scen = make_scenario(tmp_path, rows=(bad, PED_ROW))
    errors = validate_csv.validate_scenario(scen)
    assert any("angle_deg" in e for e in errors)


def test_negative_speed_fails(tmp_path):
    bad = "0.0,vehicle,car_1,3686.5,1345.5,37.4963,126.9575,-1.0,41.0,0\n"
    scen = make_scenario(tmp_path, rows=(bad, PED_ROW))
    errors = validate_csv.validate_scenario(scen)
    assert any("speed_mps" in e for e in errors)


def test_non_numeric_t_s_fails(tmp_path):
    bad = "abc,vehicle,car_1,3686.5,1345.5,37.4963,126.9575,3.6,41.0,0\n"
    scen = make_scenario(tmp_path, rows=(bad, PED_ROW))
    errors = validate_csv.validate_scenario(scen)
    assert any("t_s" in e for e in errors)


def test_risk_label_out_of_range_fails(tmp_path):
    bad = "0.0,vehicle,car_1,3686.5,1345.5,37.4963,126.9575,3.6,41.0,7\n"
    scen = make_scenario(tmp_path, rows=(bad, PED_ROW))
    errors = validate_csv.validate_scenario(scen)
    assert any("sumo_risk_label" in e for e in errors)


def test_bad_obj_type_fails(tmp_path):
    bad = "0.0,bicycle,b_1,3686.5,1345.5,37.4963,126.9575,3.6,41.0,0\n"
    scen = make_scenario(tmp_path, rows=(bad, PED_ROW, VEHICLE_ROW))
    errors = validate_csv.validate_scenario(scen)
    assert any("obj_type" in e for e in errors)


def test_meta_invalid_kind_fails(tmp_path):
    meta = {"scenario_id": "x", "kind": "과거", "config": "base", "model_version": "v1"}
    scen = make_scenario(tmp_path, meta=meta)
    errors = validate_csv.validate_scenario(scen)
    assert any("kind" in e for e in errors)


def test_meta_missing_key_fails(tmp_path):
    meta = {"kind": "current", "config": "base", "model_version": "v1"}
    scen = make_scenario(tmp_path, meta=meta)
    errors = validate_csv.validate_scenario(scen)
    assert any("scenario_id" in e for e in errors)


def test_error_lines_report_line_numbers(tmp_path):
    bad = "0.0,vehicle,car_1,3686.5,1345.5,37.4963,126.9575,-1.0,41.0,0\n"
    scen = make_scenario(tmp_path, rows=(VEHICLE_ROW, bad, PED_ROW))
    errors = validate_csv.validate_scenario(scen)
    # 헤더가 1행이므로 문제 행은 3행
    assert any("3" in e and "speed_mps" in e for e in errors)
