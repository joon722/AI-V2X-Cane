#!/usr/bin/env python3
"""Turn step 6 kinematics into a risk score and a 0-3 risk level.

The scoring table is the team's, copied verbatim (see below). The only thing
step 7 adds on top is a DCPA suppression gate: the team table has no term for
closest-approach distance, so a car grazing past the kerb and a car on a
head-on course score the same. Step 6 computes dcpa precisely to tell those
apart; this gate is where that distinction is finally used.

step2-6 are not modified; this file imports the pipeline and scores the
filtered (Kalman) track, which is the one step 7 was always meant to consume.
"""

import argparse
import csv
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from step3_parse_v2x import SOURCE_MODES, normalize_record, serial_lines
from step4_state_store import FRESH_WINDOW_S, StateStore, has_position
from step5_test_vehicle import SPEED_MPS, START_DISTANCE_M, TestVehicle
from step6_kinematics import GPS_SIGMA_M, KinematicsPipeline, to_float


# --- Team scoring table -----------------------------------------------------
# Verbatim copy of tmp/AI-V2X-Cane-audit/scripts/risk_calculator.py. The audit
# folder is throwaway, so importing from it is fragile; the numbers are frozen
# here instead and mirrored by test_step7_risk.py. If the team retunes the
# table, re-sync these three functions and nothing else.

def calculate_ttc(distance_m, relative_speed_mps):
    """TTC in seconds. A non-approaching pair returns a large sentinel."""
    if relative_speed_mps <= 0:
        return 9999.0
    return distance_m / relative_speed_mps


def calculate_risk_score(
    distance_m,
    relative_speed_mps,
    vehicle_speed_mps,
    ttc,
    zone_base_risk=0,
):
    """0-100 risk from distance, TTC, relative speed, vehicle speed and zone."""
    score = 0.0

    # 1. Distance: up to 30
    if distance_m <= 10:
        score += 30
    elif distance_m <= 20:
        score += 25
    elif distance_m <= 40:
        score += 18
    elif distance_m <= 60:
        score += 10
    elif distance_m <= 100:
        score += 5

    # 2. TTC: up to 35
    if ttc <= 1:
        score += 35
    elif ttc <= 2:
        score += 30
    elif ttc <= 3:
        score += 25
    elif ttc <= 5:
        score += 20
    elif ttc <= 8:
        score += 15
    elif ttc <= 12:
        score += 8

    # 3. Relative speed: up to 20
    if relative_speed_mps >= 20:
        score += 20
    elif relative_speed_mps >= 15:
        score += 17
    elif relative_speed_mps >= 10:
        score += 13
    elif relative_speed_mps >= 5:
        score += 8
    elif relative_speed_mps > 0:
        score += 3

    # 4. Vehicle speed: up to 10
    if vehicle_speed_mps >= 20:
        score += 10
    elif vehicle_speed_mps >= 15:
        score += 8
    elif vehicle_speed_mps >= 10:
        score += 6
    elif vehicle_speed_mps >= 5:
        score += 3

    # 5. Hazard-zone correction: up to 5
    score += min(max(zone_base_risk, 0), 5)

    return round(min(score, 100), 2)


def classify_risk_level(risk_score):
    """0 safe, 1 caution, 2 warning, 3 danger."""
    if risk_score >= 70:
        return 3
    if risk_score >= 45:
        return 2
    if risk_score >= 20:
        return 1
    return 0

# --- End team scoring table -------------------------------------------------


# DCPA gate margins, tied to the GPS noise the filtered position carries. A dcpa
# inside 1 sigma could well be a real hit given measurement error, so it is not
# suppressed; only a miss clearly outside the noise (>3 sigma) is pulled down,
# and never to zero, because dcpa is itself an estimate that can be wrong.
DCPA_NEAR_M = GPS_SIGMA_M          # 2.5 m, ~1 sigma: no suppression inside this
DCPA_FAR_M = 3.0 * GPS_SIGMA_M     # 7.5 m, ~3 sigma: full suppression beyond
DCPA_FLOOR = 0.2                   # a clear miss still keeps a fifth of the score

# 안전 하한. TTC가 이 값 아래면 점수표도 DCPA 게이트도 무시하고 최고 레벨을 낸다.
# 근거는 ttc_study/SPEC.md에 있고, 항의 합으로 나온 값이다.
#
# GPS 주기만 상수로 뺀 이유는 그것만 하드웨어에 달려 있고 나머지는 사람과 전파의
# 값이기 때문이다.
#
#   2026-08-12 실측으로 5 Hz 확정: 펌웨어 UDP 텔레메트리의 GPS채택 카운터가
#   지팡이 전 세션 +5.0/초, 차량도 전원이 안정된 21:11 이후 +4.2~5.0/초.
#   8/8의 "1 Hz" 판정은 차량 재부팅 루프(UBX 설정 ACK 유실)와 측정 도구의
#   1초 샘플링·좌표 양자화가 만든 착시였다.
#
# 전제: 차량 전원이 안정적일 것. 재부팅이 반복되면 그 부팅은 5Hz 설정이
# 유실되어 1 Hz로 남고, 그때 필요한 하한은 2.8초다. 전원 문제가 재발하면
# 이 줄을 1.0으로 되돌린다.
GPS_PERIOD_S = 0.2
TRANSPORT_S = 0.1      # 2026-08-05 실측. 위험도 변경 29건 전수에서
                       # 젯슨 계산 1.0 ms + 무선 왕복 중앙값 102 ms.
PERCEPTION_S = 0.3     # 촉각 단순반응 0.12~0.18 s의 1.5~2배 (선택 반응).
STOPPING_S = 1.2       # 보행 급정지 실험 0.84~1.21 s의 보수적인 끝.
MARGIN_S = 0.2

# round: 부동소수점 합이 2.8000000000000003이 되어 도움말과 로그에 그대로 새어 나온다.
T_FLOOR_TTC_S = round(
    GPS_PERIOD_S + TRANSPORT_S + PERCEPTION_S + STOPPING_S + MARGIN_S, 3
)

# 게이트까지 무시하는 이유: 게이트는 '스쳐 갈 차'를 걸러내는 장치인데 그 판단
# 자체가 추정이고, 접촉까지 하한도 남지 않았을 때 추정이 틀리면 대가가 너무 크다.
#
# 하한 값별 손익 (시나리오 1200 x 3회, gps_sigma 2.5, 평가 기준 2.8초):
#
#   하한   적시경보   오경보   확실안전 경보   검출
#   없음    52.2%     2.14%      1.64%      87.3/97
#   2.0s    52.2%     3.36%      2.76%      95.3/97
#   2.8s    57.7%     4.75%      4.07%      96.3/97
#
# 위 표는 GPS 1 Hz 시절(하한 2.8초 운용) 분석이다. 당시 2.0초는 GPS 주기
# 때문에 적시성에 기여하지 못해 2.8초를 썼고, 그 대가로 인도 오경보가
# 2.76% -> 4.07%로 늘었다 (ROADSIDE_ALARM.md 2절).
#
# 2026-08-12 5 Hz 확정으로 하한이 2.0초로 돌아왔다. 원래 설계값이고,
# 인도 오경보도 함께 내려간다.


def dcpa_gate(dcpa, near_m=DCPA_NEAR_M, far_m=DCPA_FAR_M, floor=DCPA_FLOOR):
    """Multiplier in [floor, 1] that fades risk out as the miss distance grows.

    dcpa is None when there is no closest approach ahead (receding or static);
    there is nothing to suppress, so the gate is transparent.
    """
    if dcpa is None or dcpa <= near_m:
        return 1.0
    if dcpa >= far_m:
        return floor
    frac = (dcpa - near_m) / (far_m - near_m)
    return 1.0 + frac * (floor - 1.0)


# 경보 상한. TTC가 이보다 크면 상충이 아니라 "그냥 근처에 있다"이므로 규칙
# 레벨을 0으로 누른다. 점수는 기록용으로 그대로 남는다.
#
# 2026-08-12 실측: 경보 1004건 중 접근 안 함(TTC 9999) 50.4% + 30초 초과
# 23.6% = 74%가 위험이 아닌데 울렸다. 점수표의 거리 항(10 m 이내 30점)만으로
# 레벨 1 기준(20점)을 넘기 때문이다. 반대로 실제 접근의 TTC는 3.7~9.1초로
# 전부 30초 안 - 이 상한은 진짜 위험을 하나도 걸지 않는다.
#
# 30초는 model_features.TTC_MAX_S, upload_events.MAX_TTC_S와 같은 값이다.
# 판단에 의미가 있는 TTC 구간이 거기까지라는 하나의 결정을 셋이 공유한다.
#
# 모델(model_gate)은 이 상한의 영향을 받지 않고 레벨을 올릴 수 있다. TTC가
# 무한대인 측면 위험을 잡는 것이 모델의 존재 이유라서다.
T_ALARM_MAX_TTC_S = 30.0


@dataclass(frozen=True)
class RiskAssessment:
    distance_m: float
    closing_los: float
    vehicle_speed_mps: float
    ttc: float
    dcpa: float | None
    base_score: float
    gate: float
    final_score: float
    risk_level: int
    reason: str = "table"  # "table" | "safety_floor" | "ttc_capped"


def assess_risk(
    kinematics,
    vehicle_speed_mps,
    zone_base_risk=0,
    near_m=DCPA_NEAR_M,
    far_m=DCPA_FAR_M,
    floor=DCPA_FLOOR,
    floor_ttc_s=T_FLOOR_TTC_S,
    alarm_max_ttc_s=T_ALARM_MAX_TTC_S,
):
    """Score one filtered-track Kinematics sample.

    `closing_los` is the line-of-sight closing speed, which is what the team
    table means by relative speed. TTC is recomputed through the team's own
    function so the score stays exactly consistent with that table.

    TTC가 floor_ttc_s 아래면 점수와 게이트를 모두 건너뛰고 레벨 3을 낸다
    (reason="safety_floor"). TTC가 alarm_max_ttc_s를 넘으면 규칙 레벨을
    0으로 누른다(reason="ttc_capped"). 분석용으로 끄려면 각각 0을 준다.
    """
    closing = kinematics.closing_los
    ttc = calculate_ttc(kinematics.distance_m, closing)
    base = calculate_risk_score(
        kinematics.distance_m, closing, vehicle_speed_mps, ttc, zone_base_risk
    )
    gate = dcpa_gate(kinematics.dcpa, near_m, far_m, floor)
    final = round(base * gate, 2)

    # 접근하지 않는 쌍은 calculate_ttc가 9999를 주므로 하한에 걸리지 않는다.
    breached = floor_ttc_s > 0 and ttc <= floor_ttc_s
    capped = not breached and alarm_max_ttc_s > 0 and ttc > alarm_max_ttc_s
    if breached:
        level, reason = 3, "safety_floor"
    elif capped:
        level, reason = 0, "ttc_capped"
    else:
        level, reason = classify_risk_level(final), "table"
    return RiskAssessment(
        distance_m=kinematics.distance_m,
        closing_los=closing,
        vehicle_speed_mps=vehicle_speed_mps,
        ttc=ttc,
        dcpa=kinematics.dcpa,
        base_score=base,
        gate=round(gate, 3),
        final_score=final,
        risk_level=level,
        reason=reason,
    )


CSV_FIELDS = (
    "pc_time",
    "cane_seq",
    "vehicle_seq",
    "distance_m",
    "closing_los",
    "ttc",
    "vehicle_speed",
    "dcpa",
    "base_score",
    "gate",
    "final_score",
    "risk_level",
)


def fmt(value, digits=2):
    return "-" if value is None else f"{value:.{digits}f}"


def format_risk(assessment):
    return (
        f"[RISK] score={assessment.final_score:.2f} level={assessment.risk_level} "
        f"(base={assessment.base_score:.2f} "
        f"dcpa={fmt(assessment.dcpa)}m gate={assessment.gate:.2f})"
    )


def csv_row(now, store, assessment):
    return {
        "pc_time": round(now, 3),
        "cane_seq": store.latest["cane"]["seq"],
        "vehicle_seq": store.latest["vehicle"]["seq"],
        "distance_m": round(assessment.distance_m, 3),
        "closing_los": round(assessment.closing_los, 3),
        "ttc": round(assessment.ttc, 3),
        "vehicle_speed": round(assessment.vehicle_speed_mps, 3),
        "dcpa": None if assessment.dcpa is None else round(assessment.dcpa, 3),
        "base_score": assessment.base_score,
        "gate": assessment.gate,
        "final_score": assessment.final_score,
        "risk_level": assessment.risk_level,
    }


def append_row(csv_path, row):
    path = Path(csv_path)
    needs_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        if needs_header:
            writer.writeheader()
        writer.writerow(row)


def process_line(raw_line, source_mode, pipeline, csv_path, gate_params):
    line = raw_line.strip()
    if not line:
        return False
    try:
        payload = json.loads(line)
        if not isinstance(payload, dict):
            raise ValueError("top-level JSON must be an object")
        row = normalize_record(payload, source_mode)
    except (json.JSONDecodeError, ValueError) as exc:
        print(f"[WARN] parse_failed error={exc} source={source_mode}", file=sys.stderr)
        return False

    if not pipeline.store.update(row):
        print(f"[WARN] ignored_type type={row['type']!r}", file=sys.stderr)
        return False

    pipeline.observe(row)
    result = pipeline.compute()
    if result is None:
        return True

    now, _raw, filtered = result
    vehicle_speed = to_float(pipeline.store.latest["vehicle"]["speed_mps"])
    assessment = assess_risk(filtered, vehicle_speed, **gate_params)
    print(format_risk(assessment), flush=True)
    append_row(csv_path, csv_row(now, pipeline.store, assessment))
    return True


def inject_vehicle(vehicle, pipeline, source_mode, csv_path, gate_params, now):
    cane = pipeline.store.latest.get("cane")
    if cane is None or not has_position(cane) or not vehicle.is_due(now):
        return False
    payload, distance_m = vehicle.record(
        to_float(cane["lat"]), to_float(cane["lng"]), now
    )
    print(f"[TESTVEH] seq={payload['seq']} distance_m={distance_m:.1f}", flush=True)
    return process_line(json.dumps(payload), source_mode, pipeline, csv_path, gate_params)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Score the cane/vehicle pair into a risk level (0-3)."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial port")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate")
    parser.add_argument(
        "--source-mode",
        choices=SOURCE_MODES,
        default="test",
        help="origin of this input stream (default: test)",
    )
    parser.add_argument("--csv", default="step7_risk_log.csv", help="output CSV path")
    parser.add_argument(
        "--fresh-window-ms",
        type=int,
        default=int(FRESH_WINDOW_S * 1000),
        help="how recent a record must be to count as READY (default: 500)",
    )
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="read JSON lines from stdin instead of a serial port",
    )
    parser.add_argument(
        "--test-vehicle",
        action="store_true",
        help="inject a simulated vehicle closing in on the cane (step 5)",
    )
    parser.add_argument("--vehicle-speed", type=float, default=SPEED_MPS)
    parser.add_argument("--vehicle-start-m", type=float, default=START_DISTANCE_M)
    parser.add_argument(
        "--dcpa-near-m",
        type=float,
        default=DCPA_NEAR_M,
        help=f"dcpa below this is never suppressed (default: {DCPA_NEAR_M})",
    )
    parser.add_argument(
        "--dcpa-far-m",
        type=float,
        default=DCPA_FAR_M,
        help=f"dcpa above this is fully suppressed to the floor (default: {DCPA_FAR_M})",
    )
    parser.add_argument(
        "--dcpa-floor",
        type=float,
        default=DCPA_FLOOR,
        help=f"score multiplier for a clear miss (default: {DCPA_FLOOR})",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    store = StateStore(fresh_window_s=args.fresh_window_ms / 1000)
    pipeline = KinematicsPipeline(store)
    gate_params = {
        "near_m": args.dcpa_near_m,
        "far_m": args.dcpa_far_m,
        "floor": args.dcpa_floor,
    }
    print(
        f"[INFO] source_mode={args.source_mode} csv={args.csv} "
        f"dcpa_near_m={args.dcpa_near_m} dcpa_far_m={args.dcpa_far_m} "
        f"dcpa_floor={args.dcpa_floor}",
        file=sys.stderr,
    )

    vehicle = None
    if args.test_vehicle:
        vehicle = TestVehicle(
            start_distance_m=args.vehicle_start_m, speed_mps=args.vehicle_speed
        )
        print(
            f"[INFO] test_vehicle start_m={args.vehicle_start_m} "
            f"speed={args.vehicle_speed}",
            file=sys.stderr,
        )

    lines = sys.stdin if args.stdin else serial_lines(args.port, args.baud)
    try:
        for line in lines:
            process_line(line, args.source_mode, pipeline, args.csv, gate_params)
            if vehicle is not None:
                inject_vehicle(
                    vehicle, pipeline, args.source_mode, args.csv, gate_params, time.time()
                )
    except KeyboardInterrupt:
        print("\n[INFO] stopped", file=sys.stderr)


if __name__ == "__main__":
    main()
