#!/usr/bin/env python3
"""차량 GPS 상대 바이어스 측정 → vehicle_bias.json.

지팡이·차량 수신기를 같은 자리에 두어도 원시 좌표는 몇 m 떨어져 보인다
(2026-08-17 실측: 동 +6.98 m, 북 −1.35 m). 노드별 평활로는 안 보이는 두 수신기
사이의 계통 오프셋이라, 상대 기하(거리·CPA·TTC) 전에 차량 좌표에서 빼야 한다.
빼는 쪽은 step 6(KinematicsPipeline.vehicle_bias)이고, 값을 재는 쪽이 이 파일이다.

현장 절차
  1. 두 안테나를 나란히(30 cm 이내) 두고 8초 이상 가만히 둔다.
  2. python3 calibrate_bias.py          # 엔진이 쓰는 최신 logs/raw_*.log 를 읽는다
  3. sudo systemctl restart v2x-risk-engine
  4. 나란히 둔 채 scripts/watch_risk.py 거리 ≈ 0~2 m 확인.
바이어스는 시간에 따라 변하므로 20~30분 지나거나 장소가 바뀌면 다시 잰다.

엔진 코드는 건드리지 않는다. 읽는 것: step 8 RawLog 가 줄마다 flush 하는 raw 로그
(`epoch RX {json}`). 쓰는 것: vehicle_bias.json (logs/ 는 root 소유라 코드 폴더).
"""

import argparse
import json
import math
import re
import sys
import time
from pathlib import Path

from step6_kinematics import LocalFrame, to_float


HERE = Path(__file__).resolve().parent
DEFAULT_OUT = HERE / "vehicle_bias.json"
DEFAULT_LOG_DIR = HERE / "logs"
WINDOW_S = 8.0
MIN_SAMPLES = 8
# 창 안에서 평균 주변 RMS 퍼짐이 이보다 크면 노드가 움직였거나 GPS 가 튄 것.
MAX_SPREAD_M = 3.0
# 데이터 마지막 줄이 지금보다 이만큼 오래됐으면 엔진이 수신 중이 아닐 가능성.
STALE_WARN_S = 10.0
# 같은 자리의 두 수신기가 이보다 멀게 나오면 나란히 두지 않았을 가능성이 크다
# (지금까지 실측 6~7 m). 스크립트는 안테나가 실제로 붙어 있었는지 알 수 없다.
LARGE_GAP_WARN_M = 10.0

_RX_LINE = re.compile(r"^(\d+(?:\.\d+)?) RX (\{.*\})\s*$")


class CalibrationError(Exception):
    """측정을 신뢰할 수 없어 저장하지 않는 경우."""


def newest_raw_log(log_dir):
    """가장 최근에 *쓰인* raw 로그. 파일명의 시각은 부팅 직후 시계가 틀릴 수 있다."""
    logs = sorted(Path(log_dir).glob("raw_*.log"), key=lambda p: p.stat().st_mtime)
    if not logs:
        raise CalibrationError(f"raw 로그 없음: {log_dir}/raw_*.log")
    return logs[-1]


def read_fixes(lines):
    """(t, type, lat, lng) for cane/vehicle RX lines that carry a valid fix."""
    fixes = []
    for line in lines:
        match = _RX_LINE.match(line)
        if not match:
            continue
        try:
            payload = json.loads(match.group(2))
        except json.JSONDecodeError:
            continue
        if not isinstance(payload, dict) or payload.get("type") not in ("cane", "vehicle"):
            continue
        if to_float(payload.get("gps_valid"), 1.0) < 0.5:
            continue
        lat = to_float(payload.get("lat"))
        lng = to_float(payload.get("lng"))
        if lat == 0.0 and lng == 0.0:
            continue
        fixes.append((float(match.group(1)), payload["type"], lat, lng))
    return fixes


def _mean(points):
    count = float(len(points))
    return sum(p[0] for p in points) / count, sum(p[1] for p in points) / count


def _spread(points, mean):
    """RMS distance from the mean: how still the node held during the window."""
    total = sum((e - mean[0]) ** 2 + (n - mean[1]) ** 2 for e, n in points)
    return math.sqrt(total / len(points))


def estimate_bias(fixes, window_s=WINDOW_S, min_samples=MIN_SAMPLES,
                  max_spread_m=MAX_SPREAD_M):
    """mean(vehicle − cane) in ENU metres over the last `window_s` of fixes.

    Both nodes are assumed still and co-located for the window, so each side's
    fixes are averaged separately and the difference of the means is the
    bias; no per-sample pairing is needed.
    """
    if not fixes:
        raise CalibrationError("지팡이/차량 유효 fix 가 한 줄도 없음")
    t_end = max(fix[0] for fix in fixes)
    t_start = t_end - window_s
    recent = [fix for fix in fixes if fix[0] >= t_start]
    cane = [(lat, lng) for _, type_, lat, lng in recent if type_ == "cane"]
    veh = [(lat, lng) for _, type_, lat, lng in recent if type_ == "vehicle"]
    if len(cane) < min_samples or len(veh) < min_samples:
        raise CalibrationError(
            f"표본 부족: 최근 {window_s:.0f}s 에 지팡이 {len(cane)}·차량 {len(veh)} "
            f"(각 {min_samples} 이상 필요). 두 노드가 켜져 있고 엔진이 수신 중인지 확인"
        )
    frame = LocalFrame(*cane[0])
    cane_enu = [frame.to_enu(lat, lng) for lat, lng in cane]
    veh_enu = [frame.to_enu(lat, lng) for lat, lng in veh]
    cane_mean = _mean(cane_enu)
    veh_mean = _mean(veh_enu)
    cane_spread = _spread(cane_enu, cane_mean)
    veh_spread = _spread(veh_enu, veh_mean)
    if cane_spread > max_spread_m or veh_spread > max_spread_m:
        raise CalibrationError(
            f"퍼짐 과다: 지팡이 {cane_spread:.2f} m·차량 {veh_spread:.2f} m "
            f"(허용 {max_spread_m:.1f}). 노드가 움직였거나 GPS 가 불안정 — 가만히 두고 다시"
        )
    return {
        "bias_east_m": round(veh_mean[0] - cane_mean[0], 3),
        "bias_north_m": round(veh_mean[1] - cane_mean[1], 3),
        "n_cane": len(cane),
        "n_vehicle": len(veh),
        "spread_cane_m": round(cane_spread, 3),
        "spread_vehicle_m": round(veh_spread, 3),
        "window_s": window_s,
        "t_start": round(t_start, 3),
        "t_end": round(t_end, 3),
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description="나란히 둔 지팡이·차량 GPS 의 상대 바이어스를 raw 로그에서 재어 저장한다."
    )
    parser.add_argument("--raw-log", help="읽을 raw 로그 (기본: --log-dir 의 최신 raw_*.log)")
    parser.add_argument("--log-dir", default=str(DEFAULT_LOG_DIR))
    parser.add_argument("--seconds", type=float, default=WINDOW_S, help="평균 낼 최근 구간(초)")
    parser.add_argument("--min-samples", type=int, default=MIN_SAMPLES)
    parser.add_argument("--max-spread-m", type=float, default=MAX_SPREAD_M)
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    parser.add_argument("--dry-run", action="store_true", help="계산만 하고 저장하지 않음")
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        raw_log = Path(args.raw_log) if args.raw_log else newest_raw_log(args.log_dir)
        with raw_log.open(encoding="utf-8", errors="replace") as handle:
            fixes = read_fixes(handle)
        result = estimate_bias(fixes, args.seconds, args.min_samples, args.max_spread_m)
    except (CalibrationError, OSError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    now = time.time()
    raw_gap = math.hypot(result["bias_east_m"], result["bias_north_m"])
    print(f"[CAL] raw_log={raw_log}")
    print(
        f"[CAL] bias_east={result['bias_east_m']:+.2f} m "
        f"bias_north={result['bias_north_m']:+.2f} m "
        f"(보정 전 나란히 거리 ≈ {raw_gap:.2f} m)"
    )
    print(
        f"[CAL] 표본 지팡이 {result['n_cane']}·차량 {result['n_vehicle']}, "
        f"퍼짐 {result['spread_cane_m']:.2f}·{result['spread_vehicle_m']:.2f} m, "
        f"창 {result['window_s']:.0f}s"
    )
    if raw_gap > LARGE_GAP_WARN_M:
        print(
            f"[WARN] 나란히 거리 {raw_gap:.1f} m 는 너무 큼 — 측정 구간에 두 안테나가 "
            f"실제로 붙어 있었는지 확인. 아니었다면 이 값을 쓰지 말 것",
            file=sys.stderr,
        )
    age = now - result["t_end"]
    if age > STALE_WARN_S:
        print(
            f"[WARN] 마지막 데이터가 {age:.0f}s 전 것 — 엔진이 지금 수신 중인지 확인 "
            f"(raw 로그가 갱신되는지)",
            file=sys.stderr,
        )
    if args.dry_run:
        print("[CAL] dry-run: 저장하지 않음")
        return 0

    result["created_at"] = round(now, 3)
    result["raw_log"] = str(raw_log)
    out = Path(args.out)
    tmp = out.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    tmp.replace(out)  # 엔진이 반쯤 쓰인 파일을 읽는 일이 없게
    print(f"[CAL] 저장 {out}")
    print("[CAL] 다음: sudo systemctl restart v2x-risk-engine → watch_risk 거리 ≈ 0~2 m 확인")
    return 0


if __name__ == "__main__":
    sys.exit(main())
