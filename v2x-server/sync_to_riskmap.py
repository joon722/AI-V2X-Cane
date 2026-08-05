#!/usr/bin/env python3
"""SUMO 시뮬 결과(results/*.csv.gz)를 위경도로 변환해 위험지도 서버에 축적 전송.

- VM에서 cron으로 주기 실행. 이미 보낸 파일은 state 파일로 기록해 건너뜀.
- risk_score >= 1 인 행만 전송 (팀 결정: risk>=1 + 중복 억제)
- 같은 (시나리오, 차량, 시각)은 event_uid로 서버가 중복 차단하므로 재실행 안전.
- VM에서 원본이 cleanup으로 삭제돼도 서버 DB에는 영구 축적됨.

사용:
    export API_KEY=...            # 또는 --api-key
    python3 sync_to_riskmap.py
"""
import argparse
import csv
import gzip
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timedelta, timezone
from pathlib import Path

BASE = Path.home() / "SUMO_project"
RESULTS_DIR = BASE / "results"
STATE_FILE = BASE / "map_data" / "sync_state.json"
DEFAULT_URL = "https://riskmap-api-193571596396.asia-northeast3.run.app"

# net_clean.net.xml의 <location> 값 (projParameter: +proj=utm +zone=52 +datum=WGS84)
NET_OFFSET_X = -315516.76
NET_OFFSET_Y = -4150401.46
UTM_ZONE_LON0 = 129.0

BATCH = 400            # 한 번에 보낼 이벤트 수
MAX_FILES_PER_RUN = 40 # 한 실행에서 처리할 파일 수 (cron 주기 내에 끝나도록)
KST = timezone(timedelta(hours=9))


def utm_to_latlng(easting, northing):
    """UTM zone 52N (WGS84) -> 위경도"""
    a = 6378137.0
    f = 1 / 298.257223563
    k0 = 0.9996
    e2 = f * (2 - f)
    ep2 = e2 / (1 - e2)
    e1 = (1 - math.sqrt(1 - e2)) / (1 + math.sqrt(1 - e2))
    lon0 = math.radians(UTM_ZONE_LON0)

    x = easting - 500000.0
    M = northing / k0
    mu = M / (a * (1 - e2/4 - 3*e2**2/64 - 5*e2**3/256))
    phi1 = (mu + (3*e1/2 - 27*e1**3/32) * math.sin(2*mu)
            + (21*e1**2/16 - 55*e1**4/32) * math.sin(4*mu)
            + (151*e1**3/96) * math.sin(6*mu)
            + (1097*e1**4/512) * math.sin(8*mu))
    s1, c1, t1 = math.sin(phi1), math.cos(phi1), math.tan(phi1)
    C1 = ep2 * c1**2
    T1 = t1**2
    N1 = a / math.sqrt(1 - e2 * s1**2)
    R1 = a * (1 - e2) / (1 - e2 * s1**2) ** 1.5
    D = x / (N1 * k0)

    lat = phi1 - (N1 * t1 / R1) * (
        D**2/2 - (5 + 3*T1 + 10*C1 - 4*C1**2 - 9*ep2) * D**4/24
        + (61 + 90*T1 + 298*C1 + 45*T1**2 - 252*ep2 - 3*C1**2) * D**6/720)
    lon = lon0 + (D - (1 + 2*T1 + C1) * D**3/6
        + (5 - 2*C1 + 28*T1 - 3*C1**2 + 8*ep2 + 24*T1**2) * D**5/120) / c1
    return math.degrees(lat), math.degrees(lon)


def sumo_to_latlng(x, y):
    return utm_to_latlng(x - NET_OFFSET_X, y - NET_OFFSET_Y)


def load_state():
    try:
        with open(STATE_FILE, encoding="utf-8") as f:
            return set(json.load(f).get("done", []))
    except Exception:
        return set()


def save_state(done):
    STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
    tmp = STATE_FILE.with_suffix(".tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump({"done": sorted(done), "updated": datetime.now(KST).isoformat()}, f)
    tmp.replace(STATE_FILE)


def post_batch(url, events, api_key, retries=3):
    """서버에 이벤트 묶음 전송. 성공 건수 반환."""
    ok = 0
    for ev in events:
        body = json.dumps(ev).encode()
        req = urllib.request.Request(
            url + "/api/events", data=body, method="POST",
            headers={"Content-Type": "application/json", "x-api-key": api_key})
        for attempt in range(retries):
            try:
                with urllib.request.urlopen(req, timeout=20) as res:
                    res.read()
                ok += 1
                break
            except urllib.error.HTTPError as e:
                if e.code == 401:
                    print("ERROR: API 키가 거부됨(401). 중단합니다.", file=sys.stderr)
                    sys.exit(1)
                if attempt == retries - 1:
                    print(f"  전송 실패({e.code}): {ev['event_uid']}", file=sys.stderr)
                time.sleep(1 + attempt)
            except Exception as e:
                if attempt == retries - 1:
                    print(f"  전송 실패: {ev['event_uid']} -> {e}", file=sys.stderr)
                time.sleep(1 + attempt)
    return ok


def process_file(path, base_dt):
    """gz 결과 파일 -> 전송용 이벤트 목록 (risk>=1만, 연속 중복 억제)"""
    scen = path.name.replace("_result.csv.gz", "")
    events = []
    last_key = None
    with gzip.open(path, "rt", encoding="utf-8-sig", newline="") as f:
        for row in csv.DictReader(f):
            # 위험 등급 기준: onnx_risk_level (0~3, AI 모델 예측)
            # build_map_data.py(민서)와 동일한 컬럼·필터를 사용해 두 지도가 어긋나지 않게 함.
            # onnx_risk_level이 없는 구버전 파일은 risk_score(0~100)를 팀 기준으로 환산.
            raw = row.get("onnx_risk_level")
            if raw not in (None, ""):
                try:
                    risk = int(float(raw))
                except ValueError:
                    continue
                risk = max(0, min(3, risk))
            else:
                try:
                    score = float(row.get("risk_score") or 0)
                except ValueError:
                    continue
                risk = 3 if score >= 70 else 2 if score >= 45 else 1 if score >= 20 else 0
            if risk < 1:
                continue

            try:
                x = float(row["veh_x"]); y = float(row["veh_y"])
                ts_ms = int(float(row["ts_ms"]))
            except (KeyError, ValueError):
                continue

            lat, lng = sumo_to_latlng(x, y)
            # 같은 차량이 같은 위치·등급으로 연속되면 1건만 (중복 억제)
            key = (row.get("vehicle_id"), round(lat, 5), round(lng, 5), risk)
            if key == last_key:
                continue
            last_key = key

            ttc = row.get("ttc")
            try:
                ttc = float(ttc)
                if ttc >= 9999:
                    ttc = None
            except (TypeError, ValueError):
                ttc = None

            dist = row.get("distance_m")
            try:
                dist = float(dist)
            except (TypeError, ValueError):
                dist = None

            events.append({
                "event_uid": f"{scen}-{row.get('vehicle_id','v')}-{ts_ms}",
                "source": "sumo",
                "scenario_id": scen,
                "device_id": "risk-server",
                "lat": round(lat, 7),
                "lng": round(lng, 7),
                "risk": risk,
                "ttc": ttc,
                "distance_m": dist,
                "occurred_at": (base_dt + timedelta(milliseconds=ts_ms)).isoformat(),
            })
    return events


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--api-key", default=os.environ.get("API_KEY", ""))
    ap.add_argument("--max-files", type=int, default=MAX_FILES_PER_RUN)
    ap.add_argument("--refresh", action="store_true", default=True,
                    help="전송 후 서버 집계 갱신 호출")
    args = ap.parse_args()

    if not args.api_key:
        print("ERROR: API_KEY가 없습니다. export API_KEY=... 또는 --api-key 사용", file=sys.stderr)
        sys.exit(1)

    url = args.url.rstrip("/")
    done = load_state()
    files = sorted(RESULTS_DIR.glob("*_result.csv.gz"))
    todo = [p for p in files if p.name not in done][:args.max_files]

    if not todo:
        print(f"[{datetime.now(KST):%Y-%m-%d %H:%M}] 새 파일 없음 (누적 {len(done)}개 완료)")
        return

    print(f"[{datetime.now(KST):%Y-%m-%d %H:%M}] 처리 대상 {len(todo)}개 "
          f"(전체 {len(files)}개 중 {len(done)}개는 이미 전송됨)")

    total_sent = 0
    for path in todo:
        # 시뮬 기준 시각: 파일 생성 시각을 시나리오 시작점으로 사용 (팀 결정: 적재일 기준)
        base_dt = datetime.fromtimestamp(path.stat().st_mtime, KST)
        try:
            events = process_file(path, base_dt)
        except Exception as e:
            print(f"  파싱 실패 {path.name}: {e}", file=sys.stderr)
            continue

        sent = 0
        for i in range(0, len(events), BATCH):
            sent += post_batch(url, events[i:i+BATCH], args.api_key)
        total_sent += sent
        done.add(path.name)
        save_state(done)
        print(f"  {path.name}: 위험 {len(events)}건 중 {sent}건 전송")

    print(f"합계 {total_sent}건 전송 완료 (누적 파일 {len(done)}개)")

    if args.refresh and total_sent:
        req = urllib.request.Request(url + "/api/admin/refresh-stats", data=b"{}",
                                     method="POST",
                                     headers={"Content-Type": "application/json",
                                              "x-api-key": args.api_key})
        try:
            with urllib.request.urlopen(req, timeout=180) as res:
                print("집계 갱신:", res.read().decode()[:120])
        except Exception as e:
            print(f"집계 갱신 실패: {e}", file=sys.stderr)


if __name__ == "__main__":
    main()
