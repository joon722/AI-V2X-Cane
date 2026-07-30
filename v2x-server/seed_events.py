"""숭실대 일대에 테스트용 샘플 이벤트를 뿌리고 집계를 갱신하는 스크립트.

⚠️ 이 데이터는 전부 가짜(데모용)입니다. 실제 측정/시뮬 결과가 아닙니다.
   진짜 데이터가 오면 이 스크립트 대신 /api/import 또는 Jetson 업로더를 씁니다.

사용법 (Cloud Shell에서):
    export API_KEY=$(gcloud secrets versions access latest --secret=api-key-jetson)
    python3 seed_events.py --url https://riskmap-api-193571596396.asia-northeast3.run.app

옵션:
    --url   서버 주소 (기본: http://localhost:8000)
"""
import argparse
import json
import os
import random
import sys
import urllib.request
import urllib.error
from datetime import datetime, timedelta, timezone

# 숭실대 일대 핫스팟 (기존 위험구역 zones와 같은 지점 + 주변 도로 몇 곳)
# risk_bias가 높을수록 위험 이벤트 비중이 높아짐
HOTSPOTS = [
    # (이름, lat, lng, 실측 건수, 시뮬 건수, risk_bias)
    ("정문 앞 횡단보도", 37.4966, 126.9573, 30, 45, 3),
    ("주차장 출구",     37.4958, 126.9562, 20, 30, 2),
    ("중문",           37.4972, 126.9558, 15, 25, 2),
    ("상도로 방면",     37.4950, 126.9581, 10, 30, 1),
    ("캠퍼스 북측",     37.4983, 126.9566,  8,  0, 1),   # 실측만 있는 구간 (live_only 시연용)
    ("후문 골목",       37.4975, 126.9590,  0, 25, 2),   # 시뮬만 있는 구간 (sumo_only 시연용)
]

def risk_to_ttc(risk):
    base = {0: 5.0, 1: 3.0, 2: 1.5, 3: 0.7}[risk]
    return round(base + random.uniform(-0.3, 0.3), 2)

def gen_events():
    events = []
    now = datetime.now(timezone.utc)
    for name, lat, lng, n_live, n_sumo, bias in HOTSPOTS:
        for src, n in (("live", n_live), ("sumo", n_sumo)):
            for i in range(n):
                risk = max(0, min(3, bias + random.choice([-1, 0, 0, 0, 1])))
                ts = now - timedelta(minutes=random.randint(0, 60 * 24 * 3))
                events.append({
                    "event_uid": f"seed-{src}-{name}-{i}",
                    "source": src,
                    "lat": round(lat + random.uniform(-0.0003, 0.0003), 6),
                    "lng": round(lng + random.uniform(-0.0003, 0.0003), 6),
                    "risk": risk,
                    "ttc": risk_to_ttc(risk),
                    "device_id": "seed-script",
                    "scenario_id": "demo-sample" if src == "sumo" else None,
                    "occurred_at": ts.isoformat(),
                })
    return events

def post(url, payload, api_key):
    req = urllib.request.Request(
        url,
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json", "x-api-key": api_key},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=30) as res:
        return json.loads(res.read().decode())

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://localhost:8000")
    args = parser.parse_args()
    base = args.url.rstrip("/")

    api_key = os.environ.get("API_KEY", "")
    if not api_key:
        print("오류: API_KEY 환경변수가 비어 있습니다. 먼저 이걸 실행하세요:")
        print('  export API_KEY=$(gcloud secrets versions access latest --secret=api-key-jetson)')
        sys.exit(1)

    events = gen_events()
    print(f"{len(events)}건 업로드 시작 -> {base}/api/events")
    ok = fail = 0
    for i, ev in enumerate(events, 1):
        try:
            post(base + "/api/events", ev, api_key)
            ok += 1
        except urllib.error.HTTPError as e:
            fail += 1
            print(f"  실패({e.code}): {ev['event_uid']} -> {e.read().decode()[:120]}")
        except Exception as e:
            fail += 1
            print(f"  실패: {ev['event_uid']} -> {e}")
        if i % 30 == 0:
            print(f"  ...{i}/{len(events)}")

    print(f"업로드 완료: 성공 {ok}건 / 실패 {fail}건")

    print("집계 갱신 중 (events -> road_segment_stats)...")
    try:
        res = post(base + "/api/admin/refresh-stats", {}, api_key)
        print(f"집계 완료: 도로 구간 {res.get('segments')}개 생성")
        print("이제 브라우저에서 지도를 새로고침 하세요!")
    except Exception as e:
        print(f"집계 실패: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
