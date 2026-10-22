[← 프로젝트 개요](../README.ko.md)

# v2x-server — 위험지도 웹 서버

현장에서 수집된 위험 이벤트를 누적 저장하고 지도 위에 시각화하는 웹 서버입니다. FastAPI + PostgreSQL(Cloud SQL) + Leaflet 구성이며, Dockerfile로 Cloud Run에 배포할 수 있습니다.

## 파일 구성

| 파일 | 역할 |
| --- | --- |
| `main.py` | FastAPI 앱 — API 키 인증(HMAC 비교), 이벤트 수집(`/api/events`·`/api/import`), 지도 집계(`/api/segments`·`/api/status`), 실시간 상태(`/api/live`·`/api/live/push`·`/api/live/latest`) |
| `roads.py` / `roads.json` | 이벤트 좌표를 가장 가까운 실제 도로 엣지에 스냅 (50m 초과 시 제외해 지도 오염 방지) |
| `schema.sql` / `seed.sql` | DB 스키마와 초기 데이터 |
| `seed_events.py` | 테스트 이벤트 주입 스크립트 |
| `static/index.html` | Leaflet 기반 위험지도 페이지 — 현장 실측 / AI 예측(시뮬) 탭, 도로별 위험 우선순위 TOP5 |
| `static/drive.html` | 차량 1인칭 실시간 화면 — `/api/live/latest`를 폴링해 주변 보행자와 위험도 표시 |
| `static/sim.html` / `sim_data.json` | 시뮬레이션 데이터 재생 페이지 |
| `sync_to_riskmap.py` | SUMO 시뮬 결과를 위경도로 바꿔 `/api/events`로 누적 전송 (VM cron용) |
| `Dockerfile` | Cloud Run 배포용 컨테이너 |

## 환경 변수

`INSTANCE_CONNECTION_NAME`(Cloud SQL), `DB_NAME`, `DB_USER`, `DB_PASS`, `API_KEY`

## 현재 상태

Cloud Run에 배포해 운영 중입니다. 젯슨이 `rsu/v2x/03_jetson/deploy/`의 `upload_events.py`로 위험 이벤트(레벨 1 이상)를 `/api/events`에, `stream_live.py`로 실시간 상태를 `/api/live`에 올립니다. 위험지도의 현장 실측 탭과 `drive.html`이 이 데이터를 씁니다.
