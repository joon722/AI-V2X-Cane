[← 프로젝트 개요](../README.ko.md)

# v2x-server — 위험지도 웹 서버

현장에서 수집된 위험 이벤트를 누적 저장하고 지도 위에 시각화하는 웹 서버입니다. FastAPI + PostgreSQL(Cloud SQL) + Leaflet 구성이며, Dockerfile로 Cloud Run에 배포할 수 있습니다.

## 파일 구성

| 파일 | 역할 |
| --- | --- |
| `main.py` | FastAPI 앱 — API 키 인증(HMAC 비교), 이벤트 수집, 집계 API |
| `roads.py` / `roads.json` | 이벤트 좌표를 가장 가까운 실제 도로 엣지에 스냅 (300m 초과 시 제외해 지도 오염 방지) |
| `schema.sql` / `seed.sql` | DB 스키마와 초기 데이터 |
| `seed_events.py` | 테스트 이벤트 주입 스크립트 |
| `static/index.html` | Leaflet 기반 위험지도 페이지 |
| `static/sim.html` / `sim_data.json` | 시뮬레이션 데이터 재생 페이지 |
| `Dockerfile` | Cloud Run 배포용 컨테이너 |

## 환경 변수

`INSTANCE_CONNECTION_NAME`(Cloud SQL), `DB_NAME`, `DB_USER`, `DB_PASS`, `API_KEY`

## 현재 상태

서버 자체는 구현·검증 완료. Jetson(`lux/`) 쪽에서 위험 이벤트를 업로드하는 클라이언트 연결은 로드맵 항목입니다.
