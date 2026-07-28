# 통합 규격 합의서 (강현준 ↔ 최민서 ↔ 박중선)

> 목적: 세 사람이 각자 만든 부분(통신/Jetson · AI/SUMO/위험도 · 하드웨어)을 실제로 붙이려면
> 서로 값·규격·좌표계가 맞아야 한다. 지금은 "인터페이스는 있는데 값이 안 맞는" 지점이 여러 개다.
> 이 문서는 그 지점을 전부 모아 **결정 → 문서화**하기 위한 합의서다. 각 항목의 `[ ]`를 팀이 채운다.
>
> 작성 근거: goag 리포 전체(lux/arduino/AI_Model/scripts/dataset/docs) + `Documents/han/map`(위험지도 웹앱)
> + `github/AI-V2X-Cane`(원격 사본) 전수 조사(2026-07-24).

## 담당 (문서 기준)

| 팀원 | 담당 |
|---|---|
| 강현준 | 통신·시스템 통합. ESP-NOW 수신, Jetson UART 수신, 실시간 rule 파이프라인(lux/), 융합·전송 |
| 최민서 | AI·데이터. SUMO 시뮬레이션, zone 정의, risk 라벨링, Transformer 학습·ONNX, 위험지도 시각화 |
| 박중선 | 하드웨어. 송/수신 H/W, 전원(Jetson 배터리), 센서, 진동·음성 액추에이터 |

---

## ⚠️ 조사에서 드러난 가장 시급한 결론 3가지

1. **좌표계가 3종이다** (실시간 위경도 37.0/127.0 · SUMO 로컬 3600/1400 · 위험지도 상도동 37.49/126.95). 통일 안 하면 zone·AI·지도가 전부 어긋난다. → Part 2.
2. **risk_level 0~3의 정의가 두 벌이다** (점수표 컷오프 vs 직접 임계값). 같은 상황을 다르게 라벨링한다. → Part 3.
3. **프로젝트 사본이 최소 4개고 서로 갈라졌다.** 펌웨어 `V2X_VERSION`이 사본마다 1/2로 달라 무선 호환이 깨진다. → Part 1.

---

# Part 1. 저장소·사본 정본화 (선행 — 이거부터)

조사 중 발견한 프로젝트 사본:

| 위치 | 성격 | 상태 |
|---|---|---|
| `OneDrive/바탕 화면/goag` | 현재 작업 중인 사본 | 원격 `joon722/AI-V2X-Cane`의 체크아웃. ~~구버전(V2X_VERSION 1)~~ **2026-07-24 origin/main으로 fast-forward 완료(V2X_VERSION 2). 이제 원격과 동일**, 갈라짐 아니었음(behind 8/ahead 0) |
| `Documents/github/AI-V2X-Cane` | 같은 원격의 다른 체크아웃 | **펌웨어 최신(V2X_VERSION 2), lux 리팩터 포함, 진동패턴 구현됨** |
| `Documents/han/arduino` | 또 다른 전체 사본 + arduino-cli 빌드환경 | 구버전 lux(4파일), 09 파일명도 다름 |
| `Documents/han/lux` | 초기 프로토타입(v2x_realtime_risk_engine 단일파일) | 7/8 시점, 별도 .git |
| `Documents/han/map` | 위험지도 웹앱 | **어느 리포에도 커밋 안 됨** (버전관리 밖) |

**핵심 위험:** goag(V2X_VERSION 1)와 github(V2X_VERSION 2)의 무선 구조체 버전이 다르다.
`06`/`09` 펌웨어는 `message.version != V2X_VERSION`이면 패킷을 버리므로, 팀원이 서로 다른 사본에서
플래시하면 **차량↔지팡이 통신이 아예 안 된다.** 또한 goag 09.ino에는 없는 단계별 진동/부저 패턴이
github 09.ino에는 있다(goag가 뒤처짐).

### 결정할 것
- [ ] **정본 리포 하나 확정** (권장: `github/AI-V2X-Cane` = 원격 최신). goag는 stale 체크아웃으로 폐기 or `git pull`로 동기화
- [ ] 실시간 파이프라인 작업을 어느 사본에서 할지 (goag를 계속 쓸 거면 먼저 원격과 동기화)
- [ ] **모든 하드웨어를 동일 `V2X_VERSION`으로 재플래시** (V2=최신으로 통일)
- [ ] `han/map`(위험지도)을 정본 리포에 편입할지, 별도 리포로 둘지
- [ ] `han/arduino`, `han/lux` 등 구버전 사본 정리/보관 방침

담당: 강현준(리포) · 박중선(재플래시 확인)

---

# Part 2. 좌표계 통일 (최우선 기술 합의)

세 좌표계가 공존한다:

| 파이프라인 | 좌표 | 원점/범위 | 근거 |
|---|---|---|---|
| 실시간 lux | GPS 위경도 → ENU 평면 | 현재 **가짜 고정 37.0/127.0**, 원점=첫 cane 좌표 | `lux/kinematics.py`, `SOURCE_MODE.md` |
| SUMO/AI/zone | SUMO 로컬 미터 | x≈3600, y≈1400 | `scaler.json`(mean 3600/1400), `zones/zone_definition.csv`(center 3602~3872) |
| 위험지도 map | GPS 위경도(WGS84) | **상도동 37.4997/126.9516** + OSM 실도로 스냅 | `han/map/app/config.py` |

**문제:** 실시간은 위경도인데 AI 모델·zone은 SUMO 좌표로 학습/정의됐고(위경도 넣으면 정규화·zone 판정
전멸), 지도는 또 다른 위경도 원점(상도동)을 쓴다.

**실마리:** `han/map/import_sumo_results.py`가 명시 — SUMO x/y만 있으면 `sumolib.net.convertXY2LonLat()`로
위경도 변환 후 업로드. 즉 **SUMO net.xml이 상도동 기반으로 지오레퍼런싱돼 있으면 SUMO↔위경도 변환이 가능**하다.
그러면 실시간·SUMO·지도를 전부 위경도로 통일할 수 있다.

### 결정할 것 (최민서에게 확인)
- [ ] SUMO net.xml이 실제 지역(상도동?)에 지오레퍼런싱돼 있나? `netconvert` 시 `--proj`/OSM 사용했나?
- [ ] `convertXY2LonLat()`로 SUMO(x,y) → (lat,lng) 변환식(원점·투영 파라미터) 확보 가능한가?
- [ ] **AI 모델 입력 좌표계 최종 결정**: (a) SUMO 로컬 유지(실시간이 위경도→SUMO 역변환) / (b) 위경도로 재학습
- [ ] `zone_definition.csv`를 위경도로 재발행할지, 실시간이 위경도→SUMO 변환해서 판정할지
- [ ] 실외 실측 GPS의 실제 지역이 상도동인가? (지도 중심과 맞춰야 함)

담당: 최민서(SUMO 좌표계) · 강현준(실시간 변환 어댑터)

---

# Part 3. risk_level 0~3 정의 단일화

두 정의가 공존한다:

**A. 점수표 방식** (`scripts/risk_calculator.py` → `lux/risk_scoring.py`가 복사):
0~100점 합산 후 컷오프.
- 거리(≤10:30, ≤20:25, ≤40:18, ≤60:10, ≤100:5) + TTC(≤1:35, ≤2:30, ≤3:25, ≤5:20, ≤8:15, ≤12:8)
  + 상대속도(최대20) + 차량속도(최대10) + zone(최대5)
- **level: ≥70→3, ≥45→2, ≥20→1, else 0**

**B. 직접 임계값 방식** (최민서 라벨 규칙 / `lux/risk_engine.py`의 rule):
- `dist<3 또는 ttc<1.5 → 3`, `<6/<3 → 2`, `<10/<5 → 1`, else 0

→ 같은 상황이 A와 B에서 다른 level이 된다. Transformer는 A(risk_score 기반 label)로 학습됐다.

### 결정할 것
- [ ] **단일 정의 채택**: A(점수표) or B(임계값). (Transformer가 A로 학습됐으니 A 권장)
- [ ] 채택 시 나머지 하나는 코드에서 제거/정렬
- [ ] `risk_calculator.py`(원본)와 `lux/risk_scoring.py`(복사본) 중 **단일 소스** 지정 (drift 방지)

담당: 최민서(라벨 정의 소유) · 강현준(실시간 반영)

---

# Part 4. AI 모델 (Transformer/ONNX) 입출력 규격

**실제 학습된 모델** (`AI_Model/transformer/train_transformer.py`, `models/scaler.json`, `export_onnx.py`):
- 입력 텐서: `[batch, 10, 11]`, dtype float32, input 이름 `input`
- **feature 11개 (순서 고정):** `ped_x, ped_y, veh_x, veh_y, ped_speed_mps, veh_speed_mps, distance_m, rel_speed_mps, ttc, risk_score, zone_base_risk`
- 정규화: **StandardScaler(z-score) 필수** (scaler.json에 feature별 mean/scale)
- 출력: `[batch, 4]` **logits** (softmax 없음), argmax = risk_level 0~3, **현재 시점 분류(미래예측 아님)**
- window 10프레임 슬라이딩

**강현준 `lux/predict_risk.py`의 현재 가정 (불일치):**
- feature **8개**(ttc·risk_score·zone_base_risk 누락), **정규화 없음**, 주석엔 "미래 예측"

**함정:** 입력 11개 중 뒤 3개(ttc, risk_score, zone_base_risk)는 원시 센서값이 아니라
**rule 파이프라인 산출물**이다. 즉 AI를 돌리려면 실시간 쪽이 rule(TTC·score·zone)을 먼저 계산해
feature로 넣어야 한다. (AI가 rule을 대체하는 게 아니라 rule 출력을 먹는 구조)

### 결정할 것 (최민서에게 확인 + 파일 확보)
- [ ] feature 11개 이름·순서·단위 최종 확정 (위 목록 맞나?)
- [ ] `scaler.json` mean/scale 값이 최종본인가? (좌표계 결정 Part 2에 종속 — SUMO 좌표 정규화라 위경도면 무효)
- [ ] window 10프레임의 **샘플 주기(Hz)** 확정 (10Hz=1초? 코드에 미명시)
- [ ] 출력이 "현재 분류"인 게 맞나? 미래 예측이 필요하면 재학습 필요
- [ ] **확보물:** `risk_transformer.onnx` + feature 추출 스크립트(정규화 포함) + 샘플 입력/출력 1쌍(회귀 테스트 고정용)
- [ ] `lux/predict_risk.py`를 11 feature·StandardScaler·현재분류로 수정

담당: 최민서(모델·규격) · 강현준(추론 슬롯 수정)

---

# Part 5. zone base_risk → risk_level 매핑

**⚠️ zone csv가 리포에 3개 있고 서로 다르다 (정본 미정):**
| 파일 | 좌표(center) | 스키마 | 내용 |
|---|---|---|---|
| `zones/zone_definition.csv` | SUMO 로컬 3602~3872 | speed_limit **있음** | Z01~Z04, base_risk 2~5 |
| `lux/zone_definition.csv` | 작은 값 10/5 | speed_limit **없음** | parking_exit 등, base_risk 2~ |
| `zones/zoneszone_definition.csv` | — | 상이 | **헤더만, 데이터 0행 (오타 파일)** |

- base_risk 값 범위: **0~5** (zones/ 기준 Z01=3 정문, Z02=4 중문/교차로, Z03=2 학생회관, Z04=5 주차장 출구)
- 좌표: SUMO 로컬 (Part 2에 종속), radius 30m

융합 시 base_risk(0~5)를 risk_level(0~3)로 바꿔야 한다. 잠정안:

| base_risk | risk_level | 근거 |
|---|---|---|
| 5 (주차장 출구) | 3 | 시야차단 + 상시 차량 진출 |
| 4 (중문/교차로) | 2 | 건물 사각 |
| 3 (정문) | 1 | 보행 밀집 |
| 2 (학생회관) | 1 | 보행 밀집 |
| 0 (OUT) | 0 | 구역 밖 |

**이중계산 주의:** 점수표(A)는 zone을 최대 5점 가점하는데, 융합은 zone을 level로 max한다.
둘을 동시에 하면 zone이 두 번 반영된다 → 융합 러너는 `assess_risk(zone_base_risk=0)`로 호출.

### 결정할 것
- [ ] **zone csv 정본 하나 지정** (3종 중) + 좌표계는 Part 2와 통일
- [ ] 위 매핑표 확정 (최민서 검토)
- [ ] zone 가점(점수표) vs zone level(융합) 중 **하나만** 쓰기로 확정

담당: 최민서(zone 정의) · 강현준(융합 반영)

---

# Part 6. 통신 wire 계약 (강현준 ↔ 박중선/펌웨어)

## 6-1. 무선(ESP-NOW) 구조체 — 좋은 소식
차량(06)과 지팡이(09)의 바이너리 struct `v2x_status_message_t`는 **바이트 단위로 동일**하다
(magic `0x56325831`, version, msg_type, node_type, risk_level, gps_valid, node_id, lat/lng(float),
speed_mps, heading_deg, timestamp_ms, seq_num). → 9단계 실차 교체 시 스키마 걱정 없음.
- [ ] **단, `V2X_VERSION`을 전 하드웨어 통일** (Part 1: 현재 1/2 혼재)

## 6-2. RSU 브리지 → Jetson JSON 스키마 (3종 갈림)
| 브리지 | 스키마 | Jetson(lux) 가정과 |
|---|---|---|
| **파일 10 (멀티)** | `type,node_id,seq,gps_valid,lat,lng,speed_mps,heading_deg,node_risk,tx_ms,rx_ms,recv_count,lost_count,rssi,src_mac` | ✅ 일치 |
| 파일 08 (단일) | `node_risk` 없음, **cane drop** | ⚠️ |
| 파일 05 (UART) | `speed`/`heading`/`lost`로 키 이름 다름 | ❌ |

- [ ] **표준 브리지 = 파일 10 확정** (vehicle/cane 동일 키 + node_risk + cane 지원)

## 6-3. Jetson → RSU 다운링크 형식 (키 불일치)
- lux `send_risk.py`가 보냄: `{"target_id":0, "risk":N}`
- 파일 10이 받음: 브로드캐스트는 **`{"risk":N}` (target_id 키 생략!)**, 지정은 `target_id`
- 파일 08이 받음: 지정 키가 **`node_id`** (target_id 아님)
- **⚠️ 함정:** 파일 10에서 `{"target_id":0}`은 브로드캐스트가 아니라 **드롭**된다(node_id 0 단말 없음).

- [ ] lux 다운링크를 파일 10 규격에 맞추기: 브로드캐스트 시 target_id 키 **생략**
- [ ] RSU ack 타입(`risk_tx`, `risk_broadcast_to_seen`, `risk_drop`) Jetson 파서가 무시하는지 확인

담당: 강현준(Jetson 송신) · 박중선(브리지 펌웨어)

---

# Part 7. 위험지도 업로드 계약 (강현준 ↔ 최민서)

`Documents/han/map` = FastAPI+SQLite+Leaflet 위험지도(누적 통계). 제안서 "기능 7"이 실제 구현돼 있음.
**단, 지금 lux 파이프라인은 여기로 이벤트를 업로드하지 않는다** (RSU로만 보냄).

**업로드 계약 `POST /api/events`:**
```json
{ "lat": 37.4997, "lng": 126.9516, "risk": 3, "ttc": 0.8, "timestamp": "2026-07-04T10:00:00" }
```
- 좌표 = **GPS 위경도(상도동)**, risk 0~3, ttc·timestamp 선택
- 집계 조회: `GET /api/risk-segments` → 도로 구간별 `{points, grade, events, avg, ttc}`

### 결정할 것
- [ ] lux 파이프라인에 **위험 이벤트 발생 시 `POST /api/events` 호출부 추가**
- [ ] 지도 서버 주소 합의 (localhost / 팀 공용 / cloudflared 터널)
- [ ] 업로드 좌표는 상도동 위경도여야 함 → 실시간 cane의 37.0/127.0 가짜값 문제(Part 2와 연동)
- [ ] 업로드 트리거 정책 (매 이벤트? risk≥1만? 중복 억제?)
- [ ] map을 정본 리포에 편입 (Part 1)

담당: 최민서(지도 서버) · 강현준(업로드 클라이언트)

---

# Part 8. 하드웨어 출력 (박중선 ↔ 팀)

## 8-1. 진동/부저 단계 패턴
- **최신 펌웨어(github 09.ino, V2X_VERSION 2)엔 이미 구현됨:** CAUTION(0.5s진동/1.5s정지),
  WARNING(0.4/0.4), DANGER(0.15s 빠른진동+0.1s 부저). → goag 구버전엔 없음(사본 동기화 필요).
- [ ] 최종 패턴 값 확정 + 전 하드웨어에 반영 확인

## 8-2. 액추에이터 핀 충돌
- `02_receiver`(UART): 진동 GPIO26 / 부저 GPIO25
- `03_sound`(UDP): 진동 GPIO25 / 부저 GPIO27
- [ ] 최종 배선 하나로 통일 (데이터 흐름도 UART vs Wi-Fi UDP 정리)

## 8-3. DFPlayer 음성
- 미팅 노트: "0001~0004.mp3 분기 성공"인데 `코드_검증_정리.md`: "저장소에 재생코드 없음, 부저만"
- [ ] 실제 음성 재생 동작 상태 확인, 트랙-risk 매핑 확정

## 8-4. IMU 가속도 전달
- 차량·지팡이 다 IMU를 읽지만 무선 struct에 필드가 없어 **Jetson 미도달** (로컬 로그만)
- [ ] 낙상감지 등에 쓸 거면 struct에 ax/ay/az 추가할지 결정

담당: 박중선(하드웨어) · 강현준(Jetson 수신 시 파싱)

---

# Part 9. 위험도 스택·기타 정리 (강현준 내부 + 팀)

## 9-1. 위험도 스택 3개 공존 → 1개로
lux 안에 연결 안 된 3갈래: **Stack A**(parse→...→send_risk, 실전송) · **Stack B**(jetson_rsu_bridge→risk_engine, zone+rule+AI-placeholder max, x/y 가정) · **Stack C**(predict_risk, 미연결).
- [ ] Stack A를 기준으로 zone·AI 합류(융합 플랜 B1~B4), Stack B는 참고용으로 정리

## 9-2. TTC "미접근" 센티넬 통일
- `9999`(risk_calculator/risk_scoring) / `None`(kinematics) / `rel_speed≤0.2`(risk_engine) 제각각
- 수식 자체(`거리/상대속도`)는 동일 → 상수만 통일하면 됨
- [ ] 미접근 표현 하나로 통일

## 9-3. 제안서 vs 실제 구현 범위 (발표 표기)
- 제안서 핵심기능 중 현재 미구현: 차량 LCD HMI, 역방향 LED 경고, V2I 신호등 연장
- 구현됨(위치만 밖): 클라우드 위험지도(han/map)
- [ ] 발표에서 "제안 기능 중 구현 범위" 합의·표기

---

## 부록: 근거 파일

- 실시간: `lux/{parse_v2x,state_store,kinematics,risk_scoring,send_risk,risk_engine,predict_risk}.py`
- AI/SUMO: `AI_Model/transformer/{train_transformer,export_onnx}.py`, `models/scaler.json`, `scripts/{risk_calculator,zone_detector,event_classifier}.py`, `zones/zone_definition.csv`
- 펌웨어: `arduino/{05,06,08,09,10}_*/*.ino` (goag=V2X_VERSION 1, github=2)
- 위험지도: `Documents/han/map/{app/,README.md,import_sumo_results.py}`
- 사본: `OneDrive/바탕 화면/goag`, `Documents/github/AI-V2X-Cane`, `Documents/han/{arduino,lux,map}`

## 완료 기준

이 문서의 모든 `[ ]`가 채워지고, `risk_transformer.onnx` + 샘플 입출력 1쌍이 확보되며,
좌표계·risk_level·wire 스키마가 단일 값으로 확정되면 융합 통합(플랜 B1~B4)을 값이 맞는 상태로 진행할 수 있다.
