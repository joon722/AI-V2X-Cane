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
- [x] `risk_calculator.py`(원본)와 `lux/risk_scoring.py`(복사본) 중 **단일 소스** 지정 (drift 방지)
      → **정본 = `scripts/risk_calculator.py`.** 2026-07-28 기준 두 파일의 세 함수는 숫자까지
        동일함을 확인했다. lux 쪽 복사본은 유지한다(Jetson에는 `lux/`만 배포돼 `scripts/`가
        같이 가지 않으므로 import 하면 깨진다). 대신 **복사본이 조용히 갈라지는 것**만 막았다:
        `test_risk_scoring.TeamTableDriftTest`가 `scripts/risk_calculator.py`를 파일 경로로
        직접 로드해 9개 케이스 + 컷오프 경계에서 두 구현의 출력이 같은지 검증한다.
        (팀 표를 1점만 바꿔도 실패하는 것을 실제로 확인했다. `scripts/`가 없는 환경에서는 skip)
        → **팀이 표를 바꾸면 이 테스트가 깨지는 것이 재동기화 신호다.**

담당: 최민서(라벨 정의 소유) · 강현준(실시간 반영)

---

# Part 4. AI 모델 (Transformer/ONNX) 입출력 규격

**규격은 2026-07-29 학습 산출물에서 직접 읽어 확정했다. 최민서 답변을 기다릴 필요가 없었다** —
`risk_transformer.onnx`와 `scaler.json`이 **이미 리포에 있다**(`AI_Model/transformer/models/`).

| 항목 | 확정값 | 근거 |
|---|---|---|
| 입력 텐서 | `[batch, 10, 11]` float32, 이름 `input` | `export_onnx.py` |
| feature 11개 순서 | `ped_x, ped_y, veh_x, veh_y, ped_speed_mps, veh_speed_mps, distance_m, rel_speed_mps, ttc, risk_score, zone_base_risk` | `scaler.json: feature_columns` |
| 정규화 | StandardScaler z-score `(x-mean)/scale` | `train_transformer.py:179` |
| 출력 | `[batch, 4]` **logits**(softmax 없음), argmax = level | `export_onnx.py` |
| 시점 | **현재 시점 분류.** 미래 예측 아님 | 분류기가 `x[:, -1, :]`, 라벨도 `labels[end_index]`(윈도우 마지막 프레임) |
| window | 10프레임 | `scaler.json: sequence_length` |
| **ttc 미접근** | **9999** | 학습 데이터 12,621행 중 **7,802행(61.8%)이 정확히 9999** |

- [x] feature 11개 이름·순서·단위 확정
- [x] 출력이 "현재 분류"임을 코드로 확인
- [x] ttc 미접근 표현 확정(9999) — 팀에 물을 필요 없어졌다
- [x] **확보물 3종 중 2종은 이미 리포에 있음** (`risk_transformer.onnx`, `scaler.json`)
- [x] `lux/predict_risk.py`를 11 feature·StandardScaler·현재분류로 수정
      → `Scaler` 클래스(z-score) 추가, `DEFAULT_FEATURE_ORDER` 11개로 교체,
        `NO_APPROACH_TTC=9999`로 결측 ttc 채움, docstring의 "미래 예측" 오기 정정.
        `TrainedModelContractTest`가 `scaler.json`을 직접 읽어 feature 순서·window가
        어긋나면 실패한다(Part 3의 drift 테스트와 같은 방식).
- [ ] window 10프레임의 **샘플 주기(Hz)** — 여전히 미확정 (SUMO step length 확인 필요)
- [ ] 샘플 입력/출력 1쌍 (회귀 테스트 고정용) — onnxruntime 설치 후 직접 생성 가능

**함정 1:** 입력 11개 중 뒤 3개(ttc, risk_score, zone_base_risk)는 원시 센서값이 아니라
**rule 파이프라인 산출물**이다. 즉 AI를 돌리려면 실시간 쪽이 rule(TTC·score·zone)을 먼저 계산해
feature로 넣어야 한다. (AI가 rule을 대체하는 게 아니라 rule 출력을 먹는 구조)

**함정 2 (실제로 있던 버그):** 결측 ttc를 0.0으로 채우면 모델은 "TTC 0초 = 즉시 충돌"로 읽는다.
학습이 9999로 채웠으므로 정반대 해석이 된다. `predict_risk`에서 수정 완료.

---

## 🔴 Part 4-1. 학습 데이터의 구조적 한계 (2026-07-29 발견 — 최민서 확인 필요)

`dataset/labeled_master_dataset.csv`(12,621행)를 집계한 결과, **모델이 실시간 입력을 그대로
받을 수 없는 상태**임이 드러났다. 규격이 맞아도 값이 맞지 않는다.

| feature | 학습 데이터 실제 분포 | 의미 |
|---|---|---|
| `ped_x` | **12,621행 전부 `3600`** (고유값 1개) | 보행자가 한 지점에 고정 |
| `ped_y` | **12,621행 전부 `1400`** (고유값 1개) | 〃 |
| `ped_speed_mps` | **12,621행 전부 `1.2`** (고유값 1개) | 보행자 속도도 상수 |
| `zone_base_risk` | `0`이 12,367행(98.0%), `3`이 176, `4`가 78 | zone 안 표본이 2% |
| `risk_level` | 0: 10,796 / 1: 1,523 / 2: 272 / **3: 30행(0.24%)** | 최고 위험 표본 30건 |

**결론 1 — 모델은 "보행자가 움직이는 상황"을 한 번도 본 적이 없다.**
보행자 관련 3개 feature가 상수라 학습 시 그 축에서 아무 정보도 얻지 못했다. 실제로
`scaler.json`의 해당 `scale`이 전부 `1.0`인데, 이는 sklearn이 **표준편차 0을 1.0으로 대체**한
결과다(`_handle_zeros_in_scale`).

**결론 2 — 그래서 정규화가 보행자 좌표를 눌러주지 못한다.**
`scale=1.0`이므로 정규화 후 값이 `(ped_x - 3600)`으로 **그대로 남는다.** 실시간 ENU 좌표를
넣으면 보행자가 원점 근처(0)이므로 `-3600`이 되고, 나머지 feature는 z-score라 대개 `±3` 범위다.
**한 축만 1000배 큰 입력**이 들어가 예측이 무의미해진다. 좌표계를 SUMO로 변환해도, 보행자가
정확히 (3600, 1400) 근처가 아니면 같은 문제가 난다.

→ **Part 2(좌표계)를 어떻게 합의하든 이 문제는 안 풀린다.** 재학습이 필요한 사안이다.

### 최민서에게 물을 것
- [ ] 보행자가 고정된 시나리오로만 학습한 게 의도인가? (SUMO에서 보행자를 안 움직인 건지,
      아니면 궤적이 있는데 feature 추출에서 빠진 건지)
- [ ] **움직이는 보행자 데이터로 재학습이 가능한가?** 아니면 보행자 3개 feature를 아예 빼고
      차량 중심 8개 feature로 재학습하는 게 맞는가 (상수 feature는 어차피 정보가 0)
- [ ] `risk_level=3`이 30건뿐인데 이 클래스 성능이 신뢰할 만한가? (test set에는 몇 건 들어갔나)
- [ ] window 10프레임이 몇 Hz인가 (SUMO step-length)

**그때까지 실시간 쪽 조치:** `predict_risk`는 규격을 맞춰 두되, 융합 러너에서 **AI는 기본 OFF**
(`--model` 미지정 → `NullPredictor`)로 둔다. rule+zone만으로 안전 기능은 완결된다.

담당: 최민서(모델·재학습) · 강현준(추론 슬롯 — 완료)

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

## 6-3. Jetson → RSU 다운링크 형식 (키 불일치) — **lux 쪽 수정 완료(2026-07-28)**
- lux `send_risk.py`가 보냈던 것: `{"target_id":0, "risk":N}`
- 파일 10이 받음: 브로드캐스트는 **`{"risk":N}` (target_id 키 생략!)**, 지정은 `target_id`
- 파일 08이 받음: 지정 키가 **`node_id`** (target_id 아님)
- **⚠️ 함정:** 파일 10에서 `{"target_id":0}`은 브로드캐스트가 아니라 **드롭**된다(node_id 0 단말 없음).

펌웨어 코드로 확인한 근거 (`10_..._bridge.ino` `handleJetsonLine`):

```c
int targetRaw = extractIntAfterKey(line, "target_id", -1);  // 키 없으면 -1
if (targetRaw >= 0) { ... 못 찾으면 risk_drop ... return; } // 0 도 "지정"으로 간다
for (...) sendRiskToDevice(...);                            // 키가 없을 때만 브로드캐스트
```

**키를 생략하면 파일 08에서도 브로드캐스트가 된다**(08은 `node_id`를 보므로 target_id 유무와
무관하게 키 없음 → 전체 전송). 즉 키 없는 형태가 두 브리지 모두에서 브로드캐스트로 동작하는
유일한 형식이라, 6-2의 브리지 표준이 확정되기 전에도 안전하게 바꿀 수 있다.

- [x] lux 다운링크를 파일 10 규격에 맞추기: 브로드캐스트 시 target_id 키 **생략**
      → `send_risk.RiskTransmitter.command()` 수정 + `BROADCAST_TARGET_ID` 상수,
        테스트 2개 추가(`test_send_risk.CommandFormatTest`)
- [x] RSU ack 타입(`risk_tx`, `risk_broadcast_to_seen`, `risk_drop`) Jetson 파서가 무시하는지 확인
      → `send_risk.RSU_ACK_TYPES`가 앞의 둘을 조용히 소비. `risk_drop`은 미등록이라
        `[WARN] ignored_type`으로 찍힌다. 수정 후에는 발생하지 않아야 하므로 **일부러 남겨둠**
        (드롭이 다시 보이면 경고로 드러나는 게 낫다)

### ⚠️ 실기 기록과 펌웨어 코드가 모순된다 (박중선 확인 필요)

`lux/v2x_session_summary_next_steps.md`의 8단계 실기 기록에는 `{"target_id":0,"risk":N}`을
보냈을 때 RSU가 **`risk_broadcast_to_seen`으로 응답했고 지팡이 node_risk도 따라왔다**고
적혀 있다. 그런데 위 코드대로면 그 입력은 `risk_drop`이 나와야 한다. 응답 타입 문자열
(`risk_tx` / `risk_broadcast_to_seen`)은 파일 10에만 있으므로 08을 쓴 것도 아니다.

가능성: **RSU에 실제로 플래시된 브리지 펌웨어가 리포의 파일 10과 다른 버전이다.**
Part 1의 사본 갈라짐과 같은 뿌리일 수 있다.

- [ ] RSU 보드에 올라간 브리지 펌웨어의 실제 버전 확인
- [ ] 이번 수정본으로 재검증 (여전히 `risk_broadcast_to_seen`이 나오면 정상)

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

### 실제로 돌려본 결과 (2026-07-28, 임시 DB로 격리 실행)

서버 자체는 **정상 동작한다.** 실 `risk.db`는 건드리지 않고 별도 작업폴더에서 띄워 확인:

| 확인 | 결과 |
|---|---|
| `POST /api/events` (lat/lng/risk/ttc/timestamp 전체) | 200, id 발급 |
| `POST /api/events` (lat/lng/risk 만 — SUMO 업로더 최소 형태) | 200, timestamp 서버시각 자동 |
| `POST` risk=7 (범위 밖) | **422 거부** — 스키마 0~3 제약이 실제로 걸린다 |
| `GET /api/risk-segments` | 200, 도로 엣지 단위 집계 정상 (`grade`/`events`/`avg`/`ttc`) |

**🔴 그런데 여기서 결함 하나가 나왔다: 도로 스냅에 거리 제한이 없다.**

실시간 lux가 실내에서 쓰는 fallback 좌표 `37.0/127.0`을 그대로 올려봤더니, 거부되지 않고
**상도동 도로(37.4901/126.9535)에 위험도 3으로 찍혔다.** 두 지점은 약 55km 떨어져 있다.
`app/roads.py::nearest_edge_id`가 거리 상한 없이 무조건 가장 가까운 엣지를 돌려주기 때문이다.

→ 지금 상태에서 업로드 클라이언트를 붙이면 **실내 개발 중에 지도가 조용히 오염된다.**
   (전송 쪽에는 이미 같은 성격의 방어가 있다: `send_risk`의 트러스트 게이팅이 `gps_valid=0`이면
   nonzero risk를 막는다. 업로드에도 같은 게이트가 필요하고, 서버에도 상한이 필요하다.)

### 결정할 것
- [ ] lux 파이프라인에 **위험 이벤트 발생 시 `POST /api/events` 호출부 추가**
      (현재 lux 전체에 업로드 호출부가 **0건**임을 grep으로 확인. RSU로만 보내고 있다)
- [ ] **업로드에 트러스트 게이트 적용**: `gps_valid=0`이면 올리지 않기 (전송 정책과 동일하게)
- [ ] **서버 스냅 거리 상한 추가**: 가장 가까운 엣지가 N m 이상 떨어지면 저장 거부(400)
      또는 `off_road`로 표시. N은 도로망 커버리지 기준으로 정하면 됨 (예: 50m)
- [ ] 지도 서버 주소 합의 (localhost / 팀 공용 / cloudflared 터널)
- [ ] 업로드 좌표는 상도동 위경도여야 함 → 실시간 cane의 37.0/127.0 가짜값 문제(Part 2와 연동)
- [ ] 업로드 트리거 정책 (매 이벤트? risk≥1만? 중복 억제?)
- [ ] map을 정본 리포에 편입 (Part 1)

### SUMO → 지도 경로

`import_sumo_results.py`는 CSV(`lat,lng,risk,ttc,timestamp`)를 읽어 같은 `POST /api/events`로
올린다. **파이프는 놓여 있는데 아직 데이터가 안 흐른다** — 리포에 있는 건 4줄짜리 포맷 예시
(`sumo_risk_sample.csv`, 이미 상도동 위경도)뿐이고, 실제 SUMO 결과는 로컬 좌표(3600/1400)라
그대로는 못 올린다. 스크립트 docstring도 `convertXY2LonLat()`로 먼저 변환하라고 적고 있다.
→ **Part 2(좌표계)가 풀려야 이 경로가 살아난다.** 스크립트 자체를 고칠 일은 없다.

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

## 9-2. TTC "미접근" 센티넬 — **통일하지 않기로 판단(2026-07-28)**
- `9999`(risk_calculator/risk_scoring) / `None`(kinematics) / `rel_speed≤0.2`(risk_engine) 제각각
- 수식 자체(`거리/상대속도`)는 동일

세 값이 흐르는 경로를 따라가 본 결과, **통일이 이득이 없고 오히려 손해**다.

| 위치 | 표현 | 이유 |
|---|---|---|
| `risk_scoring.calculate_ttc` | `9999.0` | 팀 점수표 입력값. 표의 마지막 컷오프가 `ttc<=12`라 9999는 "TTC 0점"을 만드는 **표의 일부**다. 여기를 바꾸면 팀 표를 훼손하고 Part 3 drift 테스트가 깨진다 |
| `kinematics` | `None` | 물리량. "최근접이 앞에 없음"은 값이 없는 것이지 큰 값이 아니다 |
| `risk_engine.estimate_ttc` | `None` (+`rel_speed≤0.2` 게이트) | Stack B 소속. Part 9-1에서 **정리 대상**으로 이미 지정됨 |

세 값이 실제로 섞이는 경계는 지금 **없다.** `assess_risk`는 kinematics의 ttc를 쓰지 않고
팀 함수로 다시 계산한다(그래서 점수가 팀 표와 정확히 일치한다). 억지로 상수 하나로 묶으면
팀 표를 건드리거나 안 쓰는 변환 레이어가 생긴다.

**대신 진짜 함정은 따로 있다 (Part 4와 함께 처리):**
AI feature 11개 중 9번째가 `ttc`인데, `predict_risk.TrajectoryBuffer.features()`는
`sample.get(name, 0.0)`으로 없는 값을 **0.0**으로 채운다. 미접근(None)이 그대로 흘러들면
**TTC 0초 = 즉시 충돌 = 최고 위험**으로 뒤집힌다. 아직 배선 전이라 지금 터지지는 않는다.

- [x] 미접근 표현 통일 → **하지 않음**(위 근거). 대신 각 위치에 왜 다른지 이 표로 고정
- [ ] AI feature 배선 시 `ttc` 미접근을 무엇으로 넣을지 결정 (9999? 팀 학습 데이터가 미접근을
      어떤 값으로 채웠는지 최민서 확인 필요 — 학습과 추론이 같아야 한다)

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
