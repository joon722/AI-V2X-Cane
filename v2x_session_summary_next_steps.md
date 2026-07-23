# V2X Jetson/RSU/Smart Cane Session Summary

## Current Goal

Jetson에서 RSU ESP32가 USB Serial로 보내는 V2X JSON 신호를 안정적으로 수신하고, CSV로 저장한 뒤, 이후 risk 계산/전송 구조로 확장한다.

현재 차량 ESP32는 팀원이 작업 중이라 실제 차량 신호는 아직 사용 불가.  
현재는 지팡이 ESP32 신호만 사용해서 단계별 검증 중.

---

## 현재 실제 입력 상태

```text
지팡이 ESP32 신호: 수신 성공
차량 ESP32 신호: 아직 없음
RSU -> Jetson USB Serial: 정상
포트: /dev/ttyUSB0
baud: 115200
현재 지팡이 좌표: 37.0 / 127.0
gps_valid: 0 (실내에서 fix 실패, 펌웨어가 고정 좌표로 대체)
좌표 상태: fallback
```

확인이 필요한 하드웨어 관찰 사항:

```text
지팡이 ESP32가 재부팅된 적 있음 (seq가 34525 -> 7019 로 되돌아감)
lost_count=60141 / recv_count=39132 로 손실률이 60%대로 보이지만
rssi는 -33 ~ -35 로 신호가 강함. 리부트로 seq가 튀면서 생긴
카운터 착시일 가능성이 높으나 확인되지 않음.
```

---

## 단계별 진행 방식

앞으로는 한 번에 전체를 하지 않고, 아래 단계 중 하나씩 진행한다.  
사용자가 "다음 시작해줘"라고 하면 다음 단계로 넘어간다.

```text
1단계. UART/Serial 원본 JSON 정상화
2단계. Python에서 JSON 파싱 + CSV 저장
3단계. 실제 데이터 / test 데이터 구분 정리
4단계. 최신 cane / vehicle 상태 저장 구조 만들기
5단계. 차량 없을 때 test vehicle 생성해서 거리 감소 확인
6단계. 거리/TTC/closing speed 계산
7단계. risk score/risk_level 수식 적용
8단계. risk_level을 RSU/ESP32로 전송
9단계. 실제 차량 신호 들어오면 test vehicle을 real vehicle로 교체
```

현재 완료:

```text
1단계 완료
2단계 완료
3단계 완료 (Jetson 실기 확인만 남음)
4단계 완료
5단계 완료 (Jetson 실기 확인 완료)
6단계 완료 (Jetson 실기 확인 완료)
7단계 완료 (Jetson 실기 확인 완료)
8단계 완료 (Jetson 실기 확인 완료)
```

다음 채팅에서 시작할 단계:

```text
9단계. 실제 차량 신호로 test vehicle 교체
```

---

## 1단계 완료 내용: UART/Serial 원본 JSON 정상화

Jetson에서 아래 명령으로 원본 UART 수신 확인:

```bash
sudo fuser -k /dev/ttyUSB0
stty -F /dev/ttyUSB0 115200 raw -echo
cat /dev/ttyUSB0
```

정상 출력 예:

```json
{"type":"cane","node_id":4125577512,"seq":32447,"gps_valid":1,"lat":37.000000,"lng":127.000000,"speed_mps":0.000,"heading_deg":0.00,"node_risk":0,"tx_ms":3246114,"rx_ms":3388464,"recv_count":32447,"lost_count":1,"rssi":-21,"src_mac":"30:76:F5:E7:51:28"}
```

성공 기준:

```text
줄마다 { 로 시작
줄마다 } 로 끝남
type=cane 정상
seq가 계속 증가
node_risk=0 정상
```

현재 결과:

```text
지팡이 -> RSU -> Jetson UART 수신 정상
```

---

## 2단계 완료 내용: Python 파싱 + CSV 저장

Jetson 폴더:

```bash
cd ~/v2x/03_jetson
```

사용한 파일:

```text
step2_parse_cane.py
```

실행:

```bash
sudo fuser -k /dev/ttyUSB0
python3 step2_parse_cane.py
```

생성된 CSV:

```text
step2_cane_parsed_log.csv
```

CSV 확인:

```bash
head -5 step2_cane_parsed_log.csv
```

확인된 출력:

```csv
pc_time,type,node_id,seq,gps_valid,lat,lng,speed_mps,heading_deg,node_risk,tx_ms,rx_ms,recv_count,lost_count,rssi,src_mac,source_mode
1782981067.319,cane,4125577512,3358,1,37.0,127.0,0.0,0.0,0,337213,1514353,3359,0,-43,30:76:F5:E7:51:28,test
1782981067.423,cane,4125577512,3359,1,37.0,127.0,0.0,0.0,0,337313,1514457,3360,0,-40,30:76:F5:E7:51:28,test
1782981067.516,cane,4125577512,3360,1,37.0,127.0,0.0,0.0,0,337413,1514551,3361,0,-42,30:76:F5:E7:51:28,test
```

의미:

```text
type=cane: 지팡이 신호
node_id=4125577512: 지팡이 노드 ID
seq=3358,3359,3360: 패킷 순번 정상 증가
gps_valid=1: GPS 유효 표시
lat/lng=37.0/127.0: 현재 고정 테스트 좌표
speed_mps=0.0: 지팡이 정지
heading_deg=0.0: 방향각 0도
node_risk=0: 지팡이 risk 0
lost_count=0: 누락 없음
rssi=-40대: 수신 양호
source_mode=test: 고정 좌표이므로 테스트 데이터로 분류
```

2단계 결론:

```text
Python에서 JSON 파싱 성공
CSV 저장 성공
node_risk 필드 정상 처리
source_mode=test 분류 성공
```

---

## 3단계 완료 내용: 출처 구분

`step2_parse_cane.py`를 고치지 않고 `step3_parse_v2x.py`를 새로 만들었다.

실행:

```bash
sudo fuser -k /dev/ttyUSB0
python3 step3_parse_v2x.py --source-mode test
```

`source_mode`는 `real / test / fallback / simulation` 네 값만 허용한다. 자세한 기준은 `STEP3_SOURCE_MODE.md` 참고.

송신 JSON에 유효한 `source_mode`가 들어 있으면 실행 인자보다 우선한다. cane과 vehicle이 한 스트림에 섞여도 레코드별 출처가 보존된다.

출력:

```text
[STATE] type=cane seq=3360 node_risk=0 gps_valid=1 source=test
```

CSV `step3_v2x_parsed_log.csv`에도 `source_mode` 컬럼이 남는다.

3단계 상태:

```text
파싱/CSV/출력 로직: 검증 완료 (test_step3_parse_v2x.py 4개 통과)
시리얼 경로: Jetson 실기 확인 완료
```

Jetson 실행 시 첫 줄에서 아래 경고가 1회 날 수 있다. 전송 중이던 줄의 중간부터 읽어서 생기는 것이며 정상이다.

```text
[WARN] parse_failed error=Expecting ',' delimiter: line 1 column 33
```

아래 경고가 수백 줄 쏟아지던 문제는 수정했다.

```text
[WARN] parse_failed error=Expecting value: line 1 column 1 (char 0)
```

포트를 열 때 커널 버퍼에 쌓여 있던 묵은 바이트를 읽어서 생긴 것이다. 깨진 바이트는
`errors="replace"` 때문에 U+FFFD 로 바뀌는데, 이 문자는 공백이 아니라서 `strip()`으로도
빈 줄 가드로도 걸러지지 않고 JSON 파싱 첫 글자에서 터진다.

`serial_lines`에서 포트를 연 직후 입력 버퍼를 비우도록 고쳤다.

```python
connection.reset_input_buffer()
```

CSV는 append 방식이므로 이번 실행분을 보려면 `head`가 아니라 `tail`을 쓴다.

```bash
tail -3 step3_v2x_parsed_log.csv
```

---

## 4단계 완료 내용: 최신 상태 저장

`step4_state_store.py`를 새로 만들었다. step2/step3은 수정하지 않고 `step3_parse_v2x`에서 파싱 함수를 import해서 쓴다.

실행:

```bash
sudo fuser -k /dev/ttyUSB0
python3 step4_state_store.py --source-mode fallback
```

동작:

```text
type=cane 이면 최신 cane 상태 갱신
type=vehicle 이면 최신 vehicle 상태 갱신
각 레코드의 Jetson 수신 시각(pc_time) 저장
현재 시각 기준 각각 500ms 이내인지 검사
거리/TTC/risk 계산은 하지 않음
```

상태 값:

```text
MISSING  한 번도 받은 적 없음
READY    받았고 500ms 이내
STALE    받았으나 500ms 초과
```

판정 기준:

```text
pair_valid = cane READY 이고 vehicle READY
risk_valid = pair_valid 이고 양쪽에 쓸 수 있는 좌표가 있음
```

좌표 판정은 `gps_valid`가 아니라 lat/lng 값 자체로 한다. 빈 값이거나 `0.0/0.0`이면 좌표 없음으로 본다.

`gps_valid=0`이어도 fallback 좌표가 있으면 계산은 진행한다. 실내에서 GPS fix가 안 잡히는 동안에도 6~8단계를 개발할 수 있어야 하기 때문이다. 그 데이터를 믿을 수 있는지는 `gps_valid`가 아니라 `source_mode`로 판단한다.

현재 출력 (차량 신호 없음):

```text
[STATE] type=cane seq=7065 gps_valid=0 source=fallback
cane=READY
vehicle=MISSING
pair_valid=False
risk_valid=False
```

test vehicle을 넣었을 때:

```text
[STATE] type=vehicle seq=1 gps_valid=0 source=simulation
cane=READY
vehicle=READY
pair_valid=True
risk_valid=True
```

알려진 한계:

```text
레코드가 들어올 때만 출력하므로 스트림이 완전히 끊기면 STALE이 화면에 안 뜬다.
주기적 tick은 8단계(RSU 전송) 즈음에 필요해지면 넣는다.
```

---

## 지금까지 만든/사용한 주요 개념

### 기본 risk 처리 생각

```text
cane 신호 없음 -> risk 0
vehicle 신호 없음 -> risk 0
둘 다 없음 -> risk 0
cane만 있음 -> vehicle risk 계산 불가, 기본 risk 0
vehicle만 있음 -> cane 위치 없으면 계산 불가, 기본 risk 0
둘 다 있음 -> distance/TTC/relative speed 계산 후 risk 분류
```

### risk score / risk_level 방향

추후 사용자가 수식 확정 후 코드화 예정.

예상 구조:

```text
Distance score
TTC score
Relative speed score
Approach direction score
Road environment score
합산 -> risk_score
risk_score -> risk_level 0/1/2/3
```

ESP32로는 score 전체가 아니라 최종 risk_level만 보내는 방향.

```json
{"target_id":0,"risk":2}
```

---

## 8단계 전송 경로 사전 확인 (완료)

5~7단계를 건너뛰고 다운링크 경로가 살아있는지만 먼저 확인했다. `probe_risk_downlink.py`는
일회성 실험 파일이며 파이프라인의 다른 파일은 이것을 참조하지 않는다.

```bash
python3 probe_risk_downlink.py
```

3초마다 risk 값을 2와 0으로 번갈아 보내고 지팡이의 `node_risk`가 따라오는지 본다.

결과:

```text
7번 전송 / 7번 모두 반영. 어긋난 구간 없음.
보낸 risk 값: [0, 2]
관찰된 node_risk 값: [0, 2]
```

명령 형식은 요약 문서에 적어둔 그대로가 맞았다.

```json
{"target_id":0,"risk":2}
```

RSU 펌웨어가 이미 risk 다운링크를 구현하고 있다. 처리 결과를 아래 타입으로 시리얼에
올려보낸다. 8단계에서 펌웨어 추가 작업은 필요 없다.

```text
type=risk_tx
type=risk_broadcast_to_seen
```

지팡이 시리얼 모니터로 반대편도 확인했다. 경로가 한 바퀴 다 돈다.

```text
[CANE RX]  risk=0 target=4125577512 src=0 seq=11
[CANE OUT] risk 2 -> 0
[CANE SEND] id=4125577512 seq=651 ... risk=0 ... result=OK
```

### target_id=0 은 브로드캐스트다

`target_id=0`으로 보내면 RSU가 `risk_broadcast_to_seen`으로 자기가 본 모든 노드에
뿌린다. 지팡이는 `target=4125577512`(자기 ID)로 받는다.

차량이 붙으면 지팡이와 차량이 둘 다 같은 risk를 받게 된다. 노드별로 다른 값을 주려면
`--target-id`에 실제 node_id를 지정해야 한다.

### 지팡이에 가속도 센서가 있다

지팡이 시리얼 모니터에는 아래 값이 찍히지만 Jetson으로 오는 JSON에는 없다.

```text
ax=0.78 ay=-0.37 az=9.74
```

`az≈9.7`은 중력이므로 정지 상태 기준값이다. 낙상 감지 등에 쓰려면 펌웨어에서 JSON에
추가해달라고 요청해야 한다.

아직 확인되지 않은 것:

```text
node_risk 값과 [CANE OUT] 로그는 확인했으나
지팡이가 물리적으로 반응하는지(진동/소리)는 확인하지 않았다.
```

---

## 파일 구조

단계마다 파일을 새로 만들고 이전 파일은 고치지 않는다.

```text
step2_parse_cane.py        기존 파싱/CSV 검증용
step3_parse_v2x.py         source_mode 명시적 구분
step4_state_store.py       cane/vehicle 최신 상태 저장
step5_test_vehicle.py      test vehicle 궤적 생성 (단독 실행도 가능)
step6_kinematics.py        거리/closing speed/TTC/CPA + 칼만 필터
step7_risk.py              risk_score/risk_level + DCPA 억제 게이트
step8_send_risk.py         risk_level RSU 전송 (트러스트 게이팅 + heartbeat)
probe_risk_downlink.py     일회성 실험. 다운링크 경로 확인용
```

테스트 (94개):

```bash
python3 -m unittest discover -p "test_*.py"
```

---

## 5단계 완료 내용: test vehicle 생성 및 거리 감소 확인

`step5_test_vehicle.py`가 궤적을 만들고, `step4_state_store.py`가 같은 루프 안에서
그 레코드를 vehicle로 주입한다. 별도 프로세스로 분리하지 않은 이유는 하나다.

```text
목표 지점이 StateStore에 들어있는 "지팡이의 최신 좌표"여야 하기 때문.
별도 프로세스면 지팡이가 어디 있는지 알 수 없어 고정 좌표로만 달릴 수 있다.
```

동작:

```text
지팡이 최신 좌표에서 bearing 0도(북쪽) 방향으로 start_m 떨어진 지점에서 출발
0.2초마다 speed m/s 만큼 지팡이 쪽으로 접근
거리 0에 도달하면 그 자리에 멈춘다 (거리 그래프가 단조 감소로 유지됨)
source_mode=simulation 이 payload에 박혀 있어 CLI 값보다 우선한다
```

실행:

```bash
python3 step4_state_store.py --source-mode fallback --test-vehicle
python3 step4_state_store.py --source-mode fallback --test-vehicle --vehicle-speed 10 --vehicle-start-m 80
```

성공 화면:

```text
[TESTVEH] seq=1 distance_m=50.0
[STATE] type=vehicle seq=1 gps_valid=1 source=simulation
cane=READY
vehicle=READY
pair_valid=True
risk_valid=True
```

`distance_m`은 생성기가 "의도한" 거리다. 6단계에서 두 좌표로 거리를 독립 계산하면
이 값과 대조해서 검증할 수 있다.

Jetson 실기 확인 결과 (2026-07-21):

```text
[TESTVEH] seq=1  distance_m=50.0
[TESTVEH] seq=25 distance_m=25.5
[TESTVEH] seq=50 distance_m=0.0
pair_valid=True  (5단계 전까지는 계속 False였음)
```

주입 시점은 시리얼 수신에 물려 있지만 거리는 벽시계 시간으로 계산한다.
시리얼이 잠깐 끊겨도 위치가 어긋나지 않고 따라잡는다.
6단계 TTC도 같은 이유로 타임스탬프 기반으로 짠다.

### 지팡이 좌표는 실내에서 고정 fallback 값이다

```text
gps_valid=0 일 때 지팡이가 보내는 lat/lng는 실제 위치가 아니라
GPS 실패 시 나오는 고정 좌표다. (0,0)이 아니라서 has_position은 통과한다.
```

5~7단계에는 지장이 없다. 가짜 차량은 그 좌표를 기준점으로 삼아 상대 거리를
만들 뿐이므로 기준점이 상수여도 접근 시뮬레이션과 수식 검증은 성립한다.

8단계에서 결정해야 할 것:

```text
gps_valid=0 이면 계산된 risk도 실제와 무관하다.
그런데 그 risk를 전송하면 지팡이는 진짜로 진동한다.
근거 없는 경고를 보낼 것인가, gps_valid=0 이면 전송을 막을 것인가.
```

주의:

```text
test vehicle은 실측이 아니므로 source_mode를 real로 두지 않는다.
9단계에서 실제 차량 신호로 교체할 때 --test-vehicle 플래그만 빼면 되고
6~8단계 코드는 손대지 않는다.
```

---

## 6단계 완료 내용: 거리 / closing speed / TTC / CPA

`step6_kinematics.py`를 새로 만들었다. step3/4/5는 손대지 않고 import만 한다.

### 좌표계

두 좌표를 로컬 ENU 평면(미터)으로 바꾼 뒤 모든 계산을 벡터로 한다. CPA와 벡터 투영에는
내적이 필요한데 위경도로는 내적을 할 수 없다.

```text
원점 = 첫 cane 좌표 (세션 내내 고정)
east  = (lng - lng0) * cos(lat0) * 111320
north = (lat - lat0) * 111320
```

`111320`은 5단계 `offset_position`이 쓰는 상수와 **일부러 같은 값**이다. 그래야 가짜 차량이
의도한 거리와 6단계가 독립 계산한 거리가 소수점까지 일치해서, 어긋나면 모델 차이가 아니라
버그라고 단정할 수 있다. `lux/risk_engine.py`의 haversine을 쓰면 지구 반지름 모델이 달라
0.1%(50m에서 5cm) 어긋난다.

### 두 개의 트랙

같은 계산을 두 입력에 각각 돌린다.

```text
raw       수신 좌표 그대로 + JSON의 speed_mps/heading_deg
filtered  칼만 추정 위치 + 칼만 추정 속도
```

raw는 가짜 차량 정답과 정확히 맞아야 하는 수식 검증용이고, filtered가 7단계가 쓸 값이다.

### 수식

`r` = cane→vehicle 상대위치 벡터, `v` = 상대속도 벡터.

```text
closing_los  = -(r·v) / |r|          벡터 투영. 양수면 접근
closing_diff = -Δ거리 / Δt            거리 미분 (raw 트랙만)
ttc_simple   = |r| / closing_los     closing<=0 이면 없음
tcpa         = -(r·v) / |v|²          최근접까지 남은 시간
dcpa         = |r + v·tcpa|           그때 벌어질 최소 거리
```

`ttc_simple`은 1대1 기본 규칙용이라 기존 팀 점수표(`scripts/risk_calculator.py`)와 바로
호환된다. CPA는 정확도용이다. 차가 보도 옆을 4m 빗겨 지나가면 `ttc_simple`은 "곧 충돌"이라
하지만 `dcpa=4.0m`이 나와 헛경보를 걸러낸다. 이게 CPA를 넣은 이유다.

`closing_diff`는 칼만 트랙에는 없다. 거기서는 속도가 이미 상태 변수라 미분할 이유가 없다.

### 칼만 필터

east/north가 이 모델에서 서로 독립이므로 **2상태(위치·속도) 1차원 필터 두 개**로 분해했다.
4x4 행렬 하나와 수학적으로 동일하면서 2x2 산술로 끝나고, Jetson에 numpy 의존성이 안 생긴다.

상수는 추측이 아니라 스펙에서 유도했다.

```text
측정 노이즈   sigma_pos = 2.5 m      GPS 수평 정확도 2.5m CEP
프로세스 노이즈 sigma_a  = 3.0 m/s²    차량 가속 한계 기준 (보행자 ~1.5보다 큰 쪽)
초기 속도 분산  10 m/s                 처음엔 속도를 모르므로 넓게 시작
```

둘 다 `--gps-sigma-m` / `--accel-sigma`로 노출해두었다. 실측 GPS 로그가 쌓이면 여기서 바로
재조정한다.

레코드가 올 때마다 그 시각까지 predict → 측정으로 update 한다. cane 10Hz / vehicle 5Hz로
비동기여도 되고, 상대운동 계산 직전에 양쪽을 공통 시각으로 predict 해서 시간을 맞춘다.

### 개발 중 발견한 결함 두 개

**1. 거리에 잘못된 시각을 붙이고 있었다.**

융합된 거리는 두 입력 중 *더 오래된* 쪽만큼만 최신인데 처음엔 더 최신인 쪽 시각을 썼다.
그래서 새 차량 좌표 없이 cane만 들어와도 "시간은 흘렀는데 거리는 그대로"가 되어 접근속도
0이 찍혔다. 칼만 트랙은 예측이 가능하니 최신 시각(max)으로 보고하고, raw 트랙은 예측이
없으니 오래된 쪽(min)을 쓰도록 나눴다.

**2. 미분 구간이 너무 짧았다.**

cane과 vehicle은 같은 배치에서 몇 ms 차이로 도착한다. 그 3ms를 미분하면 "거리 변화 없음
→ 접근속도 0"이 나온다. 이제 미분은 **두 노드가 모두 새 좌표를 낸 구간**에서만 계산하고,
그 외에는 값을 내지 않는다(`-`). 그래서 `closing_diff`는 차량 갱신마다 한 번씩만 나온다.

### 실행

```bash
sudo fuser -k /dev/ttyUSB0
python3 step6_kinematics.py --source-mode fallback --test-vehicle
```

성공 화면:

```text
[TESTVEH] seq=85 distance_m=33.0
[KIN] d=33.00m closing los/diff=5.00/5.00 kf=5.01 ttc=6.60s tcpa=6.60s dcpa=0.00m
[KIN] d=33.00m closing los/diff=5.00/- kf=5.01 ttc=6.60s tcpa=6.60s dcpa=0.00m
```

읽는 법:

```text
d           두 좌표로 독립 계산한 거리. [TESTVEH] distance_m 과 일치해야 한다
closing los 벡터 투영 접근속도. 정면 접근이면 차량 속도와 같다
closing diff 거리 미분. los와 일치해야 한다. 차량 갱신 때만 나오고 그 외엔 -
kf          칼만 트랙의 접근속도. 처음 1~2초는 0에서 올라오다가 수렴한다
ttc         거리/접근속도
tcpa/dcpa   최근접 시점과 그때의 거리. 정면 충돌 코스면 dcpa≈0
```

CSV `step6_kinematics_log.csv`에 raw/filtered 값이 모두 남는다. 7단계 임계값을 정하거나
칼만 상수를 재조정할 때 이 로그가 근거 데이터가 된다.

### 검증 상태

```text
수식/필터/파이프라인: 단위테스트 61개 통과 (6단계분 22개)
Jetson 실기 확인: 완료
```

노트북 확인 결과 (2026-07-22):

```text
raw 거리       50.00 → 31.00 까지 생성기 의도와 정확히 일치
closing_los    5.00 (차량 속도와 일치)
closing_diff   4.96 ~ 5.08
kf             5.01 로 수렴
dcpa           0.00 (정면 충돌 코스)
```

Jetson 실기 확인 결과 (2026-07-22, 지팡이 실신호 + --test-vehicle):

```text
d           42.86 → 33.68 로 단조 감소, [TESTVEH] distance_m 과 일치
closing_los 5.00
closing_diff 5.00 (차량 갱신 때만, 그 외 -)
kf          4.85 → 4.90 → 4.96 → 4.99 → 5.00 → 5.01 로 약 2초 만에 수렴
ttc         8.57 → 6.74 로 거리와 함께 감소
dcpa        0.00

실기에서는 미분 폭발이 재현되지 않았다. 시리얼을 한 프로세스가 직접 읽어
도착 시각이 뭉개지지 않기 때문. "PowerShell 파이프 문제였다"는 진단이 확정됐다.
```

### Windows에서 실험할 때 주의

PowerShell 파이프(`python feed.py | python step6_kinematics.py`)로 테스트하면 레코드가
뭉쳐서 전달되어 도착 시각이 뭉개진다. 그러면 `closing_diff`가 476, 1116 같은 값이 되고
칼만도 발산하는 것처럼 보인다. **코드 결함이 아니라 실험 장치의 문제다.** 실제로 이것 때문에
멀쩡한 코드를 한참 의심했다. 타이밍을 봐야 하면 셸 파이프를 거치지 말고 한 프로세스 안에서
`process_line`을 직접 호출해라.

---

## 7단계 완료 내용: risk_score / risk_level 수식 적용

`step7_risk.py`를 새로 만들었다. step2~6은 손대지 않고 import만 한다
(`KinematicsPipeline`, `StateStore`, `TestVehicle`, `serial_lines`). 스코어링 대상은
6단계 filtered(칼만) 트랙이다.

### 팀 점수표는 그대로, DCPA만 게이트로 얹었다

팀 점수표는 `tmp/AI-V2X-Cane-audit/scripts/risk_calculator.py`에 있는데 audit용 임시
폴더라 import 의존이 취약하다. 세 함수(`calculate_ttc` / `calculate_risk_score` /
`classify_risk_level`)를 **숫자 그대로 step7에 복사(vendor)**하고 출처 주석을 달았다.
팀이 표를 바꾸면 여기서 재동기화한다. 팀 표 자체는 한 글자도 안 바꿨다.

7단계가 더한 것은 DCPA 억제 게이트 하나다. 팀 표에는 DCPA 항이 없어서 보도 옆 4m를 빗겨
지나가는 "스침"과 정면 충돌 코스가 같은 점수를 받는다. 6단계가 CPA를 계산한 이유가 이
구분인데, DCPA는 "실제로 나를 맞히는가"의 게이트 성격이라 가점 항이 아니라 배율로 얹었다.

```
base_score  = 팀_calculate_risk_score(...)          # 0~100, 팀 표 그대로
gate        = g(filtered.dcpa)                        # 0~1 배율
final_score = base_score * gate
risk_level  = 팀_classify_risk_level(final_score)     # 0~3
```

게이트는 소프트(선형 보간)다. 하드 컷오프가 아니다.

```text
dcpa <= near_m         → gate = 1.0            (경로 안, 억제 없음)
near_m < dcpa < far_m  → 1.0 → floor 선형 보간
dcpa >= far_m          → gate = floor          (명백한 빗겨감)
dcpa 없음(멀어짐/정지) → gate = 1.0            (표가 closing<=0로 이미 저점 처리)
```

기본값은 GPS 노이즈에 묶었다 (`GPS_SIGMA_M=2.5`).

```text
near_m = 2.5  (≈1σ)   far_m = 7.5  (≈3σ)   floor = 0.2
```

filtered dcpa도 추정치라 오차가 있다. GPS CEP 2.5m 안쪽 dcpa는 실제 충돌일 수 있어
억제하지 않고, 명백히 벗어난(>3σ) 경우만 강하게 낮춘다. floor를 0이 아닌 0.2로 둬서
추정 오류로 진짜 위험을 완전히 지우지 않는다. `--dcpa-near-m` / `--dcpa-far-m` /
`--dcpa-floor`로 노출했다. **실외 GPS 로그가 쌓이면 여기가 1순위 재조정 지점이다.**

### 실행

```bash
sudo fuser -k /dev/ttyUSB0
python3 step7_risk.py --source-mode fallback --test-vehicle
```

성공 화면:

```text
[TESTVEH] seq=25 distance_m=25.5
[RISK] score=49.00 level=2 (base=49.00 dcpa=0.00m gate=1.00)
```

`step7_risk_log.csv`에 base_score·gate·final_score까지 남는다. 8단계 전송값과 게이트
재조정의 근거 데이터가 된다.

### 검증 상태

```text
수식/게이트/vendor 표: 단위테스트 75개 통과 (7단계분 14개)
Jetson 실기 확인: 완료 (2026-07-23)
```

Jetson 실기 확인 결과 (2026-07-23, 지팡이 실신호 + --test-vehicle):

```text
distance  33.7 → 21.4 로 단조 감소
score     44 (level 1) → 거리 25에서 TTC가 5s를 끊자 49 (level 2)로 승격
dcpa      0.00, gate 1.00 유지 (정면 접근이라 게이트는 no-op)
```

정면(dcpa≈0)이라 게이트는 얌전히 빠져 있고, 기존 5·6단계 검증 화면을 안 깨는 것도
확인됐다. 게이트가 실제로 눌리는 건 실차 기하(옆으로 빗겨감)가 들어와야 재현된다.
인프로세스 스모크에서는 9m 빗겨감이 base 51점을 gate 0.2로 눌러 level 0으로 떨어뜨리는
것을 확인했다.

---

## 8단계 완료 내용: risk_level RSU 전송

`step8_send_risk.py`를 새로 만들었다. step2~7은 안 고치고 step7 채점을 import해서 쓴다.
다운링크 형식 `{"target_id":0,"risk":N}`은 probe로 이미 검증됐고, 여기에 step7의 live
risk_level을 물렸다.

### 한 포트로 읽으며 쓴다

`serial_lines`(step3)는 connection을 안 내주므로, step8은 probe처럼 자기
`serial.Serial(port, baud)`을 열어 같은 `/dev/ttyUSB0`에서 read와 write를 모두 한다.
`--stdin` 모드는 시리얼 대신 stdin을 읽고 명령을 stderr `[WIRE]`로 찍어 하드웨어 없이 검증한다.

### 전송 정책은 순수 함수 `RiskTransmitter`로 분리 (시리얼 없이 테스트 가능)

**트러스트 게이팅** (5단계에서 짚어둔 결정을 여기서 확정):

```text
trusted = allow_untrusted or cane gps_valid==1
effective_level = computed_level if trusted else 0
```

- 기본값: cane `gps_valid=0`(실내 fallback)이면 nonzero risk를 막고 0만 보낼 수 있다.
  0은 이전 trusted 경보를 지팡이에서 해제(진동 멈춤)하는 용도다. → 근거 없는 진동 안 나감.
- `--tx-untrusted`: 실내 개발에서 fallback 좌표로도 end-to-end 진동을 확인할 때만 켠다.
  운영에서는 끈다.

**변화시 즉시 + 주기 heartbeat** (cane ~10Hz라 매 레코드 전송은 RSU 홍수):

```text
last_level 없음 또는 effective != last_level → 즉시 (reason=change)
그 외 now - last_send >= heartbeat_s        → 재전송 (reason=heartbeat)
그 외                                        → 보류 (reason=hold)
```

heartbeat 기본 1.0s (`--tx-heartbeat-s`). 다운링크가 드롭돼도 지팡이 상태를 최신으로 유지하고,
경보 해제(0)가 드롭돼 지팡이가 계속 진동하는 것도 막는다. target_id 기본 0=브로드캐스트.

### RSU 확인응답 처리

전송하면 RSU가 `type=risk_tx` / `type=risk_broadcast_to_seen`을 같은 시리얼로 되돌려 보낸다
(요약 위쪽에 기록됨). 이건 명령이 먹혔다는 확인응답이지 파이프라인 입력이 아니다. step8은
`RSU_ACK_TYPES`로 인식해 조용히 소비한다. 처음엔 `[WARN] ignored_type`으로 잘못 찍혔는데
(StateStore가 cane/vehicle만 받으므로), 이 두 타입만 예외 처리했다.

### 실행

```bash
sudo fuser -k /dev/ttyUSB0
# 실내(gps_valid=0)에서 전송/진동까지 눈으로 보려면 --tx-untrusted 필요
python3 step8_send_risk.py --source-mode fallback --test-vehicle --tx-untrusted
```

성공 화면:

```text
[TESTVEH] seq=50 distance_m=25.5
[TX] risk=2 reason=change
```

`step8_risk_tx_log.csv`에 computed_level·effective_level·trusted·reason이 남는다.

### 검증 상태

```text
전송 정책(RiskTransmitter): 단위테스트 85개 통과 (8단계분 10개)
인프로세스 스모크: gps_valid=0 기본 억제(effective=0) / --tx-untrusted 통과 / heartbeat 확인
Jetson 실기 확인: 완료 (2026-07-23)
```

Jetson 실기 확인 결과 (2026-07-23, --test-vehicle --tx-untrusted):

```text
[TX] risk=0 -> 1 -> 2 로 접근에 따라 change 전송, 사이는 heartbeat 로 채움
RSU 가 risk_tx / risk_broadcast_to_seen 를 되돌려보냄 = 명령 수락+브로드캐스트 확인
```

지팡이가 물리적으로 진동하는지(node_risk 반영 후 진동/소리)는 이번에도 별도 확인 안 함.
probe 단계에서 node_risk 가 따라오는 것은 이미 봤고, 이번 실기는 전송+RSU 수락까지 확인했다.

---

## 다음 채팅에서 바로 할 일

다음 채팅 시작 문장 예:

```text
이전 세션 요약 파일 기준으로 9단계 시작해줘.
```

9단계 목표:

```text
팀원 차량 ESP32 신호가 들어오면 --test-vehicle 플래그만 빼서 test vehicle을
real vehicle로 교체한다. 6~8단계 코드는 손대지 않는다.
```

9단계 사전 검증 (신호 없이 가짜 데이터로 완료):

```text
test_step9_real_vehicle.py 로 real 모드(스트림에서 오는 vehicle) 경로를 고정했다.
  - 정면(dcpa~0):   게이트 no-op, level 2까지 상승, nonzero 전송됨
  - 빗겨감(dcpa~8): 게이트 floor(0.20), level 0 유지, 아무것도 전송 안 됨
test vehicle이 정면만 돌아 한 번도 안 걸리던 DCPA 게이트가 여기서 처음 end-to-end로 작동.
```

9단계에 남은 진짜 미지수 (실물이 있어야 함):

```text
실제 차량 ESP32 JSON의 필드 이름/형식이 가정(type/lat/lng/speed_mps/heading_deg/
gps_valid)과 맞는지. 팀원한테 샘플 한 줄 받으면:
  echo '<실차 JSON 한 줄>' | python3 step8_send_risk.py --stdin --source-mode real
로 필드 매핑을 지금 검증 가능. 다르면 step9에서 작은 어댑터만 추가.
실차 기하가 들어오면 CSV 로그로 게이트 임계값(near/far/floor) 재조정도 그때.
```

### git

```text
이번 세션에 .git 이 비어 있던 것을 git init 으로 초기화하고 첫 커밋 생성.
tmp/(자체 git 가진 감사본)와 __pycache__ 는 .gitignore 로 제외.
```

---

## 중요 주의사항

`cat /dev/ttyUSB0`는 계속 실행되는 명령이라 VS Code 터미널에 실행 중 표시가 뜰 수 있음.  
멈추려면:

```text
Ctrl + C
```

Serial 포트가 잡혀 있으면 Python 실행 전에 정리:

```bash
sudo fuser -k /dev/ttyUSB0
```

원본 확인은:

```bash
cat /dev/ttyUSB0
```

Python 파싱 확인은:

```bash
python3 step2_parse_cane.py
```

