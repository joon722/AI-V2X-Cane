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
좌표 상태: 실제 GPS라기보다 test/fallback 고정 좌표로 취급
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
```

다음 채팅에서 시작할 단계:

```text
5단계. 차량 없을 때 test vehicle 생성해서 거리 감소 확인
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
시리얼 경로: Jetson 실기에서 1회 확인 필요
```

---

## 4단계 완료 내용: 최신 상태 저장

`step4_state_store.py`를 새로 만들었다. step2/step3은 수정하지 않고 `step3_parse_v2x`에서 파싱 함수를 import해서 쓴다.

실행:

```bash
sudo fuser -k /dev/ttyUSB0
python3 step4_state_store.py --source-mode test
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
risk_valid = pair_valid 이고 양쪽 gps_valid=1
```

`risk_valid`를 따로 둔 이유는 "짝은 맞았지만 GPS가 없어 계산 불가"인 경우를 구분하기 위해서다.

현재 출력 (차량 신호 없음):

```text
[STATE] type=cane seq=3360 gps_valid=1 source=test
cane=READY
vehicle=MISSING
pair_valid=False
risk_valid=False
```

test vehicle을 넣었을 때:

```text
[STATE] type=vehicle seq=1 gps_valid=1 source=simulation
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

## 파일 구조

단계마다 파일을 새로 만들고 이전 파일은 고치지 않는다.

```text
step2_parse_cane.py        기존 파싱/CSV 검증용
step3_parse_v2x.py         source_mode 명시적 구분
step4_state_store.py       cane/vehicle 최신 상태 저장
step5_test_vehicle.py      (예정) test vehicle 주입
```

테스트:

```bash
python3 -m unittest test_step3_parse_v2x test_step4_state_store
```

---

## 다음 채팅에서 바로 할 일

다음 채팅 시작 문장 예:

```text
이전 세션 요약 파일 기준으로 5단계 시작해줘.
```

5단계 목표:

```text
차량 ESP32가 아직 없으므로 test vehicle을 만들어서
cane과의 거리가 실제로 줄어드는지 확인한다.
```

5단계에서 할 작업:

```text
1. test vehicle 좌표를 생성하는 코드 작성 (source_mode=test 또는 simulation)
2. step4의 StateStore에 vehicle로 주입
3. pair_valid=True 가 되는지 확인
4. 시간에 따라 cane-vehicle 거리가 감소하는지 확인
```

5단계 성공 화면 예:

```text
cane=READY
vehicle=READY
pair_valid=True
risk_valid=True
```

주의:

```text
test vehicle은 실측이 아니므로 source_mode를 real로 두지 않는다.
9단계에서 실제 차량 신호로 교체할 때 이 구분이 기준이 된다.
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

