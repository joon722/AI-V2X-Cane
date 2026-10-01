# V5 검토 후 Claude가 고친 내용 (2026-10-01)

GPT가 만든 V5(GitHub `codex/v5-new-hardware` 945f2e1)를 검토하고 아래를 고쳤다.
고치기 전 파일은 각 폴더에 `*.before_20261001_claude_review.bak`로 남겨 두었다.
V2X 패킷(버전 3, 40/38/35 B)과 기존 키 이름은 바꾸지 않았다. 네 스케치 모두 다시 컴파일 확인.

## 1. UWB 각도(AoA) 사용 — 가장 큰 변경

- `c33_uwb_bridge_V5.ino`: 거리만 보내던 줄 뒤에 쉴드가 잰 각도를 덧붙인다.
  `status="SUCCESS", distance[cm]=345, RSSI[dBm]=nan, azimuth[deg]=-12.3, aoa_fom=87, nlos=0`
  - `aoa_azimuth`(Q9.7) /128 = 도, `aoa_fom` 0~100, `nlos` 0/1/255. 앞부분 형식은 그대로라 DWM3001CDK로 바꿔 끼워도 동작한다.
  - `UWB_VERBOSE_LOG`(기본 0): 1이면 `UWB.begin(Serial, UWB_RX_LEVEL)` 상세 로그(측거 실패 이슈 #5 진단용).
  - USB 진단 출력에 마지막 각도·신뢰도·가림도 표시.
- `car_debug_V5.ino`:
  - `UwbRangeState`에 `hasAngle/azimuthDeg/azimuthFom/nlos/lastAngleMs/angleCount` 추가, `parseUwbLine()`이 각도 항목을 읽는다(없으면 거리만).
  - `uwbAngleIsValid()`: 최신 + |각도| ≤ 60° + 신뢰도 ≥ `uwbfom`(새 튜닝값, 기본 50) + 가림 아님.
  - `uwb status`에 각도 줄 추가.
  - 새 로그 키(두 모드 모두): `UWB각도유효`, `UWB각도`, `UWB각도신뢰`, `UWB가림`.
  - 각도는 아직 위험 판단에 쓰지 않는다(실물 각도시험 후 결정). 젯슨으로 보내는 건 `V5_RSU_AOA_PROPOSAL.md`(팀장 협의).

## 2. 안전·오동작 관련

- 차량 충격 기준 `IMPACT_THRESHOLD_MPS2` 30 → **300(사실상 끔)**. BNO055는 NDOF 모드에서 가속도가 ±4g(약 39 m/s²)로 고정돼 큰 충격과 거친 노면이 구분되지 않는다(V4 정상 접근 주행 피크 42.95). 시험 후 `impact`로 조정.
- 차량 GPS 영점 버튼(GPIO33): 한 번 누르면 바로 시작하던 것을 **2초 이상 누를 때만 한 번** 시작하게 바꿈(9/28 합의).
- `BNO_USE_EXTERNAL_CRYSTAL` 기본 1 → **0** (차량·지팡이). 크리스털을 납땜했을 때만 1.

## 3. 센서

- GPS L5: `configureMgF10L5()` 추가(차량·지팡이). `CFG_SIGNAL_GPS_L5_ENA=1`, `CFG_SIGNAL_GPS_L5_HEALTH_OVERRIDE=1`을 RAM에, 5 Hz 설정과 **따로** 먼저 보낸다(알 수 없는 키가 섞이면 VALSET 전체가 거절되므로). 부팅 로그에 `[GPS] L5 enable+health override=ACK/FAILED`.
- IMU 읽기 부하: 10 ms마다 벡터 6개(차량)/4개(지팡이)를 100 kHz I2C로 따로 읽던 것을, 10 ms마다 필요한 것만(차량: 자이로·선형가속·오일러 / 지팡이: 자이로·오일러), 가속도·자력계·보정상태는 100 ms마다 읽게 바꿈(`IMU_SLOW_INTERVAL_MS`). 차량의 중력 벡터는 어디에도 쓰지 않아 읽지 않음.
- 지팡이 방향 출력 조건에서 가속도 보정도(A ≥ 2)를 뺌(G·M ≥ 2 + 정렬). 저장 오프셋을 불러와도 재부팅 직후 A가 낮게 보여 방향이 오래 무효가 될 수 있어서.

## 4. 로그(뷰어)

- 차량 `logmode 0`에 추가: `시각ms, 원시위험, TTC, 계산거리, 접근속도, 전방여부, UWB원시거리, UWB접근속도, UWB보정, UWB각도유효, UWB각도, UWB각도신뢰, UWB가림, RSSI평활, 지팡이수신`.
- 차량 `RSU위험`이 실제로는 `lastRiskLevel`(차량 최종값)이던 것을 바로잡음: RSU가 보낸 값을 `lastRsuRiskLevel`에 따로 저장. 키 `RSU경과ms` → **`RSU위험경과ms`**(지팡이·뷰어 요청서와 같은 이름).
- 지팡이 `logmode 0`에 추가: `시각ms, 차량위험, 차량위험경과ms, GPS복구횟수, IMU방향유효, IMU방향, UWB보정, UWB원시거리, UWB접근속도, RSSI평활, 송신, 차량수신`.
- `logmode 1`(상세)에도 `IMU보정`, `IMU정렬`(차량은 + UWB 각도 4개, `RSU위험`, `RSU위험경과ms`, `부팅횟수`, `리셋원인`)을 추가.
- 기본 2 Hz(`rate 500`)는 그대로 둠(채널 6을 ESP-NOW와 나눠 쓰므로 평소엔 적게 보내는 게 맞다). 실험 기록은 뷰어의 `상세 기록 모드`(logmode 1 + rate 100).

## 5. 웹뷰어

- 차량 스케치 폴더에 `web_page.h`를 새로 둠. `.ino`의 WEB_PAGE 블록은 `#include "web_page.h"` 한 줄.
  이유: 새 웹페이지를 `.ino` 안 원시 문자열로 넣으니 Arduino의 자동 함수선언(ctags)이 따옴표 때문에 길을 잃어 `cmdReply was not declared` 등으로 컴파일이 깨졌다. `.h`는 그 처리를 거치지 않는다.
  웹뷰어를 고칠 때는 `클로드 코드/web_viewer.html` → `python3 build_web_viewer.py`.

## 6. 하지 않은 것 (다음 작업)

- UWB 각도를 RSU/젯슨으로 보내는 새 패킷 → `V5_RSU_AOA_PROPOSAL.md` (팀장 합의 후).
- 각도를 차량 자체 위험 판단(정면 여부 등)에 쓰기 → 실물 각도시험 후.
- 지팡이 버튼(GPIO33) 기능(스윙 중앙 기준 맞추기) → 아직 없음.
- 뷰어 로그 유니캐스트 전환(브로드캐스트 손실 줄이기) → 필요하면 뷰어와 같이.
- GitHub `codex/v5-new-hardware` 브랜치는 아직 이 수정 전 상태다.
