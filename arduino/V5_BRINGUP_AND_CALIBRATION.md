# V5 펌웨어 업로드·조립 후 점검 순서

> 2026-10-01 Claude 검토 반영본. 바뀐 내용은 `V5_CHANGES_BY_CLAUDE.md` 참고.
> 원본은 `V5_BRINGUP_AND_CALIBRATION.md.before_20261001_claude_review.bak`.

## 1. V5 구성과 V4 호환성

V5는 센서와 UWB 하드웨어만 교체하고, 기존 동작과 V2X 프로토콜은 유지한다.

| 보드 | 업로드할 스케치 (이 폴더 기준) | 역할 |
|---|---|---|
| 차량 ESP32 | `car_debug_V5/` (`car_debug_V5.ino` + `web_page.h`) | GPS/IMU/오디오/ESP-NOW/RSU 위험 수신, UWB 거리·각도 수신 |
| 지팡이 ESP32 | `cane_debug_V5/cane_debug_V5.ino` | GPS/IMU/ESP-NOW/진동·부저 |
| Portenta C33 + UWB Shield | `c33_uwb_bridge_V5/c33_uwb_bridge_V5.ino` | 5 Hz DS-TWR controller, 거리 + 각도(AoA)를 차량 ESP32 UART로 전달 |
| Arduino Stella | `stella_uwb_responder_V5/stella_uwb_responder_V5.ino` | 5 Hz DS-TWR responder |

차량 스케치 폴더의 `web_page.h`는 아이패드 웹뷰어 화면이다. `.ino`와 같은 폴더에 꼭 같이 둔다.
(웹뷰어를 고칠 때는 `클로드 코드/web_viewer.html`을 고치고 `python3 build_web_viewer.py`를 실행하면 `web_page.h`가 다시 만들어진다.)

V2X 구조체는 V4와 바이트 단위로 같다.

- `V2X_MAGIC = 0x56325831`
- `V2X_VERSION = 3`
- 상태 패킷 40 B
- UWB 패킷 38 B (`MSG_UWB_RANGE = 5`)
- 위험 패킷 35 B
- 새 크기의 V2X 패킷 없음

`origin/main` 커밋 `9837a07`의 `rsu/v2x/v2x_bridge/v2x_bridge.ino`가 이미 40 B 상태와 38 B UWB 패킷을 처리하므로 지금 펌웨어로는 RSU/Jetson 수정이 필요 없다.
단, UWB 각도를 젯슨 화면·경보까지 보내려면 새 패킷과 RSU 수정이 필요하다 → `V5_RSU_AOA_PROPOSAL.md` (팀장 협의용).

## 2. 라이브러리와 보드 패키지

ESP32 두 대:

- ESP32 core 3.3.11
- TinyGPSPlus 1.0.3
- SparkFun u-blox GNSS v3 3.1.15
- Adafruit BNO055 1.6.4
- Adafruit Unified Sensor 1.1.15

UWB 두 대:

- Arduino Renesas Portenta Boards 1.6.0 (`arduino:renesas_portenta:portenta_c33`)
- Arduino Mbed OS Stella Boards 4.6.0 (`arduino:mbed_stella:stella`)
- PortentaUWBShield 1.0.2
- StellaUWB 1.0.3
- ArduinoBLE 2.1.0

**PortentaUWBShield와 StellaUWB는 Arduino 라이브러리 관리자에 없다.** GitHub(Truesense-it/PortentaUWBShield, Truesense-it/StellaUWB)에서 ZIP으로 받아 `스케치 → 라이브러리 포함하기 → .ZIP 라이브러리 추가`로 설치한다.
이 노트북에는 이미 `~/Documents/Arduino/libraries/`에 설치돼 있다(2026-10-01).

C33 core 1.6.0에서는 `Precompiled library ... not found` / `undefined reference to UWBHAL`이 난다(GitHub 이슈 #13). `src/cortex-m33/fpv5-sp-d16-hard/libUWBShieldApi.a`를 컴파일 로그가 요구하는 `src/cortex-m33/-mfpu=fpv5-sp-d16--mfloat-abi=hard/`에도 복사하면 된다. 이 노트북의 설치본에는 이미 복사돼 있다.
core를 1.5.x로 내려도 아래 측거 실패 문제(#5)는 피할 수 없으니 1.6.0을 그대로 쓴다.

## 3. 업로드 순서와 배선 확인

1. **UWB는 공식 예제부터 확인한다.** C33+UWB Shield 조합은 측거 세션이 아예 시작되지 않는 문제가 2025-10부터 보고돼 아직 해결되지 않았다(GitHub 이슈 #5, `event 162 Failed`).
   - 쉴드(C33)에 라이브러리 예제 `UWB_RangingControlee`, Stella에 Controller 예제를 올려 거리가 찍히는지 본다.
   - 예제도 실패하면 `c33_uwb_bridge_V5.ino`의 `UWB_VERBOSE_LOG`를 1로 바꿔 올린 뒤 USB 시리얼 로그를 저장해 둔다(이슈에 올릴 자료). 이 경우 차량 GPIO32에 DWM3001CDK를 다시 꽂으면 펌웨어 수정 없이 거리 측정은 계속된다(C33 출력 형식이 DWM3001CDK와 같다).
   - 예제는 되는데 V5 스케치만 안 되면 역할을 예제와 같게 바꿔 본다(V5는 C33=Controller, Stella=Controlee로 예제와 반대).
2. Stella에 responder V5를 업로드한다. 시험 중에는 USB 전원, 최종은 CR2032로 같은 동작을 다시 확인한다.
3. Portenta C33에 controller V5를 업로드하고 UWB Shield를 끝까지 결합한다.
4. C33 `D14/TX`를 차량 ESP32 `GPIO32/RX1`에 연결하고 GND를 공통으로 묶는다. (같은 UART1의 TX GPIO27이 DFPlayer로 가므로 C33도 115200 bps여야 한다.)
5. 차량·지팡이 ESP32에 각각 V5를 업로드한다. **예전에 쓰던 ESP32를 다시 쓰면 먼저 `factory`** 를 입력한다(V4 때 저장한 UWB 오프셋 등이 그대로 불려 온다).
6. 두 MG-F10은 `TX→GPIO16`, `RX←GPIO17`, 115200 bps로 연결한다.
7. 두 BNO055는 3.3 V, `SDA→GPIO21`, `SCL→GPIO22`, `COM3/I2C_SEL→GND`로 연결한다. 주소는 0x28이다.
   보드와 따로 들어 있는 32.768 kHz 크리스털을 납땜했으면 두 `.ino`의 `BNO_USE_EXTERNAL_CRYSTAL`을 1로 바꾼다(기본 0).
8. 차량 DFPlayer RX는 GPIO27이다. 차량 버튼(GPIO33)은 **2초 이상 누르면** GPS 영점을 시작한다. 지팡이 버튼(GPIO33)은 아직 기능이 없다.

안테나와 UWB 보드는 최종 케이스 방향으로 고정한 후 보정한다. 보정 뒤 위치나 방향을 바꾸면 다시 보정한다.

## 4. 최초 보정

뷰어(맥 앱·아이패드)의 `IMU 상태`, `IMU 저장`, `UWB 상태` 버튼으로 아래 명령을 보낼 수 있다.

### BNO055 — 필요함

차량과 지팡이 각각 다음을 수행한다.

1. 완성된 케이스 상태로 전원을 켠다. 차량은 스피커와 DFPlayer도 평소처럼 켠다.
2. `imu`(= IMU 상태 버튼)로 `S/G/A/M`을 본다.
3. 보정도를 3까지 올린다.
   - G(자이로): 책상에 두고 3~5초 가만히.
   - A(가속도): **6방향(위·아래·앞·뒤·왼·오른쪽이 바닥을 보게)으로 놓고 각각 3~5초씩 가만히.** 돌리기만 해서는 A가 3까지 잘 안 올라간다.
   - M(자력계): 공중에서 천천히 8자를 몇 번 그린다.
4. `S/G/A/M=3/3/3/3`이 되면 `imusave`(= IMU 저장 버튼)로 오프셋을 저장한다.
5. **방향 부호 확인:** 노드를 책상 위에서 시계방향으로 90° 돌렸을 때 차량 `차체방향`(지팡이 `IMU방향`)이 약 90 늘어나는지 본다. 줄어들거나 엉뚱하게 움직이면 BNO055가 기울어져 붙었거나 축이 다른 것이니 장착을 확인한다.
6. 차체 또는 지팡이를 알고 있는 실제 방위로 정확히 맞춘다. 북쪽이면 `imualign 0`, 동쪽이면 `imualign 90`처럼 입력한다.
7. 재부팅 후 `imu`에서 `offsets=1`, `aligned=1`인지 확인한다. 재부팅 직후 A는 낮게 보일 수 있다(오프셋은 적용돼 있음). 지팡이 방향은 G·M이 2 이상이면 나온다.

`imureset`은 저장된 IMU 보정값과 장착 방향 정렬값을 지운다. BNO055 위치, 보드 방향, 케이스, 스피커 또는 자석 위치가 바뀌면 `imureset` 후 다시 수행한다.

차량은 100 Hz BNO055 회전 변화와 GPS 차체방향 보정을 주로 쓰고, 자력계 절대방향은 정렬·보정이 된 뒤 섞는다. 실내에서는 철골·철제 책상 근처에서 방향이 틀어질 수 있으니 시연 자리에서 5번(90° 확인)을 한 번 더 한다. 지팡이는 NDOF 센서융합 방향을 사용한다.

### MG-F10 GPS — 센서 자체 보정은 불필요

1. 두 노드를 하늘이 트인 실외에 둔다.
2. 부팅 로그(USB 시리얼)에서 `[GPS] L5 enable+health override=ACK`와 `5Hz config=ACK`를 확인한다. 펌웨어가 켤 때마다 L5 사용·5 Hz·GGA/RMC를 RAM에 설정한다.
3. 최초 cold fix는 1~2분 이상 기다린다.
4. 뷰어에서 `GPS유효=1`, `GPS위성>=4`, `GPS_HDOP<=3.0~3.5`를 확인한다.
5. 5 Hz인지 확인한다. 정상값은 새 위치 간격 약 200 ms이다.
6. L1과 L5가 함께 잡히는지는 u-center 2의 신호 화면으로만 볼 수 있다. MG-F10에는 USB가 없어서 USB-시리얼 변환기(3.3 V)가 필요하다. 없으면 이 단계는 건너뛰어도 된다.
7. 차량과 지팡이 안테나를 나란히 고정하고 차량 버튼(GPIO33)을 **2초 이상** 누르거나 차량 명령창에 `cal`을 입력한다. 약 8초 동안 움직이지 않는다. 이것은 기존 V4의 두 GPS 상대좌표 영점 보정이다.

안테나 위치, 접지, 케이스 또는 두 GPS 간 설치 간격이 바뀌면 상대좌표 `cal`을 다시 한다.

### UWB — 1점 거리 보정 + 각도 확인

1. C33/UWB Shield와 Stella를 최종 장착 방향으로 세우고 LOS를 확보한다.
2. `uwb status`(= UWB 상태 버튼)로 fresh 값과 샘플 증가, 그리고 `UWB angle valid=... azimuth=... fom=... nlos=...` 줄이 나오는지 확인한다.
3. 안테나 기준점을 줄자로 정확히 3.000 m 떨어뜨려 Stella를 쉴드 정면(0°)에 고정한다.
4. `uwb cal 3`을 실행한다. 100개 샘플(5 Hz 기준 약 20초)을 모을 때까지 움직이지 않는다.
5. 완료 메시지의 오프셋 자동저장을 확인하고 `uwb status`에서 보정 거리 약 3.000 m를 확인한다.
6. **거리:** 1 m, 3 m, 5 m, 8 m에서 각각 30초 기록해 평균오차, 95% 오차, 끊김률을 확인한다.
7. **각도:** 1·3·5 m에서 Stella를 쉴드 기준 0°, ±30°, ±60°에 놓고 각각 30초 기록한다. 먼저 **오른쪽에 둘 때 `UWB각도`가 +인지 −인지** 적어 둔다. 기준(이전 합의): 중앙오차 10° 이하, 튀는 값 5% 이하, 가렸다 뗀 뒤 0.5초 안에 회복.
   - 각도 유효 판단: 최신값 + ±60° 안 + 신뢰도(`UWB각도신뢰`) ≥ `uwbfom`(기본 50) + 가림 아님. 시험 결과를 보고 `uwbfom` 값을 조정하고 `save`한다.
8. 실물 검증이 끝나기 전까지 `uwbrisk 0`을 유지한다. 거리·접근시험이 통과한 뒤에만 `uwbrisk 1` 후 `save`한다. (각도는 아직 위험 판단에 쓰지 않고 화면·기록에만 나온다.)

UWB 안테나 위치/방향이나 케이스를 바꾸면 `uwb reset` 후 다시 보정한다.

## 5. 기능 시험 순서

1. **전원 최대부하:** GPS 2개, BNO055 2개, UWB 2개, Wi-Fi/ESP-NOW, 차량 음성, 지팡이 모터·부저를 함께 켠다. 10분 이상 로그의 `리셋원인`에 `BROWNOUT`이 없어야 한다. 부하 중 각 5 V 레일은 4.5 V 이상을 권장한다. (뷰어가 재부팅을 감지하면 노드 이름 옆에 ⚠ 재부팅 표시가 뜬다.)
2. **출력:** 차량에서 `play 1`~`play 4`, 지팡이에서 `test 1`~`test 3`을 각각 확인한다.
3. **센서 속도:** GPS 5 Hz, UWB 5 Hz, ESP-NOW 상태 송신 10 Hz를 확인한다(뷰어 `통신` 줄의 송신 /s). IMU 읽기가 루프를 막지 않는지도 이걸로 함께 본다.
4. **링크:** 차량/지팡이 `송신`이 증가하고 RSU의 `recv_count`가 증가하며 `lost_count`가 급증하지 않아야 한다.
5. **RSU 위험 왕복:** RSU/Jetson이 0→1→2→3 위험을 보낼 때 차량 음성과 지팡이 진동·부저가 V4와 같은 단계로 동작해야 한다.
6. **안전복귀:** RSU 입력이 끊기면 기존 timeout 뒤 SAFE로 복귀하는지 확인한다.
7. **충격 음성:** 기본은 꺼져 있다(`impact` 300). BNO055는 이 모드에서 가속도가 약 39 m/s²에서 잘려 큰 충격과 거친 노면을 구분하기 어렵다. 시험해 보고 쓸 거면 `impact` 값을 정한 뒤 `save`한다.
8. **주행:** 정면 접근, 평행 통과, 후진, 정지, GPS 순간 끊김, UWB 차폐를 각각 기록한다(아래 상세 기록 모드로).

## 6. 로그 모드

- 기본 `logmode 0`(운영 모드): 운영에 필요한 값만 2 Hz로 보낸다. 채널(6번)을 ESP-NOW와 나눠 쓰므로 평소엔 이게 좋다.
- `logmode 1`: V4의 전체 진단 필드 + V5 추가 값(IMU보정·UWB각도 등)을 보낸다.
- `rate 500`: UDP 로그 주기 500 ms. 센서/제어 주기와 독립이다.
- 뷰어 맨 위 버튼: `운영 모드` = `logmode 0` + `rate 500`, `상세 기록 모드` = `logmode 1` + `rate 100` (두 노드에 같이 보냄).
- **실험을 기록할 때는 `상세 기록 모드`로 바꾼다.** 2 Hz는 화면 보기용이라 분석하기엔 너무 성기다. (브로드캐스트라 받는 쪽에서 일부가 빠질 수 있다. 8월 기록은 10 Hz 중 평균 5.3 Hz만 받았다.)
- 변경을 재부팅 후에도 유지하려면 `save`한다.

## 7. 현재 검증 범위

네 스케치는 해당 보드 코어와 라이브러리로 컴파일을 통과했다(2026-10-01, Claude 재확인: 차량 82%, 지팡이 76%, C33 21%, Stella 54%). 맥 뷰어·웹뷰어는 V5 형식 가짜 데이터로 화면을 확인했다.
실제 부품이 아직 없으므로 전원 안정성, BNO055 축/장착 방향, C33 UWB Shield의 실제 세션 시작, 거리·각도 오차, 각도 부호는 조립 후 위 순서대로 확인해야 한다.
