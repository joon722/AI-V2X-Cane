# V5 펌웨어 업로드·조립 후 점검 순서

## 1. V5 구성과 V4 호환성

V5는 센서와 UWB 하드웨어만 교체하고, 기존 동작과 V2X 프로토콜은 유지한다.

| 보드 | 업로드할 스케치 | 역할 |
|---|---|---|
| 차량 ESP32 | `car_bt_debug/car_debug_V5.ino` | GPS/IMU/오디오/ESP-NOW/RSU 위험 수신 |
| 지팡이 ESP32 | `cane_bt_debug/cane_debug_V5.ino` | GPS/IMU/ESP-NOW/진동·부저 |
| Portenta C33 + UWB Shield | `c33_uwb_bridge_V5/c33_uwb_bridge_V5.ino` | 5 Hz DS-TWR controller, 거리를 차량 ESP32 UART로 전달 |
| Arduino Stella | `stella_uwb_responder_V5/stella_uwb_responder_V5.ino` | 5 Hz DS-TWR responder |

V2X 구조체는 V4와 바이트 단위로 같다.

- `V2X_MAGIC = 0x56325831`
- `V2X_VERSION = 3`
- 상태 패킷 40 B
- UWB 패킷 38 B (`MSG_UWB_RANGE = 5`)
- 위험 패킷 35 B
- 새 크기의 V2X 패킷 없음

`origin/main` 커밋 `9837a07`의 `rsu/v2x/v2x_bridge/v2x_bridge.ino`가 이미 40 B 상태와 38 B UWB 패킷을 처리하므로 RSU/Jetson 수정은 필요 없다.

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

PortentaUWBShield를 GitHub ZIP으로 설치했는데 `undefined reference to UWBHAL`이 나오면, 라이브러리의 선컴파일 아카이브 폴더 이름과 설치된 C33 core의 FPU 폴더 이름이 맞지 않는 경우다. Arduino Library Manager의 정식 패키지를 우선 사용한다. 같은 오류가 재현되면 `src/cortex-m33/fpv5-sp-d16-hard/libUWBShieldApi.a`를 컴파일 로그가 요구하는 `src/cortex-m33/<요구된 폴더>/`에도 복사한 뒤 다시 빌드한다.

## 3. 업로드 순서와 배선 확인

1. Stella에 responder V5를 업로드하고 USB 전원을 유지한다.
2. Portenta C33에 controller V5를 업로드하고 UWB Shield를 끝까지 결합한다.
3. C33 `D14/TX`를 차량 ESP32 `GPIO32/RX1`에 연결하고 GND를 공통으로 묶는다.
4. 차량·지팡이 ESP32에 각각 V5를 업로드한다.
5. 두 MG-F10은 `TX→GPIO16`, `RX←GPIO17`, 115200 bps로 연결한다.
6. 두 BNO055는 3.3 V, `SDA→GPIO21`, `SCL→GPIO22`, `COM3/I2C_SEL→GND`로 연결한다. 주소는 0x28이다.
7. 차량 DFPlayer RX는 GPIO27, 차량/지팡이 버튼은 GPIO33이다.

안테나와 UWB 보드는 최종 케이스 방향으로 고정한 후 보정한다. 보정 뒤 위치나 방향을 바꾸면 다시 보정한다.

## 4. 최초 보정

### BNO055 — 필요함

차량과 지팡이 각각 다음을 수행한다.

1. 완성된 케이스 상태로 전원을 켠다. 차량은 스피커와 DFPlayer도 평소처럼 켠다.
2. 뷰어 명령창에서 `imu`를 입력한다.
3. 정지 상태 유지, 천천히 3축 회전, 8자 회전을 반복해 `S/G/A/M=3/3/3/3`을 만든다.
4. `imusave`로 오프셋을 NVS에 저장한다.
5. 차체 또는 지팡이를 알고 있는 실제 방위로 정확히 맞춘다. 북쪽이면 `imualign 0`, 동쪽이면 `imualign 90`처럼 입력한다.
6. 재부팅 후 `imu`에서 `offsets=1`, `aligned=1`인지 확인한다.

`imureset`은 저장된 IMU 보정값과 장착 방향 정렬값을 지운다. BNO055 위치, 보드 방향, 케이스, 스피커 또는 자석 위치가 바뀌면 `imureset` 후 다시 수행한다.

차량은 스피커 자석 간섭을 줄이기 위해 100 Hz BNO055 회전 변화와 GPS 차체방향 보정을 주로 사용하고, 자력계 절대방향은 느린 보정에만 사용한다. 지팡이는 NDOF 센서융합 방향을 사용한다.

### MG-F10 GPS — 센서 자체 보정은 불필요

GPS는 IMU처럼 사용자가 오프셋을 보정하지 않는다. 대신 다음 수신 검증이 필요하다.

1. 두 노드를 하늘이 트인 실외에 둔다.
2. 최초 cold fix는 1~2분 이상 기다린다.
3. 로그에서 `GPS유효=1`, `GPS위성>=4`, `GPS_HDOP<=3.0~3.5`를 확인한다.
4. 5 Hz인지 확인한다. 정상값은 새 위치 간격 약 200 ms이다.
5. 가능하면 u-center 2의 신호 화면에서 L1과 L5 신호가 함께 잡히는지 확인한다.
6. 차량과 지팡이 안테나를 나란히 고정하고 차량 버튼(GPIO33)을 누르거나 차량 명령창에 `cal`을 입력한다. 약 8초 동안 움직이지 않는다. 이것은 기존 V4의 두 GPS 상대좌표 영점 보정이다.

안테나 위치, 접지, 케이스 또는 두 GPS 간 설치 간격이 바뀌면 상대좌표 `cal`을 다시 한다.

### UWB — 1점 거리 보정 필요

1. C33/UWB Shield와 Stella를 최종 장착 방향으로 세우고 LOS를 확보한다.
2. 차량 명령창에서 `uwb status`를 실행해 fresh 값과 샘플 증가를 확인한다.
3. 안테나 기준점을 줄자로 정확히 3.000 m 떨어뜨려 고정한다.
4. `uwb cal 3`을 실행한다. 100개 샘플(5 Hz 기준 약 20초)을 모을 때까지 움직이지 않는다.
5. 완료 메시지의 오프셋 자동저장을 확인하고 `uwb status`에서 보정 거리 약 3.000 m를 확인한다.
6. 1 m, 3 m, 5 m, 8 m에서 각각 30초 기록해 평균오차, 95% 오차, 끊김률을 확인한다.
7. 실물 검증이 끝나기 전까지 `uwbrisk 0`을 유지한다. 거리·접근시험이 통과한 뒤에만 `uwbrisk 1` 후 `save`한다.

UWB 안테나 위치/방향이나 케이스를 바꾸면 `uwb reset` 후 다시 보정한다.

## 5. 기능 시험 순서

1. **전원 최대부하:** GPS 2개, BNO055 2개, UWB 2개, Wi-Fi/ESP-NOW, 차량 음성, 지팡이 모터·부저를 함께 켠다. 10분 이상 로그의 `리셋원인`에 `BROWNOUT`이 없어야 한다. 부하 중 각 5 V 레일은 4.5 V 이상을 권장한다.
2. **출력:** 차량에서 `play 1`~`play 4`, 지팡이에서 `test 1`~`test 3`을 각각 확인한다.
3. **센서 속도:** BNO055 내부 읽기 100 Hz, GPS 5 Hz, UWB 5 Hz, ESP-NOW 상태 송신 10 Hz를 확인한다.
4. **링크:** 차량/지팡이 `송신`이 증가하고 RSU의 `recv_count`가 증가하며 `lost_count`가 급증하지 않아야 한다.
5. **RSU 위험 왕복:** RSU/Jetson이 0→1→2→3 위험을 보낼 때 차량 음성과 지팡이 진동·부저가 V4와 같은 단계로 동작해야 한다.
6. **안전복귀:** RSU 입력이 끊기면 기존 timeout 뒤 SAFE로 복귀하는지 확인한다.
7. **주행:** 정면 접근, 평행 통과, 후진, 정지, GPS 순간 끊김, UWB 차폐를 각각 기록한다.

## 6. 로그 모드

- 기본 `logmode 0`: 기존 이름 중 운영에 필요한 값만 2 Hz로 보낸다.
- `logmode 1`: V4의 전체 진단 필드를 보낸다.
- `rate 500`: UDP 로그 주기 500 ms. 센서/제어 주기와 독립이다.
- 변경을 재부팅 후에도 유지하려면 `save`한다.

평소에는 `logmode 0`, 보정이나 원인 분석 때만 `logmode 1`을 사용한다.

## 7. 현재 검증 범위

네 스케치는 해당 보드 코어와 공식 라이브러리로 컴파일을 통과했다. 실제 부품이 아직 없으므로 전원 안정성, BNO055 축/장착 방향, C33 UWB Shield의 실제 세션 시작, RF 거리 오차는 조립 후 위 순서대로 확인해야 한다.
