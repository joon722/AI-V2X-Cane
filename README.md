# AI-V2X Smart Cane

AI 기반 V2X 협력형 시각장애인 보행 안전 지팡이 프로젝트입니다.

이 저장소는 ESP32 송신기/수신기, Jetson 또는 Python 위험도 처리, 진동/부저 피드백 코드를 정리하기 위한 공간입니다.

## 바로 사용할 코드

Arduino IDE에서 아래 폴더를 각각 열어 업로드합니다.

| 순서 | 폴더 | 역할 | 업로드 대상 |
| --- | --- | --- | --- |
| 1 | `arduino/01_sender_gps_imu_espnow` | GPS/IMU 값을 읽고 ESP-NOW로 송신 | 송신기 ESP32 |
| 2 | `arduino/02_receiver_espnow_uart_actuator` | ESP-NOW 수신, Jetson UART 전달, 진동/부저 기본 제어 | 수신기 ESP32 |
| 3 | `arduino/03_sound_udp_receiver` | Jetson이 보낸 risk UDP 명령으로 진동/부저 패턴 출력 | 피드백용 ESP32 |

## 권장 실행 순서

1. `01_sender_gps_imu_espnow`를 송신기 ESP32에 업로드합니다.
2. `02_receiver_espnow_uart_actuator`를 수신기 ESP32에 업로드합니다.
3. Serial Monitor에서 `[SEND]`, `[RX]` 로그가 증가하는지 확인합니다.
4. Jetson 또는 PC에서 위험도 `risk`를 계산합니다.
5. `03_sound_udp_receiver`를 사용해 `{"risk":0}`, `{"risk":1}`, `{"risk":2}`, `{"risk":3}` 명령에 따른 진동/부저 패턴을 확인합니다.

## Wi-Fi 설정 방법

`03_sound_udp_receiver`는 Wi-Fi UDP를 사용하므로 실제 Wi-Fi 이름과 비밀번호가 필요합니다.

1. `arduino/03_sound_udp_receiver/config.example.h`를 같은 폴더에 복사합니다.
2. 복사한 파일 이름을 `config.h`로 바꿉니다.
3. 아래처럼 값을 채웁니다.

```cpp
#pragma once

const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
```

`config.h`는 실제 비밀번호가 들어가므로 GitHub에 올리지 않습니다. `.gitignore`에 등록되어 있습니다.

## 핀 요약

### Sender ESP32

| 기능 | 핀 |
| --- | --- |
| IMU SDA | GPIO21 |
| IMU SCL | GPIO22 |
| GPS RX | GPIO16 |
| GPS TX | GPIO17 |

### Receiver ESP32

| 기능 | 핀 |
| --- | --- |
| Jetson RX | GPIO16 |
| Jetson TX | GPIO17 |
| Buzzer | GPIO25 |
| Vibration Motor | GPIO26 |

### Sound UDP Receiver

| 기능 | 핀 |
| --- | --- |
| Vibration Motor | GPIO25 |
| Buzzer | GPIO27 |

## 참고 문서

- `docs/코드_검증_정리.md`
- `docs/수행계획서/5.10 Project Implementation Plan.pdf`
- `docs/수행계획서/5.17 5차_수행계획서_최종.pdf`

## 현재 확인된 상태

- GPS/IMU ESP-NOW 송신 코드 있음
- ESP-NOW 수신 및 Jetson UART 전달 코드 있음
- risk 0-3에 따른 진동/부저 패턴 코드 있음
- DFPlayer Mini 기반 실제 음성 안내 코드는 아직 없음
- Arduino 실제 컴파일 검증은 Arduino IDE 또는 arduino-cli 환경에서 추가 확인 필요
