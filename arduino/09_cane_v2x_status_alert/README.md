# 09_cane_v2x_status_alert

지팡이 V2X 단말 코드입니다.

## 역할

- 지팡이 ID/위치/risk 상태를 ESP-NOW broadcast로 10Hz 송신
- RSU/Jetson이 보낸 risk alert를 ESP-NOW로 수신
- risk에 따라 진동모터, 부저, DFPlayer 음성 출력

## 한 줄 구조

```text
Cane ESP32 --ESP-NOW--> RSU Bridge ESP32 --USB--> Jetson
Cane ESP32 <--ESP-NOW-- RSU Bridge ESP32 <--USB-- Jetson risk
```

## 기본 핀

```text
BUZZER_PIN = GPIO25
MOTOR_PIN  = GPIO26
DFPlayer TX -> ESP32 GPIO32
DFPlayer RX -> ESP32 GPIO33
```

## 테스트 전 수정

현재는 고정 좌표를 사용합니다.

```cpp
#define CANE_FIXED_LAT 37.000000
#define CANE_FIXED_LNG 127.000000
```

실험 위치 좌표로 바꾼 뒤 업로드하면 Jetson 거리 계산 테스트가 쉬워집니다.
