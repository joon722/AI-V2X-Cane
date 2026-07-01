# 09_cane_v2x_status_alert

지팡이 V2X 단말 코드입니다. `01_sender_gps_imu_espnow`의 실제 지팡이 하드웨어 구성에 맞춰 GPS, IMU, 진동모터, 삐 소리 부저만 사용합니다. DFPlayer는 사용하지 않습니다.

## 역할

- 지팡이 ID/GPS 위치/risk 상태를 ESP-NOW broadcast로 10Hz 송신
- ICM-20948 IMU 값을 읽어 시리얼 디버그에 표시
- RSU/Jetson이 보낸 risk alert를 ESP-NOW로 수신
- risk에 따라 진동모터와 삐 소리 부저 출력

## 한 줄 구조

```text
Cane ESP32 --ESP-NOW--> RSU Bridge ESP32 --USB--> Jetson
Cane ESP32 <--ESP-NOW-- RSU Bridge ESP32 <--USB-- Jetson risk
```

## 기본 핀

```text
[GPS]
GPS TX -> ESP32 GPIO16
GPS RX -> ESP32 GPIO17

[IMU ICM-20948]
SDA -> ESP32 GPIO21
SCL -> ESP32 GPIO22

[Actuator]
BUZZER_PIN = GPIO25  // Active LOW
MOTOR_PIN  = GPIO26  // Active HIGH
```

## 테스트 전 수정

GPS가 아직 fix되지 않았을 때는 fallback 좌표를 사용합니다.

```cpp
#define CANE_FIXED_LAT 37.000000
#define CANE_FIXED_LNG 127.000000
```

실험 위치 좌표로 바꾼 뒤 업로드하면 Jetson 거리 계산 테스트가 쉬워집니다. 실제 GPS가 잡히면 GPS 좌표가 우선 사용됩니다.
