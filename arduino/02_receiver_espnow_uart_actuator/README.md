# 02 Receiver ESP-NOW UART Actuator

수신기 ESP32에 업로드하는 코드입니다.

## 기능

- 송신기 ESP32의 ESP-NOW 패킷 수신
- 패킷 크기 검증
- Jetson UART로 JSON 한 줄씩 전달
- risk 값에 따라 진동모터/부저 기본 제어

## 확인 방법

Serial Monitor를 `115200` baud로 열고 아래 로그가 보이는지 확인합니다.

```text
[RX] seq=...
```

Jetson 쪽에서는 UART로 JSON 라인이 들어오는지 확인합니다.

## 주의

이 코드는 기본 부저 제어만 포함합니다. DFPlayer Mini 음성 재생은 아직 포함되어 있지 않습니다.
