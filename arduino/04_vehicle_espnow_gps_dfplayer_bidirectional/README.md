# 04_vehicle_espnow_gps_dfplayer_bidirectional

차량용 ESP32 코드입니다. 기존 지팡이/수신기 코드를 덮어쓰지 않고 별도 Arduino 스케치로 추가했습니다.

## 기능

- GPS를 GPIO16/17에서 읽음
- ESP-NOW로 지팡이 ESP32에 `v2x_message_t` 패킷을 10Hz 송신
- 지팡이 ESP32에서 돌아오는 응답 패킷을 수신
- GPIO25 부저, GPIO27 등/LED, DFPlayer Mini 스피커 출력 제어
- 시리얼 모니터에서 `0`, `1`, `2`, `3`, `a`, `p`, `v` 명령 테스트 가능

## 배선

```text
[GPS]
GPS VCC -> ESP32 3V3
GPS GND -> ESP32 GND
GPS TX  -> ESP32 GPIO16
GPS RX  -> ESP32 GPIO17

[DFPlayer]
DFPlayer VIN -> ESP32 5V
DFPlayer GND -> ESP32 GND
DFPlayer TX  -> ESP32 GPIO32
DFPlayer RX  -> ESP32 GPIO33

[부저]
부저 SIG -> ESP32 GPIO25
부저 VCC -> 3V3 또는 5V
부저 GND -> GND

[등/LED]
등 SIG -> ESP32 GPIO27
등 VCC -> 3V3 또는 5V
등 GND -> GND

[스피커]
스피커 선 1 -> DFPlayer L+
스피커 선 2 -> DFPlayer L-
```

## 업로드 전 수정

아래 값을 지팡이/수신기 ESP32의 MAC 주소로 바꿔야 합니다.

```cpp
uint8_t caneMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
```

수신기 시리얼 모니터에 보이는 값:

```text
[RX] STA MAC Address: XX:XX:XX:XX:XX:XX
```

예를 들어 `24:6F:28:AA:BB:CC`라면:

```cpp
uint8_t caneMAC[] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};
```

## 테스트

시리얼 모니터는 `115200`으로 엽니다.

```text
p -> 0001.mp3 재생 테스트
0 -> risk 0 고정
1 -> risk 1 고정
2 -> risk 2 고정
3 -> risk 3 고정
a -> 자동 risk 순환
v -> 볼륨 25
```

성공하면 차량 쪽에서는 `[SEND]`, 지팡이 응답이 있으면 `[RX BACK]` 로그가 보입니다.
