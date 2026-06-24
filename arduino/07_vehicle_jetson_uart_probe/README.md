# 07_vehicle_jetson_uart_probe

차량 ESP32 단독 테스트용 코드입니다. 지팡이 ESP32가 아직 준비되지 않아도 Jetson UART 입력과 risk 회신 흐름을 먼저 확인할 수 있습니다.

## 역할

- 차량 상태 JSON을 10Hz로 생성
- USB Serial Monitor와 Jetson UART에 같은 JSON 전송
- ESP-NOW broadcast도 함께 송신
- Jetson이 `{"risk":2}` 같은 줄을 보내면 ESP32 Serial Monitor에 표시

## 배선

```text
ESP32 GPIO33 TX1 -> Jetson RX
ESP32 GPIO32 RX1 <- Jetson TX
ESP32 GND        -> Jetson GND
```

Baud rate: `115200`

## 기본 동작

GPS 없이도 바로 테스트할 수 있게 기본값은 가상 좌표입니다.

```cpp
#define USE_SIMULATED_GPS 1
#define START_LAT 37.000000
#define START_LNG 127.000000
```

Jetson에는 이런 JSON이 들어갑니다.

```json
{"type":"vehicle","node_id":123,"seq":1,"gps_valid":1,"lat":37.000000,"lng":127.000010,"speed_mps":2.100,"heading_deg":90.00,"tx_ms":1000,"espnow":"ok"}
```

