# 10_rsu_espnow_multi_jetson_bridge

차량 ESP와 지팡이 ESP를 모두 받는 RSU 브리지 코드입니다.

## 역할

- 차량 상태 패킷 수신 후 Jetson에 JSON 출력
- 지팡이 상태 패킷 수신 후 Jetson에 JSON 출력
- Jetson이 보낸 risk JSON을 대상 단말에 ESP-NOW로 전송

## 구조

```text
Vehicle ESP --ESP-NOW--> RSU Bridge --USB Serial--> Jetson
Cane ESP    --ESP-NOW--> RSU Bridge --USB Serial--> Jetson
Vehicle ESP <--ESP-NOW-- RSU Bridge <--USB Serial-- Jetson risk
Cane ESP    <--ESP-NOW-- RSU Bridge <--USB Serial-- Jetson risk
```

## Jetson으로 나가는 JSON

```json
{"type":"vehicle","node_id":123,"seq":1,"lat":37.0,"lng":127.0,"speed_mps":2.0}
{"type":"cane","node_id":456,"seq":1,"lat":37.0,"lng":127.0,"speed_mps":0.0}
```

## Jetson에서 받는 risk JSON

전체 단말에 같은 risk:

```json
{"risk":2}
```

특정 단말에 risk:

```json
{"target_id":456,"src_id":123,"risk":2}
```

`target_id`는 경고를 받을 단말, `src_id`는 위험 원인이 된 상대 단말입니다.
