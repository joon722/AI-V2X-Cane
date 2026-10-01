# UWB 각도를 RSU·젯슨까지 보내기 — 팀장 협의용 제안 (2026-10-01)

## 왜 필요한가

V5 차량은 Portenta UWB Shield가 잰 **보행자(Stella) 방향 각도**를 받는다(쉴드 정면 기준 약 ±60°).
하지만 지금 ESP-NOW UWB 패킷(38 B, `MSG_UWB_RANGE=5`)에는 각도 칸이 없고, RSU 브리지도 거리 값만 젯슨에 넘긴다.
그래서 각도는 차량 펌웨어와 뷰어에서만 보이고, 젯슨 화면(drive.html)과 경보에는 쓰이지 않는다.
9/28에 정한 우선순위 ① UWB 거리·각도를 화면에 표시, ② 젯슨 경보를 UWB 기반으로 하려면 아래가 필요하다.

## 제안: 기존 패킷은 그대로 두고 26 B 각도 패킷을 하나 더 보낸다

기존 40/38/35 B는 손대지 않는다. 지금 브리지는 길이가 40·38 B가 아니면 조용히 버리므로, 새 패킷을 먼저 보내도 아무것도 깨지지 않는다.

```c
#define MSG_UWB_AOA 6

typedef struct __attribute__((packed)) v2x_uwb_aoa_message {
  uint32_t magic;          // V2X_MAGIC 0x56325831
  uint8_t  version;        // V2X_VERSION 3
  uint8_t  msg_type;       // MSG_UWB_AOA
  uint8_t  node_type;      // NODE_VEHICLE 0x10
  uint8_t  flags;          // bit0=각도 유효, bit1=거리 보정 완료
  uint32_t src_id;         // 차량 node_id
  uint16_t seq_num;        // 같은 측정의 38 B UWB 패킷과 같은 seq
  uint32_t timestamp_ms;
  float    distance_m;     // 보정 거리
  int16_t  azimuth_cdeg;   // 각도 × 100 (도). +가 어느 쪽인지는 실물 각도시험으로 확정
  uint8_t  azimuth_fom;    // 각도 신뢰도 0~100
  uint8_t  nlos;           // 0=가림 없음, 1=가림, 255=판단 불가
} v2x_uwb_aoa_message_t;   // 26 B
static_assert(sizeof(v2x_uwb_aoa_message_t) == 26, "uwb aoa packet must be 26 bytes");
```

## RSU 쪽 수정 지점 (main `rsu/v2x/v2x_bridge/v2x_bridge.ino` 기준)

1. 위 구조체·`MSG_UWB_AOA` 정의 추가.
2. `onDataRecv()`: 받는 길이에 26 B 추가, 26 B면 `msg_type == MSG_UWB_AOA`만 통과.
   (`rx_item_t.raw`는 40 B라 그대로 담긴다.)
3. `forwardQueuedPackets()`: 26 B면 아래 JSON 한 줄 출력.
   `{"type":"uwb_aoa","node_id":...,"seq":...,"uwb_dist":...,"uwb_az":...,"uwb_az_fom":...,"uwb_nlos":...,"uwb_az_valid":0/1,"tx_ms":...,"rx_ms":...,"rssi":...,"src_mac":"..."}`
4. 젯슨 `step3_parse_v2x.py`: `type == "uwb_aoa"` 줄을 파싱해 같은 seq의 `uwb` 값 옆에 붙인다.
5. `step6`/`drive.html`: 각도가 유효하면 차량 기준 위치를 `forward = d·cos(az)`, `right = ±d·sin(az)`(부호는 실물로 확정)로 쓰고, 무효면 지금처럼 거리만(9/28 합의: 틀린 방향보다 "모름").

## 차량·지팡이 쪽 (합의되면 Claude가 추가)

- 차량: `sendUwbRangePacket()` 바로 뒤에 각도가 있을 때 26 B 패킷 송신(5 Hz, 추가 전송량은 매우 작음).
- 지팡이: 모르는 `msg_type 6`을 조용히 무시하게 처리(안 하면 USB 시리얼에 크기 오류 메시지가 초당 5번 찍힌다).

## 시험 순서

1. 실물 각도시험 통과(1·3·5 m × 0·±30·±60°, 중앙오차 10° 이하, 튀는 값 5% 이하, 가린 뒤 0.5초 안 회복).
2. 브리지 수정 → 젯슨 로그에 `uwb_aoa` 줄이 5 Hz로 찍히는지.
3. drive.html에 방향 표시 → 정면 접근·옆 통과에서 좌우가 맞는지.
