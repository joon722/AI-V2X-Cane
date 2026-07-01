# 01 Sender GPS IMU ESP-NOW

송신기 ESP32에 업로드하는 코드입니다.

## 기능

- ICM-20948 IMU 읽기
- Neo-6M GPS 읽기
- GPS/IMU 값을 `v2x_message_t` 구조체에 담기
- ESP-NOW로 수신기 MAC 주소에 100ms마다 전송

## 확인 방법

Serial Monitor를 `115200` baud로 열고 아래 로그가 증가하는지 확인합니다.

```text
[SEND] seq=...
[ESP-NOW TX] seq=... result=OK
```

## 주의

수신기 ESP32의 MAC 주소가 바뀌면 코드 안의 `receiverMAC` 값을 수정해야 합니다.
