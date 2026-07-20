# Arduino Code Map

Arduino IDE에서 각 폴더를 열어 업로드합니다. 폴더 이름과 `.ino` 파일 이름이 같아야 Arduino 스케치로 인식됩니다.

## 01_sender_gps_imu_espnow

송신기 ESP32용 코드입니다.

- GPS Neo-6M 데이터 수집
- ICM-20948 IMU 데이터 수집
- ESP-NOW로 수신기 ESP32에 `v2x_message_t` 패킷 송신
- 100ms 주기 송신

## 02_receiver_espnow_uart_actuator

수신기 ESP32용 코드입니다.

- ESP-NOW 패킷 수신
- 수신 데이터를 Jetson UART로 JSON 전달
- risk 값에 따라 진동모터/부저 기본 제어

## 03_sound_udp_receiver

Jetson 위험도 출력 이후 피드백 확인용 코드입니다.

- Wi-Fi 연결
- UDP 포트 `6001`에서 명령 수신
- 명령 예시: `{"risk":2}`
- risk 0: 전체 OFF
- risk 1: 약한 진동
- risk 2: 약한 진동 + 느린 부저
- risk 3: 강한 진동 + 3연속 부저

업로드 전 `config.example.h`를 복사해서 `config.h`를 만들고 Wi-Fi 정보를 입력해야 합니다.

## 09_vehicle_v2x_direct_risk_alert

Vehicle-side direct ESP-NOW risk test sketch.

- Vehicle ESP32 broadcasts `MSG_VEHICLE_STATUS`.
- Vehicle ESP32 receives `MSG_CANE_STATUS`.
- Vehicle ESP32 calculates simple distance/TTC risk locally.
- Vehicle ESP32 sends `MSG_RISK_ALERT` directly to the cane ESP32.
- Use `06_vehicle_v2x_gps_broadcast` + `08_rsu_espnow_jetson_bridge` for the Jetson-centered flow.

## 11_cane_v2x_status_alert

Cane-side endpoint for the current `10_rsu_espnow_multi_jetson_bridge` protocol.

- Cane ESP32 broadcasts `MSG_CANE_STATUS`.
- Cane ESP32 receives `MSG_VEHICLE_STATUS`.
- Cane ESP32 calculates fallback `local_risk`.
- Cane ESP32 receives `MSG_RISK_ALERT` as `remote_risk`.
- Output risk is `final_risk = max(local_risk, remote_risk)`.

## 99_legacy_originals

정리 전 원본 코드 보관 폴더입니다.

## 99_legacy_tests

초기 테스트 코드 보관 폴더입니다. 일부 파일은 한 파일에 송신기/수신기 코드가 같이 있어 그대로 업로드하면 안 될 수 있습니다.
