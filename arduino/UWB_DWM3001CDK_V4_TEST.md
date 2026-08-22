# DWM3001CDK + V4 장착 직후 시험

## 이번 코드의 동작

- 차량의 GPS+IMU 및 지팡이 GPS로 상대 방향을 유지한다.
- 차량 쪽 DWM3001CDK가 측정한 거리를 5개 중앙값과 저역통과 필터로 안정화한다.
- 보정이 끝난 최신 UWB 거리가 있으면 GPS 상대벡터의 길이를 UWB 거리로 맞춘다.
- V4 상태 패킷의 `rssi_distance_m`에는 최신 보정 UWB가 우선 실리고, UWB가 끊기면 기존 RSSI 거리로 자동 복귀한다.
- 원시거리, 보정거리, 접근속도, 오프셋은 `MSG_UWB_RANGE` 상세 패킷으로 지팡이에 함께 보낸다.
- UWB가 아직 보정되지 않았을 때는 기존 위험 판정을 바꾸지 않는다.

## DWM3001CDK 두 대 준비

1. 두 CDK에 같은 버전의 Qorvo CLI 펌웨어를 올린다.
2. User USB(J20) 시리얼 터미널에서 `help`를 확인한다.
3. 지팡이 CDK는 `RESPF`, 차량 CDK는 `INITF`로 같은 채널·세션 설정을 사용해 연속 DS-TWR을 시작한다.
4. 거리 출력이 계속 나오는 것을 확인한 뒤 각 CDK에서 `save`하여 보조배터리 전원만 넣어도 같은 역할로 시작하게 한다.
5. 정상 출력에는 다음 필드가 있어야 한다.

   ```text
   status="SUCCESS", distance[cm]=150, RSSI[dBm]=-45.5
   ```

CLI 명령 인자는 펌웨어 버전마다 다를 수 있으므로 해당 펌웨어의 `help INITF`, `help RESPF` 결과를 우선한다.

## 차량 배선

- 차량 CDK의 CLI UART TX → 차량 ESP32 GPIO32(RX1)
- 차량 CDK GND → 차량 ESP32 GND
- CDK 전원 → User USB 또는 별도 5V 보조배터리
- ESP32 GPIO에는 5V 전원을 연결하지 않는다.
- 기존 DFPlayer Pro는 ESP32 GPIO27(TX1) 연결을 유지한다.

UART1은 방향을 나눠 사용한다. RX(GPIO32)는 CDK 거리 출력만 받고, TX(GPIO27)는 DFPlayer 명령만 보낸다. 두 장치의 속도는 115200 bps다.

지팡이 CDK는 Responder로 켜서 지팡이에 장착하면 된다. 현재 지팡이 ESP32와 별도 UART 연결은 필요하지 않고, 차량이 측정한 UWB 상세값을 ESP-NOW로 받는다.

## 업로드 후 첫 확인

차량 USB 시리얼 모니터를 115200 bps로 열고 다음을 입력한다.

```text
uwb status
```

`samples`가 계속 증가하고 `fresh=1`이면 배선과 ranging 출력이 정상이다. 이 단계에서는 아직 UWB를 거리 보정에 사용하지 않는다.

## 1점 오프셋 보정

1. 두 CDK 안테나 중심 사이를 정확히 3.000 m로 맞춘다.
2. 안테나 방향과 높이를 같게 하고 사람·금속·벽을 가까이 두지 않는다.
3. 차량 시리얼 또는 차량 웹뷰어 명령창에서 입력한다.

   ```text
   uwb cal 3
   ```

4. 움직이지 않고 성공 샘플 100개가 모일 때까지 기다린다.
5. `UWB 보정 완료`와 저장된 오프셋을 확인한다.
6. 다시 `uwb status`를 입력해 `calibrated=1`, `fresh=1`인지 확인한다.

보정이 완료되는 순간부터 V4 근접거리 필드가 UWB 값을 자동 사용한다. UWB가 750 ms 넘게 끊기면 RSSI로 자동 복귀한다.

차량 자체 UWB 위험 계산까지 따로 시험할 때만 다음을 실행한다. 현재 RSU 단독 위험 판정 모드에서는 필수가 아니다.

```text
uwbrisk 1
save
```

원래 판정으로 되돌리려면 `uwbrisk 0`, 모든 UWB 보정값을 지우려면 `uwb reset`을 사용한다.

## 거리별 검증

0.5 m, 1 m, 2 m, 3 m, 5 m에서 각 20초 이상 정지 측정한다. 차량 로그에서 아래 값을 확인한다.

- `UWB유효:1`
- `UWB원시거리`
- `UWB보정거리`
- `UWB접근속도`
- `UWB경과ms`
- `UWB실패`
- `UWB보정:1`

지팡이 로그에서는 `UWB수신`이 계속 증가해야 한다. 일정한 거리에서 접근속도가 0 m/s 부근이고 보정거리 오차가 거의 일정하면 정상이다.

## 값이 안 들어올 때

- 차량 CDK가 Initiator이고 지팡이 CDK가 Responder인지 확인한다.
- 두 CDK의 채널, 세션 ID, 주소 설정이 서로 맞는지 확인한다.
- CDK를 PC에 연결했을 때 `distance[cm]` 성공 로그가 실제로 반복되는지 확인한다.
- CLI의 UART 출력이 활성화됐는지 확인한다.
- CDK TX와 ESP32 GPIO32가 연결됐는지, GND가 공통인지 확인한다.
- `uwb status`의 `samples=0`이면 UART/CLI 출력 문제이고, `fail`만 증가하면 ranging 실패 또는 출력 형식 문제다.
- 사람이나 금속이 안테나 바로 앞에 있으면 NLOS 오차가 커질 수 있다.
