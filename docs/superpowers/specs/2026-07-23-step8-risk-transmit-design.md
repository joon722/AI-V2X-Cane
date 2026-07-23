# 8단계 설계: risk_level을 RSU/ESP32로 전송

날짜: 2026-07-23
대상: `step8_send_risk.py` (신규), `test_step8_send_risk.py` (신규)

## 목표

step7이 계산한 `risk_level`(0~3)을 RSU로 다운링크한다. 명령 형식은 이미 검증된
`{"target_id":0,"risk":N}` (probe_risk_downlink.py, 7/7 반영). RSU는 `risk_broadcast_to_seen`으로
자기가 본 노드에 뿌리고, 지팡이는 `node_risk`로 받는다.

## 파일 구조 규칙

step2~7은 고치지 않고 import만 한다. step7의 채점(`KinematicsPipeline`, `assess_risk`)을
재사용하되, step7의 `process_line`은 bool을 반환하고 전송값을 안 내주므로 step8은 자체
루프에서 `store.update → observe → compute → assess_risk`를 직접 부른다(~8줄 중복은 규칙상 감수,
step7이 step6에서 그랬던 것과 같다).

## 시리얼: 한 포트로 읽으며 쓰기

`serial_lines`(step3)는 포트를 내부에서 열고 줄만 yield해서 connection을 안 내준다. 전송은
같은 `/dev/ttyUSB0`에 write해야 하므로 step8은 probe처럼 자기 `serial.Serial(port, baud,
timeout=1)`을 열어 read와 write를 모두 한다. `--stdin` 모드에서는 시리얼 대신 stdin을 읽고
전송은 `[TX]`를 stderr로 찍어 하드웨어 없이 검증한다.

전송은 콜러블 하나로 추상화한다: 시리얼 모드 → `conn.write`, stdin 모드 → stderr 출력.

## 전송 정책: 순수 함수로 분리

시리얼과 무관한 `RiskTransmitter`가 "지금 이 level을 보낼까"를 결정한다. 그래야 하드웨어
없이 단위테스트가 된다.

```
consider(computed_level, cane_gps_valid, now) -> TxDecision
```

### 트러스트 게이팅 (요약이 짚어둔 결정)

cane `gps_valid=0`이면 fallback 좌표라 계산된 risk가 실제 위치와 무관하다. 그대로 전송하면
지팡이가 근거 없이 진동한다.

```
trusted = allow_untrusted or gps_valid==1
effective_level = computed_level if trusted else 0
```

- 기본값(`allow_untrusted=False`): `gps_valid=0`이면 nonzero risk를 막고 **0만** 보낼 수 있다.
  0을 보내는 건 이전 trusted 경보를 지팡이에서 해제하기 위함이다(진동 멈춤).
- `--tx-untrusted`: 실내 개발에서 fallback 좌표로도 end-to-end 진동을 확인하고 싶을 때만 켠다.
  운영에서는 끈 채로 둔다.

### 변화시 즉시 + 주기 heartbeat

cane이 ~10Hz라 매 레코드 전송은 RSU를 홍수낸다. `effective_level` 기준으로:

```
last_level 없음 또는 effective != last_level → 즉시 전송 (reason=change)
그 외 now - last_send >= heartbeat_s        → 재전송   (reason=heartbeat)
그 외                                        → 보류     (reason=hold)
```

heartbeat(기본 1.0s, `--tx-heartbeat-s`)는 다운링크 패킷이 드롭돼도 지팡이 상태가 최신으로
유지되게 한다. level이 0일 때도 heartbeat를 보내 "안전" 상태를 갱신한다(경보 해제가 드롭되면
지팡이가 계속 진동하는 것을 막는다).

## target_id

기본 0 = 브로드캐스트(RSU가 본 모든 노드). 지금은 cane만 있으니 0으로 충분. 노드별로 다른
값을 주려면 `--target-id`에 실제 node_id 지정(9단계 실차 붙을 때).

## 출력 / CSV

- 콘솔: 전송할 때 `[TX] risk=N target=0 reason=change`. 트러스트 억제로 값이 눌렸으면
  `computed=2->0` 형태로 표시.
- `step8_risk_tx_log.csv`: `pc_time, cane_seq, computed_level, effective_level, trusted,
  reason, target_id`. 전송된 이벤트만 남긴다(10Hz 고려 매 계산 로깅은 안 함).

## 테스트 (`test_step8_send_risk.py`)

- 첫 전송(change), heartbeat 내 보류(hold), heartbeat 경과 재전송(heartbeat), level 변화 즉시 전송.
- 트러스트 억제: gps_valid=0 → effective 0 / 이전 trusted 경보(2)가 gps 끊기면 0으로 change(해제).
- `allow_untrusted=True` → 그대로 통과.
- 명령 형식 `{"target_id":0,"risk":2}` 정확성.

## 검증 계획

1. `python3 -m unittest discover -p "test_*.py"` — 기존 75 + 신규 통과.
2. 인프로세스 스모크: 접근하며 level 상승 시 change 전송, 정지 구간은 heartbeat만.
3. Jetson 실기: `scp step8_send_risk.py test_step8_send_risk.py ssu212324@192.168.55.1:~/v2x/03_jetson/`
   - `python3 step8_send_risk.py --source-mode fallback --test-vehicle --tx-untrusted`
     (실내라 gps_valid=0이므로 개방 플래그 필요) → 접근에 따라 `[TX] risk=…`가 나가고 지팡이
     시리얼 모니터의 `[CANE RX] risk=…` / node_risk가 따라오는지 확인.
   - 플래그 없이 돌리면 gps_valid=0이라 0만 나가는 것도 확인(억제 동작).
