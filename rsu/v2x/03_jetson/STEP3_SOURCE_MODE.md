# 3단계: V2X 데이터 출처 구분

`source_mode`는 데이터가 어디에서 왔는지를 나타내며 다음 네 값만 사용합니다.

| 값 | 기준 |
|---|---|
| `real` | 실제 GPS 또는 실측 센서 데이터 |
| `test` | 사람이 직접 지정한 고정 테스트 데이터 |
| `fallback` | 실측 실패 시 마지막 정상값 또는 대체 고정값을 사용한 데이터 |
| `simulation` | SUMO, replay 등 시뮬레이션 데이터 |

`gps_valid`는 좌표 필드의 신뢰도를 나타낼 뿐, 데이터의 출처를 뜻하지 않습니다. 출처 판단은 `source_mode`로 합니다.

현재 지팡이는 `gps_valid=0`이면서 좌표는 `37.0/127.0`으로 보냅니다. GPS 실측에 실패하자 펌웨어가 고정 기본값을 대신 넣고 있는 것이므로 `fallback`입니다.

```text
gps_valid=1, 실측 좌표          real
gps_valid=1, 사람이 지정한 좌표  test
gps_valid=0, 대체 고정 좌표      fallback   <- 현재 지팡이 상태
```

## Jetson 실행

기존 시리얼 점유 프로세스를 정리한 다음 실행합니다.

```bash
sudo fuser -k /dev/ttyUSB0
python3 step3_parse_v2x.py --source-mode fallback
```

지팡이 GPS가 실외에서 fix를 잡아 `gps_valid=1`이 되면 `--source-mode real`로 바꿉니다.

기본 CSV 파일은 `step3_v2x_parsed_log.csv`입니다. 다른 출처를 받을 때는 실행 인자를 바꿉니다.

```bash
python3 step3_parse_v2x.py --source-mode real
python3 step3_parse_v2x.py --source-mode fallback
python3 step3_parse_v2x.py --source-mode simulation
```

파서가 처리하는 각 정상 레코드는 출처를 포함해 출력됩니다.

```text
[STATE] type=cane seq=3360 node_risk=0 gps_valid=1 source=test
```

송신 JSON 자체에 유효한 `source_mode`가 있으면 그 값이 실행 인자보다 우선합니다. 따라서 나중에 cane과 vehicle 데이터가 한 스트림에 섞여도 레코드별 출처를 보존할 수 있습니다.

시리얼 장비 없이 한 줄을 시험하려면 다음처럼 표준 입력을 사용합니다.

```bash
echo '{"type":"cane","seq":3360,"gps_valid":1,"node_risk":0}' | python3 step3_parse_v2x.py --stdin --source-mode test
```
