# Cloud 시뮬레이션 생성 서버 (Google Cloud VM: risk-server)

SUMO 시나리오를 자동으로 무한 생성하고, Jetson이 가져갈 feature 데이터를 만드는 서버 파이프라인.
서버 위치: `~/SUMO_project/` (Ubuntu 24.04, SUMO 1.27.1)

## 역할 분리 원칙

- **서버**: SUMO 실행 → fcd.xml → feature.csv 생성**까지만** 수행
- **Jetson**: TTC·risk_level 계산, Transformer ONNX 추론 (서버에서는 하지 않음)

## 파일 설명

| 파일 | 역할 |
|---|---|
| `generate_scenarios.py` | SUMO 자동 반복 실행. 시드를 바꿔 매번 다른 교통 흐름 생성 → `generated_data/scenario_NNNN/feature.csv`(차량) + `pedestrian.csv`(보행자) + `DONE` 마커. 실패 시 백오프 재시도, 디스크 여유 부족 시 자동 일시정지 |
| `run_generator.sh` | 실행 래퍼. cron이 1분마다 호출 — flock으로 중복 방지, 죽어 있으면 재시작 |
| `cleanup_server.py` | 디스크 자동 정리 (10분마다). Jetson이 결과까지 업로드한 시나리오 삭제(최신 30개 보존), 1시간 지난 결과 gzip, 14일 지난 결과 삭제. 원본은 Jetson에 보관됨 |
| `build_map_data.py` | 위험지도 데이터 추출 (30분마다). Jetson 추론 결과를 10m 격자로 집계, SUMO 좌표→위경도 변환(sumolib+pyproj) → `map_data/risk_map_data.json`. 실측 혼합: `--real-csv 파일` (가중치 실측 3 : 가상 1) |
| `jetson_fetch.sh` | Jetson에서 실행하는 수신 스크립트 (참고용 사본) |

## 서버 crontab 설정

```
* * * * *   ~/SUMO_project/run_generator.sh --interval 60 --min-free-gb 1.5
*/10 * * * * cd ~/SUMO_project && python3 cleanup_server.py
*/30 * * * * cd ~/SUMO_project && .venv/bin/python build_map_data.py
```

## 데이터 흐름

```
서버: SUMO 생성 (1분/개) → generated_data/
  ↓ Jetson이 rsync로 pull (1분마다)
Jetson: feature 계산 → ONNX 추론 (~2초/시나리오) → 결과 업로드
  ↓ scp
서버: results/ → build_map_data.py → 위험지도 JSON
```
