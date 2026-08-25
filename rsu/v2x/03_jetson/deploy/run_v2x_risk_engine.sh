#!/bin/bash
# V2X 실시간 위험 엔진 실행 래퍼.
# systemd(v2x-risk-engine.service)가 부팅 시 이 스크립트를 실행한다.
# 하는 일: 포트 대기 -> 포트 점유 정리 -> 타임스탬프 로그 파일 생성 -> step8 실행
set -u

# deploy/ 의 상위 폴더 = 03_jetson
DIR="$(cd "$(dirname "$0")/.." && pwd)"

# udev 규칙이 만들어주는 고정 이름. 없으면 ttyUSB0으로 폴백.
PORT="${V2X_PORT:-/dev/v2x-rsu}"
SOURCE_MODE="${V2X_SOURCE_MODE:-real}"
LOG_DIR="$DIR/logs"
mkdir -p "$LOG_DIR"

# V2X_NO_MODEL=1 이면 규칙만으로 돈다. 실험에서 기준선을 뽑을 때 쓴다 -
# 모델을 켠 채로만 찍으면 경보가 규칙 때문인지 모델 때문인지 구분할 수 없다.
# 변수를 주지 않으면 기존과 동일하게 모델을 싣는다.
NO_MODEL=""
if [ "${V2X_NO_MODEL:-0}" = "1" ]; then
  NO_MODEL="--no-model"
fi

# V2X_NO_FUSION=1 이면 RSSI 거리추세 융합을 끈다. 융합은 v4 펌웨어의 rssi_dist가
# 있어야 발동하지만(구 펌웨어면 자동 OFF), 실기 검증 전까지 실시간 판정에 넣지 않으려면
# 이 변수를 준다. 변수를 빼면 기본값(융합 ON)으로 돈다.
NO_FUSION=""
if [ "${V2X_NO_FUSION:-0}" = "1" ]; then
  NO_FUSION="--no-fusion"
fi

# V2X_NO_UWB=1 이면 UWB 거리 융합·실내 UWB 단독 판정을 끈다. UWB는 브리지 v5의
# type="uwb" 행이 있어야 발동하므로(행이 없으면 자동 무동작) 평소엔 변수가 필요
# 없다 - 비상 차단용이다. 변수를 빼면 기본값(UWB ON)으로 돈다.
NO_UWB=""
if [ "${V2X_NO_UWB:-0}" = "1" ]; then
  NO_UWB="--no-uwb"
fi

# 같은 등급을 지팡이로 재전송하는 주기(초). 낮추면 경보가 더 촘촘히 갱신돼
# 반응이 빨라진다(등급이 바뀔 때는 원래도 즉시 나감). 기본 1.0은 라디오 절약용.
# 예: V2X_HEARTBEAT_S=0.2 → 초당 5회 재전송.
HEARTBEAT_ARG=""
if [ -n "${V2X_HEARTBEAT_S:-}" ]; then
  HEARTBEAT_ARG="--tx-heartbeat-s ${V2X_HEARTBEAT_S}"
fi

# 부팅 직후 USB 인식이 늦을 수 있으므로 최대 2분 대기
for i in $(seq 1 60); do
  [ -e "$PORT" ] && break
  echo "[WAIT] $PORT 없음 ($i/60)"
  sleep 2
done

if [ ! -e "$PORT" ]; then
  echo "[WARN] $PORT 를 찾지 못함 -> /dev/ttyUSB0 으로 폴백"
  PORT=/dev/ttyUSB0
fi

if [ ! -e "$PORT" ]; then
  echo "[ERROR] 시리얼 포트 없음. RSU ESP32 USB 연결을 확인하세요."
  exit 1
fi

# 다른 프로세스(cat, 이전 실행 등)가 포트를 잡고 있으면 정리
fuser -k "$PORT" 2>/dev/null || true
sleep 1

# 세션별 타임스탬프 -> 재시작해도 이전 기록을 덮어쓰지 않음.
# CSV(판단 결과)와 RAW(오간 줄 원문)가 같은 시각을 달고 있어 짝을 찾기 쉽다.
STAMP="$(date +%Y%m%d_%H%M%S)"
CSV="$LOG_DIR/risk_tx_$STAMP.csv"
RAW="$LOG_DIR/raw_$STAMP.log"

echo "[START] port=$PORT csv=$CSV raw=$RAW source_mode=$SOURCE_MODE${NO_MODEL:+ MODEL=off}"
exec python3 "$DIR/step8_send_risk.py" \
  --port "$PORT" \
  --csv "$CSV" \
  --raw-log "$RAW" \
  --source-mode "$SOURCE_MODE" \
  $HEARTBEAT_ARG \
  $NO_MODEL \
  $NO_FUSION \
  $NO_UWB
