#!/bin/bash
# 실시간 위험 로그(logs/*.csv)를 서버로 업로드.
# systemd 타이머(v2x-log-upload.timer)가 5분마다 실행한다.
# 인터넷이 없으면 경고만 남기고 넘어갔다가 다음 주기에 다시 시도한다.
set -u

DIR="$(cd "$(dirname "$0")/.." && pwd)"   # 03_jetson
LOG_DIR="$DIR/logs"

SERVER="${V2X_SERVER:-}"   # 사용자@서버주소. v2x-log-upload.service가 /etc/default/v2x-riskmap 에서 읽음
KEY="$HOME/.ssh/risk_server_key"
REMOTE_DIR="v2x_logs/$(hostname)"
SSH_CMD="ssh -i $KEY -o ConnectTimeout=15 -o StrictHostKeyChecking=accept-new"

if [ -z "$SERVER" ]; then
  echo "[SKIP] V2X_SERVER 미설정 -> /etc/default/v2x-riskmap 에 V2X_SERVER=사용자@서버주소 추가"
  exit 0
fi

if [ ! -d "$LOG_DIR" ] || [ -z "$(ls -A "$LOG_DIR" 2>/dev/null)" ]; then
  echo "[SKIP] 업로드할 로그 없음: $LOG_DIR"
  exit 0
fi

# rsync는 바뀐 부분만 전송하므로 append 중인 CSV도 매번 전체를 다시 보내지 않는다.
# --rsync-path 의 mkdir로 서버 쪽 폴더를 없으면 만든다.
if ! rsync -az -e "$SSH_CMD" \
    --rsync-path="mkdir -p $REMOTE_DIR && rsync" \
    "$LOG_DIR/" "$SERVER:$REMOTE_DIR/"; then
  echo "[WARN] 업로드 실패 (오프라인?) -> 다음 주기에 재시도"
  exit 0
fi

echo "[OK] $(ls "$LOG_DIR" | wc -l)개 파일 동기화됨 -> $SERVER:$REMOTE_DIR"
