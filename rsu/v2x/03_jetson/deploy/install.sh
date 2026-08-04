#!/bin/bash
# 젯슨에서 한 번만 실행: sudo ./install.sh
# 하는 일: udev 규칙 설치 -> systemd 서비스 등록/시작 -> 자동 시작 활성화
set -e

if [ "$(id -u)" -ne 0 ]; then
  echo "sudo로 실행하세요: sudo ./install.sh"
  exit 1
fi

DIR="$(cd "$(dirname "$0")" && pwd)"

chmod +x "$DIR/run_v2x_risk_engine.sh" "$DIR/upload_logs.sh"

# sudo로 실행해도 서비스는 원래 로그인 사용자로 돌아야 함 (SSH 키 위치 때문)
RUN_USER="${SUDO_USER:-$(logname)}"

# pyserial 확인 (step8 실행에 필요)
if ! python3 -c "import serial" 2>/dev/null; then
  echo "[INFO] pyserial 설치 중..."
  pip3 install pyserial
fi

# 1. udev 규칙: /dev/v2x-rsu 고정 이름
cp "$DIR/99-v2x-rsu.rules" /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger
echo "[OK] udev 규칙 설치됨 (/dev/v2x-rsu)"

# 2. systemd 서비스 등록
sed "s|@DEPLOY_DIR@|$DIR|g" "$DIR/v2x-risk-engine.service" \
  > /etc/systemd/system/v2x-risk-engine.service
systemctl daemon-reload
systemctl enable v2x-risk-engine.service
systemctl restart v2x-risk-engine.service
echo "[OK] 서비스 등록 및 시작됨"

# 3. 로그 업로드 타이머 (5분마다, 오프라인이면 다음 주기 재시도)
sed -e "s|@DEPLOY_DIR@|$DIR|g" -e "s|@RUN_USER@|$RUN_USER|g" \
  "$DIR/v2x-log-upload.service" > /etc/systemd/system/v2x-log-upload.service
cp "$DIR/v2x-log-upload.timer" /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now v2x-log-upload.timer
echo "[OK] 로그 업로드 타이머 등록됨 (5분 주기, 사용자=$RUN_USER)"

# 4. 위험 이벤트 업로드 타이머 (1분마다, 위험지도 서버로)
if [ ! -f /etc/default/v2x-riskmap ]; then
  cat > /etc/default/v2x-riskmap <<'EOF'
# 위험지도 서버 설정. API 키를 채워 넣어야 이벤트 업로드가 동작한다.
RISKMAP_URL=https://riskmap-api-193571596396.asia-northeast3.run.app
RISKMAP_API_KEY=
EOF
  chmod 600 /etc/default/v2x-riskmap
fi
sed -e "s|@DEPLOY_DIR@|$DIR|g" -e "s|@RUN_USER@|$RUN_USER|g" \
  "$DIR/v2x-event-upload.service" > /etc/systemd/system/v2x-event-upload.service
cp "$DIR/v2x-event-upload.timer" /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now v2x-event-upload.timer
echo "[OK] 이벤트 업로드 타이머 등록됨 (1분 주기)"
if ! grep -q "RISKMAP_API_KEY=." /etc/default/v2x-riskmap; then
  echo "[주의] /etc/default/v2x-riskmap 의 RISKMAP_API_KEY 가 비어 있음 -> 채우기 전까지 업로드 안 함"
fi

echo ""
echo "===== 설치 완료 ====="
echo "상태 확인:   systemctl status v2x-risk-engine"
echo "실시간 로그: journalctl -u v2x-risk-engine -f"
echo "기록 CSV:    $(dirname "$DIR")/logs/"
echo "업로드 확인: journalctl -u v2x-log-upload -n 20"
echo "즉시 업로드: sudo systemctl start v2x-log-upload"
echo "이벤트 업로드 확인: journalctl -u v2x-event-upload -n 20"
echo "이벤트 즉시 업로드: sudo systemctl start v2x-event-upload"
echo "서버 설정: /etc/default/v2x-riskmap (API 키)"
echo "중지:        sudo systemctl stop v2x-risk-engine"
echo "자동시작 해제: sudo systemctl disable v2x-risk-engine"
