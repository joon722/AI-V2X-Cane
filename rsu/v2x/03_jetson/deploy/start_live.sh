#!/bin/bash
# 라이브뷰(stream_live) 하나만 깔끔히 재기동. 반드시 root로: sudo bash start_live.sh
# 하는 일: 기존 stream_live 종료 -> RISKMAP 키 로드 -> 딱 하나 실행(로그 /tmp/stream_live.log)
pkill -f 'deploy/stream_live.py' 2>/dev/null
sleep 1
if [ ! -r /etc/default/v2x-riskmap ]; then
  echo "[에러] /etc/default/v2x-riskmap 읽기 실패 (sudo로 실행하세요)"; exit 1
fi
set -a; . /etc/default/v2x-riskmap; set +a
if [ -z "$RISKMAP_API_KEY" ]; then
  echo "[에러] RISKMAP_API_KEY 비어있음 -> /etc/default/v2x-riskmap 에 키 채우기"; exit 1
fi
cd /home/ssu212324/v2x/03_jetson || exit 1
export PYTHONUTF8=1 PYTHONUNBUFFERED=1
setsid python3 deploy/stream_live.py > /tmp/stream_live.log 2>&1 < /dev/null &
sleep 6
echo "=== stream_live 상태 ==="
if ps -eo pid,etime,cmd | grep stream_live | grep -v grep; then
  echo "[OK] 떴습니다. 드라이브뷰가 곧 갱신됨."
else
  echo "[실패] 안 떴음. 로그:"; tail -6 /tmp/stream_live.log
fi
