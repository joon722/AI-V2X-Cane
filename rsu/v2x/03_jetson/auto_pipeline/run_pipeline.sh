#!/bin/bash
# Jetson 추론 파이프라인 실행 래퍼 (cron이 1분마다 확인, 죽어 있으면 재시작)
# 서버 계정은 환경변수로 준다. crontab 예:
#   * * * * * V2X_SERVER=<사용자>@<서버 주소> $HOME/v2x/03_jetson/auto_pipeline/run_pipeline.sh
cd "$(dirname "$0")"
exec flock -n /tmp/jetson_pipeline.lock     $HOME/v2x/.venv/bin/python process_scenarios.py --interval 60
