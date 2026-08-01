#!/bin/bash
# Jetson 추론 파이프라인 실행 래퍼 (cron이 1분마다 확인, 죽어 있으면 재시작)
cd "$(dirname "$0")"
exec flock -n /tmp/jetson_pipeline.lock     $HOME/v2x/.venv/bin/python process_scenarios.py --interval 60
