#!/bin/bash
# SUMO 시나리오 생성기 실행 래퍼.
# flock으로 중복 실행을 막아서, cron이 1분마다 불러도
# 이미 돌고 있으면 아무것도 하지 않고 죽어 있으면 다시 시작한다.
cd "$(dirname "$0")"
exec flock -n /tmp/sumo_generator.lock \
    .venv/bin/python generate_scenarios.py "$@"
