#!/bin/bash
# Jetson Nano에서 실행하는 스크립트.
# 서버(generated_data/)에서 생성이 완료된(DONE 표시가 있는) 시나리오만 가져온다.
#
# 사전 준비 (한 번만):
#   1. Jetson에서 SSH 키 생성:  ssh-keygen -t ed25519 -f ~/.ssh/risk_server_key -N ""
#   2. 공개키(~/.ssh/risk_server_key.pub 내용)를 서버의 ~/.ssh/authorized_keys에 추가
#
# 사용:
#   ./jetson_fetch.sh            # 새 시나리오 가져오기 (이미 받은 것은 건너뜀)
#   crontab에 등록하면 주기적으로 자동 수신:
#   * * * * * /home/<jetson사용자>/jetson_fetch.sh >> /home/<jetson사용자>/fetch.log 2>&1

SERVER="ssukpc347@8.230.1.67"
KEY="$HOME/.ssh/risk_server_key"
DEST="$HOME/incoming_data"

mkdir -p "$DEST"

rsync -av -e "ssh -i $KEY -o ConnectTimeout=15" \
    --include='scenario_*/' \
    --include='scenario_*/feature.csv' \
    --include='scenario_*/pedestrian.csv' \
    --include='scenario_*/DONE' \
    --exclude='*' \
    "$SERVER:SUMO_project/generated_data/" "$DEST/"

# 주의: 추론 코드는 DONE 파일이 있는 폴더만 처리할 것.
# (DONE이 없는 폴더는 아직 생성 중일 수 있음)
