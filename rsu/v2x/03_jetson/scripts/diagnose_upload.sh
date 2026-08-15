#!/bin/bash
# 젯슨 -> 서버 결과 업로드가 막혔을 때 원인을 한 번에 좁힌다.
#
# 업로드는 세 곳에서 막힐 수 있다. 어디인지 모르는 채로 손대면 엉뚱한 것을
# 고치게 되므로, 확인 순서를 고정해 둔다.
#
#   1) 파이프라인이 돌고 있는가        - 안 돌면 재시도 자체가 없다
#   2) 업로드를 시도했는가             - 로그에 실패 기록이 있는가
#   3) 시도했다면 왜 실패했는가        - 서버가 돌려준 stderr
#   4) 시도조차 안 했다면 마커 때문인가 - .uploaded가 찍혀 있으면 재전송하지 않는다
#
# 4번이 특히 헷갈린다. 서버에서 결과를 지워도 젯슨은 "이미 보냈다"고 기억하므로
# 영원히 다시 보내지 않는다. 그때는 실패 기록이 하나도 없이 조용하다.
#
# 사용: bash diagnose_upload.sh

set -u
LOG="$HOME/pipeline.log"
IN="$HOME/incoming_data"
OUT="$HOME/inference_results"

echo "=============================================="
echo " 1. 파이프라인이 살아 있는가"
echo "=============================================="
n=$(pgrep -fc "process_scenarios" || true)
echo "  프로세스: ${n:-0}개"
if [ "${n:-0}" -eq 0 ]; then
  echo "  >> 죽어 있다. cron이 1분마다 살리므로 잠시 뒤 다시 확인하거나"
  echo "     로그 끝부분에서 종료 원인을 본다."
fi
echo "  최근 로그 5줄:"
tail -5 "$LOG" 2>/dev/null | sed 's/^/    /' || echo "    (로그 없음: $LOG)"

echo
echo "=============================================="
echo " 2. 업로드를 시도한 적이 있는가"
echo "=============================================="
fail=$(grep -c "업로드 실패\|폴더 생성 실패" "$LOG" 2>/dev/null || echo 0)
echo "  실패 기록: ${fail}건"
if [ "$fail" -eq 0 ]; then
  echo "  >> 실패한 적이 없다. 4번(마커)을 확인할 것."
else
  echo "  >> 실패했다. 아래가 서버가 돌려준 이유다:"
  grep "업로드 실패\|폴더 생성 실패" "$LOG" | tail -5 | sed 's/^/    /'
fi

echo
echo "=============================================="
echo " 3. 서버에 붙을 수 있는가"
echo "=============================================="
KEY=$(grep -oP 'SERVER_KEY\s*=\s*.*?"\K[^"]+' \
      "$HOME/v2x/03_jetson/auto_pipeline/process_scenarios.py" 2>/dev/null \
      | head -1)
KEY="${KEY/#\~/$HOME}"
echo "  키 파일: ${KEY:-(못 찾음)}"
[ -n "${KEY:-}" ] && ls -l "$KEY" 2>/dev/null | sed 's/^/    /'
ssh -i "${KEY:-$HOME/.ssh/id_rsa}" -o ConnectTimeout=15 -o BatchMode=yes \
    ssukpc347@8.230.1.67 \
    'echo "    접속 OK"; echo -n "    서버 디스크: "; df -h ~ | tail -1; \
     echo -n "    결과 파일 수: "; ls SUMO_project/results/ 2>/dev/null | wc -l' \
    2>&1 | sed 's/^/  /'

echo
echo "=============================================="
echo " 4. 마커 상태 - 젯슨이 무엇을 보냈다고 기억하는가"
echo "=============================================="
proc=$(ls -d "$IN"/scenario_*/.processed 2>/dev/null | wc -l)
upl=$(ls -d "$IN"/scenario_*/.uploaded 2>/dev/null | wc -l)
csv=$(ls "$OUT"/*.csv 2>/dev/null | wc -l)
echo "  추론 완료(.processed): $proc"
echo "  업로드 완료(.uploaded): $upl"
echo "  결과 CSV 파일:          $csv"
echo "  업로드 대기 중:         $((proc - upl))"

echo
echo "=============================================="
echo " 판정"
echo "=============================================="
if [ "$fail" -gt 0 ]; then
  echo "  실패 기록이 있다 -> 2번에 찍힌 서버 stderr가 원인이다."
elif [ "$upl" -gt 0 ] && [ "$((proc - upl))" -eq 0 ]; then
  echo "  실패 기록이 없고 전부 '보냈다'고 기록되어 있다."
  echo "  서버가 비어 있다면 젯슨은 재전송하지 않는다 - 마커를 지워야 한다:"
  echo
  echo "      rm $IN/scenario_*/.uploaded"
  echo
  echo "  추론은 다시 하지 않고 결과 CSV만 1분 안에 재전송된다."
else
  echo "  대기 중인 것이 $((proc - upl))개 있다. 파이프라인이 살아 있으면"
  echo "  다음 주기에 자동으로 올라간다. 1~2분 뒤 다시 실행해 볼 것."
fi
