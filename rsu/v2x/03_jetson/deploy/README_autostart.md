# V2X 부팅 자동실행 설치 가이드

젯슨 전원만 켜면 위험 계산 → ESP32 회신 → CSV 기록이 자동으로 돌아가게 만드는 설정.

## 구성 파일

```text
deploy/
  run_v2x_risk_engine.sh    실행 래퍼 (포트 대기, 포트 정리, 타임스탬프 로그)
  v2x-risk-engine.service   systemd 서비스 (부팅 자동시작, 죽으면 3초 후 재시작)
  99-v2x-rsu.rules          udev 규칙 (/dev/v2x-rsu 고정 이름)
  upload_logs.sh            로그 CSV를 서버(risk-server)로 rsync 업로드
  v2x-log-upload.service    업로드 단발 작업 (일반 사용자로 실행)
  v2x-log-upload.timer      부팅 2분 후 + 5분마다 업로드 실행
  install.sh                설치 스크립트 (젯슨에서 한 번만 실행)
```

로그 업로드는 서버 `~/v2x_logs/<젯슨호스트명>/` 에 쌓인다.
인터넷이 없으면 경고만 남기고 다음 주기(5분 뒤)에 자동 재시도한다.

메인 엔진은 `step8_send_risk.py`를 그대로 사용한다 (수정 없음).
step8을 쓰는 이유: 지팡이 실좌표 사용 + GPS 신뢰도 게이팅 + 1초 heartbeat.
지팡이 펌웨어의 `RSU_RISK_TIMEOUT_MS`(3초)는 1초 heartbeat의 3배로 설계되어 있어
heartbeat가 없는 구버전 `v2x_rsu.py`를 쓰면 위험 상태가 3초 만에 풀리는 문제가 있다.

## 1. 젯슨으로 파일 올리기

Windows PowerShell에서 (젯슨 IP와 사용자명은 실제 값으로 교체):

```bash
scp -r "C:\Users\user\OneDrive\바탕 화면\v2x(lux)\03_jetson\deploy" jetson@192.168.x.x:~/v2x/03_jetson/
```

네트워크가 안 되면 USB 메모리로 `deploy` 폴더를 `~/v2x/03_jetson/` 아래에 복사.

## 2. 젯슨에서 설치 (한 번만)

```bash
cd ~/v2x/03_jetson/deploy
chmod +x install.sh run_v2x_risk_engine.sh
sudo ./install.sh
```

이후에는 전원만 켜면 자동으로 시작된다.

## 3. 동작 확인

```bash
# 서비스 상태 (active (running) 이어야 함)
systemctl status v2x-risk-engine

# 실시간 로그 보기 ([TX] risk=... 줄이 나오는지)
journalctl -u v2x-risk-engine -f

# 고정 포트 이름 생겼는지
ls -l /dev/v2x-rsu

# 기록 CSV (세션마다 새 파일)
ls ~/v2x/03_jetson/logs/
```

재부팅 테스트: `sudo reboot` 후 위 명령을 다시 실행해서 자동으로 떠 있는지 확인.

## 4. 수동 조작

```bash
sudo systemctl stop v2x-risk-engine       # 잠시 중지 (수동 실험할 때)
sudo systemctl start v2x-risk-engine      # 다시 시작
sudo systemctl disable v2x-risk-engine    # 부팅 자동시작 끄기
```

수동으로 step8을 직접 돌리고 싶으면 먼저 서비스를 stop 할 것
(둘 다 켜면 시리얼 포트를 서로 뺏는다).

## 주의사항

- 젯슨에 ESP32가 **RSU 브리지 하나만** USB로 연결되어 있어야 한다.
  같은 칩을 쓰는 보드가 2개 이상 꽂히면 /dev/v2x-rsu 가 엉뚱한 보드를 가리킬 수 있다.
- step8은 지팡이+차량 데이터가 **둘 다** 들어와야 위험을 계산한다.
  차량 신호 없이 테스트하려면 서비스를 stop 하고
  `python3 step8_send_risk.py --test-vehicle` 로 수동 실행.
- 로그 CSV는 세션마다 쌓이므로 오래 운용하면 가끔 `logs/` 정리 필요.
