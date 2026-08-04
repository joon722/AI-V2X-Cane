# rsu — Jetson/RSU 현장 작업 아카이브

Jetson(RSU 쪽)에서 실기 개발할 때 사용한 작업 파일들의 스냅샷입니다. 단계별 개발 파일(step2~step8), 실기 실험 로그, 자동 시작 배포 스크립트가 들어 있습니다.

> **정리된 최신 코드는 [`lux/`](../lux/)에 있습니다.** 이 폴더는 현장 작업 이력과 실험 로그 보존용입니다.

## 주요 내용

| 경로 | 내용 |
| --- | --- |
| `v2x/03_jetson/step2~step8*.py` | 단계별 개발 파일 (파싱 → 상태 저장 → 운동학 → 위험도 → 전송) — `lux/` 모듈의 원형 |
| `v2x/03_jetson/scripts/` | 실시간 위험도 엔진, 시리얼 로거, 링크 모니터 등 현장 실행 스크립트 |
| `v2x/03_jetson/deploy/` | systemd 서비스·udev 규칙·설치 스크립트 (Jetson 부팅 시 자동 시작) |
| `v2x/03_jetson/experiments/` | 실기 실험 로그 (수신 로그, 위험도 로그, 시리얼 링크 모니터링) |
| `v2x/03_jetson/scenarios/` | 시나리오 재생용 CSV (정면 접근, 사각지대, 고위험 등) |
| `v2x/v2x_bridge/v2x_bridge.ino` | RSU 브리지 ESP32 스케치 사본 |
