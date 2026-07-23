# 7단계 설계: risk_score / risk_level 수식 적용

날짜: 2026-07-23
대상: `step7_risk.py` (신규), `test_step7_risk.py` (신규)

## 목표

6단계(`step6_kinematics.py`)가 낸 거리 / closing speed / TTC / DCPA를 `risk_score`(0~100)와
`risk_level`(0~3)으로 바꾼다. 스코어링 대상은 filtered(칼만) 트랙이다.

## 파일 구조 규칙

단계마다 새 파일을 만들고 이전 파일은 고치지 않는다. `step7_risk.py`는 step3~6을 import만 한다
(`KinematicsPipeline`, `StateStore`, `TestVehicle`, `serial_lines`, `has_position`, `to_float`).

팀 점수표는 `tmp/AI-V2X-Cane-audit/scripts/risk_calculator.py`에 있으나 audit용 임시 폴더라
import 의존이 취약하다. 세 함수(`calculate_ttc` / `calculate_risk_score` /
`classify_risk_level`)를 **숫자 그대로 step7에 vendor(복사)**하고 출처·"팀 표 원본과 동일"
주석을 단다. 팀이 표를 바꾸면 여기서 재동기화한다. 팀 표 자체는 한 글자도 바꾸지 않고,
DCPA 게이트만 그 위에 얹는다.

## 데이터 흐름

6단계 `KinematicsPipeline.compute()`가 주는 `(now, raw, filtered)` 중 **filtered**를 점수화한다.
필드 매핑:

| 팀 표 입력            | 소스                                        |
| --------------------- | ------------------------------------------- |
| `distance_m`          | `filtered.distance_m`                       |
| `relative_speed_mps`  | `filtered.closing_los` (시선방향 접근속도)  |
| `ttc`                 | 팀 `calculate_ttc(distance, closing_los)` 재계산 |
| `vehicle_speed_mps`   | `store.latest["vehicle"]["speed_mps"]` (raw JSON) |
| `zone_base_risk`      | `0` (zone 감지 미통합, 기본값)              |

`vehicle_speed_mps`는 raw JSON 값을 쓴다. 팀 표의 속도항이 5/10/15/20 4버킷이라 raw로 충분하고,
filtered 차량 절대속도를 얻으려면 step6를 고쳐야 하는데 규칙상 그건 피한다.

## DCPA 억제 게이트 (핵심)

```
base_score  = 팀_calculate_risk_score(distance, closing_los, vehicle_speed, ttc, zone=0)  # 팀 표 그대로
gate        = g(filtered.dcpa)                                                             # 0~1 배율
final_score = base_score * gate
risk_level  = 팀_classify_risk_level(final_score)                                          # 0~3
```

### 왜 게이트인가

팀 표에는 DCPA 항이 없어 보도 옆 4m를 빗겨 지나가는 "스침"과 정면 충돌 코스가 같은 점수를 받는다.
6단계에서 CPA를 계산한 이유가 이 구분이다. DCPA는 "실제로 나를 맞히는가"의 게이트 성격이라
가점 항이 아니라 배율로 얹는 것이 맞고, 팀의 검증된 0~100 표를 보존한다.

### 게이트 함수 `g(dcpa)` — 소프트(선형 보간), 하드 컷오프 아님

```
dcpa <= near_m         → g = 1.0            (경로 안, 억제 없음)
near_m < dcpa < far_m  → 선형 보간 (1.0 → floor)
dcpa >= far_m          → g = floor          (명백한 빗겨감)
dcpa 없음(멀어짐/정지) → g = 1.0            (표가 closing<=0로 이미 저점 처리)
```

### 기본값은 GPS 노이즈에 묶는다 (`GPS_SIGMA_M = 2.5`)

- `near_m = 2.5` (≈1σ), `far_m = 7.5` (≈3σ), `floor = 0.2`
- 근거: filtered dcpa도 추정치라 오차가 있다. GPS CEP 2.5m 안쪽 dcpa는 실제 충돌일 수 있어
  억제하지 않고, 명백히 벗어난(>3σ) 경우만 강하게 낮춘다. floor를 0이 아닌 0.2로 둬서
  추정 오류로 진짜 위험을 완전히 지우지 않는다.
- `--dcpa-near-m` / `--dcpa-far-m` / `--dcpa-floor`로 노출한다 (step6가 칼만 상수를 노출한 방식과 동일).
  실외 GPS 로그가 쌓이면 **여기가 1순위 재조정 지점**이다.
- 지금 test vehicle은 정면(dcpa≈0)이라 게이트는 사실상 no-op → 기존 검증 화면이 안 깨지고
  실차 기하에서만 작동한다.

## 출력 / CSV

- 콘솔: `[RISK] score=XX.X level=N (base=YY.Y dcpa=Z.ZZm gate=0.GG)` — 억제가 눈에 보이게.
- `step7_risk_log.csv` 컬럼: `pc_time, cane_seq, vehicle_seq, distance_m, closing_los,
  ttc, vehicle_speed, dcpa, base_score, gate, final_score, risk_level`.
  8단계 전송값·재조정 근거로 남긴다.
- risk_level만 계산해 로그한다. 실제 전송은 8단계 담당.

## 메인 루프

step7은 step6를 고치지 않으므로 자체 루프를 갖는다(step6 루프와 ~15줄 중복은 규칙상 감수).
각 줄마다 `store.update` → `pipeline.observe` → `pipeline.compute()` → filtered 트랙 스코어링 →
`[RISK]` 출력 + step7 CSV append. `--test-vehicle` 주입 경로도 step6와 동일하게 유지한다.

## 테스트 (`test_step7_risk.py`)

- 게이트 함수: 끝점(near/far), 단조 감소, dcpa=None → 1.0, floor 하한.
- vendor한 팀 표: 대표 입력 몇 개로 팀 원본과 동일 점수 확인 (회귀 방지).
- 통합: 정면 접근(dcpa≈0) → 게이트 no-op으로 base와 동일 / 빗겨감(dcpa 큼) → level 하락.
- 기존 61개 스타일에 맞춰 `python3 -m unittest discover -p "test_*.py"`로 같이 돈다.

## 검증 계획

1. `python3 -m unittest discover -p "test_*.py"` — 기존 61 + 신규 모두 통과.
2. Jetson 실기: `scp step7_risk.py test_step7_risk.py ssu212324@192.168.55.1:~/v2x/03_jetson/`
   후 `python3 step7_risk.py --source-mode fallback --test-vehicle` — 거리 감소에 따라
   risk_score/level이 함께 오르는지 눈으로 확인. dcpa≈0이라 gate는 1.0 유지.
