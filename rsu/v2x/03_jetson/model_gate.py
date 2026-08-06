#!/usr/bin/env python3
"""모델 판정을 규칙 위에 얹는 계층.

    최종 레벨 = max( 안전 하한, 규칙 점수, 모델 )

세 층이 서로를 덮어쓰지 않는 것이 이 모듈의 전부다.

  안전 하한(step7_risk.T_FLOOR_TTC_S)은 모델보다 세다. 모델은 학습 분포 밖에서
  예고 없이 틀릴 수 있지만 하한은 물리로 계산한 값이라 그럴 일이 없다.

  모델은 더하기만 한다. 모델이 안전하다고 판단해도 규칙이 올린 레벨을 내리지
  않는다. 검증되지 않은 쪽이 검증된 쪽을 무르게 만들어서는 안 된다.

  모델이 없거나 실패해도 규칙 판정은 그대로 나간다. 경보 경로에 있는 코드라
  모델의 문제로 시스템이 멈추면 안 된다.

모델 파일(risk_model.json)이 없으면 조용히 규칙만으로 동작한다. 젯슨에 파일을
올리지 않으면 이전과 완전히 같게 도는 셈이다.
"""

import sys
from dataclasses import dataclass
from pathlib import Path

# 모델이 경보로 판단했을 때 부여하는 레벨. 학습 라벨이 "3초 안에 2m 이내 접근"
# 이고 그것은 step7 기준으로 경보(2) 수준이다. 레벨 3은 안전 하한의 몫이라
# 모델이 그 자리를 침범하지 않는다.
MODEL_ALARM_LEVEL = 2

DEFAULT_MODEL_FILE = "risk_model.json"


@dataclass(frozen=True)
class GateResult:
    level: int
    proba: float | None
    source: str          # "table" | "model"


class ModelGate:
    """규칙 레벨과 모델 판정을 합친다."""

    def __init__(self, model=None, streamer=None):
        self.model = model
        self.streamer = streamer
        self.failures = 0

    @classmethod
    def load(cls, path=None, quiet=False):
        """모델 파일을 읽어 게이트를 만든다. 없으면 모델 없는 게이트.

        경로를 주지 않으면 이 파일 옆의 risk_model.json을 본다.
        """
        from model_features import StreamingFeatures

        target = Path(path) if path else Path(__file__).with_name(DEFAULT_MODEL_FILE)
        if not target.exists():
            if not quiet:
                print(f"[INFO] 모델 파일 없음 ({target.name}) - 규칙만으로 동작",
                      file=sys.stderr)
            return cls(model=None, streamer=None)

        from model_runtime import TreeEnsemble

        model = TreeEnsemble.load(target)
        if not quiet:
            meta = model.meta or {}
            print(f"[INFO] 모델 적재 {target.name} threshold={model.threshold:.4f} "
                  f"trained_scenarios={meta.get('trained_scenarios', '?')}",
                  file=sys.stderr)
        return cls(model=model, streamer=StreamingFeatures())

    def apply(self, rule_level, now, states):
        """규칙 레벨에 모델 판정을 얹는다.

        states는 (cane_state, veh_state)이며 각각 (동, 북, 동속, 북속)이다.
        step6 KinematicsPipeline.last_states 그대로 넘기면 된다.
        """
        if self.model is None or self.streamer is None or states is None:
            return GateResult(level=rule_level, proba=None, source="table")

        try:
            features = self.streamer.update(now, states[0], states[1])
            proba = self.model.predict_proba(features)
        except Exception as exc:  # noqa: BLE001 - 경보 경로는 어떤 이유로도 멈추면 안 된다
            self.failures += 1
            if self.failures <= 3:
                print(f"[WARN] model_failed error={exc}", file=sys.stderr)
            return GateResult(level=rule_level, proba=None, source="table")

        if proba >= self.model.threshold and rule_level < MODEL_ALARM_LEVEL:
            return GateResult(level=MODEL_ALARM_LEVEL, proba=proba, source="model")
        return GateResult(level=rule_level, proba=proba, source="table")
