#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GPS 튐(8/17 실측) 강조 프리셋으로 학습 공장 스크립트를 실행하는 래퍼.

왜: --randomize 는 FieldNoiseParams.randomized() 로 시나리오마다 잡음을 흔들지만
     재획득 점프 하한이 0 m, 무효 빈도 하한이 0/분이라 "튐이 거의 없는" 시나리오도 섞인다.
     현준 지시 ②("오늘 GPS 튐 문제를 모델이 배우게")를 확실히 하려면 모든 시나리오에
     실측 튐(무효 3~6 s, 재획득 점프 최대 19 m, 격자, 손실)을 항상 넣은 데이터로 학습해 비교한다.

어떻게: gps_noise.FieldNoiseParams.randomized 를 "튐 강조" 버전으로 몽키패치한 뒤
       sim_to_rsu_stream / build_dataset_from_streams 의 main 을 그대로 호출한다 (팀 코드 무수정).
       환경 의존 σ·τ 등은 여전히 흔들고, 튐 관련 4개만 실측 하한을 올린다.

사용
    python run_with_gps_jump.py stream  <scenario_dir> --vehicle-id V --person-id P --out DIR --randomize
    python run_with_gps_jump.py build   --streams DIR --from-scenario-sim 4000 --families --randomize --out ds.csv
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import gps_noise  # noqa: E402

# 8/17 오후 실측 (gps_noise.FieldNoiseParams 주석·설명서 기준) — 튐 4개만 하한을 올린 범위
JUMP_PRESET = {
    "dropout_per_min": (1.5, 4.0),   # 무효 에피소드: 실측 5~10분당 수 회 → 최소 1.5/분
    "dropout_s_max": (5.0, 8.0),     # 무효 지속 상한 (하한 3 s 고정): 실측 3~6 s 흔함
    "reacq_jump_lo": (5.0, 10.0),    # 재획득 점프 하한: 실측 "10~19 m 점프" 재현
    "reacq_jump_hi": (12.0, 20.0),
}

_orig_randomized = gps_noise.FieldNoiseParams.randomized.__func__


def _jump_randomized(cls, rng):
    p = _orig_randomized(cls, rng)
    lo = float(rng.uniform(*JUMP_PRESET["reacq_jump_lo"]))
    hi = float(rng.uniform(max(lo, JUMP_PRESET["reacq_jump_hi"][0]), JUMP_PRESET["reacq_jump_hi"][1]))
    p.dropout_per_min = float(rng.uniform(*JUMP_PRESET["dropout_per_min"]))
    p.dropout_s = (3.0, float(rng.uniform(*JUMP_PRESET["dropout_s_max"])))
    p.reacq_jump_m = (lo, hi)
    return p


gps_noise.FieldNoiseParams.randomized = classmethod(_jump_randomized)


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("stream", "build"):
        raise SystemExit(__doc__)
    mode = sys.argv.pop(1)
    if mode == "stream":
        import sim_to_rsu_stream as m
    else:
        import build_dataset_from_streams as m
    sys.argv[0] = str(HERE / (m.__name__ + ".py"))
    m.main()


if __name__ == "__main__":
    main()
