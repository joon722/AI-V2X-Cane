"""오라클 라벨 검증.

이 라벨이 연구 전체의 정답지다. 규칙(TTC 점수표, DCPA 게이트)이 한 줄도
개입하지 않아야 하고, "그때 실제로 위험했는가"를 미래 정보로만 판정해야 한다.
여기가 틀리면 뒤의 모든 비교가 무의미해진다.
"""

import numpy as np
import pytest

from oracle import label_scenario, pet_seconds
from scenario_sim import ScenarioParams, simulate


def params(**overrides):
    base = dict(
        approach_deg=0.0,
        start_distance_m=60.0,
        veh_speed_mps=10.0,
        miss_offset_m=0.0,
        ped_speed_mps=0.0,
        ped_heading_deg=0.0,
        veh_accel_mps2=0.0,
        turn_rate_dps=0.0,
    )
    base.update(overrides)
    return ScenarioParams(**base)


def test_head_on_is_labelled_dangerous_before_impact():
    """정면 충돌 코스는 접촉 이전 시점들이 위험으로 라벨된다."""
    sc = simulate(params(miss_offset_m=0.0), dt=0.1, duration_s=20.0)

    lab = label_scenario(sc, d_crit=2.0, horizon_s=10.0)

    # 차가 60m를 10m/s로 달려 t=6.0에 도달한다. horizon 10초 안이므로 t=0부터 위험.
    assert lab.y[0] == 1


def test_clear_miss_is_never_dangerous():
    """명백히 스쳐 가는 차는 어느 시점에서도 위험이 아니다."""
    sc = simulate(params(miss_offset_m=12.0), dt=0.1, duration_s=20.0)

    lab = label_scenario(sc, d_crit=2.0, horizon_s=10.0)

    assert lab.y.sum() == 0


def test_danger_clears_after_the_vehicle_has_passed():
    """차가 지나간 뒤에는 더 이상 위험이 아니다."""
    sc = simulate(params(miss_offset_m=0.0), dt=0.1, duration_s=20.0)

    lab = label_scenario(sc, d_crit=2.0, horizon_s=10.0)

    assert lab.y[-1] == 0


def test_horizon_bounds_how_far_ahead_danger_counts():
    """horizon 밖의 위험은 아직 위험이 아니다.

    t=0에서 접촉까지 6초 걸리는 시나리오를, horizon 3초로 보면 t=0은 안전이어야
    한다. 라벨이 미래를 보는 창의 크기를 실제로 지키는지 확인한다.
    """
    sc = simulate(params(miss_offset_m=0.0), dt=0.1, duration_s=20.0)

    short = label_scenario(sc, d_crit=2.0, horizon_s=3.0)
    long = label_scenario(sc, d_crit=2.0, horizon_s=10.0)

    assert short.y[0] == 0
    assert long.y[0] == 1


def test_lead_time_ignores_danger_that_is_already_too_close():
    """lead_s를 주면 코앞의 위험은 더 이상 정답이 아니다.

    "3초 안에 위험"으로 라벨을 만들면 0.5초 뒤에 부딪히는 시점도 정답이 된다.
    그 순간에 울려봐야 사람은 반응할 수 없으므로, 모델은 쓸모없는 경보를 정답으로
    배우게 된다. 창을 T_floor만큼 밀어 "지금 울려야 피할 수 있는 위험"만 남긴다.
    """
    sc = simulate(params(miss_offset_m=0.0), dt=0.1, duration_s=20.0)

    # 60m를 10 m/s로 오므로 t=6.0에 접촉한다.
    plain = label_scenario(sc, d_crit=2.0, horizon_s=3.0, lead_s=0.0)
    lead = label_scenario(sc, d_crit=2.0, horizon_s=3.0, lead_s=2.0)

    at_5_5s = 55  # t=5.5, 접촉 0.5초 전
    assert plain.y[at_5_5s] == 1, "코앞의 위험이 그냥 라벨에서는 정답이다"
    assert lead.y[at_5_5s] == 0, "밀린 창에서는 정답이 아니어야 한다"


def test_lead_time_keeps_danger_that_is_still_avoidable():
    """아직 피할 수 있는 거리의 위험은 그대로 정답으로 남는다."""
    sc = simulate(params(miss_offset_m=0.0), dt=0.1, duration_s=20.0)

    lead = label_scenario(sc, d_crit=2.0, horizon_s=3.0, lead_s=2.0)

    at_3_5s = 35  # t=3.5, 접촉 2.5초 전 -> 창 [5.5, 6.5] 안에 접촉이 들어온다
    assert lead.y[at_3_5s] == 1


def test_zero_lead_matches_the_plain_window():
    """lead_s=0이면 기존 동작과 같다."""
    sc = simulate(params(miss_offset_m=1.0), dt=0.1, duration_s=20.0)

    a = label_scenario(sc, d_crit=2.0, horizon_s=3.0)
    b = label_scenario(sc, d_crit=2.0, horizon_s=3.0, lead_s=0.0)

    assert (a.y == b.y).all()


def test_lead_must_be_shorter_than_the_horizon():
    """창이 비어버리는 설정은 조용히 넘어가지 않는다."""
    sc = simulate(params(), dt=0.1, duration_s=5.0)

    with pytest.raises(ValueError):
        label_scenario(sc, d_crit=2.0, horizon_s=3.0, lead_s=3.0)


def test_d_crit_decides_what_counts_as_contact():
    """빗나감 3m 시나리오는 d_crit 2m에서는 안전, 5m에서는 위험이다."""
    sc = simulate(params(miss_offset_m=3.0), dt=0.1, duration_s=20.0)

    assert label_scenario(sc, d_crit=2.0, horizon_s=10.0).y.sum() == 0
    assert label_scenario(sc, d_crit=5.0, horizon_s=10.0).y.sum() > 0


def test_time_to_hit_matches_the_actual_closest_approach():
    """t_hit이 실제로 가장 가까워지는 시각을 가리킨다."""
    sc = simulate(params(miss_offset_m=0.0), dt=0.1, duration_s=20.0)

    lab = label_scenario(sc, d_crit=2.0, horizon_s=10.0)

    # 60m / 10 m/s = 6.0초에 접촉한다.
    assert lab.t_hit[0] == pytest.approx(6.0, abs=0.15)


def test_future_minimum_never_looks_backwards():
    """미래 최소거리는 현재 시점 이후만 본다.

    과거를 섞으면 이미 지나간 위험이 계속 위험으로 남아, 경보를 늦게 울리는
    모델이 좋아 보이게 된다.
    """
    sc = simulate(params(miss_offset_m=0.0), dt=0.1, duration_s=20.0)

    lab = label_scenario(sc, d_crit=2.0, horizon_s=10.0)

    # 접촉(t=6.0) 한참 뒤에는 차가 멀어지기만 하므로 미래 최소거리가 커야 한다.
    late = lab.d_min_future[-1]
    assert late > 2.0


def test_label_uses_no_risk_rule():
    """라벨 계산에 규칙 모듈이 관여하지 않는다.

    이 연구의 전제가 '정답은 규칙과 독립'이다. import 한 줄이 섞이면 순환 논리가
    되므로 import 수준에서 못 박는다. 문자열 검색이 아니라 AST를 보는 이유는
    docstring에서 규칙 모듈을 설명으로 언급하는 것까지 막을 이유는 없기 때문이다.
    """
    import ast
    import pathlib

    src = pathlib.Path(__file__).with_name("oracle.py").read_text(encoding="utf-8")
    imported = set()
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, ast.Import):
            imported.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module)
            imported.update(a.name for a in node.names)

    offenders = [m for m in imported if "risk" in m or "step7" in m]
    assert not offenders, f"오라클이 규칙에 의존한다: {offenders}"


def test_pet_is_small_when_paths_cross_closely_in_time():
    """보행자가 막 지나간 자리를 차가 곧바로 통과하면 PET가 작다.

    보행자는 동쪽으로 1.2 m/s로 걸어가고, 차는 30m 북쪽에서 10 m/s로 내려와
    t=3.0에 보행자의 출발점을 지난다. 보행자가 그 자리(2m 이내)를 벗어난 것이
    t=1.7 무렵이므로 시간차는 1초 남짓이다 - 아슬아슬하게 스친 상충이다.
    """
    sc = simulate(
        params(
            ped_speed_mps=1.2,
            ped_heading_deg=90.0,
            miss_offset_m=0.0,
            start_distance_m=30.0,
        ),
        dt=0.1,
        duration_s=20.0,
    )

    pet = pet_seconds(sc, d_crit=2.0)

    assert pet is not None
    assert pet < 3.0


def test_pet_is_none_when_paths_never_meet():
    """경로가 아예 겹치지 않으면 PET는 정의되지 않는다."""
    sc = simulate(params(miss_offset_m=30.0), dt=0.1, duration_s=20.0)

    assert pet_seconds(sc, d_crit=2.0) is None


def test_labels_line_up_with_the_timeline():
    """라벨 배열 길이가 시간축과 같다."""
    sc = simulate(params(), dt=0.1, duration_s=20.0)

    lab = label_scenario(sc, d_crit=2.0, horizon_s=10.0)

    assert len(lab.y) == len(sc.t)
    assert len(lab.d_min_future) == len(sc.t)
    assert len(lab.t_hit) == len(sc.t)
