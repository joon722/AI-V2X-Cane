#!/usr/bin/env python3
"""안전 하한 T_floor와 그것으로 정의되는 '적시 경보' 지표.

T_floor는 학습으로 정하지 않는다. AI가 무엇이라 하든 이 시간보다 늦은 경보는
사람이 반응할 수 없으므로, 물리로 정하고 모델 아래에 깐다. 자율주행 쪽에서
안전 감시자(safety supervisor)를 두는 것과 같은 구조다 - 모델은 학습 데이터에
없던 상황에서 조용히 틀리기 때문에, 판단을 통째로 맡기지 않는다.

    T_floor = GPS 갱신 주기 + 전송 지연 + 인지·판단 + 정지 동작 + 안전 마진

각 항의 근거:

  GPS 갱신 주기 0.2s   NEO-6M을 5Hz로 올린 뒤의 값. 1Hz면 1.0s로 올려 잡아야
                       한다 - 위치가 묵은 만큼 더 일찍 울려야 하기 때문이다.
  전송 지연 0.1s       2026-08-05 실측. 위험도 변경 29건 전수에서 젯슨 계산
                       1.0ms + 무선 왕복 중앙값 102ms.
  인지·판단 0.3s       촉각 단순 반응시간은 120~180ms로 청각·시각보다 빠르지만,
                       그것은 버튼을 누르는 과제다. "진동을 느낌 -> 위험이구나
                       -> 멈추자"는 선택 반응이라 단순 반응의 1.5~2배로 잡는다.
  정지 동작 1.2s       보행 중 급정지 실험의 제동시간 0.84~1.21s(감속
                       0.91~1.57 m/s^2) 중 보수적인 끝을 쓴다.
  안전 마진 0.2s

합이 2.0초로, GB/T 33577(ISO 15623 준용)의 최소 2초 및 NHTSA NCAP FCW의
2.0~2.4초와 같은 자리에 떨어진다. 우리 시스템 실측, 인간 반응 문헌, 국제 기준이
서로 독립적으로 같은 값을 가리키는 셈이다.

한계: 시각장애인이 진동 경보를 받고 멈추기까지의 시간을 직접 측정한 연구는 찾지
못했다. 일반 보행자의 급정지 데이터로 대용했으며, 이는 명시해야 할 가정이다.
지팡이로 탐색하며 걷는 보행자는 보행 속도가 낮아 정지가 더 빠를 수도, 상황
파악이 늦어 더 느릴 수도 있다.
"""

import numpy as np

DEFAULT_BUDGET = {
    "gps_period_s": 0.2,
    "transport_s": 0.1,
    "perception_s": 0.3,
    "stopping_s": 1.2,
    "margin_s": 0.2,
}


def safety_floor_s(
    gps_period_s=DEFAULT_BUDGET["gps_period_s"],
    transport_s=DEFAULT_BUDGET["transport_s"],
    perception_s=DEFAULT_BUDGET["perception_s"],
    stopping_s=DEFAULT_BUDGET["stopping_s"],
    margin_s=DEFAULT_BUDGET["margin_s"],
):
    """물리 항의 합. 이 시간보다 늦은 경보는 반응이 불가능하다."""
    return gps_period_s + transport_s + perception_s + stopping_s + margin_s


T_FLOOR_S = safety_floor_s()


def timely_alarm_rate(df, alarm, floor_s=T_FLOOR_S):
    """위험 시나리오 중 '반응할 수 있을 만큼 미리' 울린 비율.

    재현율이 답하지 못하는 것을 답한다. 재현율은 위험 시점 하나하나를 세므로,
    라벨이 위험 10초 전부터 켜져 있으면 "아직 울릴 필요 없는 시점"까지 놓친
    것으로 세어 점수를 부당하게 깎는다. 반대로 위험 직전에만 울려도 재현율은
    올라가는데, 그 경보는 사람이 반응할 수 없어 쓸모가 없다.

    여기서는 시나리오 단위로 첫 경보 시각과 위험 실현 시각의 차를 보고, 그
    여유가 floor_s 이상일 때만 성공으로 센다.
    """
    y = df.y.to_numpy().astype(bool)
    alarm = np.asarray(alarm, dtype=bool)

    dangerous_ids = df.loc[y, "scenario_id"].unique()
    if len(dangerous_ids) == 0:
        return float("nan")

    timely = 0
    for sid in dangerous_ids:
        rows = df[df.scenario_id == sid]
        mask = alarm[rows.index.to_numpy()]
        if not mask.any():
            continue
        first_alarm_t = float(rows.t.to_numpy()[mask][0])
        hits = rows.t_hit.to_numpy()
        hits = hits[~np.isnan(hits)]
        if len(hits) and float(hits.min()) - first_alarm_t >= floor_s:
            timely += 1

    return timely / len(dangerous_ids)


def main():
    print(f"T_floor = {T_FLOOR_S:.2f} s")
    for name, value in DEFAULT_BUDGET.items():
        print(f"  {name:14s} {value:.2f}")
    print()
    print("GPS 주기별 하한:")
    for hz, period in ((1, 1.0), (5, 0.2), (10, 0.1)):
        print(f"  {hz:2d} Hz -> {safety_floor_s(gps_period_s=period):.2f} s")


if __name__ == "__main__":
    main()
