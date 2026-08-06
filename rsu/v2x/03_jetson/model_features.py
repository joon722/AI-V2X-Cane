#!/usr/bin/env python3
"""모델 입력 피처를 한 시점씩 계산한다 (젯슨 스트리밍용).

학습은 ttc_study/features.py가 궤적 전체를 배열로 받아 만든 피처로 했다. 젯슨은
한 시점씩 흘러오므로 같은 값을 다른 방식으로 계산해야 하고, 하나라도 어긋나면
학습한 적 없는 입력을 넣는 셈이 되어 시뮬레이션에서 측정한 성능이 무의미해진다.
그래서 test_model_features.py가 두 경로의 값을 소수점까지 대조한다.

numpy를 쓰지 않는다. step6_kinematics가 칼만 필터를 순수 파이썬으로 구현해 둔
이유와 같다 - 젯슨에 무엇도 새로 설치하지 않기 위해서다.

변화율은 후방차분이다. 미래를 볼 수 없는 것이 스트리밍의 조건이고, 학습 쪽도
같은 이유로 후방차분을 쓴다.
"""

import math

from step6_kinematics import relative_kinematics

# 아래 상수는 ttc_study/features.py와 같은 값이어야 한다. 학습과 추론이 다른
# 기준으로 자르면 같은 상황에 다른 피처가 만들어진다.
TTC_MAX_S = 30.0
PHYS_HORIZON_S = 3.0
MAX_REL_ACCEL_MPS2 = 20.0
MAX_BEARING_RATE_DPS = 180.0

FEATURE_ORDER = (
    "distance_m",
    "closing_los",
    "ttc",
    "veh_speed_mps",
    "ped_speed_mps",
    "dx",
    "dy",
    "dvx",
    "dvy",
    "dcpa_m",
    "tcpa_s",
    "phys_min_dist_3s",
    "phys_t_cpa_3s",
    "d_closing_dt",
    "d_heading_dt",
)


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def _wrap_degrees(delta):
    """각도 차를 -180~180으로 접는다. 배치 쪽 np.unwrap과 같은 일을 한다."""
    while delta > 180.0:
        delta -= 360.0
    while delta < -180.0:
        delta += 360.0
    return delta


def _phys_extrapolation(rx, ry, vx, vy, horizon_s=PHYS_HORIZON_S):
    """등속 가정으로 horizon 안의 최소 거리와 그 시각. features.py와 같은 식."""
    v2 = vx * vx + vy * vy
    t_star = 0.0 if v2 <= 0 else -(rx * vx + ry * vy) / v2
    t_star = max(0.0, min(t_star, horizon_s))
    return math.hypot(rx + vx * t_star, ry + vy * t_star), t_star


class StreamingFeatures:
    """관측을 한 시점씩 먹여 피처 딕셔너리를 얻는다.

    변화율을 내려면 직전 값이 필요하므로 상태를 들고 있다. 노드가 오래 끊겼다
    돌아오면 그 사이의 차분은 의미가 없으므로 reset()으로 지운다.
    """

    def __init__(self):
        self.prev_time = None
        self.prev_closing = None
        self.prev_bearing = None

    def reset(self):
        self.__init__()

    def update(self, now, cane_state, veh_state):
        """cane_state, veh_state 는 (동, 북, 동속, 북속) 미터·미터/초.

        step6 KinematicsPipeline의 trackers[...].state_ahead()가 주는 형식 그대로다.
        """
        cane_e, cane_n, cane_ve, cane_vn = cane_state
        veh_e, veh_n, veh_ve, veh_vn = veh_state

        kin = relative_kinematics(
            (cane_e, cane_n), (cane_ve, cane_vn),
            (veh_e, veh_n), (veh_ve, veh_vn),
        )

        dx = veh_e - cane_e
        dy = veh_n - cane_n
        dvx = veh_ve - cane_ve
        dvy = veh_vn - cane_vn

        # None 처리 규약도 features.py와 같아야 한다. dcpa가 없다는 것은
        # 최근접이 앞에 없다는 뜻이므로 현재 거리로, tcpa는 0으로 채운다.
        ttc = TTC_MAX_S if kin.ttc_simple is None else min(kin.ttc_simple, TTC_MAX_S)
        dcpa = kin.distance_m if kin.dcpa is None else kin.dcpa
        tcpa = 0.0 if kin.tcpa is None else min(kin.tcpa, TTC_MAX_S)

        phys_dist, phys_t = _phys_extrapolation(dx, dy, dvx, dvy)
        bearing = math.degrees(math.atan2(dx, dy))

        d_closing = 0.0
        d_heading = 0.0
        if self.prev_time is not None:
            dt = now - self.prev_time
            if dt > 0:
                d_closing = _clamp((kin.closing_los - self.prev_closing) / dt,
                                   MAX_REL_ACCEL_MPS2)
                d_heading = _clamp(_wrap_degrees(bearing - self.prev_bearing) / dt,
                                   MAX_BEARING_RATE_DPS)

        self.prev_time = now
        self.prev_closing = kin.closing_los
        self.prev_bearing = bearing

        return {
            "distance_m": kin.distance_m,
            "closing_los": kin.closing_los,
            "ttc": ttc,
            "veh_speed_mps": math.hypot(veh_ve, veh_vn),
            "ped_speed_mps": math.hypot(cane_ve, cane_vn),
            "dx": dx,
            "dy": dy,
            "dvx": dvx,
            "dvy": dvy,
            "dcpa_m": dcpa,
            "tcpa_s": tcpa,
            "phys_min_dist_3s": phys_dist,
            "phys_t_cpa_3s": phys_t,
            "d_closing_dt": d_closing,
            "d_heading_dt": d_heading,
        }
