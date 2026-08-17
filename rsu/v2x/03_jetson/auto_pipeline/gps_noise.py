"""SUMO의 완벽한 좌표에 실제 GPS 오차를 입힌다.

SUMO는 오차 없는 참값을 준다. 반면 현장의 지팡이/차량은 GPS로 위치를 얻으므로
몇 미터씩 흔들린 좌표를 받는다. 그 격차(sim-to-real gap)를 그대로 두면
완벽한 좌표에서만 잘 도는 판정 로직이 만들어진다. 특히 DCPA는 위치를 미분해
상대속도를 구하므로 위치 노이즈에 민감하다.

오차 모델은 1차 Gauss-Markov 과정을 쓴다. 실제 GPS 오차는 매 순간 새로 튀는
백색잡음이 아니라 수십 초 동안 한쪽으로 치우쳐 흘러가기 때문이다(전리층 지연,
다중경로, 위성 배치). 축(x, y)마다 독립이고, 객체마다 독립이다.

    e[k] = phi * e[k-1] + w[k],  phi = exp(-dt / tau)

정상상태 분산을 sigma^2으로 유지하려고 w의 표준편차를 sigma*sqrt(1-phi^2)로 두고,
첫 표본도 N(0, sigma)에서 뽑는다. 0에서 출발시키면 시나리오 앞부분이 실제보다
정확해져서 판정이 낙관적으로 나온다.

sigma 기본값 2.5m의 근거: 2026-08-01 야외 정지 세션 실측에서 지팡이 GPS의
위치 흩어짐이 sigma 약 2m(최악 4m)로 측정되었고, step7_risk.py가 DCPA 게이트
임계값을 유도할 때 쓰는 GPS_SIGMA_M 과 같은 값이다.

이상치(순간적으로 크게 튀는 값)는 넣지 않는다. 위 실측값은 펌웨어의 이상치
제거를 거친 뒤의 좌표에서 측정한 것이고, 파이프라인이 받는 것도 같은 단계의
값이기 때문이다.

process_scenarios.py의 build_features() 안에서, 좌표를 읽은 직후이자
distance_m 을 계산하기 전에 부른다. 그 위치여야 거리/TTC/risk_score/DCPA가
전부 오차 있는 좌표를 쓰게 되어 현장과 같아진다.

    from gps_noise import add_gps_noise
    df = add_gps_noise(df, seed=scenario_seed(scenario_dir.name))

위치 정확도가 달라지면 판정이 얼마나 흔들리는지 보려면 sigma_m 만 바꿔
같은 원본을 다시 돌려 비교한다.
"""
import math
import zlib

import numpy as np

# 실측 기반 기본값. step7_risk.GPS_SIGMA_M 과 같은 값을 쓴다.
DEFAULT_SIGMA_M = 2.5
# 오차가 얼마나 오래 같은 방향으로 머무는지(초). GPS 표준적인 범위.
DEFAULT_TAU_S = 60.0

TIME_COL = "timestep_time"
VEHICLE_COL = "vehicle_id"
VEHICLE_XY = ("veh_x", "veh_y")
PED_XY = ("ped_x", "ped_y")


def scenario_seed(name):
    """시나리오 이름 -> 안정적인 정수 seed.

    같은 시나리오를 다시 처리하면 같은 오차가 재현되도록 한다. 파이썬 hash()는
    실행마다 달라지므로 쓰지 않는다.
    """
    return zlib.crc32(str(name).encode("utf-8"))


def _gauss_markov(times, sigma_m, tau_s, rng):
    """시간 상관을 갖는 1축 위치 오차를 times 길이만큼 생성."""
    n = len(times)
    error = np.empty(n, dtype=float)
    error[0] = rng.normal(0.0, sigma_m)
    for i in range(1, n):
        dt = times[i] - times[i - 1]
        phi = math.exp(-dt / tau_s) if dt > 0 else 1.0
        drive = sigma_m * math.sqrt(max(0.0, 1.0 - phi * phi))
        error[i] = phi * error[i - 1] + rng.normal(0.0, drive)
    return error


def _offset_track(times, sigma_m, tau_s, seed_parts):
    """한 객체의 (x오차, y오차). 축끼리도 독립이다."""
    rng = np.random.default_rng(list(seed_parts))
    return (_gauss_markov(times, sigma_m, tau_s, rng),
            _gauss_markov(times, sigma_m, tau_s, rng))


def add_gps_noise(df, sigma_m=DEFAULT_SIGMA_M, tau_s=DEFAULT_TAU_S, seed=0):
    """차량·보행자 좌표에 GPS 오차를 입힌 df를 반환 (제자리 수정).

    차량은 vehicle_id 마다 독립적인 오차 궤적을 갖는다. 보행자는 한 명이므로
    시각별로 하나의 궤적을 만들어 모든 차량 행에 같은 값을 적용한다 - 같은 순간의
    보행자가 차량마다 다른 곳에 있으면 안 되기 때문이다.
    """
    if sigma_m <= 0:
        return df

    times = df[TIME_COL].to_numpy(dtype=float)

    ped_times = np.unique(times)
    ped_ex, ped_ey = _offset_track(ped_times, sigma_m, tau_s, (seed, 0))
    at = np.searchsorted(ped_times, times)
    df[PED_XY[0]] = df[PED_XY[0]].to_numpy(dtype=float) + ped_ex[at]
    df[PED_XY[1]] = df[PED_XY[1]].to_numpy(dtype=float) + ped_ey[at]

    vehicle_ids = df[VEHICLE_COL].to_numpy()
    veh_x = df[VEHICLE_XY[0]].to_numpy(dtype=float, copy=True)
    veh_y = df[VEHICLE_XY[1]].to_numpy(dtype=float, copy=True)
    for vid in np.unique(vehicle_ids):
        rows = np.flatnonzero(vehicle_ids == vid)
        order = rows[np.argsort(times[rows], kind="stable")]
        ex, ey = _offset_track(times[order], sigma_m, tau_s,
                               (seed, 1, scenario_seed(vid)))
        veh_x[order] += ex
        veh_y[order] += ey
    df[VEHICLE_XY[0]] = veh_x
    df[VEHICLE_XY[1]] = veh_y
    return df


# ---------------------------------------------------------------------------
# 스트림 수준 현장 노이즈 모델 (2026-08-17 오후 실측 기반)
#
# 위 add_gps_noise 는 좌표 오차 하나만 입힌다. 실기에서 판정을 흔든 것은 그 외에
# 네 가지가 더 있었다(설명서/실험리뷰_오후_20260817.md §2 C·D):
#
#   양자화   패킷 lat/lng 가 float32 -> 위도 0.42 m / 경도 0.53 m 격자. 5 Hz 에서
#            한 칸 뜀 = 2~2.7 m/s 순간속도. 정지 시 KF 접근속도 잡음의 큰 몫.
#   무효     노드가 3 초간 fix 를 채택 못 하면 gps_valid=0, lat=lng=0, speed=0,
#            heading=0 을 보낸다. 3~6 초 지속이 흔했다.
#   재획득   무효 뒤 첫 fix 는 이전 위치에서 수 m~19 m 떨어져 시작한다(펌웨어 필터
#            재초기화 + 그 사이 쌓인 바이어스). KF 가 이걸 이동으로 읽어 유령 접근.
#   heading  GPS 이동방향이라 speed>=0.4 m/s 일 때만 heading_valid=1. 아니면 0 또는
#            직전 값(stale). 그래서 RC 속도에선 대부분 무효.
#   반복송신 fix 는 5 Hz, 패킷은 10 Hz - 같은 fix 가 두 번 온다. seq 는 패킷마다 +1.
#
# 여기서는 깨끗한 궤적(t, east, north, speed, heading)을 받아 "RSU 가 실제로 받는
# 패킷" 목록으로 바꾼다. 이 패킷을 젯슨 파이프라인(step6 KF -> model_features)에
# 그대로 흘리면 학습 특징 = 실행 특징이 되어 sim-to-real 격차가 줄어든다.
#
# 장비·프로토콜 고유값(양자화, heading 문턱, 5/10 Hz, 무효 시 0 송신)은 어디서나
# 같으므로 고정. 환경 의존값(sigma, 무효 빈도·길이, 점프 크기, 패킷 손실)은 실측을
# 대표값으로 두되 randomized() 로 범위 안에서 흔든다 - 한 장소에서 잰 잡음을
# 다른 장소에도 일반화하기 위한 도메인 랜덤화.
# ---------------------------------------------------------------------------
import struct
from dataclasses import dataclass

METERS_PER_DEGREE_LAT = 111320.0


@dataclass
class FieldNoiseParams:
    # --- 환경 의존 (실측 대표값; randomized() 가 범위 안에서 흔든다) ---
    sigma_m: float = 2.5              # 위치 오차 정상상태 sigma(축별). 8/12 좋은 조건 1.6~2.1, 8/13 배회 26 m 까지
    tau_s: float = 60.0               # 오차 상관시간 (Gauss-Markov)
    dropout_per_min: float = 1.5      # GPS 무효 에피소드 빈도 (8/17 오후 차량: 5~10 분당 수 회)
    dropout_s: tuple = (3.0, 6.0)     # 무효 지속 (균등)
    reacq_jump_m: tuple = (0.0, 12.0) # 재획득 첫 fix 의 추가 오프셋 크기 (균등, 방향 무작위; 8/17 최대 19 m)
    packet_loss: float = 0.05         # 패킷 손실 비율 (8/17 seq 결손 2~10 %)
    speed_sigma_mps: float = 0.1      # 도플러 속도 잡음
    heading_sigma_deg: float = 5.0    # 유효 heading 잡음
    heading_zero_prob: float = 0.75   # 무효 heading 이 0 으로 오는 비율 (나머지는 stale). 8/17: 73~89 %
    # --- 장비·프로토콜 고유 (고정) ---
    quantize_float32: bool = True
    heading_min_speed_mps: float = 0.4
    fix_hz: float = 5.0
    packet_hz: float = 10.0
    # --- 선택: 노드 결함 재현 (기본 꺼짐; P0 로 고칠 대상이라 학습엔 넣지 않는다) ---
    stall_period_s: float | None = None   # 차량: 60 초마다
    stall_len_s: float = 10.8             #   10.8 초 송신 정지
    reboot_per_min: float = 0.0           # 지팡이: 분당 재부팅 횟수
    reboot_silence_s: float = 7.0         #   재부팅 무음 (seq 0 부터 다시)

    @classmethod
    def randomized(cls, rng):
        """환경 의존값만 범위 안에서 무작위. 고유값은 그대로."""
        lo = float(rng.uniform(0.0, 8.0))
        return cls(
            sigma_m=float(rng.uniform(1.5, 6.0)),
            tau_s=float(rng.uniform(20.0, 120.0)),
            dropout_per_min=float(rng.uniform(0.0, 4.0)),
            dropout_s=(3.0, float(rng.uniform(3.0, 8.0))),
            reacq_jump_m=(lo, float(rng.uniform(lo, 20.0))),
            packet_loss=float(rng.uniform(0.0, 0.15)),
            speed_sigma_mps=float(rng.uniform(0.05, 0.3)),
            heading_sigma_deg=float(rng.uniform(2.0, 15.0)),
            heading_zero_prob=float(rng.uniform(0.3, 1.0)),
        )


def _float32(value):
    return struct.unpack("f", struct.pack("f", value))[0]


def _to_latlng(east, north, origin):
    lat0, lng0 = origin
    lat = lat0 + north / METERS_PER_DEGREE_LAT
    lng = lng0 + east / (METERS_PER_DEGREE_LAT * math.cos(math.radians(lat0)))
    return lat, lng


def _gm_step(prev, dt, sigma, tau, rng):
    """Gauss-Markov 1축 한 걸음. sigma 0 이면 항상 0."""
    if sigma <= 0.0:
        return 0.0
    phi = math.exp(-dt / tau) if dt > 0 else 1.0
    return phi * prev + rng.normal(0.0, sigma * math.sqrt(max(0.0, 1.0 - phi * phi)))


def noisify_track(samples, params, rng, origin, node_type, node_id=None, start_seq=0):
    """깨끗한 궤적 -> RSU 가 받는 패킷 목록.

    samples: (t_s, east_m, north_m, speed_mps, heading_deg) 를 fix 주기(params.fix_hz)로.
    origin:  (lat0, lng0) - east/north 를 lat/lng 로 바꿀 기준.
    node_type: "cane" | "vehicle". node_id 는 없으면 타입별 실기값.
    반환: pc_time 오름차순 dict 목록 (raw 로그의 RX JSON 과 같은 키).
    """
    node_id = node_id if node_id is not None else (4125577512 if node_type == "cane" else 2882630492)
    repeats = max(1, int(round(params.packet_hz / params.fix_hz)))
    packet_dt = 1.0 / params.packet_hz
    fix_dt = 1.0 / params.fix_hz

    packets = []
    seq = start_seq
    err_e = rng.normal(0.0, params.sigma_m) if params.sigma_m > 0 else 0.0
    err_n = rng.normal(0.0, params.sigma_m) if params.sigma_m > 0 else 0.0
    jump_e = jump_n = 0.0          # 재획득 오프셋: 바이어스처럼 tau 로 감쇠
    dropout_until = -1.0
    prev_in_dropout = False
    last_heading = 0.0
    stall_next = params.stall_period_s if params.stall_period_s else None
    silence_until = -1.0
    prev_t = None
    dropout_rate = params.dropout_per_min / 60.0
    reboot_rate = params.reboot_per_min / 60.0

    for t, east, north, speed, heading in samples:
        dt = 0.0 if prev_t is None else t - prev_t
        prev_t = t
        err_e = _gm_step(err_e, dt, params.sigma_m, params.tau_s, rng)
        err_n = _gm_step(err_n, dt, params.sigma_m, params.tau_s, rng)
        if dt > 0 and params.tau_s > 0:
            decay = math.exp(-dt / params.tau_s)
            jump_e *= decay
            jump_n *= decay

        # 노드 결함(선택): 송신 정지 / 재부팅 무음
        if stall_next is not None and t >= stall_next:
            silence_until = max(silence_until, t + params.stall_len_s)
            stall_next += params.stall_period_s
        if reboot_rate > 0 and dt > 0 and rng.random() < reboot_rate * dt:
            silence_until = max(silence_until, t + params.reboot_silence_s)
            seq = 0
        if t < silence_until:
            continue

        # GPS 무효 에피소드 시작 / 지속
        in_dropout = t < dropout_until
        if not in_dropout and dropout_rate > 0 and dt > 0 and rng.random() < dropout_rate * dt:
            dropout_until = t + rng.uniform(*params.dropout_s)
            in_dropout = True
        if in_dropout:
            lat = lng = 0.0
            gps_valid = 0
            spd = 0.0
            hd, hv = 0.0, 0
        else:
            if prev_in_dropout:
                # 재획득: 펌웨어 필터가 새 raw fix 에서 다시 시작하므로 그 순간의
                # 오프셋은 이전 것과 무관하게 새로 뽑힌다(더해지지 않는다).
                mag = rng.uniform(*params.reacq_jump_m)
                ang = rng.uniform(0.0, 2.0 * math.pi)
                jump_e, jump_n = mag * math.cos(ang), mag * math.sin(ang)
            gps_valid = 1
            spd = speed + (rng.normal(0.0, params.speed_sigma_mps) if params.speed_sigma_mps > 0 else 0.0)
            spd = max(0.0, spd)
            if speed >= params.heading_min_speed_mps:
                hd = heading + (rng.normal(0.0, params.heading_sigma_deg) if params.heading_sigma_deg > 0 else 0.0)
                hd %= 360.0
                hv = 1
                last_heading = hd
            else:
                hv = 0
                hd = 0.0 if rng.random() < params.heading_zero_prob else last_heading
            e = east + err_e + jump_e
            n = north + err_n + jump_n
            lat, lng = _to_latlng(e, n, origin)
            if params.quantize_float32:
                lat, lng = _float32(lat), _float32(lng)
            lat, lng = round(lat, 6), round(lng, 6)  # 브리지가 %.6f 로 찍는다

        for k in range(repeats):
            if params.packet_loss > 0 and rng.random() < params.packet_loss:
                seq += 1
                continue
            packets.append({
                "pc_time": round(t + k * packet_dt, 3),
                "type": node_type,
                "node_id": node_id,
                "seq": seq,
                "gps_valid": gps_valid,
                "lat": lat,
                "lng": lng,
                "speed_mps": round(spd, 3),
                "heading_deg": round(hd, 2),
                "heading_valid": hv,
                "node_risk": 0,
            })
            seq += 1
        prev_in_dropout = in_dropout

    return packets
