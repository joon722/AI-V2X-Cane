def calculate_ttc(distance_m, relative_speed_mps):
    """
    TTC(Time To Collision)를 계산한다.
    상대속도가 0 이하이면 접근하지 않는 것으로 판단한다.
    """

    if relative_speed_mps <= 0:
        return 9999.0

    return distance_m / relative_speed_mps


def calculate_risk_score(
    distance_m,
    relative_speed_mps,
    vehicle_speed_mps,
    ttc,
    zone_base_risk=0,
):
    """
    거리, TTC, 상대속도, 차량속도, Zone 정보를 이용하여
    0~100점 범위의 위험 점수를 계산한다.
    """

    score = 0.0

    # 1. 거리 점수: 최대 30점
    if distance_m <= 10:
        score += 30
    elif distance_m <= 20:
        score += 25
    elif distance_m <= 40:
        score += 18
    elif distance_m <= 60:
        score += 10
    elif distance_m <= 100:
        score += 5

    # 2. TTC 점수: 최대 35점
    if ttc <= 1:
        score += 35
    elif ttc <= 2:
        score += 30
    elif ttc <= 3:
        score += 25
    elif ttc <= 5:
        score += 20
    elif ttc <= 8:
        score += 15
    elif ttc <= 12:
        score += 8

    # 3. 상대속도 점수: 최대 20점
    if relative_speed_mps >= 20:
        score += 20
    elif relative_speed_mps >= 15:
        score += 17
    elif relative_speed_mps >= 10:
        score += 13
    elif relative_speed_mps >= 5:
        score += 8
    elif relative_speed_mps > 0:
        score += 3

    # 4. 차량속도 점수: 최대 10점
    if vehicle_speed_mps >= 20:
        score += 10
    elif vehicle_speed_mps >= 15:
        score += 8
    elif vehicle_speed_mps >= 10:
        score += 6
    elif vehicle_speed_mps >= 5:
        score += 3

    # 5. 주요 위험구역 보정: 최대 5점
    score += min(max(zone_base_risk, 0), 5)

    return round(min(score, 100), 2)


def classify_risk_level(risk_score):
    """
    위험 점수를 4단계로 분류한다.

    Level 0: 안전
    Level 1: 주의
    Level 2: 경고
    Level 3: 위험
    """

    if risk_score >= 70:
        return 3

    if risk_score >= 45:
        return 2

    if risk_score >= 20:
        return 1

    return 0


def analyze_risk(
    distance_m,
    relative_speed_mps,
    vehicle_speed_mps,
    zone_base_risk=0,
):
    ttc = calculate_ttc(
        distance_m=distance_m,
        relative_speed_mps=relative_speed_mps,
    )

    risk_score = calculate_risk_score(
        distance_m=distance_m,
        relative_speed_mps=relative_speed_mps,
        vehicle_speed_mps=vehicle_speed_mps,
        ttc=ttc,
        zone_base_risk=zone_base_risk,
    )

    risk_level = classify_risk_level(risk_score)

    return {
        "ttc": round(ttc, 2),
        "risk_score": risk_score,
        "risk_level": risk_level,
    }