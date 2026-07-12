def classify_event(
    distance_m,
    relative_speed_mps,
    vehicle_speed_mps,
    ttc,
    risk_level,
    zone_type,
):
    """
    거리, 속도, TTC, 위험 수준과 Zone 정보를 이용해
    이벤트 유형을 분류한다.
    """

    # 충돌까지 남은 시간이 매우 짧은 고위험 상황
    if risk_level == 3 and ttc <= 2:
        return "Near Miss"

    # 등록된 주차장 출구에서 발생한 위험
    if zone_type == "Parking" and risk_level >= 1:
        return "Parking Exit Risk"

    # 등록된 교차로나 사각지대에서 발생한 위험
    if zone_type == "Intersection" and risk_level >= 1:
        return "Blind Spot Risk"

    # 차량이 매우 느리거나 정지한 상태에서 보행자에게 양보
    if vehicle_speed_mps <= 2 and distance_m <= 10:
        return "Vehicle Yield"

    # 서로 멀어지거나 접근하지 않는 상황
    if relative_speed_mps <= 0 and distance_m <= 10:
        return "Safe Pass"

    if risk_level == 2:
        return "Warning Approach"

    if risk_level == 1:
        return "Caution Approach"

    if risk_level == 3:
        return "High Risk Approach"

    return "Normal"