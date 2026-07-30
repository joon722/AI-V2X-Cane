import numpy as np
import pandas as pd
from pathlib import Path

OUT = Path("minseo_outputs")
OUT.mkdir(exist_ok=True)

# 보행자(지팡이) 고정 위치
PED_X, PED_Y = 5.0, 10.0

def add_risk_columns(g):
    g = g.copy().sort_values("time")
    dx = g["x"] - PED_X
    dy = g["y"] - PED_Y
    g["ped_x"] = PED_X
    g["ped_y"] = PED_Y
    g["distance_to_ped"] = np.sqrt(dx*dx + dy*dy)
    dt = g["time"].diff().fillna(0.1).replace(0, 0.1)
    dd = -g["distance_to_ped"].diff().fillna(0)
    g["relative_speed"] = (dd / dt).clip(lower=0)
    g["ttc"] = np.where(g["relative_speed"] > 0.1, g["distance_to_ped"] / g["relative_speed"], 999.0)
    risk = np.zeros(len(g), dtype=int)
    risk[(g["distance_to_ped"] < 10) | (g["ttc"] < 5)] = 1
    risk[(g["distance_to_ped"] < 6) | (g["ttc"] < 3)] = 2
    risk[(g["distance_to_ped"] < 3) | (g["ttc"] < 1.5)] = 3
    g["risk_level"] = risk
    return g

# 시나리오 1: 정면 접근 (차량이 보행자를 향해 다가옴)
t = np.arange(0, 10.1, 0.1)
front = pd.DataFrame({
    "time": t, "vehicle_id": "car_01",
    "x": 30 - 2.8*t, "y": 10.0, "speed": 2.8
})
front = add_risk_columns(front)
front["scenario"] = "front_approach"
front.to_csv(OUT / "scenario_front_approach.csv", index=False)

# 시나리오 2: 측면 접근 (차량이 옆에서 다가옴)
side = pd.DataFrame({
    "time": t, "vehicle_id": "side_car_01",
    "x": PED_X + 1.0, "y": PED_Y + 22 - 2.2*t, "speed": 2.2
})
side = add_risk_columns(side)
side["scenario"] = "side_blindspot"
side.to_csv(OUT / "scenario_side_blindspot.csv", index=False)

# 시나리오 3: 안전 통과 (차량이 멀리서 지나감)
safe = pd.DataFrame({
    "time": t, "vehicle_id": "safe_car_01",
    "x": PED_X + 20 + 2.0*t, "y": PED_Y + 15, "speed": 2.0
})
safe = add_risk_columns(safe)
safe["scenario"] = "safe_pass"
safe.to_csv(OUT / "scenario_safe_pass.csv", index=False)

# 결과 확인
print("=== 정면 접근 risk 분포 ===")
print(front["risk_level"].value_counts().sort_index())
print("\n=== 측면 접근 risk 분포 ===")
print(side["risk_level"].value_counts().sort_index())
print("\n=== 안전 통과 risk 분포 ===")
print(safe["risk_level"].value_counts().sort_index())
print("\nCSV 파일 생성 완료:", list(OUT.glob("*.csv")))