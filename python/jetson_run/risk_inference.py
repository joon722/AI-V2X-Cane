import pandas as pd

# ==========================
# 1. 데이터 읽기
# ==========================

df = pd.read_csv("feature_9cols.csv")

print("데이터 개수 :", len(df))


# ==========================
# 2. TTC 계산
# ==========================

ttc = []

for _, row in df.iterrows():

    distance = row["distance_m"]
    relative_speed = row["rel_speed_mps"]

    if relative_speed > 0:
        ttc.append(distance / relative_speed)
    else:
        ttc.append(9999)

df["ttc"] = ttc


# ==========================
# 3. 위험도 계산
# ==========================

risk = []

for _, row in df.iterrows():

    if row["distance_m"] < 3:
        risk.append(3)

    elif row["distance_m"] < 10:
        risk.append(2)

    elif row["distance_m"] < 30:
        risk.append(1)

    else:
        risk.append(0)

df["risk_level"] = risk


# ==========================
# 4. 저장
# ==========================

df.to_csv("risk_result.csv", index=False)

print("완료!")

print(df.head())