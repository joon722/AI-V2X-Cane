import sys
import time
import pandas as pd
import serial

csv_path = sys.argv[1] if len(sys.argv) > 1 else "minseo_outputs/scenario_front_clean.csv"
port = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyUSB0"
baud = 115200

df = pd.read_csv(csv_path)

if "timestep_time" in df.columns and "time" not in df.columns:
    df = df.rename(columns={"timestep_time": "time"})

df = df.sort_values("time").reset_index(drop=True)

for c in ["time", "risk_level"]:
    if c not in df.columns:
        raise ValueError(f"missing column: {c}")

ser = serial.Serial(port, baud, timeout=0, write_timeout=0)
print(f"[REPLAY] {csv_path} -> {port}")
time.sleep(3)  # ESP32 부팅 충분히 대기

# 버퍼 비우기
ser.reset_input_buffer()
ser.reset_output_buffer()

prev_risk = -1
for _, row in df.iterrows():
    risk = int(row["risk_level"])
    dist = float(row.get("distance_to_ped", -1))
    ttc = float(row.get("ttc", -1))
    t = float(row["time"])

    msg = f"RISK,{risk},{t:.2f},{dist:.2f},{ttc:.2f}\n"
    ser.write(msg.encode("utf-8"))
    ser.flush()
    print(msg.strip())

    if risk != prev_risk:
        time.sleep(1.5)  # risk 바뀔 때 반응 시간 충분히
    else:
        time.sleep(0.8)  # 같은 risk면 좀 더 빠르게

    prev_risk = risk

print("[REPLAY DONE]")
ser.close()