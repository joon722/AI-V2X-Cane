import pandas as pd
import glob
import os

all_data = []

folders = glob.glob("../dataset/scenario_*")

for folder in folders:
    file = os.path.join(folder, "feature.csv")

    if os.path.exists(file):
        df = pd.read_csv(file)
        df["scenario"] = os.path.basename(folder)
        all_data.append(df)

if len(all_data) == 0:
    print("데이터가 없습니다.")
else:
    master = pd.concat(all_data, ignore_index=True)
    master.to_csv("../master_dataset.csv", index=False)
    print("완료!")
    print(master.head())