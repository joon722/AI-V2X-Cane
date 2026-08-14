# -*- coding: utf-8 -*-
"""v5 학습 실행기 — v4.1과 같은 구조/특징(20개), 생성기 v3(진짜 교내 주행) 데이터만 사용.

출력 파일명은 Jetson 배포 호환을 위해 v3 이름을 유지한다 (models_v5/risk_transformer_v3.onnx 등).
"""
import sys
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8")
import train_transformer_v3 as t

t.DATA_FILE = Path(__file__).parent / "training_dataset_v5.csv"
t.OUT_DIR = Path(__file__).parent / "models_v5"

# v4.1과 동일: ttc 30초 클램프 + ttc_valid 포함 총 20개 특징
t.FEATURE_COLUMNS = [
    "ped_x", "ped_y", "veh_x", "veh_y",
    "ped_speed_mps", "veh_speed_mps",
    "distance_m", "rel_speed_mps", "ttc", "ttc_valid",
    "risk_score", "zone_base_risk",
    "dx", "dy", "dvx", "dvy", "dcpa_m",
    "phys_min_dist_3s", "phys_t_cpa", "phys_score_3s",
]

if __name__ == "__main__":
    t.main()
