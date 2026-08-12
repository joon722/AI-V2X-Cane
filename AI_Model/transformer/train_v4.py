# -*- coding: utf-8 -*-
"""v4 학습 실행기 — v3와 같은 구조/특징으로, 현실성 v2 시나리오 데이터만 사용"""
import sys
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8")
import train_transformer_v3 as t

t.DATA_FILE = Path(__file__).parent / "training_dataset_v4.csv"
t.OUT_DIR = Path(__file__).parent / "models_v4"

# 팀 리뷰 반영: ttc 30초 클램프 + ttc_valid 이진 특징 추가 (총 20개)
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
