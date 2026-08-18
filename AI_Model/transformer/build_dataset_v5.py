# -*- coding: utf-8 -*-
"""v5 학습 데이터셋 생성 실행기 — 생성기 v3(진짜 교내 주행) 시나리오 전용.

build_dataset_local.py의 로직(팀 공식 동결 사본)을 그대로 쓰고
입력/출력 경로만 v5용으로 바꾼다.
입력: v5data/scenario_*/ (Jetson incoming_data에서 수신, campus_*/숫자 id 혼합)
출력: training_dataset_v5base.csv
"""
import sys
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8")
import build_dataset_local as b

b.DATA_DIR = Path(__file__).parent / "v5data"
b.OUT_FILE = Path(__file__).parent / "training_dataset_v5base.csv"

# 사용법: python build_dataset_v5.py            (깨끗한 좌표, 기존과 동일)
#         python build_dataset_v5.py --gps-noise (GPS 오차 주입 → *_noisy.csv 로 저장)
if __name__ == "__main__":
    if "--gps-noise" in sys.argv:
        b.GPS_NOISE = True
        b.OUT_FILE = Path(__file__).parent / "training_dataset_v5base_noisy.csv"
        print("GPS 오차 주입 켜짐 (gps_noise.add_gps_noise, sigma 2.5 m 실측값)")
    b.main()
