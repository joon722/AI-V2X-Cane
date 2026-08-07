# -*- coding: utf-8 -*-
"""v4 학습 실행기 — v3와 같은 구조/특징으로, 현실성 v2 시나리오 데이터만 사용"""
import sys
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8")
import train_transformer_v3 as t

t.DATA_FILE = Path(__file__).parent / "training_dataset_v4.csv"
t.OUT_DIR = Path(__file__).parent / "models_v4"

if __name__ == "__main__":
    t.main()
