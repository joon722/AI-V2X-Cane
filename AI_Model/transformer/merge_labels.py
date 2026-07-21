from pathlib import Path

import numpy as np
import pandas as pd


FEATURE_FILE = Path("master_dataset.csv")
LABEL_FILE = Path("dataset/event_result.csv")
OUTPUT_FILE = Path("labeled_master_dataset.csv")

KEY_COLUMNS = [
    "ts_ms",
    "ped_x",
    "ped_y",
    "veh_x",
    "veh_y",
]

LABEL_COLUMNS = [
    "ttc",
    "risk_score",
    "risk_level",
    "event_type",
    "event_id",
    "event_x",
    "event_y",
    "zone_id",
    "zone_name",
    "zone_type",
    "zone_distance_m",
    "zone_base_risk",
    "zone_speed_limit_kmh",
]


def main() -> None:
    if not FEATURE_FILE.exists():
        raise FileNotFoundError(f"파일 없음: {FEATURE_FILE}")

    if not LABEL_FILE.exists():
        raise FileNotFoundError(f"파일 없음: {LABEL_FILE}")

    features = pd.read_csv(FEATURE_FILE)
    labels = pd.read_csv(LABEL_FILE)

    print(f"master_dataset 행 수: {len(features)}")
    print(f"event_result 행 수: {len(labels)}")

    if len(features) != len(labels):
        raise ValueError(
            "두 파일의 행 수가 달라서 순서대로 병합할 수 없습니다."
        )

    # 두 파일이 같은 SUMO 결과인지 좌표와 시간으로 검증
    for column in KEY_COLUMNS:
        if column not in features.columns or column not in labels.columns:
            raise KeyError(f"검증용 컬럼 누락: {column}")

        feature_values = pd.to_numeric(
            features[column], errors="coerce"
        ).to_numpy()

        label_values = pd.to_numeric(
            labels[column], errors="coerce"
        ).to_numpy()

        if not np.allclose(
            feature_values,
            label_values,
            equal_nan=True,
            atol=1e-6,
        ):
            mismatch_count = int(
                np.sum(
                    ~np.isclose(
                        feature_values,
                        label_values,
                        equal_nan=True,
                        atol=1e-6,
                    )
                )
            )

            raise ValueError(
                f"{column} 값이 {mismatch_count}행에서 다릅니다. "
                "서로 다른 시뮬레이션 결과일 수 있습니다."
            )

    usable_label_columns = [
        column
        for column in LABEL_COLUMNS
        if column in labels.columns
    ]

    # 기존에 같은 이름의 컬럼이 있다면 중복 방지
    features = features.drop(
        columns=[
            column
            for column in usable_label_columns
            if column in features.columns
        ],
        errors="ignore",
    )

    merged = pd.concat(
        [
            features.reset_index(drop=True),
            labels[usable_label_columns].reset_index(drop=True),
        ],
        axis=1,
    )

    merged.to_csv(
        OUTPUT_FILE,
        index=False,
        encoding="utf-8-sig",
    )

    print(f"\n저장 완료: {OUTPUT_FILE}")
    print(f"최종 행 수: {len(merged)}")

    print("\nRisk Level 분포")
    print(
        merged["risk_level"]
        .value_counts(dropna=False)
        .sort_index()
    )

    print("\nEvent Type 분포")
    print(
        merged["event_type"]
        .value_counts(dropna=False)
    )


if __name__ == "__main__":
    main()