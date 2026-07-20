from pathlib import Path

import pandas as pd


def run():
    current_dir = Path(__file__).resolve().parent

    input_file = current_dir / "feature_9cols.csv"
    output_dir = current_dir / "output"
    output_file = output_dir / "risk_result.csv"

    if not input_file.exists():
        raise FileNotFoundError(
            f"입력 파일을 찾을 수 없습니다: {input_file}"
        )

    output_dir.mkdir(parents=True, exist_ok=True)

    # 1. 데이터 읽기
    df = pd.read_csv(input_file)

    print("데이터 개수:", len(df))

    required_columns = ["distance_m", "rel_speed_mps"]

    missing_columns = [
        column
        for column in required_columns
        if column not in df.columns
    ]

    if missing_columns:
        raise ValueError(
            f"필요한 컬럼이 없습니다: {missing_columns}"
        )

    # 2. TTC 계산
    closing_speed = df["rel_speed_mps"]

    df["ttc"] = (
        df["distance_m"]
        .div(closing_speed.where(closing_speed > 0))
        .fillna(9999)
    )

    # 3. Rule 기반 위험도 계산
    def calculate_risk_level(distance):
        if distance < 3:
            return 3
        if distance < 10:
            return 2
        if distance < 30:
            return 1
        return 0

    df["risk_level"] = df["distance_m"].apply(
        calculate_risk_level
    )

    # 4. 결과 저장
    df.to_csv(output_file, index=False)

    print("위험도 계산 완료")
    print("저장 위치:", output_file)
    print(df.head())

    return df


if __name__ == "__main__":
    run()