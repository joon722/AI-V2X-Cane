from pathlib import Path

import pandas as pd


PROJECT_DIR = Path(__file__).resolve().parent.parent
DATASET_DIR = PROJECT_DIR / "dataset"
OUTPUT_FILE = DATASET_DIR / "master_dataset.csv"


def main():
    scenario_folders = sorted(DATASET_DIR.glob("scenario_*"))

    all_data = []
    loaded_scenarios = []

    for scenario_folder in scenario_folders:
        event_file = scenario_folder / "event_result.csv"

        if not event_file.exists():
            print(
                f"건너뜀: {scenario_folder.name}에 "
                "event_result.csv가 없습니다."
            )
            continue

        try:
            df = pd.read_csv(event_file)
        except Exception as error:
            print(f"읽기 실패: {event_file}")
            print(f"오류 내용: {error}")
            continue

        # 어느 시나리오에서 생성된 데이터인지 표시
        df.insert(0, "scenario", scenario_folder.name)

        all_data.append(df)
        loaded_scenarios.append(scenario_folder.name)

        print(
            f"불러오기 완료: {scenario_folder.name} "
            f"({len(df)}행)"
        )

    if not all_data:
        print("합칠 event_result.csv 파일이 없습니다.")
        return

    master_dataset = pd.concat(
        all_data,
        ignore_index=True,
    )

    # AI 학습용 label 컬럼 추가
    # 현재 risk_level과 같은 값을 사용
    if "risk_level" in master_dataset.columns:
        master_dataset["label"] = master_dataset[
            "risk_level"
        ].astype(int)
    else:
        print("주의: risk_level 컬럼이 없어 label을 만들지 못했습니다.")

    master_dataset.to_csv(
        OUTPUT_FILE,
        index=False,
        encoding="utf-8-sig",
    )

    print("\n" + "=" * 45)
    print("Master Dataset 생성 완료")
    print("포함 시나리오:", len(loaded_scenarios))
    print("시나리오 목록:", ", ".join(loaded_scenarios))
    print("전체 데이터 수:", len(master_dataset))
    print("전체 컬럼 수:", len(master_dataset.columns))
    print("저장 위치:", OUTPUT_FILE)

    if "label" in master_dataset.columns:
        print("\nLabel별 데이터 개수")
        print(
            master_dataset["label"]
            .value_counts()
            .sort_index()
        )

    print("=" * 45)


if __name__ == "__main__":
    main()