import shutil
import subprocess
import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parent.parent
DATASET_DIR = PROJECT_DIR / "dataset"
JETSON_DIR = PROJECT_DIR / "python" / "jetson_run"


def run_command(command):
    print("\n실행:", " ".join(map(str, command)))

    result = subprocess.run(
        command,
        cwd=PROJECT_DIR,
        check=False,
    )

    if result.returncode != 0:
        raise RuntimeError(
            f"명령 실행 실패: {' '.join(map(str, command))}"
        )


def update_event_analyzer_paths(scenario_name):
    analyzer_file = PROJECT_DIR / "scripts" / "event_analyzer.py"

    text = analyzer_file.read_text(encoding="utf-8")

    lines = text.splitlines()
    new_lines = []

    for line in lines:
        if line.startswith("INPUT_FILE ="):
            new_lines.append(
                'INPUT_FILE = PROJECT_DIR / "dataset" '
                f'/ "{scenario_name}" / "feature.csv"'
            )
        elif line.startswith("OUTPUT_FILE ="):
            new_lines.append(
                'OUTPUT_FILE = PROJECT_DIR / "dataset" '
                f'/ "{scenario_name}" / "event_result.csv"'
            )
        else:
            new_lines.append(line)

    analyzer_file.write_text(
        "\n".join(new_lines) + "\n",
        encoding="utf-8",
    )


def main():
    scenario_name = input(
        "실행할 시나리오 이름을 입력하세요 "
        "(예: scenario_000): "
    ).strip()

    if not scenario_name:
        print("시나리오 이름이 입력되지 않았습니다.")
        return

    scenario_dir = DATASET_DIR / scenario_name
    feature_file = scenario_dir / "feature.csv"

    if not scenario_dir.exists():
        raise FileNotFoundError(
            f"시나리오 폴더를 찾을 수 없습니다: {scenario_dir}"
        )

    if not feature_file.exists():
        raise FileNotFoundError(
            f"feature.csv를 찾을 수 없습니다: {feature_file}"
        )

    jetson_input = JETSON_DIR / "feature_9cols.csv"
    jetson_output = JETSON_DIR / "output" / "risk_result.csv"

    scenario_risk_output = scenario_dir / "risk_result.csv"
    scenario_event_output = scenario_dir / "event_result.csv"

    print("\n" + "=" * 45)
    print("Scenario 자동 분석")
    print("대상:", scenario_name)
    print("=" * 45)

    # 1. feature.csv를 Jetson 입력 위치로 복사
    shutil.copy2(feature_file, jetson_input)
    print("Jetson 입력 파일 복사 완료")

    # 2. Rule 기반 위험도 계산
    run_command(
        [
            sys.executable,
            str(JETSON_DIR / "main.py"),
        ]
    )

    # 3. risk_result.csv 저장
    if not jetson_output.exists():
        raise FileNotFoundError(
            f"위험도 결과 파일이 없습니다: {jetson_output}"
        )

    shutil.copy2(
        jetson_output,
        scenario_risk_output,
    )

    print("risk_result.csv 저장 완료")

    # 4. event_analyzer.py 입력·출력 경로 변경
    update_event_analyzer_paths(scenario_name)
    print("event_analyzer.py 경로 설정 완료")

    # 5. 개선된 이벤트 분석
    run_command(
        [
            sys.executable,
            str(PROJECT_DIR / "scripts" / "event_analyzer.py"),
        ]
    )

    if not scenario_event_output.exists():
        raise FileNotFoundError(
            f"이벤트 결과 파일이 없습니다: "
            f"{scenario_event_output}"
        )

    print("\n" + "=" * 45)
    print("Scenario 분석 완료")
    print("저장 폴더:", scenario_dir)
    print("=" * 45)


if __name__ == "__main__":
    main()