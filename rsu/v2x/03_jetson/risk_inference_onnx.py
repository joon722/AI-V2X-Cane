import json
from pathlib import Path

import numpy as np
import onnxruntime as ort
import pandas as pd


# =========================================================
# 파일 경로 설정
# =========================================================

BASE_DIR = Path(__file__).resolve().parent

MODEL_PATH = BASE_DIR / "risk_transformer.onnx"
SCALER_PATH = BASE_DIR / "scaler.json"
INPUT_CSV_PATH = BASE_DIR / "labeled_master_dataset.csv"
OUTPUT_CSV_PATH = BASE_DIR / "onnx_risk_result.csv"

BATCH_SIZE = 256


# =========================================================
# Softmax
# =========================================================

def softmax(logits: np.ndarray) -> np.ndarray:
    logits = logits - np.max(logits, axis=1, keepdims=True)
    exp_logits = np.exp(logits)

    denominator = np.sum(exp_logits, axis=1, keepdims=True)

    return exp_logits / denominator


# =========================================================
# scaler.json 읽기
# =========================================================

def load_scaler():
    if not SCALER_PATH.exists():
        raise FileNotFoundError(
            f"scaler.json 파일이 없습니다: {SCALER_PATH}"
        )

    with open(SCALER_PATH, "r", encoding="utf-8") as file:
        scaler_data = json.load(file)

    required_keys = [
        "feature_columns",
        "mean",
        "scale",
        "sequence_length",
    ]

    for key in required_keys:
        if key not in scaler_data:
            raise KeyError(
                f"scaler.json에 '{key}' 항목이 없습니다."
            )

    feature_columns = scaler_data["feature_columns"]

    mean = np.array(
        scaler_data["mean"],
        dtype=np.float32,
    )

    scale = np.array(
        scaler_data["scale"],
        dtype=np.float32,
    )

    sequence_length = int(
        scaler_data["sequence_length"]
    )

    if len(feature_columns) != len(mean):
        raise ValueError(
            "feature_columns와 mean의 길이가 다릅니다."
        )

    if len(feature_columns) != len(scale):
        raise ValueError(
            "feature_columns와 scale의 길이가 다릅니다."
        )

    # 0으로 나누는 오류 방지
    scale = np.where(
        scale == 0,
        1.0,
        scale,
    ).astype(np.float32)

    return (
        feature_columns,
        mean,
        scale,
        sequence_length,
    )


# =========================================================
# CSV 읽기 및 특성 추출
# =========================================================

def load_input_data(feature_columns):
    if not INPUT_CSV_PATH.exists():
        raise FileNotFoundError(
            f"입력 CSV가 없습니다: {INPUT_CSV_PATH}"
        )

    dataframe = pd.read_csv(INPUT_CSV_PATH)

    missing_columns = [
        column
        for column in feature_columns
        if column not in dataframe.columns
    ]

    if missing_columns:
        raise ValueError(
            "CSV에 필요한 열이 없습니다: "
            + ", ".join(missing_columns)
        )

    feature_dataframe = dataframe[
        feature_columns
    ].copy()

    for column in feature_columns:
        feature_dataframe[column] = pd.to_numeric(
            feature_dataframe[column],
            errors="coerce",
        )

    nan_count = int(
        feature_dataframe.isna().sum().sum()
    )

    if nan_count > 0:
        print(
            f"[경고] 비어 있거나 숫자가 아닌 값 "
            f"{nan_count}개를 0으로 변경합니다."
        )

        feature_dataframe = (
            feature_dataframe.fillna(0.0)
        )

    feature_array = feature_dataframe.to_numpy(
        dtype=np.float32
    )

    return dataframe, feature_array


# =========================================================
# 정규화
# =========================================================

def normalize_features(
    feature_array,
    mean,
    scale,
):
    normalized = (
        feature_array - mean
    ) / scale

    return normalized.astype(np.float32)


# =========================================================
# 시퀀스 생성
# =========================================================

def create_sequences(
    normalized_features,
    sequence_length,
):
    total_rows = len(normalized_features)

    if total_rows < sequence_length:
        raise ValueError(
            f"데이터가 {total_rows}행입니다. "
            f"최소 {sequence_length}행이 필요합니다."
        )

    sequence_count = (
        total_rows - sequence_length + 1
    )

    feature_count = normalized_features.shape[1]

    sequences = np.empty(
        (
            sequence_count,
            sequence_length,
            feature_count,
        ),
        dtype=np.float32,
    )

    for index in range(sequence_count):
        sequences[index] = normalized_features[
            index:index + sequence_length
        ]

    return sequences


# =========================================================
# ONNX 세션 생성
# =========================================================

def create_onnx_session():
    if not MODEL_PATH.exists():
        raise FileNotFoundError(
            f"ONNX 모델이 없습니다: {MODEL_PATH}"
        )

    session_options = ort.SessionOptions()

    session_options.graph_optimization_level = (
        ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    )

    session = ort.InferenceSession(
        str(MODEL_PATH),
        sess_options=session_options,
        providers=["CPUExecutionProvider"],
    )

    return session


# =========================================================
# 배치 단위 추론
# =========================================================

def run_inference(
    session,
    sequences,
):
    input_info = session.get_inputs()[0]
    output_info = session.get_outputs()[0]

    input_name = input_info.name
    output_name = output_info.name

    print(
        f"모델 입력: {input_name} "
        f"{input_info.shape}"
    )

    print(
        f"모델 출력: {output_name} "
        f"{output_info.shape}"
    )

    total_sequences = len(sequences)

    logits_list = []

    for start_index in range(
        0,
        total_sequences,
        BATCH_SIZE,
    ):
        end_index = min(
            start_index + BATCH_SIZE,
            total_sequences,
        )

        batch = sequences[
            start_index:end_index
        ]

        output = session.run(
            [output_name],
            {
                input_name: batch
            },
        )

        logits = np.asarray(
            output[0],
            dtype=np.float32,
        )

        logits_list.append(logits)

        print(
            f"\r추론 진행: "
            f"{end_index}/{total_sequences}",
            end="",
            flush=True,
        )

    print()

    all_logits = np.concatenate(
        logits_list,
        axis=0,
    )

    probabilities = softmax(all_logits)

    predictions = np.argmax(
        probabilities,
        axis=1,
    ).astype(np.int64)

    confidence = np.max(
        probabilities,
        axis=1,
    )

    return (
        all_logits,
        probabilities,
        predictions,
        confidence,
    )


# =========================================================
# 결과 저장
# =========================================================

def save_results(
    original_dataframe,
    sequence_length,
    logits,
    probabilities,
    predictions,
    confidence,
):
    # 첫 예측은 원본 데이터의 10번째 행에 대응
    result_dataframe = original_dataframe.iloc[
        sequence_length - 1:
    ].copy()

    result_dataframe.reset_index(
        drop=True,
        inplace=True,
    )

    result_dataframe[
        "onnx_risk_level"
    ] = predictions

    result_dataframe[
        "onnx_confidence"
    ] = confidence

    for level in range(4):
        result_dataframe[
            f"prob_level_{level}"
        ] = probabilities[:, level]

        result_dataframe[
            f"logit_level_{level}"
        ] = logits[:, level]

    result_dataframe.to_csv(
        OUTPUT_CSV_PATH,
        index=False,
        encoding="utf-8-sig",
    )

    return result_dataframe


# =========================================================
# 결과 출력
# =========================================================

def print_summary(result_dataframe):
    print()
    print("========== 추론 결과 ==========")

    counts = (
        result_dataframe["onnx_risk_level"]
        .value_counts()
        .sort_index()
    )

    for level in range(4):
        count = int(
            counts.get(level, 0)
        )

        print(
            f"Risk Level {level}: {count}"
        )

    print()
    print(
        f"결과 저장 완료: "
        f"{OUTPUT_CSV_PATH}"
    )

    preview_columns = [
        column
        for column in [
            "ts_ms",
            "distance_m",
            "ttc",
            "risk_level",
            "onnx_risk_level",
            "onnx_confidence",
        ]
        if column in result_dataframe.columns
    ]

    print()
    print("결과 일부:")

    print(
        result_dataframe[
            preview_columns
        ]
        .head(10)
        .to_string(index=False)
    )

    if (
        "risk_level"
        in result_dataframe.columns
    ):
        accuracy = (
            result_dataframe["risk_level"]
            == result_dataframe[
                "onnx_risk_level"
            ]
        ).mean()

        print()
        print(
            f"기존 라벨 기준 정확도: "
            f"{accuracy:.4f}"
        )


# =========================================================
# 메인
# =========================================================

def main():
    print("1. scaler.json 읽는 중...")

    (
        feature_columns,
        mean,
        scale,
        sequence_length,
    ) = load_scaler()

    print(
        f"특성 개수: "
        f"{len(feature_columns)}"
    )

    print(
        f"시퀀스 길이: "
        f"{sequence_length}"
    )

    print()
    print("2. 입력 CSV 읽는 중...")

    (
        original_dataframe,
        feature_array,
    ) = load_input_data(
        feature_columns
    )

    print(
        f"원본 데이터 개수: "
        f"{len(original_dataframe)}"
    )

    print()
    print("3. 데이터 정규화 중...")

    normalized_features = normalize_features(
        feature_array,
        mean,
        scale,
    )

    print()
    print("4. 시퀀스 생성 중...")

    sequences = create_sequences(
        normalized_features,
        sequence_length,
    )

    print(
        f"생성된 시퀀스 개수: "
        f"{len(sequences)}"
    )

    print(
        f"입력 배열 shape: "
        f"{sequences.shape}"
    )

    print()
    print("5. ONNX 모델 불러오는 중...")

    session = create_onnx_session()

    print(
        f"사용 Provider: "
        f"{session.get_providers()}"
    )

    print()
    print("6. ONNX 추론 시작...")

    (
        logits,
        probabilities,
        predictions,
        confidence,
    ) = run_inference(
        session,
        sequences,
    )

    print()
    print("7. 결과 저장 중...")

    result_dataframe = save_results(
        original_dataframe,
        sequence_length,
        logits,
        probabilities,
        predictions,
        confidence,
    )

    print_summary(
        result_dataframe
    )


if __name__ == "__main__":
    try:
        main()

    except Exception as error:
        print()
        print("========== 오류 발생 ==========")
        print(type(error).__name__)
        print(error)
        raise