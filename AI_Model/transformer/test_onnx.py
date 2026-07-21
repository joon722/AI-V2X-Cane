from pathlib import Path
import json

import numpy as np
import onnxruntime as ort
import pandas as pd


DATA_FILE = Path("labeled_master_dataset.csv")
MODEL_FILE = Path("models/risk_transformer.onnx")
SCALER_FILE = Path("models/scaler.json")


def main():
    scaler_info = json.loads(
        SCALER_FILE.read_text(encoding="utf-8")
    )

    feature_columns = scaler_info["feature_columns"]
    mean = np.asarray(scaler_info["mean"], dtype=np.float32)
    scale = np.asarray(scaler_info["scale"], dtype=np.float32)
    sequence_length = scaler_info["sequence_length"]

    data = pd.read_csv(DATA_FILE)

    features = (
        data[feature_columns]
        .apply(pd.to_numeric, errors="coerce")
        .replace([np.inf, -np.inf], np.nan)
        .fillna(0)
        .to_numpy(dtype=np.float32)
    )

    scaled = (features - mean) / scale

    sample = scaled[:sequence_length]
    sample = np.expand_dims(sample, axis=0).astype(np.float32)

    session = ort.InferenceSession(
        str(MODEL_FILE),
        providers=["CPUExecutionProvider"],
    )

    input_name = session.get_inputs()[0].name
    logits = session.run(None, {input_name: sample})[0]

    predicted_level = int(np.argmax(logits, axis=1)[0])

    print("ONNX 추론 성공")
    print(f"입력 shape: {sample.shape}")
    print(f"출력 logits: {logits}")
    print(f"예측 위험도: Level {predicted_level}")


if __name__ == "__main__":
    main()