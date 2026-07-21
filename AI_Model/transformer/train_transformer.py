from pathlib import Path
import json
import random

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from torch.utils.data import DataLoader, Dataset


SEED = 42
DATA_FILE = Path("labeled_master_dataset.csv")
MODEL_DIR = Path("models")
MODEL_FILE = MODEL_DIR / "risk_transformer.pt"
SCALER_FILE = MODEL_DIR / "scaler.json"
REPORT_FILE = MODEL_DIR / "training_report.txt"

FEATURE_COLUMNS = [
    "ped_x",
    "ped_y",
    "veh_x",
    "veh_y",
    "ped_speed_mps",
    "veh_speed_mps",
    "distance_m",
    "rel_speed_mps",
    "ttc",
    "risk_score",
    "zone_base_risk",
]

TARGET_COLUMN = "risk_level"
SEQUENCE_LENGTH = 10
BATCH_SIZE = 64
EPOCHS = 20
LEARNING_RATE = 0.001


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


class SequenceDataset(Dataset):
    def __init__(self, x: np.ndarray, y: np.ndarray) -> None:
        self.x = torch.tensor(x, dtype=torch.float32)
        self.y = torch.tensor(y, dtype=torch.long)

    def __len__(self) -> int:
        return len(self.y)

    def __getitem__(self, index: int):
        return self.x[index], self.y[index]


class RiskTransformer(nn.Module):
    def __init__(
        self,
        num_features: int,
        num_classes: int = 4,
        d_model: int = 64,
        nhead: int = 4,
        num_layers: int = 2,
    ) -> None:
        super().__init__()

        self.input_projection = nn.Linear(num_features, d_model)

        encoder_layer = nn.TransformerEncoderLayer(
            d_model=d_model,
            nhead=nhead,
            dim_feedforward=128,
            dropout=0.1,
            batch_first=True,
        )

        self.encoder = nn.TransformerEncoder(
            encoder_layer,
            num_layers=num_layers,
        )

        self.classifier = nn.Sequential(
            nn.LayerNorm(d_model),
            nn.Linear(d_model, num_classes),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.input_projection(x)
        x = self.encoder(x)
        x = x[:, -1, :]
        return self.classifier(x)


def create_sequences(
    features: np.ndarray,
    labels: np.ndarray,
    sequence_length: int,
):
    sequences = []
    sequence_labels = []

    for end_index in range(sequence_length - 1, len(features)):
        start_index = end_index - sequence_length + 1
        sequences.append(features[start_index:end_index + 1])
        sequence_labels.append(labels[end_index])

    return np.asarray(sequences), np.asarray(sequence_labels)


def calculate_class_weights(labels: np.ndarray, num_classes: int):
    counts = np.bincount(labels, minlength=num_classes)
    total = counts.sum()

    weights = total / (num_classes * np.maximum(counts, 1))
    return torch.tensor(weights, dtype=torch.float32), counts


def evaluate(
    model: nn.Module,
    loader: DataLoader,
    device: torch.device,
):
    model.eval()

    predictions = []
    targets = []

    with torch.no_grad():
        for x_batch, y_batch in loader:
            x_batch = x_batch.to(device)

            logits = model(x_batch)
            predicted = torch.argmax(logits, dim=1).cpu().numpy()

            predictions.extend(predicted.tolist())
            targets.extend(y_batch.numpy().tolist())

    return np.asarray(targets), np.asarray(predictions)


def main() -> None:
    set_seed(SEED)
    MODEL_DIR.mkdir(exist_ok=True)

    if not DATA_FILE.exists():
        raise FileNotFoundError(f"데이터 파일이 없습니다: {DATA_FILE}")

    data = pd.read_csv(DATA_FILE)

    required_columns = FEATURE_COLUMNS + [TARGET_COLUMN]
    missing_columns = [
        column for column in required_columns
        if column not in data.columns
    ]

    if missing_columns:
        raise KeyError(f"필수 컬럼이 없습니다: {missing_columns}")

    data[FEATURE_COLUMNS] = (
        data[FEATURE_COLUMNS]
        .apply(pd.to_numeric, errors="coerce")
        .replace([np.inf, -np.inf], np.nan)
        .fillna(0)
    )

    data[TARGET_COLUMN] = pd.to_numeric(
        data[TARGET_COLUMN],
        errors="coerce",
    ).fillna(0).astype(int)

    features = data[FEATURE_COLUMNS].to_numpy(dtype=np.float32)
    labels = data[TARGET_COLUMN].to_numpy(dtype=np.int64)

    scaler = StandardScaler()
    scaled_features = scaler.fit_transform(features).astype(np.float32)

    sequences, sequence_labels = create_sequences(
        scaled_features,
        labels,
        SEQUENCE_LENGTH,
    )

    indices = np.arange(len(sequence_labels))

    train_indices, test_indices = train_test_split(
        indices,
        test_size=0.2,
        random_state=SEED,
        stratify=sequence_labels,
    )

    train_indices, val_indices = train_test_split(
        train_indices,
        test_size=0.2,
        random_state=SEED,
        stratify=sequence_labels[train_indices],
    )

    train_dataset = SequenceDataset(
        sequences[train_indices],
        sequence_labels[train_indices],
    )
    val_dataset = SequenceDataset(
        sequences[val_indices],
        sequence_labels[val_indices],
    )
    test_dataset = SequenceDataset(
        sequences[test_indices],
        sequence_labels[test_indices],
    )

    train_loader = DataLoader(
        train_dataset,
        batch_size=BATCH_SIZE,
        shuffle=True,
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=BATCH_SIZE,
        shuffle=False,
    )
    test_loader = DataLoader(
        test_dataset,
        batch_size=BATCH_SIZE,
        shuffle=False,
    )

    device = torch.device(
        "cuda" if torch.cuda.is_available() else "cpu"
    )

    model = RiskTransformer(
        num_features=len(FEATURE_COLUMNS),
        num_classes=4,
    ).to(device)

    class_weights, class_counts = calculate_class_weights(
        sequence_labels[train_indices],
        num_classes=4,
    )

    criterion = nn.CrossEntropyLoss(
        weight=class_weights.to(device)
    )

    optimizer = torch.optim.Adam(
        model.parameters(),
        lr=LEARNING_RATE,
    )

    best_val_loss = float("inf")

    print(f"사용 장치: {device}")
    print(f"전체 시퀀스 수: {len(sequences)}")
    print(f"학습 라벨 분포: {class_counts.tolist()}")

    for epoch in range(1, EPOCHS + 1):
        model.train()
        train_loss_sum = 0.0

        for x_batch, y_batch in train_loader:
            x_batch = x_batch.to(device)
            y_batch = y_batch.to(device)

            optimizer.zero_grad()

            logits = model(x_batch)
            loss = criterion(logits, y_batch)

            loss.backward()
            optimizer.step()

            train_loss_sum += loss.item() * len(y_batch)

        train_loss = train_loss_sum / len(train_dataset)

        model.eval()
        val_loss_sum = 0.0

        with torch.no_grad():
            for x_batch, y_batch in val_loader:
                x_batch = x_batch.to(device)
                y_batch = y_batch.to(device)

                logits = model(x_batch)
                loss = criterion(logits, y_batch)

                val_loss_sum += loss.item() * len(y_batch)

        val_loss = val_loss_sum / len(val_dataset)

        print(
            f"Epoch {epoch:02d}/{EPOCHS} | "
            f"train_loss={train_loss:.4f} | "
            f"val_loss={val_loss:.4f}"
        )

        if val_loss < best_val_loss:
            best_val_loss = val_loss

            torch.save(
                {
                    "model_state_dict": model.state_dict(),
                    "feature_columns": FEATURE_COLUMNS,
                    "sequence_length": SEQUENCE_LENGTH,
                    "num_classes": 4,
                },
                MODEL_FILE,
            )

    checkpoint = torch.load(
        MODEL_FILE,
        map_location=device,
    )

    model.load_state_dict(checkpoint["model_state_dict"])

    y_true, y_pred = evaluate(
        model,
        test_loader,
        device,
    )

    report = classification_report(
        y_true,
        y_pred,
        labels=[0, 1, 2, 3],
        target_names=[
            "Normal",
            "Caution",
            "Warning",
            "Near Miss",
        ],
        digits=4,
        zero_division=0,
    )

    matrix = confusion_matrix(
        y_true,
        y_pred,
        labels=[0, 1, 2, 3],
    )

    scaler_info = {
        "feature_columns": FEATURE_COLUMNS,
        "mean": scaler.mean_.tolist(),
        "scale": scaler.scale_.tolist(),
        "sequence_length": SEQUENCE_LENGTH,
    }

    SCALER_FILE.write_text(
        json.dumps(
            scaler_info,
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    report_text = (
        "Classification Report\n"
        "=====================\n"
        f"{report}\n\n"
        "Confusion Matrix\n"
        "================\n"
        f"{matrix}\n"
    )

    REPORT_FILE.write_text(
        report_text,
        encoding="utf-8",
    )

    print("\n테스트 결과")
    print(report)
    print("Confusion Matrix")
    print(matrix)
    print(f"\n모델 저장: {MODEL_FILE}")
    print(f"스케일러 저장: {SCALER_FILE}")
    print(f"평가 결과 저장: {REPORT_FILE}")


if __name__ == "__main__":
    main()