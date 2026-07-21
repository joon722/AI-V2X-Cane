from pathlib import Path

import onnx
import torch
import torch.nn as nn


MODEL_FILE = Path("models/risk_transformer.pt")
ONNX_FILE = Path("models/risk_transformer.onnx")


class RiskTransformer(nn.Module):
    def __init__(
        self,
        num_features: int,
        num_classes: int = 4,
        d_model: int = 64,
        nhead: int = 4,
        num_layers: int = 2,
    ):
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

    def forward(self, x):
        x = self.input_projection(x)
        x = self.encoder(x)
        x = x[:, -1, :]
        return self.classifier(x)


def main():
    if not MODEL_FILE.exists():
        raise FileNotFoundError(f"모델 파일 없음: {MODEL_FILE}")

    checkpoint = torch.load(
        MODEL_FILE,
        map_location="cpu",
        weights_only=False,
    )

    feature_columns = checkpoint["feature_columns"]
    sequence_length = checkpoint["sequence_length"]
    num_classes = checkpoint["num_classes"]

    model = RiskTransformer(
        num_features=len(feature_columns),
        num_classes=num_classes,
    )

    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()

    dummy_input = torch.randn(
        1,
        sequence_length,
        len(feature_columns),
        dtype=torch.float32,
    )

    torch.onnx.export(
        model,
        dummy_input,
        ONNX_FILE,
        input_names=["input"],
        output_names=["logits"],
        dynamic_axes={
            "input": {0: "batch_size"},
            "logits": {0: "batch_size"},
        },
        opset_version=17,
        do_constant_folding=True,
        dynamo=False,
    )

    onnx_model = onnx.load(ONNX_FILE)
    onnx.checker.check_model(onnx_model)

    print(f"ONNX 변환 완료: {ONNX_FILE}")
    print(f"입력 크기: [batch, {sequence_length}, {len(feature_columns)}]")
    print(f"출력 크기: [batch, {num_classes}]")
    print("ONNX 모델 구조 검사 통과")


if __name__ == "__main__":
    main()