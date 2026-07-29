#!/usr/bin/env python3
"""Risk-classification slot for the fusion stage.

Adapter around Minseo's Transformer (`AI_Model/transformer/`, exported to ONNX):
feeds a recent trajectory window in, returns risk_level 0-3, or None when there
is nothing to say (model absent, or not enough history yet).

Everything below is read off the trained artefacts, not guessed -- see
`export_onnx.py` and `models/scaler.json`:

  input   [batch, 10, 11] float32, tensor name "input"
  output  [batch, 4] LOGITS (no softmax), argmax = risk_level 0-3
  scaling StandardScaler z-score per feature, parameters in scaler.json
  timing  the classifier reads the LAST frame of the window (`x[:, -1, :]`),
          so this is a CURRENT-instant classification, NOT a future prediction.
          (An earlier version of this file said "future" -- it was wrong.)

Two traps this module has to defend against, both from the contract:

  1. ttc's no-approach case. Training filled it with a large sentinel (the
     scaler mean is ~6198 against a 9999 sentinel), so a missing ttc must NOT
     become 0.0 -- that reads as "collision now" and inverts the answer.
  2. Feature order. The 11 slots are positional; a silent reorder feeds the
     model garbage without raising. TrainedModelContractTest pins the order to
     scaler.json.

Note that ttc / risk_score / zone_base_risk are rule-pipeline outputs, not raw
sensor values: the rule stage must run FIRST and its results become features.

Coordinate frame is still unsettled (contract Part 2) -- callers hand in (x, y)
in whatever frame the model was trained on. Without onnxruntime or a model file
the factory returns NullPredictor and the pipeline degrades to rule+zone.
"""

import json
from abc import ABC, abstractmethod
from collections import deque

# Read off models/scaler.json. Pinned by TrainedModelContractTest.
DEFAULT_WINDOW = 10
DEFAULT_FEATURE_ORDER = [
    "ped_x", "ped_y", "veh_x", "veh_y",
    "ped_speed_mps", "veh_speed_mps", "distance_m", "rel_speed_mps",
    "ttc", "risk_score", "zone_base_risk",
]

# "No approach" ttc. Matches the team scoring table's sentinel
# (risk_scoring.calculate_ttc) and the training distribution.
NO_APPROACH_TTC = 9999.0

# Features whose absent/None value is not 0.0.
FEATURE_FILL = {"ttc": NO_APPROACH_TTC}


class Scaler:
    """StandardScaler (z-score) parameters exported alongside the model."""

    def __init__(self, mean, scale, feature_columns, sequence_length):
        self.mean = mean
        self.scale = scale
        self.feature_columns = feature_columns
        self.sequence_length = sequence_length

    @classmethod
    def from_json(cls, path):
        with open(path, "r", encoding="utf-8") as handle:
            spec = json.load(handle)
        return cls(
            mean=spec["mean"],
            scale=spec["scale"],
            feature_columns=spec["feature_columns"],
            sequence_length=spec["sequence_length"],
        )

    def transform(self, matrix):
        """(x - mean) / scale, row by row. Pure python: no numpy on the Jetson
        unless onnxruntime already pulled it in."""
        for row in matrix:
            if len(row) != len(self.mean):
                raise ValueError(
                    f"expected {len(self.mean)} features, got {len(row)}"
                )
        return [
            [(value - m) / s for value, m, s in zip(row, self.mean, self.scale)]
            for row in matrix
        ]


class TrajectoryBuffer:
    """Rolling window of recent samples, newest last."""

    def __init__(self, window=DEFAULT_WINDOW, feature_order=None):
        self.window = window
        self.feature_order = feature_order or DEFAULT_FEATURE_ORDER
        self._samples = deque(maxlen=window)

    def append(self, sample):
        self._samples.append(sample)

    def ready(self):
        return len(self._samples) >= self.window

    def features(self):
        """window x n_features float matrix in feature_order.

        Absent or None values fall back to FEATURE_FILL (0.0 by default); ttc
        gets the no-approach sentinel instead of 0.0 -- see module docstring.
        """
        return [
            [self._value(sample, name) for name in self.feature_order]
            for sample in self._samples
        ]

    def _value(self, sample, name):
        value = sample.get(name)
        if value is None:
            return float(FEATURE_FILL.get(name, 0.0))
        return float(value)


class RiskPredictor(ABC):
    @abstractmethod
    def predict(self, buffer):
        """Future risk_level 0-3, or None when unavailable."""


class NullPredictor(RiskPredictor):
    """Used when onnxruntime or the model file is missing."""

    def predict(self, buffer):
        return None


class OnnxPredictor(RiskPredictor):
    """Runs the exported model. Output is risk_level 0-3 for the current frame.

    The model emits length-4 logits (argmax); a scalar output is also tolerated
    so a shape surprise fails soft, not hard.
    """

    def __init__(self, session, feature_order, window, scaler=None):
        self._session = session
        self._input_name = session.get_inputs()[0].name
        self.feature_order = feature_order
        self.window = window
        self.scaler = scaler

    def predict(self, buffer):
        if not buffer.ready():
            return None
        import numpy as np

        matrix = buffer.features()  # (window, n_feat), raw units
        if self.scaler is not None:
            matrix = self.scaler.transform(matrix)
        batch = np.asarray([matrix], dtype=np.float32)  # (1, window, n_feat)
        outputs = self._session.run(None, {self._input_name: batch})
        arr = np.asarray(outputs[0]).reshape(-1)
        if arr.size >= 4:
            level = int(arr.argmax())
        else:
            level = int(round(float(arr[0])))
        return max(0, min(3, level))


def make_predictor(model_path=None, scaler_path=None, feature_order=None,
                   window=DEFAULT_WINDOW):
    """Factory with an onnxruntime guard.

    Returns a working predictor only when both onnxruntime and a model file are
    present; otherwise NullPredictor, so callers never branch on availability.

    scaler_path is optional but effectively required in production: the model
    was trained on z-scored features, so feeding raw units without it produces
    confident nonsense rather than an error. Missing it is therefore a loud
    warning, not a silent default.
    """
    if not model_path:
        return NullPredictor()
    try:
        import onnxruntime as ort
    except ImportError:
        print("[WARN] onnxruntime not installed; prediction disabled")
        return NullPredictor()
    from pathlib import Path

    if not Path(model_path).exists():
        print(f"[WARN] model not found: {model_path}; prediction disabled")
        return NullPredictor()

    scaler = None
    if scaler_path:
        scaler = Scaler.from_json(scaler_path)
        if scaler.feature_columns != (feature_order or DEFAULT_FEATURE_ORDER):
            raise ValueError(
                "feature order does not match the scaler's trained columns: "
                f"{scaler.feature_columns}"
            )
    else:
        print("[WARN] no scaler.json given; model sees unnormalised features")

    session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    return OnnxPredictor(
        session, feature_order or DEFAULT_FEATURE_ORDER, window, scaler
    )
