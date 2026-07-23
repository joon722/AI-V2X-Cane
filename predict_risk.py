#!/usr/bin/env python3
"""Future-risk prediction slot for the fusion stage.

Minseo's Transformer is exported to ONNX and outputs a future risk_level (0-3)
directly (confirmed 2026-07-23). This module is the adapter that feeds a recent
trajectory window into that model and returns the level, or None when there is
nothing to say (model absent, or not enough history yet).

The exact feature list, coordinate frame and normalisation must match Minseo's
training pipeline -- see docs/integration_contract.md section 2. They are passed
in (feature_order, plus any scaling done by the caller) so this file does not
hard-code a guess. Without onnxruntime or a model file the factory returns a
NullPredictor and the whole pipeline degrades to rule+zone gracefully.
"""

from abc import ABC, abstractmethod
from collections import deque

# Confirmed with Minseo: match training window/features in the contract.
DEFAULT_WINDOW = 10  # ~1 s at 10 Hz
DEFAULT_FEATURE_ORDER = [
    "ped_x", "ped_y", "veh_x", "veh_y",
    "ped_speed", "veh_speed", "distance_m", "rel_speed",
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
        """window x n_features float matrix in feature_order."""
        return [
            [float(sample.get(name, 0.0)) for name in self.feature_order]
            for sample in self._samples
        ]


class RiskPredictor(ABC):
    @abstractmethod
    def predict(self, buffer):
        """Future risk_level 0-3, or None when unavailable."""


class NullPredictor(RiskPredictor):
    """Used when onnxruntime or the model file is missing."""

    def predict(self, buffer):
        return None


class OnnxPredictor(RiskPredictor):
    """Runs Minseo's exported model. Output is a future risk_level 0-3.

    Handles both output shapes: a length-4 logit/prob vector (argmax) or a
    single scalar (round + clamp). Which one it is comes from the contract; the
    code tolerates either so a shape surprise fails soft, not hard.
    """

    def __init__(self, session, feature_order, window):
        self._session = session
        self._input_name = session.get_inputs()[0].name
        self.feature_order = feature_order
        self.window = window

    def predict(self, buffer):
        if not buffer.ready():
            return None
        import numpy as np

        matrix = np.asarray(buffer.features(), dtype=np.float32)  # (window, n_feat)
        batch = matrix[np.newaxis, :, :]  # (1, window, n_feat)
        outputs = self._session.run(None, {self._input_name: batch})
        arr = np.asarray(outputs[0]).reshape(-1)
        if arr.size >= 4:
            level = int(arr.argmax())
        else:
            level = int(round(float(arr[0])))
        return max(0, min(3, level))


def make_predictor(model_path=None, feature_order=None, window=DEFAULT_WINDOW):
    """Factory with an onnxruntime guard.

    Returns a working predictor only when both onnxruntime and a model file are
    present; otherwise NullPredictor, so callers never branch on availability.
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
    session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    return OnnxPredictor(session, feature_order or DEFAULT_FEATURE_ORDER, window)
