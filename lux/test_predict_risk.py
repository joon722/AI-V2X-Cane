import json
import unittest
from pathlib import Path

from predict_risk import (
    DEFAULT_FEATURE_ORDER,
    DEFAULT_WINDOW,
    NO_APPROACH_TTC,
    OnnxPredictor,
    RiskPredictor,
    Scaler,
    NullPredictor,
    TrajectoryBuffer,
    make_predictor,
)


# The trained artefacts live outside lux/ (lux/ alone is what ships to the
# Jetson). Skip the contract tests when they are not reachable.
SCALER_JSON = (
    Path(__file__).resolve().parent.parent
    / "AI_Model" / "transformer" / "models" / "scaler.json"
)


def _sample(d):
    return {"ped_x": 0.0, "ped_y": 0.0, "veh_x": d, "veh_y": 0.0,
            "ped_speed_mps": 0.0, "veh_speed_mps": 5.0, "distance_m": d,
            "rel_speed_mps": 5.0, "ttc": d / 5.0, "risk_score": 10.0,
            "zone_base_risk": 0}


class TrajectoryBufferTest(unittest.TestCase):
    def test_not_ready_until_window_filled(self):
        buf = TrajectoryBuffer(window=3)
        self.assertFalse(buf.ready())
        for d in (30.0, 25.0):
            buf.append(_sample(d))
        self.assertFalse(buf.ready())
        buf.append(_sample(20.0))
        self.assertTrue(buf.ready())

    def test_features_are_window_by_nfeatures(self):
        buf = TrajectoryBuffer(window=2, feature_order=["distance_m", "rel_speed_mps"])
        buf.append(_sample(30.0))
        buf.append(_sample(25.0))
        feats = buf.features()
        self.assertEqual(len(feats), 2)
        self.assertEqual(feats[0], [30.0, 5.0])


class PredictorContractTest(unittest.TestCase):
    def test_null_predictor_returns_none(self):
        buf = TrajectoryBuffer(window=1)
        buf.append(_sample(10.0))
        self.assertIsNone(NullPredictor().predict(buf))

    def test_factory_falls_back_to_null_without_model(self):
        pred = make_predictor(model_path=None)
        self.assertIsInstance(pred, RiskPredictor)
        buf = TrajectoryBuffer(window=1)
        buf.append(_sample(10.0))
        self.assertIsNone(pred.predict(buf))

    def test_predict_returns_none_when_buffer_not_ready(self):
        pred = make_predictor(model_path=None)
        self.assertIsNone(pred.predict(TrajectoryBuffer(window=5)))


@unittest.skipUnless(SCALER_JSON.exists(), f"{SCALER_JSON} not reachable")
class TrainedModelContractTest(unittest.TestCase):
    """Pin this module to the artefacts Minseo actually trained.

    Same idea as test_risk_scoring.TeamTableDriftTest: read the real file and
    fail loudly if the two drift apart. A silent mismatch here does not crash --
    it feeds the model garbage in the wrong feature slots.
    """

    def setUp(self):
        self.spec = json.loads(SCALER_JSON.read_text(encoding="utf-8"))

    def test_default_feature_order_matches_trained_columns(self):
        self.assertEqual(DEFAULT_FEATURE_ORDER, self.spec["feature_columns"])

    def test_default_window_matches_trained_sequence_length(self):
        self.assertEqual(DEFAULT_WINDOW, self.spec["sequence_length"])

    def test_scaler_from_json_loads_the_real_file(self):
        scaler = Scaler.from_json(SCALER_JSON)
        self.assertEqual(scaler.feature_columns, self.spec["feature_columns"])
        self.assertEqual(scaler.mean, self.spec["mean"])
        self.assertEqual(scaler.scale, self.spec["scale"])


class MissingFeatureFillTest(unittest.TestCase):
    def test_absent_ttc_uses_no_approach_sentinel_not_zero(self):
        # 0.0 would read as "collision now" -- the exact inversion the contract
        # (Part 9-2) warns about. Training filled no-approach with 9999.
        buf = TrajectoryBuffer(window=1, feature_order=["ttc"])
        buf.append({})
        self.assertEqual(buf.features(), [[NO_APPROACH_TTC]])

    def test_none_ttc_uses_no_approach_sentinel(self):
        buf = TrajectoryBuffer(window=1, feature_order=["ttc"])
        buf.append({"ttc": None})
        self.assertEqual(buf.features(), [[NO_APPROACH_TTC]])

    def test_present_ttc_is_kept(self):
        buf = TrajectoryBuffer(window=1, feature_order=["ttc"])
        buf.append({"ttc": 2.5})
        self.assertEqual(buf.features(), [[2.5]])

    def test_other_absent_features_stay_zero(self):
        buf = TrajectoryBuffer(window=1, feature_order=["distance_m"])
        buf.append({})
        self.assertEqual(buf.features(), [[0.0]])


class ScalerTest(unittest.TestCase):
    def test_transform_applies_z_score_per_feature(self):
        scaler = Scaler(mean=[10.0, 100.0], scale=[2.0, 50.0],
                        feature_columns=["a", "b"], sequence_length=1)
        self.assertEqual(scaler.transform([[12.0, 200.0]]), [[1.0, 2.0]])

    def test_transform_rejects_wrong_feature_count(self):
        scaler = Scaler(mean=[0.0], scale=[1.0],
                        feature_columns=["a"], sequence_length=1)
        with self.assertRaises(ValueError):
            scaler.transform([[1.0, 2.0]])


class _StubSession:
    """Stands in for onnxruntime.InferenceSession (not installed here)."""

    def __init__(self, logits):
        self._logits = logits
        self.seen = None

    def get_inputs(self):
        return [type("Spec", (), {"name": "input"})()]

    def run(self, _outputs, feeds):
        self.seen = feeds["input"]
        return [[self._logits]]


class OnnxPredictorTest(unittest.TestCase):
    def _buffer(self, window=2):
        buf = TrajectoryBuffer(window=window, feature_order=["a", "b"])
        for _ in range(window):
            buf.append({"a": 12.0, "b": 200.0})
        return buf

    def test_argmax_of_logits_is_the_level(self):
        session = _StubSession([0.1, 0.2, 5.0, 0.3])
        pred = OnnxPredictor(session, ["a", "b"], window=2, scaler=None)
        self.assertEqual(pred.predict(self._buffer()), 2)

    def test_scaler_is_applied_before_inference(self):
        session = _StubSession([9.0, 0.0, 0.0, 0.0])
        scaler = Scaler(mean=[10.0, 100.0], scale=[2.0, 50.0],
                        feature_columns=["a", "b"], sequence_length=2)
        pred = OnnxPredictor(session, ["a", "b"], window=2, scaler=scaler)
        pred.predict(self._buffer())
        # (12-10)/2 = 1.0 and (200-100)/50 = 2.0, batched as (1, window, n_feat)
        self.assertEqual(session.seen.shape, (1, 2, 2))
        self.assertEqual(session.seen[0][0].tolist(), [1.0, 2.0])

    def test_partial_window_never_reaches_the_model(self):
        session = _StubSession([0.0, 0.0, 9.0, 0.0])
        pred = OnnxPredictor(session, ["a", "b"], window=5, scaler=None)
        buf = TrajectoryBuffer(window=5, feature_order=["a", "b"])
        buf.append({"a": 1.0, "b": 2.0})
        self.assertIsNone(pred.predict(buf))
        self.assertIsNone(session.seen)


if __name__ == "__main__":
    unittest.main()
