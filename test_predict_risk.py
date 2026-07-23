import unittest

from predict_risk import TrajectoryBuffer, NullPredictor, make_predictor, RiskPredictor


def _sample(d):
    return {"ped_x": 0.0, "ped_y": 0.0, "veh_x": d, "veh_y": 0.0,
            "ped_speed": 0.0, "veh_speed": 5.0, "distance_m": d, "rel_speed": 5.0}


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
        buf = TrajectoryBuffer(window=2, feature_order=["distance_m", "rel_speed"])
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


if __name__ == "__main__":
    unittest.main()
