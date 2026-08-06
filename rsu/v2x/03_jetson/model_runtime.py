#!/usr/bin/env python3
"""학습한 모델을 의존성 없이 젯슨에서 돌리기 위한 내보내기와 추론.

젯슨에는 scikit-learn도 onnxruntime도 없고, 설치하려면 ARM용 빌드를 구해야 한다.
그런데 gradient boosting의 추론은 트리를 따라 내려가며 값을 더하는 것뿐이라,
트리 구조만 JSON으로 옮기면 표준 라이브러리만으로 재현할 수 있다.

    export_model()   PC에서 학습한 모델 -> JSON (sklearn 필요)
    TreeEnsemble     JSON -> 추론 (표준 라이브러리만)

TreeEnsemble이 numpy조차 부르지 않는 것은 의도된 제약이다. 한 줄이라도 외부
의존이 섞이면 "젯슨에 아무것도 설치하지 않는다"는 목적이 사라진다.

내보낸 파일에는 임계값도 함께 담는다. 임계값은 오경보 예산에서 유도한 값이라
모델과 짝이며, 따로 두면 젯슨에서 다른 값을 쓰게 되어 측정한 성능이 재현되지
않는다.
"""

import json
import math
from pathlib import Path

MODEL_FILE = "risk_model.json"


def export_model(clf, threshold=0.5, features=None):
    """HistGradientBoostingClassifier를 JSON으로 옮길 수 있는 구조로 바꾼다.

    이진 분류이므로 트리 그룹마다 예측기가 하나씩이고, 그 출력의 합에
    baseline을 더한 값이 로짓이 된다.
    """
    if features is None:
        from features import FEATURE_COLUMNS
        features = list(FEATURE_COLUMNS)

    trees = []
    for group in clf._predictors:
        for predictor in group:
            nodes = predictor.nodes
            trees.append([
                {
                    "leaf": bool(node["is_leaf"]),
                    "value": float(node["value"]),
                    "feature": int(node["feature_idx"]),
                    "threshold": float(node["num_threshold"]),
                    "left": int(node["left"]),
                    "right": int(node["right"]),
                    "missing_left": bool(node["missing_go_to_left"]),
                }
                for node in nodes
            ])

    baseline = clf._baseline_prediction
    while hasattr(baseline, "__len__"):
        baseline = baseline[0]

    return {
        "kind": "hist_gradient_boosting_binary",
        "features": list(features),
        "baseline": float(baseline),
        "threshold": float(threshold),
        "trees": trees,
    }


def save_model(clf, path, threshold=0.5, features=None, meta=None):
    blob = export_model(clf, threshold=threshold, features=features)
    if meta:
        blob["meta"] = meta
    Path(path).write_text(json.dumps(blob), encoding="utf-8")
    return path


class TreeEnsemble:
    """내보낸 트리를 표준 라이브러리만으로 실행한다."""

    def __init__(self, blob):
        self.features = tuple(blob["features"])
        self.baseline = float(blob["baseline"])
        self.threshold = float(blob.get("threshold", 0.5))
        self.trees = blob["trees"]
        self.meta = blob.get("meta", {})

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text(encoding="utf-8")))

    def _leaf_value(self, tree, row):
        node = tree[0]
        while not node["leaf"]:
            value = row[node["feature"]]
            if value != value:  # NaN
                node = tree[node["left"] if node["missing_left"] else node["right"]]
            elif value <= node["threshold"]:
                node = tree[node["left"]]
            else:
                node = tree[node["right"]]
        return node["value"]

    def predict_proba(self, feature_map):
        """피처 딕셔너리 하나를 위험 확률로.

        빠진 피처는 KeyError로 드러낸다. 조용히 0으로 채우면 젯슨에서 특징
        하나가 빠져도 그럴듯한 값이 나와, 성능이 떨어진 것을 알 수 없다.
        """
        row = [float(feature_map[name]) for name in self.features]
        logit = self.baseline + sum(self._leaf_value(t, row) for t in self.trees)
        # 지수 폭주를 막는 표준적인 시그모이드 형태
        if logit >= 0:
            return 1.0 / (1.0 + math.exp(-logit))
        exp_logit = math.exp(logit)
        return exp_logit / (1.0 + exp_logit)

    def alarm(self, feature_map):
        return self.predict_proba(feature_map) >= self.threshold


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="모델을 학습해 젯슨용 JSON으로 내보낸다.")
    parser.add_argument("--n-scenarios", type=int, default=1200)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", default=MODEL_FILE)
    args = parser.parse_args()

    import numpy as np

    from baselines import ALARM_LEVEL, team_table_levels
    from dataset import build_dataset, split_by_scenario
    from model import predict_proba, threshold_at_false_alarm_rate, train
    from safety_floor import T_FLOOR_S, timely_alarm_rate

    print(f"시나리오 {args.n_scenarios}개로 학습...", flush=True)
    ds = build_dataset(args.n_scenarios, gps_sigma_m=2.5, lead_s=2.0)
    train_df, test_df = split_by_scenario(ds, test_ratio=0.3, seed=args.seed)
    clf = train(train_df, seed=args.seed, label_col="y_train")

    # 임계값은 현 점수표와 같은 오경보를 쓰도록 맞춘다. 이 값이 모델과 짝이다.
    levels = team_table_levels({c: test_df[c].to_numpy() for c in (
        "distance_m", "closing_los", "ttc", "veh_speed_mps", "dcpa_m")})
    rule_alarm = levels >= ALARM_LEVEL
    target_far = float(rule_alarm[test_df.y.to_numpy() == 0].mean())
    proba = predict_proba(clf, test_df)
    threshold = threshold_at_false_alarm_rate(proba, test_df.y.to_numpy(), target_far)

    model_timely = timely_alarm_rate(test_df, proba >= threshold)
    rule_timely = timely_alarm_rate(test_df, rule_alarm)
    print(f"  임계값 {threshold:.4f} (오경보 {target_far*100:.2f}% 기준)")
    print(f"  적시경보  규칙 {rule_timely*100:.1f}%  ->  모델 {model_timely*100:.1f}%")

    meta = {
        "trained_scenarios": args.n_scenarios,
        "seed": args.seed,
        "target_false_alarm_rate": target_far,
        "timely_rule": rule_timely,
        "timely_model": model_timely,
        "t_floor_s": T_FLOOR_S,
    }
    path = save_model(clf, args.out, threshold=threshold, meta=meta)

    # 저장한 파일이 sklearn과 같은 값을 내는지 그 자리에서 확인한다.
    runtime = TreeEnsemble.load(path)
    rows = test_df[list(runtime.features)].to_numpy(dtype=float)
    check = np.array([runtime.predict_proba(dict(zip(runtime.features, r)))
                      for r in rows[:200]])
    gap = float(np.max(np.abs(check - proba[:200])))
    print(f"  저장: {path} ({Path(path).stat().st_size/1024:.0f} KB)")
    print(f"  sklearn 대비 최대 오차 {gap:.2e} "
          f"({'일치' if gap < 1e-9 else '불일치 - 확인 필요'})")


if __name__ == "__main__":
    main()
