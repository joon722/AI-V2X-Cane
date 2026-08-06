#!/usr/bin/env python3
"""상황 조건부 위험 판단 모델.

트랜스포머가 아니라 Gradient Boosting부터 시작하는 이유가 둘이다.

  설명 가능   "이 상황에서 왜 위험하다고 했나"를 피처 기여도로 답할 수 있다.
              심사에서 반드시 묻는 질문이고, 안전 시스템에서는 답할 수 있어야 한다.
  상한 측정   단순한 모델로 어디까지 되는지 봐야 "그래서 트랜스포머가 필요했다"에
              근거가 생긴다. 현재 팀 파이프라인은 그 근거 없이 트랜스포머부터 썼다.

여기서 얻은 성능이 부족하면 기존 risk_transformer.onnx를 새 라벨로 재학습한다.
구조는 살아 있고 라벨만 바뀌므로 버리는 것이 아니다.

비교의 공정성은 임계값에서 온다. 모델과 규칙을 각자 편한 지점에서 비교하면
아무 말이나 할 수 있으므로, 오경보를 같은 양만 쓰도록 묶어놓고
(threshold_at_false_alarm_rate) 적시 경보를 비교한다.
"""

import numpy as np
from sklearn.ensemble import HistGradientBoostingClassifier

from features import FEATURE_COLUMNS

# 입력은 피처뿐이다. y, t_hit, d_min_future는 정답에서 유래하므로 넣으면 누수다.
MODEL_INPUTS = tuple(FEATURE_COLUMNS)


def _matrix(df):
    return df[list(MODEL_INPUTS)].to_numpy(dtype=float)


def train(df, seed=0, max_iter=200, learning_rate=0.1, max_depth=None, label_col="y"):
    """위험 여부를 맞히는 분류기를 학습한다.

    위험 시점이 전체의 2% 안팎이라 class_weight='balanced'로 균형을 잡는다.
    이것을 빼면 모델이 "전부 안전"이라고만 답해도 98%를 맞히므로 아무것도 배우지
    않는다.

    label_col='y_train'을 주면 창을 T_floor만큼 민 라벨로 배운다. 평가 기준인 y로
    배우면 코앞의 위험도 정답이라, 사람이 반응할 수 없는 시점에 울리는 것을
    정답으로 익힌다.
    """
    clf = HistGradientBoostingClassifier(
        max_iter=max_iter,
        learning_rate=learning_rate,
        max_depth=max_depth,
        class_weight="balanced",
        random_state=seed,
    )
    clf.fit(_matrix(df), df[label_col].to_numpy())
    return clf


def predict_proba(clf, df):
    """위험일 확률."""
    return clf.predict_proba(_matrix(df))[:, 1]


def threshold_at_false_alarm_rate(proba, y, target):
    """안전 시점 중 target 비율만 넘기는 임계값.

    규칙과 모델을 같은 오경보 예산에서 비교하기 위한 것이다. target=0이면 안전
    시점 어느 것도 넘지 못하는 값을 준다.
    """
    safe = np.asarray(proba)[np.asarray(y) == 0]
    if len(safe) == 0:
        return 0.0
    if target <= 0:
        return float(np.nextafter(safe.max(), np.inf))
    return float(np.quantile(safe, 1.0 - target))


def feature_importance(clf, df, seed=0, n_repeats=5):
    """치환 중요도. "무엇을 보고 판단했나"를 답하기 위한 것.

    학습 중 분할 횟수가 아니라 실제 성능 기여로 재므로, 상관된 피처가 많은
    여기에서는 이쪽이 정직하다.
    """
    from sklearn.inspection import permutation_importance

    result = permutation_importance(
        clf, _matrix(df), df.y.to_numpy(),
        n_repeats=n_repeats, random_state=seed, scoring="roc_auc",
    )
    order = np.argsort(result.importances_mean)[::-1]
    return [(MODEL_INPUTS[i], float(result.importances_mean[i])) for i in order]


def main():
    import argparse

    from dataset import build_dataset, split_by_scenario

    parser = argparse.ArgumentParser(description="모델을 학습하고 중요도를 찍는다.")
    parser.add_argument("--n-scenarios", type=int, default=400)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    ds = build_dataset(args.n_scenarios, gps_sigma_m=2.5)
    train_df, test_df = split_by_scenario(ds, test_ratio=0.3, seed=args.seed)
    print(f"학습 {train_df.scenario_id.nunique()}개 시나리오 / "
          f"평가 {test_df.scenario_id.nunique()}개")

    clf = train(train_df, seed=args.seed)
    from sklearn.metrics import roc_auc_score
    proba = predict_proba(clf, test_df)
    print(f"AUC = {roc_auc_score(test_df.y.to_numpy(), proba):.4f}\n")

    print("피처 중요도 (치환, 상위 8개)")
    for name, value in feature_importance(clf, test_df, seed=args.seed)[:8]:
        print(f"  {name:18s} {value:.4f}")


if __name__ == "__main__":
    main()
