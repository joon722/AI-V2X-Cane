"""내보낸 모델이 sklearn과 같은 값을 내는지 검증.

젯슨에는 scikit-learn도 onnxruntime도 없다. 트리를 JSON으로 덤프하고 순수
파이썬으로 추론하면 의존성 없이 돌지만, 그 추론이 학습 때와 조금이라도 다르면
시뮬레이션에서 측정한 성능은 아무 의미가 없어진다. 그래서 "같은 입력에 같은
출력"을 소수점까지 확인한다.
"""

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import pytest

from dataset import build_dataset, split_by_scenario
from model import predict_proba, train
from model_runtime import TreeEnsemble, export_model


@pytest.fixture(scope="module")
def trained():
    ds = build_dataset(n_scenarios=120, gps_sigma_m=2.5, lead_s=2.0)
    train_df, test_df = split_by_scenario(ds, test_ratio=0.3, seed=0)
    clf = train(train_df, seed=0, label_col="y_train")
    return clf, test_df


def test_exported_model_matches_sklearn(trained):
    """순수 파이썬 추론이 sklearn 예측과 일치한다."""
    clf, test_df = trained

    blob = export_model(clf)
    runtime = TreeEnsemble(blob)

    expected = predict_proba(clf, test_df)
    rows = test_df[list(runtime.features)].to_numpy(dtype=float)
    got = np.array([runtime.predict_proba(dict(zip(runtime.features, r))) for r in rows])

    assert np.allclose(got, expected, atol=1e-9)


def test_export_is_json_serialisable(trained):
    """내보낸 결과가 JSON으로 저장된다 - 젯슨에는 이 파일만 올라간다."""
    clf, _ = trained

    blob = export_model(clf)
    text = json.dumps(blob)
    restored = TreeEnsemble(json.loads(text))

    sample = {name: 1.0 for name in restored.features}
    assert 0.0 <= restored.predict_proba(sample) <= 1.0


def test_runtime_needs_no_third_party_import():
    """모듈을 불러올 때 외부 라이브러리가 필요하지 않다.

    젯슨에 아무것도 설치하지 않고 돌리는 것이 이 방식의 목적이다. 검사 대상은
    최상위 import뿐이다 - 함수 안의 import는 그 함수를 부를 때만 필요하고,
    학습·내보내기 함수는 PC에서만 돌기 때문이다.
    """
    import ast
    import pathlib

    # model_runtime은 젯슨에 올라가야 하므로 03_jetson 쪽에 둔다.
    src = (pathlib.Path(__file__).resolve().parent.parent / "model_runtime.py"
           ).read_text(encoding="utf-8")
    imported = set()
    for node in ast.parse(src).body:      # 최상위만 본다
        if isinstance(node, ast.Import):
            imported.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])

    stdlib_only = {"json", "math", "pathlib", "argparse", "typing"}
    assert imported <= stdlib_only, f"표준 라이브러리 밖 의존성: {imported - stdlib_only}"


def test_missing_feature_is_rejected(trained):
    """피처가 빠지면 조용히 0으로 채우지 않고 오류를 낸다.

    조용히 채우면 젯슨에서 특징 하나가 빠져도 그럴듯한 값이 나와, 성능이
    떨어진 것을 알아채지 못한다.
    """
    clf, _ = trained
    runtime = TreeEnsemble(export_model(clf))

    incomplete = {name: 1.0 for name in runtime.features[:-1]}

    with pytest.raises(KeyError):
        runtime.predict_proba(incomplete)


def test_feature_order_is_recorded(trained):
    """피처 이름과 순서가 파일에 남는다 - 순서가 어긋나면 예측이 무의미해진다."""
    clf, _ = trained
    from features import FEATURE_COLUMNS

    blob = export_model(clf)

    assert tuple(blob["features"]) == tuple(FEATURE_COLUMNS)


def test_threshold_is_carried_with_the_model(trained):
    """판정 임계값이 모델과 함께 저장된다.

    임계값은 오경보 예산에서 유도한 값이라 모델과 짝이다. 따로 두면 젯슨에서
    다른 값을 쓰게 되고, 그러면 측정한 성능이 재현되지 않는다.
    """
    clf, test_df = trained

    blob = export_model(clf, threshold=0.37)

    assert blob["threshold"] == pytest.approx(0.37)
    assert TreeEnsemble(blob).threshold == pytest.approx(0.37)
