"""retention.py 단위 테스트 — 하드웨어·서버 없이 tmp_path로만 검증."""
from pathlib import Path

import pytest

from retention import cleanup_old_scenarios, purged_exclude_args, purged_list_path


def make_scenario(incoming: Path, num: int, uploaded: bool = True,
                  processed: bool = True, with_result: Path = None):
    d = incoming / f"scenario_{num}"
    d.mkdir(parents=True)
    (d / "feature.csv").write_text("f")
    (d / "pedestrian.csv").write_text("p")
    (d / "DONE").touch()
    if processed:
        (d / ".processed").touch()
    if uploaded:
        (d / ".uploaded").touch()
    if with_result is not None:
        with_result.mkdir(exist_ok=True)
        (with_result / f"scenario_{num}_result.csv").write_text("r")
    return d


def test_keeps_newest_n_uploaded_and_deletes_older(tmp_path):
    incoming, results = tmp_path / "in", tmp_path / "out"
    for n in range(100, 110):
        make_scenario(incoming, n, with_result=results)

    purged = cleanup_old_scenarios(incoming, results, keep=3)

    assert purged == [f"scenario_{n}" for n in range(100, 107)]
    remaining = sorted(d.name for d in incoming.glob("scenario_*"))
    assert remaining == ["scenario_107", "scenario_108", "scenario_109"]
    # 결과 CSV도 함께 삭제, 유지분의 결과는 보존
    assert not (results / "scenario_100_result.csv").exists()
    assert (results / "scenario_107_result.csv").exists()


def test_never_deletes_without_uploaded_marker(tmp_path):
    incoming, results = tmp_path / "in", tmp_path / "out"
    make_scenario(incoming, 1, uploaded=False, processed=False,
                  with_result=results)          # 미처리 백로그
    make_scenario(incoming, 2, uploaded=False, with_result=results)  # 업로드 실패분
    make_scenario(incoming, 3, with_result=results)

    purged = cleanup_old_scenarios(incoming, results, keep=0)

    # keep=0이어도 업로드 완료분(3)만 삭제 대상
    assert purged == ["scenario_3"]
    assert (incoming / "scenario_1").exists()
    assert (incoming / "scenario_2").exists()
    assert (results / "scenario_2_result.csv").exists()


def test_purged_list_records_deleted_dirs_for_rsync(tmp_path):
    incoming, results = tmp_path / "in", tmp_path / "out"
    for n in (10, 11, 12):
        make_scenario(incoming, n, with_result=results)

    cleanup_old_scenarios(incoming, results, keep=1)

    lines = purged_list_path(incoming).read_text(encoding="utf-8").splitlines()
    assert lines == ["scenario_10/", "scenario_11/"]
    # 추가 정리 시 이어서 기록 (덮어쓰지 않음)
    make_scenario(incoming, 13, with_result=results)
    cleanup_old_scenarios(incoming, results, keep=1)
    lines = purged_list_path(incoming).read_text(encoding="utf-8").splitlines()
    assert lines == ["scenario_10/", "scenario_11/", "scenario_12/"]


def test_missing_result_csv_does_not_crash(tmp_path):
    incoming, results = tmp_path / "in", tmp_path / "out"
    results.mkdir()
    make_scenario(incoming, 20)  # 결과 CSV 없음
    make_scenario(incoming, 21)

    purged = cleanup_old_scenarios(incoming, results, keep=1)

    assert purged == ["scenario_20"]


def test_keep_larger_than_count_deletes_nothing(tmp_path):
    incoming, results = tmp_path / "in", tmp_path / "out"
    for n in (30, 31):
        make_scenario(incoming, n, with_result=results)

    assert cleanup_old_scenarios(incoming, results, keep=10) == []
    assert len(list(incoming.glob("scenario_*"))) == 2


def test_non_numeric_and_missing_dir_are_ignored(tmp_path):
    incoming, results = tmp_path / "in", tmp_path / "out"
    make_scenario(incoming, 40)
    weird = incoming / "scenario_backup"
    weird.mkdir()
    (weird / ".uploaded").touch()

    assert cleanup_old_scenarios(incoming, results, keep=1) == []
    assert weird.exists()
    # incoming 자체가 없어도 조용히 통과
    assert cleanup_old_scenarios(tmp_path / "none", results, keep=1) == []


def test_purged_exclude_args(tmp_path):
    incoming = tmp_path / "in"
    incoming.mkdir()
    assert purged_exclude_args(incoming) == []          # 목록 없으면 옵션 없음
    purged_list_path(incoming).write_text("scenario_1/\n", encoding="utf-8")
    args = purged_exclude_args(incoming)
    assert args == [f"--exclude-from={purged_list_path(incoming)}"]


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
