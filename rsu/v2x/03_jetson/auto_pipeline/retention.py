"""업로드 완료된 오래된 시나리오를 정리해 젯슨 디스크를 일정 수준으로 유지한다.

배경 (2026-08-25): process_scenarios.py 에 정리 로직이 없어 8/14부터 37GB가
쌓여 57GB 디스크가 98%까지 찼다. 서버에 결과 사본이 확인된(.uploaded) 시나리오만
최신 KEEP_UPLOADED 개를 남기고 삭제한다.

재수신 방지: sync_from_server 의 rsync 는 서버 generated_data 를 통째로 받으므로
그냥 지우면 다음 사이클에 되돌아온다(그리고 .processed 가 사라져 재추론까지 한다).
삭제한 시나리오 이름을 .purged.list 에 기록하고, rsync 에 --exclude-from 으로
넘겨 다시 받지 않는다. 목록 기록이 삭제보다 먼저다 — 중간에 죽어도 재수신은 막힌다.
"""
import shutil
from pathlib import Path

# 최근 이틀치(하루 약 1,000개 유입). 입력+결과 합쳐 약 6GB 를 상시 유지한다.
KEEP_UPLOADED = 2000


def purged_list_path(incoming_dir: Path) -> Path:
    return Path(incoming_dir) / ".purged.list"


def purged_exclude_args(incoming_dir: Path) -> list:
    """rsync 에 붙일 --exclude-from 인자. 목록 파일이 없으면 빈 리스트."""
    path = purged_list_path(incoming_dir)
    return [f"--exclude-from={path}"] if path.exists() else []


def cleanup_old_scenarios(incoming_dir: Path, result_dir: Path,
                          keep: int = KEEP_UPLOADED, log=None) -> list:
    """업로드 완료된 시나리오 중 최신 keep 개를 제외한 나머지를 삭제.

    삭제 대상: incoming_dir/scenario_N 디렉터리 + result_dir/scenario_N_result.csv
    .uploaded 마커가 없는 시나리오(미처리 백로그·업로드 실패분)는 절대 지우지 않는다.
    삭제한 시나리오 이름 목록을 오래된 순으로 반환한다.
    """
    incoming_dir, result_dir = Path(incoming_dir), Path(result_dir)
    if not incoming_dir.exists():
        return []

    uploaded = []
    for d in incoming_dir.glob("scenario_*"):
        if not (d.is_dir() and (d / ".uploaded").exists()):
            continue
        try:
            num = int(d.name.split("_", 1)[1])
        except ValueError:
            continue  # scenario_backup 같은 수동 폴더는 건드리지 않는다
        uploaded.append((num, d))

    uploaded.sort()
    doomed = uploaded[: len(uploaded) - keep] if keep > 0 else uploaded
    if not doomed:
        return []

    purged = []
    with purged_list_path(incoming_dir).open("a", encoding="utf-8") as fh:
        for _, d in doomed:
            fh.write(f"{d.name}/\n")
            fh.flush()
            shutil.rmtree(d, ignore_errors=True)
            result_csv = result_dir / f"{d.name}_result.csv"
            try:
                result_csv.unlink()
            except FileNotFoundError:
                pass
            purged.append(d.name)

    if log is not None:
        log.info("보존 정리: 오래된 시나리오 %d개 삭제 (업로드 완료분만, 최신 %d개 유지)",
                 len(purged), keep)
    return purged
